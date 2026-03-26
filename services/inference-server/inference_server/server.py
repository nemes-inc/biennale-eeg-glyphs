"""
EEG Inference Server — TCP server for multi-headband ZUNA inference.

Architecture:
  - asyncio TCP server accepts one eeg-hub connection
  - Incoming EEGM frames are accumulated in per-headband EpochBuffers
  - Complete epochs are queued for a pool of ZUNA worker threads
  - Each worker loads its own ZUNA model instance (~300–400 MB VRAM each)
  - Reconstructed results are sent back as EEGM response frames

Since ZUNA inference is ~4-5× slower than real-time, running multiple workers
in parallel reduces effective latency proportionally.  With an RTX 4090 (24 GB)
and ~400 MB per instance, 8–16 workers can run concurrently.

Usage:
    python -m inference_server.server --port 9100 --workers 8
    python -m inference_server.server --port 9100 --echo   # no GPU, just echo back
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import queue
import signal
import sys
import threading
import time

from .eegm_protocol import (
    ConnectAck,
    ConnectReq,
    EegmFrame,
    TargetChannelsAck,
    TargetChannelsReq,
    read_message,
    write_frame,
    write_message,
)
from .epoch_buffer import EpochBuffer, MAX_HEADBANDS, EPOCH_SAMPLES, NUM_MUSE_CHANNELS
from .mmap_queue import MmapJobQueue
from .zuna_worker import InferenceJob, InferenceResult, ZunaWorker, shutdown_persistent_subprocess

log = logging.getLogger(__name__)

# Maximum pending epochs in the inference queue before back-pressure.
MAX_QUEUE_DEPTH = 1024

# Default number of parallel ZUNA worker threads.
DEFAULT_WORKERS = 4


class InferenceServer:
    """Async TCP server that bridges eeg-hub ↔ ZUNA GPU worker."""

    def __init__(
        self,
        host: str = "0.0.0.0",
        port: int = 9100,
        echo_mode: bool = False,
        gpu_device: int | str = 0,
        diffusion_steps: int = 50,
        target_channels: list[str] | None = None,
        num_workers: int = DEFAULT_WORKERS,
    ) -> None:
        self.host = host
        self.port = port
        self.echo_mode = echo_mode
        self.gpu_device = gpu_device
        self.diffusion_steps = diffusion_steps
        self.target_channels = target_channels
        self.num_workers = max(1, num_workers)

        # Per-headband epoch buffers
        self.buffers: list[EpochBuffer] = [
            EpochBuffer() for _ in range(MAX_HEADBANDS)
        ]
        # Per-headband epoch sequence counters (for outgoing frames)
        self.out_seq: list[int] = [0] * MAX_HEADBANDS

        # Thread-safe job queue backed by a memory-mapped file so epoch
        # data lives on disk (via kernel page cache) instead of Python heap.
        self._job_queue = MmapJobQueue(
            maxsize=MAX_QUEUE_DEPTH,
            max_channels=NUM_MUSE_CHANNELS,
            max_samples=EPOCH_SAMPLES,
        )

        # Results queue (workers push, asyncio loop reads)
        self._result_queue: asyncio.Queue[InferenceResult] = asyncio.Queue()

        # Active writer (only one hub connection at a time)
        self._writer: asyncio.StreamWriter | None = None
        self._running = True

        # Worker threads
        self._worker_threads: list[threading.Thread] = []
        self._workers_ready = threading.Event()
        self._workers_loaded = 0
        self._workers_lock = threading.Lock()

        # Stats
        self.frames_received = 0
        self.epochs_queued = 0
        self.epochs_processed = 0
        self.epochs_dropped = 0

        self._tcp_server = None
        self._send_task: asyncio.Task | None = None

        # Per-session target channels (set by client via TargetChannelsReq,
        # overrides the CLI --target-channels default for this connection).
        self._session_target_channels: list[str] | None = None

    async def start(self) -> None:
        """Start the TCP server and worker pool."""
        self._loop = asyncio.get_running_loop()
        if not self.echo_mode:
            self._start_worker_pool()
            log.info(
                "Waiting for %d ZUNA worker(s) to load models…",
                self.num_workers,
            )
            # Wait in a non-blocking way so asyncio loop stays responsive
            while not self._workers_ready.is_set():
                await asyncio.sleep(0.5)
            log.info("All %d workers ready.", self.num_workers)

        self._tcp_server = await asyncio.start_server(
            self._handle_client, self.host, self.port
        )
        addrs = ", ".join(str(s.getsockname()) for s in self._tcp_server.sockets)
        log.info("Inference server listening on %s", addrs)
        if self.echo_mode:
            log.info("ECHO MODE — frames are returned without GPU inference")

        # Start result sender task
        self._send_task = asyncio.create_task(self._send_results())

        async with self._tcp_server:
            try:
                await self._tcp_server.serve_forever()
            except asyncio.CancelledError:
                log.info("Server shutting down")

    async def _handle_client(
        self,
        reader: asyncio.StreamReader,
        writer: asyncio.StreamWriter,
    ) -> None:
        """Handle one eeg-hub connection."""
        addr = writer.get_extra_info("peername")
        log.info("Hub connected: %s", addr)

        try:
            # ── Handshake phase ────────────────────────────────────────
            if not await self._do_handshake(reader, writer, addr):
                return

            self._writer = writer

            # Reset buffers and session state for new connection
            for buf in self.buffers:
                buf.clear()
            self.frames_received = 0
            self._session_target_channels = None

            # ── Streaming phase ────────────────────────────────────────
            while self._running:
                msg = await read_message(reader)
                if msg is None:
                    log.info("Hub disconnected (EOF): %s", addr)
                    break

                if isinstance(msg, ConnectReq):
                    log.warning("Duplicate ConnectReq from %s — ignoring", addr)
                    continue

                if isinstance(msg, TargetChannelsReq):
                    self._session_target_channels = msg.channel_names
                    log.info(
                        "Client %s set target channels: %s",
                        addr,
                        msg.channel_names,
                    )
                    ack = TargetChannelsAck.ok(len(msg.channel_names))
                    await write_message(writer, ack)
                    continue

                if not isinstance(msg, EegmFrame):
                    log.warning("Unexpected message type %s from %s", type(msg).__name__, addr)
                    continue

                frame = msg
                self.frames_received += 1
                hid = frame.headband_id

                if hid >= MAX_HEADBANDS:
                    log.warning("Ignoring frame with headband_id=%d", hid)
                    continue

                if self.echo_mode:
                    # Echo mode: send frame back immediately (for testing)
                    await write_frame(writer, frame)
                    continue

                # Accumulate into epoch buffer
                channels = [
                    frame.channel_data(ch) for ch in range(frame.n_channels)
                ]
                self.buffers[hid].push(channels)

                # Check for complete epochs
                while True:
                    epoch = self.buffers[hid].pop_epoch()
                    if epoch is None:
                        break
                    tc = self._session_target_channels or self.target_channels
                    job = InferenceJob(
                        headband_id=hid,
                        epoch=epoch,
                        target_channels=tc,
                    )
                    try:
                        self._job_queue.put_nowait(job)
                        self.epochs_queued += 1
                    except queue.Full:
                        self.epochs_dropped += 1
                        log.warning(
                            "Inference queue full (%d) — dropping epoch "
                            "(total dropped=%d)",
                            MAX_QUEUE_DEPTH,
                            self.epochs_dropped,
                        )

        except Exception:
            log.exception("Error handling hub connection %s", addr)
        finally:
            self._writer = None
            writer.close()

    async def _do_handshake(
        self,
        reader: asyncio.StreamReader,
        writer: asyncio.StreamWriter,
        addr,
    ) -> bool:
        """Wait for ConnectReq, validate, send ConnectAck.

        Returns True if handshake succeeded and streaming may begin.
        """
        try:
            msg = await asyncio.wait_for(read_message(reader), timeout=10.0)
        except asyncio.TimeoutError:
            log.warning("Handshake timeout from %s — closing", addr)
            writer.close()
            return False

        if msg is None:
            log.info("Hub disconnected before handshake: %s", addr)
            writer.close()
            return False

        if not isinstance(msg, ConnectReq):
            log.warning(
                "Expected ConnectReq from %s, got %s — rejecting",
                addr,
                type(msg).__name__,
            )
            ack = ConnectAck.error(1)  # 1 = protocol error
            await write_message(writer, ack)
            writer.close()
            return False

        req: ConnectReq = msg
        log.info(
            "ConnectReq from %s: version=%d headbands=%d sample_rate=%d",
            addr,
            req.protocol_version,
            req.n_headbands,
            req.sample_rate,
        )

        # Validate
        if req.n_headbands < 1 or req.n_headbands > MAX_HEADBANDS:
            log.warning("Invalid n_headbands=%d — rejecting", req.n_headbands)
            ack = ConnectAck.error(2)  # 2 = bad headband count
            await write_message(writer, ack)
            writer.close()
            return False

        # Accept
        ack = ConnectAck.ok(req.n_headbands, req.sample_rate)
        await write_message(writer, ack)
        log.info("Handshake OK — session: %d headband(s) @ %d Hz", req.n_headbands, req.sample_rate)
        return True

    async def _send_results(self) -> None:
        """Send reconstructed frames back to the hub as they arrive."""
        try:
            await self._send_results_loop()
        except asyncio.CancelledError:
            log.debug("Result sender task cancelled — shutting down")

    async def _send_results_loop(self) -> None:
        while self._running:
            try:
                result = await asyncio.wait_for(
                    self._result_queue.get(), timeout=0.5
                )
            except asyncio.TimeoutError:
                continue

            writer = self._writer
            if writer is None or writer.is_closing():
                continue

            frame = EegmFrame.from_channels(
                headband_id=result.headband_id,
                epoch_seq=result.epoch_seq,
                channels=result.channels,
                n_samples=result.n_samples,
            )
            try:
                await write_frame(writer, frame)
                self.epochs_processed += 1
                if self.epochs_processed % 5 == 0:
                    log.info(
                        "Stats: received=%d queued=%d processed=%d dropped=%d",
                        self.frames_received,
                        self.epochs_queued,
                        self.epochs_processed,
                        self.epochs_dropped,
                    )
            except Exception:
                log.exception("Failed to send result frame")

    def _start_worker_pool(self) -> None:
        """Start N ZUNA inference workers in background threads."""
        log.info("Starting %d ZUNA worker thread(s)…", self.num_workers)
        for wid in range(self.num_workers):
            t = threading.Thread(
                target=self._worker_loop,
                args=(wid,),
                daemon=True,
                name=f"zuna-worker-{wid}",
            )
            t.start()
            self._worker_threads.append(t)

    def _worker_loop(self, worker_id: int) -> None:
        """Worker thread: load own model instance, process epochs from shared queue."""
        wlog = logging.getLogger(f"{__name__}.worker-{worker_id}")
        wlog.info("Worker %d starting — loading ZUNA model…", worker_id)

        worker = ZunaWorker(
            gpu_device=self.gpu_device,
            diffusion_steps=self.diffusion_steps,
            target_channels=self.target_channels,
        )

        try:
            worker.load_model()
        except Exception:
            wlog.exception("Worker %d: failed to load ZUNA model — exiting", worker_id)
            return
        finally:
            with self._workers_lock:
                self._workers_loaded += 1
                if self._workers_loaded >= self.num_workers:
                    self._workers_ready.set()

        wlog.info("Worker %d ready.", worker_id)
        loop = self._loop

        while self._running:
            try:
                job = self._job_queue.get(timeout=1.0)
            except queue.Empty:
                continue

            if job is None:
                # Poison pill — shutdown signal
                break

            t0 = time.monotonic()
            wlog.info(
                "[W%d] Processing epoch: headband=%d seq=%d (queue=%d)",
                worker_id,
                job.headband_id,
                job.epoch.seq,
                self._job_queue.qsize(),
            )

            result = worker.process_epoch(job)
            elapsed = time.monotonic() - t0

            if result is not None:
                wlog.info(
                    "[W%d] Epoch done: headband=%d seq=%d → %d ch × %d samples (%.1fs)",
                    worker_id,
                    result.headband_id,
                    result.epoch_seq,
                    result.n_channels,
                    result.n_samples,
                    elapsed,
                )
                loop.call_soon_threadsafe(self._result_queue.put_nowait, result)
            else:
                wlog.warning(
                    "[W%d] Epoch failed: headband=%d seq=%d (%.1fs)",
                    worker_id,
                    job.headband_id,
                    job.epoch.seq,
                    elapsed,
                )

        wlog.info("Worker %d exiting.", worker_id)

    def shutdown(self) -> None:
        self._running = False
        if self._send_task is not None:
            self._send_task.cancel()
            self._send_task = None
        if self._tcp_server is not None:
            self._tcp_server.close()
        # Send poison pills to all workers so they exit cleanly
        for _ in self._worker_threads:
            try:
                self._job_queue.put_nowait(None)
            except queue.Full:
                pass
        # Clean up the mmap backing file
        self._job_queue.close()
        # Shut down the persistent inference subprocess
        shutdown_persistent_subprocess()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="EEG Inference Server — ZUNA GPU inference over TCP"
    )
    parser.add_argument(
        "--host",
        default="0.0.0.0",
        help="Bind address (default: 0.0.0.0)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=9100,
        help="TCP port (default: 9100)",
    )
    parser.add_argument(
        "--echo",
        action="store_true",
        help="Echo mode: return frames without GPU inference (for testing)",
    )
    parser.add_argument(
        "--workers",
        type=int,
        default=DEFAULT_WORKERS,
        help=f"Number of parallel ZUNA model instances (default: {DEFAULT_WORKERS}). "
        "Each uses ~300–400 MB VRAM.",
    )
    parser.add_argument(
        "--gpu-device",
        default="0",
        help="GPU device ID for ZUNA inference (default: 0), or empty string for CPU",
    )
    parser.add_argument(
        "--diffusion-steps",
        type=int,
        default=50,
        help="ZUNA diffusion sampling steps (default: 50)",
    )
    parser.add_argument(
        "--target-channels",
        default=None,
        help="Comma-separated 10-20 channel names for upsampling (e.g. Fz,Cz,Pz)",
    )
    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
    )
    args = parser.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(levelname)-7s %(name)s  %(message)s",
        datefmt="%H:%M:%S",
    )

    gpu = "" if args.gpu_device.strip() == "" else int(args.gpu_device)
    target_ch = (
        [x.strip() for x in args.target_channels.split(",") if x.strip()]
        if args.target_channels
        else None
    )

    server = InferenceServer(
        host=args.host,
        port=args.port,
        echo_mode=args.echo,
        gpu_device=gpu,
        diffusion_steps=args.diffusion_steps,
        target_channels=target_ch,
        num_workers=args.workers,
    )

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, server.shutdown)

    try:
        loop.run_until_complete(server.start())
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        # Let cancelled tasks finalize before closing the loop
        pending = asyncio.all_tasks(loop)
        if pending:
            loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
        loop.close()


if __name__ == "__main__":
    main()
