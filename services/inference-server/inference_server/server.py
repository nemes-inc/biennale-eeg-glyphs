"""
EEG Inference Server — TCP server for multi-headband ZUNA inference.

Architecture:
  - asyncio TCP server accepts one eeg-hub connection
  - Incoming EEGM frames are accumulated in per-headband EpochBuffers
  - Complete epochs are queued for the ZUNA worker thread
  - Reconstructed results are sent back as EEGM response frames

Since ZUNA inference is ~4-5× slower than real-time, the server processes
epochs asynchronously.  The eeg-hub displays raw data live and overlays
processed results as they arrive (delayed).

Usage:
    python -m inference_server.server --port 9100
    python -m inference_server.server --port 9100 --echo   # no GPU, just echo back
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import signal
import sys
import threading
import time
from collections import deque

from .eegm_protocol import EegmFrame, read_frame, write_frame
from .epoch_buffer import EpochBuffer, MAX_HEADBANDS
from .zuna_worker import InferenceJob, InferenceResult, ZunaWorker

log = logging.getLogger(__name__)

# Maximum pending epochs in the inference queue before dropping oldest.
MAX_QUEUE_DEPTH = 16


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
    ) -> None:
        self.host = host
        self.port = port
        self.echo_mode = echo_mode
        self.gpu_device = gpu_device
        self.diffusion_steps = diffusion_steps
        self.target_channels = target_channels

        # Per-headband epoch buffers
        self.buffers: list[EpochBuffer] = [
            EpochBuffer() for _ in range(MAX_HEADBANDS)
        ]
        # Per-headband epoch sequence counters (for outgoing frames)
        self.out_seq: list[int] = [0] * MAX_HEADBANDS

        # Inference queue (worker thread pulls from here)
        self._job_queue: deque[InferenceJob] = deque(maxlen=MAX_QUEUE_DEPTH)
        self._job_event = threading.Event()

        # Results queue (worker pushes, asyncio loop reads)
        self._result_queue: asyncio.Queue[InferenceResult] = asyncio.Queue()

        # Active writer (only one hub connection at a time)
        self._writer: asyncio.StreamWriter | None = None
        self._running = True

        # Stats
        self.frames_received = 0
        self.epochs_queued = 0
        self.epochs_processed = 0
        self.epochs_dropped = 0

    async def start(self) -> None:
        """Start the TCP server and worker thread."""
        if not self.echo_mode:
            self._start_worker_thread()

        server = await asyncio.start_server(
            self._handle_client, self.host, self.port
        )
        addrs = ", ".join(str(s.getsockname()) for s in server.sockets)
        log.info("Inference server listening on %s", addrs)
        if self.echo_mode:
            log.info("ECHO MODE — frames are returned without GPU inference")

        # Start result sender task
        asyncio.create_task(self._send_results())

        async with server:
            await server.serve_forever()

    async def _handle_client(
        self,
        reader: asyncio.StreamReader,
        writer: asyncio.StreamWriter,
    ) -> None:
        """Handle one eeg-hub connection."""
        addr = writer.get_extra_info("peername")
        log.info("Hub connected: %s", addr)
        self._writer = writer

        # Reset buffers for new session
        for buf in self.buffers:
            buf.clear()
        self.frames_received = 0

        try:
            while self._running:
                frame = await read_frame(reader)
                if frame is None:
                    log.info("Hub disconnected (EOF): %s", addr)
                    break

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
                    job = InferenceJob(headband_id=hid, epoch=epoch)
                    if len(self._job_queue) >= MAX_QUEUE_DEPTH:
                        self.epochs_dropped += 1
                        log.warning(
                            "Inference queue full — dropping oldest epoch "
                            "(dropped=%d)",
                            self.epochs_dropped,
                        )
                    self._job_queue.append(job)
                    self._job_event.set()
                    self.epochs_queued += 1

        except Exception:
            log.exception("Error handling hub connection %s", addr)
        finally:
            self._writer = None
            writer.close()

    async def _send_results(self) -> None:
        """Send reconstructed frames back to the hub as they arrive."""
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

    def _start_worker_thread(self) -> None:
        """Start the ZUNA inference worker in a background thread."""
        t = threading.Thread(target=self._worker_loop, daemon=True)
        t.start()

    def _worker_loop(self) -> None:
        """Worker thread: load model, process epochs from queue."""
        worker = ZunaWorker(
            gpu_device=self.gpu_device,
            diffusion_steps=self.diffusion_steps,
            target_channels=self.target_channels,
        )

        try:
            worker.load_model()
        except Exception:
            log.exception("Failed to load ZUNA model — worker exiting")
            return

        loop = asyncio.get_event_loop()

        while self._running:
            self._job_event.wait(timeout=1.0)
            self._job_event.clear()

            while self._job_queue:
                job = self._job_queue.popleft()
                t0 = time.monotonic()
                log.info(
                    "Processing epoch: headband=%d seq=%d",
                    job.headband_id,
                    job.epoch.seq,
                )

                result = worker.process_epoch(job)
                elapsed = time.monotonic() - t0

                if result is not None:
                    log.info(
                        "Epoch done: headband=%d seq=%d → %d ch × %d samples (%.1fs)",
                        result.headband_id,
                        result.epoch_seq,
                        result.n_channels,
                        result.n_samples,
                        elapsed,
                    )
                    loop.call_soon_threadsafe(self._result_queue.put_nowait, result)
                else:
                    log.warning(
                        "Epoch failed: headband=%d seq=%d (%.1fs)",
                        job.headband_id,
                        job.epoch.seq,
                        elapsed,
                    )

    def shutdown(self) -> None:
        self._running = False


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
        loop.close()


if __name__ == "__main__":
    main()
