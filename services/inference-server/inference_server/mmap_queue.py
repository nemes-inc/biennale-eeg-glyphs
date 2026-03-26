"""
Memory-mapped file-backed job queue for EEG inference epochs.

Replaces an in-memory ``queue.Queue`` with an mmap-backed circular buffer
so that queued epoch data lives on disk (via the kernel page cache) rather
than in the Python heap.  The OS lazily pages slots in and out, keeping
resident memory bounded even with a very large queue capacity.

Layout
------
::

    [ ring header (24 B) ][ slot 0 ][ slot 1 ] … [ slot N-1 ]

Ring header (3 × uint64):
    head  – next write index
    tail  – next read index
    count – current occupancy

Each slot:
    tag          int8    (1 = job, -1 = poison pill / None sentinel)
    headband_id  int32
    seq          uint32
    n_channels   uint32
    n_samples    uint32
    timestamp_us uint64
    data         float64[max_channels × max_samples]
"""

from __future__ import annotations

import mmap
import os
import queue
import struct
import tempfile
import threading
from pathlib import Path

from .epoch_buffer import Epoch
from .zuna_worker import InferenceJob

# ── binary layout constants ─────────────────────────────────────────────

_RING_HDR = struct.Struct("<QQQ")  # head, tail, count
_RING_HDR_SIZE = _RING_HDR.size  # 24

_SLOT_HDR = struct.Struct("<b i I I I Q")  # tag, headband_id, seq, n_ch, n_samp, timestamp_us
_SLOT_HDR_SIZE = _SLOT_HDR.size  # 25

_F64 = 8  # sizeof(float64)


class MmapJobQueue:
    """Thread-safe, mmap-backed circular queue for ``InferenceJob | None``.

    Drop-in replacement for ``queue.Queue[InferenceJob | None]`` with the
    subset of methods used by :class:`InferenceServer`:

    * ``put_nowait(item)`` – enqueue, raises ``queue.Full``
    * ``get(timeout=…)``  – dequeue, raises ``queue.Empty``
    * ``qsize()``
    * ``close()``         – unmap and delete the backing file
    """

    def __init__(
        self,
        maxsize: int = 1024,
        max_channels: int = 4,
        max_samples: int = 1280,
        path: Path | str | None = None,
    ) -> None:
        self._maxsize = maxsize
        self._max_channels = max_channels
        self._max_samples = max_samples

        # Slot geometry
        self._slot_data_bytes = max_channels * max_samples * _F64
        self._slot_size = _SLOT_HDR_SIZE + self._slot_data_bytes
        total_bytes = _RING_HDR_SIZE + self._slot_size * maxsize

        # Create backing file
        if path is None:
            fd, self._path = tempfile.mkstemp(
                prefix="eeg_queue_", suffix=".mmap"
            )
        else:
            self._path = str(path)
            fd = os.open(self._path, os.O_RDWR | os.O_CREAT)

        os.ftruncate(fd, total_bytes)
        self._fd = fd
        self._mm = mmap.mmap(fd, total_bytes)

        # Zero the ring header (head=0, tail=0, count=0)
        _RING_HDR.pack_into(self._mm, 0, 0, 0, 0)

        # Threading primitives
        self._lock = threading.Lock()
        self._not_empty = threading.Condition(self._lock)

    # ── public API ──────────────────────────────────────────────────────

    def put_nowait(self, item: InferenceJob | None) -> None:
        """Enqueue *item*.  Raises :exc:`queue.Full` when the buffer is full."""
        with self._not_empty:
            head, tail, count = _RING_HDR.unpack_from(self._mm, 0)
            if count >= self._maxsize:
                raise queue.Full
            self._write_slot(head, item)
            _RING_HDR.pack_into(
                self._mm, 0, (head + 1) % self._maxsize, tail, count + 1
            )
            self._not_empty.notify()

    def get(self, timeout: float | None = None) -> InferenceJob | None:
        """Dequeue an item.  Blocks up to *timeout* seconds."""
        with self._not_empty:
            while True:
                _, tail, count = _RING_HDR.unpack_from(self._mm, 0)
                if count > 0:
                    break
                if not self._not_empty.wait(timeout=timeout):
                    raise queue.Empty
            head, tail, count = _RING_HDR.unpack_from(self._mm, 0)
            item = self._read_slot(tail)
            _RING_HDR.pack_into(
                self._mm, 0, head, (tail + 1) % self._maxsize, count - 1
            )
            return item

    def qsize(self) -> int:
        with self._lock:
            _, _, count = _RING_HDR.unpack_from(self._mm, 0)
            return count

    def close(self) -> None:
        """Unmap and delete the backing file."""
        try:
            self._mm.close()
        except Exception:
            pass
        try:
            os.close(self._fd)
        except Exception:
            pass
        try:
            os.unlink(self._path)
        except OSError:
            pass

    # ── private helpers ─────────────────────────────────────────────────

    def _slot_offset(self, index: int) -> int:
        return _RING_HDR_SIZE + index * self._slot_size

    def _write_slot(self, index: int, item: InferenceJob | None) -> None:
        off = self._slot_offset(index)

        if item is None:
            # Poison pill: tag = -1
            _SLOT_HDR.pack_into(self._mm, off, -1, 0, 0, 0, 0, 0)
            return

        epoch = item.epoch
        n_ch = epoch.n_channels
        n_samp = epoch.n_samples
        assert n_ch <= self._max_channels, (
            f"n_channels {n_ch} > max {self._max_channels}"
        )
        assert n_samp <= self._max_samples, (
            f"n_samples {n_samp} > max {self._max_samples}"
        )

        _SLOT_HDR.pack_into(
            self._mm, off, 1, item.headband_id, epoch.seq, n_ch, n_samp,
            epoch.timestamp_us,
        )

        data_off = off + _SLOT_HDR_SIZE
        fmt = f"<{n_samp}d"
        row_bytes = n_samp * _F64
        for ch_idx in range(n_ch):
            struct.pack_into(
                fmt, self._mm, data_off + ch_idx * row_bytes,
                *epoch.channels[ch_idx],
            )

    def _read_slot(self, index: int) -> InferenceJob | None:
        off = self._slot_offset(index)
        tag, headband_id, seq, n_ch, n_samp, timestamp_us = _SLOT_HDR.unpack_from(
            self._mm, off
        )

        if tag == -1:
            return None  # Poison pill

        data_off = off + _SLOT_HDR_SIZE
        fmt = f"<{n_samp}d"
        row_bytes = n_samp * _F64
        channels: list[list[float]] = []
        for ch_idx in range(n_ch):
            samples = list(
                struct.unpack_from(fmt, self._mm, data_off + ch_idx * row_bytes)
            )
            channels.append(samples)

        epoch = Epoch(seq=seq, channels=channels, timestamp_us=timestamp_us)
        return InferenceJob(headband_id=headband_id, epoch=epoch)
