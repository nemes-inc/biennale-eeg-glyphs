"""
Per-headband epoch accumulation buffer.

Accumulates raw EEG samples and yields complete 5-second epochs (1280 samples
at 256 Hz) as required by the ZUNA model.  Uses a 50 % overlap so that a new
epoch is emitted every 2.5 s of incoming data once the first full window is
filled.
"""

from __future__ import annotations

from dataclasses import dataclass, field

# ZUNA fixed parameters
SAMPLE_RATE = 256
EPOCH_SECONDS = 5
EPOCH_SAMPLES = SAMPLE_RATE * EPOCH_SECONDS  # 1280
HOP_SAMPLES = EPOCH_SAMPLES // 2  # 640  (2.5 s hop → 50 % overlap)
NUM_MUSE_CHANNELS = 4
MAX_HEADBANDS = 4


@dataclass
class EpochBuffer:
    """Accumulates raw f32 samples and yields complete epochs.

    One buffer per headband.  ``push()`` appends an EEGM frame's worth of
    samples; ``pop_epoch()`` returns the next ready epoch (or ``None``).
    """

    n_channels: int = NUM_MUSE_CHANNELS
    # Per-channel ring buffer (list of lists).
    _buffers: list[list[float]] = field(default_factory=list)
    # Monotonic epoch counter.
    _epoch_seq: int = 0
    # Timestamp of the first frame contributing to the current window.
    _window_start_ts: int = 0

    def __post_init__(self) -> None:
        if not self._buffers:
            self._buffers = [[] for _ in range(self.n_channels)]

    def push(self, channels: list[list[float]], timestamp_us: int = 0) -> None:
        """Append samples from an EEGM frame (channel-major lists)."""
        if self.available() == 0 and timestamp_us > 0:
            self._window_start_ts = timestamp_us
        for ch_idx, samples in enumerate(channels):
            if ch_idx < self.n_channels:
                self._buffers[ch_idx].extend(samples)

    def available(self) -> int:
        """Minimum sample count across all channels."""
        if not self._buffers:
            return 0
        return min(len(b) for b in self._buffers)

    def pop_epoch(self) -> "Epoch | None":
        """Return the next complete epoch, or ``None`` if not enough data."""
        if self.available() < EPOCH_SAMPLES:
            return None

        channels = [b[:EPOCH_SAMPLES] for b in self._buffers]
        seq = self._epoch_seq
        ts = self._window_start_ts
        self._epoch_seq += 1

        # Advance by hop (discard consumed samples).
        for b in self._buffers:
            del b[:HOP_SAMPLES]

        # Next window starts HOP_SAMPLES later
        if ts > 0:
            hop_us = int(HOP_SAMPLES / SAMPLE_RATE * 1_000_000)
            self._window_start_ts = ts + hop_us

        return Epoch(seq=seq, channels=channels, timestamp_us=ts)

    def clear(self) -> None:
        """Reset all buffers."""
        self._buffers = [[] for _ in range(self.n_channels)]


@dataclass
class Epoch:
    """A complete 5-second epoch ready for ZUNA inference."""

    seq: int
    channels: list[list[float]]  # [n_channels][EPOCH_SAMPLES]
    timestamp_us: int = 0  # timestamp of the first sample in this epoch

    @property
    def n_channels(self) -> int:
        return len(self.channels)

    @property
    def n_samples(self) -> int:
        return len(self.channels[0]) if self.channels else 0
