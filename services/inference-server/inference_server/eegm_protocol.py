"""
EEGM binary frame protocol — Python implementation.

Wire format (little-endian):

    Offset  Type   Value
    0       u32    Magic: 0x4545474D ("EEGM")
    4       u32    headband_id (0–3)
    8       u32    epoch_seq
    12      u32    n_channels
    16      u32    n_samples per channel
    20      f32[]  channel-major payload

Matches the Rust implementation in ``muse_rs::eegm``.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field

MAGIC_EEGM = 0x4545_474D
HEADER_SIZE = 20
HEADER_FMT = "<5I"  # 5 × u32 little-endian
MAX_HEADBANDS = 4


@dataclass
class EegmFrame:
    """Decoded EEGM frame."""

    headband_id: int
    epoch_seq: int
    n_channels: int
    n_samples: int
    data: list[float] = field(default_factory=list)

    @staticmethod
    def from_channels(
        headband_id: int,
        epoch_seq: int,
        channels: list[list[float]],
        n_samples: int | None = None,
    ) -> "EegmFrame":
        """Build a frame from per-channel sample lists."""
        n_ch = len(channels)
        if n_samples is None:
            n_samples = len(channels[0]) if channels else 0
        data: list[float] = []
        for ch in channels:
            data.extend(ch[:n_samples])
        return EegmFrame(
            headband_id=headband_id,
            epoch_seq=epoch_seq,
            n_channels=n_ch,
            n_samples=n_samples,
            data=data,
        )

    def encode(self) -> bytes:
        """Encode to wire bytes."""
        header = struct.pack(
            HEADER_FMT,
            MAGIC_EEGM,
            self.headband_id,
            self.epoch_seq,
            self.n_channels,
            self.n_samples,
        )
        payload = struct.pack(f"<{len(self.data)}f", *self.data)
        return header + payload

    def channel_data(self, ch: int) -> list[float]:
        """Extract samples for channel ``ch`` (0-indexed)."""
        start = ch * self.n_samples
        return self.data[start : start + self.n_samples]


async def read_frame(reader) -> EegmFrame | None:
    """Read one EEGM frame from an asyncio StreamReader.

    Returns ``None`` on clean EOF.
    """
    header_bytes = await reader.read(HEADER_SIZE)
    if len(header_bytes) == 0:
        return None
    if len(header_bytes) < HEADER_SIZE:
        raise IOError(f"truncated EEGM header ({len(header_bytes)} bytes)")

    magic, headband_id, epoch_seq, n_channels, n_samples = struct.unpack(
        HEADER_FMT, header_bytes
    )
    if magic != MAGIC_EEGM:
        raise IOError(f"bad EEGM magic: 0x{magic:08X} (expected 0x{MAGIC_EEGM:08X})")
    if n_channels > 64 or n_samples > 65536:
        raise IOError(
            f"implausible EEGM dimensions: {n_channels} ch × {n_samples} samples"
        )

    payload_size = n_channels * n_samples * 4
    payload_bytes = await reader.readexactly(payload_size)
    data = list(struct.unpack(f"<{n_channels * n_samples}f", payload_bytes))

    return EegmFrame(
        headband_id=headband_id,
        epoch_seq=epoch_seq,
        n_channels=n_channels,
        n_samples=n_samples,
        data=data,
    )


async def write_frame(writer, frame: EegmFrame) -> None:
    """Write one EEGM frame to an asyncio StreamWriter."""
    writer.write(frame.encode())
    await writer.drain()
