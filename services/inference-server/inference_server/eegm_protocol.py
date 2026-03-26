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
MAGIC_EEGC = 0x4545_4743
HEADER_SIZE = 20
CTRL_SIZE = 24
HEADER_FMT = "<5I"  # 5 × u32 little-endian
CTRL_FMT = "<6I"   # 6 × u32 little-endian
MAX_HEADBANDS = 4
PROTOCOL_VERSION = 1

MSG_CONNECT_REQ = 1
MSG_CONNECT_ACK = 2


@dataclass
class ConnectReq:
    """Connection request sent by the hub to the inference server."""

    protocol_version: int = PROTOCOL_VERSION
    n_headbands: int = 1
    sample_rate: int = 256

    def encode(self) -> bytes:
        return struct.pack(
            CTRL_FMT,
            MAGIC_EEGC,
            MSG_CONNECT_REQ,
            self.protocol_version,
            self.n_headbands,
            self.sample_rate,
            0,  # reserved
        )


@dataclass
class ConnectAck:
    """Connection acknowledgement sent by the server back to the hub."""

    protocol_version: int = PROTOCOL_VERSION
    n_headbands: int = 0
    sample_rate: int = 0
    status: int = 0  # 0 = OK, non-zero = error

    @staticmethod
    def ok(n_headbands: int, sample_rate: int) -> "ConnectAck":
        return ConnectAck(
            protocol_version=PROTOCOL_VERSION,
            n_headbands=n_headbands,
            sample_rate=sample_rate,
            status=0,
        )

    @staticmethod
    def error(code: int) -> "ConnectAck":
        return ConnectAck(status=code)

    def is_ok(self) -> bool:
        return self.status == 0

    def encode(self) -> bytes:
        return struct.pack(
            CTRL_FMT,
            MAGIC_EEGC,
            MSG_CONNECT_ACK,
            self.protocol_version,
            self.n_headbands,
            self.sample_rate,
            self.status,
        )


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


async def read_message(reader) -> ConnectReq | ConnectAck | EegmFrame | None:
    """Read the next message from an asyncio StreamReader.

    Dispatches on the magic bytes (EEGC for control, EEGM for data).
    Returns ``None`` on clean EOF.
    """
    magic_bytes = await reader.read(4)
    if len(magic_bytes) == 0:
        return None
    if len(magic_bytes) < 4:
        raise IOError(f"truncated magic ({len(magic_bytes)} bytes)")

    (magic,) = struct.unpack("<I", magic_bytes)

    if magic == MAGIC_EEGC:
        rest = await reader.readexactly(CTRL_SIZE - 4)
        msg_type, version, n_headbands, sample_rate, status = struct.unpack(
            "<5I", rest
        )
        if msg_type == MSG_CONNECT_REQ:
            return ConnectReq(
                protocol_version=version,
                n_headbands=n_headbands,
                sample_rate=sample_rate,
            )
        elif msg_type == MSG_CONNECT_ACK:
            return ConnectAck(
                protocol_version=version,
                n_headbands=n_headbands,
                sample_rate=sample_rate,
                status=status,
            )
        else:
            raise IOError(f"unknown EEGC msg_type: {msg_type}")

    elif magic == MAGIC_EEGM:
        hdr_rest = await reader.readexactly(HEADER_SIZE - 4)
        headband_id, epoch_seq, n_channels, n_samples = struct.unpack(
            "<4I", hdr_rest
        )
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

    else:
        raise IOError(f"unknown magic: 0x{magic:08X} (expected EEGM or EEGC)")


async def read_frame(reader) -> EegmFrame | None:
    """Read one EEGM data frame (legacy helper, skips control messages)."""
    msg = await read_message(reader)
    if msg is None:
        return None
    if isinstance(msg, EegmFrame):
        return msg
    raise IOError(f"expected EegmFrame, got {type(msg).__name__}")


async def write_message(writer, msg: ConnectReq | ConnectAck | EegmFrame) -> None:
    """Write any EEGM/EEGC message to an asyncio StreamWriter."""
    writer.write(msg.encode())
    await writer.drain()


async def write_frame(writer, frame: EegmFrame) -> None:
    """Write one EEGM data frame to an asyncio StreamWriter."""
    writer.write(frame.encode())
    await writer.drain()
