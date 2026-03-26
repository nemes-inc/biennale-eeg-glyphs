"""Tests for EEGM binary protocol — encode/decode roundtrip with timestamps."""

import asyncio
import struct

import pytest

from inference_server.eegm_protocol import (
    HEADER_SIZE,
    MAGIC_EEGM,
    ConnectAck,
    ConnectReq,
    EegmFrame,
    read_message,
    write_message,
)


def _reader_from_bytes(data: bytes) -> asyncio.StreamReader:
    """Create a StreamReader pre-loaded with data for testing."""
    reader = asyncio.StreamReader()
    reader.feed_data(data)
    reader.feed_eof()
    return reader


class TestEegmFrameEncode:
    def test_header_size_is_28(self):
        assert HEADER_SIZE == 28

    def test_encode_produces_correct_header_size(self):
        frame = EegmFrame(
            headband_id=0, epoch_seq=1, n_channels=2, n_samples=3,
            timestamp_us=123456789, data=[1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
        )
        encoded = frame.encode()
        # 28-byte header + 6 floats × 4 bytes = 52 bytes
        assert len(encoded) == 28 + 6 * 4

    def test_encode_timestamp_in_header(self):
        ts = 1700000000_000000  # a realistic microsecond timestamp
        frame = EegmFrame(
            headband_id=1, epoch_seq=42, n_channels=2, n_samples=2,
            timestamp_us=ts, data=[0.1, 0.2, 0.3, 0.4],
        )
        encoded = frame.encode()
        # Parse header manually: magic(4) + hid(4) + seq(4) + nch(4) + nsamp(4) + ts(8)
        magic, hid, seq, nch, nsamp, timestamp = struct.unpack("<5IQ", encoded[:28])
        assert magic == MAGIC_EEGM
        assert hid == 1
        assert seq == 42
        assert nch == 2
        assert nsamp == 2
        assert timestamp == ts

    def test_from_channels_preserves_timestamp(self):
        ts = 9999999
        frame = EegmFrame.from_channels(
            headband_id=0, epoch_seq=0,
            channels=[[1.0, 2.0], [3.0, 4.0]],
            timestamp_us=ts,
        )
        assert frame.timestamp_us == ts
        encoded = frame.encode()
        _, _, _, _, _, timestamp = struct.unpack("<5IQ", encoded[:28])
        assert timestamp == ts


class TestReadMessageRoundtrip:
    @pytest.mark.asyncio
    async def test_eegm_frame_roundtrip_with_timestamp(self):
        ts = 1700000000_123456
        frame = EegmFrame(
            headband_id=2, epoch_seq=10, n_channels=4, n_samples=3,
            timestamp_us=ts,
            data=[float(i) for i in range(12)],
        )
        encoded = frame.encode()
        reader = _reader_from_bytes(encoded)
        decoded = await read_message(reader)

        assert isinstance(decoded, EegmFrame)
        assert decoded.headband_id == 2
        assert decoded.epoch_seq == 10
        assert decoded.n_channels == 4
        assert decoded.n_samples == 3
        assert decoded.timestamp_us == ts
        assert decoded.data == frame.data

    @pytest.mark.asyncio
    async def test_eegm_frame_roundtrip_zero_timestamp(self):
        """Frames with timestamp_us=0 should still roundtrip correctly."""
        frame = EegmFrame(
            headband_id=0, epoch_seq=0, n_channels=2, n_samples=2,
            timestamp_us=0, data=[1.0, 2.0, 3.0, 4.0],
        )
        reader = _reader_from_bytes(frame.encode())
        decoded = await read_message(reader)

        assert isinstance(decoded, EegmFrame)
        assert decoded.timestamp_us == 0

    @pytest.mark.asyncio
    async def test_eegm_frame_roundtrip_max_timestamp(self):
        """u64 max value should survive encode/decode."""
        ts = (1 << 64) - 1
        frame = EegmFrame(
            headband_id=0, epoch_seq=0, n_channels=1, n_samples=1,
            timestamp_us=ts, data=[42.0],
        )
        reader = _reader_from_bytes(frame.encode())
        decoded = await read_message(reader)

        assert isinstance(decoded, EegmFrame)
        assert decoded.timestamp_us == ts

    @pytest.mark.asyncio
    async def test_connect_req_still_works(self):
        """Control messages should be unaffected by the EEGM header change."""
        req = ConnectReq(protocol_version=1, n_headbands=2, sample_rate=256)
        reader = _reader_from_bytes(req.encode())
        decoded = await read_message(reader)

        assert isinstance(decoded, ConnectReq)
        assert decoded.n_headbands == 2
        assert decoded.sample_rate == 256

    @pytest.mark.asyncio
    async def test_connect_ack_still_works(self):
        ack = ConnectAck.ok(n_headbands=3, sample_rate=512)
        reader = _reader_from_bytes(ack.encode())
        decoded = await read_message(reader)

        assert isinstance(decoded, ConnectAck)
        assert decoded.is_ok()
        assert decoded.n_headbands == 3

    @pytest.mark.asyncio
    async def test_multiple_frames_in_stream(self):
        """Read two consecutive EEGM frames with different timestamps."""
        f1 = EegmFrame(
            headband_id=0, epoch_seq=1, n_channels=2, n_samples=2,
            timestamp_us=100, data=[1.0, 2.0, 3.0, 4.0],
        )
        f2 = EegmFrame(
            headband_id=1, epoch_seq=2, n_channels=2, n_samples=2,
            timestamp_us=200, data=[5.0, 6.0, 7.0, 8.0],
        )
        reader = _reader_from_bytes(f1.encode() + f2.encode())

        d1 = await read_message(reader)
        d2 = await read_message(reader)

        assert isinstance(d1, EegmFrame)
        assert d1.timestamp_us == 100
        assert d1.headband_id == 0

        assert isinstance(d2, EegmFrame)
        assert d2.timestamp_us == 200
        assert d2.headband_id == 1

    @pytest.mark.asyncio
    async def test_eof_returns_none(self):
        reader = _reader_from_bytes(b"")
        result = await read_message(reader)
        assert result is None
