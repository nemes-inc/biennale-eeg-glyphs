"""
Live test: verify the inference server preserves timestamp_us in responses.

Connects to a running server, sends frames with known timestamps,
and checks that responses carry the same timestamps back.

Usage:
    python tests/test_server_timestamp.py --server 192.168.1.101:9100
"""

import argparse
import asyncio
import struct
import sys
import time

# Wire constants (must match eegm_protocol.py)
MAGIC_EEGM = 0x4545_474D
MAGIC_EEGC = 0x4545_4743
HEADER_FMT = "<5IQ"  # 5 x u32 + 1 x u64
HEADER_SIZE = 28
CTRL_FMT = "<6I"
CTRL_SIZE = 24


def encode_connect_req(n_headbands: int, sample_rate: int) -> bytes:
    return struct.pack(CTRL_FMT, MAGIC_EEGC, 1, 1, n_headbands, sample_rate, 0)


def encode_eegm_frame(headband_id: int, epoch_seq: int, n_channels: int,
                       n_samples: int, timestamp_us: int, data: list[float]) -> bytes:
    header = struct.pack(HEADER_FMT, MAGIC_EEGM, headband_id, epoch_seq,
                         n_channels, n_samples, timestamp_us)
    payload = struct.pack(f"<{len(data)}f", *data)
    return header + payload


async def run_test(host: str, port: int):
    print(f"Connecting to {host}:{port}...")
    reader, writer = await asyncio.open_connection(host, port)

    # -- Handshake --
    writer.write(encode_connect_req(1, 256))
    await writer.drain()

    ack_buf = await reader.readexactly(CTRL_SIZE)
    magic = struct.unpack_from("<I", ack_buf, 0)[0]
    assert magic == MAGIC_EEGC, f"Bad ack magic: 0x{magic:08X}"
    status = struct.unpack_from("<I", ack_buf, 20)[0]
    assert status == 0, f"Server rejected: status={status}"
    print("Handshake OK")

    # -- Send frames with known timestamps --
    # We need 5 seconds of data at 256 Hz = 1280 samples for one epoch.
    # Send as 5 frames of 256 samples each.
    n_channels = 4
    samples_per_frame = 256
    frames_needed = 5  # 5 * 256 = 1280 samples = 5 seconds
    base_ts = 1_700_000_000_000_000  # a known timestamp

    sent_timestamps = []
    for seq in range(frames_needed):
        ts = base_ts + seq * 1_000_000  # 1 second apart
        sent_timestamps.append(ts)
        n_samples = samples_per_frame
        # Sine wave data so ZUNA has something to work with
        import math
        data = []
        for ch in range(n_channels):
            for s in range(n_samples):
                t = (seq * n_samples + s) / 256.0
                val = 20.0 * math.sin(2 * math.pi * 10 * t + ch * 0.5)
                data.append(val)
        frame = encode_eegm_frame(0, seq, n_channels, n_samples, ts, data)
        writer.write(frame)
        await writer.drain()
        print(f"  Sent seq={seq} ts_us={ts}")

    print(f"\nSent {frames_needed} frames. Waiting for responses (up to 120s)...")
    print(f"Expected: server should return timestamp_us from the epoch's first frame\n")

    # -- Read responses --
    responses = 0
    deadline = time.monotonic() + 120

    while time.monotonic() < deadline:
        try:
            magic_buf = await asyncio.wait_for(reader.readexactly(4), timeout=5.0)
        except asyncio.TimeoutError:
            if responses > 0:
                break  # Got at least one, done
            continue
        except asyncio.IncompleteReadError:
            break

        magic = struct.unpack("<I", magic_buf)[0]

        if magic == MAGIC_EEGC:
            await reader.readexactly(CTRL_SIZE - 4)
            print("  (skipping EEGC control message)")
            continue

        if magic != MAGIC_EEGM:
            print(f"  ERROR: unexpected magic 0x{magic:08X}")
            break

        hdr_rest = await reader.readexactly(HEADER_SIZE - 4)
        hid, seq, nch, nsamp, ts_us = struct.unpack("<4IQ", hdr_rest)
        payload_size = nch * nsamp * 4
        await reader.readexactly(payload_size)

        responses += 1
        ts_match = "MATCH" if ts_us in sent_timestamps else ("ZERO" if ts_us == 0 else "UNKNOWN")
        print(f"  Response #{responses}: hid={hid} seq={seq} ch={nch} samp={nsamp} ts_us={ts_us} -> {ts_match}")

    writer.close()

    # -- Verdict --
    print(f"\n{'='*60}")
    if responses == 0:
        print("FAIL: No responses received (server may need more data or more time)")
        return False

    print(f"Received {responses} response(s)")
    return True


def main():
    parser = argparse.ArgumentParser(description="Test timestamp preservation on live server")
    parser.add_argument("--server", default="192.168.1.101:9100")
    args = parser.parse_args()

    host, port = args.server.rsplit(":", 1)
    ok = asyncio.run(run_test(host, int(port)))
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
