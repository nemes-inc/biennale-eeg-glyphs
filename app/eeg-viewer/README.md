# eeg-viewer

Multi-Muse EEG viewer with per-device recording and real-time ZUNA inference overlay.

## What it does

- Connects up to 4 Muse headsets over BLE with per-device tabbed waveforms
- Records independently per device to FIF files
- Streams raw EEG to a Python inference server over TCP using the EEGM binary protocol
- Overlays reconstructed (ZUNA-inferred) EEG at the correct time position on the original trace
- Every outbound frame is write-ahead logged to a per-device session file before entering the TCP path
- On server reconnect, replays unsent frames automatically from the session file offset
- Legacy stdin and TCP pipe modes still work with `--stdin` or `--tcp` flags

## Timestamp-aligned overlay

Each EEGM frame carries a `timestamp_us` field (microseconds since Unix epoch). The BLE loop stamps outgoing frames with `SystemTime::now()`. The inference server threads this timestamp through its epoch buffer, ZUNA pipeline, and back in the response. The viewer converts timestamps to seconds relative to the first frame and stores plot data as `[time_secs, amplitude]` pairs. When reconstructed data arrives — potentially 30-75 seconds after the original capture — it gets placed at the correct x-position on the plot, directly overlapping the raw trace it was inferred from.

If the server sends `timestamp_us=0` (old server or echo mode without timestamp support), the viewer falls back to appending at the current end of the raw timeline.

## Wire protocol

```text
Inference server API contract (EEGM/EEGC binary protocol over plain TCP)

1. Viewer connects to server
2. Viewer sends ConnectReq (24 bytes, little-endian)

   offset  type   value
   0       u32    magic 0x45454743 ("EEGC")
   4       u32    msg_type = 1
   8       u32    protocol_version = 1
   12      u32    n_headbands (1-4)
   16      u32    sample_rate (256)
   20      u32    reserved = 0

3. Server replies ConnectAck (24 bytes, same layout, msg_type = 2, status 0 = OK)

4. Viewer streams EEGM data frames (variable length, little-endian)

   offset  type                          value
   0       u32                           magic 0x4545474D ("EEGM")
   4       u32                           headband_id (0-3)
   8       u32                           epoch_seq (monotonic per headband)
   12      u32                           n_channels
   16      u32                           n_samples per channel
   20      u64                           timestamp_us (capture time, microseconds)
   28      f32[n_channels * n_samples]   channel-major payload

   total frame size = 28 + 4 * n_channels * n_samples bytes

5. Server sends reconstructed EEGM frames back using the same 28-byte header
   with the original timestamp_us preserved, routed by headband_id
```

## Build

```bash
cd app/eeg-viewer && cargo build --release
```

## Usage

### Start the inference server

Echo mode returns frames immediately without GPU inference. Use it to verify handshake, TCP plumbing, session replay, and UI rendering.

```bash
cd services/inference-server
python -m inference_server.server --port 41820 --echo
```

Production mode runs ZUNA diffusion workers on buffered 5-second epochs.

```bash
python -m inference_server.server --port 41820 --workers 8
```

### Connect the viewer

```bash
cd app/eeg-viewer
./target/release/eeg-viewer --server 127.0.0.1:41820
```

Click Scan to find Muse headsets, click a device to connect, then click Connect Server. The bottom status bar shows frame counters for local processing, disk writes, and server sends. Reconstructed traces overlay in red at the original capture timestamp.

### Test without hardware

The `eegm-test-sender` generates synthetic multi-headband EEG with realistic signal profiles (alpha, theta, beta, mixed) and timestamps.

```bash
cargo run --release --bin eegm-test-sender -- --target 127.0.0.1:41820 --headbands 4
```

## Flags

| Flag | Description |
| ---- | ----------- |
| `--server ADDR` | Connect to inference server at ADDR |
| `--stdin` | Read EEGF/EEGD from stdin, disables BLE |
| `--tcp ADDR` | Listen for one TCP client, disables BLE |
| `--no-muse-ble` | Disable BLE, empty plot unless `--stdin` or `--tcp` |
| `--max-points N` | Rolling buffer length per channel (default: 8192) |
| `--zuna-dir PATH` | ZUNA root containing `run_fif_pipeline.py` |
| `--muse-record-bin PATH` | Path to `muse-record-fif` binary |
