eeg-viewer

- Connects up to 4 Muse headsets over BLE with per-device tabbed waveforms
- Records independently per device to FIF files
- Streams raw EEG to a Python inference server over TCP using the EEGM binary protocol
- Every outbound frame is appended to a per-device session file on disk before entering the TCP send path
- The TCP client tracks frames_sent per device against frames_written on the session file
- On server reconnect the client reads each session file from the last-sent offset forward, replays unsent frames, then resumes live streaming
- Session files persist across restarts and the user deletes them manually
- Legacy stdin and TCP pipe modes still work with --stdin or --tcp flags

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
   20      u32    status = 0

3. Server replies ConnectAck (24 bytes, same layout, msg_type = 2, status 0 = OK)

4. Viewer streams EEGM data frames (variable length, little-endian)

   offset  type                          value
   0       u32                           magic 0x4545474D ("EEGM")
   4       u32                           headband_id (0-3)
   8       u32                           epoch_seq (monotonic per headband)
   12      u32                           n_channels
   16      u32                           n_samples
   20      f32[n_channels * n_samples]   channel-major payload

   total frame size = 20 + 4 * n_channels * n_samples bytes

5. Server sends reconstructed EEGM frames back using the same wire format
   routed by headband_id to the correct device tab
```

Build

```bash
cd app/eeg-viewer && cargo build --release
```

Start the inference server

Echo mode returns every received EEGM frame back to the viewer immediately without running GPU inference. It skips worker pool startup so it needs no GPU, no model weights, no CUDA. Use it to verify the handshake, TCP plumbing, session replay, and UI rendering end to end.

```bash
cd services/inference-server
python -m inference_server.server --port 41820 --echo
```

Production mode spawns a pool of ZUNA diffusion workers that run actual GPU inference on buffered 5-second EEG epochs. The reconstructed frames are sent back with the same headband_id so the viewer routes them to the correct device tab.

```bash
python -m inference_server.server --port 41820 --workers 8
```

Connect the viewer to the server

```bash
cd app/eeg-viewer
./target/release/eeg-viewer --server 127.0.0.1:41820
```

Click Scan to find Muse headsets, click a device to connect, then click Connect Server. The bottom status bar shows frame counters for local processing, disk writes, and server sends. In echo mode the server mirrors raw frames back as reconstructed traces shown in red.

Flags

- --server ADDR connects to inference server at ADDR
- --stdin reads EEGF/EEGD from stdin, disables BLE
- --tcp ADDR listens for one TCP client, disables BLE
- --no-muse-ble disables BLE, empty plot unless --stdin or --tcp
- --max-points N sets rolling buffer length per channel, default 8192
- --zuna-dir PATH sets ZUNA root containing run_fif_pipeline.py
- --muse-record-bin PATH sets path to muse-record-fif binary
