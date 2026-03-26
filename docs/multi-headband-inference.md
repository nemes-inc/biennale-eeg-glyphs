# Multi-Headband EEG Streaming + GPU Inference

## Architecture

```
┌──────────────────────────────────┐       TCP (EEGM, bidirectional)      ┌──────────────────────────────┐
│  MacBook — eeg-hub               │ ──────────────────────────────────▶  │  GPU Server — inference-server│
│                                  │ ◀──────────────────────────────────  │  (Ubuntu + RTX 4090)         │
│  Muse #1 ─┐                     │                                      │                              │
│  Muse #2 ─┤ BLE → egui viewer   │   EEGM raw frames ──────────────▶   │  ZUNA model loaded on GPU    │
│  Muse #3 ─┤   raw + processed   │   EEGM reconstructed frames ◀────   │  Per-headband epoch buffer   │
│  Muse #4 ─┘   per headband      │                                      │  Async inference pipeline    │
└──────────────────────────────────┘                                      └──────────────────────────────┘
```

### Key constraint: ZUNA inference is ~4–5× slower than real-time

A 5-second epoch takes ~20–25 seconds to process on GPU.  The architecture is
therefore a **delayed asynchronous pipeline**:

- Raw EEG streams to the viewer in real-time
- Epochs are queued on the server and processed FIFO
- Reconstructed results arrive delayed (tagged with epoch sequence numbers)
- The viewer overlays processed data when it arrives

---

## Components

### 1. EEGM Protocol (`app/src/eegm.rs` + `inference_server/eegm_protocol.py`)

Extended EEGF format with headband identification:

```
Offset  Type   Value
0       u32    Magic: 0x4545474D ("EEGM")
4       u32    headband_id (0–3)
8       u32    epoch_seq (monotonic per headband)
12      u32    n_channels
16      u32    n_samples per channel
20      f32[]  channel-major payload
```

### 2. eeg-hub (`app/eeg-hub/`)

Rust + egui application:
- **Multi-BLE**: connects to up to 4 Muse headbands simultaneously
- **TCP client**: streams EEGM frames to inference server, receives results
- **Viewer**: per-headband panels showing raw traces + reconstructed overlay
- **Scaffolding**: UI team completes the viewer; core plumbing is in place

### 3. inference-server (`services/inference-server/`)

Python asyncio TCP server:
- **EEGM parser**: reads frames, routes to per-headband epoch buffers
- **Epoch buffer**: accumulates 5s windows (1280 samples @ 256 Hz), 50% overlap hop
- **ZUNA worker thread**: loads model on startup, processes epochs via temp-file shim
  (write .fif → `zuna.preprocessing()` → `zuna.inference()` → `zuna.pt_to_fif()` → read .fif)
- **Result sender**: pushes reconstructed EEGM frames back to eeg-hub
- **Queue management**: drops oldest epochs when queue exceeds 16 (server falling behind)
- **Echo mode**: `--echo` flag returns frames without GPU processing (for testing)

### 4. Test tools

- `eegm-test-sender` (`app/src/bin/eegm_test_sender.rs`): synthetic multi-headband
  EEGM frame generator, connects to inference server, optional response logging

---

## Build

```bash
# ── MacBook (eeg-hub + test tools) ──────────────────────────────────
cd app
cargo build --release --bin eegm-test-sender
cargo build --release --manifest-path eeg-hub/Cargo.toml

# ── GPU Server (inference-server) ───────────────────────────────────
cd services/inference-server
pip install -e .    # or: uv pip install -e .
```

---

## Test: echo mode (no GPU required)

Validates the full TCP pipeline locally without ZUNA:

### Terminal 1 — Start inference server in echo mode
```bash
cd services/inference-server
python -m inference_server.server --port 9100 --echo
```

### Terminal 2 — Start synthetic sender
```bash
cd app
./target/release/eegm-test-sender --target 127.0.0.1:9100 --headbands 2 --read-responses
```

You should see frames being sent and echoed back.

### Terminal 1+2 (with eeg-hub)
```bash
# Terminal 1: inference server (echo)
python -m inference_server.server --port 9100 --echo

# Terminal 2: eeg-hub with server connection (no BLE — will timeout)
./eeg-hub/target/release/eeg-hub --server 127.0.0.1:9100 --no-server
# or just open the viewer without inference:
./eeg-hub/target/release/eeg-hub --no-server
```

---

## Test: GPU inference (RTX 4090 server)

### GPU Server
```bash
cd services/inference-server
python -m inference_server.server --port 9100 --gpu-device 0
```

### MacBook
```bash
cd app
# Synthetic test (no headset):
./target/release/eegm-test-sender --target <GPU_SERVER_IP>:9100 --headbands 1 --read-responses

# Real headband:
./eeg-hub/target/release/eeg-hub --server <GPU_SERVER_IP>:9100 --headbands 1
```

---

## ZUNA Pipeline (per epoch)

```
Raw EEG (5s, 4ch) → MNE RawArray → temp .fif
    → zuna.preprocessing() → .pt (normalized, epoched)
    → zuna.inference()      → .pt (reconstructed, GPU)
    → zuna.pt_to_fif()      → .fif
    → read .fif → extract samples → EEGM response frame
```

Each epoch: ~20–25s on RTX 4090. With 4 headbands producing epochs every 2.5s,
the queue grows at ~1.6 epochs/s but drains at ~0.04–0.05 epochs/s per headband.
The server drops old epochs to stay bounded.

---

## Phased roadmap

| Phase | Status | Scope |
|-------|--------|-------|
| **1** | ✅ Done | EEGM protocol, inference-server skeleton, eeg-hub scaffolding, test sender |
| **2** | Planned | Single headband end-to-end with GPU inference |
| **3** | Planned | Multi-headband GPU batching, epoch priority/scheduling |
| **4** | Planned | UI polish (team), reconnection, latency monitor, recording |

---

## File inventory

```
app/
  src/eegm.rs                          # EEGM protocol (Rust, 5 tests)
  src/bin/eegm_test_sender.rs           # synthetic EEGM sender
  eeg-hub/
    Cargo.toml
    src/main.rs                         # egui viewer scaffolding
    src/muse_multi.rs                   # multi-BLE connection manager
    src/tcp_client.rs                   # TCP client for inference server

services/
  inference-server/
    pyproject.toml
    inference_server/
      __init__.py
      eegm_protocol.py                 # EEGM protocol (Python)
      epoch_buffer.py                   # per-headband epoch accumulation
      zuna_worker.py                    # ZUNA GPU inference wrapper
      server.py                         # async TCP server + main()
```
