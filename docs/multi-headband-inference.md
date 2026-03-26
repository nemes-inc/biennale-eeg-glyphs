# Multi-Headband EEG Streaming + GPU Inference

## Architecture

```
┌──────────────────────────────────┐       TCP (EEGC + EEGM)              ┌──────────────────────────────┐
│  MacBook — eeg-hub               │ ──────────────────────────────────▶  │  GPU Server — inference-server│
│                                  │ ◀──────────────────────────────────  │  (Ubuntu + RTX 4090)         │
│  Muse #1 ─┐                     │                                      │                              │
│  Muse #2 ─┤ BLE → egui viewer   │   1. EEGC ConnectReq ───────────▶   │  ZUNA model loaded on GPU    │
│  Muse #3 ─┤   raw + processed   │   2. EEGC ConnectAck ◀──────────    │  Per-headband epoch buffer   │
│  Muse #4 ─┘   per headband      │   3. EEGM raw frames ──────────▶   │  Async inference pipeline    │
│                                  │   4. EEGM reconstructed ◀──────    │                              │
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

### 1. Protocol (`app/src/eegm.rs` + `inference_server/eegm_protocol.py`)

Two message types share the wire, distinguished by their magic bytes:

#### EEGC — Control messages (24 bytes, fixed size)

```
Offset  Type   Value
0       u32    Magic: 0x45454743 ("EEGC")
4       u32    msg_type (1 = CONNECT_REQ, 2 = CONNECT_ACK)
8       u32    protocol_version (currently 1)
12      u32    n_headbands (1–4)
16      u32    sample_rate (e.g. 256)
20      u32    status (ACK only: 0 = OK, non-zero = error)
```

#### EEGM — Data frames (20 + payload bytes)

```
Offset  Type   Value
0       u32    Magic: 0x4545474D ("EEGM")
4       u32    headband_id (0–3)
8       u32    epoch_seq (monotonic per headband)
12      u32    n_channels
16      u32    n_samples per channel
20      f32[]  channel-major payload
```

#### Connection handshake

```
Hub                          Server
 │  ── TCP connect ──────────▶ │
 │  ── EEGC ConnectReq ──────▶ │  (n_headbands, sample_rate, version)
 │  ◀── EEGC ConnectAck ────── │  (status=0 → OK, or error code)
 │  ── EEGM data frames ─────▶ │  (streaming begins)
 │  ◀── EEGM reconstructed ─── │  (delayed results)
```

The server enforces a 10-second handshake timeout. If the first message is not
a `ConnectReq`, or `n_headbands` is out of range, the server replies with a
non-zero status and closes the connection.

### 2. eeg-hub (`app/eeg-hub/`)

Rust + egui application:
- **Multi-BLE**: connects to up to 4 Muse headbands simultaneously
- **TCP client**: performs EEGC handshake, then streams EEGM frames bidirectionally
- **Viewer**: per-headband panels showing raw traces + reconstructed overlay
- **Connection status**: UI toolbar shows handshake progress (`Connecting…` → `Handshake…` → `Connected (N band(s), 256 Hz)`)
- **Scaffolding**: UI team completes the viewer; core plumbing is in place

### 3. inference-server (`services/inference-server/`)

Python asyncio TCP server:
- **Handshake**: waits for `ConnectReq`, validates headband count, replies `ConnectAck`
- **EEGM parser**: reads data frames, routes to per-headband epoch buffers
- **Epoch buffer**: accumulates 5s windows (1280 samples @ 256 Hz), 50% overlap hop
- **Worker pool**: N parallel ZUNA model instances (default 4, configurable via `--workers`)
  - Each worker loads its own model (~300–400 MB VRAM)
  - Workers pull jobs from a shared thread-safe queue
  - With RTX 4090 (24 GB), 8–16 workers fit comfortably
  - Server waits for all workers to load before accepting connections
  - Clean shutdown via poison pills
- **ZUNA pipeline** (per worker, per epoch): temp-file shim
  (write .fif → `zuna.preprocessing()` → `zuna.inference()` → `zuna.pt_to_fif()` → read .fif)
- **Result sender**: pushes reconstructed EEGM frames back to eeg-hub
- **Queue management**: drops epochs when queue exceeds 64 (server falling behind)
- **Echo mode**: `--echo` flag returns frames without GPU processing (for testing)

### 4. Test tools

- `eegm-test-sender` (`app/src/bin/eegm_test_sender.rs`): synthetic multi-headband
  EEGM frame generator, performs EEGC handshake then streams, optional response logging

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

You should see the EEGC handshake logged on both sides, then frames being sent and echoed back.

### Terminal 1+2 (with eeg-hub)
```bash
# Terminal 1: inference server (echo mode, no GPU)
python -m inference_server.server --port 9100 --echo

# Terminal 2: eeg-hub connected to server
./eeg-hub/target/release/eeg-hub --server 127.0.0.1:9100 --headbands 1
# or viewer-only, no inference:
./eeg-hub/target/release/eeg-hub --no-server
```

---

## Test: GPU inference (RTX 4090 server)

### GPU Server
```bash
cd services/inference-server
python -m inference_server.server --port 9100 --gpu-device 0 --workers 8
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

### Throughput analysis

Each epoch: ~20–25s on RTX 4090 with a single worker.

| Workers | VRAM used | Throughput (epochs/s) | Can keep up with |
|---------|-----------|----------------------|-------------------|
| 1       | ~400 MB   | ~0.04–0.05           | — (4–5× behind)    |
| 4       | ~1.6 GB   | ~0.16–0.20           | — (still behind)  |
| 8       | ~3.2 GB   | ~0.32–0.40           | 1 headband        |
| 16      | ~6.4 GB   | ~0.64–0.80           | 2 headbands       |

With 4 headbands producing epochs every 2.5s (50% overlap), the ingest rate is
~1.6 epochs/s. Even with 16 workers the server falls behind, so the queue
caps at 64 and drops oldest epochs. More workers = less delay.

> **Note**: Actual GPU parallelism depends on compute contention. The numbers
> above assume linear scaling; real throughput may plateau. Profile on the
> target RTX 4090 to find the sweet spot.

---

## Phased roadmap

| Phase | Status | Scope |
|-------|--------|-------|
| **1** | ✅ Done | EEGM/EEGC protocol (data + connect handshake), inference-server with worker pool, eeg-hub scaffolding, test sender |
| **2** | Planned | Single headband end-to-end with GPU inference |
| **3** | Planned | Multi-headband GPU batching, epoch priority/scheduling |
| **4** | Planned | UI polish (team), reconnection, latency monitor, recording |

---

## File inventory

```
app/
  src/eegm.rs                          # EEGM/EEGC protocol (Rust, 8 tests)
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
