# MacBook Setup: Stream Muse EEG to PC over TCP

This guide is for the **MacBook** side only. The PC/WSL side is already set up and waiting.

---

## What this does

```
MacBook (you are here)              PC / WSL
┌──────────────────────┐    TCP     ┌─────────────────────┐
│ Muse ──BLE──▶ sender │ ────────▶  │ eeg-viewer (GPU)    │
│ muse-stream-tcp      │   EEGF    │ live plot + ZUNA    │
└──────────────────────┘  frames   └─────────────────────┘
```

The MacBook captures Muse EEG via Bluetooth and streams it over the local network to the PC, where `eeg-viewer` plots it live and can run ZUNA inference on the GPU.

---

## 1. Prerequisites

- **Rust toolchain** — install if missing:
  ```bash
  curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
  source ~/.cargo/env
  ```
- **Xcode Command Line Tools**:
  ```bash
  xcode-select --install
  ```
- **Bluetooth enabled**, Muse headset powered on

---

## 2. Clone the repo (if not already)

```bash
git clone https://github.com/nemes-inc/biennale-eeg-glyphs.git
cd biennale-eeg-glyphs
```

---

## 3. Build the sender binary

```bash
cd app
cargo build --release --bin muse-stream-tcp
```

The binary is at `app/target/release/muse-stream-tcp`.

---

## 4. Get the PC's IP address

Ask the PC operator, or on the PC run (in PowerShell):

```powershell
ipconfig | findstr "IPv4"
```

Use the **LAN IP** (e.g. `192.168.1.50`), not `127.0.0.1`.

---

## 5. Confirm the PC is listening

The PC should already be running:

```bash
# On the PC / WSL:
eeg-viewer --tcp 0.0.0.0:9000
```

You can verify from the MacBook:

```bash
nc -zv <PC_IP> 9000
```

If you see `Connection to ... port 9000 [tcp/*] succeeded!`, you're good.

---

## 6. Start streaming

Replace `<PC_IP>` with the actual IP:

```bash
cd app
./target/release/muse-stream-tcp --target <PC_IP>:9000
```

That's it. The binary will:
1. Scan for a Muse headset via BLE (~30 s timeout)
2. Connect and start streaming EEG
3. Send EEGF frames to the PC over TCP

You should see live EEG traces appear in the PC's eeg-viewer window.

---

## CLI options

| Flag | Default | Description |
|------|---------|-------------|
| `--target ADDR:PORT` | *(required)* | PC's eeg-viewer TCP address |
| `--scan-timeout SECS` | `30` | BLE scan timeout |
| `--name-prefix STR` | `"Muse"` | BLE device name filter |
| `--chunk N` | `256` | Samples/channel per frame (256 ≈ 1 s at 256 Hz) |
| `--tcp-retry` | off | Retry TCP connection on failure |

### Lower latency

```bash
./target/release/muse-stream-tcp --target <PC_IP>:9000 --chunk 64
```

Sends frames 4× more often (~250 ms intervals instead of ~1 s).

---

## Troubleshooting

| Problem | Fix |
|---------|-----|
| `Muse not found` | Ensure Muse is on (solid light), Bluetooth enabled. Try `--scan-timeout 60` |
| `Connection refused` | Confirm eeg-viewer is running on the PC with `--tcp 0.0.0.0:9000`. Check PC firewall |
| `TCP write failed` | eeg-viewer on PC was closed or crashed. Restart it and re-run the sender |
| Build fails on macOS | Run `xcode-select --install`. Ensure `rustup` is installed |

---

## Stop

Press **Ctrl-C** to stop the sender. The Muse BLE connection and TCP socket are closed cleanly.
