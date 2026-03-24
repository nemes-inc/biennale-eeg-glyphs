# Streaming Muse EEG from MacBook to PC/WSL over TCP

This tutorial sets up a two-machine pipeline:

```
┌─────────────────────────┐        TCP (EEGF)        ┌──────────────────────────────┐
│  MacBook (sender)       │ ───────────────────────▶  │  PC / WSL (receiver)         │
│                         │                           │                              │
│  Muse headset ─── BLE   │                           │  eeg-viewer --tcp            │
│  muse-stream-tcp        │                           │  (live plot + Record/ZUNA)   │
└─────────────────────────┘                           └──────────────────────────────┘
```

- **MacBook**: pairs with the Muse via Bluetooth, captures EEG, and sends binary EEGF frames over TCP.
- **PC (WSL)**: receives the stream in `eeg-viewer`, plots live traces, and can trigger ZUNA inference on the GPU.

This avoids all BLE-over-WSL headaches — the MacBook handles Bluetooth natively, the PC handles compute.

---

## Prerequisites

| Machine | What you need |
|---------|---------------|
| **MacBook** | Rust toolchain (`rustup`), Xcode Command Line Tools, Bluetooth enabled, Muse headset paired |
| **PC / WSL** | Rust toolchain, system dev packages (see main README), `uv sync` in `services/zuna`, NVIDIA GPU drivers (optional, for ZUNA) |

Both machines need the project repo cloned.

---

## 1. Build on both machines

### MacBook (sender)

```bash
cd app
cargo build --release --bin muse-stream-tcp
```

This produces `app/target/release/muse-stream-tcp`.

### PC / WSL (receiver)

```bash
cd app
cargo build --release --bin muse-record-fif
cargo build --release --manifest-path eeg-viewer/Cargo.toml
```

If you haven't already, also install the ZUNA Python environment:

```bash
cd services/zuna
uv sync
```

---

## 2. Find your PC's IP address

On the PC, find the IP address that the MacBook can reach. In WSL:

```bash
# WSL's virtual IP (reachable from the Windows host, but NOT from the MacBook):
ip addr show eth0 | grep 'inet '

# Windows host IP on the local network (reachable from the MacBook):
# Run in PowerShell (not WSL):
ipconfig | findstr "IPv4"
```

You need the **Windows host LAN IP** (e.g. `192.168.1.50`). Since WSL shares the Windows host's network in mirrored/NAT mode, traffic arriving at the Windows IP on a given port will reach WSL if the port is open.

> **WSL2 networking note**: By default, WSL2 uses NAT. Ports that WSL listens on are usually forwarded automatically on newer Windows 11 builds (with `networkingMode=mirrored` in `.wslconfig`). If not, you may need a port-forward rule — see [Section 5: Troubleshooting](#5-troubleshooting).

---

## 3. Start the receiver (PC / WSL)

Pick a port (e.g. `9000`) and start the viewer in TCP listen mode:

```bash
cd app/eeg-viewer
./target/release/eeg-viewer --tcp 0.0.0.0:9000
```

The viewer window opens and waits for a TCP connection. The status bar will say **"Batch: idle (plot from TCP)"** until the MacBook connects.

### Binding to `0.0.0.0`

Using `0.0.0.0` makes eeg-viewer listen on all interfaces, so the MacBook can reach it via the Windows LAN IP. If you want to restrict it to the WSL-internal interface, use `127.0.0.1:9000` — but then only local connections work.

---

## 4. Start the sender (MacBook)

Replace `192.168.1.50` with your PC's actual LAN IP:

```bash
cd app
./target/release/muse-stream-tcp --target 192.168.1.50:9000
```

The binary will:

1. **Scan** for a Muse headset via BLE (~15 s timeout by default).
2. **Connect** and start streaming EEG.
3. **Send EEGF frames** over TCP — one frame every ~1 second (256 samples at 256 Hz per channel).

You should see live EEG traces appear in the eeg-viewer window on the PC almost instantly.

### Sender CLI options

| Flag | Default | Meaning |
|------|---------|---------|
| `--target ADDR:PORT` | *(required)* | IP:port of the eeg-viewer TCP listener |
| `--scan-timeout SECS` | `30` | BLE scan timeout |
| `--name-prefix STR` | `"Muse"` | BLE device name filter |
| `--chunk N` | `256` | Samples per channel per EEGF frame |
| `--tcp-retry` | off | Keep retrying TCP connection on failure |

### Example: longer scan, smaller frames (lower latency)

```bash
./target/release/muse-stream-tcp \
    --target 192.168.1.50:9000 \
    --scan-timeout 60 \
    --chunk 64
```

Smaller `--chunk` means frames are sent ~4× more often (every ~250 ms) for lower display latency, at the cost of more TCP overhead.

---

## 5. Troubleshooting

### WSL port forwarding (if MacBook can't connect)

If your WSL2 instance doesn't have `networkingMode=mirrored`, you may need to forward the port from Windows to WSL. In an **Administrator PowerShell**:

```powershell
# Find WSL's internal IP
wsl hostname -I
# e.g. 172.25.160.1

# Forward port 9000 from all Windows interfaces to WSL
netsh interface portproxy add v4tov4 listenport=9000 listenaddress=0.0.0.0 connectport=9000 connectaddress=172.25.160.1

# Verify
netsh interface portproxy show all
```

Also ensure the Windows Firewall allows inbound TCP on port 9000:

```powershell
New-NetFirewallRule -DisplayName "EEG Viewer TCP" -Direction Inbound -Protocol TCP -LocalPort 9000 -Action Allow
```

To remove the proxy later:

```powershell
netsh interface portproxy delete v4tov4 listenport=9000 listenaddress=0.0.0.0
```

### Mirrored networking (recommended, Windows 11)

Add to `%USERPROFILE%\.wslconfig`:

```ini
[wsl2]
networkingMode=mirrored
```

Then `wsl --shutdown` and restart WSL. With mirrored mode, WSL shares the host's IP — no port forwarding needed.

### "Connection refused" from MacBook

1. Confirm eeg-viewer is running and listening: `ss -tlnp | grep 9000` inside WSL.
2. Test from the MacBook: `nc -zv 192.168.1.50 9000`.
3. Check Windows Firewall (see above).

### Muse not found (MacBook)

- Ensure Bluetooth is on and the Muse is powered on (solid light).
- Try increasing scan timeout: `--scan-timeout 60`.
- Muse S/2 headsets use "Muse" as the default prefix — the `--name-prefix` filter should match.

### High latency or choppy display

- Reduce `--chunk` to 64 or 128 for more frequent, smaller frames.
- Ensure both machines are on the same LAN (not across a VPN/WAN).
- Wi-Fi 5 GHz band is preferred over 2.4 GHz.

---

## 6. Full workflow: capture → ZUNA → view comparison

Once the live stream is running on the PC, you can also use the MacBook to **record** and then run **ZUNA inference** entirely on the PC side, using sample data or the `run_fif_pipeline.py` script:

```bash
# On PC / WSL — run ZUNA on sample data (or your own .fif captures):
cd services/zuna
uv run python run_fif_pipeline.py \
    --input ./zuna_repo/tutorials/data/1_fif_input \
    --work-dir ./test_run \
    --headless
```

The eeg-viewer's **Record** and **Preprocess** toolbar buttons are designed for local Muse BLE mode. When using TCP mode, recording and ZUNA processing can be done separately via the CLI.

---

## 7. Protocol reference: EEGF binary frame

For anyone writing a custom sender, the EEGF frame format (little-endian) is:

| Offset | Type | Value |
|--------|------|-------|
| 0 | `u32` | Magic: `0x45454746` (ASCII `"EEGF"`) |
| 4 | `u32` | `n_channels` (e.g. 4 for Muse) |
| 8 | `u32` | `n_samples` per channel |
| 12 | `f32[n_channels × n_samples]` | Channel-major: all samples for ch0, then ch1, … |

Total frame size: `12 + 4 × n_channels × n_samples` bytes.

There's also **EEGD** (magic `0x45454744`) which carries two blocks (original + reconstructed) in the same layout — used by `fif_pair_to_eegd.py` for before/after comparison viewing.

---

## Quick-start cheat sheet

```bash
# ── PC / WSL (receiver) ──────────────────────────────────────────
cd app/eeg-viewer
./target/release/eeg-viewer --tcp 0.0.0.0:9000

# ── MacBook (sender) ─────────────────────────────────────────────
cd app
./target/release/muse-stream-tcp --target <PC_LAN_IP>:9000
```
