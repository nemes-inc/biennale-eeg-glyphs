# muse-rs — Project Analysis

## Architecture

The project is a **Rust BLE client library + two binary frontends** for Interaxon Muse EEG headsets.

### Module Structure

| Module | File | Role |
|---|---|---|
| **`lib.rs`** | `src/lib.rs` | Crate root — declares modules, exports `prelude` |
| **`muse_client`** | `src/muse_client.rs` | BLE scan, connect, GATT subscription, notification dispatch, `MuseHandle` command API |
| **`protocol`** | `src/protocol.rs` | GATT UUIDs, sampling constants, `encode_command` / `decode_response` wire helpers |
| **`parse`** | `src/parse.rs` | Binary decoders — Classic (12-bit EEG, 24-bit PPG, BE IMU) and Athena (tag-based demux for EEG/PPG/IMU/battery) |
| **`types`** | `src/types.rs` | Data types: `EegReading`, `PpgReading`, `ImuData`, `TelemetryData`, `ControlResponse`, `MuseEvent` enum |

### Binaries

| Binary | File | Description |
|---|---|---|
| **`muse-rs`** | `src/main.rs` | Headless CLI — scan, connect, print all decoded events to stdout. Interactive stdin commands (q/p/r/i). |
| **`tui`** | `src/bin/tui.rs` | Full-screen ratatui TUI — real-time EEG + PPG charts, device picker, smooth overlay, simulator mode (`--simulate`). Requires `tui` feature (default). |

### Key Dependencies

| Crate | Purpose |
|---|---|
| **btleplug** | Cross-platform BLE (patched fork for macOS disconnect handling) |
| **tokio** | Async runtime (full features) |
| **ratatui + crossterm** | Terminal UI (optional, behind `tui` feature) |
| **serde / serde_json** | Control JSON parsing |

- **`build.rs`** embeds `Info.plist` into Mach-O `__TEXT,__info_plist` section so macOS grants CoreBluetooth permissions to the CLI binary.
- Uses a [forked btleplug](https://github.com/eugenehp/btleplug/tree/imrpoved_mac_version) with improved macOS disconnect handling, expanded broadcast channel buffers, and null-safety fixes.

---

## Data Flow

```
┌──────────┐  BLE scan   ┌────────────┐  GATT subscribe   ┌──────────────┐
│ Muse     │ ──────────► │ MuseClient │ ──────────────►   │  Peripheral  │
│ Headset  │  advertise  │ (scan/     │  notifications    │  (btleplug)  │
└──────────┘             │  connect)  │                   └──────┬───────┘
                         └────────────┘                          │
                                                                 ▼
                  ┌─────────────────────────────────────────────────────┐
                  │   Firmware auto-detect (Athena char present?)       │
                  ├─────────────────┬───────────────────────────────────┤
                  │  Classic path   │  Athena path                      │
                  │  1 char/sensor  │  1 universal char (tag demux)     │
                  └────────┬────────┴──────────┬────────────────────────┘
                           │                   │
                           ▼                   ▼
                  ┌──────────────────────────────────┐
                  │  parse.rs decoders               │
                  │  decode_eeg_samples()            │
                  │  parse_athena_notification()     │
                  │  ControlAccumulator              │
                  └──────────────┬───────────────────┘
                                 │
                                 ▼ mpsc::channel<MuseEvent>
                  ┌──────────────────────────────────┐
                  │  Consumer (main.rs or tui.rs)    │
                  │  rx.recv().await → match event   │
                  └──────────────────────────────────┘
```

1. **Scan** — `MuseClient` creates a BLE `Manager`, scans for devices matching the `"Muse"` name prefix.
2. **Connect** — `setup_peripheral()` connects, discovers GATT services, detects firmware (Athena if char `0x273e0013` exists).
3. **Subscribe** — Classic: subscribes to individual EEG/PPG/IMU/telemetry/control chars. Athena: subscribes to the single universal sensor char + control.
4. **Dispatch** — A spawned tokio task reads notifications, decodes them via `parse.rs`, and sends typed `MuseEvent`s through an `mpsc` channel.
5. **Consume** — The CLI prints events; the TUI pushes samples into ring buffers and renders waveform charts at ~30 FPS.
6. **Commands** — `MuseHandle` exposes `start()`, `pause()`, `resume()`, `disconnect()`, `send_command()` — all write to the control characteristic.

---

## Firmware Support

### Auto-detection

At connect time the library inspects the GATT service table. If the **universal sensor characteristic** (`273e0013-…`) is present, the device is running Athena firmware. Classic devices do not expose this characteristic.

### Classic (Muse 1, Muse 2, Muse S ≤ fw 3.x)

- One dedicated GATT characteristic per sensor
- EEG: 12-bit BE packed, 12 samples/pkt, 256 Hz, scale 0.48828125 µV/LSB
- PPG: 24-bit BE unsigned, 6 samples/pkt, 64 Hz
- IMU: i16 BE, accel 0.0000610352 g/LSB, gyro +0.0074768 °/s/LSB
- Startup: `h` → `s` → preset (`p21`/`p20`/`p50`) → `d`

### Athena (Muse S fw ≥ 4.x)

- All sensors multiplexed on one characteristic (`273e0013`) via tag-based binary framing
- EEG: 14-bit LE packed, 2 or 4 samples/pkt (8ch or 4ch), scale 0.0885 µV/LSB
- PPG: 20-bit LE packed, 3 samples × 4 channels, 64 Hz
- IMU: i16 LE, gyro scale negated (−0.0074768 °/s/LSB)
- Startup: `v4` → `s` → `h` → `p1045` → `dc001` × 2 → `d` (fallback) → `L1` → 2 s wait

### MuseEvent Variants

| Variant | Classic | Athena |
|---|---|---|
| `Eeg` (ch 0–3) | ✓ | ✓ |
| `Eeg` (ch 4–7) | ✗ | ✓ |
| `Ppg` | ✓ (opt-in) | ✓ |
| `Accelerometer` | ✓ | ✓ |
| `Gyroscope` | ✓ | ✓ |
| `Telemetry` | ✓ | ✓ |
| `Control` | ✓ | ✓ |

---

## Build & Run Commands

### Prerequisites

- **Rust ≥ 1.86** (ratatui 0.30.0 requirement; tested with rustc 1.94.0)
- **macOS**: CoreBluetooth — handled automatically by `build.rs` + `Info.plist`
- **Linux**: `sudo apt-get install libdbus-1-dev pkg-config`

### Build

```bash
cargo build --release              # lib + both binaries (tui feature on by default)
cargo build --no-default-features  # lib + headless CLI only (no ratatui/crossterm)
```

### Run

```bash
cargo run --release                        # headless CLI: scan → connect → print events
cargo run --release --bin tui              # TUI: scan → auto-connect → waveform viewer
cargo run --release --bin tui -- --simulate  # TUI simulator (no hardware needed)
```

### Logging

```bash
RUST_LOG=debug cargo run --release                 # verbose CLI
RUST_LOG=debug cargo run --release --bin tui        # TUI logs → muse-tui.log
RUST_LOG=muse_rs=debug cargo run --release          # library logs only
```

---

## Verified Status (2026-03-13)

| Check | Result |
|---|---|
| `cargo build --release` | ✅ Compiles cleanly (rustc 1.94.0, macOS aarch64) |
| `cargo run --bin tui -- --simulate` | ✅ TUI simulator runs, renders EEG waveforms |
| Hardware test (BLE connect) | ⏳ Not yet tested |
