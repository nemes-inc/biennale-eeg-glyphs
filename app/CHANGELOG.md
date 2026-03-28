# v0.6.0

## March 27, 2026

## Per-dimension all-devices mode

Settings gear icon next to dimension controls. "All devices" checkbox sends each Start/Stop command to every connected Muse simultaneously — click "Start Absorption" once and it starts on all headsets.

## Detectors run until stop

Entrainment and Approach detectors no longer auto-complete at their default windows (40s and 20s). All detectors run continuously until you click Stop. Progress buttons show elapsed seconds.

---

# v0.5.0

## March 27, 2026

## Per-device BLE reconnect

Reconnect button on disconnected devices re-scans, auto-matches by `device_id`, and respawns the BLE task on the existing slot — preserving plot data, session file, and analysis state. Tab blinks red for 5 seconds on disconnect, status label shows Connected/Disconnected/Reconnecting.

## Color-coded device tabs

Each tab gets a painted circle dot with a light outline. Colors auto-assigned from `devices.json` (suffix → color mapping: 3623→green, 361d→blue, c346→white, 1bd5→black). Manual override via color picker button that cycles through 5 options.

## Domain vocabulary for dimension labels

Detector outputs now use glyph domain labels instead of generic signal labels. Absorption: Deep/Surface. Unknown: Lean In/Hold Back. Witnessed: Approach/Withdraw. Print selector and viz panels show the same vocabulary. Split `asymmetry_to_value()` and `snapshot_approach()` into per-dimension functions.

## Auto-stop detectors on BLE disconnect

Running dimension detectors (Settling/Measuring) reset to Idle when BLE drops. Prevents stale readings from persisting after headset disconnects.

---

# v0.4.0

## March 27, 2026

## Alpha trend replaces baseline + absorption

Replaced the 60-second baseline calibration and absorption detector with `AlphaTrendDetector` — online linear regression on frontal alpha (AF7 + AF8 mean). 5-second settling, runs continuously, classifies slope as Rising/Flat/Falling. Glyph mapping: Rising → deep, else → surface.

## Remote glyph printing

Viewer sends `POST /print` to the admin HTTP endpoint (port + 1) with dimension values as JSON. Server runs `print_receipt` locally where the USB thermal printer is attached.

---

# v0.3.0

## March 26, 2026

## Dimension analysis

Four real-time psychological measurements: Absorption, Attunement (ACT entrainment), The Unknown, and Witnessed (frontal asymmetry). A signal pipeline in the BLE task runs bandpass filtering, artifact rejection, and FFT at ~250ms hops, producing typed state-machine snapshots for the UI.

## Two visualization modes

**Neural Aura** — neon arc gauge cards with glow strokes, sparklines, and progress bars in a right sidebar. **Brain Topography** — compile-time-parsed OBJ wireframe head with depth-based alpha, fan-triangulated electrode glow meshes, frontal asymmetry bar, and compact dimension status cards.

## Simulate mode

`--simulate` spawns a fake Muse with per-channel amplitude modulation feeding the real pipeline. Full measurement flow without Bluetooth.

---

# v0.2.0

## March 18, 2026

## Admin endpoint for remote restart

Redeploying the inference server meant SSH-ing into the GPU box, killing the process, pulling code, and restarting — which is annoying when you're iterating on timestamp bugs from across the room. The server now runs a zero-dependency HTTP admin endpoint on TCP port + 1 (so 9101 if the main server is on 9100) using raw asyncio. `GET /status` returns JSON stats (uptime, frames, epochs, workers, client connected). `POST /restart` runs `git pull --ff-only` in the repo root, cleanly shuts down all ZUNA workers with poison pills, tears down the mmap job queue, and `os.execv`s the process with the same arguments so it comes back up on the same port with updated code. `POST /shutdown` does the clean teardown without restart. No auth — this is a local network admin tool, not a public API.

## Timestamp-aligned inference overlay

Reconstructed EEG from the inference server used to just get appended to the end of the plot buffer — so if ZUNA took 30 seconds to process an epoch, the red overlay would show up 30 seconds to the right of where the original data was. Useless for visual comparison. The EEGM wire format now carries a `timestamp_us` field (u64, microseconds) in every frame header, bumping it from 20 to 28 bytes. The Rust BLE loop stamps each outgoing frame with `SystemTime::now()`. The Python server threads that timestamp through the epoch buffer (advancing by hop duration on each 50% overlap), through ZUNA inference, and back out in the response frame. The viewer stores plot data as `[f64; 2]` (time_seconds, amplitude) instead of plain `f32`, computes relative time from the first frame's timestamp, and places reconstructed data at the exact x-position where it was originally captured. If the server sends `timestamp_us=0` (old server, echo mode), the viewer falls back to placing data at the current end of the raw timeline.

## Multi-Muse EEG Viewer

The old viewer talked to exactly one Muse. Running a multi-headband installation meant running multiple viewer processes and praying they didn't fight over CoreBluetooth. Now the viewer supports up to 4 simultaneous Muse headsets through a single shared tokio runtime. Each device gets its own BLE task, its own tab, and its own recording session. The scan-then-pick flow uses the muse-rs `scan_all()` + `connect_to(device)` API so each connection reuses the adapter handle from discovery instead of spawning a fresh CBCentralManager.

Streaming to the inference server used to be fire-and-forget over TCP. If the Python server crashed mid-session you lost every frame in flight and had no way to replay. The BLE loop now writes each EEGM frame to a per-device session file on disk before pushing it to the TCP channel. A `frames_sent` counter per device tracks what actually made it to the server. When you reconnect, the TCP client reads each session file from the last-sent offset and replays the gap automatically. No user interaction required, no data lost. Session files are just concatenated EEGM binary frames so they're self-describing and trivially replayable.

The bottom status bar was missing entirely. You had no idea whether the server was connected, how many frames were flowing, or if anything was stuck. Now the bar shows server connection state, local frames processed, frames written to disk, frames sent to server, and a yellow pending count when the two diverge. Boring but essential.

Per-device recording was a natural consequence of the multi-device split. Each tab has its own Record/Stop button, its own accumulation buffer, and its own FIF output path with the device name baked in for disambiguation. The ZUNA pipeline preprocess button works per-tab too.

Legacy stdin and TCP pipe modes still work unchanged with --stdin or --tcp.

## Outbound channel was silently dead

The BLE-to-server pipeline looked connected but was shipping zero frames. The outbound channel was created inside `connect_to_server()`, but BLE tasks received their sender clone earlier in `connect_device()` — so they got `None` and quietly dropped every frame. Worse, the TCP recv task built a static device map at connect time, meaning any Muse that connected after the server got no routing entry at all. We replaced the `Option<OutboundTx>` with a `SharedOutboundTx` backed by `Arc<Mutex<Option<Sender>>>` so BLE tasks always read the latest sender, and swapped the static device map for a shared `Arc<Mutex<HashMap>>` (the standard Tokio pattern) that updates live as devices connect and disconnect. The status bar "pending" count now actually drains.

## Inference server shutdown was noisy

Ctrl+C on the Python inference server produced a `CancelledError` traceback and a "Task was destroyed but pending" warning because `_send_results` was an orphaned asyncio task. The fix stores the task handle, cancels it explicitly in `shutdown()`, catches `CancelledError` in the coroutine, and drains pending tasks with `asyncio.gather(*pending, return_exceptions=True)` before closing the event loop. Both SIGTERM and SIGINT now exit cleanly with code 0 and zero stderr noise.

---

# v0.1.0

First feature-complete release of `muse-rs` — an async Rust library and terminal UI for streaming real-time sensor data from Interaxon Muse EEG headsets over Bluetooth Low Energy.

## Highlights

- **Full Athena firmware support** — automatic protocol detection for Muse S devices running the newer Athena firmware (tag-based multiplexed packets on a single BLE characteristic)
- **PPG (optical) decoding** — both Classic (24-bit BE) and Athena (20-bit LE packed) PPG data decoded into `MuseEvent::Ppg` events with 3 channels (ambient, infrared, red) at 64 Hz
- **Real-time TUI** with EEG and PPG views, device picker, smooth overlay, and auto-reconnect
- **Cross-platform** — Linux (BlueZ), macOS (CoreBluetooth), Windows (WinRT)

## What's included

### Library (`muse-rs`)

| Sensor | Classic | Athena |
|---|---|---|
| EEG (4ch / 8ch) | ✓ 12-bit BE, 256 Hz | ✓ 14-bit LE, 256 Hz |
| PPG / Optical | ✓ 24-bit BE, 64 Hz | ✓ 20-bit LE, 64 Hz |
| Accelerometer | ✓ | ✓ |
| Gyroscope | ✓ | ✓ |
| Battery | ✓ u16 BE / 512 | ✓ u16 LE / 256 |
| Control JSON | ✓ | ✓ (fragment reassembly) |
| Disconnect detection | ✓ | ✓ (adapter event stream) |

### TUI (`cargo run --bin tui`)

- **EEG view** — 4-channel scrolling braille waveforms (TP9, AF7, AF8, TP10) with per-channel min/max/RMS stats, clipping indicator, and configurable ±µV scale
- **PPG view** — 3-channel optical waveforms (ambient, infrared, red) with auto-scaling Y axis
- **Smooth overlay** — dim raw trace + bright 9-sample moving average (toggle with `v`)
- **Device picker** — scan, select, and switch between multiple Muse devices
- **Auto-reconnect** — automatic rescan after unexpected disconnect
- **Simulator** — `--simulate` flag for UI development without hardware

### Console streamer (`cargo run`)

- Prints all decoded events (EEG, PPG, IMU, battery, control) to stdout
- Interactive commands: pause, resume, device info, raw command passthrough

## Athena protocol support

The Athena decoder handles the full tag-based packet format documented by the [OpenMuse](https://github.com/DominiqueMakowski/OpenMuse) project:

| Tag | Sensor | Payload |
|---|---|---|
| `0x11` | EEG 4ch (4 samples) | 28 B |
| `0x12` | EEG 8ch (2 samples) | 28 B |
| `0x34` | Optical 4ch (3 samples) | 30 B |
| `0x35` | Optical 8ch (2 samples) | 40 B |
| `0x36` | Optical 16ch (1 sample) | 40 B |
| `0x47` | IMU accel+gyro (3 samples) | 36 B |
| `0x53` | DRL/REF | 24 B |
| `0x88` | Battery (new fw, variable) | 188–230 B |
| `0x98` | Battery (old fw) | 20 B |

### Transitional firmware compatibility

Muse S devices with firmware 3.x on Athena hardware expose the Athena sensor characteristic but reject the `dc001` data-start command (rc:69). The library sends both `dc001` and the Classic `d` command as fallback, ensuring streaming starts regardless of firmware version.

## macOS improvements

Uses a [fork of btleplug](https://github.com/eugenehp/btleplug/tree/imrpoved_mac_version) with:

- Reliable disconnect detection via `CentralEvent::DeviceDisconnected`
- Expanded broadcast channel buffers (16 → 256)
- Null-safety improvements preventing hangs on unreachable peripherals
- Single adapter instance reuse (avoids duplicate `CBCentralManager` creation)

## Quick start

```rust
use muse_rs::prelude::*;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let client = MuseClient::new(MuseClientConfig::default());
    let (mut rx, handle) = client.connect().await?;
    handle.start(false, false).await?;

    while let Some(event) = rx.recv().await {
        match event {
            MuseEvent::Eeg(r) => println!("EEG ch{}: {:.2} µV", r.electrode, r.samples[0]),
            MuseEvent::Ppg(r) => println!("PPG ch{}: {:?}", r.ppg_channel, r.samples),
            MuseEvent::Disconnected => break,
            _ => {}
        }
    }
    Ok(())
}
```

## Install

```shell
cargo add muse-rs
```

Or as a git dependency:

```toml
[dependencies]
muse-rs = { git = "https://github.com/eugenehp/muse-rs.git", tag = "v0.1.0" }
```

## Acknowledgments

- [OpenMuse](https://github.com/DominiqueMakowski/OpenMuse) by Dominique Makowski — Python Athena decoder used as reference for packet structure, payload sizes, and battery scaling
- [muse-js](https://github.com/urish/muse-js) by Uri Shaked — original Muse Web Bluetooth implementation
- [btleplug](https://github.com/deviceplug/btleplug) — cross-platform BLE for Rust

**Full Changelog**: https://github.com/eugenehp/muse-rs/commits/v0.1.0
