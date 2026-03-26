# v0.2.0

## March 26, 2026

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
