# eeg-viewer

Muse-focused EEG GUI.

1. Live Bluetooth Muse EEG plotted in-process (no TCP hop).
2. Record — runs `muse-record-fif`, writes `live_<timestamp_ms>.fif` under `--zuna-dir`/live_captures/. Does not run ZUNA. BLE preview pauses for the capture, then resumes.
3. Preprocess — same capture, then `uv run python run_fif_pipeline.py` in `--zuna-dir` (toolbar preset → `--preset`; `--headless`). On success, preprocessed vs reconstructed FIFs load as EEGD dual traces. Live streaming stays paused until Resume Muse so the comparison is not overwritten.
4. Stdin / TCP — EEGF or EEGD binary frames from any producer that speaks the protocol in `src/main.rs`.

## Build

`muse-record-fif` defaults to `app/target/release/muse-record-fif`. Build from `app/`:

```bash
cd app
cargo build --release --bin muse-record-fif
cargo build --release --manifest-path eeg-viewer/Cargo.toml
```

Or from `app/eeg-viewer`:

```bash
cd app/eeg-viewer
cargo build --release
cd .. && cargo build --release --bin muse-record-fif
```

## Run (default — Muse BLE)

```bash
cd app/eeg-viewer
./target/release/eeg-viewer
```

Needs a paired Muse and Bluetooth enabled. Record needs `muse-record-fif` at the default path (or `--muse-record-bin`) and `muse_eeg_to_fif.py` under `--zuna-dir`. Preprocess also needs `uv` on PATH and `uv sync` in `services/zuna`.

## Stdin (EEGF / EEGD pipe)

```bash
./target/release/eeg-viewer --stdin
# producer | ./target/release/eeg-viewer --stdin
```

## TCP

The viewer listens for one client and reads EEGF/EEGD frames:

```bash
./target/release/eeg-viewer --tcp 127.0.0.1:9810
```

## CLI

| Flag | Meaning |
|------|---------|
| `--stdin` | Read EEGF/EEGD from stdin; disables Muse BLE. |
| `--tcp ADDR` | Listen for one TCP client; disables Muse BLE. |
| `--no-muse-ble` | No Muse BLE thread; empty plot unless `--stdin` / `--tcp`. |
| `--max-points N` | Rolling buffer length per channel (default 8192). |
| `--zuna-dir PATH` | ZUNA root containing `run_fif_pipeline.py`, `live_captures/`, `muse_eeg_to_fif.py`. |
| `--muse-record-bin PATH` | Path to `muse-record-fif` binary. |

Defaults: `--zuna-dir` is `../../services/zuna` relative to this crate; `--muse-record-bin` is `../target/release/muse-record-fif`.
