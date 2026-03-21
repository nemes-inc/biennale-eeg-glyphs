//! Record Muse EEG over BLE, write aligned CSV, then call MNE Python to save FIF.
//!
//! ```text
//! cargo run -p muse-rs --bin muse-record-fif -- -o ./recording_raw.fif --duration 120
//! ```

use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::PathBuf;
use std::process::Command;
use std::time::Duration;

use anyhow::{anyhow, Context, Result};
use clap::Parser;
use log::info;
use uuid::Uuid;

use muse_rs::muse_client::{MuseClient, MuseClientConfig};
use muse_rs::protocol::EEG_FREQUENCY;
use muse_rs::types::MuseEvent;

/// Default path to `muse_eeg_to_fif.py`
const DEFAULT_SCRIPT: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../services/zuna/muse_eeg_to_fif.py"
);

#[derive(Parser, Debug)]
#[command(name = "muse-record-fif")]
#[command(about = "Stream Muse EEG from Bluetooth, then build a FIF via MNE Python (for ZUNA / MNE pipelines).")]
struct Args {
    /// Output FIF path
    #[arg(short = 'o', long = "output")]
    output: PathBuf,

    /// Stop recording after this many seconds
    #[arg(short = 'd', long = "duration", default_value_t = 60)]
    duration_secs: u64,

    /// Python interpreter
    #[arg(long, default_value = "python3")]
    python: String,

    /// `muse_eeg_to_fif.py` path
    #[arg(long, default_value = DEFAULT_SCRIPT)]
    script: PathBuf,

    /// Optional: keep the intermediate CSV
    #[arg(long)]
    keep_csv: bool,

    /// Match BLE device name prefix
    #[arg(long, default_value = "Muse")]
    name_prefix: String,
}

#[tokio::main]
async fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let mut args = Args::parse();

    // Prefer ZUNA's uv venv next to muse_eeg_to_fif.py
    if args.python == "python3" {
        if let Some(dir) = args.script.parent() {
            let venv_py = if cfg!(windows) {
                dir.join(".venv").join("Scripts").join("python.exe")
            } else {
                dir.join(".venv").join("bin").join("python")
            };
            if venv_py.is_file() {
                args.python = venv_py.to_string_lossy().into_owned();
            }
        }
    }

    if !args.script.exists() {
        return Err(anyhow!(
            "MNE script not found: {}\nPass --script /path/to/muse_eeg_to_fif.py",
            args.script.display()
        ));
    }

    let config = MuseClientConfig {
        enable_aux: false,
        enable_ppg: false,
        scan_timeout_secs: 15,
        name_prefix: args.name_prefix.clone(),
    };

    let client = MuseClient::new(config);
    info!("Connecting to Muse headset …");
    let (mut rx, handle) = client.connect().await?;
    let handle = std::sync::Arc::new(handle);

    handle.start(false, false).await?;
    info!(
        "Recording up to {} s (Ctrl-C to stop early). Sampling ~{} Hz.",
        args.duration_secs, EEG_FREQUENCY
    );

    let mut ch: [Vec<f64>; 4] = std::array::from_fn(|_| Vec::new());

    let deadline = tokio::time::Instant::now() + Duration::from_secs(args.duration_secs);
    let mut sleep = Box::pin(tokio::time::sleep_until(deadline));
    let mut ctrl_c = Box::pin(tokio::signal::ctrl_c());

    loop {
        tokio::select! {
            _ = sleep.as_mut() => {
                info!("Duration elapsed.");
                break;
            }
            _ = ctrl_c.as_mut() => {
                info!("Interrupted (Ctrl-C).");
                break;
            }
            evt = rx.recv() => {
                let Some(evt) = evt else { break };
                if let MuseEvent::Eeg(r) = evt {
                    if r.electrode < 4 {
                        ch[r.electrode].extend_from_slice(&r.samples);
                    }
                } else if matches!(evt, MuseEvent::Disconnected) {
                    info!("Device disconnected.");
                    break;
                }
            }
        }
    }

    handle.disconnect().await.ok();

    let n = ch.iter().map(|v| v.len()).min().unwrap_or(0);
    if n < 256 {
        return Err(anyhow!(
            "Too few aligned samples ({}). Need at least ~1 s at 256 Hz for a useful file.",
            n
        ));
    }

    for c in &mut ch {
        c.truncate(n);
    }

    let csv_path = std::env::temp_dir().join(format!("muse_eeg_{}.csv", Uuid::new_v4()));
    {
        let f = File::create(&csv_path).with_context(|| format!("create {}", csv_path.display()))?;
        let mut w = BufWriter::new(f);
        writeln!(w, "TP9,AF7,AF8,TP10")?;
        for i in 0..n {
            writeln!(
                w,
                "{},{},{},{}",
                ch[0][i], ch[1][i], ch[2][i], ch[3][i]
            )?;
        }
        w.flush()?;
    }

    info!("Wrote {} samples × 4 ch → {}", n, csv_path.display());

    let status = Command::new(&args.python)
        .arg(&args.script)
        .arg("--csv")
        .arg(&csv_path)
        .arg("-o")
        .arg(&args.output)
        .arg("--sfreq")
        .arg(&format!("{}", EEG_FREQUENCY))
        .status()
        .with_context(|| format!("failed to spawn {}", args.python))?;

    if !args.keep_csv {
        let _ = std::fs::remove_file(&csv_path);
    }

    if !status.success() {
        return Err(anyhow!("Python exited with {}", status));
    }

    Ok(())
}
