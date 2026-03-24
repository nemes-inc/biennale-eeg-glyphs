//! **muse-stream-tcp** — connect to a Muse headset via BLE and stream EEGF
//! binary frames over TCP to a remote `eeg-viewer --tcp` instance.
//!
//! Usage (MacBook side):
//!
//! ```text
//! muse-stream-tcp --target 192.168.1.50:9000
//! ```
//!
//! The binary scans for a Muse, connects, and pushes EEGF frames as fast as
//! they arrive.  The remote `eeg-viewer` should already be listening:
//!
//! ```text
//! eeg-viewer --tcp 0.0.0.0:9000
//! ```

use std::io::Write;
use std::net::TcpStream;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use anyhow::{Context, Result};
use clap::Parser;
use log::{error, info, warn};

use muse_rs::muse_client::{MuseClient, MuseClientConfig};
use muse_rs::types::MuseEvent;

/// EEGF magic: ASCII "EEGF" = 0x4545_4746 (little-endian).
const MAGIC_EEGF: u32 = 0x4545_4746;

/// How many samples per channel to accumulate before sending one EEGF frame.
const CHUNK_SIZE: usize = 256;

/// Muse EEG has 4 electrodes (TP9, AF7, AF8, TP10).
const NUM_CHANNELS: usize = 4;

#[derive(Parser, Debug)]
#[command(name = "muse-stream-tcp", about = "Stream Muse EEG over TCP as EEGF frames")]
struct Args {
    /// Target address for the eeg-viewer TCP listener, e.g. 192.168.1.50:9000
    #[arg(long)]
    target: String,

    /// BLE scan timeout in seconds.
    #[arg(long, default_value_t = 30)]
    scan_timeout: u64,

    /// Muse device name prefix for BLE scan filtering.
    #[arg(long, default_value = "Muse")]
    name_prefix: String,

    /// Samples per channel per EEGF frame (default 256 ≈ 1 s at 256 Hz).
    #[arg(long, default_value_t = CHUNK_SIZE)]
    chunk: usize,

    /// Retry connecting to the TCP target on failure (with back-off).
    #[arg(long)]
    tcp_retry: bool,
}

/// Build a single EEGF binary frame (little-endian):
///   u32 magic | u32 n_channels | u32 n_samples | f32[] channel-major payload
fn build_eegf_frame(channels: &[Vec<f32>], n_samples: usize) -> Vec<u8> {
    let n_ch = channels.len() as u32;
    let n_s = n_samples as u32;
    let header_bytes = 12; // 3 × u32
    let payload_bytes = (n_ch as usize) * n_samples * 4;
    let mut buf = Vec::with_capacity(header_bytes + payload_bytes);

    buf.extend_from_slice(&MAGIC_EEGF.to_le_bytes());
    buf.extend_from_slice(&n_ch.to_le_bytes());
    buf.extend_from_slice(&n_s.to_le_bytes());

    // Channel-major: all samples for ch0, then ch1, …
    for ch in channels {
        for &sample in &ch[..n_samples] {
            buf.extend_from_slice(&sample.to_le_bytes());
        }
    }
    buf
}

fn connect_tcp(target: &str, retry: bool) -> Result<TcpStream> {
    loop {
        match TcpStream::connect(target) {
            Ok(s) => {
                s.set_nodelay(true).ok();
                info!("TCP connected to {target}");
                return Ok(s);
            }
            Err(e) => {
                if retry {
                    warn!("TCP connect to {target} failed ({e}), retrying in 2 s …");
                    std::thread::sleep(Duration::from_secs(2));
                } else {
                    return Err(e).context(format!("TCP connect to {target}"));
                }
            }
        }
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let args = Args::parse();
    let chunk = args.chunk.max(1);

    // ── Ctrl-C handler ───────────────────────────────────────────────────────
    let running = Arc::new(AtomicBool::new(true));
    {
        let r = Arc::clone(&running);
        ctrlc::set_handler(move || {
            info!("Ctrl-C received — shutting down");
            r.store(false, Ordering::SeqCst);
        })
        .ok();
    }

    // ── TCP connect ──────────────────────────────────────────────────────────
    let mut stream = connect_tcp(&args.target, args.tcp_retry)?;

    // ── BLE connect ──────────────────────────────────────────────────────────
    let config = MuseClientConfig {
        enable_aux: false,
        enable_ppg: false,
        scan_timeout_secs: args.scan_timeout,
        name_prefix: args.name_prefix.clone(),
    };

    info!("Scanning for Muse headset (prefix '{}') …", args.name_prefix);
    let client = MuseClient::new(config);
    let (mut rx, handle) = client.connect().await?;
    handle.start(false, false).await?;
    info!("Muse connected and streaming EEG. Sending EEGF frames to {} (chunk={chunk})", args.target);

    // ── Main loop: BLE → EEGF → TCP ─────────────────────────────────────────
    let mut buffers: Vec<Vec<f32>> = vec![Vec::new(); NUM_CHANNELS];
    let mut frames_sent: u64 = 0;

    while running.load(Ordering::SeqCst) {
        tokio::select! {
            evt = rx.recv() => {
                match evt {
                    Some(MuseEvent::Eeg(r)) if r.electrode < NUM_CHANNELS => {
                        buffers[r.electrode].extend(r.samples.iter().map(|&s| s as f32));

                        // Send a frame once every channel has >= chunk samples.
                        while buffers.iter().map(|v| v.len()).min().unwrap_or(0) >= chunk {
                            let frame_data: Vec<Vec<f32>> = buffers
                                .iter_mut()
                                .map(|b| b.drain(0..chunk).collect())
                                .collect();

                            let frame = build_eegf_frame(&frame_data, chunk);
                            if let Err(e) = stream.write_all(&frame) {
                                error!("TCP write failed: {e}");
                                let _ = handle.disconnect().await;
                                return Err(e.into());
                            }
                            frames_sent += 1;
                            if frames_sent % 10 == 0 {
                                info!("Sent {frames_sent} EEGF frames ({} samples/ch each)", chunk);
                            }
                        }
                    }
                    Some(MuseEvent::Connected(name)) => {
                        info!("Muse BLE connected: {name}");
                    }
                    Some(MuseEvent::Disconnected) => {
                        warn!("Muse BLE disconnected");
                        break;
                    }
                    None => break,
                    _ => {}
                }
            }
            _ = tokio::time::sleep(Duration::from_millis(100)) => {
                if !running.load(Ordering::SeqCst) {
                    break;
                }
            }
        }
    }

    info!("Shutting down — sent {frames_sent} frames total.");
    let _ = handle.disconnect().await;
    Ok(())
}
