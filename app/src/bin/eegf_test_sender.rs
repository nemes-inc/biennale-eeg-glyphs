//! **eegf-test-sender** — synthetic EEGF frame generator for testing the
//! TCP streaming pipeline without a real Muse headset.
//!
//! Generates 4-channel fake EEG (10 Hz alpha sine + pink noise) and streams
//! EEGF binary frames over TCP to an `eeg-viewer --tcp` instance.
//!
//! Usage:
//! ```text
//! eegf-test-sender --target 127.0.0.1:9000
//! ```

use std::io::Write;
use std::net::TcpStream;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use anyhow::{Context, Result};
use clap::Parser;

/// EEGF magic: ASCII "EEGF" = 0x4545_4746 (little-endian).
const MAGIC_EEGF: u32 = 0x4545_4746;

/// Simulated sample rate (Muse = 256 Hz).
const SAMPLE_RATE: f64 = 256.0;

/// Number of EEG channels (TP9, AF7, AF8, TP10).
const NUM_CHANNELS: usize = 4;

#[derive(Parser, Debug)]
#[command(
    name = "eegf-test-sender",
    about = "Stream synthetic EEGF frames over TCP for pipeline testing"
)]
struct Args {
    /// Target address for the eeg-viewer TCP listener, e.g. 127.0.0.1:9000
    #[arg(long, default_value = "127.0.0.1:9000")]
    target: String,

    /// Samples per channel per EEGF frame.
    #[arg(long, default_value_t = 256)]
    chunk: usize,

    /// Alpha frequency in Hz (dominant oscillation).
    #[arg(long, default_value_t = 10.0)]
    alpha_hz: f64,

    /// Alpha amplitude in µV.
    #[arg(long, default_value_t = 15.0)]
    alpha_amp: f64,

    /// Noise amplitude in µV (Gaussian).
    #[arg(long, default_value_t = 5.0)]
    noise_amp: f64,

    /// Total duration in seconds (0 = run forever until Ctrl-C).
    #[arg(long, default_value_t = 0)]
    duration: u64,

    /// Gradually increase alpha amplitude over the session (simulates rising trend).
    #[arg(long)]
    rising: bool,

    /// Gradually decrease alpha amplitude over the session (simulates falling trend).
    #[arg(long)]
    falling: bool,
}

/// Simple LCG-based pseudo-random for reproducible noise without extra deps.
struct SimpleRng {
    state: u64,
}

impl SimpleRng {
    fn new(seed: u64) -> Self {
        Self { state: seed }
    }

    /// Returns a pseudo-random f64 in [0, 1).
    fn next_f64(&mut self) -> f64 {
        // xorshift64
        self.state ^= self.state << 13;
        self.state ^= self.state >> 7;
        self.state ^= self.state << 17;
        (self.state as f64) / (u64::MAX as f64)
    }

    /// Approximate Gaussian via Box-Muller.
    fn next_gaussian(&mut self) -> f64 {
        let u1 = self.next_f64().max(1e-15);
        let u2 = self.next_f64();
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
    }
}

/// Build a single EEGF binary frame.
fn build_eegf_frame(channels: &[Vec<f32>], n_samples: usize) -> Vec<u8> {
    let n_ch = channels.len() as u32;
    let n_s = n_samples as u32;
    let header_bytes = 12;
    let payload_bytes = (n_ch as usize) * n_samples * 4;
    let mut buf = Vec::with_capacity(header_bytes + payload_bytes);

    buf.extend_from_slice(&MAGIC_EEGF.to_le_bytes());
    buf.extend_from_slice(&n_ch.to_le_bytes());
    buf.extend_from_slice(&n_s.to_le_bytes());

    for ch in channels {
        for &sample in &ch[..n_samples] {
            buf.extend_from_slice(&sample.to_le_bytes());
        }
    }
    buf
}

/// Per-channel phase offsets to make traces visually distinct.
const CHANNEL_PHASE_OFFSETS: [f64; NUM_CHANNELS] = [0.0, 0.3, 0.6, 0.9];

/// Per-channel alpha amplitude multipliers (frontal channels stronger).
const CHANNEL_AMP_SCALE: [f64; NUM_CHANNELS] = [0.6, 1.0, 1.0, 0.6];

fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let args = Args::parse();
    let chunk = args.chunk.max(1);

    // ── Ctrl-C handler ───────────────────────────────────────────────────────
    let running = Arc::new(AtomicBool::new(true));
    {
        let r = Arc::clone(&running);
        ctrlc::set_handler(move || {
            log::info!("Ctrl-C received — shutting down");
            r.store(false, Ordering::SeqCst);
        })
        .ok();
    }

    // ── TCP connect ──────────────────────────────────────────────────────────
    log::info!("Connecting to {} …", args.target);
    let mut stream =
        TcpStream::connect(&args.target).context(format!("TCP connect to {}", args.target))?;
    stream.set_nodelay(true).ok();
    log::info!("Connected. Streaming synthetic EEGF frames (chunk={chunk}).");

    // ── Generate and send ────────────────────────────────────────────────────
    let mut rng = SimpleRng::new(42);
    let mut sample_idx: u64 = 0;
    let mut frames_sent: u64 = 0;
    let frame_duration = Duration::from_secs_f64(chunk as f64 / SAMPLE_RATE);
    let total_samples = if args.duration > 0 {
        (args.duration as f64 * SAMPLE_RATE) as u64
    } else {
        u64::MAX
    };

    while running.load(Ordering::SeqCst) && sample_idx < total_samples {
        let mut channels: Vec<Vec<f32>> = vec![Vec::with_capacity(chunk); NUM_CHANNELS];

        for s in 0..chunk {
            let t = (sample_idx + s as u64) as f64 / SAMPLE_RATE;

            // Trend modulation: scale alpha amp over time
            let trend_scale = if args.rising {
                1.0 + 0.5 * (t / 60.0).min(3.0) // +50% per minute, caps at 3 min
            } else if args.falling {
                (1.0 - 0.3 * (t / 60.0).min(3.0)).max(0.1) // -30% per minute
            } else {
                1.0
            };

            for ch in 0..NUM_CHANNELS {
                let alpha = args.alpha_amp
                    * trend_scale
                    * CHANNEL_AMP_SCALE[ch]
                    * (2.0 * std::f64::consts::PI * args.alpha_hz * t
                        + CHANNEL_PHASE_OFFSETS[ch])
                        .sin();

                // Add a small theta component (5 Hz)
                let theta =
                    3.0 * CHANNEL_AMP_SCALE[ch] * (2.0 * std::f64::consts::PI * 5.0 * t).sin();

                let noise = args.noise_amp * rng.next_gaussian();

                channels[ch].push((alpha + theta + noise) as f32);
            }
        }

        let frame = build_eegf_frame(&channels, chunk);
        if let Err(e) = stream.write_all(&frame) {
            log::error!("TCP write failed: {e}");
            return Err(e.into());
        }

        frames_sent += 1;
        sample_idx += chunk as u64;

        if frames_sent % 10 == 0 {
            let elapsed = sample_idx as f64 / SAMPLE_RATE;
            log::info!(
                "Sent {frames_sent} frames ({elapsed:.1}s simulated, {chunk} samples/ch each)"
            );
        }

        // Pace to real-time
        std::thread::sleep(frame_duration);
    }

    log::info!(
        "Done — sent {frames_sent} frames ({:.1}s simulated).",
        sample_idx as f64 / SAMPLE_RATE
    );
    Ok(())
}
