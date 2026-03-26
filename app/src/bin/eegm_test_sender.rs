//! **eegm-test-sender** — synthetic EEGM frame generator for testing the
//! multi-headband inference pipeline without real Muse headsets.
//!
//! Connects to the inference server and sends synthetic 4-channel EEG
//! (10 Hz alpha + noise) as EEGM frames for 1–4 simulated headbands.
//!
//! Usage:
//! ```text
//! eegm-test-sender --target 127.0.0.1:9100
//! eegm-test-sender --target 127.0.0.1:9100 --headbands 4 --chunk 64
//! ```

use std::io::{Read, Write};
use std::net::TcpStream;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use anyhow::{Context, Result};
use clap::Parser;

use muse_rs::eegm::{EegmFrame, HEADER_SIZE, MAGIC_EEGM};

/// Simulated sample rate (Muse = 256 Hz).
const SAMPLE_RATE: f64 = 256.0;

/// Number of EEG channels per headband.
const NUM_CHANNELS: usize = 4;

#[derive(Parser, Debug)]
#[command(
    name = "eegm-test-sender",
    about = "Stream synthetic EEGM frames for multi-headband inference testing"
)]
struct Args {
    /// Target address for the inference server, e.g. 127.0.0.1:9100
    #[arg(long, default_value = "127.0.0.1:9100")]
    target: String,

    /// Number of simulated headbands (1–4).
    #[arg(long, default_value_t = 1)]
    headbands: usize,

    /// Samples per channel per EEGM frame.
    #[arg(long, default_value_t = 256)]
    chunk: usize,

    /// Alpha frequency in Hz.
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

    /// Also read and log response frames from the server.
    #[arg(long)]
    read_responses: bool,
}

/// Simple xorshift64 PRNG.
struct SimpleRng { state: u64 }

impl SimpleRng {
    fn new(seed: u64) -> Self { Self { state: seed } }

    fn next_f64(&mut self) -> f64 {
        self.state ^= self.state << 13;
        self.state ^= self.state >> 7;
        self.state ^= self.state << 17;
        (self.state as f64) / (u64::MAX as f64)
    }

    fn next_gaussian(&mut self) -> f64 {
        let u1 = self.next_f64().max(1e-15);
        let u2 = self.next_f64();
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
    }
}

/// Per-channel phase offsets for visual distinction.
const PHASE_OFFSETS: [f64; NUM_CHANNELS] = [0.0, 0.3, 0.6, 0.9];
const AMP_SCALE: [f64; NUM_CHANNELS] = [0.6, 1.0, 1.0, 0.6];

fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let args = Args::parse();
    let chunk = args.chunk.max(1);
    let n_headbands = args.headbands.min(4).max(1);

    let running = Arc::new(AtomicBool::new(true));
    {
        let r = Arc::clone(&running);
        ctrlc::set_handler(move || {
            log::info!("Ctrl-C — shutting down");
            r.store(false, Ordering::SeqCst);
        }).ok();
    }

    log::info!("Connecting to {} …", args.target);
    let mut stream = TcpStream::connect(&args.target)
        .context(format!("TCP connect to {}", args.target))?;
    stream.set_nodelay(true).ok();
    log::info!(
        "Connected. Streaming {} headband(s), chunk={chunk}.",
        n_headbands
    );

    // Optional: spawn a thread to read responses
    if args.read_responses {
        let mut reader = stream.try_clone()?;
        let r = Arc::clone(&running);
        std::thread::spawn(move || {
            let mut hdr = [0u8; HEADER_SIZE];
            loop {
                if !r.load(Ordering::SeqCst) { break; }
                match reader.read_exact(&mut hdr) {
                    Ok(()) => {
                        let magic = u32::from_le_bytes([hdr[0], hdr[1], hdr[2], hdr[3]]);
                        if magic != MAGIC_EEGM {
                            log::warn!("Bad response magic: 0x{magic:08X}");
                            break;
                        }
                        let hid = u32::from_le_bytes([hdr[4], hdr[5], hdr[6], hdr[7]]);
                        let seq = u32::from_le_bytes([hdr[8], hdr[9], hdr[10], hdr[11]]);
                        let nch = u32::from_le_bytes([hdr[12], hdr[13], hdr[14], hdr[15]]);
                        let ns = u32::from_le_bytes([hdr[16], hdr[17], hdr[18], hdr[19]]);
                        let payload_size = (nch as usize) * (ns as usize) * 4;
                        let mut payload = vec![0u8; payload_size];
                        if reader.read_exact(&mut payload).is_err() { break; }
                        log::info!(
                            "← Response: headband={hid} seq={seq} {nch}ch×{ns}samples"
                        );
                    }
                    Err(_) => break,
                }
            }
        });
    }

    let mut rngs: Vec<SimpleRng> = (0..n_headbands)
        .map(|h| SimpleRng::new(42 + h as u64 * 1000))
        .collect();
    let mut sample_idx: Vec<u64> = vec![0; n_headbands];
    let mut epoch_seq: Vec<u32> = vec![0; n_headbands];
    let mut total_frames: u64 = 0;
    let frame_duration = Duration::from_secs_f64(chunk as f64 / SAMPLE_RATE);
    let total_samples = if args.duration > 0 {
        (args.duration as f64 * SAMPLE_RATE) as u64
    } else {
        u64::MAX
    };

    while running.load(Ordering::SeqCst) && sample_idx[0] < total_samples {
        for h in 0..n_headbands {
            let mut channels: Vec<Vec<f32>> = vec![Vec::with_capacity(chunk); NUM_CHANNELS];

            for s in 0..chunk {
                let t = (sample_idx[h] + s as u64) as f64 / SAMPLE_RATE;
                // Per-headband frequency offset for distinction
                let freq_offset = h as f64 * 0.5;

                for ch in 0..NUM_CHANNELS {
                    let alpha = args.alpha_amp
                        * AMP_SCALE[ch]
                        * (2.0 * std::f64::consts::PI * (args.alpha_hz + freq_offset) * t
                            + PHASE_OFFSETS[ch])
                            .sin();
                    let noise = args.noise_amp * rngs[h].next_gaussian();
                    channels[ch].push((alpha + noise) as f32);
                }
            }

            let frame = EegmFrame::new(h as u32, epoch_seq[h], &channels, chunk);
            let encoded = frame.encode();
            if let Err(e) = stream.write_all(&encoded) {
                log::error!("TCP write failed: {e}");
                return Err(e.into());
            }

            sample_idx[h] += chunk as u64;
            epoch_seq[h] += 1;
            total_frames += 1;
        }

        if total_frames % (10 * n_headbands as u64) == 0 {
            let elapsed = sample_idx[0] as f64 / SAMPLE_RATE;
            log::info!(
                "Sent {total_frames} frames total ({elapsed:.1}s simulated, {n_headbands} headband(s))"
            );
        }

        std::thread::sleep(frame_duration);
    }

    log::info!("Done — sent {total_frames} frames total.");
    Ok(())
}
