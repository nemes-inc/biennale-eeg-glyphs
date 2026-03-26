//! **eegm-test-sender** — synthetic multi-headband EEGM frame generator.
//!
//! Connects to the inference server and sends synthetic 4-channel EEG for
//! 1–4 simulated headbands, each with a distinct signal profile:
//!
//! - **Band 0**: strong 10 Hz alpha — relaxed subject, eyes closed
//! - **Band 1**: 6 Hz theta dominant — drowsy/meditative subject
//! - **Band 2**: 20 Hz beta dominant — alert/focused subject
//! - **Band 3**: mixed alpha + theta — transitional state
//!
//! Each headband also has per-channel amplitude scaling (temporal channels
//! weaker than frontal, matching real Muse topology) and periodic eye-blink
//! artifacts on the frontal channels (AF7, AF8).
//!
//! Usage:
//! ```text
//! eegm-test-sender --target 127.0.0.1:9100 --headbands 4
//! eegm-test-sender --target 127.0.0.1:9100 --headbands 2 --chunk 64
//! eegm-test-sender --target 127.0.0.1:9100 --headbands 1 --duration 30
//! ```

use std::collections::HashMap;
use std::io::{Read, Write};
use std::net::TcpStream;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use clap::Parser;

use muse_rs::eegm::{ConnectReq, EegmFrame, CTRL_SIZE, HEADER_SIZE, MAGIC_EEGC, MAGIC_EEGM};

/// Simulated sample rate (Muse = 256 Hz).
const SAMPLE_RATE: f64 = 256.0;

/// Number of EEG channels per headband (TP9, AF7, AF8, TP10).
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
    #[arg(long, default_value_t = 4)]
    headbands: usize,

    /// Samples per channel per EEGM frame.
    #[arg(long, default_value_t = 256)]
    chunk: usize,

    /// Global amplitude scale factor (1.0 = typical Muse µV levels).
    #[arg(long, default_value_t = 1.0)]
    amplitude: f64,

    /// Total duration in seconds (0 = run forever until Ctrl-C).
    #[arg(long, default_value_t = 0)]
    duration: u64,

    /// Disable reading/logging response frames from the server.
    #[arg(long)]
    no_responses: bool,
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

/// Per-channel amplitude scaling: temporal (TP9/TP10) weaker than frontal (AF7/AF8).
const CH_AMP_SCALE: [f64; NUM_CHANNELS] = [0.6, 1.0, 1.0, 0.6];

/// Signal profile for one simulated headband.
struct HeadbandProfile {
    /// Human-readable label.
    label: &'static str,
    /// Oscillatory components: (frequency_hz, amplitude_uv, phase_offset).
    oscillations: Vec<(f64, f64, f64)>,
    /// Background noise amplitude in µV.
    noise_amp: f64,
    /// Eye-blink interval in seconds (0 = no blinks).
    blink_interval: f64,
    /// Eye-blink amplitude in µV (added to AF7/AF8 only).
    blink_amp: f64,
}

fn headband_profiles() -> Vec<HeadbandProfile> {
    vec![
        HeadbandProfile {
            label: "alpha-dominant (relaxed, eyes closed)",
            oscillations: vec![
                (10.0, 18.0, 0.0),   // strong alpha
                (20.0, 3.0, 0.5),    // weak beta harmonic
            ],
            noise_amp: 4.0,
            blink_interval: 4.0,
            blink_amp: 80.0,
        },
        HeadbandProfile {
            label: "theta-dominant (drowsy/meditative)",
            oscillations: vec![
                (6.0, 14.0, 0.0),    // dominant theta
                (10.0, 5.0, 0.3),    // mild alpha
            ],
            noise_amp: 5.0,
            blink_interval: 6.0,
            blink_amp: 60.0,
        },
        HeadbandProfile {
            label: "beta-dominant (alert/focused)",
            oscillations: vec![
                (20.0, 10.0, 0.0),   // dominant beta
                (10.0, 4.0, 0.7),    // suppressed alpha
                (40.0, 2.0, 0.2),    // weak gamma
            ],
            noise_amp: 6.0,
            blink_interval: 3.0,
            blink_amp: 90.0,
        },
        HeadbandProfile {
            label: "mixed alpha+theta (transitional)",
            oscillations: vec![
                (10.0, 12.0, 0.0),   // moderate alpha
                (6.0, 10.0, 0.4),    // moderate theta
                (13.0, 3.0, 0.9),    // SMR component
            ],
            noise_amp: 5.0,
            blink_interval: 5.0,
            blink_amp: 70.0,
        },
    ]
}

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

    // ── Handshake ────────────────────────────────────────────────────────
    let req = ConnectReq::new(n_headbands as u32, SAMPLE_RATE as u32);
    stream.write_all(&req.encode()).context("send ConnectReq")?;
    stream.flush().ok();
    log::info!("ConnectReq sent ({n_headbands} headband(s), {} Hz). Waiting for ack…", SAMPLE_RATE as u32);

    let mut ack_buf = [0u8; CTRL_SIZE];
    stream.read_exact(&mut ack_buf).context("read ConnectAck")?;
    let ack_magic = u32::from_le_bytes([ack_buf[0], ack_buf[1], ack_buf[2], ack_buf[3]]);
    if ack_magic != MAGIC_EEGC {
        anyhow::bail!("expected EEGC ConnectAck, got 0x{ack_magic:08X}");
    }
    let ack_status = u32::from_le_bytes([ack_buf[20], ack_buf[21], ack_buf[22], ack_buf[23]]);
    if ack_status != 0 {
        anyhow::bail!("server rejected connection: status={ack_status}");
    }
    let ack_hb = u32::from_le_bytes([ack_buf[12], ack_buf[13], ack_buf[14], ack_buf[15]]);
    let ack_sr = u32::from_le_bytes([ack_buf[16], ack_buf[17], ack_buf[18], ack_buf[19]]);
    log::info!("ConnectAck OK — server accepted {ack_hb} headband(s) @ {ack_sr} Hz");

    // ── Log headband profiles ──────────────────────────────────────────
    let profiles = headband_profiles();
    for h in 0..n_headbands {
        let p = &profiles[h % profiles.len()];
        let osc_desc: Vec<String> = p.oscillations.iter()
            .map(|(f, a, _)| format!("{f:.0} Hz @ {a:.0} µV"))
            .collect();
        log::info!(
            "Headband {h}: {} — [{}], noise={:.0} µV, blinks every {:.0}s",
            p.label,
            osc_desc.join(", "),
            p.noise_amp,
            p.blink_interval,
        );
    }
    log::info!(
        "Streaming {} headband(s), chunk={chunk}, amp_scale={:.1}",
        n_headbands,
        args.amplitude,
    );

    // Shared map: (headband_id, epoch_seq) → Instant when frame was sent.
    // The response reader looks up send times to compute round-trip latency.
    let send_times: Arc<Mutex<HashMap<(u32, u32), Instant>>> =
        Arc::new(Mutex::new(HashMap::new()));

    // Spawn response reader thread (unless --no-responses)
    if !args.no_responses {
        let mut reader = stream.try_clone()?;
        let r = Arc::clone(&running);
        let st = Arc::clone(&send_times);
        std::thread::spawn(move || {
            let ch_names = ["TP9", "AF7", "AF8", "TP10"];
            let mut resp_count: u64 = 0;
            let mut magic_buf = [0u8; 4];
            loop {
                if !r.load(Ordering::SeqCst) { break; }
                match reader.read_exact(&mut magic_buf) {
                    Ok(()) => {
                        let magic = u32::from_le_bytes(magic_buf);
                        if magic == MAGIC_EEGC {
                            let mut skip = [0u8; CTRL_SIZE - 4];
                            if reader.read_exact(&mut skip).is_err() { break; }
                            log::debug!("← Skipping EEGC control message");
                            continue;
                        }
                        if magic != MAGIC_EEGM {
                            log::warn!("Bad response magic: 0x{magic:08X}");
                            break;
                        }
                        let mut hdr_rest = [0u8; HEADER_SIZE - 4];
                        if reader.read_exact(&mut hdr_rest).is_err() { break; }
                        let hid = u32::from_le_bytes([hdr_rest[0], hdr_rest[1], hdr_rest[2], hdr_rest[3]]);
                        let seq = u32::from_le_bytes([hdr_rest[4], hdr_rest[5], hdr_rest[6], hdr_rest[7]]);
                        let nch = u32::from_le_bytes([hdr_rest[8], hdr_rest[9], hdr_rest[10], hdr_rest[11]]) as usize;
                        let ns = u32::from_le_bytes([hdr_rest[12], hdr_rest[13], hdr_rest[14], hdr_rest[15]]) as usize;
                        let payload_size = nch * ns * 4;
                        let mut payload = vec![0u8; payload_size];
                        if reader.read_exact(&mut payload).is_err() { break; }

                        // Compute round-trip latency
                        let latency_str = {
                            let mut map = st.lock().unwrap();
                            if let Some(sent_at) = map.remove(&(hid, seq)) {
                                format!("{:.1}s", sent_at.elapsed().as_secs_f64())
                            } else {
                                "?".to_string()
                            }
                        };

                        // Compute per-channel RMS (µV)
                        let mut rms_parts: Vec<String> = Vec::with_capacity(nch);
                        for ch in 0..nch {
                            let offset = ch * ns * 4;
                            let mut sum_sq = 0.0_f64;
                            for s in 0..ns {
                                let i = offset + s * 4;
                                let v = f32::from_le_bytes([
                                    payload[i], payload[i+1], payload[i+2], payload[i+3],
                                ]) as f64;
                                sum_sq += v * v;
                            }
                            let rms = (sum_sq / ns as f64).sqrt();
                            let name = if ch < ch_names.len() { ch_names[ch] } else { "?" };
                            rms_parts.push(format!("{name}={rms:.1}"));
                        }

                        resp_count += 1;
                        log::info!(
                            "← #{resp_count} band={hid} seq={seq} {nch}ch×{ns}smp \
                            latency={latency_str} RMS(µV): {}",
                            rms_parts.join(" "),
                        );
                    }
                    Err(_) => break,
                }
            }
        });
    }

    // ── Signal generation ────────────────────────────────────────────────
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
    let two_pi = 2.0 * std::f64::consts::PI;

    while running.load(Ordering::SeqCst) && sample_idx[0] < total_samples {
        for h in 0..n_headbands {
            let profile = &profiles[h % profiles.len()];
            let mut channels: Vec<Vec<f32>> = vec![Vec::with_capacity(chunk); NUM_CHANNELS];

            for s in 0..chunk {
                let t = (sample_idx[h] + s as u64) as f64 / SAMPLE_RATE;

                for ch in 0..NUM_CHANNELS {
                    // Sum oscillatory components
                    let mut val = 0.0_f64;
                    for &(freq, amp, phase) in &profile.oscillations {
                        val += amp * CH_AMP_SCALE[ch]
                            * (two_pi * freq * t + phase).sin();
                    }

                    // Gaussian noise
                    val += profile.noise_amp * rngs[h].next_gaussian();

                    // Eye-blink artifacts on frontal channels (AF7=ch1, AF8=ch2)
                    if profile.blink_interval > 0.0 && (ch == 1 || ch == 2) {
                        let blink_phase = t % profile.blink_interval;
                        // Blink is a ~150 ms Gaussian pulse
                        if blink_phase < 0.15 {
                            let blink_t = blink_phase / 0.075 - 1.0; // centered at 75ms
                            val += profile.blink_amp * (-0.5 * blink_t * blink_t).exp();
                        }
                    }

                    // Global amplitude scaling
                    val *= args.amplitude;

                    channels[ch].push(val as f32);
                }
            }

            let frame = EegmFrame::new(h as u32, epoch_seq[h], &channels, chunk);
            let encoded = frame.encode();

            // Record send time for latency tracking
            if let Ok(mut map) = send_times.lock() {
                map.insert((h as u32, epoch_seq[h]), Instant::now());
                // Prune old entries (keep last 256 per headband)
                if map.len() > 1024 {
                    let cutoff = Instant::now() - Duration::from_secs(120);
                    map.retain(|_, t| *t > cutoff);
                }
            }

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
                "Sent {total_frames} frames ({elapsed:.1}s simulated × {n_headbands} headband(s))"
            );
        }

        std::thread::sleep(frame_duration);
    }

    log::info!("Done — sent {total_frames} frames total.");
    Ok(())
}
