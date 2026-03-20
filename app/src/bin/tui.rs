//! Real-time EEG chart viewer for Muse headsets.
//!
//! Usage:
//!   cargo run --bin tui               # scan forever until a Muse is found, then stream
//!   cargo run --bin tui -- --simulate # use the built-in EEG simulator (no hardware needed)
//!
//! Keys (streaming view)
//! ---------------------
//!   Tab      open device picker
//!   1        switch to EEG view
//!   2        switch to PPG view
//!   3        switch to Info view (contact quality, battery, device info, control log)
//!   4        switch to IMU view (accelerometer + gyroscope charts)
//!   5        switch to ACT view (adaptive chirplet transform results, requires --features act)
//!   s        trigger a fresh BLE scan right now
//!   +  / =   zoom out  (increase µV scale, EEG only)
//!   -        zoom in   (decrease µV scale, EEG only)
//!   a        auto-scale: fit Y axis to current peak amplitude (EEG only)
//!   v        toggle smooth overlay (dim raw + bright 9-pt moving-average)
//!   p        pause streaming   (sends 'h' to real device)
//!   r        resume streaming  (sends 'd' to real device)
//!   c        clear waveform buffers
//!   d        disconnect current device
//!   q / Esc  quit
//!
//! Keys (device picker overlay)
//! ----------------------------
//!   ↑ / ↓   navigate list
//!   Enter    connect to highlighted device
//!   s        rescan while picker is open
//!   Esc      close picker

use std::collections::VecDeque;
use std::f64::consts::PI;
use std::io;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use anyhow::Result;
use crossterm::{
    event::{self, Event, KeyCode, KeyModifiers},
    execute,
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
};
use ratatui::{
    backend::CrosstermBackend,
    layout::{Constraint, Layout, Margin, Rect},
    style::{Color, Modifier, Style},
    symbols,
    text::{Line, Span},
    widgets::{
        Axis, Block, Borders, Chart, Clear, Dataset, GraphType, List, ListItem, ListState,
        Paragraph,
    },
    Frame, Terminal,
};
use tokio::sync::{mpsc, oneshot};

use muse_rs::compute::{
    touching_forehead, ContactQuality, ContactQualityTracker, CONTACT_QUALITY_WINDOW_SAMPLES,
};
use muse_rs::muse_client::{MuseClient, MuseClientConfig, MuseDevice, MuseHandle};
use muse_rs::protocol::EEG_CHANNEL_NAMES;
use muse_rs::types::MuseEvent;

// ── Constants ─────────────────────────────────────────────────────────────────

/// Width of the scrolling waveform window in seconds.
/// Increasing this shows more history but reduces per-sample horizontal resolution.
const WINDOW_SECS: f64 = 2.0;

/// EEG sample rate expected from the headset (Hz).
/// Used to convert sample indices to x-axis time values.
const EEG_HZ: f64 = 256.0;

/// Number of samples retained per channel for the EEG display window.
const BUF_SIZE: usize = (WINDOW_SECS * EEG_HZ) as usize; // 512

/// Actual ring-buffer capacity per channel — large enough for both the display
/// window and the entrainment ACT window (which may be longer).
const EEG_BUF_CAP: usize = 1024;

/// Number of EEG channels displayed (TP9, AF7, AF8, TP10).
/// The optional AUX channel (index 4) is not shown in the TUI.
const NUM_CH: usize = 4;

/// PPG sample rate (Hz). Athena optical data arrives at 64 Hz.
const PPG_HZ: f64 = 64.0;

/// Number of PPG samples retained per channel — enough for `WINDOW_SECS`.
const PPG_BUF_SIZE: usize = (WINDOW_SECS * PPG_HZ) as usize; // 128

/// Number of PPG optical channels: ambient, infrared, red.
const PPG_NUM_CH: usize = 3;

/// PPG channel display names.
const PPG_CHANNEL_NAMES: [&str; 3] = ["Ambient", "Infrared", "Red"];

/// PPG channel colours.
const PPG_COLORS: [Color; 3] = [Color::LightBlue, Color::LightRed, Color::Red];

/// Discrete Y-axis scale steps in µV (half the full symmetric range ±scale).
/// The user cycles through these with `+` / `-`; `a` picks the best fit automatically.
const Y_SCALES: &[f64] = &[10.0, 25.0, 50.0, 100.0, 200.0, 500.0, 1000.0, 2000.0];

/// Index into `Y_SCALES` used when the app starts with a real device.
/// ±500 µV covers typical real-world EEG amplitudes with headroom for artefacts.
const DEFAULT_SCALE: usize = 5;

/// Per-channel line colours: TP9=Cyan, AF7=Yellow, AF8=Green, TP10=Magenta.
const COLORS: [Color; 4] = [Color::Cyan, Color::Yellow, Color::Green, Color::Magenta];

/// Dimmed versions of COLORS used for the raw background trace when smooth is on.
const DIM_COLORS: [Color; 4] = [
    Color::Rgb(0, 90, 110),   // dim cyan
    Color::Rgb(110, 90, 0),   // dim yellow
    Color::Rgb(0, 110, 0),    // dim green
    Color::Rgb(110, 0, 110),  // dim magenta
];

/// Moving-average window in samples. 9 samples ≈ 35 ms at 256 Hz.
/// Passes alpha (8-13 Hz) and beta (13-30 Hz) clearly while removing HF noise.
const SMOOTH_WINDOW: usize = 9;

/// IMU sample rate (Hz). Both accelerometer and gyroscope produce ~52 Hz.
const IMU_HZ: f64 = 52.0;

/// Number of IMU samples retained per axis — enough for `WINDOW_SECS`.
const IMU_BUF_SIZE: usize = (WINDOW_SECS * IMU_HZ) as usize; // 104

/// Maximum number of control log entries retained for the Info view.
const CONTROL_LOG_MAX: usize = 50;

/// Braille spinner frames cycled at ~100 ms intervals to indicate background activity.
const SPINNER: &[&str] = &["⠋", "⠙", "⠹", "⠸", "⠼", "⠴", "⠦", "⠧", "⠇", "⠏"];

/// Seconds between automatic scan retries when no Muse device is found at all.
/// Long enough to avoid flooding the BLE adapter but short enough to feel responsive.
const RETRY_SECS: u64 = 6;

/// Seconds to wait before re-scanning after an unexpected disconnect.
/// Shorter than `RETRY_SECS` — the device is almost certainly nearby and will
/// resume advertising within a second or two of losing the link.
const RECONNECT_DELAY_SECS: u64 = 2;

// ── App mode ──────────────────────────────────────────────────────────────────

#[derive(Clone)]
pub enum AppMode {
    /// BLE scan is running (initial or retry).
    Scanning,
    /// Actively establishing a connection.
    Connecting(String),
    /// Streaming live EEG. Carries the device name and its short identifier.
    Connected { name: String, id: String },
    /// `--simulate` flag: running the built-in signal generator.
    Simulated,
    /// Last scan found no Muse devices. Will retry automatically.
    NoDevices,
    /// Was connected; link was lost. Will retry automatically.
    Disconnected,
}

// ── App state (shared with the BLE event task via Arc<Mutex<_>>) ──────────────

/// Which signal type is shown in the main chart area.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum ViewMode {
    Eeg,
    Ppg,
    Info,
    Imu,
    #[cfg(feature = "act")]
    Act,
    #[cfg(feature = "act")]
    Baseline,
    #[cfg(feature = "act")]
    Absorption,
    #[cfg(feature = "act")]
    Entrainment,
    #[cfg(feature = "act")]
    Bands,
    #[cfg(feature = "act")]
    Approach,
}

// ── CSV recording ─────────────────────────────────────────────────────────────

/// Streams raw EEG samples and FFT band powers to CSV files for offline analysis.
pub(crate) struct CsvRecorder {
    /// Raw EEG: timestamp_s, tp9, af7, af8, tp10
    raw_file: std::io::BufWriter<std::fs::File>,
    /// FFT bands: timestamp_s, channel, delta, theta, alpha, beta, gamma, total
    fft_file: std::io::BufWriter<std::fs::File>,
    /// Monotonic start time for relative timestamps.
    t0: Instant,
    /// Accumulate samples per channel until all 4 arrive for the same packet group.
    raw_buf: [Vec<f64>; NUM_CH],
    /// Sample counter for raw EEG (used to compute timestamp).
    sample_idx: u64,
    /// Display path prefix.
    pub prefix: String,
}

impl CsvRecorder {
    fn open() -> std::io::Result<Self> {
        use std::fs::File;
        use std::io::Write;

        let ts = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_secs();
        let prefix = format!("muse_{ts}");
        let raw_path = format!("{prefix}_raw.csv");
        let fft_path = format!("{prefix}_fft.csv");

        let mut raw_file = std::io::BufWriter::new(File::create(&raw_path)?);
        let mut fft_file = std::io::BufWriter::new(File::create(&fft_path)?);

        writeln!(raw_file, "timestamp_s,tp9,af7,af8,tp10")?;
        writeln!(fft_file, "timestamp_s,channel,delta,theta,alpha,beta,gamma,total,rel_alpha")?;

        log::info!("CSV recording started: {raw_path}, {fft_path}");

        Ok(Self {
            raw_file,
            fft_file,
            t0: Instant::now(),
            raw_buf: std::array::from_fn(|_| Vec::with_capacity(12)),
            sample_idx: 0,
            prefix,
        })
    }

    /// Buffer incoming samples for one channel. When all 4 channels have data,
    /// flush aligned rows.
    fn push_raw(&mut self, ch: usize, samples: &[f64]) {
        use std::io::Write;

        if ch >= NUM_CH {
            return;
        }
        self.raw_buf[ch].extend_from_slice(samples);

        // Find the minimum length across all 4 channels
        let min_len = self.raw_buf.iter().map(|b| b.len()).min().unwrap_or(0);
        if min_len == 0 {
            return;
        }

        // Write aligned rows
        for i in 0..min_len {
            let t = self.sample_idx as f64 / EEG_HZ;
            let _ = writeln!(
                self.raw_file,
                "{t:.6},{:.4},{:.4},{:.4},{:.4}",
                self.raw_buf[0][i],
                self.raw_buf[1][i],
                self.raw_buf[2][i],
                self.raw_buf[3][i],
            );
            self.sample_idx += 1;
        }

        // Remove flushed samples
        for buf in &mut self.raw_buf {
            buf.drain(..min_len);
        }
    }

    /// Write FFT band powers for all channels.
    #[cfg(feature = "act")]
    fn push_fft(&mut self, snap: &muse_rs::alpha::FftSnapshot) {
        use std::io::Write;

        let t = self.t0.elapsed().as_secs_f64();
        let names = ["TP9", "AF7", "AF8", "TP10"];
        for (ch, bp) in snap.channels.iter().enumerate() {
            let _ = writeln!(
                self.fft_file,
                "{t:.6},{},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6}",
                names[ch],
                bp.delta_power,
                bp.theta_power,
                bp.alpha_power,
                bp.beta_power,
                bp.gamma_power,
                bp.total_power,
                bp.relative_alpha,
            );
        }
    }

    fn flush(&mut self) {
        use std::io::Write;
        let _ = self.raw_file.flush();
        let _ = self.fft_file.flush();
    }
}

pub struct App {
    // ── EEG data (raw + filtered)
    bufs: [VecDeque<f64>; NUM_CH],
    filtered_bufs: [VecDeque<f64>; NUM_CH],
    eeg_filters: [muse_rs::filters::ChannelFilter; NUM_CH],
    epoch_validators: [muse_rs::filters::EpochValidator; NUM_CH],

    // ── PPG data (raw ADC u32 stored as f64 for chart compatibility)
    ppg_bufs: [VecDeque<f64>; PPG_NUM_CH],

    // ── Which signal type is displayed
    pub view: ViewMode,

    // ── Status
    pub mode: AppMode,
    pub battery: Option<f32>,
    pub accel: Option<(f32, f32, f32)>,
    pub gyro: Option<(f32, f32, f32)>,

    // ── Contact quality (horseshoe)
    pub contact_quality: [ContactQuality; NUM_CH],
    pub contact_trackers: [ContactQualityTracker; NUM_CH],
    pub is_touching_forehead: bool,

    // ── Device info (from control JSON `v1` response)
    pub device_info: Option<serde_json::Map<String, serde_json::Value>>,

    // ── Control message log (most recent at the back)
    pub control_log: VecDeque<String>,

    // ── IMU ring buffers (3 axes each, stored as f64 for chart compat)
    pub accel_bufs: [VecDeque<f64>; 3],
    pub gyro_bufs: [VecDeque<f64>; 3],

    // ── Rate tracking
    total_samples: u64,
    pkt_times: VecDeque<Instant>,

    // ── UI controls
    scale_idx: usize,
    pub paused: bool,

    // ── Device picker
    pub show_picker: bool,
    pub picker_cursor: usize,
    /// Display strings for each discovered device: "Name  [short-id]"
    pub picker_entries: Vec<String>,
    /// Index into picker_entries that is currently connected.
    pub picker_connected_idx: Option<usize>,
    /// Spinner shown inside the picker while a scan is running.
    pub picker_scanning: bool,

    /// Last connection / setup error, shown in the Disconnected status line.
    pub last_error: Option<String>,

    /// When true, draw a dim raw trace + a bright smoothed overlay on each chart.
    pub smooth: bool,

    // ── CSV recording ──────────────────────────────────────────────────────
    pub(crate) csv_recorder: Option<CsvRecorder>,

    // ── ACT (adaptive chirplet transform) — feature-gated ──────────────────
    #[cfg(feature = "act")]
    pub act_engine: Option<muse_rs::act::ActEngine>,
    #[cfg(feature = "act")]
    pub act_results: [Option<muse_rs::act::ChannelResult>; NUM_CH],
    #[cfg(feature = "act")]
    pub act_opts: muse_rs::act::TransformOpts,
    #[cfg(feature = "act")]
    pub act_hop: usize,
    #[cfg(feature = "act")]
    pub act_samples_since_last: usize,
    #[cfg(feature = "act")]
    pub act_last_ms: f64,

    // ── Alpha band analysis — feature-gated ────────────────────────────────
    #[cfg(feature = "act")]
    pub alpha_snapshot: Option<muse_rs::alpha::AlphaSnapshot>,
    #[cfg(feature = "act")]
    pub alpha_history: std::collections::VecDeque<muse_rs::alpha::AlphaSnapshot>,
    #[cfg(feature = "act")]
    pub eye_detector: muse_rs::alpha::EyeStateDetector,
    #[cfg(feature = "act")]
    pub fft_snapshot: Option<muse_rs::alpha::FftSnapshot>,

    // ── Baseline detector — feature-gated ───────────────────────────────
    #[cfg(feature = "act")]
    pub baseline: muse_rs::baseline::BaselineDetector,

    // ── Absorption detector — feature-gated ──────────────────────────────
    #[cfg(feature = "act")]
    pub absorption: muse_rs::absorption::AbsorptionDetector,

    // ── Entrainment detector — feature-gated ────────────────────────────
    #[cfg(feature = "act")]
    pub entrainment: muse_rs::entrainment::EntrainmentDetector,

    // ── Approach detector — feature-gated ────────────────────────────────
    #[cfg(feature = "act")]
    pub approach: muse_rs::approach::ApproachDetector,

    // ── Band power time-series — feature-gated ─────────────────────────────
    #[cfg(feature = "act")]
    pub band_history: muse_rs::alpha::BandHistory,
    /// Number of time-points visible on x-axis in Bands view (zoom level).
    #[cfg(feature = "act")]
    pub bands_x_span: usize,
}

impl App {
    /// Create a fresh `App` in [`AppMode::Scanning`] with all fields at their defaults.
    fn new() -> Self {
        Self {
            bufs: std::array::from_fn(|_| VecDeque::with_capacity(EEG_BUF_CAP + 16)),
            filtered_bufs: std::array::from_fn(|_| VecDeque::with_capacity(EEG_BUF_CAP + 16)),
            eeg_filters: std::array::from_fn(|_| muse_rs::filters::ChannelFilter::new(EEG_HZ)),
            epoch_validators: std::array::from_fn(|_| muse_rs::filters::EpochValidator::default()),
            ppg_bufs: std::array::from_fn(|_| VecDeque::with_capacity(PPG_BUF_SIZE + 16)),
            view: ViewMode::Eeg,
            mode: AppMode::Scanning,
            battery: None,
            accel: None,
            gyro: None,
            contact_quality: [ContactQuality::NoContact; NUM_CH],
            contact_trackers: std::array::from_fn(|i| ContactQualityTracker::new(i)),
            is_touching_forehead: false,
            device_info: None,
            control_log: VecDeque::with_capacity(CONTROL_LOG_MAX + 1),
            accel_bufs: std::array::from_fn(|_| VecDeque::with_capacity(IMU_BUF_SIZE + 16)),
            gyro_bufs: std::array::from_fn(|_| VecDeque::with_capacity(IMU_BUF_SIZE + 16)),
            total_samples: 0,
            pkt_times: VecDeque::with_capacity(256),
            scale_idx: DEFAULT_SCALE,
            paused: false,
            show_picker: false,
            picker_cursor: 0,
            picker_entries: vec![],
            picker_connected_idx: None,
            picker_scanning: false,
            last_error: None,
            smooth: true,
            csv_recorder: None,
            #[cfg(feature = "act")]
            act_engine: None,
            #[cfg(feature = "act")]
            act_results: std::array::from_fn(|_| None),
            #[cfg(feature = "act")]
            act_opts: muse_rs::act::TransformOpts::default(),
            #[cfg(feature = "act")]
            act_hop: 64,
            #[cfg(feature = "act")]
            act_samples_since_last: 0,
            #[cfg(feature = "act")]
            act_last_ms: 0.0,
            #[cfg(feature = "act")]
            alpha_snapshot: None,
            #[cfg(feature = "act")]
            alpha_history: std::collections::VecDeque::with_capacity(256),
            #[cfg(feature = "act")]
            eye_detector: muse_rs::alpha::EyeStateDetector::default(),
            #[cfg(feature = "act")]
            fft_snapshot: None,
            #[cfg(feature = "act")]
            baseline: muse_rs::baseline::BaselineDetector::default(),
            #[cfg(feature = "act")]
            absorption: muse_rs::absorption::AbsorptionDetector::default(),
            #[cfg(feature = "act")]
            entrainment: muse_rs::entrainment::EntrainmentDetector::default(),
            #[cfg(feature = "act")]
            approach: muse_rs::approach::ApproachDetector::default(),
            #[cfg(feature = "act")]
            band_history: muse_rs::alpha::BandHistory::new(),
            #[cfg(feature = "act")]
            bands_x_span: 120, // default: 30 seconds (120 × 250ms)
        }
    }

    /// Append `samples` from electrode `ch` to the rolling ring-buffer.
    ///
    /// Drops the oldest samples to keep the buffer at exactly `BUF_SIZE` entries.
    /// Channel 0 (TP9) is also used to update the packet-rate tracker.
    ///
    /// No-ops when `paused` is `true` or `ch >= NUM_CH`.
    pub fn push(&mut self, ch: usize, samples: &[f64]) {
        if self.paused || ch >= NUM_CH {
            return;
        }
        // CSV recording: buffer raw samples
        if let Some(rec) = &mut self.csv_recorder {
            rec.push_raw(ch, samples);
        }
        let buf = &mut self.bufs[ch];
        let fbuf = &mut self.filtered_bufs[ch];
        let filt = &mut self.eeg_filters[ch];
        for &v in samples {
            buf.push_back(v);
            while buf.len() > EEG_BUF_CAP {
                buf.pop_front();
            }
            let fv = filt.process(v);
            fbuf.push_back(fv);
            while fbuf.len() > EEG_BUF_CAP {
                fbuf.pop_front();
            }
        }
        if ch == 0 {
            self.total_samples += samples.len() as u64;
            #[cfg(feature = "act")]
            {
                self.act_samples_since_last += samples.len();
            }
            let now = Instant::now();
            self.pkt_times.push_back(now);
            while self
                .pkt_times
                .front()
                .map(|t| now.duration_since(*t) > Duration::from_secs(2))
                .unwrap_or(false)
            {
                self.pkt_times.pop_front();
            }
        }
    }

    /// Append raw PPG ADC samples to the rolling ring-buffer for channel `ch`.
    pub fn push_ppg(&mut self, ch: usize, samples: &[u32]) {
        if self.paused || ch >= PPG_NUM_CH {
            return;
        }
        let buf = &mut self.ppg_bufs[ch];
        for &v in samples {
            buf.push_back(v as f64);
            while buf.len() > PPG_BUF_SIZE {
                buf.pop_front();
            }
        }
    }

    /// Wipe all EEG buffers and reset transient sensor readings.
    ///
    /// Called when disconnecting or reconnecting so the charts start blank.
    /// Does **not** change `mode`, `show_picker`, or any picker state — those
    /// are managed by the caller.
    pub fn clear(&mut self) {
        for b in &mut self.bufs {
            b.clear();
        }
        for b in &mut self.filtered_bufs {
            b.clear();
        }
        for f in &mut self.eeg_filters {
            f.reset();
        }
        for v in &mut self.epoch_validators {
            v.reset();
        }
        for b in &mut self.ppg_bufs {
            b.clear();
        }
        for b in &mut self.accel_bufs {
            b.clear();
        }
        for b in &mut self.gyro_bufs {
            b.clear();
        }
        self.total_samples = 0;
        self.pkt_times.clear();
        self.battery = None;
        self.accel = None;
        self.gyro = None;
        self.contact_quality = [ContactQuality::NoContact; NUM_CH];
        for t in &mut self.contact_trackers {
            t.reset();
        }
        self.is_touching_forehead = false;
        self.device_info = None;
        self.control_log.clear();
        self.last_error = None;
        #[cfg(feature = "act")]
        {
            self.act_results = std::array::from_fn(|_| None);
            self.act_samples_since_last = 0;
            self.act_last_ms = 0.0;
            self.alpha_snapshot = None;
            self.alpha_history.clear();
            self.eye_detector = muse_rs::alpha::EyeStateDetector::default();
            self.fft_snapshot = None;
            self.baseline.stop();
            self.absorption.stop();
            self.entrainment.stop();
            self.approach.stop();
            self.band_history.clear();
        }
    }

    /// Append IMU samples to the accelerometer or gyroscope ring buffers.
    pub fn push_accel(&mut self, samples: &[(f32, f32, f32)]) {
        if self.paused {
            return;
        }
        for &(x, y, z) in samples {
            for (buf, val) in self.accel_bufs.iter_mut().zip([x, y, z]) {
                buf.push_back(val as f64);
                while buf.len() > IMU_BUF_SIZE {
                    buf.pop_front();
                }
            }
        }
    }

    pub fn push_gyro(&mut self, samples: &[(f32, f32, f32)]) {
        if self.paused {
            return;
        }
        for &(x, y, z) in samples {
            for (buf, val) in self.gyro_bufs.iter_mut().zip([x, y, z]) {
                buf.push_back(val as f64);
                while buf.len() > IMU_BUF_SIZE {
                    buf.pop_front();
                }
            }
        }
    }

    /// Recompute contact quality for a single electrode channel from its EEG buffer.
    /// Uses only the last ~1 second of raw samples and applies hysteresis.
    pub fn update_contact_quality(&mut self, ch: usize) {
        if ch >= NUM_CH {
            return;
        }
        let buf = &self.bufs[ch];
        let n = buf.len().min(CONTACT_QUALITY_WINDOW_SAMPLES);
        let samples: Vec<f64> = buf.iter().skip(buf.len() - n).copied().collect();
        self.contact_quality[ch] = self.contact_trackers[ch].update(&samples);
        self.is_touching_forehead = touching_forehead(&self.contact_quality);

        if self.view == ViewMode::Info {
            let t = &self.contact_trackers[ch];
            log::debug!(
                "CQ ch{}({}): RMS={:.1} thresh={:.0} raw={} stable={} streak={}",
                ch,
                EEG_CHANNEL_NAMES[ch],
                t.last_rms(),
                t.rms_good_threshold(),
                t.last_raw(),
                t.quality(),
                t.streak(),
            );
        }
    }

    /// Check if enough EEG samples have accumulated to run an ACT transform.
    /// If so, extract windows from all 4 channels and run the batched transform.
    #[cfg(feature = "act")]
    pub fn maybe_run_act(&mut self) {
        let engine = match &self.act_engine {
            Some(e) => e,
            None => return,
        };

        if self.act_samples_since_last < self.act_hop {
            return;
        }

        let wlen = engine.window_length();

        // All 4 channels must have enough data (both raw and filtered)
        if self.bufs.iter().any(|b| b.len() < wlen) {
            return;
        }
        if self.filtered_bufs.iter().any(|b| b.len() < wlen) {
            return;
        }

        // Extract latest `wlen` samples from each channel (filtered for FFT, raw for ACT)
        let filtered_windows: Vec<Vec<f64>> = self
            .filtered_bufs
            .iter()
            .map(|b| b.iter().skip(b.len() - wlen).copied().collect())
            .collect();
        let filtered_refs: Vec<&[f64]> = filtered_windows.iter().map(|w| w.as_slice()).collect();

        let raw_windows: Vec<Vec<f64>> = self
            .bufs
            .iter()
            .map(|b| b.iter().skip(b.len() - wlen).copied().collect())
            .collect();
        let _raw_refs: Vec<&[f64]> = raw_windows.iter().map(|w| w.as_slice()).collect();

        // Epoch-level artifact rejection (per channel, on filtered signal)
        let mut epoch_ok = [true; NUM_CH];
        for ch in 0..NUM_CH {
            let bad_contact = self.contact_quality[ch] == ContactQuality::NoContact;
            let status = self.epoch_validators[ch].validate(&filtered_windows[ch], bad_contact);
            epoch_ok[ch] = status.is_clean();
            if !status.is_clean() {
                log::trace!("Epoch ch{ch} rejected: {status:?}");
            }
        }

        // Compute FFT band powers from the **filtered** windows
        // Only include channels that passed artifact rejection
        let fft_snap = muse_rs::alpha::FftSnapshot::from_windows(&filtered_refs, EEG_HZ);

        // CSV recording: write FFT band powers
        if let Some(rec) = &mut self.csv_recorder {
            rec.push_fft(&fft_snap);
            rec.flush();
        }

        // Feed baseline detector with FFT results + artifact status
        if self.baseline.is_running() {
            self.baseline.feed(&fft_snap, &epoch_ok, &self.contact_quality);
            if !self.baseline.is_running() {
                beep_twice();
            }
        }

        // Feed absorption detector with FFT results + artifact status
        if self.absorption.is_running() {
            self.absorption.feed(&fft_snap, &epoch_ok, &self.contact_quality);
            if !self.absorption.is_running() {
                beep_twice();
            }
        }

        // Feed entrainment detector with filtered EEG windows + artifact status
        // Entrainment uses its own window length (may differ from main ACT engine)
        if self.entrainment.is_running() {
            let ent_wlen = self.entrainment.dict_config.length;
            let ent_ready = self.filtered_bufs.iter().all(|b| b.len() >= ent_wlen);
            if ent_ready {
                let ent_windows: Vec<Vec<f64>> = self
                    .filtered_bufs
                    .iter()
                    .map(|b| b.iter().skip(b.len() - ent_wlen).copied().collect())
                    .collect();
                let ent_refs: Vec<&[f64]> = ent_windows.iter().map(|w| w.as_slice()).collect();
                self.entrainment.feed(&ent_refs, &epoch_ok, &self.contact_quality);
                if !self.entrainment.is_running() {
                    beep_twice();
                }
            }
        }

        // Feed approach detector with FFT results + artifact status
        if self.approach.is_running() {
            self.approach.feed(&fft_snap, &epoch_ok, &self.contact_quality);
            if !self.approach.is_running() {
                beep_twice();
            }
        }

        self.fft_snapshot = Some(fft_snap);

        let t0 = Instant::now();
        // ACT uses filtered signal too (consistent with Cui 2008 which bandpass-filtered first)
        match engine.transform_batch(&filtered_refs, &self.act_opts) {
            Ok(results) => {
                self.act_last_ms = t0.elapsed().as_secs_f64() * 1000.0;

                // Compute alpha metrics before moving results
                if results.len() >= NUM_CH {
                    let snap = muse_rs::alpha::AlphaSnapshot::from_channel_results(&results);
                    self.eye_detector.update(&snap);
                    self.alpha_history.push_back(snap);
                    // Keep ~60 seconds of history at hop=64 (250ms) → 240 snapshots
                    while self.alpha_history.len() > 240 {
                        self.alpha_history.pop_front();
                    }
                    self.alpha_snapshot = Some(snap);

                    // Push band history (requires both ACT + FFT snapshots)
                    if let Some(fft) = &self.fft_snapshot {
                        self.band_history.push(&snap, fft);
                    }
                }

                for (ch, r) in results.into_iter().enumerate() {
                    if ch < NUM_CH {
                        for (i, c) in r.chirplets.iter().enumerate() {
                            log::debug!(
                                "ACT ch{ch} chirp[{i}]: fc={:.3} log_dt={:.2} cr={:.3} coeff={:.1}",
                                c.fc, c.log_dt, c.chirp_rate, c.coeff,
                            );
                        }
                        self.act_results[ch] = Some(r);
                    }
                }
            }
            Err(e) => {
                log::warn!("ACT transform_batch failed: {e}");
            }
        }
        self.act_samples_since_last = 0;
    }

    /// Append a control JSON string to the log, trimming old entries.
    pub fn push_control_log(&mut self, msg: String) {
        self.control_log.push_back(msg);
        while self.control_log.len() > CONTROL_LOG_MAX {
            self.control_log.pop_front();
        }
    }

    /// Compute the current EEG packet arrival rate in packets per second.
    ///
    /// Uses a 2-second sliding window of arrival timestamps recorded in
    /// `pkt_times`.  Returns `0.0` when fewer than 2 timestamps are available.
    fn pkt_rate(&self) -> f64 {
        let n = self.pkt_times.len();
        if n < 2 {
            return 0.0;
        }
        let span = self
            .pkt_times
            .back()
            .unwrap()
            .duration_since(self.pkt_times[0])
            .as_secs_f64();
        if span < 1e-9 {
            0.0
        } else {
            (n as f64 - 1.0) / span
        }
    }

    /// Return the current half-range of the Y axis in µV (e.g. 500 for ±500 µV).
    fn y_range(&self) -> f64 {
        Y_SCALES[self.scale_idx]
    }

    /// Increase the Y-axis scale (zoom out) by one step.  Clamped at the largest step.
    fn scale_up(&mut self) {
        if self.scale_idx + 1 < Y_SCALES.len() {
            self.scale_idx += 1;
        }
    }

    /// Decrease the Y-axis scale (zoom in) by one step.  Clamped at the smallest step.
    fn scale_down(&mut self) {
        if self.scale_idx > 0 {
            self.scale_idx -= 1;
        }
    }

    /// Choose the smallest Y-scale step that fits the current peak amplitude.
    ///
    /// Adds a 10 % headroom margin above the observed peak so the waveform
    /// doesn't immediately hit the axis.  Selects the largest scale if the
    /// peak exceeds all available steps.
    fn auto_scale(&mut self) {
        let peak = self
            .bufs
            .iter()
            .flat_map(|b| b.iter())
            .fold(0.0_f64, |acc, &v| acc.max(v.abs()));
        // Round up to the next scale step with a small headroom margin (10 %).
        let needed = peak * 1.1;
        self.scale_idx = Y_SCALES
            .iter()
            .position(|&s| s >= needed)
            .unwrap_or(Y_SCALES.len() - 1);
    }
}

// ── Helpers ───────────────────────────────────────────────────────────────────

/// Shorten a BLE identifier for compact display.
/// UUID  → last 8 hex chars, e.g. "90ABCDEF"
/// MAC   → last 8 chars, e.g.  "EE:FF"
fn short_id(id: &str) -> String {
    let trimmed = id.trim_matches(|c: char| c == '{' || c == '}');
    if trimmed.len() > 8 {
        trimmed[trimmed.len() - 8..].to_uppercase()
    } else {
        trimmed.to_uppercase()
    }
}

/// Build the display string shown in the picker list and the header.
fn device_entry(d: &MuseDevice) -> String {
    format!("{}  [{}]", d.name, short_id(&d.id))
}

/// Symmetric moving-average (boxcar) smoother.
///
/// Each output point is the mean of the `window` nearest input points (up to
/// `window/2` on each side, clamped at the buffer edges so the length is
/// preserved).  At 256 Hz a 9-sample window has a -3 dB point of ≈ 75 Hz —
/// it visibly quiets high-frequency noise while leaving alpha/beta intact.
fn smooth_signal(data: &[(f64, f64)], window: usize) -> Vec<(f64, f64)> {
    if data.len() < 3 || window < 2 {
        return data.to_vec();
    }
    let half = window / 2;
    data.iter()
        .enumerate()
        .map(|(i, &(x, _))| {
            let start = i.saturating_sub(half);
            let end = (i + half + 1).min(data.len());
            let sum: f64 = data[start..end].iter().map(|&(_, y)| y).sum();
            (x, sum / (end - start) as f64)
        })
        .collect()
}

// ── EEG simulator ─────────────────────────────────────────────────────────────

/// Generate one synthetic EEG sample at time `t` (seconds) for channel `ch`.
///
/// The signal is a superposition of three physiological-band sinusoids with
/// a deterministic pseudo-random noise floor:
///
/// | Component | Frequency | Amplitude | Notes |
/// |-----------|-----------|-----------|-------|
/// | Alpha     | 10 Hz     | ±20 µV    | phase-shifted per channel |
/// | Beta      | 22 Hz     | ±6 µV     | — |
/// | Theta     | 6 Hz      | ±10 µV    | — |
/// | Noise     | —         | ±4 µV     | deterministic hash of (t, ch) |
///
// ── Audio cues (macOS) ────────────────────────────────────────────────────────

/// Play a macOS system sound in a background thread (non-blocking).
fn play_system_sound(name: &str) {
    let path = format!("/System/Library/Sounds/{name}.aiff");
    std::thread::spawn(move || {
        let _ = std::process::Command::new("afplay")
            .arg(&path)
            .status();
    });
}

/// Single beep — used when baseline starts.
fn beep_once() {
    play_system_sound("Tink");
}

/// Double beep — used when baseline completes or fails.
fn beep_twice() {
    std::thread::spawn(|| {
        let path = "/System/Library/Sounds/Tink.aiff";
        let _ = std::process::Command::new("afplay").arg(path).status();
        std::thread::sleep(std::time::Duration::from_millis(250));
        let _ = std::process::Command::new("afplay").arg(path).status();
    });
}

/// Peak amplitude is approximately ±40 µV, which fits comfortably within the
/// ±50 µV scale automatically selected for `--simulate` mode.
fn sim_sample(t: f64, ch: usize) -> f64 {
    let phi = ch as f64 * PI / 2.5;
    let alpha = 20.0 * (2.0 * PI * 10.0 * t + phi).sin();
    let beta = 6.0 * (2.0 * PI * 22.0 * t + phi * 1.7).sin();
    let theta = 10.0 * (2.0 * PI * 6.0 * t + phi * 0.9).sin();
    let nx = t * 1000.7 + ch as f64 * 137.508;
    let noise = ((nx.sin() * 9973.1).fract() - 0.5) * 8.0;
    alpha + beta + theta + noise
}

/// Spawn a background task that generates synthetic EEG, accelerometer, and
/// gyroscope data and writes it into `app` at the real headset rate.
///
/// * EEG: 12 samples per channel every `12 / 256 Hz ≈ 46.9 ms` (4 channels).
/// * Telemetry / IMU: updated every 21 EEG ticks (≈ 984 ms).
///
/// The task runs for the lifetime of the process and is only started when the
/// `--simulate` flag is passed.  Time is advanced even while paused so that
/// the waveform resumes from the correct phase when unpaused.
fn spawn_simulator(app: Arc<Mutex<App>>) {
    tokio::spawn(async move {
        let pkt_interval = Duration::from_secs_f64(12.0 / EEG_HZ);
        let mut ticker = tokio::time::interval(pkt_interval);
        let dt = 1.0 / EEG_HZ;
        let mut t = 0.0_f64;
        let mut seq = 0u32;
        loop {
            ticker.tick().await;
            let mut s = app.lock().unwrap();
            if s.paused {
                t += 12.0 * dt;
                continue;
            }
            for ch in 0..NUM_CH {
                let samples: Vec<f64> =
                    (0..12).map(|i| sim_sample(t + i as f64 * dt, ch)).collect();
                s.push(ch, &samples);
            }
            // Update contact quality from EEG buffers
            for ch in 0..NUM_CH {
                s.update_contact_quality(ch);
            }
            #[cfg(feature = "act")]
            s.maybe_run_act();

            seq = seq.wrapping_add(1);

            // Generate IMU data every ~4 EEG packets (≈ 52 Hz equivalent)
            if seq % 4 == 0 {
                let accel_sample = (
                    (0.01 * (2.0 * PI * 0.3 * t).sin()) as f32,
                    (0.02 * (2.0 * PI * 0.5 * t).cos()) as f32,
                    (-1.0 + 0.005 * (2.0 * PI * 0.1 * t).sin()) as f32,
                );
                let gyro_sample = (
                    (0.12 * (2.0 * PI * 0.2 * t).sin()) as f32,
                    (0.08 * (2.0 * PI * 0.3 * t).cos()) as f32,
                    (0.05 * (2.0 * PI * 0.1 * t).sin()) as f32,
                );
                s.accel = Some(accel_sample);
                s.gyro = Some(gyro_sample);
                s.push_accel(&[accel_sample]);
                s.push_gyro(&[gyro_sample]);
            }

            if seq.is_multiple_of(21) {
                s.battery = Some((85.0 - t as f32 / 300.0).clamp(0.0, 100.0));
            }

            // Simulate device info once at startup
            if seq == 1 {
                let mut info = serde_json::Map::new();
                info.insert("fw".into(), serde_json::Value::String("SIM.0.0".into()));
                info.insert("hw".into(), serde_json::Value::String("SIM-HW".into()));
                info.insert("sn".into(), serde_json::Value::String("SIM-0000".into()));
                info.insert("hn".into(), serde_json::Value::String("Muse-SIM".into()));
                info.insert("tp".into(), serde_json::Value::String("Simulated".into()));
                info.insert("bl".into(), serde_json::Value::String("0.0.0".into()));
                info.insert("pv".into(), serde_json::Value::Number(2.into()));
                info.insert("bn".into(), serde_json::Value::Number(9999.into()));
                s.device_info = Some(info);
                s.push_control_log(r#"{"rc":0,"fw":"SIM.0.0","hw":"SIM-HW","sn":"SIM-0000"}"#.into());
            }

            // Periodic simulated control messages
            if seq > 1 && seq.is_multiple_of(100) {
                let bp = s.battery.unwrap_or(0.0);
                s.push_control_log(format!(r#"{{"rc":0,"bp":{bp:.0},"ts":{t:.0}}}"#));
            }

            t += 12.0 * dt;
        }
    });
}

// ── BLE helpers ───────────────────────────────────────────────────────────────

/// Start a background scan; return a receiver for the results.
///
/// The task has a hard deadline of `scan_timeout + 10 s` to guard against
/// `Manager::new()` or `adapter.start_scan()` hanging indefinitely inside
/// btleplug when the Bluetooth stack is in a bad state.
/// Scan result delivered through the oneshot channel.
struct ScanResult {
    devices: Vec<MuseDevice>,
    /// `Some(msg)` when the scan failed or timed out.
    error: Option<String>,
}

fn start_scan(config: MuseClientConfig) -> oneshot::Receiver<ScanResult> {
    let (tx, rx) = oneshot::channel();
    let deadline = Duration::from_secs(config.scan_timeout_secs + 10);
    tokio::spawn(async move {
        let result = match tokio::time::timeout(deadline, MuseClient::new(config).scan_all()).await
        {
            Ok(Ok(devices)) => {
                log::info!("Scan completed: {} device(s) found", devices.len());
                ScanResult { devices, error: None }
            }
            Ok(Err(e)) => {
                log::error!("Scan failed: {e}");
                ScanResult { devices: vec![], error: Some(format!("{e}")) }
            }
            Err(_) => {
                log::error!("Scan timed out after {deadline:?}");
                ScanResult { devices: vec![], error: Some("scan timed out".into()) }
            }
        };
        let _ = tx.send(result);
    });
    rx
}

/// Wipe all live signal data and schedule a fresh BLE scan.
///
/// Called identically from both "unexpected disconnect" (step 2) and
/// "connection attempt failed" (step 1b), ensuring the recovery path is the
/// same in both cases:
///   • EEG buffers, battery, sensors, last error are cleared
///   • Picker overlay is closed so the UI is in a clean state
///   • Mode switches to Scanning immediately (spinner appears right away)
///   • A new scan is queued after `delay_secs` (gives BLE stack time to settle)
fn restart_scan(
    app: &Arc<Mutex<App>>,
    pending_scan: &mut Option<oneshot::Receiver<ScanResult>>,
    retry_at: &mut Option<tokio::time::Instant>,
    delay_secs: u64,
) {
    {
        let mut s = app.lock().unwrap();
        s.clear();
        s.picker_connected_idx = None;
        s.picker_entries.clear();   // remove stale device from the list immediately
        s.show_picker = false;
        s.mode = AppMode::Scanning;
        s.picker_scanning = true;
    }
    if pending_scan.is_none() {
        *retry_at = Some(tokio::time::Instant::now() + Duration::from_secs(delay_secs));
    }
}

/// Spawn a task that forwards BLE events into `app`.
/// Returns immediately; the task runs until the peripheral disconnects.
fn spawn_event_task(mut rx: tokio::sync::mpsc::Receiver<MuseEvent>, app: Arc<Mutex<App>>) {
    tokio::spawn(async move {
        while let Some(ev) = rx.recv().await {
            let mut s = app.lock().unwrap();
            match ev {
                // The Connected event from setup_peripheral is now redundant
                // (mode is set by do_connect before spawning this task), but
                // we handle it anyway as a safety net.
                MuseEvent::Connected(_) => {}
                MuseEvent::Disconnected => {
                    s.mode = AppMode::Disconnected;
                    s.picker_connected_idx = None;
                    break;
                }
                MuseEvent::Eeg(r) if r.electrode < NUM_CH => {
                    s.push(r.electrode, &r.samples);
                    s.update_contact_quality(r.electrode);
                    #[cfg(feature = "act")]
                    if r.electrode == 0 {
                        s.maybe_run_act();
                    }
                }
                MuseEvent::Ppg(r) if r.ppg_channel < PPG_NUM_CH => {
                    s.push_ppg(r.ppg_channel, &r.samples);
                }
                MuseEvent::Telemetry(t) => {
                    s.battery = Some(t.battery_level);
                }
                MuseEvent::Accelerometer(a) => {
                    let imu_samples: Vec<(f32, f32, f32)> =
                        a.samples.iter().map(|v| (v.x, v.y, v.z)).collect();
                    if let Some(v) = imu_samples.last() {
                        s.accel = Some(*v);
                    }
                    s.push_accel(&imu_samples);
                }
                MuseEvent::Gyroscope(g) => {
                    let imu_samples: Vec<(f32, f32, f32)> =
                        g.samples.iter().map(|v| (v.x, v.y, v.z)).collect();
                    if let Some(v) = imu_samples.last() {
                        s.gyro = Some(*v);
                    }
                    s.push_gyro(&imu_samples);
                }
                MuseEvent::Control(c) => {
                    s.push_control_log(c.raw.clone());
                    if c.fields.contains_key("fw") {
                        s.device_info = Some(c.fields);
                    }
                }
                _ => {}
            }
        }
    });
}

/// Payload delivered on a successful connection.
struct ConnectOutcome {
    rx: mpsc::Receiver<MuseEvent>,
    handle: MuseHandle,
    device_idx: usize,
    name: String,
    id: String,
}

/// Kick off a background connection attempt and return immediately.
///
/// The caller receives a `oneshot` that resolves to:
/// - `Some(ConnectOutcome)` on success
/// - `None` on failure  (`app.mode` is already set to `Disconnected`)
///
/// All blocking work (`peripheral.connect()`, service discovery, startup
/// sequence) runs inside a spawned task so the main loop stays responsive.
fn start_connect(
    idx: usize,
    device: MuseDevice,
    app: Arc<Mutex<App>>,
    scan_cfg: MuseClientConfig,
) -> oneshot::Receiver<Option<ConnectOutcome>> {
    let (tx, rx) = oneshot::channel();

    // Immediately update UI to "Connecting…" without blocking.
    {
        let mut s = app.lock().unwrap();
        s.clear();
        s.mode = AppMode::Connecting(device.name.clone());
        s.picker_connected_idx = None;
        s.show_picker = false;
    }

    tokio::spawn(async move {
        let client = MuseClient::new(scan_cfg);
        match client.connect_to(device.clone()).await {
            Ok((evt_rx, h)) => {
                match h.start(false, false).await {
                    Ok(()) => {
                        let _ = tx.send(Some(ConnectOutcome {
                            rx: evt_rx,
                            handle: h,
                            device_idx: idx,
                            name: device.name.clone(),
                            id: short_id(&device.id),
                        }));
                    }
                    Err(e) => {
                        log::warn!("start() failed: {e}");
                        // Disconnect so BlueZ doesn't keep the link half-open.
                        let _ = h.disconnect().await;
                        let mut s = app.lock().unwrap();
                        s.mode = AppMode::Disconnected;
                        s.last_error = Some(format!("start() failed: {e}"));
                        let _ = tx.send(None);
                    }
                }
            }
            Err(e) => {
                log::warn!("connect_to failed: {e}");
                let mut s = app.lock().unwrap();
                s.mode = AppMode::Disconnected;
                s.last_error = Some(format!("connect_to failed: {e}"));
                let _ = tx.send(None);
            }
        }
    });

    rx
}

// ── Rendering ─────────────────────────────────────────────────────────────────

/// Top-level render callback handed to [`Terminal::draw`].
///
/// Divides the terminal into three horizontal bands (header / charts / footer)
/// and then optionally overlays the device-picker modal on top.
fn draw(frame: &mut Frame, app: &App) {
    let area = frame.area();
    let root = Layout::vertical([
        Constraint::Length(3),
        Constraint::Min(0),
        Constraint::Length(3),
    ])
    .split(area);

    draw_header(frame, root[0], app);
    draw_charts(frame, root[1], app);
    draw_footer(frame, root[2], app);

    if app.show_picker {
        draw_device_picker(frame, area, app);
    }
}

// ── Header ────────────────────────────────────────────────────────────────────

/// Return the current braille spinner frame based on wall-clock milliseconds.
///
/// The frame advances every 100 ms, producing a smooth animation at the
/// ~30 FPS render rate without requiring any additional state.
fn spinner_str() -> &'static str {
    let ms = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis();
    SPINNER[(ms / 100) as usize % SPINNER.len()]
}

/// Render the status bar at the top of the screen.
///
/// Displays (left to right): app title, connection status (with spinner when
/// scanning), battery percentage, packet rate, Y-axis scale, and total sample count.
/// The status label colour reflects the current mode: green = connected,
/// yellow = scanning/no devices, red = disconnected.
fn draw_header(frame: &mut Frame, area: Rect, app: &App) {
    let (label, color) = match &app.mode {
        AppMode::Scanning => (format!("{} Scanning…", spinner_str()), Color::Yellow),
        AppMode::Connecting(name) => (
            format!("{} Connecting to {}…", spinner_str(), name),
            Color::Yellow,
        ),
        AppMode::Connected { name, id } => {
            (format!("● {}  [{}]", name, id), Color::Green)
        }
        AppMode::Simulated => ("◆ Simulated".to_owned(), Color::Cyan),
        AppMode::NoDevices => (
            format!("{} No devices found — retrying…", spinner_str()),
            Color::Yellow,
        ),
        AppMode::Disconnected => {
            let reason = app
                .last_error
                .as_deref()
                .map(|e| format!(" ({e})"))
                .unwrap_or_default();
            (
                format!("{} Disconnected{reason} — retrying…", spinner_str()),
                Color::Red,
            )
        }
    };

    let bat = app
        .battery
        .map(|b| format!("Bat {b:.0}%"))
        .unwrap_or_else(|| "Bat N/A".into());

    let rate = format!("{:.1} pkt/s", app.pkt_rate());
    let scale = match app.view {
        ViewMode::Eeg => format!("±{:.0} µV", app.y_range()),
        ViewMode::Ppg | ViewMode::Imu => "auto".to_string(),
        ViewMode::Info => "—".to_string(),
        #[cfg(feature = "act")]
        ViewMode::Act => "—".to_string(),
        #[cfg(feature = "act")]
        ViewMode::Baseline => "—".to_string(),
        #[cfg(feature = "act")]
        ViewMode::Absorption => "—".to_string(),
        #[cfg(feature = "act")]
        ViewMode::Entrainment => "—".to_string(),
        #[cfg(feature = "act")]
        ViewMode::Bands => "—".to_string(),
        #[cfg(feature = "act")]
        ViewMode::Approach => "—".to_string(),
    };
    let view_label = match app.view {
        ViewMode::Eeg => "EEG",
        ViewMode::Ppg => "PPG",
        ViewMode::Info => "INFO",
        ViewMode::Imu => "IMU",
        #[cfg(feature = "act")]
        ViewMode::Act => "ACT",
        #[cfg(feature = "act")]
        ViewMode::Baseline => "BASELINE",
        #[cfg(feature = "act")]
        ViewMode::Absorption => "ABSORB",
        #[cfg(feature = "act")]
        ViewMode::Entrainment => "ENTRAIN",
        #[cfg(feature = "act")]
        ViewMode::Bands => "BANDS",
        #[cfg(feature = "act")]
        ViewMode::Approach => "FAA",
    };
    let total = format!("{}K smp", app.total_samples / 1_000);

    let line = Line::from(vec![
        Span::styled(
            " MUSE EEG Monitor ",
            Style::default().fg(Color::White).add_modifier(Modifier::BOLD),
        ),
        sep(),
        Span::styled(label, Style::default().fg(color).add_modifier(Modifier::BOLD)),
        sep(),
        Span::styled(
            view_label,
            Style::default()
                .fg(Color::LightYellow)
                .add_modifier(Modifier::BOLD),
        ),
        sep(),
        Span::styled(bat, Style::default().fg(Color::White)),
        sep(),
        Span::styled(rate, Style::default().fg(Color::White)),
        sep(),
        Span::styled(
            scale,
            Style::default()
                .fg(Color::LightBlue)
                .add_modifier(Modifier::BOLD),
        ),
        sep(),
        Span::styled(total, Style::default().fg(Color::DarkGray)),
        if let Some(rec) = &app.csv_recorder {
            Span::styled(
                format!("  ● REC {}", rec.prefix),
                Style::default().fg(Color::Red).add_modifier(Modifier::BOLD),
            )
        } else {
            Span::raw("")
        },
        Span::raw(" "),
    ]);

    frame.render_widget(
        Paragraph::new(line).block(Block::default().borders(Borders::ALL)),
        area,
    );
}

/// Dimmed vertical separator used between header fields.
#[inline]
fn sep<'a>() -> Span<'a> {
    Span::styled(" │ ", Style::default().fg(Color::DarkGray))
}

// ── EEG charts ────────────────────────────────────────────────────────────────

/// Render the waveform charts (EEG, PPG, Info, or IMU depending on `app.view`).
fn draw_charts(frame: &mut Frame, area: Rect, app: &App) {
    match app.view {
        ViewMode::Eeg => draw_eeg_charts(frame, area, app),
        ViewMode::Ppg => draw_ppg_charts(frame, area, app),
        ViewMode::Info => draw_info_view(frame, area, app),
        ViewMode::Imu => draw_imu_charts(frame, area, app),
        #[cfg(feature = "act")]
        ViewMode::Act => draw_act_view(frame, area, app),
        #[cfg(feature = "act")]
        ViewMode::Baseline => draw_baseline_view(frame, area, app),
        #[cfg(feature = "act")]
        ViewMode::Absorption => draw_absorption_view(frame, area, app),
        #[cfg(feature = "act")]
        ViewMode::Entrainment => draw_entrainment_view(frame, area, app),
        #[cfg(feature = "act")]
        ViewMode::Bands => draw_bands_view(frame, area, app),
        #[cfg(feature = "act")]
        ViewMode::Approach => draw_approach_view(frame, area, app),
    }
}

/// Render the four EEG waveform charts stacked vertically.
fn draw_eeg_charts(frame: &mut Frame, area: Rect, app: &App) {
    let rows = Layout::vertical([
        Constraint::Ratio(1, 4),
        Constraint::Ratio(1, 4),
        Constraint::Ratio(1, 4),
        Constraint::Ratio(1, 4),
    ])
    .split(area);

    let y_range = app.y_range();
    let data: Vec<Vec<(f64, f64)>> = (0..NUM_CH)
        .map(|ch| {
            app.bufs[ch]
                .iter()
                .enumerate()
                .map(|(i, &v)| (i as f64 / EEG_HZ, v.clamp(-y_range, y_range)))
                .collect()
        })
        .collect();

    for ch in 0..NUM_CH {
        draw_channel(frame, rows[ch], ch, &data[ch], app);
    }
}

/// Render the three PPG optical channel charts stacked vertically.
fn draw_ppg_charts(frame: &mut Frame, area: Rect, app: &App) {
    let rows = Layout::vertical([
        Constraint::Ratio(1, 3),
        Constraint::Ratio(1, 3),
        Constraint::Ratio(1, 3),
    ])
    .split(area);

    for ch in 0..PPG_NUM_CH {
        draw_ppg_channel(frame, rows[ch], ch, app);
    }
}

/// Render a single PPG channel chart.
fn draw_ppg_channel(frame: &mut Frame, area: Rect, ch: usize, app: &App) {
    let color = PPG_COLORS[ch];
    let name = PPG_CHANNEL_NAMES[ch];
    let buf = &app.ppg_bufs[ch];

    // Compute auto Y range from actual data.
    let (min_v, max_v) = if buf.is_empty() {
        (0.0_f64, 1.0_f64)
    } else {
        let min = buf.iter().copied().fold(f64::INFINITY, f64::min);
        let max = buf.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        // Add 5% margin so the waveform doesn't touch the edges.
        let margin = (max - min).max(1.0) * 0.05;
        (min - margin, max + margin)
    };

    let data: Vec<(f64, f64)> = buf
        .iter()
        .enumerate()
        .map(|(i, &v)| (i as f64 / PPG_HZ, v))
        .collect();

    let smoothed: Vec<(f64, f64)> = if app.smooth {
        smooth_signal(&data, SMOOTH_WINDOW)
    } else {
        vec![]
    };

    let dim_color = match ch {
        0 => Color::Rgb(0, 60, 90),   // dim light-blue
        1 => Color::Rgb(90, 40, 40),  // dim light-red
        _ => Color::Rgb(90, 0, 0),    // dim red
    };

    let datasets: Vec<Dataset> = if app.smooth {
        vec![
            Dataset::default()
                .marker(symbols::Marker::Braille)
                .graph_type(GraphType::Line)
                .style(Style::default().fg(dim_color))
                .data(&data),
            Dataset::default()
                .marker(symbols::Marker::Braille)
                .graph_type(GraphType::Line)
                .style(Style::default().fg(color))
                .data(&smoothed),
        ]
    } else {
        vec![Dataset::default()
            .marker(symbols::Marker::Braille)
            .graph_type(GraphType::Line)
            .style(Style::default().fg(color))
            .data(&data)]
    };

    let smooth_tag = if app.smooth { " [SMOOTH]" } else { "" };
    let title = format!(
        " {name}  min:{min_v:.0}  max:{max_v:.0}{smooth_tag} "
    );

    let y_mid = (min_v + max_v) / 2.0;
    let y_labels: Vec<String> = vec![
        format!("{:.0}", min_v),
        format!("{:.0}", y_mid),
        format!("{:.0}", max_v),
    ];
    let x_labels = vec![
        "0s".to_string(),
        format!("{:.1}s", WINDOW_SECS / 2.0),
        format!("{:.0}s", WINDOW_SECS),
    ];

    let chart = Chart::new(datasets)
        .block(
            Block::default()
                .title(Span::styled(
                    title,
                    Style::default().fg(color).add_modifier(Modifier::BOLD),
                ))
                .borders(Borders::ALL)
                .border_style(Style::default().fg(color)),
        )
        .x_axis(
            Axis::default()
                .bounds([0.0, WINDOW_SECS])
                .labels(x_labels)
                .style(Style::default().fg(Color::DarkGray)),
        )
        .y_axis(
            Axis::default()
                .bounds([min_v, max_v])
                .labels(y_labels)
                .style(Style::default().fg(Color::DarkGray)),
        );

    frame.render_widget(chart, area);
}

/// Render a single EEG channel chart into `area`.
///
/// * **Title bar** — electrode name, live min/max/RMS stats in µV,
///   and optional `[CLIP]` / `[SMOOTH]` badges.
/// * **Border** — turns red when any sample in the buffer exceeds the Y axis.
/// * **Smooth mode** — draws the raw signal in a dim colour as background
///   context, then overlays the 9-sample moving-average in the full channel colour.
/// * **Raw mode** — single bright line with no overlay.
fn draw_channel(frame: &mut Frame, area: Rect, ch: usize, data: &[(f64, f64)], app: &App) {
    let color = COLORS[ch];
    let y_range = app.y_range();
    let name = EEG_CHANNEL_NAMES[ch];

    // Compute stats from the raw (un-clamped) buffer so min/max reflect true signal.
    let (min_v, max_v, rms_v) = {
        let buf = &app.bufs[ch];
        if buf.is_empty() {
            (0.0_f64, 0.0_f64, 0.0_f64)
        } else {
            let min = buf.iter().copied().fold(f64::INFINITY, f64::min);
            let max = buf.iter().copied().fold(f64::NEG_INFINITY, f64::max);
            let rms = (buf.iter().map(|&v| v * v).sum::<f64>() / buf.len() as f64).sqrt();
            (min, max, rms)
        }
    };

    // Clipping = any sample lies outside the current Y window.
    let clipping = max_v > y_range || min_v < -y_range;

    let border_color = if clipping { Color::Red } else { color };

    let clip_tag = if clipping { " [CLIP +]" } else { "" };
    let smooth_tag = if app.smooth { " [SMOOTH]" } else { "" };
    let title = format!(
        " {name}  min:{min_v:+6.1}  max:{max_v:+6.1}  rms:{rms_v:5.1} µV{clip_tag}{smooth_tag} "
    );

    let y_labels: Vec<String> = [-1.0, -0.5, 0.0, 0.5, 1.0]
        .iter()
        .map(|&f| format!("{:+.0}", f * y_range))
        .collect();

    let x_labels = vec![
        "0s".to_string(),
        format!("{:.1}s", WINDOW_SECS / 2.0),
        format!("{:.0}s", WINDOW_SECS),
    ];

    // Build dataset(s).
    // Smooth mode: dim raw trace underneath + bright smoothed line on top.
    // Raw mode:    single bright raw trace.
    let smoothed: Vec<(f64, f64)> = if app.smooth {
        smooth_signal(data, SMOOTH_WINDOW)
    } else {
        vec![]
    };

    let datasets: Vec<Dataset> = if app.smooth {
        vec![
            // Layer 1 – raw signal, dimmed so it reads as context/background.
            Dataset::default()
                .marker(symbols::Marker::Braille)
                .graph_type(GraphType::Line)
                .style(Style::default().fg(DIM_COLORS[ch]))
                .data(data),
            // Layer 2 – smoothed overlay in the full channel colour.
            Dataset::default()
                .marker(symbols::Marker::Braille)
                .graph_type(GraphType::Line)
                .style(Style::default().fg(color))
                .data(&smoothed),
        ]
    } else {
        vec![Dataset::default()
            .marker(symbols::Marker::Braille)
            .graph_type(GraphType::Line)
            .style(Style::default().fg(color))
            .data(data)]
    };

    let chart = Chart::new(datasets)
        .block(
            Block::default()
                .title(Span::styled(
                    title,
                    Style::default().fg(color).add_modifier(Modifier::BOLD),
                ))
                .borders(Borders::ALL)
                .border_style(Style::default().fg(border_color)),
        )
        .x_axis(
            Axis::default()
                .bounds([0.0, WINDOW_SECS])
                .labels(x_labels)
                .style(Style::default().fg(Color::DarkGray)),
        )
        .y_axis(
            Axis::default()
                .bounds([-y_range, y_range])
                .labels(y_labels)
                .style(Style::default().fg(Color::DarkGray)),
        );

    frame.render_widget(chart, area);
}

// ── Info view ─────────────────────────────────────────────────────────────────

/// Render the Info view: horseshoe contact quality, battery, device info, control log.
fn draw_info_view(frame: &mut Frame, area: Rect, app: &App) {
    let [top_area, log_area] = Layout::vertical([
        Constraint::Length(12),
        Constraint::Min(3),
    ])
    .areas(area);

    let [horseshoe_area, info_area] = Layout::horizontal([
        Constraint::Length(28),
        Constraint::Min(30),
    ])
    .areas(top_area);

    // ── Horseshoe panel ──────────────────────────────────────────────────
    draw_horseshoe(frame, horseshoe_area, app);

    // ── Device info + battery panel ──────────────────────────────────────
    draw_device_info_panel(frame, info_area, app);

    // ── Control log ──────────────────────────────────────────────────────
    draw_control_log(frame, log_area, app);
}

/// Colour for a contact quality tier.
fn quality_color(q: ContactQuality) -> Color {
    match q {
        ContactQuality::Good => Color::Green,
        ContactQuality::Ok => Color::Yellow,
        ContactQuality::Bad => Color::Red,
        ContactQuality::NoContact => Color::DarkGray,
    }
}

/// Symbol for a contact quality tier.
fn quality_symbol(q: ContactQuality) -> &'static str {
    match q {
        ContactQuality::Good => "●●",
        ContactQuality::Ok => "●○",
        ContactQuality::Bad => "○○",
        ContactQuality::NoContact => "  ",
    }
}

/// Render the horseshoe contact quality indicator.
fn draw_horseshoe(frame: &mut Frame, area: Rect, app: &App) {
    let names = ["TP9 ", "AF7 ", "AF8 ", "TP10"];
    let mut lines: Vec<Line> = Vec::new();

    lines.push(Line::from(""));

    // Horseshoe visual layout (mimics the Muse headband shape)
    //      AF7  AF8
    //  TP9          TP10
    lines.push(Line::from(vec![
        Span::raw("       "),
        Span::styled(
            format!("{} {}", names[1], quality_symbol(app.contact_quality[1])),
            Style::default().fg(quality_color(app.contact_quality[1])),
        ),
        Span::raw("  "),
        Span::styled(
            format!("{} {}", quality_symbol(app.contact_quality[2]), names[2]),
            Style::default().fg(quality_color(app.contact_quality[2])),
        ),
    ]));

    lines.push(Line::from(""));

    lines.push(Line::from(vec![
        Span::raw("  "),
        Span::styled(
            format!("{} {}", names[0], quality_symbol(app.contact_quality[0])),
            Style::default().fg(quality_color(app.contact_quality[0])),
        ),
        Span::raw("        "),
        Span::styled(
            format!("{} {}", quality_symbol(app.contact_quality[3]), names[3]),
            Style::default().fg(quality_color(app.contact_quality[3])),
        ),
    ]));

    lines.push(Line::from(""));

    // Per-channel quality labels with debug info
    for (i, name) in names.iter().enumerate() {
        let q = app.contact_quality[i];
        let t = &app.contact_trackers[i];
        let rms = t.last_rms();
        let thresh = t.rms_good_threshold();
        let raw = t.last_raw();
        let streak = t.streak();
        lines.push(Line::from(vec![
            Span::raw("  "),
            Span::styled(
                format!("{name}"),
                Style::default().fg(COLORS[i]).add_modifier(Modifier::BOLD),
            ),
            Span::raw("  "),
            Span::styled(
                format!("{q}"),
                Style::default().fg(quality_color(q)),
            ),
            Span::styled(
                format!("  RMS:{rms:5.1} / {thresh:.0}  raw:{raw}  streak:{streak}"),
                Style::default().fg(Color::DarkGray),
            ),
        ]));
    }

    lines.push(Line::from(""));

    let forehead_color = if app.is_touching_forehead {
        Color::Green
    } else {
        Color::Red
    };
    let forehead_text = if app.is_touching_forehead {
        "✓ On forehead"
    } else {
        "✗ Not on forehead"
    };
    lines.push(Line::from(vec![
        Span::raw("  "),
        Span::styled(
            forehead_text,
            Style::default()
                .fg(forehead_color)
                .add_modifier(Modifier::BOLD),
        ),
    ]));

    frame.render_widget(
        Paragraph::new(lines).block(
            Block::default()
                .title(Span::styled(
                    " Contact Quality ",
                    Style::default()
                        .fg(Color::White)
                        .add_modifier(Modifier::BOLD),
                ))
                .borders(Borders::ALL)
                .border_style(Style::default().fg(Color::White)),
        ),
        area,
    );
}

/// Render device info and battery gauge.
fn draw_device_info_panel(frame: &mut Frame, area: Rect, app: &App) {
    let mut lines: Vec<Line> = Vec::new();

    // Battery gauge
    let bat_pct = app.battery.unwrap_or(0.0);
    let bar_len = 20usize;
    let filled = ((bat_pct / 100.0) * bar_len as f32).round() as usize;
    let bar: String = format!(
        "{}{}",
        "█".repeat(filled.min(bar_len)),
        "░".repeat(bar_len.saturating_sub(filled)),
    );
    let bat_color = if bat_pct > 50.0 {
        Color::Green
    } else if bat_pct > 20.0 {
        Color::Yellow
    } else {
        Color::Red
    };
    lines.push(Line::from(vec![
        Span::styled(
            "  Battery  ",
            Style::default()
                .fg(Color::White)
                .add_modifier(Modifier::BOLD),
        ),
        Span::styled(bar, Style::default().fg(bat_color)),
        Span::styled(
            format!("  {bat_pct:.0}%"),
            Style::default().fg(bat_color).add_modifier(Modifier::BOLD),
        ),
    ]));

    lines.push(Line::from(""));

    // Device info fields
    if let Some(ref info) = app.device_info {
        let fields = [
            ("fw", "Firmware"),
            ("hw", "Hardware"),
            ("sn", "Serial  "),
            ("hn", "Name    "),
            ("tp", "Type    "),
            ("bl", "Bootldr "),
            ("pv", "Protocol"),
            ("bn", "Build # "),
        ];
        for (key, label) in &fields {
            if let Some(val) = info.get(*key) {
                let val_str = match val {
                    serde_json::Value::String(s) => s.clone(),
                    other => other.to_string(),
                };
                lines.push(Line::from(vec![
                    Span::styled(
                        format!("  {label}  "),
                        Style::default().fg(Color::DarkGray),
                    ),
                    Span::styled(val_str, Style::default().fg(Color::White)),
                ]));
            }
        }
    } else {
        lines.push(Line::from(Span::styled(
            "  Device info not yet received (send 'v1' or connect)",
            Style::default().fg(Color::DarkGray),
        )));
    }

    frame.render_widget(
        Paragraph::new(lines).block(
            Block::default()
                .title(Span::styled(
                    " Device Info ",
                    Style::default()
                        .fg(Color::White)
                        .add_modifier(Modifier::BOLD),
                ))
                .borders(Borders::ALL)
                .border_style(Style::default().fg(Color::White)),
        ),
        area,
    );
}

/// Render the scrollable control message log.
fn draw_control_log(frame: &mut Frame, area: Rect, app: &App) {
    let inner_height = area.height.saturating_sub(2) as usize;
    let lines: Vec<Line> = app
        .control_log
        .iter()
        .rev()
        .take(inner_height)
        .rev()
        .enumerate()
        .map(|(i, msg)| {
            let color = if i % 2 == 0 {
                Color::Gray
            } else {
                Color::DarkGray
            };
            Line::from(Span::styled(format!("  {msg}"), Style::default().fg(color)))
        })
        .collect();

    let count = app.control_log.len();
    frame.render_widget(
        Paragraph::new(lines).block(
            Block::default()
                .title(Span::styled(
                    format!(" Control Log ({count} messages) "),
                    Style::default()
                        .fg(Color::White)
                        .add_modifier(Modifier::BOLD),
                ))
                .borders(Borders::ALL)
                .border_style(Style::default().fg(Color::DarkGray)),
        ),
        area,
    );
}

// ── IMU charts ────────────────────────────────────────────────────────────────

/// IMU axis display names and colours.
const IMU_AXIS_NAMES: [&str; 3] = ["X", "Y", "Z"];
const IMU_AXIS_COLORS: [Color; 3] = [Color::Cyan, Color::Yellow, Color::Magenta];

/// Render six stacked IMU charts: 3 accelerometer axes + 3 gyroscope axes.
fn draw_imu_charts(frame: &mut Frame, area: Rect, app: &App) {
    let [accel_area, gyro_area] = Layout::vertical([
        Constraint::Ratio(1, 2),
        Constraint::Ratio(1, 2),
    ])
    .areas(area);

    let accel_rows = Layout::vertical([
        Constraint::Ratio(1, 3),
        Constraint::Ratio(1, 3),
        Constraint::Ratio(1, 3),
    ])
    .split(accel_area);

    let gyro_rows = Layout::vertical([
        Constraint::Ratio(1, 3),
        Constraint::Ratio(1, 3),
        Constraint::Ratio(1, 3),
    ])
    .split(gyro_area);

    for axis in 0..3 {
        draw_imu_axis(
            frame,
            accel_rows[axis],
            &app.accel_bufs[axis],
            &format!("Accel {}", IMU_AXIS_NAMES[axis]),
            "g",
            IMU_AXIS_COLORS[axis],
        );
        draw_imu_axis(
            frame,
            gyro_rows[axis],
            &app.gyro_bufs[axis],
            &format!("Gyro {}", IMU_AXIS_NAMES[axis]),
            "°/s",
            IMU_AXIS_COLORS[axis],
        );
    }
}

/// Render a single IMU axis chart with auto-scaled Y axis.
fn draw_imu_axis(
    frame: &mut Frame,
    area: Rect,
    buf: &VecDeque<f64>,
    title: &str,
    unit: &str,
    color: Color,
) {
    let (min_v, max_v) = if buf.is_empty() {
        (-1.0_f64, 1.0_f64)
    } else {
        let min = buf.iter().copied().fold(f64::INFINITY, f64::min);
        let max = buf.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        let margin = (max - min).max(0.01) * 0.1;
        (min - margin, max + margin)
    };

    let data: Vec<(f64, f64)> = buf
        .iter()
        .enumerate()
        .map(|(i, &v)| (i as f64 / IMU_HZ, v))
        .collect();

    let datasets = vec![Dataset::default()
        .marker(symbols::Marker::Braille)
        .graph_type(GraphType::Line)
        .style(Style::default().fg(color))
        .data(&data)];

    let cur_val = buf.back().copied().unwrap_or(0.0);
    let title_str = format!(" {title}  {cur_val:+.4} {unit} ");

    let y_mid = (min_v + max_v) / 2.0;
    let y_labels = vec![
        format!("{min_v:.3}"),
        format!("{y_mid:.3}"),
        format!("{max_v:.3}"),
    ];
    let x_labels = vec![
        "0s".to_string(),
        format!("{:.1}s", WINDOW_SECS / 2.0),
        format!("{:.0}s", WINDOW_SECS),
    ];

    let chart = Chart::new(datasets)
        .block(
            Block::default()
                .title(Span::styled(
                    title_str,
                    Style::default().fg(color).add_modifier(Modifier::BOLD),
                ))
                .borders(Borders::ALL)
                .border_style(Style::default().fg(color)),
        )
        .x_axis(
            Axis::default()
                .bounds([0.0, WINDOW_SECS])
                .labels(x_labels)
                .style(Style::default().fg(Color::DarkGray)),
        )
        .y_axis(
            Axis::default()
                .bounds([min_v, max_v])
                .labels(y_labels)
                .style(Style::default().fg(Color::DarkGray)),
        );

    frame.render_widget(chart, area);
}

// ── ACT view ──────────────────────────────────────────────────────────────────

/// Render the ACT (Adaptive Chirplet Transform) results view.
///
/// Shows a per-channel table of extracted chirplets plus timing information.
#[cfg(feature = "act")]
fn draw_act_view(frame: &mut Frame, area: Rect, app: &App) {
    let rows = Layout::vertical([
        Constraint::Ratio(1, 4),
        Constraint::Ratio(1, 4),
        Constraint::Ratio(1, 4),
        Constraint::Ratio(1, 4),
    ])
    .split(area);

    for ch in 0..NUM_CH {
        draw_act_channel(frame, rows[ch], ch, app);
    }
}

/// Render ACT results for a single EEG channel.
#[cfg(feature = "act")]
fn draw_act_channel(frame: &mut Frame, area: Rect, ch: usize, app: &App) {
    let color = COLORS[ch];
    let name = EEG_CHANNEL_NAMES[ch];

    let mut lines: Vec<Line> = Vec::new();

    match &app.act_results[ch] {
        Some(r) => {
            // Header line with channel name, error, and timing
            lines.push(Line::from(vec![
                Span::styled(
                    format!(" {name} "),
                    Style::default().fg(color).add_modifier(Modifier::BOLD),
                ),
                Span::styled(
                    format!(
                        " err={:.4}  ({:.1} ms)",
                        r.error, app.act_last_ms
                    ),
                    Style::default().fg(Color::DarkGray),
                ),
            ]));

            // Column headers
            lines.push(Line::from(Span::styled(
                "   #   fc(Hz)  logDt  chirp   coeff   tc(smp)",
                Style::default()
                    .fg(Color::White)
                    .add_modifier(Modifier::BOLD),
            )));

            // Chirplet rows
            for (i, c) in r.chirplets.iter().enumerate() {
                let dur_ms = 1000.0 * (c.log_dt as f64).exp();
                lines.push(Line::from(Span::styled(
                    format!(
                        "  {:2}  {:6.1}  {:5.2}  {:+6.1}  {:+7.4}  {:6.1}  ({:.1}ms)",
                        i + 1,
                        c.fc,
                        c.log_dt,
                        c.chirp_rate,
                        c.coeff,
                        c.tc,
                        dur_ms,
                    ),
                    Style::default().fg(color),
                )));
            }

            if r.chirplets.is_empty() {
                lines.push(Line::from(Span::styled(
                    "  (no chirplets extracted)",
                    Style::default().fg(Color::DarkGray),
                )));
            }
        }
        None => {
            lines.push(Line::from(vec![
                Span::styled(
                    format!(" {name} "),
                    Style::default().fg(color).add_modifier(Modifier::BOLD),
                ),
                Span::styled(
                    if app.act_engine.is_some() {
                        " waiting for data…"
                    } else {
                        " ACT engine not initialised"
                    },
                    Style::default().fg(Color::DarkGray),
                ),
            ]));
        }
    }

    frame.render_widget(
        Paragraph::new(lines).block(
            Block::default()
                .borders(Borders::ALL)
                .border_style(Style::default().fg(Color::DarkGray)),
        ),
        area,
    );
}

// ── Baseline view ─────────────────────────────────────────────────────────────

/// Render the baseline detector view.
///
/// Layout:
/// - Top: phase indicator + progress bar
/// - Middle: 4-channel columns with per-channel stats
/// - Bottom: summary metrics / final results
#[cfg(feature = "act")]
fn draw_baseline_view(frame: &mut Frame, area: Rect, app: &App) {
    let rows = Layout::vertical([
        Constraint::Length(5),    // status + progress bar
        Constraint::Min(8),      // channel panels
        Constraint::Length(8),    // summary / results
    ])
    .split(area);

    // ── Status + progress bar ────────────────────────────────────────────────
    draw_baseline_status(frame, rows[0], app);

    // ── Channel panels ───────────────────────────────────────────────────────
    let ch_cols = Layout::horizontal([
        Constraint::Ratio(1, 4),
        Constraint::Ratio(1, 4),
        Constraint::Ratio(1, 4),
        Constraint::Ratio(1, 4),
    ])
    .split(rows[1]);

    for ch in 0..NUM_CH {
        draw_baseline_channel(frame, ch_cols[ch], ch, app);
    }

    // ── Summary / results ────────────────────────────────────────────────────
    draw_baseline_summary(frame, rows[2], app);
}

/// Render the baseline status bar: phase label, timer, and progress bar.
#[cfg(feature = "act")]
fn draw_baseline_status(frame: &mut Frame, area: Rect, app: &App) {
    use muse_rs::baseline::BaselinePhase;

    let bl = &app.baseline;
    let mut lines: Vec<Line> = Vec::new();

    // Phase label + timing
    let (phase_label, phase_color) = match &bl.phase {
        BaselinePhase::Idle => ("IDLE — press [b] to start", Color::DarkGray),
        BaselinePhase::Settling => ("SETTLING — hold still, eyes closed…", Color::Yellow),
        BaselinePhase::Recording => ("RECORDING", Color::Green),
        BaselinePhase::Complete => ("COMPLETE", Color::Cyan),
        BaselinePhase::Failed(_) => ("FAILED", Color::Red),
    };

    let timer_text = match &bl.phase {
        BaselinePhase::Settling => {
            format!(
                "  settling {:.0}/{:.0}s",
                bl.elapsed_s(),
                bl.config.settling_duration_s,
            )
        }
        BaselinePhase::Recording => {
            format!(
                "  recording {:.0}/{:.0}s",
                bl.recording_elapsed_s(),
                bl.config.recording_duration_s(),
            )
        }
        BaselinePhase::Complete => "  baseline stored".to_string(),
        BaselinePhase::Failed(reason) => format!("  {reason}"),
        _ => String::new(),
    };

    lines.push(Line::from(vec![
        Span::styled(
            format!(" {phase_label}"),
            Style::default().fg(phase_color).add_modifier(Modifier::BOLD),
        ),
        Span::styled(timer_text, Style::default().fg(Color::White)),
    ]));

    // Progress bar
    if bl.is_running() || bl.phase == BaselinePhase::Complete {
        let bar_w = (area.width as usize).saturating_sub(4);
        let progress = bl.progress();
        let filled = ((progress * bar_w as f64) as usize).min(bar_w);
        let empty = bar_w.saturating_sub(filled);

        // Color the settling portion differently
        let settling_frac = bl.config.settling_duration_s / bl.config.total_duration_s;
        let settling_w = ((settling_frac * bar_w as f64) as usize).min(bar_w);

        let filled_settling = filled.min(settling_w);
        let filled_recording = filled.saturating_sub(settling_w);

        lines.push(Line::from(vec![
            Span::raw("  "),
            Span::styled("█".repeat(filled_settling), Style::default().fg(Color::Yellow)),
            Span::styled("█".repeat(filled_recording), Style::default().fg(Color::Green)),
            Span::styled("░".repeat(empty), Style::default().fg(Color::DarkGray)),
            Span::styled(
                format!(" {:.0}%", progress * 100.0),
                Style::default().fg(Color::White),
            ),
        ]));
    }

    lines.push(Line::from(Span::styled(
        " [b] Start/Stop baseline    [6] Switch to this view",
        Style::default().fg(Color::DarkGray),
    )));

    frame.render_widget(
        Paragraph::new(lines).block(
            Block::default()
                .title(Span::styled(
                    " Baseline Detector ",
                    Style::default()
                        .fg(Color::Cyan)
                        .add_modifier(Modifier::BOLD),
                ))
                .borders(Borders::ALL)
                .border_style(Style::default().fg(Color::Cyan)),
        ),
        area,
    );
}

/// Render per-channel baseline stats in a single column.
#[cfg(feature = "act")]
fn draw_baseline_channel(frame: &mut Frame, area: Rect, ch: usize, app: &App) {
    let color = COLORS[ch];
    let name = EEG_CHANNEL_NAMES[ch];
    let bl = &app.baseline;
    let acc = &bl.channels[ch];

    let mut lines: Vec<Line> = Vec::new();

    // Current live alpha (from FFT snapshot)
    let live_alpha = app
        .fft_snapshot
        .as_ref()
        .map(|s| s.channels[ch].alpha_power)
        .unwrap_or(0.0);

    lines.push(Line::from(vec![
        Span::styled(
            format!(" α now: "),
            Style::default().fg(Color::White),
        ),
        Span::styled(
            format!("{:.4}", live_alpha),
            Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD),
        ),
    ]));

    // Baseline mean alpha (accumulated)
    if acc.clean_snapshots > 0 {
        let mean_a = acc.mean_alpha();
        let cv = acc.alpha_cv();
        let cv_color = if cv <= 0.30 {
            Color::Green
        } else if cv <= 0.50 {
            Color::Yellow
        } else {
            Color::Red
        };

        lines.push(Line::from(vec![
            Span::styled(" α mean: ", Style::default().fg(Color::White)),
            Span::styled(
                format!("{mean_a:.4}"),
                Style::default().fg(Color::Green).add_modifier(Modifier::BOLD),
            ),
        ]));

        lines.push(Line::from(vec![
            Span::styled(" CV: ", Style::default().fg(Color::White)),
            Span::styled(
                format!("{:.0}%", cv * 100.0),
                Style::default().fg(cv_color).add_modifier(Modifier::BOLD),
            ),
        ]));

        // Clean ratio
        let clean_pct = (1.0 - acc.artifact_ratio()) * 100.0;
        let clean_color = if clean_pct >= 70.0 {
            Color::Green
        } else if clean_pct >= 50.0 {
            Color::Yellow
        } else {
            Color::Red
        };
        lines.push(Line::from(vec![
            Span::styled(" Clean: ", Style::default().fg(Color::White)),
            Span::styled(
                format!("{clean_pct:.0}%"),
                Style::default().fg(clean_color),
            ),
            Span::styled(
                format!(" ({}/{})", acc.clean_snapshots, acc.total_snapshots),
                Style::default().fg(Color::DarkGray),
            ),
        ]));

        // Alpha power sparkline from accumulated values
        if acc.alpha_powers.len() > 2 {
            let max_w = (area.width as usize).saturating_sub(4);
            let spark_chars = ['▁', '▂', '▃', '▄', '▅', '▆', '▇', '█'];
            let values: Vec<f32> = acc
                .alpha_powers
                .iter()
                .rev()
                .take(max_w)
                .copied()
                .collect::<Vec<_>>()
                .into_iter()
                .rev()
                .collect();
            let max_val = values.iter().copied().fold(0.0_f32, f32::max).max(1e-10);
            let spark: String = values
                .iter()
                .map(|&v| {
                    let idx = ((v / max_val) * 7.0).round() as usize;
                    spark_chars[idx.min(7)]
                })
                .collect();
            lines.push(Line::from(vec![
                Span::raw(" "),
                Span::styled(spark, Style::default().fg(Color::Green)),
            ]));
        }
    } else if bl.is_running() {
        lines.push(Line::from(Span::styled(
            " accumulating…",
            Style::default().fg(Color::DarkGray),
        )));
    } else {
        lines.push(Line::from(Span::styled(
            " no data",
            Style::default().fg(Color::DarkGray),
        )));
    }

    // Contact quality indicator
    let cq = app.contact_quality[ch];
    let (cq_label, cq_color) = match cq {
        ContactQuality::Good => ("●", Color::Green),
        ContactQuality::Ok => ("●", Color::Yellow),
        ContactQuality::Bad => ("●", Color::Red),
        ContactQuality::NoContact => ("○", Color::DarkGray),
    };
    lines.push(Line::from(vec![
        Span::styled(" Contact: ", Style::default().fg(Color::DarkGray)),
        Span::styled(cq_label, Style::default().fg(cq_color)),
    ]));

    frame.render_widget(
        Paragraph::new(lines).block(
            Block::default()
                .title(Span::styled(
                    format!(" {name} "),
                    Style::default().fg(color).add_modifier(Modifier::BOLD),
                ))
                .borders(Borders::ALL)
                .border_style(Style::default().fg(color)),
        ),
        area,
    );
}

/// Render the baseline summary: asymmetry, noise, overall quality, final result.
#[cfg(feature = "act")]
fn draw_baseline_summary(frame: &mut Frame, area: Rect, app: &App) {
    use muse_rs::baseline::BaselinePhase;

    let bl = &app.baseline;
    let mut lines: Vec<Line> = Vec::new();

    // Show final results if complete
    if let Some(result) = &bl.result {
        let asym = result.alpha_asymmetry;
        let asym_color = if asym > 0.1 {
            Color::Cyan
        } else if asym < -0.1 {
            Color::Magenta
        } else {
            Color::White
        };
        let asym_label = if asym > 0.1 {
            "R>L"
        } else if asym < -0.1 {
            "L>R"
        } else {
            "balanced"
        };

        lines.push(Line::from(vec![
            Span::styled(
                " Frontal asymmetry: ",
                Style::default().fg(Color::White),
            ),
            Span::styled(
                format!("{asym:+.4} ({asym_label})"),
                Style::default().fg(asym_color).add_modifier(Modifier::BOLD),
            ),
        ]));

        lines.push(Line::from(vec![
            Span::styled(" Noise δ: ", Style::default().fg(Color::White)),
            Span::styled(
                format!("{:.6}", result.noise_delta),
                Style::default().fg(Color::Yellow),
            ),
            Span::styled("    θ: ", Style::default().fg(Color::White)),
            Span::styled(
                format!("{:.6}", result.noise_theta),
                Style::default().fg(Color::Yellow),
            ),
        ]));

        let art_pct = result.overall_artifact_ratio * 100.0;
        let art_color = if art_pct <= 30.0 { Color::Green } else { Color::Red };
        lines.push(Line::from(vec![
            Span::styled(" Artifact ratio: ", Style::default().fg(Color::White)),
            Span::styled(
                format!("{art_pct:.0}%"),
                Style::default().fg(art_color).add_modifier(Modifier::BOLD),
            ),
            Span::styled(
                format!(
                    "    Clean snapshots: {}    Duration: {:.0}s",
                    result.total_clean_snapshots, result.recording_duration_s
                ),
                Style::default().fg(Color::DarkGray),
            ),
        ]));

        lines.push(Line::from(vec![
            Span::styled(" α per channel: ", Style::default().fg(Color::White)),
            Span::styled(
                format!(
                    "TP9={:.4}  AF7={:.4}  AF8={:.4}  TP10={:.4}",
                    result.channel_alpha[0],
                    result.channel_alpha[1],
                    result.channel_alpha[2],
                    result.channel_alpha[3],
                ),
                Style::default().fg(Color::Green),
            ),
        ]));

        let quality = if result.overall_artifact_ratio <= 0.30
            && result.channel_alpha_cv.iter().all(|&cv| cv <= 0.30)
        {
            ("GOOD", Color::Green)
        } else if result.overall_artifact_ratio <= 0.30 {
            ("FAIR", Color::Yellow)
        } else {
            ("POOR", Color::Red)
        };

        lines.push(Line::from(vec![
            Span::styled(" Quality: ", Style::default().fg(Color::White)),
            Span::styled(
                quality.0,
                Style::default().fg(quality.1).add_modifier(Modifier::BOLD),
            ),
        ]));
    } else if bl.is_running() {
        // Show live running stats
        let worst_art = bl
            .channels
            .iter()
            .map(|ch| ch.artifact_ratio())
            .fold(0.0_f64, f64::max);
        let total_clean: usize = bl.channels.iter().map(|ch| ch.clean_snapshots).sum();

        lines.push(Line::from(vec![
            Span::styled(" Worst artifact ratio: ", Style::default().fg(Color::White)),
            Span::styled(
                format!("{:.0}%", worst_art * 100.0),
                Style::default().fg(
                    if worst_art <= 0.30 { Color::Green } else { Color::Red }
                ),
            ),
            Span::styled(
                format!("    Total clean snapshots: {total_clean}"),
                Style::default().fg(Color::DarkGray),
            ),
        ]));

        // Live asymmetry from FFT
        if let Some(fft) = &app.fft_snapshot {
            let af7 = fft.channels[1].alpha_power as f64;
            let af8 = fft.channels[2].alpha_power as f64;
            let asym = if (af7 + af8) > 0.0 {
                (af8 - af7) / (af8 + af7)
            } else {
                0.0
            };
            lines.push(Line::from(vec![
                Span::styled(" Live asymmetry: ", Style::default().fg(Color::White)),
                Span::styled(
                    format!("{asym:+.4}"),
                    Style::default().fg(Color::Yellow),
                ),
            ]));
        }
    } else {
        // Idle
        lines.push(Line::from(Span::styled(
            " Press [b] to begin baseline collection (60s: 15s settling + 45s recording)",
            Style::default().fg(Color::DarkGray),
        )));
        lines.push(Line::from(Span::styled(
            " Ensure good electrode contact. Close eyes when ready.",
            Style::default().fg(Color::DarkGray),
        )));
    }

    let border_color = match &bl.phase {
        BaselinePhase::Complete => Color::Green,
        BaselinePhase::Failed(_) => Color::Red,
        BaselinePhase::Recording => Color::Green,
        BaselinePhase::Settling => Color::Yellow,
        BaselinePhase::Idle => Color::DarkGray,
    };

    frame.render_widget(
        Paragraph::new(lines).block(
            Block::default()
                .title(Span::styled(
                    " Summary ",
                    Style::default()
                        .fg(border_color)
                        .add_modifier(Modifier::BOLD),
                ))
                .borders(Borders::ALL)
                .border_style(Style::default().fg(border_color)),
        ),
        area,
    );
}

// ── Absorption view ───────────────────────────────────────────────────────────

/// Render the absorption measurement view.
///
/// Layout:
/// - Top: status bar + progress
/// - Middle-left: per-channel alpha comparison (baseline vs immersion)
/// - Middle-right: big absorption % gauge + classification
/// - Bottom: controls + threshold info
#[cfg(feature = "act")]
fn draw_absorption_view(frame: &mut Frame, area: Rect, app: &App) {
    use muse_rs::absorption::AbsorptionPhase;

    // If no baseline and idle, show a message
    if app.absorption.phase == AbsorptionPhase::NeedBaseline
        || (app.absorption.phase == AbsorptionPhase::Idle && app.baseline.result.is_none())
    {
        let lines = vec![
            Line::from(""),
            Line::from(Span::styled(
                "  ⚠  Collect baseline first (press [b] on baseline view)",
                Style::default()
                    .fg(Color::Yellow)
                    .add_modifier(Modifier::BOLD),
            )),
            Line::from(""),
            Line::from(Span::styled(
                "  The absorption measurement requires a completed baseline to compare against.",
                Style::default().fg(Color::DarkGray),
            )),
            Line::from(Span::styled(
                "  Go to [6] Baseline view and press [b] to start a 60-second baseline collection.",
                Style::default().fg(Color::DarkGray),
            )),
        ];
        frame.render_widget(
            Paragraph::new(lines).block(
                Block::default()
                    .title(Span::styled(
                        " Absorption — Dimension 1 ",
                        Style::default()
                            .fg(Color::Yellow)
                            .add_modifier(Modifier::BOLD),
                    ))
                    .borders(Borders::ALL)
                    .border_style(Style::default().fg(Color::Yellow)),
            ),
            area,
        );
        return;
    }

    let rows = Layout::vertical([
        Constraint::Length(5),   // status + progress
        Constraint::Min(10),    // main content
        Constraint::Length(5),   // controls
    ])
    .split(area);

    draw_absorption_status(frame, rows[0], app);
    draw_absorption_main(frame, rows[1], app);
    draw_absorption_controls(frame, rows[2], app);
}

/// Render absorption status bar: phase, timer, progress.
#[cfg(feature = "act")]
fn draw_absorption_status(frame: &mut Frame, area: Rect, app: &App) {
    use muse_rs::absorption::AbsorptionPhase;

    let ab = &app.absorption;
    let mut lines: Vec<Line> = Vec::new();

    let (phase_label, phase_color) = match &ab.phase {
        AbsorptionPhase::Idle => ("IDLE — press [m] to measure", Color::DarkGray),
        AbsorptionPhase::NeedBaseline => ("NEED BASELINE", Color::Yellow),
        AbsorptionPhase::Settling => ("SETTLING — hold still…", Color::Yellow),
        AbsorptionPhase::Measuring => ("MEASURING", Color::Magenta),
        AbsorptionPhase::Complete => ("COMPLETE", Color::Cyan),
    };

    let timer_text = match &ab.phase {
        AbsorptionPhase::Settling => {
            format!(
                "  settling {:.0}/{:.0}s",
                ab.elapsed_s(),
                ab.config.settling_duration_s,
            )
        }
        AbsorptionPhase::Measuring => {
            format!(
                "  measuring {:.0}/{:.0}s",
                ab.measuring_elapsed_s(),
                ab.config.recording_duration_s(),
            )
        }
        AbsorptionPhase::Complete => "  result ready".to_string(),
        _ => String::new(),
    };

    lines.push(Line::from(vec![
        Span::styled(
            format!(" {phase_label}"),
            Style::default().fg(phase_color).add_modifier(Modifier::BOLD),
        ),
        Span::styled(timer_text, Style::default().fg(Color::White)),
    ]));

    // Progress bar
    if ab.is_running() || ab.phase == AbsorptionPhase::Complete {
        let bar_w = (area.width as usize).saturating_sub(4);
        let progress = ab.progress();
        let filled = ((progress * bar_w as f64) as usize).min(bar_w);
        let empty = bar_w.saturating_sub(filled);

        let settling_frac = ab.config.settling_duration_s / ab.config.measurement_duration_s;
        let settling_w = ((settling_frac * bar_w as f64) as usize).min(bar_w);
        let filled_settling = filled.min(settling_w);
        let filled_measuring = filled.saturating_sub(settling_w);

        lines.push(Line::from(vec![
            Span::raw("  "),
            Span::styled(
                "█".repeat(filled_settling),
                Style::default().fg(Color::Yellow),
            ),
            Span::styled(
                "█".repeat(filled_measuring),
                Style::default().fg(Color::Magenta),
            ),
            Span::styled("░".repeat(empty), Style::default().fg(Color::DarkGray)),
            Span::styled(
                format!(" {:.0}%", progress * 100.0),
                Style::default().fg(Color::White),
            ),
        ]));
    }

    frame.render_widget(
        Paragraph::new(lines).block(
            Block::default()
                .title(Span::styled(
                    " Absorption — Dimension 1 ",
                    Style::default()
                        .fg(Color::Magenta)
                        .add_modifier(Modifier::BOLD),
                ))
                .borders(Borders::ALL)
                .border_style(Style::default().fg(Color::Magenta)),
        ),
        area,
    );
}

/// Render the main absorption content: per-channel comparison + big gauge.
#[cfg(feature = "act")]
fn draw_absorption_main(frame: &mut Frame, area: Rect, app: &App) {
    let cols = Layout::horizontal([
        Constraint::Ratio(1, 2), // per-channel comparison
        Constraint::Ratio(1, 2), // big gauge + classification
    ])
    .split(area);

    draw_absorption_channels(frame, cols[0], app);
    draw_absorption_gauge(frame, cols[1], app);
}

/// Render per-channel alpha comparison: baseline vs immersion.
#[cfg(feature = "act")]
fn draw_absorption_channels(frame: &mut Frame, area: Rect, app: &App) {
    let ab = &app.absorption;
    let mut lines: Vec<Line> = Vec::new();

    let bl_alpha = ab.baseline_alpha.as_ref().or_else(|| {
        app.baseline.result.as_ref().map(|r| &r.channel_alpha)
    });

    let imm_alpha = ab.live_channel_alpha();
    let has_data = ab.is_running()
        || matches!(ab.phase, muse_rs::absorption::AbsorptionPhase::Complete);

    // Header
    lines.push(Line::from(vec![
        Span::styled(
            " Channel    Baseline    Immersion   Change",
            Style::default().fg(Color::DarkGray),
        ),
    ]));
    lines.push(Line::from(Span::styled(
        " ─────────────────────────────────────────",
        Style::default().fg(Color::DarkGray),
    )));

    for ch in 0..NUM_CH {
        let name = EEG_CHANNEL_NAMES[ch];
        let color = COLORS[ch];

        let bl_val = bl_alpha.map(|a| a[ch]).unwrap_or(0.0);
        let imm_val = if has_data { imm_alpha[ch] } else { 0.0 };

        let change_pct = if bl_val > 0.0 && has_data {
            ((imm_val - bl_val) / bl_val) * 100.0
        } else {
            0.0
        };

        let change_color = if change_pct >= 25.0 {
            Color::Green
        } else if change_pct >= 10.0 {
            Color::Yellow
        } else {
            Color::White
        };

        lines.push(Line::from(vec![
            Span::styled(
                format!(" {name:<8}"),
                Style::default().fg(color).add_modifier(Modifier::BOLD),
            ),
            Span::styled(format!("  {bl_val:>8.4}"), Style::default().fg(Color::Cyan)),
            if has_data {
                Span::styled(
                    format!("  {imm_val:>8.4}"),
                    Style::default().fg(Color::Magenta),
                )
            } else {
                Span::styled("       —  ", Style::default().fg(Color::DarkGray))
            },
            if has_data {
                Span::styled(
                    format!("  {:>+6.1}%", change_pct),
                    Style::default().fg(change_color).add_modifier(Modifier::BOLD),
                )
            } else {
                Span::styled("      — ", Style::default().fg(Color::DarkGray))
            },
        ]));
    }

    // Artifact info
    if has_data {
        lines.push(Line::from(""));
        let art = ab.live_channel_artifact_ratio();
        let worst = art.iter().fold(0.0_f64, |a, &b| a.max(b));
        let art_color = if worst <= 0.30 { Color::Green } else { Color::Red };
        lines.push(Line::from(vec![
            Span::styled(" Artifact ratio: ", Style::default().fg(Color::DarkGray)),
            Span::styled(
                format!("{:.0}%", worst * 100.0),
                Style::default().fg(art_color),
            ),
        ]));
    }

    frame.render_widget(
        Paragraph::new(lines).block(
            Block::default()
                .title(Span::styled(
                    " Per-Channel Alpha (8-12 Hz) ",
                    Style::default().fg(Color::White).add_modifier(Modifier::BOLD),
                ))
                .borders(Borders::ALL)
                .border_style(Style::default().fg(Color::DarkGray)),
        ),
        area,
    );
}

/// Render the big absorption gauge and classification label.
#[cfg(feature = "act")]
fn draw_absorption_gauge(frame: &mut Frame, area: Rect, app: &App) {
    use muse_rs::absorption::{AbsorptionLabel, AbsorptionPhase};

    let ab = &app.absorption;
    let mut lines: Vec<Line> = Vec::new();

    let threshold = ab.config.threshold_pct;

    // Show final result or live reading
    let (absorption_pct, label, is_final) = if let Some(result) = &ab.result {
        (Some(result.absorption_pct), Some(result.label), true)
    } else if ab.is_running() {
        (ab.live_absorption_pct(), ab.live_label(), false)
    } else {
        (None, None, false)
    };

    // Big absorption percentage
    lines.push(Line::from(""));
    match absorption_pct {
        Some(pct) => {
            let pct_color = if pct >= threshold {
                Color::Green
            } else if pct >= threshold * 0.7 {
                Color::Yellow
            } else {
                Color::White
            };

            lines.push(Line::from(vec![
                Span::styled(
                    "  Absorption: ",
                    Style::default().fg(Color::White),
                ),
                Span::styled(
                    format!("{pct:>+6.1}%"),
                    Style::default().fg(pct_color).add_modifier(Modifier::BOLD),
                ),
                if is_final {
                    Span::styled(" (final)", Style::default().fg(Color::Cyan))
                } else {
                    Span::styled(" (live)", Style::default().fg(Color::DarkGray))
                },
            ]));

            // Visual gauge
            let gauge_w = (area.width as usize).saturating_sub(6);
            // Map absorption to gauge: -20% to +50% range
            let gauge_min = -20.0_f64;
            let gauge_max = 50.0_f64;
            let gauge_range = gauge_max - gauge_min;
            let gauge_pos =
                (((pct - gauge_min) / gauge_range) * gauge_w as f64).clamp(0.0, gauge_w as f64)
                    as usize;
            let thresh_pos =
                (((threshold - gauge_min) / gauge_range) * gauge_w as f64).clamp(0.0, gauge_w as f64)
                    as usize;

            let mut gauge_chars: Vec<Span> = Vec::new();
            gauge_chars.push(Span::raw("  "));
            for i in 0..gauge_w {
                let ch = if i == thresh_pos { '|' } else if i <= gauge_pos { '█' } else { '░' };
                let color = if i == thresh_pos {
                    Color::Yellow
                } else if i <= gauge_pos {
                    pct_color
                } else {
                    Color::DarkGray
                };
                gauge_chars.push(Span::styled(
                    ch.to_string(),
                    Style::default().fg(color),
                ));
            }
            lines.push(Line::from(gauge_chars));

            // Threshold marker label
            let pad = thresh_pos + 2;
            lines.push(Line::from(vec![
                Span::raw(" ".repeat(pad.min(gauge_w))),
                Span::styled(
                    format!("↑ {threshold:.0}% threshold"),
                    Style::default().fg(Color::Yellow),
                ),
            ]));
        }
        None => {
            lines.push(Line::from(Span::styled(
                "  Absorption: —",
                Style::default().fg(Color::DarkGray),
            )));
        }
    }

    // Big classification label
    lines.push(Line::from(""));
    match label {
        Some(l) => {
            let (label_str, label_color) = match l {
                AbsorptionLabel::Deep => ("   ██  DEEP  ██", Color::Green),
                AbsorptionLabel::Surface => ("   ▒▒  SURFACE  ▒▒", Color::Yellow),
            };
            lines.push(Line::from(Span::styled(
                label_str,
                Style::default().fg(label_color).add_modifier(Modifier::BOLD),
            )));
        }
        None => {
            lines.push(Line::from(Span::styled(
                "   …",
                Style::default().fg(Color::DarkGray),
            )));
        }
    }

    // Baseline reference
    if let Some(bl) = &app.baseline.result {
        let bl_mean = (bl.channel_alpha[1] + bl.channel_alpha[2]) / 2.0;
        lines.push(Line::from(""));
        lines.push(Line::from(vec![
            Span::styled(
                "  Baseline α (AF7+AF8)/2: ",
                Style::default().fg(Color::DarkGray),
            ),
            Span::styled(
                format!("{bl_mean:.4}"),
                Style::default().fg(Color::Cyan),
            ),
        ]));
    }

    let border_color = match &ab.phase {
        AbsorptionPhase::Complete => {
            if ab.result.as_ref().map(|r| r.label) == Some(AbsorptionLabel::Deep) {
                Color::Green
            } else {
                Color::Yellow
            }
        }
        AbsorptionPhase::Measuring => Color::Magenta,
        _ => Color::DarkGray,
    };

    frame.render_widget(
        Paragraph::new(lines).block(
            Block::default()
                .title(Span::styled(
                    " Classification ",
                    Style::default()
                        .fg(border_color)
                        .add_modifier(Modifier::BOLD),
                ))
                .borders(Borders::ALL)
                .border_style(Style::default().fg(border_color)),
        ),
        area,
    );
}

/// Render absorption controls: threshold, start/stop, key hints.
#[cfg(feature = "act")]
fn draw_absorption_controls(frame: &mut Frame, area: Rect, app: &App) {
    let ab = &app.absorption;
    let threshold = ab.config.threshold_pct;

    let mut lines: Vec<Line> = Vec::new();

    lines.push(Line::from(vec![
        Span::styled(" Threshold: ", Style::default().fg(Color::White)),
        Span::styled(
            format!("{threshold:.0}%"),
            Style::default()
                .fg(Color::Yellow)
                .add_modifier(Modifier::BOLD),
        ),
        Span::styled(
            "    [t] −5%    [T] +5%",
            Style::default().fg(Color::DarkGray),
        ),
    ]));

    lines.push(Line::from(vec![
        Span::styled(
            " [m] Start/Stop measurement    [8] Switch to this view",
            Style::default().fg(Color::DarkGray),
        ),
    ]));

    frame.render_widget(
        Paragraph::new(lines).block(
            Block::default()
                .borders(Borders::ALL)
                .border_style(Style::default().fg(Color::DarkGray)),
        ),
        area,
    );
}

// ── Approach view ─────────────────────────────────────────────────────────────

/// Render the approach/withdraw (FAA) measurement view.
///
/// Layout:
/// - Top: status bar + progress
/// - Middle-left: per-channel alpha powers (AF7 vs AF8)
/// - Middle-right: asymmetry gauge + classification label
/// - Bottom: controls + deadband info
#[cfg(feature = "act")]
fn draw_approach_view(frame: &mut Frame, area: Rect, app: &App) {
    let rows = Layout::vertical([
        Constraint::Length(5),  // status + progress
        Constraint::Min(10),   // main content
        Constraint::Length(5),  // controls
    ])
    .split(area);

    draw_approach_status(frame, rows[0], app);
    draw_approach_main(frame, rows[1], app);
    draw_approach_controls(frame, rows[2], app);
}

/// Render approach status bar: phase, timer, progress.
#[cfg(feature = "act")]
fn draw_approach_status(frame: &mut Frame, area: Rect, app: &App) {
    use muse_rs::approach::ApproachPhase;

    let ap = &app.approach;
    let mut lines: Vec<Line> = Vec::new();

    let (phase_label, phase_color) = match &ap.phase {
        ApproachPhase::Idle => ("IDLE — press [g] to measure", Color::DarkGray),
        ApproachPhase::Settling => ("SETTLING — hold still…", Color::Yellow),
        ApproachPhase::Measuring => ("MEASURING", Color::Magenta),
        ApproachPhase::Complete => ("COMPLETE", Color::Cyan),
    };

    let timer_text = match &ap.phase {
        ApproachPhase::Settling => {
            format!(
                "  settling {:.0}/{:.0}s",
                ap.elapsed_s(),
                ap.config.settling_duration_s,
            )
        }
        ApproachPhase::Measuring => {
            format!(
                "  measuring {:.0}/{:.0}s",
                ap.measuring_elapsed_s(),
                ap.config.recording_duration_s(),
            )
        }
        ApproachPhase::Complete => "  result ready".to_string(),
        _ => String::new(),
    };

    lines.push(Line::from(vec![
        Span::styled(
            format!(" {phase_label}"),
            Style::default().fg(phase_color).add_modifier(Modifier::BOLD),
        ),
        Span::styled(timer_text, Style::default().fg(Color::White)),
    ]));

    // Progress bar
    if ap.is_running() || ap.phase == ApproachPhase::Complete {
        let bar_w = (area.width as usize).saturating_sub(4);
        let progress = ap.progress();
        let filled = ((progress * bar_w as f64) as usize).min(bar_w);
        let empty = bar_w.saturating_sub(filled);

        let settling_frac = ap.config.settling_duration_s / ap.config.measurement_duration_s;
        let settling_w = ((settling_frac * bar_w as f64) as usize).min(bar_w);
        let filled_settling = filled.min(settling_w);
        let filled_measuring = filled.saturating_sub(settling_w);

        lines.push(Line::from(vec![
            Span::raw("  "),
            Span::styled(
                "█".repeat(filled_settling),
                Style::default().fg(Color::Yellow),
            ),
            Span::styled(
                "█".repeat(filled_measuring),
                Style::default().fg(Color::Magenta),
            ),
            Span::styled("░".repeat(empty), Style::default().fg(Color::DarkGray)),
            Span::styled(
                format!(" {:.0}%", progress * 100.0),
                Style::default().fg(Color::White),
            ),
        ]));
    }

    frame.render_widget(
        Paragraph::new(lines).block(
            Block::default()
                .title(Span::styled(
                    " Approach/Withdraw — Dimension 4 (FAA) ",
                    Style::default()
                        .fg(Color::Magenta)
                        .add_modifier(Modifier::BOLD),
                ))
                .borders(Borders::ALL)
                .border_style(Style::default().fg(Color::Magenta)),
        ),
        area,
    );
}

/// Render the main approach content: channel alphas + asymmetry gauge.
#[cfg(feature = "act")]
fn draw_approach_main(frame: &mut Frame, area: Rect, app: &App) {
    let cols = Layout::horizontal([
        Constraint::Ratio(1, 2), // per-channel alpha
        Constraint::Ratio(1, 2), // asymmetry gauge + label
    ])
    .split(area);

    draw_approach_channels(frame, cols[0], app);
    draw_approach_gauge(frame, cols[1], app);
}

/// Render per-channel alpha powers (AF7 left, AF8 right).
#[cfg(feature = "act")]
fn draw_approach_channels(frame: &mut Frame, area: Rect, app: &App) {
    use muse_rs::approach::ApproachPhase;

    let ap = &app.approach;
    let mut lines: Vec<Line> = Vec::new();

    let has_data = ap.is_running()
        || matches!(ap.phase, ApproachPhase::Complete);

    // Header
    lines.push(Line::from(vec![Span::styled(
        " Channel    Alpha (8-12 Hz)    Artifact%",
        Style::default().fg(Color::DarkGray),
    )]));
    lines.push(Line::from(Span::styled(
        " ─────────────────────────────────────────",
        Style::default().fg(Color::DarkGray),
    )));

    let channel_indices = [
        (muse_rs::alpha::CH_AF7, "AF7 (L)", Color::Green),
        (muse_rs::alpha::CH_AF8, "AF8 (R)", Color::Blue),
    ];

    for &(ch, name, color) in &channel_indices {
        let alpha_val = if has_data {
            ap.channels[ch].mean_alpha()
        } else {
            0.0
        };
        let art = ap.channels[ch].artifact_ratio();

        let art_color = if art <= 0.30 { Color::Green } else { Color::Red };

        lines.push(Line::from(vec![
            Span::styled(
                format!(" {name:<10}"),
                Style::default().fg(color).add_modifier(Modifier::BOLD),
            ),
            if has_data && ap.channels[ch].clean_snapshots > 0 {
                Span::styled(
                    format!("{alpha_val:>12.4}"),
                    Style::default().fg(Color::White),
                )
            } else {
                Span::styled("         —  ", Style::default().fg(Color::DarkGray))
            },
            Span::raw("       "),
            if has_data && ap.channels[ch].total_snapshots > 0 {
                Span::styled(
                    format!("{:.0}%", art * 100.0),
                    Style::default().fg(art_color),
                )
            } else {
                Span::styled("—", Style::default().fg(Color::DarkGray))
            },
        ]));
    }

    // Show live asymmetry score
    lines.push(Line::from(""));
    if let Some(asym) = ap.live_asymmetry() {
        let asym_color = if asym > 0.0 { Color::Green } else { Color::Red };
        lines.push(Line::from(vec![
            Span::styled(" Asymmetry:  ", Style::default().fg(Color::DarkGray)),
            Span::styled(
                format!("{asym:>+.4}"),
                Style::default().fg(asym_color).add_modifier(Modifier::BOLD),
            ),
        ]));
        lines.push(Line::from(vec![
            Span::styled(
                "   (R−L)/(R+L)   +positive = Lean, −negative = Hold",
                Style::default().fg(Color::DarkGray),
            ),
        ]));
    } else {
        lines.push(Line::from(Span::styled(
            " Asymmetry:  —",
            Style::default().fg(Color::DarkGray),
        )));
    }

    // Show final result details
    if let Some(result) = &ap.result {
        lines.push(Line::from(""));
        lines.push(Line::from(vec![
            Span::styled(" Clean snapshots: ", Style::default().fg(Color::DarkGray)),
            Span::styled(
                format!("{}", result.total_clean_snapshots),
                Style::default().fg(Color::White),
            ),
            Span::styled(
                format!("   ({:.1}s recording)", result.recording_duration_s),
                Style::default().fg(Color::DarkGray),
            ),
        ]));
    }

    frame.render_widget(
        Paragraph::new(lines).block(
            Block::default()
                .title(Span::styled(
                    " Frontal Alpha Power ",
                    Style::default().fg(Color::White).add_modifier(Modifier::BOLD),
                ))
                .borders(Borders::ALL)
                .border_style(Style::default().fg(Color::DarkGray)),
        ),
        area,
    );
}

/// Render the asymmetry gauge and classification label.
#[cfg(feature = "act")]
fn draw_approach_gauge(frame: &mut Frame, area: Rect, app: &App) {
    use muse_rs::approach::{ApproachLabel, ApproachConfidence, ApproachPhase};

    let ap = &app.approach;
    let mut lines: Vec<Line> = Vec::new();
    let deadband = ap.config.deadband;

    // Show final result or live reading
    let (asymmetry, classification, is_final) = if let Some(result) = &ap.result {
        (
            Some(result.asymmetry),
            Some((result.label, result.confidence)),
            true,
        )
    } else if ap.is_running() {
        (ap.live_asymmetry(), ap.live_classification(), false)
    } else {
        (None, None, false)
    };

    // Asymmetry score display
    lines.push(Line::from(""));
    match asymmetry {
        Some(asym) => {
            let asym_color = if asym.abs() > deadband {
                if asym > 0.0 { Color::Green } else { Color::Red }
            } else {
                Color::Yellow
            };

            lines.push(Line::from(vec![
                Span::styled("  Asymmetry: ", Style::default().fg(Color::White)),
                Span::styled(
                    format!("{asym:>+.4}"),
                    Style::default().fg(asym_color).add_modifier(Modifier::BOLD),
                ),
                if is_final {
                    Span::styled(" (final)", Style::default().fg(Color::Cyan))
                } else {
                    Span::styled(" (live)", Style::default().fg(Color::DarkGray))
                },
            ]));

            // Visual gauge: range [-1.0, +1.0], centered at 0
            let gauge_w = (area.width as usize).saturating_sub(6);
            let center = gauge_w / 2;
            let deadband_left = center - ((deadband * center as f64) as usize).min(center);
            let deadband_right = center + ((deadband * center as f64) as usize).min(center);
            let score_pos = ((asym + 1.0) / 2.0 * gauge_w as f64).clamp(0.0, gauge_w as f64 - 1.0) as usize;

            let mut gauge_chars: Vec<Span> = Vec::new();
            gauge_chars.push(Span::raw("  "));
            for i in 0..gauge_w {
                let is_deadband_edge = i == deadband_left || i == deadband_right;
                let is_center = i == center;
                let is_score = i == score_pos;

                let ch = if is_score {
                    '◆'
                } else if is_center {
                    '│'
                } else if is_deadband_edge {
                    '┊'
                } else {
                    '░'
                };

                let color = if is_score {
                    asym_color
                } else if is_center {
                    Color::White
                } else if is_deadband_edge {
                    Color::Yellow
                } else if i > deadband_left && i < deadband_right {
                    Color::DarkGray
                } else if i < center {
                    Color::Red
                } else {
                    Color::Green
                };

                gauge_chars.push(Span::styled(
                    ch.to_string(),
                    Style::default().fg(color),
                ));
            }
            lines.push(Line::from(gauge_chars));

            // Labels under gauge
            lines.push(Line::from(vec![
                Span::raw("  "),
                Span::styled("Hold (withdraw)", Style::default().fg(Color::Red)),
                Span::raw(" ".repeat(gauge_w.saturating_sub(30))),
                Span::styled("Lean (approach)", Style::default().fg(Color::Green)),
            ]));
        }
        None => {
            lines.push(Line::from(Span::styled(
                "  Asymmetry: —",
                Style::default().fg(Color::DarkGray),
            )));
        }
    }

    // Big classification label
    lines.push(Line::from(""));
    match classification {
        Some((label, confidence)) => {
            let (label_str, label_color) = match label {
                ApproachLabel::Lean => ("   ██  LEAN  ██", Color::Green),
                ApproachLabel::Hold => ("   ██  HOLD  ██", Color::Red),
            };
            let conf_str = match confidence {
                ApproachConfidence::High => " (high confidence)",
                ApproachConfidence::Low => " (low confidence)",
            };
            let conf_color = match confidence {
                ApproachConfidence::High => Color::White,
                ApproachConfidence::Low => Color::Yellow,
            };

            lines.push(Line::from(vec![
                Span::styled(
                    label_str,
                    Style::default().fg(label_color).add_modifier(Modifier::BOLD),
                ),
                Span::styled(conf_str, Style::default().fg(conf_color)),
            ]));
        }
        None => {
            lines.push(Line::from(Span::styled(
                "   …",
                Style::default().fg(Color::DarkGray),
            )));
        }
    }

    let border_color = match &ap.phase {
        ApproachPhase::Complete => {
            if ap.result.as_ref().map(|r| r.label) == Some(ApproachLabel::Lean) {
                Color::Green
            } else {
                Color::Red
            }
        }
        ApproachPhase::Measuring => Color::Magenta,
        _ => Color::DarkGray,
    };

    frame.render_widget(
        Paragraph::new(lines).block(
            Block::default()
                .title(Span::styled(
                    " Classification ",
                    Style::default()
                        .fg(border_color)
                        .add_modifier(Modifier::BOLD),
                ))
                .borders(Borders::ALL)
                .border_style(Style::default().fg(border_color)),
        ),
        area,
    );
}

/// Render approach controls: deadband, start/stop, key hints.
#[cfg(feature = "act")]
fn draw_approach_controls(frame: &mut Frame, area: Rect, app: &App) {
    let ap = &app.approach;
    let deadband = ap.config.deadband;

    let mut lines: Vec<Line> = Vec::new();

    lines.push(Line::from(vec![
        Span::styled(" Deadband: ", Style::default().fg(Color::White)),
        Span::styled(
            format!("±{deadband:.2}"),
            Style::default()
                .fg(Color::Yellow)
                .add_modifier(Modifier::BOLD),
        ),
        Span::styled(
            "    [h] −0.02    [H] +0.02",
            Style::default().fg(Color::DarkGray),
        ),
    ]));

    lines.push(Line::from(vec![Span::styled(
        " [g] Start/Stop measurement    [0] Switch to this view",
        Style::default().fg(Color::DarkGray),
    )]));

    frame.render_widget(
        Paragraph::new(lines).block(
            Block::default()
                .borders(Borders::ALL)
                .border_style(Style::default().fg(Color::DarkGray)),
        ),
        area,
    );
}

// ── Bands time-series view ────────────────────────────────────────────────────

/// Band colors used in the time-series charts.
#[cfg(feature = "act")]
const BAND_COLORS: [Color; 5] = [
    Color::Magenta, // δ delta
    Color::Cyan,    // θ theta
    Color::Green,   // α alpha
    Color::Yellow,  // β beta
    Color::Red,     // γ gamma
];

// ── Entrainment view ─────────────────────────────────────────────────────────

/// Render the entrainment measurement view.
///
/// Layout:
/// - Top: status bar + progress
/// - Middle-left: per-channel energy table (beat vs total, SNR, artifact%)
/// - Middle-right: big SNR gauge + Porous/Boundaried label
/// - Bottom: controls + config info
#[cfg(feature = "act")]
fn draw_entrainment_view(frame: &mut Frame, area: Rect, app: &App) {
    use muse_rs::entrainment::EntrainmentPhase;

    // If idle, show configuration summary
    if app.entrainment.phase == EntrainmentPhase::Idle && app.entrainment.result.is_none() {
        draw_entrainment_idle(frame, area, app);
        return;
    }

    let rows = Layout::vertical([
        Constraint::Length(3),  // status bar + progress
        Constraint::Min(10),   // main content
        Constraint::Length(3), // controls
    ])
    .split(area);

    draw_entrainment_status(frame, rows[0], app);
    draw_entrainment_main(frame, rows[1], app);
    draw_entrainment_controls(frame, rows[2], app);
}

/// Idle state: show configuration summary and instructions.
#[cfg(feature = "act")]
fn draw_entrainment_idle(frame: &mut Frame, area: Rect, app: &App) {
    let cfg = &app.entrainment.config;
    let dc = &app.entrainment.dict_config;
    let lines = vec![
        Line::from(""),
        Line::from(Span::styled(
            "  Entrainment (SSEP) Measurement",
            Style::default()
                .fg(Color::Cyan)
                .add_modifier(Modifier::BOLD),
        )),
        Line::from(""),
        Line::from(format!(
            "    Beat frequency:  {:.1} Hz    fc tolerance: ±{:.2} Hz",
            cfg.beat_freq_hz, cfg.fc_tolerance_hz,
        )),
        Line::from(format!(
            "    SNR threshold:   {:.2}       duration: {:.0}s (settling {:.0}s)",
            cfg.snr_threshold, cfg.measurement_duration_s, cfg.settling_duration_s,
        )),
        Line::from(""),
        Line::from(Span::styled(
            "  Dictionary config:",
            Style::default().fg(Color::DarkGray),
        )),
        Line::from(format!(
            "    fc range: ({:.2}, {:.2}, {:.2})   chirp: ({:.1}, {:.1}, {:.1})",
            dc.fc_range.0, dc.fc_range.1, dc.fc_range.2,
            dc.chirp_range.0, dc.chirp_range.1, dc.chirp_range.2,
        )),
        Line::from(format!(
            "    log_dt: ({:.1}, {:.1}, {:.1})     order: {}   refine: {}",
            dc.log_dt_range.0, dc.log_dt_range.1, dc.log_dt_range.2,
            dc.order, dc.refine,
        )),
        Line::from(""),
        Line::from(Span::styled(
            "  Press [n] to start entrainment measurement",
            Style::default()
                .fg(Color::Green)
                .add_modifier(Modifier::BOLD),
        )),
        Line::from(Span::styled(
            "  [f]/[F] beat freq ±0.1    [j]/[J] SNR threshold ±0.25",
            Style::default().fg(Color::DarkGray),
        )),
    ];
    let block = Block::default()
        .borders(Borders::ALL)
        .border_style(Style::default().fg(Color::Cyan))
        .title(" Entrainment ");
    let para = Paragraph::new(lines).block(block);
    frame.render_widget(para, area);
}

/// Status bar: phase, timer, progress.
#[cfg(feature = "act")]
fn draw_entrainment_status(frame: &mut Frame, area: Rect, app: &App) {
    use muse_rs::entrainment::EntrainmentPhase;

    let elapsed = app.entrainment.elapsed_s();
    let total = app.entrainment.config.measurement_duration_s;
    let progress = app.entrainment.progress();

    let (phase_str, phase_color) = match &app.entrainment.phase {
        EntrainmentPhase::Idle => ("IDLE", Color::DarkGray),
        EntrainmentPhase::Settling => ("SETTLING", Color::Yellow),
        EntrainmentPhase::Measuring => ("MEASURING", Color::Magenta),
        EntrainmentPhase::Complete => ("COMPLETE", Color::Green),
    };

    // Progress bar characters
    let bar_width = (area.width as usize).saturating_sub(30);
    let filled = (progress * bar_width as f64) as usize;
    let settle_frac = app.entrainment.config.settling_duration_s / total;
    let settle_chars = (settle_frac * bar_width as f64) as usize;

    let mut bar_spans = Vec::new();
    for i in 0..bar_width {
        let ch = if i < filled { "█" } else { "░" };
        let color = if i < settle_chars {
            Color::Yellow
        } else {
            Color::Magenta
        };
        bar_spans.push(Span::styled(ch, Style::default().fg(color)));
    }

    let line = Line::from(vec![
        Span::styled(
            format!(" {phase_str} "),
            Style::default()
                .fg(Color::Black)
                .bg(phase_color)
                .add_modifier(Modifier::BOLD),
        ),
        Span::raw(format!(" {:.0}/{:.0}s ", elapsed, total)),
    ]);

    let bar_line = Line::from(bar_spans);

    let block = Block::default()
        .borders(Borders::ALL)
        .border_style(Style::default().fg(Color::Cyan))
        .title(format!(
            " Entrainment — {:.1} Hz beat ",
            app.entrainment.config.beat_freq_hz
        ));

    let para = Paragraph::new(vec![line, bar_line]).block(block);
    frame.render_widget(para, area);
}

/// Main content: left = per-channel table, right = big SNR + label.
#[cfg(feature = "act")]
fn draw_entrainment_main(frame: &mut Frame, area: Rect, app: &App) {
    let cols = Layout::horizontal([
        Constraint::Percentage(55),
        Constraint::Percentage(45),
    ])
    .split(area);

    draw_entrainment_channels(frame, cols[0], app);
    draw_entrainment_gauge(frame, cols[1], app);
}

/// Per-channel table: beat energy, total energy, SNR, artifact%.
#[cfg(feature = "act")]
fn draw_entrainment_channels(frame: &mut Frame, area: Rect, app: &App) {
    let ch_names = ["TP9", "AF7", "AF8", "TP10"];
    let ch_snr = app.entrainment.live_channel_snr();
    let ch_artifact = app.entrainment.live_channel_artifact_ratio();

    let mut lines = vec![
        Line::from(vec![
            Span::styled(
                " CH     Beat E     Total E      SNR   Art%",
                Style::default()
                    .fg(Color::Cyan)
                    .add_modifier(Modifier::BOLD),
            ),
        ]),
        Line::from("─".repeat(area.width as usize)),
    ];

    for (i, name) in ch_names.iter().enumerate() {
        let ch = &app.entrainment.channels[i];
        let snr_str = if ch.clean_windows > 0 {
            format!("{:.3}", ch_snr[i])
        } else {
            "—".to_string()
        };
        let snr_color = if ch_snr[i] >= app.entrainment.config.snr_threshold {
            Color::Green
        } else {
            Color::Yellow
        };

        lines.push(Line::from(vec![
            Span::styled(
                format!(" {:<5}", name),
                Style::default().fg(Color::White).add_modifier(Modifier::BOLD),
            ),
            Span::raw(format!("  {:>9.4}", ch.beat_energy)),
            Span::raw(format!("  {:>9.4}", ch.total_energy)),
            Span::styled(
                format!("  {:>7}", snr_str),
                Style::default().fg(snr_color),
            ),
            Span::styled(
                format!("  {:>4.0}%", ch_artifact[i] * 100.0),
                Style::default().fg(if ch_artifact[i] > 0.3 {
                    Color::Red
                } else {
                    Color::DarkGray
                }),
            ),
        ]));
    }

    // Summary row
    let mean_snr = app.entrainment.live_snr();
    lines.push(Line::from("─".repeat(area.width as usize)));
    lines.push(Line::from(vec![
        Span::styled(
            " AVG  ",
            Style::default()
                .fg(Color::Cyan)
                .add_modifier(Modifier::BOLD),
        ),
        Span::raw("                        "),
        Span::styled(
            format!("{:>7.3}", mean_snr),
            Style::default()
                .fg(if mean_snr >= app.entrainment.config.snr_threshold {
                    Color::Green
                } else {
                    Color::Yellow
                })
                .add_modifier(Modifier::BOLD),
        ),
    ]));

    // Beat chirplet counts
    lines.push(Line::from(""));
    let total_beat: usize = app.entrainment.channels.iter().map(|c| c.beat_chirplet_count).sum();
    let total_clean: usize = app.entrainment.channels.iter().map(|c| c.clean_windows).sum();
    lines.push(Line::from(Span::styled(
        format!(" Beat chirplets: {}   Clean windows: {}", total_beat, total_clean),
        Style::default().fg(Color::DarkGray),
    )));

    let block = Block::default()
        .borders(Borders::ALL)
        .border_style(Style::default().fg(Color::DarkGray))
        .title(" Per-Channel Energy ");
    let para = Paragraph::new(lines).block(block);
    frame.render_widget(para, area);
}

/// Big SNR gauge + classification label.
#[cfg(feature = "act")]
fn draw_entrainment_gauge(frame: &mut Frame, area: Rect, app: &App) {
    use muse_rs::entrainment::{EntrainmentLabel, EntrainmentPhase};

    let snr = app.entrainment.live_snr();
    let threshold = app.entrainment.config.snr_threshold;
    let label = app.entrainment.live_label();

    let (label_str, label_color) = match label {
        EntrainmentLabel::Porous => ("POROUS", Color::Green),
        EntrainmentLabel::Boundaried => ("BOUNDARIED", Color::Yellow),
    };

    // If we have a final result, show that instead
    let (display_snr, display_label, display_color, is_final) =
        if let Some(result) = &app.entrainment.result {
            let (ls, lc) = match result.label {
                EntrainmentLabel::Porous => ("POROUS", Color::Green),
                EntrainmentLabel::Boundaried => ("BOUNDARIED", Color::Yellow),
            };
            (result.mean_snr, ls, lc, true)
        } else {
            (snr, label_str, label_color, false)
        };

    let mut lines = vec![
        Line::from(""),
        Line::from(Span::styled(
            format!("   SNR: {:.3}", display_snr),
            Style::default()
                .fg(display_color)
                .add_modifier(Modifier::BOLD),
        )),
        Line::from(""),
    ];

    // Visual gauge: SNR from 0 to max(3.0, snr+0.5)
    let gauge_max = 3.0_f64.max(display_snr + 0.5);
    let gauge_width = (area.width as usize).saturating_sub(6);
    if gauge_width > 4 {
        let filled = ((display_snr / gauge_max) * gauge_width as f64).min(gauge_width as f64) as usize;
        let thresh_pos = ((threshold / gauge_max) * gauge_width as f64).min(gauge_width as f64) as usize;

        let mut bar_spans = vec![Span::raw("   ")];
        for i in 0..gauge_width {
            let ch = if i < filled { "█" } else { "░" };
            let color = if i < filled { display_color } else { Color::DarkGray };
            if i == thresh_pos {
                bar_spans.push(Span::styled("▎", Style::default().fg(Color::White)));
            } else {
                bar_spans.push(Span::styled(ch, Style::default().fg(color)));
            }
        }
        lines.push(Line::from(bar_spans));
        lines.push(Line::from(Span::styled(
            format!("   0{:>width$}{:.1}", "", gauge_max, width = gauge_width - 4),
            Style::default().fg(Color::DarkGray),
        )));
    }

    lines.push(Line::from(""));

    // Classification label
    let final_tag = if is_final { " (FINAL)" } else { "" };
    lines.push(Line::from(Span::styled(
        format!("   {}{}", display_label, final_tag),
        Style::default()
            .fg(display_color)
            .add_modifier(Modifier::BOLD),
    )));

    lines.push(Line::from(""));
    lines.push(Line::from(Span::styled(
        format!("   Threshold: {:.2}", threshold),
        Style::default().fg(Color::DarkGray),
    )));

    // Show result details if complete
    if let Some(result) = &app.entrainment.result {
        lines.push(Line::from(Span::styled(
            format!(
                "   Duration: {:.1}s  Artifact: {:.0}%",
                result.recording_duration_s,
                result.artifact_ratio * 100.0,
            ),
            Style::default().fg(Color::DarkGray),
        )));
    }

    let title = if app.entrainment.phase == EntrainmentPhase::Complete {
        " Result "
    } else {
        " Live Classification "
    };

    let block = Block::default()
        .borders(Borders::ALL)
        .border_style(Style::default().fg(display_color))
        .title(title);
    let para = Paragraph::new(lines).block(block);
    frame.render_widget(para, area);
}

/// Controls bar: beat freq, threshold, key hints.
#[cfg(feature = "act")]
fn draw_entrainment_controls(frame: &mut Frame, area: Rect, app: &App) {
    let cfg = &app.entrainment.config;
    let line = Line::from(vec![
        Span::styled(" Beat: ", Style::default().fg(Color::DarkGray)),
        Span::styled(
            format!("{:.1} Hz", cfg.beat_freq_hz),
            Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD),
        ),
        Span::raw("  "),
        Span::styled("Thr: ", Style::default().fg(Color::DarkGray)),
        Span::styled(
            format!("{:.2}", cfg.snr_threshold),
            Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD),
        ),
        Span::raw("    "),
        Span::styled("[n]", Style::default().fg(Color::White).add_modifier(Modifier::BOLD)),
        Span::raw(" start/stop  "),
        Span::styled("[f/F]", Style::default().fg(Color::White).add_modifier(Modifier::BOLD)),
        Span::raw(" freq ±0.1  "),
        Span::styled("[j/J]", Style::default().fg(Color::White).add_modifier(Modifier::BOLD)),
        Span::raw(" thr ±0.25  "),
    ]);

    let block = Block::default()
        .borders(Borders::ALL)
        .border_style(Style::default().fg(Color::DarkGray));
    let para = Paragraph::new(vec![line]).block(block);
    frame.render_widget(para, area);
}

/// Render the Bands time-series view: 4 channel rows, each split into ACT (left)
/// and FFT (right) with overlaid per-band relative-power line charts.
#[cfg(feature = "act")]
fn draw_bands_view(frame: &mut Frame, area: Rect, app: &App) {
    use muse_rs::alpha::BAND_LABELS;

    let rows = Layout::vertical([
        Constraint::Ratio(1, 5), // ch 0
        Constraint::Ratio(1, 5), // ch 1
        Constraint::Ratio(1, 5), // ch 2
        Constraint::Ratio(1, 5), // ch 3
        Constraint::Min(3),      // legend / info row
    ])
    .split(area);

    for ch in 0..NUM_CH {
        draw_bands_channel(frame, rows[ch], ch, app);
    }

    // ── Legend + zoom info row ────────────────────────────────────────────
    let span = app.bands_x_span;
    let secs = span as f64 * 0.25; // each hop ≈ 250 ms
    let n_pts = app.band_history.channels[0].len();

    let mut legend_spans: Vec<Span> = vec![Span::styled(
        " Bands: ",
        Style::default().fg(Color::White).add_modifier(Modifier::BOLD),
    )];
    for (i, &label) in BAND_LABELS.iter().enumerate() {
        legend_spans.push(Span::styled(
            format!("■ {label} "),
            Style::default().fg(BAND_COLORS[i]).add_modifier(Modifier::BOLD),
        ));
    }
    legend_spans.push(Span::raw("   "));
    legend_spans.push(Span::styled(
        format!("X span: {secs:.0}s ({span} pts)  "),
        Style::default().fg(Color::White),
    ));
    legend_spans.push(Span::styled(
        format!("History: {n_pts} pts  "),
        Style::default().fg(Color::DarkGray),
    ));
    legend_spans.push(Span::styled(
        "[  zoom in  ] zoom out",
        Style::default().fg(Color::Cyan),
    ));

    frame.render_widget(
        Paragraph::new(Line::from(legend_spans)).block(
            Block::default()
                .borders(Borders::ALL)
                .border_style(Style::default().fg(Color::DarkGray)),
        ),
        rows[4],
    );
}

/// Render one channel's band power time-series: ACT (left) vs FFT (right).
#[cfg(feature = "act")]
fn draw_bands_channel(frame: &mut Frame, area: Rect, ch: usize, app: &App) {
    use muse_rs::alpha::{BAND_LABELS, NUM_BANDS};
    use ratatui::widgets::{Axis, Chart, Dataset, GraphType};
    use ratatui::symbols::Marker;

    let color = COLORS[ch];
    let name = EEG_CHANNEL_NAMES[ch];
    let ch_hist = &app.band_history.channels[ch];
    let span = app.bands_x_span;

    let cols = Layout::horizontal([Constraint::Ratio(1, 2), Constraint::Ratio(1, 2)]).split(area);

    let total_pts = ch_hist.len();
    // Show the latest `span` points (or fewer if not enough data)
    let start = total_pts.saturating_sub(span);

    // Helper: build datasets for one source (ACT or FFT)
    let build_datasets = |source: &std::collections::VecDeque<muse_rs::alpha::BandFrame>| -> Vec<Vec<(f64, f64)>> {
        let mut band_data: Vec<Vec<(f64, f64)>> = (0..NUM_BANDS).map(|_| Vec::new()).collect();
        for (i, frame) in source.iter().skip(start).enumerate() {
            let x = i as f64;
            for b in 0..NUM_BANDS {
                band_data[b].push((x, frame.bands[b] as f64 * 100.0)); // convert to %
            }
        }
        band_data
    };

    let x_max = span.max(1) as f64;

    // ── ACT chart ────────────────────────────────────────────────────────
    {
        let act_data = build_datasets(&ch_hist.act);
        let datasets: Vec<Dataset> = (0..NUM_BANDS)
            .map(|b| {
                Dataset::default()
                    .name(BAND_LABELS[b])
                    .marker(Marker::Braille)
                    .graph_type(GraphType::Line)
                    .style(Style::default().fg(BAND_COLORS[b]))
                    .data(&act_data[b])
            })
            .collect();

        let chart = Chart::new(datasets)
            .block(
                Block::default()
                    .title(Span::styled(
                        format!(" {name} ACT "),
                        Style::default().fg(color).add_modifier(Modifier::BOLD),
                    ))
                    .borders(Borders::ALL)
                    .border_style(Style::default().fg(Color::Cyan)),
            )
            .x_axis(
                Axis::default()
                    .bounds([0.0, x_max])
                    .labels(vec![
                        Span::raw("now"),
                    ]),
            )
            .y_axis(
                Axis::default()
                    .bounds([0.0, 100.0])
                    .labels(vec![
                        Span::raw("0%"),
                        Span::raw("50%"),
                        Span::raw("100%"),
                    ]),
            );

        frame.render_widget(chart, cols[0]);
    }

    // ── FFT chart ────────────────────────────────────────────────────────
    {
        let fft_data = build_datasets(&ch_hist.fft);
        let datasets: Vec<Dataset> = (0..NUM_BANDS)
            .map(|b| {
                Dataset::default()
                    .name(BAND_LABELS[b])
                    .marker(Marker::Braille)
                    .graph_type(GraphType::Line)
                    .style(Style::default().fg(BAND_COLORS[b]))
                    .data(&fft_data[b])
            })
            .collect();

        let chart = Chart::new(datasets)
            .block(
                Block::default()
                    .title(Span::styled(
                        format!(" {name} FFT "),
                        Style::default().fg(color).add_modifier(Modifier::BOLD),
                    ))
                    .borders(Borders::ALL)
                    .border_style(Style::default().fg(Color::Yellow)),
            )
            .x_axis(
                Axis::default()
                    .bounds([0.0, x_max])
                    .labels(vec![
                        Span::raw("now"),
                    ]),
            )
            .y_axis(
                Axis::default()
                    .bounds([0.0, 100.0])
                    .labels(vec![
                        Span::raw("0%"),
                        Span::raw("50%"),
                        Span::raw("100%"),
                    ]),
            );

        frame.render_widget(chart, cols[1]);
    }
}

// ── Footer ────────────────────────────────────────────────────────────────────

/// Render the two-line footer at the bottom of the screen.
///
/// **Line 1** — keyboard shortcut reference.  A `⏸ PAUSED` badge is appended
/// when streaming is paused.
///
/// **Line 2** — context-dependent:
/// * `NoDevices` mode: a platform-specific hint for how to grant Bluetooth access.
/// * All other modes: live accelerometer and gyroscope readings.
fn draw_footer(frame: &mut Frame, area: Rect, app: &App) {
    let pause_span = if app.paused {
        Span::styled(
            "  ⏸ PAUSED",
            Style::default().fg(Color::Red).add_modifier(Modifier::BOLD),
        )
    } else {
        Span::raw("")
    };

    let keys = Line::from(vec![
        Span::raw(" "),
        key("[Tab]"),
        Span::raw("Devices  "),
        key("[1]"),
        Span::raw("EEG  "),
        key("[2]"),
        Span::raw("PPG  "),
        key("[3]"),
        Span::raw("Info  "),
        key("[4]"),
        Span::raw("IMU  "),
        #[cfg(feature = "act")]
        key("[5]"),
        #[cfg(feature = "act")]
        Span::raw("ACT  "),
        #[cfg(feature = "act")]
        key("[6]"),
        #[cfg(feature = "act")]
        Span::raw("Baseline  "),
        #[cfg(feature = "act")]
        key("[7]"),
        #[cfg(feature = "act")]
        Span::raw("Absorb  "),
        #[cfg(feature = "act")]
        key("[8]"),
        #[cfg(feature = "act")]
        Span::raw("Entrain  "),
        #[cfg(feature = "act")]
        key("[9]"),
        #[cfg(feature = "act")]
        Span::raw("Bands  "),
        #[cfg(feature = "act")]
        key("[b]"),
        #[cfg(feature = "act")]
        if app.baseline.is_running() {
            Span::styled("● BL  ", Style::default().fg(Color::Green).add_modifier(Modifier::BOLD))
        } else {
            Span::raw("Baseline  ")
        },
        #[cfg(feature = "act")]
        key("[m]"),
        #[cfg(feature = "act")]
        if app.absorption.is_running() {
            Span::styled("● MEAS  ", Style::default().fg(Color::Magenta).add_modifier(Modifier::BOLD))
        } else {
            Span::raw("Measure  ")
        },
        #[cfg(feature = "act")]
        key("[n]"),
        #[cfg(feature = "act")]
        if app.entrainment.is_running() {
            Span::styled("● ENT  ", Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD))
        } else {
            Span::raw("Entrain  ")
        },
        key("[w]"),
        if app.csv_recorder.is_some() {
            Span::styled("● REC  ", Style::default().fg(Color::Red).add_modifier(Modifier::BOLD))
        } else {
            Span::raw("Record  ")
        },
        key("[d]"),
        Span::raw("Disconnect  "),
        key("[+/-]"),
        Span::raw("Scale  "),
        key("[a]"),
        Span::raw("Auto  "),
        key("[v]"),
        Span::raw(if app.smooth { "Raw  " } else { "Smooth  " }),
        key("[p]"),
        Span::raw("Pause  "),
        key("[r]"),
        Span::raw("Resume  "),
        key("[c]"),
        Span::raw("Clear  "),
        key("[q]"),
        Span::raw("Quit"),
        pause_span,
    ]);

    // Second row: macOS permission hint when waiting for devices, else IMU.
    let second_line = match &app.mode {
        AppMode::NoDevices => {
            let base = if cfg!(target_os = "macos") {
                " No Muse found. On macOS grant Bluetooth access: System Settings → Privacy & Security → Bluetooth."
            } else {
                " No Muse found. Make sure the headset is powered on and in range."
            };
            let detail = app
                .last_error
                .as_deref()
                .map(|e| format!(" Error: {e}"))
                .unwrap_or_default();
            Line::from(Span::styled(
                format!("{base}{detail} Retrying…"),
                Style::default().fg(Color::Yellow),
            ))
        }
        _ => {
            let (ax, ay, az) = app.accel.unwrap_or((0.0, 0.0, 0.0));
            let (gx, gy, gz) = app.gyro.unwrap_or((0.0, 0.0, 0.0));
            Line::from(vec![
                Span::raw(" "),
                Span::styled("Accel ", Style::default().fg(Color::DarkGray)),
                Span::styled(
                    format!("x:{ax:+.3}g  y:{ay:+.3}g  z:{az:+.3}g"),
                    Style::default().fg(Color::Cyan),
                ),
                Span::raw("   "),
                Span::styled("Gyro ", Style::default().fg(Color::DarkGray)),
                Span::styled(
                    format!("x:{gx:+.3}°/s  y:{gy:+.3}°/s  z:{gz:+.3}°/s"),
                    Style::default().fg(Color::Magenta),
                ),
            ])
        }
    };

    frame.render_widget(
        Paragraph::new(vec![keys, second_line]).block(Block::default().borders(Borders::ALL)),
        area,
    );
}

/// Styled keybinding label (bold yellow) used in the footer hint line.
#[inline]
fn key(s: &str) -> Span<'_> {
    Span::styled(
        s,
        Style::default()
            .fg(Color::Yellow)
            .add_modifier(Modifier::BOLD),
    )
}

// ── Device picker overlay ─────────────────────────────────────────────────────

/// Render the centered device-picker modal over the main UI.
///
/// The popup is sized to 60 % of the terminal width (min 52, max full width)
/// and tall enough to show all discovered devices plus a 2-line hint area.
///
/// * Devices are shown as `"Name  [SHORT-ID]"` with the connected device
///   highlighted in green with a `← connected` suffix.
/// * While a BLE scan is running a spinner appears in the title bar.
/// * The cursor row is highlighted with a `▶` prefix; `↑` / `↓` scroll it.
fn draw_device_picker(frame: &mut Frame, area: Rect, app: &App) {
    let n = app.picker_entries.len().max(1);
    let inner_h = n as u16 + 4;
    let box_h = inner_h + 2;
    let box_w = (area.width * 60 / 100).max(52).min(area.width);
    let x = area.x + (area.width.saturating_sub(box_w)) / 2;
    let y = area.y + (area.height.saturating_sub(box_h)) / 2;
    let popup = Rect::new(x, y, box_w, box_h.min(area.height));

    frame.render_widget(Clear, popup);

    let title = if app.picker_scanning {
        format!(" {} Scanning…  ({} found) ", spinner_str(), app.picker_entries.len())
    } else {
        format!(" Select Device  ({} found) ", app.picker_entries.len())
    };

    frame.render_widget(
        Block::default()
            .title(Span::styled(
                title,
                Style::default().fg(Color::White).add_modifier(Modifier::BOLD),
            ))
            .borders(Borders::ALL)
            .border_style(Style::default().fg(Color::White)),
        popup,
    );

    let inner = popup.inner(Margin {
        horizontal: 1,
        vertical: 1,
    });

    let hint_h = 2u16;
    let [list_area, _, hint_area] = Layout::vertical([
        Constraint::Length(inner.height.saturating_sub(hint_h + 1)),
        Constraint::Length(1),
        Constraint::Length(hint_h),
    ])
    .areas(inner);

    // Build list items
    let items: Vec<ListItem> = if app.picker_entries.is_empty() {
        vec![ListItem::new(Span::styled(
            "  No devices found — press [s] to scan",
            Style::default().fg(Color::DarkGray),
        ))]
    } else {
        app.picker_entries
            .iter()
            .enumerate()
            .map(|(i, entry)| {
                let connected = app.picker_connected_idx == Some(i);
                let (bullet, color, suffix) = if connected {
                    ("● ", Color::Green, "  ← connected")
                } else {
                    ("  ", Color::White, "")
                };
                ListItem::new(Span::styled(
                    format!("{bullet}{entry}{suffix}"),
                    Style::default().fg(color),
                ))
            })
            .collect()
    };

    let mut list_state = ListState::default();
    if !app.picker_entries.is_empty() {
        list_state.select(Some(app.picker_cursor));
    }

    frame.render_stateful_widget(
        List::new(items)
            .highlight_style(
                Style::default()
                    .fg(Color::Black)
                    .bg(Color::White)
                    .add_modifier(Modifier::BOLD),
            )
            .highlight_symbol("▶ "),
        list_area,
        &mut list_state,
    );

    frame.render_widget(
        Paragraph::new(vec![
            Line::from(vec![
                key(" [↑↓]"),
                Span::raw(" Navigate  "),
                key("[↵]"),
                Span::raw(" Connect  "),
                key("[s]"),
                Span::raw(" Rescan  "),
                key("[Esc]"),
                Span::raw(" Close"),
            ]),
            Line::from(Span::styled(
                " Device list is refreshed after every scan",
                Style::default().fg(Color::DarkGray),
            )),
        ]),
        hint_area,
    );
}

// ── Entry point ───────────────────────────────────────────────────────────────

#[tokio::main]
async fn main() -> Result<()> {
    use std::io::IsTerminal as _;
    if !io::stdout().is_terminal() {
        eprintln!("Error: muse-rs tui requires a real terminal (TTY).");
        eprintln!("Run it directly in a terminal emulator, not piped or redirected.");
        std::process::exit(1);
    }

    // ── Logging ─────────────────────────────────────────────────────────────
    // Write logs to a file so they never interfere with the TUI display.
    // Set RUST_LOG=debug for verbose BLE diagnostics, e.g.:
    //   RUST_LOG=debug cargo run --bin tui
    // Logs are written to muse-tui.log in the current directory.
    {
        use std::fs::File;
        if let Ok(file) = File::create("muse-tui.log") {
            env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info"))
                .target(env_logger::Target::Pipe(Box::new(file)))
                .init();
        }
    }

    let simulate = std::env::args().any(|a| a == "--simulate");

    // ── Shared UI state ───────────────────────────────────────────────────────
    let app = Arc::new(Mutex::new(App::new()));

    // ── Session state (owned by main task only) ───────────────────────────────
    let mut devices: Vec<MuseDevice> = vec![];
    let mut connected_idx: Option<usize> = None;
    let mut handle: Option<Arc<MuseHandle>> = None;

    // Oneshot receiver for the current background scan task.
    let mut pending_scan: Option<oneshot::Receiver<ScanResult>> = None;
    // Oneshot receiver for the current background connect task.
    let mut pending_connect: Option<oneshot::Receiver<Option<ConnectOutcome>>> = None;
    // Instant at which the next automatic retry scan should start.
    let mut retry_at: Option<tokio::time::Instant> = None;

    let scan_cfg = MuseClientConfig {
        scan_timeout_secs: 5,
        ..Default::default()
    };

    // ── Initialize ACT engine (both real device and simulator) ─────────────
    #[cfg(feature = "act")]
    {
        let mut s = app.lock().unwrap();
        let act_cfg = muse_rs::act::ActConfig {
            fs: EEG_HZ,
            length: BUF_SIZE,  // use same window as EEG display buffer
            tc_range: (0.0, (BUF_SIZE - 1) as f64, 16.0),
            fc_range: (0.5, 30.0, 1.0),
            log_dt_range: (-2.5, -0.3, 0.3),
            chirp_range: (-20.0, 20.0, 5.0),
        };
        match muse_rs::act::ActEngine::new(&act_cfg) {
            Ok(engine) => {
                log::info!(
                    "ACT engine initialised: dict_size={}, window={}",
                    engine.dict_size(),
                    engine.window_length()
                );
                s.act_engine = Some(engine);
                s.act_opts = muse_rs::act::TransformOpts {
                    order: 10,
                    residual_threshold: 1e-6,
                    refine: true,
                };
                s.act_hop = 64; // ~250 ms at 256 Hz
            }
            Err(e) => {
                log::warn!("ACT engine init failed: {e}");
            }
        }
    }

    // ── Start data source ─────────────────────────────────────────────────────
    if simulate {
        let mut s = app.lock().unwrap();
        s.mode = AppMode::Simulated;
        // Simulator peak ≈ ±40 µV — start at ±50 µV so the waveform fills the chart.
        // Real devices use the DEFAULT_SCALE (±500 µV) set at App::new().
        s.scale_idx = 2; // Y_SCALES[2] = 50.0
        drop(s);
        spawn_simulator(Arc::clone(&app));
    } else {
        app.lock().unwrap().picker_scanning = true;
        pending_scan = Some(start_scan(scan_cfg.clone()));
    }

    // ── Terminal setup ────────────────────────────────────────────────────────
    enable_raw_mode()?;
    let mut stdout = io::stdout();
    execute!(stdout, EnterAlternateScreen)?;
    let mut terminal = Terminal::new(CrosstermBackend::new(stdout))?;
    let tick = Duration::from_millis(33); // ~30 FPS

    // ── Main loop ─────────────────────────────────────────────────────────────
    'main: loop {
        // ── 1. Collect finished scan results ─────────────────────────────────
        if let Some(ref mut rx) = pending_scan {
            if let Ok(scan_result) = rx.try_recv() {
                pending_scan = None;
                devices = scan_result.devices;

                {
                    let mut s = app.lock().unwrap();
                    s.picker_entries = devices.iter().map(device_entry).collect();
                    s.picker_scanning = false;
                    if devices.is_empty() {
                        s.mode = AppMode::NoDevices;
                        if let Some(err) = scan_result.error {
                            s.last_error = Some(err);
                        }
                    }
                }

                if devices.is_empty() {
                    retry_at = Some(
                        tokio::time::Instant::now() + Duration::from_secs(RETRY_SECS),
                    );
                } else if handle.is_none() && pending_connect.is_none() {
                    // Auto-connect to the first device — non-blocking.
                    pending_connect = Some(start_connect(
                        0,
                        devices[0].clone(),
                        Arc::clone(&app),
                        scan_cfg.clone(),
                    ));
                }
            }
        }

        // ── 1b. Collect finished connection attempt ───────────────────────────
        if let Some(ref mut rx) = pending_connect {
            if let Ok(result) = rx.try_recv() {
                pending_connect = None;
                if let Some(outcome) = result {
                    let h = Arc::new(outcome.handle);
                    {
                        let mut s = app.lock().unwrap();
                        s.mode = AppMode::Connected {
                            name: outcome.name,
                            id: outcome.id,
                        };
                        s.last_error = None;
                        s.picker_connected_idx = Some(outcome.device_idx);
                    }
                    connected_idx = Some(outcome.device_idx);
                    handle = Some(Arc::clone(&h));
                    spawn_event_task(outcome.rx, Arc::clone(&app));
                } else {
                    // Connection attempt failed — wipe state and restart scanning.
                    connected_idx = None;
                    devices.clear();    // remove stale device entries from local list
                    restart_scan(&app, &mut pending_scan, &mut retry_at, RECONNECT_DELAY_SECS);
                }
            }
        }

        // ── 2. Detect unexpected disconnection ───────────────────────────────
        //    The event task sets mode = Disconnected; we react here.
        {
            let is_disconnected =
                matches!(app.lock().unwrap().mode, AppMode::Disconnected);
            if is_disconnected && handle.is_some() {
                // Drop the dead handle; best-effort remote disconnect in background.
                if let Some(h) = handle.take() {
                    tokio::spawn(async move { let _ = h.disconnect().await; });
                }
                connected_idx = None;
                devices.clear();    // remove stale device entries from local list
                restart_scan(&app, &mut pending_scan, &mut retry_at, RECONNECT_DELAY_SECS);
            }
        }

        // ── 3. Fire pending retry scan ────────────────────────────────────────
        if let Some(t) = retry_at {
            if tokio::time::Instant::now() >= t && pending_scan.is_none() {
                retry_at = None;
                app.lock().unwrap().mode = AppMode::Scanning;
                app.lock().unwrap().picker_scanning = true;
                pending_scan = Some(start_scan(scan_cfg.clone()));
            }
        }

        // ── 4. Render ─────────────────────────────────────────────────────────
        {
            let s = app.lock().unwrap();
            terminal.draw(|f| draw(f, &s))?;
        }

        // ── 5. Handle keyboard ────────────────────────────────────────────────
        if !event::poll(tick)? {
            continue;
        }
        let Event::Key(key) = event::read()? else {
            continue;
        };

        // ── Global quit — fires regardless of picker or any other overlay ────
        // In raw mode Ctrl+C is not SIGINT; it arrives as a key event and must
        // be handled explicitly.  Without this check the app is impossible to
        // exit while the picker is open.
        let ctrl_c = key.modifiers.contains(KeyModifiers::CONTROL)
            && key.code == KeyCode::Char('c');
        if key.code == KeyCode::Char('q') || ctrl_c {
            break 'main;
        }

        // ── Picker overlay keys ───────────────────────────────────────────────
        if app.lock().unwrap().show_picker {
            match key.code {
                KeyCode::Esc => {
                    app.lock().unwrap().show_picker = false;
                }
                KeyCode::Char('s') => {
                    if pending_scan.is_none() {
                        retry_at = None;
                        let mut s = app.lock().unwrap();
                        s.mode = AppMode::Scanning;
                        s.picker_scanning = true;
                        drop(s);
                        pending_scan = Some(start_scan(scan_cfg.clone()));
                    }
                }
                KeyCode::Up => {
                    let mut s = app.lock().unwrap();
                    if s.picker_cursor > 0 {
                        s.picker_cursor -= 1;
                    }
                }
                KeyCode::Down => {
                    let mut s = app.lock().unwrap();
                    let max = s.picker_entries.len().saturating_sub(1);
                    if s.picker_cursor < max {
                        s.picker_cursor += 1;
                    }
                }
                KeyCode::Enter => {
                    let (cursor, n) = {
                        let s = app.lock().unwrap();
                        (s.picker_cursor, s.picker_entries.len())
                    };
                    if n > 0 && cursor < n && cursor < devices.len() {
                        retry_at = None;
                        // Disconnect old handle in the background.
                        if let Some(h) = handle.take() {
                            tokio::spawn(async move { let _ = h.disconnect().await; });
                        }
                        connected_idx = None;
                        pending_connect = Some(start_connect(
                            cursor,
                            devices[cursor].clone(),
                            Arc::clone(&app),
                            scan_cfg.clone(),
                        ));
                    }
                }
                _ => {}
            }
            continue;
        }

        // ── Normal-view keys ──────────────────────────────────────────────────
        match key.code {
            KeyCode::Char('q') | KeyCode::Esc => break 'main,

            // Open device picker
            KeyCode::Tab => {
                let mut s = app.lock().unwrap();
                s.show_picker = true;
                if let Some(ci) = connected_idx {
                    s.picker_cursor = ci;
                }
            }

            // Manual rescan
            KeyCode::Char('s') => {
                if pending_scan.is_none() {
                    retry_at = None;
                    app.lock().unwrap().mode = AppMode::Scanning;
                    app.lock().unwrap().picker_scanning = true;
                    pending_scan = Some(start_scan(scan_cfg.clone()));
                }
            }

            // Disconnect current device (keep scanning)
            KeyCode::Char('d') => {
                if let Some(h) = handle.take() {
                    // Fire-and-forget: never block the main loop on a BLE call.
                    tokio::spawn(async move { let _ = h.disconnect().await; });
                }
                pending_connect = None;
                connected_idx = None;
                app.lock().unwrap().picker_connected_idx = None;
                // Trigger a fresh scan to allow re-connecting
                if pending_scan.is_none() {
                    retry_at = None;
                    app.lock().unwrap().mode = AppMode::Scanning;
                    app.lock().unwrap().picker_scanning = true;
                    pending_scan = Some(start_scan(scan_cfg.clone()));
                }
            }

            // µV scale
            KeyCode::Char('+') | KeyCode::Char('=') => {
                app.lock().unwrap().scale_up();
            }
            KeyCode::Char('-') => {
                app.lock().unwrap().scale_down();
            }
            KeyCode::Char('a') => {
                app.lock().unwrap().auto_scale();
            }

            // View toggle: EEG / PPG / Info / IMU
            KeyCode::Char('1') => {
                app.lock().unwrap().view = ViewMode::Eeg;
            }
            KeyCode::Char('2') => {
                app.lock().unwrap().view = ViewMode::Ppg;
            }
            KeyCode::Char('3') => {
                app.lock().unwrap().view = ViewMode::Info;
            }
            KeyCode::Char('4') => {
                app.lock().unwrap().view = ViewMode::Imu;
            }
            #[cfg(feature = "act")]
            KeyCode::Char('5') => {
                app.lock().unwrap().view = ViewMode::Act;
            }
            #[cfg(feature = "act")]
            KeyCode::Char('6') => {
                app.lock().unwrap().view = ViewMode::Baseline;
            }
            #[cfg(feature = "act")]
            KeyCode::Char('7') => {
                app.lock().unwrap().view = ViewMode::Absorption;
            }
            #[cfg(feature = "act")]
            KeyCode::Char('8') => {
                app.lock().unwrap().view = ViewMode::Entrainment;
            }
            #[cfg(feature = "act")]
            KeyCode::Char('9') => {
                app.lock().unwrap().view = ViewMode::Bands;
            }
            #[cfg(feature = "act")]
            KeyCode::Char('0') => {
                app.lock().unwrap().view = ViewMode::Approach;
            }

            // Bands view: zoom x-axis with [ and ]
            #[cfg(feature = "act")]
            KeyCode::Char('[') => {
                let mut s = app.lock().unwrap();
                if s.view == ViewMode::Bands {
                    s.bands_x_span = (s.bands_x_span / 2).max(20); // zoom in
                }
            }
            #[cfg(feature = "act")]
            KeyCode::Char(']') => {
                let mut s = app.lock().unwrap();
                if s.view == ViewMode::Bands {
                    s.bands_x_span = (s.bands_x_span * 2).min(muse_rs::alpha::BAND_HISTORY_MAX); // zoom out
                }
            }

            // Absorption measurement start/stop
            #[cfg(feature = "act")]
            KeyCode::Char('m') => {
                let mut s = app.lock().unwrap();
                if s.absorption.is_running() {
                    s.absorption.stop();
                } else if let Some(bl) = s.baseline.result.clone() {
                    s.absorption.start(&bl);
                    beep_once();
                    s.view = ViewMode::Absorption;
                } else {
                    s.absorption.start_no_baseline();
                    s.view = ViewMode::Absorption;
                }
            }

            // Absorption threshold: [t] decrease, [T] increase
            #[cfg(feature = "act")]
            KeyCode::Char('t') => {
                let mut s = app.lock().unwrap();
                s.absorption.config.threshold_pct = (s.absorption.config.threshold_pct - 5.0).max(5.0);
            }
            #[cfg(feature = "act")]
            KeyCode::Char('T') => {
                let mut s = app.lock().unwrap();
                s.absorption.config.threshold_pct = (s.absorption.config.threshold_pct + 5.0).min(80.0);
            }

            // Entrainment measurement start/stop
            #[cfg(feature = "act")]
            KeyCode::Char('n') => {
                let mut s = app.lock().unwrap();
                if s.entrainment.is_running() {
                    s.entrainment.stop();
                } else {
                    match s.entrainment.start() {
                        Ok(()) => {
                            beep_once();
                            s.view = ViewMode::Entrainment;
                        }
                        Err(e) => {
                            log::warn!("Entrainment start failed: {e}");
                        }
                    }
                }
            }

            // Entrainment beat frequency: [f] decrease, [F] increase
            #[cfg(feature = "act")]
            KeyCode::Char('f') => {
                let mut s = app.lock().unwrap();
                s.entrainment.config.beat_freq_hz = (s.entrainment.config.beat_freq_hz - 0.1).max(0.5);
            }
            #[cfg(feature = "act")]
            KeyCode::Char('F') => {
                let mut s = app.lock().unwrap();
                s.entrainment.config.beat_freq_hz = (s.entrainment.config.beat_freq_hz + 0.1).min(8.0);
            }

            // Entrainment SNR threshold: [j] decrease, [J] increase
            #[cfg(feature = "act")]
            KeyCode::Char('j') => {
                let mut s = app.lock().unwrap();
                s.entrainment.config.snr_threshold = (s.entrainment.config.snr_threshold - 0.25).max(0.25);
            }
            #[cfg(feature = "act")]
            KeyCode::Char('J') => {
                let mut s = app.lock().unwrap();
                s.entrainment.config.snr_threshold = (s.entrainment.config.snr_threshold + 0.25).min(10.0);
            }

            // Approach measurement start/stop
            #[cfg(feature = "act")]
            KeyCode::Char('g') => {
                let mut s = app.lock().unwrap();
                if s.approach.is_running() {
                    s.approach.stop();
                } else {
                    s.approach.start();
                    beep_once();
                    s.view = ViewMode::Approach;
                }
            }

            // Approach deadband: [h] decrease, [H] increase
            #[cfg(feature = "act")]
            KeyCode::Char('h') => {
                let mut s = app.lock().unwrap();
                s.approach.config.deadband = (s.approach.config.deadband - 0.02).max(0.02);
            }
            #[cfg(feature = "act")]
            KeyCode::Char('H') => {
                let mut s = app.lock().unwrap();
                s.approach.config.deadband = (s.approach.config.deadband + 0.02).min(0.50);
            }

            // Baseline detector start/stop
            #[cfg(feature = "act")]
            KeyCode::Char('b') => {
                let mut s = app.lock().unwrap();
                if s.baseline.is_running() {
                    s.baseline.stop();
                } else {
                    s.baseline.start();
                    beep_once();
                    // Auto-switch to baseline view
                    s.view = ViewMode::Baseline;
                }
            }

            // Toggle CSV recording
            KeyCode::Char('w') => {
                let mut s = app.lock().unwrap();
                if s.csv_recorder.is_some() {
                    // Stop recording
                    if let Some(mut rec) = s.csv_recorder.take() {
                        rec.flush();
                    }
                    log::info!("CSV recording stopped");
                } else {
                    // Start recording
                    match CsvRecorder::open() {
                        Ok(rec) => {
                            s.csv_recorder = Some(rec);
                        }
                        Err(e) => {
                            log::warn!("Failed to start CSV recording: {e}");
                            s.last_error = Some(format!("CSV error: {e}"));
                        }
                    }
                }
            }

            // Toggle smooth overlay
            KeyCode::Char('v') => {
                let mut s = app.lock().unwrap();
                s.smooth = !s.smooth;
            }

            // Pause / resume
            KeyCode::Char('p') => {
                app.lock().unwrap().paused = true;
                if let Some(h) = handle.clone() {
                    tokio::spawn(async move { let _ = h.pause().await; });
                }
            }
            KeyCode::Char('r') => {
                app.lock().unwrap().paused = false;
                if let Some(h) = handle.clone() {
                    tokio::spawn(async move { let _ = h.resume().await; });
                }
            }

            // Clear buffers (guard: Ctrl+C is already handled above as quit)
            KeyCode::Char('c') if !key.modifiers.contains(KeyModifiers::CONTROL) => {
                app.lock().unwrap().clear();
            }

            _ => {}
        }
    }

    // ── Teardown ──────────────────────────────────────────────────────────────
    if let Some(h) = handle {
        let _ = h.disconnect().await;
    }
    disable_raw_mode()?;
    execute!(terminal.backend_mut(), LeaveAlternateScreen)?;
    terminal.show_cursor()?;
    Ok(())
}
