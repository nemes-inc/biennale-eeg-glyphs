//! Signal processing pipeline for EEG dimension analysis.
//!
//! Separates pure semantic functions from I/O pragmatic shell.
//! All detectors run inside the BLE task, producing AnalysisFrame
//! snapshots that cross the mutex boundary to the UI thread.

use std::collections::VecDeque;

use muse_rs::alpha::FftSnapshot;
use muse_rs::alpha_trend::{AlphaTrendDetector, AlphaTrendPhase};
use muse_rs::approach::{ApproachDetector, ApproachPhase};
use muse_rs::compute::{ContactQuality, ContactQualityTracker};
use muse_rs::entrainment::{EntrainmentDetector, EntrainmentPhase};
use muse_rs::filters::{ChannelFilter, EpochStatus, EpochValidator};

// ── Constants ────────────────────────────────────────────────────────────────

const FS: f64 = 256.0;
const HOP_SAMPLES: usize = 64;       // 250ms analysis hop at 256 Hz
const FFT_WINDOW: usize = 512;       // 2 second FFT window
const ENT_WINDOW: usize = 1024;      // 4 second entrainment window
const BUF_CAP: usize = 2048;         // ~8 seconds ring buffer
const HISTORY_CAP: usize = 120;      // ~30 seconds sparkline at 250ms hop

// ── Brand Types ──────────────────────────────────────────────────────────────

/// Channel index [0..3] for Muse 4-channel layout.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ChannelId(u8);

#[allow(dead_code)]
impl ChannelId {
    pub const TP9: Self = Self(0);
    pub const AF7: Self = Self(1);
    pub const AF8: Self = Self(2);
    pub const TP10: Self = Self(3);
    pub const ALL: [Self; 4] = [Self::TP9, Self::AF7, Self::AF8, Self::TP10];

    pub fn new(idx: usize) -> Option<Self> {
        if idx < 4 {
            Some(Self(idx as u8))
        } else {
            None
        }
    }

    pub fn index(self) -> usize {
        self.0 as usize
    }
}

// ── VizMode ──────────────────────────────────────────────────────────────────

/// Which visualization mode is active in the UI.
#[derive(Clone, Copy, PartialEq, Eq, Default)]
pub enum VizMode {
    #[default]
    Waveforms,
    NeuralAura,
    BrainTopography,
}

// ── Dimension Reading ────────────────────────────────────────────────────────

/// A resolved dimension measurement result.
#[derive(Debug, Clone)]
pub struct DimensionValue {
    /// Display label: "Deep", "Surface", "Porous", "Boundaried", "Lean", "Hold"
    pub label: &'static str,
    /// 0.0-1.0 fill fraction for the arc gauge
    pub gauge_fraction: f32,
    /// Human-readable center text: "+32.5%", "SNR 2.3", "+0.15"
    pub display_text: String,
}

/// State of a single dimension measurement.
/// Each variant carries exactly the data available in that state.
#[derive(Debug, Clone)]
pub enum DimensionReading {
    Idle,
    Settling { progress: f32 },
    Measuring { progress: f32, live: Option<DimensionValue>, elapsed_s: Option<f32> },
    Complete(DimensionValue, Option<f32>),
}

impl DimensionReading {
    pub fn is_active(&self) -> bool {
        matches!(self, Self::Settling { .. } | Self::Measuring { .. })
    }

    pub fn progress(&self) -> f32 {
        match self {
            Self::Settling { progress } | Self::Measuring { progress, .. } => *progress,
            _ => 0.0,
        }
    }

    pub fn display_value(&self) -> Option<&DimensionValue> {
        match self {
            Self::Complete(v, _) => Some(v),
            Self::Measuring { live: Some(v), .. } => Some(v),
            _ => None,
        }
    }
}

// ── Dimension Meta ───────────────────────────────────────────────────────────

/// Static identity of a psychological dimension.
#[derive(Debug, Clone, Copy)]
pub struct DimensionMeta {
    pub name: &'static str,
    pub color: egui::Color32,
}

use eframe::egui;

pub const DIM_ABSORPTION: DimensionMeta = DimensionMeta {
    name: "ABSORPTION",
    color: egui::Color32::from_rgb(0, 220, 255),
};

pub const DIM_ATTUNEMENT: DimensionMeta = DimensionMeta {
    name: "ATTUNEMENT",
    color: egui::Color32::from_rgb(255, 0, 200),
};

pub const DIM_UNKNOWN: DimensionMeta = DimensionMeta {
    name: "THE UNKNOWN",
    color: egui::Color32::from_rgb(255, 180, 0),
};

pub const DIM_WITNESSED: DimensionMeta = DimensionMeta {
    name: "WITNESSED",
    color: egui::Color32::from_rgb(0, 255, 120),
};

// ── Dimension Card ───────────────────────────────────────────────────────────

/// Everything the UI needs to render one dimension gauge card.
#[derive(Clone)]
pub struct DimensionCard {
    pub meta: DimensionMeta,
    pub reading: DimensionReading,
    pub history: Vec<f32>,
}

impl Default for DimensionCard {
    fn default() -> Self {
        Self {
            meta: DIM_ABSORPTION,
            reading: DimensionReading::Idle,
            history: Vec::new(),
        }
    }
}

// ── Electrode State ──────────────────────────────────────────────────────────

/// Per-electrode data for the head map visualization.
#[derive(Clone)]
pub struct ElectrodeState {
    pub alpha_power: [f32; 4],
    pub contact: [ContactQuality; 4],
    /// (AF8 - AF7) / (AF8 + AF7), range [-1, +1]
    pub frontal_asymmetry: f32,
}

impl Default for ElectrodeState {
    fn default() -> Self {
        Self {
            alpha_power: [0.0; 4],
            contact: [ContactQuality::NoContact; 4],
            frontal_asymmetry: 0.0,
        }
    }
}

// ── Analysis Frame ───────────────────────────────────────────────────────────

/// Complete analysis snapshot that crosses the mutex boundary.
/// Cloned once per UI frame.
#[derive(Clone)]
pub struct AnalysisFrame {
    pub dimensions: [DimensionCard; 4],
    pub electrodes: ElectrodeState,
}

impl Default for AnalysisFrame {
    fn default() -> Self {
        Self {
            dimensions: [
                DimensionCard { meta: DIM_ABSORPTION, ..Default::default() },
                DimensionCard { meta: DIM_ATTUNEMENT, ..Default::default() },
                DimensionCard { meta: DIM_UNKNOWN, ..Default::default() },
                DimensionCard { meta: DIM_WITNESSED, ..Default::default() },
            ],
            electrodes: ElectrodeState::default(),
        }
    }
}

impl AnalysisFrame {
    /// True if any dimension detector is in Settling or Measuring.
    pub fn has_active_detectors(&self) -> bool {
        self.dimensions.iter().any(|d| d.reading.is_active())
    }
}

// ── Pipeline Command ─────────────────────────────────────────────────────────

/// Commands from UI thread to signal pipeline in BLE task.
#[derive(Debug, Clone, Copy)]
#[allow(dead_code)]
pub enum PipelineCommand {
    StartAlphaTrend,
    StopAlphaTrend,
    StartEntrainment,
    StopEntrainment,
    StartUnknown,
    StopUnknown,
    StartWitnessed,
    StopWitnessed,
    StartAll,
    StopAll,
    SetContinuous(bool),
}

// ── Semantic Functions (pure, testable) ──────────────────────────────────────

fn ring_push(buf: &mut VecDeque<f64>, value: f64, capacity: usize) {
    if buf.len() >= capacity {
        buf.pop_front();
    }
    buf.push_back(value);
}

fn extract_window(buf: &VecDeque<f64>, len: usize) -> Vec<f64> {
    let start = buf.len().saturating_sub(len);
    buf.iter().skip(start).copied().collect()
}

/// Map alpha trend result to a DimensionValue.
/// Gauge range: -10 to +10 %/min, center at 0.5.
fn trend_to_value(result: &muse_rs::alpha_trend::AlphaTrendResult) -> DimensionValue {
    let label = match result.label {
        muse_rs::alpha_trend::TrendLabel::Rising => "Deep",
        muse_rs::alpha_trend::TrendLabel::Flat => "Surface",
        muse_rs::alpha_trend::TrendLabel::Falling => "Surface",
    };
    let fraction = ((result.trend_pct_per_min + 10.0) / 20.0).clamp(0.0, 1.0) as f32;
    DimensionValue {
        label,
        gauge_fraction: fraction,
        display_text: format!("{:+.1}%/min", result.trend_pct_per_min),
    }
}

/// Map entrainment SNR to a DimensionValue.
/// Gauge range: 0.0 to 5.0, threshold at 1.5.
fn entrainment_to_value(snr: f64, threshold: f64) -> DimensionValue {
    let label = if snr >= threshold {
        "Porous"
    } else {
        "Boundaried"
    };
    let fraction = (snr / 5.0).clamp(0.0, 1.0) as f32;
    DimensionValue {
        label,
        gauge_fraction: fraction,
        display_text: format!("SNR {snr:.2}"),
    }
}

/// Map frontal asymmetry to Unknown dimension value.
fn asymmetry_to_unknown_value(asymmetry: f64, deadband: f64) -> DimensionValue {
    let label = if asymmetry > deadband {
        "Lean In"
    } else if asymmetry < -deadband {
        "Hold Back"
    } else {
        "Neutral"
    };
    let fraction = ((asymmetry + 0.5) / 1.0).clamp(0.0, 1.0) as f32;
    DimensionValue {
        label,
        gauge_fraction: fraction,
        display_text: format!("{asymmetry:+.3}"),
    }
}

/// Map frontal asymmetry to Witnessed dimension value.
fn asymmetry_to_witnessed_value(asymmetry: f64, deadband: f64) -> DimensionValue {
    let label = if asymmetry > deadband {
        "Approach"
    } else if asymmetry < -deadband {
        "Withdraw"
    } else {
        "Neutral"
    };
    let fraction = ((asymmetry + 0.5) / 1.0).clamp(0.0, 1.0) as f32;
    DimensionValue {
        label,
        gauge_fraction: fraction,
        display_text: format!("{asymmetry:+.3}"),
    }
}

// ── Detector Snapshots (read-only, pure transforms) ──────────────────────────

fn snapshot_alpha_trend(det: &AlphaTrendDetector, elapsed: Option<f32>) -> DimensionReading {
    match &det.phase {
        AlphaTrendPhase::Idle => DimensionReading::Idle,
        AlphaTrendPhase::Settling => {
            let progress = (det.elapsed_s() / det.config.settling_duration_s).clamp(0.0, 1.0);
            DimensionReading::Settling {
                progress: progress as f32,
            }
        }
        AlphaTrendPhase::Measuring => DimensionReading::Measuring {
            progress: 0.0,
            live: det.result.as_ref().map(trend_to_value),
            elapsed_s: elapsed,
        },
        AlphaTrendPhase::Complete => {
            let result = det.result.as_ref().unwrap();
            DimensionReading::Complete(trend_to_value(result), elapsed)
        }
    }
}

fn snapshot_entrainment(det: &EntrainmentDetector, elapsed: Option<f32>) -> DimensionReading {
    match &det.phase {
        EntrainmentPhase::Idle => DimensionReading::Idle,
        EntrainmentPhase::Settling => DimensionReading::Settling {
            progress: det.progress() as f32,
        },
        EntrainmentPhase::Measuring => DimensionReading::Measuring {
            progress: det.progress() as f32,
            live: {
                let snr = det.live_snr();
                if snr > 0.0 {
                    Some(entrainment_to_value(snr, det.config.snr_threshold))
                } else {
                    None
                }
            },
            elapsed_s: elapsed,
        },
        EntrainmentPhase::Complete => {
            let result = det.result.as_ref().unwrap();
            DimensionReading::Complete(entrainment_to_value(
                result.mean_snr,
                result.snr_threshold,
            ), elapsed)
        }
    }
}

fn snapshot_unknown(det: &ApproachDetector, elapsed: Option<f32>) -> DimensionReading {
    match &det.phase {
        ApproachPhase::Idle => DimensionReading::Idle,
        ApproachPhase::Settling => DimensionReading::Settling {
            progress: det.progress() as f32,
        },
        ApproachPhase::Measuring => DimensionReading::Measuring {
            progress: det.progress() as f32,
            live: det
                .live_asymmetry()
                .map(|a| asymmetry_to_unknown_value(a, det.config.deadband)),
            elapsed_s: elapsed,
        },
        ApproachPhase::Complete => {
            let result = det.result.as_ref().unwrap();
            DimensionReading::Complete(asymmetry_to_unknown_value(
                result.asymmetry,
                result.deadband,
            ), elapsed)
        }
    }
}

fn snapshot_witnessed(det: &ApproachDetector, elapsed: Option<f32>) -> DimensionReading {
    match &det.phase {
        ApproachPhase::Idle => DimensionReading::Idle,
        ApproachPhase::Settling => DimensionReading::Settling {
            progress: det.progress() as f32,
        },
        ApproachPhase::Measuring => DimensionReading::Measuring {
            progress: det.progress() as f32,
            live: det
                .live_asymmetry()
                .map(|a| asymmetry_to_witnessed_value(a, det.config.deadband)),
            elapsed_s: elapsed,
        },
        ApproachPhase::Complete => {
            let result = det.result.as_ref().unwrap();
            DimensionReading::Complete(asymmetry_to_witnessed_value(
                result.asymmetry,
                result.deadband,
            ), elapsed)
        }
    }
}

fn extract_channel_alpha(fft: &FftSnapshot) -> [f32; 4] {
    std::array::from_fn(|ch| fft.channels[ch].alpha_power)
}

fn extract_frontal_asymmetry(fft: &FftSnapshot) -> f32 {
    fft.frontal_asymmetry
}

// ── Assemble Analysis Frame ──────────────────────────────────────────────────

fn assemble_analysis_frame(
    alpha_trend: &AlphaTrendDetector,
    entrainment: &EntrainmentDetector,
    approach_unknown: &ApproachDetector,
    approach_witnessed: &ApproachDetector,
    fft: &FftSnapshot,
    contact: &[ContactQuality; 4],
    histories: &DimensionHistories,
    elapsed: [Option<f32>; 4],
) -> AnalysisFrame {
    AnalysisFrame {
        dimensions: [
            DimensionCard {
                meta: DIM_ABSORPTION,
                reading: snapshot_alpha_trend(alpha_trend, elapsed[0]),
                history: histories.absorption.iter().copied().collect(),
            },
            DimensionCard {
                meta: DIM_ATTUNEMENT,
                reading: snapshot_entrainment(entrainment, elapsed[1]),
                history: histories.entrainment.iter().copied().collect(),
            },
            DimensionCard {
                meta: DIM_UNKNOWN,
                reading: snapshot_unknown(approach_unknown, elapsed[2]),
                history: histories.unknown.iter().copied().collect(),
            },
            DimensionCard {
                meta: DIM_WITNESSED,
                reading: snapshot_witnessed(approach_witnessed, elapsed[3]),
                history: histories.witnessed.iter().copied().collect(),
            },
        ],
        electrodes: ElectrodeState {
            alpha_power: extract_channel_alpha(fft),
            contact: *contact,
            frontal_asymmetry: extract_frontal_asymmetry(fft),
        },
    }
}

// ── Signal Pipeline ──────────────────────────────────────────────────────────

struct DimensionHistories {
    absorption: VecDeque<f32>,
    entrainment: VecDeque<f32>,
    unknown: VecDeque<f32>,
    witnessed: VecDeque<f32>,
}

pub struct SignalPipeline {
    filters: [ChannelFilter; 4],
    validators: [EpochValidator; 4],
    contact_trackers: [ContactQualityTracker; 4],
    contact: [ContactQuality; 4],

    raw_bufs: [VecDeque<f64>; 4],
    filtered_bufs: [VecDeque<f64>; 4],

    samples_since_hop: usize,

    alpha_trend: AlphaTrendDetector,
    entrainment: EntrainmentDetector,
    approach_unknown: ApproachDetector,
    approach_witnessed: ApproachDetector,

    histories: DimensionHistories,
    last_fft: Option<FftSnapshot>,

    /// When true, skip contact quality and epoch validation
    /// (reconstructed data is already processed by the inference server).
    trust_input: bool,

    continuous: bool,
    /// Per-detector start times: [absorption, attunement, unknown, witnessed]
    detector_start_times: [Option<std::time::Instant>; 4],
    /// Frozen elapsed seconds after stop (overrides live elapsed from start_times)
    detector_frozen_elapsed: [Option<f32>; 4],
}

impl SignalPipeline {
    pub fn new() -> Self {
        Self {
            filters: std::array::from_fn(|_| ChannelFilter::new(FS)),
            validators: std::array::from_fn(|_| EpochValidator::default()),
            contact_trackers: std::array::from_fn(|i| ContactQualityTracker::new(i)),
            contact: [ContactQuality::NoContact; 4],
            raw_bufs: std::array::from_fn(|_| VecDeque::with_capacity(BUF_CAP)),
            filtered_bufs: std::array::from_fn(|_| VecDeque::with_capacity(BUF_CAP)),
            samples_since_hop: 0,
            alpha_trend: AlphaTrendDetector::default(),
            entrainment: EntrainmentDetector::default(),
            approach_unknown: ApproachDetector::default(),
            approach_witnessed: ApproachDetector::default(),
            histories: DimensionHistories {
                absorption: VecDeque::with_capacity(HISTORY_CAP),
                entrainment: VecDeque::with_capacity(HISTORY_CAP),
                unknown: VecDeque::with_capacity(HISTORY_CAP),
                witnessed: VecDeque::with_capacity(HISTORY_CAP),
            },
            last_fft: None,
            trust_input: false,
            continuous: false,
            detector_start_times: [None; 4],
            detector_frozen_elapsed: [None; 4],
        }
    }

    /// Create a pipeline for server-reconstructed EEG data.
    ///
    /// The inference server already denoises and validates the signal,
    /// so raw-BLE artifact checks (contact quality, 150 µV amplitude
    /// limit) are inappropriate and would reject valid reconstructed
    /// epochs.  This constructor disables those checks.
    pub fn new_for_reconstructed() -> Self {
        let mut p = Self::new();
        p.trust_input = true;
        p
    }

    /// Ingest raw EEG samples for one channel.
    /// Filters each sample and stores both raw and filtered in ring buffers.
    pub fn ingest_samples(&mut self, channel: ChannelId, samples: &[f64]) {
        let ch = channel.index();
        for &s in samples {
            let filtered = self.filters[ch].process(s);
            ring_push(&mut self.raw_bufs[ch], s, BUF_CAP);
            ring_push(&mut self.filtered_bufs[ch], filtered, BUF_CAP);
        }
        // Only count hops on TP9 to avoid double-counting
        if channel == ChannelId::TP9 {
            self.samples_since_hop += samples.len();
        }
    }

    /// Try to produce an analysis frame. Returns Some when enough samples
    /// have accumulated since the last hop and all channels have sufficient data.
    pub fn try_analyze(&mut self) -> Option<AnalysisFrame> {
        if self.samples_since_hop < HOP_SAMPLES {
            return None;
        }
        for ch in 0..4 {
            if self.filtered_bufs[ch].len() < FFT_WINDOW {
                return None;
            }
        }
        self.samples_since_hop -= HOP_SAMPLES;

        // Extract 512-sample windows
        let windows: [Vec<f64>; 4] =
            std::array::from_fn(|ch| extract_window(&self.filtered_bufs[ch], FFT_WINDOW));

        let epoch_ok;
        if self.trust_input {
            // Reconstructed data: trust the inference server's processing
            self.contact = [ContactQuality::Good; 4];
            epoch_ok = [true; 4];
        } else {
            // Raw BLE data: run contact quality and epoch validation
            for ch in 0..4 {
                let n = self.raw_bufs[ch].len().min(256);
                if n > 0 {
                    let samples: Vec<f64> = self.raw_bufs[ch].iter().rev().take(n).copied().collect();
                    self.contact[ch] = self.contact_trackers[ch].update(&samples);
                }
            }
            let mut ok = [false; 4];
            for ch in 0..4 {
                let bad_contact = self.contact[ch] == ContactQuality::NoContact;
                ok[ch] = matches!(
                    self.validators[ch].validate(&windows[ch], bad_contact),
                    EpochStatus::Clean
                );
            }
            epoch_ok = ok;
        }

        // FFT
        let window_refs: Vec<&[f64]> = windows.iter().map(|w| w.as_slice()).collect();
        let fft = FftSnapshot::from_windows(&window_refs, FS);
        self.last_fft = Some(fft.clone());

        // Feed running detectors
        self.alpha_trend.feed(&fft, &epoch_ok, &self.contact);
        self.approach_unknown.feed(&fft, &epoch_ok, &self.contact);
        self.approach_witnessed.feed(&fft, &epoch_ok, &self.contact);

        // Entrainment needs 1024-sample windows
        if self.entrainment.is_running() {
            let has_enough = (0..4).all(|ch| self.filtered_bufs[ch].len() >= ENT_WINDOW);
            if has_enough {
                let ent_windows: [Vec<f64>; 4] =
                    std::array::from_fn(|ch| extract_window(&self.filtered_bufs[ch], ENT_WINDOW));
                let ent_refs: Vec<&[f64]> = ent_windows.iter().map(|w| w.as_slice()).collect();
                self.entrainment.feed(&ent_refs, &epoch_ok, &self.contact);
            }
        }

        // Update sparkline histories
        self.push_history_values();

        // Assemble frame
        let elapsed = self.detector_elapsed_s();
        let frame = assemble_analysis_frame(
            &self.alpha_trend,
            &self.entrainment,
            &self.approach_unknown,
            &self.approach_witnessed,
            &fft,
            &self.contact,
            &self.histories,
            elapsed,
        );
        Some(frame)
    }

    fn push_history_values(&mut self) {
        fn push_val(history: &mut VecDeque<f32>, reading: &DimensionReading, cap: usize) {
            let val = reading
                .display_value()
                .map(|v| v.gauge_fraction)
                .unwrap_or(0.0);
            if history.len() >= cap {
                history.pop_front();
            }
            history.push_back(val);
        }
        push_val(
            &mut self.histories.absorption,
            &snapshot_alpha_trend(&self.alpha_trend, None),
            HISTORY_CAP,
        );
        push_val(
            &mut self.histories.entrainment,
            &snapshot_entrainment(&self.entrainment, None),
            HISTORY_CAP,
        );
        push_val(
            &mut self.histories.unknown,
            &snapshot_unknown(&self.approach_unknown, None),
            HISTORY_CAP,
        );
        push_val(
            &mut self.histories.witnessed,
            &snapshot_witnessed(&self.approach_witnessed, None),
            HISTORY_CAP,
        );
    }

    pub fn execute_command(&mut self, cmd: PipelineCommand) -> Result<(), &'static str> {
        let now = std::time::Instant::now();
        match cmd {
            PipelineCommand::StartAlphaTrend => {
                self.alpha_trend.start();
                self.detector_start_times[0] = Some(now);
                self.detector_frozen_elapsed[0] = None;
                Ok(())
            }
            PipelineCommand::StopAlphaTrend => {
                self.freeze_elapsed(0);
                self.alpha_trend.stop();
                Ok(())
            }
            PipelineCommand::StartEntrainment => {
                self.entrainment
                    .start()
                    .map_err(|_| "ACT engine creation failed")?;
                self.detector_start_times[1] = Some(now);
                self.detector_frozen_elapsed[1] = None;
                Ok(())
            }
            PipelineCommand::StopEntrainment => {
                self.freeze_elapsed(1);
                self.entrainment.stop();
                Ok(())
            }
            PipelineCommand::StartUnknown => {
                self.approach_unknown.start();
                self.detector_start_times[2] = Some(now);
                self.detector_frozen_elapsed[2] = None;
                Ok(())
            }
            PipelineCommand::StopUnknown => {
                self.freeze_elapsed(2);
                self.approach_unknown.stop();
                Ok(())
            }
            PipelineCommand::StartWitnessed => {
                self.approach_witnessed.start();
                self.detector_start_times[3] = Some(now);
                self.detector_frozen_elapsed[3] = None;
                Ok(())
            }
            PipelineCommand::StopWitnessed => {
                self.freeze_elapsed(3);
                self.approach_witnessed.stop();
                Ok(())
            }
            PipelineCommand::StartAll => self.start_all_detectors(),
            PipelineCommand::StopAll => {
                self.stop_all_detectors();
                Ok(())
            }
            PipelineCommand::SetContinuous(on) => {
                self.set_continuous(on);
                Ok(())
            }
        }
    }

    /// Start all four dimension detectors. In continuous mode, sets
    /// measuring windows/snapshots to usize::MAX so they never auto-complete.
    pub fn start_all_detectors(&mut self) -> Result<(), &'static str> {
        if self.continuous {
            self.entrainment.config.measuring_windows = usize::MAX;
            self.approach_unknown.config.measuring_snapshots = usize::MAX;
            self.approach_witnessed.config.measuring_snapshots = usize::MAX;
        }
        let now = std::time::Instant::now();
        self.alpha_trend.start();
        self.entrainment
            .start()
            .map_err(|_| "ACT engine creation failed")?;
        self.approach_unknown.start();
        self.approach_witnessed.start();
        self.detector_start_times = [Some(now); 4];
        self.detector_frozen_elapsed = [None; 4];
        Ok(())
    }

    /// Stop all detectors that are currently running (Settling or Measuring).
    /// Freezes elapsed times so they stop ticking.
    pub fn stop_all_detectors(&mut self) {
        for i in 0..4 {
            self.freeze_elapsed(i);
        }
        if self.alpha_trend.is_running() {
            self.alpha_trend.stop();
        }
        if self.entrainment.is_running() {
            self.entrainment.stop();
        }
        if self.approach_unknown.is_running() {
            self.approach_unknown.stop();
        }
        if self.approach_witnessed.is_running() {
            self.approach_witnessed.stop();
        }
    }

    pub fn set_continuous(&mut self, on: bool) {
        self.continuous = on;
        if on {
            self.entrainment.config.measuring_windows = usize::MAX;
            self.approach_unknown.config.measuring_snapshots = usize::MAX;
            self.approach_witnessed.config.measuring_snapshots = usize::MAX;
        } else {
            self.entrainment.config.measuring_windows = 160;
            self.approach_unknown.config.measuring_snapshots = 80;
            self.approach_witnessed.config.measuring_snapshots = 80;
        }
    }

    /// Freeze the elapsed time for detector `idx` so it stops ticking.
    fn freeze_elapsed(&mut self, idx: usize) {
        if let Some(t) = self.detector_start_times[idx] {
            self.detector_frozen_elapsed[idx] = Some(t.elapsed().as_secs_f32());
        }
    }

    /// Per-detector elapsed seconds: [absorption, attunement, unknown, witnessed].
    /// Returns frozen elapsed after stop, live elapsed while running.
    pub fn detector_elapsed_s(&self) -> [Option<f32>; 4] {
        std::array::from_fn(|i| {
            self.detector_frozen_elapsed[i].or_else(|| {
                self.detector_start_times[i].map(|t| t.elapsed().as_secs_f32())
            })
        })
    }

    /// Snapshot the current dimension readings from all detectors.
    /// Used to update the cached analysis after stop without waiting for
    /// the next try_analyze() call.
    pub fn snapshot_readings(&self) -> [DimensionReading; 4] {
        let elapsed = self.detector_elapsed_s();
        [
            snapshot_alpha_trend(&self.alpha_trend, elapsed[0]),
            snapshot_entrainment(&self.entrainment, elapsed[1]),
            snapshot_unknown(&self.approach_unknown, elapsed[2]),
            snapshot_witnessed(&self.approach_witnessed, elapsed[3]),
        ]
    }
}

// ── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── ChannelId ────────────────────────────────────────────────────────

    #[test]
    fn sp_channel_id_valid_range() {
        assert!(ChannelId::new(0).is_some());
        assert!(ChannelId::new(3).is_some());
        assert!(ChannelId::new(4).is_none());
    }

    #[test]
    fn sp_channel_id_constants() {
        assert_eq!(ChannelId::TP9.index(), 0);
        assert_eq!(ChannelId::AF7.index(), 1);
        assert_eq!(ChannelId::AF8.index(), 2);
        assert_eq!(ChannelId::TP10.index(), 3);
    }

    #[test]
    fn sp_channel_id_all() {
        assert_eq!(ChannelId::ALL.len(), 4);
        for (i, ch) in ChannelId::ALL.iter().enumerate() {
            assert_eq!(ch.index(), i);
        }
    }

    // ── ring_push ────────────────────────────────────────────────────────

    #[test]
    fn sp_ring_push_within_capacity() {
        let mut buf = VecDeque::new();
        ring_push(&mut buf, 1.0, 5);
        ring_push(&mut buf, 2.0, 5);
        assert_eq!(buf.len(), 2);
        assert_eq!(buf[0], 1.0);
    }

    #[test]
    fn sp_ring_push_at_capacity_evicts_oldest() {
        let mut buf = VecDeque::new();
        for i in 0..5 {
            ring_push(&mut buf, i as f64, 3);
        }
        assert_eq!(buf.len(), 3);
        assert_eq!(buf[0], 2.0);
        assert_eq!(buf[2], 4.0);
    }

    // ── extract_window ───────────────────────────────────────────────────

    #[test]
    fn sp_extract_window_from_full_buffer() {
        let mut buf = VecDeque::new();
        for i in 0..10 {
            buf.push_back(i as f64);
        }
        let win = extract_window(&buf, 4);
        assert_eq!(win, vec![6.0, 7.0, 8.0, 9.0]);
    }

    #[test]
    fn sp_extract_window_buffer_smaller_than_len() {
        let mut buf = VecDeque::new();
        buf.push_back(1.0);
        buf.push_back(2.0);
        let win = extract_window(&buf, 5);
        assert_eq!(win, vec![1.0, 2.0]);
    }

    // ── Value mapping ────────────────────────────────────────────────────

    #[test]
    fn sp_trend_value_rising() {
        let result = muse_rs::alpha_trend::AlphaTrendResult {
            slope: 0.01,
            mean_alpha: 1.0,
            trend_pct_per_min: 5.0,
            r_squared: 0.9,
            label: muse_rs::alpha_trend::TrendLabel::Rising,
            deadband: 2.0,
            n_samples: 100,
            duration_s: 60.0,
            artifact_ratio: 0.05,
        };
        let v = trend_to_value(&result);
        assert_eq!(v.label, "Deep");
        // 5.0 maps to (5+10)/20 = 0.75
        assert!((v.gauge_fraction - 0.75).abs() < 0.01);
    }

    #[test]
    fn sp_trend_value_falling() {
        let result = muse_rs::alpha_trend::AlphaTrendResult {
            slope: -0.01,
            mean_alpha: 1.0,
            trend_pct_per_min: -5.0,
            r_squared: 0.9,
            label: muse_rs::alpha_trend::TrendLabel::Falling,
            deadband: 2.0,
            n_samples: 100,
            duration_s: 60.0,
            artifact_ratio: 0.05,
        };
        let v = trend_to_value(&result);
        assert_eq!(v.label, "Surface");
        // -5.0 maps to (-5+10)/20 = 0.25
        assert!((v.gauge_fraction - 0.25).abs() < 0.01);
    }

    #[test]
    fn sp_trend_value_flat() {
        let result = muse_rs::alpha_trend::AlphaTrendResult {
            slope: 0.0,
            mean_alpha: 1.0,
            trend_pct_per_min: 0.0,
            r_squared: 0.0,
            label: muse_rs::alpha_trend::TrendLabel::Flat,
            deadband: 2.0,
            n_samples: 100,
            duration_s: 60.0,
            artifact_ratio: 0.0,
        };
        let v = trend_to_value(&result);
        assert_eq!(v.label, "Surface");
        // 0.0 maps to (0+10)/20 = 0.5
        assert!((v.gauge_fraction - 0.5).abs() < 0.01);
    }

    #[test]
    fn sp_entrainment_value_porous() {
        let v = entrainment_to_value(2.0, 1.5);
        assert_eq!(v.label, "Porous");
        assert!((v.gauge_fraction - 0.4).abs() < 0.01);
    }

    #[test]
    fn sp_entrainment_value_boundaried() {
        let v = entrainment_to_value(1.0, 1.5);
        assert_eq!(v.label, "Boundaried");
    }

    #[test]
    fn sp_unknown_value_lean_in() {
        let v = asymmetry_to_unknown_value(0.2, 0.1);
        assert_eq!(v.label, "Lean In");
    }

    #[test]
    fn sp_unknown_value_hold_back() {
        let v = asymmetry_to_unknown_value(-0.2, 0.1);
        assert_eq!(v.label, "Hold Back");
    }

    #[test]
    fn sp_unknown_value_neutral() {
        let v = asymmetry_to_unknown_value(0.05, 0.1);
        assert_eq!(v.label, "Neutral");
    }

    #[test]
    fn sp_witnessed_value_approach() {
        let v = asymmetry_to_witnessed_value(0.2, 0.1);
        assert_eq!(v.label, "Approach");
    }

    #[test]
    fn sp_witnessed_value_withdraw() {
        let v = asymmetry_to_witnessed_value(-0.2, 0.1);
        assert_eq!(v.label, "Withdraw");
    }

    #[test]
    fn sp_witnessed_value_neutral() {
        let v = asymmetry_to_witnessed_value(0.05, 0.1);
        assert_eq!(v.label, "Neutral");
    }

    #[test]
    fn sp_asymmetry_gauge_centered() {
        // asymmetry = 0 should map to gauge fraction 0.5
        let v = asymmetry_to_unknown_value(0.0, 0.1);
        assert!((v.gauge_fraction - 0.5).abs() < 0.01);
    }

    // ── DimensionReading ─────────────────────────────────────────────────

    #[test]
    fn sp_reading_idle_no_value() {
        let r = DimensionReading::Idle;
        assert!(!r.is_active());
        assert!(r.display_value().is_none());
        assert_eq!(r.progress(), 0.0);
    }

    #[test]
    fn sp_reading_settling_is_active() {
        let r = DimensionReading::Settling { progress: 0.5 };
        assert!(r.is_active());
        assert_eq!(r.progress(), 0.5);
        assert!(r.display_value().is_none());
    }

    #[test]
    fn sp_reading_measuring_with_live() {
        let v = DimensionValue {
            label: "Test",
            gauge_fraction: 0.7,
            display_text: "test".into(),
        };
        let r = DimensionReading::Measuring {
            progress: 0.8,
            live: Some(v),
            elapsed_s: None,
        };
        assert!(r.is_active());
        assert_eq!(r.progress(), 0.8);
        assert_eq!(r.display_value().unwrap().gauge_fraction, 0.7);
    }

    #[test]
    fn sp_reading_complete_has_value() {
        let v = DimensionValue {
            label: "Done",
            gauge_fraction: 1.0,
            display_text: "done".into(),
        };
        let r = DimensionReading::Complete(v, None);
        assert!(!r.is_active());
        assert_eq!(r.display_value().unwrap().label, "Done");
    }

    // ── Pipeline ─────────────────────────────────────────────────────────

    #[test]
    fn sp_pipeline_no_analysis_without_enough_data() {
        let mut pipeline = SignalPipeline::new();
        // Feed fewer than HOP_SAMPLES on TP9
        pipeline.ingest_samples(ChannelId::TP9, &[0.0; 32]);
        assert!(pipeline.try_analyze().is_none());
    }

    #[test]
    fn sp_pipeline_no_analysis_without_all_channels() {
        let mut pipeline = SignalPipeline::new();
        // Feed enough on TP9 but nothing on other channels
        pipeline.ingest_samples(ChannelId::TP9, &[0.0; 128]);
        assert!(pipeline.try_analyze().is_none());
    }

    #[test]
    fn sp_pipeline_produces_frame_when_ready() {
        let mut pipeline = SignalPipeline::new();
        // Feed enough data on all 4 channels
        let signal: Vec<f64> = (0..FFT_WINDOW + HOP_SAMPLES)
            .map(|i| (i as f64 * 10.0 * std::f64::consts::TAU / FS).sin() * 50.0)
            .collect();
        for ch in ChannelId::ALL {
            pipeline.ingest_samples(ch, &signal);
        }
        let frame = pipeline.try_analyze();
        assert!(frame.is_some(), "should produce a frame when all channels have enough data");
        let frame = frame.unwrap();
        // All dimensions should be idle
        for dim in &frame.dimensions {
            assert!(matches!(dim.reading, DimensionReading::Idle));
        }
    }

    #[test]
    fn sp_pipeline_hop_drains_excess_samples() {
        let mut pipeline = SignalPipeline::new();
        // Feed exactly FFT_WINDOW + HOP_SAMPLES (576 = 512 + 64)
        let signal: Vec<f64> = vec![0.0; FFT_WINDOW + HOP_SAMPLES];
        for ch in ChannelId::ALL {
            pipeline.ingest_samples(ch, &signal);
        }
        // First analysis consumes 64 of the 576 hop-samples
        assert!(pipeline.try_analyze().is_some());
        // 512 remain — should keep producing frames until < HOP_SAMPLES
        let mut extra = 0;
        while pipeline.try_analyze().is_some() {
            extra += 1;
        }
        // 576 / 64 = 9 total frames; first consumed above, so 8 extra
        assert_eq!(extra, (FFT_WINDOW + HOP_SAMPLES) / HOP_SAMPLES - 1);
    }

    #[test]
    fn sp_pipeline_command_alpha_trend() {
        let mut pipeline = SignalPipeline::new();
        assert!(pipeline.execute_command(PipelineCommand::StartAlphaTrend).is_ok());
        assert!(pipeline.execute_command(PipelineCommand::StopAlphaTrend).is_ok());
    }

    #[test]
    fn sp_pipeline_command_approach() {
        let mut pipeline = SignalPipeline::new();
        assert!(pipeline.execute_command(PipelineCommand::StartUnknown).is_ok());
        assert!(pipeline.execute_command(PipelineCommand::StartWitnessed).is_ok());
    }

    // ── AnalysisFrame default ────────────────────────────────────────────

    #[test]
    fn sp_analysis_frame_default_has_four_dimensions() {
        let frame = AnalysisFrame::default();
        assert_eq!(frame.dimensions.len(), 4);
        assert_eq!(frame.dimensions[0].meta.name, "ABSORPTION");
        assert_eq!(frame.dimensions[1].meta.name, "ATTUNEMENT");
        assert_eq!(frame.dimensions[2].meta.name, "THE UNKNOWN");
        assert_eq!(frame.dimensions[3].meta.name, "WITNESSED");
    }
}
