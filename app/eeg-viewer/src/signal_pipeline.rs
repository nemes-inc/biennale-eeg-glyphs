//! Signal processing pipeline for EEG dimension analysis.
//!
//! Separates pure semantic functions from I/O pragmatic shell.
//! All detectors run inside the BLE task, producing AnalysisFrame
//! snapshots that cross the mutex boundary to the UI thread.

use std::collections::VecDeque;

use muse_rs::alpha::FftSnapshot;
use muse_rs::absorption::{AbsorptionDetector, AbsorptionPhase};
use muse_rs::approach::{ApproachDetector, ApproachPhase};
use muse_rs::baseline::{BaselineDetector, BaselinePhase};
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
    Measuring { progress: f32, live: Option<DimensionValue> },
    Complete(DimensionValue),
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
            Self::Complete(v) => Some(v),
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

// ── Baseline State ───────────────────────────────────────────────────────────

/// Baseline is a prerequisite, not a dimension.
#[derive(Debug, Clone, Default)]
#[allow(dead_code)]
pub enum BaselineState {
    #[default]
    NotCollected,
    Collecting { progress: f32 },
    Collected,
    Failed(String),
}

// ── Analysis Frame ───────────────────────────────────────────────────────────

/// Complete analysis snapshot that crosses the mutex boundary.
/// Cloned once per UI frame.
#[derive(Clone)]
pub struct AnalysisFrame {
    pub dimensions: [DimensionCard; 4],
    pub electrodes: ElectrodeState,
    pub baseline_available: bool,
    pub baseline_state: BaselineState,
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
            baseline_available: false,
            baseline_state: BaselineState::NotCollected,
        }
    }
}

impl AnalysisFrame {
    pub fn baseline_state(&self) -> &BaselineState {
        &self.baseline_state
    }
}

// ── Pipeline Command ─────────────────────────────────────────────────────────

/// Commands from UI thread to signal pipeline in BLE task.
#[derive(Debug, Clone, Copy)]
pub enum PipelineCommand {
    StartBaseline,
    StopBaseline,
    StartAbsorption,
    StopAbsorption,
    StartEntrainment,
    StopEntrainment,
    StartUnknown,
    StopUnknown,
    StartWitnessed,
    StopWitnessed,
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

/// Map absorption percentage to a DimensionValue.
/// Gauge range: -20% to +50%, so 0% maps to ~0.29 fraction.
fn absorption_to_value(pct: f64, threshold: f64) -> DimensionValue {
    let label = if pct >= threshold { "Deep" } else { "Surface" };
    let fraction = ((pct - (-20.0)) / 70.0).clamp(0.0, 1.0) as f32;
    DimensionValue {
        label,
        gauge_fraction: fraction,
        display_text: format!("{pct:+.1}%"),
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

/// Map frontal asymmetry to a DimensionValue.
/// Center gauge at 0.5, positive right, negative left.
fn asymmetry_to_value(asymmetry: f64, deadband: f64) -> DimensionValue {
    let label = if asymmetry > deadband {
        "Lean"
    } else if asymmetry < -deadband {
        "Hold"
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

fn snapshot_absorption(det: &AbsorptionDetector) -> DimensionReading {
    match &det.phase {
        AbsorptionPhase::Idle | AbsorptionPhase::NeedBaseline => DimensionReading::Idle,
        AbsorptionPhase::Settling => DimensionReading::Settling {
            progress: det.progress() as f32,
        },
        AbsorptionPhase::Measuring => DimensionReading::Measuring {
            progress: det.progress() as f32,
            live: det
                .live_absorption_pct()
                .map(|pct| absorption_to_value(pct, det.config.threshold_pct)),
        },
        AbsorptionPhase::Complete => {
            let result = det.result.as_ref().unwrap();
            DimensionReading::Complete(absorption_to_value(
                result.absorption_pct,
                result.threshold_pct,
            ))
        }
    }
}

fn snapshot_entrainment(det: &EntrainmentDetector) -> DimensionReading {
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
        },
        EntrainmentPhase::Complete => {
            let result = det.result.as_ref().unwrap();
            DimensionReading::Complete(entrainment_to_value(
                result.mean_snr,
                result.snr_threshold,
            ))
        }
    }
}

fn snapshot_approach(det: &ApproachDetector) -> DimensionReading {
    match &det.phase {
        ApproachPhase::Idle => DimensionReading::Idle,
        ApproachPhase::Settling => DimensionReading::Settling {
            progress: det.progress() as f32,
        },
        ApproachPhase::Measuring => DimensionReading::Measuring {
            progress: det.progress() as f32,
            live: det
                .live_asymmetry()
                .map(|a| asymmetry_to_value(a, det.config.deadband)),
        },
        ApproachPhase::Complete => {
            let result = det.result.as_ref().unwrap();
            DimensionReading::Complete(asymmetry_to_value(
                result.asymmetry,
                result.deadband,
            ))
        }
    }
}

fn snapshot_baseline(det: &BaselineDetector) -> BaselineState {
    match &det.phase {
        BaselinePhase::Idle => {
            if det.result.is_some() {
                BaselineState::Collected
            } else {
                BaselineState::NotCollected
            }
        }
        BaselinePhase::Settling | BaselinePhase::Recording => {
            BaselineState::Collecting {
                progress: det.progress() as f32,
            }
        }
        BaselinePhase::Complete => BaselineState::Collected,
        BaselinePhase::Failed(msg) => BaselineState::Failed(msg.clone()),
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
    absorption: &AbsorptionDetector,
    entrainment: &EntrainmentDetector,
    approach_unknown: &ApproachDetector,
    approach_witnessed: &ApproachDetector,
    baseline: &BaselineDetector,
    fft: &FftSnapshot,
    contact: &[ContactQuality; 4],
    histories: &DimensionHistories,
) -> AnalysisFrame {
    AnalysisFrame {
        dimensions: [
            DimensionCard {
                meta: DIM_ABSORPTION,
                reading: snapshot_absorption(absorption),
                history: histories.absorption.iter().copied().collect(),
            },
            DimensionCard {
                meta: DIM_ATTUNEMENT,
                reading: snapshot_entrainment(entrainment),
                history: histories.entrainment.iter().copied().collect(),
            },
            DimensionCard {
                meta: DIM_UNKNOWN,
                reading: snapshot_approach(approach_unknown),
                history: histories.unknown.iter().copied().collect(),
            },
            DimensionCard {
                meta: DIM_WITNESSED,
                reading: snapshot_approach(approach_witnessed),
                history: histories.witnessed.iter().copied().collect(),
            },
        ],
        electrodes: ElectrodeState {
            alpha_power: extract_channel_alpha(fft),
            contact: *contact,
            frontal_asymmetry: extract_frontal_asymmetry(fft),
        },
        baseline_available: baseline.result.is_some(),
        baseline_state: snapshot_baseline(baseline),
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

    baseline: BaselineDetector,
    absorption: AbsorptionDetector,
    entrainment: EntrainmentDetector,
    approach_unknown: ApproachDetector,
    approach_witnessed: ApproachDetector,

    histories: DimensionHistories,
    last_fft: Option<FftSnapshot>,
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
            baseline: BaselineDetector::default(),
            absorption: AbsorptionDetector::default(),
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
        }
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
        self.samples_since_hop = 0;

        // Contact quality
        for ch in 0..4 {
            let n = self.raw_bufs[ch].len().min(256);
            if n > 0 {
                let samples: Vec<f64> = self.raw_bufs[ch].iter().rev().take(n).copied().collect();
                self.contact[ch] = self.contact_trackers[ch].update(&samples);
            }
        }

        // Extract 512-sample windows and validate epochs
        let windows: [Vec<f64>; 4] =
            std::array::from_fn(|ch| extract_window(&self.filtered_bufs[ch], FFT_WINDOW));
        let mut epoch_ok = [false; 4];
        for ch in 0..4 {
            let bad_contact = self.contact[ch] == ContactQuality::NoContact;
            epoch_ok[ch] = matches!(
                self.validators[ch].validate(&windows[ch], bad_contact),
                EpochStatus::Clean
            );
        }

        // FFT
        let window_refs: Vec<&[f64]> = windows.iter().map(|w| w.as_slice()).collect();
        let fft = FftSnapshot::from_windows(&window_refs, FS);
        self.last_fft = Some(fft.clone());

        // Feed running detectors
        self.baseline.feed(&fft, &epoch_ok, &self.contact);
        self.absorption.feed(&fft, &epoch_ok, &self.contact);
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
        let frame = assemble_analysis_frame(
            &self.absorption,
            &self.entrainment,
            &self.approach_unknown,
            &self.approach_witnessed,
            &self.baseline,
            &fft,
            &self.contact,
            &self.histories,
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
            &snapshot_absorption(&self.absorption),
            HISTORY_CAP,
        );
        push_val(
            &mut self.histories.entrainment,
            &snapshot_entrainment(&self.entrainment),
            HISTORY_CAP,
        );
        push_val(
            &mut self.histories.unknown,
            &snapshot_approach(&self.approach_unknown),
            HISTORY_CAP,
        );
        push_val(
            &mut self.histories.witnessed,
            &snapshot_approach(&self.approach_witnessed),
            HISTORY_CAP,
        );
    }

    pub fn execute_command(&mut self, cmd: PipelineCommand) -> Result<(), &'static str> {
        match cmd {
            PipelineCommand::StartBaseline => {
                self.baseline.start();
                Ok(())
            }
            PipelineCommand::StopBaseline => {
                self.baseline.stop();
                Ok(())
            }
            PipelineCommand::StartAbsorption => {
                let bl = self
                    .baseline
                    .result
                    .as_ref()
                    .ok_or("baseline required")?;
                self.absorption.start(bl);
                Ok(())
            }
            PipelineCommand::StopAbsorption => {
                self.absorption.stop();
                Ok(())
            }
            PipelineCommand::StartEntrainment => self
                .entrainment
                .start()
                .map_err(|_| "ACT engine creation failed"),
            PipelineCommand::StopEntrainment => {
                self.entrainment.stop();
                Ok(())
            }
            PipelineCommand::StartUnknown => {
                self.approach_unknown.start();
                Ok(())
            }
            PipelineCommand::StopUnknown => {
                self.approach_unknown.stop();
                Ok(())
            }
            PipelineCommand::StartWitnessed => {
                self.approach_witnessed.start();
                Ok(())
            }
            PipelineCommand::StopWitnessed => {
                self.approach_witnessed.stop();
                Ok(())
            }
        }
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
    fn sp_absorption_value_deep() {
        let v = absorption_to_value(30.0, 25.0);
        assert_eq!(v.label, "Deep");
        let expected_frac = (30.0 - (-20.0)) / 70.0;
        assert!((v.gauge_fraction - expected_frac as f32).abs() < 0.01);
    }

    #[test]
    fn sp_absorption_value_surface() {
        let v = absorption_to_value(10.0, 25.0);
        assert_eq!(v.label, "Surface");
    }

    #[test]
    fn sp_absorption_value_clamped_low() {
        let v = absorption_to_value(-30.0, 25.0);
        assert_eq!(v.gauge_fraction, 0.0);
    }

    #[test]
    fn sp_absorption_value_clamped_high() {
        let v = absorption_to_value(60.0, 25.0);
        assert_eq!(v.gauge_fraction, 1.0);
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
    fn sp_asymmetry_value_lean() {
        let v = asymmetry_to_value(0.2, 0.1);
        assert_eq!(v.label, "Lean");
    }

    #[test]
    fn sp_asymmetry_value_hold() {
        let v = asymmetry_to_value(-0.2, 0.1);
        assert_eq!(v.label, "Hold");
    }

    #[test]
    fn sp_asymmetry_value_neutral() {
        let v = asymmetry_to_value(0.05, 0.1);
        assert_eq!(v.label, "Neutral");
    }

    #[test]
    fn sp_asymmetry_gauge_centered() {
        // asymmetry = 0 should map to gauge fraction 0.5
        let v = asymmetry_to_value(0.0, 0.1);
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
        let r = DimensionReading::Complete(v);
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
    fn sp_pipeline_hop_resets_after_analysis() {
        let mut pipeline = SignalPipeline::new();
        let signal: Vec<f64> = vec![0.0; FFT_WINDOW + HOP_SAMPLES];
        for ch in ChannelId::ALL {
            pipeline.ingest_samples(ch, &signal);
        }
        // First analysis should succeed
        assert!(pipeline.try_analyze().is_some());
        // Immediately after, not enough new samples
        assert!(pipeline.try_analyze().is_none());
    }

    #[test]
    fn sp_pipeline_command_baseline() {
        let mut pipeline = SignalPipeline::new();
        assert!(pipeline.execute_command(PipelineCommand::StartBaseline).is_ok());
        assert!(pipeline.execute_command(PipelineCommand::StopBaseline).is_ok());
    }

    #[test]
    fn sp_pipeline_command_absorption_needs_baseline() {
        let mut pipeline = SignalPipeline::new();
        let result = pipeline.execute_command(PipelineCommand::StartAbsorption);
        assert!(result.is_err());
        assert_eq!(result.unwrap_err(), "baseline required");
    }

    #[test]
    fn sp_pipeline_command_approach_no_baseline_needed() {
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
        assert!(!frame.baseline_available);
    }
}
