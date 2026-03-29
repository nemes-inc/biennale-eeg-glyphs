//! Entrainment dimension — measures how readily the nervous system locks onto external rhythms.
//!
//! # Neural Signature
//!
//! Steady-state evoked potential (SSEP) at the beat frequency (default 2.0 Hz).
//!
//! # Method (ACT-based)
//!
//! Instead of FFT-based SNR, we use the Adaptive Chirplet Transform to detect
//! entrainment.  A dedicated ACT engine with a dictionary optimised for low
//! frequencies decomposes the EEG into chirplets.  Chirplets near the beat
//! frequency with near-zero chirp rate and long time-spread indicate
//! steady-state entrainment (per Cui & Wong 2006, 2008).
//!
//! # Scoring
//!
//! ```text
//! beat_energy  = Σ |coeff|² for chirplets with fc ∈ [beat - tol, beat + tol]
//!                                                  AND |chirp_rate| < max_cr
//! total_energy = Σ |coeff|² for all chirplets
//!
//! SNR = beat_energy / (total_energy - beat_energy)     (if denom > 0)
//!
//! Label = Porous     if SNR > threshold
//!         Boundaried otherwise
//! ```
//!
//! Default threshold: 1.5.  Adjustable at runtime.

use crate::act::{ActConfig, ActEngine, Chirplet, TransformOpts};
use crate::alpha::NUM_CH;
use crate::compute::ContactQuality;

// ── Configuration ────────────────────────────────────────────────────────────

/// Tuneable parameters for the entrainment measurement.
///
/// Data-driven: uses window counts instead of wall-clock durations.
#[derive(Debug, Clone)]
pub struct EntrainmentConfig {
    /// Beat frequency in Hz (stimulus frequency to detect).
    pub beat_freq_hz: f64,
    /// Tolerance around beat frequency for chirplet matching (±Hz).
    pub fc_tolerance_hz: f64,
    /// SNR threshold above which entrainment is classified as "Porous".
    pub snr_threshold: f64,
    /// Windows to discard during settling.
    pub settling_windows: usize,
    /// Windows to accumulate during measuring.
    pub measuring_windows: usize,
}

impl Default for EntrainmentConfig {
    fn default() -> Self {
        Self {
            beat_freq_hz: 2.0,
            fc_tolerance_hz: 0.5,
            snr_threshold: 1.5,
            settling_windows: 20,  // ~5 s at 4 hops/s
            measuring_windows: 160, // ~40 s at 4 hops/s
        }
    }
}

impl EntrainmentConfig {
    /// Total windows required (settling + measuring).
    pub fn total_windows(&self) -> usize {
        self.settling_windows + self.measuring_windows
    }
}

/// Dictionary parameters for the dedicated entrainment ACT engine.
#[derive(Debug, Clone)]
pub struct EntrainmentDictConfig {
    /// Window length in samples.
    pub length: usize,
    /// Sampling rate Hz.
    pub fs: f64,
    /// Frequency-center range: (min, max, step) in Hz.
    pub fc_range: (f64, f64, f64),
    /// Time-center range: (min, max, step) in samples.
    pub tc_range: (f64, f64, f64),
    /// Log-duration range: (min, max, step).
    pub log_dt_range: (f64, f64, f64),
    /// Chirp-rate range: (min, max, step) in Hz/s.
    pub chirp_range: (f64, f64, f64),
    /// Number of chirplets to extract per transform.
    pub order: usize,
    /// Whether to run BFGS refinement.
    pub refine: bool,
}

impl Default for EntrainmentDictConfig {
    fn default() -> Self {
        Self {
            length: 1024,
            fs: 256.0,
            fc_range: (0.5, 4.0, 0.25),
            tc_range: (0.0, 1023.0, 32.0),
            log_dt_range: (-1.5, 0.7, 0.3),
            chirp_range: (-5.0, 5.0, 2.5),
            order: 7,
            refine: true,
        }
    }
}

impl EntrainmentDictConfig {
    /// Build an `ActConfig` from these dictionary parameters.
    pub fn to_act_config(&self) -> ActConfig {
        ActConfig {
            fs: self.fs,
            length: self.length,
            tc_range: self.tc_range,
            fc_range: self.fc_range,
            log_dt_range: self.log_dt_range,
            chirp_range: self.chirp_range,
        }
    }

    /// Build `TransformOpts` from these dictionary parameters.
    pub fn to_transform_opts(&self) -> TransformOpts {
        TransformOpts {
            order: self.order,
            residual_threshold: 1e-6,
            refine: self.refine,
        }
    }
}

// ── Classification label ─────────────────────────────────────────────────────

/// Binary classification output.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EntrainmentLabel {
    Porous,
    Boundaried,
}

impl std::fmt::Display for EntrainmentLabel {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Porous => write!(f, "Porous"),
            Self::Boundaried => write!(f, "Boundaried"),
        }
    }
}

// ── Phase ────────────────────────────────────────────────────────────────────

/// Current phase of the entrainment detector.
#[derive(Debug, Clone, PartialEq)]
pub enum EntrainmentPhase {
    /// Not running.
    Idle,
    /// Initial settling period — data discarded.
    Settling,
    /// Actively measuring entrainment.
    Measuring,
    /// Measurement complete — result available.
    Complete,
}

// ── Per-channel accumulator ──────────────────────────────────────────────────

/// Accumulates beat-frequency and total chirplet energy for one channel.
#[derive(Debug, Clone, Default)]
pub struct ChannelAccum {
    /// Sum of |coeff|² for chirplets matching the beat frequency.
    pub beat_energy: f64,
    /// Sum of |coeff|² for all chirplets.
    pub total_energy: f64,
    /// Number of ACT windows processed.
    pub total_windows: usize,
    /// Number of clean (artifact-free) windows.
    pub clean_windows: usize,
    /// Number of beat-matching chirplets found.
    pub beat_chirplet_count: usize,
}

impl ChannelAccum {
    fn reset(&mut self) {
        self.beat_energy = 0.0;
        self.total_energy = 0.0;
        self.total_windows = 0;
        self.clean_windows = 0;
        self.beat_chirplet_count = 0;
    }

    /// Channel-level SNR: beat_energy / noise_energy.
    pub fn snr(&self) -> f64 {
        let noise = self.total_energy - self.beat_energy;
        if noise > 0.0 {
            self.beat_energy / noise
        } else if self.beat_energy > 0.0 {
            f64::INFINITY
        } else {
            0.0
        }
    }

    pub fn accumulate(&mut self, chirplets: &[Chirplet], beat_lo: f64, beat_hi: f64, is_clean: bool) {
        self.total_windows += 1;
        if !is_clean { return; }
        self.clean_windows += 1;
        for chirp in chirplets {
            let energy = (chirp.coeff as f64).powi(2);
            self.total_energy += energy;
            if is_beat_chirplet(chirp, beat_lo, beat_hi) {
                self.beat_energy += energy;
                self.beat_chirplet_count += 1;
            }
        }
    }

    /// Artifact ratio: fraction of windows rejected.
    pub fn artifact_ratio(&self) -> f64 {
        if self.total_windows == 0 {
            return 0.0;
        }
        1.0 - (self.clean_windows as f64 / self.total_windows as f64)
    }
}

// ── Result ───────────────────────────────────────────────────────────────────

/// Final entrainment measurement result.
#[derive(Debug, Clone)]
pub struct EntrainmentResult {
    /// Per-channel SNR.
    pub per_channel_snr: [f64; NUM_CH],
    /// Mean SNR across all channels.
    pub mean_snr: f64,
    /// Beat frequency used.
    pub beat_freq_hz: f64,
    /// SNR threshold used for classification.
    pub snr_threshold: f64,
    /// Binary label.
    pub label: EntrainmentLabel,
    /// Per-channel beat energy.
    pub per_channel_beat_energy: [f64; NUM_CH],
    /// Per-channel total energy.
    pub per_channel_total_energy: [f64; NUM_CH],
    /// Overall artifact ratio (worst channel).
    pub artifact_ratio: f64,
    /// Total measuring windows accumulated.
    pub measuring_windows: usize,
    /// Total clean windows across all channels.
    pub total_clean_windows: usize,
}

// ── Detector ─────────────────────────────────────────────────────────────────

/// State machine that measures neural entrainment using a dedicated ACT engine.
///
/// Data-driven: phases transition based on window counts, not wall-clock.
pub struct EntrainmentDetector {
    pub config: EntrainmentConfig,
    pub dict_config: EntrainmentDictConfig,
    pub phase: EntrainmentPhase,
    /// Windows received during settling (discarded).
    settling_count: usize,
    /// Windows received during measuring.
    measuring_count: usize,
    /// Dedicated ACT engine for entrainment (created on start).
    engine: Option<ActEngine>,
    /// Transform options (derived from dict_config).
    opts: TransformOpts,
    /// Per-channel accumulators.
    pub channels: [ChannelAccum; NUM_CH],
    /// Completed result.
    pub result: Option<EntrainmentResult>,
}

// Manual Debug impl because ActEngine doesn't derive Debug
impl std::fmt::Debug for EntrainmentDetector {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("EntrainmentDetector")
            .field("config", &self.config)
            .field("dict_config", &self.dict_config)
            .field("phase", &self.phase)
            .field("engine_active", &self.engine.is_some())
            .field("channels", &self.channels)
            .field("result", &self.result)
            .finish()
    }
}

impl Default for EntrainmentDetector {
    fn default() -> Self {
        let dict_config = EntrainmentDictConfig::default();
        let opts = dict_config.to_transform_opts();
        Self {
            config: EntrainmentConfig::default(),
            dict_config,
            phase: EntrainmentPhase::Idle,
            settling_count: 0,
            measuring_count: 0,
            engine: None,
            opts,
            channels: std::array::from_fn(|_| ChannelAccum::default()),
            result: None,
        }
    }
}

impl EntrainmentDetector {
    /// Start the entrainment measurement.
    ///
    /// Creates a dedicated ACT engine from the current dict config.
    /// Returns `Ok(())` on success, or an error if engine creation fails.
    pub fn start(&mut self) -> anyhow::Result<()> {
        let act_cfg = self.dict_config.to_act_config();
        let engine = ActEngine::new(&act_cfg)?;
        self.opts = self.dict_config.to_transform_opts();
        self.engine = Some(engine);
        self.phase = EntrainmentPhase::Settling;
        self.settling_count = 0;
        self.measuring_count = 0;
        for ch in &mut self.channels {
            ch.reset();
        }
        self.result = None;
        Ok(())
    }

    pub fn start_without_engine(&mut self) {
        self.opts = self.dict_config.to_transform_opts();
        self.engine = None;
        self.phase = EntrainmentPhase::Settling;
        self.settling_count = 0;
        self.measuring_count = 0;
        for ch in &mut self.channels { ch.reset(); }
        self.result = None;
    }

    pub fn advance_settling(&mut self) -> bool {
        if !matches!(self.phase, EntrainmentPhase::Settling) { return false; }
        self.settling_count += 1;
        if self.settling_count >= self.config.settling_windows {
            self.phase = EntrainmentPhase::Measuring;
            return true;
        }
        false
    }

    pub fn needs_transform(&self) -> bool {
        matches!(self.phase, EntrainmentPhase::Measuring)
    }

    pub fn apply_transform_result(
        &mut self,
        results: &[crate::act::ChannelResult],
        epoch_ok: &[bool; NUM_CH],
        contact: &[ContactQuality; NUM_CH],
    ) {
        if !matches!(self.phase, EntrainmentPhase::Measuring) { return; }
        self.measuring_count += 1;

        let beat_lo = self.config.beat_freq_hz - self.config.fc_tolerance_hz;
        let beat_hi = self.config.beat_freq_hz + self.config.fc_tolerance_hz;
        for (ch, cr) in results.iter().enumerate().take(NUM_CH) {
            let is_clean = epoch_ok[ch] && contact[ch] != ContactQuality::NoContact;
            self.channels[ch].accumulate(&cr.chirplets, beat_lo, beat_hi, is_clean);
        }

        if self.measuring_count >= self.config.measuring_windows {
            self.finish();
        }
    }

    /// Stop measurement. If measuring, finalize result and go to Complete.
    /// Otherwise return to Idle.
    pub fn stop(&mut self) {
        if matches!(self.phase, EntrainmentPhase::Measuring) {
            self.finish();
        } else {
            self.phase = EntrainmentPhase::Idle;
            self.engine = None;
        }
    }

    /// Whether the detector is actively running (settling or measuring).
    pub fn is_running(&self) -> bool {
        matches!(self.phase, EntrainmentPhase::Settling | EntrainmentPhase::Measuring)
    }

    /// Overall progress fraction (0.0–1.0).
    pub fn progress(&self) -> f64 {
        let total = self.config.total_windows();
        if total == 0 {
            return 1.0;
        }
        let done = self.settling_count + self.measuring_count;
        (done as f64 / total as f64).clamp(0.0, 1.0)
    }

    /// Feed filtered EEG windows (one per channel) from the signal pipeline.
    ///
    /// Each element of `windows` must have length == `dict_config.length`.
    pub fn feed(
        &mut self,
        windows: &[&[f64]],
        epoch_ok: &[bool; NUM_CH],
        contact: &[ContactQuality; NUM_CH],
    ) {
        match self.phase {
            EntrainmentPhase::Idle | EntrainmentPhase::Complete => return,
            EntrainmentPhase::Settling => {
                self.settling_count += 1;
                if self.settling_count >= self.config.settling_windows {
                    self.phase = EntrainmentPhase::Measuring;
                } else {
                    return;
                }
            }
            EntrainmentPhase::Measuring => { /* process below */ }
        }

        // Run the dedicated ACT engine
        self.measuring_count += 1;
        let engine = match &self.engine {
            Some(e) => e,
            None => return,
        };

        let results = match engine.transform_batch(windows, &self.opts) {
            Ok(r) => r,
            Err(e) => {
                log::warn!("Entrainment ACT transform failed: {e}");
                return;
            }
        };

        let beat_lo = self.config.beat_freq_hz - self.config.fc_tolerance_hz;
        let beat_hi = self.config.beat_freq_hz + self.config.fc_tolerance_hz;
        for (ch, cr) in results.iter().enumerate().take(NUM_CH) {
            let is_clean = epoch_ok[ch] && contact[ch] != ContactQuality::NoContact;
            self.channels[ch].accumulate(&cr.chirplets, beat_lo, beat_hi, is_clean);
        }

        // Check completion
        if self.measuring_count >= self.config.measuring_windows {
            self.finish();
        }
    }

    /// Compute the *live* mean SNR from accumulated data so far.
    pub fn live_snr(&self) -> f64 {
        let snrs: Vec<f64> = self.channels.iter()
            .filter(|ch| ch.clean_windows > 0)
            .map(|ch| ch.snr())
            .collect();
        if snrs.is_empty() {
            return 0.0;
        }
        snrs.iter().sum::<f64>() / snrs.len() as f64
    }

    /// Live per-channel SNR.
    pub fn live_channel_snr(&self) -> [f64; NUM_CH] {
        std::array::from_fn(|i| self.channels[i].snr())
    }

    /// Live per-channel artifact ratio.
    pub fn live_channel_artifact_ratio(&self) -> [f64; NUM_CH] {
        std::array::from_fn(|i| self.channels[i].artifact_ratio())
    }

    /// Live classification based on current accumulated data.
    pub fn live_label(&self) -> EntrainmentLabel {
        if self.live_snr() >= self.config.snr_threshold {
            EntrainmentLabel::Porous
        } else {
            EntrainmentLabel::Boundaried
        }
    }

    /// Compute final result and transition to Complete.
    fn finish(&mut self) {
        let per_channel_snr: [f64; NUM_CH] =
            std::array::from_fn(|i| self.channels[i].snr());
        let per_channel_beat_energy: [f64; NUM_CH] =
            std::array::from_fn(|i| self.channels[i].beat_energy);
        let per_channel_total_energy: [f64; NUM_CH] =
            std::array::from_fn(|i| self.channels[i].total_energy);

        let active: Vec<f64> = self.channels.iter()
            .filter(|ch| ch.clean_windows > 0)
            .map(|ch| ch.snr())
            .collect();
        let mean_snr = if active.is_empty() {
            0.0
        } else {
            active.iter().sum::<f64>() / active.len() as f64
        };

        let label = if mean_snr >= self.config.snr_threshold {
            EntrainmentLabel::Porous
        } else {
            EntrainmentLabel::Boundaried
        };

        let worst_artifact = self
            .channels
            .iter()
            .map(|ch| ch.artifact_ratio())
            .fold(0.0_f64, f64::max);

        let total_clean: usize = self.channels.iter().map(|ch| ch.clean_windows).sum();

        self.result = Some(EntrainmentResult {
            per_channel_snr,
            mean_snr,
            beat_freq_hz: self.config.beat_freq_hz,
            snr_threshold: self.config.snr_threshold,
            label,
            per_channel_beat_energy,
            per_channel_total_energy,
            artifact_ratio: worst_artifact,
            measuring_windows: self.measuring_count,
            total_clean_windows: total_clean,
        });

        self.phase = EntrainmentPhase::Complete;
        self.engine = None; // release engine
    }
}

// ── Helpers ──────────────────────────────────────────────────────────────────

/// Test whether a chirplet matches the beat-frequency criterion.
fn is_beat_chirplet(c: &Chirplet, fc_lo: f64, fc_hi: f64) -> bool {
    let fc = c.fc as f64;
    fc >= fc_lo && fc <= fc_hi
}

// ── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn idle_by_default() {
        let det = EntrainmentDetector::default();
        assert_eq!(det.phase, EntrainmentPhase::Idle);
        assert!(det.result.is_none());
        assert!(!det.is_running());
    }

    #[test]
    fn channel_accum_snr() {
        let mut acc = ChannelAccum::default();
        acc.beat_energy = 3.0;
        acc.total_energy = 6.0;
        // noise = 6.0 - 3.0 = 3.0 → SNR = 3.0 / 3.0 = 1.0
        assert!((acc.snr() - 1.0).abs() < 1e-10);
    }

    #[test]
    fn channel_accum_snr_all_beat() {
        let mut acc = ChannelAccum::default();
        acc.beat_energy = 5.0;
        acc.total_energy = 5.0;
        // noise = 0 → SNR = inf
        assert!(acc.snr().is_infinite());
    }

    #[test]
    fn channel_accum_snr_no_beat() {
        let mut acc = ChannelAccum::default();
        acc.beat_energy = 0.0;
        acc.total_energy = 5.0;
        assert!((acc.snr() - 0.0).abs() < 1e-10);
    }

    #[test]
    fn channel_accum_artifact_ratio() {
        let mut acc = ChannelAccum::default();
        acc.total_windows = 10;
        acc.clean_windows = 8;
        assert!((acc.artifact_ratio() - 0.2).abs() < 1e-10);
    }

    #[test]
    fn is_beat_chirplet_yes() {
        let c = Chirplet { tc: 0.0, fc: 2.0, log_dt: -1.0, chirp_rate: 5.0, coeff: 1.0 };
        assert!(is_beat_chirplet(&c, 1.5, 2.5));
    }

    #[test]
    fn is_beat_chirplet_wrong_freq() {
        let c = Chirplet { tc: 0.0, fc: 5.0, log_dt: -1.0, chirp_rate: 0.1, coeff: 1.0 };
        assert!(!is_beat_chirplet(&c, 1.5, 2.5));
    }

    #[test]
    fn classification_logic() {
        // SNR 2.0 > threshold 1.5 → Porous
        let mut det = EntrainmentDetector::default();
        for ch in &mut det.channels {
            ch.beat_energy = 3.0;
            ch.total_energy = 4.5; // noise = 1.5, SNR = 3.0/1.5 = 2.0
            ch.clean_windows = 1;
        }
        assert_eq!(det.live_label(), EntrainmentLabel::Porous);
        assert!((det.live_snr() - 2.0).abs() < 1e-10);
    }

    #[test]
    fn classification_boundaried() {
        // SNR 0.5 < threshold 1.5 → Boundaried
        let mut det = EntrainmentDetector::default();
        for ch in &mut det.channels {
            ch.beat_energy = 1.0;
            ch.total_energy = 3.0; // noise = 2.0, SNR = 1.0/2.0 = 0.5
            ch.clean_windows = 1;
        }
        assert_eq!(det.live_label(), EntrainmentLabel::Boundaried);
        assert!((det.live_snr() - 0.5).abs() < 1e-10);
    }

    #[test]
    fn dict_config_defaults_are_sane() {
        let dc = EntrainmentDictConfig::default();
        let act_cfg = dc.to_act_config();
        assert_eq!(act_cfg.length, 1024);
        assert!((act_cfg.fs - 256.0).abs() < 1e-10);
        // fc range covers beat freq
        assert!(act_cfg.fc_range.0 <= 2.0);
        assert!(act_cfg.fc_range.1 >= 2.0);
        // Fine resolution
        assert!(act_cfg.fc_range.2 <= 0.5);
    }

    #[test]
    fn accumulate_beat_chirplets() {
        let mut acc = ChannelAccum::default();
        let chirplets = vec![
            Chirplet { tc: 0.0, fc: 2.0, log_dt: -1.0, chirp_rate: 0.0, coeff: 3.0 },
            Chirplet { tc: 0.0, fc: 8.0, log_dt: -1.0, chirp_rate: 0.0, coeff: 4.0 },
        ];
        acc.accumulate(&chirplets, 1.5, 2.5, true);
        assert_eq!(acc.total_windows, 1);
        assert_eq!(acc.clean_windows, 1);
        assert!((acc.beat_energy - 9.0).abs() < 1e-10);
        assert!((acc.total_energy - 25.0).abs() < 1e-10);
        assert_eq!(acc.beat_chirplet_count, 1);
    }

    #[test]
    fn accumulate_dirty_window() {
        let mut acc = ChannelAccum::default();
        let chirplets = vec![
            Chirplet { tc: 0.0, fc: 2.0, log_dt: -1.0, chirp_rate: 0.0, coeff: 3.0 },
        ];
        acc.accumulate(&chirplets, 1.5, 2.5, false);
        assert_eq!(acc.total_windows, 1);
        assert_eq!(acc.clean_windows, 0);
        assert!((acc.beat_energy - 0.0).abs() < 1e-10);
    }

    #[test]
    fn start_without_engine_resets_state() {
        let mut det = EntrainmentDetector::default();
        det.start_without_engine();
        assert_eq!(det.phase, EntrainmentPhase::Settling);
        assert!(det.engine.is_none());
        assert_eq!(det.settling_count, 0);
        assert_eq!(det.measuring_count, 0);
        assert!(det.is_running());
    }

    #[test]
    fn advance_settling_transitions_at_threshold() {
        let mut det = EntrainmentDetector::default();
        det.start_without_engine();
        for _ in 0..det.config.settling_windows - 1 {
            assert!(!det.advance_settling());
            assert_eq!(det.phase, EntrainmentPhase::Settling);
        }
        assert!(det.advance_settling());
        assert_eq!(det.phase, EntrainmentPhase::Measuring);
    }

    #[test]
    fn advance_settling_noop_outside_settling() {
        let mut det = EntrainmentDetector::default();
        assert!(!det.advance_settling()); // Idle
    }

    #[test]
    fn needs_transform_only_in_measuring() {
        let mut det = EntrainmentDetector::default();
        assert!(!det.needs_transform());
        det.start_without_engine();
        assert!(!det.needs_transform()); // Settling
        det.phase = EntrainmentPhase::Measuring;
        assert!(det.needs_transform());
    }

    #[test]
    fn apply_transform_result_accumulates() {
        use crate::act::ChannelResult;

        let mut det = EntrainmentDetector::default();
        det.start_without_engine();
        det.phase = EntrainmentPhase::Measuring;

        let results: Vec<ChannelResult> = (0..NUM_CH).map(|_| ChannelResult {
            chirplets: vec![
                Chirplet { tc: 0.0, fc: 2.0, log_dt: -1.0, chirp_rate: 0.0, coeff: 3.0 },
            ],
            error: 0.0,
        }).collect();
        let epoch_ok = [true; NUM_CH];
        let contact = [ContactQuality::Good; NUM_CH];

        det.apply_transform_result(&results, &epoch_ok, &contact);
        assert_eq!(det.measuring_count, 1);
        assert!((det.channels[0].beat_energy - 9.0).abs() < 1e-10);
    }

    #[test]
    fn apply_transform_result_discards_outside_measuring() {
        use crate::act::ChannelResult;

        let mut det = EntrainmentDetector::default();
        // Idle phase — should discard
        let results: Vec<ChannelResult> = (0..NUM_CH).map(|_| ChannelResult {
            chirplets: vec![
                Chirplet { tc: 0.0, fc: 2.0, log_dt: -1.0, chirp_rate: 0.0, coeff: 3.0 },
            ],
            error: 0.0,
        }).collect();
        let epoch_ok = [true; NUM_CH];
        let contact = [ContactQuality::Good; NUM_CH];

        det.apply_transform_result(&results, &epoch_ok, &contact);
        assert_eq!(det.measuring_count, 0);
        assert!((det.channels[0].beat_energy - 0.0).abs() < 1e-10);
    }
}
