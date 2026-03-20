//! Absorption dimension — measures how deeply someone drops into immersive states.
//!
//! # Neural Signature
//!
//! Increase in alpha power (8–12 Hz) during immersion vs. baseline.
//!
//! # Formula
//!
//! ```text
//! P̄α_baseline  = (Pα(AF7, baseline) + Pα(AF8, baseline)) / 2
//! P̄α_immersion = (Pα(AF7, immersion) + Pα(AF8, immersion)) / 2
//!
//! Absorption%  = ((P̄α_immersion − P̄α_baseline) / P̄α_baseline) × 100
//!
//! Label = Deep    if Absorption% > Threshold
//!         Surface otherwise
//! ```
//!
//! Default threshold: 25 % (conservative). Adjustable at runtime.

use std::time::Instant;

use crate::alpha::{FftSnapshot, NUM_CH, CH_AF7, CH_AF8};
use crate::baseline::BaselineResult;
use crate::compute::ContactQuality;

// ── Configuration ────────────────────────────────────────────────────────────

/// Tuneable parameters for the absorption measurement.
#[derive(Debug, Clone)]
pub struct AbsorptionConfig {
    /// Threshold (percent) above which absorption is classified as "Deep".
    pub threshold_pct: f64,
    /// Total measurement duration in seconds.
    pub measurement_duration_s: f64,
    /// Initial settling period (discarded) in seconds.
    pub settling_duration_s: f64,
}

impl Default for AbsorptionConfig {
    fn default() -> Self {
        Self {
            threshold_pct: 25.0,
            measurement_duration_s: 45.0,
            settling_duration_s: 5.0,
        }
    }
}

impl AbsorptionConfig {
    /// Effective recording duration (total minus settling).
    pub fn recording_duration_s(&self) -> f64 {
        self.measurement_duration_s - self.settling_duration_s
    }
}

// ── Classification label ─────────────────────────────────────────────────────

/// Binary classification output.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AbsorptionLabel {
    Deep,
    Surface,
}

impl std::fmt::Display for AbsorptionLabel {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Deep => write!(f, "Deep"),
            Self::Surface => write!(f, "Surface"),
        }
    }
}

// ── Phase ────────────────────────────────────────────────────────────────────

/// Current phase of the absorption detector.
#[derive(Debug, Clone, PartialEq)]
pub enum AbsorptionPhase {
    /// Not running. Press key to start.
    Idle,
    /// No baseline data available — must collect baseline first.
    NeedBaseline,
    /// Initial settling period — data discarded.
    Settling,
    /// Actively measuring immersion alpha.
    Measuring,
    /// Measurement complete — result available.
    Complete,
}

// ── Per-channel accumulator ──────────────────────────────────────────────────

#[derive(Debug, Clone, Default)]
pub struct ChannelAccum {
    pub alpha_powers: Vec<f32>,
    pub total_snapshots: usize,
    pub clean_snapshots: usize,
}

impl ChannelAccum {
    fn reset(&mut self) {
        self.alpha_powers.clear();
        self.total_snapshots = 0;
        self.clean_snapshots = 0;
    }

    fn mean_alpha(&self) -> f64 {
        if self.alpha_powers.is_empty() {
            return 0.0;
        }
        self.alpha_powers.iter().map(|&v| v as f64).sum::<f64>() / self.alpha_powers.len() as f64
    }

    fn artifact_ratio(&self) -> f64 {
        if self.total_snapshots == 0 {
            return 0.0;
        }
        1.0 - (self.clean_snapshots as f64 / self.total_snapshots as f64)
    }
}

// ── Result ───────────────────────────────────────────────────────────────────

/// Final absorption measurement result.
#[derive(Debug, Clone)]
pub struct AbsorptionResult {
    /// Baseline mean frontal alpha: (AF7 + AF8) / 2.
    pub baseline_alpha_mean: f64,
    /// Immersion mean frontal alpha: (AF7 + AF8) / 2.
    pub immersion_alpha_mean: f64,
    /// Per-channel baseline alpha (from BaselineResult).
    pub per_channel_baseline: [f64; NUM_CH],
    /// Per-channel immersion alpha (measured).
    pub per_channel_immersion: [f64; NUM_CH],
    /// Absorption percentage: ((immersion − baseline) / baseline) × 100.
    pub absorption_pct: f64,
    /// Threshold used for classification.
    pub threshold_pct: f64,
    /// Binary label.
    pub label: AbsorptionLabel,
    /// Overall artifact ratio during measurement.
    pub artifact_ratio: f64,
    /// Effective recording duration (seconds).
    pub recording_duration_s: f64,
    /// Total clean snapshots accumulated.
    pub total_clean_snapshots: usize,
}

// ── Detector ─────────────────────────────────────────────────────────────────

/// State machine that measures absorption by comparing immersion alpha to baseline.
#[derive(Debug, Clone)]
pub struct AbsorptionDetector {
    pub config: AbsorptionConfig,
    pub phase: AbsorptionPhase,
    start_time: Option<Instant>,
    /// Per-channel accumulators for immersion period.
    pub channels: [ChannelAccum; NUM_CH],
    /// Baseline alpha per channel (copied from BaselineResult on start).
    pub baseline_alpha: Option<[f64; NUM_CH]>,
    /// Completed result.
    pub result: Option<AbsorptionResult>,
}

impl Default for AbsorptionDetector {
    fn default() -> Self {
        Self {
            config: AbsorptionConfig::default(),
            phase: AbsorptionPhase::Idle,
            start_time: None,
            channels: std::array::from_fn(|_| ChannelAccum::default()),
            baseline_alpha: None,
            result: None,
        }
    }
}

impl AbsorptionDetector {
    /// Attempt to start an absorption measurement.
    ///
    /// Requires a completed baseline. Returns `false` if no baseline is available.
    pub fn start(&mut self, baseline: &BaselineResult) -> bool {
        self.baseline_alpha = Some(baseline.channel_alpha);
        self.phase = AbsorptionPhase::Settling;
        self.start_time = Some(Instant::now());
        for ch in &mut self.channels {
            ch.reset();
        }
        self.result = None;
        true
    }

    /// Start without baseline — transitions to NeedBaseline phase.
    pub fn start_no_baseline(&mut self) {
        self.phase = AbsorptionPhase::NeedBaseline;
    }

    /// Stop measurement and return to Idle.
    pub fn stop(&mut self) {
        self.phase = AbsorptionPhase::Idle;
        self.start_time = None;
    }

    /// Whether the detector is actively running (settling or measuring).
    pub fn is_running(&self) -> bool {
        matches!(self.phase, AbsorptionPhase::Settling | AbsorptionPhase::Measuring)
    }

    /// Seconds elapsed since start.
    pub fn elapsed_s(&self) -> f64 {
        self.start_time
            .map(|t| t.elapsed().as_secs_f64())
            .unwrap_or(0.0)
    }

    /// Seconds elapsed in the measuring phase specifically.
    pub fn measuring_elapsed_s(&self) -> f64 {
        (self.elapsed_s() - self.config.settling_duration_s).max(0.0)
    }

    /// Overall progress fraction (0.0–1.0).
    pub fn progress(&self) -> f64 {
        let total = self.config.measurement_duration_s;
        if total <= 0.0 {
            return 1.0;
        }
        (self.elapsed_s() / total).clamp(0.0, 1.0)
    }

    /// Compute the *live* absorption percentage from accumulated data so far.
    ///
    /// Returns `None` if no baseline or not enough data yet.
    pub fn live_absorption_pct(&self) -> Option<f64> {
        let bl = self.baseline_alpha.as_ref()?;
        let bl_mean = (bl[CH_AF7] + bl[CH_AF8]) / 2.0;
        if bl_mean <= 0.0 {
            return None;
        }

        let af7_alpha = self.channels[CH_AF7].mean_alpha();
        let af8_alpha = self.channels[CH_AF8].mean_alpha();
        if self.channels[CH_AF7].alpha_powers.is_empty()
            || self.channels[CH_AF8].alpha_powers.is_empty()
        {
            return None;
        }

        let imm_mean = (af7_alpha + af8_alpha) / 2.0;
        Some(((imm_mean - bl_mean) / bl_mean) * 100.0)
    }

    /// Live classification based on current accumulated data.
    pub fn live_label(&self) -> Option<AbsorptionLabel> {
        self.live_absorption_pct().map(|pct| {
            if pct >= self.config.threshold_pct {
                AbsorptionLabel::Deep
            } else {
                AbsorptionLabel::Surface
            }
        })
    }

    /// Per-channel immersion alpha (live, for display).
    pub fn live_channel_alpha(&self) -> [f64; NUM_CH] {
        std::array::from_fn(|i| self.channels[i].mean_alpha())
    }

    /// Per-channel artifact ratio (live, for display).
    pub fn live_channel_artifact_ratio(&self) -> [f64; NUM_CH] {
        std::array::from_fn(|i| self.channels[i].artifact_ratio())
    }

    /// Feed a new FFT snapshot from the signal pipeline.
    pub fn feed(
        &mut self,
        fft: &FftSnapshot,
        epoch_ok: &[bool; NUM_CH],
        contact: &[ContactQuality; NUM_CH],
    ) -> &AbsorptionPhase {
        let elapsed = self.elapsed_s();

        match self.phase {
            AbsorptionPhase::Idle
            | AbsorptionPhase::NeedBaseline
            | AbsorptionPhase::Complete => {
                return &self.phase;
            }
            AbsorptionPhase::Settling => {
                if elapsed >= self.config.settling_duration_s {
                    self.phase = AbsorptionPhase::Measuring;
                } else {
                    return &self.phase;
                }
            }
            AbsorptionPhase::Measuring => { /* process below */ }
        }

        // ── Measuring: accumulate per-channel alpha ─────────────────────
        for ch in 0..NUM_CH {
            self.channels[ch].total_snapshots += 1;
            let is_clean = epoch_ok[ch] && contact[ch] != ContactQuality::NoContact;
            if is_clean {
                self.channels[ch].clean_snapshots += 1;
                self.channels[ch].alpha_powers.push(fft.channels[ch].alpha_power);
            }
        }

        // ── Check completion ────────────────────────────────────────────
        if elapsed >= self.config.measurement_duration_s {
            self.finish();
        }

        &self.phase
    }

    /// Compute final result and transition to Complete.
    fn finish(&mut self) {
        let bl = match &self.baseline_alpha {
            Some(bl) => *bl,
            None => {
                self.phase = AbsorptionPhase::NeedBaseline;
                return;
            }
        };

        let per_channel_immersion: [f64; NUM_CH] =
            std::array::from_fn(|i| self.channels[i].mean_alpha());

        let bl_mean = (bl[CH_AF7] + bl[CH_AF8]) / 2.0;
        let imm_mean = (per_channel_immersion[CH_AF7] + per_channel_immersion[CH_AF8]) / 2.0;

        let absorption_pct = if bl_mean > 0.0 {
            ((imm_mean - bl_mean) / bl_mean) * 100.0
        } else {
            0.0
        };

        let label = if absorption_pct >= self.config.threshold_pct {
            AbsorptionLabel::Deep
        } else {
            AbsorptionLabel::Surface
        };

        let worst_artifact = self
            .channels
            .iter()
            .map(|ch| ch.artifact_ratio())
            .fold(0.0_f64, f64::max);

        let total_clean: usize = self.channels.iter().map(|ch| ch.clean_snapshots).sum();

        self.result = Some(AbsorptionResult {
            baseline_alpha_mean: bl_mean,
            immersion_alpha_mean: imm_mean,
            per_channel_baseline: bl,
            per_channel_immersion,
            absorption_pct,
            threshold_pct: self.config.threshold_pct,
            label,
            artifact_ratio: worst_artifact,
            recording_duration_s: self.measuring_elapsed_s(),
            total_clean_snapshots: total_clean,
        });

        self.phase = AbsorptionPhase::Complete;
    }
}

// ── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::alpha::FftBandPower;

    fn make_baseline(af7_alpha: f64, af8_alpha: f64) -> BaselineResult {
        let mut channel_alpha = [0.0; NUM_CH];
        channel_alpha[CH_AF7] = af7_alpha;
        channel_alpha[CH_AF8] = af8_alpha;
        BaselineResult {
            channel_alpha,
            channel_alpha_cv: [0.1; NUM_CH],
            channel_artifact_ratio: [0.05; NUM_CH],
            alpha_asymmetry: 0.0,
            noise_delta: 1.0,
            noise_theta: 0.5,
            overall_artifact_ratio: 0.05,
            recording_duration_s: 45.0,
            total_clean_snapshots: 180,
        }
    }

    fn make_fft(alpha: f32) -> FftSnapshot {
        let bp = FftBandPower {
            delta_power: 1.0,
            theta_power: 0.5,
            alpha_power: alpha,
            beta_power: 0.3,
            gamma_power: 0.1,
            total_power: 1.0 + 0.5 + alpha + 0.3 + 0.1,
            relative_alpha: alpha / (1.0 + 0.5 + alpha + 0.3 + 0.1),
        };
        FftSnapshot {
            channels: [bp; NUM_CH],
            frontal_asymmetry: 0.0,
            temporal_asymmetry: 0.0,
            mean_relative_alpha: bp.relative_alpha,
        }
    }

    #[test]
    fn idle_by_default() {
        let det = AbsorptionDetector::default();
        assert_eq!(det.phase, AbsorptionPhase::Idle);
        assert!(det.result.is_none());
    }

    #[test]
    fn needs_baseline() {
        let mut det = AbsorptionDetector::default();
        det.start_no_baseline();
        assert_eq!(det.phase, AbsorptionPhase::NeedBaseline);
    }

    #[test]
    fn start_with_baseline() {
        let mut det = AbsorptionDetector::default();
        let bl = make_baseline(10.0, 10.0);
        assert!(det.start(&bl));
        assert_eq!(det.phase, AbsorptionPhase::Settling);
    }

    #[test]
    fn absorption_classification() {
        // Baseline alpha = 10.0 for AF7 and AF8
        // Immersion alpha = 13.0 → 30% increase → Deep at 25% threshold
        let bl = make_baseline(10.0, 10.0);

        let mut det = AbsorptionDetector::default();
        det.config.measurement_duration_s = 0.0; // instant complete
        det.config.settling_duration_s = 0.0;
        det.start(&bl);

        let fft = make_fft(13.0); // 30% above baseline of 10.0
        let ok = [true; NUM_CH];
        let contact = [ContactQuality::Good; NUM_CH];
        det.feed(&fft, &ok, &contact);

        assert_eq!(det.phase, AbsorptionPhase::Complete);
        let result = det.result.as_ref().unwrap();
        assert!((result.absorption_pct - 30.0).abs() < 0.1);
        assert_eq!(result.label, AbsorptionLabel::Deep);
    }

    #[test]
    fn surface_classification() {
        // Baseline alpha = 10.0, immersion = 11.0 → 10% → Surface
        let bl = make_baseline(10.0, 10.0);

        let mut det = AbsorptionDetector::default();
        det.config.measurement_duration_s = 0.0;
        det.config.settling_duration_s = 0.0;
        det.start(&bl);

        let fft = make_fft(11.0); // 10% above baseline
        let ok = [true; NUM_CH];
        let contact = [ContactQuality::Good; NUM_CH];
        det.feed(&fft, &ok, &contact);

        assert_eq!(det.phase, AbsorptionPhase::Complete);
        let result = det.result.as_ref().unwrap();
        assert!((result.absorption_pct - 10.0).abs() < 0.1);
        assert_eq!(result.label, AbsorptionLabel::Surface);
    }

    #[test]
    fn threshold_adjustment() {
        let bl = make_baseline(10.0, 10.0);

        let mut det = AbsorptionDetector::default();
        det.config.threshold_pct = 20.0; // more aggressive
        det.config.measurement_duration_s = 0.0;
        det.config.settling_duration_s = 0.0;
        det.start(&bl);

        let fft = make_fft(12.2); // 22% → Deep at 20% threshold
        let ok = [true; NUM_CH];
        let contact = [ContactQuality::Good; NUM_CH];
        det.feed(&fft, &ok, &contact);

        let result = det.result.as_ref().unwrap();
        assert_eq!(result.label, AbsorptionLabel::Deep);
    }
}
