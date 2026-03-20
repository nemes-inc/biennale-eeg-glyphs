//! Approach/Withdraw dimension — frontal alpha asymmetry (FAA).
//!
//! # Neural Signature
//!
//! Frontal alpha asymmetry between AF7 (left) and AF8 (right) during a
//! threshold pause (silence / ambiguity).
//!
//! # Formula
//!
//! ```text
//! alpha_left  = mean_power(AF7, 8–12 Hz, during pause)
//! alpha_right = mean_power(AF8, 8–12 Hz, during pause)
//!
//! asymmetry = (alpha_right − alpha_left) / (alpha_right + alpha_left)
//!
//! Positive → more right alpha → more LEFT cortical activity → Approach ("Lean")
//! Negative → more left alpha  → more RIGHT cortical activity → Withdraw ("Hold")
//! ```
//!
//! # Classification
//!
//! | Score              | Label | Confidence |
//! |--------------------|-------|------------|
//! | > +deadband        | Lean  | High       |
//! | 0 … +deadband      | Lean  | Low        |
//! | −deadband … 0      | Hold  | Low        |
//! | < −deadband        | Hold  | High       |
//!
//! Default deadband: ±0.10 (conservative, exceeds Muse noise floor ±0.05–0.08).

use std::time::Instant;

use crate::alpha::{FftSnapshot, NUM_CH, CH_AF7, CH_AF8};
use crate::compute::ContactQuality;

// ── Configuration ────────────────────────────────────────────────────────────

/// Tuneable parameters for the approach/withdraw measurement.
#[derive(Debug, Clone)]
pub struct ApproachConfig {
    /// Deadband half-width for high-confidence classification.
    /// Scores inside ±deadband are classified with low confidence.
    pub deadband: f64,
    /// Total measurement duration in seconds (including settling).
    pub measurement_duration_s: f64,
    /// Initial settling period (discarded) in seconds.
    pub settling_duration_s: f64,
}

impl Default for ApproachConfig {
    fn default() -> Self {
        Self {
            deadband: 0.10,
            measurement_duration_s: 23.0, // 3 s settling + 20 s recording
            settling_duration_s: 3.0,
        }
    }
}

impl ApproachConfig {
    /// Effective recording duration (total minus settling).
    pub fn recording_duration_s(&self) -> f64 {
        self.measurement_duration_s - self.settling_duration_s
    }
}

// ── Classification ──────────────────────────────────────────────────────────

/// Binary classification output.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ApproachLabel {
    /// Approach motivation — lean toward the unknown.
    Lean,
    /// Withdrawal motivation — hold back from the unknown.
    Hold,
}

impl std::fmt::Display for ApproachLabel {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Lean => write!(f, "Lean"),
            Self::Hold => write!(f, "Hold"),
        }
    }
}

/// Confidence level of the classification.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ApproachConfidence {
    /// Asymmetry score exceeds the deadband — strong signal.
    High,
    /// Asymmetry score is within the deadband — classified by direction only.
    Low,
}

impl std::fmt::Display for ApproachConfidence {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::High => write!(f, "High"),
            Self::Low => write!(f, "Low"),
        }
    }
}

// ── Phase ───────────────────────────────────────────────────────────────────

/// Current phase of the approach detector.
#[derive(Debug, Clone, PartialEq)]
pub enum ApproachPhase {
    /// Not running. Press key to start.
    Idle,
    /// Initial settling period — data discarded.
    Settling,
    /// Actively measuring frontal alpha asymmetry.
    Measuring,
    /// Measurement complete — result available.
    Complete,
}

// ── Per-channel accumulator ────────────────────────────────────────────────

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

    pub fn mean_alpha(&self) -> f64 {
        if self.alpha_powers.is_empty() {
            return 0.0;
        }
        self.alpha_powers.iter().map(|&v| v as f64).sum::<f64>()
            / self.alpha_powers.len() as f64
    }

    pub fn artifact_ratio(&self) -> f64 {
        if self.total_snapshots == 0 {
            return 0.0;
        }
        1.0 - (self.clean_snapshots as f64 / self.total_snapshots as f64)
    }
}

// ── Result ──────────────────────────────────────────────────────────────────

/// Final approach/withdraw measurement result.
#[derive(Debug, Clone)]
pub struct ApproachResult {
    /// Mean alpha power for AF7 (left frontal) during measurement.
    pub alpha_left: f64,
    /// Mean alpha power for AF8 (right frontal) during measurement.
    pub alpha_right: f64,
    /// Normalized asymmetry score: (R − L) / (R + L).
    /// Range: [−1.0, +1.0].
    pub asymmetry: f64,
    /// Deadband used for classification.
    pub deadband: f64,
    /// Binary label.
    pub label: ApproachLabel,
    /// Confidence level.
    pub confidence: ApproachConfidence,
    /// Worst-channel artifact ratio during measurement.
    pub artifact_ratio: f64,
    /// Effective recording duration (seconds).
    pub recording_duration_s: f64,
    /// Total clean snapshots accumulated (sum of AF7 + AF8).
    pub total_clean_snapshots: usize,
}

// ── Detector ────────────────────────────────────────────────────────────────

/// Compute normalized alpha asymmetry: (right − left) / (right + left).
///
/// Returns 0.0 if the denominator is near zero.
pub fn normalized_asymmetry(right: f64, left: f64) -> f64 {
    let denom = right + left;
    if denom.abs() < 1e-12 {
        return 0.0;
    }
    (right - left) / denom
}

/// Classify an asymmetry score with a deadband.
pub fn classify(asymmetry: f64, deadband: f64) -> (ApproachLabel, ApproachConfidence) {
    let label = if asymmetry >= 0.0 {
        ApproachLabel::Lean
    } else {
        ApproachLabel::Hold
    };

    let confidence = if asymmetry.abs() > deadband {
        ApproachConfidence::High
    } else {
        ApproachConfidence::Low
    };

    (label, confidence)
}

/// State machine that measures approach/withdraw via frontal alpha asymmetry.
#[derive(Debug, Clone)]
pub struct ApproachDetector {
    pub config: ApproachConfig,
    pub phase: ApproachPhase,
    start_time: Option<Instant>,
    /// Per-channel accumulators (only AF7 and AF8 are used for the score).
    pub channels: [ChannelAccum; NUM_CH],
    /// Completed result.
    pub result: Option<ApproachResult>,
}

impl Default for ApproachDetector {
    fn default() -> Self {
        Self {
            config: ApproachConfig::default(),
            phase: ApproachPhase::Idle,
            start_time: None,
            channels: std::array::from_fn(|_| ChannelAccum::default()),
            result: None,
        }
    }
}

impl ApproachDetector {
    /// Start a new measurement.
    pub fn start(&mut self) {
        self.phase = ApproachPhase::Settling;
        self.start_time = Some(Instant::now());
        for ch in &mut self.channels {
            ch.reset();
        }
        self.result = None;
    }

    /// Stop measurement and return to Idle.
    pub fn stop(&mut self) {
        self.phase = ApproachPhase::Idle;
        self.start_time = None;
    }

    /// Whether the detector is actively running (settling or measuring).
    pub fn is_running(&self) -> bool {
        matches!(self.phase, ApproachPhase::Settling | ApproachPhase::Measuring)
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

    /// Compute live asymmetry from accumulated data.
    ///
    /// Returns `None` if not enough data yet.
    pub fn live_asymmetry(&self) -> Option<f64> {
        let af7 = &self.channels[CH_AF7];
        let af8 = &self.channels[CH_AF8];
        if af7.alpha_powers.is_empty() || af8.alpha_powers.is_empty() {
            return None;
        }
        Some(normalized_asymmetry(af8.mean_alpha(), af7.mean_alpha()))
    }

    /// Live classification based on current accumulated data.
    pub fn live_classification(&self) -> Option<(ApproachLabel, ApproachConfidence)> {
        self.live_asymmetry().map(|a| classify(a, self.config.deadband))
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
    ) -> &ApproachPhase {
        let elapsed = self.elapsed_s();

        match self.phase {
            ApproachPhase::Idle | ApproachPhase::Complete => {
                return &self.phase;
            }
            ApproachPhase::Settling => {
                if elapsed >= self.config.settling_duration_s {
                    self.phase = ApproachPhase::Measuring;
                } else {
                    return &self.phase;
                }
            }
            ApproachPhase::Measuring => { /* process below */ }
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
        let alpha_left = self.channels[CH_AF7].mean_alpha();
        let alpha_right = self.channels[CH_AF8].mean_alpha();
        let asymmetry = normalized_asymmetry(alpha_right, alpha_left);
        let (label, confidence) = classify(asymmetry, self.config.deadband);

        let worst_artifact = self
            .channels
            .iter()
            .map(|ch| ch.artifact_ratio())
            .fold(0.0_f64, f64::max);

        let total_clean: usize = self.channels[CH_AF7].clean_snapshots
            + self.channels[CH_AF8].clean_snapshots;

        self.result = Some(ApproachResult {
            alpha_left,
            alpha_right,
            asymmetry,
            deadband: self.config.deadband,
            label,
            confidence,
            artifact_ratio: worst_artifact,
            recording_duration_s: self.measuring_elapsed_s(),
            total_clean_snapshots: total_clean,
        });

        self.phase = ApproachPhase::Complete;
    }
}

// ── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::alpha::FftBandPower;

    fn make_fft(af7_alpha: f32, af8_alpha: f32) -> FftSnapshot {
        let make_bp = |alpha: f32| {
            let total = 1.0 + 0.5 + alpha + 0.3 + 0.1;
            FftBandPower {
                delta_power: 1.0,
                theta_power: 0.5,
                alpha_power: alpha,
                beta_power: 0.3,
                gamma_power: 0.1,
                total_power: total,
                relative_alpha: alpha / total,
            }
        };

        let mut channels = [make_bp(1.0); NUM_CH];
        channels[CH_AF7] = make_bp(af7_alpha);
        channels[CH_AF8] = make_bp(af8_alpha);

        FftSnapshot {
            channels,
            frontal_asymmetry: 0.0,
            temporal_asymmetry: 0.0,
            mean_relative_alpha: 0.0,
        }
    }

    // ── normalized_asymmetry ─────────────────────────────────────────

    #[test]
    fn asymmetry_balanced() {
        assert!((normalized_asymmetry(10.0, 10.0)).abs() < 1e-10);
    }

    #[test]
    fn asymmetry_right_dominant() {
        // More right alpha → positive → Lean (approach)
        let a = normalized_asymmetry(20.0, 10.0);
        assert!(a > 0.0);
        // (20 - 10) / (20 + 10) = 1/3
        assert!((a - 1.0 / 3.0).abs() < 1e-10);
    }

    #[test]
    fn asymmetry_left_dominant() {
        // More left alpha → negative → Hold (withdraw)
        let a = normalized_asymmetry(10.0, 20.0);
        assert!(a < 0.0);
        assert!((a - (-1.0 / 3.0)).abs() < 1e-10);
    }

    #[test]
    fn asymmetry_zero_denom() {
        assert_eq!(normalized_asymmetry(0.0, 0.0), 0.0);
    }

    // ── classify ─────────────────────────────────────────────────────

    #[test]
    fn classify_strong_approach() {
        let (label, conf) = classify(0.15, 0.10);
        assert_eq!(label, ApproachLabel::Lean);
        assert_eq!(conf, ApproachConfidence::High);
    }

    #[test]
    fn classify_strong_withdraw() {
        let (label, conf) = classify(-0.15, 0.10);
        assert_eq!(label, ApproachLabel::Hold);
        assert_eq!(conf, ApproachConfidence::High);
    }

    #[test]
    fn classify_weak_approach() {
        let (label, conf) = classify(0.05, 0.10);
        assert_eq!(label, ApproachLabel::Lean);
        assert_eq!(conf, ApproachConfidence::Low);
    }

    #[test]
    fn classify_weak_withdraw() {
        let (label, conf) = classify(-0.05, 0.10);
        assert_eq!(label, ApproachLabel::Hold);
        assert_eq!(conf, ApproachConfidence::Low);
    }

    #[test]
    fn classify_exactly_zero() {
        // Zero → Lean with low confidence (per spec: >= 0 → Lean)
        let (label, conf) = classify(0.0, 0.10);
        assert_eq!(label, ApproachLabel::Lean);
        assert_eq!(conf, ApproachConfidence::Low);
    }

    #[test]
    fn classify_at_boundary() {
        // Exactly at deadband → Low confidence (not strictly above)
        let (label, conf) = classify(0.10, 0.10);
        assert_eq!(label, ApproachLabel::Lean);
        assert_eq!(conf, ApproachConfidence::Low);
    }

    // ── Detector lifecycle ──────────────────────────────────────────

    #[test]
    fn idle_by_default() {
        let det = ApproachDetector::default();
        assert_eq!(det.phase, ApproachPhase::Idle);
        assert!(det.result.is_none());
    }

    #[test]
    fn start_transitions_to_settling() {
        let mut det = ApproachDetector::default();
        det.start();
        assert_eq!(det.phase, ApproachPhase::Settling);
        assert!(det.is_running());
    }

    #[test]
    fn stop_returns_to_idle() {
        let mut det = ApproachDetector::default();
        det.start();
        det.stop();
        assert_eq!(det.phase, ApproachPhase::Idle);
        assert!(!det.is_running());
    }

    #[test]
    fn approach_classification_lean() {
        // AF7 alpha = 8.0, AF8 alpha = 12.0
        // asymmetry = (12 - 8) / (12 + 8) = 0.2 → Lean, High
        let mut det = ApproachDetector::default();
        det.config.measurement_duration_s = 0.0;
        det.config.settling_duration_s = 0.0;
        det.start();

        let fft = make_fft(8.0, 12.0);
        let ok = [true; NUM_CH];
        let contact = [ContactQuality::Good; NUM_CH];
        det.feed(&fft, &ok, &contact);

        assert_eq!(det.phase, ApproachPhase::Complete);
        let r = det.result.as_ref().unwrap();
        assert!((r.asymmetry - 0.2).abs() < 0.01);
        assert_eq!(r.label, ApproachLabel::Lean);
        assert_eq!(r.confidence, ApproachConfidence::High);
    }

    #[test]
    fn approach_classification_hold() {
        // AF7 alpha = 12.0, AF8 alpha = 8.0
        // asymmetry = (8 - 12) / (8 + 12) = -0.2 → Hold, High
        let mut det = ApproachDetector::default();
        det.config.measurement_duration_s = 0.0;
        det.config.settling_duration_s = 0.0;
        det.start();

        let fft = make_fft(12.0, 8.0);
        let ok = [true; NUM_CH];
        let contact = [ContactQuality::Good; NUM_CH];
        det.feed(&fft, &ok, &contact);

        assert_eq!(det.phase, ApproachPhase::Complete);
        let r = det.result.as_ref().unwrap();
        assert!((r.asymmetry - (-0.2)).abs() < 0.01);
        assert_eq!(r.label, ApproachLabel::Hold);
        assert_eq!(r.confidence, ApproachConfidence::High);
    }

    #[test]
    fn weak_signal_low_confidence() {
        // AF7 = 10.0, AF8 = 10.5
        // asymmetry = (10.5 - 10.0) / (10.5 + 10.0) ≈ 0.024 → Lean, Low
        let mut det = ApproachDetector::default();
        det.config.measurement_duration_s = 0.0;
        det.config.settling_duration_s = 0.0;
        det.start();

        let fft = make_fft(10.0, 10.5);
        let ok = [true; NUM_CH];
        let contact = [ContactQuality::Good; NUM_CH];
        det.feed(&fft, &ok, &contact);

        assert_eq!(det.phase, ApproachPhase::Complete);
        let r = det.result.as_ref().unwrap();
        assert!(r.asymmetry.abs() < 0.10);
        assert_eq!(r.label, ApproachLabel::Lean);
        assert_eq!(r.confidence, ApproachConfidence::Low);
    }

    #[test]
    fn artifact_gating() {
        // If contact is NoContact, samples should be excluded
        let mut det = ApproachDetector::default();
        det.config.measurement_duration_s = 0.0;
        det.config.settling_duration_s = 0.0;
        det.start();

        let fft = make_fft(10.0, 10.0);
        let ok = [true; NUM_CH];
        let mut contact = [ContactQuality::Good; NUM_CH];
        contact[CH_AF7] = ContactQuality::NoContact;

        det.feed(&fft, &ok, &contact);

        assert_eq!(det.phase, ApproachPhase::Complete);
        // AF7 had no clean samples → mean_alpha = 0.0
        // asymmetry = (10.0 - 0.0) / (10.0 + 0.0) = 1.0
        let r = det.result.as_ref().unwrap();
        assert_eq!(r.alpha_left, 0.0);
        assert!(r.artifact_ratio > 0.0);
    }

    #[test]
    fn deadband_adjustment() {
        // Test with tighter deadband (0.05)
        let mut det = ApproachDetector::default();
        det.config.deadband = 0.05;
        det.config.measurement_duration_s = 0.0;
        det.config.settling_duration_s = 0.0;
        det.start();

        // asymmetry ≈ 0.073 → above 0.05 deadband → High confidence
        let fft = make_fft(10.0, 11.5);
        let ok = [true; NUM_CH];
        let contact = [ContactQuality::Good; NUM_CH];
        det.feed(&fft, &ok, &contact);

        let r = det.result.as_ref().unwrap();
        assert_eq!(r.confidence, ApproachConfidence::High);
    }

    #[test]
    fn config_defaults() {
        let cfg = ApproachConfig::default();
        assert!((cfg.deadband - 0.10).abs() < 1e-10);
        assert!((cfg.measurement_duration_s - 23.0).abs() < 1e-10);
        assert!((cfg.settling_duration_s - 3.0).abs() < 1e-10);
        assert!((cfg.recording_duration_s() - 20.0).abs() < 1e-10);
    }
}
