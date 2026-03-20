//! Baseline detector for EEG resting-state measurement.
//!
//! Implements a two-phase protocol (settling → recording) that accumulates
//! clean FFT snapshots to compute per-channel baseline alpha power, frontal
//! asymmetry, and noise-band estimates.
//!
//! # Protocol
//!
//! | Phase      | Default duration | Purpose                        |
//! |------------|-----------------|--------------------------------|
//! | Settling   | 15 s            | Let alpha stabilise; data discarded |
//! | Recording  | 45 s            | Accumulate clean epochs         |
//!
//! The detector auto-completes when the recording phase ends, or fails if
//! more than 30 % of epochs are rejected by artifact checks.

use std::time::Instant;

use crate::alpha::{FftSnapshot, NUM_CH, CH_AF7, CH_AF8};
use crate::compute::ContactQuality;

// ── Configuration ────────────────────────────────────────────────────────────

/// Tuneable parameters for the baseline protocol.
#[derive(Debug, Clone)]
pub struct BaselineConfig {
    /// Total baseline duration in seconds (settling + recording).
    pub total_duration_s: f64,
    /// Duration of the initial settling period (discarded data).
    pub settling_duration_s: f64,
    /// Maximum allowed artifact ratio (0.0–1.0). If exceeded → fail.
    pub max_artifact_ratio: f64,
    /// Maximum allowed coefficient of variation for alpha across epochs.
    pub max_alpha_cv: f64,
}

impl Default for BaselineConfig {
    fn default() -> Self {
        Self {
            total_duration_s: 60.0,
            settling_duration_s: 15.0,
            max_artifact_ratio: 0.30,
            max_alpha_cv: 0.30,
        }
    }
}

impl BaselineConfig {
    /// Effective recording duration (total minus settling).
    pub fn recording_duration_s(&self) -> f64 {
        self.total_duration_s - self.settling_duration_s
    }
}

// ── Phase ────────────────────────────────────────────────────────────────────

/// Current phase of the baseline detector.
#[derive(Debug, Clone, PartialEq)]
pub enum BaselinePhase {
    /// Not running. Press key to start.
    Idle,
    /// Initial settling period — data is observed but discarded.
    Settling,
    /// Actively recording clean epochs.
    Recording,
    /// Successfully completed baseline collection.
    Complete,
    /// Failed due to quality issues.
    Failed(String),
}

// ── Per-channel accumulator ──────────────────────────────────────────────────

/// Per-channel statistics accumulated during recording.
#[derive(Debug, Clone, Default)]
pub struct ChannelAccum {
    /// Alpha powers from each clean snapshot (µV²).
    pub alpha_powers: Vec<f32>,
    /// Delta powers from each clean snapshot (proxy for 1–3 Hz noise).
    pub delta_powers: Vec<f32>,
    /// Theta powers from each clean snapshot (proxy for 3–7 Hz noise).
    pub theta_powers: Vec<f32>,
    /// Total snapshots offered to this channel (clean + rejected).
    pub total_snapshots: usize,
    /// Snapshots that passed artifact checks.
    pub clean_snapshots: usize,
}

impl ChannelAccum {
    fn reset(&mut self) {
        self.alpha_powers.clear();
        self.delta_powers.clear();
        self.theta_powers.clear();
        self.total_snapshots = 0;
        self.clean_snapshots = 0;
    }

    /// Mean alpha power across clean snapshots.
    pub fn mean_alpha(&self) -> f64 {
        if self.alpha_powers.is_empty() {
            return 0.0;
        }
        self.alpha_powers.iter().map(|&v| v as f64).sum::<f64>() / self.alpha_powers.len() as f64
    }

    /// Coefficient of variation of alpha power (std / mean).
    pub fn alpha_cv(&self) -> f64 {
        let n = self.alpha_powers.len();
        if n < 2 {
            return f64::MAX;
        }
        let mean = self.mean_alpha();
        if mean <= 0.0 {
            return f64::MAX;
        }
        let var = self
            .alpha_powers
            .iter()
            .map(|&v| {
                let d = v as f64 - mean;
                d * d
            })
            .sum::<f64>()
            / (n - 1) as f64;
        var.sqrt() / mean
    }

    /// Fraction of snapshots rejected (0.0–1.0).
    pub fn artifact_ratio(&self) -> f64 {
        if self.total_snapshots == 0 {
            return 0.0;
        }
        1.0 - (self.clean_snapshots as f64 / self.total_snapshots as f64)
    }

    /// Mean delta power (proxy for 1–3 Hz noise).
    pub fn mean_delta(&self) -> f64 {
        if self.delta_powers.is_empty() {
            return 0.0;
        }
        self.delta_powers.iter().map(|&v| v as f64).sum::<f64>() / self.delta_powers.len() as f64
    }

    /// Mean theta power (proxy for 3–7 Hz noise).
    pub fn mean_theta(&self) -> f64 {
        if self.theta_powers.is_empty() {
            return 0.0;
        }
        self.theta_powers.iter().map(|&v| v as f64).sum::<f64>() / self.theta_powers.len() as f64
    }
}

// ── Baseline result ──────────────────────────────────────────────────────────

/// Final computed baseline metrics, stored after a successful collection.
#[derive(Debug, Clone)]
pub struct BaselineResult {
    /// Per-channel mean alpha power (µV²).
    pub channel_alpha: [f64; NUM_CH],
    /// Per-channel alpha coefficient of variation.
    pub channel_alpha_cv: [f64; NUM_CH],
    /// Per-channel artifact rejection ratio (0.0–1.0).
    pub channel_artifact_ratio: [f64; NUM_CH],
    /// Frontal alpha asymmetry: (AF8 − AF7) / (AF8 + AF7).
    pub alpha_asymmetry: f64,
    /// Mean delta power across all channels (proxy for 1–3 Hz noise).
    pub noise_delta: f64,
    /// Mean theta power across all channels (proxy for 3–7 Hz noise).
    pub noise_theta: f64,
    /// Overall artifact ratio (worst channel).
    pub overall_artifact_ratio: f64,
    /// Actual recording duration (seconds).
    pub recording_duration_s: f64,
    /// Total clean snapshots accumulated across all channels.
    pub total_clean_snapshots: usize,
}

// ── Detector ─────────────────────────────────────────────────────────────────

/// State machine that implements the two-phase baseline protocol.
#[derive(Debug, Clone)]
pub struct BaselineDetector {
    pub config: BaselineConfig,
    pub phase: BaselinePhase,
    /// Wall-clock instant when `start()` was called.
    start_time: Option<Instant>,
    /// Per-channel accumulators.
    pub channels: [ChannelAccum; NUM_CH],
    /// Completed baseline (set when phase transitions to `Complete`).
    pub result: Option<BaselineResult>,
}

impl Default for BaselineDetector {
    fn default() -> Self {
        Self {
            config: BaselineConfig::default(),
            phase: BaselinePhase::Idle,
            start_time: None,
            channels: std::array::from_fn(|_| ChannelAccum::default()),
            result: None,
        }
    }
}

impl BaselineDetector {
    /// Start (or restart) the baseline collection.
    pub fn start(&mut self) {
        self.phase = BaselinePhase::Settling;
        self.start_time = Some(Instant::now());
        for ch in &mut self.channels {
            ch.reset();
        }
        self.result = None;
    }

    /// Stop the baseline collection and return to idle.
    pub fn stop(&mut self) {
        self.phase = BaselinePhase::Idle;
        self.start_time = None;
    }

    /// Seconds elapsed since start (0.0 if not running).
    pub fn elapsed_s(&self) -> f64 {
        self.start_time
            .map(|t| t.elapsed().as_secs_f64())
            .unwrap_or(0.0)
    }

    /// Seconds remaining in the current phase.
    pub fn remaining_s(&self) -> f64 {
        let elapsed = self.elapsed_s();
        match self.phase {
            BaselinePhase::Settling => {
                (self.config.settling_duration_s - elapsed).max(0.0)
            }
            BaselinePhase::Recording => {
                (self.config.total_duration_s - elapsed).max(0.0)
            }
            _ => 0.0,
        }
    }

    /// Progress fraction (0.0–1.0) across the entire protocol.
    pub fn progress(&self) -> f64 {
        let elapsed = self.elapsed_s();
        (elapsed / self.config.total_duration_s).clamp(0.0, 1.0)
    }

    /// Seconds elapsed in the recording phase specifically.
    pub fn recording_elapsed_s(&self) -> f64 {
        let elapsed = self.elapsed_s();
        (elapsed - self.config.settling_duration_s).max(0.0)
    }

    /// Feed a new FFT snapshot from the signal pipeline.
    ///
    /// `epoch_ok[ch]` indicates whether the epoch validator passed for that
    /// channel. `contact` provides the current horseshoe quality per channel.
    ///
    /// Returns the current phase after processing.
    pub fn feed(
        &mut self,
        fft: &FftSnapshot,
        epoch_ok: &[bool; NUM_CH],
        contact: &[ContactQuality; NUM_CH],
    ) -> &BaselinePhase {
        let elapsed = self.elapsed_s();

        match self.phase {
            BaselinePhase::Idle | BaselinePhase::Complete | BaselinePhase::Failed(_) => {
                return &self.phase;
            }
            BaselinePhase::Settling => {
                if elapsed >= self.config.settling_duration_s {
                    self.phase = BaselinePhase::Recording;
                    // Fall through to process this snapshot as recording
                } else {
                    return &self.phase;
                }
            }
            BaselinePhase::Recording => { /* process below */ }
        }

        // ── Recording phase: accumulate per-channel data ────────────────
        for ch in 0..NUM_CH {
            self.channels[ch].total_snapshots += 1;

            let is_clean = epoch_ok[ch]
                && contact[ch] != ContactQuality::NoContact;

            if is_clean {
                self.channels[ch].clean_snapshots += 1;
                self.channels[ch].alpha_powers.push(fft.channels[ch].alpha_power);
                self.channels[ch].delta_powers.push(fft.channels[ch].delta_power);
                self.channels[ch].theta_powers.push(fft.channels[ch].theta_power);
            }
        }

        // ── Check completion / failure ──────────────────────────────────
        if elapsed >= self.config.total_duration_s {
            self.finish();
        }

        &self.phase
    }

    /// Compute final metrics and transition to Complete or Failed.
    fn finish(&mut self) {
        // Check artifact ratios
        let worst_artifact = self
            .channels
            .iter()
            .map(|ch| ch.artifact_ratio())
            .fold(0.0_f64, f64::max);

        if worst_artifact > self.config.max_artifact_ratio {
            self.phase = BaselinePhase::Failed(format!(
                "Artifact ratio {:.0}% exceeds {:.0}% limit — restart baseline",
                worst_artifact * 100.0,
                self.config.max_artifact_ratio * 100.0,
            ));
            return;
        }

        // Check we have enough data
        let min_clean = self.channels.iter().map(|ch| ch.clean_snapshots).min().unwrap_or(0);
        if min_clean < 10 {
            self.phase = BaselinePhase::Failed(
                "Too few clean snapshots — check electrode contact".to_string(),
            );
            return;
        }

        // Check alpha CV
        let worst_cv = self
            .channels
            .iter()
            .map(|ch| ch.alpha_cv())
            .fold(0.0_f64, f64::max);

        // Build result
        let channel_alpha: [f64; NUM_CH] = std::array::from_fn(|i| self.channels[i].mean_alpha());
        let channel_alpha_cv: [f64; NUM_CH] = std::array::from_fn(|i| self.channels[i].alpha_cv());
        let channel_artifact_ratio: [f64; NUM_CH] =
            std::array::from_fn(|i| self.channels[i].artifact_ratio());

        let af7_alpha = channel_alpha[CH_AF7];
        let af8_alpha = channel_alpha[CH_AF8];
        let alpha_asymmetry = if (af7_alpha + af8_alpha) > 0.0 {
            (af8_alpha - af7_alpha) / (af8_alpha + af7_alpha)
        } else {
            0.0
        };

        let noise_delta = self.channels.iter().map(|ch| ch.mean_delta()).sum::<f64>() / NUM_CH as f64;
        let noise_theta = self.channels.iter().map(|ch| ch.mean_theta()).sum::<f64>() / NUM_CH as f64;

        let total_clean: usize = self.channels.iter().map(|ch| ch.clean_snapshots).sum();

        let result = BaselineResult {
            channel_alpha,
            channel_alpha_cv,
            channel_artifact_ratio,
            alpha_asymmetry,
            noise_delta,
            noise_theta,
            overall_artifact_ratio: worst_artifact,
            recording_duration_s: self.recording_elapsed_s(),
            total_clean_snapshots: total_clean,
        };

        // Warn about high CV but still complete (it's informational)
        if worst_cv > self.config.max_alpha_cv {
            log::warn!(
                "Baseline alpha CV {:.0}% exceeds {:.0}% target — results may be less reliable",
                worst_cv * 100.0,
                self.config.max_alpha_cv * 100.0,
            );
        }

        self.result = Some(result);
        self.phase = BaselinePhase::Complete;
    }

    /// Whether the detector is actively running (settling or recording).
    pub fn is_running(&self) -> bool {
        matches!(self.phase, BaselinePhase::Settling | BaselinePhase::Recording)
    }
}

// ── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::alpha::FftBandPower;

    fn make_snapshot(alpha: f32) -> FftSnapshot {
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
    fn channel_accum_mean_and_cv() {
        let mut acc = ChannelAccum::default();
        // 10 identical values → CV = 0
        for _ in 0..10 {
            acc.alpha_powers.push(5.0);
            acc.clean_snapshots += 1;
            acc.total_snapshots += 1;
        }
        assert!((acc.mean_alpha() - 5.0).abs() < 1e-6);
        assert!(acc.alpha_cv() < 1e-6);
        assert!(acc.artifact_ratio() < 1e-6);
    }

    #[test]
    fn channel_accum_artifact_ratio() {
        let mut acc = ChannelAccum::default();
        acc.total_snapshots = 10;
        acc.clean_snapshots = 7;
        let ratio = acc.artifact_ratio();
        assert!((ratio - 0.3).abs() < 1e-6);
    }

    #[test]
    fn detector_idle_by_default() {
        let det = BaselineDetector::default();
        assert_eq!(det.phase, BaselinePhase::Idle);
        assert!(det.result.is_none());
    }

    #[test]
    fn detector_start_resets() {
        let mut det = BaselineDetector::default();
        det.channels[0].total_snapshots = 99;
        det.start();
        assert_eq!(det.phase, BaselinePhase::Settling);
        assert_eq!(det.channels[0].total_snapshots, 0);
    }

    #[test]
    fn detector_stop_returns_idle() {
        let mut det = BaselineDetector::default();
        det.start();
        det.stop();
        assert_eq!(det.phase, BaselinePhase::Idle);
    }
}
