//! Alpha Trend — single-phase continuous alpha power trend analysis.
//!
//! # Neural Signature
//!
//! Tracks whether frontal alpha power (AF7 + AF8 mean, 8–13 Hz) is trending
//! upward or downward over the course of a session, using online linear
//! regression.  No separate baseline phase is required.
//!
//! # Algorithm
//!
//! ```text
//! For each clean FFT snapshot at time t (seconds since start):
//!   α_frontal = (alpha_power(AF7) + alpha_power(AF8)) / 2
//!
//! Online least-squares linear regression:  α = slope·t + intercept
//!
//!   slope = (n·Σtα − Σt·Σα) / (n·Σt² − (Σt)²)
//!
//! Normalised slope (%/min):
//!   trend = (slope / ᾱ) × 60 × 100
//!
//! R² = explained variance / total variance
//!
//! Label = Rising   if trend >  +deadband
//!         Falling  if trend <  −deadband
//!         Flat     otherwise
//! ```

use std::time::Instant;

use crate::alpha::{FftSnapshot, NUM_CH, CH_AF7, CH_AF8};
use crate::compute::ContactQuality;

// ── Configuration ────────────────────────────────────────────────────────────

/// Tuneable parameters for the alpha trend measurement.
#[derive(Debug, Clone)]
pub struct AlphaTrendConfig {
    /// Deadband for classification (normalised %/min).
    /// Trend values inside ±deadband are labelled "Flat".
    pub deadband_pct_per_min: f64,
    /// Initial settling period (seconds) — data discarded.
    pub settling_duration_s: f64,
    /// Maximum number of samples to keep in the history ring buffer for display.
    pub max_history: usize,
}

impl Default for AlphaTrendConfig {
    fn default() -> Self {
        Self {
            deadband_pct_per_min: 2.0,
            settling_duration_s: 5.0,
            max_history: 1200, // ~5 min at 4 Hz hop rate
        }
    }
}

// ── Classification ──────────────────────────────────────────────────────────

/// Trend direction label.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrendLabel {
    Rising,
    Flat,
    Falling,
}

impl std::fmt::Display for TrendLabel {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Rising => write!(f, "Rising"),
            Self::Flat => write!(f, "Flat"),
            Self::Falling => write!(f, "Falling"),
        }
    }
}

// ── Phase ───────────────────────────────────────────────────────────────────

/// Current phase of the alpha trend detector.
#[derive(Debug, Clone, PartialEq)]
pub enum AlphaTrendPhase {
    /// Not running.
    Idle,
    /// Initial settling period — data discarded.
    Settling,
    /// Actively collecting and computing trend.
    Measuring,
    /// Stopped — final result available.
    Complete,
}

// ── Online regression accumulator ───────────────────────────────────────────

/// Welford-style online linear regression accumulator.
///
/// Computes slope, intercept, R² incrementally without storing all points.
#[derive(Debug, Clone, Default)]
struct LinRegAccum {
    n: u64,
    sum_t: f64,
    sum_y: f64,
    sum_tt: f64,
    sum_ty: f64,
    sum_yy: f64,
}

impl LinRegAccum {
    fn reset(&mut self) {
        *self = Self::default();
    }

    fn push(&mut self, t: f64, y: f64) {
        self.n += 1;
        self.sum_t += t;
        self.sum_y += y;
        self.sum_tt += t * t;
        self.sum_ty += t * y;
        self.sum_yy += y * y;
    }

    fn count(&self) -> u64 {
        self.n
    }

    fn mean_y(&self) -> f64 {
        if self.n == 0 { return 0.0; }
        self.sum_y / self.n as f64
    }

    /// Slope of the best-fit line: α = slope·t + intercept.
    fn slope(&self) -> f64 {
        let n = self.n as f64;
        let denom = n * self.sum_tt - self.sum_t * self.sum_t;
        if denom.abs() < 1e-30 || self.n < 2 {
            return 0.0;
        }
        (n * self.sum_ty - self.sum_t * self.sum_y) / denom
    }

    /// Intercept of the best-fit line.
    fn intercept(&self) -> f64 {
        let n = self.n as f64;
        if self.n < 2 {
            return self.mean_y();
        }
        (self.sum_y - self.slope() * self.sum_t) / n
    }

    /// Coefficient of determination R² ∈ [0, 1].
    fn r_squared(&self) -> f64 {
        let n = self.n as f64;
        if self.n < 3 {
            return 0.0;
        }
        let ss_tot = self.sum_yy - self.sum_y * self.sum_y / n;
        if ss_tot.abs() < 1e-30 {
            return 0.0;
        }
        let slope = self.slope();
        let intercept = self.intercept();
        // SS_res = Σ(y - ŷ)² = Σy² - 2·slope·Σty - 2·intercept·Σy + slope²·Σt² + 2·slope·intercept·Σt + n·intercept²
        let ss_res = self.sum_yy
            - 2.0 * slope * self.sum_ty
            - 2.0 * intercept * self.sum_y
            + slope * slope * self.sum_tt
            + 2.0 * slope * intercept * self.sum_t
            + n * intercept * intercept;
        (1.0 - ss_res / ss_tot).clamp(0.0, 1.0)
    }
}

// ── Result ──────────────────────────────────────────────────────────────────

/// Snapshot of current trend analysis (available live and at completion).
#[derive(Debug, Clone)]
pub struct AlphaTrendResult {
    /// Raw slope in alpha-power-units per second.
    pub slope: f64,
    /// Mean alpha power across the measurement so far.
    pub mean_alpha: f64,
    /// Normalised trend: (slope / mean_alpha) × 60 × 100, in %/min.
    pub trend_pct_per_min: f64,
    /// R² of the linear fit (0 = no fit, 1 = perfect line).
    pub r_squared: f64,
    /// Classification label.
    pub label: TrendLabel,
    /// Deadband used.
    pub deadband: f64,
    /// Number of clean samples accumulated.
    pub n_samples: u64,
    /// Duration of measurement so far (seconds).
    pub duration_s: f64,
    /// Artifact ratio (worst channel of AF7/AF8).
    pub artifact_ratio: f64,
}

// ── Detector ────────────────────────────────────────────────────────────────

/// State machine that tracks frontal alpha power trend over a session.
#[derive(Debug, Clone)]
pub struct AlphaTrendDetector {
    pub config: AlphaTrendConfig,
    pub phase: AlphaTrendPhase,
    start_time: Option<Instant>,
    reg: LinRegAccum,
    /// Rolling history of (elapsed_s, frontal_alpha) for sparkline display.
    pub history: Vec<(f64, f64)>,
    /// Total/clean snapshot counts for AF7 and AF8 (for artifact ratio).
    total_snapshots: [usize; 2],
    clean_snapshots: [usize; 2],
    /// Latest computed result (updated every feed).
    pub result: Option<AlphaTrendResult>,
}

impl Default for AlphaTrendDetector {
    fn default() -> Self {
        Self {
            config: AlphaTrendConfig::default(),
            phase: AlphaTrendPhase::Idle,
            start_time: None,
            reg: LinRegAccum::default(),
            history: Vec::with_capacity(1200),
            total_snapshots: [0; 2],
            clean_snapshots: [0; 2],
            result: None,
        }
    }
}

impl AlphaTrendDetector {
    /// Start a new trend measurement.
    pub fn start(&mut self) {
        self.phase = AlphaTrendPhase::Settling;
        self.start_time = Some(Instant::now());
        self.reg.reset();
        self.history.clear();
        self.total_snapshots = [0; 2];
        self.clean_snapshots = [0; 2];
        self.result = None;
    }

    /// Stop the measurement and finalise.
    pub fn stop(&mut self) {
        if self.is_running() {
            self.finish();
        } else {
            self.phase = AlphaTrendPhase::Idle;
        }
    }

    /// Whether the detector is actively running.
    pub fn is_running(&self) -> bool {
        matches!(self.phase, AlphaTrendPhase::Settling | AlphaTrendPhase::Measuring)
    }

    /// Seconds elapsed since start.
    pub fn elapsed_s(&self) -> f64 {
        self.start_time
            .map(|t| t.elapsed().as_secs_f64())
            .unwrap_or(0.0)
    }

    /// Seconds in the measuring phase.
    pub fn measuring_elapsed_s(&self) -> f64 {
        (self.elapsed_s() - self.config.settling_duration_s).max(0.0)
    }

    /// Worst artifact ratio across AF7 and AF8.
    pub fn artifact_ratio(&self) -> f64 {
        let ratios: [f64; 2] = std::array::from_fn(|i| {
            if self.total_snapshots[i] == 0 {
                0.0
            } else {
                1.0 - (self.clean_snapshots[i] as f64 / self.total_snapshots[i] as f64)
            }
        });
        ratios[0].max(ratios[1])
    }

    /// Compute the current trend result from accumulated data.
    fn compute_result(&self) -> Option<AlphaTrendResult> {
        if self.reg.count() < 2 {
            return None;
        }
        let slope = self.reg.slope();
        let mean_alpha = self.reg.mean_y();
        let trend = if mean_alpha.abs() > 1e-12 {
            (slope / mean_alpha) * 60.0 * 100.0
        } else {
            0.0
        };
        let r2 = self.reg.r_squared();
        let db = self.config.deadband_pct_per_min;
        let label = if trend > db {
            TrendLabel::Rising
        } else if trend < -db {
            TrendLabel::Falling
        } else {
            TrendLabel::Flat
        };

        Some(AlphaTrendResult {
            slope,
            mean_alpha,
            trend_pct_per_min: trend,
            r_squared: r2,
            label,
            deadband: db,
            n_samples: self.reg.count(),
            duration_s: self.measuring_elapsed_s(),
            artifact_ratio: self.artifact_ratio(),
        })
    }

    /// Feed a new FFT snapshot.
    pub fn feed(
        &mut self,
        fft: &FftSnapshot,
        epoch_ok: &[bool; NUM_CH],
        contact: &[ContactQuality; NUM_CH],
    ) -> &AlphaTrendPhase {
        let elapsed = self.elapsed_s();

        match self.phase {
            AlphaTrendPhase::Idle | AlphaTrendPhase::Complete => {
                return &self.phase;
            }
            AlphaTrendPhase::Settling => {
                if elapsed >= self.config.settling_duration_s {
                    self.phase = AlphaTrendPhase::Measuring;
                } else {
                    return &self.phase;
                }
            }
            AlphaTrendPhase::Measuring => { /* process below */ }
        }

        // ── Measuring: check both frontal channels ──────────────────────
        let af7_clean = epoch_ok[CH_AF7] && contact[CH_AF7] != ContactQuality::NoContact;
        let af8_clean = epoch_ok[CH_AF8] && contact[CH_AF8] != ContactQuality::NoContact;

        self.total_snapshots[0] += 1;
        self.total_snapshots[1] += 1;
        if af7_clean { self.clean_snapshots[0] += 1; }
        if af8_clean { self.clean_snapshots[1] += 1; }

        // Only accumulate when BOTH frontal channels are clean
        if af7_clean && af8_clean {
            let af7_alpha = fft.channels[CH_AF7].alpha_power as f64;
            let af8_alpha = fft.channels[CH_AF8].alpha_power as f64;
            let frontal_alpha = (af7_alpha + af8_alpha) / 2.0;
            let t = self.measuring_elapsed_s();

            self.reg.push(t, frontal_alpha);

            // Store history for sparkline (capped)
            self.history.push((t, frontal_alpha));
            if self.history.len() > self.config.max_history {
                self.history.remove(0);
            }

            // Update live result
            self.result = self.compute_result();
        }

        &self.phase
    }

    /// Finalise measurement and transition to Complete.
    fn finish(&mut self) {
        self.result = self.compute_result();
        self.phase = AlphaTrendPhase::Complete;
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
        FftSnapshot {
            channels: [
                FftBandPower::default(), // TP9
                make_bp(af7_alpha),      // AF7
                make_bp(af8_alpha),      // AF8
                FftBandPower::default(), // TP10
            ],
            frontal_asymmetry: 0.0,
            temporal_asymmetry: 0.0,
            mean_relative_alpha: 0.0,
        }
    }

    #[test]
    fn test_linreg_slope_positive() {
        let mut acc = LinRegAccum::default();
        // y = 2t + 1 → slope = 2
        for i in 0..100 {
            let t = i as f64;
            acc.push(t, 2.0 * t + 1.0);
        }
        assert!((acc.slope() - 2.0).abs() < 1e-6);
        assert!((acc.intercept() - 1.0).abs() < 1e-6);
        assert!((acc.r_squared() - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_linreg_flat() {
        let mut acc = LinRegAccum::default();
        for i in 0..100 {
            acc.push(i as f64, 5.0);
        }
        assert!(acc.slope().abs() < 1e-10);
    }

    #[test]
    fn test_linreg_negative() {
        let mut acc = LinRegAccum::default();
        for i in 0..100 {
            let t = i as f64;
            acc.push(t, -0.5 * t + 10.0);
        }
        assert!((acc.slope() - (-0.5)).abs() < 1e-6);
    }

    #[test]
    fn test_detector_rising_trend() {
        // Bypass real clock by pushing directly into the regression accumulator
        let mut det = AlphaTrendDetector::default();
        det.config.settling_duration_s = 0.0;
        det.phase = AlphaTrendPhase::Measuring;

        // Simulate 240 samples over 60s (0.25s apart) with linearly increasing alpha
        for i in 0..240 {
            let t = i as f64 * 0.25;
            let alpha = 0.5 + 0.01 * i as f64;
            det.reg.push(t, alpha);
            det.history.push((t, alpha));
        }
        det.result = det.compute_result();

        let result = det.result.as_ref().unwrap();
        assert!(result.slope > 0.0);
        assert!(result.trend_pct_per_min > 0.0);
        assert_eq!(result.label, TrendLabel::Rising);
        assert!(result.r_squared > 0.99);
    }

    #[test]
    fn test_detector_falling_trend() {
        let mut det = AlphaTrendDetector::default();
        det.config.settling_duration_s = 0.0;
        det.phase = AlphaTrendPhase::Measuring;

        for i in 0..240 {
            let t = i as f64 * 0.25;
            let alpha = (3.0 - 0.01 * i as f64).max(0.1);
            det.reg.push(t, alpha);
            det.history.push((t, alpha));
        }
        det.result = det.compute_result();

        let result = det.result.as_ref().unwrap();
        assert!(result.slope < 0.0);
        assert!(result.trend_pct_per_min < 0.0);
        assert_eq!(result.label, TrendLabel::Falling);
    }

    #[test]
    fn test_detector_flat_trend() {
        let mut det = AlphaTrendDetector::default();
        det.config.settling_duration_s = 0.0;
        det.config.deadband_pct_per_min = 5.0;
        det.phase = AlphaTrendPhase::Measuring;

        // Constant alpha
        for i in 0..100 {
            let t = i as f64 * 0.25;
            det.reg.push(t, 1.0);
            det.history.push((t, 1.0));
        }
        det.result = det.compute_result();

        let result = det.result.as_ref().unwrap();
        assert_eq!(result.label, TrendLabel::Flat);
    }

    #[test]
    fn test_artifact_rejection() {
        let mut det = AlphaTrendDetector::default();
        det.config.settling_duration_s = 0.0;
        det.start();
        det.phase = AlphaTrendPhase::Measuring;

        let contact = [ContactQuality::Good; NUM_CH];

        // AF7 bad epoch → both channels must be clean, so nothing accumulates
        let mut epoch_ok = [true; NUM_CH];
        epoch_ok[CH_AF7] = false;

        for _ in 0..10 {
            let fft = make_fft(1.0, 1.0);
            det.feed(&fft, &epoch_ok, &contact);
        }

        assert!(det.result.is_none());
        assert!(det.reg.count() == 0);
    }

    #[test]
    fn test_stop_completes() {
        let mut det = AlphaTrendDetector::default();
        det.config.settling_duration_s = 0.0;
        det.start();
        det.phase = AlphaTrendPhase::Measuring;

        let epoch_ok = [true; NUM_CH];
        let contact = [ContactQuality::Good; NUM_CH];

        for i in 0..20 {
            let fft = make_fft(1.0 + 0.1 * i as f32, 1.0 + 0.1 * i as f32);
            det.feed(&fft, &epoch_ok, &contact);
        }

        det.stop();
        assert_eq!(det.phase, AlphaTrendPhase::Complete);
        assert!(det.result.is_some());
    }
}
