//! Alpha-band analyzer built on top of the ACT chirplet decomposition.
//!
//! Extracts per-channel alpha power (8–13 Hz) from chirplet results and
//! computes derived metrics: relative alpha, frontal/temporal asymmetry,
//! and an eyes-open/closed state estimate.
//!
//! This module is only compiled when the `act` Cargo feature is enabled.

use crate::act::{ChannelResult, Chirplet};

// ── Band definitions ─────────────────────────────────────────────────────────

/// Standard EEG frequency band edges (Hz).
#[derive(Debug, Clone, Copy)]
pub struct Band {
    pub lo: f32,
    pub hi: f32,
}

/// The five canonical EEG bands.
pub const DELTA: Band = Band { lo: 1.0, hi: 4.0 };
pub const THETA: Band = Band { lo: 4.0, hi: 8.0 };
pub const ALPHA: Band = Band { lo: 8.0, hi: 13.0 };
pub const BETA: Band = Band { lo: 13.0, hi: 30.0 };
pub const GAMMA: Band = Band { lo: 30.0, hi: 44.0 };

impl Band {
    /// Does a chirplet's center frequency fall within this band?
    #[inline]
    pub fn contains(&self, fc: f32) -> bool {
        fc >= self.lo && fc < self.hi
    }
}

// ── Per-channel metrics ──────────────────────────────────────────────────────

/// Alpha-band metrics for a single EEG channel, derived from one ACT window.
#[derive(Debug, Clone, Copy, Default)]
pub struct AlphaMetrics {
    /// Sum of coeff² for chirplets with fc in the alpha band.
    pub alpha_power: f32,
    /// Sum of coeff² for all chirplets (total signal power captured by ACT).
    pub total_power: f32,
    /// alpha_power / total_power (0.0–1.0). `NaN`-safe: 0.0 if total is zero.
    pub relative_alpha: f32,
    /// Individual band powers for context.
    pub delta_power: f32,
    pub theta_power: f32,
    pub beta_power: f32,
    pub gamma_power: f32,
}

impl AlphaMetrics {
    /// Compute metrics from a set of chirplets extracted by ACT.
    pub fn from_chirplets(chirplets: &[Chirplet]) -> Self {
        let mut m = Self::default();
        for c in chirplets {
            let power = c.coeff * c.coeff;
            m.total_power += power;

            if DELTA.contains(c.fc) {
                m.delta_power += power;
            } else if THETA.contains(c.fc) {
                m.theta_power += power;
            } else if ALPHA.contains(c.fc) {
                m.alpha_power += power;
            } else if BETA.contains(c.fc) {
                m.beta_power += power;
            } else if GAMMA.contains(c.fc) {
                m.gamma_power += power;
            }
        }

        m.relative_alpha = if m.total_power > 0.0 {
            m.alpha_power / m.total_power
        } else {
            0.0
        };

        m
    }
}

// ── Multi-channel snapshot ───────────────────────────────────────────────────

/// Number of Muse EEG channels.
pub const NUM_CH: usize = 4;

/// Channel indices for the Muse headset.
pub const CH_TP9: usize = 0;
pub const CH_AF7: usize = 1;
pub const CH_AF8: usize = 2;
pub const CH_TP10: usize = 3;

/// A snapshot of alpha metrics across all 4 Muse channels plus derived indices.
#[derive(Debug, Clone, Copy, Default)]
pub struct AlphaSnapshot {
    /// Per-channel metrics.
    pub channels: [AlphaMetrics; NUM_CH],
    /// Frontal alpha asymmetry: ln(AF8_alpha) − ln(AF7_alpha).
    /// Positive → right-dominant (approach motivation).
    /// Negative → left-dominant (withdrawal motivation).
    pub frontal_asymmetry: f32,
    /// Temporal alpha asymmetry: ln(TP10_alpha) − ln(TP9_alpha).
    pub temporal_asymmetry: f32,
    /// Mean relative alpha across all 4 channels.
    pub mean_relative_alpha: f32,
}

impl AlphaSnapshot {
    /// Build a snapshot from 4-channel ACT results.
    ///
    /// `results` must have exactly `NUM_CH` elements (TP9, AF7, AF8, TP10).
    pub fn from_channel_results(results: &[ChannelResult]) -> Self {
        assert!(results.len() >= NUM_CH);

        let channels: [AlphaMetrics; NUM_CH] = std::array::from_fn(|ch| {
            AlphaMetrics::from_chirplets(&results[ch].chirplets)
        });

        let frontal_asymmetry = log_asymmetry(
            channels[CH_AF8].alpha_power,
            channels[CH_AF7].alpha_power,
        );
        let temporal_asymmetry = log_asymmetry(
            channels[CH_TP10].alpha_power,
            channels[CH_TP9].alpha_power,
        );

        let mean_relative_alpha =
            channels.iter().map(|m| m.relative_alpha).sum::<f32>() / NUM_CH as f32;

        Self {
            channels,
            frontal_asymmetry,
            temporal_asymmetry,
            mean_relative_alpha,
        }
    }
}

/// Compute log asymmetry index: ln(right) − ln(left).
///
/// Returns 0.0 when either side has near-zero power to avoid −∞.
fn log_asymmetry(right: f32, left: f32) -> f32 {
    const FLOOR: f32 = 1e-12;
    if right < FLOOR || left < FLOOR {
        return 0.0;
    }
    right.ln() - left.ln()
}

// ── Eyes-open/closed detector ────────────────────────────────────────────────

/// Simple state machine for detecting eyes-open vs eyes-closed from alpha power.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EyeState {
    Open,
    Closed,
    Unknown,
}

impl Default for EyeState {
    fn default() -> Self {
        Self::Unknown
    }
}

/// Threshold-based eyes-open/closed detector with hysteresis.
///
/// Uses mean relative alpha across channels.  When alpha dominance exceeds
/// `close_threshold` for `hold_count` consecutive windows → Closed.
/// When it drops below `open_threshold` for `hold_count` windows → Open.
#[derive(Debug, Clone)]
pub struct EyeStateDetector {
    /// Relative-alpha threshold above which eyes are considered closed.
    pub close_threshold: f32,
    /// Relative-alpha threshold below which eyes are considered open.
    pub open_threshold: f32,
    /// Number of consecutive windows required to confirm a state change.
    pub hold_count: usize,
    /// Current confirmed state.
    state: EyeState,
    /// Consecutive windows in the candidate state.
    streak: usize,
    /// Candidate state being accumulated.
    candidate: EyeState,
}

impl Default for EyeStateDetector {
    fn default() -> Self {
        Self {
            close_threshold: 0.35,
            open_threshold: 0.20,
            hold_count: 4, // 4 × 250 ms hop = 1 second
            state: EyeState::Unknown,
            streak: 0,
            candidate: EyeState::Unknown,
        }
    }
}

impl EyeStateDetector {
    /// Feed a new alpha snapshot and return the (possibly updated) eye state.
    pub fn update(&mut self, snap: &AlphaSnapshot) -> EyeState {
        let ra = snap.mean_relative_alpha;

        let new_candidate = if ra >= self.close_threshold {
            EyeState::Closed
        } else if ra <= self.open_threshold {
            EyeState::Open
        } else {
            // In the hysteresis gap — keep current candidate
            self.candidate
        };

        if new_candidate == self.candidate {
            self.streak += 1;
        } else {
            self.candidate = new_candidate;
            self.streak = 1;
        }

        if self.streak >= self.hold_count && self.candidate != self.state {
            self.state = self.candidate;
        }

        self.state
    }

    /// Current confirmed state.
    pub fn state(&self) -> EyeState {
        self.state
    }
}

// ── FFT-based band power (classical comparison) ─────────────────────────────

use rustfft::{num_complex::Complex, FftPlanner};

/// Band powers computed via classical FFT + PSD binning.
#[derive(Debug, Clone, Copy, Default)]
pub struct FftBandPower {
    pub delta_power: f32,
    pub theta_power: f32,
    pub alpha_power: f32,
    pub beta_power: f32,
    pub gamma_power: f32,
    pub total_power: f32,
    /// Relative alpha = alpha_power / total_power.
    pub relative_alpha: f32,
}

/// Multi-channel FFT snapshot (matches AlphaSnapshot layout).
#[derive(Debug, Clone, Default)]
pub struct FftSnapshot {
    pub channels: [FftBandPower; NUM_CH],
    pub frontal_asymmetry: f32,
    pub temporal_asymmetry: f32,
    pub mean_relative_alpha: f32,
}

/// Compute FFT-based band powers for a single channel's time-domain signal.
///
/// `signal` is the raw EEG window (same length/fs as used by ACT).
/// Applies a Hann window before the FFT to reduce spectral leakage.
pub fn fft_band_powers(signal: &[f64], fs: f64) -> FftBandPower {
    let n = signal.len();
    if n == 0 {
        return FftBandPower::default();
    }

    // Apply Hann window and convert to Complex<f64>
    let mut buf: Vec<Complex<f64>> = signal
        .iter()
        .enumerate()
        .map(|(i, &x)| {
            let w = 0.5 * (1.0 - (2.0 * std::f64::consts::PI * i as f64 / n as f64).cos());
            Complex::new(x * w, 0.0)
        })
        .collect();

    // Run in-place FFT
    let mut planner = FftPlanner::new();
    let fft = planner.plan_fft_forward(n);
    fft.process(&mut buf);

    // Compute one-sided PSD: |X[k]|² / N, only bins 0..N/2+1
    let n_bins = n / 2 + 1;
    let freq_res = fs / n as f64; // Hz per bin

    let bands = [DELTA, THETA, ALPHA, BETA, GAMMA];
    let mut powers = [0.0_f64; 5];
    let mut total = 0.0_f64;

    for k in 0..n_bins {
        let mag_sq = buf[k].norm_sqr() / (n as f64 * n as f64);
        // Double one-sided bins (except DC and Nyquist)
        let scale = if k == 0 || k == n / 2 { 1.0 } else { 2.0 };
        let psd = mag_sq * scale;

        let freq = k as f64 * freq_res;
        total += psd;

        for (bi, band) in bands.iter().enumerate() {
            if freq >= band.lo as f64 && freq < band.hi as f64 {
                powers[bi] += psd;
                break;
            }
        }
    }

    let rel = if total > 0.0 {
        (powers[2] / total) as f32
    } else {
        0.0
    };

    FftBandPower {
        delta_power: powers[0] as f32,
        theta_power: powers[1] as f32,
        alpha_power: powers[2] as f32,
        beta_power: powers[3] as f32,
        gamma_power: powers[4] as f32,
        total_power: total as f32,
        relative_alpha: rel,
    }
}

impl FftSnapshot {
    /// Compute FFT band powers for all 4 channels from raw EEG windows.
    ///
    /// Each element of `windows` is one channel's time-domain signal of length `n`.
    pub fn from_windows(windows: &[&[f64]], fs: f64) -> Self {
        assert!(windows.len() >= NUM_CH);

        let channels: [FftBandPower; NUM_CH] =
            std::array::from_fn(|ch| fft_band_powers(windows[ch], fs));

        let frontal_asymmetry = log_asymmetry(
            channels[CH_AF8].alpha_power,
            channels[CH_AF7].alpha_power,
        );
        let temporal_asymmetry = log_asymmetry(
            channels[CH_TP10].alpha_power,
            channels[CH_TP9].alpha_power,
        );
        let mean_relative_alpha =
            channels.iter().map(|m| m.relative_alpha).sum::<f32>() / NUM_CH as f32;

        Self {
            channels,
            frontal_asymmetry,
            temporal_asymmetry,
            mean_relative_alpha,
        }
    }
}

// ── Band power time-series history ────────────────────────────────────────────

/// Number of standard EEG bands (δ, θ, α, β, γ).
pub const NUM_BANDS: usize = 5;

/// Band labels for display.
pub const BAND_LABELS: [&str; NUM_BANDS] = ["δ", "θ", "α", "β", "γ"];

/// A single time-point of relative band powers (each 0.0–1.0).
#[derive(Debug, Clone, Copy, Default)]
pub struct BandFrame {
    /// Relative power per band: [delta, theta, alpha, beta, gamma].
    pub bands: [f32; NUM_BANDS],
}

impl BandFrame {
    /// Build from `AlphaMetrics` (ACT-derived).
    pub fn from_alpha_metrics(m: &AlphaMetrics) -> Self {
        let t = m.total_power.max(1e-12);
        Self {
            bands: [
                m.delta_power / t,
                m.theta_power / t,
                m.alpha_power / t,
                m.beta_power / t,
                m.gamma_power / t,
            ],
        }
    }

    /// Build from `FftBandPower`.
    pub fn from_fft(f: &FftBandPower) -> Self {
        let t = f.total_power.max(1e-12);
        Self {
            bands: [
                f.delta_power / t,
                f.theta_power / t,
                f.alpha_power / t,
                f.beta_power / t,
                f.gamma_power / t,
            ],
        }
    }
}

/// Rolling time-series of band powers for one channel, both ACT and FFT.
#[derive(Debug, Clone)]
pub struct ChannelBandHistory {
    pub act: std::collections::VecDeque<BandFrame>,
    pub fft: std::collections::VecDeque<BandFrame>,
}

impl ChannelBandHistory {
    pub fn new(capacity: usize) -> Self {
        Self {
            act: std::collections::VecDeque::with_capacity(capacity),
            fft: std::collections::VecDeque::with_capacity(capacity),
        }
    }

    pub fn push(&mut self, act_frame: BandFrame, fft_frame: BandFrame, max_len: usize) {
        self.act.push_back(act_frame);
        self.fft.push_back(fft_frame);
        while self.act.len() > max_len {
            self.act.pop_front();
        }
        while self.fft.len() > max_len {
            self.fft.pop_front();
        }
    }

    pub fn clear(&mut self) {
        self.act.clear();
        self.fft.clear();
    }

    pub fn len(&self) -> usize {
        self.act.len()
    }
}

/// All 4 channels of band history.
#[derive(Debug, Clone)]
pub struct BandHistory {
    pub channels: [ChannelBandHistory; NUM_CH],
}

/// Maximum number of time-points to store (~4 minutes at 250ms hop).
pub const BAND_HISTORY_MAX: usize = 960;

impl BandHistory {
    pub fn new() -> Self {
        Self {
            channels: std::array::from_fn(|_| ChannelBandHistory::new(BAND_HISTORY_MAX)),
        }
    }

    /// Push one snapshot of ACT + FFT results into the history.
    pub fn push(&mut self, act_snap: &AlphaSnapshot, fft_snap: &FftSnapshot) {
        for ch in 0..NUM_CH {
            let act_frame = BandFrame::from_alpha_metrics(&act_snap.channels[ch]);
            let fft_frame = BandFrame::from_fft(&fft_snap.channels[ch]);
            self.channels[ch].push(act_frame, fft_frame, BAND_HISTORY_MAX);
        }
    }

    pub fn clear(&mut self) {
        for ch in &mut self.channels {
            ch.clear();
        }
    }
}

impl Default for BandHistory {
    fn default() -> Self {
        Self::new()
    }
}

// ── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::act::Chirplet;

    fn chirplet(fc: f32, coeff: f32) -> Chirplet {
        Chirplet {
            tc: 0.0,
            fc,
            log_dt: -1.0,
            chirp_rate: 0.0,
            coeff,
        }
    }

    #[test]
    fn alpha_metrics_pure_alpha() {
        let chirplets = vec![
            chirplet(10.0, 5.0), // alpha
            chirplet(11.0, 3.0), // alpha
        ];
        let m = AlphaMetrics::from_chirplets(&chirplets);
        assert!((m.alpha_power - (25.0 + 9.0)).abs() < 1e-6);
        assert!((m.total_power - 34.0).abs() < 1e-6);
        assert!((m.relative_alpha - 1.0).abs() < 1e-6);
    }

    #[test]
    fn alpha_metrics_mixed_bands() {
        let chirplets = vec![
            chirplet(10.0, 4.0), // alpha: power=16
            chirplet(20.0, 3.0), // beta:  power=9
            chirplet(2.0, 2.0),  // delta: power=4
        ];
        let m = AlphaMetrics::from_chirplets(&chirplets);
        assert!((m.alpha_power - 16.0).abs() < 1e-6);
        assert!((m.beta_power - 9.0).abs() < 1e-6);
        assert!((m.delta_power - 4.0).abs() < 1e-6);
        assert!((m.total_power - 29.0).abs() < 1e-6);
        assert!((m.relative_alpha - 16.0 / 29.0).abs() < 1e-4);
    }

    #[test]
    fn alpha_metrics_empty() {
        let m = AlphaMetrics::from_chirplets(&[]);
        assert_eq!(m.alpha_power, 0.0);
        assert_eq!(m.total_power, 0.0);
        assert_eq!(m.relative_alpha, 0.0);
    }

    #[test]
    fn asymmetry_balanced() {
        assert!((log_asymmetry(10.0, 10.0)).abs() < 1e-6);
    }

    #[test]
    fn asymmetry_right_dominant() {
        let a = log_asymmetry(20.0, 10.0);
        assert!(a > 0.0, "expected positive asymmetry, got {a}");
    }

    #[test]
    fn asymmetry_zero_floor() {
        assert_eq!(log_asymmetry(0.0, 10.0), 0.0);
        assert_eq!(log_asymmetry(10.0, 0.0), 0.0);
    }

    #[test]
    fn snapshot_from_results() {
        use crate::act::ChannelResult;

        // All fc values inside alpha band [8, 13)
        let fcs = [9.0_f32, 10.0, 11.0, 12.0];
        let results: Vec<ChannelResult> = fcs
            .iter()
            .map(|&fc| ChannelResult {
                chirplets: vec![chirplet(fc, 5.0)],
                error: 0.01,
            })
            .collect();

        let snap = AlphaSnapshot::from_channel_results(&results);
        assert!((snap.mean_relative_alpha - 1.0).abs() < 1e-6);
        // All channels have identical alpha power → asymmetry ≈ 0
        assert!(snap.frontal_asymmetry.abs() < 0.1);
        assert!(snap.temporal_asymmetry.abs() < 0.1);
    }

    #[test]
    fn eye_state_detector_transitions() {
        let mut det = EyeStateDetector {
            close_threshold: 0.35,
            open_threshold: 0.20,
            hold_count: 3,
            ..Default::default()
        };

        // Start unknown
        assert_eq!(det.state(), EyeState::Unknown);

        // Feed high alpha (eyes closed) for hold_count windows
        let closed_snap = AlphaSnapshot {
            mean_relative_alpha: 0.50,
            ..Default::default()
        };
        for _ in 0..2 {
            det.update(&closed_snap);
            assert_eq!(det.state(), EyeState::Unknown); // not yet confirmed
        }
        det.update(&closed_snap);
        assert_eq!(det.state(), EyeState::Closed); // confirmed after 3

        // Feed low alpha (eyes open)
        let open_snap = AlphaSnapshot {
            mean_relative_alpha: 0.10,
            ..Default::default()
        };
        for _ in 0..2 {
            det.update(&open_snap);
            assert_eq!(det.state(), EyeState::Closed); // not yet flipped
        }
        det.update(&open_snap);
        assert_eq!(det.state(), EyeState::Open); // confirmed after 3
    }

    #[test]
    fn eye_state_hysteresis_gap() {
        let mut det = EyeStateDetector {
            close_threshold: 0.35,
            open_threshold: 0.20,
            hold_count: 2,
            ..Default::default()
        };

        // Confirm closed
        let closed = AlphaSnapshot {
            mean_relative_alpha: 0.50,
            ..Default::default()
        };
        det.update(&closed);
        det.update(&closed);
        assert_eq!(det.state(), EyeState::Closed);

        // Feed value in hysteresis gap — state should NOT change
        let gap = AlphaSnapshot {
            mean_relative_alpha: 0.28,
            ..Default::default()
        };
        for _ in 0..10 {
            det.update(&gap);
        }
        assert_eq!(det.state(), EyeState::Closed);
    }

    // ── FFT tests ────────────────────────────────────────────────────────

    /// Generate a pure sine at `freq_hz` sampled at `fs` for `n` samples.
    fn sine(freq_hz: f64, fs: f64, n: usize) -> Vec<f64> {
        (0..n)
            .map(|i| (2.0 * std::f64::consts::PI * freq_hz * i as f64 / fs).sin())
            .collect()
    }

    #[test]
    fn fft_pure_alpha_tone() {
        let sig = sine(10.0, 256.0, 512);
        let bp = fft_band_powers(&sig, 256.0);

        assert!(bp.alpha_power > 0.0, "alpha power should be non-zero");
        assert!(
            bp.relative_alpha > 0.8,
            "10 Hz tone should be >80% alpha, got {:.1}%",
            bp.relative_alpha * 100.0
        );
    }

    #[test]
    fn fft_mixed_bands() {
        // 3 Hz (delta) + 10 Hz (alpha) + 20 Hz (beta)
        let n = 512;
        let fs = 256.0;
        let sig: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                4.0 * (2.0 * std::f64::consts::PI * 3.0 * t).sin()  // delta
              + 3.0 * (2.0 * std::f64::consts::PI * 10.0 * t).sin() // alpha
              + 2.0 * (2.0 * std::f64::consts::PI * 20.0 * t).sin() // beta
            })
            .collect();
        let bp = fft_band_powers(&sig, fs);

        // All three bands should have power
        assert!(bp.delta_power > 0.0);
        assert!(bp.alpha_power > 0.0);
        assert!(bp.beta_power > 0.0);
        // Delta has highest amplitude (4.0) → most power
        assert!(bp.delta_power > bp.alpha_power);
        assert!(bp.alpha_power > bp.beta_power);
    }

    #[test]
    fn fft_empty_signal() {
        let bp = fft_band_powers(&[], 256.0);
        assert_eq!(bp.total_power, 0.0);
        assert_eq!(bp.relative_alpha, 0.0);
    }
}
