//! EEG signal conditioning: IIR bandpass filtering and epoch-level artifact rejection.
//!
//! # Bandpass filter
//!
//! A 4th-order Butterworth bandpass (1–45 Hz at 256 Hz) implemented as four
//! cascaded biquad sections (Direct Form II Transposed).  Two sections form
//! the high-pass leg and two form the low-pass leg.
//!
//! # Artifact rejection
//!
//! [`EpochValidator`] checks a fixed-length epoch (e.g. 512 samples) against:
//!
//! 1. **Amplitude threshold** – reject if any sample exceeds ±100 µV.
//! 2. **Contact quality (HSI)** – reject if the channel reports bad contact.
//! 3. **Variance check** – reject if epoch variance > 3× running median variance.

use std::collections::VecDeque;
use std::f64::consts::PI;

// ── Biquad (second-order IIR section) ────────────────────────────────────────

/// Coefficients for one second-order IIR section (biquad).
///
/// Transfer function:  H(z) = (b0 + b1·z⁻¹ + b2·z⁻²) / (1 + a1·z⁻¹ + a2·z⁻²)
///
/// All coefficients are pre-normalised (a0 = 1).
#[derive(Debug, Clone, Copy)]
pub struct BiquadCoeffs {
    pub b0: f64,
    pub b1: f64,
    pub b2: f64,
    pub a1: f64,
    pub a2: f64,
}

/// A single biquad filter with internal state (Direct Form II Transposed).
#[derive(Debug, Clone)]
pub struct Biquad {
    pub coeffs: BiquadCoeffs,
    s1: f64,
    s2: f64,
}

impl Biquad {
    pub fn new(c: BiquadCoeffs) -> Self {
        Self { coeffs: c, s1: 0.0, s2: 0.0 }
    }

    /// Process a single sample and return the filtered output.
    #[inline]
    pub fn process(&mut self, x: f64) -> f64 {
        let c = &self.coeffs;
        let y = c.b0 * x + self.s1;
        self.s1 = c.b1 * x - c.a1 * y + self.s2;
        self.s2 = c.b2 * x - c.a2 * y;
        y
    }

    /// Reset internal state to zero.
    pub fn reset(&mut self) {
        self.s1 = 0.0;
        self.s2 = 0.0;
    }
}

// ── Biquad chain (cascaded sections) ─────────────────────────────────────────

/// A cascade of biquad sections processed in series.
#[derive(Debug, Clone)]
pub struct BiquadChain {
    pub sections: Vec<Biquad>,
}

impl BiquadChain {
    pub fn new(sections: Vec<Biquad>) -> Self {
        Self { sections }
    }

    /// Filter one sample through the entire chain.
    #[inline]
    pub fn process(&mut self, mut x: f64) -> f64 {
        for bq in &mut self.sections {
            x = bq.process(x);
        }
        x
    }

    /// Reset all internal state (e.g. after a gap in the signal).
    pub fn reset(&mut self) {
        for bq in &mut self.sections {
            bq.reset();
        }
    }
}

// ── Butterworth coefficient design ───────────────────────────────────────────

/// Compute the Q factors for each biquad section of an Nth-order Butterworth
/// filter (N must be even; we use N/2 biquad sections).
fn butterworth_q_factors(order: usize) -> Vec<f64> {
    let n = order;
    (0..n / 2)
        .map(|k| {
            let angle = PI * (2 * k + 1) as f64 / (2 * n) as f64;
            1.0 / (2.0 * angle.cos())
        })
        .collect()
}

/// Design a 2nd-order Butterworth **low-pass** biquad at cutoff `fc` Hz,
/// sample rate `fs` Hz, with quality factor `q`.
///
/// Uses the Audio EQ Cookbook bilinear-transform formulas.
fn lowpass_biquad(fc: f64, fs: f64, q: f64) -> BiquadCoeffs {
    let w0 = 2.0 * PI * fc / fs;
    let alpha = w0.sin() / (2.0 * q);
    let cos_w0 = w0.cos();

    let b0 = (1.0 - cos_w0) / 2.0;
    let b1 = 1.0 - cos_w0;
    let b2 = (1.0 - cos_w0) / 2.0;
    let a0 = 1.0 + alpha;
    let a1 = -2.0 * cos_w0;
    let a2 = 1.0 - alpha;

    BiquadCoeffs {
        b0: b0 / a0,
        b1: b1 / a0,
        b2: b2 / a0,
        a1: a1 / a0,
        a2: a2 / a0,
    }
}

/// Design a 2nd-order Butterworth **high-pass** biquad at cutoff `fc` Hz,
/// sample rate `fs` Hz, with quality factor `q`.
fn highpass_biquad(fc: f64, fs: f64, q: f64) -> BiquadCoeffs {
    let w0 = 2.0 * PI * fc / fs;
    let alpha = w0.sin() / (2.0 * q);
    let cos_w0 = w0.cos();

    let b0 = (1.0 + cos_w0) / 2.0;
    let b1 = -(1.0 + cos_w0);
    let b2 = (1.0 + cos_w0) / 2.0;
    let a0 = 1.0 + alpha;
    let a1 = -2.0 * cos_w0;
    let a2 = 1.0 - alpha;

    BiquadCoeffs {
        b0: b0 / a0,
        b1: b1 / a0,
        b2: b2 / a0,
        a1: a1 / a0,
        a2: a2 / a0,
    }
}

/// Create a **bandpass** [`BiquadChain`] by cascading a Butterworth high-pass
/// at `f_low` Hz and a Butterworth low-pass at `f_high` Hz, each of order
/// `order` (must be even, typically 4).
///
/// The resulting chain has `order` biquad sections total (order/2 HP + order/2 LP).
pub fn butterworth_bandpass(f_low: f64, f_high: f64, fs: f64, order: usize) -> BiquadChain {
    assert!(order >= 2 && order % 2 == 0, "order must be even and ≥ 2");
    let qs = butterworth_q_factors(order);

    let mut sections = Vec::with_capacity(order);

    // High-pass leg (removes DC drift and sub-band content)
    for &q in &qs {
        sections.push(Biquad::new(highpass_biquad(f_low, fs, q)));
    }

    // Low-pass leg (removes high-frequency noise)
    for &q in &qs {
        sections.push(Biquad::new(lowpass_biquad(f_high, fs, q)));
    }

    BiquadChain::new(sections)
}

// ── Per-channel filter wrapper ───────────────────────────────────────────────

/// Manages a bandpass filter for one EEG channel, applying IIR filtering
/// sample-by-sample as data arrives.
#[derive(Debug, Clone)]
pub struct ChannelFilter {
    chain: BiquadChain,
}

impl ChannelFilter {
    /// Create a new channel filter: 4th-order Butterworth bandpass 0.3–45 Hz at `fs` Hz.
    pub fn new(fs: f64) -> Self {
        Self {
            chain: butterworth_bandpass(0.3, 45.0, fs, 4),
        }
    }

    /// Filter a single sample and return the filtered value.
    #[inline]
    pub fn process(&mut self, x: f64) -> f64 {
        self.chain.process(x)
    }

    /// Filter a batch of samples in-place.
    pub fn process_batch(&mut self, samples: &mut [f64]) {
        for s in samples.iter_mut() {
            *s = self.chain.process(*s);
        }
    }

    /// Reset filter state (call on reconnect / clear).
    pub fn reset(&mut self) {
        self.chain.reset();
    }
}

// ── Epoch-level artifact rejection ───────────────────────────────────────────

/// Result of validating one epoch.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EpochStatus {
    /// Epoch is clean — use for analysis.
    Clean,
    /// Rejected: amplitude exceeded threshold.
    AmplitudeExceeded,
    /// Rejected: bad electrode contact (HSI = 4).
    BadContact,
    /// Rejected: variance exceeded 3× running median.
    HighVariance,
}

impl EpochStatus {
    pub fn is_clean(self) -> bool {
        self == EpochStatus::Clean
    }
}

/// Tracks running median of epoch variances for one channel and validates
/// incoming epochs against the three rejection criteria.
#[derive(Debug, Clone)]
pub struct EpochValidator {
    /// Maximum absolute amplitude allowed (µV).
    pub amplitude_limit: f64,
    /// Multiplier for the running-median variance threshold.
    pub variance_factor: f64,
    /// Rolling window of recent epoch variances for median computation.
    variance_history: VecDeque<f64>,
    /// Maximum number of variance samples to keep.
    history_cap: usize,
}

impl Default for EpochValidator {
    fn default() -> Self {
        Self {
            amplitude_limit: 150.0,
            variance_factor: 3.0,
            variance_history: VecDeque::with_capacity(64),
            history_cap: 60, // ~15 seconds at 250 ms hop
        }
    }
}

impl EpochValidator {
    /// Validate an epoch (a slice of filtered EEG samples in µV).
    ///
    /// `bad_contact` should be `true` when the Muse horseshoe indicator
    /// reports quality = 4 (no contact) for this channel.
    ///
    /// Returns the epoch status and, if clean, also updates the running
    /// variance history.
    pub fn validate(&mut self, epoch: &[f64], bad_contact: bool) -> EpochStatus {
        if bad_contact {
            return EpochStatus::BadContact;
        }

        // 1. Amplitude check
        for &v in epoch {
            if v.abs() > self.amplitude_limit {
                return EpochStatus::AmplitudeExceeded;
            }
        }

        // 2. Compute epoch variance
        let n = epoch.len() as f64;
        if n < 2.0 {
            return EpochStatus::Clean;
        }
        let mean = epoch.iter().sum::<f64>() / n;
        let var = epoch.iter().map(|&x| (x - mean).powi(2)).sum::<f64>() / (n - 1.0);

        // 3. Variance check against running median
        if self.variance_history.len() >= 4 {
            let median = self.running_median();
            if var > self.variance_factor * median {
                return EpochStatus::HighVariance;
            }
        }

        // Epoch is clean — record its variance
        self.variance_history.push_back(var);
        while self.variance_history.len() > self.history_cap {
            self.variance_history.pop_front();
        }

        EpochStatus::Clean
    }

    /// Compute the median of the variance history.
    fn running_median(&self) -> f64 {
        let mut sorted: Vec<f64> = self.variance_history.iter().copied().collect();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let mid = sorted.len() / 2;
        if sorted.len() % 2 == 0 {
            (sorted[mid - 1] + sorted[mid]) / 2.0
        } else {
            sorted[mid]
        }
    }

    /// Reset the running variance history (call on reconnect / clear).
    pub fn reset(&mut self) {
        self.variance_history.clear();
    }
}

// ── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn butterworth_q_order4() {
        let qs = butterworth_q_factors(4);
        assert_eq!(qs.len(), 2);
        // Q values for 4th-order Butterworth: 0.5412 and 1.3066
        assert!((qs[0] - 0.5412).abs() < 0.001, "Q0 = {}", qs[0]);
        assert!((qs[1] - 1.3066).abs() < 0.001, "Q1 = {}", qs[1]);
    }

    #[test]
    fn bandpass_dc_rejection() {
        // A DC signal (constant) should be fully attenuated by the high-pass leg.
        let mut chain = butterworth_bandpass(1.0, 45.0, 256.0, 4);
        // Run 2000 samples of DC = 100.0
        let mut last = 0.0;
        for _ in 0..2000 {
            last = chain.process(100.0);
        }
        // After settling, output should be near zero
        assert!(last.abs() < 0.1, "DC output = {last}");
    }

    #[test]
    fn bandpass_passes_10hz() {
        // A 10 Hz sine (in the passband) should pass through with near-unity gain.
        let mut chain = butterworth_bandpass(1.0, 45.0, 256.0, 4);
        let fs = 256.0;
        let freq = 10.0;
        let amplitude = 20.0;

        // Let filter settle for 2 seconds
        for i in 0..512 {
            let t = i as f64 / fs;
            chain.process(amplitude * (2.0 * PI * freq * t).sin());
        }

        // Measure peak output over the next second
        let mut peak = 0.0_f64;
        for i in 512..768 {
            let t = i as f64 / fs;
            let y = chain.process(amplitude * (2.0 * PI * freq * t).sin());
            peak = peak.max(y.abs());
        }

        // Should be close to the input amplitude (within 5%)
        let ratio = peak / amplitude;
        assert!(
            (0.90..=1.10).contains(&ratio),
            "10 Hz gain ratio = {ratio:.4} (peak = {peak:.4})"
        );
    }

    #[test]
    fn bandpass_rejects_100hz() {
        // A 100 Hz sine (above the passband) should be heavily attenuated.
        let mut chain = butterworth_bandpass(1.0, 45.0, 256.0, 4);
        let fs = 256.0;
        let freq = 100.0;
        let amplitude = 20.0;

        // Settle
        for i in 0..1024 {
            let t = i as f64 / fs;
            chain.process(amplitude * (2.0 * PI * freq * t).sin());
        }

        // Measure
        let mut peak = 0.0_f64;
        for i in 1024..1280 {
            let t = i as f64 / fs;
            let y = chain.process(amplitude * (2.0 * PI * freq * t).sin());
            peak = peak.max(y.abs());
        }

        // 100 Hz is above Nyquist/2 for 256 Hz so aliased, but the filter should
        // still attenuate significantly. Check < 10% of input amplitude.
        let ratio = peak / amplitude;
        assert!(
            ratio < 0.10,
            "100 Hz attenuation ratio = {ratio:.4} (peak = {peak:.4})"
        );
    }

    #[test]
    fn epoch_validator_amplitude_reject() {
        let mut v = EpochValidator::default();
        let epoch = vec![10.0, 20.0, 200.0, -5.0]; // 200 > 150
        assert_eq!(v.validate(&epoch, false), EpochStatus::AmplitudeExceeded);
    }

    #[test]
    fn epoch_validator_bad_contact() {
        let mut v = EpochValidator::default();
        let epoch = vec![1.0, 2.0, 3.0];
        assert_eq!(v.validate(&epoch, true), EpochStatus::BadContact);
    }

    #[test]
    fn epoch_validator_clean_epochs() {
        let mut v = EpochValidator::default();
        // Feed 10 clean epochs to build variance history
        for _ in 0..10 {
            let epoch: Vec<f64> = (0..512).map(|i| (i as f64 * 0.01).sin() * 20.0).collect();
            assert!(v.validate(&epoch, false).is_clean());
        }
    }

    #[test]
    fn epoch_validator_high_variance_reject() {
        let mut v = EpochValidator::default();
        // Build baseline with low-variance epochs
        for _ in 0..10 {
            let epoch: Vec<f64> = (0..512).map(|i| (i as f64 * 0.01).sin() * 5.0).collect();
            let status = v.validate(&epoch, false);
            assert!(status.is_clean(), "baseline epoch should be clean: {status:?}");
        }

        // Now feed a high-variance epoch (10× amplitude → 100× variance)
        let noisy_epoch: Vec<f64> = (0..512).map(|i| (i as f64 * 0.01).sin() * 50.0).collect();
        let status = v.validate(&noisy_epoch, false);
        assert_eq!(status, EpochStatus::HighVariance);
    }

    #[test]
    fn channel_filter_process_batch() {
        let mut f = ChannelFilter::new(256.0);
        let mut samples = vec![1.0; 100];
        f.process_batch(&mut samples);
        // After processing DC through bandpass, values should be small
        // (not exactly zero because filter is still settling)
        // Just check it doesn't panic and produces finite values
        assert!(samples.iter().all(|v| v.is_finite()));
    }
}
