//! Client-side signal quality computation.
//!
//! The Muse headset does **not** transmit contact quality or artifact
//! indicators over BLE.  The "horseshoe" display, `touching_forehead`,
//! blink, and jaw-clench signals historically provided by the Muse SDK
//! and apps like Mind Monitor are all derived from the raw EEG data on
//! the receiving side.
//!
//! This module provides pure, allocation-light functions for computing
//! electrode contact quality from a sliding window of EEG samples.
//! No I/O or async code — safe to call from any context.

/// Per-electrode contact quality tier.
///
/// Mirrors the Muse SDK horseshoe values:
/// - `Good`  (SDK value 1) — clean EEG, low noise
/// - `Ok`    (SDK value 2) — marginal; signal is usable but noisy
/// - `Bad`   (SDK value 4) — heavy noise, likely poor skin contact
/// - `NoContact`           — signal is railing or absent
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ContactQuality {
    Good,
    Ok,
    Bad,
    NoContact,
}

impl ContactQuality {
    /// Numeric value matching the original Muse SDK horseshoe convention.
    pub fn sdk_value(self) -> u8 {
        match self {
            Self::Good => 1,
            Self::Ok => 2,
            Self::Bad => 4,
            Self::NoContact => 4,
        }
    }
}

impl std::fmt::Display for ContactQuality {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Good => write!(f, "Good"),
            Self::Ok => write!(f, "OK"),
            Self::Bad => write!(f, "Bad"),
            Self::NoContact => write!(f, "No Contact"),
        }
    }
}

// ── Thresholds ──────────────────────────────────────────────────────────────

/// RMS below this (µV) with very low variance likely means no signal at all.
const RMS_FLOOR: f64 = 0.5;

/// RMS below this (µV) indicates marginal but usable contact (all electrodes).
const RMS_OK: f64 = 100.0;

/// Per-electrode "Good" RMS threshold (µV).  Frontal electrodes (AF7, AF8)
/// sit on the forehead with less muscle interference, so they can use a
/// tighter threshold.  Temporal electrodes (TP9, TP10) are behind the ears
/// where EMG raises baseline RMS even with perfect skin contact.
///
/// Electrode order: TP9=0, AF7=1, AF8=2, TP10=3.
pub const RMS_GOOD_PER_ELECTRODE: [f64; 4] = [
    85.0,  // TP9  — temporal (dry-contact RMS typically 65–85 µV)
    45.0,  // AF7  — frontal
    45.0,  // AF8  — frontal
    85.0,  // TP10 — temporal (dry-contact RMS typically 65–85 µV)
];

/// Fraction of samples at the ADC rail that triggers `NoContact`.
///
/// Classic firmware uses a 12-bit ADC centred at 2048 with scale 0.48828125;
/// the rail values in µV are approximately ±1000 µV.  Athena uses a 14-bit
/// ADC with scale 0.0885; rail values are approximately ±725 µV.
///
/// We use a generous rail threshold of ±900 µV (works for both firmwares)
/// and flag as railing when more than 20 % of the window hits it.
const RAIL_UV_THRESHOLD: f64 = 900.0;
const RAIL_FRACTION_THRESHOLD: f64 = 0.20;

/// Number of samples to use for contact quality estimation (~1 second at 256 Hz).
pub const CONTACT_QUALITY_WINDOW_SAMPLES: usize = 256;

/// Number of consecutive identical readings required before the displayed
/// contact quality transitions to a new state (hysteresis).
const HYSTERESIS_COUNT: usize = 3;

// ── Public API ──────────────────────────────────────────────────────────────

/// Compute the contact quality for a single electrode from a window of
/// EEG samples (in µV).
///
/// The input should be roughly 1 second of data (256 samples at 256 Hz).
/// Shorter windows still work but produce noisier estimates.
///
/// `rms_good` is the per-electrode threshold separating `Good` from `Ok`.
/// Use [`RMS_GOOD_PER_ELECTRODE`] to look up the appropriate value.
///
/// # Algorithm
///
/// 1. **Railing check** — if >20 % of samples exceed ±900 µV the electrode
///    is considered to have no contact.
/// 2. **RMS amplitude** — standard deviation of the window.
///    - RMS < 0.5 µV      → `NoContact` (flat line, likely disconnected)
///    - RMS < `rms_good`   → `Good`
///    - RMS < 100 µV       → `Ok`
///    - RMS ≥ 100 µV       → `Bad`
pub fn compute_contact_quality(samples: &[f64], rms_good: f64) -> ContactQuality {
    if samples.is_empty() {
        return ContactQuality::NoContact;
    }

    // ── Railing check ────────────────────────────────────────────────────
    let rail_count = samples
        .iter()
        .filter(|&&v| v.abs() >= RAIL_UV_THRESHOLD)
        .count();
    if rail_count as f64 / samples.len() as f64 > RAIL_FRACTION_THRESHOLD {
        return ContactQuality::NoContact;
    }

    // ── RMS (standard deviation) ─────────────────────────────────────────
    let mean = samples.iter().sum::<f64>() / samples.len() as f64;
    let variance = samples.iter().map(|&v| (v - mean).powi(2)).sum::<f64>() / samples.len() as f64;
    let rms = variance.sqrt();

    if rms < RMS_FLOOR {
        ContactQuality::NoContact
    } else if rms < rms_good {
        ContactQuality::Good
    } else if rms < RMS_OK {
        ContactQuality::Ok
    } else {
        ContactQuality::Bad
    }
}

// ── Hysteresis tracker ──────────────────────────────────────────────────────

/// Tracks contact quality for one electrode with hysteresis to avoid
/// rapid toggling between states near threshold boundaries.
///
/// The displayed quality only changes after [`HYSTERESIS_COUNT`] consecutive
/// raw readings agree on the new state.
#[derive(Debug, Clone)]
pub struct ContactQualityTracker {
    /// The electrode index (0=TP9, 1=AF7, 2=AF8, 3=TP10).
    electrode: usize,
    /// Current stable (displayed) quality.
    current: ContactQuality,
    /// Candidate quality that consecutive readings are converging toward.
    candidate: ContactQuality,
    /// How many consecutive readings have matched `candidate`.
    streak: usize,
    /// Most recent raw (pre-hysteresis) quality.
    last_raw: ContactQuality,
    /// Most recent RMS value (µV).
    last_rms: f64,
}

impl ContactQualityTracker {
    /// Create a new tracker for the given electrode index.
    pub fn new(electrode: usize) -> Self {
        Self {
            electrode,
            current: ContactQuality::NoContact,
            candidate: ContactQuality::NoContact,
            streak: 0,
            last_raw: ContactQuality::NoContact,
            last_rms: 0.0,
        }
    }

    /// Feed a new 1-second window of raw EEG samples.  Returns the stable
    /// (hysteresis-filtered) contact quality.
    pub fn update(&mut self, samples: &[f64]) -> ContactQuality {
        let rms_good = RMS_GOOD_PER_ELECTRODE
            .get(self.electrode)
            .copied()
            .unwrap_or(45.0);
        let raw = compute_contact_quality(samples, rms_good);

        // Compute and store RMS for diagnostics
        if !samples.is_empty() {
            let mean = samples.iter().sum::<f64>() / samples.len() as f64;
            let var = samples.iter().map(|&v| (v - mean).powi(2)).sum::<f64>() / samples.len() as f64;
            self.last_rms = var.sqrt();
        }
        self.last_raw = raw;

        if raw == self.current {
            // Readings agree with current — reset candidate.
            self.candidate = self.current;
            self.streak = 0;
        } else if raw == self.candidate {
            self.streak += 1;
            if self.streak >= HYSTERESIS_COUNT {
                self.current = self.candidate;
                self.streak = 0;
            }
        } else {
            // New candidate — start counting.
            self.candidate = raw;
            self.streak = 1;
        }

        self.current
    }

    /// Current stable quality (read-only).
    pub fn quality(&self) -> ContactQuality {
        self.current
    }

    /// Most recent raw (pre-hysteresis) quality reading.
    pub fn last_raw(&self) -> ContactQuality {
        self.last_raw
    }

    /// Most recent RMS value in µV.
    pub fn last_rms(&self) -> f64 {
        self.last_rms
    }

    /// Per-electrode "Good" threshold for this electrode.
    pub fn rms_good_threshold(&self) -> f64 {
        RMS_GOOD_PER_ELECTRODE
            .get(self.electrode)
            .copied()
            .unwrap_or(45.0)
    }

    /// Current hysteresis streak count.
    pub fn streak(&self) -> usize {
        self.streak
    }

    /// Reset to `NoContact`.
    pub fn reset(&mut self) {
        self.current = ContactQuality::NoContact;
        self.candidate = ContactQuality::NoContact;
        self.streak = 0;
        self.last_raw = ContactQuality::NoContact;
        self.last_rms = 0.0;
    }
}

/// Returns `true` if at least one of the frontal electrodes (AF7 or AF8)
/// has `Good` or `Ok` contact, indicating the headband is on the forehead.
///
/// `qualities` is indexed by electrode: 0=TP9, 1=AF7, 2=AF8, 3=TP10.
pub fn touching_forehead(qualities: &[ContactQuality]) -> bool {
    let af7 = qualities.get(1).copied().unwrap_or(ContactQuality::NoContact);
    let af8 = qualities.get(2).copied().unwrap_or(ContactQuality::NoContact);
    matches!(af7, ContactQuality::Good | ContactQuality::Ok)
        || matches!(af8, ContactQuality::Good | ContactQuality::Ok)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_is_no_contact() {
        assert_eq!(compute_contact_quality(&[], 45.0), ContactQuality::NoContact);
    }

    #[test]
    fn flat_line_is_no_contact() {
        let samples = vec![0.0; 256];
        assert_eq!(compute_contact_quality(&samples, 45.0), ContactQuality::NoContact);
    }

    #[test]
    fn clean_signal_is_good_frontal() {
        // 10 Hz sine at 15 µV peak → RMS ≈ 10.6 µV (well below 45 µV frontal threshold)
        let samples: Vec<f64> = (0..256)
            .map(|i| 15.0 * (2.0 * std::f64::consts::PI * 10.0 * i as f64 / 256.0).sin())
            .collect();
        assert_eq!(compute_contact_quality(&samples, 45.0), ContactQuality::Good);
    }

    #[test]
    fn moderate_signal_good_for_temporal_ok_for_frontal() {
        // 10 Hz sine at 70 µV peak → RMS ≈ 49.5 µV
        // Temporal threshold 55 µV → Good;  Frontal threshold 45 µV → Ok
        let samples: Vec<f64> = (0..256)
            .map(|i| 70.0 * (2.0 * std::f64::consts::PI * 10.0 * i as f64 / 256.0).sin())
            .collect();
        assert_eq!(compute_contact_quality(&samples, 55.0), ContactQuality::Good);
        assert_eq!(compute_contact_quality(&samples, 45.0), ContactQuality::Ok);
    }

    #[test]
    fn noisy_signal_is_ok() {
        // 10 Hz sine at 120 µV peak → RMS ≈ 84.9 µV
        let samples: Vec<f64> = (0..256)
            .map(|i| 120.0 * (2.0 * std::f64::consts::PI * 10.0 * i as f64 / 256.0).sin())
            .collect();
        assert_eq!(compute_contact_quality(&samples, 55.0), ContactQuality::Ok);
    }

    #[test]
    fn railing_is_no_contact() {
        let mut samples = vec![0.0; 256];
        // 30 % railing — exceeds 20 % threshold
        for s in samples.iter_mut().take(80) {
            *s = 950.0;
        }
        assert_eq!(compute_contact_quality(&samples, 45.0), ContactQuality::NoContact);
    }

    #[test]
    fn tracker_hysteresis() {
        let mut t = ContactQualityTracker::new(1); // AF7 — frontal, rms_good=45
        assert_eq!(t.quality(), ContactQuality::NoContact);

        // Feed clean signal (RMS ≈ 10.6 µV) — needs HYSTERESIS_COUNT consecutive
        let good: Vec<f64> = (0..256)
            .map(|i| 15.0 * (2.0 * std::f64::consts::PI * 10.0 * i as f64 / 256.0).sin())
            .collect();

        // First two readings shouldn't change state yet
        t.update(&good);
        assert_eq!(t.quality(), ContactQuality::NoContact);
        t.update(&good);
        assert_eq!(t.quality(), ContactQuality::NoContact);
        // Third consecutive reading triggers transition
        t.update(&good);
        assert_eq!(t.quality(), ContactQuality::Good);
    }

    #[test]
    fn tracker_hysteresis_interrupted() {
        let mut t = ContactQualityTracker::new(0); // TP9 — temporal

        let good: Vec<f64> = (0..256)
            .map(|i| 15.0 * (2.0 * std::f64::consts::PI * 10.0 * i as f64 / 256.0).sin())
            .collect();
        let noisy: Vec<f64> = (0..256)
            .map(|i| 180.0 * (2.0 * std::f64::consts::PI * 10.0 * i as f64 / 256.0).sin())
            .collect();

        // Two good readings, then a noisy one interrupts the streak
        t.update(&good);
        t.update(&good);
        t.update(&noisy); // interrupts
        assert_eq!(t.quality(), ContactQuality::NoContact);

        // Restart the streak — three more good readings needed
        t.update(&good);
        t.update(&good);
        assert_eq!(t.quality(), ContactQuality::NoContact);
        t.update(&good);
        assert_eq!(t.quality(), ContactQuality::Good);
    }

    #[test]
    fn touching_forehead_logic() {
        let good_all = [
            ContactQuality::Good,
            ContactQuality::Good,
            ContactQuality::Good,
            ContactQuality::Good,
        ];
        assert!(touching_forehead(&good_all));

        let no_frontal = [
            ContactQuality::Good,
            ContactQuality::NoContact,
            ContactQuality::NoContact,
            ContactQuality::Good,
        ];
        assert!(!touching_forehead(&no_frontal));

        let one_frontal = [
            ContactQuality::NoContact,
            ContactQuality::Ok,
            ContactQuality::NoContact,
            ContactQuality::NoContact,
        ];
        assert!(touching_forehead(&one_frontal));
    }
}
