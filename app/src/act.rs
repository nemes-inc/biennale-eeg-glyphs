//! Safe Rust wrapper around the ACT C++ FFI bridge.
//!
//! Provides [`ActEngine`] for creating / loading chirplet dictionaries and
//! running batched adaptive chirplet transforms on multi-channel EEG data.
//!
//! This module is only compiled when the `act` Cargo feature is enabled.

use std::ffi::CString;
use std::os::raw::c_void;

use crate::act_ffi;

// ── Configuration types ──────────────────────────────────────────────────────

/// Parameter ranges that define the chirplet dictionary grid.
#[derive(Debug, Clone)]
pub struct ActConfig {
    /// Sampling frequency in Hz (e.g. 256.0 for Muse EEG).
    pub fs: f64,
    /// Window length in samples.  Must match the dictionary length.
    pub length: usize,
    /// Time-center range: (min, max, step) in samples.
    pub tc_range: (f64, f64, f64),
    /// Frequency-center range: (min, max, step) in Hz.
    pub fc_range: (f64, f64, f64),
    /// Log-duration range: (min, max, step).
    pub log_dt_range: (f64, f64, f64),
    /// Chirp-rate range: (min, max, step) in Hz/s.
    pub chirp_range: (f64, f64, f64),
}

/// Options controlling a single transform invocation.
#[derive(Debug, Clone)]
pub struct TransformOpts {
    /// Number of chirplets to extract (matching pursuit order).
    pub order: usize,
    /// Residual energy threshold for early stopping.
    pub residual_threshold: f64,
    /// If `true`, run BFGS refinement after coarse dictionary search.
    pub refine: bool,
}

impl Default for TransformOpts {
    fn default() -> Self {
        Self {
            order: 5,
            residual_threshold: 1e-6,
            refine: false,
        }
    }
}

/// A single extracted chirplet atom.
#[derive(Debug, Clone, Copy)]
pub struct Chirplet {
    /// Time center (in samples within the window).
    pub tc: f32,
    /// Frequency center (Hz).
    pub fc: f32,
    /// Log duration parameter.
    pub log_dt: f32,
    /// Chirp rate (Hz/s).
    pub chirp_rate: f32,
    /// Amplitude coefficient.
    pub coeff: f32,
}

/// Result of the transform for a single signal / channel.
#[derive(Debug, Clone)]
pub struct ChannelResult {
    /// Extracted chirplets, ordered by energy contribution (strongest first).
    pub chirplets: Vec<Chirplet>,
    /// L2 residual norm after extraction.
    pub error: f32,
}

// ── Engine ───────────────────────────────────────────────────────────────────

/// Opaque handle to an ACT_MLX_f instance on the C++ side.
///
/// `ActEngine` is `Send` because the underlying C++ object is thread-safe for
/// read-only use (dictionary search is const).  It is **not** `Sync` because
/// MLX device state is not designed for concurrent access from multiple threads.
pub struct ActEngine {
    handle: *mut c_void,
}

// Safety: the ACT_MLX_f instance is heap-allocated and only accessed through
// &self (read-only transform) or via exclusive &mut self (destroy).
unsafe impl Send for ActEngine {}

impl ActEngine {
    /// Create a new engine and generate the chirplet dictionary from `config`.
    pub fn new(config: &ActConfig) -> anyhow::Result<Self> {
        let ranges = act_ffi::ActParameterRanges {
            tc_min: config.tc_range.0,
            tc_max: config.tc_range.1,
            tc_step: config.tc_range.2,
            fc_min: config.fc_range.0,
            fc_max: config.fc_range.1,
            fc_step: config.fc_range.2,
            log_dt_min: config.log_dt_range.0,
            log_dt_max: config.log_dt_range.1,
            log_dt_step: config.log_dt_range.2,
            c_min: config.chirp_range.0,
            c_max: config.chirp_range.1,
            c_step: config.chirp_range.2,
        };

        let handle = unsafe {
            act_ffi::act_create(config.fs, config.length as i32, &ranges, false)
        };

        if handle.is_null() {
            anyhow::bail!("act_create failed (returned null)");
        }
        Ok(Self { handle })
    }

    /// Load a pre-generated dictionary from a binary file.
    pub fn load(path: &str) -> anyhow::Result<Self> {
        let c_path = CString::new(path)?;
        let handle = unsafe { act_ffi::act_load_dictionary(c_path.as_ptr(), false) };
        if handle.is_null() {
            anyhow::bail!("act_load_dictionary failed for '{path}'");
        }
        Ok(Self { handle })
    }

    /// Save the current dictionary to a binary file.
    pub fn save(&self, path: &str) -> anyhow::Result<()> {
        let c_path = CString::new(path)?;
        let ok = unsafe { act_ffi::act_save_dictionary(self.handle, c_path.as_ptr()) };
        if !ok {
            anyhow::bail!("act_save_dictionary failed for '{path}'");
        }
        Ok(())
    }

    /// Number of chirplet atoms in the dictionary.
    pub fn dict_size(&self) -> usize {
        unsafe { act_ffi::act_get_dict_size(self.handle) as usize }
    }

    /// Window length in samples (must match signal lengths passed to transform).
    pub fn window_length(&self) -> usize {
        unsafe { act_ffi::act_get_length(self.handle) as usize }
    }

    /// Sampling frequency used to build the dictionary.
    pub fn fs(&self) -> f64 {
        unsafe { act_ffi::act_get_fs(self.handle) }
    }

    /// Run the adaptive chirplet transform on a batch of signals.
    ///
    /// Each element of `signals` is one channel's window of `window_length()`
    /// samples.  Returns one [`ChannelResult`] per input signal.
    pub fn transform_batch(
        &self,
        signals: &[&[f64]],
        opts: &TransformOpts,
    ) -> anyhow::Result<Vec<ChannelResult>> {
        let n_signals = signals.len();
        if n_signals == 0 {
            return Ok(vec![]);
        }

        let length = self.window_length();
        for (i, s) in signals.iter().enumerate() {
            anyhow::ensure!(
                s.len() == length,
                "signal {i} has length {} but engine expects {length}",
                s.len()
            );
        }

        // Flatten into contiguous row-major buffer
        let mut flat = Vec::with_capacity(n_signals * length);
        for s in signals {
            flat.extend_from_slice(s);
        }

        let c_opts = act_ffi::ActTransformOpts {
            order: opts.order as i32,
            residual_threshold: opts.residual_threshold,
            refine: opts.refine,
        };

        let mut result = act_ffi::ActBatchResult::default();
        let rc = unsafe {
            act_ffi::act_transform_batch(
                self.handle,
                flat.as_ptr(),
                n_signals as i32,
                &c_opts,
                &mut result,
            )
        };

        if rc != 0 {
            // Clean up any partial allocation
            unsafe { act_ffi::act_free_result(&mut result) };
            anyhow::bail!("act_transform_batch failed with rc={rc}");
        }

        let order = opts.order;
        let mut out = Vec::with_capacity(n_signals);

        for s in 0..n_signals {
            let used = unsafe { *result.used_orders.add(s) } as usize;
            let error = unsafe { *result.errors.add(s) };

            let mut chirplets = Vec::with_capacity(used);
            for i in 0..used.min(order) {
                let pidx = (s * order + i) * 4;
                let tc = unsafe { *result.params.add(pidx) };
                let fc = unsafe { *result.params.add(pidx + 1) };
                let log_dt = unsafe { *result.params.add(pidx + 2) };
                let chirp_rate = unsafe { *result.params.add(pidx + 3) };
                let coeff = unsafe { *result.coeffs.add(s * order + i) };

                chirplets.push(Chirplet {
                    tc,
                    fc,
                    log_dt,
                    chirp_rate,
                    coeff,
                });
            }

            out.push(ChannelResult { chirplets, error });
        }

        unsafe { act_ffi::act_free_result(&mut result) };
        Ok(out)
    }
}

impl Drop for ActEngine {
    fn drop(&mut self) {
        if !self.handle.is_null() {
            unsafe { act_ffi::act_destroy(self.handle) };
            self.handle = std::ptr::null_mut();
        }
    }
}

// ── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    /// Small dictionary config for fast tests.
    fn test_config() -> ActConfig {
        ActConfig {
            fs: 256.0,
            length: 64,
            tc_range: (0.0, 63.0, 16.0),
            fc_range: (1.0, 30.0, 2.0),
            log_dt_range: (-2.0, -0.3, 0.4),
            chirp_range: (-15.0, 15.0, 5.0),
        }
    }

    /// Generate a pure sine at `freq_hz` sampled at `fs` for `n` samples.
    fn sine(freq_hz: f64, fs: f64, n: usize) -> Vec<f64> {
        (0..n)
            .map(|i| (2.0 * PI * freq_hz * i as f64 / fs).sin())
            .collect()
    }

    #[test]
    fn act_engine_synthetic() {
        let cfg = test_config();
        let engine = ActEngine::new(&cfg).expect("create failed");
        assert!(engine.dict_size() > 0);
        assert_eq!(engine.window_length(), 64);

        let sig = sine(10.0, 256.0, 64);
        let opts = TransformOpts {
            order: 5,
            residual_threshold: 1e-6,
            refine: false,
        };

        let results = engine.transform_batch(&[&sig], &opts).expect("transform failed");
        assert_eq!(results.len(), 1);
        assert!(!results[0].chirplets.is_empty(), "no chirplets extracted");

        // Dominant chirplet should have fc reasonably near 10 Hz
        let top_fc = results[0].chirplets[0].fc;
        assert!(
            (top_fc - 10.0).abs() < 6.0,
            "dominant fc={top_fc}, expected near 10 Hz"
        );
    }

    #[test]
    fn act_engine_save_load() {
        let cfg = test_config();
        let engine = ActEngine::new(&cfg).expect("create failed");
        let ds = engine.dict_size();

        let path = "/tmp/muse_rs_act_test.bin";
        engine.save(path).expect("save failed");

        let loaded = ActEngine::load(path).expect("load failed");
        assert_eq!(loaded.dict_size(), ds);
        assert_eq!(loaded.window_length(), engine.window_length());

        // Transform the same signal and verify similar results
        let sig = sine(10.0, 256.0, 64);
        let opts = TransformOpts::default();

        let r1 = engine
            .transform_batch(&[&sig], &opts)
            .expect("transform 1 failed");
        let r2 = loaded
            .transform_batch(&[&sig], &opts)
            .expect("transform 2 failed");

        assert_eq!(r1[0].chirplets.len(), r2[0].chirplets.len());
        // Errors should be identical (same dictionary)
        assert!(
            (r1[0].error - r2[0].error).abs() < 1e-4,
            "error mismatch: {} vs {}",
            r1[0].error,
            r2[0].error
        );

        // Clean up
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn act_engine_batch_4ch() {
        let cfg = test_config();
        let engine = ActEngine::new(&cfg).expect("create failed");

        // 4 channels at different frequencies
        let freqs = [8.0, 10.0, 12.0, 20.0];
        let sigs: Vec<Vec<f64>> = freqs.iter().map(|&f| sine(f, 256.0, 64)).collect();
        let refs: Vec<&[f64]> = sigs.iter().map(|s| s.as_slice()).collect();

        let opts = TransformOpts {
            order: 5,
            residual_threshold: 1e-6,
            refine: false,
        };

        let results = engine.transform_batch(&refs, &opts).expect("batch failed");
        assert_eq!(results.len(), 4);

        for (ch, r) in results.iter().enumerate() {
            assert!(
                !r.chirplets.is_empty(),
                "channel {ch} produced no chirplets"
            );
            assert!(
                r.error < 2.0,
                "channel {ch} error {} too large",
                r.error
            );
        }
    }
}
