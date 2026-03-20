//! Raw `extern "C"` bindings for the ACT C++ bridge (`ffi/act_bridge.h`).
//!
//! This module is only compiled when the `act` Cargo feature is enabled.
//! All functions are unsafe and operate on opaque handles — prefer the safe
//! wrapper in [`crate::act`] for normal usage.

use std::os::raw::{c_char, c_double, c_float, c_int, c_void};

/// Parameter ranges for chirplet dictionary generation.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct ActParameterRanges {
    pub tc_min: c_double,
    pub tc_max: c_double,
    pub tc_step: c_double,
    pub fc_min: c_double,
    pub fc_max: c_double,
    pub fc_step: c_double,
    pub log_dt_min: c_double,
    pub log_dt_max: c_double,
    pub log_dt_step: c_double,
    pub c_min: c_double,
    pub c_max: c_double,
    pub c_step: c_double,
}

/// Transform options.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct ActTransformOpts {
    pub order: c_int,
    pub residual_threshold: c_double,
    pub refine: bool,
}

/// Batch result buffer filled by [`act_transform_batch`].
#[repr(C)]
#[derive(Debug)]
pub struct ActBatchResult {
    pub n_signals: c_int,
    pub max_order: c_int,
    pub params: *mut c_float,
    pub coeffs: *mut c_float,
    pub errors: *mut c_float,
    pub used_orders: *mut c_int,
}

impl Default for ActBatchResult {
    fn default() -> Self {
        Self {
            n_signals: 0,
            max_order: 0,
            params: std::ptr::null_mut(),
            coeffs: std::ptr::null_mut(),
            errors: std::ptr::null_mut(),
            used_orders: std::ptr::null_mut(),
        }
    }
}

extern "C" {
    // ── Lifecycle ─────────────────────────────────────────────────────────────
    pub fn act_create(
        fs: c_double,
        length: c_int,
        ranges: *const ActParameterRanges,
        verbose: bool,
    ) -> *mut c_void;

    pub fn act_load_dictionary(path: *const c_char, verbose: bool) -> *mut c_void;

    pub fn act_save_dictionary(handle: *mut c_void, path: *const c_char) -> bool;

    pub fn act_destroy(handle: *mut c_void);

    // ── Accessors ─────────────────────────────────────────────────────────────
    pub fn act_get_dict_size(handle: *mut c_void) -> c_int;
    pub fn act_get_length(handle: *mut c_void) -> c_int;
    pub fn act_get_fs(handle: *mut c_void) -> c_double;

    // ── Batch transform ───────────────────────────────────────────────────────
    pub fn act_transform_batch(
        handle: *mut c_void,
        signals_flat: *const c_double,
        n_signals: c_int,
        opts: *const ActTransformOpts,
        out: *mut ActBatchResult,
    ) -> c_int;

    pub fn act_free_result(result: *mut ActBatchResult);
}

// ── Smoke tests ──────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ffi_create_destroy() {
        let ranges = ActParameterRanges {
            tc_min: 0.0,
            tc_max: 31.0,
            tc_step: 8.0,
            fc_min: 1.0,
            fc_max: 15.0,
            fc_step: 2.0,
            log_dt_min: -2.0,
            log_dt_max: -0.5,
            log_dt_step: 0.5,
            c_min: -10.0,
            c_max: 10.0,
            c_step: 5.0,
        };

        unsafe {
            let h = act_create(256.0, 32, &ranges, false);
            assert!(!h.is_null(), "act_create returned null");

            let ds = act_get_dict_size(h);
            assert!(ds > 0, "dict_size should be > 0, got {ds}");

            let len = act_get_length(h);
            assert_eq!(len, 32, "length mismatch");

            let fs = act_get_fs(h);
            assert!((fs - 256.0).abs() < 1e-6, "fs mismatch");

            act_destroy(h);
        }
    }

    #[test]
    fn ffi_transform_batch_roundtrip() {
        let ranges = ActParameterRanges {
            tc_min: 0.0,
            tc_max: 31.0,
            tc_step: 8.0,
            fc_min: 1.0,
            fc_max: 15.0,
            fc_step: 2.0,
            log_dt_min: -2.0,
            log_dt_max: -0.5,
            log_dt_step: 0.5,
            c_min: -10.0,
            c_max: 10.0,
            c_step: 5.0,
        };

        unsafe {
            let h = act_create(256.0, 32, &ranges, false);
            assert!(!h.is_null());

            // Generate 2 sine signals
            let length = 32;
            let n_signals = 2;
            let mut signals = vec![0.0f64; n_signals * length];
            for ch in 0..n_signals {
                for i in 0..length {
                    let t = i as f64 / 256.0;
                    signals[ch * length + i] =
                        0.5 * (2.0 * std::f64::consts::PI * 10.0 * t).sin();
                }
            }

            let opts = ActTransformOpts {
                order: 3,
                residual_threshold: 1e-6,
                refine: false,
            };

            let mut result = ActBatchResult::default();
            let rc = act_transform_batch(
                h,
                signals.as_ptr(),
                n_signals as c_int,
                &opts,
                &mut result,
            );
            assert_eq!(rc, 0, "transform_batch failed with rc={rc}");
            assert_eq!(result.n_signals, n_signals as c_int);
            assert_eq!(result.max_order, 3);

            // Check we got valid results
            for s in 0..n_signals {
                let used = *result.used_orders.add(s);
                assert!(used > 0, "signal {s} had 0 chirplets");
                let err = *result.errors.add(s);
                assert!(err >= 0.0, "negative error for signal {s}");
            }

            act_free_result(&mut result);
            act_destroy(h);
        }
    }
}
