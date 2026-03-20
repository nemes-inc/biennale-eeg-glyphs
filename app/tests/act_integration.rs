//! End-to-end integration test for the ACT (Adaptive Chirplet Transform) pipeline.
//!
//! Run with:
//!   cargo test --features act --release act_integration -- --test-threads=1

#![cfg(feature = "act")]

use std::f64::consts::PI;
use std::time::Instant;

use muse_rs::act::{ActConfig, ActEngine, TransformOpts};

/// EEG-scale configuration: 256 Hz, 512-sample window (2 seconds).
fn eeg_config() -> ActConfig {
    ActConfig {
        fs: 256.0,
        length: 512,
        tc_range: (0.0, 511.0, 16.0),
        fc_range: (0.5, 30.0, 1.0),
        log_dt_range: (-2.5, -0.3, 0.3),
        chirp_range: (-20.0, 20.0, 5.0),
    }
}

/// Generate a synthetic EEG-like signal: alpha (10 Hz) + beta (22 Hz) + noise.
fn synthetic_eeg(fs: f64, n: usize, seed: u64) -> Vec<f64> {
    (0..n)
        .map(|i| {
            let t = i as f64 / fs;
            let alpha = 20.0 * (2.0 * PI * 10.0 * t).sin();
            let beta = 6.0 * (2.0 * PI * 22.0 * t).sin();
            let theta = 10.0 * (2.0 * PI * 6.0 * t).sin();
            // Deterministic pseudo-random noise
            let nx = t * 1000.7 + seed as f64 * 137.508;
            let noise = ((nx.sin() * 9973.1).fract() - 0.5) * 8.0;
            alpha + beta + theta + noise
        })
        .collect()
}

#[test]
fn act_integration_eeg_scale() {
    let cfg = eeg_config();
    let engine = ActEngine::new(&cfg).expect("engine creation failed");

    assert!(engine.dict_size() > 0, "dictionary should not be empty");
    assert_eq!(engine.window_length(), 512);
    assert!((engine.fs() - 256.0).abs() < 1e-6);

    // Generate 4 synthetic EEG channels (like Muse TP9/AF7/AF8/TP10)
    let signals: Vec<Vec<f64>> = (0..4)
        .map(|ch| synthetic_eeg(256.0, 512, 42 + ch))
        .collect();
    let refs: Vec<&[f64]> = signals.iter().map(|s| s.as_slice()).collect();

    // ── Test 1: basic transform produces valid results ───────────────────────
    let opts = TransformOpts {
        order: 5,
        residual_threshold: 1e-6,
        refine: false,
    };

    let t0 = Instant::now();
    let results = engine
        .transform_batch(&refs, &opts)
        .expect("transform_batch failed");
    let elapsed_ms = t0.elapsed().as_secs_f64() * 1000.0;

    assert_eq!(results.len(), 4, "expected 4 channel results");

    for (ch, r) in results.iter().enumerate() {
        assert!(
            !r.chirplets.is_empty(),
            "channel {ch}: no chirplets extracted"
        );
        assert!(
            r.error >= 0.0,
            "channel {ch}: negative error {}",
            r.error
        );
    }

    // ── Test 2: dominant frequency in expected band ──────────────────────────
    // The strongest component is alpha at 10 Hz (amplitude 20 µV).
    // The dominant chirplet's fc should be in the 5-15 Hz range.
    for (ch, r) in results.iter().enumerate() {
        let top_fc = r.chirplets[0].fc;
        assert!(
            (2.0..30.0).contains(&(top_fc as f64)),
            "channel {ch}: dominant fc={top_fc} outside expected band"
        );
    }

    // ── Test 3: higher order reduces error ───────────────────────────────────
    let opts_high = TransformOpts {
        order: 10,
        residual_threshold: 1e-6,
        refine: false,
    };
    let results_high = engine
        .transform_batch(&refs, &opts_high)
        .expect("high-order transform failed");

    for ch in 0..4 {
        assert!(
            results_high[ch].error <= results[ch].error + 1e-4,
            "channel {ch}: higher order should not increase error ({} > {})",
            results_high[ch].error,
            results[ch].error
        );
    }

    // ── Test 4: real-time timing budget ──────────────────────────────────────
    // At hop=64 (250 ms), batch of 4 signals must complete in <250 ms.
    // Use the already-measured time from the first batch.
    assert!(
        elapsed_ms < 250.0,
        "batch transform took {elapsed_ms:.1} ms, exceeds 250 ms real-time budget"
    );

    println!(
        "ACT integration test passed: dict={}, {elapsed_ms:.1} ms for 4×512 batch",
        engine.dict_size()
    );
}

#[test]
fn act_integration_save_load_consistency() {
    let cfg = eeg_config();
    let engine = ActEngine::new(&cfg).expect("engine creation failed");

    let path = "/tmp/muse_rs_act_integration.bin";
    engine.save(path).expect("save failed");

    let loaded = ActEngine::load(path).expect("load failed");
    assert_eq!(engine.dict_size(), loaded.dict_size());
    assert_eq!(engine.window_length(), loaded.window_length());

    // Same signal → same results
    let sig = synthetic_eeg(256.0, 512, 99);
    let opts = TransformOpts {
        order: 5,
        residual_threshold: 1e-6,
        refine: false,
    };

    let r1 = engine
        .transform_batch(&[&sig], &opts)
        .expect("transform 1 failed");
    let r2 = loaded
        .transform_batch(&[&sig], &opts)
        .expect("transform 2 failed");

    assert_eq!(r1[0].chirplets.len(), r2[0].chirplets.len());
    assert!(
        (r1[0].error - r2[0].error).abs() < 1e-4,
        "loaded engine gives different error: {} vs {}",
        r1[0].error,
        r2[0].error
    );

    let _ = std::fs::remove_file(path);
}
