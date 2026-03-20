/**
 * test_bridge.cpp — Standalone smoke test for the act_bridge C API.
 *
 * Compile & run:
 *   g++ -std=c++17 -O2 -DUSE_MLX -DACCELERATE_NEW_LAPACK \
 *       -I../ACT-lib/actlib/include -I../ACT-lib/actlib/lib -I/usr/local/include \
 *       -o test_bridge test_bridge.cpp act_bridge.cpp \
 *       ../ACT-lib/actlib/src/ACT.cpp ../ACT-lib/actlib/src/ACT_CPU.cpp \
 *       ../ACT-lib/actlib/src/ACT_Accelerate.cpp ../ACT-lib/actlib/src/ACT_MLX.cpp \
 *       ../ACT-lib/actlib/lib/alglib/alglib-cpp/src/ap.cpp \
 *       ../ACT-lib/actlib/lib/alglib/alglib-cpp/src/alglibinternal.cpp \
 *       ../ACT-lib/actlib/lib/alglib/alglib-cpp/src/alglibmisc.cpp \
 *       ../ACT-lib/actlib/lib/alglib/alglib-cpp/src/optimization.cpp \
 *       ../ACT-lib/actlib/lib/alglib/alglib-cpp/src/linalg.cpp \
 *       ../ACT-lib/actlib/lib/alglib/alglib-cpp/src/solvers.cpp \
 *       ../ACT-lib/actlib/lib/alglib/alglib-cpp/src/dataanalysis.cpp \
 *       ../ACT-lib/actlib/lib/alglib/alglib-cpp/src/interpolation.cpp \
 *       ../ACT-lib/actlib/lib/alglib/alglib-cpp/src/specialfunctions.cpp \
 *       ../ACT-lib/actlib/lib/alglib/alglib-cpp/src/statistics.cpp \
 *       ../ACT-lib/actlib/lib/alglib/alglib-cpp/src/fasttransforms.cpp \
 *       ../ACT-lib/actlib/lib/alglib/alglib-cpp/src/integration.cpp \
 *       ../ACT-lib/actlib/lib/alglib/alglib-cpp/src/diffequations.cpp \
 *       -L/usr/local/lib -lmlx \
 *       -framework Accelerate -framework Metal -framework MetalPerformanceShaders \
 *       -framework Foundation -framework QuartzCore \
 *       -lm -lpthread
 *   ./test_bridge
 */

#include "act_bridge.h"
#include <cstdio>
#include <cmath>
#include <cstdlib>

static const double PI = 3.14159265358979323846;

int main() {
    printf("=== act_bridge smoke test ===\n");

    /* Dictionary parameters (small for speed) */
    ActParameterRanges ranges;
    ranges.tc_min = 0;   ranges.tc_max = 31;  ranges.tc_step = 8;
    ranges.fc_min = 1.0; ranges.fc_max = 15.0; ranges.fc_step = 2.0;
    ranges.logDt_min = -2.0; ranges.logDt_max = -0.5; ranges.logDt_step = 0.5;
    ranges.c_min = -10.0; ranges.c_max = 10.0; ranges.c_step = 5.0;

    const double fs = 256.0;
    const int length = 32;

    /* 1. Create engine */
    ActHandle h = act_create(fs, length, &ranges, true);
    if (!h) { fprintf(stderr, "FAIL: act_create returned NULL\n"); return 1; }

    int ds = act_get_dict_size(h);
    int len = act_get_length(h);
    double got_fs = act_get_fs(h);
    printf("  dict_size=%d  length=%d  fs=%.1f\n", ds, len, got_fs);
    if (ds <= 0 || len != length) { fprintf(stderr, "FAIL: bad accessors\n"); act_destroy(h); return 1; }

    /* 2. Generate 4 synthetic signals (10 Hz sine per channel, different phases) */
    const int n_signals = 4;
    double* signals = (double*)malloc(sizeof(double) * n_signals * length);
    for (int ch = 0; ch < n_signals; ++ch) {
        double phase = ch * PI / 4.0;
        for (int i = 0; i < length; ++i) {
            double t = (double)i / fs;
            signals[ch * length + i] = 0.5 * sin(2.0 * PI * 10.0 * t + phase);
        }
    }

    /* 3. Transform batch */
    ActTransformOpts opts;
    opts.order = 3;
    opts.residual_threshold = 1e-6;
    opts.refine = false;

    ActBatchResult result;
    result.params = NULL;
    result.coeffs = NULL;
    result.errors = NULL;
    result.used_orders = NULL;

    int rc = act_transform_batch(h, signals, n_signals, &opts, &result);
    if (rc != 0) { fprintf(stderr, "FAIL: act_transform_batch returned %d\n", rc); act_destroy(h); free(signals); return 1; }

    printf("  n_signals=%d  max_order=%d\n", result.n_signals, result.max_order);
    for (int s = 0; s < n_signals; ++s) {
        printf("  signal %d: used_order=%d  error=%.4f", s, result.used_orders[s], result.errors[s]);
        if (result.used_orders[s] > 0) {
            int idx = s * opts.order;
            printf("  chirp0: tc=%.2f fc=%.2f logDt=%.2f c=%.2f coeff=%.4f",
                   result.params[idx*4+0], result.params[idx*4+1],
                   result.params[idx*4+2], result.params[idx*4+3],
                   result.coeffs[idx]);
        }
        printf("\n");
    }

    /* 4. Save / load round-trip */
    const char* dict_path = "/tmp/act_bridge_test.bin";
    bool saved = act_save_dictionary(h, dict_path);
    printf("  save_dictionary: %s\n", saved ? "OK" : "FAIL");

    if (saved) {
        ActHandle h2 = act_load_dictionary(dict_path, false);
        if (h2) {
            int ds2 = act_get_dict_size(h2);
            printf("  load_dictionary: OK  dict_size=%d (match=%s)\n", ds2, ds2 == ds ? "yes" : "NO");
            act_destroy(h2);
        } else {
            printf("  load_dictionary: FAIL (returned NULL)\n");
        }
    }

    /* Cleanup */
    act_free_result(&result);
    free(signals);
    act_destroy(h);

    printf("=== bridge OK ===\n");
    return 0;
}
