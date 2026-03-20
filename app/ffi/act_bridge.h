/**
 * act_bridge.h — C API for the ACT_MLX_f (float32) adaptive chirplet transform.
 *
 * This thin shim exposes the C++ ACT_MLX_f batch-transform pipeline through
 * an opaque handle so that it can be called from Rust via FFI.
 */
#ifndef ACT_BRIDGE_H
#define ACT_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/* Opaque handle to an ACT_MLX_f instance. */
typedef void* ActHandle;

/* Parameter ranges for dictionary generation. */
typedef struct {
    double tc_min,    tc_max,    tc_step;
    double fc_min,    fc_max,    fc_step;
    double logDt_min, logDt_max, logDt_step;
    double c_min,     c_max,     c_step;
} ActParameterRanges;

/* Transform options. */
typedef struct {
    int    order;               /* number of chirplets to extract          */
    double residual_threshold;  /* early-stopping residual threshold       */
    bool   refine;              /* true = BFGS refinement after coarse     */
} ActTransformOpts;

/* Result buffer filled by act_transform_batch. Caller owns all pointers. */
typedef struct {
    int    n_signals;
    int    max_order;
    float* params;       /* [n_signals * max_order * 4]  (tc,fc,logDt,c) */
    float* coeffs;       /* [n_signals * max_order]                       */
    float* errors;       /* [n_signals]                                   */
    int*   used_orders;  /* [n_signals]                                   */
} ActBatchResult;

/* ── Lifecycle ─────────────────────────────────────────────────────────── */

/**
 * Create a new ACT engine, generate the chirplet dictionary from ranges.
 * Returns NULL on failure.
 */
ActHandle act_create(double fs, int length,
                     const ActParameterRanges* ranges,
                     bool verbose);

/**
 * Load a pre-generated dictionary from a binary file.
 * Returns NULL on failure.
 */
ActHandle act_load_dictionary(const char* path, bool verbose);

/**
 * Save the current dictionary to a binary file.
 * Returns true on success.
 */
bool act_save_dictionary(ActHandle handle, const char* path);

/**
 * Destroy an ACT engine and free all associated memory.
 */
void act_destroy(ActHandle handle);

/* ── Accessors ─────────────────────────────────────────────────────────── */

int    act_get_dict_size(ActHandle handle);
int    act_get_length(ActHandle handle);
double act_get_fs(ActHandle handle);

/* ── Batch transform ───────────────────────────────────────────────────── */

/**
 * Run the adaptive chirplet transform on a batch of signals.
 *
 * @param handle       Engine handle (must not be NULL).
 * @param signals_flat Contiguous row-major [n_signals * length] doubles.
 * @param n_signals    Number of signals in the batch.
 * @param opts         Transform options.
 * @param out          Caller-allocated result struct. The inner pointer
 *                     fields (params, coeffs, errors, used_orders) must
 *                     point to buffers of the correct sizes, OR be NULL
 *                     in which case act_transform_batch will allocate them
 *                     (caller must then free with act_free_result).
 * @return 0 on success, non-zero on error.
 */
int act_transform_batch(ActHandle handle,
                        const double* signals_flat,
                        int n_signals,
                        const ActTransformOpts* opts,
                        ActBatchResult* out);

/**
 * Free the inner buffers of an ActBatchResult that was allocated by
 * act_transform_batch (i.e. when the caller passed NULL pointers).
 */
void act_free_result(ActBatchResult* result);

#ifdef __cplusplus
}
#endif

#endif /* ACT_BRIDGE_H */
