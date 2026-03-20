/**
 * act_bridge.cpp — C API implementation for ACT_MLX_f batch transforms.
 *
 * Wraps the C++ ACT_MLX_f class behind an opaque handle with extern "C"
 * linkage so that Rust (or any C consumer) can call into it via FFI.
 */

#include "act_bridge.h"

#include "ACT_CPU.h"
#include "ACT_Accelerate.h"
#include "ACT_MLX.h"
#include "ACT_MLX_MT.h"
#include "ACT_CPU_MT.h"

#include <vector>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <iostream>

/* ── helpers ───────────────────────────────────────────────────────────── */

static inline ACT_MLX_f* as_act(ActHandle h) {
    return reinterpret_cast<ACT_MLX_f*>(h);
}

/* ── Lifecycle ─────────────────────────────────────────────────────────── */

ActHandle act_create(double fs, int length,
                     const ActParameterRanges* ranges,
                     bool verbose)
{
    if (!ranges) return nullptr;

    ACT_MLX_f::ParameterRanges pr(
        ranges->tc_min,    ranges->tc_max,    ranges->tc_step,
        ranges->fc_min,    ranges->fc_max,    ranges->fc_step,
        ranges->logDt_min, ranges->logDt_max, ranges->logDt_step,
        ranges->c_min,     ranges->c_max,     ranges->c_step
    );

    try {
        auto* act = new ACT_MLX_f(fs, length, pr, verbose);
        act->generate_chirplet_dictionary();
        return reinterpret_cast<ActHandle>(act);
    } catch (const std::exception& e) {
        if (verbose) std::cerr << "[act_bridge] act_create failed: " << e.what() << std::endl;
        return nullptr;
    }
}

ActHandle act_load_dictionary(const char* path, bool verbose)
{
    if (!path) return nullptr;
    try {
        auto ptr = ACT_CPU_T<float>::load_dictionary<ACT_MLX_f>(std::string(path), verbose);
        if (!ptr) return nullptr;
        return reinterpret_cast<ActHandle>(ptr.release());
    } catch (const std::exception& e) {
        if (verbose) std::cerr << "[act_bridge] act_load_dictionary failed: " << e.what() << std::endl;
        return nullptr;
    }
}

bool act_save_dictionary(ActHandle handle, const char* path)
{
    if (!handle || !path) return false;
    try {
        return as_act(handle)->save_dictionary(std::string(path));
    } catch (...) {
        return false;
    }
}

void act_destroy(ActHandle handle)
{
    delete as_act(handle);
}

/* ── Accessors ─────────────────────────────────────────────────────────── */

int act_get_dict_size(ActHandle handle)
{
    return handle ? as_act(handle)->get_dict_size() : 0;
}

int act_get_length(ActHandle handle)
{
    return handle ? as_act(handle)->get_length() : 0;
}

double act_get_fs(ActHandle handle)
{
    return handle ? as_act(handle)->get_FS() : 0.0;
}

/* ── Batch transform ───────────────────────────────────────────────────── */

int act_transform_batch(ActHandle handle,
                        const double* signals_flat,
                        int n_signals,
                        const ActTransformOpts* opts,
                        ActBatchResult* out)
{
    if (!handle || !signals_flat || n_signals <= 0 || !opts || !out)
        return -1;

    ACT_MLX_f* act = as_act(handle);
    const int length = act->get_length();
    const int order  = opts->order;

    /* Pack signals into Eigen vectors */
    std::vector<Eigen::VectorXd> xs;
    xs.reserve(n_signals);
    for (int i = 0; i < n_signals; ++i) {
        xs.emplace_back(Eigen::Map<const Eigen::VectorXd>(
            signals_flat + static_cast<size_t>(i) * length, length));
    }

    /* Transform options */
    ACT_CPU::TransformOptions topts;
    topts.order = order;
    topts.residual_threshold = opts->residual_threshold;
    topts.refine = opts->refine;

    /* Run batched transform (MLX GPU coarse → optional BFGS refine) */
    std::vector<ACT_MLX_f::TransformResult> results;
    try {
        results = actmlx::transform_batch(*act, xs, topts);
    } catch (const std::exception& e) {
        std::cerr << "[act_bridge] transform_batch failed: " << e.what() << std::endl;
        return -2;
    }

    if (static_cast<int>(results.size()) != n_signals) return -3;

    /* Determine whether we need to allocate output buffers */
    bool alloc = (out->params == nullptr);
    out->n_signals = n_signals;
    out->max_order = order;

    const size_t params_len  = static_cast<size_t>(n_signals) * order * 4;
    const size_t coeffs_len  = static_cast<size_t>(n_signals) * order;
    const size_t errors_len  = static_cast<size_t>(n_signals);
    const size_t orders_len  = static_cast<size_t>(n_signals);

    if (alloc) {
        out->params      = static_cast<float*>(calloc(params_len,  sizeof(float)));
        out->coeffs      = static_cast<float*>(calloc(coeffs_len,  sizeof(float)));
        out->errors      = static_cast<float*>(calloc(errors_len,  sizeof(float)));
        out->used_orders = static_cast<int*>  (calloc(orders_len,  sizeof(int)));
        if (!out->params || !out->coeffs || !out->errors || !out->used_orders) {
            act_free_result(out);
            return -4;
        }
    } else {
        /* Zero caller-provided buffers */
        memset(out->params,      0, params_len * sizeof(float));
        memset(out->coeffs,      0, coeffs_len * sizeof(float));
        memset(out->errors,      0, errors_len * sizeof(float));
        memset(out->used_orders, 0, orders_len * sizeof(int));
    }

    /* Copy results into flat C arrays */
    for (int s = 0; s < n_signals; ++s) {
        const auto& R = results[s];
        int used = static_cast<int>(R.params.rows());
        out->used_orders[s] = used;
        out->errors[s]      = R.error;

        for (int i = 0; i < used && i < order; ++i) {
            size_t pidx = (static_cast<size_t>(s) * order + i) * 4;
            out->params[pidx + 0] = R.params(i, 0);  /* tc     */
            out->params[pidx + 1] = R.params(i, 1);  /* fc     */
            out->params[pidx + 2] = R.params(i, 2);  /* logDt  */
            out->params[pidx + 3] = R.params(i, 3);  /* c      */

            out->coeffs[static_cast<size_t>(s) * order + i] = R.coeffs[i];
        }
    }

    return 0;
}

/* ── Result cleanup ────────────────────────────────────────────────────── */

void act_free_result(ActBatchResult* result)
{
    if (!result) return;
    free(result->params);      result->params      = nullptr;
    free(result->coeffs);      result->coeffs      = nullptr;
    free(result->errors);      result->errors      = nullptr;
    free(result->used_orders); result->used_orders = nullptr;
}
