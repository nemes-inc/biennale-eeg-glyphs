#!/usr/bin/env python3
"""
Run the full ZUNA pipeline on one or more .fif files: preprocess → inference →
reconstruct FIF → before/after comparison figures.

ZUNA skips files without a 3D channel montage. This script can apply
``standard_1020`` to channels that exist in the file when ``--ensure-montage``
is set (default: on), which covers typical Muse exports (TP9, AF7, AF8, TP10).

**Upsampling to 8–10 “10–20” channels (Muse 4 → dense layout)**  
The public ``zuna`` package does **not** expose ``ZunaModel.from_pretrained()`` or
``model.upsample(raw, ...)``. Instead, ``preprocessing(..., target_channel_count=[names])``
adds requested channels from ``standard_1005`` (zeros where absent) so the diffusion
model can fill them — same mechanism as the tutorial notebook’s
``TARGET_CHANNEL_COUNT = ['AF3', ...]`` list.

Usage (from ``services/zuna`` with ``uv sync`` already done)::

    uv run python run_fif_pipeline.py --input ./my_fifs --work-dir ./run1

    # Ten 10–20 sites (Fz, Cz, Pz, F3, F4, C3, C4, P3, P4, Oz):
    uv run python run_fif_pipeline.py --input ./my_fifs --work-dir ./run1 --preset 10ch

See also: ``running_zuna.ipynb`` for the same pipeline in notebook form.
"""

from __future__ import annotations

import argparse
import os
import shutil
import socket
from pathlib import Path

import mne
import numpy as np

# Help MPS on macOS when an op falls back (inference runs in a subprocess too).
os.environ.setdefault("PYTORCH_ENABLE_MPS_FALLBACK", "1")
# Inductor CPU codegen often fails on macOS (CppCompileError: at::vec / clang++). ZUNA inference
# uses dynamo; disabling compile avoids that path (slightly slower, same numerics).
os.environ.setdefault("TORCH_COMPILE_DISABLE", "1")

# Presets: names must exist in MNE ``standard_1005`` (used internally for positions).
TARGET_10_20_10CH = [
    "Fz",
    "Cz",
    "Pz",
    "F3",
    "F4",
    "C3",
    "C4",
    "P3",
    "P4",
    "Oz",
]
TARGET_10_20_8CH = ["Fz", "Cz", "Pz", "F3", "F4", "C3", "C4", "Oz"]


def _pick_free_tcp_port() -> int:
    """Bind port 0 and return it — avoids torch.distributed EADDRINUSE on a fixed default port."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("", 0))
        return int(s.getsockname()[1])


def _upsample_raw_like_zuna(
    raw: mne.io.BaseRaw,
    target_channel_names: list[str],
    *,
    montage_source: str = "standard_1005",
) -> mne.io.BaseRaw:
    """Append missing preset channels as zeros (same idea as ``add_specific_channels`` on epochs)."""
    montage = mne.channels.make_standard_montage(montage_source)
    montage_pos = montage.get_positions()["ch_pos"]
    channel_names = list(raw.ch_names)
    by_lower_cur = {n.lower(): n for n in channel_names}
    by_lower_montage = {n.lower(): n for n in montage_pos.keys()}

    new_names: list[str] = []
    for t in target_channel_names:
        tl = t.lower()
        if tl in by_lower_cur:
            continue
        if tl not in by_lower_montage:
            continue
        canon = by_lower_montage[tl]
        pos = montage_pos[canon]
        pos_a = np.array([pos[0], pos[1], pos[2]], dtype=np.float64)
        if np.allclose(pos_a, [0.0, 0.0, 0.0]):
            continue
        new_names.append(canon)

    if not new_names:
        return raw

    data = raw.get_data()
    n_times = int(data.shape[1])
    pad = np.zeros((len(new_names), n_times), dtype=np.float64)
    combined = np.vstack([data.astype(np.float64, copy=False), pad])
    names_out = channel_names + new_names

    info = mne.create_info(names_out, sfreq=float(raw.info["sfreq"]), ch_types="eeg", verbose=False)
    out = mne.io.RawArray(combined, info, verbose=False)
    out.set_montage(montage, match_case=False, on_missing="ignore")
    return out


def _upsample_preprocessed_fif_dir(pre_fif_dir: Path, target_channel_names: list[str]) -> int:
    """Rewrite ``1_fif_filter/*.fif`` in place so channel count matches ZUNA epochs. Returns count updated."""
    mne.set_log_level("ERROR")
    pre_fif_dir = Path(pre_fif_dir)
    paths = sorted(pre_fif_dir.glob("*.fif")) + sorted(pre_fif_dir.glob("*.fiff"))
    n = 0
    for p in paths:
        raw = mne.io.read_raw_fif(p, preload=True, verbose=False)
        out = _upsample_raw_like_zuna(raw, target_channel_names)
        if len(out.ch_names) == len(raw.ch_names):
            continue
        out.save(str(p), overwrite=True, verbose=False)
        n += 1
    return n


def _collect_fifs(path: Path) -> list[Path]:
    if path.is_file():
        if path.suffix.lower() not in (".fif", ".fiff"):
            raise SystemExit(f"Not a FIF file: {path}")
        return [path]
    if not path.is_dir():
        raise SystemExit(f"Not a file or directory: {path}")
    fifs = sorted(path.glob("*.fif")) + sorted(path.glob("*.fiff"))
    if not fifs:
        raise SystemExit(f"No .fif / .fiff files in {path}")
    return fifs


def ensure_montage(fif_path: Path, montage_name: str = "standard_1020") -> None:
    """If the file has no montage, set one so ZUNA preprocessing accepts it."""
    raw = mne.io.read_raw_fif(fif_path, preload=True, verbose=False)
    if raw.get_montage() is not None:
        return
    montage = mne.channels.make_standard_montage(montage_name)
    raw.set_montage(montage, match_case=False, on_missing="ignore")
    raw.save(fif_path, overwrite=True, verbose=False)


def stage_inputs(
    sources: list[Path],
    dest_dir: Path,
    *,
    ensure_montage_flag: bool,
    montage_name: str,
) -> None:
    dest_dir.mkdir(parents=True, exist_ok=True)
    for src in sources:
        dst = dest_dir / src.name
        shutil.copy2(src, dst)
        if ensure_montage_flag:
            ensure_montage(dst, montage_name)


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument(
        "--input",
        type=Path,
        required=True,
        help="Directory containing .fif files, or a single .fif path",
    )
    p.add_argument(
        "--work-dir",
        type=Path,
        default=Path("zuna_pipeline_run"),
        help="Working directory for pipeline stages (default: ./zuna_pipeline_run)",
    )
    p.add_argument(
        "--ensure-montage",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Apply standard_1020 montage when missing (needed for ZUNA). Default: true",
    )
    p.add_argument(
        "--montage",
        default="standard_1020",
        help="Montage name for --ensure-montage (default: standard_1020)",
    )
    p.add_argument(
        "--gpu-device",
        default="0",
        help='GPU id for inference subprocess (CUDA_VISIBLE_DEVICES), or empty string for CPU',
    )
    p.add_argument("--tokens-per-batch", type=int, default=1000)
    p.add_argument("--diffusion-steps", type=int, default=50)
    p.add_argument("--num-samples", type=int, default=2, help="Files to include in comparison plots")
    p.add_argument(
        "--plot-pt",
        action="store_true",
        help="Also generate .pt tensor comparisons (slower)",
    )
    p.add_argument(
        "--keep-intermediate",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Keep intermediate .pt dirs (default: true)",
    )
    p.add_argument(
        "--preset",
        choices=("none", "8ch", "10ch"),
        default="none",
        help="Upsample toward 8 or 10 standard_1005 channel names (Muse 4ch + padded targets). "
        "Ignored if --target-channels is set.",
    )
    p.add_argument(
        "--target-channels",
        metavar="NAMES",
        default=None,
        help="Comma-separated 10–20 names, e.g. Fz,Cz,Pz,... (overrides --preset). "
        "Passed to zuna.preprocessing as target_channel_count.",
    )
    p.add_argument(
        "--headless",
        action="store_true",
        help="No matplotlib windows: use Agg backend, skip comparison PNG step (eeg-viewer loads FIFs).",
    )
    args = p.parse_args()

    if args.headless:
        # Before `import zuna` (matplotlib) — avoids interactive windows during inference / plots.
        os.environ["MPLBACKEND"] = "Agg"

    if args.target_channels:
        target_channel_count = [x.strip() for x in args.target_channels.split(",") if x.strip()]
        if not target_channel_count:
            raise SystemExit("--target-channels produced an empty list")
    elif args.preset == "8ch":
        target_channel_count = TARGET_10_20_8CH
    elif args.preset == "10ch":
        target_channel_count = TARGET_10_20_10CH
    else:
        target_channel_count = None

    sources = _collect_fifs(args.input.resolve())
    work = args.work_dir.resolve()
    input_dir = work / "1_fif_input"
    pre_fif = work / "1_fif_filter"
    pt_in = work / "2_pt_input"
    pt_out = work / "3_pt_output"
    fif_out = work / "4_fif_output"
    figures = work / "FIGURES"

    if work.exists() and any(work.iterdir()):
        print(f"Using existing work dir (stages may merge with old data): {work}")

    for d in (input_dir, pre_fif, pt_in, pt_out, fif_out, figures):
        d.mkdir(parents=True, exist_ok=True)

    print("[0/4] Staging inputs and montage …", flush=True)
    stage_inputs(
        sources,
        input_dir,
        ensure_montage_flag=args.ensure_montage,
        montage_name=args.montage,
    )

    from zuna import compare_plot_pipeline, inference, preprocessing, pt_to_fif

    print("[1/4] Preprocessing (.fif → .pt) …", flush=True)
    if target_channel_count is not None:
        print(
            "  target_channel_count (",
            len(target_channel_count),
            "):",
            ", ".join(target_channel_count),
            flush=True,
        )
    else:
        print("  (no extra targets — same channel set as input)", flush=True)
    preprocessing(
        input_dir=str(input_dir),
        output_dir=str(pt_in),
        apply_notch_filter=False,
        apply_highpass_filter=True,
        apply_average_reference=True,
        preprocessed_fif_dir=str(pre_fif),
        drop_bad_channels=False,
        drop_bad_epochs=False,
        zero_out_artifacts=False,
        target_channel_count=target_channel_count,
        bad_channels=None,
    )

    # zuna saves 1_fif_filter/*.fif *before* epoch upsampling; PT/inference use the full channel
    # list. Rewrite FIFs so eeg-viewer compares the same names as 4_fif_output.
    if isinstance(target_channel_count, list) and target_channel_count:
        print(
            "[1b/4] Upsampling preprocessed FIFs for viewer (match ZUNA channel list) …",
            flush=True,
        )
        n = _upsample_preprocessed_fif_dir(pre_fif, target_channel_count)
        print(f"  Updated {n} preprocessed FIF file(s) under {pre_fif}", flush=True)

    gs = str(args.gpu_device).strip()
    gpu_arg: int | str = "" if gs == "" else int(gs)

    # ZUNA inference uses torch.distributed (gloo); default MASTER_PORT can stay busy after a crash
    # or overlap another run → DistNetworkError EADDRINUSE. Pick a fresh port each invocation.
    os.environ.setdefault("MASTER_ADDR", "127.0.0.1")
    os.environ["MASTER_PORT"] = str(_pick_free_tcp_port())

    print("[2/4] Inference …", flush=True)
    inference(
        input_dir=str(pt_in),
        output_dir=str(pt_out),
        gpu_device=gpu_arg,
        tokens_per_batch=args.tokens_per_batch,
        data_norm=10.0,
        diffusion_cfg=1.0,
        diffusion_sample_steps=args.diffusion_steps,
        plot_eeg_signal_samples=False,
        inference_figures_dir=str(figures),
    )

    print("[3/4] Reconstruct FIF …", flush=True)
    pt_to_fif(input_dir=str(pt_out), output_dir=str(fif_out))

    if args.headless:
        print(
            "[4/4] Comparison figures … (skipped — headless; view traces in eeg-viewer)",
            flush=True,
        )
    else:
        print("[4/4] Comparison figures …", flush=True)
        compare_plot_pipeline(
            input_dir=str(input_dir),
            fif_input_dir=str(pre_fif),
            fif_output_dir=str(fif_out),
            pt_input_dir=str(pt_in),
            pt_output_dir=str(pt_out),
            output_dir=str(figures),
            plot_pt=args.plot_pt,
            plot_fif=True,
            num_samples=args.num_samples,
        )

    if not args.keep_intermediate:
        shutil.rmtree(pt_in, ignore_errors=True)
        shutil.rmtree(pt_out, ignore_errors=True)

    print("Done.")
    print("  Reconstructed FIF:", fif_out)
    print("  Figures:", figures)
    for png in sorted(figures.glob("*.png")):
        print("   ", png.name)


if __name__ == "__main__":
    main()
