"""
Persistent ZUNA inference subprocess.

Loads the model **once**, then loops reading JSON commands from stdin.
Each command specifies ``input_dir`` and ``output_dir`` for one epoch.
Responds with a JSON status line on stdout.

This eliminates the ~15 s per-epoch overhead of process startup, NCCL
init, HuggingFace weight loading, and ``torch.compile``.

Protocol (JSON lines over stdin/stdout)
---------------------------------------
← (startup) ``{"status":"ready"}``
→ ``{"input_dir":"/tmp/…/2_pt_input","output_dir":"/tmp/…/3_pt_output",
     "diffusion_cfg":1.0,"diffusion_sample_steps":50,"data_norm":10.0,
     "tokens_per_batch":1000}``
← ``{"status":"ok"}``  or  ``{"status":"error","msg":"…"}``
→ ``{"command":"shutdown"}``

Usage (called by ``zuna_worker.py``)::

    python _persistent_inference.py <config.yaml> [key=value overrides…]
"""

from __future__ import annotations

import gc
import json
import logging
import os
import random
import sys
import time
from contextlib import ExitStack
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional

import numpy as np
import torch
from omegaconf import OmegaConf

# HuggingFace model loading
from huggingface_hub import hf_hub_download
from safetensors.torch import load_file as safe_load

# lingua / zuna internals  (script dir must be on sys.path)
from lingua.args import dataclass_from_dict
from lingua.distributed import (
    DistributedArgs,
    EnvironmentArgs,
    get_device_mesh,
    init_signal_handler,
    setup_env,
    setup_torch_distributed,
    check_model_value_range,
)
from lingua.metrics import GPUMemoryMonitor, LoggingArgs, get_num_params

from apps.AY2latent_bci.eeg_data import (
    BCIDatasetArgs,
    EEGProcessor,
    create_dataloader_v2,
)
from apps.AY2latent_bci.transformer import (
    DecoderTransformerArgs,
    EncoderDecoder,
)

logger = logging.getLogger()

# ── TrainArgs mirrored from eeg_eval.py ─────────────────────────────────

@dataclass
class TrainArgs:
    name: str = "lingua"
    dump_dir: str = ""
    seed: int = 42
    grad_acc_steps: int = 1
    gc_collect_freq: int = 1000
    probe_freq: Optional[int] = None
    steps: int = 1000
    data: BCIDatasetArgs = field(default_factory=BCIDatasetArgs)
    model: DecoderTransformerArgs = field(default_factory=DecoderTransformerArgs)
    distributed: DistributedArgs = field(default_factory=DistributedArgs)
    env: EnvironmentArgs = field(default_factory=EnvironmentArgs)
    logging: LoggingArgs = field(default_factory=LoggingArgs)
    async_eval_gpus: Optional[int] = None
    eval: Optional[Any] = None
    load_distillation_model: bool = False
    channel_loss_weighting: bool = False
    distill_into_encoder: bool = False
    repa_into_encoder: bool = False
    repa_into_decoder: bool = False
    decoder_loss_weight: float = 1.0
    decoder_repa_weight: float = 1.0
    encoder_mmd_weight: float = 1.0
    encoder_repa_weight: float = 1.0
    encoder_distill_weight: float = 1.0
    diffusion_cfg: float = 1.0
    diffusion_sample_steps: int = 50
    plot_eeg_signal_samples: bool = False
    inference_figures_dir: str = "inference_figures"


# ── Signal handler (from eeg_eval.py) ───────────────────────────────────

preemption_flag = dict(flag=False)

def _set_preemption_flag(signum, frame):
    preemption_flag["flag"] = True


# ── File parsing / saving helpers (from eeg_eval.py) ────────────────────

def _parse_filename_num_samples(filename: str) -> int | None:
    try:
        parts = filename.removesuffix(".pt").split("_")
        return int(parts[4])
    except (IndexError, ValueError):
        return None


def _save_reconstructed_file(filename, file_data, export_dir):
    output_path = Path(export_dir) / filename
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_dict = {
        "data": file_data["data_reconstructed"],
        "data_original": file_data["data_original"],
        "channel_positions": file_data["channel_positions"],
        "metadata": file_data["metadata"],
    }
    torch.save(output_dict, output_path)


def _check_and_save_complete(accumulator, export_dir) -> list[str]:
    done = []
    for fn, fd in accumulator.items():
        if fd["collected_samples"] == fd["expected_samples"]:
            _save_reconstructed_file(fn, fd, export_dir)
            done.append(fn)
    return done


# ── Unwrap signals (simplified from eeg_eval.py) ────────────────────────

def _unwrap_signals(model_output, batch, args):
    """Unwrap variable-length packed sequences into per-sample arrays."""
    model_input = batch["encoder_input"]
    eeg_signal = batch["eeg_signal"]
    seq_lens = batch["seq_lens"].cpu().numpy()

    tf = args.data.num_fine_time_pts
    tc = args.data.seq_len // tf
    if args.data.use_coarse_time == "C":
        tc = 1

    out_sig = []
    out_orig = []
    out_pos = []
    seqlen_accum = 0

    for i, seqlen in enumerate(seq_lens):
        num_chans = seqlen // tc
        sl = slice(seqlen_accum, seqlen_accum + seqlen)

        if args.data.cat_chan_xyz_and_eeg:
            sig_in = model_input[sl, 3:]
            eeg_s = eeg_signal[sl, 3:]
            sig_out = model_output.squeeze(0)[sl, 3:]
            pos_in = model_input[sl, :3]
        else:
            sig_in = model_input[sl, :]
            eeg_s = eeg_signal[sl, :]
            sig_out = model_output.squeeze(0)[sl, :]
            pos_in = batch["chan_pos"][sl, :]

        out_sig.append(sig_out.reshape(num_chans, tc, -1).cpu().numpy())
        out_orig.append(eeg_s.reshape(num_chans, tc, -1).cpu().numpy())
        out_pos.append(pos_in.reshape(num_chans, tc, -1).cpu().numpy())
        seqlen_accum += seqlen

    return out_sig, out_orig, out_pos


# ── Core ─────────────────────────────────────────────────────────────────

def _load_model(device: torch.device):
    """Download (or use cached) ZUNA weights and build the model."""
    REPO_ID = "Zyphra/ZUNA"

    def load_model_args_from_hf(repo_id, config_filename="config.json"):
        path = hf_hub_download(repo_id=repo_id, filename=config_filename)
        with open(path) as f:
            cfig = json.load(f)
        return dataclass_from_dict(DecoderTransformerArgs, cfig["model"])

    model_args = load_model_args_from_hf(REPO_ID)
    weights_path = hf_hub_download(
        repo_id=REPO_ID,
        filename="model-00001-of-00001.safetensors",
        token=False,
    )
    sd_raw = safe_load(weights_path, device="cpu")
    sd = {k.removeprefix("model."): v for k, v in sd_raw.items()}

    model = EncoderDecoder(model_args).to(device)
    model.load_state_dict({k: v.to(device) for k, v in sd.items()}, strict=True)
    model.eval()

    if device.type == "cuda":
        model.sample = torch.compile(model.sample)
        model.encoder = torch.compile(model.encoder)

    for p in model.parameters():
        p.requires_grad = False

    check_model_value_range(model, range=10.0, std=1.0)
    return model


def _run_epoch(
    args: TrainArgs,
    model,
    data_processor,
    device: torch.device,
    input_dir: str,
    output_dir: str,
    dp_rank: int = 0,
):
    """Run inference on all .pt files in *input_dir*, write results to *output_dir*."""
    # Point dataloader at the new directories
    args.data.data_dir = input_dir
    args.data.export_dir = output_dir
    os.makedirs(output_dir, exist_ok=True)

    data_loader = create_dataloader_v2(args.data, args.seed, dp_rank)

    eeg_sig_norm = args.data.data_norm
    eeg_sig_clip = getattr(args.data, "data_clip", None)
    tf = args.data.num_fine_time_pts
    tc_val = args.data.seq_len // tf
    if args.data.use_coarse_time == "C":
        tc_val = 1

    results_accumulator: dict = {}
    epoch_counter = 0

    def make_batch_iterator():
        nonlocal epoch_counter
        while True:
            epoch_counter += 1
            for _idx, batch in enumerate(data_loader):
                eeg_signal = batch["eeg_signal"] / eeg_sig_norm
                if eeg_sig_clip is not None:
                    eeg_signal = eeg_signal.clamp(
                        min=-eeg_sig_clip, max=eeg_sig_clip
                    )
                batch["eeg_signal"] = eeg_signal
                yield batch

    batch_iter = make_batch_iterator()

    while True:
        batch = next(batch_iter)
        if epoch_counter > 1:
            break

        batch_filenames = batch.pop("filename", None)
        batch_sample_indices = batch.pop("sample_idx", None)
        batch_metadata_list = batch.pop("metadata", None)
        batch.pop("idx", None)
        batch.pop("dataset_id", None)

        with torch.no_grad():
            batch = data_processor.process(**batch)
        batch = {
            k: v.to(device, non_blocking=(device.type == "cuda"))
            for k, v in batch.items()
        }

        # Build tok_idx
        if args.model.tok_idx_type == "{x,y,z,tc}" and args.model.rope_dim == 4:
            chan_pos_d = batch["chan_pos_discrete"].cpu().unsqueeze(0)
            t_coarse = batch["t_coarse"].cpu().unsqueeze(0)
            tok_idx = torch.cat((chan_pos_d, t_coarse), dim=2)
        elif args.model.tok_idx_type is None:
            tok_idx = None
        elif args.model.tok_idx_type == "t_coarse" and args.model.rope_dim == 1:
            tok_idx = batch["t_coarse"].cpu().unsqueeze(0)
        elif args.model.tok_idx_type == "chan_id" and args.model.rope_dim == 1:
            tok_idx = batch["chan_id"].cpu().unsqueeze(0)
        elif (
            args.model.tok_idx_type == "stack_arange_seqlen"
            and args.model.rope_dim == 1
        ):
            tok_idx = (
                torch.hstack(
                    [
                        torch.arange(sl)
                        for sl in batch["seq_lens"].cpu().numpy()
                    ]
                )
                .unsqueeze(0)
                .unsqueeze(-1)
            )
        else:
            raise ValueError(
                f"Unknown tok_idx_type={args.model.tok_idx_type} "
                f"rope_dim={args.model.rope_dim}"
            )

        with torch.no_grad():
            z, _inference_at_step = model.sample(
                encoder_input=batch["encoder_input"].unsqueeze(0),
                seq_lens=batch["seq_lens"],
                tok_idx=tok_idx,
                cfg=args.diffusion_cfg,
                sample_steps=args.diffusion_sample_steps,
            )

        # Unwrap and accumulate results
        out_sig, out_orig, out_pos = _unwrap_signals(z, batch, args)

        for i in range(len(out_sig)):
            filename = batch_filenames[i]
            sample_idx = batch_sample_indices[i]
            metadata = batch_metadata_list[i]

            if filename not in results_accumulator:
                num_samples = _parse_filename_num_samples(filename)
                if num_samples is None:
                    continue
                results_accumulator[filename] = {
                    "expected_samples": num_samples,
                    "collected_samples": 0,
                    "data_original": [None] * num_samples,
                    "data_reconstructed": [None] * num_samples,
                    "channel_positions": [None] * num_samples,
                    "metadata": metadata,
                }

            entry = results_accumulator[filename]
            entry["data_original"][sample_idx] = out_orig[i] * eeg_sig_norm
            entry["data_reconstructed"][sample_idx] = out_sig[i] * eeg_sig_norm
            entry["channel_positions"][sample_idx] = out_pos[i].reshape(-1, tc_val, 3)[:, 0, :]
            entry["collected_samples"] += 1

        # Save any complete files immediately
        for fn in _check_and_save_complete(results_accumulator, output_dir):
            del results_accumulator[fn]

    # Save remaining
    for fn, fd in results_accumulator.items():
        _save_reconstructed_file(fn, fd, output_dir)


# ── Entry point ──────────────────────────────────────────────────────────

def main():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s  %(message)s",
        stream=sys.stderr,  # keep stdout clean for JSON protocol
    )

    # Parse config (same as eeg_eval.py main())
    cli_args = OmegaConf.from_cli()
    file_cfg = OmegaConf.load(cli_args.config)
    del cli_args.config
    default_cfg = OmegaConf.structured(TrainArgs())
    cfg = OmegaConf.merge(default_cfg, file_cfg, cli_args)
    args = OmegaConf.to_object(cfg)

    # ── One-time setup ──────────────────────────────────────────────
    if torch.cuda.is_available():
        device = torch.device("cuda")
    elif torch.backends.mps.is_available():
        device = torch.device("mps")
    else:
        device = torch.device("cpu")

    with ExitStack():
        init_signal_handler(_set_preemption_flag)
        setup_env(args.env)
        setup_torch_distributed(args.distributed, device=device)
        get_device_mesh(args.distributed, device=device)

    logger.info("Loading ZUNA model from HuggingFace…")
    model = _load_model(device)
    logger.info("Model loaded and compiled.")

    data_processor = EEGProcessor(args.data).to(device)

    if device.type == "cuda":
        gpu_mon = GPUMemoryMonitor("cuda")
        logger.info(
            "GPU: %s (%.2f GiB) — %s",
            gpu_mon.device_name,
            gpu_mon.device_capacity_gib,
            gpu_mon,
        )

    gc.disable()
    torch.manual_seed(args.seed)
    if device.type == "cuda":
        torch.cuda.manual_seed(args.seed)
    np.random.seed(args.seed)
    random.seed(args.seed)

    # ── Signal ready ────────────────────────────────────────────────
    print(json.dumps({"status": "ready"}), flush=True)
    logger.info("Persistent inference daemon ready — waiting for commands on stdin.")

    # ── Command loop ────────────────────────────────────────────────
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            cmd = json.loads(line)
        except json.JSONDecodeError as exc:
            print(json.dumps({"status": "error", "msg": str(exc)}), flush=True)
            continue

        if cmd.get("command") == "shutdown":
            break

        input_dir = cmd.get("input_dir")
        output_dir = cmd.get("output_dir")
        if not input_dir or not output_dir:
            print(
                json.dumps({"status": "error", "msg": "missing input_dir/output_dir"}),
                flush=True,
            )
            continue

        # Apply per-request overrides
        if "diffusion_cfg" in cmd:
            args.diffusion_cfg = cmd["diffusion_cfg"]
        if "diffusion_sample_steps" in cmd:
            args.diffusion_sample_steps = cmd["diffusion_sample_steps"]
        if "data_norm" in cmd:
            args.data.data_norm = cmd["data_norm"]
        if "tokens_per_batch" in cmd:
            args.data.target_packed_seqlen = cmd["tokens_per_batch"]

        t0 = time.monotonic()
        try:
            _run_epoch(args, model, data_processor, device, input_dir, output_dir)
            elapsed = time.monotonic() - t0
            print(
                json.dumps({"status": "ok", "elapsed": round(elapsed, 2)}),
                flush=True,
            )
        except Exception as exc:
            logger.exception("Inference epoch failed")
            elapsed = time.monotonic() - t0
            print(
                json.dumps(
                    {"status": "error", "msg": str(exc), "elapsed": round(elapsed, 2)}
                ),
                flush=True,
            )

    # ── Cleanup ─────────────────────────────────────────────────────
    logger.info("Shutting down persistent inference daemon.")
    if torch.distributed.is_initialized():
        torch.distributed.destroy_process_group()


if __name__ == "__main__":
    main()
