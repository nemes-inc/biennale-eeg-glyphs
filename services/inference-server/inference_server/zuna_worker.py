"""
ZUNA GPU inference worker.

Runs in a dedicated thread/process.  Receives epochs from an asyncio queue,
batches them, runs the ZUNA pipeline (preprocess → inference → reconstruct),
and pushes reconstructed frames into an output queue.

Since ZUNA's public API is file-based, we use a temp-directory shim:
  1. Write epoch → temp .fif (MNE RawArray)
  2. Call ``zuna.preprocessing()`` → .pt
  3. Call ``zuna.inference()`` → .pt
  4. Call ``zuna.pt_to_fif()`` → .fif
  5. Read reconstructed .fif → extract samples → return

This adds disk I/O overhead but is the only path available with the public API.
A future optimisation could call the model's forward pass directly.
"""

from __future__ import annotations

import logging
import os
import shutil
import socket
import tempfile
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .epoch_buffer import Epoch

log = logging.getLogger(__name__)

# ZUNA constants (must match the pretrained model)
SAMPLE_RATE = 256
EPOCH_SECONDS = 5

# Muse 4-channel names matching standard_1020
MUSE_CHANNEL_NAMES = ["TP9", "AF7", "AF8", "TP10"]


def _pick_free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("", 0))
        return int(s.getsockname()[1])


@dataclass
class InferenceResult:
    """Reconstructed epoch returned by the worker."""

    headband_id: int
    epoch_seq: int
    n_channels: int
    n_samples: int
    channels: list[list[float]]  # [n_channels][n_samples]


@dataclass
class InferenceJob:
    """A single epoch to process."""

    headband_id: int
    epoch: Epoch


class ZunaWorker:
    """Manages ZUNA model lifecycle and epoch-level inference.

    Call ``load_model()`` once at startup, then ``process_epoch()`` for each
    incoming epoch.  All methods are synchronous (meant for a worker thread).
    """

    def __init__(
        self,
        gpu_device: int | str = 0,
        tokens_per_batch: int = 1000,
        diffusion_steps: int = 50,
        channel_names: list[str] | None = None,
        target_channels: list[str] | None = None,
    ) -> None:
        self.gpu_device = gpu_device
        self.tokens_per_batch = tokens_per_batch
        self.diffusion_steps = diffusion_steps
        self.channel_names = channel_names or MUSE_CHANNEL_NAMES
        self.target_channels = target_channels
        self._model_loaded = False

    def load_model(self) -> None:
        """Import zuna and trigger weight download (first run)."""
        log.info("Loading ZUNA model (weights download on first run) …")
        # Import triggers model weight download via HuggingFace
        import zuna  # noqa: F401

        self._model_loaded = True
        log.info("ZUNA model ready.")

    def process_epoch(self, job: InferenceJob) -> InferenceResult | None:
        """Run full ZUNA pipeline on a single epoch.

        Returns ``InferenceResult`` on success, ``None`` on failure.
        """
        if not self._model_loaded:
            log.error("Model not loaded — call load_model() first")
            return None

        import mne

        mne.set_log_level("ERROR")

        tmpdir = Path(tempfile.mkdtemp(prefix="zuna_epoch_"))
        try:
            return self._run_pipeline(job, tmpdir)
        except Exception:
            log.exception(
                "ZUNA pipeline failed for headband=%d seq=%d",
                job.headband_id,
                job.epoch.seq,
            )
            return None
        finally:
            shutil.rmtree(tmpdir, ignore_errors=True)

    def _run_pipeline(
        self, job: InferenceJob, work: Path
    ) -> InferenceResult | None:
        import mne
        from zuna import inference, preprocessing, pt_to_fif

        epoch = job.epoch
        n_ch = epoch.n_channels
        n_samples = epoch.n_samples

        # ── 1. Write raw epoch → temp .fif ────────────────────────────────
        input_dir = work / "0_fif_input"
        pre_fif = work / "1_fif_filter"
        pt_in = work / "2_pt_input"
        pt_out = work / "3_pt_output"
        fif_out = work / "4_fif_output"
        for d in (input_dir, pre_fif, pt_in, pt_out, fif_out):
            d.mkdir(parents=True, exist_ok=True)

        ch_names = self.channel_names[:n_ch]
        data = np.array(epoch.channels, dtype=np.float64)  # (n_ch, n_samples)
        # Convert µV → V for MNE (Muse outputs µV)
        data *= 1e-6
        info = mne.create_info(ch_names, sfreq=SAMPLE_RATE, ch_types="eeg")
        raw = mne.io.RawArray(data, info, verbose=False)
        montage = mne.channels.make_standard_montage("standard_1020")
        raw.set_montage(montage, match_case=False, on_missing="ignore")

        fif_path = input_dir / f"epoch_{job.headband_id}_{epoch.seq}.fif"
        raw.save(str(fif_path), overwrite=True, verbose=False)

        # ── 2. Preprocess ─────────────────────────────────────────────────
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
            target_channel_count=self.target_channels,
            bad_channels=None,
        )

        # ── 3. Inference ──────────────────────────────────────────────────
        # Avoid torch.distributed port collisions
        os.environ.setdefault("MASTER_ADDR", "127.0.0.1")
        os.environ["MASTER_PORT"] = str(_pick_free_port())

        inference(
            input_dir=str(pt_in),
            output_dir=str(pt_out),
            gpu_device=self.gpu_device,
            tokens_per_batch=self.tokens_per_batch,
            data_norm=10.0,
            diffusion_cfg=1.0,
            diffusion_sample_steps=self.diffusion_steps,
            plot_eeg_signal_samples=False,
            inference_figures_dir=str(work / "FIGURES"),
        )

        # ── 4. Reconstruct .pt → .fif ────────────────────────────────────
        pt_to_fif(input_dir=str(pt_out), output_dir=str(fif_out))

        # ── 5. Read reconstructed .fif ────────────────────────────────────
        recon_fifs = sorted(fif_out.glob("*.fif")) + sorted(fif_out.glob("*.fiff"))
        if not recon_fifs:
            log.warning("No reconstructed .fif found for seq=%d", epoch.seq)
            return None

        recon_raw = mne.io.read_raw_fif(str(recon_fifs[0]), preload=True, verbose=False)
        recon_data = recon_raw.get_data()  # (n_ch_out, n_samples_out) in V
        # Convert V → µV for transmission
        recon_data *= 1e6

        out_channels = [recon_data[i].tolist() for i in range(recon_data.shape[0])]
        out_n_samples = recon_data.shape[1]

        return InferenceResult(
            headband_id=job.headband_id,
            epoch_seq=epoch.seq,
            n_channels=recon_data.shape[0],
            n_samples=out_n_samples,
            channels=out_channels,
        )
