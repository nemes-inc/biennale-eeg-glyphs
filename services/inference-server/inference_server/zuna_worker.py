"""
ZUNA GPU inference worker.

Runs in a dedicated thread/process.  Receives epochs from an asyncio queue,
batches them, runs the ZUNA pipeline (preprocess -> inference -> reconstruct),
and pushes reconstructed frames into an output queue.

Since ZUNA's public API is file-based, we use a temp-directory shim:
  1. Write epoch -> temp .fif (MNE RawArray)
  2. Call ``zuna.preprocessing()`` -> .pt
  3. Inference via persistent subprocess -> .pt
  4. Call ``zuna.pt_to_fif()`` -> .fif
  5. Read reconstructed .fif -> extract samples -> return

The inference step uses a **persistent subprocess** that keeps the model
loaded in GPU memory between epochs, eliminating ~15 s of startup overhead
per epoch (NCCL init, HuggingFace weight load, torch.compile).
"""

from __future__ import annotations

import json
import logging
import os
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
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

# Serialize inference() calls so workers don't race on GPU.
_inference_lock = threading.Lock()

# Module-level persistent inference subprocess (shared by all workers).
_persistent_proc: subprocess.Popen | None = None
_persistent_ready = False


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
        """Import zuna and start the persistent inference subprocess (once)."""
        log.info("Loading ZUNA model (weights download on first run) …")
        import zuna  # noqa: F401

        # Start the persistent subprocess exactly once (first worker wins).
        _ensure_persistent_subprocess(
            gpu_device=self.gpu_device,
            diffusion_steps=self.diffusion_steps,
            tokens_per_batch=self.tokens_per_batch,
        )

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
        import zuna
        from zuna import preprocessing, pt_to_fif

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
        # Guard: skip if preprocessing produced no .pt files
        pt_files = list(pt_in.glob("*.pt"))
        if not pt_files:
            log.warning(
                "Preprocessing produced 0 .pt files for headband=%d seq=%d — skipping inference",
                job.headband_id,
                epoch.seq,
            )
            return None

        # Serialize: workers share a single persistent GPU subprocess.
        with _inference_lock:
            _send_inference_request(
                input_dir=str(pt_in.absolute()),
                output_dir=str(pt_out.absolute()),
                diffusion_steps=self.diffusion_steps,
                tokens_per_batch=self.tokens_per_batch,
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


# ── Persistent inference subprocess management ──────────────────────────


def _ensure_persistent_subprocess(
    gpu_device: int | str = 0,
    diffusion_steps: int = 50,
    tokens_per_batch: int = 1000,
) -> None:
    """Start the persistent inference subprocess if it isn't running yet."""
    global _persistent_proc, _persistent_ready

    with _inference_lock:
        if _persistent_proc is not None and _persistent_proc.poll() is None:
            # Already running
            return

        import zuna
        zuna_root = Path(zuna.__file__).parent
        config_path = (
            zuna_root
            / "inference/AY2l/lingua/apps/AY2latent_bci/configs/config_infer.yaml"
        )
        script_dir = str(
            zuna_root / "inference/AY2l/lingua/apps/AY2latent_bci"
        )
        shim_script = str(Path(__file__).parent / "_inference_shim.py")
        persistent_script = str(Path(__file__).parent / "_persistent_inference.py")

        cmd = [
            sys.executable,
            shim_script,
            persistent_script,
            f"config={config_path}",
            f"diffusion_sample_steps={diffusion_steps}",
            f"data.target_packed_seqlen={tokens_per_batch}",
            f"data.data_norm=10.0",
            f"plot_eeg_signal_samples=False",
        ]

        env = os.environ.copy()
        env["CUDA_VISIBLE_DEVICES"] = str(gpu_device)
        env["MASTER_ADDR"] = "127.0.0.1"
        env["_INFERENCE_MASTER_PORT"] = str(_pick_free_port())
        # Ensure the eeg_eval sibling modules are importable
        env["PYTHONPATH"] = script_dir + os.pathsep + env.get("PYTHONPATH", "")

        log.info("Starting persistent inference subprocess …")
        _persistent_proc = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=None,  # inherit parent stderr so logs are visible
            env=env,
            text=True,
            bufsize=1,  # line-buffered
        )

        # Wait for the "ready" message (model loaded)
        log.info("Waiting for persistent inference subprocess to load model …")
        ready_line = _persistent_proc.stdout.readline()
        if not ready_line:
            rc = _persistent_proc.wait()
            raise RuntimeError(
                f"Persistent inference subprocess exited immediately (rc={rc})"
            )
        msg = json.loads(ready_line)
        if msg.get("status") != "ready":
            raise RuntimeError(
                f"Unexpected first message from persistent subprocess: {msg}"
            )
        _persistent_ready = True
        log.info("Persistent inference subprocess ready.")


def _send_inference_request(
    input_dir: str,
    output_dir: str,
    diffusion_steps: int = 50,
    tokens_per_batch: int = 1000,
) -> None:
    """Send one inference request to the persistent subprocess.

    Must be called while holding ``_inference_lock``.
    """
    global _persistent_proc, _persistent_ready

    if _persistent_proc is None or _persistent_proc.poll() is not None:
        raise RuntimeError("Persistent inference subprocess is not running")

    request = json.dumps({
        "input_dir": input_dir,
        "output_dir": output_dir,
        "diffusion_sample_steps": diffusion_steps,
        "tokens_per_batch": tokens_per_batch,
    })
    _persistent_proc.stdin.write(request + "\n")
    _persistent_proc.stdin.flush()

    # Block until the subprocess replies
    response_line = _persistent_proc.stdout.readline()
    if not response_line:
        rc = _persistent_proc.poll()
        _persistent_ready = False
        raise RuntimeError(
            f"Persistent inference subprocess died (rc={rc})"
        )

    resp = json.loads(response_line)
    if resp.get("status") == "ok":
        log.info(
            "Inference completed in %.1fs (persistent subprocess)",
            resp.get("elapsed", 0),
        )
    else:
        raise RuntimeError(
            f"Persistent inference failed: {resp.get('msg', resp)}"
        )


def shutdown_persistent_subprocess() -> None:
    """Gracefully shut down the persistent inference subprocess."""
    global _persistent_proc, _persistent_ready

    with _inference_lock:
        if _persistent_proc is None:
            return
        if _persistent_proc.poll() is not None:
            _persistent_proc = None
            _persistent_ready = False
            return

        try:
            _persistent_proc.stdin.write(
                json.dumps({"command": "shutdown"}) + "\n"
            )
            _persistent_proc.stdin.flush()
            _persistent_proc.wait(timeout=30)
        except Exception:
            log.warning("Killing persistent inference subprocess")
            _persistent_proc.kill()
        finally:
            _persistent_proc = None
            _persistent_ready = False
