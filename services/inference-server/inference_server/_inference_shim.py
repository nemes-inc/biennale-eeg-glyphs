"""
Shim for eeg_eval.py — patches torch.distributed.init_process_group
to use the caller-provided MASTER_PORT instead of the hardcoded 29481
in lingua.distributed.setup_torch_distributed.

Usage:  python3 _inference_shim.py <path/to/eeg_eval.py> [config args...]

The caller sets _INFERENCE_MASTER_PORT in the environment; this shim
restores it into MASTER_PORT right before NCCL reads it.
"""

import os
import sys

import torch.distributed as _td

_orig_init_process_group = _td.init_process_group


def _patched_init_process_group(*args, **kwargs):
    """Restore MASTER_PORT from _INFERENCE_MASTER_PORT before NCCL init."""
    port = os.environ.get("_INFERENCE_MASTER_PORT")
    if port:
        os.environ["MASTER_PORT"] = port
    return _orig_init_process_group(*args, **kwargs)


_td.init_process_group = _patched_init_process_group

if __name__ == "__main__":
    script = sys.argv[1]
    sys.argv = sys.argv[1:]

    # Ensure the script's directory is on sys.path so sibling imports
    # (e.g. ``from utils_pt_mne import ...``) work the same way they
    # do when Python runs the script directly.
    script_dir = os.path.dirname(os.path.abspath(script))
    if script_dir not in sys.path:
        sys.path.insert(0, script_dir)

    import runpy

    runpy.run_path(script, run_name="__main__")
