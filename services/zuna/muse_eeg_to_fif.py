#!/usr/bin/env python3
"""
Convert aligned Muse EEG CSV (µV) to a continuous EEG FIF for MNE / ZUNA.

CSV format (written by `muse-record-fif`):
  - Header: TP9,AF7,AF8,TP10
  - Rows: one sample instant across 4 channels (aligned by min length).

Requires: mne, numpy  (e.g. `uv add mne numpy` or use `services/zuna` env).
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

try:
    import numpy as np
except ModuleNotFoundError as e:
    if e.name == "numpy":
        print(
            "Missing dependency: numpy (and usually mne). Install with:\n"
            "  cd services/zuna && uv sync\n"
            "Or use your ZUNA venv and pass it to muse-record-fif:\n"
            "  --python /path/to/biennale-eeg-glyphs/services/zuna/.venv/bin/python",
            file=sys.stderr,
        )
        sys.exit(1)
    raise


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--csv", type=Path, required=True, help="Aligned 4-channel CSV (µV)")
    p.add_argument("-o", "--output", type=Path, required=True, help="Output .fif path")
    p.add_argument("--sfreq", type=float, default=256.0, help="EEG sampling rate (Hz)")
    args = p.parse_args()

    import mne

    data = np.loadtxt(args.csv, delimiter=",", skiprows=1)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[1] != 4:
        raise SystemExit(f"Expected 4 columns, got {data.shape[1]}")

    # (n_times, n_ch) -> (n_ch, n_times) in volts
    data = data.T.astype(np.float64) * 1e-6
    ch_names = ["TP9", "AF7", "AF8", "TP10"]
    info = mne.create_info(ch_names=ch_names, sfreq=args.sfreq, ch_types="eeg")
    raw = mne.io.RawArray(data, info, verbose=False)
    raw.save(args.output, overwrite=True, verbose=False)
    print(f"Wrote {args.output.resolve()}")


if __name__ == "__main__":
    main()
