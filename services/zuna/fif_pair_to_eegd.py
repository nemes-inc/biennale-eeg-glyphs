#!/usr/bin/env python3
"""Emit EEGD frames to stdout for eeg-viewer: preprocessed FIF vs reconstructed FIF.

First line: NAMES:ch1,ch2,... (ASCII)
Then binary chunks: magic EEGD, n_ch, n_samples, orig f32 channel-major, recon f32 channel-major.
"""

from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

import mne
import numpy as np

MAGIC_DUAL = 0x4545_4744  # EEGD
CHUNK = 512


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--original",
        type=Path,
        required=True,
        help="Preprocessed FIF (e.g. 1_fif_filter/…)",
    )
    p.add_argument(
        "--reconstructed",
        type=Path,
        required=True,
        help="Reconstructed FIF (e.g. 4_fif_output/…)",
    )
    p.add_argument(
        "--max-points",
        type=int,
        default=0,
        help="Cap samples per channel (0 = all)",
    )
    args = p.parse_args()

    if not args.original.is_file():
        print(f"Not found: {args.original}", file=sys.stderr)
        sys.exit(1)
    if not args.reconstructed.is_file():
        print(f"Not found: {args.reconstructed}", file=sys.stderr)
        sys.exit(1)

    orig = mne.io.read_raw_fif(args.original, preload=True, verbose=False)
    recon = mne.io.read_raw_fif(args.reconstructed, preload=True, verbose=False)

    # Preprocessed FIF defines the channel set (e.g. 8ch / 10ch 10–20 targets from run_fif_pipeline --preset).
    # Reconstructed FIF from pt_to_fif often only carries a subset (e.g. Muse TP9/AF7/AF8/TP10). Intersecting names
    # would drop every channel missing from recon — wrong for “8ch preset” UX. Align recon to orig order; zeros
    # where recon has no row for that name (stderr note).
    names = list(orig.ch_names)
    if not names:
        print("Preprocessed FIF has no channels", file=sys.stderr)
        sys.exit(1)

    o = orig.get_data()
    rd = recon.get_data()
    recon_idx = {ch: i for i, ch in enumerate(recon.ch_names)}
    n_s = min(o.shape[1], rd.shape[1])
    if args.max_points > 0:
        n_s = min(n_s, args.max_points)
    if n_s < 1:
        print(
            "No overlapping samples in preprocessed vs reconstructed FIF (n_s=0)",
            file=sys.stderr,
        )
        sys.exit(1)

    n_ch = int(o.shape[0])
    r = np.zeros((n_ch, n_s), dtype=np.float64)
    missing_recon: list[str] = []
    for i, ch in enumerate(names):
        if ch in recon_idx:
            r[i, :] = rd[recon_idx[ch], :n_s]
        else:
            missing_recon.append(ch)
    if missing_recon:
        print(
            "fif_pair_to_eegd: reconstructed FIF has no data for "
            f"{len(missing_recon)} preprocessed channel(s); "
            f"filled reconstructed trace with zeros: {', '.join(missing_recon)}",
            file=sys.stderr,
        )

    o = np.ascontiguousarray(o[:, :n_s], dtype=np.float32)
    r = np.ascontiguousarray(r[:, :n_s], dtype=np.float32)

    sys.stdout.write("NAMES:" + ",".join(names) + "\n")
    sys.stdout.flush()

    for start in range(0, n_s, CHUNK):
        end = min(start + CHUNK, n_s)
        chunk = end - start
        flat_o = o[:, start:end].reshape(-1, order="C")
        flat_r = r[:, start:end].reshape(-1, order="C")
        hdr = struct.pack("<III", MAGIC_DUAL, n_ch, chunk)
        sys.stdout.buffer.write(hdr)
        sys.stdout.buffer.write(flat_o.tobytes())
        sys.stdout.buffer.write(flat_r.tobytes())
    sys.stdout.buffer.flush()


if __name__ == "__main__":
    main()
