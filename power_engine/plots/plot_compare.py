#!/usr/bin/env python3
"""Overlay one probe across several `pe run` CSVs (method/param comparison).

Usage:
  pe run --netlist buck.net --method trap --out a.csv
  pe run --netlist buck.net --method trbdf2 --out b.csv
  python3 plot_compare.py --probe v:3 a.csv b.csv [--labels trap,trbdf2]
                          [--out compare.png]

All files must share the time grid for a fair overlay; mismatched grids
abort loudly instead of silently interpolating.
"""
import argparse
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from plot_run import load  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("csvs", nargs="+", help="`pe run` output CSVs")
    ap.add_argument("--probe", required=True, help="probe column to overlay")
    ap.add_argument("--labels", default="",
                    help="comma-separated legend labels (default: file names)")
    ap.add_argument("--out", default="compare.png", help="PNG path")
    args = ap.parse_args()

    labels = [s for s in args.labels.split(",") if s] if args.labels else args.csvs
    if len(labels) != len(args.csvs):
        raise SystemExit("--labels must match the number of CSVs")

    fig, ax = plt.subplots(figsize=(8, 3.2))
    ref_t = None
    for path, label in zip(args.csvs, labels):
        header, data = load(path)
        if args.probe not in header[1:]:
            raise SystemExit(f"{path}: unknown probe {args.probe!r}")
        t, y = data[:, 0], data[:, header.index(args.probe)]
        if ref_t is None:
            ref_t = t
        elif len(t) != len(ref_t) or not np.array_equal(t, ref_t):
            raise SystemExit(f"{path}: time grid differs from {args.csvs[0]}")
        ax.plot(t, y, lw=0.8, label=label)
    ax.set_xlabel("time (s)")
    ax.set_ylabel(args.probe)
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(args.out, dpi=100)
    print(f"wrote {args.out} ({len(args.csvs)} runs overlaid)")


if __name__ == "__main__":
    sys.exit(main())
