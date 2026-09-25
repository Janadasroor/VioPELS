#!/usr/bin/env python3
"""Time-domain traces from `pe run` CSV.

Usage:
  pe run --netlist buck.net --out run.csv
  python3 plot_run.py run.csv [--probes v:3,i:L1] [--out run.png] [--title T]

CSV contract: first column is `time`, remaining columns are probes in
sorted order (see `pe run --help`). One stacked panel per probe, shared
time axis. Headless-safe (Agg backend).
"""
import argparse
import csv
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


def load(csv_path):
    with open(csv_path, newline="") as f:
        rows = list(csv.reader(f))
    if len(rows) < 2:
        raise SystemExit(f"{csv_path}: need header + at least one data row")
    header, data = rows[0], np.array(rows[1:], dtype=float)
    if header[0] != "time":
        raise SystemExit(f"{csv_path}: first column must be 'time', got {header[0]!r}")
    return header, data


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("csv", help="`pe run` output CSV")
    ap.add_argument("--probes", default="",
                    help="comma-separated subset (default: all probes)")
    ap.add_argument("--out", default="", help="PNG path (default: <csv>.png)")
    ap.add_argument("--title", default="", help="figure title")
    args = ap.parse_args()

    header, data = load(args.csv)
    t = data[:, 0]
    want = [p for p in args.probes.split(",") if p] if args.probes else header[1:]
    for p in want:
        if p not in header[1:]:
            raise SystemExit(f"{args.csv}: unknown probe {p!r} (have {header[1:]})")

    fig, axes = plt.subplots(len(want), 1, sharex=True, squeeze=False,
                             figsize=(8, 2.2 * len(want)))
    for ax, p in zip(axes[:, 0], want):
        ax.plot(t, data[:, header.index(p)], lw=0.8)
        ax.set_ylabel(p)
        ax.grid(True, alpha=0.3)
    axes[-1, 0].set_xlabel("time (s)")
    if args.title:
        fig.suptitle(args.title)
    fig.tight_layout()
    out = args.out or (args.csv + ".png")
    fig.savefig(out, dpi=100)
    print(f"wrote {out} ({len(t)} points, probes: {', '.join(want)})")


if __name__ == "__main__":
    sys.exit(main())
