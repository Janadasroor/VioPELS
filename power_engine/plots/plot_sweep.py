#!/usr/bin/env python3
"""Measure-vs-parameter curve from a `pe sweep` table CSV.

Usage:
  pe sweep --netlist buck.net --axis R=1,2,5,10 \\
           --measure 'mean(v:3) as vout' --out sweep.csv
  python3 plot_sweep.py sweep.csv --x R --y vout [--out sweep.png]

Table contract: param columns, then measure columns, then `error`
(see `pe sweep --help`). Errored rows are skipped with a stderr warning,
never silently plotted.
"""
import argparse
import csv
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("csv", help="`pe sweep` output table CSV")
    ap.add_argument("--x", required=True, help="parameter column")
    ap.add_argument("--y", required=True, help="measure column")
    ap.add_argument("--out", default="", help="PNG path (default: <csv>.png)")
    args = ap.parse_args()

    with open(args.csv, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit(f"{args.csv}: no data rows")
    for col in (args.x, args.y):
        if col not in rows[0]:
            raise SystemExit(f"{args.csv}: unknown column {col!r} (have {list(rows[0])})")

    pts, skipped = [], 0
    for r in rows:
        if r.get("error"):
            skipped += 1
            continue
        pts.append((float(r[args.x]), float(r[args.y])))
    if skipped:
        print(f"warning: skipped {skipped} errored rows", file=sys.stderr)
    if not pts:
        raise SystemExit(f"{args.csv}: no successful rows to plot")
    pts.sort()

    fig, ax = plt.subplots(figsize=(8, 3.2))
    ax.plot([p[0] for p in pts], [p[1] for p in pts], "o-", lw=0.8, ms=4)
    ax.set_xlabel(args.x)
    ax.set_ylabel(args.y)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    out = args.out or (args.csv + ".png")
    fig.savefig(out, dpi=100)
    print(f"wrote {out} ({len(pts)} points)")


if __name__ == "__main__":
    sys.exit(main())
