#!/usr/bin/env python3
"""Spectrum (dBFS) of one probe from a `pe run` CSV — quick EMI eyeball.

Usage:
  pe run --netlist buck.net --out run.csv
  python3 plot_spectrum.py run.csv --probe v:3 [--out spectrum.png] [--top 5]

Mean removed, Hann window, rfft; magnitude in dB relative to the peak bin.
Top peaks print to stdout (freq + dB). Uniform time grid required — `pe`
fixed-step output qualifies; resampled/external CSVs abort loudly.
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
    ap.add_argument("csv", help="`pe run` output CSV")
    ap.add_argument("--probe", required=True, help="probe column to analyse")
    ap.add_argument("--out", default="", help="PNG path (default: <csv>.spectrum.png)")
    ap.add_argument("--top", type=int, default=5, help="peaks to report")
    args = ap.parse_args()

    header, data = load(args.csv)
    if args.probe not in header[1:]:
        raise SystemExit(f"{args.csv}: unknown probe {args.probe!r}")
    t, y = data[:, 0], data[:, header.index(args.probe)] - data[:, header.index(args.probe)].mean()
    dt = np.diff(t)
    if not np.allclose(dt, dt[0], rtol=1e-9, atol=0):
        raise SystemExit(f"{args.csv}: non-uniform time grid, refusing to FFT")

    win = np.hanning(len(y))
    spec = np.abs(np.fft.rfft(y * win))
    freq = np.fft.rfftfreq(len(y), dt[0])
    db = 20 * np.log10(spec / spec.max())

    order = np.argsort(db)[::-1]
    seen, peaks = set(), []
    for i in order:  # keep distinct lobes, not neighbouring bins
        b = int(round(freq[i] / (freq[1] - freq[0]))) if len(freq) > 1 else 0
        if all(abs(b - s) > 1 for s in seen):
            seen.add(b)
            peaks.append((freq[i], db[i]))
        if len(peaks) == args.top:
            break
    for f, d in peaks:
        print(f"peak: {f:.3g} Hz @ {d:.1f} dBFS")

    fig, ax = plt.subplots(figsize=(8, 3.2))
    ax.semilogx(freq[1:], db[1:], lw=0.8)
    ax.set_xlabel("frequency (Hz)")
    ax.set_ylabel(f"{args.probe} (dBFS)")
    ax.grid(True, which="both", alpha=0.3)
    fig.tight_layout()
    out = args.out or (args.csv + ".spectrum.png")
    fig.savefig(out, dpi=100)
    print(f"wrote {out}")


if __name__ == "__main__":
    sys.exit(main())
