#!/usr/bin/env python3
"""VioPELS <-> ngspice cross-validation (roadmap item 15).

Runs identical fixtures in our engine (demo CSV binaries) and in ngspice
batch mode, compares settled-window means with agreement bounds, and
prints a wall-clock benchmark table.

Usage:
  python3 xval.py [--build-dir build] [--ngspice ngspice] [--workdir DIR]

Exit code is nonzero on any bound violation (CI consumes this).
Skips gracefully (exit 0 + message) only if ngspice is missing AND
--allow-missing is given; CI installs ngspice so a missing binary there
is a hard error.
"""
import argparse
import math
import os
import statistics
import subprocess
import sys
import tempfile
import time

NG_BUCK = """* buck open loop 12V 20kHz D=0.5 (mirrors netlist_buck_demo)
V1 1 0 DC 12
Vg 10 0 PULSE(0 1 0 1n 1n 25u 50u)
S1 1 2 10 0 SMOD
.model SMOD SW(Ron=5m Roff=1Meg Vt=0.5 Vh=0)
D1 0 2 DMOD
.model DMOD D(Is=1e-14 N=0.01 Rs=10m)
L1 2 3 200u
C1 3 0 200u IC=0
Rload 3 0 5
.tran 0.1u 6m 0 0.5u UIC
.control
run
print v(3) > {out}
.endc
.end
"""

NG_VIENNA = """* Vienna diode bridge 3x230V/50Hz (mirrors vienna_demo, no ramp)
VA 1 10 SIN(0 325.269 50)
VB 2 10 SIN(0 325.269 50 0 0 -120)
VC 3 10 SIN(0 325.269 50 0 0 120)
RAg 1 11 0.5
RBg 2 12 0.5
RCg 3 13 0.5
LA 11 4 5m
LB 12 5 5m
LC 13 6 5m
DAu 4 7 DMOD
DBu 5 7 DMOD
DCu 6 7 DMOD
DAl 0 4 DMOD
DBl 0 5 DMOD
DCl 0 6 DMOD
.model DMOD D(Is=1e-14 N=0.01 Rs=10m Vfwd=0.7)
C1 7 9 2m IC=0
C2 9 0 2m IC=0
Rload 7 0 100
.tran 1u 60m 0 5u UIC
.control
run
print v(7) > {out}
.endc
.end
"""

NG_RC = """* RC charge 1V/1k/1u (harness self-check vs analytic)
V1 1 0 DC 1
R1 1 2 1k
C1 2 0 1u IC=0
.tran 1u 5m UIC
.control
run
print v(2) > {out}
.endc
.end
"""


def run(cmd, **kw):
    t0 = time.monotonic()
    kw.setdefault("stdout", subprocess.PIPE)
    r = subprocess.run(cmd, stderr=subprocess.STDOUT, text=True, **kw)
    return r, time.monotonic() - t0


def ngspice_mean(ng, netlist, workdir, t_lo):
    cir = os.path.join(workdir, "fx.cir")
    out = os.path.join(workdir, "ng.txt")
    with open(cir, "w") as f:
        f.write(netlist.format(out=out))
    r, wall = run([ng, "-b", cir], cwd=workdir)
    if r.returncode != 0 or not os.path.exists(out):
        raise RuntimeError(f"ngspice failed:\n{r.stdout[-2000:]}")
    ts, vs = [], []
    with open(out) as f:
        for line in f:
            parts = line.split()
            if len(parts) == 3 and parts[0].rstrip(".").isdigit():
                try:
                    t, v = float(parts[1]), float(parts[2])
                except ValueError:
                    continue
                if t >= t_lo:
                    ts.append(t)
                    vs.append(v)
    if not vs:
        raise RuntimeError("no ngspice samples parsed")
    return statistics.fmean(vs), wall


def our_csv_mean(exe, col, t_lo, workdir):
    csv = os.path.join(workdir, os.path.basename(exe) + ".csv")
    with open(csv, "w") as f:
        r, wall = run([exe], stdout=f, cwd=workdir)
    if r.returncode != 0:
        raise RuntimeError(f"{exe} failed:\n{r.stdout[-2000:]}")
    with open(csv) as f:
        header = f.readline().strip().split(",")
        idx = header.index(col)
        vs = [float(line.split(",")[idx]) for line in f
              if line.strip() and not line.startswith("#")
              and float(line.split(",")[0]) >= t_lo]
    if not vs:
        raise RuntimeError(f"no samples parsed from {exe}")
    return statistics.fmean(vs), wall


def check(name, ours, ref, bound, rows, ok):
    dev = abs(ours - ref) / abs(ref)
    status = "OK " if dev <= bound else "FAIL"
    rows.append((name, ours, ref, dev, bound, status))
    if dev > bound:
        ok[0] = False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--build-dir", default="build")
    ap.add_argument("--ngspice", default="ngspice")
    ap.add_argument("--workdir", default=None)
    ap.add_argument("--allow-missing", action="store_true")
    args = ap.parse_args()

    workdir = args.workdir or tempfile.mkdtemp(prefix="xval-")
    os.makedirs(workdir, exist_ok=True)
    build_dir = os.path.abspath(args.build_dir)
    ng = args.ngspice
    try:
        subprocess.run([ng, "--version"], capture_output=True, check=True)
    except (OSError, subprocess.CalledProcessError):
        msg = "ngspice not found; install it (apt install ngspice) or pass --ngspice"
        if args.allow_missing:
            print(msg + " (skipped)")
            return 0
        print("ERROR: " + msg, file=sys.stderr)
        return 2

    ex = lambda n: os.path.join(build_dir, "power_engine", "examples", n)
    rows, ok = [], [True]
    bench = []

    # RC: harness self-check, ngspice vs analytic 1-exp(-5).
    m_ng, w_ng = ngspice_mean(ng, NG_RC, workdir, 4.999e-3)
    check("rc/ng-vs-theory", m_ng, 1.0 - math.exp(-5.0), 1e-3, rows, ok)
    bench.append(("rc", None, w_ng))

    # Buck: ngspice vs our engine (settled last 1ms), 3% bound.
    m_ng, w_ng = ngspice_mean(ng, NG_BUCK, workdir, 5e-3)
    m_ours, w_ours = our_csv_mean(ex("netlist_buck_demo"), "vout", 5e-3, workdir)
    check("buck/ng-vs-ours", m_ng, m_ours, 0.03, rows, ok)
    bench.append(("buck-open-6ms", w_ours, w_ng))

    # Vienna: ngspice vs our engine (settled 40-60ms), 3% bound.
    m_ng, w_ng = ngspice_mean(ng, NG_VIENNA, workdir, 40e-3)
    m_ours, w_ours = our_csv_mean(ex("vienna_demo"), "vdc", 40e-3, workdir)
    check("vienna/ng-vs-ours", m_ng, m_ours, 0.03, rows, ok)
    bench.append(("vienna-60ms", w_ours, w_ng))

    print(f"{'fixture':<18}{'ours':>12}{'ngspice':>12}{'rel.dev':>10}{'bound':>8}  status")
    for name, ours, ref, dev, bound, status in rows:
        if name == "rc/ng-vs-theory":
            print(f"{name:<18}{ref:>12.6f}{ours:>12.6f}{dev:>10.2e}{bound:>8.1e}  {status}")
        else:
            print(f"{name:<18}{ours:>12.4f}{ref:>12.4f}{dev:>10.2e}{bound:>8.1e}  {status}")
    print(f"\n{'fixture':<18}{'ours wall':>12}{'ngspice wall':>14}")
    for name, w_ours, w_ng in bench:
        wo = f"{w_ours:.2f}s" if w_ours is not None else "-"
        print(f"{name:<18}{wo:>12}{w_ng:>13.2f}s")
    return 0 if ok[0] else 1


if __name__ == "__main__":
    sys.exit(main())
