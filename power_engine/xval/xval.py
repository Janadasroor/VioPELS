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

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "scripts"))
import preflight

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

# Demo-CSV schema contract (refine-roadmap R6): the single source of truth
# for which demo binary prints which columns. Each demo carries a pointer
# comment back here; a column rename must update both sides. Drift on the
# demo side raises RuntimeError naming the demo + expected + actual header
# (never a bare ValueError); drift diagnosis always says which side moved.
FIXTURES = {
    "netlist_buck_demo": {"time": "time", "value": "vout"},
    "vienna_demo": {"time": "time", "value": "vdc"},
}


def run(cmd, **kw):
    t0 = time.monotonic()
    kw.setdefault("stdout", subprocess.PIPE)
    r = subprocess.run(cmd, stderr=subprocess.STDOUT, text=True, **kw)
    return r, time.monotonic() - t0


def ngspice_mean(ng, netlist, workdir, t_lo, fixture="<ngspice>"):
    cir = os.path.join(workdir, "fx.cir")
    out = os.path.join(workdir, "ng.txt")
    with open(cir, "w") as f:
        f.write(netlist.format(out=out))
    r, wall = run([ng, "-b", cir], cwd=workdir)
    if r.returncode != 0 or not os.path.exists(out):
        raise RuntimeError(f"ngspice side ({fixture}) failed:\n{r.stdout[-2000:]}")
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
        raise RuntimeError(f"ngspice side ({fixture}): no samples parsed")
    return statistics.fmean(vs), wall


def read_demo_csv(path, demo, t_lo):
    """Settled-window mean of the contract value column (demo side).

    Every failure names the demo binary + what was expected + what was
    found, so a schema drift points at the side that moved.
    """
    contract = FIXTURES[demo]
    time_col, val_col = contract["time"], contract["value"]
    try:
        f = open(path)
    except OSError as e:
        raise RuntimeError(f"demo side ({demo}): cannot read {path}: {e}")
    with f:
        raw_header = f.readline()
        if not raw_header:
            raise RuntimeError(f"demo side ({demo}): empty output (no header)")
        header = raw_header.strip().split(",")
        if not header or header[0] != time_col:
            raise RuntimeError(
                f"demo side ({demo}): expected time column {time_col!r} first, "
                f"got header {header}")
        if val_col not in header:
            raise RuntimeError(
                f"demo side ({demo}): expected column {val_col!r} "
                f"(contract {demo}.{val_col}), got header {header}")
        idx = header.index(val_col)
        vs = []
        for lineno, line in enumerate(f, start=2):
            if not line.strip() or line.startswith("#"):
                continue
            fields = line.split(",")
            if len(fields) != len(header):
                raise RuntimeError(
                    f"demo side ({demo}): line {lineno}: {len(fields)} fields, "
                    f"header has {len(header)}")
            try:
                t = float(fields[0])
            except ValueError:
                raise RuntimeError(
                    f"demo side ({demo}): line {lineno}: bad time value "
                    f"{fields[0]!r}")
            if t < t_lo:
                continue
            try:
                vs.append(float(fields[idx]))
            except ValueError:
                raise RuntimeError(
                    f"demo side ({demo}): line {lineno}: bad {val_col} value "
                    f"{fields[idx]!r}")
    if not vs:
        raise RuntimeError(f"demo side ({demo}): no samples at/after t={t_lo}")
    return statistics.fmean(vs)


def our_csv_mean(exe, demo, t_lo, workdir):
    csv = os.path.join(workdir, os.path.basename(exe) + ".csv")
    with open(csv, "w") as f:
        r, wall = run([exe], stdout=f, cwd=workdir)
    if r.returncode != 0:
        raise RuntimeError(f"demo side ({demo}) failed:\n{r.stdout[-2000:]}")
    return read_demo_csv(csv, demo, t_lo), wall


def check(name, ours, ref, bound, rows, ok):
    dev = abs(ours - ref) / abs(ref)
    status = "OK " if dev <= bound else "FAIL"
    rows.append((name, ours, ref, dev, bound, status))
    if dev > bound:
        ok[0] = False


def self_check():
    """Contract tests for the demo-CSV reader (no ngspice/build needed).

    Proves schema drift is caught with the side named: each failing case
    must raise RuntimeError mentioning the demo binary + expected column.
    """
    import tempfile
    cases = []

    def csv_file(workdir, name, text):
        path = os.path.join(workdir, name)
        with open(path, "w") as f:
            f.write(text)
        return path

    with tempfile.TemporaryDirectory(prefix="xval-selfcheck-") as td:
        demo = "netlist_buck_demo"
        results = []
        # Valid: comments/blanks tolerated, settled mean exact.
        p = csv_file(td, "ok.csv",
                     "time,vout,tj_s1\n# trailer comment\n\n"
                     "0.004,5.0,300.0\n0.005,6.0,301.0\n0.006,8.0,302.0\n")
        ok = abs(read_demo_csv(p, demo, 5e-3) - 7.0) < 1e-12
        print(f"self-check: valid-mean: {'OK' if ok else 'WRONG-VALUE'}")
        results.append(("valid-mean", ok))

        def expect_error(case, text, *needles):
            p = csv_file(td, case + ".csv", text)
            try:
                read_demo_csv(p, demo, 5e-3)
            except RuntimeError as e:
                ok = all(n in str(e) for n in needles)
                print(f"self-check: {case}: {'OK' if ok else 'WRONG-MESSAGE: ' + e}")
                return ok
            print(f"self-check: {case}: NO-THROW (bad)")
            return False

        results.append(("missing-column",
                        expect_error("missing-column", "time,voot,tj_s1\n0.005,6.0,301.0\n",
                                     demo, "'vout'", "voot")))
        results.append(("renamed-time",
                        expect_error("renamed-time", "t,vout,tj_s1\n0.005,6.0,301.0\n",
                                     demo, "'time'")))
        results.append(("ragged-row",
                        expect_error("ragged-row", "time,vout,tj_s1\n0.005,6.0\n",
                                     demo, "line 2")))
        results.append(("bad-float",
                        expect_error("bad-float", "time,vout,tj_s1\n0.005,xx,301.0\n",
                                     demo, "line 2", "vout")))
        results.append(("no-samples",
                        expect_error("no-samples", "time,vout,tj_s1\n0.001,6.0,301.0\n",
                                     demo, "no samples")))

    passed = sum(1 for _, ok in results if ok)
    total = len(results)
    print(f"self-check: {passed}/{total} passed")
    return 0 if passed == total else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--build-dir", default="build")
    ap.add_argument("--ngspice", default="ngspice")
    ap.add_argument("--workdir", default=None)
    ap.add_argument("--allow-missing", action="store_true")
    ap.add_argument("--self-check", action="store_true",
                    help="run demo-CSV contract tests only (no ngspice/build needed)")
    args = ap.parse_args()
    if args.self_check:
        return self_check()

    workdir = args.workdir or tempfile.mkdtemp(prefix="xval-")
    os.makedirs(workdir, exist_ok=True)
    build_dir = os.path.abspath(args.build_dir)
    ng = args.ngspice
    try:
        preflight.require(ng)
    except preflight.MissingToolError as e:
        msg = f"{e} (or pass --ngspice to point at one)"
        if args.allow_missing:
            print(msg + " (skipped)")
            return 0
        print("ERROR: " + msg, file=sys.stderr)
        return 2

    ex = lambda n: os.path.join(build_dir, "power_engine", "examples", n)
    rows, ok = [], [True]
    bench = []

    # RC: harness self-check, ngspice vs analytic 1-exp(-5).
    m_ng, w_ng = ngspice_mean(ng, NG_RC, workdir, 4.999e-3, "rc")
    check("rc/ng-vs-theory", m_ng, 1.0 - math.exp(-5.0), 1e-3, rows, ok)
    bench.append(("rc", None, w_ng))

    # Buck: ngspice vs our engine (settled last 1ms), 3% bound.
    m_ng, w_ng = ngspice_mean(ng, NG_BUCK, workdir, 5e-3, "buck")
    m_ours, w_ours = our_csv_mean(ex("netlist_buck_demo"), "netlist_buck_demo",
                                  5e-3, workdir)
    check("buck/ng-vs-ours", m_ng, m_ours, 0.03, rows, ok)
    bench.append(("buck-open-6ms", w_ours, w_ng))

    # Vienna: ngspice vs our engine (settled 40-60ms), 3% bound.
    m_ng, w_ng = ngspice_mean(ng, NG_VIENNA, workdir, 40e-3, "vienna")
    m_ours, w_ours = our_csv_mean(ex("vienna_demo"), "vienna_demo", 40e-3,
                                  workdir)
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
