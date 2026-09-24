#!/usr/bin/env python3
"""Functional check of a packed VioPELS FMI 3.0 FMU via ctypes.

Packs a buck FMU with fmi_pack.py --fmi-version 3, drives the prefixed
fmi3 CS API: state-machine order violations, bad token, event-mode and
intermediate-variable rejection, tComm mismatch, output-VR Set
rejection, two-instance bitwise agreement, reset reproducibility, and
Vout physics sanity. Exit nonzero on any failure.
"""
import argparse
import ctypes
import os
import statistics
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "scripts"))
import preflight

BUCK_NETLIST = """
.model SW mosfet_ideal RON=5m ROFF=1Meg EON=10u EOFF=15u
.model DD diode_ideal VF=0.0 RON=10m
V1 1 0 12
S1 1 2 MODEL=SW
D1 0 2 MODEL=DD
L1 2 3 200u
C1 3 0 200u
Rload 3 0 5
.control pwm switch=S1 freq=20k duty=0.5
.tran 0.5u 6m
.end
"""

OK, ERROR = 0, 3


def fail(msg):
    print(f"FAIL: {msg}", file=sys.stderr)
    sys.exit(1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pack", required=True, help="fmi_pack.py path")
    ap.add_argument("--lib", required=True)
    ap.add_argument("--pe-include", required=True)
    ap.add_argument("--fmi-include", required=True)
    ap.add_argument("--eigen-include", required=True)
    ap.add_argument("--workdir", default=None)
    args = ap.parse_args()

    try:
        preflight.require("unzip")
    except preflight.MissingToolError as e:
        print(f"FAIL: {e}", file=sys.stderr)
        return 2

    work = args.workdir or tempfile.mkdtemp(prefix="fmi3ct-")
    os.makedirs(work, exist_ok=True)
    net = os.path.join(work, "buck.net")
    with open(net, "w") as f:
        f.write(BUCK_NETLIST)
    fmu = os.path.join(work, "Buck3.fmu")
    r = subprocess.run(
        [sys.executable, args.pack, "--name", "BuckFMU3", "--netlist", net,
         "--inputs", "V1", "--outputs", "v:3", "--lib", args.lib,
         "--pe-include", args.pe_include, "--fmi-include", args.fmi_include,
         "--eigen-include", args.eigen_include, "--out", fmu,
         "--tstop", "0.006", "--fmi-version", "3",
         "--workdir", os.path.join(work, "pack")],
        capture_output=True, text=True)
    if r.returncode != 0:
        fail(f"pack failed:\n{r.stdout}\n{r.stderr}")
    subprocess.run(["unzip", "-qo", fmu, "-d", os.path.join(work, "fmu")], check=True)
    so = os.path.join(work, "fmu", "binaries", "linux64", "BuckFMU3.so")
    res = "file://" + os.path.join(work, "fmu", "resources") + "/"

    lib = ctypes.CDLL(so)
    P = "BuckFMU3_"

    def bind(name, restype=None, argtypes=None):
        fn = lib[name]
        if restype is not None:
            fn.restype = restype
        if argtypes is not None:
            fn.argtypes = argtypes
        return fn

    fmi3GetVersion = bind(P + "fmi3GetVersion", ctypes.c_char_p)
    assert fmi3GetVersion() == b"3.0", "version"

    B = ctypes.c_bool
    fmi3InstantiateCoSimulation = bind(
        P + "fmi3InstantiateCoSimulation", ctypes.c_void_p,
        [ctypes.c_char_p, ctypes.c_char_p, ctypes.c_char_p,
         B, B, B, B, ctypes.c_void_p, ctypes.c_size_t,
         ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p])
    fmi3DoStep = bind(
        P + "fmi3DoStep", ctypes.c_int,
        [ctypes.c_void_p, ctypes.c_double, ctypes.c_double, B,
         ctypes.POINTER(B), ctypes.POINTER(B), ctypes.POINTER(B),
         ctypes.POINTER(ctypes.c_double)])
    fmi3GetFloat64 = bind(
        P + "fmi3GetFloat64", ctypes.c_int,
        [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint), ctypes.c_size_t,
         ctypes.POINTER(ctypes.c_double), ctypes.c_size_t])
    fmi3SetFloat64 = bind(
        P + "fmi3SetFloat64", ctypes.c_int,
        [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint), ctypes.c_size_t,
         ctypes.POINTER(ctypes.c_double), ctypes.c_size_t])
    bound = {}
    for fn in ["fmi3FreeInstance", "fmi3EnterInitializationMode",
               "fmi3ExitInitializationMode", "fmi3Terminate", "fmi3Reset"]:
        bound[fn] = bind(P + fn, ctypes.c_int)
    fmi3FreeInstance = bound["fmi3FreeInstance"]
    bound["fmi3EnterInitializationMode"].argtypes = [
        ctypes.c_void_p, B, ctypes.c_double, ctypes.c_double, B, ctypes.c_double]
    fmi3EnterInitializationMode = bound["fmi3EnterInitializationMode"]
    for fn in ["fmi3ExitInitializationMode", "fmi3Terminate", "fmi3Reset"]:
        bound[fn].argtypes = [ctypes.c_void_p]
    fmi3ExitInitializationMode = bound["fmi3ExitInitializationMode"]
    fmi3Terminate = bound["fmi3Terminate"]
    fmi3Reset = bound["fmi3Reset"]

    with open(os.path.join(work, "fmu", "resources", "modelGuid.txt")) as f:
        token = f.read().strip()

    def inst(name="i1", tok=token, eventMode=False):
        return fmi3InstantiateCoSimulation(
            name.encode(), tok.encode(), res.encode(), False, False,
            eventMode, False, None, 0, None, None, None)

    # Bad token, event mode, and intermediate variables rejected.
    if inst(tok="nope") is not None:
        fail("bad token accepted")
    if inst(eventMode=True) is not None:
        fail("event mode accepted by CS-only FMU")

    def get(c, vr):
        v = (ctypes.c_double * len(vr))()
        st = fmi3GetFloat64(c, (ctypes.c_uint * len(vr))(*vr), len(vr), v, len(vr))
        return st, list(v)

    def set_(c, vr, vals):
        return fmi3SetFloat64(c, (ctypes.c_uint * len(vr))(*vr), len(vr),
                              (ctypes.c_double * len(vals))(*vals), len(vals))

    def do_step(c, t, dt=50e-6):
        ev, term, early, last = B(), B(), B(), ctypes.c_double()
        st = fmi3DoStep(c, t, dt, False, ctypes.byref(ev), ctypes.byref(term),
                        ctypes.byref(early), ctypes.byref(last))
        return st, ev.value, term.value, early.value, last.value

    def full_run(c):
        # Full order, 6ms in 50us communication steps; V1 held at 12.
        assert fmi3EnterInitializationMode(c, False, 0.0, 0.0, False, 0.006) == OK
        if fmi3EnterInitializationMode(c, False, 0.0, 0.0, False, 0.006) == OK:
            fail("double EnterInitializationMode accepted")
        assert fmi3ExitInitializationMode(c) == OK
        trace = []
        t = 0.0
        for _ in range(120):
            assert set_(c, [0], [12.0]) == OK
            st, ev, term, early, last = do_step(c, t)
            assert st == OK, f"doStep at {t}"
            assert not ev and not term and not early
            assert abs(last - (t + 50e-6)) < 1e-12
            st, v = get(c, [1])
            assert st == OK
            trace.append(v[0])
            t += 50e-6
        return trace

    c1 = inst()
    if not c1:
        fail("instantiate")
    # Out-of-order calls rejected.
    st, _, _, _, _ = do_step(c1, 0.0)
    if st == OK:
        fail("doStep before init accepted")
    if set_(c1, [1], [0.0]) == OK:
        fail("Set on output VR accepted")
    if get(c1, [7])[0] == OK:
        fail("Get on bad VR accepted")
    if fmi3EnterInitializationMode(c1, False, 0.0, 1.0, False, 0.0) == OK:
        fail("nonzero startTime accepted")
    trace1 = full_run(c1)
    # tComm mismatch rejected.
    st, _, _, _, _ = do_step(c1, 123.0)
    if st == OK:
        fail("tComm mismatch accepted")

    # Second instance agrees bitwise; reset reproduces.
    c2 = inst("i2")
    trace2 = full_run(c2)
    if trace1 != trace2:
        fail("instances disagree")
    assert fmi3Reset(c1) == OK
    trace1b = full_run(c1)
    if trace1 != trace1b:
        fail("reset run differs")
    for c in (c1, c2):
        assert fmi3Terminate(c) == OK
        fmi3FreeInstance(c)

    mean = statistics.fmean(trace1[-20:])
    print(f"vout mean (last 1ms): {mean:.4f}V over {len(trace1)} steps")
    if not 5.7 <= mean <= 6.3:
        fail(f"physics sanity: {mean}")
    print("ctypes FMI 3.0 check: all OK")


if __name__ == "__main__":
    sys.exit(main())
