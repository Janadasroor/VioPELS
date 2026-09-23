#!/usr/bin/env python3
"""Functional check of a packed VioPELS FMU via ctypes (no importer needed).

Packs a buck FMU with fmi_pack.py, drives the prefixed fmi2 CS API:
state-machine order violations, bad GUID, tComm mismatch, output-VR
Set rejection, two-instance bitwise agreement, reset reproducibility,
and Vout physics sanity. Exit nonzero on any failure.
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

    # R9: missing unzip used to die as a CalledProcessError traceback.
    try:
        preflight.require("unzip")
    except preflight.MissingToolError as e:
        print(f"FAIL: {e}", file=sys.stderr)
        return 2

    work = args.workdir or tempfile.mkdtemp(prefix="fmict-")
    os.makedirs(work, exist_ok=True)
    net = os.path.join(work, "buck.net")
    with open(net, "w") as f:
        f.write(BUCK_NETLIST)
    fmu = os.path.join(work, "Buck.fmu")
    r = subprocess.run(
        [sys.executable, args.pack, "--name", "BuckFMU", "--netlist", net,
         "--inputs", "V1", "--outputs", "v:3", "--lib", args.lib,
         "--pe-include", args.pe_include, "--fmi-include", args.fmi_include,
         "--eigen-include", args.eigen_include, "--out", fmu,
         "--tstop", "0.006", "--workdir", os.path.join(work, "pack")],
        capture_output=True, text=True)
    if r.returncode != 0:
        fail(f"pack failed:\n{r.stdout}\n{r.stderr}")
    subprocess.run(["unzip", "-qo", fmu, "-d", os.path.join(work, "fmu")], check=True)
    so = os.path.join(work, "fmu", "binaries", "linux64", "BuckFMU.so")
    res = "file://" + os.path.join(work, "fmu", "resources") + "/"

    lib = ctypes.CDLL(so)
    P = "BuckFMU_"

    def bind(name, restype=None, argtypes=None):
        fn = lib[name]
        if restype is not None:
            fn.restype = restype
        if argtypes is not None:
            fn.argtypes = argtypes
        return fn

    fmi2GetVersion = bind(P + "fmi2GetVersion", ctypes.c_char_p)
    assert fmi2GetVersion() == b"2.0", "version"
    fmi2GetTypesPlatform = bind(P + "fmi2GetTypesPlatform", ctypes.c_char_p)
    assert fmi2GetTypesPlatform() == b"default", "platform"

    fmi2Instantiate = bind(
        P + "fmi2Instantiate", ctypes.c_void_p,
        [ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_char_p,
         ctypes.c_void_p, ctypes.c_int, ctypes.c_int])
    fmi2DoStep = bind(P + "fmi2DoStep", ctypes.c_int,
                      [ctypes.c_void_p, ctypes.c_double, ctypes.c_double, ctypes.c_int])
    fmi2GetReal = bind(
        P + "fmi2GetReal", ctypes.c_int,
        [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint), ctypes.c_size_t,
         ctypes.POINTER(ctypes.c_double)])
    fmi2SetReal = bind(
        P + "fmi2SetReal", ctypes.c_int,
        [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint), ctypes.c_size_t,
         ctypes.POINTER(ctypes.c_double)])
    bound = {}
    for fn in ["fmi2FreeInstance", "fmi2SetupExperiment", "fmi2EnterInitializationMode",
               "fmi2ExitInitializationMode", "fmi2Terminate", "fmi2Reset"]:
        bound[fn] = bind(P + fn, ctypes.c_int)
    fmi2FreeInstance = bound["fmi2FreeInstance"]
    bound["fmi2SetupExperiment"].argtypes = [
        ctypes.c_void_p, ctypes.c_int, ctypes.c_double, ctypes.c_double,
        ctypes.c_int, ctypes.c_double]
    fmi2SetupExperiment = bound["fmi2SetupExperiment"]
    for fn in ["fmi2EnterInitializationMode", "fmi2ExitInitializationMode",
               "fmi2Terminate", "fmi2Reset"]:
        bound[fn].argtypes = [ctypes.c_void_p]
    fmi2EnterInitializationMode = bound["fmi2EnterInitializationMode"]
    fmi2ExitInitializationMode = bound["fmi2ExitInitializationMode"]
    fmi2Terminate = bound["fmi2Terminate"]
    fmi2Reset = bound["fmi2Reset"]

    with open(os.path.join(work, "fmu", "resources", "modelGuid.txt")) as f:
        guid = f.read().strip()

    def inst(name="i1", g=guid):
        return fmi2Instantiate(name.encode(), 1, g.encode(), res.encode(), None, 0, 0)

    # Bad GUID and wrong type rejected.
    if inst(g="nope") is not None:
        fail("bad GUID accepted")
    if fmi2Instantiate(b"x", 0, guid.encode(), res.encode(), None, 0, 0):
        fail("ME type accepted by CS-only FMU")

    def get(c, vr):
        v = (ctypes.c_double * len(vr))()
        st = fmi2GetReal(c, (ctypes.c_uint * len(vr))(*vr), len(vr), v)
        return st, list(v)

    def set_(c, vr, vals):
        return fmi2SetReal(c, (ctypes.c_uint * len(vr))(*vr), len(vr),
                                      (ctypes.c_double * len(vals))(*vals))

    def full_run(c):
        # Full order, 6ms in 50us communication steps; V1 held at 12.
        assert fmi2SetupExperiment(c, 0, 0.0, 0.0, 0, 0.006) == OK
        assert fmi2EnterInitializationMode(c) == OK
        assert fmi2ExitInitializationMode(c) == OK
        trace = []
        t = 0.0
        for _ in range(120):
            assert set_(c, [0], [12.0]) == OK
            assert fmi2DoStep(c, t, 50e-6, 1) == OK, f"doStep at {t}"
            st, v = get(c, [1])
            assert st == OK
            trace.append(v[0])
            t += 50e-6
        return trace

    c1 = inst()
    if not c1:
        fail("instantiate")
    # Out-of-order calls rejected.
    if fmi2DoStep(c1, 0.0, 50e-6, 1) == OK:
        fail("doStep before init accepted")
    if set_(c1, [1], [0.0]) == OK:
        fail("Set on output VR accepted")
    if get(c1, [7])[0] == OK:
        fail("Get on bad VR accepted")
    trace1 = full_run(c1)
    # tComm mismatch rejected.
    if fmi2DoStep(c1, 123.0, 50e-6, 1) == OK:
        fail("tComm mismatch accepted")

    # Second instance agrees bitwise; reset reproduces.
    c2 = inst("i2")
    trace2 = full_run(c2)
    if trace1 != trace2:
        fail("instances disagree")
    assert fmi2Reset(c1) == OK
    trace1b = full_run(c1)
    if trace1 != trace1b:
        fail("reset run differs")
    for c in (c1, c2):
        assert fmi2Terminate(c) == OK
        fmi2FreeInstance(c)

    mean = statistics.fmean(trace1[-20:])
    print(f"vout mean (last 1ms): {mean:.4f}V over {len(trace1)} steps")
    if not 5.7 <= mean <= 6.3:
        fail(f"physics sanity: {mean}")
    print("ctypes FMU check: all OK")


if __name__ == "__main__":
    sys.exit(main())
