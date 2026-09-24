#!/usr/bin/env python3
"""Pack a VioPELS netlist + I/O map into an FMI 2.0/3.0 Co-Simulation FMU.

Usage:
  fmi_pack.py --name Buck --netlist buck.net --inputs V1 --outputs v:3 \\
      --lib build/power_engine/libpower_engine.a \\
      --pe-include power_engine/include --fmi-include power_engine/fmi/include \\
      --eigen-include /usr/include/eigen3 --out /tmp/Buck.fmu [--tstop 0.006]
      [--fmi-version 2]

Layout: modelDescription.xml + binaries/linux64/<Name>.so + resources/
(model.netlist, io.txt, modelGuid.txt). The wrapper TU is model-agnostic;
FMI2/3_FUNCTION_PREFIX bakes the model name into the exports (per standard).
Version 3 uses fmi3_wrapper.cpp, fmiVersion 3.0 + instantiationToken (stored
in modelGuid.txt, read as the token), Float64 variables with initial.
"""
import argparse
import os
import shutil
import subprocess
import sys
import tempfile
import uuid
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "scripts"))
import preflight

WRAPPER = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fmi_wrapper.cpp")
WRAPPER3 = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fmi3_wrapper.cpp")


def esc(s):
    return (s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
             .replace('"', "&quot;"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", required=True)
    ap.add_argument("--netlist", required=True)
    ap.add_argument("--inputs", default="")
    ap.add_argument("--outputs", default="")
    ap.add_argument("--lib", required=True)
    ap.add_argument("--pe-include", required=True)
    ap.add_argument("--fmi-include", required=True)
    ap.add_argument("--eigen-include", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--tstop", default="0.006")
    ap.add_argument("--fmi-version", default="2", choices=("2", "3"))
    ap.add_argument("--cxx", default="c++")
    ap.add_argument("--workdir", default=None)
    args = ap.parse_args()
    if args.fmi_version not in ("2", "3"):
        sys.exit("fmi-version must be 2 or 3")

    if not args.name.replace("_", "").isalnum() or args.name[0].isdigit():
        sys.exit("model name must be C-identifier-like")
    # R9: one clear missing-tool error (exit 2) instead of tracebacks.
    try:
        preflight.require(args.cxx)
        preflight.require("zip")
        preflight.require_path(args.netlist, "netlist")
        preflight.require_path(args.lib, "static lib")
        for d in (args.pe_include, args.fmi_include, args.eigen_include):
            preflight.require_path(d, "include dir")
    except preflight.MissingToolError as e:
        print(f"fmi_pack: {e}", file=sys.stderr)
        sys.exit(preflight.EXIT_MISSING)
    inputs = [s for s in args.inputs.split(",") if s]
    outputs = [s for s in args.outputs.split(",") if s]
    with open(args.netlist) as f:
        netlist = f.read()

    guid = str(uuid.uuid4())
    work = args.workdir or tempfile.mkdtemp(prefix="fmipack-")
    res = os.path.join(work, "resources")
    bind = os.path.join(work, "binaries", "linux64")
    os.makedirs(res, exist_ok=True)
    os.makedirs(bind, exist_ok=True)
    with open(os.path.join(res, "model.netlist"), "w") as f:
        f.write(netlist)
    with open(os.path.join(res, "io.txt"), "w") as f:
        for d in inputs:
            f.write(f"input {d}\n")
        for p in outputs:
            f.write(f"output {p}\n")
    with open(os.path.join(res, "modelGuid.txt"), "w") as f:
        f.write(guid + "\n")

    vars_xml = []
    for i, d in enumerate(inputs):
        vars_xml.append(
            f'    <ScalarVariable name="{esc(d)}" valueReference="{i}" '
            f'causality="input" variability="continuous"><Real start="0.0"/></ScalarVariable>')
    for j, p in enumerate(outputs):
        vars_xml.append(
            f'    <ScalarVariable name="{esc(p)}" valueReference="{len(inputs) + j}" '
            f'causality="output" variability="continuous"><Real/></ScalarVariable>')
    unknowns = "\n".join(
        f'      <Unknown index="{i + 1}"/>' for i in range(len(inputs) + len(outputs)))
    xml = f"""<?xml version="1.0" encoding="UTF-8"?>
<fmiModelDescription fmiVersion="2.0" modelName="{esc(args.name)}" guid="{guid}" numberOfEventIndicators="0">
  <CoSimulation modelIdentifier="{esc(args.name)}_"/>
  <LogCategories><Category name="logAll"/><Category name="logError"/></LogCategories>
  <DefaultExperiment startTime="0.0" stopTime="{esc(args.tstop)}"/>
  <ModelVariables>
{chr(10).join(vars_xml)}
  </ModelVariables>
  <ModelStructure>
    <Outputs>
{unknowns}
    </Outputs>
  </ModelStructure>
</fmiModelDescription>
"""
    xml_path = os.path.join(work, "modelDescription.xml")
    with open(xml_path, "w") as f:
        f.write(xml)
    # Well-formedness now (schema check is the validator's job; CI runs xmllint).
    ET.parse(xml_path)

    if args.fmi_version == "3":
        vars3 = []
        for i, d in enumerate(inputs):
            vars3.append(
                f'    <Float64 name="{esc(d)}" valueReference="{i}" '
                f'causality="input" variability="continuous" initial="exact" start="0.0"/>')
        for j, p in enumerate(outputs):
            vars3.append(
                f'    <Float64 name="{esc(p)}" valueReference="{len(inputs) + j}" '
                f'causality="output" variability="continuous" initial="calculated"/>')
        unknowns3 = "\n".join(
            f'      <Output valueReference="{len(inputs) + j}"/>' for j in range(len(outputs)))
        xml = f"""<?xml version="1.0" encoding="UTF-8"?>
<fmiModelDescription fmiVersion="3.0" modelName="{esc(args.name)}" instantiationToken="{guid}">
  <CoSimulation modelIdentifier="{esc(args.name)}_"/>
  <ModelVariables>
{chr(10).join(vars3)}
  </ModelVariables>
  <ModelStructure>
{unknowns3}
  </ModelStructure>
</fmiModelDescription>
"""
        with open(xml_path, "w") as f:
            f.write(xml)
        ET.parse(xml_path)

    so = os.path.join(bind, f"{args.name}.so")
    if args.fmi_version == "3":
        wrapper, prefix = WRAPPER3, f"-DFMI3_FUNCTION_PREFIX={args.name}_"
    else:
        wrapper, prefix = WRAPPER, f"-DFMI2_FUNCTION_PREFIX={args.name}_"
    cmd = [args.cxx, "-shared", "-fPIC", "-O2", prefix, wrapper,
           "-I", args.pe_include, "-I", args.fmi_include, "-I", args.eigen_include,
           args.lib, "-o", so]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        sys.exit(f"wrapper compile failed:\n{r.stdout}\n{r.stderr}")
    fmu = os.path.abspath(args.out)
    if os.path.exists(fmu):
        os.remove(fmu)
    subprocess.run(["zip", "-qr", fmu, "modelDescription.xml", "binaries", "resources"],
                   cwd=work, check=True)
    print(f"packed {fmu} ({os.path.getsize(fmu)} bytes)")


if __name__ == "__main__":
    sys.exit(main())
