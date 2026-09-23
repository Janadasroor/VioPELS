#!/usr/bin/env python3
"""Shared external-tool preflight for the VioPELS Python tooling
(refine-roadmap R9): xval.py + fmi_pack.py + ctypes_check.py.

One clear error naming the install, one exit code (2) for every
missing-tool failure — instead of FileNotFoundError tracebacks,
"command not found" from shells, or per-script ad-hoc probes.

Usage as a module:
    from preflight import require, require_path, MissingToolError, EXIT_MISSING
    require("ngspice")                    # raises MissingToolError (exit 2)
    require_path(args.lib, "static lib")  # same for required files/dirs

Usage as a CLI gate (CI + humans):
    python3 preflight.py zip unzip xmllint c++   # exit 0 or 2
    python3 preflight.py --self-test             # unit tests, no tools needed
"""
import os
import shutil
import subprocess
import sys

EXIT_MISSING = 2  #: uniform exit code for missing-tool failures

INSTALL_HINTS = {
    "ngspice": "apt install ngspice",
    "zip": "apt install zip",
    "unzip": "apt install unzip",
    "xmllint": "apt install libxml2-utils",
    "c++": "apt install g++",
    "cc": "apt install gcc",
    "gcc": "apt install gcc",
    "g++": "apt install g++",
    "clang++": "apt install clang",
    "cmake": "apt install cmake",
    "python3": "apt install python3",
}


class MissingToolError(RuntimeError):
    """A required external tool (or file) is missing; exit EXIT_MISSING."""


def hint_for(prog):
    base = os.path.basename(prog)
    if base in INSTALL_HINTS:
        return INSTALL_HINTS[base]
    if base.endswith(("++", "cc", "c")) or "clang" in base or "gcc" in base:
        return "install a C++ compiler (apt install g++)"
    return "install it and ensure it is on PATH"


def require(prog):
    """Raise MissingToolError unless `prog` resolves on PATH (or is an
    executable path). Returns the resolved path otherwise."""
    if prog and shutil.which(prog):
        return prog
    raise MissingToolError(
        f"missing required tool {prog!r} (install: {hint_for(prog or '?')})")


def require_path(path, what="file"):
    """Raise MissingToolError unless `path` exists. Returns `path`."""
    if path and os.path.exists(path):
        return path
    raise MissingToolError(f"missing required {what}: {path}")


def check_all(progs):
    """Return the list of MissingToolErrors for `progs` (empty == all ok)."""
    missing = []
    for p in progs:
        try:
            require(p)
        except MissingToolError as e:
            missing.append(e)
    return missing


def _self_test():
    cases = []

    def expect_missing(prog, *needles):
        try:
            require(prog)
        except MissingToolError as e:
            ok = all(n in str(e) for n in needles)
            print(f"self-test: {prog or '?'}: {'OK' if ok else 'WRONG-MESSAGE: ' + e}")
            return ok
        print(f"self-test: {prog or '?'}: NO-THROW (bad)")
        return False

    cases.append(("bogus-tool",
                  expect_missing("definitely-not-a-viopeIs-tool-xyz",
                                 "definitely-not-a-viopeIs-tool-xyz", "install:")))
    hint_ok = hint_for("ngspice") == "apt install ngspice"
    print(f"self-test: hint-known: {'OK' if hint_ok else 'BAD'}")
    cases.append(("hint-known", hint_ok))
    try:
        require(sys.executable)
        print("self-test: existing-tool: OK")
        cases.append(("existing-tool", True))
    except MissingToolError as e:
        print(f"self-test: existing-tool: BAD ({e})")
        cases.append(("existing-tool", False))
    try:
        require_path("/definitely/not/here-xyz", "static lib")
        print("self-test: missing-path: NO-THROW (bad)")
        cases.append(("missing-path", False))
    except MissingToolError as e:
        ok = "static lib" in str(e)
        print(f"self-test: missing-path: {'OK' if ok else 'WRONG-MESSAGE: ' + e}")
        cases.append(("missing-path", ok))
    cases.append(("existing-path", require_path(__file__, "self") == __file__))
    print(f"self-test: existing-path: {'OK' if cases[-1][1] else 'BAD'}")
    # CLI exit codes: 0 when all present, 2 with a missing tool.
    r = subprocess.run([sys.executable, __file__, sys.executable],
                       capture_output=True, text=True)
    ok = r.returncode == 0
    print(f"self-test: cli-ok: {'OK' if ok else f'BAD rc={r.returncode}'}")
    cases.append(("cli-ok", ok))
    r = subprocess.run([sys.executable, __file__, "definitely-not-a-viopeIs-tool-xyz"],
                       capture_output=True, text=True)
    ok = r.returncode == EXIT_MISSING and "install:" in (r.stdout + r.stderr)
    print(f"self-test: cli-missing: {'OK' if ok else f'BAD rc={r.returncode}'}")
    cases.append(("cli-missing", ok))

    passed = sum(1 for _, ok in cases if ok)
    print(f"self-test: {passed}/{len(cases)} passed")
    return 0 if passed == len(cases) else 1


def main(argv):
    if "--self-test" in argv:
        return _self_test()
    progs = [a for a in argv if not a.startswith("-")]
    if not progs:
        print("usage: preflight.py [--self-test] tool [tool ...]", file=sys.stderr)
        return EXIT_MISSING
    missing = check_all(progs)
    if not missing:
        print("preflight: all tools present: " + ", ".join(progs))
        return 0
    for e in missing:
        print(f"preflight: {e}", file=sys.stderr)
    return EXIT_MISSING


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
