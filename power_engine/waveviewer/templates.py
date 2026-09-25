"""Netlist template library for the editor (stdlib only, Tk-free).

Every template must simulate clean under `pe run` — enforced by
selfcheck.py, so the Template menu never ships a broken starting point.
Rule the templates obey (and teach): `.param` declarations precede use
(single-pass elaboration).
"""

BUCK = """\
.param R 5
.model SW mosfet_ideal RON=5m ROFF=1Meg
.model DD diode_ideal VF=0.0 RON=10m
V1 1 0 12
S1 1 2 MODEL=SW
D1 0 2 MODEL=DD
L1 2 3 200u
C1 3 0 200u
Rload 3 0 {R}
.control pwm switch=S1 freq=20k duty=0.5
.tran 0.5u 3m
.end
"""

BOOST = """\
.param R 10
.model SW mosfet_ideal RON=5m ROFF=1Meg
.model DD diode_ideal VF=0.0 RON=10m
V1 1 0 12
L1 1 2 200u
S1 2 0 MODEL=SW
D1 2 3 MODEL=DD
C1 3 0 200u
Rload 3 0 {R}
.control pwm switch=S1 freq=20k duty=0.5
.tran 0.5u 3m
.end
"""

RC = """\
V1 1 0 5
R1 1 2 1k
C1 2 0 1u
.tran 1u 5m
.end
"""

PUSHPULL = """\
.param R 10
.model SW mosfet_ideal RON=5m ROFF=1Meg
.model DD diode_ideal VF=0.7 RON=10m
V1 1 0 12
S1 2 0 MODEL=SW
S2 3 0 MODEL=SW
T1 2 1 3 4 0 RATIO=0.5
D1 4 5 MODEL=DD
C1 5 0 100u
Rload 5 0 {R}
.control pwm switch=S1 freq=50k duty=0.45 complement=S2 deadtime=300n
.tran 0.2u 2m
.end
"""

SYNCBUCK = """\
V1 1 0 12
P1 1 2 RON=5m ROFF=1Meg VF=0.7
P2 2 0 RON=5m ROFF=1Meg VF=0.7
L1 2 3 200u
C1 3 0 200u
Rload 3 0 5
.control pwm switch=P1 freq=20k duty=0.5 complement=P2 deadtime=500n
.tran 0.5u 3m
.end
"""

ZENER = """\
V1 1 0 12
R1 1 2 1k
D1 0 2 VF=0.7 RON=10m ROFF=1Meg VBR=5.1 RBR=10
.tran 1u 1m
.end
"""

TEMPLATES = {
    "buck (PWM)": BUCK,
    "boost (PWM)": BOOST,
    "RC step": RC,
    "push-pull converter": PUSHPULL,
    "sync buck (combo)": SYNCBUCK,
    "zener clamp": ZENER,
}


def validate(text):
    """Cheap pre-flight before calling `pe` (pe remains the authority).

    Returns a list of human hints; empty means 'looks plausible'.
    """
    hints = []
    lines = [ln.strip() for ln in text.splitlines()
             if ln.strip() and not ln.strip().startswith("*")]
    if not lines:
        return ["empty netlist"]
    if lines[-1] != ".end":
        hints.append("netlist should end with `.end`")
    if not any(ln.startswith(".tran") for ln in lines):
        hints.append("no `.tran dt tstop` line — pe run needs a stop time")
    params, used_before_decl = set(), set()
    for ln in lines:
        if ln.startswith(".param"):
            parts = ln.split()
            if len(parts) >= 3 and parts[1] not in params:
                params.add(parts[1])
        for tok in ln.split("{")[1:]:
            name = tok.split("}")[0] if "}" in tok else ""
            if name and name not in params:
                used_before_decl.add(name)
    for name in sorted(used_before_decl):
        hints.append(f"{{{name}}} used before `.param {name}` declaration")
    return hints
