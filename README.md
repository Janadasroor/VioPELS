# VioPELS / power_engine — Phases 1–6 + extensions

[![CI](https://github.com/Janadasroor/VioPELS/actions/workflows/ci.yml/badge.svg)](https://github.com/Janadasroor/VioPELS/actions/workflows/ci.yml)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

Headless C++20 system-level power-electronics engine.

## Scope
- Circuit: R, L, C, V/I sources, ideal Switch (Ron/Roff, Eon/Eoff, tail),
  ideal Diode (Vf, Ron/Roff, Qrr recovery), ideal Transformer (ratio),
  coupled inductors (M/k), saturable inductor (tanh λ(i), Newton),
  ground `0`
- Fixed-step trapezoidal MNA solver (opt-in adaptive step-doubling);
  matrices re-assembled every step; diodes iterate in-step with hysteresis;
  exact-time gate edges via sub-stepping; dense PartialPivLU / SparseLU
  auto-selection at 64 rows
- Netlist: SPICE-like with SI suffixes, `{expressions}`, `.param`, `.model`,
  `.tran`, `.control pwm`, `.thermal`, `.subckt`/X, `W`/`Y` magnetics
- Control: PWM (trailing/symmetric, deadtime pair), PI + anti-windup,
  hysteretic comparator, Tustin transfer functions
- Electro-thermal: conduction (exact v·i) + switching (Eon/Eoff, Qrr·Vr)
  losses, Foster (exact update) / Cauer networks, `tj:<dev>` probes
- Validated converters: buck (CCM/DCM, open/closed-loop, sync), boost,
  flyback; 132 tests, 23 binaries, zero warnings
- AC analysis: Fourier-meter Bode (RC + buck Gvd), `ac_demo` CSV
- Steady-state shooting: periodic orbits without startup transient

## Build
```sh
cmake -B build -DPOWER_ENGINE_BUILD_TESTS=ON   # Release by default (see below)
cmake --build build
ctest --test-dir build --output-on-failure
./build/power_engine/examples/netlist_buck_demo | head
```

Build types: default is **Release** — unoptimized Eigen is ~17x slower
(measured on the 6ms buck sim: 0.65s → 0.037s), so always benchmark and
ship Release. Sanitizers (as in CI):
```sh
cmake -B build-san -DCMAKE_BUILD_TYPE=Debug -DPOWER_ENGINE_SANITIZE=address,undefined
cmake --build build-san && ctest --test-dir build-san --output-on-failure
```

CI (`.github/workflows/ci.yml`): Release matrix over
Ubuntu/Windows/macOS × gcc/clang/MSVC plus an ASan+UBSan job — all green.

## Netlist sketch
```
.param VIN 12 FSW 20k D 0.5
.model SW mosfet_ideal RON=5m ROFF=1Meg EON=10u EOFF=15u
.model DD diode_ideal VF=0.0 RON=10m
V1 1 0 {VIN}
S1 1 2 MODEL=SW
D1 0 2 MODEL=DD
L1 2 3 200u
C1 3 0 200u
Rload 3 0 5
.control pwm switch=S1 freq={FSW} duty={D}
.thermal foster device=S1 R1=0.5 C1=0.01 Tamb=25
.tran 0.5u 6m
.end
```

## Layout
See `power_engine/` for `include/`, `src/{core,solver,api,netlist,control,thermal}/`,
`tests/`, `examples/`. Details in `power_engine/ARCHITECTURE.md`.
