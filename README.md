# VioPELS / power_engine — Phases 1–6 + extensions

[![CI](https://github.com/Janadasroor/VioPELS/actions/workflows/ci.yml/badge.svg)](https://github.com/Janadasroor/VioPELS/actions/workflows/ci.yml)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

Headless C++20 system-level power-electronics engine.

## Scope
- Circuit: R, L, C, V/I sources, ideal Switch (Ron/Roff, Eon/Eoff, tail,
  slew-limited `tsw` transitions), ideal Diode (Vf, Ron/Roff, Qrr
  recovery), ideal Transformer (ratio), center-tap transformer
  (shared-core push-pull), coupled inductors (M/k),
  saturable inductor (tanh λ(i), Newton), hysteresis core, nodal
  reluctance network + windings, ground `0`
- Solver: fixed-step trapezoidal MNA (opt-in step-doubling adaptive;
  opt-in TR-BDF2 stiff integrator with auto switching + embedded error
  control); factorization caching; exact-time gate edges via sub-stepping;
  dense PartialPivLU / SparseLU auto-selection at 64 rows
- Netlist: SPICE-like with SI suffixes, `{expressions}`, `.param`,
  `.model`, `.tran [TRAP|TRBDF2|AUTO]`, `.control pwm`, `.thermal`,
  `.subckt`/X, `W`/`Y` magnetics
- Control: PWM (trailing/symmetric, deadtime pair), PI + anti-windup,
  hysteretic comparator, SVPWM, state machine, half-bridge driver,
  Tustin transfer functions
- Electro-thermal: conduction (exact v·i) + switching (Eon/Eoff, Qrr·Vr)
  losses, Foster (exact update) / Cauer networks, `tj:<dev>` probes
- Validated: buck (CCM/DCM, open/closed-loop, sync), boost, flyback,
  full-bridge SPWM, 3-ph SVPWM, Vienna diode bridge + closed-loop PFC,
  DAB, LLC, PMSM (hysteretic + field-oriented), induction; 232 tests,
  24 binaries, zero warnings
- Analysis: Fourier-meter Bode, multitone, series-injection loop gain,
  state-space export, shooting steady-state, THD/ripple toolkit,
  grid×netlist sweep (threaded) + Monte Carlo tolerance analysis
- Magnetics & EMI: hysteresis core + eddy loss, reluctance networks,
  Tj-aware inductor synthesis (Cu rho(T), hot-Bs bound, Dowell at temp),
  conducted-EMI screening (DC LISN + CISPR 32 Class B)
- Interop: installable CMake package (`find_package` smoke-tested),
  FMI 2.0 co-simulation export (XSD-validated) + FMI 2.0 CS import
  (`fmi_import.h`: external controller FMUs drive gates via `CoSim`),
  ngspice cross-validation harness (agreement 5e-4..1.5e-3)

## Build
```sh
cmake -B build -DPOWER_ENGINE_BUILD_TESTS=ON   # Release by default (see below)
cmake --build build
ctest --test-dir build --output-on-failure
./build/power_engine/examples/netlist_buck_demo | head
```

Command line (`pe`, subcommand registry — `pe <command> --help`):
```sh
cmake --build build --target pe
./build/power_engine/tools/pe run --netlist buck.net --out sim.csv
./build/power_engine/tools/pe run --netlist buck.net --probe v:3 \
  --tstop 0.002 --method trbdf2 --param RLOAD=10
```
`pe run` loads a netlist, expands `.control pwm` / `.thermal` / `.etable`
specs, runs to `.tran tstop` (overridable), and writes `time,<probes>`
CSV to stdout or `--out`. Exit codes: 0 ok, 1 sim error, 2 usage/input
error. New features plug in as subcommands (one file + one registration;
in-process test seam, no spawned processes in tests).

`sweep` runs a parameter grid (`--axis K=list|lo:step:hi|uniform|gauss`,
`--measure last|mean|min|max(PROBE)`, `--jobs`, `--yield`, `--stats`):
```sh
./build/power_engine/tools/pe sweep --netlist buck.net \
  --axis RLOAD=4,5,6 --measure 'last(v:3)' --jobs 0
```

Plotting (`power_engine/plots/`, matplotlib recipes over `pe` CSVs):
```sh
./build/power_engine/tools/pe run --netlist buck.net --out run.csv
python3 power_engine/plots/plot_run.py run.csv            # stacked traces
python3 power_engine/plots/plot_compare.py --probe v:3 a.csv b.csv  # overlay
python3 power_engine/plots/plot_spectrum.py run.csv --probe v:3     # dBFS FFT
./build/power_engine/tools/pe sweep --netlist buck.net --axis R=1,2,5,10 \
  --measure 'mean(v:3) as vout' --out sweep.csv
python3 power_engine/plots/plot_sweep.py sweep.csv --x R --y vout
bash power_engine/plots/selfcheck.sh  # headless end-to-end check (also in CI)
```

Wave viewer + netlist editor (`power_engine/waveviewer/`, stdlib only):
```sh
python3 power_engine/waveviewer/wview.py run.csv        # viewer (+overlay)
python3 power_engine/waveviewer/wview.py --netlist buck.net  # editor
python3 power_engine/waveviewer/selfcheck.py  # headless logic (also in CI)
```
Tkinter ships with Python — no pip packages. Waves tab: probe toggles,
second-run overlay, drag/wheel zoom, cursor readout. Netlist tab:
template starters (each CI-proven to simulate clean), pre-flight hints,
Run via `pe`, one-click Plot.

Build types: default is **Release** — unoptimized Eigen is ~17x slower
(measured on the 6ms buck sim: 0.65s → 0.037s), so always benchmark and
ship Release. Sanitizers (as in CI):
```sh
cmake -B build-san -DCMAKE_BUILD_TYPE=Debug -DPOWER_ENGINE_SANITIZE=address,undefined
cmake --build build-san && ctest --test-dir build-san --output-on-failure
```

CI (`.github/workflows/ci.yml`, 12 jobs): Release matrix over
Ubuntu/Windows/macOS × gcc/clang/MSVC plus ASan+UBSan, Ninja,
install-smoke, ngspice cross-validation, plots workflow, fuzz corpus, and FMI
pack+check jobs — all green.

## Cross-validation vs ngspice + benchmark

`power_engine/xval/xval.py` runs identical fixtures in our engine and in
ngspice 45 batch mode (CI runs it on Ubuntu; needs `ngspice` + `python3`
locally). Settled-window means agree within the bounds (measured):

| fixture | ours | ngspice | rel. dev. | bound |
|---|---|---|---|---|
| buck Vout mean, last 1ms | 5.9635V | 5.9665V | 5.0e-4 | 3e-2 |
| Vienna Vdc mean, 40–60ms | 523.44V | 522.64V | 1.5e-3 | 3e-2 |

Wall-clock on the same fixtures (our demos print CSV; ngspice runs
variable-step — methodology in `xval.py`):

| fixture | ours | ngspice |
|---|---|---|
| buck 6ms | 0.03s | 24.7s |
| Vienna 60ms | 0.32s | 27.2s |

Ideal-switch fixed-step vs SPICE variable-step: the expected
orders-of-magnitude gap on system-long runs, measured honestly.

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
See `power_engine/` for `include/`, `src/` (one dir per domain),
`tests/`, `examples/`, plus `bench/` (benchmark harness + baselines),
`xval/` (ngspice cross-validation), `fuzz/` (libFuzzer target + corpus),
`fmi/` (FMI 2.0 export: wrapper, packager, schemas; import lives in
`include/power_engine/fmi_import.h` + `src/fmi/`), `coverage/`
(gcov script). Details in `power_engine/ARCHITECTURE.md`.
