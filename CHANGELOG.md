# Changelog

All notable changes to `power_engine`. Format follows Keep a Changelog;
versioning follows SemVer (`find_package(power_engine 0.1 CONFIG)`).

## [Unreleased]

FOC program (roadmap 19 + follow-ups): voltage-form cascaded FOC
(speed PI + velocity FF -> iq*, dq PIs + decoupling, carrier PWM);
SVPWM-fed variant (7-segment, Vdc/sqrt(3) ceiling); field weakening +
MTPA (Rs-inclusive ellipse FF + voltage-trim FB, current circle, demand
yoke; 48V drive holds 260 rad/s against 228 base); MTPV-lite
(geometric iqRef backoff); sensing module (incremental Encoder +
SpeedEstimator); sensorless observer (voltage-model flux + PLL, honest
I-f startup with speed-scheduled handoff, encoder-tight bounds); joint
PI anti-windup through the magnitude clamp (current outward-step
revert + deep-corner outer freeze; 12V recovery dead time ~190ms ->
~50ms). Debugging paid twice: codebase Park needs opposite decoupling
signs to textbooks (plant-ID proven), and freeze-holds-entry beats
back-calculation for the outer loop.

Solver performance (roadmap 14d): event cursor over the .at-sorted edge
list (amortized O(1) lookup/apply, rewind on early insert) — 22x on a
2400-edge PWM run with bit-identical trajectories.

Magnetics (roadmap 11/18): gapped-inductor synthesis (N from Bsat,
gap with fringing, saturable-network verification, roll-off/Bpeak
enforcement); winding copper + window fill; Dowell AC copper loss;
hysteretic inductor device (explicit tangent companion, no Newton,
in-circuit loop-area loss, H netlist form, `hysteresisLoss`); vendor
core database stays future, thermal coupling caller-side.

Ecosystem + tooling: `pe` CLI (subcommand registry; `run` netlist->CSV
with overrides/methods/probes; `sweep` grids incl. deterministic MC +
yield/stats; exit 0/1/2); FMI 3.0 co-simulation export (token check,
no event mode, Float64-only, XSD-validated, ctypes-proven like 2.0);
`LinearStepper` exact-ZOH runtime simulation of exports (RC/buck match
MNA, proving export fidelity).

`pe steady-state` CLI (Phase 1 tooling): Newton-shooting periodic orbit
as a command — `--auto` (single-frequency `.control pwm`) or `--period`,
`--pop N` pre-charge, `--max-iters/--rel-tol/--abs-tol`, `--probe`/`--out`
steady-orbit CSV. stdout prints `converged=/iters=/residual=`; exit 0 on
convergence, 1 on non-convergence, 2 on usage. Hysteretic nets rejected
(memory state unmappable); `.tran tstop` ignored (command sets horizons).

`pe ac` CLI (Phase 1 tooling): settle-then-measure Bode via the Fourier
meter — source modulation (`src:`) or PWM duty modulation (`duty:`, exact
analytic edge scheduling, pwm/2 alias check), log grid `--from/--to/
--points`, `--method stepped|multitone` (bin-snapped, Schroeder phases),
`freq,<probe>_mag_db,<probe>_phase_deg` CSV. RC Bode and buck Gvd match
analytic plants; ZOH footprint documented (keep f*dt <= ~0.01).

Correctness: coverage 91.63% -> 93.96% (steadystate 100%/100%,
statespace 99.4%/98.8%); statespace current-source sign fix (uniform
KCL-leaving incidence; Vf-averaged-DC end-to-end) + Debug-only
FullPivLU(0x0) guard fix; MSVC C4996/C4459 fixes; shooting rejects
hysteretic memory loudly instead of converging wrong orbits.
Declined with evidence: diode breakpoint location (straddle already
mV-accurate; Roff-float chatter), partitioning (O(n^1.2) to n=5000,
domain stays <100 nodes). 264 tests / 30 suites.

Typed gate probes: `Solution::gates` carries `gate:<switch>` as a bool
(logic signal kept distinct from physical `probes`); `pe run --probe
gate:S1` prints it as a 0/1 CSV column. 264 tests / 30 suites.

Engine usability polish: MSVC C4996 crash fixed in `src/fmi/fmi_import.cpp`
(fmi import now uses `std::ifstream` instead of raw `FILE*`/`fopen` —
the sole Windows-lane CI blocker on `main`); googletest no longer leaks
into the install tree (`INSTALL_GTEST OFF`); install layout verified
clean (headers + `power_engine` + `pe` only) and the find_package
consumer smoke test passes against a real install prefix.

Phase 1 complete: damped saturable-Newton line search (1.4; two-pass
residual+damp, floor 1/16, `SolverStats::lineSearchFailures` monitor,
bit-identical on well-behaved loops); switched-sparse scaling benchmark
(1.5; `pe_scaling` — n=2000 at ~0.53 ms/step sub-cubic under gate-edge
churn, 94.5% cache hits, KLU stays declined). 1.3 (DOPRI) formally
declined with engineering justification: explicit RK needs a pointwise
ẋ = f(x,t) this implicit-MNA engine doesn't have; the statespace path is
explicit but excludes magnetics/saturable/hysteresis and can't re-settle
diodes in-stage; at ideal switching discontinuities error control is
LTE-bypassed anyway. Stiffness stays TR-BDF2/AUTO; any RK-flavor
high-order variant belongs in Phase 2 (Radau IIA).

## [0.2.0] — 2026-09-22

The competing-engine program: solver speed, stiff integrators,
credibility, scale, and ecosystem. 152 tests / 24 suites, CI 11/11
green (matrix + sanitizers + install + xval + fuzz + FMI),
Apache-2.0.

Solver performance (roadmap 14): topology+dt factorization cache
(Vienna 1.54x, ladders ~2x); hot-loop de-churn — probe snapshot,
device index, diode fast path (Vienna 1.84x more, ladders to 3.8x;
cumulative Vienna 2.8x, ladder-320 5.4x); `pe_bench` harness with
waveform-checksum baselines. KLU investigated and declined on evidence
(Eigen >= KLU below n~2000 on our matrix class).

Stiff integrators (roadmap 16): opt-in TR-BDF2 (2nd-order L-stable;
interrupt ringing 1674V to 1e-62); auto trap/BDF2 switching on
diode-pileup, Newton-strain, amplitude+decay-gated Nyquist flips;
embedded stage-difference adaptive control (RC 124 vs 5000 steps,
Vienna 1648 vs 60000); event-accepting controllers; SingularError +
grow-to-escape; `.tran [TRAP|TRBDF2|AUTO]`.

Credibility (roadmap 15): ngspice cross-validation (buck 5e-4, Vienna
1.5e-3) + wall-clock table; libFuzzer parser target + seed corpus;
91.63% line coverage; slew-limited `tsw` switch transitions
(geometric ramp; 190V vs 500kV inductive peaks; energy to 0.01%).

Scale + features (roadmap 17/18): threaded sweep (`jobs`, bitwise
tables, 2.8-5.7x); deterministic Monte Carlo samplers + stats;
conducted-EMI screening (LISN, CISPR 32 Class B; filter demo passes
+18dB); FMI 2.0 co-simulation export (XSD-validated .fmu, ctypes
proven: two-instance agreement, reset reproducibility).

## [0.1.0] — 2026-09-22

First tagged release: headless C++20 ideal-switching power-electronics
engine. 125 tests / 22 suites green on Linux/macOS/Windows (gcc/clang/
MSVC) + ASan/UBSan, warnings-as-errors, Apache-2.0.

- Core: fixed-step trapezoidal MNA (R/L/C/V/I, ideal switch/diode with
  event iteration, exact gate sub-stepping), opt-in adaptive
  step-doubling, dense/sparse auto-select, Newton loop for saturables.
- Devices: transformer, coupled inductors, saturable inductor, diode Qrr,
  switch tail, hysteresis magnetic core, reluctance network + windings.
- Netlist: SPICE-like (`.param/.model/.tran/.control/.thermal/.subckt`,
  `W`/`Y` magnetics).
- Control: PWM (trailing/symmetric, deadtime), PI + anti-windup,
  hysteresis, SVPWM, state machine, Tustin filters.
- Analysis: Fourier-meter Bode, multitone, series-injection loop gain,
  state-space export, shooting steady-state, measurement toolkit
  (RMS/ripple/THD/spectrum).
- Electro-thermal: table losses (Eon/Eoff, Ron(Tj)/Vf(Tj)), Foster/Cauer,
  averaged-loss thermal stepping.
- Validated: buck (CCM/DCM, open/closed-loop, sync), boost, flyback,
  full-bridge SPWM, 3-ph SVPWM, Vienna diode bridge + closed-loop PFC
  with hysteresis threshold-shift midpoint balancing, DAB, LLC, PMSM,
  induction V/f, grid×netlist sweep harness.
- Packaging: installable library with CMake package config
  (`PowerEngine::power_engine` static/shared) + `find_package` smoke test.
