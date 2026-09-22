# Changelog

All notable changes to `power_engine`. Format follows Keep a Changelog;
versioning follows SemVer (`find_package(power_engine 0.1 CONFIG)`).

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
