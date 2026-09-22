# Changelog

All notable changes to `power_engine`. Format follows Keep a Changelog;
versioning follows SemVer (`find_package(power_engine 0.1 CONFIG)`).

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
