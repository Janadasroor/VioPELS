# VioPELS Roadmap

Status: Phases 1–6 complete, plus extensions (converters, magnetics,
adaptive stepping, recovery physics, AC analysis). 51 tests / 12 binaries,
all green, zero warnings. See `power_engine/ARCHITECTURE.md` for the
as-built design and `README.md` for scope.

## Done

- [x] Phase 1 — core graph, trapezoidal MNA solver, R/L/C (RC/RLC vs analytical)
- [x] Phase 2 — ideal switch/diode, event iteration, open-loop buck, exact gate edges
- [x] Phase 3 — netlist parser (`.param/.model/.tran/.control/.thermal/.subckt`),
      transformer, `loadNetlist`/`setParameter`
- [x] Phase 4 — PWM/PI/comparator/transfer functions, closed-loop buck, `.control` wiring
- [x] Phase 5 — conduction + switching losses, Foster/Cauer, `tj:` probes
- [x] Phase 6 — CI matrix + ASan/UBSan, Release-by-default, dense/sparse
      auto-select, install rules, profiling notes
- [x] Extensions — boost/sync-buck/flyback/DCM validation, coupled inductors,
      adaptive step-doubling, saturable inductor (Newton), diode Qrr / IGBT tail
- [x] AC analysis — Fourier-meter Bode (RC + buck Gvd), `ac_demo`
- [x] Repo hygiene — `roadmap.md`, `agent.md`, `.gitignore`, git init

## Next (ordered by value)

### 1. GitHub push + live CI
Repo is initialized locally (`main` branch). Still needed: create the remote,
push, confirm all four CI jobs green (Release matrix, sanitizers, Ninja).
Acceptance: green badge run on `main`.

### 2. Piecewise-linear time-varying sources
Per-step (ZOH) drive carries an exact −ωdt/2 footprint vs continuous LTI
(measured, textbook-verified, linear in f·dt; see ARCHITECTURE "AC lessons").
PWL-continuous sources would restore full 2nd-order accuracy for driven sims
and shrink the oversampling needed for high-f Bode points.
Scope: source companion with intra-step ramp (V + I sources), netlist
`PULSE`/`SIN` element syntax, AC tests re-run at coarser dt to prove the win.
Acceptance: 10kHz RC Bode within tolerance at dt=5µs (no oversampling).

### 3. Closed-loop stability margins (Middlebrook injection)
All pieces exist (PWM, AC meter, scheduler). Add series/shunt injection into
the feedback path of the closed-loop buck, measure return ratio, report
gain/phase margins. Acceptance: margins match the PI design calculation
within loop-tolerance; demo prints margin table.

### 4. True electro-thermal coupling
Losses currently flow electrical → thermal only. Close the loop:
Tj-dependent Ron/Vf/Eon (lookup or linear coefs), iterate electrical +
thermal to consistency per step (or per N steps with documented lag).
Acceptance: buck efficiency droop vs Tambiente matches datasheet-style
curves within 5%; no solver instability introduced (existing suite green).

### 5. Steinmetz core loss for magnetics
`W`/`Y` devices + transformer currently lossless magnetically. Add
volume-normalized Steinmetz (k, alpha, beta params via `.model`), driven by
measured dB/dt per winding, accumulated into `deviceLoss` + thermals.
Acceptance: sinusoidal-excitation core loss vs analytical within 5%.

### 6. Averaged models for long-horizon sims
State-space-averaged buck/boost (continuous, no switching) for
seconds-scale runs (thermal soak, mission profiles). Validate averaged vs
switched envelope within 2% in slow dynamics.
Acceptance: 1s mission sim runs faster than real time in Release.

### 7. Robustness hardening
- Parser fuzzing (malformed-input corpus; every input throws cleanly, never hangs)
- Golden CSV regression: reference waveforms checked into `tests/golden/`,
  bitwise-compared in CI against committed tolerances
- Large-system stress: 1000+ node ladder, sparse-path timing assertion
Acceptance: fuzz clean for 1M cases, goldens green, stress under budget.

### 8. Release engineering
Version tags + changelog, install-tree smoke test (`find_package` consumer),
README badges, `ARCHITECTURE.md` kept in sync per change.

## Non-goals (standing constraints)

No GUI, no Qt, no ngspice, no FFI/Python bindings, no external runtimes.
Pure C++20 library; examples may print CSV only.
