# ARCHITECTURE (Phases 1–5)

- `include/power_engine/`: public API (`circuit.h`, `device.h`, `solver.h`,
  `engine.h`, `types.h`, `netlist.h`, `control.h`, `thermal.h`).
- `src/core/circuit.cpp`: device container, node enumeration, validation.
  `addSwitch` (Ron/Roff + `closed` gate + Eon/Eoff), `addDiode`
  (anode/cathode, Vf, Ron/Roff, `conducting`), `addTransformer`
  (primary/secondary + ratio), `setSwitch` lookup. `numExtraUnknowns()`
  counts MNA branch rows (1 per V-source, 2 per transformer).
- `src/solver/solver.cpp`: MNA assembly + trapezoidal companions:
  - C: `G=2C/dt`, `Ihist=-G*Vprev-Iprev`
  - L: `G=dt/2L`, `Ihist=Iprev+G*Vprev`
  - Switch: resistor `closed ? Ron : Roff`, re-assembled every step.
  - Diode conducting: Norton `G=1/Ron || Isrc=G*Vf` (so `Vd=Vf+I*Ron`);
    blocking: `Roff`. After each solve, diode Vd/Id are checked and flipped
    states trigger an immediate re-solve at the same time point (up to 10
    iterations). Chatter suppressed with hysteresis (on when `Vd>Vf+1nV`,
    off when `Id<-1nA`). `SolverStats{steps,diodeEvents,resolves}`.
  - Transformer: branch currents Ip/Is with rows `Vp-n*Vs=0`,
    `n*Ip+Is=0` (KCL coupling kept; ideal algebraic, DC passes — no
    magnetics/saturation by design).
  - Solved per-step with `Eigen::FullPivLU`; singular → throw.
  - Histories (incl. switch/diode/transformer branch currents) updated once
    per accepted step; deterministic, single-threaded.
- `src/api/engine.cpp`: owns `Circuit` + `TransientSolver`, exposes
  `Solution{t, probes{"v:N"[, "tj:dev"]}, states}`, step callback,
  `setSwitch()` (next-step effect) plus `scheduleSwitch(name,closed,at)`:
  exact-time edges split `step()` into sub-steps
  (`dt_sub = t_event - t_now`, companions use the sub-step dt);
  diode iteration runs inside every sub-step; nominal dt restored after;
  `step()` clamps to `tStop`. Device *values* may change mid-run (load
  steps); topology may not. `deviceCurrent()` passthrough.
- `src/netlist/parser.cpp`: single-pass elaborator. SI suffixes
  (f p n u m k Meg G T), `{expressions}` over `.param` (recursive descent,
  + - * / ^, parens), named nodes (`0/GND` = ground), `.model` registry
  (mosfet_ideal/igbt_ideal/diode_ideal + EON/EOFF), `.tran`, `.control pwm`
  (incl. complement+deadtime, carrier), `.thermal` (bare form binds to last
  switch/diode), `.subckt`/X with hierarchical names/nodes (depth ≤ 16).
  MOSFET/IGBT are logic-gated ideal switches (VTO recorded, gate analog
  dynamics out of scope). Unknown directives/devices throw with line
  numbers; `serialize()` round-trips. `Engine::loadNetlist` replaces the
  circuit + applies `.tran`; `setParameter` re-elaborates with a locked
  override (pre-start only, `started_` latch).
- `src/control/control.cpp`: `Pwm` (trailing/symmetric carriers as pure
  time functions; `nextEdge` strictly increasing with fp guards;
  `complementary` deadtime pair never overlaps, both off for td after each
  edge, main on-time shrinks by td), `PiController` (clamp +
  conditional-integration anti-windup, backward Euler), `Comparator`
  (hysteresis band), `TransferFunction` (DF-II-t evaluation; Tustin via
  bilinear substitution with polynomial arithmetic, order ≤ 8, proper
  only). `Engine::applyPwmSpecs()` expands `.control pwm` to exact edges
  (explicit; needs stop time).
- `src/thermal/thermal.cpp` + engine coupling: conduction loss = ∫v·i per
  accepted sub-step (exact in the ideal model); switching loss = Eon/Eoff
  on detected gate edges (scheduled or manual). Foster stepped with the
  EXACT piecewise-constant-P update (unconditionally stable); Cauer with
  explicit Euler at solver sub-step dt (µs « ms–s time constants).
  `attachThermal` / `applyThermalSpecs`, `junctionTemp`, `deviceLoss`,
  `tj:` probes. Diode reverse recovery (Qrr) not modeled (ideal).
- Datasheet loss tables (`loss_tables.h`, `src/loss_tables/`): N-D grid
  tables (axes I/V/TJ) with multilinear interp + edge clamping, never
  extrapolation. Switch Eon/Eoff(I,V,TJ) sampled at gate edges — turn-on
  uses pre-edge blocking V + post-edge commutated I, turn-off the reverse
  (histories are post-step, so pre-edge comes from a pre-solve snapshot).
  Tj-dependent conduction: Ron(TJ)/Vf(TJ) tables refresh device params
  every sub-step (one-way explicit coupling; iteration is a later item);
  conduction loss stays ∫v·i (exact, self-consistent). Tj = thermal Tj or
  25C ambient. Netlist: `.etable` defs + EON_TABLE/EOFF_TABLE/RON_TABLE/
  VF_TABLE refs (via MODEL or inline), `Engine::applyLossModels`.
  Validated: constant tables reproduce scalar accounting; bilinear E=k*I*V
  buck switching and coupled Ron(Tj) electro-thermal match closed-form
  analytics within 5%.
- Numerical notes: SI/double/seconds; ground `0`; companions from `ic`;
  thermal states reset at `start()`; loss accumulators rebuilt at `start()`.
- Performance (Phase 6, measured on 6ms/12k-step open-loop buck, gcc):
  Release ≈ 0.035s (~3µs/step); unoptimized ≈ 0.65s (~54µs/step) — hence
  Release-by-default in the root CMake. Solver hot loop uses PartialPivLU
  (residuals equal to FullPivLU at this size: ~1.6e-16 on 5x5), reused MNA
  buffers and precomputed node rows (≈5% over the naive port at -O2, free).
  Full suite: ~0.3s Release, ~21s under ASan+UBSan (Debug).
- Extensions (post-Phase-6):
- Measurements (`measurements.h`, `src/measurements/`): Trace recording
  (probe or arbitrary sampler, exact windows via interpolated cropping),
  time-weighted stats (mean/RMS/min/max/pk-pk), windowed ripple, harmonic
  spectrum + THD via exact-kernel correlation (integer-period windows).
- Steady-state shooting (`steadystate.h`, `src/steadystate/`): Newton on
  the period map over continuous states (L currents, C voltages; diodes
  re-settle inside every period sim), finite-difference Jacobian,
  backtracking, per-component tolerances. Engine support: exact `runUntil`,
  scheduled-event replay, accumulator reset, solver save/restore +
  orbit repositioning (`rewindTo`). Thermals frozen over one period
  (timescale-separated by design; averaged-loss outer loop is follow-up).
- AC small-signal (`ac.h`, `src/ac/ac.cpp`): FourierMeter correlates the
  ACTUAL input/output against an exactly-integrated e^{-jwt} kernel with
  exact DC rejection; input is interval values (exact for ZOH/gates),
  output interval midpoints. Validated: RC Bode 0.5dB/4°, buck Gvd vs
  second-order plant 1.5dB/8° with resonance peak, DC gain, asymptotes.
  - LESSONS (each debugged, all documented so nobody repeats them):
    settle in max(periods, circuit Taus), not periods alone; pre-charge to
    the DC operating point (POP-style) or mV signals drown in startup
    residue; AGC the perturbation (duty*|Gvd|<=5%) or resonance overdrives
    into nonlinearity; cap finj below fsw/2 (Nyquist); open-loop analytic
    duty has NO sampler+ZOH (reference is plant-only Gvd, verified to
    0.1dB); per-step (ZOH) drive carries an exact -ωdt/2 footprint vs
    continuous LTI (order reduction on nonsmooth forcing, textbook-verified,
    linear in f*dt) — oversample high-f points (f*dt<=0.01).
    NOTE on "PWL sources": for ideal V/I sources the solver takes exactly
    one value per step (nodal constraints are instantaneous), so PWL vs ZOH
    vs midpoint sampling differ only in sample phase — measured *ratios*
    are invariant (verified). True intra-step ramps would need source
    companions; the footprint above is inherent to fixed-step sampling and
    is handled by oversampling, not by drive waveform choice. Drive
    waveforms live in `waveforms.h` (SIN/PULSE/PWL breakpoints for tests).
- Loop-gain analysis (`loopgain.h`, `src/loopgain/`): series injection at
  the sense node of the closed-loop buck (v_sense = vout + vinj, Vref AC=0),
  Gcl = vout/vinj via FourierMeter, T = -Gcl/(1+Gcl), worst-PM crossing +
  gain-margin report. Validated: Gcl within 1.5dB/8deg on flanks, critical
  crossover/PM within 15%/6deg of the design calculation.
  - FINDINGS (each debugged, all documented so nobody repeats them):
    the validated buck loop is CONDITIONALLY STABLE — LC resonance pushes
    |T| back above 0dB, critical margin is PM ~7deg at ~1.07kHz (not 147deg
    at the 167Hz first crossing); the ~1kHz mode rings with tau ~7ms, so
    fixtures need bumpless PI init (new setIntegrator) + exact DC-state
    reuse + slow injection envelope. Trailing-edge PWM is NOT a T/2
    zero-order hold — the modulated sliver sits at the trailing edge, delay
    D*T (verified to <2deg); ZOH overstates loop delay by (1-D)*T/2. Meter
    is fed the continuous sine (unbiased fundamental); the staircase would
    bias by its spectrum. Near the closed-loop peak Gcl ~ -1 makes
    T = -Gcl/(1+Gcl) ill-conditioned: validate Gcl per-point, T via margins.
  - Converters: boost, synchronous buck (deadtime + body diodes clamp Vsw),
    flyback (dot-convention secondary), DCM buck — all vs closed-form theory.
  - Coupled inductors: trapezoidal 2-port Norton from L*i flux linkage,
    k<1 enforced (k=1 singular → use Transformer); netlist `W` device.
  - Adaptive stepping (opt-in): step-doubling on node voltages, order-2
    controller, save/restore trials, floor-accept with growth recovery;
    exact landing via stepTo across event-free intervals. Note: error
    estimates are invalid across step discontinuities (e.g. t=0 source
    application), handled by floor-accept + regrow; fixedStep's dt_
    side effect is fenced by the controller (both debugged, tested).
  - Saturable inductor: λ(i)=Lsat*i+(Lunsat-Lsat)*Isat*tanh(i/Isat),
    Newton loop around the MNA solve (≤25 iters, honest throw on
    non-convergence); netlist `Y` device.
  - Reverse recovery / tail: diode triangular Qrr (impressed current —
    a parallel Ron would swallow it under reverse bias), switch
    exponential tail; recE=Qrr*|V| booked into losses+thermals at release.
    Fixed-shape approximations (no di/dt dependence), documented.
