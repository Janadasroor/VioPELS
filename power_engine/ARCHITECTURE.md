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
  - Switch: resistor `closed ? Ron : Roff` (ideal), or a slew-limited
    geometric sweep over `tsw` after a gate toggle (PAT-style behavioral
    edge: constant ratio per step; linear-in-R would jump decades on step
    one and re-excite the ringing). Transition starts in the toggle step
    itself (frac = 0, no ideal-commutation violence first); mid-ramp
    retoggles restart from the present R; turn-off tails start at ramp
    completion. Ramp v*i books into conduction loss exactly — never
    combine with Eon/Eoff tables on the same edge. `tsw` in addSwitch +
    netlist `TSW=` (direct/MODEL, round-trips). Matrix signature covers
    the ramp state (transT/From/To); snapshot/restore carries it.
    Diode conducting: Norton
    `G=1/Ron || Isrc=G*Vf` (so `Vd=Vf+I*Ron`); blocking: `Roff`.
    Factorization caching (14b): the MNA matrix depends only on the
    topology signature (switch/diode states incl. diode-recovery-active,
    R/L/C/k values, dt) — histories, source values and recovery currents
    live in the RHS. Unchanged-signature steps skip A-reassembly and
    refactorization (`factorSkips`); saturable-Newton circuits bypass.
    Full frames pass exactly `baseDt` (never recomputed `(t+dt)-t`, whose
    1-ulp jitter would defeat the cache). Factorization objects are
    per cache slot (one per integrator stage): a hit reuses slot factors
    regardless of what the shared workA_ scratch currently holds.
  - Integrator option (item 16, opt-in): `Trapezoidal` (default, 2nd-order)
    or `TrBdf2` (trap half step + BDF2 full step, 2nd-order L-stable —
    damps switch-excited Nyquist ringing ~1e65x on the interrupt fixture).
    BDF2 companions for C/L/coupled/sat (Newton included, frozen midpoint
    flux); midpoint history fields on Device; stage salt in the cache
    signature;     `.tran dt tstop [TRAP|TRBDF2]`; `Engine::setIntegrator`.
  - Auto stiffness switching (opt-in): trap while smooth, TR-BDF2 on
    diode-iteration pile-up (>=6 resolves/64 steps; normal clusters peak
    at 3, coarse stress at 13), Newton strain (>=8/step), or sustained
    Nyquist alternation (8+ consecutive same-node flips above 5% of node
    level with non-decaying amplitude — decaying commutation bursts,
    measured to 107 flips at 0.002%, are ignored); back to trap after 512
    quiet steps. Histories shared, exact mid-run switching; committed
    steps only (adaptive trials excluded).
  - After each solve, diode Vd/Id are checked and flipped
    states trigger an immediate re-solve at the same time point (up to 10
    iterations). Chatter suppressed with hysteresis (on when `Vd>Vf+1nV`,
    off when `Id<-1nA`). `SolverStats{steps,diodeEvents,resolves,
    sparseSolves,newtonIters,factorSkips}`.
  - Transformer: branch currents Ip/Is with rows `Vp-n*Vs=0`,
    `n*Ip+Is=0` (KCL coupling kept; ideal algebraic, DC passes — no
    magnetics/saturation by design).
  - Solved per-step with `Eigen::PartialPivLU` (dense) / `SparseLU`
    (rows >= 64, symbolic pattern cached); singular → throw. Cached
    factors are reused across unchanged-signature steps (see above).
  - Histories (incl. switch/diode/transformer branch currents) updated once
    per accepted step; deterministic, single-threaded.
- `src/api/engine.cpp`: owns `Circuit` + `TransientSolver`, exposes
  `Solution{t, probes{"v:N"[, "tj:dev"]}, states}`, step callback,
  `setSwitch()` (next-step effect) plus `scheduleSwitch(name,closed,at)`:
  exact-time edges split `step()` into sub-steps
  (`dt_sub = t_event - t_now`, companions use the sub-step dt);
  diode iteration runs inside every sub-step; nominal dt restored after;
  `step()` clamps to `tStop`. Hot-loop de-churn: probe node ids + key
  strings are snapshotted at `start()` and probes updated in place (no
  per-step `nodes()` set+sort, `to_string`, or map reinsert); device
  lookup is a lazy name->index map; the diode scan is skipped when the
  circuit has no diodes. Device *values* may change mid-run (load
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
  - Breadth (one test per block): `HysteresisController` (latching
    bang-bang, ON when low — the complement of Comparator), `StateMachine`
    (dwell-gated transitions for sequencers/protection), `HalfBridgeDriver`
    (complementary pair with deadtime + schedulable exact edges; short
    on-intervals blank that side), `Svpwm` (sector/dwell math + symmetric
    7-segment states; 3-phase sim validates 27V fundamentals at 0/-120/
    +120deg), discrete filters (`lowPass`/`highPass` Tustin 1st-order,
    `movingAverage` FIR). Lessons: sim dt must resolve the shortest
    switching segment (midpoint sampling misses interior edges otherwise);
    SVPWM meter references follow the inverter cosine convention.
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
- Electro-thermal stepping (`electrothermal.h`): timescale-separated
  fixed point. Within-step iteration is unnecessary (one sub-step moves Tj
  by ~P*dt/C, so explicit coupling is consistent to that order); the
  averaged-loss outer loop (electrical window -> P_avg -> macro thermal
  step -> Tj override, repeat to |dTj| < tol) reaches thermal equilibrium
  without brute-force warm-up. `setJunctionTempOverride` pins Tj (wins
  over attached networks; never combine both). Validated: efficiency
  droop vs ambient (25/85/125C) within 5% of the coupled closed form;
  brute-force transient agrees with the outer loop.
- Machines (`machine.h`, `src/machine/`): exact J/B mechanical stepping
  (closed-form, B=0 branch); sinusoidal PMSM via phase-domain back-EMF
  sources (value-only drive) + Park transform + power-balance torque
  (exact, no speed singularity), co-simulated per electrical step
  (timescale-separated explicit coupling, same philosophy as thermal).
  Voltage-form FOC: Clarke (control.h) + inverse Park + FocController
  (speed PI/FF -> iq*, dq PIs + decoupling, id* = 0, carrier PWM,
 validated motor params at construction); ideal theta (no
  encoder/observer).
  Hysteretic id=0 drive (self-commutated by construction) with speed PI +
  velocity feedforward + setpoint ramp (no windup). Lessons: PI gains must
  be designed against the real loop (sluggish ki fails load recovery,
  hot ki rings — feedforward fixes ramp lag honestly); sim dt must
  resolve hysteresis crossings with margin (overshoot = slope*dt << band).
  Motor-drive demo prints speed/iq/torque CSV.
- Hysteresis material (`magnetics.h`, `src/magnetics/`): saturating-offset
  memory B(H) = Bs*tanh((H-s)/a), s in [-Hc,+Hc] tracking +H. Major limbs
  ride bare shifted branches (CCW = positive loss); nested loops close
  exactly; saturation wipes outer memory; lens-shaped minors; Hc=0 is
  anhysteretic. Rate-independent (no dt). Validated: coercivity sides,
  remanence, saturation, area vs exact 4*Hc*B integral (2%), periodicity,
  step-invariance. Lesson: naive shifted-tanh with the WRONG SIGN gives a
  mirrored (clockwise, negative-area) loop — pin coercivity sides AND area
  sign, not just magnitudes. Classical eddy density alongside; reluctance
  network + winding interfaces are part 2.
- Reluctance network (`magnetics.h` ReluctanceNetwork): nodal magnetic
  domain (ground = reference, MMF Nortonizes without extra unknowns),
  saturable tanh branches via fixed-point iteration. Winding interface =
  Norton companion (R = 2*Lt/dt + history source, mirroring solver stamps;
  exact for linear, tangent per step when saturating). Lesson: series
  voltage-source back-EMF with 1-step lag is violently unstable for stiff
  inductive coupling (gain L/(R*dt)) — never use it. Buck via network
  matches plain-L buck.
- Sweep harness (`sweep.h`, `src/sweep/`): parameter grid × netlist runs
  (loadNetlist + setParameter re-elaboration per point), setup callback
  for wiring (PWM/thermal/loss specs, SolutionCallback windowed stats)
  and measure callback for outputs, per-row error capture (or
  stopOnError), CSV table. Threaded (`jobs`: 1 serial, 0 auto, N workers;
  rows by index so output is bitwise identical; callbacks must be
  thread-safe — no shared mutable accumulator). Monte Carlo tolerance
  analysis rides the harness: deterministic splitmix64 uniform/gaussian
  samplers fill axes, columnStats/yieldWithin summarize output columns.
  Lessons: cumulative loss energies include
  startup transient (edge-counted switching is transient-free; compare
  conduction by shape/monotonicity or POP-init); light-load buck runs
  DCM (Vout rises — use measured Vout in analytics, never ideal D*Vin).
- Induction (`machine.h` induction section): dq synchronous-frame flux
  model (RK4) + exact steady-torque equivalent circuit as reference.
  Rotating machines use dq ODEs (position-varying mutuals have no MNA
  companion); switching drives couple through ideal pole voltages +
  Park (no MNA needed for stiff-DC inverters). V/f needs no feedback;
  loads must stay below locked-rotor torque to self-start; analytic slip
  inverts Te(s) = Tload + B*w (friction matters at light load).
  - Numerical notes: SI/double/seconds; ground `0`; companions from `ic`;
    thermal states reset at `start()`; loss accumulators rebuilt at `start()`.
  - Event sub-stepping hardening: arbitrary-duty PWM edges land at arbitrary
    sub-step alignments; picosecond slivers used to explode capacitor
    companions and false-trip the singularity guard, and the first guard
    version could spin forever with no event pending. Rule now: imminent
    event fires (<=1ns early) else the residual steps normally (companions
    safe at ns scale); near-boundary residuals extend past it. Event
    scheduling is append-ordered O(1) (binary insert fallback), stable for
    equal times.
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
- EMI screening (`emi.h`, `src/emi/`): DC LISN builder, peak table at
  switching harmonics (coherent DFT — exact for stationary line spectra),
  CISPR 32 Class B QP table + margin check. Pre-compliance screening
  only (single port, stationary, peak-vs-QP); needs dt << tsw.
- FMI 2.0 co-simulation export (`fmi/`: vendored headers, generic wrapper
  TU compiled per FMU with FMI2_FUNCTION_PREFIX, `fmi_pack.py`,
  `ctypes_check.py`, vendored XSD): netlist + io map -> .fmu; inputs are
  source devices, outputs are probes; horizons master-driven; GUID/state
  machine enforced, unsupported calls honestly fmi2Error; engine is
  non-assignable so instances live in unique_ptr.
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
- Multitone + impulse (`multitone.h`, `src/multitone/`): single-sim Bode  from prime-multiple tones on integer window bins (no harmonic overlap),
  Schroeder phases + numeric crest rescale, per-tone AGC, one FourierMeter
  per tone; ring-down fit (zero-cross freq + log-decrement tau) for loop
  characterization. Validated vs plant model and stepped-sine spots; LC
  ring values match. Lessons: Schroeder is near-optimal only for harmonic
  combs (partial for log-spread — the numeric rescale carries the headroom
  guarantee); ring fits need ripple pre-averaged (per-cycle means).
- State-space export (`statespace.h`, `src/statespace/`): continuous-time
  (A,B,C,D) per frozen switching state from direct nodal assembly —
  algebraic/dynamic node partition, Kron-style elimination with Eigen,
  states = inductor currents + dynamic-node voltages, inputs = grounded
  sources + const-1 (diode Vf affine). Restrictions (thrown): no floating
  V-sources, no magnetics, no active recovery, nonsingular algebraic and
  capacitance blocks. Duty averaging + textbook duty-to-output transfer
  (Erickson §7.3 operating-point sensitivity). Validated: averaged buck
  reproduces the lossy second-order model within 0.5%/0.5deg;
  eigenvalues are the LC poles.
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
  - Mains topologies (tests + CSV demos): full-bridge SPWM (unipolar,
    differential fundamental = m*Vdc), 3-phase 2-level SVPWM inverter with
    3-wire star RL (current = Vph/|Z|, power closes), Vienna diode bridge
    (Vdc = 1.35*Vll minus overlap/drops).
  - Resonant/duals (tests + CSV demos): dual-active-bridge SPS ±30deg
    (power both directions at the coupled Vout=V1*K*R operating point, not
    the stiff-bus formula), LLC series-resonant (FHA gain at/below
    resonance vs coupled-inductor transformer model).
  - LESSONS (3-phase mains, each verified the hard way): the mains neutral
    MUST float (true 3-wire) — grounded neutrals + grounded dc- give lower
    diodes a zero-impedance ground return that shorts phases (~400A latch,
    bridge reads ~400V instead of 540V). Diode bridges bootstrap from EMPTY
    caps (pre-charge above commutation level blocks uppers and latches
    lowers). Vienna PFC is CLOSED-LOOP regulated (750Vdc, PF 0.99,
    vienna_pfc.h + vienna_pfc_demo): carrier average-current-mode holds Vdc
    and PF but the midpoint sits at dc- (measured Vmid ~0V) — duty-side
    balancing is STRUCTURALLY futile in 3-wire (common-mode duty shifts are
    KCL-null, T*d0*SUM(i)=0; differential trims are disturbance-rejected by
    the current-loop integrators; reference shaping is confined to SUM=0).
    Regulation and balance DECOUPLE (PF 0.97 WITH collapse). Balance needs
    an architecture with no tracking integrator in the current path:
    hysteretic control with sign-independent threshold shift
    s = alpha*(Vc1-Vc2) holds Vmid = Vdc/2 within 0.3V against a 0.75A
    midpoint bleed + asymmetric caps (both current polarities cooperate:
    thresholds-up lengthens midpoint clamping for i>0 and shrinks the ON
    region for i<0).
  - Coupled inductors: trapezoidal 2-port Norton from L*i flux linkage,
    k<1 enforced (k=1 singular → use Transformer); netlist `W` device.
  - Adaptive stepping (opt-in): step-doubling on node voltages, order-2
    controller, save/restore trials, floor-accept with growth recovery;
    exact landing via stepTo across event-free intervals. Note: error
    estimates are invalid across step discontinuities (e.g. t=0 source
    application), handled by floor-accept + regrow; fixedStep's dt_
    side effect is fenced by the controller (both debugged, tested).
    TR-BDF2 uses embedded stage-difference control instead (one converged
    step per attempt vs three; composes automatically with setAdaptive).
    Commutations bypass error control in both controllers (measured: err
    pins at ~1 from dtMax to dtMin across an event straddle — accept at
    current dt, no shrink ratchet; SPICE-LTE-bypass principle). Singular
    factorizations inside attempts escalate dt upward (shrinking worsens
    companion spread) with bounded retries, then rethrow honestly
    (SingularError, still a runtime_error for existing catch sites).
  - Saturable inductor: λ(i)=Lsat*i+(Lunsat-Lsat)*Isat*tanh(i/Isat),
    Newton loop around the MNA solve (≤25 iters, honest throw on
    non-convergence); netlist `Y` device.
  - Reverse recovery / tail: diode triangular Qrr (impressed current —
    a parallel Ron would swallow it under reverse bias), switch
    exponential tail; recE=Qrr*|V| booked into losses+thermals at release.
    Fixed-shape approximations (no di/dt dependence), documented.
