# Power-engine capability roadmap

Status basis: commit `dd65c6a` (Phase 0). Read with `refine-roadmap.md`
(hardening track) and `power_engine/ARCHITECTURE.md` (implementation
notes). Goal: a reference-class, headless power-electronics circuit
simulator — robust, fast, scriptable, and trustworthy, with better
physics than generic SPICE where it matters and validation to prove it.

Non-goals (explicit): no GUI, no Simulink/blockset integration, no HIL
hardware support, no embedded code generation, no third-party thermal or
semiconductor data licensing.

## Strengths to protect (do not regress)

- Fixed-step TRAP/TR-BDF2 trapezoidal MNA with exact-time gate edges via
  event sub-stepping; opt-in TR-BDF2-AUTO stiffness switching; opt-in
  adaptive control (step-doubling for trap, embedded stage-difference
  for TR-BDF2); L-stable only at TR-BDF2 today.
- Factorization cache (FNV signature, per-integrator-stage slots);
  dense `PartialPivLU` / `SparseLU` at >= 64 rows with cached symbolic
  pattern; ~97–100% cache hits on the buck/Vienna fixtures.
- Electro-thermal engine: exact ∫v·i conduction loss booked per
  sub-step, Eon/Eoff impulses on gate edges, `tj:` probes, Foster
  (exact update) / Cauer (explicit Euler), `runToThermalSteady`
  averaged-loss outer loop.
- Loss tables Eon/Eoff(I,V,Tj) 3-D, multilinear + edge clamping;
  Ron(Tj)/Vf(Tj) one-way refresh.
- Magnetics: Preisach-memory hysteresis core (explicit companion),
  nodal reluctance networks, gapped-inductor synthesis (N/gap/fringing/
  Dowell/window fill), Tj-aware copper & Bsat bounds, eddy loss.
- Drives: voltage-form FOC + SVPWM seven-segment, field weakening +
  MTPA, sensorless observer with I-f startup, hysteretic id=0 drive.
- Analyses as engine code: periodic shooting steady state (Newton on
  the period map, backtracking), AC Fourier meter with exact DC
  rejection, multitone, loop gain (series injection), state-space
  export + `LinearStepper` fidelity proof.
- Ecosystem: threaded grid sweeps with deterministic splitmix64
  Monte Carlo + yield/stats (bitwise-identical across job counts);
  FMI 2.0/3.0 co-simulation export (XSD-validated, ctypes-proven) and
  2.0 CS import; libFuzzer target; 93.96% line coverage; ngspice
  cross-validation (5e-4…1.5e-3 settled means); CI: 12 jobs.
- Trust boundaries R1–R9 (same-node rejection, generation counter,
  sweep thread-safety, parser caps, CSV contract).
- EMI pre-compliance screening (DC LISN, CISPR 32 Class B margins).

## Known constraints (decisions, not oversights)

- KLU declined below n≈2000 (Eigen faster on our matrix class);
  revisit only with a >2000-row switched fixture.
- Matrix partitioning declined (O(n^1.2), domain stays <100 nodes).
- Diode iteration converges in a few passes by construction; the
  10-iteration cap stays a safety net (`diodeCapHits` monitors).
- Variable dt defeats the factorization cache (dt is in the FNV key);
  variable-step paths refactorize per attempt.
- Hysteretic circuits reject shooting and state-space export by
  design (memory has no slot kind) — documented, keep it.
- Breakpoint/zero-crossing relocation declined before with evidence
  (straddle already mV-accurate); re-opening requires Phase 2
  evidence, not intuition.

## Phase 0 — credibility (DONE, `dd65c6a`)

- [x] MSVC-conformant FMU stub (unnamed parameters rejected with
  C2055); the sole red CI lane on `main`.
- [x] README/CHANGELOG counts synced to reality (252 tests / 28
  suites); unverifiable "all green" claims removed.
- [x] 28/28 suites green locally (GCC 15, Release).
- [ ] Future: a CI lint step that fails when README/CHANGELOG quote
  stale test/suite counts.

## Phase 1 — solver credibility (~6–8 weeks)

Goal: variable-step timestepping exists and is trustworthy; analyses
are usable tools, not just library calls. Order by risk: tooling first,
new integrator second, scale benchmark third.

### 1.1 `pe steady-state` (S, ~0.5–1 wk)

Subcommand wrapping the periodic shooting harness for CLI use.

- Flags: `--netlist x.net [--period 50u | --auto] [--pop] [--rel-tol
  1e-6] [--periods N] [--out orbit.csv]`; `--auto` derives the period
  from the fastest scheduled source/PWM; `--pop` pre-cycles to the
  DC operating point before Newton (reuse the POP practice from the
  AC lessons).
- Output: `converged iters residual` line; optional CSV of the steady
  orbit (propagate exact interpolation already in `measurements.h`).
- Reuse: `solvePeriodicSteadyState` as-is; no physics changes.
- Exit criterion: buck + a PFC demo converge to |residual| < tol;
  new CLI test proves the harness end-to-end in-process.

### 1.2 `pe ac` (M, ~1–2 wk)

Perturbation-based Bode directly from the CLI.

- Seed: run `--pop` to periodic steady state, then perturb.
- Flags: `--netlist x.net --probe v:3 [--vin-ac duty|voltage|current]
  --from 1 --to 1e5 --points 41 [--perturb abs|rel] [--method stepped|
  multitone] [--out ac.csv]`.
- `stepped`: one simulation per frequency (exact, slow). `multitone`:
  prime-bin sum with Schroeder phases, one simulation, one
  `FourierMeter` per tone (the `multitone.h` machinery is built for
  exactly this).
- Output: `freq,mag_dB,phase_deg` CSV; stdout summary line with DC
  gain / crossover.
- Exit criterion: RC Bode within 0.5 dB/4° of theory; buck Gvd within
  1.5 dB of the second-order plant model; CLI test on both.

### 1.3 DOPRI 5(4) adaptive integrator (L, ~2–3 wk)

- Embedded RK pair: 5th-order DOPRI used for propagation, 4th-order
  for error estimation; controller reuses the `AdaptiveConfig`
  clamp/grow/accept machinery from the step-doubling path.
- Event straddle LTE-bypass ports over unchanged (measurements already
  show err pinning to ~1 at event straddles — accept at current dt).
- Netlist: `.tran dt tstop [TRAP|TRBDF2|AUTO|DOPRI]`; `Engine::
  setIntegrator("dopri")`.
- Factorization: dt is in the FNV key ⇒ near-zero cache hits;
  refactorize per attempt (dense ≤ ~200 nodes fine; sparse path at
  >= 64 rows with cached pattern is reusable unchanged). Drop the
  FNV recompute for this integrator.
- Output density stays user-controlled (`OutputTimes`-style refine
  factor later if needed).
- Exit criterion: ngspice cross-validation parity on buck + Vienna
  fixtures at comparable or better step counts; no regression in the
  252-test suite; `BASELINE.md` entries (or explicit "cache bypassed"
  note); perf documented against trap.

### 1.4 Damped Newton in the transient path (M, ~1 wk)

- Add backtracking line search on the Newton residual norm inside the
  saturable-loop (25-iter cap unchanged); if the residual fails to
  decrease after k backsteps, fall back to the last accepted stage
  and count `lineSearchFailures` (new stat, honest throw on final
  failure).
- No change to accepted inputs; regression test on the hardest
  fixture (Vienna diode bridge).

### 1.5 Switched-sparse scaling benchmark (S, ~0.5–1 wk)

- New fixture family: passive ladders scaled to n ∈ {100, 200, 500,
  1000, 2000} **with 50+ switches** injected (cache invalidation
  pressure, unlike the current passive-only ladder fixtures).
- Report per-step cost, refactorize rate, cache hit-rate, wall-clock.
- Output: table in `power_engine/bench/` + decision note: if the
  switched curve stays sub-cubic to n=2000, KLU stays declined; else
  reopen with data.

**Phase 1 non-goals:** no Radau yet (plumbing first, core second); no
zero-crossing relocation (needs Phase 2 evidence); no device-level
changes.

## Phase 2 — implicit variable-step core (~10–12 weeks)

The centerpiece. DOPRI covers non-stiff workloads; the rest of the
desert is stiff circuits (fast diodes, tiny time constants, breakpoint
density).

- `Integrator::RadauIIA`: 3-stage, order 5, L-stable; continuous
  extension for dense output so `runUntil`/probes land exactly.
- **Analytical Jacobian**: ∂F/∂x assembled from the closed-form
  companions — L/C/coupled/saturable/hysteresis branches carry
  explicit dI/dV terms; switches/diodes contribute state-dependent
  Norton terms. Finite differences remain only in shooting.
  This removes the FD cost and the Newton-strain blind spots that
  today trip the AUTO stiffness detector.
- Newton outer loop with line search / trust region; singular
  factorization escalation path kept (grow dt, bounded retries).
- **Zero-crossing event location, re-justified**: sign-track the
  natural-commutation probes (diode `vd−vf`, switch `vce`, coil
  current) every step, bisect (≤4–6 iterations) to the crossing, step
  just after the event; bypass LTE control on the straddling step.
  Evidence required to close the Phase-0-era decline: the straddle
  re-solve is no longer mV-accurate once Radau's error control is
  the definition of "converged".
- Factorization cache: per-attempt refactorize; FNV signature kept
  only for the (rare) repeated-signature case in multi-stage schemes.
- **Gates**: xval parity at equal or better step counts on all
  fixtures; 252-test suite green; per-fixture BASELINE.md update;
  AUTO detector retired for Radau runs (Radau *is* the stiff solver).

Risk notes: Radau + trapezoidal histories don't compose trivially
(needs its own stage-state plumbing, modeled on the BDF2 path);
continuous extension must keep the exact-time event sub-stepping
contract (`dt_sub = t_event − t_now`, sliver guard unchanged).

## Phase 3 — device switching fidelity (~8–10 weeks)

Every item ships behind a `.model`/`MODEL=` attribute with a fixture
test; none changes default behavior of accepted netlists.

- **Coss (S/M)**: linear output capacitance first; then `COSS_TABLE`
  (V → C) nonlinear stamp with stored-energy E_oss = ∫ ½C(v)v² booked
  into losses at each edge. Closes the "no device-level dV/dt fidelity"
  gap; improves EMI screening credibility.
- **Channel/body conduction split (M)**: MOSFET gate-dependent split
  (channel R_ON(i) vs body diode) — loss accounting separates the
  two paths, matching how real devices heat differently.
- **Paralleling N identical devices (M)**: Ron → Ron/N, loss tables
  divide by N, thermal impedance scales 1/N; documented equal-current
  assumption.
- **di/dt-aware Qrr (M)**: recovery shape parameterized by current
  slope rather than the fixed triangle; trr/Qrr bounds enforced.
- **Bidirectional Ron(Tj)/Vf(Tj) (M)**: per-sub-step fixed-point
  (Jacobi, ≤4 iters) replacing today's one-way refresh; conduction
  loss stays exact ∫v·i.
- **Sub-cycle averaged semiconductor mode (L)**: duty-cycle-averaged
  companions for switch/diode pairs on long-horizon thermal/efficiency
  runs (orders of magnitude faster, slightly lossier volt-second
  accuracy at current zero-crossings — documented).

## Phase 4 — thermal breadth + tooling surface (~8 weeks)

- **Heat sink / isotherm enclosure model (S)**: one node enforces a
  common temperature across enclosed components; propagates to its
  own thermal network.
- **Junction↔case interface resistance (S)**: extra Cauer R stage or
  a dedicated `.thermal interface=` directive.
- **All-device coupled thermal DC analysis (M)**: generalize
  `runToThermalSteady` from per-device loops to a coupled
  multi-device solve (one Newton across all Tj) for initial
  temperatures before transient runs.
- **`pe thermal-fit` (M)**: CSV heating curve → Foster coefficients
  (nonlinear least squares, 3–5 elements typical) and Cauer reduction
  with order diagnostics; consumes vendor-supplied transient curves
  the user already has; no data redistribution involved.
- **JSON-RPC localhost server (`pe serve`) + minimal Python binding
  (M)**: `simulate` / `get` / `set` / `sweep` semantics mirroring the
  CLI, so automation scripts and test harnesses can drive the engine
  in-process. Deterministic (same seeds → same bits).
- **FMI auto io map (S–M)**: derive `--inputs` from source devices and
  `--outputs` from probes; manual override stays.

## Declined / scoped out (with reasons, revisit conditions)

| Item | Reason | Reopen condition |
|---|---|---|
| KLU refactorization | Eigen ≥ KLU below n≈2000 on our class | Phase 1.5 shows switched n>2000 |
| Matrix partitioning | O(n^1.2), domain <100 nodes | n>~500 switched domains |
| Multi-chip thermal impedance matrices (Z_ij) | Needs data partner | Module vendor engagement |
| Vendor thermal-model XML import | Data distribution question | Partner signed |
| FMI 3.0 import, FMI ME export | Demand unproven; XL | Documented customer ask |
| Zero-crossing relocation (pre-Radau) | Straddle already mV-accurate | Phase 2 evidence |
| HIL / RT-box class codegen | Out of scope | Never in this program |
| GUI / Simulink blockset | Out of scope | Never |

## Metrics & exit gates (every phase)

- `ctest`: 100% pass, warnings-as-errors unchanged.
- ngspice cross-validation: no regression in fixture bounds.
- `BASELINE.md`: hashes updated with justification or unchanged.
- README/CHANGELOG counts updated in the same commit.
- Trust boundaries: new untrusted inputs validated at the boundary
  with throwing messages; hot loop never slowed for checking.
