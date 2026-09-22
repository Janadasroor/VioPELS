# VioPELS Refine Roadmap (robustness track)

Runs in **parallel** with `roadmap.md` (feature track). This track changes no
accepted-input physics: every item is hardening with a regression test.
Status basis: full exploration + E2E run on 2026-09-22 — Release build green,
**24/24 suites**, `rc/buck/foc/emi/sweep/vienna` demos exit 0, 9 failure-path
probes thrown correctly except §B-R1 below.

## A. Trust boundaries (inventory — do not weaken)

1. **Netlist text → `Parser::parse`** (`src/netlist/parser.cpp:804`): UNTRUSTED.
   Contract: any invalid input throws `runtime_error` with `netlist line N:`
   (`parser.cpp:28-30`); throws-are-rejection is the fuzzer contract
   (`fuzz/fuzz_netlist.cpp:22-24`). Case-insensitive, `*`/`;` comments,
   `+` continuations, SI suffixes, `{expr}`, `0/GND/GROUND` aliases,
   `Xname:` subcircuit scoping, locked `.param` overrides.
2. **Public C++ API → `Circuit`/`Engine`** (`src/core/circuit.cpp:42-189`,
   `src/api/engine.cpp`): validates nodes/positivity/finiteness/names
   (`Roff>Ron`, `0<k<1`, `Lsat<=Lunsat`, `Qrr→Trr`), dt/t/tolerances,
   lifecycle order (`loadNetlist` before `start`, `setParameter` before
   `start`, `runUntil` while running, no `tStop` set). Wrong-type access
   (`setSwitch` on R, `deviceLoss` on wrong type) throws.
3. **Solver internals** (`src/solver/solver.cpp`): ASSUMED, solver-owned.
   `Device` fields mutated directly via `mutableDevices()` (ron/roff/vf,
   histories, flux, timers) with no re-validation; source values bypass
   `matrixSig` (`solver.cpp:515-517`); `restoreState` checks sizes only;
   FNV cache assumes no collision; `1e-9` diode hysteresis / `64eps` pivot /
   `1ns` sliver are tuned constants.
4. **FMI 2.0 C ABI** (`power_engine/fmi/`): HOSTILE. GUID/VR/state-machine
   enforced, `startTime!=0`/tComm-skew/non-finite-SetReal rejected,
   unsupported (derivatives, FMU-state, ME) honestly `fmi2Error`;
   engine exceptions never cross the ABI (caught → log + `fmi2Error`).
   Proven by `ctypes_check.py` negative tests + CI `fmi-smoke` (XSD-validated).
5. **Sweep threading** (`sweep.h:46-55`): per-point local `Engine` is safe,
   but `setup`/`measure` captures MUST be thread-safe when `jobs!=1`.
   `csv()` is bitwise-identical across job counts; `stopOnError` rethrows
   post-join in threaded mode (vs immediate serial).
6. **Demo/CSV/Python edges**: demos use `probes.at(...)` (throws if schema
   drifts); `xval.py` uses `header.index(col)` + `float(...)` (throws
   `ValueError` on drift); `ngspice` output skips unparsable lines silently.
7. **External tools** (not our code, CI depends on them): `ngspice` (xval),
   `zip/unzip` (FMI pack), `xmllint` (XSD gate), Eigen headers (configure),
   Clang (fuzz). Each has exactly one failure exit path — see §B.

## B. Failure scenarios (E2E-verified 2026-09-22)

Handled correctly (regression-anchored, do not regress):
- Floating-node / V-source-loop → `SingularError` at step
  (`solver.cpp:414,429`); adaptive retries 2×dt ≤5× then rethrows.
- Newton non-convergence → `runtime_error` after 25 iters (`solver.cpp:861`).
- Topology add/remove mid-run → throw on size change (`solver.cpp:1053`).
- All 9 parser misuse probes (div-zero expr, unknown subckt/device,
  missing `.ends`, shorted V-src, `k=1`, `Roff<=Ron`) throw with line info.
- All FMI misuse sequences (pre-init `DoStep`, output-VR `SetReal`,
  bad VR/GUID/tComm, ME type, NaN step) rejected + covered in CI.

Gaps found (the refinement backlog §C):
- **R1 (found today, live-probed): same-node no-op devices silently accepted.**
  `addResistor("R1",3,3,100)` does not throw (`circuit.cpp:11-12` rejects only
  `node<0` and *both*-ground); the device stamps nothing and the sim runs on
  as if it weren't there (probed: `v3=5` from the Vsrc, R dead weight).
  Same hole shape likely in C/L/switch/diode adds. Severity: low (no crash,
  no wrong numbers — just a silently ignored device), fix is cheap.
- **R2 (in-tree contract violation): `sweep_demo.cpp:27-47` shares a mutable
  `Acc` across `setup`/`measure`/callback** — safe at default `jobs==1`,
  data race at `jobs=0|N`, violating `sweep.h:50-53`. Demo-only, but it is
  the copy-paste template for every user.
- **R3: `Parser` has no input-size cap** (fuzzer caps at 4096B, direct API
  callers unbounded); deep `((((...))))` recursion, wide subckt fan-out,
  giant `.etable` grids allocate before validation (only guard: 1M node
  limit `parser.cpp:262`, 16-deep nesting `parser.cpp:542`).
- **R4: diode iteration silently accepts the last state** when `kMaxDiodeIters`
  (10) saturates — no stat, no warning; `autoUpdate` latches BDF2 on
  `trigResolves6` so chronic chatter hides as integrator switches.
- **R5: same-size topology swap mid-run aliases maps/cache silently**
  (only size is checked at `step()`; type/node swaps reuse factorization).
- **R6: xval/demo CSV coupling is stringly-typed** (`header.index` /
  `probes.at`); a renamed probe breaks xval with `ValueError`/`out_of_range`
  instead of a schema error naming the contract.
- **R7: `Engine::setTime` trusts caller to realign histories/events**
  (`solver.h:152-154` documents it, nothing enforces it).

## C. Refinement backlog (ordered by value × risk; parallel-safe)

- [x] **R1. Reject same-node devices** (`circuit.cpp:checkNodes` — DONE
  2026-09-22): `n1==n2` non-ground throws in all two-terminal adds + both
  transformer/coupled windings (single choke point); redundant V-source
  `np==nm` check removed (subsumed); `CircuitValidation.RejectsSameNodeDevices`
  covers all 10 add-forms; 24/24 green, all demos exit 0.
- [x] **R2. Thread-safe sweep template** (DONE 2026-09-22): `sweep_demo.cpp`
  shared-`Acc` replaced with `thread_local` per-point state (runPoint runs
  setup→run→measure atomically on one worker) + demo now runs `jobs=0` as a
  live threaded proof; threaded output byte-identical to the serial baseline
  across repeats. New `SweepThreads.WindowedMeanThreadSafe` proves csv()
  equality jobs 1/4/0 for the callback pattern; stale NOTE corrected.
  24/24 green.
- [ ] **R3. Parser hardening**: input byte cap + nesting-depth accounting
  for expressions, pre-allocation sanity on `.etable` grids; fuzzer dict
  extension (`.subckt`, deep parens). Tests: oversized/deep inputs throw
  fast (assert wall-time bound), corpus still clean.
- [ ] **R4. Diode-iteration observability**: `SolverStats` chatter counter
  (iterations-hit-cap events) + `bench/BASELINE.md` entry; no behavior
  change. Later: consider escalating chronic saturation to BDF2 hint.
- [ ] **R5. Topology generation counter**: `Circuit::generation()` bumped on
  any add/remove; `solver.step()` throws on mismatch even when sizes agree.
  Test: same-size swap throws instead of aliasing.
- [ ] **R6. CSV schema contract**: named `DemoCsv` writer/reader shared by
  demos + `xval.py` (header assert naming the expected probe, not
  `ValueError`); xval emits which side drifted.
- [ ] **R7. `setTime` guardrails**: document-or-enforce history/event
  realignment (at minimum: debug-mode assert that histories match `t`).
- [ ] **R8. Trust-boundary doc**: one `ARCHITECTURE.md` section pointing at
  §A (what is validated vs assumed at each boundary) so future modules
  inherit the discipline.
- [ ] **R9. Missing-tool UX**: single `tools.py`/shell preflight used by
  xval + FMI scripts (`ngspice/zip/xmllint/eigen/clang` → one clear error
  naming the install, same exit code everywhere).

## D. Critical flows to protect (regression anchors for both tracks)

Solve-step hot path (`convergeStep`→`assembleCached`→factors→diode loop);
`Engine` exact event sub-stepping + sliver guard; TR-BDF2/AUTO/adaptive
(incl. event-straddle accept + `SingularError` grow-to-escape); FOC loop
(Clarke→dq PIs→PWM→plant→mechanics); thermal one-way coupling + outer soak;
EMI screening (`dt<<tsw`, coherent DFT, Class-B check); sweep/MC determinism
(bitwise `csv()` across jobs, splitmix64 samplers); FMI pack→instantiate→
step→reset; xval settled-means vs ngspice (+ wall-clock table).

## E. Working rules (both roadmaps)

No physics change on accepted inputs without a benchmark/proof entry;
warnings-as-errors on all lanes; full `ctest` after solver/engine touches;
`README` counts + `ARCHITECTURE` notes + the touched roadmap's boxes move
in the same commit; `roadmap.md`/`agent.md` stay local-only
(gitignored) — never commit them. This file (`refine-roadmap.md`) is the
shared track and IS committed.
