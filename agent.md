# agent.md — instructions for AI coding agents in VioPELS

## Build / test (run these; never guess results)

```sh
cmake -B build -DCMAKE_BUILD_TYPE=Release   # Release is default; -O0 Eigen is ~17x slower
cmake --build build -j4
ctest --test-dir build --output-on-failure  # 12 binaries, ~2s in Release
```

- Warnings are errors: `-Wall -Wextra -Wpedantic -Werror` (`/W4 /WX` on MSVC).
- Sanitizer lane (mirrors CI): `-DCMAKE_BUILD_TYPE=Debug
  -DPOWER_ENGINE_SANITIZE=address,undefined`, then `ctest` (~20s).
- Ninja works: `-G Ninja`. Matrix + sanitizer + Ninja jobs live in
  `.github/workflows/ci.yml`.
- After touching the solver or engine, always run the FULL suite, not just
  the new test — phases interact (diode iteration, histories, losses).

## Code conventions

- C++20, `pragma once`, `namespace power_engine` (+ `netlist`, `control`,
  `thermal`, `ac` sub-namespaces). snake_case files, CamelCase classes.
- SI units, seconds, `double`. Ground node is always `0`. No raw `new`/`delete`.
- Invalid topology/params → `throw std::runtime_error` with a message
  (never silent ignores, never clamp silently where physics forbids:
  e.g. coupled `k=1`, `Qrr` without `Trr`).
- Public headers in `power_engine/include/power_engine/`; everything else
  private under `src/{core,solver,api,netlist,control,thermal,ac}/`.
- Keep `README.md` test counts, `ARCHITECTURE.md` design notes, and
  `roadmap.md` checkboxes in sync with every change.

## Architecture map

- `src/core/circuit.cpp` — device container, validation, node enumeration.
- `src/solver/solver.cpp` — trapezoidal MNA core. Hot loop uses reused
  buffers + precomputed rows + PartialPivLU (<64 rows) / SparseLU (>=64).
  Diode iteration, Newton loop (saturable L), recovery states all live here.
- `src/api/engine.cpp` — owns `Circuit` + solver; exact-time `scheduleSwitch`
  sub-stepping, loss/thermal accumulation per sub-step, netlist elaboration.
- `src/netlist/`, `src/control/`, `src/thermal/`, `src/ac/` — one domain each.
- Tests mirror sources: `test_rc/rlc/switch/buck/netlist/control/thermal/
  converters/magnetics/adaptive/recovery/ac`.

## Hard-won rules (violating these caused real multi-hour debugs)

- AC measurement: settle in `max(periods, circuit Taus)`; pre-charge to the
  DC operating point; AGC the perturbation (`duty*|G|<=5%`); cap finj below
  fsw/2; per-step (ZOH) drive has an exact −ωdt/2 footprint — oversample
  high-f points (`f*dt<=0.01`).
- Reference frames: open-loop analytic duty has NO sampler+ZOH (plant-only
  Gvd); input samples belong to interval starts, states to ends — keep the
  association explicit in any new correlation code.
- Device values may change mid-run (load steps); topology may not (solver
  throws; tested).
- Diode recovery must be an impressed current (parallel Ron would swallow it
  under reverse bias); switch tails keep `Roff || source`.
- Adaptive stepping: error estimates are invalid across discontinuities
  (floor-accept + regrow); `fixedStep`'s `dt_` side effects belong to the
  controller — see ARCHITECTURE before touching.
- MOSFET/IGBT gates are logic-level (VTO recorded, not simulated);
  transformer is algebraic (DC passes); coupled `k<1` strictly.

## Git rules

- Do NOT commit, amend, push, or open PRs unless explicitly asked.
- Do NOT touch git config. `build*/` and friends are git-ignored; never
  force-add them.
- Keep the working tree clean: no scratch files in the repo (use `/tmp`),
  no edits to generated output.
