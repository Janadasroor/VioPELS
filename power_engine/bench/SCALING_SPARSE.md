# Switched-sparse scaling (roadmap item 1.5)

Measured on the same machine class as pe_bench (gcc, Release, 2000 steps
@1us, switched RC ladder: source-driven, ~50 switch+diode shunt branches
toggling staggered across the window). Complements pe_bench's passive
ladder (dense/sparse switch at ~64 nodes) — this fixture injects gate-edge
signature churn, the load regime the passive ladder never exercises.

## Fixture

`power_engine/bench/scaling.cpp` — `pe_scaling` binary. RC ladder rungs
{100, 200, 500, 1000, 2000}; nodes 1..N; every 2nd node carries a
switch+diode shunt to ground, gates toggled at staggered times (~100
edges/run). `C=1uF` per node, R=1k series. Sizes >= 101 nodes use the
SparseLU path (>=64-row threshold) — every step solves sparse.

## Results (Release)

```
fixture       steps  wall(ms)  us/step  sparseSolves  factorSkips  hit-rate  v_end
switched-100    2000    23.1     11.54       2000         1891      94.5%    4.001
switched-200    2000    42.6     21.28       2000         1891      94.5%    6.142
switched-500    2000   132.8     66.40       2000         1891      94.5%    6.142
switched-1000   2000   371.7    185.83       2000         1891      94.5%    6.142
switched-2000   2000  1029.5    514.77       2000         1891      94.5%    6.142
```

Scaling per step vs nodes: 100→200→500→1000→2000 ⇒ 11.5→21.3→66.4→186→515
us/step — roughly 3x per doubling in the upper range, i.e. clearly
sub-cubic; sparse factorization dominates; forward/backward substitution
(the cached-pattern triangular solve) carries the rest.

Cache behavior under gate-edge churn: ~109 refactorizations per run
(100 event edges + start/restore), 1891/2000 hits — unchanged across
sizes. The constant-dt + gate-edge signature invalidator works as
designed; nothing about sparse-at-scale defeats the cache.

## Notes and decision

- Diodes never forward-biased in this fixture (monotonic positive drive;
  shunts clamp to ground) — diode iteration (`resolves`, `diodeEvents`)
  is unexercised here by construction; gate-edge signature churn is what
  invalidates the cache and is what was measured.
- Data keeps the KLU decline in place: the SparseLU path carries n=2000
  at ~0.5 us per node-step with no failure, no pathological fill-in,
  94.5% cache hits. KLU stays declined unless a future fixture shows
  the sparse solve exploding (e.g. fill-in beyond ~O(n^1.5) on a denser
  coupling pattern) — revisit with that evidence, not before.
- Partitioning stays declined: at n=2000 the run is sub-cubic and
  comfortable; no need for domain splitting.

## Reproduce

```sh
cmake -B build -DPOWER_ENGINE_BUILD_TESTS=ON \
  -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=<your googletest-src>
cmake --build build --target pe_scaling
./build/power_engine/bench/pe_scaling
```
