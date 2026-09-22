# pe_bench baselines (roadmap item 14a)

Machine: contributor workstation, gcc, Release. Timing is informational
(compare ratios on the same machine, not absolutes); checksums are the
waveform guard — any solver performance work (items 14b–d) must reproduce
them bitwise or within 1e-9 relative, or the "speedup" is a behavior change.

## v0.1.0 + factorization cache (14b) + hot-loop de-churn

De-churn: probe keys/nodes snapshotted at start() with in-place probe
update (no per-step nodes() set+sort, to_string, map reinsert);
lazy device name->index map (findDevice/deviceCurrent O(log D));
diode-state scan skipped when the circuit has no diodes. All checksums
bitwise identical to the 14b section (pure overhead removal).

```
buck-open-6ms      steps=12000   wall=    15.4ms us/step=  1.285 checksum=73604.5196971
vienna-60ms        steps=60000   wall=    91.3ms us/step=  1.521 checksum=28383342.7156
ladder-10          steps=2000    wall=     1.7ms us/step=  0.854 checksum=8534.87986295
ladder-40          steps=2000    wall=     7.3ms us/step=  3.650 checksum=8534.87986295
ladder-160         steps=2000    wall=    38.0ms us/step= 19.017 checksum=8534.87986295
ladder-320         steps=2000    wall=   125.0ms us/step= 62.499 checksum=8534.87986295
```

Per-fixture solver stats (new in pe_bench): buck resolves=239 (2% of
steps), vienna resolves=41 (0.07%) — diode iteration is NOT a speed
problem (see 14c note below). Cache hits: buck 97%, vienna/ladders ~100%.

## v0.1.0 + factorization cache (14b) + dt-exactness fix

```
buck-open-6ms      steps=12000   wall=    17.3ms us/step=  1.438 checksum=73604.5196971
vienna-60ms        steps=60000   wall=   167.7ms us/step=  2.795 checksum=28383342.7156
ladder-10          steps=2000    wall=     4.2ms us/step=  2.085 checksum=8534.87986295
ladder-40          steps=2000    wall=    22.9ms us/step= 11.426 checksum=8534.87986295
ladder-160         steps=2000    wall=   145.5ms us/step= 72.733 checksum=8534.87986295
ladder-320         steps=2000    wall=   318.4ms us/step=159.201 checksum=8534.87986295
```

Speedup vs pre-14b: vienna 1.54x, ladders ~2x (dense and sparse);
buck is noise-dominated at this size (17-21ms run-to-run, unchanged).
Sub-30ms fixtures carry +-10-20% machine noise; trust ladder/vienna.

Checksum notes: buck + ladders are BITWISE identical to pre-14b (the
cache is exact, including across switch/diode invalidations). Vienna
moved in the last digit (.7157 -> .7156, ~1e-13 relative) because of
the companion dt-exactness fix (full frames now pass exactly baseDt
instead of (t+dt)-t with 1-ulp jitter); the new trajectory is
deterministic across reruns and more correct by construction.

## v0.1.0 (pre-14b, for reference)

```
buck-open-6ms      steps=12000   wall=    17.2ms us/step=  1.436 checksum=73604.5196971
vienna-60ms        steps=60000   wall=   257.2ms us/step=  4.286 checksum=28383342.7157
ladder-10          steps=2000    wall=     5.8ms us/step=  2.886 checksum=8534.87986295
ladder-40          steps=2000    wall=    54.4ms us/step= 27.200 checksum=8534.87986295
ladder-160         steps=2000    wall=   264.4ms us/step=132.195 checksum=8534.87986295
ladder-320         steps=2000    wall=   679.3ms us/step=339.661 checksum=8534.87986295
```

Scaling read: ladder-10 -> ladder-40 (4x nodes, dense path) costs ~9.4x
(O(n^3) signature); ladder-160/320 run the sparse path (sub-cubic).
Per-step cost on small switching circuits: ~1.5–4.3us.
