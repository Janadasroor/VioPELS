# pe_bench baselines (roadmap item 14a)

Machine: contributor workstation, gcc, Release. Timing is informational
(compare ratios on the same machine, not absolutes); checksums are the
waveform guard — any solver performance work (items 14b–d) must reproduce
them bitwise or within 1e-9 relative, or the "speedup" is a behavior change.

```
# pe_bench (Release): fixture, steps, wall, us/step, checksum
buck-open-6ms      steps=12000   wall=    17.2ms us/step=  1.436 checksum=73604.5196971
vienna-60ms        steps=60000   wall=   257.2ms us/step=  4.286 checksum=28383342.7157
ladder-10          steps=2000    wall=     5.8ms us/step=  2.886 checksum=8534.87986295
ladder-40          steps=2000    wall=    54.4ms us/step= 27.200 checksum=8534.87986295
ladder-160         steps=2000    wall=   264.4ms us/step=132.195 checksum=8534.87986295
ladder-320         steps=2000    wall=   679.3ms us/step=339.661 checksum=8534.87986295
```

Scaling read: ladder-10 → ladder-40 (4x nodes, dense path) costs ~9.4x
(O(n^3) signature); ladder-160/320 run the sparse path (sub-cubic).
Per-step cost on small switching circuits: ~1.5–4.3us.
