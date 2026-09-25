#!/usr/bin/env bash
# Headless plots workflow check: pe -> CSVs -> PNGs. Fails loudly on any
# missing output. Used locally and in CI (ubuntu plots-check job).
# Env: PE = pe binary (default: build/power_engine/tools/pe).
set -euo pipefail

PE="${PE:-build/power_engine/tools/pe}"
PLOTS="$(dirname "$0")"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

cat > "$TMP/buck.net" <<'EOF'
.param R 5
.model SW mosfet_ideal RON=5m ROFF=1Meg
.model DD diode_ideal VF=0.0 RON=10m
V1 1 0 12
S1 1 2 MODEL=SW
D1 0 2 MODEL=DD
L1 2 3 200u
C1 3 0 200u
Rload 3 0 {R}
.control pwm switch=S1 freq=20k duty=0.5
.tran 0.5u 3m
.end
EOF

"$PE" run --netlist "$TMP/buck.net" --out "$TMP/run.csv"
"$PE" run --netlist "$TMP/buck.net" --method trbdf2 --out "$TMP/run_bdf2.csv"
"$PE" sweep --netlist "$TMP/buck.net" --axis R=1,2,5,10 \
  --measure 'mean(v:3) as vout' --out "$TMP/sweep.csv"

export MPLBACKEND=Agg
python3 "$PLOTS/plot_run.py" "$TMP/run.csv" --out "$TMP/run.png"
python3 "$PLOTS/plot_compare.py" --probe v:3 \
  "$TMP/run.csv" "$TMP/run_bdf2.csv" --labels trap,trbdf2 --out "$TMP/compare.png"
python3 "$PLOTS/plot_spectrum.py" "$TMP/run.csv" --probe v:3 \
  --out "$TMP/spectrum.png" | grep -q '^peak:'
python3 "$PLOTS/plot_sweep.py" "$TMP/sweep.csv" --x R --y vout \
  --out "$TMP/sweep.png"

for f in run.png compare.png spectrum.png sweep.png; do
  test -s "$TMP/$f" || { echo "selfcheck: missing $f" >&2; exit 1; }
done
echo "plots selfcheck OK (4 PNGs)"
