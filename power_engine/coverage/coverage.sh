#!/usr/bin/env bash
# Line coverage over power_engine/src (gcov, needs GCC + POWER_ENGINE_COVERAGE).
# Usage: power_engine/coverage/coverage.sh [--jobs N]
# Builds build-cov (lib+tests, no examples), runs ctest, aggregates per-file
# "Lines executed" for src/** excluding tests/examples/fuzz. Not gated in
# CI (informational); recorded in roadmap item 15.
set -u
JOBS=8
if [ "${1:-}" = "--jobs" ]; then JOBS="$2"; fi
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
B="$ROOT/build-cov"
cmake -B "$B" -S "$ROOT" -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_COMPILER=g++ -DCMAKE_C_COMPILER=gcc \
  -DPOWER_ENGINE_COVERAGE=ON \
  -DPOWER_ENGINE_BUILD_TESTS=ON -DPOWER_ENGINE_BUILD_EXAMPLES=OFF > /dev/null
cmake --build "$B" -j"$JOBS" > /dev/null
(cd "$B" && ctest --output-on-failure > /dev/null)
OBJ="$B/power_engine/CMakeFiles/power_engine.dir/src"
TOTAL_L=0; TOTAL_E=0
printf '%-42s %8s\n' "file" "line-cov"
while IFS= read -r src; do
  rel="${src#$ROOT/power_engine/src/}"
  dir="$OBJ/$(dirname "$rel")"
  base="$(basename "$rel" .cpp)"
  line=$(gcov -n -o "$dir" "$dir/$base.cpp.gcno" 2>/dev/null | \
    awk -v want="'$src'" '$1=="File"{$1=""; f=substr($0,2); next} f==want && /^Lines executed/{print; exit}')
  # line looks like: Lines executed:88.00% of 120
  pct=$(echo "$line" | grep -oE '[0-9]+\.[0-9]+%' | tr -d '%')
  ln=$(echo "$line" | grep -oE 'of [0-9]+' | tr -dc '0-9')
  if [ -z "$pct" ]; then printf '%-42s %8s\n' "$rel" "n/a"; continue; fi
  ex=$(echo "$pct $ln" | awk '{printf "%d", $1*$2/100}')
  TOTAL_L=$((TOTAL_L + ln)); TOTAL_E=$((TOTAL_E + ex))
  printf '%-42s %7s%%\n' "$rel" "$pct"
done < <(find "$ROOT/power_engine/src" -name '*.cpp' | sort)
echo "----------------------------------------------------"
awk "BEGIN {printf \"TOTAL: %.2f%% of %d lines (%d executed)\n\", 100*$TOTAL_E/$TOTAL_L, $TOTAL_L, $TOTAL_E}"
