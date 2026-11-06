// pe_scaling: switched-sparse scaling benchmark (roadmap item 1.5).
// Complements pe_bench's passive ladder: the same RC ladder with ~50
// switch+diode shunt branches whose gates toggle during the run,
// invalidating the factorization cache at every edge. This measures the
// sparse path (>= 64 nodes) under switched load, where the passive
// ladder fixture sees ~100% cache hits.
// Not part of ctest (timing is informational); runtime target <60s.
#include <chrono>
#include <cstdio>
#include <string>

#include "power_engine/engine.h"

namespace {

struct Result {
  std::string name;
  long long steps = 0;
  double wallMs = 0.0;
  double vEnd = 0.0;
  long long sparseSolves = 0;
  long long factorSkips = 0;
  long long diodeEvents = 0;
  long long resolves = 0;
};

void report(const Result& r) {
  const double skipRate =
      r.steps > 0 ? 100.0 * static_cast<double>(r.factorSkips) / static_cast<double>(r.steps) : 0.0;
  std::printf("%-14s steps=%-6lld wall=%9.1fms us/step=%9.3f sparse=%lld v_end=%.4f\n",
              r.name.c_str(), r.steps, r.wallMs,
              r.wallMs * 1000.0 / static_cast<double>(r.steps), r.sparseSolves, r.vEnd);
  std::printf("               factorSkips=%lld (%.1f%%) events=%lld resolves=%lld\n",
              r.factorSkips, skipRate, r.diodeEvents, r.resolves);
}

// RC ladder rung count nRung (nodes 1..nRung, source node nRung+1); every
// 2nd node carries a shunt switch+diode branch (anti-parallel) that turns
// on staggered across the run (~100 edges total). Node count ~ nRung+1,
// so sizes >= 64 exercise the SparseLU path.
Result switchedLadder(int nRung) {
  power_engine::Engine eng;
  eng.setTimeStep(1e-6);
  eng.setStopTime(2e-3);
  const int src = nRung + 1;
  eng.circuit().addVoltageSource("V1", src, 0, 10.0);
  eng.circuit().addResistor("R0", src, nRung, 1000.0);
  for (int k = nRung; k >= 1; --k) {
    eng.circuit().addCapacitor("C" + std::to_string(k), k, 0, 1e-6, 0.0);
    if (k > 1) eng.circuit().addResistor("R" + std::to_string(k), k, k - 1, 1000.0);
  }
  const int nBranches = std::min(50, nRung / 2);
  for (int b = 0; b < nBranches; ++b) {
    const int node = 2 * b + 2;  // nodes 2,4,6,...
    eng.circuit().addSwitch("SW" + std::to_string(b), node, 0, 10e-3, 1e6, false);
    eng.circuit().addDiode("D" + std::to_string(b), 0, node, 0.7, 10e-3, 1e6);
    const double tOn = 0.1e-3 + b * 4e-6;  // staggered turn-on edges
    const double tOff = tOn + 1e-3;        // turn-off half a period later
    eng.scheduleSwitch("SW" + std::to_string(b), true, tOn);
    eng.scheduleSwitch("SW" + std::to_string(b), false, tOff);
  }
  const auto t0 = std::chrono::steady_clock::now();
  eng.start();
  long steps = 0;
  while (eng.status() == power_engine::SimulationStatus::Running) {
    eng.step();
    ++steps;
  }
  const auto t1 = std::chrono::steady_clock::now();
  const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  const auto& st = eng.solverStats();
  const double vEnd = eng.currentSolution().probes.at("v:" + std::to_string(nRung));
  return {"switched-" + std::to_string(nRung), steps, ms, vEnd,
          st.sparseSolves, st.factorSkips, st.diodeEvents, st.resolves};
}

}  // namespace

int main() {
  std::printf("# pe_scaling (Release): fixture, steps, wall, us/step, sparse count\n");
  for (int n : {100, 200, 500, 1000, 2000}) report(switchedLadder(n));
  return 0;
}
