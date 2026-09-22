#include <cmath>
#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

#include "power_engine/engine.h"

using power_engine::Engine;

// RC charge: Vsrc 1V (1-gnd), R 1k (1-2), C 1u (2-gnd). tau = 1ms.
// Vc(t) = V0*(1-exp(-t/tau)). dt=1us, run 5ms.
TEST(RCTransient, MatchesAnalyticalWithin1Pct) {
  Engine eng;
  eng.setTimeStep(1e-6);
  eng.circuit().addVoltageSource("V1", 1, 0, 1.0);
  eng.circuit().addResistor("R1", 1, 2, 1000.0);
  eng.circuit().addCapacitor("C1", 2, 0, 1e-6, 0.0);
  eng.setStopTime(5e-3);
  eng.start();

  const double tau = 1e-3;
  auto expected = [&](double t) { return 1.0 * (1.0 - std::exp(-t / tau)); };

  double vc_1ms = 0.0, vc_2ms = 0.0, vc_5ms = 0.0;
  while (eng.status() == power_engine::SimulationStatus::Running) {
    eng.step();
    const double t = eng.currentSolution().t;
    const double vc = eng.currentSolution().probes.at("v:2");
    if (std::abs(t - 1e-3) < 0.6e-6) vc_1ms = vc;
    if (std::abs(t - 2e-3) < 0.6e-6) vc_2ms = vc;
    if (std::abs(t - 5e-3) < 1.1e-6) vc_5ms = vc;
  }
  for (auto [got, t] : {std::pair<double, double>{vc_1ms, 1e-3},
                        {vc_2ms, 2e-3},
                        {vc_5ms, 5e-3}}) {
    const double ref = expected(t);
    EXPECT_GT(got, 0.0);
    EXPECT_NEAR(got, ref, 0.01 * ref) << "t=" << t << " got=" << got << " ref=" << ref;
  }
}

// Large RC ladder (70 rungs, 72 rows): exercises the SparseLU path
// (threshold 64). Diffusion time ~R*C*N^2/2 = 2.5ms; far end settles.
TEST(RCLadder, SparsePathSettlesToSource) {
  constexpr int kRungs = 70;
  power_engine::Engine eng;
  eng.setTimeStep(5e-6);
  eng.circuit().addVoltageSource("V1", 1, 0, 5.0);
  int prev = 1;
  int node = 2;
  for (int i = 0; i < kRungs; ++i) {
    const std::string r = "R" + std::to_string(i);
    const std::string c = "C" + std::to_string(i);
    eng.circuit().addResistor(r, prev, node, 10.0);
    eng.circuit().addCapacitor(c, node, 0, 100e-9, 0.0);
    prev = node++;
  }
  eng.setStopTime(12e-3);
  eng.start();
  while (eng.status() == power_engine::SimulationStatus::Running) eng.step();
  EXPECT_GT(eng.solverStats().sparseSolves, 0);
  const double vFar = eng.currentSolution().probes.at("v:" + std::to_string(prev));
  EXPECT_NEAR(vFar, 5.0, 0.02 * 5.0);
}

// Factorization cache (item 14b): a diode-/switch-free run with constant
// dt must factor exactly once — every later step reuses the factors and
// only re-assembles the RHS. The voltage assertion ties cache correctness
// to physics (stale factors would corrupt the trajectory).
TEST(SolverCache, SkipsRefactorOnSteadyRun) {
  Engine eng;
  eng.setTimeStep(1e-6);
  eng.circuit().addVoltageSource("V1", 1, 0, 1.0);
  eng.circuit().addResistor("R1", 1, 2, 1000.0);
  eng.circuit().addCapacitor("C1", 2, 0, 1e-6, 0.0);
  eng.setStopTime(5e-3);
  eng.start();
  while (eng.status() == power_engine::SimulationStatus::Running) eng.step();
  const auto& st = eng.solverStats();
  EXPECT_EQ(st.steps, 5000);
  // Exactly one cold factorization; the stop-time landing step may clamp
  // to a short dt (one more legitimate refactor). Anything else is a
  // caching failure.
  EXPECT_LE(st.factorSkips, st.steps - 1);
  EXPECT_GE(st.factorSkips, st.steps - 2);
  EXPECT_EQ(st.resolves, 0);
  const double vc = eng.currentSolution().probes.at("v:2");
  EXPECT_NEAR(vc, 1.0 - std::exp(-5.0), 0.01);
}

// A mid-run switch toggle changes the topology signature: exactly one
// extra factorization (cold start + toggle), cache hits everywhere else,
// and the held capacitor voltage follows the Roff-drift analytic.
TEST(SolverCache, RefactorizesOnSwitchEvent) {
  Engine eng;
  eng.setTimeStep(1e-6);
  eng.circuit().addVoltageSource("V1", 1, 0, 1.0);
  eng.circuit().addSwitch("S1", 1, 2, 5e-3, 1e6, true);
  eng.circuit().addResistor("R1", 2, 3, 1000.0);
  eng.circuit().addCapacitor("C1", 3, 0, 1e-6, 0.0);
  eng.setStopTime(5e-3);
  eng.start();
  while (eng.status() == power_engine::SimulationStatus::Running) {
    if (eng.time() >= 2e-3) eng.setSwitch("S1", false);
    eng.step();
  }
  const auto& st = eng.solverStats();
  EXPECT_EQ(st.steps, 5000);
  // Cold + toggle (+ maybe the stop-time landing short step, as above).
  EXPECT_LE(st.factorSkips, st.steps - 2);
  EXPECT_GE(st.factorSkips, st.steps - 3);
  // v(2ms) held, then Roff-drift toward 1V with tau ~= 1.001s over 3ms.
  const double vHold = 1.0 - std::exp(-2.0);
  const double ref = 1.0 - (1.0 - vHold) * std::exp(-3e-3 / 1.001);
  const double vc = eng.currentSolution().probes.at("v:3");
  EXPECT_NEAR(vc, ref, 0.01);
}
// Large floating ladder (no ground path at all): sparse path must still
// detect singularity instead of returning garbage.
TEST(RCLadder, SparsePathDetectsSingular) {
  power_engine::Engine eng;
  eng.setTimeStep(10e-6);
  int prev = 1;
  int node = 2;
  for (int i = 0; i < 100; ++i) {
    eng.circuit().addResistor("R" + std::to_string(i), prev, node, 100.0);
    prev = node++;
  }
  eng.setStopTime(10e-6);
  eng.start();
  EXPECT_THROW(eng.step(), std::runtime_error);
}
