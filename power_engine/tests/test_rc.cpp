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
