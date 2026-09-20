#include <cmath>
#include <stdexcept>
#include <gtest/gtest.h>

#include "power_engine/engine.h"

using power_engine::Engine;

// RC charge under adaptive stepping: base (outer) dt = dtMax = 1ms, the
// controller subdivides only as needed. Must match analytical within 1%
// with far fewer than the fixed 1us run (5000 steps).
TEST(AdaptiveStep, RcMatchesAnalyticalWithFewSteps) {
  Engine eng;
  eng.setTimeStep(1e-3);  // outer frame; inner steps adapt within [1ns, 1ms]
  eng.setAdaptive(1e-3, 1e-9, 1e-3);
  eng.circuit().addVoltageSource("V1", 1, 0, 5.0);
  eng.circuit().addResistor("R1", 1, 2, 1000.0);
  eng.circuit().addCapacitor("C1", 2, 0, 1e-6, 0.0);
  eng.setStopTime(5e-3);
  eng.start();
  double vc = 0.0;
  while (eng.status() == power_engine::SimulationStatus::Running) eng.step();
  vc = eng.currentSolution().probes.at("v:2");
  const double ref = 5.0 * (1.0 - std::exp(-5.0));
  EXPECT_NEAR(vc, ref, 0.01 * ref);
  EXPECT_LT(eng.solverStats().steps, 1500);
  EXPECT_GT(eng.solverStats().steps, 5);
}

TEST(AdaptiveStep, BadConfigThrows) {
  Engine eng;
  EXPECT_THROW(eng.setAdaptive(0.0, 1e-9, 1e-3), std::runtime_error);
  EXPECT_THROW(eng.setAdaptive(1e-3, 1e-3, 1e-9), std::runtime_error);  // min > max
  EXPECT_THROW(eng.setAdaptive(1e-3, 0.0, 1e-3), std::runtime_error);
}
