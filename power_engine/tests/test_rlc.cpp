#include <gtest/gtest.h>

#include "power_engine/engine.h"

using power_engine::Engine;

// Series RLC step: Vsrc 1V, R=1, L=1mH, C=100uF (underdamped).
// Checks: stable, final Vc -> 1V within 1%, overshoot peak > 1V.
TEST(RLCTransient, SettlesToSourceWithOvershoot) {
  Engine eng;
  eng.setTimeStep(5e-6);
  eng.circuit().addVoltageSource("V1", 1, 0, 1.0);
  eng.circuit().addResistor("R1", 1, 2, 1.0);
  eng.circuit().addInductor("L1", 2, 3, 1e-3, 0.0);
  eng.circuit().addCapacitor("C1", 3, 0, 100e-6, 0.0);
  eng.setStopTime(40e-3);
  eng.start();

  double peak = 0.0;
  double last = 0.0;
  while (eng.status() == power_engine::SimulationStatus::Running) {
    eng.step();
    last = eng.currentSolution().probes.at("v:3");
    if (last > peak) peak = last;
    ASSERT_TRUE(std::isfinite(last));
  }
  EXPECT_GT(peak, 1.0);                 // underdamped overshoot
  EXPECT_NEAR(last, 1.0, 0.01);         // settles to 1V within 1%
}
