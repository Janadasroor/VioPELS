#include <gtest/gtest.h>

#include "power_engine/engine.h"

using power_engine::Engine;

// Switch as Ron/Roff: divider V1=1V -> R1=1k -> node2 -> Rload=1k -> gnd,
// switch in parallel with Rload? Simpler: V1 - R1 - node2 - switch - gnd.
// Closed (5mOhm): v2 ~ 0. Open (1Meg): v2 = 0.5V.
TEST(Switch, RonRoffStamping) {
  Engine eng;
  eng.setTimeStep(1e-6);
  eng.circuit().addVoltageSource("V1", 1, 0, 1.0);
  eng.circuit().addResistor("R1", 1, 2, 1000.0);
  eng.circuit().addSwitch("S1", 2, 0, 5e-3, 1e6, true);  // closed
  eng.setStopTime(20e-6);
  eng.start();
  while (eng.status() == power_engine::SimulationStatus::Running) eng.step();
  EXPECT_LT(eng.currentSolution().probes.at("v:2"), 1e-4);

  Engine eng2;
  eng2.setTimeStep(1e-6);
  eng2.circuit().addVoltageSource("V1", 1, 0, 1.0);
  eng2.circuit().addResistor("R1", 1, 2, 1000.0);
  eng2.circuit().addResistor("R2", 2, 0, 1000.0);
  eng2.circuit().addSwitch("S1", 2, 0, 5e-3, 1e6, false);  // open: Roff || 1k
  eng2.setStopTime(20e-6);
  eng2.start();
  while (eng2.status() == power_engine::SimulationStatus::Running) eng2.step();
  EXPECT_NEAR(eng2.currentSolution().probes.at("v:2"), 0.5, 0.01);
}

// Diode forward conduction with Vf=0: half-wave behavior.
// Vsin approximated by DC sweep: +1V conducts, -1V blocks.
TEST(Diode, ForwardBlocksReverse) {
  Engine eng;
  eng.setTimeStep(1e-6);
  eng.circuit().addVoltageSource("V1", 1, 0, 1.0);
  eng.circuit().addResistor("R1", 1, 2, 100.0);
  eng.circuit().addDiode("D1", 2, 0, 0.0, 10e-3, 1e6);
  eng.setStopTime(50e-6);
  eng.start();
  while (eng.status() == power_engine::SimulationStatus::Running) eng.step();
  // Conducting: v2 ~ 0 (Vf=0, Ron small), diode on.
  EXPECT_TRUE(eng.circuit().diodeConducting("D1"));
  EXPECT_LT(eng.currentSolution().probes.at("v:2"), 0.05);
  EXPECT_GT(eng.solverStats().diodeEvents, 0);
}

TEST(Diode, ReverseBiasStaysOff) {
  Engine eng;
  eng.setTimeStep(1e-6);
  eng.circuit().addVoltageSource("V1", 1, 0, -1.0);
  eng.circuit().addResistor("R1", 1, 2, 100.0);
  eng.circuit().addDiode("D1", 2, 0, 0.7, 10e-3, 1e6);
  eng.setStopTime(50e-6);
  eng.start();
  while (eng.status() == power_engine::SimulationStatus::Running) eng.step();
  EXPECT_FALSE(eng.circuit().diodeConducting("D1"));
  // Roff divider: v2 ~ -1V * (1Meg/(100+1Meg)) ~ -1V.
  EXPECT_NEAR(eng.currentSolution().probes.at("v:2"), -1.0, 0.01);
}

TEST(Diode, ForwardDropStamped) {
  // Vf=0.7 conducting: v2 should sit at ~Vf (diode 2->0, forward).
  Engine eng;
  eng.setTimeStep(1e-6);
  eng.circuit().addVoltageSource("V1", 1, 0, 2.0);
  eng.circuit().addResistor("R1", 1, 2, 100.0);
  eng.circuit().addDiode("D1", 2, 0, 0.7, 10e-3, 1e6);
  eng.setStopTime(100e-6);
  eng.start();
  while (eng.status() == power_engine::SimulationStatus::Running) eng.step();
  EXPECT_TRUE(eng.circuit().diodeConducting("D1"));
  EXPECT_NEAR(eng.currentSolution().probes.at("v:2"), 0.7, 0.02);
}

TEST(SolverGuard, FloatingNodesThrowSingular) {
  // Nodes 1-2 connected only to each other (no ground path, no source):
  // structurally singular MNA must raise, not NaN.
  Engine eng;
  eng.setTimeStep(1e-6);
  eng.circuit().addResistor("R1", 1, 2, 1000.0);
  eng.setStopTime(10e-6);
  eng.start();
  EXPECT_THROW(eng.step(), std::runtime_error);
  EXPECT_EQ(eng.status(), power_engine::SimulationStatus::Error);
}
