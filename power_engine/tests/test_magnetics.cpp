#include <cmath>
#include <stdexcept>
#include <gtest/gtest.h>

#include "power_engine/engine.h"
#include "power_engine/netlist.h"

using power_engine::Engine;

// Open secondary: at t=0+ the full step appears across L1, so the induced
// secondary voltage is v2 = (M/L1)*v1 = k*sqrt(L2/L1)*10V = 9V.
TEST(CoupledInductors, OpenSecondaryVoltageRatio) {
  Engine eng;
  eng.setTimeStep(1e-6);
  eng.circuit().addVoltageSource("V1", 1, 0, 10.0);
  eng.circuit().addResistor("R1", 1, 2, 100.0);
  eng.circuit().addCoupledInductors("W1", 2, 0, 3, 0, 1e-3, 1e-3, 0.9);
  eng.circuit().addResistor("Rmeas", 3, 0, 1e6);  // near-open probe
  eng.setStopTime(2e-6);
  eng.start();
  eng.step();  // first step: transient division, no resistive drops yet
  EXPECT_NEAR(eng.currentSolution().probes.at("v:3"), 9.0, 0.05 * 9.0);
}

// Shorted secondary: primary sees leakage L1*(1-k^2), tau = L/R = 19us.
// (Asserted on the settled exponential, not the first step: zero-state
// histories make the first trapezoidal step see half the applied step.)
TEST(CoupledInductors, ShortedSecondarySlope) {
  Engine eng;
  eng.setTimeStep(1e-6);
  eng.circuit().addVoltageSource("V1", 1, 0, 10.0);
  eng.circuit().addResistor("R1", 1, 2, 10.0);
  eng.circuit().addCoupledInductors("W1", 2, 0, 3, 0, 1e-3, 1e-3, 0.9);
  eng.circuit().addResistor("Rshort", 3, 0, 1e-3);
  eng.setStopTime(20e-6);
  eng.start();
  while (eng.status() == power_engine::SimulationStatus::Running) eng.step();
  const double i1 = eng.deviceCurrent("W1");
  const double i2 = eng.circuit().findDevice("W1").i2_prev;
  // i(t) = (V/R)(1 - exp(-t/tau)), tau = 1e-3*(1-0.81)/10 = 19us.
  const double ref = 1.0 * (1.0 - std::exp(-20e-6 / 19e-6));
  EXPECT_NEAR(i1, ref, 0.03 * ref);
  EXPECT_LT(i2, 0.0);  // Lenz opposition
}

// Saturable inductor: driven past Isat, current runs away above the linear
// prediction but stays under the resistive limit. Newton must engage.
TEST(SaturableInductor, CurrentRunsAwayPastKnee) {
  Engine eng;
  eng.setTimeStep(10e-6);
  eng.circuit().addVoltageSource("V1", 1, 0, 10.0);
  eng.circuit().addResistor("R1", 1, 2, 1.0);
  eng.circuit().addSaturableInductor("Y1", 2, 0, 10e-3, 1e-3, 1.0, 0.0);
  eng.setStopTime(2e-3);
  eng.start();
  while (eng.status() == power_engine::SimulationStatus::Running) eng.step();
  const double iSat = eng.deviceCurrent("Y1");
  const double iLin = 10.0 * (1.0 - std::exp(-2e-3 / 10e-3));  // linear 10mH
  EXPECT_GT(iSat, 1.5 * iLin);
  EXPECT_LT(iSat, 9.5);  // resistive limit 10V/1ohm
  EXPECT_GT(eng.solverStats().newtonIters, 0);
}

// Below the knee it behaves as the unsaturated inductance.
TEST(SaturableInductor, SmallSignalMatchesLinear) {
  Engine eng;
  eng.setTimeStep(10e-6);
  eng.circuit().addVoltageSource("V1", 1, 0, 0.5);  // stays << Isat
  eng.circuit().addResistor("R1", 1, 2, 1.0);
  eng.circuit().addSaturableInductor("Y1", 2, 0, 10e-3, 1e-3, 1.0, 0.0);
  eng.setStopTime(2e-3);
  eng.start();
  while (eng.status() == power_engine::SimulationStatus::Running) eng.step();
  const double iSat = eng.deviceCurrent("Y1");
  const double iLin = 0.5 * (1.0 - std::exp(-2e-3 / 10e-3));
  EXPECT_NEAR(iSat, iLin, 0.02 * iLin);
}

TEST(SaturableInductor, BadParamsThrow) {
  Engine eng;
  EXPECT_THROW(eng.circuit().addSaturableInductor("Y1", 1, 0, 1e-3, 2e-3, 1.0),
               std::runtime_error);  // Lsat > Lunsat
  EXPECT_THROW(eng.circuit().addSaturableInductor("Y1", 1, 0, 1e-3, 1e-3, 0.0),
               std::runtime_error);  // Isat = 0
  power_engine::netlist::Parser p;
  auto r = p.parse("V1 1 0 10\nR1 1 2 1\nY1 2 0 LUNSAT=10m LSAT=1m ISAT=1\n.end\n");
  EXPECT_DOUBLE_EQ(r.circuit.findDevice("Y1").lsat, 1e-3);
  auto r2 = p.parse(r.serialize());
  EXPECT_DOUBLE_EQ(r2.circuit.findDevice("Y1").isat, 1.0);
}

// Unphysical couplings are rejected, not silently clipped.
TEST(CoupledInductors, BadCouplingThrows) {
  Engine eng;
  EXPECT_THROW(eng.circuit().addCoupledInductors("W1", 1, 0, 2, 0, 1e-3, 1e-3, 1.0),
               std::runtime_error);  // singular companion
  EXPECT_THROW(eng.circuit().addCoupledInductors("W1", 1, 0, 2, 0, 1e-3, 1e-3, 1.2),
               std::runtime_error);
  EXPECT_THROW(eng.circuit().addCoupledInductors("W1", 1, 0, 2, 0, 1e-3, 1e-3, 0.0),
               std::runtime_error);
}

// Netlist W device parses, elaborates, and round-trips.
TEST(CoupledInductors, NetlistWinding) {
  power_engine::netlist::Parser p;
  auto r = p.parse(R"(
V1 1 0 10
R1 1 2 100
W1 2 0 3 0 L1=1m L2=1m K=0.9
Rmeas 3 0 1Meg
.tran 1u 2u
.end
)");
  EXPECT_DOUBLE_EQ(r.circuit.findDevice("W1").l1, 1e-3);
  EXPECT_NEAR(r.circuit.findDevice("W1").m, 0.9e-3, 1e-12);
  auto r2 = p.parse(r.serialize());
  EXPECT_NEAR(r2.circuit.findDevice("W1").m, 0.9e-3, 1e-9);
  EXPECT_THROW(p.parse("V1 1 0 1\nW1 1 0 2 0 L1=1m L2=1m K=0.5 M=0.4m\n.end\n"),
               std::runtime_error);  // K and M together
}
