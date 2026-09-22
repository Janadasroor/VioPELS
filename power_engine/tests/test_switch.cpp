#include <cmath>
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

// --- Slew-limited transitions (tsw): geometric R sweep over tsw seconds.

namespace {
// R during a turn-on ramp started at t0 (Roff -> Ron over tsw).
double rampR(double t, double t0, double tsw, double roff, double ron) {
  const double f = std::min(1.0, std::max(0.0, (t - t0) / tsw));
  return roff * std::pow(ron / roff, f);
}
}  // namespace

// Resistive turn-on: V=10 -> switch -> Rload=10, tsw=200us. Load voltage
// follows the divider with the geometric ramp exactly.
TEST(SlewTransition, GeometricRampShape) {
  constexpr double kTsw = 200e-6, kT0 = 100e-6;
  Engine eng;
  eng.setTimeStep(1e-6);
  eng.circuit().addVoltageSource("V1", 1, 0, 10.0);
  eng.circuit().addSwitch("S1", 1, 2, 5e-3, 1e6, false, 0.0, 0.0, 0.0, 0.1, kTsw);
  eng.circuit().addResistor("R1", 2, 0, 10.0);
  eng.setStopTime(600e-6);
  eng.start();
  eng.scheduleSwitch("S1", true, kT0);
  for (double f : {0.25, 0.5, 0.75}) {
    const double tWant = kT0 + f * kTsw;
    while (eng.time() < tWant) eng.step();
    const double rr = rampR(eng.time(), kT0, kTsw, 1e6, 5e-3);
    EXPECT_NEAR(eng.currentSolution().probes.at("v:2"), 10.0 * 10.0 / (rr + 10.0),
                0.02 * 10.0)
        << "f=" << f;
  }
  while (eng.status() == power_engine::SimulationStatus::Running) eng.step();
  // Settled ON: full rail.
  EXPECT_NEAR(eng.currentSolution().probes.at("v:2"), 10.0, 0.01);
}

// Inductive turn-off: 10mH/0.6A interrupted through the 200us ramp —
// current transfers into the rising resistance with a ~190V peak (vs
// ~500kV ideal) and decays cleanly to zero, no ringing.
TEST(SlewTransition, InductiveNoSpike) {
  Engine eng;
  eng.setTimeStep(1e-6);
  eng.circuit().addInductor("L1", 1, 2, 10e-3, 1.0);
  eng.circuit().addResistor("R1", 2, 0, 10.0);
  eng.circuit().addSwitch("S1", 0, 1, 5e-3, 1e6, true, 0.0, 0.0, 0.0, 0.1, 200e-6);
  eng.setStopTime(1200e-6);
  eng.start();
  eng.scheduleSwitch("S1", false, 500e-6);
  double vMax = 0.0;
  while (eng.status() == power_engine::SimulationStatus::Running) {
    eng.step();
    const double v = std::abs(eng.currentSolution().probes.at("v:1") -
                              eng.currentSolution().probes.at("v:2"));
    if (v > vMax) vMax = v;
  }
  EXPECT_LT(vMax, 400.0);
  EXPECT_NEAR(eng.deviceCurrent("L1"), 0.0, 1e-3);
}

// Energy conservation: parallel L+S loop (no R) opened through the ramp —
// the full 1/2*L*I^2 must book into switch conduction loss.
TEST(SlewTransition, TransitionEnergyConserved) {
  Engine eng;
  eng.setTimeStep(1e-6);
  eng.circuit().addInductor("L1", 0, 1, 10e-3, 1.0);
  eng.circuit().addSwitch("S1", 1, 0, 5e-3, 1e6, true, 0.0, 0.0, 0.0, 0.1, 200e-6);
  eng.setStopTime(600e-6);
  eng.start();
  eng.scheduleSwitch("S1", false, 100e-6);
  while (eng.status() == power_engine::SimulationStatus::Running) {
    eng.step();
    if (eng.time() >= 100e-6 && eng.time() < 101e-6) eng.resetAccumulators();
  }
  EXPECT_NEAR(eng.deviceLoss("S1").econd, 0.5 * 10e-3 * 1.0 * 1.0, 0.01 * 0.005);
}

// Mid-ramp retoggle: on at t0, off halfway, on again — restarts cleanly
// from the ramp value and settles to the correct steady state.
TEST(SlewTransition, RetoggleMidRamp) {
  Engine eng;
  eng.setTimeStep(1e-6);
  eng.circuit().addVoltageSource("V1", 1, 0, 10.0);
  eng.circuit().addSwitch("S1", 1, 2, 5e-3, 1e6, false, 0.0, 0.0, 0.0, 0.1, 200e-6);
  eng.circuit().addResistor("R1", 2, 0, 10.0);
  eng.setStopTime(800e-6);
  eng.start();
  eng.scheduleSwitch("S1", true, 100e-6);
  eng.scheduleSwitch("S1", false, 200e-6);
  eng.scheduleSwitch("S1", true, 250e-6);
  while (eng.status() == power_engine::SimulationStatus::Running) eng.step();
  EXPECT_NEAR(eng.currentSolution().probes.at("v:2"), 10.0, 0.01);
  EXPECT_TRUE(std::isfinite(eng.deviceCurrent("S1")));
}

// Refine-roadmap R4: diodeCapHits safety-net monitor. Clean fixtures never
// saturate the 10-iteration diode loop (ideal-diode + R + independent-source
// nets converge in a few passes by construction — the ripple needed to
// exhaust the loop is damped out, which is also why item 14c was declined).
// Exhaustion is therefore not physically forceable; the test locks in the
// zero baseline and the counter plumbing (incl. snapshot round-trip).
TEST(DiodeCapHits, ZeroOnCleanFixtures) {
  {
    Engine eng;  // diode-free RC: loop breaks on the first scan
    eng.setTimeStep(1e-6);
    eng.circuit().addVoltageSource("V1", 1, 0, 1.0);
    eng.circuit().addResistor("R1", 1, 2, 1000.0);
    eng.circuit().addCapacitor("C1", 2, 0, 1e-6, 0.0);
    eng.setStopTime(50e-6);
    eng.start();
    while (eng.status() == power_engine::SimulationStatus::Running) eng.step();
    EXPECT_EQ(eng.solverStats().diodeCapHits, 0);
  }
  {
    Engine eng;  // buck with switch, diode, recovery: real commutations
    eng.setTimeStep(0.5e-6);
    eng.circuit().addVoltageSource("V1", 1, 0, 12.0);
    eng.circuit().addSwitch("S1", 1, 2, 5e-3, 1e6, true);
    eng.circuit().addDiode("D1", 0, 2, 0.7, 10e-3, 1e6, 50e-9, 25e-9);
    eng.circuit().addInductor("L1", 2, 3, 200e-6, 0.0);
    eng.circuit().addCapacitor("C1", 3, 0, 200e-6, 0.0);
    eng.circuit().addResistor("R1", 3, 0, 5.0);
    eng.setStopTime(200e-6);
    eng.start();
    while (eng.status() == power_engine::SimulationStatus::Running) {
      if (eng.time() >= 100e-6) eng.setSwitch("S1", false);
      eng.step();
    }
    const auto& st = eng.solverStats();
    EXPECT_GT(st.diodeEvents, 0);  // commutations happened...
    EXPECT_EQ(st.diodeCapHits, 0);  // ...but the loop never saturated
  }
  {
    // Adaptive trials snapshot/restore whole SolverStats (incl. the new
    // field): diode-active adaptive run completes clean with zero hits.
    Engine eng;
    eng.setTimeStep(0.5e-6);
    eng.circuit().addVoltageSource("V1", 1, 0, 12.0);
    eng.circuit().addSwitch("S1", 1, 2, 5e-3, 1e6, true);
    eng.circuit().addDiode("D1", 0, 2, 0.7, 10e-3, 1e6, 50e-9, 25e-9);
    eng.circuit().addInductor("L1", 2, 3, 200e-6, 0.0);
    eng.circuit().addCapacitor("C1", 3, 0, 200e-6, 0.0);
    eng.circuit().addResistor("R1", 3, 0, 5.0);
    eng.setStopTime(200e-6);
    eng.setAdaptive(1e-3, 1e-9, 10e-6);
    eng.start();
    while (eng.status() == power_engine::SimulationStatus::Running) {
      if (eng.time() >= 100e-6) eng.setSwitch("S1", false);
      eng.step();
    }
    EXPECT_EQ(eng.solverStats().diodeCapHits, 0);
    EXPECT_GT(eng.currentSolution().probes.at("v:3"), 0.0);
  }
}
