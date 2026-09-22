#include <cmath>
#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

#include "power_engine/engine.h"

using power_engine::Engine;
using power_engine::Integrator;

// Switch-interrupted inductor: L(10mH, 1A) 1-2, R(10ohm) 2-0, switch 0-1.
// Opens at 0.5ms with ~0.6A flowing. Trapezoidal traps the interrupt
// energy in a sustained Nyquist oscillation (kV); TR-BDF2 damps it in a
// few steps (L-stable). Returns peak |vL| over (0.55ms, 2ms].
static double interruptPeak(bool bdf2) {
  Engine eng;
  eng.setTimeStep(1e-6);
  eng.setStopTime(2e-3);
  if (bdf2) eng.setIntegrator(Integrator::TrBdf2);
  eng.circuit().addInductor("L1", 1, 2, 10e-3, 1.0);
  eng.circuit().addResistor("R1", 2, 0, 10.0);
  eng.circuit().addSwitch("S1", 0, 1, 5e-3, 1e6, true);
  eng.start();
  double peak = 0.0;
  while (eng.status() == power_engine::SimulationStatus::Running) {
    if (eng.time() >= 0.5e-3) eng.setSwitch("S1", false);
    eng.step();
    if (eng.time() > 0.55e-3) {
      const double v = std::abs(eng.currentSolution().probes.at("v:1") -
                                eng.currentSolution().probes.at("v:2"));
      if (v > peak) peak = v;
    }
  }
  return peak;
}

TEST(TrBdf2, DampsInterruptRinging) {
  EXPECT_GT(interruptPeak(false), 500.0);  // trap rings (measured ~1670V)
  EXPECT_LT(interruptPeak(true), 1.0);     // BDF2 damps (measured ~1e-62)
}

// Second-order accuracy preserved: RC charge under TR-BDF2 matches the
// analytic curve.
TEST(TrBdf2, RcAccuracy) {
  Engine eng;
  eng.setTimeStep(1e-6);
  eng.setIntegrator(Integrator::TrBdf2);
  eng.circuit().addVoltageSource("V1", 1, 0, 1.0);
  eng.circuit().addResistor("R1", 1, 2, 1000.0);
  eng.circuit().addCapacitor("C1", 2, 0, 1e-6, 0.0);
  eng.setStopTime(5e-3);
  eng.start();
  while (eng.status() == power_engine::SimulationStatus::Running) eng.step();
  EXPECT_NEAR(eng.currentSolution().probes.at("v:2"), 1.0 - std::exp(-5.0), 0.01);
}

// Saturable inductor under TR-BDF2: 230Vrms/50Hz into R=10 + sat-L
// (Lunsat=100mH, Lsat=5mH, Isat=2A). Must converge every step and agree
// with trapezoidal on RMS current (both resolve the same physics).
static double satIrms(bool bdf2) {
  Engine eng;
  eng.setTimeStep(1e-6);
  if (bdf2) eng.setIntegrator(Integrator::TrBdf2);
  eng.circuit().addVoltageSource("V1", 1, 0, 0.0);
  eng.circuit().addResistor("R1", 1, 2, 10.0);
  eng.circuit().addSaturableInductor("L1", 2, 0, 100e-3, 5e-3, 2.0, 0.0);
  eng.setStopTime(60e-3);
  eng.start();
  const double w = 2.0 * std::acos(-1.0) * 50.0;
  double s2 = 0.0, n = 0.0;
  while (eng.status() == power_engine::SimulationStatus::Running) {
    const double t = eng.time();
    eng.circuit().findDevice("V1").value = 325.0 * std::sin(w * t);
    eng.step();
    if (t > 20e-3) {
      const double i = eng.deviceCurrent("L1");
      s2 += i * i;
      n += 1.0;
    }
  }
  return std::sqrt(s2 / n);
}

TEST(TrBdf2, SatAgreesWithTrap) {
  const double a = satIrms(false), b = satIrms(true);
  EXPECT_GT(a, 0.0);
  EXPECT_NEAR(b, a, 0.05 * a);
}

// Coupled inductors under TR-BDF2: 10V/1kHz primary, loaded secondary —
// RMS agreement with trapezoidal.
static double coupledVrms(bool bdf2) {
  Engine eng;
  eng.setTimeStep(1e-6);
  if (bdf2) eng.setIntegrator(Integrator::TrBdf2);
  eng.circuit().addVoltageSource("V1", 1, 0, 0.0);
  eng.circuit().addResistor("R1", 1, 2, 1.0);
  eng.circuit().addCoupledInductors("W1", 2, 0, 3, 0, 1e-3, 1e-3, 0.9, 0.0, 0.0);
  eng.circuit().addResistor("R2", 3, 0, 10.0);
  eng.setStopTime(5e-3);
  eng.start();
  const double w = 2.0 * std::acos(-1.0) * 1000.0;
  double s2 = 0.0, n = 0.0;
  while (eng.status() == power_engine::SimulationStatus::Running) {
    const double t = eng.time();
    eng.circuit().findDevice("V1").value = 10.0 * std::sin(w * t);
    eng.step();
    if (t > 2e-3) {
      const double v = eng.currentSolution().probes.at("v:3");
      s2 += v * v;
      n += 1.0;
    }
  }
  return std::sqrt(s2 / n);
}

TEST(TrBdf2, CoupledAgreesWithTrap) {
  const double a = coupledVrms(false), b = coupledVrms(true);
  EXPECT_GT(a, 0.0);
  EXPECT_NEAR(b, a, 0.03 * a);
}

// Netlist `.tran dt tstop TRBDF2`: parses, round-trips, and drives the
// solver integrator; bad methods throw.
TEST(TrBdf2, NetlistMethod) {
  Engine eng;
  eng.loadNetlist(R"(
V1 1 0 1
R1 1 2 1k
C1 2 0 1u
.tran 1u 5m TRBDF2
.end
)");
  EXPECT_EQ(eng.solverStats().steps, 0);
  eng.start();
  while (eng.status() == power_engine::SimulationStatus::Running) eng.step();
  EXPECT_NEAR(eng.currentSolution().probes.at("v:2"), 1.0 - std::exp(-5.0), 0.01);
  EXPECT_NE(eng.netlist().serialize().find("TRBDF2"), std::string::npos);
  Engine bad;
  EXPECT_THROW(bad.loadNetlist("V1 1 0 1\nR1 1 0 1k\n.tran 1u 1m BOGUS\n.end\n"),
               std::runtime_error);
}
