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

// Automatic stiffness switching: the interrupt fixture rings under trap,
// so auto must engage TR-BDF2 (damping the peak), then return to trap on
// the quiet tail — exactly two switches, no flapping.
TEST(TrBdf2, AutoDampsAndReturns) {
  Engine eng;
  eng.setTimeStep(1e-6);
  eng.setIntegratorAuto(true);
  eng.circuit().addInductor("L1", 1, 2, 10e-3, 1.0);
  eng.circuit().addResistor("R1", 2, 0, 10.0);
  eng.circuit().addSwitch("S1", 0, 1, 5e-3, 1e6, true);
  eng.setStopTime(2e-3);
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
  EXPECT_LT(peak, 1.0);
  EXPECT_EQ(eng.solverStats().integratorSwitches, 2);
  EXPECT_EQ(eng.integrator(), power_engine::Integrator::Trapezoidal);
}

// Smooth circuits never trigger: auto stays trapezoidal with identical
// numerics (bitwise vs pure trap — same discretization error) and zero
// switches.
TEST(TrBdf2, AutoLeavesSmoothAlone) {
  auto run = [](bool autoOn) {
    Engine eng;
    eng.setTimeStep(1e-6);
    if (autoOn) eng.setIntegratorAuto(true);
    eng.circuit().addVoltageSource("V1", 1, 0, 1.0);
    eng.circuit().addResistor("R1", 1, 2, 1000.0);
    eng.circuit().addCapacitor("C1", 2, 0, 1e-6, 0.0);
    eng.setStopTime(5e-3);
    eng.start();
    while (eng.status() == power_engine::SimulationStatus::Running) eng.step();
    return std::pair(eng.currentSolution().probes.at("v:2"),
                     eng.solverStats().integratorSwitches);
  };
  const auto [vTrap, swTrap] = run(false);
  const auto [vAuto, swAuto] = run(true);
  EXPECT_EQ(swTrap, 0);
  EXPECT_EQ(swAuto, 0);
  EXPECT_DOUBLE_EQ(vAuto, vTrap);
}

// Coarse-step buck (5us at 20kHz: 10 steps/period): late diode detection
// piles up resolves, auto engages BDF2, and Vdc still regulates.
TEST(TrBdf2, AutoRescuesCoarseBuck) {
  auto run = [](bool autoOn) {
    Engine eng;
    eng.loadNetlist(R"(
.param VIN 12 FSW 20k D 0.5
.model SW mosfet_ideal RON=5m ROFF=1Meg EON=10u EOFF=15u
.model DD diode_ideal VF=0.0 RON=10m
V1 1 0 {VIN}
S1 1 2 MODEL=SW
D1 0 2 MODEL=DD
L1 2 3 200u
C1 3 0 200u
Rload 3 0 5
.control pwm switch=S1 freq={FSW} duty={D}
.tran 0.5u 6m
.end
)");
    eng.applyPwmSpecs();
    eng.setTimeStep(5e-6);  // coarse: 10 steps per switching period
    if (autoOn) eng.setIntegratorAuto(true);
    eng.start();
    while (eng.status() == power_engine::SimulationStatus::Running) eng.step();
    return std::pair(eng.currentSolution().probes.at("v:3"),
                     eng.solverStats().integratorSwitches);
  };
  const auto [vTrap, swTrap] = run(false);
  const auto [vAuto, swAuto] = run(true);
  EXPECT_EQ(swTrap, 0);
  EXPECT_GT(swAuto, 0);  // stiffness detected under coarse stepping
  EXPECT_NEAR(vTrap, 6.0, 0.05 * 6.0);
  EXPECT_NEAR(vAuto, vTrap, 0.03 * 6.0);
}
// Adaptive TR-BDF2 with the embedded stage-difference estimate: outer
// frame = dtMax (1ms), the controller subdivides within. Must match
// analytical within 1% with far fewer than 5000 fixed steps, and the
// reported estimate must honor its post-accept invariant.
TEST(TrBdf2, AdaptiveFewStepsAccurate) {
  Engine eng;
  eng.setTimeStep(1e-3);
  eng.setIntegrator(power_engine::Integrator::TrBdf2);
  eng.setAdaptive(1e-3, 1e-9, 1e-3);
  eng.circuit().addVoltageSource("V1", 1, 0, 1.0);
  eng.circuit().addResistor("R1", 1, 2, 1000.0);
  eng.circuit().addCapacitor("C1", 2, 0, 1e-6, 0.0);
  eng.setStopTime(5e-3);
  eng.start();
  while (eng.status() == power_engine::SimulationStatus::Running) eng.step();
  EXPECT_NEAR(eng.currentSolution().probes.at("v:2"), 1.0 - std::exp(-5.0), 0.01);
  EXPECT_LT(eng.solverStats().steps, 1500);
  EXPECT_GT(eng.solverStats().steps, 5);
  EXPECT_GE(eng.lastStepError(), 0.0);
  EXPECT_LE(eng.lastStepError(), 1e-3);
}

// Adaptive TR-BDF2 on the switching Vienna bridge (1ms outer frames):
// finishes with correct Vdc in far fewer than 60000 fixed steps.
TEST(TrBdf2, AdaptiveViennaRuns) {
  Engine eng;
  eng.setTimeStep(1e-3);
  eng.setIntegrator(power_engine::Integrator::TrBdf2);
  eng.setAdaptive(1e-3, 1e-9, 1e-3);
  eng.circuit().addVoltageSource("VA", 1, 10, 0.0);
  eng.circuit().addVoltageSource("VB", 2, 10, 0.0);
  eng.circuit().addVoltageSource("VC", 3, 10, 0.0);
  eng.circuit().addResistor("RAg", 1, 11, 0.5);
  eng.circuit().addResistor("RBg", 2, 12, 0.5);
  eng.circuit().addResistor("RCg", 3, 13, 0.5);
  eng.circuit().addInductor("LA", 11, 4, 5e-3, 0.0);
  eng.circuit().addInductor("LB", 12, 5, 5e-3, 0.0);
  eng.circuit().addInductor("LC", 13, 6, 5e-3, 0.0);
  eng.circuit().addDiode("DAu", 4, 7, 0.7, 10e-3, 1e6);
  eng.circuit().addDiode("DBu", 5, 7, 0.7, 10e-3, 1e6);
  eng.circuit().addDiode("DCu", 6, 7, 0.7, 10e-3, 1e6);
  eng.circuit().addDiode("DAl", 0, 4, 0.7, 10e-3, 1e6);
  eng.circuit().addDiode("DBl", 0, 5, 0.7, 10e-3, 1e6);
  eng.circuit().addDiode("DCl", 0, 6, 0.7, 10e-3, 1e6);
  eng.circuit().addSwitch("SA", 4, 9, 5e-3, 1e6, false);
  eng.circuit().addSwitch("SB", 5, 9, 5e-3, 1e6, false);
  eng.circuit().addSwitch("SC", 6, 9, 5e-3, 1e6, false);
  eng.circuit().addCapacitor("C1", 7, 9, 2e-3, 0.0);
  eng.circuit().addCapacitor("C2", 9, 0, 2e-3, 0.0);
  eng.circuit().addResistor("Rload", 7, 0, 100.0);
  eng.setStopTime(60e-3);
  eng.start();
  const double w = 2.0 * std::acos(-1.0) * 50.0;
  const double vpk = 230.0 * std::sqrt(2.0);
  double sum = 0.0, n = 0.0;
  while (eng.status() == power_engine::SimulationStatus::Running) {
    const double t = eng.time();
    const double e = t >= 10e-3 ? 1.0 : 0.5 * (1.0 - std::cos(std::acos(-1.0) * t / 10e-3));
    eng.circuit().findDevice("VA").value = e * vpk * std::sin(w * t);
    eng.circuit().findDevice("VB").value = e * vpk * std::sin(w * t - 2.0943951023931953);
    eng.circuit().findDevice("VC").value = e * vpk * std::sin(w * t + 2.0943951023931953);
    eng.step();
    if (t > 40e-3) {
      sum += eng.currentSolution().probes.at("v:7");
      n += 1.0;
    }
  }
  EXPECT_NEAR(sum / n, 540.0, 0.04 * 540.0);
  EXPECT_LT(eng.solverStats().steps, 60000);
}
// Netlist `.tran dt tstop TRBDF2|AUTO`: parses, round-trips, and drives
// the solver integrator; bad methods throw.
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
  Engine autoEng;
  autoEng.loadNetlist(R"(
V1 1 0 1
R1 1 0 1k
.tran 1u 10u AUTO
.end
)");
  autoEng.start();
  while (autoEng.status() == power_engine::SimulationStatus::Running) autoEng.step();
  EXPECT_EQ(autoEng.solverStats().integratorSwitches, 0);  // smooth: never fires
  Engine bad;
  EXPECT_THROW(bad.loadNetlist("V1 1 0 1\nR1 1 0 1k\n.tran 1u 1m BOGUS\n.end\n"),
               std::runtime_error);
}
