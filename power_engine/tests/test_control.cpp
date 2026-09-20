#include <cmath>
#include <limits>
#include <stdexcept>
#include <gtest/gtest.h>

#include "power_engine/control.h"
#include "power_engine/engine.h"

using power_engine::Engine;
using power_engine::control::Carrier;
using power_engine::control::Comparator;
using power_engine::control::PiController;
using power_engine::control::Pwm;
using power_engine::control::TransferFunction;

TEST(Pwm, TrailingEdgeLevelsAndEdges) {
  Pwm pwm(20e3, 0.5);  // T = 50us, ton = 25us
  EXPECT_TRUE(pwm.output(0.0));
  EXPECT_TRUE(pwm.output(24.999e-6));
  EXPECT_FALSE(pwm.output(25e-6));
  EXPECT_FALSE(pwm.output(49.999e-6));
  EXPECT_TRUE(pwm.output(50e-6));
  EXPECT_DOUBLE_EQ(pwm.nextEdge(0.0), 25e-6);
  EXPECT_DOUBLE_EQ(pwm.nextEdge(30e-6), 50e-6);
  EXPECT_EQ(pwm.nextEdge(25e-6), 50e-6);
  Pwm full(1e3, 1.0);
  EXPECT_TRUE(full.output(0.123));
  EXPECT_EQ(full.nextEdge(0.0), std::numeric_limits<double>::infinity());
  Pwm off(1e3, 0.0);
  EXPECT_FALSE(off.output(0.123));
}

TEST(Pwm, SymmetricCarrierCentersPulse) {
  Pwm pwm(20e3, 0.5, 0.0, Carrier::Symmetric);  // on during [12.5, 37.5)us
  EXPECT_FALSE(pwm.output(0.0));
  EXPECT_FALSE(pwm.output(12.499e-6));
  EXPECT_TRUE(pwm.output(12.501e-6));
  EXPECT_TRUE(pwm.output(37.499e-6));
  EXPECT_FALSE(pwm.output(37.501e-6));
  EXPECT_DOUBLE_EQ(pwm.nextEdge(0.0), 12.5e-6);
  EXPECT_DOUBLE_EQ(pwm.nextEdge(20e-6), 37.5e-6);
}

TEST(Pwm, ComplementaryDeadtimeNeverOverlaps) {
  Pwm pwm(20e3, 0.5);
  constexpr double td = 200e-9;
  bool sawBothOff = false;
  for (double t = 0.0; t < 150e-6; t += 10e-9) {
    auto c = pwm.complementary(t, td);
    EXPECT_FALSE(c.hi && c.lo) << "overlap at t=" << t;
    if (!c.hi && !c.lo) sawBothOff = true;
  }
  EXPECT_TRUE(sawBothOff);
  // hi on-time shrinks by exactly td; lo likewise.
  double hiOn = 0.0;
  for (double t = 0.0; t < 50e-6; t += 10e-9) {
    if (pwm.complementary(t, td).hi) hiOn += 10e-9;
  }
  EXPECT_NEAR(hiOn, 25e-6 - td, 20e-9);
}

TEST(PiController, SaturationAntiWindup) {
  PiController pi(1.0, 100.0, 0.0, 1.0);
  for (int i = 0; i < 1000; ++i) EXPECT_DOUBLE_EQ(pi.update(10.0, 1e-3), 1.0);
  EXPECT_DOUBLE_EQ(pi.integrator(), 0.0);  // frozen, not wound up
  // Immediate response on sign reversal (no windup lag).
  EXPECT_DOUBLE_EQ(pi.update(-10.0, 1e-3), 0.0);
}

TEST(PiController, TracksWithoutSteadyError) {
  PiController pi(2.0, 50.0, -10.0, 10.0);
  double u = 0.0;
  // First-order plant y += (u - y)*dt/tau in the loop.
  double y = 0.0;
  for (int i = 0; i < 20000; ++i) {
    u = pi.update(1.0 - y, 1e-4);
    y += (u - y) * 1e-4 / 0.05;
  }
  EXPECT_NEAR(y, 1.0, 1e-3);
}

TEST(Comparator, HysteresisBand) {
  Comparator c(1.0, 0.2);  // trip high > 1.1, low < 0.9
  EXPECT_FALSE(c.update(1.05));
  EXPECT_TRUE(c.update(1.15));
  EXPECT_TRUE(c.update(1.05));  // sticks
  EXPECT_FALSE(c.update(0.85));
}

TEST(TransferFunction, LowPassStepMatchesAnalytical) {
  // H(s) = 2 / (1ms*s + 1), unit step -> 2*(1 - exp(-t/1ms)).
  TransferFunction tf = TransferFunction::fromContinuous({2.0}, {1e-3, 1.0}, 100e-6);
  double y = 0.0;
  for (int i = 0; i < 100; ++i) y = tf.update(1.0);  // 10ms = 10*tau
  EXPECT_NEAR(y, 2.0 * (1.0 - std::exp(-10.0)), 0.005 * 2.0);
  // Early transient also tracks (t = 1ms).
  TransferFunction tf2 = TransferFunction::fromContinuous({2.0}, {1e-3, 1.0}, 100e-6);
  double y2 = 0.0;
  for (int i = 0; i < 10; ++i) y2 = tf2.update(1.0);
  EXPECT_NEAR(y2, 2.0 * (1.0 - std::exp(-1.0)), 0.02 * 2.0);
}

TEST(TransferFunction, RejectsBadDefinitions) {
  EXPECT_THROW(TransferFunction::fromContinuous({1.0, 2.0}, {1.0}, 1e-3),
               std::runtime_error);  // improper
  std::vector<double> n(10, 1.0), d(10, 1.0);
  d[0] = 1.0;
  EXPECT_THROW(TransferFunction::fromContinuous(n, d, 1e-3), std::runtime_error);  // order 9
  EXPECT_THROW(TransferFunction::fromDigital({1.0}, {0.0}), std::runtime_error);
}

// Netlist-driven open-loop buck: .control pwm expands to exact edges.
TEST(NetlistPwm, BuckMatchesDutyTheory) {
  Engine eng;
  eng.loadNetlist(R"(
.param VIN 12 FSW 20k D 0.5
.model SW mosfet_ideal RON=5m ROFF=1Meg
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
  EXPECT_GT(eng.pendingEventCount(), 200u);
  eng.start();
  double sum = 0.0, n = 0.0;
  while (eng.status() == power_engine::SimulationStatus::Running) {
    eng.step();
    const double t = eng.time();
    if (t > 6e-3 - 50e-6) {
      sum += eng.currentSolution().probes.at("v:3");
      n += 1.0;
    }
  }
  EXPECT_EQ(eng.pendingEventCount(), 0u);
  EXPECT_NEAR(sum / n, 0.5 * 12.0, 0.02 * 6.0);
}
// Closed-loop buck: voltage-mode control. Plant Vin=12, f=20kHz, L=200uH,
// C=200uF, R=5ohm (LC ~796Hz). Controller samples Vout each period start,
// PI -> duty -> exact scheduled edges. Vref=5V; load steps 5 -> 2.5 ohm
// at 20ms; must be back within 2% by 40ms.
TEST(ClosedLoopBuck, ReachesSetpointAndRejectsLoadStep) {
  constexpr double Vin = 12.0;
  constexpr double freq = 20e3;
  constexpr double T = 1.0 / freq;
  constexpr double dt = 1e-6;
  constexpr double Vref = 5.0;
  constexpr double tStep = 20e-3;
  constexpr double tStop = 40e-3;

  Engine eng;
  eng.setTimeStep(dt);
  eng.circuit().addVoltageSource("Vin", 1, 0, Vin);
  eng.circuit().addSwitch("S1", 1, 2, 5e-3, 1e6, true);
  eng.circuit().addDiode("D1", 0, 2, 0.0, 10e-3, 1e6);
  eng.circuit().addInductor("L1", 2, 3, 200e-6, 0.0);
  eng.circuit().addCapacitor("C1", 3, 0, 200e-6, 0.0);
  eng.circuit().addResistor("Rload", 3, 0, 5.0);
  eng.setStopTime(tStop);

  PiController pi(0.07, 40.0, 0.02, 0.95);
  double duty = 0.4;
  eng.scheduleSwitch("S1", true, 0.0);
  eng.scheduleSwitch("S1", false, duty * T);
  double nextTick = T;
  bool stepped = false;

  double preSum = 0.0, preN = 0.0;
  double postSum = 0.0, postN = 0.0;
  double dutyMin = 1.0, dutyMax = 0.0;

  eng.start();
  while (eng.status() == power_engine::SimulationStatus::Running) {
    eng.step();
    const double t = eng.time();
    const double vout = eng.currentSolution().probes.at("v:3");
    ASSERT_TRUE(std::isfinite(vout)) << "t=" << t;
    if (!stepped && t >= tStep) {
      eng.circuit().findDevice("Rload").value = 2.5;  // load step (value-only)
      stepped = true;
    }
    if (t >= nextTick - 1e-12) {
      duty = pi.update(Vref - vout, T);
      dutyMin = duty < dutyMin ? duty : dutyMin;
      dutyMax = duty > dutyMax ? duty : dutyMax;
      eng.scheduleSwitch("S1", true, nextTick);
      eng.scheduleSwitch("S1", false, nextTick + duty * T);
      nextTick += T;
    }
    if (t >= 18e-3 && t < tStep) {
      preSum += vout;
      preN += 1.0;
    }
    if (t >= tStop - 2e-3) {
      postSum += vout;
      postN += 1.0;
    }
  }
  ASSERT_TRUE(stepped);
  ASSERT_GT(preN, 0.0);
  ASSERT_GT(postN, 0.0);
  EXPECT_NEAR(preSum / preN, Vref, 0.02 * Vref) << "pre-step setpoint";
  EXPECT_NEAR(postSum / postN, Vref, 0.02 * Vref) << "post-step recovery";
  EXPECT_GE(dutyMin, 0.02);
  EXPECT_LE(dutyMax, 0.95);
  EXPECT_GT(eng.solverStats().diodeEvents, 0);
}
