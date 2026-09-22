#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <gtest/gtest.h>

#include "power_engine/ac.h"
#include "power_engine/control.h"
#include "power_engine/engine.h"
#include "power_engine/measurements.h"

using power_engine::Engine;
using power_engine::control::Carrier;
using power_engine::control::Comparator;
using power_engine::control::HalfBridgeDriver;
using power_engine::control::HysteresisController;
using power_engine::control::PiController;
using power_engine::control::Pwm;
using power_engine::control::StateMachine;
using power_engine::control::Svpwm;
using power_engine::control::TransferFunction;

namespace {
constexpr double kPi = std::numbers::pi;
}  // namespace

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

TEST(PiController, PresetIntegratorBumpless) {
  PiController pi(0.07, 40.0, 0.02, 0.95);
  pi.setIntegrator(0.4167 / 40.0);  // known steady-state duty d0
  // First update from steady state: no rail, duty stays ~d0.
  EXPECT_NEAR(pi.update(0.0, 1.0 / 20e3), 0.4167, 1e-3);
  EXPECT_THROW(pi.setIntegrator(std::numeric_limits<double>::infinity()),
               std::runtime_error);
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

TEST(Filters, LowPassStepHighPassBlockMovingAverage) {
  // 1st-order low-pass, fc=100Hz, dt=100us: step response 1-exp(-t/tau).
  TransferFunction lp = TransferFunction::lowPass(100.0, 100e-6);
  double y = 0.0;
  for (int i = 0; i < 500; ++i) y = lp.update(1.0);  // 50ms = ~31 tau
  EXPECT_NEAR(y, 1.0 - std::exp(-50e-3 * 2.0 * kPi * 100.0), 0.02);
  // High-pass blocks DC: step kicks then decays to zero.
  TransferFunction hp = TransferFunction::highPass(100.0, 100e-6);
  double yh = 0.0;
  for (int i = 0; i < 500; ++i) yh = hp.update(1.0);
  EXPECT_LT(std::abs(yh), 0.05);
  // 4-point moving average: exact fractions, unit DC gain.
  TransferFunction ma = TransferFunction::movingAverage(4);
  EXPECT_DOUBLE_EQ(ma.update(0.0), 0.0);
  EXPECT_DOUBLE_EQ(ma.update(4.0), 1.0);
  EXPECT_DOUBLE_EQ(ma.update(4.0), 2.0);
  EXPECT_DOUBLE_EQ(ma.update(4.0), 3.0);
  EXPECT_DOUBLE_EQ(ma.update(4.0), 4.0);  // window full: all 4s
  EXPECT_THROW(TransferFunction::lowPass(0.0, 1e-3), std::runtime_error);
  EXPECT_THROW(TransferFunction::highPass(100.0, 0.0), std::runtime_error);
  EXPECT_THROW(TransferFunction::movingAverage(0), std::runtime_error);
}

TEST(HysteresisController, BandSwitching) {
  HysteresisController hc(3.0, 0.2, false);
  EXPECT_FALSE(hc.update(3.5));
  EXPECT_TRUE(hc.update(2.5));    // below 2.9: ON
  EXPECT_TRUE(hc.update(3.05));   // inside band: sticky ON
  EXPECT_FALSE(hc.update(3.2));   // above 3.1: OFF
  EXPECT_FALSE(hc.update(2.95));  // inside band: sticky OFF
  EXPECT_TRUE(hc.update(2.8));    // below 2.9: ON
  EXPECT_TRUE(hc.state());
  hc.reset(false);
  EXPECT_FALSE(hc.state());
  EXPECT_THROW(HysteresisController(3.0, -0.1), std::runtime_error);
}

TEST(HysteresisCurrentLoop, RegulatesRLCurrent) {
  // Hysteretic buck: V=12 -> switch -> L=10mH -> R=2ohm with freewheel
  // diode; band 3A +/- 0.1A.
  Engine eng;
  eng.setTimeStep(10e-6);
  eng.circuit().addVoltageSource("V1", 1, 0, 12.0);
  eng.circuit().addSwitch("S1", 1, 2, 5e-3, 1e6, false);
  eng.circuit().addDiode("D1", 0, 2, 0.3, 10e-3, 1e6);
  eng.circuit().addInductor("L1", 2, 3, 10e-3, 0.0);
  eng.circuit().addResistor("R1", 3, 0, 2.0);
  eng.setStopTime(60e-3);
  HysteresisController hc(3.0, 0.2, true);
  eng.start();
  double sum = 0.0, n = 0.0;
  while (eng.status() == power_engine::SimulationStatus::Running) {
    eng.step();
    const double i = eng.deviceCurrent("L1");
    eng.setSwitch("S1", hc.update(i));
    if (eng.time() > 20e-3) {
      EXPECT_GT(i, 2.85);
      EXPECT_LT(i, 3.15);
      sum += i;
      n += 1.0;
    }
  }
  EXPECT_NEAR(sum / n, 3.0, 0.05);
}

TEST(StateMachine, SequencerWithDwell) {
  using Guard = StateMachine::GuardDir;
  StateMachine sm(0, {{0, 1, 4.5, Guard::Above, 1.5e-3},
                      {1, 2, 9.0, Guard::Above, 0.0},
                      {1, 0, 1.0, Guard::Below, 0.0}});
  constexpr double dt = 1e-3;
  EXPECT_EQ(sm.update(5.0, dt), 0);  // guard true, dwell (1.5ms) not met
  EXPECT_EQ(sm.update(5.0, dt), 1);  // 2ms in state: released
  EXPECT_EQ(sm.update(9.5, dt), 2);  // fault: immediate (dwell 0)
  EXPECT_EQ(sm.update(0.0, dt), 2);  // latched, no way back
  sm.reset(0);
  EXPECT_EQ(sm.state(), 0);
  EXPECT_DOUBLE_EQ(sm.timeInState(), 0.0);
  sm.update(0.5, dt);
  EXPECT_NEAR(sm.timeInState(), dt, 1e-12);
  EXPECT_THROW(StateMachine(0, {{0, 1, 1.0, Guard::Above, -1.0}}), std::runtime_error);
}

TEST(HalfBridgeDriver, TimingAndDeadtime) {
  // 20kHz, D=0.5 (T=50us), deadtime 2us.
  HalfBridgeDriver drv(20e3, 0.5, 2e-6);
  EXPECT_FALSE(drv.hi(1e-6));   // inside deadtime after rise
  EXPECT_TRUE(drv.hi(3e-6));
  EXPECT_TRUE(drv.hi(24e-6));
  EXPECT_FALSE(drv.hi(25e-6));  // falling edge sharp
  EXPECT_FALSE(drv.lo(25e-6));  // deadtime after fall
  EXPECT_FALSE(drv.lo(26e-6));
  EXPECT_TRUE(drv.lo(28e-6));
  EXPECT_TRUE(drv.lo(49e-6));
  EXPECT_FALSE(drv.lo(50e-6));  // wraps: deadtime after rise
  for (double t = 0.0; t < 50e-6; t += 100e-9) {
    EXPECT_FALSE(drv.hi(t) && drv.lo(t)) << "shoot-through at t=" << t;
  }
  const double eps = 1e-9;
  EXPECT_NEAR(drv.nextEdge(0.0), 2e-6, eps);
  EXPECT_NEAR(drv.nextEdge(2e-6 + eps), 25e-6, 1e-6);
  EXPECT_NEAR(drv.nextEdge(25e-6 + eps), 27e-6, 1e-6);
  EXPECT_NEAR(drv.nextEdge(27e-6 + eps), 50e-6, 1e-6);
  // Degenerate: ton < deadtime -> hi never on, lo still switches.
  HalfBridgeDriver drv2(20e3, 0.01, 2e-6);
  for (double t = 0.0; t < 50e-6; t += 100e-9) EXPECT_FALSE(drv2.hi(t));
  EXPECT_TRUE(std::isfinite(drv2.nextEdge(0.0)));
  EXPECT_THROW(HalfBridgeDriver(0.0, 0.5, 1e-6), std::runtime_error);
  EXPECT_THROW(HalfBridgeDriver(20e3, 0.5, -1e-6), std::runtime_error);
}

TEST(Svpwm, ModulatorMath) {
  Svpwm sv(10e3);
  const double T = 100e-6;
  // u along alpha: sector 1, T2 = 0, T1 = m*T*sin60, m = 0.5*sqrt(3)/2.
  auto s1 = sv.sequence(0.5, 0.0);
  EXPECT_EQ(s1.sector, 1);
  EXPECT_NEAR(s1.t1, 0.5 * std::sqrt(3.0) / 2.0 * T * std::sin(kPi / 3.0), 1e-12);
  EXPECT_NEAR(s1.t2, 0.0, 1e-12);
  EXPECT_NEAR(s1.t1 + s1.t2 + s1.t0, T, 1e-12);
  // 90 degrees: sector 2, symmetric T1/T2.
  auto s2 = sv.sequence(0.0, 0.5);
  EXPECT_EQ(s2.sector, 2);
  EXPECT_NEAR(s2.t1, s2.t2, 1e-12);
  EXPECT_NEAR(s2.t1 + s2.t2 + s2.t0, T, 1e-12);
  // Overmodulation clamps (no negative zero time).
  auto s3 = sv.sequence(2.0, 0.0);
  EXPECT_GE(s3.t0, 0.0);
  EXPECT_NEAR(s3.t1 + s3.t2 + s3.t0, T, 1e-9);
  // 7-segment symmetry: V0 at both ends, V7 centered.
  auto p0 = sv.switches(1, 0.0, s1.t1, s1.t2, s1.t0);
  EXPECT_FALSE(p0.a || p0.b || p0.c);
  auto pc = sv.switches(1, T / 2.0, s1.t1, s1.t2, s1.t0);
  EXPECT_TRUE(pc.a && pc.b && pc.c);
  EXPECT_THROW(sv.switches(0, 0.0, s1.t1, s1.t2, s1.t0), std::runtime_error);
  EXPECT_THROW(sv.switches(1, T, s1.t1, s1.t2, s1.t0), std::runtime_error);
}

TEST(SvpwmThreePhase, WaveformsCorrect) {
  // 3-phase inverter: Vdc=60, fsw=10kHz, 100Hz fundamental at |u|=0.9,
  // star R load. Each phase fundamental must read 0.9*Vdc/2 = 27V peak
  // at the right phase (A=0, B=-120, C=+120 deg).
  constexpr double kVdc = 60.0, kFsw = 10e3, kF0 = 100.0, kM = 0.9;
  constexpr double kT = 1.0 / kFsw, kR = 10.0;
  constexpr double kDt = 1e-6;  // resolves the shortest (~2.5us) segments
  const Svpwm sv(kFsw);
  Engine eng;
  eng.setTimeStep(kDt);
  eng.circuit().addVoltageSource("Vdc", 4, 0, kVdc);
  eng.circuit().addSwitch("SAh", 4, 1, 5e-3, 1e6, false);
  eng.circuit().addSwitch("SAl", 1, 0, 5e-3, 1e6, true);
  eng.circuit().addSwitch("SBh", 4, 2, 5e-3, 1e6, false);
  eng.circuit().addSwitch("SBl", 2, 0, 5e-3, 1e6, true);
  eng.circuit().addSwitch("SCh", 4, 3, 5e-3, 1e6, false);
  eng.circuit().addSwitch("SCl", 3, 0, 5e-3, 1e6, true);
  eng.circuit().addResistor("RA", 1, 5, kR);
  eng.circuit().addResistor("RB", 2, 5, kR);
  eng.circuit().addResistor("RC", 3, 5, kR);
  // Exact 7-segment edges per switching period (lowers = complement).
  // Resistive load: no transient, so no settle; 2 exact fundamental periods.
  constexpr double kSettle = 0.0, kMeasure = 20e-3, kEnd = kSettle + kMeasure;
  const char* hi[3] = {"SAh", "SBh", "SCh"};
  const char* lo[3] = {"SAl", "SBl", "SCl"};
  for (double tk = 0.0; tk < kEnd; tk += kT) {
    const double w = 2.0 * kPi * kF0;
    const auto seq = sv.sequence(kM * std::cos(w * tk), kM * std::sin(w * tk));
    const double e[8] = {0.0,
                         seq.t0 * 0.25,
                         seq.t0 * 0.25 + seq.t1 * 0.5,
                         seq.t0 * 0.25 + seq.t1 * 0.5 + seq.t2 * 0.5,
                         seq.t0 * 0.75 + seq.t1 * 0.5 + seq.t2 * 0.5,
                         seq.t0 * 0.75 + seq.t1 * 0.5 + seq.t2,
                         seq.t0 * 0.75 + seq.t1 + seq.t2,
                         kT};
    for (int s = 0; s < 8; ++s) {
      const auto st = sv.switches(seq.sector, std::min(e[s] + 1e-12, kT - 1e-12),
                                  seq.t1, seq.t2, seq.t0);
      const bool ph[3] = {st.a, st.b, st.c};
      for (int p = 0; p < 3; ++p) {
        eng.scheduleSwitch(hi[p], ph[p], tk + e[s]);
        eng.scheduleSwitch(lo[p], !ph[p], tk + e[s]);
      }
    }
  }
  eng.setStopTime(kEnd + kT);
  eng.start();
  while (eng.time() < kSettle) eng.step();
  // One Fourier meter per phase-neutral voltage; reference sine as input.
  power_engine::ac::FourierMeter mA, mB, mC;
  mA.begin(kF0);
  mB.begin(kF0);
  mC.begin(kF0);
  double tPrev = eng.time(), aPrev = 0.0, bPrev = 0.0, cPrev = 0.0;
  {
    const auto& pr = eng.currentSolution().probes;
    aPrev = pr.at("v:1") - pr.at("v:5");
    bPrev = pr.at("v:2") - pr.at("v:5");
    cPrev = pr.at("v:3") - pr.at("v:5");
  }
  while (eng.time() < kEnd) {
    eng.step();
    const double t = eng.time();
    const double tm = 0.5 * (tPrev + t);
    const double w = 2.0 * kPi * kF0;
    // Inverter phase convention: va peaks in cosine (alpha axis at t=0).
    const double uA = kM * (kVdc / 2.0) * std::cos(w * tm);
    const double uB = kM * (kVdc / 2.0) * std::cos(w * tm - 2.0 * kPi / 3.0);
    const double uC = kM * (kVdc / 2.0) * std::cos(w * tm + 2.0 * kPi / 3.0);
    const auto& pr = eng.currentSolution().probes;
    const double a = pr.at("v:1") - pr.at("v:5");
    const double b = pr.at("v:2") - pr.at("v:5");
    const double c = pr.at("v:3") - pr.at("v:5");
    mA.sample(tPrev, t, uA, 0.5 * (aPrev + a));
    mB.sample(tPrev, t, uB, 0.5 * (bPrev + b));
    mC.sample(tPrev, t, uC, 0.5 * (cPrev + c));
    tPrev = t;
    aPrev = a;
    bPrev = b;
    cPrev = c;
  }
  eng.stop();
  // Fourier gain Y/X: tracking ratio ~1 (27V peak each on 27V reference).
  const auto pA = mA.result(), pB = mB.result(), pC = mC.result();
  EXPECT_NEAR(pA.mag, 1.0, 0.05);
  EXPECT_NEAR(pB.mag, 1.0, 0.05);
  EXPECT_NEAR(pC.mag, 1.0, 0.05);
  auto wrap180 = [](double deg) {
    while (deg > 180.0) deg -= 360.0;
    while (deg <= -180.0) deg += 360.0;
    return deg;
  };
  // Tracking phase near 0 (sample delay ~1deg); all phases track equally.
  EXPECT_LT(std::abs(wrap180(pA.phaseDeg)), 5.0);
  EXPECT_LT(std::abs(wrap180(pB.phaseDeg - pA.phaseDeg)), 5.0);
  EXPECT_LT(std::abs(wrap180(pC.phaseDeg - pA.phaseDeg)), 5.0);
  // Absolute measured fundamentals are 120deg apart (references sit at
  // 0/-120/+120, ratios carry the common tracking phase).
  EXPECT_NEAR(std::abs(wrap180(pB.phaseDeg - 120.0 - pA.phaseDeg)), 120.0, 5.0);
  EXPECT_NEAR(std::abs(wrap180(pC.phaseDeg + 120.0 - pA.phaseDeg)), 120.0, 5.0);
}

TEST(Clarke, BalancedSineMapsToRotatingVector) {
  // a = sin(wt): alpha = sin(wt), beta = -cos(wt) (amplitude-invariant).
  const auto q0 = power_engine::control::clarke(0.0, -0.8660254037844386, 0.8660254037844386);
  EXPECT_NEAR(q0.alpha, 0.0, 1e-12);
  EXPECT_NEAR(q0.beta, -1.0, 1e-12);
  const auto q1 = power_engine::control::clarke(1.0, -0.5, -0.5);
  EXPECT_NEAR(q1.alpha, 1.0, 1e-12);
  EXPECT_NEAR(q1.beta, 0.0, 1e-12);
  // Zero sequence is rejected: (1,1,1) -> (0,0).
  const auto qz = power_engine::control::clarke(1.0, 1.0, 1.0);
  EXPECT_NEAR(qz.alpha, 0.0, 1e-12);
  EXPECT_NEAR(qz.beta, 0.0, 1e-12);
  EXPECT_THROW(power_engine::control::clarke(0.0, std::numeric_limits<double>::quiet_NaN(), 0.0),
               std::runtime_error);
}
