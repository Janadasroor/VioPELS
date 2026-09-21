#include <cmath>
#include <numbers>
#include <stdexcept>
#include <gtest/gtest.h>

#include "power_engine/control.h"
#include "power_engine/engine.h"
#include "power_engine/machine.h"

using power_engine::Engine;
using power_engine::SimulationStatus;
using power_engine::machine::MechanicalParams;
using power_engine::machine::MechanicalState;
using power_engine::machine::park;
using power_engine::machine::PmsmParams;
using power_engine::machine::pmsmEmf;
using power_engine::machine::pmsmTorque;
using power_engine::machine::stepMechanical;
using power_engine::machine::ThreePhase;

namespace {
constexpr double kPi = std::numbers::pi;
}  // namespace

// Exact J/B stepping: spin-down matches closed form to numerical precision.
TEST(MechanicalMath, ExactStepVsAnalytical) {
  const MechanicalParams p{1e-4, 2e-3};  // tau = 50ms; 1000x100us = 100ms = 2 tau
  MechanicalState st{100.0, 0.0};
  for (int i = 0; i < 1000; ++i) stepMechanical(st, 0.0, 0.0, p, 100e-6);
  EXPECT_NEAR(st.omega, 100.0 * std::exp(-2.0), 1e-9);
  EXPECT_NEAR(st.theta, 100.0 * 0.05 * (1.0 - std::exp(-2.0)), 1e-9);
  // Frictionless uniform accel is exact too.
  const MechanicalParams free{2e-4, 0.0};
  MechanicalState s2{0.0, 0.0};
  for (int i = 0; i < 500; ++i) stepMechanical(s2, 0.4, 0.0, free, 100e-6);
  EXPECT_NEAR(s2.omega, 0.4 / 2e-4 * 50e-3, 1e-9);
  EXPECT_NEAR(s2.theta, 0.5 * 0.4 / 2e-4 * 50e-3 * 50e-3, 1e-9);
  EXPECT_THROW(stepMechanical(st, 0.0, 0.0, {0.0, 0.0}, 1e-3), std::runtime_error);
  EXPECT_THROW(stepMechanical(st, 0.0, 0.0, p, 0.0), std::runtime_error);
}

// Park/EMF/torque consistency: id=0/iq=I alignment, torque == power form.
TEST(PmsmMath, ParkEmfTorqueIdentity) {
  const PmsmParams m{2, 0.05, 0.5, 2e-3, 2e-3, {1e-4, 0.0}};
  const double th = 0.7, w = 120.0, I = 3.0;
  const ThreePhase e = pmsmEmf(th, w, m);
  EXPECT_NEAR(e.a, 2.0 * w * 0.05 * std::sin(2.0 * th), 1e-12);
  EXPECT_NEAR(std::abs(e.a), 2.0 * w * 0.05 * std::abs(std::sin(2.0 * th)), 1e-9);
  const double ia = I * std::sin(2.0 * th);
  const double ib = I * std::sin(2.0 * th - 2.0 * kPi / 3.0);
  const double ic = I * std::sin(2.0 * th + 2.0 * kPi / 3.0);
  double id = 0.0, iq = 0.0;
  park(ia, ib, ic, 2.0 * th, id, iq);
  EXPECT_NEAR(id, 0.0, 1e-9);
  EXPECT_NEAR(iq, I, 1e-9);
  // Torque identity: dq form == electrical power / mechanical speed.
  const double tDq = pmsmTorque(id, iq, m);
  EXPECT_NEAR(tDq, 1.5 * 2.0 * 0.05 * I, 1e-9);
  EXPECT_NEAR(tDq, (e.a * ia + e.b * ib + e.c * ic) / w, 1e-9);
  EXPECT_THROW(pmsmEmf(th, w, {-1, 0.05, 0.5, 2e-3, 2e-3, {1e-4, 0.0}}),
               std::runtime_error);
}

// Open-loop torque proof: fixed 1A amplitude (id=0 refs, self-commutated
// by construction) for 10ms — accel must equal (3/2)*p*lambda*I/J
// (friction correction <= 5% below 15 rad/s inside tolerance).
TEST(PmsmTorque, FixedCurrentAccelMatchesTheory) {
  constexpr double kVdc = 24.0, kP = 2.0, kLam = 0.05, kRs = 0.5, kLs = 2e-3;
  constexpr double kJ = 5e-5, kB = 5e-4, kDt = 1e-6;
  Engine eng;
  eng.setTimeStep(kDt);
  eng.circuit().addVoltageSource("Vdc", 7, 0, kVdc);
  eng.circuit().addSwitch("SAh", 7, 1, 5e-3, 1e6, false);
  eng.circuit().addSwitch("SAl", 1, 0, 5e-3, 1e6, true);
  eng.circuit().addSwitch("SBh", 7, 2, 5e-3, 1e6, false);
  eng.circuit().addSwitch("SBl", 2, 0, 5e-3, 1e6, true);
  eng.circuit().addSwitch("SCh", 7, 3, 5e-3, 1e6, false);
  eng.circuit().addSwitch("SCl", 3, 0, 5e-3, 1e6, true);
  eng.circuit().addResistor("RA", 1, 4, kRs);
  eng.circuit().addInductor("LA", 4, 6, kLs, 0.0);
  eng.circuit().addVoltageSource("EA", 6, 5, 0.0);
  eng.circuit().addResistor("RB", 2, 10, kRs);
  eng.circuit().addInductor("LB", 10, 11, kLs, 0.0);
  eng.circuit().addVoltageSource("EB", 11, 5, 0.0);
  eng.circuit().addResistor("RC", 3, 12, kRs);
  eng.circuit().addInductor("LC", 12, 13, kLs, 0.0);
  eng.circuit().addVoltageSource("EC", 13, 5, 0.0);
  eng.setStopTime(10e-3);
  power_engine::control::HysteresisController hcA(0.0, 0.15, false);
  power_engine::control::HysteresisController hcB(0.0, 0.15, false);
  power_engine::control::HysteresisController hcC(0.0, 0.15, false);
  const MechanicalParams mech{kJ, kB};
  MechanicalState rotor{0.0, 0.0};
  eng.start();
  double wEnd = 0.0;
  while (eng.status() == SimulationStatus::Running) {
    const ThreePhase e = pmsmEmf(rotor.theta, rotor.omega, {2, kLam, kRs, kLs, kLs, mech});
    eng.circuit().findDevice("EA").value = e.a;
    eng.circuit().findDevice("EB").value = e.b;
    eng.circuit().findDevice("EC").value = e.c;
    eng.step();
    const double ia = eng.deviceCurrent("LA");
    const double ib = eng.deviceCurrent("LB");
    const double ic = eng.deviceCurrent("LC");
    const double thE = kP * rotor.theta;
    hcA.setRef(1.0 * std::sin(thE));
    hcB.setRef(1.0 * std::sin(thE - 2.0 * kPi / 3.0));
    hcC.setRef(1.0 * std::sin(thE + 2.0 * kPi / 3.0));
    eng.setSwitch("SAh", hcA.update(ia));
    eng.setSwitch("SAl", !hcA.state());
    eng.setSwitch("SBh", hcB.update(ib));
    eng.setSwitch("SBl", !hcB.state());
    eng.setSwitch("SCh", hcC.update(ic));
    eng.setSwitch("SCl", !hcC.state());
    double id = 0.0, iq = 0.0;
    park(ia, ib, ic, thE, id, iq);
    stepMechanical(rotor, pmsmTorque(id, iq, {2, kLam, kRs, kLs, kLs, mech}), 0.0,
                   mech, kDt);
    wEnd = rotor.omega;
  }
  // Torque (3/2)*p*lambda*1A = 0.15 N m on J = 5e-5: 3000 rad/s^2.
  EXPECT_NEAR(wEnd / 10e-3, 1.5 * kP * kLam * 1.0 / kJ, 0.10 * 1.5 * kP * kLam / kJ);
  EXPECT_GT(wEnd, 0.0);  // motoring direction (not braking)
}

// Closed-loop PMSM speed drive: 24V 6-switch inverter, hysteretic phase
// currents from speed-PI amplitude (id=0 refs) against a 50ms setpoint
// ramp (no windup, no handover transient). Surface PMSM (p=2, lambda=0.05,
// Rs=0.5, Ls=2mH, J=5e-5, B=5e-4).
// Acceptance: setpoint reached, load step rejected, field orientation
// (id~0) and analytic current hold.
TEST(PmsmDrive, SpeedRegulationAndLoadStep) {
  constexpr double kVdc = 24.0, kImax = 2.0, kWref = 100.0;
  constexpr double kP = 2.0, kLam = 0.05, kRs = 0.5, kLs = 2e-3;
  constexpr double kJ = 5e-5, kB = 5e-4, kDt = 1e-6;
  constexpr double kTq = 1.5 * kP * kLam;  // torque per q-amp = 0.15
  Engine eng;
  eng.setTimeStep(kDt);
  eng.circuit().addVoltageSource("Vdc", 7, 0, kVdc);
  eng.circuit().addSwitch("SAh", 7, 1, 5e-3, 1e6, false);
  eng.circuit().addSwitch("SAl", 1, 0, 5e-3, 1e6, true);
  eng.circuit().addSwitch("SBh", 7, 2, 5e-3, 1e6, false);
  eng.circuit().addSwitch("SBl", 2, 0, 5e-3, 1e6, true);
  eng.circuit().addSwitch("SCh", 7, 3, 5e-3, 1e6, false);
  eng.circuit().addSwitch("SCl", 3, 0, 5e-3, 1e6, true);
  eng.circuit().addResistor("RA", 1, 4, kRs);
  eng.circuit().addInductor("LA", 4, 6, kLs, 0.0);
  eng.circuit().addVoltageSource("EA", 6, 5, 0.0);
  eng.circuit().addResistor("RB", 2, 10, kRs);
  eng.circuit().addInductor("LB", 10, 11, kLs, 0.0);
  eng.circuit().addVoltageSource("EB", 11, 5, 0.0);
  eng.circuit().addResistor("RC", 3, 12, kRs);
  eng.circuit().addInductor("LC", 12, 13, kLs, 0.0);
  eng.circuit().addVoltageSource("EC", 13, 5, 0.0);
  // Phase windings (R + L + back-EMF) star at floating node 5.
  constexpr double kStop = 250e-3, kStepT = 120e-3, kTload = 0.05;
  eng.setStopTime(kStop);
  power_engine::control::PiController speedPi(0.0067, 0.22, 0.0, kImax);
  power_engine::control::HysteresisController hcA(0.0, 0.15, false);
  power_engine::control::HysteresisController hcB(0.0, 0.15, false);
  power_engine::control::HysteresisController hcC(0.0, 0.15, false);
  const MechanicalParams mech{kJ, kB};
  MechanicalState rotor{0.0, 0.0};
  double tload = 0.0;
  eng.start();
  double idMean = 0.0, iqMean = 0.0, iMean = 0.0, nMean = 0.0;
  double wPre = 0.0, wMin = 1e18;
  bool stepped = false;
  while (eng.status() == SimulationStatus::Running) {
    const double t = eng.time();
    if (!stepped && t >= kStepT) {
      tload = kTload;
      stepped = true;
    }
    // Back-EMF from rotor state (value-only drive, established pattern).
    const ThreePhase e = pmsmEmf(rotor.theta, rotor.omega, {2, kLam, kRs, kLs, kLs, mech});
    eng.circuit().findDevice("EA").value = e.a;
    eng.circuit().findDevice("EB").value = e.b;
    eng.circuit().findDevice("EC").value = e.c;
    eng.step();
    const double ia = eng.deviceCurrent("LA");
    const double ib = eng.deviceCurrent("LB");
    const double ic = eng.deviceCurrent("LC");
    // Speed PI against the 50ms ramp + velocity feedforward (J*alpha +
    // B*wref)/kTq so tracking lag doesn't wind the integrator; hysteretic
    // id=0 tracking.
    const double wref = std::min(kWref, kWref * t / 50e-3);
    const double alpha = t < 50e-3 ? kWref / 50e-3 : 0.0;
    const double amp = std::min(std::max(speedPi.update(wref - rotor.omega, kDt) +
                                         (kJ * alpha + kB * wref) / kTq,
                                         0.0),
                                kImax);
    const double thE = kP * rotor.theta;
    hcA.setRef(amp * std::sin(thE));
    hcB.setRef(amp * std::sin(thE - 2.0 * kPi / 3.0));
    hcC.setRef(amp * std::sin(thE + 2.0 * kPi / 3.0));
    eng.setSwitch("SAh", hcA.update(ia));
    eng.setSwitch("SAl", !hcA.state());
    eng.setSwitch("SBh", hcB.update(ib));
    eng.setSwitch("SBl", !hcB.state());
    eng.setSwitch("SCh", hcC.update(ic));
    eng.setSwitch("SCl", !hcC.state());
    // Electromechanical coupling: Park -> torque -> exact mechanical step.
    double id = 0.0, iq = 0.0;
    park(ia, ib, ic, thE, id, iq);
    stepMechanical(rotor, pmsmTorque(id, iq, {2, kLam, kRs, kLs, kLs, mech}), tload,
                   mech, kDt);
    if (t >= 100e-3 && t < kStepT) wPre = rotor.omega;
    if (t >= kStepT) wMin = std::min(wMin, rotor.omega);
    if (t >= 200e-3) {
      idMean += id;
      iqMean += iq;
      iMean += amp;
      nMean += 1.0;
    }
  }
  ASSERT_TRUE(stepped);
  // Setpoint reached before the load step; dip then recovery after.
  EXPECT_NEAR(wPre, kWref, 0.03 * kWref);
  // Load step dips then recovers; field orientation + analytic current hold.
  EXPECT_LT(wMin, kWref - 2.0);
  EXPECT_NEAR(rotor.omega, kWref, 0.03 * kWref);
  EXPECT_NEAR(idMean / nMean, 0.0, 0.1);
  EXPECT_NEAR(iqMean / nMean, iMean / nMean, 0.10 * iMean / nMean);
  EXPECT_NEAR(iMean / nMean, (kB * kWref + kTload) / kTq, 0.15 * (kB * kWref + kTload) / kTq);
}
