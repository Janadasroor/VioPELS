#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <gtest/gtest.h>

#include "power_engine/control.h"
#include "power_engine/engine.h"
#include "power_engine/machine.h"
#include "power_engine/sensing.h"

using power_engine::Engine;
using power_engine::SimulationStatus;
using power_engine::machine::InductionParams;
using power_engine::machine::InductionState;
using power_engine::machine::inductionCurrents;
using power_engine::machine::inductionSteadyTorque;
using power_engine::machine::inductionTorque;
using power_engine::machine::MechanicalParams;
using power_engine::machine::MechanicalState;
using power_engine::machine::park;
using power_engine::machine::inversePark;
using power_engine::machine::FocController;
using power_engine::machine::FocParams;
using power_engine::machine::PmsmParams;
using power_engine::machine::pmsmEmf;
using power_engine::machine::pmsmTorque;
using power_engine::machine::stepInduction;
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

// --- Induction machine (dq synchronous frame, V/f) ---

namespace {

const InductionParams kInd{1.0, 0.8, 0.15, 0.15, 0.14, 2, {0.05, 0.002}};
constexpr double kWe = 2.0 * kPi * 50.0;  // 50Hz synchronous frame
constexpr double kVpk = 230.0 * 1.4142135623730951;  // 230Vrms phase

// Analytic slip for a load torque (rising branch): dense scan + interp.
// Requires load below pullout (asserted by callers via margin).
// Includes viscous friction (B*omega electrical equilibrium): the sim
// balances Te = Tload + B*w, so the reference must too (fixed point,
// matters at light load where friction rivals Tload).
double analyticSlip(const InductionParams& m, double vRms, double freq, double tload) {
  const double we = 2.0 * kPi * freq;
  auto invert = [&](double t) {
    double tMax = 0.0;
    for (int i = 0; i < 400; ++i) {
      const double s = 1e-4 * std::pow(0.5 / 1e-4, i / 399.0);
      tMax = std::max(tMax, inductionSteadyTorque(m, vRms, freq, s));
    }
    EXPECT_GT(tMax, t / 0.5) << "load beyond sane pullout margin";
    double sLo = 1e-4, sHi = 0.5;
    for (int i = 0; i < 400; ++i) {
      const double s = 1e-4 * std::pow(0.5 / 1e-4, i / 399.0);
      if (inductionSteadyTorque(m, vRms, freq, s) >= t) {
        sHi = s;
        break;
      }
      sLo = s;
    }
    return 0.5 * (sLo + sHi);
  };
  double s = invert(tload);
  for (int k = 0; k < 4; ++k) {
    s = invert(tload + m.mech.b * we * (1.0 - s) / m.polePairs);
  }
  return s;
}

// V/f run at fixed load: returns steady slip (mean over last 0.3s).
double vfSlip(double tload, double tEnd, double dt) {
  InductionState st;
  MechanicalState rotor{0.0, 0.0};
  double t = 0.0, slipSum = 0.0, nSum = 0.0;
  while (t < tEnd) {
    stepInduction(st, 0.0, kVpk, kWe, kInd.polePairs * rotor.omega, kInd, dt);
    double ids = 0.0, iqs = 0.0, idr = 0.0, iqr = 0.0;
    inductionCurrents(st, kInd, ids, iqs, idr, iqr);
    stepMechanical(rotor, inductionTorque(st, ids, iqs, kInd), tload, kInd.mech, dt);
    t += dt;
    if (t > tEnd - 0.3) {
      slipSum += (kWe - kInd.polePairs * rotor.omega) / kWe;
      nSum += 1.0;
    }
  }
  return slipSum / nSum;
}

}  // namespace

TEST(InductionMath, SteadyTorqueSanityAndValidation) {
  // Locked-rotor torque > 0 (self-starting) and pullout exists.
  EXPECT_GT(inductionSteadyTorque(kInd, 230.0, 50.0, 1.0), 0.0);
  double tMax = 0.0;
  for (int i = 0; i < 200; ++i) {
    tMax = std::max(tMax, inductionSteadyTorque(kInd, 230.0, 50.0, 0.001 + i * 0.001));
  }
  EXPECT_GT(tMax, 40.0);
  EXPECT_THROW(inductionSteadyTorque(kInd, 230.0, 50.0, 0.0), std::runtime_error);
  EXPECT_THROW(inductionSteadyTorque(kInd, 230.0, 50.0, 1.5), std::runtime_error);
  EXPECT_THROW(inductionSteadyTorque({1.0, 0.8, 0.15, 0.15, 0.2, 2, {0.05, 0.002}},
                                     230.0, 50.0, 0.05),
               std::runtime_error);  // Lm^2 > Ls*Lr
  InductionState st;
  EXPECT_THROW(stepInduction(st, 0.0, 0.0, kWe, 0.0, kInd, 0.0), std::runtime_error);
}

// No-load run-up: reaches synchronous speed (slip -> ~0, settles).
TEST(InductionRunUp, ReachesSynchronousSpeed) {
  const double s = vfSlip(0.0, 2.0, 50e-6);
  EXPECT_LT(s, 0.03);
  EXPECT_GT(s, -0.001);
}

// Loaded torque-speed points: sim slip vs analytic inversion within 10%.
// Loads stay below locked-rotor torque (~19 N m) so runs self-start.
TEST(InductionTorqueSpeed, MatchesEquivalentCircuit) {
  for (double tload : {2.0, 5.0, 10.0, 15.0}) {
    const double sSim = vfSlip(tload, 2.0, 50e-6);
    const double sRef = analyticSlip(kInd, 230.0, 50.0, tload);
    EXPECT_GT(sRef, 0.0);
    EXPECT_NEAR(sSim, sRef, 0.10 * sRef) << "tload=" << tload;
  }
}

// SVPWM-fed spot (10kHz ideal pole voltages, 2us steps): same 10 N m load
// settles to the V/f slip within 10% (drive harmonics don't move the mean).
TEST(InductionSvpwm, MatchesFundamentalDrive) {
  constexpr double kVdc = 600.0, kM = 325.0 / 300.0;  // |u| = 1.083 < 1.1547
  constexpr double kFsw = 10e3, kT = 100e-6, kDt = 2e-6;
  const power_engine::control::Svpwm sv(kFsw);
  InductionState st;
  MechanicalState rotor{0.0, 0.0};
  const double tEnd = 1.5, wRef = kWe;
  double t = 0.0, slipSum = 0.0, nSum = 0.0;
  while (t < tEnd) {
    const double the = wRef * t;
    const auto seq = sv.sequence(kM * std::cos(the), kM * std::sin(the));
    const double tk = std::floor(t / kT) * kT;
    const auto ps = sv.switches(seq.sector, std::min(t - tk + 1e-12, kT - 1e-12),
                                seq.t1, seq.t2, seq.t0);
    const double sSum = (ps.a ? 1.0 : 0.0) + (ps.b ? 1.0 : 0.0) + (ps.c ? 1.0 : 0.0);
    const double vaN = ((ps.a ? 1.0 : 0.0) - sSum / 3.0) * kVdc;
    const double vbN = ((ps.b ? 1.0 : 0.0) - sSum / 3.0) * kVdc;
    const double vcN = ((ps.c ? 1.0 : 0.0) - sSum / 3.0) * kVdc;
    // Park at the fundamental angle (lib convention, shared with V/f).
    double vd = 0.0, vq = 0.0;
    power_engine::machine::park(vaN, vbN, vcN, the, vd, vq);
    stepInduction(st, vd, vq, wRef, kInd.polePairs * rotor.omega, kInd, kDt);
    double ids = 0.0, iqs = 0.0, idr = 0.0, iqr = 0.0;
    inductionCurrents(st, kInd, ids, iqs, idr, iqr);
    stepMechanical(rotor, inductionTorque(st, ids, iqs, kInd), 10.0, kInd.mech, kDt);
    t += kDt;
    if (t > tEnd - 0.3) {
      slipSum += (kWe - kInd.polePairs * rotor.omega) / kWe;
      nSum += 1.0;
    }
  }
  const double sPwm = slipSum / nSum;
  const double sVf = vfSlip(10.0, 2.0, 50e-6);
  EXPECT_NEAR(sPwm, sVf, 0.10 * sVf);
}

TEST(PmsmMath, InverseParkRoundTrip) {
  // park(inversePark(vd, vq)) == (vd, vq) at several angles (balanced).
  for (double thE : {0.0, 0.3, 1.1, 2.5, -0.7}) {
    const auto v = inversePark(3.0, -4.0, thE);
    double id = 0.0, iq = 0.0;
    park(v.a, v.b, v.c, thE, id, iq);
    EXPECT_NEAR(id, 3.0, 1e-9) << "thE=" << thE;
    EXPECT_NEAR(iq, -4.0, 1e-9) << "thE=" << thE;
  }
  // Motoring convention: vd=0, vq>0 gives phase-a voltage in phase with
  // the back-EMF sine (ia = I*sin(thE) convention).
  const auto v = inversePark(0.0, 10.0, 0.5);
  EXPECT_NEAR(v.a, 10.0 * std::sin(0.5), 1e-9);
  EXPECT_THROW(inversePark(0.0, std::numeric_limits<double>::quiet_NaN(), 0.0),
               std::runtime_error);
}

// Degenerate motors are rejected at construction (also silences divide-by-zero
// diagnostics on divisor parameters).
TEST(PmsmFoc, RejectsBadMotor) {
  FocParams fp;
  fp.motor = {2, 0.05, 0.5, 2e-3, 2e-3, {5e-5, 5e-4}};
  const FocController good(fp);
  (void)good;
  FocParams bad = fp;
  bad.motor.ld = 0.0;
  EXPECT_THROW({ FocController c(bad); (void)c; }, std::runtime_error);
  bad = fp;
  bad.motor.lambdaPm = 0.0;
  EXPECT_THROW({ FocController c(bad); (void)c; }, std::runtime_error);
  bad = fp;
  bad.vdc = -1.0;
  EXPECT_THROW({ FocController c(bad); (void)c; }, std::runtime_error);
}

// --- Voltage-form FOC: cascaded speed + dq current loops with decoupling,
// id* = 0, carrier-PWM drive. Same plant and load profile as the
// hysteretic speed test, plus field-orientation and reference tracking.
TEST(PmsmFoc, SpeedRampLoadStepAndOrientation) {
  constexpr double kVdc = 24.0, kP = 2.0, kLam = 0.05, kRs = 0.5, kLs = 2e-3;
  constexpr double kJ = 5e-5, kB = 5e-4, kDt = 1e-6, kWref = 100.0;
  constexpr double kTq = 1.5 * kP * kLam;
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
  constexpr double kStop = 300e-3, kStepT = 120e-3, kTload = 0.05;
  eng.setStopTime(kStop);
  FocParams fp;
  fp.motor = {2, kLam, kRs, kLs, kLs, {kJ, kB}};
  fp.vdc = kVdc;
  FocController foc(fp);
  const MechanicalParams mech{kJ, kB};
  MechanicalState rotor{0.0, 0.0};
  double tload = 0.0;
  eng.start();
  double wPre = 0.0, wMin = 1e18, idMean = 0.0, iqMean = 0.0, iqRefMean = 0.0, nMean = 0.0;
  bool stepped = false;
  while (eng.status() == SimulationStatus::Running) {
    const double t = eng.time();
    if (!stepped && t >= kStepT) {
      tload = kTload;
      stepped = true;
    }
    const ThreePhase e = pmsmEmf(rotor.theta, rotor.omega, fp.motor);
    eng.circuit().findDevice("EA").value = e.a;
    eng.circuit().findDevice("EB").value = e.b;
    eng.circuit().findDevice("EC").value = e.c;
    eng.step();
    const ThreePhase im{eng.deviceCurrent("LA"), eng.deviceCurrent("LB"),
                        eng.deviceCurrent("LC")};
    const double wref = std::min(kWref, kWref * t / 50e-3);
    const double alpha = t < 50e-3 ? kWref / 50e-3 : 0.0;
    const auto g = foc.update(t, wref, alpha, rotor.omega, im, kP * rotor.theta, kDt);
    eng.setSwitch("SAh", g.aHi);
    eng.setSwitch("SAl", !g.aHi);
    eng.setSwitch("SBh", g.bHi);
    eng.setSwitch("SBl", !g.bHi);
    eng.setSwitch("SCh", g.cHi);
    eng.setSwitch("SCl", !g.cHi);
    stepMechanical(rotor, pmsmTorque(foc.id(), foc.iq(), fp.motor), tload, mech, kDt);
    if (t >= 100e-3 && t < kStepT) wPre = rotor.omega;
    if (t >= kStepT) wMin = std::min(wMin, rotor.omega);
    if (t >= 260e-3) {
      idMean += foc.id();
      iqMean += foc.iq();
      iqRefMean += foc.iqRef();
      nMean += 1.0;
    }
  }
  ASSERT_TRUE(stepped);
  // Setpoint reached before the load step; dip then recovery after.
  EXPECT_NEAR(wPre, kWref, 0.03 * kWref);
  EXPECT_LT(wMin, kWref - 2.0);
  EXPECT_NEAR(rotor.omega, kWref, 0.03 * kWref);
  // Field orientation: id pinned at zero, iq tracks its reference, and the
  // reference matches the analytic torque demand.
  EXPECT_NEAR(idMean / nMean, 0.0, 0.1);
  EXPECT_NEAR(iqMean / nMean, iqRefMean / nMean, 0.10 * iqRefMean / nMean);
  EXPECT_NEAR(iqRefMean / nMean, (kB * kWref + kTload) / kTq,
              0.15 * (kB * kWref + kTload) / kTq);
}

// --- SVPWM-fed FOC variant: identical current loops, symmetric 7-segment
// space-vector gates at the same switching frequency instead of carrier
// PWM. Same plant, same ramp + load-step profile, same acceptance bounds.
TEST(PmsmFocSvpwm, SpeedRampLoadStepAndOrientation) {
  constexpr double kVdc = 24.0, kP = 2.0, kLam = 0.05, kRs = 0.5, kLs = 2e-3;
  constexpr double kJ = 5e-5, kB = 5e-4, kDt = 1e-6, kWref = 100.0;
  constexpr double kTq = 1.5 * kP * kLam;
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
  constexpr double kStop = 300e-3, kStepT = 120e-3, kTload = 0.05;
  eng.setStopTime(kStop);
  FocParams fp;
  fp.motor = {2, kLam, kRs, kLs, kLs, {kJ, kB}};
  fp.vdc = kVdc;
  fp.modulation = FocParams::Modulation::Svpwm;
  FocController foc(fp);
  const MechanicalParams mech{kJ, kB};
  MechanicalState rotor{0.0, 0.0};
  double tload = 0.0;
  eng.start();
  double wPre = 0.0, wMin = 1e18, idMean = 0.0, iqMean = 0.0, iqRefMean = 0.0, nMean = 0.0;
  bool stepped = false;
  while (eng.status() == SimulationStatus::Running) {
    const double t = eng.time();
    if (!stepped && t >= kStepT) {
      tload = kTload;
      stepped = true;
    }
    const ThreePhase e = pmsmEmf(rotor.theta, rotor.omega, fp.motor);
    eng.circuit().findDevice("EA").value = e.a;
    eng.circuit().findDevice("EB").value = e.b;
    eng.circuit().findDevice("EC").value = e.c;
    eng.step();
    const ThreePhase im{eng.deviceCurrent("LA"), eng.deviceCurrent("LB"),
                        eng.deviceCurrent("LC")};
    const double wref = std::min(kWref, kWref * t / 50e-3);
    const double alpha = t < 50e-3 ? kWref / 50e-3 : 0.0;
    const auto g = foc.update(t, wref, alpha, rotor.omega, im, kP * rotor.theta, kDt);
    eng.setSwitch("SAh", g.aHi);
    eng.setSwitch("SAl", !g.aHi);
    eng.setSwitch("SBh", g.bHi);
    eng.setSwitch("SBl", !g.bHi);
    eng.setSwitch("SCh", g.cHi);
    eng.setSwitch("SCl", !g.cHi);
    stepMechanical(rotor, pmsmTorque(foc.id(), foc.iq(), fp.motor), tload, mech, kDt);
    if (t >= 100e-3 && t < kStepT) wPre = rotor.omega;
    if (t >= kStepT) wMin = std::min(wMin, rotor.omega);
    if (t >= 260e-3) {
      idMean += foc.id();
      iqMean += foc.iq();
      iqRefMean += foc.iqRef();
      nMean += 1.0;
    }
  }
  ASSERT_TRUE(stepped);
  // Setpoint reached before the load step; dip then recovery after.
  EXPECT_NEAR(wPre, kWref, 0.03 * kWref);
  EXPECT_LT(wMin, kWref - 2.0);
  EXPECT_NEAR(rotor.omega, kWref, 0.03 * kWref);
  // Field orientation: id pinned at zero, iq tracks its reference, and the
  // reference matches the analytic torque demand.
  EXPECT_NEAR(idMean / nMean, 0.0, 0.1);
  EXPECT_NEAR(iqMean / nMean, iqRefMean / nMean, 0.10 * iqRefMean / nMean);
  EXPECT_NEAR(iqRefMean / nMean, (kB * kWref + kTload) / kTq,
              0.15 * (kB * kWref + kTload) / kTq);
}

// --- Field weakening: reference motor on a 48V bus (base ~228 rad/s mech)
// driven to 260 rad/s. Above base the EMF alone exceeds the bus, so the
// drive cannot hold setpoint without FW; FW injects positive id* (which
// lowers terminal voltage under this codebase's Park convention — proven
// by open-loop plant ID to <1.2%) and holds 97% modulation with all loops
// tracking. A no-FW contrast run on the same plant stalls at the voltage
// ceiling, proving the FW does the work.
// Test-vehicle rationale (all measured): FW needs regulable headroom. On
// the 24V bus the reference motor needs extreme id* at 98%+ modulation
// for ANY overspeed and cascaded PIs deadlock there (frozen integrators +
// angle tilt under the shared clamp). A 50mOhm/20mH variant has leverage
// but L/R = 0.4s (100x the validated envelope) and never settles inside
// test horizons. Deep-corner joint anti-windup stays a recorded follow-up.
TEST(PmsmFocFw, AboveBaseSpeedDemagAndTrack) {
  constexpr double kVdc = 48.0, kP = 2.0, kLam = 0.05, kRs = 0.5, kLs = 2e-3;
  constexpr double kJ = 5e-5, kB = 5e-4, kDt = 1e-6, kWref = 260.0;
  auto build = [](Engine& eng) {
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
    eng.setStopTime(800e-3);
  };
  auto run = [&](Engine& eng, FocController& foc, const MechanicalParams& mech,
                 MechanicalState& rotor, double* wMean, double* idMean) {
    double w = 0.0, id = 0.0, n = 0.0;
    while (eng.status() == SimulationStatus::Running) {
      const double t = eng.time();
      const ThreePhase e = pmsmEmf(rotor.theta, rotor.omega, foc.motor());
      eng.circuit().findDevice("EA").value = e.a;
      eng.circuit().findDevice("EB").value = e.b;
      eng.circuit().findDevice("EC").value = e.c;
      eng.step();
      const ThreePhase im{eng.deviceCurrent("LA"), eng.deviceCurrent("LB"),
                          eng.deviceCurrent("LC")};
      const double wref = std::min(kWref, kWref * t / 300e-3);
      const double alpha = t < 300e-3 ? kWref / 300e-3 : 0.0;
      const auto g = foc.update(t, wref, alpha, rotor.omega, im, kP * rotor.theta, kDt);
      eng.setSwitch("SAh", g.aHi);
      eng.setSwitch("SAl", !g.aHi);
      eng.setSwitch("SBh", g.bHi);
      eng.setSwitch("SBl", !g.bHi);
      eng.setSwitch("SCh", g.cHi);
      eng.setSwitch("SCl", !g.cHi);
      stepMechanical(rotor, pmsmTorque(foc.id(), foc.iq(), foc.motor()), 0.0, mech,
                     kDt);
      if (t >= 600e-3) {
        w += rotor.omega;
        id += foc.id();
        n += 1.0;
      }
    }
    if (wMean) *wMean = w / n;
    if (idMean) *idMean = id / n;
  };
  const MechanicalParams mech{kJ, kB};
  // Contrast first: no FW stalls at the voltage ceiling (~base speed).
  double wNoFw = 0.0;
  {
    Engine eng;
    build(eng);
    FocParams fp;
    fp.motor = {2, kLam, kRs, kLs, kLs, {kJ, kB}};
    fp.vdc = kVdc;
    FocController foc(fp);
    MechanicalState rotor{0.0, 0.0};
    eng.start();
    run(eng, foc, mech, rotor, &wNoFw, nullptr);
  }
  EXPECT_LT(wNoFw, 232.0);
  // With FW: setpoint held at 1.3x base with sustained demagnetization.
  double wFw = 0.0, idFw = 0.0;
  {
    Engine eng;
    build(eng);
    FocParams fp;
    fp.motor = {2, kLam, kRs, kLs, kLs, {kJ, kB}};
    fp.vdc = kVdc;
    fp.fieldWeakening = true;
    fp.maxCurrent = 6.0;
    FocController foc(fp);
    MechanicalState rotor{0.0, 0.0};
    eng.start();
    run(eng, foc, mech, rotor, &wFw, &idFw);
    EXPECT_GT(foc.idRef(), 0.0);
  }
  EXPECT_NEAR(wFw, kWref, 0.03 * kWref);
  EXPECT_GT(idFw, 1.0);
}

// FW/MTPA law corners, pinned without a sim (direct update() calls are
// deterministic: no engine, no switching, fixed 1us ticks).
TEST(PmsmFocFw, LawCornersWithoutSim) {
  const ThreePhase zero{0.0, 0.0, 0.0};
  // Below base speed (w = 50, we = 100, vlim = 0.114 >> lam): no demag.
  {
    FocParams fp;
    fp.motor = {2, 0.05, 0.5, 2e-3, 2e-3, {5e-5, 5e-4}};
    fp.vdc = 24.0;
    fp.fieldWeakening = true;
    FocController foc(fp);
    for (int k = 0; k < 3000; ++k) foc.update(k * 1e-6, 50.0, 0.0, 50.0, zero, 0.0, 1e-6);
    EXPECT_EQ(foc.idRef(), 0.0);
  }
  // Anti-runaway yoke, pinned open-loop: with no plant response (id held
  // at 0) demand cannot climb past delivery plus 2A, however saturated the
  // voltage loop is. Slow trim (200ms at ki = 3) reaches the +-5A authority
  // here; the yoke, not the authority, sets the pinned demand.
  {
    FocParams fp;
    fp.motor = {2, 0.05, 0.5, 2e-3, 2e-3, {5e-5, 5e-4}};
    fp.vdc = 24.0;
    fp.fieldWeakening = true;
    fp.maxCurrent = 30.0;
    FocController foc(fp);
    for (int k = 0; k < 200000; ++k)
      foc.update(k * 1e-6, 1000.0, 0.0, 1000.0, zero, 0.0, 1e-6);
    EXPECT_DOUBLE_EQ(foc.idRef(), 2.0);
  }
  // MTPA below base speed: IPM (Lq > Ld) demagnetizes for torque while the
  // surface twin (Ld == Lq) holds id* = 0 under the same stimulus. iq comes
  // from velocity feedforward into already-matching currents (no speed
  // error, so nothing winds up and the bus stays far from the ceiling).
  {
    FocParams ipm;
    ipm.motor = {2, 0.05, 0.5, 2e-3, 4e-3, {5e-5, 5e-4}};
    ipm.vdc = 24.0;
    ipm.fieldWeakening = true;
    FocController focIpm(ipm);
    FocParams spm = ipm;
    spm.motor = {2, 0.05, 0.5, 2e-3, 2e-3, {5e-5, 5e-4}};
    FocController focSpm(spm);
    // iq demand from feedforward only: (J*200+B*50)/kTq = 0.2333A.
    const ThreePhase i0233 = inversePark(0.0, 0.07 / 0.3, 0.0);
    for (int k = 0; k < 3000; ++k) {
      focIpm.update(k * 1e-6, 50.0, 200.0, 50.0, i0233, 0.0, 1e-6);
      focSpm.update(k * 1e-6, 50.0, 200.0, 50.0, i0233, 0.0, 1e-6);
    }
    EXPECT_LT(focIpm.idRef(), -1e-4);  // MTPA active (vbus far from ceiling)
    EXPECT_DOUBLE_EQ(focSpm.idRef(), 0.0);
  }
  // FW disabled: id* identically 0 even with a huge speed error.
  {
    FocParams fp;
    fp.motor = {2, 0.05, 0.5, 2e-3, 2e-3, {5e-5, 5e-4}};
    fp.vdc = 24.0;
    FocController foc(fp);
    for (int k = 0; k < 3000; ++k) foc.update(k * 1e-6, 200.0, 0.0, 50.0, zero, 0.0, 1e-6);
    EXPECT_EQ(foc.idRef(), 0.0);
  }
}

// --- Sensed FOC: same plant/profile as the ideal-theta test, but the loop
// runs on a 1024-count encoder (quantized angle), a 2ms filtered speed
// estimate, and carrier-midpoint-sampled currents (held between ticks).
// Acceptance mirrors the ideal test with headroom for the sensing chain.
TEST(PmsmFocSensed, SpeedRampLoadStepAndOrientation) {
  constexpr double kVdc = 24.0, kP = 2.0, kLam = 0.05, kRs = 0.5, kLs = 2e-3;
  constexpr double kJ = 5e-5, kB = 5e-4, kDt = 1e-6, kWref = 100.0;
  constexpr double kTq = 1.5 * kP * kLam;
  constexpr double kTcar = 50e-6;  // carrier period: sampling ticks at midpoints
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
  constexpr double kStop = 300e-3, kStepT = 120e-3, kTload = 0.05;
  eng.setStopTime(kStop);
  FocParams fp;
  fp.motor = {2, kLam, kRs, kLs, kLs, {kJ, kB}};
  fp.vdc = kVdc;
  FocController foc(fp);
  const MechanicalParams mech{kJ, kB};
  MechanicalState rotor{0.0, 0.0};
  power_engine::sensing::Encoder enc(1024);
  power_engine::sensing::SpeedEstimator vest(2e-3);
  double tload = 0.0;
  eng.start();
  vest.update(enc.quantized(0.0), 0.0);
  ThreePhase im{0.0, 0.0, 0.0};
  double thS = 0.0, wS = 0.0, nextSamp = kTcar / 2.0;
  double wPre = 0.0, wMin = 1e18, idMean = 0.0, iqMean = 0.0, iqRefMean = 0.0, nMean = 0.0;
  bool stepped = false;
  while (eng.status() == SimulationStatus::Running) {
    const double t = eng.time();
    if (!stepped && t >= kStepT) {
      tload = kTload;
      stepped = true;
    }
    const ThreePhase e = pmsmEmf(rotor.theta, rotor.omega, fp.motor);
    eng.circuit().findDevice("EA").value = e.a;
    eng.circuit().findDevice("EB").value = e.b;
    eng.circuit().findDevice("EC").value = e.c;
    eng.step();
    if (t + kDt >= nextSamp) {  // midpoint sample, held between ticks
      im = {eng.deviceCurrent("LA"), eng.deviceCurrent("LB"), eng.deviceCurrent("LC")};
      thS = enc.quantized(rotor.theta);
      wS = vest.update(thS, nextSamp);
      nextSamp += kTcar;
    }
    const double wref = std::min(kWref, kWref * t / 50e-3);
    const double alpha = t < 50e-3 ? kWref / 50e-3 : 0.0;
    const auto g = foc.update(t, wref, alpha, wS, im, kP * thS, kDt);
    eng.setSwitch("SAh", g.aHi);
    eng.setSwitch("SAl", !g.aHi);
    eng.setSwitch("SBh", g.bHi);
    eng.setSwitch("SBl", !g.bHi);
    eng.setSwitch("SCh", g.cHi);
    eng.setSwitch("SCl", !g.cHi);
    stepMechanical(rotor, pmsmTorque(foc.id(), foc.iq(), fp.motor), tload, mech, kDt);
    if (t >= 100e-3 && t < kStepT) wPre = rotor.omega;
    if (t >= kStepT) wMin = std::min(wMin, rotor.omega);
    if (t >= 260e-3) {
      idMean += foc.id();
      iqMean += foc.iq();
      iqRefMean += foc.iqRef();
      nMean += 1.0;
    }
  }
  ASSERT_TRUE(stepped);
  // Setpoint reached before the load step; dip then recovery after. Bounds
  // carry headroom over the ideal-theta +-3% for quantization + estimation
  // lag + sampled currents (measured cost: +2% track).
  EXPECT_NEAR(wPre, kWref, 0.05 * kWref);
  EXPECT_LT(wMin, kWref - 2.0);
  EXPECT_NEAR(rotor.omega, kWref, 0.05 * kWref);
  // Field orientation holds on estimated angle; iq tracks as commanded.
  EXPECT_NEAR(idMean / nMean, 0.0, 0.15);
  EXPECT_NEAR(iqMean / nMean, iqRefMean / nMean, 0.10 * iqRefMean / nMean);
  EXPECT_NEAR(iqRefMean / nMean, (kB * kWref + kTload) / kTq,
              0.15 * (kB * kWref + kTload) / kTq);
}

// MTPV-lite, pinned open-loop: at extreme overspeed (w = 800, we = 1600)
// no id fits the ellipse at full torque current, so iqRef_ is crushed
// geometrically while id demand stays yoke-pinned. Deterministic: the
// MTPV loop completes inside every single update() call.
TEST(PmsmFocFw, MtpvDeratesTorqueDeterministically) {
  const ThreePhase zero{0.0, 0.0, 0.0};
  FocParams fp;
  fp.motor = {2, 0.05, 0.5, 2e-3, 2e-3, {5e-5, 5e-4}};
  fp.vdc = 24.0;
  fp.fieldWeakening = true;
  fp.maxCurrent = 30.0;
  FocController foc(fp);
  for (int k = 0; k < 20000; ++k)
    foc.update(k * 1e-6, 900.0, 0.0, 800.0, zero, 0.0, 1e-6);
  // Relief crushed the torque demand far below the 2A speed cap...
  EXPECT_LT(foc.iqRef(), 0.01);
  // ...while id demand sits at the yoke pin (no bricking, no windup).
  EXPECT_DOUBLE_EQ(foc.idRef(), 2.0);
}

// Closed-loop sensorless FOC: no Encoder/SpeedEstimator anywhere — the
// voltage-model observer (commanded v + sampled i) supplies angle/speed.
// Honest I-f startup: forced frame θf ramps to speed, then a
// speed-scheduled blend hands the loop to θ̂e (handoff uses only θf_dot
// + observer flux, never the true rotor angle).
TEST(PmsmFocSensorless, SpeedRampLoadStepAndOrientation) {
  constexpr double kVdc = 24.0, kP = 2.0, kLam = 0.05, kRs = 0.5, kLs = 2e-3;
  constexpr double kJ = 5e-5, kB = 5e-4, kDt = 1e-6, kWref = 100.0;
  constexpr double kTq = 1.5 * kP * kLam;
  constexpr double kTcar = 50e-6;
  constexpr double kRampA = 1000.0;    // I-f ramp [rad/s^2], reaches kWref at 100ms
  constexpr double kBlend0 = 100e-3;   // blend start [s]
  constexpr double kBlend1 = 200e-3;   // blend end (pure sensorless after) [s]
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
  constexpr double kStop = 500e-3, kStepT = 320e-3, kTload = 0.05;
  eng.setStopTime(kStop);
  FocParams fp;
  fp.motor = {2, kLam, kRs, kLs, kLs, {kJ, kB}};
  fp.vdc = kVdc;
  FocController foc(fp);
  const MechanicalParams mech{kJ, kB};
  MechanicalState rotor{0.0, 0.0};
  power_engine::sensing::SensorlessParams sp;
  sp.rs = kRs;
  sp.l = kLs;
  sp.lambdaPm = kLam;
  power_engine::sensing::SensorlessObserver obs(sp);
  double tload = 0.0;
  eng.start();
  ThreePhase im{0.0, 0.0, 0.0};
  double thF = 0.0, wF = 0.0;  // forced frame (I-f)
  double nextSamp = kTcar / 2.0;
  double thUse = 0.0, wUse = 0.0, thVolt = 0.0;
  double wPre = 0.0, wMin = 1e18, idMean = 0.0, iqMean = 0.0, iqRefMean = 0.0;
  double fxMean = 0.0, angErrMean = 0.0, nMean = 0.0;
  bool stepped = false;
  while (eng.status() == SimulationStatus::Running) {
    const double t = eng.time();
    if (!stepped && t >= kStepT) {
      tload = kTload;
      stepped = true;
    }
    // Forced-frame schedule (analytic, sensorless-legal inputs only).
    wF = std::min(kWref, kRampA * t);
    thF += wF * kDt;
    const double beta =
        std::min(std::max((t - kBlend0) / (kBlend1 - kBlend0), 0.0), 1.0);
    const ThreePhase e = pmsmEmf(rotor.theta, rotor.omega, fp.motor);
    eng.circuit().findDevice("EA").value = e.a;
    eng.circuit().findDevice("EB").value = e.b;
    eng.circuit().findDevice("EC").value = e.c;
    eng.step();
    if (t + kDt >= nextSamp) {  // midpoint sample, held between ticks
      im = {eng.deviceCurrent("LA"), eng.deviceCurrent("LB"), eng.deviceCurrent("LC")};
      const ThreePhase vv = inversePark(foc.vd(), foc.vq(), thVolt);
      const auto va = power_engine::control::clarke(vv.a, vv.b, vv.c);
      const auto ia = power_engine::control::clarke(im.a, im.b, im.c);
      obs.update(va.alpha, va.beta, ia.alpha, ia.beta, kTcar);
      nextSamp += kTcar;
    }
    const double wref = beta < 1.0 ? wF : kWref;
    const double alpha = (beta < 1.0 && wF < kWref) ? kRampA : 0.0;
    thUse = (1.0 - beta) * kP * thF + beta * obs.thetaE();
    wUse = (1.0 - beta) * wF + beta * obs.omegaE() / kP;
    const auto g = foc.update(t, wref, alpha, wUse, im, thUse, kDt);
    thVolt = thUse;
    eng.setSwitch("SAh", g.aHi);
    eng.setSwitch("SAl", !g.aHi);
    eng.setSwitch("SBh", g.bHi);
    eng.setSwitch("SBl", !g.bHi);
    eng.setSwitch("SCh", g.cHi);
    eng.setSwitch("SCl", !g.cHi);
    stepMechanical(rotor, pmsmTorque(foc.id(), foc.iq(), fp.motor), tload, mech, kDt);
    if (t >= 300e-3 && t < kStepT) wPre = rotor.omega;
    if (t >= kStepT) wMin = std::min(wMin, rotor.omega);
    if (t >= 460e-3) {
      idMean += foc.id();
      iqMean += foc.iq();
      iqRefMean += foc.iqRef();
      fxMean += obs.fluxMag();
      double ae = std::fmod(obs.thetaE() - kP * rotor.theta + kPi, 2.0 * kPi);
      if (ae < 0.0) ae += 2.0 * kPi;
      angErrMean += std::abs(ae - kPi);
      nMean += 1.0;
    }
  }
  ASSERT_TRUE(stepped);
  EXPECT_NEAR(wPre, kWref, 0.05 * kWref);
  EXPECT_LT(wMin, kWref - 2.0);
  EXPECT_NEAR(rotor.omega, kWref, 0.05 * kWref);
  EXPECT_NEAR(idMean / nMean, 0.0, 0.15);
  EXPECT_NEAR(iqMean / nMean, iqRefMean / nMean, 0.10 * iqRefMean / nMean);
  EXPECT_NEAR(iqRefMean / nMean, (kB * kWref + kTload) / kTq,
              0.15 * (kB * kWref + kTload) / kTq);
  // Observer locked: flux at λm, angle on truth.
  EXPECT_NEAR(fxMean / nMean, kLam, 0.10 * kLam);
  EXPECT_LT(angErrMean / nMean, 0.09);  // < ~5 deg electrical
}
