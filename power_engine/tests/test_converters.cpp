#include <cmath>
#include <gtest/gtest.h>

#include <numbers>
#include <stdexcept>

#include "power_engine/ac.h"
#include "power_engine/control.h"
#include "power_engine/engine.h"

using power_engine::Engine;

namespace {

constexpr double kPi = std::numbers::pi;

// Schedules one trailing-edge period [t0, t0+T) with duty D on `hi`
// and the deadtime-shifted complement on `lo`:
//   hi: [t0+td, t0+D*T), lo: [t0+D*T+td, t0+T).
void scheduleSyncPair(Engine& eng, const std::string& hi, const std::string& lo,
                      double t0, double T, double duty, double td) {
  eng.scheduleSwitch(hi, true, t0 + td);
  eng.scheduleSwitch(hi, false, t0 + duty * T);
  eng.scheduleSwitch(lo, true, t0 + duty * T + td);
  eng.scheduleSwitch(lo, false, t0 + T);
}

double avgVout(Engine& eng, double dt, double tStop, double window, int voutNode,
               double* minOut = nullptr, double* maxOut = nullptr) {
  double sum = 0.0, n = 0.0, mn = 1e18, mx = -1e18;
  eng.start();
  while (eng.status() == power_engine::SimulationStatus::Running) {
    eng.step();
    const double t = eng.time();
    const double v = eng.currentSolution().probes.at("v:" + std::to_string(voutNode));
    if (!std::isfinite(v)) throw std::runtime_error("non-finite vout");
    if (t > tStop - window) {
      sum += v;
      n += 1.0;
      if (v < mn) mn = v;
      if (v > mx) mx = v;
    }
  }
  (void)dt;
  if (minOut) *minOut = mn;
  if (maxOut) *maxOut = mx;
  return sum / n;
}

}  // namespace

// Boost CCM: Vin=5, D=0.5 -> 10V. L=200uH: dI=0.625A, ILavg=2A (R=10).
TEST(Converters, BoostMatchesTheory) {
  Engine eng;
  eng.setTimeStep(0.5e-6);
  eng.circuit().addVoltageSource("Vin", 1, 0, 5.0);
  eng.circuit().addInductor("L1", 1, 2, 200e-6, 0.0);
  eng.circuit().addSwitch("S1", 2, 0, 5e-3, 1e6, true);
  eng.circuit().addDiode("D1", 2, 3, 0.0, 10e-3, 1e6);
  eng.circuit().addCapacitor("C1", 3, 0, 200e-6, 0.0);
  eng.circuit().addResistor("Rload", 3, 0, 10.0);
  constexpr double T = 50e-6, D = 0.5, tStop = 10e-3;
  for (double t = 0.0; t < tStop; t += T) {
    eng.scheduleSwitch("S1", true, t);
    eng.scheduleSwitch("S1", false, t + D * T);
  }
  eng.setStopTime(tStop);
  EXPECT_NEAR(avgVout(eng, 0.5e-6, tStop, 1e-3, 3), 10.0, 0.02 * 10.0);
  EXPECT_GT(eng.solverStats().diodeEvents, 0);
}

// Synchronous buck: complementary pair + 200ns deadtime + body diodes.
// Deadtime conduction goes through body diodes (Vf=0.7), clamping Vsw.
TEST(Converters, SyncBuckDeadtimeClampsSwitchNode) {
  Engine eng;
  eng.setTimeStep(0.5e-6);
  eng.circuit().addVoltageSource("Vin", 1, 0, 12.0);
  eng.circuit().addSwitch("Shi", 1, 2, 5e-3, 1e6, true);
  eng.circuit().addSwitch("Slo", 2, 0, 5e-3, 1e6, false);
  eng.circuit().addDiode("Dhi", 2, 1, 0.7, 10e-3, 1e6);  // antiparallel body
  eng.circuit().addDiode("Dlo", 0, 2, 0.7, 10e-3, 1e6);  // antiparallel body
  eng.circuit().addInductor("L1", 2, 3, 200e-6, 0.0);
  eng.circuit().addCapacitor("C1", 3, 0, 200e-6, 0.0);
  eng.circuit().addResistor("Rload", 3, 0, 5.0);
  constexpr double T = 50e-6, D = 0.5, td = 200e-9, tStop = 6e-3;
  for (double t = 0.0; t < tStop; t += T) scheduleSyncPair(eng, "Shi", "Slo", t, T, D, td);
  eng.setStopTime(tStop);
  eng.start();
  double sum = 0.0, n = 0.0, vswMax = -1e18, vswMin = 1e18;
  bool overlap = false;
  while (eng.status() == power_engine::SimulationStatus::Running) {
    eng.step();
    const double t = eng.time();
    const double vout = eng.currentSolution().probes.at("v:3");
    const double vsw = eng.currentSolution().probes.at("v:2");
    ASSERT_TRUE(std::isfinite(vout)) << "t=" << t;
    ASSERT_TRUE(std::isfinite(vsw)) << "t=" << t;
    if (vsw > vswMax) vswMax = vsw;
    if (vsw < vswMin) vswMin = vsw;
    // Shoot-through check: both switches simultaneously closed never happens
    // by construction; verify branch currents never spike together.
    const double iHi = eng.deviceCurrent("Shi");
    const double iLo = eng.deviceCurrent("Slo");
    if (std::abs(iHi) > 0.05 && std::abs(iLo) > 0.05) overlap = true;
    if (t > tStop - T) {
      sum += vout;
      n += 1.0;
    }
  }
  EXPECT_NEAR(sum / n, 6.0, 0.02 * 6.0);
  // Body diodes clamp the switch node: without them L would force ~800V here.
  EXPECT_LT(vswMax, 20.0);
  EXPECT_GT(vswMin, -20.0);
  EXPECT_FALSE(overlap);
}

// Buck DCM (no special-casing: diode just stays off at zero current).
// M = 2/(1+sqrt(1+4K/D^2)), K = 2L/(R*T). Here K=0.04 -> M=0.877 -> 10.53V.
TEST(Converters, BuckDCMMatchesTheory) {
  Engine eng;
  eng.setTimeStep(0.5e-6);
  eng.circuit().addVoltageSource("Vin", 1, 0, 12.0);
  eng.circuit().addSwitch("S1", 1, 2, 5e-3, 1e6, true);
  eng.circuit().addDiode("D1", 0, 2, 0.0, 10e-3, 1e6);
  eng.circuit().addInductor("L1", 2, 3, 20e-6, 0.0);
  eng.circuit().addCapacitor("C1", 3, 0, 100e-6, 0.0);
  eng.circuit().addResistor("Rload", 3, 0, 20.0);
  constexpr double T = 50e-6, D = 0.5, tStop = 8e-3;
  for (double t = 0.0; t < tStop; t += T) {
    eng.scheduleSwitch("S1", true, t);
    eng.scheduleSwitch("S1", false, t + D * T);
  }
  eng.setStopTime(tStop);
  eng.start();
  double sum = 0.0, n = 0.0, ilMin = 1e18;
  while (eng.status() == power_engine::SimulationStatus::Running) {
    eng.step();
    const double t = eng.time();
    if (t > tStop - 1e-3) {
      sum += eng.currentSolution().probes.at("v:3");
      n += 1.0;
      const double il = eng.deviceCurrent("L1");
      if (il < ilMin) ilMin = il;
    }
  }
  EXPECT_NEAR(sum / n, 10.53, 0.05 * 10.53);
  // DCM signature: inductor current returns to ~zero every period.
  EXPECT_LT(ilMin, 0.05);
}

// Flyback CCM, n=1, D=0.4: Vout = Vin*D/(1-D) = 8V.
// Dot convention: secondary reversed (n3=0, n4=3) so the diode blocks
// while the switch is on and delivers while it is off.
TEST(Converters, FlybackMatchesTheory) {
  Engine eng;
  eng.setTimeStep(0.5e-6);
  eng.circuit().addVoltageSource("Vin", 1, 0, 12.0);
  eng.circuit().addInductor("Lm", 1, 2, 500e-6, 0.0);  // magnetizing
  eng.circuit().addSwitch("S1", 2, 0, 5e-3, 1e6, true);
  eng.circuit().addTransformer("T1", 1, 2, 0, 3, 1.0);
  eng.circuit().addDiode("Dsec", 3, 4, 0.0, 10e-3, 1e6);
  eng.circuit().addCapacitor("Cout", 4, 0, 200e-6, 0.0);
  eng.circuit().addResistor("Rload", 4, 0, 10.0);
  constexpr double T = 50e-6, D = 0.4, tStop = 12e-3;
  for (double t = 0.0; t < tStop; t += T) {
    eng.scheduleSwitch("S1", true, t);
    eng.scheduleSwitch("S1", false, t + D * T);
  }
  eng.setStopTime(tStop);
  EXPECT_NEAR(avgVout(eng, 0.5e-6, tStop, 1e-3, 4), 8.0, 0.03 * 8.0);
}

// Full-bridge with unipolar SPWM: Vdc=48, m=0.8, 50Hz into LC (2mH/10uF)
// + 20ohm. Differential fundamental = m*Vdc = 38.4V peak.
TEST(Converters, FullBridgeSpwmMatchesTheory) {
  constexpr double kVdc = 48.0, kM = 0.8, kF0 = 50.0;
  constexpr double kT = 100e-6;  // 10kHz switching
  Engine eng;
  eng.setTimeStep(1e-6);
  eng.circuit().addVoltageSource("Vdc", 5, 0, kVdc);
  eng.circuit().addSwitch("S1", 5, 1, 5e-3, 1e6, false);
  eng.circuit().addSwitch("S2", 1, 0, 5e-3, 1e6, true);
  eng.circuit().addSwitch("S3", 5, 2, 5e-3, 1e6, false);
  eng.circuit().addSwitch("S4", 2, 0, 5e-3, 1e6, true);
  eng.circuit().addDiode("D1", 1, 5, 0.0, 10e-3, 1e6);
  eng.circuit().addDiode("D2", 0, 1, 0.0, 10e-3, 1e6);
  eng.circuit().addDiode("D3", 2, 5, 0.0, 10e-3, 1e6);
  eng.circuit().addDiode("D4", 0, 2, 0.0, 10e-3, 1e6);
  eng.circuit().addInductor("L1", 1, 3, 2e-3, 0.0);
  eng.circuit().addCapacitor("C1", 3, 2, 10e-6, 0.0);
  eng.circuit().addResistor("Rload", 3, 2, 20.0);
  constexpr double kSettle = 10e-3, kMeasure = 40e-3, kEnd = kSettle + kMeasure;
  for (double tk = 0.0; tk < kEnd; tk += kT) {
    const double w = 2.0 * kPi * kF0;
    const double da = 0.5 + 0.5 * kM * std::sin(w * tk);
    const double db = 0.5 - 0.5 * kM * std::sin(w * tk);
    eng.scheduleSwitch("S1", true, tk);
    eng.scheduleSwitch("S1", false, tk + da * kT);
    eng.scheduleSwitch("S2", false, tk);
    eng.scheduleSwitch("S2", true, tk + da * kT);
    eng.scheduleSwitch("S3", true, tk);
    eng.scheduleSwitch("S3", false, tk + db * kT);
    eng.scheduleSwitch("S4", false, tk);
    eng.scheduleSwitch("S4", true, tk + db * kT);
  }
  eng.setStopTime(kEnd + kT);
  eng.start();
  while (eng.time() < kSettle) eng.step();
  power_engine::ac::FourierMeter m;
  m.begin(kF0);
  double tPrev = eng.time(), yPrev = 0.0;
  {
    const auto& pr = eng.currentSolution().probes;
    yPrev = pr.at("v:3") - pr.at("v:2");
  }
  while (eng.time() < kEnd) {
    eng.step();
    const double t = eng.time();
    const double tm = 0.5 * (tPrev + t);
    const double u = kM * kVdc * std::sin(2.0 * kPi * kF0 * tm);
    const auto& pr = eng.currentSolution().probes;
    const double y = pr.at("v:3") - pr.at("v:2");
    m.sample(tPrev, t, u, 0.5 * (yPrev + y));
    tPrev = t;
    yPrev = y;
  }
  eng.stop();
  const auto p = m.result();
  EXPECT_NEAR(p.mag, 1.0, 0.03);
  auto wrap180 = [](double deg) {
    while (deg > 180.0) deg -= 360.0;
    while (deg <= -180.0) deg += 360.0;
    return deg;
  };
  EXPECT_LT(std::abs(wrap180(p.phaseDeg)), 3.0);
}

// 3-phase 2-level inverter, SVPWM, 3-wire star RL load (10ohm + 5mH):
// phase current = Vph/|Z| at -atan(wL/R); 3-phase power closes.
TEST(Converters, ThreePhaseInverterRLMatchesTheory) {
  constexpr double kVdc = 60.0, kM = 0.9, kF0 = 100.0;
  constexpr double kT = 100e-6, kR = 10.0, kL = 5e-3;  // 10kHz SVPWM
  const power_engine::control::Svpwm sv(10e3);
  Engine eng;
  eng.setTimeStep(1e-6);
  eng.circuit().addVoltageSource("Vdc", 4, 0, kVdc);
  eng.circuit().addSwitch("SAh", 4, 1, 5e-3, 1e6, false);
  eng.circuit().addSwitch("SAl", 1, 0, 5e-3, 1e6, true);
  eng.circuit().addSwitch("SBh", 4, 2, 5e-3, 1e6, false);
  eng.circuit().addSwitch("SBl", 2, 0, 5e-3, 1e6, true);
  eng.circuit().addSwitch("SCh", 4, 3, 5e-3, 1e6, false);
  eng.circuit().addSwitch("SCl", 3, 0, 5e-3, 1e6, true);
  eng.circuit().addInductor("LA", 1, 6, kL, 0.0);
  eng.circuit().addResistor("RA", 6, 5, kR);
  eng.circuit().addInductor("LB", 2, 7, kL, 0.0);
  eng.circuit().addResistor("RB", 7, 5, kR);
  eng.circuit().addInductor("LC", 3, 8, kL, 0.0);
  eng.circuit().addResistor("RC", 8, 5, kR);
  constexpr double kSettle = 10e-3, kMeasure = 20e-3, kEnd = kSettle + kMeasure;
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
  // Admittance metering: u = phase-A voltage reference, y = LA current.
  power_engine::ac::FourierMeter m;
  m.begin(kF0);
  double tPrev = eng.time(), yPrev = eng.deviceCurrent("LA");
  double pSum = 0.0, nSum = 0.0;
  while (eng.time() < kEnd) {
    eng.step();
    const double t = eng.time();
    const double tm = 0.5 * (tPrev + t);
    const double u = kM * (kVdc / 2.0) * std::cos(2.0 * kPi * kF0 * tm);
    const double y = eng.deviceCurrent("LA");
    m.sample(tPrev, t, u, 0.5 * (yPrev + y));
    const auto& pr = eng.currentSolution().probes;
    const double vn = pr.at("v:5");
    pSum += (pr.at("v:1") - vn) * eng.deviceCurrent("LA") +
            (pr.at("v:2") - vn) * eng.deviceCurrent("LB") +
            (pr.at("v:3") - vn) * eng.deviceCurrent("LC");
    nSum += 1.0;
    tPrev = t;
    yPrev = y;
  }
  eng.stop();
  const double w = 2.0 * kPi * kF0;
  const double zMag = std::hypot(kR, w * kL);
  const double zAng = std::atan2(w * kL, kR) * 180.0 / kPi;
  const auto p = m.result();
  EXPECT_NEAR(p.mag, 1.0 / zMag, 0.05 / zMag);
  auto wrap180 = [](double deg) {
    while (deg > 180.0) deg -= 360.0;
    while (deg <= -180.0) deg += 360.0;
    return deg;
  };
  EXPECT_NEAR(std::abs(wrap180(p.phaseDeg + zAng)), 0.0, 5.0);
  // 3-phase power: 3*Vph,rms*Iph,rms*cos(phi).
  const double vPh = kM * kVdc / 2.0 / std::sqrt(2.0);
  const double pRef = 3.0 * vPh * (vPh / zMag) * std::cos(zAng * kPi / 180.0);
  EXPECT_NEAR(pSum / nSum, pRef, 0.05 * pRef);
}

// Vienna rectifier, uncontrolled (switches open): 3x230Vrms/50Hz through
// 5mH + 0.5ohm grid impedance into a diode bridge with split 2mF caps +
// 100ohm: Vdc = 1.35*Vll = 540V.
// NOTE: the mains neutral MUST float (node 10, true 3-wire). Grounded
// neutrals + grounded dc- create a zero-impedance ground return through
// which lower diodes short their phases (verified failure mode).
TEST(Converters, ViennaUncontrolledMatchesDiodeBridge) {
  constexpr double kVph = 230.0, kF0 = 50.0;
  Engine eng;
  eng.setTimeStep(1e-6);
  const double w = 2.0 * kPi * kF0;
  const double vpk = kVph * std::sqrt(2.0);
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
  constexpr double kRamp = 10e-3, kStop = 60e-3;
  eng.setStopTime(kStop);
  eng.start();
  double sum = 0.0, n = 0.0;
  while (eng.status() == power_engine::SimulationStatus::Running) {
    const double t = eng.time();
    const double e = t >= kRamp ? 1.0 : 0.5 * (1.0 - std::cos(kPi * t / kRamp));
    eng.circuit().findDevice("VA").value = e * vpk * std::sin(w * t);
    eng.circuit().findDevice("VB").value = e * vpk * std::sin(w * t - 2.0 * kPi / 3.0);
    eng.circuit().findDevice("VC").value = e * vpk * std::sin(w * t + 2.0 * kPi / 3.0);
    eng.step();
    if (t > 40e-3) {
      sum += eng.currentSolution().probes.at("v:7");
      n += 1.0;
    }
  }
  // 1.35 * 400Vll,rms = 540V ideal; overlap (3wL*Id/pi ~ 8V) + diode
  // drops + grid-R + ripple pull the loaded mean a few percent under.
  EXPECT_NEAR(sum / n, 540.0, 0.04 * 540.0);
  EXPECT_GT(eng.solverStats().diodeEvents, 0);
}
