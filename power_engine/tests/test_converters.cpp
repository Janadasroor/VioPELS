#include <cmath>
#include <gtest/gtest.h>

#include <stdexcept>

#include "power_engine/engine.h"

using power_engine::Engine;

namespace {

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
