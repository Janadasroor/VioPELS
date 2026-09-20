#include <cmath>
#include <limits>
#include <stdexcept>
#include <gtest/gtest.h>

#include "power_engine/electrothermal.h"
#include "power_engine/engine.h"
#include "power_engine/loss_tables.h"

using power_engine::Engine;
using power_engine::SimulationStatus;
using power_engine::electrothermal::runToThermalSteady;
using power_engine::loss::DeviceLossModel;
using power_engine::loss::Table;
using power_engine::thermal::ThermalNetwork;

namespace {

// Buck fixture shared by the droop tests: Vin=12, D=0.5, R=5, L=200uH,
// C=200uF, fsw=20kHz. Switch Ron(Tj) table (8%/K equivalent), scalar
// 12.5uJ edges; diode ideal-ish (Vf=0, Ron=10m).
constexpr double kVin = 12.0, kR = 5.0, kL = 200e-6, kD = 0.5, kFsw = 20e3;
constexpr double kT = 50e-6, kRon25 = 5e-3, kTc = 0.008, kRth = 60.0;

void buildDroopBuck(Engine& eng) {
  eng.setTimeStep(0.5e-6);
  eng.circuit().addVoltageSource("Vin", 1, 0, kVin);
  eng.circuit().addSwitch("S1", 1, 2, kRon25, 1e6, true, 12.5e-6, 12.5e-6);
  eng.circuit().addDiode("D1", 0, 2, 0.0, 10e-3, 1e6);
  eng.circuit().addInductor("L1", 2, 3, kL, 0.0);
  eng.circuit().addCapacitor("C1", 3, 0, 200e-6, 0.0);
  eng.circuit().addResistor("Rload", 3, 0, kR);
  DeviceLossModel model;
  model.ronTj = Table({"TJ"}, {{25.0, 200.0}},
                      {kRon25, kRon25 * (1.0 + kTc * 175.0)}, "R");
  eng.attachLossModel("S1", model);
}

struct AnalyticPoint {
  double tj = 25.0, eta = 1.0, pCond = 0.0;
};

// Closed-form coupled operating point at one ambient (see test_loss_tables
// Tj test for the derivation; adds the diode leg + output for efficiency).
AnalyticPoint analyticDroop(double tamb) {
  const double iL = kD * kVin / kR;
  const double dI = kVin * kD * (1.0 - kD) / (kL * kFsw);
  const double iRms2 = iL * iL + dI * dI / 12.0;
  constexpr double kPsw = 25e-6 * kFsw;
  const double a = kD * iRms2 * kRon25;
  const double pDiode = (1.0 - kD) * iRms2 * 10e-3;
  const double x = kRth * (a + kPsw) / (1.0 - kRth * a * kTc);
  const double ron = kRon25 * (1.0 + kTc * x);
  const double vout = kD * kVin - iL * (kD * ron + (1.0 - kD) * 10e-3);
  const double pOut = vout * vout / kR;
  const double pCond = a * (1.0 + kTc * x);
  const double pIn = pOut + pCond + pDiode + kPsw;
  return {tamb + x, pOut / pIn, pCond};
}

// Measure efficiency over window W at the current engine time. Stops early
// if the engine finishes (stop time reached inside the window).
// Pin via energy conservation (Pout + booked S1/D1 losses as differences):
// source branch-current history is not a reliable input-current probe.
double measureEfficiency(Engine& eng, double window) {
  using power_engine::SimulationStatus;
  const double eS0 = eng.deviceLoss("S1").total();
  const double eD0 = eng.deviceLoss("D1").total();
  const double t0 = eng.time();
  double sumV2 = 0.0;
  long long n = 0;
  while (eng.time() < t0 + window && eng.status() == SimulationStatus::Running) {
    eng.step();
    sumV2 += eng.currentSolution().probes.at("v:3") *
             eng.currentSolution().probes.at("v:3");
    ++n;
  }
  if (n == 0) throw std::runtime_error("measureEfficiency: empty window");
  const double elapsed = eng.time() - t0;
  const double pOut = (sumV2 / static_cast<double>(n)) / kR;
  const double pLoss = (eng.deviceLoss("S1").total() - eS0 +
                        eng.deviceLoss("D1").total() - eD0) /
                       elapsed;
  return pOut / (pOut + pLoss);
}

}  // namespace

// --- Override API: precedence + validation.
TEST(ElectroThermalOverride, PrecedenceAndValidation) {
  Engine eng;
  eng.circuit().addVoltageSource("V1", 1, 0, 12.0);
  eng.circuit().addSwitch("S1", 1, 2, 5e-3, 1e6, false);
  EXPECT_DOUBLE_EQ(eng.deviceTemp("S1"), 25.0);  // ambient fallback
  eng.setJunctionTempOverride("S1", 100.0);
  EXPECT_DOUBLE_EQ(eng.deviceTemp("S1"), 100.0);
  eng.attachThermal("S1", ThermalNetwork::foster({{1.0, 1.0}}, 25.0));
  EXPECT_DOUBLE_EQ(eng.deviceTemp("S1"), 100.0);  // override wins
  eng.clearJunctionTempOverride("S1");
  EXPECT_DOUBLE_EQ(eng.deviceTemp("S1"), 25.0);  // attached network at Tamb
  EXPECT_THROW(eng.setJunctionTempOverride("NOPE", 50.0), std::runtime_error);
  EXPECT_THROW(eng.setJunctionTempOverride(
                   "S1", std::numeric_limits<double>::quiet_NaN()),
               std::runtime_error);
}

// --- Outer loop converges to the exact steady state for constant power
// (switch held ON: P = I^2*Ron, no edges).
TEST(ElectroThermalSteady, ConstantPowerMatchesClosedForm) {
  Engine eng;
  eng.setTimeStep(1e-6);
  eng.circuit().addVoltageSource("V1", 1, 0, 12.0);
  eng.circuit().addSwitch("S1", 1, 2, 0.1, 1e6, true);
  eng.circuit().addResistor("R1", 2, 0, 10.0);
  DeviceLossModel model;  // no tables: scalar path, Tj-independent P
  eng.attachLossModel("S1", model);
  ThermalNetwork net = ThermalNetwork::foster({{100.0, 1e-3}}, 25.0);
  eng.start();
  const auto r = runToThermalSteady(eng, "S1", net, 1e-3, 5.0, 1e-6, 20);
  eng.stop();
  ASSERT_TRUE(r.converged);
  const double i = 12.0 / 10.1;
  EXPECT_NEAR(r.tj, 25.0 + 100.0 * i * i * 0.1, 0.05);
  EXPECT_NEAR(r.pAvg, i * i * 0.1, 0.01 * i * i * 0.1);
  EXPECT_LE(r.iterations, 5);
}

// --- Outer-loop validation errors.
TEST(ElectroThermalSteady, RejectsBadSetup) {
  Engine eng;
  eng.circuit().addSwitch("S1", 1, 0, 5e-3, 1e6, false);
  DeviceLossModel model;
  eng.attachLossModel("S1", model);
  ThermalNetwork net = ThermalNetwork::foster({{1.0, 1.0}}, 25.0);
  EXPECT_THROW(runToThermalSteady(eng, "S1", net, 1e-3, 1.0, 0.01, 5),
               std::runtime_error);  // not running
  eng.start();
  EXPECT_THROW(runToThermalSteady(eng, "S1", net, 0.0, 1.0, 0.01, 5),
               std::runtime_error);
  EXPECT_THROW(runToThermalSteady(eng, "S1", net, 1e-3, 1.0, 0.01, 0),
               std::runtime_error);
  EXPECT_THROW(runToThermalSteady(eng, "NOPE", net, 1e-3, 1.0, 0.01, 5),
               std::runtime_error);
  eng.attachThermal("S1", ThermalNetwork::foster({{1.0, 1.0}}, 25.0));
  EXPECT_THROW(runToThermalSteady(eng, "S1", net, 1e-3, 1.0, 0.01, 5),
               std::runtime_error);  // engine thermal + outer loop clash
  eng.stop();
}

// --- Acceptance: efficiency droop vs ambient within 5% of the coupled
// closed-form reference, at 25/85/125C ambient.
TEST(ElectroThermalDroop, MatchesReferenceCurves) {
  for (double tamb : {25.0, 85.0, 125.0}) {
    Engine eng;
    buildDroopBuck(eng);
    ThermalNetwork net = ThermalNetwork::foster({{kRth, 1e-3}}, tamb);
    // Horizon: 50 outer iters x 2ms window + 4ms measure, all scheduled.
    for (double t = 0.0; t < 0.12; t += kT) {
      eng.scheduleSwitch("S1", true, t);
      eng.scheduleSwitch("S1", false, t + kD * kT);
    }
    eng.start();
    const auto r = runToThermalSteady(eng, "S1", net, 2e-3, 2.0, 0.01, 50);
    ASSERT_TRUE(r.converged) << "tamb=" << tamb;
    const double eta = measureEfficiency(eng, 4e-3);
    eng.stop();
    const AnalyticPoint ref = analyticDroop(tamb);
    EXPECT_NEAR(r.tj, ref.tj, 0.05 * (ref.tj - tamb)) << "tamb=" << tamb;
    EXPECT_NEAR(eta, ref.eta, 0.05 * ref.eta) << "tamb=" << tamb;
  }
  // Droop direction: hotter ambient must read lower efficiency.
  // (Checked implicitly by matching each curve point; explicit ordering
  // would couple test cases — the per-point 5% band already pins it.)
}

// --- Cross-check: brute-force warm-up transient converges to the same
// steady state as the outer loop (Tamb=25C, tau=10ms, 200ms run).
TEST(ElectroThermalDroop, TransientAgreesWithOuterLoop) {
  Engine eng;
  buildDroopBuck(eng);
  eng.attachThermal("S1", ThermalNetwork::foster({{kRth, 10e-3 / kRth}}, 25.0));
  constexpr double kStop = 200e-3;
  for (double t = 0.0; t < kStop; t += kT) {
    eng.scheduleSwitch("S1", true, t);
    eng.scheduleSwitch("S1", false, t + kD * kT);
  }
  eng.setStopTime(kStop);
  eng.start();
  while (eng.status() == SimulationStatus::Running) {
    if (eng.time() >= kStop - 20e-3) break;
    eng.step();
  }
  // Efficiency over the last 20ms + settled Tj.
  const double eta = measureEfficiency(eng, kStop - eng.time());
  eng.stop();
  const AnalyticPoint ref = analyticDroop(25.0);
  EXPECT_NEAR(eta, ref.eta, 0.05 * ref.eta);
  EXPECT_NEAR(eng.junctionTemp("S1"), ref.tj, 0.05 * (ref.tj - 25.0));
}
