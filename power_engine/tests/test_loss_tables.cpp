#include <cmath>
#include <complex>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <gtest/gtest.h>

#include "power_engine/engine.h"
#include "power_engine/loss_tables.h"

using power_engine::Engine;
using power_engine::loss::DeviceLossModel;
using power_engine::loss::Table;

namespace {
constexpr double kPi = std::numbers::pi;
}  // namespace

// --- Table core: nodes exact, midpoints linear, edges clamp, bad input throws.
TEST(LossTable, Interp1D) {
  const Table t({"TJ"}, {{25.0, 125.0}}, {5e-3, 9e-3}, "R");
  EXPECT_DOUBLE_EQ(t.at({{"TJ", 25.0}}), 5e-3);
  EXPECT_DOUBLE_EQ(t.at({{"TJ", 125.0}}), 9e-3);
  EXPECT_DOUBLE_EQ(t.at({{"TJ", 75.0}}), 7e-3);
  EXPECT_DOUBLE_EQ(t.at({{"TJ", -40.0}}), 5e-3);   // clamp low
  EXPECT_DOUBLE_EQ(t.at({{"TJ", 200.0}}), 9e-3);   // clamp high
  EXPECT_THROW(t.at({{"I", 1.0}}), std::runtime_error);  // missing axis
}

TEST(LossTable, Interp3DExactOnBilinear) {
  // E = 1e-6*I*V is bilinear: multilinear interp reproduces it exactly.
  const Table t({"I", "V", "TJ"}, {{0.0, 2.0}, {0.0, 24.0}, {25.0}},
                {0.0, 0.0, 0.0, 48e-6}, "E");
  EXPECT_DOUBLE_EQ(t.at({{"I", 2.0}, {"V", 24.0}, {"TJ", 25.0}}), 48e-6);
  EXPECT_NEAR(t.at({{"I", 0.825}, {"V", 12.0}, {"TJ", 25.0}}), 9.9e-6, 1e-12);
  EXPECT_NEAR(t.at({{"I", 1.575}, {"V", 12.0}, {"TJ", 99.0}}), 18.9e-6, 1e-12);
}

TEST(LossTable, RejectsBadDefinitions) {
  EXPECT_THROW(Table({}, {}, {}), std::runtime_error);  // no axes
  EXPECT_THROW(Table({"I"}, {{2.0, 1.0}}, {1.0, 2.0}), std::runtime_error);  // decreasing
  EXPECT_THROW(Table({"I"}, {{1.0}}, {1.0, 2.0}), std::runtime_error);  // size mismatch
  EXPECT_THROW(Table({"I"}, {{1.0}}, {std::numeric_limits<double>::quiet_NaN()}),
               std::runtime_error);  // NaN value
  EXPECT_THROW(Table({"A", "B", "C", "D", "E"},
                     {{0.0}, {0.0}, {0.0}, {0.0}, {0.0}}, {1.0}),
               std::runtime_error);  // >4D
  const Table t({"I"}, {{0.0, 1.0}}, {0.0, 1.0});
  EXPECT_THROW(t.at({}), std::runtime_error);
}

// --- Constant-slice tables reproduce the scalar path exactly.
TEST(TableLoss, ConstantTablesMatchScalarAccounting) {
  constexpr double T = 50e-6;
  Engine eng;
  eng.setTimeStep(0.5e-6);
  eng.circuit().addVoltageSource("Vin", 1, 0, 12.0);
  eng.circuit().addSwitch("S1", 1, 2, 5e-3, 1e6, true);
  eng.circuit().addDiode("D1", 0, 2, 0.0, 10e-3, 1e6);
  eng.circuit().addInductor("L1", 2, 3, 200e-6, 0.0);
  eng.circuit().addCapacitor("C1", 3, 0, 200e-6, 0.0);
  eng.circuit().addResistor("Rload", 3, 0, 5.0);
  DeviceLossModel model;
  model.eon = Table({"I", "V", "TJ"}, {{0.0, 10.0}, {0.0, 100.0}, {25.0}},
                    {10e-6, 10e-6, 10e-6, 10e-6}, "E");
  model.eoff = Table({"I", "V", "TJ"}, {{0.0, 10.0}, {0.0, 100.0}, {25.0}},
                     {15e-6, 15e-6, 15e-6, 15e-6}, "E");
  eng.attachLossModel("S1", model);
  EXPECT_TRUE(eng.hasLossModel("S1"));
  EXPECT_FALSE(eng.hasLossModel("D1"));
  EXPECT_DOUBLE_EQ(eng.deviceTemp("S1"), 25.0);  // no thermal: ambient
  eng.setStopTime(6e-3);
  for (double t = 0.0; t < 6e-3; t += T) {
    eng.scheduleSwitch("S1", true, t);
    eng.scheduleSwitch("S1", false, t + 0.5 * T);
  }
  eng.start();
  while (eng.status() == power_engine::SimulationStatus::Running) eng.step();
  // Same accounting as the scalar path: 119 on + 120 off edges.
  EXPECT_NEAR(eng.deviceLoss("S1").esw, 119.0 * 10e-6 + 120.0 * 15e-6, 25e-6);
}

// --- Acceptance: E(I,V) = k*I*V tables vs hand-computed edge energies.
// CCM buck, D=0.5: valley current at turn-on, peak at turn-off.
TEST(TableLoss, SwitchingMatchesAnalyticSpotCheck) {
  constexpr double kVin = 12.0, kR = 5.0, kL = 200e-6, kFsw = 20e3, kD = 0.5;
  constexpr double kT = 50e-6;
  constexpr double kOn = 1e-6, kOff = 1.5e-6;  // E = k*I*V
  const double iL = kD * kVin / kR;                            // 1.2A
  const double dI = kVin * kD * (1.0 - kD) / (kL * kFsw);       // 0.75A
  const double iOn = iL - 0.5 * dI, iOff = iL + 0.5 * dI;

  Engine eng;
  eng.setTimeStep(0.5e-6);
  eng.circuit().addVoltageSource("Vin", 1, 0, kVin);
  eng.circuit().addSwitch("S1", 1, 2, 5e-3, 1e6, true);
  eng.circuit().addDiode("D1", 0, 2, 0.0, 10e-3, 1e6);
  eng.circuit().addInductor("L1", 2, 3, kL, 0.0);
  eng.circuit().addCapacitor("C1", 3, 0, 200e-6, 0.0);
  eng.circuit().addResistor("Rload", 3, 0, kR);
  auto bilinear = [](double k) {
    return Table({"I", "V", "TJ"}, {{0.0, 2.0}, {0.0, 24.0}, {25.0}},
                 {0.0, 0.0, 0.0, k * 48.0}, "E");
  };
  DeviceLossModel model;
  model.eon = bilinear(kOn);
  model.eoff = bilinear(kOff);
  eng.attachLossModel("S1", model);
  eng.setStopTime(6e-3);
  for (double t = 0.0; t < 6e-3; t += kT) {
    eng.scheduleSwitch("S1", true, t);
    eng.scheduleSwitch("S1", false, t + kD * kT);
  }
  eng.start();
  while (eng.status() == power_engine::SimulationStatus::Running) eng.step();
  const double expected = 119.0 * kOn * iOn * kVin + 120.0 * kOff * iOff * kVin;
  EXPECT_NEAR(eng.deviceLoss("S1").esw, expected, 0.05 * expected);
}

// --- Acceptance: Tj-dependent Ron(Tj) with thermal, vs coupled analytic.
// Pcond = Irms^2*Ron(Tj), Tj = Tamb + Rth*(Pcond+Psw): solved closed-form.
TEST(TableLoss, TjConductionMatchesCoupledAnalytic) {
  constexpr double kVin = 12.0, kR = 5.0, kL = 200e-6, kFsw = 20e3, kD = 0.5;
  constexpr double kT = 50e-6, kRth = 200.0, kTamb = 25.0;
  constexpr double kRon25 = 5e-3, kTc = 0.008;  // Ron = Ron25*(1+tc*(Tj-25))
  const double iL = kD * kVin / kR;
  const double dI = kVin * kD * (1.0 - kD) / (kL * kFsw);
  const double iRms2 = iL * iL + dI * dI / 12.0;
  constexpr double kPsw = 25e-6 * kFsw;  // 12.5uJ edges x2 per cycle
  // Closed form: x = Tj-Tamb, x = Rth*(D*Irms^2*Ron25*(1+tc*x) + Psw).
  // (D: the switch carries current only during the on-interval.)
  const double a = kD * iRms2 * kRon25;
  const double x = kRth * (a + kPsw) / (1.0 - kRth * a * kTc);
  const double tjRef = kTamb + x;
  const double pCondRef = a * (1.0 + kTc * x);

  Engine eng;
  eng.setTimeStep(0.5e-6);
  eng.circuit().addVoltageSource("Vin", 1, 0, kVin);
  eng.circuit().addSwitch("S1", 1, 2, kRon25, 1e6, true, 12.5e-6, 12.5e-6);
  eng.circuit().addDiode("D1", 0, 2, 0.0, 10e-3, 1e6);
  eng.circuit().addInductor("L1", 2, 3, kL, 0.0);
  eng.circuit().addCapacitor("C1", 3, 0, 200e-6, 0.0);
  eng.circuit().addResistor("Rload", 3, 0, kR);
  DeviceLossModel model;
  model.ronTj = Table({"TJ"}, {{25.0, 125.0}}, {kRon25, kRon25 * (1.0 + kTc * 100.0)}, "R");
  eng.attachLossModel("S1", model);
  eng.attachThermal("S1", power_engine::thermal::ThermalNetwork::foster({{kRth, 50e-6}}, kTamb));
  constexpr double kStop = 200e-3;
  eng.setStopTime(kStop);
  for (double t = 0.0; t < kStop; t += kT) {
    eng.scheduleSwitch("S1", true, t);
    eng.scheduleSwitch("S1", false, t + kD * kT);
  }
  eng.start();
  while (eng.status() == power_engine::SimulationStatus::Running) eng.step();
  // Transient deficit (Pss-P0)*tau' with tau' = tau/(1-Rth*dP/dTj).
  const double p0 = a;
  const double tauEff = kRth * 50e-6 / (1.0 - kRth * a * kTc);
  const double expected = pCondRef * kStop - (pCondRef - p0) * tauEff;
  EXPECT_NEAR(eng.deviceLoss("S1").econd, expected, 0.05 * expected);
  EXPECT_NEAR(eng.junctionTemp("S1"), tjRef, 0.03 * (tjRef - kTamb));
  // Stamped Ron tracks the table at the settled Tj.
  const double ronExpect = kRon25 * (1.0 + kTc * (eng.junctionTemp("S1") - kTamb));
  EXPECT_NEAR(eng.circuit().findDevice("S1").ron, ronExpect, 0.02 * ronExpect);
}

// --- Netlist path: .etable + .model refs + applyLossModels + round-trip.
TEST(NetlistLossTables, ParseAttachRunRoundTrip) {
  const char* text = R"(
.etable EON1 I=0,2 V=0,24 TJ=25 E=0,0,0,48u
.etable EOFF1 I=0,2 V=0,24 TJ=25 E=0,0,0,72u
.model SW mosfet_ideal RON=5m ROFF=1Meg EON_TABLE=EON1 EOFF_TABLE=EOFF1
V1 1 0 12
S1 1 2 MODEL=SW
D1 0 2 VF=0 RON=10m
L1 2 3 200u
C1 3 0 200u
Rload 3 0 5
.control pwm switch=S1 freq=20k duty=0.5
.tran 0.5u 6m
.end
)";
  Engine eng;
  eng.loadNetlist(text);
  EXPECT_EQ(eng.circuit().findDevice("S1").eonTable, "EON1");
  EXPECT_EQ(eng.circuit().findDevice("S1").eoffTable, "EOFF1");
  eng.applyPwmSpecs();
  eng.applyLossModels();
  EXPECT_TRUE(eng.hasLossModel("S1"));
  eng.start();
  while (eng.status() == power_engine::SimulationStatus::Running) eng.step();
  // Same bilinear tables as the analytic test (k=1e-6/1.5e-6): 4.58mJ.
  const double expected = 119.0 * 1e-6 * 0.825 * 12.0 + 120.0 * 1.5e-6 * 1.575 * 12.0;
  EXPECT_NEAR(eng.deviceLoss("S1").esw, expected, 0.05 * expected);

  // Round-trip: tables + device refs survive serialize -> parse.
  power_engine::netlist::Parser p;
  const auto a = p.parse(text);
  const auto b = p.parse(a.serialize());
  ASSERT_EQ(a.tables.size(), 2u);
  ASSERT_EQ(b.tables.size(), 2u);
  EXPECT_EQ(a.serialize(), b.serialize());
  EXPECT_EQ(b.circuit.findDevice("S1").eonTable, "EON1");
}

TEST(NetlistLossTables, RejectsBadTables) {
  power_engine::netlist::Parser p;
  EXPECT_THROW(p.parse(".etable B I=1 V=1 E=1,2\nR1 1 0 1\n.end\n"), std::runtime_error);
  EXPECT_THROW(p.parse(".etable B I=1 V=1\nR1 1 0 1\n.end\n"), std::runtime_error);
  // Diodes reject EON/EOFF tables at parse time (they book QRR instead).
  EXPECT_THROW(p.parse(".etable E I=1 E=1\nD1 1 0 EON_TABLE=E\nV1 1 0 1\n.end\n"),
               std::runtime_error);
  // Dangling table refs resolve at applyLossModels (not parse) and throw.
  Engine eng;
  eng.loadNetlist(".model SW mosfet_ideal EON_TABLE=NOPE\nS1 1 0 MODEL=SW\nV1 1 0 1\n.end\n");
  EXPECT_THROW(eng.applyLossModels(), std::runtime_error);
}

// --- Attachment validation.
TEST(LossModelAttach, Validation) {
  Engine eng;
  eng.circuit().addResistor("R1", 1, 0, 5.0);
  eng.circuit().addSwitch("S1", 1, 0, 5e-3, 1e6, false);
  eng.circuit().addDiode("D1", 1, 0, 0.0, 10e-3, 1e6);
  DeviceLossModel m;
  EXPECT_THROW(eng.attachLossModel("R1", m), std::runtime_error);
  EXPECT_THROW(eng.attachLossModel("NOPE", m), std::runtime_error);
  m.eon = Table({"I"}, {{0.0, 1.0}}, {1e-6, 2e-6}, "E");
  EXPECT_THROW(eng.attachLossModel("D1", m), std::runtime_error);  // diode + Eon
  m.eon = Table{};
  m.ronTj = Table({"TJ"}, {{25.0}}, {5e-3}, "R");
  eng.attachLossModel("D1", m);  // diode + Ron(Tj) is fine
  EXPECT_TRUE(eng.hasLossModel("D1"));
  (void)kPi;
}
