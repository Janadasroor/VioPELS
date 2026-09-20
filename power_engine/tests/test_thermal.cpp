#include <cmath>
#include <stdexcept>
#include <gtest/gtest.h>

#include "power_engine/engine.h"
#include "power_engine/thermal.h"

using power_engine::Engine;
using power_engine::thermal::DeviceLoss;
using power_engine::thermal::Stage;
using power_engine::thermal::ThermalNetwork;

// Foster vs analytical step response: P=10W, R=[0.5,1.0]K/W,
// C=[0.01,0.1]J/K (tau 5ms, 100ms), Tamb=25.
TEST(FosterNetwork, TransientAndSteadyStateMatchAnalytical) {
  ThermalNetwork net =
      ThermalNetwork::foster({{0.5, 0.01}, {1.0, 0.1}}, 25.0);
  // t = 50ms analytical.
  for (int i = 0; i < 500; ++i) net.step(10.0, 100e-6);
  const double ref50 =
      25.0 + 10.0 * 0.5 * (1.0 - std::exp(-50e-3 / 5e-3)) +
      10.0 * 1.0 * (1.0 - std::exp(-50e-3 / 100e-3));
  EXPECT_NEAR(net.tj(), ref50, 0.01 * (ref50 - 25.0));
  // Steady state: Tamb + P*sum(R) = 40.
  for (int i = 0; i < 20000; ++i) net.step(10.0, 100e-6);
  EXPECT_NEAR(net.tj(), 40.0, 0.001 * 15.0);
  EXPECT_THROW(ThermalNetwork::foster({}, 25.0), std::runtime_error);
  EXPECT_THROW(ThermalNetwork::foster({{0.0, 1.0}}, 25.0), std::runtime_error);
}

// Cauer shares the steady state; starts cold and rises monotonically.
TEST(CauerNetwork, SteadyStateAndMonotonicRise) {
  ThermalNetwork net = ThermalNetwork::cauer({{0.5, 0.01}, {1.0, 0.1}}, 25.0);
  EXPECT_DOUBLE_EQ(net.tj(), 25.0);
  double prev = net.tj();
  for (int i = 0; i < 20000; ++i) {
    net.step(10.0, 100e-6);
    EXPECT_GE(net.tj(), prev);
    prev = net.tj();
  }
  EXPECT_NEAR(net.tj(), 40.0, 0.005 * 15.0);
}

// Diode conduction loss: I=(12-0.7)/(10+0.01), P=Vf*I+Ron*I^2, exact.
TEST(DeviceLoss, DiodeConductionMatchesAnalytical) {
  Engine eng;
  eng.setTimeStep(1e-6);
  eng.circuit().addVoltageSource("V1", 1, 0, 12.0);
  eng.circuit().addResistor("R1", 1, 2, 10.0);
  eng.circuit().addDiode("D1", 2, 0, 0.7, 10e-3, 1e6);
  eng.setStopTime(1e-3);
  eng.start();
  while (eng.status() == power_engine::SimulationStatus::Running) eng.step();
  const double i = (12.0 - 0.7) / (10.0 + 0.01);
  const double p = 0.7 * i + 0.01 * i * i;
  const DeviceLoss loss = eng.deviceLoss("D1");
  EXPECT_NEAR(loss.econd, p * 1e-3, 0.01 * p * 1e-3);
  EXPECT_DOUBLE_EQ(loss.esw, 0.0);
  EXPECT_THROW(eng.deviceLoss("R1"), std::runtime_error);  // not a switch/diode
  EXPECT_THROW(eng.junctionTemp("D1"), std::runtime_error);  // not attached
}

// Buck switching loss: 120 periods, seeded ON at t=0 (no edge counted).
// On-edges: 119 x Eon=10uJ; off-edges: 120 x Eoff=15uJ.
TEST(DeviceLoss, BuckSwitchingEnergyCountsEdges) {
  constexpr double T = 50e-6;
  Engine eng;
  eng.setTimeStep(0.5e-6);
  eng.circuit().addVoltageSource("Vin", 1, 0, 12.0);
  eng.circuit().addSwitch("S1", 1, 2, 5e-3, 1e6, true, 10e-6, 15e-6);
  eng.circuit().addDiode("D1", 0, 2, 0.0, 10e-3, 1e6);
  eng.circuit().addInductor("L1", 2, 3, 200e-6, 0.0);
  eng.circuit().addCapacitor("C1", 3, 0, 200e-6, 0.0);
  eng.circuit().addResistor("Rload", 3, 0, 5.0);
  eng.setStopTime(6e-3);
  eng.attachThermal("S1", ThermalNetwork::foster({{0.5, 0.01}}, 25.0));
  for (double t = 0.0; t < 6e-3; t += T) {
    eng.scheduleSwitch("S1", true, t);
    eng.scheduleSwitch("S1", false, t + 0.5 * T);
  }
  eng.start();
  while (eng.status() == power_engine::SimulationStatus::Running) eng.step();
  const DeviceLoss loss = eng.deviceLoss("S1");
  const double expected = 119.0 * 10e-6 + 120.0 * 15e-6;  // 2.99 mJ
  EXPECT_NEAR(loss.esw, expected, 25e-6);  // within one switching energy
  EXPECT_GT(loss.econd, 0.0);
  // Coupled Tj rises above ambient; probe matches accessor.
  EXPECT_GT(eng.junctionTemp("S1"), 25.0);
  EXPECT_DOUBLE_EQ(eng.currentSolution().probes.at("tj:S1"), eng.junctionTemp("S1"));
  EXPECT_TRUE(eng.hasThermal("S1"));
  EXPECT_FALSE(eng.hasThermal("D1"));
}

// Netlist path: EON/EOFF from .model, .thermal attach, .control pwm drive.
TEST(NetlistThermal, AttachAndRun) {
  Engine eng;
  eng.loadNetlist(R"(
.model SW mosfet_ideal RON=5m ROFF=1Meg EON=10u EOFF=15u
V1 1 0 12
S1 1 2 MODEL=SW
Rload 2 0 5
.control pwm switch=S1 freq=20k duty=0.5
.thermal foster device=S1 R1=0.5 C1=0.01 Tamb=25
.tran 1u 2m
.end
)");
  EXPECT_DOUBLE_EQ(eng.circuit().findDevice("S1").eon, 10e-6);
  EXPECT_DOUBLE_EQ(eng.circuit().findDevice("S1").eoff, 15e-6);
  eng.applyPwmSpecs();
  eng.applyThermalSpecs();
  eng.start();
  while (eng.status() == power_engine::SimulationStatus::Running) eng.step();
  // Psw = 25uJ*20kHz = 0.5W dominates; Pcond ~ 14mW. Tj ~ 25.26C.
  const double tj = eng.junctionTemp("S1");
  EXPECT_GT(tj, 25.05);
  EXPECT_LT(tj, 30.0);
  EXPECT_GT(eng.deviceLoss("S1").esw, 0.0);
}
