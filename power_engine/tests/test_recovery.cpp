#include <cmath>
#include <stdexcept>
#include <gtest/gtest.h>

#include "power_engine/engine.h"
#include "power_engine/netlist.h"

using power_engine::Engine;

// Diode reverse recovery: buck cell switched at 20kHz; each high-side
// turn-on hard-reverse-biases the freewheeling diode with forward current.
// Triangular recovery Irr = 2*Qrr/trr must appear every cycle and end.
TEST(Recovery, DiodeQrrPeakAndDuration) {
  constexpr double kQrr = 100e-9;
  constexpr double kTrr = 1e-6;
  constexpr double kIrr = 2.0 * kQrr / kTrr;  // 0.2A
  constexpr double T = 50e-6;
  Engine eng;
  eng.setTimeStep(50e-9);
  eng.circuit().addVoltageSource("V1", 1, 0, 12.0);
  eng.circuit().addSwitch("Shi", 1, 2, 5e-3, 1e6, true);
  eng.circuit().addDiode("Dlo", 0, 2, 0.0, 10e-3, 1e6, kQrr, kTrr);
  eng.circuit().addInductor("L1", 2, 3, 100e-6, 0.0);
  eng.circuit().addResistor("Rload", 3, 0, 10.0);
  eng.setStopTime(1e-3);
  for (double t = 0.0; t < 1e-3; t += T) {
    eng.scheduleSwitch("Shi", true, t);
    eng.scheduleSwitch("Shi", false, t + 0.5 * T);
  }
  eng.start();
  double minId = 0.0;
  while (eng.status() == power_engine::SimulationStatus::Running) {
    eng.step();
    const double id = eng.deviceCurrent("Dlo");
    ASSERT_TRUE(std::isfinite(id));
    if (id < minId) minId = id;
  }
  EXPECT_NEAR(minId, -kIrr, 0.25 * kIrr);
  EXPECT_DOUBLE_EQ(eng.circuit().findDevice("Dlo").recT, 0.0);  // drained
  EXPECT_DOUBLE_EQ(eng.circuit().findDevice("Dlo").recE, 0.0);  // consumed
  EXPECT_GE(eng.deviceLoss("Dlo").esw, 0.0);
}

// IGBT-style tail: switch opens 10A inductive current with ttail=10us,
// tailk=0.1; branch current must decay exponentially, node stays clamped.
TEST(Recovery, SwitchTailDecaysExponentially) {
  constexpr double kTtail = 10e-6;
  Engine eng;
  eng.setTimeStep(1e-6);
  eng.circuit().addVoltageSource("V1", 1, 0, 12.0);
  eng.circuit().addResistor("R1", 1, 2, 1.0);
  eng.circuit().addInductor("L1", 2, 3, 1e-3, 0.0);
  eng.circuit().addSwitch("S1", 3, 0, 5e-3, 1e6, true, 0.0, 0.0, kTtail, 0.1);
  eng.circuit().addDiode("Dfree", 0, 3, 0.7, 10e-3, 1e6);  // freewheel clamp
  eng.setStopTime(4e-3);
  constexpr double tOpen = 2e-3;
  eng.scheduleSwitch("S1", false, tOpen);
  eng.start();
  double iAtTau = 0.0;
  bool gotTau = false;
  double vMin = 1e18;
  while (eng.status() == power_engine::SimulationStatus::Running) {
    eng.step();
    const double t = eng.time();
    const double isw = eng.deviceCurrent("S1");
    const double va = eng.currentSolution().probes.at("v:3");
    ASSERT_TRUE(std::isfinite(isw)) << "t=" << t;
    if (va < vMin) vMin = va;
    if (!gotTau && t >= tOpen + kTtail) {
      iAtTau = isw;
      gotTau = true;
    }
  }
  ASSERT_TRUE(gotTau);
  // Itail0 = 0.1 * I(2ms) ~= 0.1 * 10.4 = 1.04A; at +ttail expect /e.
  const double i0 = 12.0 * (1.0 - std::exp(-2.0));
  EXPECT_NEAR(iAtTau, 0.1 * i0 / std::exp(1.0), 0.25 * 0.1 * i0);
  EXPECT_GT(vMin, -2.0);  // freewheel clamp held (no inductive spike)
  EXPECT_DOUBLE_EQ(eng.circuit().findDevice("S1").recT, 0.0);  // tail over
}

// Netlist carries the new loss/recovery params and round-trips them.
TEST(Recovery, NetlistParams) {
  power_engine::netlist::Parser p;
  auto r = p.parse(R"(
.model DD diode_ideal VF=0.7 RON=10m QRR=100n TRR=1u
.model SW mosfet_ideal RON=5m ROFF=1Meg TTAIL=10u TAILK=0.1
V1 1 0 12
S1 1 2 MODEL=SW
D1 2 0 MODEL=DD QRR=200n
.tran 1u 1m
.end
)");
  EXPECT_DOUBLE_EQ(r.circuit.findDevice("S1").ttail, 10e-6);
  EXPECT_DOUBLE_EQ(r.circuit.findDevice("S1").tailk, 0.1);
  EXPECT_DOUBLE_EQ(r.circuit.findDevice("D1").qrr, 200e-9);  // inline wins
  auto r2 = p.parse(r.serialize());
  EXPECT_DOUBLE_EQ(r2.circuit.findDevice("S1").ttail, 10e-6);
  EXPECT_DOUBLE_EQ(r2.circuit.findDevice("D1").qrr, 200e-9);
  EXPECT_THROW(p.parse("V1 1 0 1\nD1 1 0 QRR=1u\n.end\n"), std::runtime_error);  // Qrr w/o Trr
}
