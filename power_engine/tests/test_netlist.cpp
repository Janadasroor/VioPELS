#include <cmath>
#include <stdexcept>
#include <string>
#include <gtest/gtest.h>

#include "power_engine/engine.h"
#include "power_engine/netlist.h"

using power_engine::Engine;
using power_engine::netlist::Parser;

namespace {

// RC charge netlist: tau = R*C = 1k*1u = 1ms, V0 = 5V.
const char* kRcNetlist = R"(
* RC charging circuit
.param V0 5 RVAL 1k CVAL 1u
V1 1 0 {V0}
R1 1 2 {RVAL}
C1 2 0 {CVAL} IC=0
.tran 1u 5m
.end
)";

}  // namespace

// Full path: netlist -> engine -> transient, matches analytical within 1%.
TEST(NetlistEngine, RcTransientMatchesAnalytical) {
  Engine eng;
  eng.loadNetlist(kRcNetlist);
  eng.start();
  const double tau = 1e-3;
  double vc5ms = 0.0;
  while (eng.status() == power_engine::SimulationStatus::Running) {
    eng.step();
    if (std::abs(eng.currentSolution().t - 5e-3) < 1.1e-6) {
      vc5ms = eng.currentSolution().probes.at("v:2");
    }
  }
  const double ref = 5.0 * (1.0 - std::exp(-5e-3 / tau));
  EXPECT_NEAR(vc5ms, ref, 0.01 * ref);
}

// setParameter re-elaborates: halving R halves tau.
TEST(NetlistEngine, SetParameterOverridesAndReElaborates) {
  Engine eng;
  eng.loadNetlist(kRcNetlist);
  eng.setParameter("RVAL", 500.0);
  ASSERT_NEAR(eng.circuit().findDevice("R1").value, 500.0, 1e-9);
  eng.start();
  double vc1ms = 0.0;
  while (eng.status() == power_engine::SimulationStatus::Running) {
    eng.step();
    if (std::abs(eng.currentSolution().t - 1e-3) < 1.1e-6) {
      vc1ms = eng.currentSolution().probes.at("v:2");
    }
  }
  const double ref = 5.0 * (1.0 - std::exp(-1e-3 / 0.5e-3));  // tau = 0.5ms
  EXPECT_NEAR(vc1ms, ref, 0.01 * ref);
  EXPECT_THROW(eng.setParameter("X", 1.0), std::runtime_error);  // running
}

// Round-trip: parse -> serialize -> parse gives identical devices.
TEST(NetlistParser, RoundTripPreservesDevices) {
  const char* text = R"(
.param FREQ 20k DUTY 0.5
.model SW1 mosfet_ideal VTO=2.0 RON=5m ROFF=1Meg
.model DD1 diode_ideal VF=0.7 RON=10m
V1 1 0 12
S1 1 2 MODEL=SW1 INIT=ON
D1 0 2 MODEL=DD1
L1 2 3 200u IC=0.1
C1 3 0 200u
Rload 3 0 5
T1 4 0 5 0 RATIO=2
Rsec 5 0 10
.control pwm switch=S1 freq={FREQ} duty={DUTY} deadtime=100n
.thermal foster device=S1 R1=0.5 C1=0.01 R2=1.0 C2=0.1 Tamb=25
.tran 0.5u 6m
.end
)";
  Parser p;
  auto a = p.parse(text);
  auto b = p.parse(a.serialize());
  ASSERT_EQ(a.circuit.devices().size(), b.circuit.devices().size());
  for (std::size_t i = 0; i < a.circuit.devices().size(); ++i) {
    const auto& da = a.circuit.devices()[i];
    const auto& db = b.circuit.devices()[i];
    EXPECT_EQ(da.name, db.name);
    EXPECT_EQ(da.type, db.type);
    EXPECT_EQ(da.n1, db.n1);
    EXPECT_EQ(da.n2, db.n2);
    EXPECT_DOUBLE_EQ(da.value, db.value);
    EXPECT_DOUBLE_EQ(da.ic, db.ic);
  }
  ASSERT_EQ(a.pwms.size(), 1u);
  EXPECT_EQ(a.pwms[0].switchName, "S1");
  EXPECT_DOUBLE_EQ(a.pwms[0].freq, 20e3);
  EXPECT_DOUBLE_EQ(a.pwms[0].duty, 0.5);
  ASSERT_EQ(a.thermals.size(), 1u);
  EXPECT_EQ(a.thermals[0].device, "S1");
  EXPECT_TRUE(a.thermals[0].foster);
  ASSERT_EQ(a.thermals[0].r.size(), 2u);
  EXPECT_DOUBLE_EQ(a.thermals[0].r[0], 0.5);
  EXPECT_TRUE(a.tran.given);
  EXPECT_DOUBLE_EQ(a.tran.dt, 0.5e-6);
}

TEST(NetlistParser, SuffixesAndExpressions) {
  Parser p;
  auto r = p.parse(R"(
.param BASE 10k HALF {BASE/2} DBL {2*BASE}
R1 1 0 {HALF}
R2 1 0 {DBL}
R3 1 0 1Meg
C1 1 0 100n
L1 1 0 2m
V1 1 0 1
.end
)");
  EXPECT_DOUBLE_EQ(r.circuit.findDevice("R1").value, 5e3);
  EXPECT_DOUBLE_EQ(r.circuit.findDevice("R2").value, 20e3);
  EXPECT_DOUBLE_EQ(r.circuit.findDevice("R3").value, 1e6);
  EXPECT_DOUBLE_EQ(r.circuit.findDevice("C1").value, 100e-9);
  EXPECT_DOUBLE_EQ(r.circuit.findDevice("L1").value, 2e-3);
}

TEST(NetlistParser, SubcktExpansion) {
  Parser p;
  auto r = p.parse(R"(
.subckt DIV IN OUT
R1 IN OUT 1k
R2 OUT 0 1k
.ends
V1 TOP 0 10
X1 TOP MID DIV
X2 MID BOT DIV
Rload BOT 0 1k
.end
)");
  // TOP=10V; divider chain: TOP-MID-BOT with 1k/1k each + 1k load.
  // X1: R TOP-MID 1k, R MID-0 1k; X2: R MID-BOT 1k, R BOT-0 1k; + Rload BOT-0.
  EXPECT_EQ(r.circuit.devices().size(), 6u);
  // Hierarchical names.
  EXPECT_NO_THROW(r.circuit.findDevice("X1:R1"));
  EXPECT_NO_THROW(r.circuit.findDevice("X2:R2"));
}

// All malformed inputs must throw (no silent ignores).
TEST(NetlistParser, ErrorsThrow) {
  Parser p;
  EXPECT_THROW(p.parse("R1 1 0\nV1 1 0 1\n.end\n"), std::runtime_error);  // missing value
  EXPECT_THROW(p.parse("Z9 1 0 1k\nV1 1 0 1\n.end\n"), std::runtime_error);  // bad kind
  EXPECT_THROW(p.parse(".frobnicate 1\nV1 1 0 1\n.end\n"), std::runtime_error);
  EXPECT_THROW(p.parse("R1 1 0 1k\nR1 2 0 2k\nV1 1 0 1\n.end\n"),
               std::runtime_error);  // duplicate
  EXPECT_THROW(p.parse("S1 1 2 MODEL=NOPE\nV1 1 0 1\n.end\n"), std::runtime_error);
  EXPECT_THROW(p.parse("X1 1 2 NOSUCH\nV1 1 0 1\n.end\n"), std::runtime_error);
  EXPECT_THROW(p.parse(".subckt A 1\nR1 1 0 1k\nV1 1 0 1\n.end\n"),
               std::runtime_error);  // missing .ends
  EXPECT_THROW(p.parse("R1 1 0 1k\n.control pwm switch=R1 freq=1k\nV1 1 0 1\n.end\n"),
               std::runtime_error);  // pwm on resistor
  EXPECT_THROW(p.parse("R1 1 0 {UNDEFINED_PARAM}\nV1 1 0 1\n.end\n"),
               std::runtime_error);
  EXPECT_THROW(p.parse(""), std::runtime_error);  // empty: no devices
}

// Ideal transformer: 12V primary, ratio 2:1, 10 ohm secondary load.
TEST(Transformer, DcRatioAndPowerBalance) {
  Engine eng;
  eng.loadNetlist(R"(
V1 1 0 12
T1 1 0 2 0 RATIO=2
Rsec 2 0 10
.tran 1u 50u
.end
)");
  eng.start();
  while (eng.status() == power_engine::SimulationStatus::Running) eng.step();
  const double vs = eng.currentSolution().probes.at("v:2");
  EXPECT_NEAR(vs, 6.0, 1e-6);
  const double ip = eng.deviceCurrent("T1");  // primary
  const double is = eng.circuit().findDevice("T1").i2_prev;  // secondary
  EXPECT_NEAR(is, -0.6, 1e-9);            // 6V/10ohm, delivered => negative in n3->n4 ref
  EXPECT_NEAR(2.0 * ip + is, 0.0, 1e-9);  // power conservation
}
