#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <vector>
#include <gtest/gtest.h>

#include "power_engine/engine.h"
#include "power_engine/magnetics.h"
#include "power_engine/netlist.h"

using power_engine::Engine;

// Open secondary: at t=0+ the full step appears across L1, so the induced
// secondary voltage is v2 = (M/L1)*v1 = k*sqrt(L2/L1)*10V = 9V.
TEST(CoupledInductors, OpenSecondaryVoltageRatio) {
  Engine eng;
  eng.setTimeStep(1e-6);
  eng.circuit().addVoltageSource("V1", 1, 0, 10.0);
  eng.circuit().addResistor("R1", 1, 2, 100.0);
  eng.circuit().addCoupledInductors("W1", 2, 0, 3, 0, 1e-3, 1e-3, 0.9);
  eng.circuit().addResistor("Rmeas", 3, 0, 1e6);  // near-open probe
  eng.setStopTime(2e-6);
  eng.start();
  eng.step();  // first step: transient division, no resistive drops yet
  EXPECT_NEAR(eng.currentSolution().probes.at("v:3"), 9.0, 0.05 * 9.0);
}

// Shorted secondary: primary sees leakage L1*(1-k^2), tau = L/R = 19us.
// (Asserted on the settled exponential, not the first step: zero-state
// histories make the first trapezoidal step see half the applied step.)
TEST(CoupledInductors, ShortedSecondarySlope) {
  Engine eng;
  eng.setTimeStep(1e-6);
  eng.circuit().addVoltageSource("V1", 1, 0, 10.0);
  eng.circuit().addResistor("R1", 1, 2, 10.0);
  eng.circuit().addCoupledInductors("W1", 2, 0, 3, 0, 1e-3, 1e-3, 0.9);
  eng.circuit().addResistor("Rshort", 3, 0, 1e-3);
  eng.setStopTime(20e-6);
  eng.start();
  while (eng.status() == power_engine::SimulationStatus::Running) eng.step();
  const double i1 = eng.deviceCurrent("W1");
  const double i2 = eng.circuit().findDevice("W1").i2_prev;
  // i(t) = (V/R)(1 - exp(-t/tau)), tau = 1e-3*(1-0.81)/10 = 19us.
  const double ref = 1.0 * (1.0 - std::exp(-20e-6 / 19e-6));
  EXPECT_NEAR(i1, ref, 0.03 * ref);
  EXPECT_LT(i2, 0.0);  // Lenz opposition
}

// Saturable inductor: driven past Isat, current runs away above the linear
// prediction but stays under the resistive limit. Newton must engage.
TEST(SaturableInductor, CurrentRunsAwayPastKnee) {
  Engine eng;
  eng.setTimeStep(10e-6);
  eng.circuit().addVoltageSource("V1", 1, 0, 10.0);
  eng.circuit().addResistor("R1", 1, 2, 1.0);
  eng.circuit().addSaturableInductor("Y1", 2, 0, 10e-3, 1e-3, 1.0, 0.0);
  eng.setStopTime(2e-3);
  eng.start();
  while (eng.status() == power_engine::SimulationStatus::Running) eng.step();
  const double iSat = eng.deviceCurrent("Y1");
  const double iLin = 10.0 * (1.0 - std::exp(-2e-3 / 10e-3));  // linear 10mH
  EXPECT_GT(iSat, 1.5 * iLin);
  EXPECT_LT(iSat, 9.5);  // resistive limit 10V/1ohm
  EXPECT_GT(eng.solverStats().newtonIters, 0);
}

// Below the knee it behaves as the unsaturated inductance.
TEST(SaturableInductor, SmallSignalMatchesLinear) {
  Engine eng;
  eng.setTimeStep(10e-6);
  eng.circuit().addVoltageSource("V1", 1, 0, 0.5);  // stays << Isat
  eng.circuit().addResistor("R1", 1, 2, 1.0);
  eng.circuit().addSaturableInductor("Y1", 2, 0, 10e-3, 1e-3, 1.0, 0.0);
  eng.setStopTime(2e-3);
  eng.start();
  while (eng.status() == power_engine::SimulationStatus::Running) eng.step();
  const double iSat = eng.deviceCurrent("Y1");
  const double iLin = 0.5 * (1.0 - std::exp(-2e-3 / 10e-3));
  EXPECT_NEAR(iSat, iLin, 0.02 * iLin);
}

TEST(SaturableInductor, BadParamsThrow) {
  Engine eng;
  EXPECT_THROW(eng.circuit().addSaturableInductor("Y1", 1, 0, 1e-3, 2e-3, 1.0),
               std::runtime_error);  // Lsat > Lunsat
  EXPECT_THROW(eng.circuit().addSaturableInductor("Y1", 1, 0, 1e-3, 1e-3, 0.0),
               std::runtime_error);  // Isat = 0
  power_engine::netlist::Parser p;
  auto r = p.parse("V1 1 0 10\nR1 1 2 1\nY1 2 0 LUNSAT=10m LSAT=1m ISAT=1\n.end\n");
  EXPECT_DOUBLE_EQ(r.circuit.findDevice("Y1").lsat, 1e-3);
  auto r2 = p.parse(r.serialize());
  EXPECT_DOUBLE_EQ(r2.circuit.findDevice("Y1").isat, 1.0);
}

// Unphysical couplings are rejected, not silently clipped.
TEST(CoupledInductors, BadCouplingThrows) {
  Engine eng;
  EXPECT_THROW(eng.circuit().addCoupledInductors("W1", 1, 0, 2, 0, 1e-3, 1e-3, 1.0),
               std::runtime_error);  // singular companion
  EXPECT_THROW(eng.circuit().addCoupledInductors("W1", 1, 0, 2, 0, 1e-3, 1e-3, 1.2),
               std::runtime_error);
  EXPECT_THROW(eng.circuit().addCoupledInductors("W1", 1, 0, 2, 0, 1e-3, 1e-3, 0.0),
               std::runtime_error);
}

// Netlist W device parses, elaborates, and round-trips.
TEST(CoupledInductors, NetlistWinding) {
  power_engine::netlist::Parser p;
  auto r = p.parse(R"(
V1 1 0 10
R1 1 2 100
W1 2 0 3 0 L1=1m L2=1m K=0.9
Rmeas 3 0 1Meg
.tran 1u 2u
.end
)");
  EXPECT_DOUBLE_EQ(r.circuit.findDevice("W1").l1, 1e-3);
  EXPECT_NEAR(r.circuit.findDevice("W1").m, 0.9e-3, 1e-12);
  auto r2 = p.parse(r.serialize());
  EXPECT_NEAR(r2.circuit.findDevice("W1").m, 0.9e-3, 1e-9);
  EXPECT_THROW(p.parse("V1 1 0 1\nW1 1 0 2 0 L1=1m L2=1m K=0.5 M=0.4m\n.end\n"),
               std::runtime_error);  // K and M together
}

// --- Hysteresis material (saturating-offset memory, Preisach family) ---

namespace {
constexpr double kPiH = std::numbers::pi;
// Normalized test material: Bs=1T, a=100A/m, Hc=50A/m.
power_engine::magnetics::HystereticMaterial testMat() { return {1.0, 100.0, 50.0}; }

// Sine H drive helper: B trace over nCycles (stepsPerCycle each), ending
// exactly closed (k <= spans the full periods).
std::vector<double> driveLoop(power_engine::magnetics::HysteresisCore& core, double hMax,
                              int nCycles, int stepsPerCycle) {
  std::vector<double> b;
  for (int k = 0; k <= nCycles * stepsPerCycle; ++k) {
    const double h = hMax * std::sin(2.0 * kPiH * k / stepsPerCycle);
    b.push_back(core.update(h));
  }
  return b;
}
}  // namespace

// Major loop landmarks: coercivity, remanence, saturation from the params.
// Textbook orientation: ascending (up from -saturation) crosses zero at
// +Hc with -Br at H=0; descending crosses at -Hc with +Br at H=0.
TEST(HysteresisLoop, ShapeMatchesParameters) {
  power_engine::magnetics::HysteresisCore core(testMat());
  // Saturate at -Hmax first, then sweep up (B must cross 0 at +Hc).
  for (int i = 100; i >= -100; --i) core.update(-1000.0 * i / 100.0);
  double hCross = 0.0;
  double bPrev = core.flux();
  for (int i = -100; i <= 100; ++i) {
    const double h = 1000.0 * i / 100.0;
    const double b = core.update(h);
    if (bPrev < 0.0 && b >= 0.0) hCross = h;
    bPrev = b;
  }
  EXPECT_NEAR(hCross, 50.0, 0.02 * 50.0);  // coercivity on ascending limb
  // Remanence: ascending branch at H=0 is -Bs*tanh(Hc/a).
  for (int i = -100; i <= 0; ++i) core.update(1000.0 * i / 100.0);
  EXPECT_NEAR(core.flux(), -std::tanh(0.5), 0.03);
  // Saturation: within 1% of Bs at 10*a.
  core.update(1000.0);
  EXPECT_NEAR(core.flux(), 1.0, 0.01);
  // Branch accessors agree with the formulas.
  EXPECT_NEAR(core.ascending(0.0), -std::tanh(0.5), 1e-12);
  EXPECT_NEAR(core.descending(0.0), std::tanh(0.5), 1e-12);
  EXPECT_THROW(power_engine::magnetics::HysteresisCore(
                   {std::numeric_limits<double>::infinity(), 100.0, 50.0}),
               std::runtime_error);
  EXPECT_THROW(power_engine::magnetics::HysteresisCore({0.0, 100.0, 50.0}),
               std::runtime_error);
  EXPECT_THROW(power_engine::magnetics::HysteresisCore({1.0, 0.0, 50.0}),
               std::runtime_error);
  EXPECT_THROW(power_engine::magnetics::HysteresisCore({1.0, 100.0, -1.0}),
               std::runtime_error);
}

// Loop area over steady cycles vs the exact branch integral:
// A = 4*Hc*Basc(Hmax) (H_asc(B) - H_desc(B) = -2*Hc constant!).
TEST(HysteresisLoop, AreaMatchesAnalytic) {
  power_engine::magnetics::HysteresisCore core(testMat());
  constexpr double kHm = 1000.0;
  driveLoop(core, kHm, 1, 400);  // precondition (virgin + first cycle)
  const double before = core.loss();
  driveLoop(core, kHm, 2, 400);
  const double area = (core.loss() - before) / 2.0;
  const double ref = 4.0 * 50.0 * std::tanh((kHm + 50.0) / 100.0);
  EXPECT_NEAR(area, ref, 0.02 * ref);
  EXPECT_GT(area, 0.0);
}

// Hc = 0 degenerates to the anhysteretic curve (zero area).
TEST(HysteresisLoop, ZeroCoercivityIsLossless) {
  power_engine::magnetics::HysteresisCore core({1.0, 100.0, 0.0});
  driveLoop(core, 500.0, 2, 200);
  EXPECT_NEAR(core.loss(), 0.0, 1e-9);
}

// Reversal memory done right: idempotent re-evaluation, periodic input
// gives exactly periodic output, nested loops span less + close exactly.
TEST(HysteresisLoop, MemoryProperties) {
  power_engine::magnetics::HysteresisCore core(testMat());
  core.update(-1000.0);
  core.update(1000.0);
  core.update(200.0);
  const double bAt200 = core.update(200.0);  // hold: idempotent
  EXPECT_DOUBLE_EQ(core.update(200.0), bAt200);
  // Periodic H drive -> exactly periodic B (cycles 3,4,5 identical).
  power_engine::magnetics::HysteresisCore cyc(testMat());
  std::vector<std::vector<double>> traces;
  for (int cycNo = 0; cycNo < 5; ++cycNo) {
    traces.push_back(driveLoop(cyc, 800.0, 1, 200));
  }
  for (int i = 0; i < 200; ++i) {
    EXPECT_DOUBLE_EQ(traces[3][i], traces[4][i]);
    EXPECT_DOUBLE_EQ(traces[2][i], traces[4][i]);
  }
  // Out-and-back trip closes EXACTLY (return-point memory): descend to
  // saturation, dip up 50 A/m (< 2*Hc, no wipe), return to the same state.
  power_engine::magnetics::HysteresisCore nest(testMat());
  driveLoop(nest, 800.0, 2, 200);
  nest.update(-800.0);
  const double bAtNeg800 = nest.update(-800.0);
  nest.update(-750.0);
  EXPECT_DOUBLE_EQ(nest.update(-800.0), bAtNeg800);
  // Nested minor loop spans less than the major loop.
  double mn = 1e18, mx = -1e18, mnN = 1e18, mxN = -1e18;
  for (int k = 0; k < 200; ++k) {  // major cycle span
    const double b = nest.update(800.0 * std::sin(2.0 * kPiH * k / 200.0));
    mn = std::min(mn, b);
    mx = std::max(mx, b);
  }
  // Narrow wiggle (+/-20 A/m) around the post-major state stays a small
  // lens (< 50% of major span); excursion never nears saturation clamps.
  const double hBase = 800.0 * std::sin(2.0 * kPiH * 199.0 / 200.0);
  for (int k = 0; k <= 80; ++k) {
    const double h = hBase + 20.0 * std::sin(2.0 * kPiH * k / 80.0);
    const double b = nest.update(h);
    mnN = std::min(mnN, b);
    mxN = std::max(mxN, b);
  }
  EXPECT_LT(mxN - mnN, 0.5 * (mx - mn));
}

// Rate independence: same H path at different step sizes gives identical
// B at shared H points (no dt anywhere in the model).
TEST(HysteresisLoop, RateIndependent) {
  power_engine::magnetics::HysteresisCore fine(testMat()), coarse(testMat());
  for (int k = 0; k <= 800; ++k) {
    const double h = 800.0 * std::sin(2.0 * kPiH * k / 800.0);
    const double bf = fine.update(h);
    if (k % 4 == 0) {
      const double bc = coarse.update(h);
      EXPECT_NEAR(bc, bf, 1e-9);  // NEAR (different op paths round differently)
    }
  }
}

// Classical eddy loss density vs hand calculation.
TEST(EddyLoss, MatchesFormula) {
  // rho=50e-8 (steel), d=0.5mm, f=50Hz, Bpk=1T.
  const double ref = kPiH * kPiH / (6.0 * 50e-8) * 0.5e-3 * 0.5e-3 * 2500.0 * 1.0;
  EXPECT_NEAR(power_engine::magnetics::eddyLossDensity(50e-8, 0.5e-3, 50.0, 1.0), ref,
              1e-9 * ref);
  EXPECT_DOUBLE_EQ(power_engine::magnetics::eddyLossDensity(50e-8, 0.5e-3, 0.0, 1.0), 0.0);
  EXPECT_THROW(power_engine::magnetics::eddyLossDensity(0.0, 0.5e-3, 50.0, 1.0),
               std::runtime_error);
  EXPECT_THROW(power_engine::magnetics::eddyLossDensity(50e-8, -1e-3, 50.0, 1.0),
               std::runtime_error);
}
