#include <cmath>
#include <complex>
#include <numbers>
#include <stdexcept>
#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include "power_engine/circuit.h"
#include "power_engine/engine.h"
#include "power_engine/statespace.h"

using power_engine::Circuit;
using power_engine::DeviceType;
using power_engine::Engine;
using power_engine::statespace::averageStateSpace;
using power_engine::statespace::evalDutyTransfer;
using power_engine::statespace::evalTransfer;
using power_engine::statespace::exportStateSpace;
using power_engine::statespace::StateSpace;

namespace {
constexpr double kPi = std::numbers::pi;
constexpr double kVin = 12.0, kR = 5.0, kL = 200e-6, kC = 200e-6;

// Ideal buck (Vf=0), switch frozen in the requested gate state.
Circuit buck(bool closed) {
  Circuit c;
  c.addVoltageSource("Vin", 1, 0, kVin);
  c.addSwitch("S1", 1, 2, 5e-3, 1e6, closed);
  c.addDiode("D1", 0, 2, 0.0, 10e-3, 1e6);
  if (!closed) c.findDevice("D1").conducting = true;  // OFF state: diode carries
  c.addInductor("L1", 2, 3, kL, 0.0);
  c.addCapacitor("C1", 3, 0, kC, 0.0);
  c.addResistor("Rload", 3, 0, kR);
  return c;
}
}  // namespace

TEST(BuckStateSpace, OnStateMatchesHandMatrices) {
  const StateSpace ss = exportStateSpace(buck(true), {"v:3"});
  ASSERT_EQ(ss.a.rows(), 2);
  ASSERT_EQ(ss.a.cols(), 2);
  ASSERT_EQ(ss.b.cols(), 2);  // Vin + const 1
  EXPECT_EQ(ss.stateNames, (std::vector<std::string>{"i:L1", "v:3"}));
  EXPECT_EQ(ss.inputNames, (std::vector<std::string>{"Vin", "1"}));
  EXPECT_EQ(ss.outputNames, (std::vector<std::string>{"v:3"}));
  // A = [[-Ron/L, -1/L], [1/C, -1/(RC)]] (Roff corrections ~1e-7 abs).
  EXPECT_NEAR(ss.a(0, 0), -5e-3 / kL, 1e-6);
  EXPECT_NEAR(ss.a(0, 1), -1.0 / kL, 1e-6);
  EXPECT_NEAR(ss.a(1, 0), 1.0 / kC, 1e-3);
  EXPECT_NEAR(ss.a(1, 1), -1.0 / (kR * kC), 1e-6);
  // B: Vin drives di/dt through 1/L; const column is zero (Vf=0).
  EXPECT_NEAR(ss.b(0, 0), 1.0 / kL, 1e-3);
  EXPECT_NEAR(ss.b(1, 0), 0.0, 1e-9);
  EXPECT_LT(std::abs(ss.b(0, 1)), 1e-9);
  EXPECT_LT(std::abs(ss.b(1, 1)), 1e-9);
  // C picks v_C; no feedthrough.
  EXPECT_DOUBLE_EQ(ss.c(0, 0), 0.0);
  EXPECT_DOUBLE_EQ(ss.c(0, 1), 1.0);
  EXPECT_DOUBLE_EQ(ss.d(0, 0), 0.0);
  EXPECT_DOUBLE_EQ(ss.d(0, 1), 0.0);
}

TEST(BuckStateSpace, EigenvaluesAreLcPoles) {
  const StateSpace ss = exportStateSpace(buck(true), {"v:3"});
  Eigen::EigenSolver<Eigen::MatrixXd> es(ss.a);
  ASSERT_EQ(es.eigenvalues().size(), 2);
  const std::complex<double> l1 = es.eigenvalues()(0);
  const std::complex<double> l2 = es.eigenvalues()(1);
  EXPECT_NEAR(l1.real(), -512.5, 0.01 * 512.5);
  EXPECT_NEAR(std::abs(l1.imag()), 4976.3, 0.005 * 4976.3);
  EXPECT_NEAR(l2.real(), l1.real(), 1e-9);
  EXPECT_NEAR(l2.imag(), -l1.imag(), 1e-6);
}

TEST(BuckStateSpace, OffStateStructure) {
  const StateSpace off = exportStateSpace(buck(false), {"v:3"});
  // Same LC skeleton (different switch-node damping): off-diagonals exact.
  EXPECT_NEAR(off.a(0, 1), -1.0 / kL, 1e-9);
  EXPECT_NEAR(off.a(1, 0), 1.0 / kC, 1e-9);
  EXPECT_LT(off.a(0, 0), 0.0);
  EXPECT_LT(off.a(1, 1), 0.0);
  // Vin feeds only through Roff: essentially zero.
  EXPECT_LT(std::abs(off.b(0, 0)), 1.0);
  EXPECT_NEAR(off.b(1, 0), 0.0, 1e-9);
}

// Acceptance: duty-averaged matrices reproduce the analytic second-order
// Gvd(s) = Vin/(1 + sL/R + s^2 L C), and the line-to-output
// M(s) = D/(1 + sL/R + s^2 L C).
TEST(BuckStateSpace, AveragedMatchesAnalyticModel) {
  const StateSpace on = exportStateSpace(buck(true), {"v:3"});
  const StateSpace off = exportStateSpace(buck(false), {"v:3"});
  const StateSpace avg = averageStateSpace(on, off, 0.5);
  auto wrap180 = [](double deg) {
    while (deg > 180.0) deg -= 360.0;
    while (deg <= -180.0) deg += 360.0;
    return deg;
  };
  auto check = [&](const std::complex<double>& got, const std::complex<double>& ref,
                   double f) {
    EXPECT_NEAR(std::abs(got), std::abs(ref), 0.005 * std::abs(ref)) << "f=" << f;
    EXPECT_NEAR(std::abs(wrap180(std::arg(got) * 180.0 / kPi - std::arg(ref) * 180.0 / kPi)),
                0.0, 0.5)
        << "f=" << f;
  };
  // Line-to-output M(s): DC gain = D.
  {
    const std::complex<double> dc = evalTransfer(avg, 0, 0, {1e-9, 0.0});
    EXPECT_NEAR(dc.real(), 0.5, 0.005 * 0.5);
    EXPECT_NEAR(std::abs(dc.imag()), 0.0, 1e-6);
  }
  Eigen::VectorXd uSs(2);
  uSs << kVin, 1.0;  // [Vin, const 1]
  // Averaged switch conduction loss Rs = D*RonSw + (1-D)*RonD trims the
  // peak Q (exported matrices include parasitics; ideal analytic overshoots
  // the peak by ~3.6%). Lossy reference, same form as the loop-gain model.
  const double kRs = 0.5 * 5e-3 + 0.5 * 10e-3;
  for (double f : {10.0, 100.0, 796.0, 2000.0}) {
    const std::complex<double> s(0.0, 2.0 * kPi * f);
    const std::complex<double> den =
        (kR + kRs + s * (kL + kC * kR * kRs) + s * s * kL * kC * kR) / kR;
    check(evalTransfer(avg, 0, 0, s), 0.5 / den, f);                 // M(s)
    check(evalDutyTransfer(on, off, 0.5, uSs, 0, s), kVin / den, f);  // Gvd(s)
  }
}

TEST(StateSpaceHelpers, AveragingValidation) {
  const StateSpace on = exportStateSpace(buck(true), {"v:3"});
  const StateSpace off = exportStateSpace(buck(false), {"v:3"});
  EXPECT_THROW(averageStateSpace(on, off, -0.1), std::runtime_error);
  EXPECT_THROW(averageStateSpace(on, off, 1.5), std::runtime_error);
  StateSpace other = off;
  other.outputNames = {"v:2"};
  EXPECT_THROW(averageStateSpace(on, other, 0.5), std::runtime_error);
  const StateSpace avg = averageStateSpace(on, off, 0.5);
  for (int r = 0; r < avg.a.rows(); ++r) {
    for (int c = 0; c < avg.a.cols(); ++c) {
      EXPECT_DOUBLE_EQ(avg.a(r, c), 0.5 * (on.a(r, c) + off.a(r, c)));
    }
  }
  EXPECT_THROW(evalTransfer(avg, 5, 0, {0.0, 1.0}), std::runtime_error);
  EXPECT_THROW(evalTransfer(avg, 0, 5, {0.0, 1.0}), std::runtime_error);
}

TEST(StateSpaceExport, RejectsUnsupported) {
  EXPECT_THROW(exportStateSpace(buck(true), {}), std::runtime_error);
  EXPECT_THROW(exportStateSpace(buck(true), {"v:0"}), std::runtime_error);
  EXPECT_THROW(exportStateSpace(buck(true), {"i:L1"}), std::runtime_error);
  Circuit c;
  c.addVoltageSource("V1", 1, 2, 5.0);  // floating source
  c.addResistor("R1", 2, 0, 10.0);
  EXPECT_THROW(exportStateSpace(c, {"v:2"}), std::runtime_error);
  Circuit t;
  t.addVoltageSource("V1", 1, 0, 5.0);
  t.addTransformer("T1", 1, 0, 2, 0, 2.0);
  t.addResistor("R1", 2, 0, 10.0);
  EXPECT_THROW(exportStateSpace(t, {"v:2"}), std::runtime_error);
}

// Current-source inputs (ISrc struct, rhsI/gDI incidence, B I-columns):
// Isrc-driven RC: dv/dt = (I - v/R)/C.
TEST(StateSpaceInputs, CurrentSourceDrivenRc) {
  Circuit c;
  c.addCurrentSource("I1", 0, 1, 2.0);
  c.addResistor("R1", 1, 0, 10.0);
  c.addCapacitor("C1", 1, 0, 100e-6);
  const StateSpace ss = exportStateSpace(c, {"v:1"});
  ASSERT_EQ(ss.a.rows(), 1);
  EXPECT_EQ(ss.inputNames, (std::vector<std::string>{"I1", "1"}));
  EXPECT_NEAR(ss.a(0, 0), -1.0 / (10.0 * 100e-6), 1e-9);
  EXPECT_NEAR(ss.b(0, 0), 1.0 / 100e-6, 1e-6);  // dV/dI column
  EXPECT_DOUBLE_EQ(ss.b(0, 1), 0.0);            // const column
  EXPECT_DOUBLE_EQ(ss.c(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(ss.d(0, 0), 0.0);
  EXPECT_DOUBLE_EQ(ss.d(0, 1), 0.0);
}

// Algebraic-node output with an I-source present (gAI incidence, ai-path
// C/D): divider + current injection at the algebraic node, one dynamic
// node elsewhere (export requires states). v2 independent of v3.
TEST(StateSpaceInputs, AlgebraicNodeWithCurrentSource) {
  Circuit c;
  c.addVoltageSource("V1", 1, 0, 10.0);
  c.addResistor("R1", 1, 2, 10.0);
  c.addResistor("R2", 2, 0, 10.0);
  c.addCurrentSource("I1", 0, 2, 1.0);
  c.addCapacitor("C3", 1, 3, 100e-6);
  c.addResistor("R3", 3, 0, 10.0);
  const StateSpace ss = exportStateSpace(c, {"v:2"});
  ASSERT_EQ(ss.a.rows(), 1);
  EXPECT_EQ(ss.inputNames, (std::vector<std::string>{"V1", "I1", "1"}));
  // v2 = (Vin/10 + 1.0)/(1/10 + 1/10): dV/dVin = 0.5, dV/dI = 5.0, no dynamics.
  EXPECT_DOUBLE_EQ(ss.c(0, 0), 0.0);
  EXPECT_NEAR(ss.d(0, 0), 0.5, 1e-12);
  EXPECT_NEAR(ss.d(0, 1), 5.0, 1e-9);
  EXPECT_DOUBLE_EQ(ss.d(0, 2), 0.0);
}

// Forced-node output (D = wF row): buck ON-state read at Vin.
TEST(StateSpaceInputs, ForcedNodeFeedthrough) {
  const StateSpace ss = exportStateSpace(buck(true), {"v:1"});
  EXPECT_EQ(ss.inputNames.front(), "Vin");
  EXPECT_DOUBLE_EQ(ss.d(0, 0), 1.0);
  for (int c = 1; c < ss.d.cols(); ++c) EXPECT_DOUBLE_EQ(ss.d(0, c), 0.0);
}

// Flipped source leg (gain -1) with a state present.
TEST(StateSpaceInputs, FlippedSourceGain) {
  Circuit c;
  c.addVoltageSource("V1", 0, 2, 5.0);  // node2 = -5V
  c.addResistor("R1", 2, 0, 10.0);
  c.addInductor("L1", 2, 3, 1e-3, 0.0);
  c.addResistor("R3", 3, 0, 10.0);
  const StateSpace ss = exportStateSpace(c, {"v:2"});
  EXPECT_EQ(ss.inputNames, (std::vector<std::string>{"V1", "1"}));
  EXPECT_NEAR(ss.d(0, 0), -1.0, 1e-12);
  EXPECT_DOUBLE_EQ(ss.d(0, 1), 0.0);
  EXPECT_DOUBLE_EQ(ss.c(0, 0), 0.0);  // forced node independent of iL
}

// Algebraic-node output alongside real states (ai-path C unit solves):
// buck ON-state read at the switch node.
TEST(StateSpaceInputs, BuckSwitchNodeOutput) {
  const StateSpace ss = exportStateSpace(buck(true), {"v:2"});
  EXPECT_EQ(ss.outputNames, (std::vector<std::string>{"v:2"}));
  EXPECT_TRUE(std::isfinite(ss.c(0, 0)));
  EXPECT_TRUE(std::isfinite(ss.c(0, 1)));
  EXPECT_TRUE(std::isfinite(ss.d(0, 0)));
}

TEST(StateSpaceExport, MoreRejectsUnsupported) {
  Circuit t;
  t.addVoltageSource("V1", 1, 0, 5.0);
  t.addSwitch("S1", 1, 2, 5e-3, 1e6, true);
  t.addResistor("R1", 2, 0, 10.0);
  t.findDevice("S1").recT = 1e-6;  // active tail: no frozen linear model
  EXPECT_THROW(exportStateSpace(t, {"v:2"}), std::runtime_error);
  Circuit d;
  d.addVoltageSource("V1", 1, 0, 5.0);
  d.addDiode("D1", 1, 2, 0.0, 10e-3, 1e6);
  d.addResistor("R1", 2, 0, 10.0);
  d.findDevice("D1").recT = 1e-6;  // in recovery: no frozen linear model
  EXPECT_THROW(exportStateSpace(d, {"v:2"}), std::runtime_error);
  Circuit f;
  f.addCurrentSource("I1", 1, 2, 1.0);
  f.addResistor("R1", 2, 0, 10.0);  // node 1 floats on the source alone
  EXPECT_THROW(exportStateSpace(f, {"v:2"}), std::runtime_error);
  Circuit cv;
  cv.addVoltageSource("V1", 1, 0, 5.0);
  cv.addCapacitor("C1", 1, 0, 100e-6);  // C across ideal V: singular block
  EXPECT_THROW(exportStateSpace(cv, {"v:1"}), std::runtime_error);
}

// Diode-Vf averaged-DC end-to-end: averaged buck with Vf = 0.7 whose DC
// point (-A^-1 B u) must match the settled transient (validates the Vf
// affine path through averaging; the sign pins are the Isrc tests above
// — this circuit has no current sources, so old and new agree here).
TEST(StateSpaceVf, AveragedDcMatchesTransient) {
  constexpr double kVf = 0.7, kDuty = 0.5;
  auto buckVf = [&](bool closed) {
    Circuit c;
    c.addVoltageSource("Vin", 1, 0, 12.0);
    c.addSwitch("S1", 1, 2, 5e-3, 1e6, closed);
    c.addDiode("D1", 0, 2, kVf, 10e-3, 1e6);
    if (!closed) c.findDevice("D1").conducting = true;
    c.addInductor("L1", 2, 3, 200e-6, 0.0);
    c.addCapacitor("C1", 3, 0, 200e-6, 0.0);
    c.addResistor("Rload", 3, 0, 5.0);
    return c;
  };
  const StateSpace on = exportStateSpace(buckVf(true), {"v:3"});
  const StateSpace off = exportStateSpace(buckVf(false), {"v:3"});
  const StateSpace avg = averageStateSpace(on, off, kDuty);
  Eigen::VectorXd u(2);
  u << 12.0, 1.0;
  const Eigen::VectorXd xss = -avg.a.fullPivLu().solve(avg.b * u);
  // Brute-force switched transient to settle (Vf drops Vout ~0.35V).
  Engine eng;
  eng.setTimeStep(0.5e-6);
  eng.circuit().addVoltageSource("Vin", 1, 0, 12.0);
  eng.circuit().addSwitch("S1", 1, 2, 5e-3, 1e6, true);
  eng.circuit().addDiode("D1", 0, 2, kVf, 10e-3, 1e6);
  eng.circuit().addInductor("L1", 2, 3, 200e-6, 0.0);
  eng.circuit().addCapacitor("C1", 3, 0, 200e-6, 0.0);
  eng.circuit().addResistor("Rload", 3, 0, 5.0);
  constexpr double kT = 50e-6;
  for (double t = 0.0; t < 12e-3; t += kT) {
    eng.scheduleSwitch("S1", true, t);
    eng.scheduleSwitch("S1", false, t + kDuty * kT);
  }
  eng.setStopTime(12e-3);
  eng.start();
  double ilSum = 0.0, vcSum = 0.0;
  long long cnt = 0;
  while (eng.status() == power_engine::SimulationStatus::Running) {
    eng.step();
    // Mean over the last period (boundary samples read valley, not mean).
    if (eng.time() > 12e-3 - kT) {
      ilSum += eng.deviceCurrent("L1");
      vcSum += eng.currentSolution().probes.at("v:3");
      ++cnt;
    }
  }
  const double ilMean = ilSum / static_cast<double>(cnt);
  const double vcMean = vcSum / static_cast<double>(cnt);
  EXPECT_NEAR(xss(0), ilMean, 0.03 * std::abs(ilMean));
  EXPECT_NEAR(xss(1), vcMean, 0.03 * std::abs(vcMean));
  EXPECT_LT(vcMean, 6.0);  // Vf sag proves the Vf path is live, not vacuous
}

// Remaining error paths: singular blocks, unknown output node, dimension
// mismatches, duty-transfer argument validation.
TEST(StateSpaceExport, RemainingErrorPaths) {
  // Singular algebraic block: node 1 floats on the source alone (states
  // exist at node 2, so the export reaches the invertibility check).
  Circuit f;
  f.addCurrentSource("I1", 1, 2, 1.0);
  f.addResistor("R1", 2, 0, 10.0);
  f.addCapacitor("C1", 2, 0, 100e-6);
  EXPECT_THROW(exportStateSpace(f, {"v:2"}), std::runtime_error);
  // Singular capacitance block: anti-parallel capacitor pair (rank-1
  // incidence, no ground reference to save it).
  Circuit cv;
  cv.addCapacitor("C1", 1, 2, 100e-6);
  cv.addCapacitor("C2", 2, 1, 100e-6);
  cv.addResistor("R1", 1, 0, 10.0);
  EXPECT_THROW(exportStateSpace(cv, {"v:1"}), std::runtime_error);
  // Output node not in the circuit.
  EXPECT_THROW(exportStateSpace(buck(true), {"v:99"}), std::runtime_error);
  // Averaging dimension mismatch (2-state buck vs 1-state RC).
  Circuit rc;
  rc.addCurrentSource("I1", 0, 1, 2.0);
  rc.addResistor("R1", 1, 0, 10.0);
  rc.addCapacitor("C1", 1, 0, 100e-6);
  const StateSpace rcSs = exportStateSpace(rc, {"v:1"});
  const StateSpace on = exportStateSpace(buck(true), {"v:3"});
  const StateSpace off = exportStateSpace(buck(false), {"v:3"});
  EXPECT_THROW(averageStateSpace(on, rcSs, 0.5), std::runtime_error);
  // Duty-transfer argument validation.
  Eigen::VectorXd uSs(2);
  uSs << kVin, 1.0;
  const std::complex<double> s(0.0, 2.0 * kPi * 100.0);
  EXPECT_THROW(evalDutyTransfer(on, off, 0.5, uSs, 5, s), std::runtime_error);
  Eigen::VectorXd badU(1);
  badU << kVin;
  EXPECT_THROW(evalDutyTransfer(on, off, 0.5, badU, 0, s), std::runtime_error);
}
