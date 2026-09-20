#include <cmath>
#include <complex>
#include <numbers>
#include <stdexcept>
#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include "power_engine/circuit.h"
#include "power_engine/statespace.h"

using power_engine::Circuit;
using power_engine::DeviceType;
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
