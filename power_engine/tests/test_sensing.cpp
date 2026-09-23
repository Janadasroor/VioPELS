#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <gtest/gtest.h>

#include "power_engine/sensing.h"

using power_engine::sensing::Encoder;
using power_engine::sensing::SpeedEstimator;

namespace {
constexpr double kPi = std::numbers::pi;
}  // namespace

TEST(Encoder, QuantizationExact) {
  Encoder enc(1024);
  EXPECT_EQ(enc.ppr(), 1024);
  // Exact counts at boundaries (floor semantics).
  EXPECT_EQ(enc.count(0.0), 0);
  EXPECT_EQ(enc.count(2.0 * kPi * 511.5 / 1024.0), 511);
  EXPECT_EQ(enc.count(2.0 * kPi - 1e-12), 1023);
  EXPECT_DOUBLE_EQ(enc.quantized(0.0), 0.0);
  // Wrap + negatives.
  EXPECT_EQ(enc.count(2.0 * kPi), 0);
  EXPECT_EQ(enc.count(4.0 * kPi + 0.1), enc.count(0.1));
  EXPECT_EQ(enc.count(-0.1), enc.count(2.0 * kPi - 0.1));
  // Quantum size exact.
  EXPECT_DOUBLE_EQ(enc.quantized(2.0 * kPi / 1024.0), 2.0 * kPi / 1024.0);
  // Degenerate resolution still well-defined.
  Encoder one(1);
  EXPECT_EQ(one.count(3.3), 0);
  EXPECT_DOUBLE_EQ(one.quantized(3.3), 0.0);
  EXPECT_THROW(Encoder(0), std::runtime_error);
  EXPECT_THROW(Encoder(-4), std::runtime_error);
  EXPECT_THROW(enc.count(std::numeric_limits<double>::quiet_NaN()),
               std::runtime_error);
}

TEST(SpeedEstimator, ConvergesTracksUnwraps) {
  SpeedEstimator est(2e-3);
  // Arms at zero.
  EXPECT_DOUBLE_EQ(est.update(0.0, 0.0), 0.0);
  // Constant 100 rad/s via exact (unquantized) angles: converges within %.
  double th = 0.0;
  for (int k = 1; k <= 2000; ++k) {
    th += 100.0 * 50e-6;
    if (th >= 2.0 * kPi) th -= 2.0 * kPi;
    est.update(th, k * 50e-6);
  }
  EXPECT_NEAR(est.omega(), 100.0, 0.01 * 100.0);
  // Ramp tracking lags within a few filter taus (no overshoot/instability).
  SpeedEstimator ramp(2e-3);
  ramp.update(0.0, 0.0);
  double wRef = 0.0, thR = 0.0, wEst = 0.0;
  for (int k = 1; k <= 4000; ++k) {
    const double t = k * 50e-6;
    wRef = std::min(100.0, 100.0 * t / 100e-3);
    thR += wRef * 50e-6;
    if (thR >= 2.0 * kPi) thR -= 2.0 * kPi;
    wEst = ramp.update(thR, t);
  }
  EXPECT_NEAR(wEst, 100.0, 0.05 * 100.0);
  // Standstill with jitter-free input decays to zero (50 filter taus).
  for (int k = 1; k <= 2000; ++k) ramp.update(thR, 0.2 + k * 50e-6);
  EXPECT_NEAR(ramp.omega(), 0.0, 1e-9);
  // Reversal: sign follows through the unwrap without glitching past tau.
  SpeedEstimator rev(1e-3);
  rev.update(0.1, 0.0);
  for (int k = 1; k <= 2000; ++k) rev.update(0.1 - k * 50e-6 * 20.0, k * 50e-6);
  EXPECT_NEAR(rev.omega(), -20.0, 0.05 * 20.0);
  // Guards.
  EXPECT_THROW(SpeedEstimator(0.0), std::runtime_error);
  EXPECT_THROW(SpeedEstimator(-1.0), std::runtime_error);
  SpeedEstimator g(1e-3);
  g.update(0.0, 1.0);
  EXPECT_THROW(g.update(0.1, 1.0), std::runtime_error);  // non-increasing t
  EXPECT_THROW(g.update(0.1, 0.5), std::runtime_error);
  EXPECT_THROW(g.update(std::numeric_limits<double>::quiet_NaN(), 2.0),
               std::runtime_error);
  g.reset();
  EXPECT_DOUBLE_EQ(g.update(0.3, 10.0), 0.0);  // re-arms cleanly
}

// Quantized input at speed: the estimator still converges (quantization
// ripple averages out through the filter).
TEST(SpeedEstimator, ToleratesQuantization) {
  Encoder enc(1024);
  SpeedEstimator est(2e-3);
  est.update(enc.quantized(0.0), 0.0);
  double th = 0.0;
  for (int k = 1; k <= 4000; ++k) {
    th += 100.0 * 50e-6;
    est.update(enc.quantized(th), k * 50e-6);
  }
  EXPECT_NEAR(est.omega(), 100.0, 0.05 * 100.0);
}
