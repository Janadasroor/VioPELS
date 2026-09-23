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

using power_engine::sensing::SensorlessObserver;
using power_engine::sensing::SensorlessParams;

TEST(SensorlessObserver, RejectsBadParams) {
  SensorlessParams ok;
  ok.rs = 0.5;
  ok.l = 2e-3;
  ok.lambdaPm = 0.05;
  EXPECT_NO_THROW(SensorlessObserver{ok});
  SensorlessParams bad = ok;
  bad.l = 0.0;
  EXPECT_THROW(SensorlessObserver{bad}, std::runtime_error);
  bad = ok;
  bad.lambdaPm = -0.01;
  EXPECT_THROW(SensorlessObserver{bad}, std::runtime_error);
  bad = ok;
  bad.rs = -1.0;
  EXPECT_THROW(SensorlessObserver{bad}, std::runtime_error);
  bad = ok;
  bad.fluxTau = 0.0;
  EXPECT_THROW(SensorlessObserver{bad}, std::runtime_error);
  bad = ok;
  bad.pllBandwidthHz = 0.0;
  EXPECT_THROW(SensorlessObserver{bad}, std::runtime_error);
  bad = ok;
  bad.pllDamping = -0.5;
  EXPECT_THROW(SensorlessObserver{bad}, std::runtime_error);
  SensorlessObserver o(ok);
  EXPECT_THROW(o.update(1.0, 0.0, 0.0, 0.0, 0.0), std::runtime_error);
  EXPECT_THROW(o.update(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.0, 1e-6),
               std::runtime_error);
}

// Synthetic back-EMF at known angle, zero current: v = dλr/dt with
// λr = -λm*(cos θe, sin θe) (the EMF sine convention, see header). The
// observer must lock θ̂e -> θe (pins the +π fold) with ω̂e -> ωe.
TEST(SensorlessObserver, TracksSyntheticEmf) {
  SensorlessParams p;
  p.rs = 0.5;
  p.l = 2e-3;
  p.lambdaPm = 0.05;
  SensorlessObserver o(p);
  constexpr double kWe = 200.0, kDt = 50e-6, kA = kWe * 0.05;
  double errMean = 0.0, omMean = 0.0, fxMean = 0.0, n = 0.0;
  for (int k = 0; k < 6000; ++k) {
    const double t = k * kDt;
    o.update(kA * std::sin(kWe * t), -kA * std::cos(kWe * t), 0.0, 0.0, kDt);
    if (t >= 0.25) {
      double e = std::fmod(o.thetaE() - kWe * t + kPi, 2.0 * kPi);
      if (e < 0.0) e += 2.0 * kPi;
      e -= kPi;
      errMean += std::abs(e);
      omMean += o.omegaE();
      fxMean += o.fluxMag();
      n += 1.0;
    }
  }
  EXPECT_LT(errMean / n, 0.035);  // < 2 deg electrical
  EXPECT_NEAR(omMean / n, kWe, 0.01 * kWe);
  EXPECT_NEAR(fxMean / n, 0.05, 0.02 * 0.05);
}

// Standstill (no EMF): nothing to observe — must stay bounded and report
// flux ~ 0 (the caller's I-f cue), never NaN.
TEST(SensorlessObserver, StandstillBlindButBounded) {
  SensorlessParams p;
  p.rs = 0.5;
  p.l = 2e-3;
  p.lambdaPm = 0.05;
  SensorlessObserver o(p);
  for (int k = 0; k < 2000; ++k) o.update(0.0, 0.0, 0.0, 0.0, 50e-6);
  EXPECT_TRUE(std::isfinite(o.thetaE()));
  EXPECT_TRUE(std::isfinite(o.omegaE()));
  EXPECT_DOUBLE_EQ(o.fluxMag(), 0.0);
  // PLL parks on the arbitrary folded angle with ~zero speed (settled
  // transient, not information): still blind, but bounded.
  EXPECT_NEAR(o.omegaE(), 0.0, 1e-6);
  o.reset();
  EXPECT_DOUBLE_EQ(o.thetaE(), 0.0);
}
