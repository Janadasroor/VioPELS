#include "power_engine/sensing.h"

#include <cmath>
#include <numbers>
#include <stdexcept>

namespace power_engine {
namespace sensing {

namespace {

constexpr double kTwoPi = 2.0 * std::numbers::pi;

// Wrap any angle to [0, 2pi).
double wrapPi2(double th) {
  double w = std::fmod(th, kTwoPi);
  if (w < 0.0) w += kTwoPi;
  // Clamp fp noise at the wrap point (floor() below must see < ppr counts).
  if (w >= kTwoPi) w = std::nextafter(kTwoPi, 0.0);
  return w;
}

}  // namespace

Encoder::Encoder(int ppr) : ppr_(ppr) {
  if (ppr < 1) throw std::runtime_error("encoder needs ppr >= 1");
}

int Encoder::count(double thetaMech) const {
  if (!std::isfinite(thetaMech)) throw std::runtime_error("encoder theta must be finite");
  return static_cast<int>(std::floor(wrapPi2(thetaMech) / kTwoPi * ppr_));
}

double Encoder::quantized(double thetaMech) const {
  return count(thetaMech) / static_cast<double>(ppr_) * kTwoPi;
}

SpeedEstimator::SpeedEstimator(double tau) : tau_(tau) {
  if (!(tau > 0.0) || !std::isfinite(tau)) {
    throw std::runtime_error("speed estimator needs tau > 0 finite");
  }
}

void SpeedEstimator::reset() {
  prev_ = 0.0;
  tPrev_ = 0.0;
  omega_ = 0.0;
  armed_ = false;
}

double SpeedEstimator::update(double thetaQuant, double t) {
  if (!std::isfinite(thetaQuant) || !std::isfinite(t)) {
    throw std::runtime_error("speed estimator inputs must be finite");
  }
  if (armed_ && !(t > tPrev_)) {
    throw std::runtime_error("speed estimator needs strictly increasing t");
  }
  if (!armed_) {
    prev_ = thetaQuant;
    tPrev_ = t;
    omega_ = 0.0;
    armed_ = true;
    return omega_;
  }
  double d = thetaQuant - prev_;
  // Unwrap to [-pi, pi): correct across revolutions and reversals as long
  // as the true motion per tick is under half a turn (always true here:
  // ticks run at kHz, mechanics at ~100 rad/s).
  if (d > std::numbers::pi) d -= kTwoPi;
  if (d < -std::numbers::pi) d += kTwoPi;
  const double dt = t - tPrev_;
  const double inst = d / dt;
  const double alpha = dt / (tau_ + dt);  // exact single-pole discretization
  omega_ += alpha * (inst - omega_);
  prev_ = thetaQuant;
  tPrev_ = t;
  return omega_;
}

}  // namespace sensing
}  // namespace power_engine
