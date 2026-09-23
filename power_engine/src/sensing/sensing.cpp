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

// Wrap any angle difference to [-pi, pi).
double wrapPi(double d) {
  double w = std::fmod(d + std::numbers::pi, kTwoPi);
  if (w < 0.0) w += kTwoPi;
  return w - std::numbers::pi;
}

SensorlessObserver::SensorlessObserver(const SensorlessParams& p)
    : rs_(p.rs), l_(p.l), lam_(p.lambdaPm) {
  if (!std::isfinite(p.rs) || p.rs < 0.0)
    throw std::runtime_error("sensorless observer needs rs finite >= 0");
  if (!(p.l > 0.0) || !std::isfinite(p.l))
    throw std::runtime_error("sensorless observer needs l finite > 0");
  if (!(p.lambdaPm > 0.0) || !std::isfinite(p.lambdaPm))
    throw std::runtime_error("sensorless observer needs lambdaPm finite > 0");
  if (!(p.fluxTau > 0.0) || !std::isfinite(p.fluxTau))
    throw std::runtime_error("sensorless observer needs fluxTau finite > 0");
  if (!(p.pllBandwidthHz > 0.0) || !std::isfinite(p.pllBandwidthHz))
    throw std::runtime_error("sensorless observer needs pllBandwidthHz finite > 0");
  if (!(p.pllDamping > 0.0) || !std::isfinite(p.pllDamping))
    throw std::runtime_error("sensorless observer needs pllDamping finite > 0");
  gamma_ = 1.0 / (2.0 * p.lambdaPm * p.lambdaPm * p.fluxTau);
  const double wn = kTwoPi * p.pllBandwidthHz;
  pllKp_ = 2.0 * p.pllDamping * wn;
  pllKi_ = wn * wn;
}

void SensorlessObserver::reset() {
  lamA_ = 0.0;
  lamB_ = 0.0;
  thHat_ = 0.0;
  omHat_ = 0.0;
  xi_ = 0.0;
  fluxMag_ = 0.0;
}

void SensorlessObserver::update(double vAlpha, double vBeta, double iAlpha, double iBeta,
                                double dt) {
  if (!std::isfinite(vAlpha) || !std::isfinite(vBeta) || !std::isfinite(iAlpha) ||
      !std::isfinite(iBeta) || !std::isfinite(dt))
    throw std::runtime_error("sensorless observer inputs must be finite");
  if (!(dt > 0.0)) throw std::runtime_error("sensorless observer needs dt > 0");
  // Voltage-model flux integration with magnitude feedback toward λm.
  double lrA = lamA_ - l_ * iAlpha;
  double lrB = lamB_ - l_ * iBeta;
  const double corr = gamma_ * (lam_ * lam_ - (lrA * lrA + lrB * lrB));
  lamA_ += (vAlpha - rs_ * iAlpha + corr * lrA) * dt;
  lamB_ += (vBeta - rs_ * iBeta + corr * lrB) * dt;
  // Rotor-flux angle (+π fold: EMF sine convention puts integrated flux
  // opposite the park reference) drives a 2nd-order PLL. At standstill
  // lr ~ 0 and atan2 is arbitrary — caller must start open-loop (I-f).
  lrA = lamA_ - l_ * iAlpha;
  lrB = lamB_ - l_ * iBeta;
  fluxMag_ = std::hypot(lrA, lrB);
  const double err = wrapPi(std::atan2(lrB, lrA) + std::numbers::pi - thHat_);
  xi_ += pllKi_ * err * dt;
  omHat_ = pllKp_ * err + xi_;
  thHat_ += omHat_ * dt;
}

}  // namespace sensing
}  // namespace power_engine
