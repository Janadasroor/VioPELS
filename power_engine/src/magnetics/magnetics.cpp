#include "power_engine/magnetics.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace power_engine {
namespace magnetics {
namespace {

constexpr double kPi = std::numbers::pi;

}  // namespace

HysteresisCore::HysteresisCore(HystereticMaterial mat) : mat_(mat) {
  if (!(mat_.bs > 0.0) || !std::isfinite(mat_.bs)) {
    throw std::runtime_error("hysteresis Bs must be positive finite");
  }
  if (!(mat_.a > 0.0) || !std::isfinite(mat_.a)) {
    throw std::runtime_error("hysteresis shape field a must be positive finite");
  }
  if (!(mat_.hc >= 0.0) || !std::isfinite(mat_.hc)) {
    throw std::runtime_error("hysteresis Hc must be finite >= 0");
  }
}

void HysteresisCore::reset() {
  s_ = 0.0;
  hPrev_ = 0.0;
  bPrev_ = 0.0;
  loss_ = 0.0;
}

double HysteresisCore::anhysteretic(double h) const {
  return mat_.bs * std::tanh(h / mat_.a);
}

double HysteresisCore::ascending(double h) const {
  return mat_.bs * std::tanh((h - mat_.hc) / mat_.a);
}

double HysteresisCore::descending(double h) const {
  return mat_.bs * std::tanh((h + mat_.hc) / mat_.a);
}

double HysteresisCore::update(double h) {
  if (!std::isfinite(h)) throw std::runtime_error("hysteresis field must be finite");
  // Saturating offset memory: track +H into [-Hc, +Hc]. Major limbs sit at
  // the clamps (bare branches); reversals walk s back (lens-shaped minors
  // that close exactly); pushing into a clamp wipes outer memory.
  s_ = std::clamp(s_ + (h - hPrev_), -mat_.hc, mat_.hc);
  const double b = mat_.bs * std::tanh((h - s_) / mat_.a);
  // Hysteresis energy density increment (trapezoid on the H-B plane).
  loss_ += 0.5 * (h + hPrev_) * (b - bPrev_);
  hPrev_ = h;
  bPrev_ = b;
  return b;
}

double eddyLossDensity(double rho, double thickness, double freqHz, double bPeak) {
  if (!(rho > 0.0) || !std::isfinite(rho)) {
    throw std::runtime_error("eddy resistivity must be positive finite");
  }
  if (!(thickness > 0.0) || !std::isfinite(thickness)) {
    throw std::runtime_error("eddy thickness must be positive finite");
  }
  if (!(freqHz >= 0.0) || !std::isfinite(freqHz)) {
    throw std::runtime_error("eddy frequency must be finite >= 0");
  }
  if (!(bPeak >= 0.0) || !std::isfinite(bPeak)) {
    throw std::runtime_error("eddy Bpk must be finite >= 0");
  }
  return kPi * kPi / (6.0 * rho) * thickness * thickness * freqHz * freqHz * bPeak * bPeak;
}

}  // namespace magnetics
}  // namespace power_engine
