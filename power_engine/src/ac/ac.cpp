#include "power_engine/ac.h"

#include <cmath>
#include <numbers>
#include <stdexcept>

namespace power_engine {
namespace ac {

namespace {
constexpr double kPi = std::numbers::pi;
}  // namespace

void FourierMeter::begin(double freqHz) {
  if (!(freqHz > 0.0) || !std::isfinite(freqHz)) {
    throw std::runtime_error("AC frequency must be positive finite");
  }
  freqHz_ = freqHz;
  w_ = 2.0 * kPi * freqHz;
  reU_ = imU_ = reY_ = imY_ = 0.0;
  reW_ = imW_ = sumU_ = sumY_ = sumDt_ = 0.0;
  n_ = 0;
}

void FourierMeter::sample(double t0, double t1, double u, double y) {
  const double dt = t1 - t0;
  if (!(dt > 0.0) || !std::isfinite(dt)) throw std::runtime_error("AC sample needs t1 > t0");
  if (!std::isfinite(u) || !std::isfinite(y)) throw std::runtime_error("AC sample must be finite");
  // Exact kernel K = ∫_{t0}^{t1} e^{-jwt} dt = (e^{-jwt0} - e^{-jwt1})/(jw).
  const double c0 = std::cos(w_ * t0), s0 = std::sin(w_ * t0);
  const double c1 = std::cos(w_ * t1), s1 = std::sin(w_ * t1);
  const double reK = (s1 - s0) / w_;
  const double imK = (c1 - c0) / w_;
  reU_ += u * reK;
  imU_ += u * imK;
  reY_ += y * reK;
  imY_ += y * imK;
  reW_ += reK;
  imW_ += imK;
  sumU_ += u * dt;
  sumY_ += y * dt;
  sumDt_ += dt;
  ++n_;
}

std::complex<double> FourierMeter::gain() const {
  if (n_ == 0) throw std::runtime_error("AC meter has no samples");
  // Exact DC rejection: subtract the windowed DC phasor (mean × window).
  // This keeps small AC signals on large DC biases accurate even for
  // slightly non-integer windows.
  const double meanU = sumU_ / sumDt_;
  const double meanY = sumY_ / sumDt_;
  const std::complex<double> x(reU_ - meanU * reW_, imU_ - meanU * imW_);
  const std::complex<double> yy(reY_ - meanY * reW_, imY_ - meanY * imW_);
  if (std::abs(x) == 0.0) throw std::runtime_error("AC input fundamental is zero");
  return yy / x;
}

AcPoint FourierMeter::result() const {
  const std::complex<double> g = gain();
  AcPoint p;
  p.freqHz = freqHz_;
  p.mag = std::abs(g);
  p.magDb = 20.0 * std::log10(p.mag);
  p.phaseDeg = std::arg(g) * 180.0 / kPi;
  return p;
}

}  // namespace ac
}  // namespace power_engine
