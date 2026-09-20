#pragma once
#include <cmath>
#include <complex>
#include <vector>

namespace power_engine {
namespace ac {

/// One swept frequency point result.
struct AcPoint {
  double freqHz = 0.0;
  double mag = 0.0;       ///< linear magnitude |Y/X|
  double magDb = 0.0;     ///< 20*log10(mag)
  double phaseDeg = 0.0;  ///< arg(Y/X) in (-180, 180]
};

/// Log-spaced sweep grid: fStart..fEnd inclusive-ish, pointsPerDecade each.
inline std::vector<double> logSweep(double fStart, double fEnd, int pointsPerDecade) {
  std::vector<double> f;
  if (!(fStart > 0.0) || !(fEnd > fStart) || pointsPerDecade < 1) return f;
  const double decades = std::log10(fEnd / fStart);
  const int n = static_cast<int>(decades * pointsPerDecade) + 1;
  for (int i = 0; i < n; ++i) {
    f.push_back(fStart * std::pow(10.0, i / static_cast<double>(pointsPerDecade)));
  }
  return f;
}

/// Single-frequency Fourier correlator over accepted simulation steps.
/// Call sample() once per step with the interval [t0, t1], the input value
/// constant over that interval (exact for ZOH drives and ideal gates), and
/// the output interval average (midpoint: second-order accurate).
/// The reference e^{-jwt} is integrated EXACTLY across each interval
/// (no end-point approximation), and DC operating points are rejected
/// exactly (mean subtraction), so small AC signals on large DC biases,
/// event-split sub-steps, and slightly non-integer windows stay accurate.
/// Measuring the ACTUAL input (not the ideal reference) cancels injection
/// distortion; harmonics are rejected over integer measurement periods.
class FourierMeter {
 public:
  FourierMeter() = default;

  /// Start measuring at freqHz. Call sample() every accepted step.
  void begin(double freqHz);
  /// One step interval [t0, t1]: u = input constant over it, y = output
  /// average over it.
  void sample(double t0, double t1, double u, double y);
  /// Complex gain Y/X over accumulated data (throws if no samples).
  std::complex<double> gain() const;
  /// Packaged Bode point at the configured frequency.
  AcPoint result() const;
  long long samples() const { return n_; }

 private:
  double w_ = 0.0;      // rad/s
  double freqHz_ = 0.0;
  double reU_ = 0.0, imU_ = 0.0;  // Σ u·e^{-jwt}·dt
  double reY_ = 0.0, imY_ = 0.0;  // Σ y·e^{-jwt}·dt
  double reW_ = 0.0, imW_ = 0.0;  // Σ e^{-jwt}·dt (window phasor)
  double sumU_ = 0.0, sumY_ = 0.0, sumDt_ = 0.0;  // DC estimates
  long long n_ = 0;
};

}  // namespace ac
}  // namespace power_engine
