#pragma once
#include <cstddef>
#include <functional>
#include <string>
#include <vector>

namespace power_engine {

class Engine;

namespace measurements {

/// A recorded time series (non-uniform steps allowed; all statistics are
/// time-weighted with trapezoidal/exact-kernel quadrature).
struct Trace {
  std::vector<double> t;  ///< strictly increasing times [s]
  std::vector<double> y;  ///< values
  bool empty() const { return t.empty(); }
  std::size_t size() const { return t.size(); }
};

/// One harmonic of a spectrum.
struct Harmonic {
  int order = 0;          ///< 0 = DC, 1 = fundamental, ...
  double freqHz = 0.0;
  double mag = 0.0;       ///< amplitude (half of peak-to-peak for order > 0)
  double phaseDeg = 0.0;  ///< arg in (-180, 180]
};

/// Step a running engine until tEnd, sampling every step.
/// probeKey is a Solution probe ("v:3", "tj:S1"); or pass any sampler
/// (e.g. device current). Stops early if the engine finishes.
Trace recordProbe(Engine& eng, const std::string& probeKey, double tEnd);
Trace recordSignal(Engine& eng, std::function<double()> sample, double tEnd);

/// Crop to [t0, t1] (endpoints interpolated linearly for exact windows).
Trace window(const Trace& tr, double t0, double t1);

/// Time-weighted statistics over the whole trace (need >= 2 samples).
double mean(const Trace& tr);
double rms(const Trace& tr);    ///< total RMS (includes DC)
double rmsAc(const Trace& tr);  ///< RMS after mean removal
double minimum(const Trace& tr);
double maximum(const Trace& tr);
double peakToPeak(const Trace& tr);
/// Ripple = peak-to-peak over [t0, t1] (steady-state switching ripple).
double ripple(const Trace& tr, double t0, double t1);

/// Harmonic spectrum over the trace at fundFreq. The trace should span an
/// integer number of fundamental periods (use window() to cut one).
/// DC (order 0) is the time-weighted mean.
std::vector<Harmonic> spectrum(const Trace& tr, double fundFreq, int maxHarmonic);
/// Total harmonic distortion = sqrt(sum_{k>=2} |Hk|^2) / |H1|.
double thd(const Trace& tr, double fundFreq, int maxHarmonic);

}  // namespace measurements
}  // namespace power_engine
