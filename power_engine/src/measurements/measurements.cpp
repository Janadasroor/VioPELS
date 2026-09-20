#include "power_engine/measurements.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <numbers>
#include <stdexcept>

#include "power_engine/engine.h"

namespace power_engine {
namespace measurements {
namespace {

constexpr double kPi = std::numbers::pi;

void requireTrace(const Trace& tr, std::size_t minPts, const char* what) {
  if (tr.t.size() != tr.y.size()) throw std::runtime_error("trace time/value size mismatch");
  if (tr.size() < minPts) throw std::runtime_error(std::string(what) + " needs more samples");
  for (std::size_t i = 1; i < tr.t.size(); ++i) {
    if (!(tr.t[i] > tr.t[i - 1])) throw std::runtime_error("trace times must be strictly increasing");
  }
}

// Time-weighted (trapezoidal) integral of f over trace samples.
double trapInt(const Trace& tr, const std::vector<double>& f, double& tTotal) {
  double sum = 0.0;
  tTotal = 0.0;
  for (std::size_t i = 0; i + 1 < tr.size(); ++i) {
    const double dt = tr.t[i + 1] - tr.t[i];
    sum += 0.5 * (f[i] + f[i + 1]) * dt;
    tTotal += dt;
  }
  return sum;
}

// Exact-kernel correlation of y against e^{-jwt} plus DC rejection.
// Returns the complex fundamental-scale amplitude (caller scales by 2/T).
std::complex<double> correlate(const Trace& tr, double freqHz, const std::vector<double>& y) {
  const double w = 2.0 * kPi * freqHz;
  if (!(w > 0.0) || !std::isfinite(w)) throw std::runtime_error("frequency must be positive");
  double reY = 0.0, imY = 0.0, reW = 0.0, imW = 0.0, sumY = 0.0, sumDt = 0.0;
  for (std::size_t i = 0; i + 1 < tr.size(); ++i) {
    const double t0 = tr.t[i], t1 = tr.t[i + 1], dt = t1 - t0;
    const double ym = 0.5 * (y[i] + y[i + 1]);  // interval average, 2nd order
    const double reK = (std::sin(w * t1) - std::sin(w * t0)) / w;
    const double imK = (std::cos(w * t1) - std::cos(w * t0)) / w;
    reY += ym * reK;
    imY += ym * imK;
    reW += reK;
    imW += imK;
    sumY += ym * dt;
    sumDt += dt;
  }
  const double meanY = sumY / sumDt;
  return {reY - meanY * reW, imY - meanY * imW};
}

}  // namespace

Trace recordProbe(Engine& eng, const std::string& probeKey, double tEnd) {
  return recordSignal(
      eng, [&] { return eng.currentSolution().probes.at(probeKey); }, tEnd);
}

Trace recordSignal(Engine& eng, std::function<double()> sample, double tEnd) {
  if (!(tEnd > eng.time()) || !std::isfinite(tEnd)) {
    throw std::runtime_error("record needs a finite end after current time");
  }
  Trace tr;
  tr.t.push_back(eng.time());
  tr.y.push_back(sample());
  while (eng.time() < tEnd && eng.status() == SimulationStatus::Running) {
    eng.step();
    tr.t.push_back(eng.time());
    const double v = sample();
    if (!std::isfinite(v)) throw std::runtime_error("recorded non-finite sample");
    tr.y.push_back(v);
  }
  return tr;
}

Trace window(const Trace& tr, double t0, double t1) {
  requireTrace(tr, 2, "window");
  if (!(t1 > t0) || !std::isfinite(t0) || !std::isfinite(t1)) {
    throw std::runtime_error("window needs finite t0 < t1");
  }
  auto at = [&](double t) {
    if (t <= tr.t.front()) return tr.y.front();
    if (t >= tr.t.back()) return tr.y.back();
    for (std::size_t i = 0; i + 1 < tr.size(); ++i) {
      if (t >= tr.t[i] && t <= tr.t[i + 1]) {
        const double f = (t - tr.t[i]) / (tr.t[i + 1] - tr.t[i]);
        return tr.y[i] + f * (tr.y[i + 1] - tr.y[i]);
      }
    }
    return tr.y.back();  // unreachable
  };
  // Clamp to trace range; keep all interior samples plus exact boundaries.
  const double a = std::max(t0, tr.t.front());
  const double b = std::min(t1, tr.t.back());
  if (!(b > a)) throw std::runtime_error("window lies outside trace range");
  Trace out;
  out.t.push_back(a);
  out.y.push_back(at(a));
  for (std::size_t i = 0; i < tr.size(); ++i) {
    if (tr.t[i] > a && tr.t[i] < b) {
      out.t.push_back(tr.t[i]);
      out.y.push_back(tr.y[i]);
    }
  }
  out.t.push_back(b);
  out.y.push_back(at(b));
  return out;
}

double mean(const Trace& tr) {
  requireTrace(tr, 2, "mean");
  double total = 0.0;
  return trapInt(tr, tr.y, total) / total;
}

double rms(const Trace& tr) {
  requireTrace(tr, 2, "rms");
  std::vector<double> sq(tr.size());
  for (std::size_t i = 0; i < tr.size(); ++i) sq[i] = tr.y[i] * tr.y[i];
  double total = 0.0;
  return std::sqrt(trapInt(tr, sq, total) / total);
}

double rmsAc(const Trace& tr) {
  const double m = mean(tr);
  Trace c = tr;
  for (auto& v : c.y) v -= m;
  return rms(c);
}

double minimum(const Trace& tr) {
  requireTrace(tr, 1, "minimum");
  double m = tr.y[0];
  for (double v : tr.y) m = std::min(m, v);
  return m;
}

double maximum(const Trace& tr) {
  requireTrace(tr, 1, "maximum");
  double m = tr.y[0];
  for (double v : tr.y) m = std::max(m, v);
  return m;
}

double peakToPeak(const Trace& tr) { return maximum(tr) - minimum(tr); }

double ripple(const Trace& tr, double t0, double t1) {
  return peakToPeak(window(tr, t0, t1));
}

std::vector<Harmonic> spectrum(const Trace& tr, double fundFreq, int maxHarmonic) {
  requireTrace(tr, 3, "spectrum");
  if (!(fundFreq > 0.0) || !std::isfinite(fundFreq)) {
    throw std::runtime_error("spectrum needs positive fundamental");
  }
  if (maxHarmonic < 1) throw std::runtime_error("spectrum needs maxHarmonic >= 1");
  double total = 0.0;
  trapInt(tr, tr.y, total);  // validates span; total = window length
  std::vector<Harmonic> out;
  Harmonic dc;
  dc.order = 0;
  dc.freqHz = 0.0;
  dc.mag = mean(tr);
  dc.phaseDeg = 0.0;
  out.push_back(dc);
  for (int k = 1; k <= maxHarmonic; ++k) {
    const std::complex<double> c = correlate(tr, k * fundFreq, tr.y);
    Harmonic h;
    h.order = k;
    h.freqHz = k * fundFreq;
    h.mag = 2.0 * std::abs(c) / total;  // amplitude from two-sided correlation
    h.phaseDeg = std::arg(c) * 180.0 / kPi;
    out.push_back(h);
  }
  return out;
}

double thd(const Trace& tr, double fundFreq, int maxHarmonic) {
  const auto spec = spectrum(tr, fundFreq, std::max(2, maxHarmonic));
  const double h1 = spec[1].mag;
  if (!(h1 > 0.0)) throw std::runtime_error("THD needs nonzero fundamental");
  double sum = 0.0;
  for (std::size_t k = 2; k < spec.size(); ++k) sum += spec[k].mag * spec[k].mag;
  return std::sqrt(sum) / h1;
}

}  // namespace measurements
}  // namespace power_engine
