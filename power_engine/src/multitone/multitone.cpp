#include "power_engine/multitone.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

#include "power_engine/engine.h"

namespace power_engine {
namespace multitone {
namespace {

constexpr double kPi = std::numbers::pi;

}  // namespace

MultitoneSignal::MultitoneSignal(std::vector<Tone> tones) : tones_(std::move(tones)) {
  if (tones_.empty()) throw std::runtime_error("MultitoneSignal needs >= 1 tone");
  for (const auto& t : tones_) {
    if (!(t.freqHz > 0.0) || !std::isfinite(t.freqHz)) {
      throw std::runtime_error("MultitoneSignal: tone freq must be finite > 0");
    }
    if (!std::isfinite(t.amp) || !std::isfinite(t.phaseRad)) {
      throw std::runtime_error("MultitoneSignal: tone amp/phase must be finite");
    }
  }
}

MultitoneSignal MultitoneSignal::schroeder(const std::vector<double>& freqs,
                                           const std::vector<double>& amps) {
  if (freqs.empty() || freqs.size() != amps.size()) {
    throw std::runtime_error("MultitoneSignal::schroeder needs matching nonempty freqs/amps");
  }
  const double n = static_cast<double>(freqs.size());
  std::vector<Tone> tones;
  for (std::size_t k = 0; k < freqs.size(); ++k) {
    tones.push_back({freqs[k], amps[k], -kPi * (k + 1) * (k + 1) / n});
  }
  return MultitoneSignal(std::move(tones));
}

double MultitoneSignal::value(double t) const {
  double v = 0.0;
  for (const auto& tone : tones_) {
    v += tone.amp * std::sin(2.0 * kPi * tone.freqHz * t + tone.phaseRad);
  }
  return v;
}

double MultitoneSignal::peakBound() const {
  double s = 0.0;
  for (const auto& tone : tones_) s += std::abs(tone.amp);
  return s;
}

std::vector<ac::AcPoint> measureBuckMultitone(const BuckMultitonePlant& plant,
                                              const std::vector<double>& freqs,
                                              double window) {
  if (freqs.empty()) throw std::runtime_error("measureBuckMultitone: empty sweep");
  if (!(window > 0.0) || !std::isfinite(window)) {
    throw std::runtime_error("measureBuckMultitone: window must be finite > 0");
  }
  const double tSw = 1.0 / plant.fsw;
  for (double f : freqs) {
    if (!(f > 0.0) || f >= plant.fsw / 2.0 || !std::isfinite(f)) {
      throw std::runtime_error("measureBuckMultitone: tone must be in (0, fsw/2)");
    }
  }
  // Plant gain for per-tone AGC (ideal second-order, matches test_ac).
  auto gvdAt = [&](double f) {
    const std::complex<double> s(0.0, 2.0 * kPi * f);
    return plant.vin / (1.0 + s * plant.l / plant.r + s * s * plant.l * plant.c);
  };
  // Per-tone duty amplitude (small-signal, resonance-aware), then a global
  // rescale capping the TOTAL duty swing (headroom, no saturation).
  std::vector<double> amps;
  double sumAmp = 0.0;
  for (double f : freqs) {
    const double a = std::min(0.05, 0.3 / std::abs(gvdAt(f)));
    amps.push_back(a);
    sumAmp += a;
  }
  const double scale = std::min(1.0, 0.2 / sumAmp);
  for (double& a : amps) a *= scale;
  MultitoneSignal sig = MultitoneSignal::schroeder(freqs, amps);
  // Numeric crest guard (Schroeder is only near-optimal for harmonic combs;
  // log-spread tones peak near the bound): scan one window, rescale so the
  // peak duty swing stays within +/-0.25 of D.
  {
    double peak = 0.0;
    const long long n = static_cast<long long>(window / 1e-6);
    for (long long i = 0; i <= n; ++i) {
      peak = std::max(peak, std::abs(sig.value(i * 1e-6)));
    }
    if (peak > 0.25 && peak > 0.0) {
      const double s = 0.25 / peak;
      std::vector<Tone> ts = sig.tones();
      for (auto& t : ts) t.amp *= s;
      sig = MultitoneSignal(std::move(ts));
    }
  }

  Engine eng;
  eng.setTimeStep(plant.dt);
  eng.circuit().addVoltageSource("Vin", 1, 0, plant.vin);
  eng.circuit().addSwitch("S1", 1, 2, 5e-3, 1e6, true);
  eng.circuit().addDiode("D1", 0, 2, 0.0, 10e-3, 1e6);
  eng.circuit().addInductor("L1", 2, 3, plant.l, plant.vin * plant.duty / plant.r);
  eng.circuit().addCapacitor("C1", 3, 0, plant.c, plant.vin * plant.duty);
  eng.circuit().addResistor("Rload", 3, 0, plant.r);
  constexpr double kRamp = 3e-3;
  auto env = [&](double t) {
    if (t >= kRamp) return 1.0;
    const double x = t / kRamp;
    return 0.5 * (1.0 - std::cos(kPi * x));
  };
  // Edges from the enveloped multitone, sampled per switching cycle.
  const double fMin = *std::min_element(freqs.begin(), freqs.end());
  const double tSettle = std::max(9e-3, 2.0 / fMin);
  const double tEnd = tSettle + window;
  for (double tk = 0.0; tk < tEnd; tk += tSw) {
    const double d = plant.duty + env(tk) * sig.value(tk);
    if (!(d > 0.0) || !(d < 1.0)) {
      throw std::runtime_error("measureBuckMultitone: duty saturated (rescale)");
    }
    eng.scheduleSwitch("S1", true, tk);
    eng.scheduleSwitch("S1", false, tk + d * tSw);
  }
  eng.setStopTime(tEnd + tSw);
  eng.start();
  while (eng.time() < tSettle) eng.step();
  std::vector<ac::FourierMeter> meters(freqs.size());
  for (std::size_t i = 0; i < freqs.size(); ++i) meters[i].begin(freqs[i]);
  double tPrev = eng.time();
  double yPrev = eng.currentSolution().probes.at("v:3");
  while (eng.time() < tEnd) {
    eng.step();
    const double t = eng.time();
    const double u = eng.circuit().switchClosed("S1") ? 1.0 : 0.0;
    const double y = eng.currentSolution().probes.at("v:3");
    for (auto& m : meters) m.sample(tPrev, t, u, 0.5 * (yPrev + y));
    tPrev = t;
    yPrev = y;
  }
  eng.stop();
  std::vector<ac::AcPoint> points;
  for (auto& m : meters) {
    if (m.samples() == 0) throw std::runtime_error("measureBuckMultitone: empty meter");
    points.push_back(m.result());
  }
  return points;
}

Ringdown fitRingdown(const measurements::Trace& tr, double t0) {
  if (tr.size() < 16) throw std::runtime_error("fitRingdown: trace too short");
  // Tail mean (settled DC) from the last 10%.
  const std::size_t tail0 = tr.size() - tr.size() / 10;
  double tail = 0.0;
  for (std::size_t i = tail0; i < tr.size(); ++i) tail += tr.y[i];
  tail /= static_cast<double>(tr.size() - tail0);
  // Rising zero crossings after t0 (linear interpolation).
  std::vector<double> crossings;
  for (std::size_t i = 1; i < tr.size(); ++i) {
    if (tr.t[i] < t0) continue;
    const double a = tr.y[i - 1] - tail, b = tr.y[i] - tail;
    if (a < 0.0 && b >= 0.0 && (tr.t[i] - tr.t[i - 1]) > 0.0) {
      crossings.push_back(tr.t[i - 1] + (tr.t[i] - tr.t[i - 1]) * (-a / (b - a)));
    }
  }
  if (crossings.size() < 3) throw std::runtime_error("fitRingdown: fewer than 2 periods");
  // Frequency from mean period (drop the first: entry transient).
  double period = 0.0;
  for (std::size_t i = 2; i < crossings.size(); ++i) {
    period += crossings[i] - crossings[i - 1];
  }
  period /= static_cast<double>(crossings.size() - 2);
  // Log-decrement: peak |amplitude| per full period, linear fit of ln.
  std::vector<double> ts, ls;
  for (std::size_t i = 1; i + 1 < crossings.size(); ++i) {
    double peak = 0.0;
    for (std::size_t j = 0; j < tr.size(); ++j) {
      if (tr.t[j] >= crossings[i] && tr.t[j] < crossings[i + 1]) {
        peak = std::max(peak, std::abs(tr.y[j] - tail));
      }
    }
    if (peak > 0.0) {
      ts.push_back(0.5 * (crossings[i] + crossings[i + 1]));
      ls.push_back(std::log(peak));
    }
  }
  if (ts.size() < 2) throw std::runtime_error("fitRingdown: no decay data");
  double st = 0.0, sl = 0.0;
  for (std::size_t i = 0; i < ts.size(); ++i) {
    st += ts[i];
    sl += ls[i];
  }
  st /= ts.size();
  sl /= ts.size();
  double num = 0.0, den = 0.0;
  for (std::size_t i = 0; i < ts.size(); ++i) {
    num += (ts[i] - st) * (ls[i] - sl);
    den += (ts[i] - st) * (ts[i] - st);
  }
  if (!(den > 0.0) || !(num < 0.0)) {
    throw std::runtime_error("fitRingdown: amplitude not decaying");
  }
  return {1.0 / period, -1.0 / (num / den)};
}

}  // namespace multitone
}  // namespace power_engine
