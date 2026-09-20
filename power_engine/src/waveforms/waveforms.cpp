#include "power_engine/waveforms.h"

#include <cmath>
#include <numbers>
#include <stdexcept>

namespace power_engine {
namespace waveforms {
namespace {
constexpr double kPi = std::numbers::pi;
}  // namespace

Waveform Waveform::constant(double v) {
  if (!std::isfinite(v)) throw std::runtime_error("waveform constant must be finite");
  Waveform w(Kind::Constant);
  w.p_[0] = v;
  return w;
}

Waveform Waveform::sin(double freq, double amplitude, double dc, double phase) {
  if (!(freq > 0.0) || !std::isfinite(freq)) throw std::runtime_error("sine freq must be positive");
  if (!std::isfinite(amplitude) || !std::isfinite(dc) || !std::isfinite(phase)) {
    throw std::runtime_error("sine amplitude/dc/phase must be finite");
  }
  Waveform w(Kind::Sin);
  w.p_[0] = freq;
  w.p_[1] = amplitude;
  w.p_[2] = dc;
  w.p_[3] = phase;
  return w;
}

Waveform Waveform::pulse(double t0, double width, double period, double amplitude, double dc,
                         long long count) {
  if (!std::isfinite(t0) || !(width > 0.0) || !std::isfinite(width)) {
    throw std::runtime_error("pulse needs finite t0 and positive width");
  }
  if (!(period > width) || !std::isfinite(period)) {
    throw std::runtime_error("pulse period must exceed width");
  }
  if (!std::isfinite(amplitude) || !std::isfinite(dc)) {
    throw std::runtime_error("pulse amplitude/dc must be finite");
  }
  if (count < -1 || count == 0) throw std::runtime_error("pulse count must be -1 or positive");
  Waveform w(Kind::Pulse);
  w.p_[0] = t0;
  w.p_[1] = width;
  w.p_[2] = period;
  w.p_[3] = amplitude;
  w.p_[4] = dc;
  w.count_ = count;
  return w;
}

Waveform Waveform::pwl(std::vector<double> times, std::vector<double> values) {
  if (times.size() != values.size() || times.empty()) {
    throw std::runtime_error("pwl needs matching nonempty time/value lists");
  }
  for (std::size_t i = 0; i < times.size(); ++i) {
    if (!std::isfinite(times[i]) || !std::isfinite(values[i])) {
      throw std::runtime_error("pwl points must be finite");
    }
    if (i > 0 && !(times[i] > times[i - 1])) {
      throw std::runtime_error("pwl times must be strictly increasing");
    }
  }
  Waveform w(Kind::Pwl);
  w.times_ = std::move(times);
  w.values_ = std::move(values);
  return w;
}

double Waveform::value(double t) const {
  switch (kind_) {
    case Kind::Constant:
      return p_[0];
    case Kind::Sin:
      return p_[2] + p_[1] * std::sin(2.0 * kPi * p_[0] * t + p_[3]);
    case Kind::Pulse: {
      if (t < p_[0]) return p_[4];
      const double k = (t - p_[0]) / p_[2];
      if (count_ >= 0 && k >= static_cast<double>(count_)) return p_[4];
      const long long n = static_cast<long long>(k);  // k >= 0 here
      return (t - p_[0] - n * p_[2] < p_[1]) ? p_[4] + p_[3] : p_[4];
    }
    case Kind::Pwl: {
      if (t <= times_.front()) return values_.front();
      if (t >= times_.back()) return values_.back();
      for (std::size_t i = 0; i + 1 < times_.size(); ++i) {
        if (t >= times_[i] && t <= times_[i + 1]) {
          const double f = (t - times_[i]) / (times_[i + 1] - times_[i]);
          return values_[i] + f * (values_[i + 1] - values_[i]);
        }
      }
      return values_.back();  // unreachable
    }
  }
  return p_[0];  // unreachable
}

}  // namespace waveforms
}  // namespace power_engine
