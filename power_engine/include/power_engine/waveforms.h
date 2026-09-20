#pragma once
#include <vector>

namespace power_engine {
namespace waveforms {

/// Deterministic drive waveforms for tests, demos, and AC injection.
/// A waveform maps time -> value; the engine samples it once per step
/// (ideal sources take exactly one value per solve — there is no
/// intra-step variation to model, so SIN/PULSE/PWL differ only in sample
/// phase, and measured *ratios* are invariant to it; see ARCHITECTURE).
class Waveform {
 public:
  /// Constant value.
  static Waveform constant(double v);
  /// dc + amplitude * sin(2*pi*freq*t + phase).
  static Waveform sin(double freq, double amplitude, double dc = 0.0, double phase = 0.0);
  /// Pulse train: dc normally, dc+amplitude on [t0 + k*period, t0 + k*period + width).
  /// count < 0 means infinite train.
  static Waveform pulse(double t0, double width, double period, double amplitude,
                        double dc = 0.0, long long count = -1);
  /// Piecewise-linear through (time, value) breakpoints (need >= 1 point);
  /// holds first/last value outside the range.
  static Waveform pwl(std::vector<double> times, std::vector<double> values);

  double value(double t) const;

 private:
  enum class Kind { Constant, Sin, Pulse, Pwl };
  Waveform(Kind kind) : kind_(kind) {}

  Kind kind_ = Kind::Constant;
  // Sin: p[0]=freq, p[1]=amplitude, p[2]=dc, p[3]=phase.
  // Pulse: p[0]=t0, p[1]=width, p[2]=period, p[3]=amplitude, p[4]=dc, count_.
  // Pwl: times_/values_ breakpoint lists.
  double p_[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
  long long count_ = -1;
  std::vector<double> times_;
  std::vector<double> values_;
};

}  // namespace waveforms
}  // namespace power_engine
