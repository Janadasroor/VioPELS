#pragma once
#include <complex>
#include <vector>

#include "power_engine/ac.h"
#include "power_engine/measurements.h"

namespace power_engine {
namespace multitone {

/// One sine tone: amp * sin(2*pi*freqHz*t + phaseRad).
struct Tone {
  double freqHz = 0.0;
  double amp = 0.0;
  double phaseRad = 0.0;
};

/// Sum-of-sines drive. Build with Schroeder phases (phase_k = -pi*k^2/N):
/// near-optimal crest for harmonic combs, partial (but still better than
/// aligned) for log-spread tones — measureBuckMultitone therefore adds a
/// numeric peak rescale. For clean Fourier separation choose tone
/// frequencies at distinct PRIME multiples of the measurement-window bin
/// (1/T_window): harmonics and intermodulation products of weakly nonlinear
/// plants then land on non-tone bins and are rejected exactly over integer
/// windows.
class MultitoneSignal {
 public:
  explicit MultitoneSignal(std::vector<Tone> tones);
  /// Schroeder-phase signal from (freq, amp) pairs.
  static MultitoneSignal schroeder(const std::vector<double>& freqs,
                                   const std::vector<double>& amps);
  double value(double t) const;
  double peakBound() const;  ///< sum |amp| (worst-case alignment)
  const std::vector<Tone>& tones() const { return tones_; }

 private:
  std::vector<Tone> tones_;
};

/// Open-loop buck fixture for multitone Bode (matches the stepped-sine AC
/// test plant: Vin=12, fsw=20kHz, L=200uH, C=200uF, R=5ohm).
struct BuckMultitonePlant {
  double vin = 12.0;
  double fsw = 20e3;
  double duty = 0.5;
  double l = 200e-6;
  double c = 200e-6;
  double r = 5.0;
  double dt = 0.5e-6;
};

/// Single-simulation Bode: duty = D + multitone (per-tone AGC from the
/// local plant gain, global cap for headroom, raised-cosine envelope),
/// one FourierMeter per tone over an exact integer window. Returns one
/// AcPoint per tone. Faster than stepped sine by roughly the point count
/// (one shared settle + one window instead of N of each).
std::vector<ac::AcPoint> measureBuckMultitone(const BuckMultitonePlant& plant,
                                              const std::vector<double>& freqs,
                                              double window);

/// Damped ring-down fit of a Trace (impulse/loop characterization):
/// zero-crossing frequency + log-decrement time constant, measured after t0
/// (skips the excitation entry transient). Throws if fewer than 2 clean
/// periods are found.
struct Ringdown {
  double freqHz = 0.0;
  double tau = 0.0;  ///< envelope e^(-t/tau) [s]
};
Ringdown fitRingdown(const measurements::Trace& tr, double t0);

}  // namespace multitone
}  // namespace power_engine
