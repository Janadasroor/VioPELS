#pragma once
#include <vector>

namespace power_engine {
namespace control {

/// Carrier type for PWM generation.
enum class Carrier {
  TrailingEdge,  ///< sawtooth: on-interval starts at period start
  Symmetric,     ///< triangle: on-interval centered in the period
};

/// Trailing-edge / symmetric PWM with exact edge times.
/// Stateless w.r.t. simulation: output(t) and nextEdge(t) are pure functions
/// of time, so edges can be pre-scheduled exactly via Engine::scheduleSwitch.
class Pwm {
 public:
  Pwm(double freq, double duty, double phase = 0.0,
      Carrier carrier = Carrier::TrailingEdge);

  void setFreq(double f);
  void setDuty(double d);  // clamped to [0,1]
  void setPhase(double p) { phase_ = p; }
  double freq() const { return freq_; }
  double duty() const { return duty_; }
  double period() const { return 1.0 / freq_; }

  /// Modulator output at time t (true = on).
  bool output(double t) const;
  /// Next edge strictly after t (returns +inf if duty is 0 or 1).
  double nextEdge(double t) const;

  /// Complementary pair with deadtime td: hi turns on td after the main
  /// rising edge, lo turns on td after the main falling edge; the outgoing
  /// side turns off immediately. Never overlapping; both off for td after
  /// every edge. Main on-time shrinks by td (documented).
  struct CompOut {
    bool hi = false;
    bool lo = false;
  };
  CompOut complementary(double t, double deadtime) const;

 private:
  // Phase within [0, period) and time since the last main rising edge.
  double phaseInPeriod(double t) const;
  double freq_ = 20e3;
  double duty_ = 0.5;
  double phase_ = 0.0;
  Carrier carrier_ = Carrier::TrailingEdge;
};

/// PI controller with output clamping + conditional-integration anti-windup:
/// the integrator freezes when saturated and the error would drive it deeper.
/// Backward-Euler integration; deterministic.
class PiController {
 public:
  PiController(double kp, double ki, double outMin, double outMax);

  void reset() { integ_ = 0.0; }
  /// Bumpless-transfer init: preset the integrator state (e.g. d0/ki when
  /// starting from a known steady-state duty) so the first update does not
  /// rail. Throws on non-finite input.
  void setIntegrator(double v);
  void setGains(double kp, double ki);
  void setLimits(double lo, double hi);
  /// One control tick: err in engineering units, dt in seconds.
  double update(double err, double dt);
  double integrator() const { return integ_; }

 private:
  double kp_ = 0.0;
  double ki_ = 0.0;
  double lo_ = 0.0;
  double hi_ = 1.0;
  double integ_ = 0.0;
};

/// Comparator with hysteresis: trips high when x > high, low when x < low.
class Comparator {
 public:
  Comparator(double threshold, double hysteresis = 0.0);
  /// Stateless query (assumes starting-low state).
  bool output(double x) const { return x > threshold_ + hysteresis_ * 0.5; }
  /// Stateful update; returns new output state.
  bool update(double x);
  bool state() const { return state_; }
  void reset(bool s = false) { state_ = s; }

 private:
  double threshold_ = 0.0;
  double hysteresis_ = 0.0;
  bool state_ = false;
};

/// Discrete SISO transfer function, Direct-Form-II-transposed evaluation.
/// Build either from digital coefficients or from a continuous num/den pair
/// discretized with Tustin (bilinear), order <= 8.
class TransferFunction {
 public:
  TransferFunction() = default;
  static TransferFunction fromDigital(std::vector<double> b, std::vector<double> a);
  static TransferFunction fromContinuous(const std::vector<double>& num,
                                         const std::vector<double>& den, double dt);
  void reset() { states_.assign(states_.size(), 0.0); }
  double update(double x);

 private:
  std::vector<double> b_;       // feedforward, b[0] normalized to a[0]=1
  std::vector<double> a_;       // feedback, a[0] == 1
  std::vector<double> states_;  // DF-II-t delays
};

}  // namespace control
}  // namespace power_engine
