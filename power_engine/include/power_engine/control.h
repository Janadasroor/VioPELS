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
  /// First-order low-pass, DC gain 1, time constant 1/(2*pi*cutoffHz).
  static TransferFunction lowPass(double cutoffHz, double dt);
  /// First-order high-pass, blocks DC, time constant 1/(2*pi*cutoffHz).
  static TransferFunction highPass(double cutoffHz, double dt);
  /// N-point moving average (FIR, linear phase, group delay (N-1)/2 samples).
  static TransferFunction movingAverage(int n);
  void reset() { states_.assign(states_.size(), 0.0); }
  double update(double x);

 private:
  std::vector<double> b_;       // feedforward, b[0] normalized to a[0]=1
  std::vector<double> a_;       // feedback, a[0] == 1
  std::vector<double> states_;  // DF-II-t delays
};

/// Bang-bang current (or voltage) regulator with a hysteresis band:
/// output latches ON when x falls below ref-band/2, OFF when x exceeds
/// ref+band/2. Deterministic, stateful. (Comparator trips the other way:
/// high when x is high; this block drives a switch ON when x is low.)
class HysteresisController {
 public:
  HysteresisController(double ref, double band, bool initialOn = false);

  void setRef(double r);
  void setBand(double b);  // must be finite >= 0
  /// One tick: returns the latched switch state.
  bool update(double x);
  bool state() const { return on_; }
  void reset(bool on = false) { on_ = on; }

 private:
  double ref_ = 0.0;
  double band_ = 0.0;
  bool on_ = false;
};

/// Dwell-gated finite state machine for sequencers and protection logic
/// (startup/shutdown sequencing, fault latching, burst mode): a transition
/// fires only when its guard holds AND the state has been held for dwell
/// seconds. First matching transition (table order) wins. Deterministic.
class StateMachine {
 public:
  enum class GuardDir { Above, Below };
  struct Transition {
    int from = 0;
    int to = 0;
    double threshold = 0.0;
    GuardDir dir = GuardDir::Above;  // Above: x > threshold fires
    double dwell = 0.0;              // min time in `from` before firing [s]
  };
  StateMachine(int initial, std::vector<Transition> transitions);

  /// One tick (dt in seconds): returns the (possibly new) state.
  int update(double x, double dt);
  int state() const { return state_; }
  double timeInState() const { return tState_; }
  void reset(int s);

 private:
  int state_ = 0;
  double tState_ = 0.0;
  std::vector<Transition> transitions_;
};

/// Half-bridge gate driver: complementary hi/lo pair with deadtime from a
/// trailing-edge duty reference. hi turns on `deadtime` after the main
/// rising edge, lo turns on `deadtime` after the falling edge; the outgoing
/// side turns off immediately — never overlapping. If an on-interval is
/// shorter than the deadtime, that side stays off (documented, no throw).
/// Unlike Pwm::complementary (stateless query), this block also exposes
/// exact edge times for Engine::scheduleSwitch.
class HalfBridgeDriver {
 public:
  HalfBridgeDriver(double freq, double duty, double deadtime, double phase = 0.0);

  void setFreq(double f);
  void setDuty(double d);      // clamped to [0,1]
  void setDeadtime(double t);  // finite >= 0
  void setPhase(double p) { phase_ = p; }
  double freq() const { return freq_; }
  double duty() const { return duty_; }
  double period() const { return 1.0 / freq_; }

  /// Gate states at time t.
  bool hi(double t) const;
  bool lo(double t) const;
  /// Next hi-or-lo edge strictly after t (+inf if fully static).
  double nextEdge(double t) const;

 private:
  double phaseInPeriod(double t) const;
  double freq_ = 20e3;
  double duty_ = 0.5;
  double deadtime_ = 0.0;
  double phase_ = 0.0;
};

/// Alpha-beta (stationary two-phase) quantity.
struct AlphaBeta {
  double alpha = 0.0;
  double beta = 0.0;
};

/// Amplitude-invariant Clarke transform: alpha = 2/3*(a-b/2-c/2),
/// beta = (b-c)/sqrt(3). Gain-matched to Svpwm (inputs normalized by
/// Vdc/2) and to machine::park (same 2/3 convention). Throws on
/// non-finite input.
AlphaBeta clarke(double a, double b, double c);

/// Space-vector PWM for 3-phase inverters. Alpha-beta voltage reference
/// normalized to Vdc/2 (|u| <= 2/sqrt(3) is the linear region); outputs the
/// sector + dwell times for one switching period plus the symmetric
/// 7-segment phase states at any intra-period time.
class Svpwm {
 public:
  explicit Svpwm(double freq);

  double period() const { return 1.0 / freq_; }

  struct Sequence {
    int sector = 1;    ///< 1..6
    double t1 = 0.0;   ///< first active-vector dwell [s]
    double t2 = 0.0;   ///< second active-vector dwell [s]
    double t0 = 0.0;   ///< zero-vector dwell [s] (t1+t2+t0 == period)
  };
  /// Sector + dwells for reference (alpha, beta). Overmodulation
  /// (|u| > 2/sqrt(3)) is clamped to the hexagon (documented).
  Sequence sequence(double alpha, double beta) const;

  struct PhaseState {
    bool a = false, b = false, c = false;  ///< upper-switch states
  };
  /// Symmetric 7-segment phase states at intra-period time tp in [0, T).
  PhaseState switches(int sector, double tp, double t1, double t2, double t0) const;

 private:
  double freq_ = 20e3;
};

}  // namespace control
}  // namespace power_engine
