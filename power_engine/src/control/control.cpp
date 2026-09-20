#include "power_engine/control.h"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace power_engine {
namespace control {
namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

void requireFinite(double v, const char* what) {
  if (!std::isfinite(v)) throw std::runtime_error(std::string(what) + " must be finite");
}

}  // namespace

// ---------------- Pwm ----------------

Pwm::Pwm(double freq, double duty, double phase, Carrier carrier)
    : phase_(phase), carrier_(carrier) {
  setFreq(freq);
  setDuty(duty);
}

void Pwm::setFreq(double f) {
  if (!(f > 0.0) || !std::isfinite(f)) throw std::runtime_error("pwm freq must be positive");
  freq_ = f;
}

void Pwm::setDuty(double d) {
  requireFinite(d, "pwm duty");
  duty_ = d < 0.0 ? 0.0 : (d > 1.0 ? 1.0 : d);
}

double Pwm::phaseInPeriod(double t) const {
  const double T = period();
  double ph = std::fmod(t - phase_, T);
  if (ph < 0.0) ph += T;
  // Clamp fp noise at the wrap point.
  if (ph >= T) ph -= T;
  return ph;
}

bool Pwm::output(double t) const {
  if (duty_ <= 0.0) return false;
  if (duty_ >= 1.0) return true;
  const double T = period();
  const double ton = duty_ * T;
  const double ph = phaseInPeriod(t);
  if (carrier_ == Carrier::TrailingEdge) {
    return ph < ton;
  }
  const double start = (T - ton) * 0.5;
  return ph >= start && ph < start + ton;
}

double Pwm::nextEdge(double t) const {
  if (duty_ <= 0.0 || duty_ >= 1.0) return kInf;
  const double T = period();
  const double ton = duty_ * T;
  const double eps = 1e-9 * T;
  // Period index of t.
  const double k = std::floor((t - phase_) / T);
  double rise, fall;
  if (carrier_ == Carrier::TrailingEdge) {
    rise = (k + 1.0) * T + phase_;
    fall = k * T + phase_ + ton;
    if (rise <= t + eps) rise += T;
    if (fall <= t + eps) fall += T;
  } else {
    const double start = (T - ton) * 0.5;
    rise = k * T + phase_ + start;
    fall = k * T + phase_ + start + ton;
    if (rise <= t + eps) rise += T;
    if (fall <= t + eps) fall += T;
  }
  return rise < fall ? rise : fall;
}

Pwm::CompOut Pwm::complementary(double t, double deadtime) const {
  if (!(deadtime >= 0.0) || !std::isfinite(deadtime)) {
    throw std::runtime_error("deadtime must be finite >= 0");
  }
  if (duty_ <= 0.0) return {false, true};
  if (duty_ >= 1.0) return {true, false};
  const double T = period();
  const double ton = duty_ * T;
  if (deadtime * 2.0 >= ton && deadtime * 2.0 >= T - ton) {
    // Deadtime eats the whole pulse: both off (documented, no overlap ever).
    return {false, false};
  }
  // Locate surrounding main edges analytically.
  const double k = std::floor((t - phase_) / T);
  double rise, fall;
  if (carrier_ == Carrier::TrailingEdge) {
    rise = k * T + phase_;
    fall = k * T + phase_ + ton;
  } else {
    const double start = (T - ton) * 0.5;
    rise = k * T + phase_ + start;
    fall = rise + ton;
  }
  const bool base = output(t);
  if (base) {
    // Last rising edge is `rise` (t is inside [rise, fall)).
    return {(t - rise) >= deadtime, false};
  }
  // Base low: last falling edge is `fall` if t >= fall else previous period's.
  double lastFall = fall;
  if (t < fall) {
    lastFall = (carrier_ == Carrier::TrailingEdge)
                   ? (k - 1.0) * T + phase_ + ton
                   : (k - 1.0) * T + phase_ + (T - ton) * 0.5 + ton;
  }
  return {false, (t - lastFall) >= deadtime};
}

// ---------------- PiController ----------------

PiController::PiController(double kp, double ki, double outMin, double outMax)
    : kp_(kp), ki_(ki), lo_(outMin), hi_(outMax) {
  requireFinite(kp, "kp");
  requireFinite(ki, "ki");
  requireFinite(outMin, "outMin");
  requireFinite(outMax, "outMax");
  if (!(hi_ > lo_)) throw std::runtime_error("PI outMax must exceed outMin");
}

void PiController::setGains(double kp, double ki) {
  requireFinite(kp, "kp");
  requireFinite(ki, "ki");
  kp_ = kp;
  ki_ = ki;
}

void PiController::setLimits(double lo, double hi) {
  requireFinite(lo, "lo");
  requireFinite(hi, "hi");
  if (!(hi > lo)) throw std::runtime_error("PI outMax must exceed outMin");
  lo_ = lo;
  hi_ = hi;
}

double PiController::update(double err, double dt) {
  requireFinite(err, "err");
  if (!(dt > 0.0) || !std::isfinite(dt)) throw std::runtime_error("PI dt must be positive");
  const double unclamped = kp_ * err + ki_ * (integ_ + err * dt);
  if (unclamped > hi_) {
    if (err < 0.0) integ_ += err * dt;  // freeze otherwise (anti-windup)
    return hi_;
  }
  if (unclamped < lo_) {
    if (err > 0.0) integ_ += err * dt;
    return lo_;
  }
  integ_ += err * dt;
  return unclamped;
}

// ---------------- Comparator ----------------

Comparator::Comparator(double threshold, double hysteresis) : threshold_(threshold) {
  requireFinite(threshold, "threshold");
  if (!(hysteresis >= 0.0) || !std::isfinite(hysteresis)) {
    throw std::runtime_error("hysteresis must be finite >= 0");
  }
  hysteresis_ = hysteresis;
}

bool Comparator::update(double x) {
  requireFinite(x, "comparator input");
  if (state_) {
    if (x < threshold_ - hysteresis_ * 0.5) state_ = false;
  } else {
    if (x > threshold_ + hysteresis_ * 0.5) state_ = true;
  }
  return state_;
}

// ---------------- TransferFunction ----------------

TransferFunction TransferFunction::fromDigital(std::vector<double> b, std::vector<double> a) {
  if (b.empty() || a.empty() || a[0] == 0.0) {
    throw std::runtime_error("TF needs non-empty b/a with a[0] != 0");
  }
  for (double v : b) requireFinite(v, "TF b coeff");
  for (double v : a) requireFinite(v, "TF a coeff");
  TransferFunction tf;
  const double a0 = a[0];
  tf.b_ = std::move(b);
  tf.a_ = std::move(a);
  for (double& v : tf.b_) v /= a0;
  for (double& v : tf.a_) v /= a0;
  tf.states_.assign((tf.b_.size() > tf.a_.size() ? tf.b_.size() : tf.a_.size()) - 1, 0.0);
  return tf;
}

namespace {
// Ascending-power polynomial helpers for Tustin.
using Poly = std::vector<double>;  // p[i] = coeff of z^i

Poly mul(const Poly& a, const Poly& b) {
  Poly c(a.size() + b.size() - 1, 0.0);
  for (std::size_t i = 0; i < a.size(); ++i) {
    for (std::size_t j = 0; j < b.size(); ++j) c[i + j] += a[i] * b[j];
  }
  return c;
}

Poly addScaled(const Poly& a, const Poly& b, double s) {
  Poly c(a.size() > b.size() ? a.size() : b.size(), 0.0);
  for (std::size_t i = 0; i < a.size(); ++i) c[i] += a[i];
  for (std::size_t i = 0; i < b.size(); ++i) c[i] += s * b[i];
  while (c.size() > 1 && c.back() == 0.0) c.pop_back();
  return c;
}

double binom(int n, int k) {
  if (k < 0 || k > n) return 0.0;
  double c = 1.0;
  for (int i = 0; i < k; ++i) c = c * (n - i) / (i + 1);
  return c;
}

// (z-1)^p and (z+1)^p, ascending.
Poly zm1pow(int p) {
  Poly c(p + 1, 0.0);
  for (int k = 0; k <= p; ++k) c[k] = binom(p, k) * ((p - k) % 2 == 0 ? 1.0 : -1.0);
  return c;
}

Poly zp1pow(int p) {
  Poly c(p + 1, 0.0);
  for (int k = 0; k <= p; ++k) c[k] = binom(p, k);
  return c;
}
}  // namespace

TransferFunction TransferFunction::fromContinuous(const std::vector<double>& num,
                                                 const std::vector<double>& den, double dt) {
  if (!(dt > 0.0) || !std::isfinite(dt)) throw std::runtime_error("TF dt must be positive");
  if (num.empty() || den.empty() || den[0] == 0.0) {
    throw std::runtime_error("TF needs non-empty num/den with den[0] != 0");
  }
  for (double v : num) requireFinite(v, "TF num coeff");
  for (double v : den) requireFinite(v, "TF den coeff");
  const int mn = static_cast<int>(num.size()) - 1;
  const int md = static_cast<int>(den.size()) - 1;
  if (md < mn) throw std::runtime_error("TF must be proper (deg den >= deg num)");
  const int M = md;
  if (M > 8) throw std::runtime_error("TF order must be <= 8");
  const double K = 2.0 / dt;
  double Kpow = 1.0;

  // Ascending-in-s coefficients: sAsc[i] = coeff of s^i.
  auto toAsc = [](const std::vector<double>& d) {
    Poly a(d.size(), 0.0);
    for (std::size_t i = 0; i < d.size(); ++i) a[i] = d[d.size() - 1 - i];
    return a;
  };
  const Poly nAsc = toAsc(num);
  const Poly dAsc = toAsc(den);

  // N(z) = (z+1)^(M-mn) * sum_i nAsc[i] K^i (z-1)^i (z+1)^(mn-i).
  Poly Nz(1, 0.0), Dz(1, 0.0);
  for (int i = 0; i <= mn; ++i) {
    Poly term = mul(zm1pow(i), zp1pow(M - i));
    for (double& v : term) v *= nAsc[static_cast<std::size_t>(i)] * Kpow;
    Nz = addScaled(Nz, term, 1.0);
    Kpow *= K;
  }
  Kpow = 1.0;
  for (int i = 0; i <= md; ++i) {
    Poly term = mul(zm1pow(i), zp1pow(M - i));
    for (double& v : term) v *= dAsc[static_cast<std::size_t>(i)] * Kpow;
    Dz = addScaled(Dz, term, 1.0);
    Kpow *= K;
  }
  // To descending digital vectors.
  std::vector<double> b(M + 1, 0.0), a(M + 1, 0.0);
  for (std::size_t i = 0; i < Nz.size() && i <= static_cast<std::size_t>(M); ++i) {
    b[static_cast<std::size_t>(M) - i] = Nz[i];
  }
  for (std::size_t i = 0; i < Dz.size() && i <= static_cast<std::size_t>(M); ++i) {
    a[static_cast<std::size_t>(M) - i] = Dz[i];
  }
  if (a[0] == 0.0) throw std::runtime_error("Tustin failed: a[0] == 0");
  return fromDigital(b, a);
}

double TransferFunction::update(double x) {
  requireFinite(x, "TF input");
  const std::size_t nb = b_.size();
  const std::size_t na = a_.size();
  const std::size_t n = states_.size() + 1;  // max(nb, na)
  double y = b_[0] * x + (states_.empty() ? 0.0 : states_[0]);
  // DF-II-transposed delay update.
  for (std::size_t i = 0; i + 1 < n; ++i) {
    const double bi = (i + 1 < nb) ? b_[i + 1] : 0.0;
    const double ai = (i + 1 < na) ? a_[i + 1] : 0.0;
    const double nxt = (i + 1 < states_.size()) ? states_[i + 1] : 0.0;
    states_[i] = bi * x - ai * y + nxt;
  }
  return y;
}

}  // namespace control
}  // namespace power_engine
