#pragma once
#include <string>

namespace power_engine {
namespace sensing {

/// Incremental position encoder model (item 19 follow-up: sensed FOC).
/// Quantizes the mechanical angle to ppr counts/rev (floor, [0, 2pi)).
/// Assumes a homed/indexed drive (homing sequences out of scope); the
/// electrical angle is p * quantized (same absolute reference as the
/// ideal-theta path, so benchmarks compare directly).
class Encoder {
 public:
  explicit Encoder(int ppr);  ///< throws unless ppr >= 1
  int ppr() const { return ppr_; }
  /// Count in [0, ppr) for any finite theta (wraps + handles negatives).
  int count(double thetaMech) const;
  /// Quantized angle in [0, 2pi).
  double quantized(double thetaMech) const;

 private:
  int ppr_ = 1024;
};

/// Rotor speed from quantized angle: unwrapped difference + single-pole
/// low-pass (exact dt discretization, robust to tick-rate changes).
/// Survives 2pi wraps and direction reversals; standstill reads 0.
class SpeedEstimator {
 public:
  explicit SpeedEstimator(double tau);  ///< filter time const [s], > 0
  void reset();
  /// thetaQuant in [0, 2pi) (Encoder output), t strictly increasing [s].
  /// First call arms (returns 0). Throws on non-finite input or
  /// non-increasing t.
  double update(double thetaQuant, double t);
  double omega() const { return omega_; }

 private:
  double tau_ = 2e-3;
  double prev_ = 0.0;
  double tPrev_ = 0.0;
  double omega_ = 0.0;
  bool armed_ = false;
};

/// Sensorless rotor observer: voltage-model stator flux in stationary
/// alpha-beta + 2nd-order PLL on the rotor-flux angle. Estimates the
/// controller-ready electrical angle/speed from applied phase voltages
/// + measured currents — no position sensor (completes the Item-19 FOC
/// story: ideal -> encoder -> sensorless).
///
/// Flux integrator dλ̂/dt = v - Rs*i + γ(λm²-|λ̂r|²)·λ̂r with rotor flux
/// λ̂r = λ̂ - L*i: the magnitude feedback kills integrator drift and
/// wrong initial conditions (no-slip startup transient beyond the flux
/// time constant). A PLL (natural freq + damping) on the rotor-flux
/// angle gives unwrapped θ̂e + ω̂e. Surface-PMSM model (single L);
/// blind at standstill (|EMF| ~ 0 -> fluxMag() ~ 0, angle arbitrary),
/// so a real drive starts open-loop (I-f) and hands off once fluxMag()
/// ≈ λm. The +π fold is load-bearing: this codebase's EMF sine
/// convention (e_a = +p·ω·λm·sin θe) puts the integrated rotor flux
/// opposite the park reference, so θ̂e = atan2(λ̂r) + π (pinned by the
/// synthetic-EMF unit test, not by derivation alone).
struct SensorlessParams {
  double rs = 0.0;               ///< phase resistance [Ohm], finite >= 0
  double l = 0.0;                ///< phase inductance [H], finite > 0
  double lambdaPm = 0.0;         ///< magnet flux linkage [Wb], finite > 0
  double fluxTau = 20e-3;        ///< magnitude-correction time const [s], > 0
  double pllBandwidthHz = 100.0;  ///< PLL natural frequency [Hz], > 0
  double pllDamping = 0.7;       ///< PLL damping ratio, > 0
};

class SensorlessObserver {
 public:
  explicit SensorlessObserver(const SensorlessParams& p);  ///< throws on bad params
  void reset();
  /// One tick: applied volts + measured amps in stationary alpha-beta,
  /// dt [s] finite > 0. Throws on non-finite input or dt <= 0.
  void update(double vAlpha, double vBeta, double iAlpha, double iBeta, double dt);
  double thetaE() const { return thHat_; }  ///< unwrapped elec angle [rad]
  double omegaE() const { return omHat_; }  ///< elec speed [rad/s]
  double fluxMag() const { return fluxMag_; }  ///< |rotor flux| [Wb], ~λm locked

 private:
  double rs_ = 0.0, l_ = 1.0, lam_ = 0.0;
  double gamma_ = 0.0;   // 1/(2*λm^2*fluxTau)
  double pllKp_ = 0.0;   // 2*zeta*wn
  double pllKi_ = 0.0;   // wn^2
  double lamA_ = 0.0, lamB_ = 0.0;  // stator flux state [Wb]
  double thHat_ = 0.0, omHat_ = 0.0, xi_ = 0.0;  // PLL state
  double fluxMag_ = 0.0;
};

}  // namespace sensing
}  // namespace power_engine
