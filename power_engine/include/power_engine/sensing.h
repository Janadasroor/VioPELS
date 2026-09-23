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

}  // namespace sensing
}  // namespace power_engine
