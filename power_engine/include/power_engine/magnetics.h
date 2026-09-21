#pragma once
#include <string>
#include <vector>

namespace power_engine {
namespace magnetics {

/// Hysteretic magnetic material (saturating-offset memory, Preisach family).
///
/// Textbook loop orientation: saturate at +Hmax, reduce H (descending)
/// through remanence +Br at H = 0 down to zero at H = -Hc, on to -Bs;
/// coming back up (ascending) through -Br at H = 0 up across zero at
/// H = +Hc. Implemented as B(H) = Bs*tanh((H - s)/a) with a memory offset
/// s in [-Hc, +Hc] tracking +H (saturating at the extremes): ascending
/// rides s = +Hc (zero at +Hc, -Br at H = 0), descending rides s = -Hc.
/// Consequences, all exact: major limbs ride the bare shifted branches
/// (counterclockwise traversal = positive loop area = loss); any loop
/// returning to a previous (H, s) state closes exactly (nested loops
/// included); pushing s into a clamp wipes outer memory (correct Preisach
/// wiping-out); minor loops are lens-shaped, thinning for small
/// excursions. Hc = 0 degenerates to the anhysteretic curve (zero area).
/// Rate-independent by construction (state advances on H only, never on
/// time).
/// Known limitation (part 2 material): s tracks every H move 1:1 while
/// unsaturated, so virgin slope and small-signal permeability read ~0
/// instead of the reversible value. Major loops, loss/cycle, closure and
/// wiping-out are exact regardless; a deadband refinement belongs with
/// the hysteresis-inductor device (part 2).
struct HystereticMaterial {
  double bs = 1.0;   ///< saturation flux density [T]
  double a = 100.0;  ///< anhysteretic shape field [A/m]
  double hc = 50.0;  ///< coercivity [A/m], >= 0
};

/// Rate-independent hysteresis state machine over H [A/m] -> B [T].
class HysteresisCore {
 public:
  explicit HysteresisCore(HystereticMaterial mat);

  void reset();  ///< virgin state, zero accumulated loss
  /// Advance to field H, return flux density B. Idempotent in H.
  double update(double h);
  double field() const { return hPrev_; }
  double flux() const { return bPrev_; }
  /// Accumulated hysteresis energy density [J/m^3] (loop area integral).
  double loss() const { return loss_; }

  // Branch curves (major loop), exposed for reference checks.
  double ascending(double h) const;
  double descending(double h) const;
  double anhysteretic(double h) const;

 private:
  HystereticMaterial mat_;
  double s_ = 0.0;  // memory offset in [-Hc, +Hc]
  double hPrev_ = 0.0, bPrev_ = 0.0;
  double loss_ = 0.0;
};

/// Classical lamination eddy-current loss density [W/m^3]:
/// Pe = pi^2/(6*rho) * d^2 * f^2 * Bpk^2 (resistivity rho [Ohm m],
/// lamination thickness d [m]). Pair with HysteresisCore loss (rate
/// independent) for the two-term separation; derive an equivalent parallel
/// resistance as R = Vrms^2/(P*Vol) at the operating point.
double eddyLossDensity(double rho, double thickness, double freqHz, double bPeak);

}  // namespace magnetics
}  // namespace power_engine
