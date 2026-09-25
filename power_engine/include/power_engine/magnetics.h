#pragma once
#include <map>
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
/// lamination thickness d [m], d = 0 for unlaminated ferrite -> Pe = 0).
/// Pair with HysteresisCore loss (rate independent) for the two-term
/// separation; derive an equivalent parallel resistance as
/// R = Vrms^2/(P*Vol) at the operating point.
/// independent) for the two-term separation; derive an equivalent parallel
/// resistance as R = Vrms^2/(P*Vol) at the operating point.
double eddyLossDensity(double rho, double thickness, double freqHz, double bPeak);

// ---------------------------------------------------------------------------
// Gapped-inductor synthesis (item 18: magnetics design). Closes the loop
// on the analysis models above: given a core geometry + material (the
// stand-in for a vendor core database, explicitly future work) and an
// electrical spec, compute turns + gap, then VERIFY with the module's own
// ReluctanceNetwork (saturable L(i) sweep), HysteresisCore (minor-loop
// loss on the ripple) and eddyLossDensity. Deliberate boundary: copper
// (window/fill/thermal) is the caller's job — N + MLT are reported for it,
// and the thermal module couples core loss onward.
// ---------------------------------------------------------------------------

/// Core geometry (one magnetic path; toroid/E-core center leg equivalent).
struct CoreGeometry {
  double ae = 0.0;   ///< effective cross-section [m^2], > 0
  double le = 0.0;   ///< effective path length [m], > 0
  double ve = 0.0;   ///< effective volume [m^3], > 0
  double mlt = 0.0;  ///< mean length per turn [m], > 0 (for copper calcs)
  double windowArea = 0.0;  ///< winding window [m^2], > 0 (fill check)
};

/// Winding (copper) spec. AC proximity/skin via Dowell below; gap-fringe
/// loss (2D FEM territory) stays out by design.
struct WindingSpec {
  double wireAreaM2 = 0.0;  ///< copper per turn [m^2], > 0 (round wire)
  double resistivity = 17.2e-9;  ///< winding resistivity [Ohm m] at 20C, > 0
  double tempAlpha = 0.00393;  ///< resistance tempco [1/K] (Cu), finite >= 0
  double maxFill = 0.4;  ///< window fill limit N*Aw/Aw_window, in (0, 1)
  int layers = 1;  ///< winding layers m >= 1 (Dowell proximity count)
};

/// Winding resistivity at temperature: rho20*(1 + alpha*(T - 20C)).
double windingResistivityAtTemp(const WindingSpec& w, double tempC);

/// Dowell AC resistance factor Fr = Rac/Rdc for m full layers at
/// normalized thickness d = h/delta (h = (pi/4)*d_wire square-equivalent,
/// delta = skin depth, porosity 1): d*((sinh2d+sin2d)/(cosh2d-cos2d) +
/// (2(m^2-1)/3)*(sinhd-sind)/(coshd+cosd)). d < 1e-3 returns exactly 1
/// (series limit; avoids 0/0). Throws unless m >= 1 and d finite >= 0.
double dowellFactor(int layers, double delta);

/// Core material: B-H (tanh) + resistivity/lamination for eddy loss.
/// Ferrite: high rho with lamThickness 0 (no laminations -> Pe = 0).
/// Temperature (needs datasheet data — defaults are inert):
/// Bs(T) = Bs*(1 + bsTempCoeff*(T - 20C)) bounds the Bsat margin and the
/// verification network; rhoCore(T) likewise feeds the eddy term.
/// Small-signal mu_i stays at the 20C slope by design (saturation bound
/// is what temperature threatens, and that is what is modeled).
struct CoreMaterial {
  HystereticMaterial bh;  ///< Bs > 0, a > 0, hc >= 0 (validated)
  double rho = 0.0;       ///< resistivity [Ohm m] at 20C, finite > 0
  double lamThickness = 0.0;  ///< lamination thickness [m], >= 0
  double bsTempCoeff = 0.0;  ///< dBs/dT [1/K], finite (ferrite ~ -0.002)
  double rhoTempCoeff = 0.0;  ///< drho/dT [1/K], finite
};

/// Electrical spec for the inductor.
struct InductorSpec {
  double inductance = 0.0;  ///< target L [H], > 0
  double iPeak = 0.0;       ///< peak current incl. ripple [A], > 0
  double iRms = 0.0;        ///< rms current [A], >= 0 (DC bias estimate)
  double iRipplePkPk = 0.0;  ///< switching ripple pk-pk [A], >= 0
  double freqHz = 0.0;      ///< ripple frequency [Hz], > 0
  double bMaxMargin = 0.75;  ///< Bpeak budget as fraction of Bs, in (0, 1)
  double maxGapFraction = 0.05;  ///< gap/le limit, in (0, 0.5)
  double maxRolloff = 0.1;  ///< L(Ipeak)/L(0) drop limit, in (0, 1)
  double lossBudgetW = 0.0;  ///< total (core + copper) loss budget [W], 0 = off
  double tempC = 20.0;  ///< winding+core temperature [degC], finite.
    ///< Resistivity, skin depth, eddy rho and the Bs bound evaluate here;
    ///< 20C reproduces the legacy numbers exactly. Temperature iteration
    ///< (losses -> thermal network -> T) stays caller-side — see the
    ///< ThermalFeedback test for the fixed-point pattern.
};

/// Synthesized gapped inductor (all fields verified, not just computed).
struct InductorDesign {
  int turns = 0;          ///< integer N (>= 1)
  double gapM = 0.0;      ///< gap length [m] (>= 0; fringing included)
  double bPeak = 0.0;     ///< peak flux density [T] at Ipeak + ripple/2
  double lAtZero = 0.0;   ///< secant L at 1mA from network sweep [H]
  double lAtPeak = 0.0;   ///< secant L at Ipeak from network sweep [H]
  double rolloff = 0.0;   ///< 1 - lAtPeak/lAtZero
  /// Minor-loop hysteresis loss at ripple [W]. Model-resolution note:
  /// HysteresisCore tracks H 1:1 below the Hc clamps, so switching
  /// ripples (dH < 2*Hc, i.e. every sane design) report exactly 0 —
  /// the model resolves major-loop loss, not minor-ripple loss. Eddy
  /// carries the switching-frequency loss (standard two-term practice).
  double hysteresisLossW = 0.0;
  double eddyLossW = 0.0;        ///< eddy loss at ripple [W]
  double copperLossW = 0.0;      ///< DC copper loss at Irms [W]
  double acCopperLossW = 0.0;    ///< Dowell AC copper at ripple RMS [W]
  double windowFill = 0.0;       ///< N*wireArea/windowArea (<= maxFill)
  double dcrOhm = 0.0;           ///< DC resistance MLT*N*rho(T)/Aw [Ohm]
  double tempC = 20.0;           ///< evaluation temperature echo [degC]
};

/// Synthesize + verify a gapped inductor. Procedure (standard gapped-core
/// flow): N = ceil(L*Imax/(Bmax*Ae)) with Imax = iPeak + ripple/2 (Bsat
/// bound, evaluated at the hot Bs(T)); raise N until the core reluctance fits inside N^2/L (gap >= 0);
/// solve gap from L = N^2/(Rcore + Rgap) with first-order fringing
/// F = 1 + lg/sqrt(Ae); then verify L(i) on a saturable-core + gap series
/// network, Bpeak, roll-off, window fill (N*Aw/Awindow), DC copper loss
/// (MLT*N*rho/Aw at Irms), minor-loop hysteresis loss (ripple triangle
/// through HysteresisCore) and eddy loss. Throws on bad inputs or
/// infeasible specs (gap over limit, B over Bs, roll-off over limit, fill
/// over limit, loss over budget). Thermal evaluation is built in
/// (spec.tempC + material tempcoefficients); thermal iteration stays
/// caller-side (feed total loss into the thermal module, redesign at
/// the new T until fixed).
InductorDesign designGappedInductor(const InductorSpec& spec, const CoreGeometry& core,
                                    const CoreMaterial& mat, const WindingSpec& winding);

// ---------------------------------------------------------------------------
// Reluctance-network magnetic domain + winding (circuit) interface.
// ---------------------------------------------------------------------------

/// Magnetic domain nodal network: scalar potentials U [A·t] on nodes,
/// fluxes Phi [Wb] through branches, MMF drops F = R*Phi.
/// Linear reluctances solve exactly (nodal); saturable branches
/// (tanh B-H) iterate by Newton with a numeric Jacobian.
/// Winding interface (explicit co-simulation, same philosophy as the
/// thermal/mechanical coupling: electrical dt steps with magnetic state
/// refreshed per step): setWindingCurrent(i) from the circuit, solve(),
/// then windingFlux() gives lambda/N for the back-EMF (finite-difference
/// dλ/dt) or equivalentInductance() for linear networks (exact, L=N^2/R).
/// Node 0 is the magnetic reference (ground).
class ReluctanceNetwork {
 public:
  ReluctanceNetwork() = default;

  /// Linear reluctance branch [A·t/Wb] between nodes.
  void addReluctance(const std::string& name, int n1, int n2, double r);
  /// Saturable branch from geometry + tanh B-H: length l [m], area A [m^2],
  /// saturation Bs [T], shape field a [A/m]. R(Phi) = l*H/(A*B) with
  /// H = a*atanh(B/Bs), B = Phi/A.
  void addSaturableReluctance(const std::string& name, int n1, int n2, double l,
                              double area, double bs, double a);
  /// N-turn winding linking a branch's flux (by reluctance name).
  void addWinding(const std::string& name, const std::string& branch, double turns);
  /// Winding (circuit) current [A]; sets the MMF N*i on its branch.
  void setWindingCurrent(const std::string& name, double current);
  /// Solve for node potentials + branch fluxes (throws on singularity or
  /// Newton non-convergence).
  void solve();
  /// Branch flux [Wb] after solve (throws if branch unknown/stale).
  double branchFlux(const std::string& branch) const;
  /// Winding flux [Wb] = N * branch flux (flux linkage per... note: this
  /// is Phi (not lambda); lambda = turns * windingFlux()).
  double windingFlux(const std::string& name) const;
  /// Equivalent inductance [H] seen at a winding (linear networks):
  /// energize with 1A (all else zero), lambda = N*Phi. Throws if any
  /// branch is saturable (use co-simulation there instead).
  double equivalentInductance(const std::string& name);

 private:
  struct Branch {
    std::string name;
    int n1 = 0, n2 = 0;
    bool saturable = false;
    double r = 0.0;  // linear reluctance (valid if !saturable)
    double l = 0.0, area = 0.0, bs = 0.0, a = 0.0;
    double flux = 0.0;  // last solve
  };
  struct Winding {
    std::string name;
    std::string branch;
    double turns = 0.0;
    double current = 0.0;
  };
  int nodeIndex(int node) const;
  std::map<std::string, std::size_t> branchByName_;
  std::map<std::string, Winding> windings_;
  std::vector<Branch> branches_;
  std::map<int, int> nodeIndex_;
  std::vector<double> potentials_;
  bool solved_ = false;
};

}  // namespace magnetics
}  // namespace power_engine
