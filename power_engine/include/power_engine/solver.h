#pragma once
#include <cstddef>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>
// Eigen is third-party: silence its headers under MSVC /W4 (GCC/Clang use
// SYSTEM include dirs from CMake; that mechanism doesn't cover MSVC).
#ifdef _MSC_VER
#pragma warning(push, 0)
#endif
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>
#ifdef _MSC_VER
#pragma warning(pop)
#endif

#include "power_engine/circuit.h"

namespace power_engine {

/// Ill-conditioned/singular MNA factorization failure. Structural causes
/// (floating nodes, V-source loops) and gray-zone companion spreads at
/// extreme dt both land here; adaptive controllers distinguish them by
/// retrying larger dt (conditioning escapes, structure rethrows).
struct SingularError : std::runtime_error {
  using std::runtime_error::runtime_error;
};

/// Time-integration method for the transient solver. Trapezoidal is the
/// default (2nd-order, energy-preserving, exact for our validation suite).
/// TrBdf2 (trapezoidal half step + BDF2 full step) is 2nd-order and
/// L-stable: it damps the trapezoidal Nyquist ringing that ideal
/// switching events excite on stiff nodes, at ~2x solves per step.
/// Opt-in per solver; histories are shared so the mode may change mid-run.
enum class Integrator { Trapezoidal, TrBdf2 };

/// Solver statistics for event-detection acceptance ("no missed events").
struct SolverStats {
  long long steps = 0;        ///< accepted time steps
  long long diodeEvents = 0;  ///< diode on<->off commutations
  long long resolves = 0;     ///< extra MNA re-solves due to diode iteration
  long long sparseSolves = 0;  ///< linear solves via SparseLU (large systems)
  long long newtonIters = 0;   ///< Newton linearizations (saturable magnetics)
  long long factorSkips = 0;   ///< solves reusing a cached factorization
  long long integratorSwitches = 0;  ///< auto-integrator mode changes
  long long diodeCapHits = 0;  ///< diode loops that exhausted kMaxDiodeIters
                               ///< and accepted the last state (refine-roadmap
                               ///< R4 safety-net monitor; ~0 on all fixtures)
};

/// Opaque solver snapshot: clock, last solution vector, and full device
/// states (trapezoidal histories, flux, diode/switch discrete states,
/// recovery timers). Used by steady-state shooting to restart exact
/// periodic orbits. Topology must match on restore (device count checked).
struct SolverState {
  double t = 0.0;
  Eigen::VectorXd x;
  std::vector<Device> devices;
};

/// Fixed-step trapezoidal MNA transient solver.
/// Phase 2: ideal switches (Ron/Roff, gate-controlled) + auto-commutated
/// diodes with in-step event iteration.
/// Factorization caching (item 14b): the MNA matrix depends only on the
/// topology signature (switch/diode states, R/L/C/k values, dt) — not on
/// histories or source values, which live in the RHS. Steps with an
/// unchanged signature skip reassembly of A and refactorization
/// (counted as factorSkips); any state/value/dt change re-factorizes.
/// Saturable-inductor circuits bypass the cache (Newton relinearizes).
/// Numerical decisions: trapezoidal companions for L/C; conducting diode =
/// Norton equivalent (G=1/Ron in parallel with current source G*Vf, i.e.
/// Vd = Vf + I*Ron); blocking diode = Roff. After each solve, diode
/// voltages/currents are checked and flipped states trigger an immediate
/// re-solve at the same time point (up to kMaxDiodeIters). Chattering is
/// suppressed with current/voltage hysteresis (turn-off only when Id < -Ihys,
/// turn-on only when Vd > Vf + Vhys). Deterministic, single-threaded.
/// Phase 6: PartialPivLU (equal residuals to FullPivLU at this size,
/// measured) with exact-zero-pivot singularity guard — MNA structural
/// singularities (floating nodes, V-source loops) produce exact zeros;
/// plus precomputed node rows and reused MNA buffers (no per-step malloc).
/// Large systems (rows >= kSparseThreshold) switch to SparseLU with a
/// once-per-topology symbolic factorization (pattern is topology-fixed:
/// switch/diode states only change values); measured crossover ~n=50,
/// threshold 64. Deterministic, single-threaded.
class TransientSolver {
 public:
  explicit TransientSolver(Circuit& circuit, double dt = 1e-6);

  void setStep(double dt);
  double stepSize() const { return dt_; }
  double time() const { return t_; }
  const SolverStats& stats() const { return stats_; }

  /// Opt-in adaptive stepping (step-doubling error control, order-2
  /// trapezoidal exponent 1/3). Off by default: fixed-step behavior is
  /// bit-identical. When on, step() may shrink/grow dt_ within
  /// [dtMin, dtMax]; stepTo(t) advances across event-free intervals.
  struct AdaptiveConfig {
    bool enabled = false;
    double tol = 1e-4;      ///< relative error tolerance per step
    double dtMin = 1e-9;
    double dtMax = 1e-3;
  };
  void setAdaptive(double tol, double dtMin, double dtMax);
  void clearAdaptive() { adaptive_ = AdaptiveConfig{}; }
  bool adaptive() const { return adaptive_.enabled; }
  void setIntegrator(Integrator m) {
    integ_ = m;
    auto_ = AutoConfig{};  // explicit mode wins: auto off
  }
  Integrator integrator() const { return integ_; }
  /// PLECS-Auto-style stiffness switching: trapezoidal while smooth,
  /// TR-BDF2 when stiffness is detected (diode-iteration pile-up, Newton
  /// strain, or sustained Nyquist alternation of a node voltage), back to
  /// trapezoidal after cleanBack quiet steps. Histories are shared, so
  /// switching mid-run is exact. Off by default (deterministic default).
  struct AutoConfig {
    bool enabled = false;
    int window = 64;        ///< diode-resolve counting window [steps]
    int trigResolves = 6;   ///< resolves in window -> BDF2 (normal clusters
                            ///< peak at 3, coarse-step stress at 13+)
    int trigNewton = 8;     ///< Newton iters in one step -> BDF2
    int flipNeed = 8;       ///< consecutive same-node delta sign flips -> BDF2
    double flipRel = 0.05;  ///< flips count only above 5% of node level:
                            ///< sustained numerical ringing is O(100%) of
                            ///< signal (measured 280%), commutation bursts
                            ///< O(0.001%) (measured 0.002%) — 5 decades apart
    int cleanBack = 512;    ///< quiet steps before returning to trap
  };
  void setIntegratorAuto(bool on) {
    auto_ = AutoConfig{};
    auto_.enabled = on;
  }
  bool integratorAuto() const { return auto_.enabled; }
  /// Error estimate of the last accepted adaptive step: embedded
  /// stage-difference for TR-BDF2, full-vs-half difference for
  /// step-doubling (-1 when the last step used another path).
  /// Post-accept invariant on smooth steps: 0 <= estimate <= tol
  /// (event accepts may exceed tol: the estimate is invalid across
  /// discontinuities, accepted deliberately like SPICE LTE bypass).
  double lastStepError() const { return lastEmbeddedErr_; }

  /// Reset t=0 and device histories from Device::ic.
  void initialize();
  /// Advance one step of size dt_ (adaptive: error-controlled, see above).
  void step();
  /// Adaptive advance to tLimit (requires adaptive mode).
  void stepTo(double tLimit);

  /// Snapshot / restore full solver state (advanced use: shooting methods).
  SolverState saveState() const;
  void restoreState(const SolverState& s);
  /// Re-stamp the simulation clock (advanced use: repositioning periodic
  /// orbits). Raw primitive: histories (device v_prev/i_prev/flux/timers),
  /// the solution vector, scheduled Engine events, and loss accumulators
  /// are NOT touched, so a lone setTime() silently corrupts the trajectory.
  /// Realignment contract (all four, in order): restoreState() the device
  /// histories, setTime() the clock, re-arm + re-apply due Engine events,
  /// refresh accumulators/probes. Sole in-tree caller: Engine::rewindTo(),
  /// which performs the full sequence (proven bitwise-equivalent by
  /// RewindToReproducesUninterruptedRun). No debug assert is possible here:
  /// histories carry no timestamps, and stamping one would cost per-step
  /// writes in the hot loop (see item 14 de-churn).
  void setTime(double t);

  double nodeVoltage(int node) const;
  /// Sorted non-ground node ids (topology-fixed mid-run; for probe loops).
  const std::vector<int>& nodeList() const { return nodeList_; }
  /// Current through device (n1->n2) at current time.
  double deviceCurrent(const std::string& name) const;

 private:
  void rebuildMaps();
  /// Fill workA_/workZ_. withMatrix=false refreshes only the RHS (z):
  /// valid when the cached factorization still matches (same topology
  /// signature + dt) — histories, source values and recovery currents
  /// live in z only.
  void assemble(bool withMatrix) const;
  /// History target/commit method for updateHistories: trapezoidal or
  /// BDF2 commit to step state, or trapezoidal capture to midpoint state
  /// (TR-BDF2 stage 1; timers/flux-release/edge state untouched).
  enum class HistHow { TrapPrev, TrapMid, Bdf2Prev };
  void updateHistories(const Eigen::VectorXd& x, HistHow how);
  Eigen::VectorXd solveLinear() const;  // factorize + solve (full path)
  void factorize() const;               // (re)factor workA_ into workLu_/workSlu_
  Eigen::VectorXd solveFactors() const;  // triangular solve with workZ_
  /// Assemble path with factorization caching: full assemble + factorize
  /// on signature change, RHS-only assemble otherwise (counts factorSkips).
  /// Saturable-inductor circuits bypass the cache (Newton relinearizes A
  /// every iteration).
  void assembleCached() const;
  /// Check diode states against current x; flip any that must commutate.
  /// Returns true if at least one diode changed state.
  bool updateDiodeStates(const Eigen::VectorXd& x);
  /// One fixed step of exactly dtNew (no error control). Advances t_.
  void fixedStep(double dtNew);
  /// One TR-BDF2 step of exactly dtNew (no error control). Advances t_.
  void trBdf2Step(double dtNew);
  /// One adaptive TR-BDF2 step (embedded stage-difference control).
  void embeddedStep();
  /// One converged solve at the current dt_/stage (Newton + diode loops).
  void convergeStep();
  /// Auto-integrator bookkeeping at step end (no-op unless enabled).
  /// Call with the per-step Newton iteration count and resolve count.
  void autoUpdate(int newtonThisStep, int resolvesThisStep);
  struct Snapshot;
  Snapshot snapshot() const;
  void restore(const Snapshot& s);
  /// Newton update for saturable inductors at solved x; true if converged.
  bool newtonUpdate(const Eigen::VectorXd& x);
  int nodeRow(int node) const {  // -1 for ground
    if (node == 0) return -1;
    auto it = nodeIndex_.find(node);
    return it == nodeIndex_.end() ? -2 : it->second;
  }

  static constexpr int kMaxDiodeIters = 10;
  static constexpr double kDiodeIhys = 1e-9;  // turn-off hysteresis [A]
  static constexpr double kDiodeVhys = 1e-9;  // turn-on hysteresis [V]
  /// System rows at/above which SparseLU beats dense PartialPivLU (measured).
  static constexpr int kSparseThreshold = 64;
  static constexpr int kMaxNewtonIters = 25;  // saturable-inductor loop
  static constexpr double kNewtonAbsTol = 1e-9;  // branch-current [A]
  static constexpr double kNewtonRelTol = 1e-6;

  Circuit& circuit_;
  double dt_;
  double t_ = 0.0;
  AdaptiveConfig adaptive_;
  Integrator integ_ = Integrator::Trapezoidal;
  AutoConfig auto_;
  // Auto-integrator runtime state (reset by rebuildMaps()).
  std::vector<int> resolveWindow_;  ///< ring buffer of per-step resolves
  int resolveWinPos_ = 0;
  int resolveWinSum_ = 0;
  std::vector<double> prevDx_;  ///< previous step delta per unknown
  std::vector<int> flipRun_;    ///< consecutive delta-sign flips per unknown
  std::vector<double> runPeak_; ///< max |dx| in the current flip run
  Eigen::VectorXd xPrevAuto_;   ///< previous committed solution (auto scan)
  bool havePrevAuto_ = false;
  int cleanSteps_ = 0;
  Eigen::VectorXd xStart_;      ///< step-start solution (embedded estimate)
  Eigen::VectorXd xMid_;        ///< half-step solution (embedded estimate)
  double lastEmbeddedErr_ = -1.0;
  // True during TR-BDF2 stage 2: selects BDF2 companions in assemble() and
  // the BDF2 Newton formula, and salts the factorization signature (stage
  // 2 shares dt_ with trapezoidal steps but stamps a different matrix).
  // Managed strictly inside trBdf2Step(); always false elsewhere.
  mutable bool bdf2Stage_ = false;
  std::vector<int> nodeList_;           // non-ground nodes, sorted
  std::map<int, int> nodeIndex_;        // node id -> 0-based row
  std::map<std::string, std::size_t> extraRow_;  // device -> base extra-var row
  // Precomputed MNA rows per device index (-1 = ground).
  std::vector<int> rowA_, rowB_, rowC_, rowD_;
  std::vector<int> rowE_;  // extra-var base row per device (-1 if none)
  bool hasNonlinear_ = false;  // any saturable inductor present
  bool hasDiodes_ = false;     // any diode present (scan fast path)
  // Topology version seen at the last rebuildMaps() (R5). Compared per
  // step: a mismatch means structural change mid-run, even at equal
  // device count (which the size check alone would miss).
  unsigned long long topoSeen_ = 0;
  Eigen::VectorXd x_;                   // last solution
  // Reused across steps: no per-step heap allocation in the hot loop.
  // Factorization objects are per cache slot (one per integrator stage):
  // a hit reuses slot factors regardless of what workA_ currently holds
  // (workA_ is scratch; the slot owns the factors of its matrix).
  mutable Eigen::MatrixXd workA_;
  mutable Eigen::VectorXd workZ_;
  mutable Eigen::PartialPivLU<Eigen::MatrixXd> workLu_[2];
  mutable Eigen::SparseMatrix<double> workAs_;
  mutable Eigen::SparseLU<Eigen::SparseMatrix<double>> workSlu_[2];
  mutable SolverStats stats_;  // bookkeeping, mutated even in const solves
  /// FNV-1a signature over every matrix-affecting input (dt, R/L/C/k,
  /// switch states + ron/roff, diode conducting/recovery + ron/roff,
  /// transformer ratio). Source values, histories and recovery currents
  /// affect only z and are deliberately excluded.
  struct MatrixSig {
    std::uint64_t h = 0;
    bool operator==(const MatrixSig& o) const { return h == o.h; }
  };
  MatrixSig matrixSig() const;
  // One cache entry per integrator stage: TR-BDF2 alternates stage 1
  // (dt/2) and stage 2 (BDF2 companions) every step, so a single entry
  // would thrash to a 0% hit rate. Trapezoidal steps always use slot 0.
  mutable MatrixSig cachedSig_[2];
  mutable bool cacheValid_[2] = {false, false};  // cleared by rebuildMaps()
};

}  // namespace power_engine
