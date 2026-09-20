#pragma once
#include <cstddef>
#include <map>
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

/// Solver statistics for event-detection acceptance ("no missed events").
struct SolverStats {
  long long steps = 0;        ///< accepted time steps
  long long diodeEvents = 0;  ///< diode on<->off commutations
  long long resolves = 0;     ///< extra MNA re-solves due to diode iteration
  long long sparseSolves = 0;  ///< linear solves via SparseLU (large systems)
  long long newtonIters = 0;   ///< Newton linearizations (saturable magnetics)
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
/// Phase 2: ideal switches (Ron/Roff, gate-controlled, re-assembled every
/// step) + auto-commutated diodes with in-step event iteration.
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
  /// orbits; histories and events are NOT touched — caller must align them).
  void setTime(double t);

  double nodeVoltage(int node) const;
  /// Current through device (n1->n2) at current time.
  double deviceCurrent(const std::string& name) const;

 private:
  void rebuildMaps();
  void assemble() const;  // fills workA_/workZ_ (reused buffers)
  void updateHistories(const Eigen::VectorXd& x);
  Eigen::VectorXd solveLinear() const;  // PartialPivLU on work buffers
  /// Check diode states against current x; flip any that must commutate.
  /// Returns true if at least one diode changed state.
  bool updateDiodeStates(const Eigen::VectorXd& x);
  /// One fixed step of exactly dtNew (no error control). Advances t_.
  void fixedStep(double dtNew);
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
  std::vector<int> nodeList_;           // non-ground nodes, sorted
  std::map<int, int> nodeIndex_;        // node id -> 0-based row
  std::map<std::string, std::size_t> extraRow_;  // device -> base extra-var row
  // Precomputed MNA rows per device index (-1 = ground).
  std::vector<int> rowA_, rowB_, rowC_, rowD_;
  bool hasNonlinear_ = false;  // any saturable inductor present
  Eigen::VectorXd x_;                   // last solution
  // Reused across steps: no per-step heap allocation in the hot loop.
  mutable Eigen::MatrixXd workA_;
  mutable Eigen::VectorXd workZ_;
  mutable Eigen::PartialPivLU<Eigen::MatrixXd> workLu_;
  mutable Eigen::SparseMatrix<double> workAs_;
  mutable Eigen::SparseLU<Eigen::SparseMatrix<double>> workSlu_;
  mutable SolverStats stats_;  // bookkeeping, mutated even in const solves
};

}  // namespace power_engine
