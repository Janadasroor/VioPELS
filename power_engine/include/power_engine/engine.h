#pragma once
#include <map>
#include <string>
#include <vector>

#include "power_engine/circuit.h"
#include "power_engine/loss_tables.h"
#include "power_engine/netlist.h"
#include "power_engine/solver.h"
#include "power_engine/thermal.h"
#include "power_engine/types.h"

namespace power_engine {

/// Gate edge scheduled at an exact simulation time.
struct SwitchEvent {
  double at = 0.0;  ///< event time [s]
  std::string name;
  bool closed = false;
  bool applied = false;
};

/// Top-level engine: programmatic circuit build + fixed-step run.
/// Phase 2: setSwitch() gate control + scheduleSwitch() exact-time edges.
/// Scheduled edges split step() into sub-steps so transitions land exactly
/// (diode iteration runs inside every sub-step). setSwitch() remains as an
/// immediate, step-boundary control.
/// Phase 3: loadNetlist() elaborates a SPICE-like netlist (circuit + .tran);
/// .control pwm / .thermal specs are stored and retrievable for Phases 4/5.
/// setParameter() re-elaborates with a locked override (call before start()).
class Engine {
 public:
  Engine();

  Circuit& circuit() { return circuit_; }
  const Circuit& circuit() const { return circuit_; }

  void setTimeStep(double dt);
  void setStopTime(double tStop);
  double stopTime() const { return tStop_; }  // 0 = none
  void clearStopTime() { tStop_ = 0.0; }
  void setCallback(SolutionCallback cb) { callback_ = std::move(cb); }
  double time() const;
  /// Opt-in adaptive stepping (solver step-doubling; composes with exact
  /// event sub-stepping: each event-free interval is stepped adaptively).
  void setAdaptive(double tol, double dtMin, double dtMax) {
    solver_.setAdaptive(tol, dtMin, dtMax);
  }
  void clearAdaptive() { solver_.clearAdaptive(); }

  /// Gate drive for ideal switches (takes effect on next step()).
  void setSwitch(const std::string& name, bool closed);
  /// Schedule an exact-time gate edge. May be called before start() or while
  /// running; applied when simulation time reaches `at`.
  void scheduleSwitch(const std::string& name, bool closed, double at);
  void clearScheduledEvents();
  /// Mark all scheduled edges un-applied (they fire again when reached).
  /// Used by steady-state shooting to replay periodic gate drives.
  void resetScheduledEvents();
  std::size_t pendingEventCount() const;
  /// Expand loaded `.control pwm` specs into exact scheduled edges over
  /// [0, stopTime]. Needs a stop time (from `.tran` or setStopTime).
  /// Explicit on purpose: manual gate drives and auto PWM don't mix.
  void applyPwmSpecs();

  /// Attach a datasheet loss model to a switch/diode (table-driven Eon/Eoff
  /// sampled at edges + Tj-dependent Ron/Vf refreshed every sub-step).
  /// Conduction loss stays integral-of-v*i (exact, automatically consistent
  /// with the Tj-stamped Ron/Vf). Coupling is one-way explicit (Tj from the
  /// previous sub-step; iteration is a later roadmap item).
  void attachLossModel(const std::string& device, loss::DeviceLossModel model);
  bool hasLossModel(const std::string& device) const;
  /// Attach all `.etable`-referenced loss models from the loaded netlist
  /// (device EON_TABLE/EOFF_TABLE/RON_TABLE/VF_TABLE refs, via MODEL or
  /// inline). Mirrors applyThermalSpecs.
  void applyLossModels();
  /// Junction temperature if a thermal network is attached, else 25C ambient.
  /// A junction-temp override (setJunctionTempOverride) wins over both and
  /// lets an outer loop impose Tj (averaged-loss electro-thermal stepping);
  /// do not combine an override with an attached thermal network.
  double deviceTemp(const std::string& device) const;
  /// Pin Tj for a switch/diode (outer-loop electro-thermal stepping).
  /// Persists across resetAccumulators (window re-runs keep the imposed Tj).
  void setJunctionTempOverride(const std::string& device, double tj);
  void clearJunctionTempOverride(const std::string& device);
  /// Attach a thermal network to a switch/diode (loss-driven Tj).
  /// Conduction loss = integral of v*i (exact in the ideal model:
  /// switch I^2*Ron, diode Vf*I + I^2*Ron); switching loss = Eon/Eoff
  /// per gate edge. Thermal networks advance with the solver sub-step.
  /// Tj appears in Solution.probes as "tj:<device>".
  void attachThermal(const std::string& device, thermal::ThermalNetwork net);
  /// Attach all `.thermal` specs from the loaded netlist.
  void applyThermalSpecs();
  bool hasThermal(const std::string& device) const;
  /// Junction temperature [degC] (throws if not attached).
  double junctionTemp(const std::string& device) const;
  /// Accumulated energies (throws for non switch/diode).
  thermal::DeviceLoss deviceLoss(const std::string& device) const;

  const SolverStats& solverStats() const { return solver_.stats(); }

  /// Elaborate a netlist (replaces the circuit; clears scheduled events).
  /// Applies `.tran` to dt/tstop. Call before start().
  void loadNetlist(const std::string& netlist);
  /// Lock a `.param` override and re-elaborate. Call before start().
  void setParameter(const std::string& name, double value);
  bool hasNetlist() const { return !netlistSource_.empty(); }
  const netlist::NetlistResult& netlist() const { return net_; }

  /// Branch current n1->n2 (primary side for transformers) at current time.
  double deviceCurrent(const std::string& name) const;

  void start();
  void step();
  void stop();

  /// Advance exactly to tEnd (requires no stop time set), leaving status
  /// Running. Used by steady-state shooting for exact period runs.
  void runUntil(double tEnd);
  /// Reset loss accumulators, thermal states, and gate-edge tracking
  /// without touching solver states (for clean per-period energies).
  void resetAccumulators();
  /// Solver state snapshot/restore (advanced use: shooting methods).
  SolverState saveSolverState() const;
  void restoreSolverState(const SolverState& s);
  /// Reposition at an orbit start for continued simulation (shooting use):
  /// restores solver state, re-stamps clock to t0, replays due gate edges,
  /// refreshes accumulators + probes.
  void rewindTo(const SolverState& state, double t0);

  SimulationStatus status() const { return status_; }
  const Solution& currentSolution() const { return solution_; }

 private:
  void refreshSolution();
  void applyDueEvents(double tNow);
  void updateLosses(double dtSub);
  /// Snapshot pre-sub-step (v,i) of modeled switches/diodes for edge I/V
  /// sampling, and refresh their Tj-dependent ron/vf from current Tj.
  /// No-op when no loss models are attached (zero overhead otherwise).
  void preStepLossHooks();

  Circuit circuit_;
  TransientSolver solver_;
  double tStop_ = 0.0;
  SimulationStatus status_ = SimulationStatus::Idle;
  Solution solution_;
  SolutionCallback callback_;
  // Probe fast path (refreshSolution runs every sub-step): node ids + key
  // strings snapshotted at start() (topology is fixed mid-run — the solver
  // throws otherwise), probes updated in place (no clear/reinsert, no
  // to_string per step).
  std::vector<int> probeNodes_;
  std::vector<std::string> probeKeys_;
  std::vector<SwitchEvent> events_;  // sorted by .at
  // Phase 3 netlist state.
  std::string netlistSource_;
  netlist::NetlistResult net_;
  std::map<std::string, double> paramOverrides_;  // UPPER(name) -> value
  bool started_ = false;  // latched by start(); cleared by loadNetlist()
  // Phase 5 electro-thermal state.
  std::map<std::string, thermal::DeviceLoss> losses_;      // all S/D, reset at start()
  std::map<std::string, thermal::ThermalNetwork> thermals_;  // attached networks
  std::map<std::string, bool> lastGate_;  // gate states after last sub-step
  // Datasheet loss models (empty = scalar path) + pre-sub-step (v,i)
  // snapshots for edge I/V sampling (histories are post-step).
  std::map<std::string, loss::DeviceLossModel> lossModels_;
  std::map<std::string, std::pair<double, double>> preStepVi_;
  std::map<std::string, double> tjOverrides_;  // pinned Tj (outer loop)
};

}  // namespace power_engine
