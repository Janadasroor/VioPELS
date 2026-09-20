#pragma once
#include <map>
#include <string>
#include <vector>

#include "power_engine/circuit.h"
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

  Circuit circuit_;
  TransientSolver solver_;
  double tStop_ = 0.0;
  SimulationStatus status_ = SimulationStatus::Idle;
  Solution solution_;
  SolutionCallback callback_;
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
};

}  // namespace power_engine
