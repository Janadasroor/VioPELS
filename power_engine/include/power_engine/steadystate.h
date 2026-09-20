#pragma once

namespace power_engine {

class Engine;

namespace steadystate {

/// Newton-shooting configuration for periodic steady-state analysis.
struct ShootingConfig {
  double period = 0.0;      ///< orbit period [s], must be > 0
  int maxIters = 25;        ///< Newton iterations cap
  double relTol = 1e-6;     ///< per-component relative tolerance
  double absTol = 1e-9;     ///< per-component absolute tolerance
  double fdStep = 1e-6;     ///< finite-difference relative step for Jacobian
};

struct ShootingResult {
  bool converged = false;
  int iters = 0;        ///< Newton iterations used
  double residual = 0.0;  ///< final normalized residual (<= 1 means converged)
};

/// Find the periodic steady state over [t0, t0+period] via Newton shooting
/// on the continuous states (inductor currents, capacitor voltages;
/// diodes re-settle inside every period simulation).
///
/// Preconditions (throw otherwise): engine started and Running, no stop
/// time set (clearStopTime), periodic gate schedule covering [t0, t0+period]
/// already scheduled, t0 >= 0. Thermal networks are treated as frozen over
/// one period (their time constants dwarf it); use averaged-loss stepping
/// outside for electro-thermal soak.
///
/// On success the engine is repositioned at the orbit start (time t0,
/// steady histories + diode states, fresh accumulators) and ready to run
/// or record steady waveforms. On failure the engine is left at the
/// best-effort orbit start likewise (check ShootingResult::converged).
ShootingResult solvePeriodicSteadyState(Engine& eng, double t0,
                                        const ShootingConfig& cfg);

}  // namespace steadystate
}  // namespace power_engine
