#pragma once
#include <string>

#include "power_engine/engine.h"
#include "power_engine/thermal.h"

namespace power_engine {
namespace electrothermal {

/// Self-consistent thermal steady state from the averaged-loss outer loop.
///
/// Stepping strategy (timescale-separated, documented): the electrical
/// switching transient settles in ms while thermal soaks over seconds —
/// brute-force co-simulation to thermal equilibrium is infeasible. Instead
/// iterate to a fixed point: run an electrical window (fresh loss
/// accumulators, Tj pinned), average the device dissipation over the window,
/// advance the thermal network by a macro step, re-pin Tj, repeat until
/// |dTj| < tol. Within-step electrical/thermal iteration is unnecessary:
/// one sub-step moves Tj by ~P*dt/C (1e-7..1e-2 K), so the explicit
/// per-sub-step coupling is already consistent to that order.
///
/// Preconditions: engine started and Running, stop time cleared
/// (clearStopTime), gate drive scheduled over the whole horizon
/// (iterations*window), loss model attached for `device`, and NO engine
/// thermal network attached for `device` (the passed-in network is the
/// thermal side; combining both double-books dissipation).
struct ThermalSteadyResult {
  double tj = 25.0;      ///< converged junction temperature [degC]
  double pAvg = 0.0;     ///< device average dissipation at convergence [W]
  int iterations = 0;    ///< outer iterations used
  bool converged = false;
};

ThermalSteadyResult runToThermalSteady(Engine& eng, const std::string& device,
                                       thermal::ThermalNetwork& thermal,
                                       double window, double macroDt, double tol,
                                       int maxIter = 50);

}  // namespace electrothermal
}  // namespace power_engine
