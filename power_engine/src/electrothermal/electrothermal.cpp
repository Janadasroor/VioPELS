#include "power_engine/electrothermal.h"

#include <cmath>
#include <stdexcept>

namespace power_engine {
namespace electrothermal {

ThermalSteadyResult runToThermalSteady(Engine& eng, const std::string& device,
                                       thermal::ThermalNetwork& thermal,
                                       double window, double macroDt, double tol,
                                       int maxIter) {
  if (!(window > 0.0) || !std::isfinite(window)) {
    throw std::runtime_error("runToThermalSteady needs finite window > 0");
  }
  if (!(macroDt > 0.0) || !std::isfinite(macroDt)) {
    throw std::runtime_error("runToThermalSteady needs finite macroDt > 0");
  }
  if (!(tol > 0.0) || !std::isfinite(tol)) {
    throw std::runtime_error("runToThermalSteady needs finite tol > 0");
  }
  if (maxIter < 1) throw std::runtime_error("runToThermalSteady needs maxIter >= 1");
  if (eng.status() != SimulationStatus::Running) {
    throw std::runtime_error("runToThermalSteady needs a running engine");
  }
  if (!eng.hasLossModel(device)) {
    throw std::runtime_error("runToThermalSteady needs a loss model on " + device);
  }
  if (eng.hasThermal(device)) {
    throw std::runtime_error("runToThermalSteady: clear the engine thermal on " + device +
                             " (outer loop owns the network)");
  }
  ThermalSteadyResult out;
  eng.setJunctionTempOverride(device, thermal.tj());
  for (int k = 0; k < maxIter; ++k) {
    eng.resetAccumulators();
    const double t0 = eng.time();
    while (eng.time() < t0 + window) eng.step();
    const double e = eng.deviceLoss(device).total();
    const double pAvg = e / (eng.time() - t0);
    thermal.step(pAvg, macroDt);
    out.iterations = k + 1;
    out.pAvg = pAvg;
    out.tj = thermal.tj();
    if (std::abs(out.tj - eng.deviceTemp(device)) < tol) {
      out.converged = true;
      eng.setJunctionTempOverride(device, out.tj);
      break;
    }
    eng.setJunctionTempOverride(device, out.tj);
  }
  return out;
}

}  // namespace electrothermal
}  // namespace power_engine
