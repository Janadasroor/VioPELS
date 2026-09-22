// Install-tree smoke test: RC charge through the INSTALLED package.
// Prints tau; exits nonzero on any failure (throw or wrong time constant).
#include <cmath>
#include <cstdio>

#include "power_engine/engine.h"

int main() {
  power_engine::Engine eng;
  eng.setTimeStep(1e-6);
  eng.setStopTime(5e-3);
  // Series RC: source -> 1k -> node 1 -> 1uF -> ground.
  eng.circuit().addVoltageSource("V1", 2, 0, 10.0);
  eng.circuit().addResistor("R1", 2, 1, 1000.0);
  eng.circuit().addCapacitor("C1", 1, 0, 1e-6, 0.0);
  eng.start();
  while (eng.status() == power_engine::SimulationStatus::Running) eng.step();
  const double v = eng.currentSolution().probes.at("v:1");
  // v(5ms) = 10*(1-exp(-5)): exact to solver tolerance.
  const double expect = 10.0 * (1.0 - std::exp(-5.0));
  std::printf("consumer: v(5ms)=%.9f expect=%.9f\n", v, expect);
  // Loose on purpose: this guards linkage/API breakage, not solver
  // accuracy (discretization error at 1us/tau=1ms is ~3e-5 by design).
  if (std::abs(v - expect) > 1e-3) return 1;
  return 0;
}
