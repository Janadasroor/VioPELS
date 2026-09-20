#include <cstdio>

#include "power_engine/engine.h"

// Minimal RC demo: prints CSV time,vc to stdout.
int main() {
  power_engine::Engine eng;
  eng.setTimeStep(1e-6);
  eng.circuit().addVoltageSource("V1", 1, 0, 1.0);
  eng.circuit().addResistor("R1", 1, 2, 1000.0);
  eng.circuit().addCapacitor("C1", 2, 0, 1e-6, 0.0);
  eng.setStopTime(5e-3);
  eng.start();
  std::printf("time,vc\n");
  while (eng.status() == power_engine::SimulationStatus::Running) {
    eng.step();
    const auto& s = eng.currentSolution();
    std::printf("%.9f,%.9f\n", s.t, s.probes.at("v:2"));
  }
  return 0;
}
