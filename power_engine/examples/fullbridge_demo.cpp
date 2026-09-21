#include <cmath>
#include <cstdio>

#include "power_engine/engine.h"

// Full-bridge demo: unipolar SPWM (m=0.8, 50Hz) into LC + load.
// Prints CSV t,vout_diff,vlegA,vlegB.
int main() {
  constexpr double Vdc = 48.0, m = 0.8, f0 = 50.0, T = 100e-6, dt = 1e-6;
  power_engine::Engine eng;
  eng.setTimeStep(dt);
  eng.circuit().addVoltageSource("Vdc", 5, 0, Vdc);
  eng.circuit().addSwitch("S1", 5, 1, 5e-3, 1e6, false);
  eng.circuit().addSwitch("S2", 1, 0, 5e-3, 1e6, true);
  eng.circuit().addSwitch("S3", 5, 2, 5e-3, 1e6, false);
  eng.circuit().addSwitch("S4", 2, 0, 5e-3, 1e6, true);
  eng.circuit().addDiode("D1", 1, 5, 0.0, 10e-3, 1e6);
  eng.circuit().addDiode("D2", 0, 1, 0.0, 10e-3, 1e6);
  eng.circuit().addDiode("D3", 2, 5, 0.0, 10e-3, 1e6);
  eng.circuit().addDiode("D4", 0, 2, 0.0, 10e-3, 1e6);
  eng.circuit().addInductor("L1", 1, 3, 2e-3, 0.0);
  eng.circuit().addCapacitor("C1", 3, 2, 10e-6, 0.0);
  eng.circuit().addResistor("Rload", 3, 2, 20.0);
  eng.setStopTime(40e-3);
  eng.start();
  std::printf("time,vout,va,vb\n");
  while (eng.status() == power_engine::SimulationStatus::Running) {
    const double t = eng.time();
    const double w = 2.0 * std::acos(-1.0) * f0;
    const double da = 0.5 + 0.5 * m * std::sin(w * t);
    const double db = 0.5 - 0.5 * m * std::sin(w * t);
    const double tk = std::floor(t / T) * T;
    eng.setSwitch("S1", t - tk < da * T);
    eng.setSwitch("S2", t - tk >= da * T);
    eng.setSwitch("S3", t - tk < db * T);
    eng.setSwitch("S4", t - tk >= db * T);
    eng.step();
    const auto& s = eng.currentSolution();
    std::printf("%.9f,%.9f,%.9f,%.9f\n", s.t, s.probes.at("v:3") - s.probes.at("v:2"),
                s.probes.at("v:1"), s.probes.at("v:2"));
  }
  return 0;
}
