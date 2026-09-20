#include <cmath>
#include <cstdio>

#include "power_engine/engine.h"

// Open-loop buck demo: prints CSV t,vout,il,switch_node.
int main() {
  constexpr double Vin = 12.0;
  constexpr double freq = 20e3;
  constexpr double period = 1.0 / freq;
  constexpr double duty = 0.5;
  constexpr double dt = 0.5e-6;

  power_engine::Engine eng;
  eng.setTimeStep(dt);
  eng.circuit().addVoltageSource("Vin", 1, 0, Vin);
  eng.circuit().addSwitch("S1", 1, 2, 5e-3, 1e6, true);
  eng.circuit().addDiode("D1", 0, 2, 0.0, 10e-3, 1e6);
  eng.circuit().addInductor("L1", 2, 3, 200e-6, 0.0);
  eng.circuit().addCapacitor("C1", 3, 0, 200e-6, 0.0);
  eng.circuit().addResistor("Rload", 3, 0, 5.0);
  eng.setStopTime(6e-3);
  eng.start();
  std::printf("time,vout,il,vsw\n");
  while (eng.status() == power_engine::SimulationStatus::Running) {
    const double tNext = eng.currentSolution().t + dt;
    eng.setSwitch("S1", std::fmod(tNext, period) < duty * period);
    eng.step();
    const auto& s = eng.currentSolution();
    const double il = eng.circuit().findDevice("L1").i_prev;
    std::printf("%.9f,%.9f,%.9f,%.9f\n", s.t, s.probes.at("v:3"), il,
                s.probes.at("v:2"));
  }
  return 0;
}
