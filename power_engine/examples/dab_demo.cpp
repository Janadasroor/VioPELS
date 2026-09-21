#include <cmath>
#include <cstdio>

#include "power_engine/engine.h"

// Dual-active-bridge demo: 48V/48V, n=1, Lk=100uH, 20kHz, +30deg shift.
// Prints CSV t,ilk,vpri,vsec (last 1ms).
int main() {
  constexpr double V = 48.0, kT = 50e-6, kLk = 100e-6, dt = 0.5e-6;
  constexpr double kPi = 3.141592653589793;
  constexpr double shift = (kPi / 6.0) * kT / (2.0 * kPi);  // +30deg
  power_engine::Engine eng;
  eng.setTimeStep(dt);
  eng.circuit().addVoltageSource("V1", 1, 0, V);
  eng.circuit().addCapacitor("C1", 1, 0, 200e-6, V);
  eng.circuit().addCapacitor("C2", 4, 0, 200e-6, V);
  eng.circuit().addResistor("R2", 4, 0, 30.0);
  eng.circuit().addSwitch("S1", 1, 2, 5e-3, 1e6, true);
  eng.circuit().addSwitch("S2", 2, 0, 5e-3, 1e6, false);
  eng.circuit().addSwitch("S3", 1, 3, 5e-3, 1e6, false);
  eng.circuit().addSwitch("S4", 3, 0, 5e-3, 1e6, true);
  eng.circuit().addInductor("Lk", 2, 7, kLk, 0.0);
  eng.circuit().addTransformer("T1", 7, 3, 5, 6, 1.0);
  eng.circuit().addSwitch("S5", 4, 5, 5e-3, 1e6, false);
  eng.circuit().addSwitch("S6", 5, 0, 5e-3, 1e6, true);
  eng.circuit().addSwitch("S7", 4, 6, 5e-3, 1e6, true);
  eng.circuit().addSwitch("S8", 6, 0, 5e-3, 1e6, false);
  constexpr double kStop = 5e-3;
  for (double tk = 0.0; tk < kStop; tk += kT) {
    eng.scheduleSwitch("S1", true, tk);
    eng.scheduleSwitch("S1", false, tk + 0.5 * kT);
    eng.scheduleSwitch("S4", true, tk);
    eng.scheduleSwitch("S4", false, tk + 0.5 * kT);
    eng.scheduleSwitch("S2", false, tk);
    eng.scheduleSwitch("S2", true, tk + 0.5 * kT);
    eng.scheduleSwitch("S3", false, tk);
    eng.scheduleSwitch("S3", true, tk + 0.5 * kT);
    eng.scheduleSwitch("S5", true, tk + shift);
    eng.scheduleSwitch("S5", false, tk + shift + 0.5 * kT);
    eng.scheduleSwitch("S8", true, tk + shift);
    eng.scheduleSwitch("S8", false, tk + shift + 0.5 * kT);
    eng.scheduleSwitch("S6", false, tk + shift);
    eng.scheduleSwitch("S6", true, tk + shift + 0.5 * kT);
    eng.scheduleSwitch("S7", false, tk + shift);
    eng.scheduleSwitch("S7", true, tk + shift + 0.5 * kT);
  }
  eng.setStopTime(kStop);
  eng.start();
  std::printf("time,ilk,vpri,vsec\n");
  long long n = 0;
  while (eng.status() == power_engine::SimulationStatus::Running) {
    eng.step();
    if (eng.time() >= 4e-3 && (n++ % 5 == 0)) {
      const auto& s = eng.currentSolution();
      std::printf("%.9f,%.9f,%.9f,%.9f\n", s.t, eng.deviceCurrent("Lk"),
                  s.probes.at("v:2") - s.probes.at("v:3"),
                  s.probes.at("v:5") - s.probes.at("v:6"));
    }
  }
  return 0;
}
