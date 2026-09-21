#include <cmath>
#include <cstdio>

#include "power_engine/engine.h"

// LLC resonant demo: full-bridge 48V at resonance (~50kHz), Lr=20uH,
// Cr=500nF, Lm=100uH (k=0.98), diode bridge + 200uF + 10ohm.
// Prints CSV t,vout,ires.
int main() {
  constexpr double Vin = 48.0, kLr = 20e-6, kCr = 500e-9, kLm = 100e-6;
  constexpr double kPi = 3.141592653589793;
  const double fr = 1.0 / (2.0 * kPi * std::sqrt(kLr * kCr));
  const double T = 1.0 / fr, dt = 0.2e-6;
  power_engine::Engine eng;
  eng.setTimeStep(dt);
  eng.circuit().addVoltageSource("Vin", 1, 0, Vin);
  eng.circuit().addSwitch("S1", 1, 2, 5e-3, 1e6, true);
  eng.circuit().addSwitch("S2", 2, 0, 5e-3, 1e6, false);
  eng.circuit().addSwitch("S3", 1, 3, 5e-3, 1e6, false);
  eng.circuit().addSwitch("S4", 3, 0, 5e-3, 1e6, true);
  eng.circuit().addInductor("Lr", 2, 4, kLr, 0.0);
  eng.circuit().addCapacitor("Cr", 4, 5, kCr, 0.0);
  eng.circuit().addCoupledInductors("T1", 5, 3, 6, 7, kLm, kLm, 0.98);
  eng.circuit().addDiode("D1", 6, 8, 0.0, 10e-3, 1e6);
  eng.circuit().addDiode("D2", 7, 8, 0.0, 10e-3, 1e6);
  eng.circuit().addDiode("D3", 0, 6, 0.0, 10e-3, 1e6);
  eng.circuit().addDiode("D4", 0, 7, 0.0, 10e-3, 1e6);
  eng.circuit().addCapacitor("Cout", 8, 0, 200e-6, 0.0);
  eng.circuit().addResistor("Rload", 8, 0, 10.0);
  constexpr double kStop = 3e-3;
  for (double tk = 0.0; tk < kStop; tk += T) {
    eng.scheduleSwitch("S1", true, tk);
    eng.scheduleSwitch("S1", false, tk + 0.5 * T);
    eng.scheduleSwitch("S4", true, tk);
    eng.scheduleSwitch("S4", false, tk + 0.5 * T);
    eng.scheduleSwitch("S2", false, tk);
    eng.scheduleSwitch("S2", true, tk + 0.5 * T);
    eng.scheduleSwitch("S3", false, tk);
    eng.scheduleSwitch("S3", true, tk + 0.5 * T);
  }
  eng.setStopTime(kStop);
  eng.start();
  std::printf("time,vout,ires\n");
  long long n = 0;
  while (eng.status() == power_engine::SimulationStatus::Running) {
    eng.step();
    if (eng.time() >= 2e-3 && (n++ % 5 == 0)) {
      const auto& s = eng.currentSolution();
      std::printf("%.9f,%.9f,%.9f\n", s.t, s.probes.at("v:8"),
                  eng.deviceCurrent("Lr"));
    }
  }
  return 0;
}
