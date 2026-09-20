#include <cmath>
#include <cstdio>

#include "power_engine/control.h"
#include "power_engine/engine.h"

// Closed-loop buck with electro-thermal switch model.
// CSV: time,vout,duty,tj_s1,econd_s1,esw_s1
int main() {
  constexpr double Vin = 12.0;
  constexpr double freq = 20e3;
  constexpr double T = 1.0 / freq;
  constexpr double dt = 1e-6;
  constexpr double Vref = 5.0;
  constexpr double tStop = 40e-3;

  power_engine::Engine eng;
  eng.setTimeStep(dt);
  eng.circuit().addVoltageSource("Vin", 1, 0, Vin);
  eng.circuit().addSwitch("S1", 1, 2, 5e-3, 1e6, true, 10e-6, 15e-6);
  eng.circuit().addDiode("D1", 0, 2, 0.0, 10e-3, 1e6);
  eng.circuit().addInductor("L1", 2, 3, 200e-6, 0.0);
  eng.circuit().addCapacitor("C1", 3, 0, 200e-6, 0.0);
  eng.circuit().addResistor("Rload", 3, 0, 5.0);
  eng.setStopTime(tStop);
  eng.attachThermal("S1", power_engine::thermal::ThermalNetwork::foster(
                               {{0.5, 0.01}, {1.0, 0.1}}, 25.0));

  power_engine::control::PiController pi(0.07, 40.0, 0.02, 0.95);
  double duty = 0.4;
  eng.scheduleSwitch("S1", true, 0.0);
  eng.scheduleSwitch("S1", false, duty * T);
  double nextTick = T;

  eng.start();
  std::printf("time,vout,duty,tj_s1,econd_s1,esw_s1\n");
  while (eng.status() == power_engine::SimulationStatus::Running) {
    eng.step();
    const double t = eng.time();
    const double vout = eng.currentSolution().probes.at("v:3");
    if (t >= nextTick - 1e-12) {
      duty = pi.update(Vref - vout, T);
      eng.scheduleSwitch("S1", true, nextTick);
      eng.scheduleSwitch("S1", false, nextTick + duty * T);
      nextTick += T;
    }
    const auto loss = eng.deviceLoss("S1");
    std::printf("%.9f,%.9f,%.6f,%.6f,%.9f,%.9f\n", t, vout, duty,
                eng.junctionTemp("S1"), loss.econd, loss.esw);
  }
  return 0;
}
