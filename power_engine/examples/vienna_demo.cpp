#include <cmath>
#include <cstdio>

#include "power_engine/engine.h"

// Vienna rectifier (uncontrolled diode-bridge mode) demo: 3x230V/50Hz with
// floating mains neutral (true 3-wire) through 5mH + 0.5ohm into a diode
// bridge with split 2mF caps + 100ohm. Prints CSV t,vdc,vphaseA,ia —
// columns pinned by power_engine/xval/xval.py FIXTURES (cross-validation
// reads "vdc"); do not rename without updating both sides.
int main() {
  constexpr double Vph = 230.0, f0 = 50.0, dt = 1e-6;
  const double w = 2.0 * std::acos(-1.0) * f0;
  const double vpk = Vph * std::sqrt(2.0);
  power_engine::Engine eng;
  eng.setTimeStep(dt);
  eng.circuit().addVoltageSource("VA", 1, 10, 0.0);
  eng.circuit().addVoltageSource("VB", 2, 10, 0.0);
  eng.circuit().addVoltageSource("VC", 3, 10, 0.0);
  eng.circuit().addResistor("RAg", 1, 11, 0.5);
  eng.circuit().addResistor("RBg", 2, 12, 0.5);
  eng.circuit().addResistor("RCg", 3, 13, 0.5);
  eng.circuit().addInductor("LA", 11, 4, 5e-3, 0.0);
  eng.circuit().addInductor("LB", 12, 5, 5e-3, 0.0);
  eng.circuit().addInductor("LC", 13, 6, 5e-3, 0.0);
  eng.circuit().addDiode("DAu", 4, 7, 0.7, 10e-3, 1e6);
  eng.circuit().addDiode("DBu", 5, 7, 0.7, 10e-3, 1e6);
  eng.circuit().addDiode("DCu", 6, 7, 0.7, 10e-3, 1e6);
  eng.circuit().addDiode("DAl", 0, 4, 0.7, 10e-3, 1e6);
  eng.circuit().addDiode("DBl", 0, 5, 0.7, 10e-3, 1e6);
  eng.circuit().addDiode("DCl", 0, 6, 0.7, 10e-3, 1e6);
  eng.circuit().addSwitch("SA", 4, 9, 5e-3, 1e6, false);
  eng.circuit().addSwitch("SB", 5, 9, 5e-3, 1e6, false);
  eng.circuit().addSwitch("SC", 6, 9, 5e-3, 1e6, false);
  eng.circuit().addCapacitor("C1", 7, 9, 2e-3, 0.0);
  eng.circuit().addCapacitor("C2", 9, 0, 2e-3, 0.0);
  eng.circuit().addResistor("Rload", 7, 0, 100.0);
  eng.setStopTime(60e-3);
  eng.start();
  std::printf("time,vdc,va,ia\n");
  while (eng.status() == power_engine::SimulationStatus::Running) {
    const double t = eng.time();
    const double e = t >= 10e-3 ? 1.0 : 0.5 * (1.0 - std::cos(3.141592653589793 * t / 10e-3));
    eng.circuit().findDevice("VA").value = e * vpk * std::sin(w * t);
    eng.circuit().findDevice("VB").value = e * vpk * std::sin(w * t - 2.0943951023931953);
    eng.circuit().findDevice("VC").value = e * vpk * std::sin(w * t + 2.0943951023931953);
    eng.step();
    const auto& s = eng.currentSolution();
    std::printf("%.9f,%.9f,%.9f,%.9f\n", s.t, s.probes.at("v:7"),
                s.probes.at("v:1") - s.probes.at("v:10"), eng.deviceCurrent("LA"));
  }
  return 0;
}
