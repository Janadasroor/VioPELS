// PMSM field-oriented control demo: voltage-form cascaded speed + dq
// current loops (id* = 0) on a 20kHz carrier driving the 24V bridge.
// CSV: time,omega,iq,id,vq.
#include <cmath>
#include <cstdio>
#include <numbers>

#include "power_engine/engine.h"
#include "power_engine/machine.h"

int main() {
  constexpr double kDt = 1e-6, kVdc = 24.0, kWref = 100.0;
  power_engine::Engine eng;
  eng.setTimeStep(kDt);
  eng.circuit().addVoltageSource("Vdc", 7, 0, kVdc);
  eng.circuit().addSwitch("SAh", 7, 1, 5e-3, 1e6, false);
  eng.circuit().addSwitch("SAl", 1, 0, 5e-3, 1e6, true);
  eng.circuit().addSwitch("SBh", 7, 2, 5e-3, 1e6, false);
  eng.circuit().addSwitch("SBl", 2, 0, 5e-3, 1e6, true);
  eng.circuit().addSwitch("SCh", 7, 3, 5e-3, 1e6, false);
  eng.circuit().addSwitch("SCl", 3, 0, 5e-3, 1e6, true);
  eng.circuit().addResistor("RA", 1, 4, 0.5);
  eng.circuit().addInductor("LA", 4, 6, 2e-3, 0.0);
  eng.circuit().addVoltageSource("EA", 6, 5, 0.0);
  eng.circuit().addResistor("RB", 2, 10, 0.5);
  eng.circuit().addInductor("LB", 10, 11, 2e-3, 0.0);
  eng.circuit().addVoltageSource("EB", 11, 5, 0.0);
  eng.circuit().addResistor("RC", 3, 12, 0.5);
  eng.circuit().addInductor("LC", 12, 13, 2e-3, 0.0);
  eng.circuit().addVoltageSource("EC", 13, 5, 0.0);
  eng.setStopTime(150e-3);
  power_engine::machine::FocParams fp;
  fp.motor = {2, 0.05, 0.5, 2e-3, 2e-3, {5e-5, 5e-4}};
  fp.vdc = kVdc;
  power_engine::machine::FocController foc(fp);
  power_engine::machine::MechanicalState rotor{0.0, 0.0};
  const power_engine::machine::MechanicalParams mech{5e-5, 5e-4};
  eng.start();
  std::printf("time,omega,iq,id,vq\n");
  while (eng.status() == power_engine::SimulationStatus::Running) {
    const double t = eng.time();
    const auto e = power_engine::machine::pmsmEmf(rotor.theta, rotor.omega, fp.motor);
    eng.circuit().findDevice("EA").value = e.a;
    eng.circuit().findDevice("EB").value = e.b;
    eng.circuit().findDevice("EC").value = e.c;
    eng.step();
    power_engine::machine::ThreePhase im{eng.deviceCurrent("LA"), eng.deviceCurrent("LB"),
                                         eng.deviceCurrent("LC")};
    const double wref = std::min(kWref, kWref * t / 50e-3);
    const double alpha = t < 50e-3 ? kWref / 50e-3 : 0.0;
    const auto g = foc.update(t, wref, alpha, rotor.omega, im, 2.0 * rotor.theta, kDt);
    eng.setSwitch("SAh", g.aHi);
    eng.setSwitch("SAl", !g.aHi);
    eng.setSwitch("SBh", g.bHi);
    eng.setSwitch("SBl", !g.bHi);
    eng.setSwitch("SCh", g.cHi);
    eng.setSwitch("SCl", !g.cHi);
    power_engine::machine::stepMechanical(
        rotor, power_engine::machine::pmsmTorque(foc.id(), foc.iq(), fp.motor), 0.0, mech,
        kDt);
    if (static_cast<long>(t * 1e6) % 10 == 0) {
      std::printf("%.9f,%.9f,%.9f,%.9f,%.9f\n", t, rotor.omega, foc.iq(), foc.id(),
                  foc.vq());
    }
  }
  return 0;
}
