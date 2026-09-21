#include <cmath>
#include <cstdio>

#include "power_engine/control.h"
#include "power_engine/engine.h"
#include "power_engine/machine.h"

// PMSM speed-drive demo: 24V inverter, hysteretic id=0 currents from a
// speed PI + velocity feedforward, surface PMSM (p=2). Prints CSV
// t,omega,iq,torque.
int main() {
  constexpr double kVdc = 24.0, kImax = 2.0, kWref = 100.0;
  constexpr double kP = 2.0, kLam = 0.05, kRs = 0.5, kLs = 2e-3;
  constexpr double kJ = 5e-5, kB = 5e-4, kDt = 1e-6, kTq = 1.5 * kP * kLam;
  constexpr double kPi = 3.141592653589793;
  power_engine::Engine eng;
  eng.setTimeStep(kDt);
  eng.circuit().addVoltageSource("Vdc", 7, 0, kVdc);
  eng.circuit().addSwitch("SAh", 7, 1, 5e-3, 1e6, false);
  eng.circuit().addSwitch("SAl", 1, 0, 5e-3, 1e6, true);
  eng.circuit().addSwitch("SBh", 7, 2, 5e-3, 1e6, false);
  eng.circuit().addSwitch("SBl", 2, 0, 5e-3, 1e6, true);
  eng.circuit().addSwitch("SCh", 7, 3, 5e-3, 1e6, false);
  eng.circuit().addSwitch("SCl", 3, 0, 5e-3, 1e6, true);
  eng.circuit().addResistor("RA", 1, 4, kRs);
  eng.circuit().addInductor("LA", 4, 6, kLs, 0.0);
  eng.circuit().addVoltageSource("EA", 6, 5, 0.0);
  eng.circuit().addResistor("RB", 2, 10, kRs);
  eng.circuit().addInductor("LB", 10, 11, kLs, 0.0);
  eng.circuit().addVoltageSource("EB", 11, 5, 0.0);
  eng.circuit().addResistor("RC", 3, 12, kRs);
  eng.circuit().addInductor("LC", 12, 13, kLs, 0.0);
  eng.circuit().addVoltageSource("EC", 13, 5, 0.0);
  eng.setStopTime(150e-3);
  power_engine::control::PiController speedPi(0.0067, 0.22, 0.0, kImax);
  power_engine::control::HysteresisController hcA(0.0, 0.15, false);
  power_engine::control::HysteresisController hcB(0.0, 0.15, false);
  power_engine::control::HysteresisController hcC(0.0, 0.15, false);
  const power_engine::machine::MechanicalParams mech{kJ, kB};
  power_engine::machine::MechanicalState rotor{0.0, 0.0};
  const power_engine::machine::PmsmParams mpm{2, kLam, kRs, kLs, kLs, mech};
  eng.start();
  std::printf("time,omega,iq,torque\n");
  long long n = 0;
  while (eng.status() == power_engine::SimulationStatus::Running) {
    const double t = eng.time();
    const auto e = power_engine::machine::pmsmEmf(rotor.theta, rotor.omega, mpm);
    eng.circuit().findDevice("EA").value = e.a;
    eng.circuit().findDevice("EB").value = e.b;
    eng.circuit().findDevice("EC").value = e.c;
    eng.step();
    const double ia = eng.deviceCurrent("LA");
    const double ib = eng.deviceCurrent("LB");
    const double ic = eng.deviceCurrent("LC");
    const double wref = std::min(kWref, kWref * t / 50e-3);
    const double alpha = t < 50e-3 ? kWref / 50e-3 : 0.0;
    double amp = speedPi.update(wref - rotor.omega, kDt) + (kJ * alpha + kB * wref) / kTq;
    amp = std::min(std::max(amp, 0.0), kImax);
    const double thE = kP * rotor.theta;
    hcA.setRef(amp * std::sin(thE));
    hcB.setRef(amp * std::sin(thE - 2.0 * kPi / 3.0));
    hcC.setRef(amp * std::sin(thE + 2.0 * kPi / 3.0));
    eng.setSwitch("SAh", hcA.update(ia));
    eng.setSwitch("SAl", !hcA.state());
    eng.setSwitch("SBh", hcB.update(ib));
    eng.setSwitch("SBl", !hcB.state());
    eng.setSwitch("SCh", hcC.update(ic));
    eng.setSwitch("SCl", !hcC.state());
    double id = 0.0, iq = 0.0;
    power_engine::machine::park(ia, ib, ic, thE, id, iq);
    const double te = power_engine::machine::pmsmTorque(id, iq, mpm);
    power_engine::machine::stepMechanical(rotor, te, 0.0, mech, kDt);
    if (t >= 100e-3 && (n++ % 20 == 0)) {
      std::printf("%.9f,%.9f,%.9f,%.9f\n", t, rotor.omega, iq, te);
    }
  }
  return 0;
}
