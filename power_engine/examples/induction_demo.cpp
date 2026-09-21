#include <cmath>
#include <cstdio>

#include "power_engine/machine.h"

// Induction run-up demo: V/f (230Vrms/50Hz) from standstill, no load.
// Prints CSV t,omega_mech,slip,torque.
int main() {
  constexpr double kPi = 3.141592653589793;
  const power_engine::machine::InductionParams m{1.0, 0.8, 0.15, 0.15, 0.14, 2,
                                                 {0.05, 0.002}};
  const double we = 2.0 * kPi * 50.0, vpk = 230.0 * std::sqrt(2.0), dt = 50e-6;
  power_engine::machine::InductionState st;
  power_engine::machine::MechanicalState rotor{0.0, 0.0};
  std::printf("time,omega,slip,torque\n");
  for (double t = 0.0; t < 1.5; t += dt) {
    power_engine::machine::stepInduction(st, 0.0, vpk, we,
                                        m.polePairs * rotor.omega, m, dt);
    double ids = 0.0, iqs = 0.0, idr = 0.0, iqr = 0.0;
    power_engine::machine::inductionCurrents(st, m, ids, iqs, idr, iqr);
    const double te = power_engine::machine::inductionTorque(st, ids, iqs, m);
    power_engine::machine::stepMechanical(rotor, te, 0.0, m.mech, dt);
    const double slip = (we - m.polePairs * rotor.omega) / we;
    std::printf("%.6f,%.6f,%.6f,%.6f\n", t, rotor.omega, slip, te);
  }
  return 0;
}
