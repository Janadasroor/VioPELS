#include <cmath>
#include <cstdio>
#include <numbers>

#include "power_engine/ac.h"
#include "power_engine/engine.h"

// RC Bode sweep demo (source modulation + Fourier correlation).
// CSV: freqHz,magDb,phaseDeg
int main() {
  constexpr double kPi = std::numbers::pi;
  constexpr double kR = 1000.0, kC = 1e-6;
  for (double f : power_engine::ac::logSweep(10.0, 10000.0, 3)) {
    const double dt = std::min(5e-6, 0.01 / f);  // oversample: f*dt <= 0.01
    const double w = 2.0 * kPi * f, T = 1.0 / f;
    power_engine::Engine eng;
    eng.setTimeStep(dt);
    eng.circuit().addVoltageSource("V1", 1, 0, 1.0);
    eng.circuit().addResistor("R1", 1, 2, kR);
    eng.circuit().addCapacitor("C1", 2, 0, kC, 1.0);
    eng.setStopTime(1e9);
    eng.start();
    auto drive = [&](double t) {
      eng.circuit().findDevice("V1").value = 1.0 + 0.1 * std::sin(w * t);
    };
    const long long nS = static_cast<long long>((5.0 * kR * kC + 2.0 * T) / dt);
    for (long long k = 0; k < nS; ++k) {
      drive(eng.time());
      eng.step();
    }
    power_engine::ac::FourierMeter m;
    m.begin(f);
    double tPrev = eng.time();
    double yPrev = eng.currentSolution().probes.at("v:2");
    const long long nM = static_cast<long long>(3.0 * T / dt);
    for (long long k = 0; k < nM; ++k) {
      drive(eng.time());
      eng.step();
      const double t = eng.time();
      const double u = eng.circuit().findDevice("V1").value;
      const double y = eng.currentSolution().probes.at("v:2");
      m.sample(tPrev, t, u, 0.5 * (yPrev + y));
      tPrev = t;
      yPrev = y;
    }
    eng.stop();
    const auto p = m.result();
    if (f == 10.0) std::printf("freqHz,magDb,phaseDeg\n");
    std::printf("%.6f,%.6f,%.6f\n", p.freqHz, p.magDb, p.phaseDeg);
  }
  return 0;
}
