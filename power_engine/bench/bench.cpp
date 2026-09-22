// pe_bench: benchmark harness (roadmap item 14a). Runs fixed fixtures,
// prints wall-clock + per-step cost + a waveform checksum. The checksum
// guards performance work: any speedup must reproduce these checksums
// (bitwise or <1e-9) — see bench/BASELINE.md. Not part of ctest (timing
// is informational); keep runtime <10s.
#include <chrono>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <string>

#include "power_engine/engine.h"

namespace {

struct Result {
  std::string name;
  long steps = 0;
  double wallMs = 0.0;
  double checksum = 0.0;
};

void report(const Result& r) {
  std::printf("%-18s steps=%-7ld wall=%8.1fms us/step=%7.3f checksum=%.12g\n",
              r.name.c_str(), r.steps, r.wallMs,
              r.wallMs * 1000.0 / static_cast<double>(r.steps), r.checksum);
}

// Open-loop buck from the README netlist sketch: 12V, 20kHz, D=0.5,
// 200uH/200uF/5ohm, 6ms @ 0.5us (12k steps).
Result buck() {
  power_engine::Engine eng;
  eng.loadNetlist(R"(
.param VIN 12 FSW 20k D 0.5
.model SW mosfet_ideal RON=5m ROFF=1Meg EON=10u EOFF=15u
.model DD diode_ideal VF=0.0 RON=10m
V1 1 0 {VIN}
S1 1 2 MODEL=SW
D1 0 2 MODEL=DD
L1 2 3 200u
C1 3 0 200u
Rload 3 0 5
.control pwm switch=S1 freq={FSW} duty={D}
.tran 0.5u 6m
.end
)");
  eng.applyPwmSpecs();
  const auto t0 = std::chrono::steady_clock::now();
  eng.start();
  long steps = 0;
  double sum = 0.0;
  while (eng.status() == power_engine::SimulationStatus::Running) {
    eng.step();
    ++steps;
    sum += eng.currentSolution().probes.at("v:3");
  }
  const auto t1 = std::chrono::steady_clock::now();
  const double ms =
      std::chrono::duration<double, std::milli>(t1 - t0).count();
  return {"buck-open-6ms", steps, ms, sum};
}

// Uncontrolled Vienna diode bridge, 60ms @ 1us (60k steps, switching).
Result vienna() {
  constexpr double kVph = 230.0, kF0 = 50.0;
  const double w = 2.0 * std::numbers::pi * kF0;
  const double vpk = kVph * std::sqrt(2.0);
  power_engine::Engine eng;
  eng.setTimeStep(1e-6);
  eng.setStopTime(60e-3);
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
  const auto t0 = std::chrono::steady_clock::now();
  eng.start();
  long steps = 0;
  double sum = 0.0;
  while (eng.status() == power_engine::SimulationStatus::Running) {
    const double t = eng.time();
    const double e =
        t >= 10e-3 ? 1.0 : 0.5 * (1.0 - std::cos(std::numbers::pi * t / 10e-3));
    eng.circuit().findDevice("VA").value = e * vpk * std::sin(w * t);
    eng.circuit().findDevice("VB").value =
        e * vpk * std::sin(w * t - 2.0 * std::numbers::pi / 3.0);
    eng.circuit().findDevice("VC").value =
        e * vpk * std::sin(w * t + 2.0 * std::numbers::pi / 3.0);
    eng.step();
    ++steps;
    sum += eng.currentSolution().probes.at("v:7");
  }
  const auto t1 = std::chrono::steady_clock::now();
  const double ms =
      std::chrono::duration<double, std::milli>(t1 - t0).count();
  return {"vienna-60ms", steps, ms, sum};
}

// RC ladder scaling probe: N rungs (1k series, 1uF shunt), 2000 steps @
// 1us, no settling needed — pure per-step scaling signal. N>=~60 crosses
// into the sparse solver path.
Result ladder(int n) {
  power_engine::Engine eng;
  eng.setTimeStep(1e-6);
  eng.setStopTime(2e-3);
  const int src = n + 1;
  eng.circuit().addVoltageSource("V1", src, 0, 10.0);
  eng.circuit().addResistor("R0", src, n, 1000.0);
  for (int k = n; k >= 1; --k) {
    eng.circuit().addCapacitor("C" + std::to_string(k), k, 0, 1e-6, 0.0);
    if (k > 1)
      eng.circuit().addResistor("R" + std::to_string(k), k, k - 1, 1000.0);
  }
  const auto t0 = std::chrono::steady_clock::now();
  eng.start();
  long steps = 0;
  double sum = 0.0;
  while (eng.status() == power_engine::SimulationStatus::Running) {
    eng.step();
    ++steps;
    // Driven end (v:n): charges on the RC timescale for every n, so the
    // checksum is a live waveform guard at all sizes (the far end v:1 is
    // still ~0 after 2ms for large n — pure diffusion).
    sum += eng.currentSolution().probes.at("v:" + std::to_string(n));
  }
  const auto t1 = std::chrono::steady_clock::now();
  const double ms =
      std::chrono::duration<double, std::milli>(t1 - t0).count();
  return {"ladder-" + std::to_string(n), steps, ms, sum};
}

}  // namespace

int main() {
  std::printf("# pe_bench (Release): fixture, steps, wall, us/step, checksum\n");
  report(buck());
  report(vienna());
  for (int n : {10, 40, 160, 320}) report(ladder(n));
  return 0;
}
