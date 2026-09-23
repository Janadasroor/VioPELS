#include <cstdio>

#include "power_engine/engine.h"

// Netlist-driven open-loop buck with thermal model (netlist -> engine -> CSV).
// CSV: time,vout,tj_s1 — columns pinned by power_engine/xval/xval.py
// FIXTURES (cross-validation reads "vout"); do not rename without updating
// both sides.
int main() {
  power_engine::Engine eng;
  eng.loadNetlist(R"(
.model SW mosfet_ideal RON=5m ROFF=1Meg EON=10u EOFF=15u
.model DD diode_ideal VF=0.0 RON=10m
V1 1 0 12
S1 1 2 MODEL=SW
D1 0 2 MODEL=DD
L1 2 3 200u
C1 3 0 200u
Rload 3 0 5
.control pwm switch=S1 freq=20k duty=0.5
.thermal foster device=S1 R1=0.5 C1=0.01 R2=1.0 C2=0.1 Tamb=25
.tran 0.5u 6m
.end
)");
  eng.applyPwmSpecs();
  eng.applyThermalSpecs();
  eng.start();
  std::printf("time,vout,tj_s1\n");
  while (eng.status() == power_engine::SimulationStatus::Running) {
    eng.step();
    const auto& s = eng.currentSolution();
    std::printf("%.9f,%.9f,%.6f\n", s.t, s.probes.at("v:3"),
                s.probes.at("tj:S1"));
  }
  const auto loss = eng.deviceLoss("S1");
  std::printf("# econd=%.9fJ esw=%.9fJ diode_events=%lld\n", loss.econd, loss.esw,
              static_cast<long long>(eng.solverStats().diodeEvents));
  return 0;
}
