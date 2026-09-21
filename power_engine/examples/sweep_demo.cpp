#include <cstdio>
#include <map>
#include <memory>
#include <string>

#include "power_engine/engine.h"
#include "power_engine/sweep.h"

// Efficiency-vs-load sweep demo: buck over RLOAD grid, prints the CSV
// results table (vout, switching + conduction energies per point).
int main() {
  const char* netlist = R"(
.param RLOAD 5
.tran 0.5u 6m
V1 1 0 12
S1 1 2 RON=5m ROFF=1Meg EON=10u EOFF=15u INIT=ON
D1 0 2 VF=0.7 RON=10m
L1 2 3 200u
C1 3 0 200u
Rload 3 0 {RLOAD}
.control pwm switch=S1 freq=20k duty=0.5
.end
)";
  struct Acc {
    double sum = 0.0, n = 0.0;
  };
  auto acc = std::make_shared<Acc>();
  power_engine::sweep::SweepConfig cfg;
  cfg.netlist = netlist;
  cfg.axes = {{"RLOAD", {2.5, 5.0, 10.0, 20.0}}};
  cfg.setup = [acc](power_engine::Engine& eng, const std::map<std::string, double>&) {
    acc->sum = 0.0;
    acc->n = 0.0;
    eng.applyPwmSpecs();
    eng.setCallback([acc](const power_engine::Solution& s) {
      if (s.t > 5e-3) {
        acc->sum += s.probes.at("v:3");
        acc->n += 1.0;
      }
    });
  };
  cfg.measure = [acc](power_engine::Engine& eng, const std::map<std::string, double>&) {
    return std::map<std::string, double>{
        {"vout", acc->sum / acc->n},
        {"esw", eng.deviceLoss("S1").esw},
        {"econd", eng.deviceLoss("S1").econd + eng.deviceLoss("D1").econd}};
  };
  std::printf("%s", power_engine::sweep::runSweep(cfg).csv().c_str());
  return 0;
}
