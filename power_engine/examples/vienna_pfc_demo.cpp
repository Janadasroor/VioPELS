#include <array>
#include <cmath>
#include <cstdio>
#include <numbers>

#include "power_engine/engine.h"
#include "power_engine/vienna_pfc.h"

// Vienna PFC with neutral-point balancing demo: 3x230V/50Hz (floating
// neutral, true 3-wire) through 5mH + 0.5ohm into the Vienna stage with
// split caps, 100ohm load, 500ohm midpoint bleed. Hysteretic current
// control regulates 750Vdc at unity PF; the midpoint-weighted threshold
// shift holds node 9 at Vdc/2 (carrier current loops provably cannot —
// see vienna_pfc.h). CSV: t,vdc,vmid,ia. Metrics to stderr.
int main() {
  constexpr double kVph = 230.0, kF0 = 50.0, kDt = 1e-6, kStop = 320e-3;
  const double w = 2.0 * std::numbers::pi * kF0;
  const double vpk = kVph * std::sqrt(2.0);
  power_engine::Engine eng;
  eng.setTimeStep(kDt);
  eng.setStopTime(kStop);
  power_engine::vienna::PlantParams plant;
  power_engine::vienna::buildPlant(eng, plant);
  power_engine::vienna::HysteresisController ctl;
  eng.start();
  std::printf("time,vdc,vmid,ia\n");
  double sVdc = 0.0, sVmid = 0.0, sP = 0.0, sI2 = 0.0, n = 0.0;
  long step = 0;
  while (eng.status() == power_engine::SimulationStatus::Running) {
    const double t = eng.time();
    const double e = t >= 10e-3 ? 1.0 : 0.5 * (1.0 - std::cos(std::numbers::pi * t / 10e-3));
    const std::array<double, 3> vs = {
        e * vpk * std::sin(w * t),
        e * vpk * std::sin(w * t - 2.0 * std::numbers::pi / 3.0),
        e * vpk * std::sin(w * t + 2.0 * std::numbers::pi / 3.0),
    };
    eng.circuit().findDevice("VA").value = vs[0];
    eng.circuit().findDevice("VB").value = vs[1];
    eng.circuit().findDevice("VC").value = vs[2];
    const std::array<double, 3> im = {
        eng.deviceCurrent("LA"),
        eng.deviceCurrent("LB"),
        eng.deviceCurrent("LC"),
    };
    const double vdc = eng.currentSolution().probes.at("v:7");
    const double vmid = eng.currentSolution().probes.at("v:9");
    const auto gates = ctl.update(t, vs, im, vdc, vmid);
    eng.setSwitch("SA", gates[0]);
    eng.setSwitch("SB", gates[1]);
    eng.setSwitch("SC", gates[2]);
    eng.step();
    // NOTE: probes/device currents are pre-step above (control law uses
    // the previous step's solution); post-step sampling for metrics:
    const double vdc2 = eng.currentSolution().probes.at("v:7");
    const double vmid2 = eng.currentSolution().probes.at("v:9");
    const double ia2 = eng.deviceCurrent("LA");
    const double ib2 = eng.deviceCurrent("LB");
    const double ic2 = eng.deviceCurrent("LC");
    if (step % 10 == 0) {
      std::printf("%.9f,%.9f,%.9f,%.9f\n", eng.time(), vdc2, vmid2, ia2);
    }
    if (eng.time() > 260e-3) {
      sVdc += vdc2;
      sVmid += vmid2;
      sP += vs[0] * ia2 + vs[1] * ib2 + vs[2] * ic2;
      sI2 += (ia2 * ia2 + ib2 * ib2 + ic2 * ic2) / 3.0;
      n += 1.0;
    }
    ++step;
  }
  const double mVdc = sVdc / n, mVmid = sVmid / n;
  const double irms = std::sqrt(sI2 / n);
  const double pf = (sP / n) / (3.0 * kVph * irms);
  std::fprintf(stderr, "metrics: Vdc=%.2f Vmid=%.2f dVmid=%.2f PF=%.4f Irms=%.2f\n",
               mVdc, mVmid, mVmid - mVdc / 2.0, pf, irms);
  return 0;
}
