// Conducted-emissions demo: buck + LISN + PAT edges, with and without an
// input LC filter. Prints CSV freq_hz,dbuv_unfiltered,dbuv_filtered over
// the in-band switching harmonics (>= 150kHz).
#include <cstdio>
#include <vector>

#include "power_engine/emi.h"
#include "power_engine/engine.h"

namespace {
std::vector<power_engine::emi::EmissionLine> run(bool filtered) {
  power_engine::Engine eng;
  eng.setTimeStep(100e-9);
  eng.setStopTime(500e-6);
  eng.circuit().addVoltageSource("V1", 1, 0, 12.0);
  power_engine::emi::addDcLisn(eng, "LISN", 1, 2, 20);
  int rail = 2;
  if (filtered) {
    eng.circuit().addInductor("LF", 2, 3, 1e-3, 0.0);
    eng.circuit().addCapacitor("CF", 3, 4, 10e-6, 12.0);
    eng.circuit().addResistor("RD", 4, 0, 0.5);
    rail = 3;
  }
  eng.circuit().addSwitch("S1", rail, 5, 5e-3, 1e6, true, 0.0, 0.0, 0.0, 0.1, 500e-9);
  eng.circuit().addDiode("D1", 0, 5, 0.7, 10e-3, 1e6);
  eng.circuit().addInductor("L1", 5, 6, 200e-6, 1.2);
  eng.circuit().addCapacitor("C1", 6, 0, 200e-6, 6.0);
  eng.circuit().addResistor("Rload", 6, 0, 5.0);
  eng.start();
  constexpr double kT = 1.0 / 20e3;
  power_engine::measurements::Trace tr;
  while (eng.status() == power_engine::SimulationStatus::Running) {
    const double ph = std::fmod(eng.time(), kT) / kT;
    eng.setSwitch("S1", ph < 0.5);
    eng.step();
    if (eng.time() > 300e-6) {
      tr.t.push_back(eng.time());
      tr.y.push_back(eng.currentSolution().probes.at("v:20"));
    }
  }
  auto all = power_engine::emi::peakTable(tr, 20e3, 100);
  std::vector<power_engine::emi::EmissionLine> peaks;
  for (const auto& p : all)
    if (p.freqHz >= 150e3) peaks.push_back(p);
  return peaks;
}
}  // namespace

int main() {
  const auto nofilt = run(false);
  const auto filt = run(true);
  std::printf("freq_hz,dbuv_unfiltered,dbuv_filtered\n");
  for (std::size_t k = 0; k < nofilt.size() && k < filt.size(); ++k) {
    std::printf("%.1f,%.4f,%.4f\n", nofilt[k].freqHz, nofilt[k].dbuv, filt[k].dbuv);
  }
  const auto chk = power_engine::emi::checkClassB(filt);
  std::fprintf(stderr, "filtered worst margin: %.1fdB at %.0fkHz\n", chk.worstMarginDb,
               chk.worstFreqHz / 1e3);
  return 0;
}
