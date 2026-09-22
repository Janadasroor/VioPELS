#include "power_engine/emi.h"

#include <cmath>
#include <stdexcept>

namespace power_engine {
namespace emi {

std::vector<EmissionLine> peakTable(const measurements::Trace& portV, double fsw,
                                    int maxHarmonic) {
  if (!(fsw > 0.0) || !std::isfinite(fsw)) throw std::runtime_error("peakTable needs fsw > 0");
  if (maxHarmonic < 1) throw std::runtime_error("peakTable needs maxHarmonic >= 1");
  std::vector<EmissionLine> out;
  for (const auto& h : measurements::spectrum(portV, fsw, maxHarmonic)) {
    if (h.order == 0) continue;  // DC is not an emission line
    EmissionLine e;
    e.freqHz = h.freqHz;
    e.dbuv = toDbuV(h.mag);
    out.push_back(e);
  }
  return out;
}

double classBLimitQp(double freqHz) {
  if (!(freqHz > 0.0) || !std::isfinite(freqHz))
    throw std::runtime_error("classBLimitQp needs freqHz > 0");
  if (freqHz < 150e3 || freqHz > 30e6)
    throw std::runtime_error("classBLimitQp defined on 150kHz..30MHz only");
  if (freqHz < 500e3) {
    // 66 dBuV at 150k down to 56 at 500k, log-linear.
    const double u =
        std::log(freqHz / 150e3) / std::log(500e3 / 150e3);
    return 66.0 - 10.0 * u;
  }
  if (freqHz < 5e6) return 56.0;
  return 60.0;
}

LimitCheck checkClassB(const std::vector<EmissionLine>& peaks) {
  if (peaks.empty()) throw std::runtime_error("checkClassB needs >= 1 peak");
  LimitCheck r;
  bool first = true;
  for (const auto& p : peaks) {
    const double m = classBLimitQp(p.freqHz) - p.dbuv;
    if (first || m < r.worstMarginDb) {
      r.worstMarginDb = m;
      r.worstFreqHz = p.freqHz;
      first = false;
    }
  }
  r.pass = r.worstMarginDb >= 0.0;
  return r;
}

}  // namespace emi
}  // namespace power_engine
