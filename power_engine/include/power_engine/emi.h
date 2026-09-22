#pragma once
#include <cmath>
#include <string>
#include <vector>

#include "power_engine/engine.h"
#include "power_engine/measurements.h"

namespace power_engine {
namespace emi {

// Conducted-emissions module (roadmap item 18): LISN + peak spectrum at
// switching harmonics + CISPR 32 Class B screening.
//
// Scope (documented limits): single-port (DM-ish) measurement on DC or
// mains input; stationary periodic noise only (coherent DFT per bin — no
// max-hold; bursty/non-stationary emissions need a time-domain detector,
// deferred); peak-vs-QP-limit comparison is pre-compliance SCREENING
// (formal QP needs the CISPR 16 detector, deferred). Analysis band is set
// by the sim: need dt << tsw (resolve PAT edges) and window >> 1/RBW.

/// DC LISN (CISPR-25-style 5uH/50ohm): series L from the source to the
/// DUT, AC-coupled 50ohm measurement port to ground (no DC loading).
/// Caller provides a free port node id. Must be added pre-start.
inline void addDcLisn(Engine& eng, const std::string& prefix, int srcNode, int dutNode,
                      int portNode, double l = 5e-6, double rPort = 50.0,
                      double cBlock = 100e-9) {
  eng.circuit().addInductor(prefix + "L", srcNode, dutNode, l, 0.0);
  eng.circuit().addCapacitor(prefix + "C", dutNode, portNode, cBlock, 0.0);
  eng.circuit().addResistor(prefix + "R", portNode, 0, rPort);
}

/// Peak amplitude [V] -> dBuV.
inline double toDbuV(double peakVolts) {
  return 20.0 * std::log10(peakVolts / 1e-6);
}

/// One emission line: switching harmonic k*fsw with peak level.
struct EmissionLine {
  double freqHz = 0.0;
  double dbuv = 0.0;
};
/// Peak table at switching harmonics 1..maxHarmonic (skips DC): coherent
/// DFT magnitudes via measurements::spectrum (exact for stationary line
/// spectra — no scalloping since bins sit on the lines).
std::vector<EmissionLine> peakTable(const measurements::Trace& portV, double fsw,
                                    int maxHarmonic);

/// CISPR 32 Class B conducted QP limit [dBuV] (screening reference):
/// 66->56 (150-500kHz log slope), 56 (0.5-5MHz), 60 (5-30MHz).
double classBLimitQp(double freqHz);

/// Worst margin (limit - peak) over the table + the line where it occurs.
/// Negative margin = screening failure at that line.
struct LimitCheck {
  double worstMarginDb = 0.0;
  double worstFreqHz = 0.0;
  bool pass = true;
};
LimitCheck checkClassB(const std::vector<EmissionLine>& peaks);

}  // namespace emi
}  // namespace power_engine
