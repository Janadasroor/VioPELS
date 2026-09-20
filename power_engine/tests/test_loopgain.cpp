#include <cmath>
#include <complex>
#include <vector>
#include <gtest/gtest.h>

#include "power_engine/loopgain.h"

using power_engine::loopgain::BuckLoopPlant;
using power_engine::loopgain::computeMargins;
using power_engine::loopgain::loopGainFromClosedLoop;
using power_engine::loopgain::LoopMargins;
using power_engine::loopgain::LoopMeasureStats;
using power_engine::loopgain::LoopPoint;
using power_engine::loopgain::measureBuckLoopGain;
using power_engine::loopgain::referenceLoopGain;

// Gcl = -T/(1+T) <=> T = -Gcl/(1+Gcl): spot checks.
TEST(LoopGainMath, FromClosedLoop) {
  const std::complex<double> t = loopGainFromClosedLoop({-0.5, 0.0});
  EXPECT_NEAR(t.real(), 1.0, 1e-12);
  EXPECT_NEAR(t.imag(), 0.0, 1e-12);
  // Gcl = j  =>  T = -j/(1+j) = -0.5-0.5j.
  const std::complex<double> t2 = loopGainFromClosedLoop({0.0, 1.0});
  EXPECT_NEAR(t2.real(), -0.5, 1e-12);
  EXPECT_NEAR(t2.imag(), -0.5, 1e-12);
  // Round trip: T=2<-120deg -> Gcl -> T.
  const std::complex<double> tRef =
      2.0 * std::exp(std::complex<double>(0.0, -120.0 * M_PI / 180.0));
  const std::complex<double> gcl = -tRef / (1.0 + tRef);
  const std::complex<double> tBack = loopGainFromClosedLoop(gcl);
  EXPECT_NEAR(tBack.real(), tRef.real(), 1e-9);
  EXPECT_NEAR(tBack.imag(), tRef.imag(), 1e-9);
  EXPECT_THROW(loopGainFromClosedLoop({-1.0, 0.0}), std::runtime_error);
}

// Margins on a synthetic integrator loop T = (fc/f)<-90deg: PM = 90,
// crossover exactly fc, no -180 crossing (GM infinite).
TEST(LoopGainMath, MarginsOfIntegratorLoop) {
  std::vector<LoopPoint> pts;
  for (double f : {100.0, 200.0, 500.0, 1000.0, 2000.0, 5000.0, 10000.0}) {
    LoopPoint p;
    p.freqHz = f;
    p.mag = 1000.0 / f;
    p.magDb = 20.0 * std::log10(p.mag);
    p.phaseDeg = -90.0;
    pts.push_back(p);
  }
  const LoopMargins m = computeMargins(pts);
  ASSERT_TRUE(m.hasCrossover);
  EXPECT_NEAR(m.crossoverHz, 1000.0, 1.0);
  EXPECT_NEAR(m.phaseMarginDeg, 90.0, 0.5);
  EXPECT_FALSE(m.hasPhaseCrossover);
}

// Design reference sanity: low-frequency loop gain is large (integrator),
// high-frequency rolls off past the LC double pole.
TEST(LoopGainMath, ReferenceShape) {
  const BuckLoopPlant plant;
  const double dcDb = 20.0 * std::log10(std::abs(referenceLoopGain(plant, 10.0)));
  const double hiDb =
      20.0 * std::log10(std::abs(referenceLoopGain(plant, 5000.0)));
  EXPECT_GT(dcDb, 10.0);
  EXPECT_LT(hiDb, 0.0);
}

// Full closed-loop buck, series injection at the sense node: measured T vs
// the PI design calculation T(s) = C*Gvd*ZOH, then margins vs margins.
// NOTE: this loop is conditionally stable (LC resonance pushes |T| back
// above 0dB): the stability-critical margin is PM ~ 5deg at ~1.1kHz, NOT
// the 147deg at the 167Hz first crossing. The test pins the worst crossing.
TEST(BuckLoopGain, MatchesDesignCalculation) {
  const BuckLoopPlant plant;
  const std::vector<double> freqs = {100.0, 200.0,  500.0,  700.0,  800.0,
                                     1000.0, 1200.0, 1500.0, 2000.0, 3000.0,
                                     5000.0};
  LoopMeasureStats stats;
  const std::vector<LoopPoint> points = measureBuckLoopGain(plant, freqs, &stats);

  // Small-signal discipline: duty never saturated, perturbation small.
  EXPECT_GT(stats.dutyMin, plant.dutyMin);
  EXPECT_LT(stats.dutyMax, plant.dutyMax);
  EXPECT_LT(stats.voutPerturbationMax, 0.3);

  ASSERT_EQ(points.size(), freqs.size());
  auto wrap180 = [](double deg) {
    while (deg > 180.0) deg -= 360.0;
    while (deg <= -180.0) deg += 360.0;
    return deg;
  };
  // Per-point check on the CLOSED-LOOP gain Gcl (the directly measured
  // quantity): near the resonance peak Gcl ~ -1, where T = -Gcl/(1+Gcl) is
  // ill-conditioned and amplifies mV residue into dB of T. Margins below
  // validate T where it matters (|T| ~ 1, well-conditioned).
  // Peak-top bins (|Gcl_ref| > 3, the sharp ~1.06kHz closed-loop resonance)
  // are validated separately below: pinning dPhase/df there would demand
  // sub-Hz peak agreement no rig can give (Q ~ 15 on both sides).
  double peakMeas = 0.0, peakRef = 0.0;
  for (std::size_t i = 0; i < points.size(); ++i) {
    const std::complex<double> tRef = referenceLoopGain(plant, freqs[i]);
    const std::complex<double> gclRef = -tRef / (1.0 + tRef);
    const std::complex<double> tMeas =
        std::polar(std::pow(10.0, points[i].magDb / 20.0),
                   points[i].phaseDeg * M_PI / 180.0);
    const std::complex<double> gclMeas = -tMeas / (1.0 + tMeas);
    if (std::abs(gclRef) > 3.0) {
      peakMeas = std::max(peakMeas, std::abs(gclMeas));
      peakRef = std::max(peakRef, std::abs(gclRef));
      continue;
    }
    // Tighter on the flanks, looser on the peak's right skirt (1.2-3kHz)
    // where small Q differences leave a repeatable ~2dB footprint.
    const bool skirt = freqs[i] >= 1150.0;
    EXPECT_NEAR(20.0 * std::log10(std::abs(gclMeas)),
                20.0 * std::log10(std::abs(gclRef)), skirt ? 3.5 : 1.5)
        << "f=" << freqs[i];
    EXPECT_NEAR(std::abs(wrap180(std::arg(gclMeas) * 180.0 / M_PI -
                                 std::arg(gclRef) * 180.0 / M_PI)),
                0.0, skirt ? 12.0 : 8.0)
        << "f=" << freqs[i];
  }
  // Closed-loop resonance exists at the predicted height (within 35%).
  EXPECT_GT(peakMeas, 0.65 * peakRef);
  EXPECT_LT(peakMeas, 1.35 * peakRef);

  // Margins: measured vs analytic (dense reference grid). The critical
  // margin is the minimum-PM downward crossing (~1.1kHz, PM ~ 5deg).
  const LoopMargins meas = computeMargins(points);
  std::vector<LoopPoint> refPts;
  for (double f = 100.0; f <= 8000.0; f *= 1.05) {
    const std::complex<double> ref = referenceLoopGain(plant, f);
    LoopPoint p;
    p.freqHz = f;
    p.mag = std::abs(ref);
    p.magDb = 20.0 * std::log10(p.mag);
    p.phaseDeg = std::arg(ref) * 180.0 / M_PI;
    refPts.push_back(p);
  }
  const LoopMargins ref = computeMargins(refPts);
  ASSERT_TRUE(ref.hasCrossover) << "design must predict a crossover";
  ASSERT_TRUE(meas.hasCrossover) << "measurement must find the crossover";
  // NOTE: no assertion on the exact crossing COUNT: the shallow 167/275Hz
  // dip pair sits within ±2dB of 0dB (rig resolution) and may not resolve.
  // What matters is the stability-critical (minimum-PM) crossing.
  EXPECT_NEAR(meas.crossoverHz, ref.crossoverHz, 0.15 * ref.crossoverHz);
  EXPECT_NEAR(meas.phaseMarginDeg, ref.phaseMarginDeg, 6.0);
  EXPECT_LT(meas.phaseMarginDeg, 20.0) << "loop must show its tight critical margin";
  EXPECT_EQ(meas.hasPhaseCrossover, ref.hasPhaseCrossover);
}
