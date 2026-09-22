#include <cmath>
#include <gtest/gtest.h>

#include <numbers>
#include <string>
#include <vector>

#include "power_engine/emi.h"
#include "power_engine/engine.h"

using power_engine::Engine;

TEST(EmiHelpers, DbuVAndLimits) {
  EXPECT_DOUBLE_EQ(power_engine::emi::toDbuV(1.0), 120.0);
  EXPECT_DOUBLE_EQ(power_engine::emi::toDbuV(1e-6), 0.0);
  EXPECT_DOUBLE_EQ(power_engine::emi::classBLimitQp(150e3), 66.0);
  EXPECT_DOUBLE_EQ(power_engine::emi::classBLimitQp(500e3), 56.0);
  EXPECT_DOUBLE_EQ(power_engine::emi::classBLimitQp(1e6), 56.0);
  EXPECT_DOUBLE_EQ(power_engine::emi::classBLimitQp(10e6), 60.0);
  EXPECT_THROW(power_engine::emi::classBLimitQp(100e3), std::runtime_error);
  EXPECT_THROW(power_engine::emi::peakTable(power_engine::measurements::Trace{}, 20e3, 10),
               std::runtime_error);  // empty trace
}

// LISN transfer impedance vs analytic: current injected at the DUT rail,
// port voltage measured. Z(f) = [ZL || (R+1/jwC)] * R/(R+1/jwC),
// ZL = jwL. Three tones at once (150k/500k/1MHz), separated by spectrum.
TEST(EmiLisn, TransferImpedanceMatchesAnalytic) {
  constexpr double kL = 5e-6, kR = 50.0, kC = 100e-9;
  Engine eng;
  eng.setTimeStep(50e-9);
  eng.setStopTime(400e-6);
  eng.circuit().addVoltageSource("V1", 1, 0, 12.0);
  power_engine::emi::addDcLisn(eng, "LISN", 1, 2, 20, kL, kR, kC);
  eng.circuit().addCurrentSource("I1", 0, 2, 0.0);
  eng.start();
  power_engine::measurements::Trace tr;
  while (eng.status() == power_engine::SimulationStatus::Running) {
    const double t = eng.time();
    eng.circuit().findDevice("I1").value =
        0.01 * std::sin(2.0 * std::numbers::pi * 150e3 * t) +
        0.01 * std::sin(2.0 * std::numbers::pi * 500e3 * t) +
        0.01 * std::sin(2.0 * std::numbers::pi * 1e6 * t);
    eng.step();
    if (t > 200e-6) {
      tr.t.push_back(eng.time());
      tr.y.push_back(eng.currentSolution().probes.at("v:20"));
    }
  }
  auto analytic = [&](double f) {
    const double w = 2.0 * std::numbers::pi * f;
    const std::complex<double> j(0.0, 1.0);
    const std::complex<double> zl = j * w * kL;
    const std::complex<double> zcr = kR + 1.0 / (j * w * kC);
    const std::complex<double> zpar = zl * zcr / (zl + zcr);
    return std::abs(zpar * kR / zcr);
  };
  for (double f : {150e3, 500e3, 1e6}) {
    double got = 0.0;
    for (const auto& h : power_engine::measurements::spectrum(tr, f, 1)) {
      if (h.order == 1) got = h.mag;
    }
    // 10mA drive: port volts = 0.01 * |Z|.
    EXPECT_NEAR(got, 0.01 * analytic(f), 0.02 * 0.01 * analytic(f)) << "f=" << f;
  }
}

namespace {
// Buck + LISN + PAT (tsw=500ns) fixture shared by the fail/pass tests.
// Pre-charged near steady state (C1=6V, L1=1.2A, Cf=12V when filtered)
// so a 500us run at dt=100ns suffices; window 300-500us.
struct EmiRun {
  double worstMarginDb = 0.0;
  double peakOddMinusEvenDb = 0.0;  // 180k (9th, odd) minus 160k (8th, even)
};
EmiRun runBuckEmi(bool filtered) {
  Engine eng;
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
  const auto chk = power_engine::emi::checkClassB(peaks);
  EmiRun r;
  r.worstMarginDb = chk.worstMarginDb;
  r.peakOddMinusEvenDb = peaks[0].dbuv;  // placeholder, refined below
  double odd = 0.0, even = 0.0;
  for (const auto& p : peaks) {
    if (std::abs(p.freqHz - 180e3) < 1.0) odd = p.dbuv;
    if (std::abs(p.freqHz - 160e3) < 1.0) even = p.dbuv;
  }
  r.peakOddMinusEvenDb = odd - even;
  return r;
}
}  // namespace

// Unfiltered buck FAILS Class B screening badly (documents the need for a
// filter); D=0.5 shows the textbook odd/even harmonic structure.
TEST(EmiBuck, UnfilteredFailsScreening) {
  const EmiRun r = runBuckEmi(false);
  EXPECT_LT(r.worstMarginDb, -20.0);  // measured about -52dB
  EXPECT_GT(r.peakOddMinusEvenDb, 2.0);  // 9th (odd) above 8th (even)
}

// Same plant with a 1mH + 10uF//0.5ohm input filter PASSES with margin.
TEST(EmiBuck, FilteredPassesScreening) {
  const EmiRun r = runBuckEmi(true);
  EXPECT_GT(r.worstMarginDb, 6.0);  // measured about +18dB
}
