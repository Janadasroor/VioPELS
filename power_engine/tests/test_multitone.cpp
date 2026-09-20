#include <cmath>
#include <complex>
#include <numbers>
#include <stdexcept>
#include <gtest/gtest.h>

#include "power_engine/ac.h"
#include "power_engine/engine.h"
#include "power_engine/measurements.h"
#include "power_engine/multitone.h"

using power_engine::Engine;
using power_engine::SimulationStatus;
using power_engine::ac::AcPoint;
using power_engine::ac::FourierMeter;
using power_engine::multitone::BuckMultitonePlant;
using power_engine::multitone::fitRingdown;
using power_engine::multitone::measureBuckMultitone;
using power_engine::multitone::MultitoneSignal;

namespace {
constexpr double kPi = std::numbers::pi;
// Prime-multiple tones of the 50ms window bin (20Hz): no tone is a
// harmonic of another, so plant harmonics never coincide with tones.
const std::vector<double> kTones = {100.0, 220.0, 460.0, 940.0, 1940.0, 5020.0};
}  // namespace

TEST(MultitoneMath, SchroederBuildsValidSignal) {
  const MultitoneSignal sig =
      MultitoneSignal::schroeder({100.0, 200.0}, {0.05, 0.05});
  EXPECT_DOUBLE_EQ(sig.value(0.0),
                   0.05 * std::sin(-kPi / 2.0) + 0.05 * std::sin(-2.0 * kPi));
  EXPECT_DOUBLE_EQ(sig.peakBound(), 0.1);
  // Crest factor: Schroeder peaks well below worst-case alignment.
  const MultitoneSignal six = MultitoneSignal::schroeder(
      kTones, std::vector<double>(kTones.size(), 0.03));
  double peak = 0.0;
  for (double t = 0.0; t < 0.05; t += 1e-6) {
    peak = std::max(peak, std::abs(six.value(t)));
  }
  EXPECT_LT(peak, 0.97 * six.peakBound());
  EXPECT_THROW(MultitoneSignal({}), std::runtime_error);
  EXPECT_THROW(MultitoneSignal::schroeder({100.0}, {0.05, 0.06}), std::runtime_error);
  EXPECT_THROW(MultitoneSignal::schroeder({}, {}), std::runtime_error);
}

// Single-sim Bode vs the second-order plant (the stepped-sine reference).
TEST(BuckMultitone, MatchesSecondOrderModel) {
  const BuckMultitonePlant plant;
  const std::vector<AcPoint> points = measureBuckMultitone(plant, kTones, 50e-3);
  ASSERT_EQ(points.size(), kTones.size());
  auto wrap180 = [](double deg) {
    while (deg > 180.0) deg -= 360.0;
    while (deg <= -180.0) deg += 360.0;
    return deg;
  };
  for (std::size_t i = 0; i < points.size(); ++i) {
    const std::complex<double> s(0.0, 2.0 * kPi * kTones[i]);
    const std::complex<double> g =
        plant.vin / (1.0 + s * plant.l / plant.r + s * s * plant.l * plant.c);
    EXPECT_NEAR(points[i].magDb, 20.0 * std::log10(std::abs(g)), 2.5)
        << "f=" << kTones[i];
    EXPECT_NEAR(std::abs(wrap180(points[i].phaseDeg - std::arg(g) * 180.0 / kPi)),
                0.0, 12.0)
        << "f=" << kTones[i];
  }
  // Structural landmarks survive the single sim: DC gain ~ Vin, resonance
  // rise toward 796Hz, ~32dB roll-off by 5kHz.
  EXPECT_NEAR(points[0].mag, plant.vin, 0.1 * plant.vin);
  EXPECT_GT(points[3].mag, points[0].mag);
  EXPECT_LT(points[5].magDb, 20.0 * std::log10(points[0].mag) - 25.0);
}

// One-off stepped sine at two tones (same physics as test_ac buck): the
// literal acceptance — multitone matches stepped-sine within tolerance.
TEST(BuckMultitone, MatchesSteppedSine) {
  const BuckMultitonePlant plant;
  const std::vector<AcPoint> multi = measureBuckMultitone(plant, kTones, 50e-3);
  auto wrap180 = [](double deg) {
    while (deg > 180.0) deg -= 360.0;
    while (deg <= -180.0) deg += 360.0;
    return deg;
  };
  for (std::size_t ti : {std::size_t{2}, std::size_t{4}}) {  // 460Hz, 1940Hz
    const double f = kTones[ti];
    Engine eng;
    eng.setTimeStep(plant.dt);
    eng.circuit().addVoltageSource("Vin", 1, 0, plant.vin);
    eng.circuit().addSwitch("S1", 1, 2, 5e-3, 1e6, true);
    eng.circuit().addDiode("D1", 0, 2, 0.0, 10e-3, 1e6);
    eng.circuit().addInductor("L1", 2, 3, plant.l, plant.vin * plant.duty / plant.r);
    eng.circuit().addCapacitor("C1", 3, 0, plant.c, plant.vin * plant.duty);
    eng.circuit().addResistor("Rload", 3, 0, plant.r);
    const double T = 1.0 / f;
    const double tSettle = std::max(9e-3, 2.0 * T);
    const double tEnd = tSettle + 3.0 * T;
    constexpr double kRamp = 3e-3;
    const std::complex<double> s(0.0, 2.0 * kPi * f);
    const std::complex<double> g =
        plant.vin / (1.0 + s * plant.l / plant.r + s * s * plant.l * plant.c);
    const double dHat = std::min(0.05, 0.3 / std::abs(g));
    auto env = [&](double t) {
      if (t >= kRamp) return 1.0;
      const double x = t / kRamp;
      return 0.5 * (1.0 - std::cos(kPi * x));
    };
    const double kT = 1.0 / plant.fsw;
    for (double tk = 0.0; tk < tEnd; tk += kT) {
      const double d = plant.duty + dHat * env(tk) * std::sin(2.0 * kPi * f * tk);
      eng.scheduleSwitch("S1", true, tk);
      eng.scheduleSwitch("S1", false, tk + d * kT);
    }
    eng.setStopTime(tEnd + kT);
    eng.start();
    while (eng.time() < tSettle) eng.step();
    FourierMeter m;
    m.begin(f);
    double tPrev = eng.time();
    double yPrev = eng.currentSolution().probes.at("v:3");
    while (eng.time() < tEnd) {
      eng.step();
      const double t = eng.time();
      const double u = eng.circuit().switchClosed("S1") ? 1.0 : 0.0;
      const double y = eng.currentSolution().probes.at("v:3");
      m.sample(tPrev, t, u, 0.5 * (yPrev + y));
      tPrev = t;
      yPrev = y;
    }
    eng.stop();
    const AcPoint stepped = m.result();
    EXPECT_NEAR(multi[ti].magDb, stepped.magDb, 2.0) << "f=" << f;
    EXPECT_NEAR(std::abs(wrap180(multi[ti].phaseDeg - stepped.phaseDeg)), 0.0, 10.0)
        << "f=" << f;
  }
}

// Impulse response: one-cycle duty bump at steady state, ring-down fit vs
// the LC values (f0 ~ 796Hz, tau = 2Q/w0 ~ 2ms with Q = R*sqrt(C/L) = 5).
TEST(BuckImpulse, RingdownMatchesLcValues) {
  constexpr double kVin = 12.0, kD = 0.5, kT = 50e-6;
  Engine eng;
  eng.setTimeStep(0.5e-6);
  eng.circuit().addVoltageSource("Vin", 1, 0, kVin);
  eng.circuit().addSwitch("S1", 1, 2, 5e-3, 1e6, true);
  eng.circuit().addDiode("D1", 0, 2, 0.0, 10e-3, 1e6);
  eng.circuit().addInductor("L1", 2, 3, 200e-6, kVin * kD / 5.0);
  eng.circuit().addCapacitor("C1", 3, 0, 200e-6, kVin * kD);
  eng.circuit().addResistor("Rload", 3, 0, 5.0);
  constexpr double kBump = 9e-3;  // bump cycle starts here (settled by then)
  constexpr double kEnd = 22e-3;
  for (double tk = 0.0; tk < kEnd; tk += kT) {
    const double d = (tk >= kBump && tk < kBump + kT) ? kD + 0.1 : kD;
    eng.scheduleSwitch("S1", true, tk);
    eng.scheduleSwitch("S1", false, tk + d * kT);
  }
  eng.setStopTime(kEnd);
  eng.start();
  power_engine::measurements::Trace raw =
      power_engine::measurements::recordProbe(eng, "v:3", kEnd);
  eng.stop();
  // Precondition: per-switching-cycle averaging removes the 20kHz ripple
  // (which would hijack zero-crossing detection) and keeps the ~800Hz ring.
  power_engine::measurements::Trace tr;
  {
    double acc = 0.0, cnt = 0.0, bin = 0.0;
    for (std::size_t i = 0; i < raw.size(); ++i) {
      if (raw.t[i] >= bin + kT && cnt > 0.0) {
        tr.t.push_back(bin + 0.5 * kT);
        tr.y.push_back(acc / cnt);
        bin += kT;
        acc = 0.0;
        cnt = 0.0;
      }
      acc += raw.y[i];
      cnt += 1.0;
    }
    if (cnt > 0.0) {
      tr.t.push_back(bin + 0.5 * kT);
      tr.y.push_back(acc / cnt);
    }
  }
  const auto rd = fitRingdown(tr, kBump + kT + 0.5e-3);
  EXPECT_NEAR(rd.freqHz, 796.0, 0.03 * 796.0);
  EXPECT_NEAR(rd.tau, 2e-3, 0.2 * 2e-3);
}
