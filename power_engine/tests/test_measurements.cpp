#include <cmath>
#include <numbers>
#include <stdexcept>
#include <gtest/gtest.h>

#include "power_engine/engine.h"
#include "power_engine/measurements.h"

using power_engine::Engine;
namespace meas = power_engine::measurements;

namespace {
constexpr double kPi = std::numbers::pi;

// Exact-grid synthetic trace: y = dc + a*sin(wt+p) [+ harmonics].
meas::Trace synth(double dc, double f, double dt, double tEnd,
                 const std::vector<std::tuple<double, double, int>>& comps) {
  meas::Trace tr;
  const double w = 2.0 * kPi * f;
  for (double t = 0.0; t <= tEnd + dt * 0.5; t += dt) {
    double y = dc;
    for (const auto& [amp, phase, k] : comps) y += amp * std::sin(k * w * t + phase);
    tr.t.push_back(t);
    tr.y.push_back(y);
  }
  return tr;
}

}  // namespace

// Pure sine: every statistic has a closed form.
TEST(Measurements, SineFixtureExact) {
  auto tr = synth(1.0, 100.0, 10e-6, 30e-3, {{2.0, 0.0, 1}});
  EXPECT_NEAR(meas::mean(tr), 1.0, 1e-9);
  EXPECT_NEAR(meas::rms(tr), std::sqrt(3.0), 1e-9);
  EXPECT_NEAR(meas::rmsAc(tr), std::sqrt(2.0), 1e-9);
  EXPECT_NEAR(meas::minimum(tr), -1.0, 1e-9);
  EXPECT_NEAR(meas::maximum(tr), 3.0, 1e-9);
  EXPECT_NEAR(meas::peakToPeak(tr), 4.0, 1e-9);
  EXPECT_NEAR(meas::thd(tr, 100.0, 10), 0.0, 1e-9);
}

// Harmonics: THD = sqrt(0.5^2+0.25^2)/2 and spectrum recovers amplitudes.
TEST(Measurements, HarmonicFixtureThdSpectrum) {
  auto tr = synth(1.0, 100.0, 10e-6, 30e-3, {{2.0, 0.0, 1}, {0.5, 0.3, 2}, {0.25, 1.0, 3}});
  EXPECT_NEAR(meas::rms(tr), std::sqrt(3.15625), 1e-6);
  EXPECT_NEAR(meas::thd(tr, 100.0, 10), 0.2795085, 1e-4);
  const auto spec = meas::spectrum(tr, 100.0, 6);
  ASSERT_EQ(spec.size(), 7u);
  EXPECT_NEAR(spec[0].mag, 1.0, 1e-9);  // DC
  EXPECT_NEAR(spec[1].mag, 2.0, 1e-3);
  EXPECT_NEAR(spec[2].mag, 0.5, 1e-3);
  EXPECT_NEAR(spec[3].mag, 0.25, 1e-3);
  for (int k = 4; k <= 6; ++k) EXPECT_LT(spec[k].mag, 1e-3);
}

// Window cropping keeps exact boundaries (linear interpolation).
TEST(Measurements, WindowCropsExactly) {
  auto tr = synth(0.0, 100.0, 10e-6, 30e-3, {{1.0, 0.0, 1}});
  auto w = meas::window(tr, 5.25e-3, 15.25e-3);  // exactly one period, off-grid start
  EXPECT_DOUBLE_EQ(w.t.front(), 5.25e-3);
  EXPECT_DOUBLE_EQ(w.t.back(), 15.25e-3);
  EXPECT_NEAR(meas::mean(w), 0.0, 1e-9);
  EXPECT_THROW(meas::window(tr, 2.0, 1.0), std::runtime_error);
  EXPECT_THROW(meas::window(tr, 40e-3, 50e-3), std::runtime_error);
}

// Buck output ripple vs textbook dV = dI/(8*f*C), plus mean.
TEST(Measurements, BuckRippleMatchesTheory) {
  Engine eng;
  eng.setTimeStep(0.5e-6);
  eng.circuit().addVoltageSource("Vin", 1, 0, 12.0);
  eng.circuit().addSwitch("S1", 1, 2, 5e-3, 1e6, true);
  eng.circuit().addDiode("D1", 0, 2, 0.0, 10e-3, 1e6);
  eng.circuit().addInductor("L1", 2, 3, 200e-6, 0.0);
  eng.circuit().addCapacitor("C1", 3, 0, 200e-6, 0.0);
  eng.circuit().addResistor("Rload", 3, 0, 5.0);
  constexpr double T = 50e-6;
  for (double t = 0.0; t < 6e-3; t += T) {
    eng.scheduleSwitch("S1", true, t);
    eng.scheduleSwitch("S1", false, t + 0.5 * T);
  }
  eng.setStopTime(6e-3);
  eng.start();
  // Device-current recording path (first, while still running).
  auto tri = meas::recordSignal(eng, [&] { return eng.deviceCurrent("L1"); }, 5e-3);
  EXPECT_GT(tri.size(), 1000u);
  EXPECT_GT(meas::maximum(tri), 1.0);  // CCM peak current
  auto tr = meas::recordProbe(eng, "v:3", 6e-3);
  EXPECT_GT(tr.size(), 1000u);
  EXPECT_NEAR(meas::mean(meas::window(tr, 5e-3, 6e-3)), 6.0, 0.02 * 6.0);
  // dI = (12-6)*0.5*50u/200u = 0.75A; dV = dI/(8*20k*200u) = 23.4mV.
  const double rip = meas::ripple(tr, 6e-3 - T, 6e-3);
  EXPECT_NEAR(rip, 23.4e-3, 0.3 * 23.4e-3);
}

// Error paths.
TEST(Measurements, ErrorsThrow) {
  meas::Trace empty;
  EXPECT_THROW(meas::mean(empty), std::runtime_error);
  EXPECT_THROW(meas::rms(empty), std::runtime_error);
  EXPECT_THROW(meas::thd(empty, 50.0, 5), std::runtime_error);
  auto dc = synth(2.0, 100.0, 10e-6, 30e-3, {});
  EXPECT_THROW(meas::thd(dc, 100.0, 5), std::runtime_error);  // no fundamental
  EXPECT_THROW(meas::spectrum(dc, -1.0, 5), std::runtime_error);
}
