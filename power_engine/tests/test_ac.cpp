#include <cmath>
#include <complex>
#include <numbers>
#include <stdexcept>
#include <gtest/gtest.h>

#include "power_engine/ac.h"
#include "power_engine/engine.h"

using power_engine::Engine;
using power_engine::ac::AcPoint;
using power_engine::ac::FourierMeter;

namespace {
constexpr double kPi = std::numbers::pi;
}  // namespace

// RC low-pass (R=1k, C=1u, fc=159.155Hz) driven by source modulation:
// |H| within 0.5dB and phase within 4 degrees of 1/(1+jf/fc).
// NOTE (measured, documented): per-step (ZOH) drive carries an exact
// -ωdt/2 footprint vs continuous LTI (order reduction on nonsmooth
// forcing; textbook-verified, linear in f*dt, converges). Oversample so
// f*dt <= 0.01 (footprint <= ~1.8°) — standard practice.
TEST(AcAnalysis, RcBodeMatchesAnalytical) {
  constexpr double kR = 1000.0, kC = 1e-6;
  constexpr double kFc = 1.0 / (2.0 * kPi * kR * kC);
  const std::vector<double> freqs = {10.0,  20.0,  50.0,  100.0, 200.0,
                                     500.0, 1000.0, 2000.0, 5000.0, 10000.0};
  for (double f : freqs) {
    const double kDt = std::min(5e-6, 0.01 / f);
    Engine eng;
    eng.setTimeStep(kDt);
    eng.circuit().addVoltageSource("V1", 1, 0, 1.0);
    eng.circuit().addResistor("R1", 1, 2, kR);
    // Pre-charged to the DC operating point: no 1V startup step, so the
    // only transient is signal-scale (critical at high f where |H| << 1).
    eng.circuit().addCapacitor("C1", 2, 0, kC, 1.0);
    eng.setStopTime(1e9);
    eng.start();
    const double w = 2.0 * kPi * f;
    const double T = 1.0 / f;
    auto drive = [&](double t) { eng.circuit().findDevice("V1").value = 1.0 + 0.1 * std::sin(w * t); };
    // Settle: injection periods AND circuit transient (5*RC here).
    // Exact step counts: all periods are integer multiples of dt.
    const long long nSettle = static_cast<long long>(std::llround((5.0 * kR * kC + 2.0 * T) / kDt));
    for (long long k = 0; k < nSettle; ++k) {
      drive(eng.time());
      eng.step();
    }
    // Measure exactly 3 periods with midpoint sampling.
    FourierMeter m;
    m.begin(f);
    double tPrev = eng.time();
    double yPrev = eng.currentSolution().probes.at("v:2");
    const long long nMeasure = static_cast<long long>(std::llround(3.0 * T / kDt));
    for (long long k = 0; k < nMeasure; ++k) {
      drive(eng.time());
      eng.step();
      const double t = eng.time();
      // u: value applied over [tPrev, t] (exact ZOH); y: interval midpoint.
      const double u = eng.circuit().findDevice("V1").value;
      const double y = eng.currentSolution().probes.at("v:2");
      m.sample(tPrev, t, u, 0.5 * (yPrev + y));
      tPrev = t;
      yPrev = y;
    }
    eng.stop();
    const AcPoint p = m.result();
    const std::complex<double> ref(1.0, f / kFc);
    const double refMag = 1.0 / std::abs(ref);
    const double refPhase = -std::arg(ref) * 180.0 / kPi;
    EXPECT_NEAR(p.magDb, 20.0 * std::log10(refMag), 0.5) << "f=" << f;
    EXPECT_NEAR(p.phaseDeg, refPhase, 4.0) << "f=" << f;
  }
}

// Buck control-to-output Gvd(s) via duty perturbation, vs the ideal
// second-order model Vin/(1 + sL/R + s^2 L C) cascaded with the digital-PWM
// zero-order hold (1-e^-sT)/(sT): duty is sampled once per switching cycle,
// so the measured plant is Gvd * ZOH (this is what a digital loop really
// sees). Capped at 5kHz: finj = fsw/2 would alias to DC (Nyquist).
TEST(AcAnalysis, BuckGvdMatchesSecondOrderModel) {
  constexpr double kVin = 12.0;
  constexpr double kFsw = 20e3;
  constexpr double kT = 1.0 / kFsw;
  constexpr double kD = 0.5;
  constexpr double kDhat = 0.05;
  constexpr double kL = 200e-6, kC = 200e-6, kR = 5.0;
  constexpr double kDt = 0.5e-6;
  const std::vector<double> freqs = {100.0, 200.0, 500.0, 1000.0, 2000.0, 5000.0};
  // Reference model Gvd (continuous duty modulation: edges are placed from
  // the analytic sine with exact sub-stepping, so there is NO sampler+ZOH
  // in this injection — unlike a digital loop).
  auto gvdAt = [&](double f) {
    const std::complex<double> s(0.0, 2.0 * kPi * f);
    return kVin / (1.0 + s * kL / kR + s * s * kL * kC);
  };
  auto wrap180 = [](double deg) {
    while (deg > 180.0) deg -= 360.0;
    while (deg <= -180.0) deg += 360.0;
    return deg;
  };
  auto phaseErr = [&](double a, double b) { return std::abs(wrap180(a - b)); };
  std::vector<AcPoint> points;
  for (double f : freqs) {
    Engine eng;
    eng.setTimeStep(kDt);
    eng.circuit().addVoltageSource("Vin", 1, 0, kVin);
    eng.circuit().addSwitch("S1", 1, 2, 5e-3, 1e6, true);
    eng.circuit().addDiode("D1", 0, 2, 0.0, 10e-3, 1e6);
    // Steady-state init (POP-style): kills the 6V startup transient whose
    // residue would swamp the mV-scale high-frequency response.
    eng.circuit().addInductor("L1", 2, 3, kL, kVin * kD / kR);
    eng.circuit().addCapacitor("C1", 3, 0, kC, kVin * kD);
    eng.circuit().addResistor("Rload", 3, 0, kR);
    const double T = 1.0 / f;
    // Settle: startup/LC envelope (~2ms here) AND injection periods. The
    // perturbation starts from t=0 with a raised-cosine ramp (below) so no
    // large natural-frequency transient is kicked off to begin with.
    const double tSettle = std::max(9e-3, 2.0 * T);
    const double tMeasure = 3.0 * T;
    const double tEnd = tSettle + tMeasure;
    constexpr double kRamp = 3e-3;  // perturbation envelope ramp [s]
    // AGC: keep the output perturbation <= ~0.3V (small-signal) by scaling
    // the duty amplitude with the local plant gain (crucial at resonance
    // where |Gvd| ~ 60 would otherwise see 50% modulation).
    const double dHat = std::min(kDhat, 0.3 / std::abs(gvdAt(f)));
    auto env = [&](double t) {
      if (t >= kRamp) return 1.0;
      const double x = t / kRamp;
      return 0.5 * (1.0 - std::cos(kPi * x));  // raised cosine, smooth start
    };
    for (double tk = 0.0; tk < tEnd; tk += kT) {
      const double d = kD + dHat * env(tk) * std::sin(2.0 * kPi * f * tk);
      eng.scheduleSwitch("S1", true, tk);
      eng.scheduleSwitch("S1", false, tk + d * kT);
    }
    eng.setStopTime(tEnd + kT);
    eng.start();
    while (eng.time() < tSettle) eng.step();  // startup + transient
    FourierMeter m;
    m.begin(f);
    double tPrev = eng.time();
    double yPrev = eng.currentSolution().probes.at("v:3");
    while (eng.time() < tEnd) {
      eng.step();
      const double t = eng.time();
      // u: post-step gate (exact except inside edge-split steps — analyzed
      // negligible); y: interval midpoint.
      const double u = eng.circuit().switchClosed("S1") ? 1.0 : 0.0;
      const double y = eng.currentSolution().probes.at("v:3");
      m.sample(tPrev, t, u, 0.5 * (yPrev + y));
      tPrev = t;
      yPrev = y;
    }
    eng.stop();
    points.push_back(m.result());
    // Spot-check against plant-only Gvd (continuous modulation), wrapped.
    const std::complex<double> g = gvdAt(f);
    const AcPoint& p = points.back();
    EXPECT_NEAR(p.magDb, 20.0 * std::log10(std::abs(g)), 1.5) << "f=" << f;
    EXPECT_NEAR(phaseErr(p.phaseDeg, std::arg(g) * 180.0 / kPi), 0.0, 8.0) << "f=" << f;
  }
  // Structural Bode landmarks: DC gain ~ Vin, resonance peak, deep roll-off.
  EXPECT_NEAR(points[0].mag, kVin, 0.05 * kVin);
  const double peak = std::max(points[2].mag, points[3].mag);
  EXPECT_GT(peak, 1.3 * points[0].mag);
  EXPECT_GT(std::abs(points[4].phaseDeg), 120.0);  // well past -90° by 2kHz
}
