#include "power_engine/loopgain.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

#include "power_engine/ac.h"
#include "power_engine/control.h"
#include "power_engine/engine.h"

namespace power_engine {
namespace loopgain {
namespace {

constexpr double kPi = std::numbers::pi;

double wrap180(double deg) {
  while (deg > 180.0) deg -= 360.0;
  while (deg <= -180.0) deg += 360.0;
  return deg;
}

}  // namespace

std::complex<double> loopGainFromClosedLoop(std::complex<double> gcl) {
  const std::complex<double> denom = 1.0 + gcl;
  if (std::abs(denom) < 1e-9) {
    throw std::runtime_error("loopGainFromClosedLoop: 1+Gcl ~ 0 (unstable loop?)");
  }
  return -gcl / denom;
}

std::complex<double> referenceLoopGain(const BuckLoopPlant& plant, double freqHz) {
  const double tSw = 1.0 / plant.fsw;
  const double d0 = plant.vref / plant.vin;  // nominal duty
  const std::complex<double> s(0.0, 2.0 * kPi * freqHz);
  // Averaged switch conduction loss: series Rs = D*RonSw + (1-D)*RonD.
  // Shifts the resonance (L' = L + C*R*Rs) and trims the peak; the lossless
  // model overshoots the closed-loop peak height by ~15%.
  const double rs = d0 * plant.ronSw + (1.0 - d0) * plant.ronD;
  const std::complex<double> gvd = plant.vin * plant.r /
      (plant.r + rs + s * (plant.l + plant.c * plant.r * rs) +
       s * s * plant.l * plant.c * plant.r);
  const std::complex<double> controller = plant.kp + plant.ki / s;
  // Trailing-edge modulator: the ON edge is fixed at the period start, only
  // the trailing edge moves, so the small-signal sliver sits D*T after the
  // sample instant: pure delay e^(-s*D*T), NOT a T/2 zero-order hold. (ZOH
  // overstates the delay by (1-D)*T/2 ~ 15us here; verified to <2deg.)
  const std::complex<double> mod = std::exp(-s * d0 * tSw);
  return controller * gvd * mod;
}

LoopMargins computeMargins(std::vector<LoopPoint> points) {
  LoopMargins m;
  if (points.size() < 2) return m;
  std::sort(points.begin(), points.end(),
            [](const LoopPoint& a, const LoopPoint& b) { return a.freqHz < b.freqHz; });
  for (std::size_t i = 1; i < points.size(); ++i) {
    const LoopPoint& a = points[i - 1];
    const LoopPoint& b = points[i];
    if (!(a.freqHz > 0.0) || !(b.freqHz > a.freqHz)) continue;
    const double logA = std::log(a.freqHz);
    const double logB = std::log(b.freqHz);
    // 0dB crossover (downward): |T| falling through 1. Track every
    // crossing; the stability-critical one has the smallest phase margin.
    if (a.magDb >= 0.0 && b.magDb < 0.0) {
      const double frac = (0.0 - a.magDb) / (b.magDb - a.magDb);
      const double fc = std::exp(logA + frac * (logB - logA));
      const double dPhase = wrap180(b.phaseDeg - a.phaseDeg);
      const double pm = 180.0 + (a.phaseDeg + frac * dPhase);
      ++m.crossings;
      if (!m.hasCrossover || pm < m.phaseMarginDeg) {
        m.hasCrossover = true;
        m.crossoverHz = fc;
        m.phaseMarginDeg = pm;
      }
    }
    // -180 deg crossing: phase falling through -180.
    if (!m.hasPhaseCrossover && a.phaseDeg > -180.0 && b.phaseDeg <= -180.0) {
      const double frac = (-180.0 - a.phaseDeg) / (b.phaseDeg - a.phaseDeg);
      m.phaseCrossoverHz = std::exp(logA + frac * (logB - logA));
      m.gainMarginDb = -(a.magDb + frac * (b.magDb - a.magDb));
      m.hasPhaseCrossover = true;
    }
  }
  return m;
}

std::vector<LoopPoint> measureBuckLoopGain(const BuckLoopPlant& plant,
                                           const std::vector<double>& freqs,
                                           LoopMeasureStats* stats) {
  if (freqs.empty()) throw std::runtime_error("measureBuckLoopGain: empty sweep");
  const double tSw = 1.0 / plant.fsw;
  const double d0 = plant.vref / plant.vin;  // nominal duty
  if (!(d0 > plant.dutyMin) || !(d0 < plant.dutyMax)) {
    throw std::runtime_error("measureBuckLoopGain: nominal duty outside limits");
  }
  auto buildBuck = [&](Engine& eng, double il0, double vc0) {
    eng.setTimeStep(plant.dt);
    eng.circuit().addVoltageSource("Vin", 1, 0, plant.vin);
    eng.circuit().addSwitch("S1", 1, 2, 5e-3, 1e6, true);
    eng.circuit().addDiode("D1", 0, 2, 0.0, 10e-3, 1e6);
    eng.circuit().addInductor("L1", 2, 3, plant.l, il0);
    eng.circuit().addCapacitor("C1", 3, 0, plant.c, vc0);
    eng.circuit().addResistor("Rload", 3, 0, plant.r);
  };
  // DC operating point first (no injection): the closed loop has a
  // lightly-damped ~1kHz mode (LC resonance inside the loop bandwidth, see
  // referenceLoopGain), so a POP-guess alone rings for ~20ms. Settle once
  // (30ms) and reuse the exact state for every frequency.
  double dcIl = plant.vin * d0 / plant.r, dcVc = plant.vref;
  double dcInteg = d0 / plant.ki, dcDuty = d0;
  {
    Engine eng;
    buildBuck(eng, dcIl, dcVc);
    control::PiController pi(plant.kp, plant.ki, plant.dutyMin, plant.dutyMax);
    pi.setIntegrator(dcInteg);
    double duty = dcDuty, nextTick = tSw;
    eng.scheduleSwitch("S1", true, 0.0);
    eng.scheduleSwitch("S1", false, duty * tSw);
    eng.setStopTime(30e-3 + tSw);
    eng.start();
    while (eng.status() == SimulationStatus::Running) {
      eng.step();
      if (eng.time() >= nextTick - 1e-12) {
        const double vout = eng.currentSolution().probes.at("v:3");
        duty = pi.update(plant.vref - vout, tSw);
        eng.scheduleSwitch("S1", true, nextTick);
        eng.scheduleSwitch("S1", false, nextTick + duty * tSw);
        nextTick += tSw;
      }
    }
    eng.stop();
    dcIl = eng.deviceCurrent("L1");
    dcVc = eng.currentSolution().probes.at("v:3");
    dcInteg = pi.integrator();
    dcDuty = duty;
    if (std::abs(dcVc - plant.vref) > 0.02 * plant.vref) {
      throw std::runtime_error("measureBuckLoopGain: DC operating point failed");
    }
  }
  // Injection envelope: 10ms raised-cosine ramp. Must be SLOW: the loop
  // contains a lightly-damped ~1kHz mode (tau ~ 7ms, see referenceLoopGain),
  // and a fast ramp edge would ring it enough to contaminate the meter.
  // A 10ms ramp suppresses 1kHz onset content to ~1e-4.
  constexpr double kRamp = 10e-3;  // injection envelope ramp [s]
  auto env = [&](double t) {
    if (t >= kRamp) return 1.0;
    const double x = t / kRamp;
    return 0.5 * (1.0 - std::cos(kPi * x));  // raised cosine, smooth start
  };

  LoopMeasureStats acc;
  std::vector<LoopPoint> points;
  for (double f : freqs) {
    if (!(f > 0.0) || f >= plant.fsw / 2.0) {
      throw std::runtime_error("measureBuckLoopGain: freq must be in (0, fsw/2)");
    }
    // AGC from the analytic closed-loop gain Gcl = -T/(1+T): hold the
    // output perturbation to ~0.15V (small-signal, no duty saturation).
    const std::complex<double> tRef = referenceLoopGain(plant, f);
    const std::complex<double> gclRef = -tRef / (1.0 + tRef);
    const double vHat =
        std::clamp(0.15 / std::abs(gclRef), 5e-3, 50e-3);

    Engine eng;
    buildBuck(eng, dcIl, dcVc);

    control::PiController pi(plant.kp, plant.ki, plant.dutyMin, plant.dutyMax);
    // Bumpless start from the settled DC state (see above).
    pi.setIntegrator(dcInteg);
    double duty = dcDuty;
    double nextTick = tSw;
    // Sample-and-held injection for the CONTROLLER: the digital loop samples
    // the sense path once per switching period, so uHold is the loop input.
    // The METER is fed the continuous sine at each step midpoint instead:
    // the plant responds to pulse AREAS (exact samples, fundamental uhat),
    // while the staircase fundamental is uhat*sinc*e^(-jwt/2) — feeding the
    // staircase would bias the reading by the inverse staircase spectrum.
    // Both share the fundamental uhat, so the ratio is the true loop gain.
    double uHold = 0.0;
    auto vinj = [&](double t) {
      return vHat * env(t) * std::sin(2.0 * kPi * f * t);
    };

    eng.scheduleSwitch("S1", true, 0.0);
    eng.scheduleSwitch("S1", false, duty * tSw);
    const double period = 1.0 / f;
    // DC state is exact and the 10ms envelope barely rings the slow mode,
    // so 20ms settle (or 3 injection periods, whichever is longer) suffices.
    // (Generous because T = -Gcl/(1+Gcl) amplifies Gcl residue near the
    // resonance peak where Gcl ~ -1.)
    const double tSettle = std::max(20e-3, 3.0 * period);
    // Measure >= 3 periods AND >= 3ms: at high f the mV-scale response
    // needs many periods to average out switching ripple/noise (SNR gain).
    const double tMeasure = std::max(3.0 * period, 3e-3);
    const double tEnd = tSettle + tMeasure;
    eng.setStopTime(tEnd + tSw);
    eng.start();

    // Settle: injection ramps in smoothly; PI tracks from the POP init.
    while (eng.time() < tSettle) {
      eng.step();
      if (eng.time() >= nextTick - 1e-12) {
        const double vout = eng.currentSolution().probes.at("v:3");
        uHold = vinj(nextTick);
        duty = pi.update(plant.vref - (vout + uHold), tSw);
        eng.scheduleSwitch("S1", true, nextTick);
        eng.scheduleSwitch("S1", false, nextTick + duty * tSw);
        nextTick += tSw;
      }
    }
    // Fail fast if the settle did not actually settle (small-signal
    // measurement on a railed loop is garbage).
    {
      const double vSettled = eng.currentSolution().probes.at("v:3");
      if (std::abs(vSettled - plant.vref) > 0.25 * plant.vref) {
        throw std::runtime_error("measureBuckLoopGain: loop did not settle");
      }
    }
    // Measure >= 3 injection periods AND >= 3ms against the sine input.
    ac::FourierMeter meter;
    meter.begin(f);
    double tPrev = eng.time();
    double yPrev = eng.currentSolution().probes.at("v:3");
    while (eng.time() < tEnd) {
      eng.step();
      const double t = eng.time();
      if (t >= nextTick - 1e-12) {
        const double vout = eng.currentSolution().probes.at("v:3");
        uHold = vinj(nextTick);
        duty = pi.update(plant.vref - (vout + uHold), tSw);
        eng.scheduleSwitch("S1", true, nextTick);
        eng.scheduleSwitch("S1", false, nextTick + duty * tSw);
        nextTick += tSw;
      }
      const double y = eng.currentSolution().probes.at("v:3");
      // u: continuous injection at the step midpoint (unbiased fundamental);
      // y: interval midpoint.
      meter.sample(tPrev, t, vinj(0.5 * (tPrev + t)), 0.5 * (yPrev + y));
      acc.dutyMin = std::min(acc.dutyMin, duty);
      acc.dutyMax = std::max(acc.dutyMax, duty);
      acc.voutPerturbationMax =
          std::max(acc.voutPerturbationMax, std::abs(y - plant.vref));
      tPrev = t;
      yPrev = y;
    }
    eng.stop();

    const std::complex<double> gcl = meter.gain();
    const std::complex<double> tLoop = loopGainFromClosedLoop(gcl);
    LoopPoint p;
    p.freqHz = f;
    p.mag = std::abs(tLoop);
    p.magDb = 20.0 * std::log10(p.mag);
    p.phaseDeg = std::arg(tLoop) * 180.0 / kPi;
    points.push_back(p);
  }
  if (stats != nullptr) *stats = acc;
  return points;
}

}  // namespace loopgain
}  // namespace power_engine
