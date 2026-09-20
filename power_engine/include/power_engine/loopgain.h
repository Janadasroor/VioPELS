#pragma once
#include <complex>
#include <vector>

namespace power_engine {
namespace loopgain {

/// Closed-loop buck plant + controller parameters for loop-gain measurement.
/// Defaults match the validated closed-loop buck (test_control.cpp) and the
/// buck Gvd fixture (test_ac.cpp): Vin=12, fsw=20kHz, L=200uH, C=200uF,
/// R=5ohm (LC ~796Hz, Q~5), PI kp=0.07/ki=40, Vref=5V.
struct BuckLoopPlant {
  double vin = 12.0;
  double fsw = 20e3;
  double l = 200e-6;
  double c = 200e-6;
  double r = 5.0;
  double vref = 5.0;
  double kp = 0.07;
  double ki = 40.0;
  double dutyMin = 0.02;
  double dutyMax = 0.95;
  double dt = 0.5e-6;
  double ronSw = 5e-3;  ///< switch on-resistance (matches sim fixture)
  double ronD = 10e-3;  ///< diode on-resistance (matches sim fixture)
};

/// Loop-gain Bode point: T(jw) at one injection frequency.
struct LoopPoint {
  double freqHz = 0.0;
  double mag = 0.0;       ///< |T| linear
  double magDb = 0.0;     ///< 20*log10(|T|)
  double phaseDeg = 0.0;  ///< arg(T) in (-180, 180]
};

/// Gain/phase margins interpolated from measured loop-gain points.
/// A loop may cross 0dB several times (conditional stability, as in the
/// validated buck where the LC resonance pushes the gain back above 0dB);
/// the reported crossover is the DOWNWARD crossing with the SMALLEST phase
/// margin (the stability-critical one), not necessarily the first.
struct LoopMargins {
  bool hasCrossover = false;       ///< any downward 0dB crossing in sweep
  int crossings = 0;               ///< number of downward 0dB crossings found
  double crossoverHz = 0.0;        ///< critical crossing (log-interpolated)
  double phaseMarginDeg = 0.0;     ///< 180 + arg(T) there (minimum over crossings)
  bool hasPhaseCrossover = false;  ///< arg(T) crosses -180 inside the sweep
  double phaseCrossoverHz = 0.0;   ///< -180 deg frequency (log-interpolated)
  double gainMarginDb = 0.0;       ///< -20*log10(|T|) at phase crossover
};

/// Small-signal stats collected over the measurement windows (guard against
/// saturation invalidating the small-signal assumption).
struct LoopMeasureStats {
  double dutyMin = 1.0;
  double dutyMax = 0.0;
  double voutPerturbationMax = 0.0;  ///< max |vout - vref| during measurement
};

/// Return ratio from measured closed-loop gain Gcl = vout/vinj:
/// Gcl = -T/(1+T)  =>  T = -Gcl/(1+Gcl). Throws if 1+Gcl is ~zero.
std::complex<double> loopGainFromClosedLoop(std::complex<double> gcl);

/// Analytic design reference: T(s) = C(s)*Gvd(s)*M(s) with C = kp+ki/s,
/// Gvd = Vin/(1+sL/R+s^2LC), M = e^(-s*D*T) for the trailing-edge modulator
/// (fixed leading edge, modulated trailing edge: the small-signal sliver
/// sits D*T after the sample instant; T = 1/fsw, D = Vref/Vin). This is the
/// "PI design calculation" the measured margins are checked against. NOTE:
/// a T/2 zero-order hold is the WRONG modulator model here (overstates the
/// delay by (1-D)*T/2); the sliver delay is verified against measurement.
std::complex<double> referenceLoopGain(const BuckLoopPlant& plant, double freqHz);

/// Interpolate gain/phase margins from sweep points (sorted by freq if not).
LoopMargins computeMargins(std::vector<LoopPoint> points);

/// Series-injection loop-gain measurement on the closed-loop buck:
/// v_sense = vout + vinj is fed to the PI (Vref AC = 0), Gcl = vout/vinj is
/// correlated with the Fourier meter, T follows from loopGainFromClosedLoop.
/// Injection amplitude is AGC'd from the analytic closed-loop gain so the
/// output perturbation stays ~0.15V (small-signal, no duty saturation).
/// IMPORTANT: the digital loop samples the injection once per switching
/// period, so the meter input is the sample-and-held injection (exactly what
/// the controller sees), not the continuous sine.
std::vector<LoopPoint> measureBuckLoopGain(const BuckLoopPlant& plant,
                                           const std::vector<double>& freqs,
                                           LoopMeasureStats* stats = nullptr);

}  // namespace loopgain
}  // namespace power_engine
