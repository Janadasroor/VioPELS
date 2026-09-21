#pragma once
#include <array>
#include <algorithm>
#include <string>

#include "power_engine/control.h"
#include "power_engine/engine.h"

namespace power_engine {
namespace vienna {

// Plant parameters for the 3x230V/50Hz Vienna PFC fixture (true 3-wire:
// the mains neutral, node 10, MUST float — a grounded neutral plus the
// grounded dc- gives the lower diodes a zero-impedance ground return
// that shorts the phases).
struct PlantParams {
  double c1 = 2e-3;      ///< dc+ to midpoint [F]
  double c2 = 1.5e-3;    ///< midpoint to dc- [F] (asymmetric by default)
  double rLoad = 100.0;  ///< dc+ to dc- [ohm]
  double rBleed = 500.0; ///< midpoint to dc- [ohm]: DC imbalance disturbance
};

// Adds the Vienna power stage to eng (node map matches the uncontrolled
// diode-bridge fixture: sources 1/2/3 vs floating neutral 10, grid
// 0.5ohm+5mH, phases 4/5/6, dc+ 7, midpoint 9, dc- 0). Switches SA/SB/SC
// start open (diode-bridge pre-charge).
inline void buildPlant(Engine& eng, const PlantParams& p) {
  eng.circuit().addVoltageSource("VA", 1, 10, 0.0);
  eng.circuit().addVoltageSource("VB", 2, 10, 0.0);
  eng.circuit().addVoltageSource("VC", 3, 10, 0.0);
  eng.circuit().addResistor("RAg", 1, 11, 0.5);
  eng.circuit().addResistor("RBg", 2, 12, 0.5);
  eng.circuit().addResistor("RCg", 3, 13, 0.5);
  eng.circuit().addInductor("LA", 11, 4, 5e-3, 0.0);
  eng.circuit().addInductor("LB", 12, 5, 5e-3, 0.0);
  eng.circuit().addInductor("LC", 13, 6, 5e-3, 0.0);
  eng.circuit().addDiode("DAu", 4, 7, 0.7, 10e-3, 1e6);
  eng.circuit().addDiode("DBu", 5, 7, 0.7, 10e-3, 1e6);
  eng.circuit().addDiode("DCu", 6, 7, 0.7, 10e-3, 1e6);
  eng.circuit().addDiode("DAl", 0, 4, 0.7, 10e-3, 1e6);
  eng.circuit().addDiode("DBl", 0, 5, 0.7, 10e-3, 1e6);
  eng.circuit().addDiode("DCl", 0, 6, 0.7, 10e-3, 1e6);
  eng.circuit().addSwitch("SA", 4, 9, 5e-3, 1e6, false);
  eng.circuit().addSwitch("SB", 5, 9, 5e-3, 1e6, false);
  eng.circuit().addSwitch("SC", 6, 9, 5e-3, 1e6, false);
  eng.circuit().addCapacitor("C1", 7, 9, p.c1, 0.0);
  eng.circuit().addCapacitor("C2", 9, 0, p.c2, 0.0);
  eng.circuit().addResistor("Rload", 7, 0, p.rLoad);
  eng.circuit().addResistor("Rbleed", 9, 0, p.rBleed);
}

// Average-current-mode carrier controller: outer Vdc PI -> conductance G,
// per-phase references i*_k = G * v_k (resistive -> unity power factor),
// inner current PIs -> duties on interleaved symmetric carriers.
//
// STRUCTURAL NOTE (verified in sim: Vdc=754V, PF=0.97, midpoint at 0V):
// this architecture regulates Vdc and PF but CANNOT balance the midpoint
// in 3-wire, and no duty-side correction can fix it:
//  - common-mode duty shifts are KCL-null (per-period midpoint charge
//    T*d0*SUM(i_k) = 0 identically, floating neutral);
//  - differential duty trims are disturbance-rejected by the current-loop
//    integrators (infinite DC gain absorbs them; net duty unchanged);
//  - reference shaping is confined to SUM=0 with vanishing authority.
// Midpoint balancing needs an architecture without a tracking integrator
// in the current path — see HysteresisController below. There is
// deliberately NO balance term here (a saturated no-op would lie).
class Controller {
 public:
  struct Params {
    double dt = 1e-6;         ///< control tick [s] (must match engine step)
    double carrierFreq = 20e3;  ///< switching frequency [Hz]
    double vdcRef = 750.0;      ///< regulated dc bus [V] (> 2*Vpk for authority)
    double tEnable = 20e-3;     ///< loop close time [s] (diode pre-charge first)
    double rampTime = 60e-3;    ///< Vdc* ramp after enable [s]
    double kpV = 2.55e-4;       ///< outer PI proportional [S/V]
    double kiV = 4.8e-3;        ///< outer PI integral [S/V/s]
    double kpI = 0.15;          ///< inner PI proportional [duty/A]
    double kiI = 400.0;         ///< inner PI integral [duty/A/s]
  };

  Controller() : Controller(typename Controller::Params{}) {}
  explicit Controller(const Params& p)
      : p_(p),
        pwm_{
            control::Pwm(p.carrierFreq, 0.0, 0.0, control::Carrier::Symmetric),
            control::Pwm(p.carrierFreq, 0.0, 0.0, control::Carrier::Symmetric),
            control::Pwm(p.carrierFreq, 0.0, 0.0, control::Carrier::Symmetric),
        },
        piI_{control::PiController(p.kpI, p.kiI, 0.0, 1.0),
              control::PiController(p.kpI, p.kiI, 0.0, 1.0),
              control::PiController(p.kpI, p.kiI, 0.0, 1.0)},
        piV_(p.kpV, p.kiV, 0.0, 0.06),
        vdcFilt_(control::TransferFunction::lowPass(40.0, p.dt)) {
    const double T = 1.0 / p.carrierFreq;
    pwm_[0].setPhase(0.0);
    pwm_[1].setPhase(T / 3.0);
    pwm_[2].setPhase(2.0 * T / 3.0);
    // Bumpless start: preset the outer integrator to the expected steady
    // conductance P*/(3*Vrms^2) with P* = VdcRef^2/Rload, Rload = 100.
    piV_.setIntegrator((p.vdcRef * p.vdcRef / 100.0) / (3.0 * 230.0 * 230.0));
    g_ = piV_.update(0.0, p.dt);  // expose the preset before the first tick
  }

  /// One tick. vsrc = ideal source voltages (clean reference, no switching
  /// ripple), i = measured phase currents (grid -> phase), vdc = v:7.
  /// Returns gate states (true = switch closed = phase clamped to the
  /// midpoint). Open (all false) before tEnable.
  std::array<bool, 3> update(double t, const std::array<double, 3>& vsrc,
                             const std::array<double, 3>& i, double vdc) {
    if (t < p_.tEnable) {
      duties_ = {0.0, 0.0, 0.0};
      return {false, false, false};
    }
    // Vdc* ramp from the diode-bridge level (~540V) to full reference.
    const double ramp = std::min(1.0, (t - p_.tEnable) / p_.rampTime);
    const double vref = 540.0 + (p_.vdcRef - 540.0) * ramp;
    const double vf = vdcFilt_.update(vdc);
    // Outer loop at 1kHz (Vdc ripple is 300Hz+; the 40Hz pre-filter plus
    // the slow loop keep it out of the current references).
    ++ticks_;
    if (ticks_ % 1000 == 0) g_ = piV_.update(vref - vf, 1000.0 * p_.dt);
    for (int k = 0; k < 3; ++k) {
      const double d = piI_[k].update(g_ * vsrc[k] - i[k], p_.dt);
      duties_[k] = d;
      pwm_[k].setDuty(d);
    }
    return {pwm_[0].output(t), pwm_[1].output(t), pwm_[2].output(t)};
  }

  std::array<double, 3> duties() const { return duties_; }
  double conductance() const { return g_; }

 private:
  Params p_;
  std::array<control::Pwm, 3> pwm_;
  std::array<control::PiController, 3> piI_;
  control::PiController piV_;
  control::TransferFunction vdcFilt_;
  std::array<double, 3> duties_ = {0.0, 0.0, 0.0};
  double g_ = 0.0;
  long ticks_ = 0;
};

// Hysteretic current controller WITH neutral-point balancing (the classic
// Vienna control structure, and the one with structural balance
// authority): bang-bang per-phase current regulation has no tracking
// integrator, so nothing cancels a deliberate duty asymmetry.
//
// Balance law: shift both hysteresis thresholds of every phase by
// s = alpha * (Vc1 - Vc2) (SIGN-INDEPENDENT). Midpoint low (dm > 0):
//   - phase with i > 0: thresholds up -> stays clamped to the midpoint
//     longer -> more charge injected into node 9;
//   - phase with i < 0 (flipped switch sense): ON region shrinks -> less
//     charge drained from node 9.
// Both polarities cooperate; the shift is small vs the band so the
// currents stay sinusoidal (KCL-projected by the floating neutral).
// Switch sense flips with current polarity because clamping to the
// midpoint raises di/dt for i > 0 but lowers it for i < 0 (OFF parks the
// phase at dc+ resp. dc- through the diodes). Polarity is taken from the
// reference (clean) to avoid chatter at zero crossings.
class HysteresisController {
 public:
  struct Params {
    double dt = 1e-6;      ///< control tick [s] (must match engine step)
    double vdcRef = 750.0; ///< regulated dc bus [V]
    double tEnable = 20e-3;  ///< loop close time [s] (diode pre-charge first)
    double rampTime = 60e-3; ///< Vdc* ramp after enable [s]
    double kpV = 2.55e-4;  ///< outer PI proportional [S/V]
    double kiV = 4.8e-3;   ///< outer PI integral [S/V/s]
    double band = 2.0;     ///< hysteresis half-band... full band is 2*band [A]
    double alpha = 0.1;    ///< balance shift per volt of imbalance [A/V]
    double shiftMax = 1.0; ///< balance shift clamp [A] (< band)
  };

  HysteresisController() : HysteresisController(typename HysteresisController::Params{}) {}
  explicit HysteresisController(const Params& p)
      : p_(p),
        piV_(p.kpV, p.kiV, 0.0, 0.06),
        vdcFilt_(control::TransferFunction::lowPass(40.0, p.dt)),
        midFilt_(control::TransferFunction::lowPass(80.0, p.dt)) {
    piV_.setIntegrator((p.vdcRef * p.vdcRef / 100.0) / (3.0 * 230.0 * 230.0));
    g_ = piV_.update(0.0, p.dt);
  }

  /// One tick. vsrc/i as in Controller, vdc = v:7, vmid = v:9.
  std::array<bool, 3> update(double t, const std::array<double, 3>& vsrc,
                             const std::array<double, 3>& i, double vdc,
                             double vmid) {
    if (t < p_.tEnable) {
      on_ = {false, false, false};
      return on_;
    }
    const double ramp = std::min(1.0, (t - p_.tEnable) / p_.rampTime);
    const double vref = 540.0 + (p_.vdcRef - 540.0) * ramp;
    ++ticks_;
    if (ticks_ % 1000 == 0)
      g_ = piV_.update(vref - vdcFilt_.update(vdc), 1000.0 * p_.dt);
    else
      vdcFilt_.update(vdc);
    // Midpoint error, filtered (must not chase the inherent 150Hz neutral
    // ripple or switching ripple).
    const double dm = midFilt_.update((vdc - vmid) - vmid);  // Vc1 - Vc2
    const double s = std::clamp(p_.alpha * dm, -p_.shiftMax, p_.shiftMax);
    for (int k = 0; k < 3; ++k) {
      const double ref = g_ * vsrc[k];
      const bool pos = ref >= 0.0;
      if (pos) {
        if (i[k] < ref - p_.band + s) on_[k] = true;
        else if (i[k] > ref + p_.band + s) on_[k] = false;
      } else {
        if (i[k] < ref - p_.band + s) on_[k] = false;
        else if (i[k] > ref + p_.band + s) on_[k] = true;
      }
    }
    return on_;
  }

  double conductance() const { return g_; }

 private:
  Params p_;
  control::PiController piV_;
  control::TransferFunction vdcFilt_;
  control::TransferFunction midFilt_;
  std::array<bool, 3> on_ = {false, false, false};
  double g_ = 0.0;
  long ticks_ = 0;
};

}  // namespace vienna
}  // namespace power_engine
