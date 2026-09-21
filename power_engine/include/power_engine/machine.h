#pragma once
#include <stdexcept>
#include <string>
#include <vector>

namespace power_engine {
namespace machine {

/// Lumped mechanical load: inertia J [kg m^2] (> 0), viscous friction B
/// [N m s] (>= 0, torque B*omega opposing motion).
struct MechanicalParams {
  double j = 0.0;
  double b = 0.0;
};

/// Rotor state: speed omega [rad/s], position theta [rad, unwrapped].
struct MechanicalState {
  double omega = 0.0;
  double theta = 0.0;
};

/// Exact advance over dt [s] under constant torques (Te motor, Tl load):
/// first-order J/B split integrated in closed form (unconditionally
/// stable, exact regardless of dt; B = 0 falls back to uniform accel).
void stepMechanical(MechanicalState& st, double te, double tload,
                    const MechanicalParams& p, double dt);

/// Surface/interior PMSM parameters (SI). Phase-domain simulation uses
/// per-phase Rs/Ls plus sinusoidal back-EMF below; Ld/Lq document the
/// dq model (surface: Ld == Lq) for reference calculations.
struct PmsmParams {
  int polePairs = 1;
  double lambdaPm = 0.0;  ///< magnet flux linkage [Wb]
  double rs = 0.0;        ///< phase resistance [Ohm]
  double ld = 0.0;        ///< d-axis inductance [H]
  double lq = 0.0;        ///< q-axis inductance [H]
  MechanicalParams mech;
};

/// Three-phase quantity (phase order a, b, c).
struct ThreePhase {
  double a = 0.0, b = 0.0, c = 0.0;
};

/// Sinusoidal back-EMF at rotor (theta, omega): flux lambda_a =
/// lambdaPm*cos(p*theta) so ea = p*omega*lambdaPm*sin(p*theta), phases at
/// 0/-120/+120 deg electrical.
ThreePhase pmsmEmf(double theta, double omega, const PmsmParams& m);

/// Amplitude-invariant Park transform at electrical angle thE.
/// Convention (fixed so motoring torque is positive): with phase currents
/// ia = I*sin(thE) (and b/c at -/+120deg), id = 0 and iq = +I.
void park(double ia, double ib, double ic, double thE, double& id, double& iq);

/// Electromagnetic torque [N m] from dq currents (exact for sinusoidal
/// machines, no speed singularity): Te = 3/2*p*(lambdaPm*iq +
/// (ld-lq)*id*iq). For surface PMSM with id = 0: Te = 3/2*p*lambdaPm*iq.
double pmsmTorque(double id, double iq, const PmsmParams& m);

/// Squirrel-cage induction machine (dq synchronous frame, SI).
struct InductionParams {
  double rs = 0.0;  ///< stator resistance [Ohm]
  double rr = 0.0;  ///< rotor resistance, referred [Ohm]
  double ls = 0.0;  ///< stator inductance [H]
  double lr = 0.0;  ///< rotor inductance, referred [H]
  double lm = 0.0;  ///< magnetizing inductance [H]
  int polePairs = 1;
  MechanicalParams mech;
};

/// dq flux-linkage state [Wb] (synchronous frame): stator d/q, rotor d/q.
struct InductionState {
  double fds = 0.0, fqs = 0.0, fdr = 0.0, fqr = 0.0;
};

/// Advance the induction flux state over dt [s] by RK4 given stator
/// voltages (vds, vqs), frame speed we [rad/s] and rotor speed wr [rad/s].
/// Electrical transients are ms-scale; dt <= 100us is plenty (switching
/// drives use the electrical dt for edge resolution instead).
void stepInduction(InductionState& st, double vds, double vqs, double we,
                   double wr, const InductionParams& m, double dt);

/// Stator/rotor dq currents from flux state (2x2 block solve).
void inductionCurrents(const InductionState& st, const InductionParams& m,
                       double& ids, double& iqs, double& idr, double& iqr);

/// Electromagnetic torque from fluxes/currents: Te = 3/2*p*(fds*iqs -
/// fqs*ids). Exact, no singularity.
double inductionTorque(const InductionState& st, double ids, double iqs,
                       const InductionParams& m);

/// Steady-state torque [N m] at slip s from the exact per-phase equivalent
/// circuit (vPhaseRms, freqHz): air-gap power through Rr/s. Reference for
/// validating dynamic runs (pullout included): solve T(s) = load for s.
double inductionSteadyTorque(const InductionParams& m, double vPhaseRms,
                             double freqHz, double slip);

}  // namespace machine
}  // namespace power_engine
