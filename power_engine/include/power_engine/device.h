#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace power_engine {

/// Supported device kinds.
enum class DeviceType {
  Resistor,
  Capacitor,
  Inductor,
  VoltageSource,
  CurrentSource,
  Switch,  ///< Ideal switch stamped as Ron/Roff, gate-controlled.
  Diode,   ///< Ideal diode: Ron+Vf when on, Roff when off, auto-commutated.
  SwitchDiode,  ///< Half-bridge atom: gate-controlled switch (n1->n2 Ron
  ///< when closed) with anti-parallel diode (anode n2, cathode n1) when
  ///< open. Closed: Ron (+ tail/tsw machinery like Switch). Open:
  ///< auto-commutated diode like Diode, incl. Qrr recovery (recovery
  ///< current flows n1->n2, i.e. positive in branch reference). Gate
  ///< edges book Eon/Eoff like Switch; diode conduction is exact v*i.
  ///< tsw shapes turn-on into a blocking (non-conducting) state only:
  ///< closing onto a conducting diode commutates ideally (instant).
  Transformer,  ///< Ideal transformer, algebraic: Vp = ratio*Vs, ratio*Ip + Is = 0.
  CenterTapTransformer,  ///< Ideal 3-winding center-tap (push-pull):
  ///< (n1,n2) = half A (end, center-tap), (n2,n3) = half B (tap, end),
  ///< (n4,n5) = secondary. Shared-core constraints with ratio n =
  ///< Np_half/Ns: (VA-VCT) - n*Vs = 0, (VCT-VB) - n*Vs = 0,
  ///< n*(IA + IB) + IS = 0 (power conservation; IA: n1->n2, IB: n2->n3,
  ///< IS: n4->n5). DC passes like Transformer (ideal algebraic model,
  ///< no magnetizing inductance or saturation). The off half flies to
  ///< 2*Vin — what two independent 2-winding parts cannot reproduce.
  CoupledInductor,  ///< Two windings with mutual M (trapezoidal 2-port Norton).
  SatInductor,  ///< Saturable inductor: lambda(i) = Lsat*i + (Lunsat-Lsat)*Isat*tanh(i/Isat).
  HystereticInductor,  ///< Hysteretic inductor: N-turn winding on a tanh
  ///< B-H core (Preisach-style memory offset), explicit companion, no
  ///< Newton. State advances only on committed steps (no iteration
  ///< pollution); loss accumulates the H-B loop area like HysteresisCore.
};

/// Single two-terminal device (four-terminal for Transformer/CoupledInductor).
/// Value meaning depends on type: R[Ohm], C[F], L[H], V[Volt], I[Amp].
/// Switch uses ron/roff + closed; Diode uses ron/roff/vf + conducting.
/// For Diode, n1 = anode, n2 = cathode. For Transformer, (n1,n2) = primary,
/// (n3,n4) = secondary with Vp/Vs = ratio (DC passes: ideal algebraic model,
/// no magnetizing inductance or saturation). CoupledInductor uses l1/l2/m
/// with dots at n1 and n3: v1 = L1 di1/dt + M di2/dt (0 < k < 1 required).
/// SatInductor uses value=Lunsat plus lsat/isat; flux holds λ(i_n).
/// HystereticInductor uses hTurns/hAe/hLe/hVe/hBs/hA/hHc; hH/hS/hB hold
/// the (H, memory, B) state, hLoss the accumulated J/m^3 loop area.
struct Device {
  DeviceType type = DeviceType::Resistor;
  std::string name;
  int n1 = 0;  // 0 = ground
  int n2 = 0;
  int n3 = 0;  // transformer secondary + / coupled winding 2 +
  int n4 = 0;  // transformer secondary - / coupled winding 2 -
  int n5 = 0;  // center-tap secondary - (0 = ground)
  double value = 0.0;
  double ratio = 1.0;  // transformer turns ratio (primary:secondary)
  double ic = 0.0;     // initial condition: Vc(0) for C, Il(0) for L / coupled i1
  double ic2 = 0.0;    // coupled winding-2 initial current
  double l1 = 0.0;     // coupled winding-1 self inductance [H]
  double l2 = 0.0;     // coupled winding-2 self inductance [H]
  double m = 0.0;      // mutual inductance [H], M = k*sqrt(L1*L2)
  double lsat = 0.0;   // saturable slope inductance [H]
  double isat = 0.0;   // saturable knee current [A]
  double flux = 0.0;   // flux linkage λ(i) at previous step [Wb]
  double newton_ik = 0.0;  // Newton operating-point current (solver-managed)
  // Hysteretic-inductor core (unused by other types; solver-managed
  // state hH/hS/hB/hLoss advances only on committed steps).
  double hTurns = 0.0;  // winding turns N
  double hAe = 0.0;     // core cross-section [m^2]
  double hLe = 0.0;     // core path length [m]
  double hVe = 0.0;     // core volume [m^3] (loss reporting)
  double hBs = 0.0;     // saturation flux density [T]
  double hA = 0.0;      // tanh shape field [A/m]
  double hHc = 0.0;     // coercivity [A/m] (0 = anhysteretic)
  double hH = 0.0;      // field H = N*i/le at previous commit [A/m]
  double hS = 0.0;      // memory offset in [-Hc, +Hc]
  double hB = 0.0;      // flux density at previous commit [T]
  double hLoss = 0.0;   // accumulated loop-area density [J/m^3]
  double fluxMid = 0.0;  // TR-BDF2 midpoint flux linkage [Wb] (stage scratch)

  // Per-step history for trapezoidal companion models (solver-managed).
  double v_prev = 0.0;   // V(n1)-V(n2) at previous step
  double i_prev = 0.0;   // current n1->n2 at previous step
  double i2_prev = 0.0;  // transformer secondary current n3->n4
  double i3_prev = 0.0;  // center-tap only: secondary current n4->n5
                         // (i_prev = IA, i2_prev = IB for that type)
  // Coupled-inductor second winding history.
  double v2_prev = 0.0;  // V(n3)-V(n4) at previous step
  // TR-BDF2 intra-step midpoint state (solver-managed scratch: captured
  // after stage 1, consumed by stage 2, never carried across steps).
  double v_mid = 0.0;    // V(n1)-V(n2) at the half step
  double i_mid = 0.0;    // current n1->n2 at the half step
  double v2_mid = 0.0;   // V(n3)-V(n4) at the half step
  double i2_mid = 0.0;   // secondary current at the half step
  double ik_mid = 0.0;   // Newton operating point at the half step (sat)

  // Switch / diode parameters (unused by other types).
  double ron = 5e-3;     // on-resistance [Ohm]
  double roff = 1e6;     // off-resistance [Ohm]
  double vf = 0.0;       // diode forward drop [Volt]
  double vbr = std::numeric_limits<double>::infinity();  // Zener reverse
  ///< breakdown [V]; inf = ideal blocking. Slope rbr below.
  double rbr = 0.0;      // Zener breakdown slope [Ohm]; 0 = follow ron
  double eon = 0.0;      // switch turn-on energy per edge [J]
  double eoff = 0.0;     // switch turn-off energy per edge [J]
  bool closed = false;      // switch gate state (true = on/Ron)
  bool conducting = false;  // diode state (true = on/Ron+Vf)
  bool breakdown = false;   // Zener reverse-breakdown state (solver-managed)
  // Reverse-recovery (diode) / tail-current (switch) model:
  // triangular recovery Qrr/trr, exponential tail ttail/tailk.
  // Fixed-shape approximations (no di/dt dependence), documented.
  double qrr = 0.0;   // diode recovery charge [C], 0 = ideal
  double trr = 0.0;   // diode recovery time [s]
  double ttail = 0.0;  // switch tail time constant [s], 0 = none
  double tailk = 0.1;  // tail amplitude fraction of turn-off current
  // Recovery state (solver-managed).
  double recT = 0.0;  // remaining recovery/tail time [s]
  double recI = 0.0;  // recovery amplitude [A, branch reference direction]
  double recE = 0.0;  // pending release energy [J], consumed by Engine losses
  bool recTail = false;  // SwitchDiode only: recT/recI is a switch tail
                         // (exponential) rather than diode recovery
                         // (triangular). z-only, never salted.
  bool closedPrev = false;  // gate state at previous step (edge detection)
  // Slew-limited switching transition (PAT-style behavioral edge, 0 = ideal
  // instant). On a gate toggle the resistance sweeps geometrically from its
  // present value to the new steady value over tsw [s] (constant ratio per
  // step — linear-in-R would jump decades on the first step and re-excite
  // the ringing the ramp removes; MNA stays linear; no Miller plateau by
  // design). The transition starts in the toggle step itself (frac = 0 =
  // old R, so no ideal-commutation violence precedes it). The v*i integral over the ramp books into conduction loss
  // exactly — do NOT combine with Eon/Eoff tables for the same edge
  // (double counting). Solver-managed during the ramp.
  double tsw = 0.0;        // transition time [s], 0 = ideal
  double transT = 0.0;     // remaining transition time [s]
  double transFrom = 0.0;  // resistance at toggle [Ohm]
  double transTo = 0.0;    // target resistance [Ohm]
  // Datasheet loss-table references (netlist .etable names, UPPER-cased).
  // Empty = inactive (scalar eon/eoff/ron/vf used). Resolved by
  // Engine::applyLossModels into DeviceLossModel attachments.
  std::string eonTable;   // switch turn-on energy E(I,V,TJ)
  std::string eoffTable;  // switch turn-off energy E(I,V,TJ)
  std::string ronTable;   // on-resistance R(TJ), switch + diode
  std::string vfTable;    // forward drop VF(TJ), diode
};

/// Saturable-inductor flux linkage shared by Circuit (init) and the
/// solver's Newton loop — single definition on purpose.
/// λ(i) = Lsat*i + (Lunsat-Lsat)*Isat*tanh(i/Isat).
inline double satFlux(double i, double lunsat, double lsat, double isat) {
  return lsat * i + (lunsat - lsat) * isat * std::tanh(i / isat);
}

/// Differential inductance dλ/di (always in [Lsat, Lunsat]).
inline double satSlope(double i, double lunsat, double lsat, double isat) {
  const double s = 1.0 / std::cosh(i / isat);
  return lsat + (lunsat - lsat) * s * s;
}

/// Hysteretic-inductor linkage shared by Circuit (init) and the solver's
/// explicit companion — single definition on purpose.
/// B(H) = Bs*tanh((H-s)/a) on memory offset s; λ = N*Ae*B, H = N*i/le.
inline double hystFlux(double i, double s, double turns, double ae, double le, double bs,
                       double a) {
  const double h = turns * i / le;
  return turns * ae * bs * std::tanh((h - s) / a);
}

/// Tangent dλ/di at (i, s): N^2*Ae*Bs/(a*le)*sech^2((H-s)/a) (always > 0).
inline double hystTangent(double i, double s, double turns, double ae, double le, double bs,
                          double a) {
  const double t = std::tanh((turns * i / le - s) / a);
  return turns * turns * ae * bs / (a * le) * (1.0 - t * t);
}

/// Companion tangent with air-core floor: the tanh slope -> 0 in deep
/// saturation would singularize MNA (G = dt/(2Lt) explodes); a saturated
/// core is still mu0. Single definition for stamp, signature salt, and
/// histories — all three must agree bit-for-bit.
inline double hystLt(double i, double s, double turns, double ae, double le, double bs,
                     double a) {
  constexpr double kMu0 = 4.0 * 3.141592653589793e-7;
  const double lt = hystTangent(i, s, turns, ae, le, bs, a);
  const double air = turns * turns * ae * kMu0 / le;
  return lt > air ? lt : air;
}

/// Advance (H, s, B, loss) one committed segment (mirrors HysteresisCore:
/// saturating offset memory + trapezoid H-B area; idempotent in H).
inline void hystAdvance(double hNew, double& hH, double& hS, double& hB, double& hLoss,
                        double hc, double bs, double a) {
  hS = std::clamp(hS + (hNew - hH), -hc, hc);
  const double b = bs * std::tanh((hNew - hS) / a);
  hLoss += 0.5 * (hNew + hH) * (b - hB);
  hH = hNew;
  hB = b;
}

}  // namespace power_engine
