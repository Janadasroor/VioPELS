#pragma once
#include <cmath>
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
  Transformer,  ///< Ideal transformer, algebraic: Vp = ratio*Vs, ratio*Ip + Is = 0.
  CoupledInductor,  ///< Two windings with mutual M (trapezoidal 2-port Norton).
  SatInductor,  ///< Saturable inductor: lambda(i) = Lsat*i + (Lunsat-Lsat)*Isat*tanh(i/Isat).
};

/// Single two-terminal device (four-terminal for Transformer/CoupledInductor).
/// Value meaning depends on type: R[Ohm], C[F], L[H], V[Volt], I[Amp].
/// Switch uses ron/roff + closed; Diode uses ron/roff/vf + conducting.
/// For Diode, n1 = anode, n2 = cathode. For Transformer, (n1,n2) = primary,
/// (n3,n4) = secondary with Vp/Vs = ratio (DC passes: ideal algebraic model,
/// no magnetizing inductance or saturation). CoupledInductor uses l1/l2/m
/// with dots at n1 and n3: v1 = L1 di1/dt + M di2/dt (0 < k < 1 required).
/// SatInductor uses value=Lunsat plus lsat/isat; flux holds λ(i_n).
struct Device {
  DeviceType type = DeviceType::Resistor;
  std::string name;
  int n1 = 0;  // 0 = ground
  int n2 = 0;
  int n3 = 0;  // transformer secondary + / coupled winding 2 +
  int n4 = 0;  // transformer secondary - / coupled winding 2 -
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

  // Per-step history for trapezoidal companion models (solver-managed).
  double v_prev = 0.0;   // V(n1)-V(n2) at previous step
  double i_prev = 0.0;   // current n1->n2 at previous step
  double i2_prev = 0.0;  // transformer secondary current n3->n4
  // Coupled-inductor second winding history.
  double v2_prev = 0.0;  // V(n3)-V(n4) at previous step

  // Switch / diode parameters (unused by other types).
  double ron = 5e-3;     // on-resistance [Ohm]
  double roff = 1e6;     // off-resistance [Ohm]
  double vf = 0.0;       // diode forward drop [Volt]
  double eon = 0.0;      // switch turn-on energy per edge [J]
  double eoff = 0.0;     // switch turn-off energy per edge [J]
  bool closed = false;      // switch gate state (true = on/Ron)
  bool conducting = false;  // diode state (true = on/Ron+Vf)
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
  bool closedPrev = false;  // gate state at previous step (edge detection)
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

}  // namespace power_engine
