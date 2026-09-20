#pragma once
#include <vector>

namespace power_engine {
namespace thermal {

/// One RC stage: R [K/W], C [J/K].
struct Stage {
  double r = 0.0;
  double c = 0.0;
};

/// Lumped thermal network driven by device power dissipation.
///
/// Foster (series of parallel R-C stages, behavioral):
///   C_i dT_i/dt + T_i/R_i = P,   Tj = Tamb + sum(T_i).
///   Stepped with the EXACT update for piecewise-constant P:
///   T_i += (P*R_i - T_i) * (1 - exp(-dt/tau_i)), tau_i = R_i*C_i.
///   Unconditionally stable, exact regardless of dt.
///
/// Cauer (ladder, physical ordering hot -> ambient):
///   C_1 dT_1/dt = P - (T_1-T_2)/R_1
///   C_i dT_i/dt = (T_{i-1}-T_i)/R_{i-1} - (T_i-T_{i+1})/R_i
///   C_n dT_n/dt = (T_{n-1}-T_n)/R_{n-1} - (T_n-Tamb)/R_n
///   Tj = T_1, stepped with explicit Euler (solver dt ~us << thermal
///   time constants ~ms/s, so truncation error is negligible; advancing
///   with the solver sub-step keeps it exact in time).
///
/// Both share the steady state Tj = Tamb + P * sum(R).
class ThermalNetwork {
 public:
  enum class Kind { Foster, Cauer };

  static ThermalNetwork foster(std::vector<Stage> stages, double tamb);
  static ThermalNetwork cauer(std::vector<Stage> stages, double tamb);

  void reset();
  /// Advance by dt [s] under constant power P [W].
  void step(double power, double dt);
  /// Junction temperature [degC].
  double tj() const;
  double tamb() const { return tamb_; }
  Kind kind() const { return kind_; }
  const std::vector<Stage>& stages() const { return stages_; }

 private:
  ThermalNetwork(Kind kind, std::vector<Stage> stages, double tamb);

  Kind kind_;
  std::vector<Stage> stages_;
  double tamb_ = 25.0;
  std::vector<double> states_;  // Foster: stage rises; Cauer: node temps
};

/// Accumulated energies for one switching device.
struct DeviceLoss {
  double econd = 0.0;  ///< conduction energy [J], integral of v*i
  double esw = 0.0;    ///< switching energy [J], sum of Eon/Eoff edges
  double total() const { return econd + esw; }
};

}  // namespace thermal
}  // namespace power_engine
