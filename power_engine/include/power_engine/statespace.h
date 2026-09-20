#pragma once
#include <complex>
#include <string>
#include <vector>

#include <Eigen/Dense>

namespace power_engine {

class Circuit;

namespace statespace {

/// Continuous-time state-space model dx/dt = A*x + B*u, y = C*x + D*u.
///
/// States are inductor currents ("i:NAME") and dynamic-node voltages
/// ("v:NODE"). Inputs are grounded independent-source values (device names)
/// plus a constant "1" input carrying affine terms (diode Vf drops).
/// Outputs are probed node voltages ("v:N").
struct StateSpace {
  Eigen::MatrixXd a;
  Eigen::MatrixXd b;
  Eigen::MatrixXd c;
  Eigen::MatrixXd d;
  std::vector<std::string> stateNames;
  std::vector<std::string> inputNames;
  std::vector<std::string> outputNames;
};

/// Export the continuous-time state-space of a circuit with FROZEN
/// switch/diode states (set gates/conduction first; one export per
/// switching configuration).
///
/// Supported: R, L, C, grounded V-sources, I-sources, Switch, Diode.
/// Restrictions (thrown otherwise): no floating V-sources, no transformers
/// or coupled/saturable magnetics, no active recovery/tail, nonsingular
/// algebraic conductance block (no floating nodes) and nonsingular
/// capacitance block (no capacitor-only loops with sources, no inductor
/// cutsets on current sources) — the normal power-converter case.
StateSpace exportStateSpace(const Circuit& circuit,
                            const std::vector<std::string>& outputs);

/// State-space averaging for PWM: A,B,C,D averaged by duty. Requires
/// matching dimensions and names (same circuit, two switch states).
StateSpace averageStateSpace(const StateSpace& on, const StateSpace& off, double duty);

/// SISO transfer function H(s) = C[out,:]*(sI-A)^-1*B[:,in] + D[out,in].
std::complex<double> evalTransfer(const StateSpace& ss, int outIdx, int inIdx,
                                  std::complex<double> s);

/// Duty-to-output transfer of the averaged PWM model (state-space averaging):
/// Gvd(s) = C*(sI-Aavg)^-1*((Aon-Aoff)*X + (Bon-Boff)*u), with the steady
/// operating point X solved from the averaged model at input values uSs.
/// Needs matching on/off models (same circuit, two switch states).
std::complex<double> evalDutyTransfer(const StateSpace& on, const StateSpace& off,
                                      double duty, const Eigen::VectorXd& uSs,
                                      int outIdx, std::complex<double> s);

}  // namespace statespace
}  // namespace power_engine
