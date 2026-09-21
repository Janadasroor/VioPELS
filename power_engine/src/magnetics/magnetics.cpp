#include "power_engine/magnetics.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

#include <Eigen/Dense>

namespace power_engine {
namespace magnetics {
namespace {

constexpr double kPi = std::numbers::pi;

}  // namespace

HysteresisCore::HysteresisCore(HystereticMaterial mat) : mat_(mat) {
  if (!(mat_.bs > 0.0) || !std::isfinite(mat_.bs)) {
    throw std::runtime_error("hysteresis Bs must be positive finite");
  }
  if (!(mat_.a > 0.0) || !std::isfinite(mat_.a)) {
    throw std::runtime_error("hysteresis shape field a must be positive finite");
  }
  if (!(mat_.hc >= 0.0) || !std::isfinite(mat_.hc)) {
    throw std::runtime_error("hysteresis Hc must be finite >= 0");
  }
}

void HysteresisCore::reset() {
  s_ = 0.0;
  hPrev_ = 0.0;
  bPrev_ = 0.0;
  loss_ = 0.0;
}

double HysteresisCore::anhysteretic(double h) const {
  return mat_.bs * std::tanh(h / mat_.a);
}

double HysteresisCore::ascending(double h) const {
  return mat_.bs * std::tanh((h - mat_.hc) / mat_.a);
}

double HysteresisCore::descending(double h) const {
  return mat_.bs * std::tanh((h + mat_.hc) / mat_.a);
}

double HysteresisCore::update(double h) {
  if (!std::isfinite(h)) throw std::runtime_error("hysteresis field must be finite");
  // Saturating offset memory: track +H into [-Hc, +Hc]. Major limbs sit at
  // the clamps (bare branches); reversals walk s back (lens-shaped minors
  // that close exactly); pushing into a clamp wipes outer memory.
  s_ = std::clamp(s_ + (h - hPrev_), -mat_.hc, mat_.hc);
  const double b = mat_.bs * std::tanh((h - s_) / mat_.a);
  // Hysteresis energy density increment (trapezoid on the H-B plane).
  loss_ += 0.5 * (h + hPrev_) * (b - bPrev_);
  hPrev_ = h;
  bPrev_ = b;
  return b;
}

double eddyLossDensity(double rho, double thickness, double freqHz, double bPeak) {
  if (!(rho > 0.0) || !std::isfinite(rho)) {
    throw std::runtime_error("eddy resistivity must be positive finite");
  }
  if (!(thickness > 0.0) || !std::isfinite(thickness)) {
    throw std::runtime_error("eddy thickness must be positive finite");
  }
  if (!(freqHz >= 0.0) || !std::isfinite(freqHz)) {
    throw std::runtime_error("eddy frequency must be finite >= 0");
  }
  if (!(bPeak >= 0.0) || !std::isfinite(bPeak)) {
    throw std::runtime_error("eddy Bpk must be finite >= 0");
  }
  return kPi * kPi / (6.0 * rho) * thickness * thickness * freqHz * freqHz * bPeak * bPeak;
}

// ---------------- ReluctanceNetwork ----------------

int ReluctanceNetwork::nodeIndex(int node) const {
  if (node == 0) return -1;
  auto it = nodeIndex_.find(node);
  if (it == nodeIndex_.end()) throw std::runtime_error("reluctance: unknown node");
  return it->second;
}

void ReluctanceNetwork::addReluctance(const std::string& name, int n1, int n2, double r) {
  if (name.empty()) throw std::runtime_error("reluctance needs a name");
  if (!(r > 0.0) || !std::isfinite(r)) {
    throw std::runtime_error("reluctance '" + name + "' must be positive finite");
  }
  if (branchByName_.count(name) != 0u) {
    throw std::runtime_error("duplicate reluctance '" + name + "'");
  }
  branchByName_[name] = branches_.size();
  branches_.push_back({name, n1, n2});
  branches_.back().r = r;
  solved_ = false;
}

void ReluctanceNetwork::addSaturableReluctance(const std::string& name, int n1, int n2,
                                               double l, double area, double bs, double a) {
  if (name.empty()) throw std::runtime_error("reluctance needs a name");
  if (!(l > 0.0) || !std::isfinite(l) || !(area > 0.0) || !std::isfinite(area) ||
      !(bs > 0.0) || !std::isfinite(bs) || !(a > 0.0) || !std::isfinite(a)) {
    throw std::runtime_error("saturable reluctance '" + name + "' needs positive finite l/A/Bs/a");
  }
  if (branchByName_.count(name) != 0u) {
    throw std::runtime_error("duplicate reluctance '" + name + "'");
  }
  branchByName_[name] = branches_.size();
  branches_.push_back({name, n1, n2, true, 0.0, l, area, bs, a, 0.0});
  solved_ = false;
}

void ReluctanceNetwork::addWinding(const std::string& name, const std::string& branch,
                                   double turns) {
  if (name.empty()) throw std::runtime_error("winding needs a name");
  if (!std::isfinite(turns) || turns == 0.0) {
    throw std::runtime_error("winding '" + name + "' needs finite nonzero turns");
  }
  if (branchByName_.find(branch) == branchByName_.end()) {
    throw std::runtime_error("winding '" + name + "': unknown branch '" + branch + "'");
  }
  if (windings_.count(name) != 0u) throw std::runtime_error("duplicate winding '" + name + "'");
  windings_[name] = {name, branch, turns, 0.0};
  solved_ = false;
}

void ReluctanceNetwork::setWindingCurrent(const std::string& name, double current) {
  auto it = windings_.find(name);
  if (it == windings_.end()) throw std::runtime_error("unknown winding '" + name + "'");
  if (!std::isfinite(current)) throw std::runtime_error("winding current must be finite");
  it->second.current = current;
  solved_ = false;
}

void ReluctanceNetwork::solve() {
  if (branches_.empty()) throw std::runtime_error("reluctance network has no branches");
  // Node indexing (ground excluded).
  nodeIndex_.clear();
  for (const auto& b : branches_) {
    for (int n : {b.n1, b.n2}) {
      if (n != 0 && nodeIndex_.find(n) == nodeIndex_.end()) {
        nodeIndex_[n] = static_cast<int>(nodeIndex_.size());
      }
    }
  }
  const int n = static_cast<int>(nodeIndex_.size());
  // Winding MMF per branch (N*i summed over windings on it).
  auto mmf = [&](const Branch& b) {
    double f = 0.0;
    for (const auto& [wn, w] : windings_) {
      (void)wn;
      if (w.branch == b.name) f += w.turns * w.current;
    }
    return f;
  };
  // Branch flux for given terminal drop Vb: linear is direct; saturable
  // is closed-form too (Vb + m = Phi*R(Phi) = l*H exactly, then B + Phi
  // from the tanh curve — no iteration, unbounded MMF fine).
  auto branchPhi = [&](const Branch& b, double vb, double m) {
    if (!b.saturable) return (vb + m) / b.r;
    const double h = (vb + m) / b.l;
    return b.area * b.bs * std::tanh(h / b.a);
  };
  // Newton on node potentials with exact branch evaluations: KCL residual
  // r(U) = 0 via branchPhi (closed-form per branch, linear or saturable).
  // Analytic Jacobian (differential permeances) + backtracking; warm-start
  // from the previous solution (co-simulation steps land nearby). Linear
  // networks converge in one iteration (Jacobian exact up to FP).
  auto residual = [&](const Eigen::VectorXd& U) {
    Eigen::VectorXd r = Eigen::VectorXd::Zero(n);
    for (const auto& b : branches_) {
      const int r1 = nodeIndex(b.n1), r2 = nodeIndex(b.n2);
      const double u1 = r1 >= 0 ? U(r1) : 0.0;
      const double u2 = r2 >= 0 ? U(r2) : 0.0;
      const double f = branchPhi(b, u1 - u2, mmf(b));  // flux n1 -> n2
      if (r1 >= 0) r(r1) += f;                         // leaving n1
      if (r2 >= 0) r(r2) -= f;                         // entering n2
    }
    return r;
  };
  // Differential permeance dPhi/dVb (exact): linear 1/R; saturable from
  // d/dVb[A*Bs*tanh(((Vb+m)/l)/a)].
  auto diffPerm = [&](const Branch& b, double vb, double m) {
    if (!b.saturable) return 1.0 / b.r;
    const double c = std::cosh((vb + m) / (b.l * b.a));
    return b.area * b.bs / (b.l * b.a * c * c);
  };
  Eigen::VectorXd u = Eigen::VectorXd::Zero(n);
  if (potentials_.size() == static_cast<std::size_t>(n) && n > 0) {
    u = Eigen::Map<Eigen::VectorXd>(potentials_.data(), n);
  }
  double fScale = 0.0;
  for (const auto& b : branches_) fScale += std::abs(mmf(b)) + 1.0;
  for (int iter = 0; iter < 100; ++iter) {
    const Eigen::VectorXd r = residual(u);
    if (r.cwiseAbs().maxCoeff() <= 1e-12 * fScale) break;
    if (iter == 99) throw std::runtime_error("reluctance network did not converge");
    Eigen::MatrixXd jac = Eigen::MatrixXd::Zero(n, n);
    for (const auto& b : branches_) {
      const int r1 = nodeIndex(b.n1), r2 = nodeIndex(b.n2);
      const double u1 = r1 >= 0 ? u(r1) : 0.0;
      const double u2 = r2 >= 0 ? u(r2) : 0.0;
      const double p = diffPerm(b, u1 - u2, mmf(b));
      if (r1 >= 0) jac(r1, r1) += p;
      if (r2 >= 0) jac(r2, r2) += p;
      if (r1 >= 0 && r2 >= 0) {
        jac(r1, r2) -= p;
        jac(r2, r1) -= p;
      }
    }
    Eigen::FullPivLU<Eigen::MatrixXd> lu(jac);
    if (!lu.isInvertible()) {
      throw std::runtime_error("reluctance network singular (floating node?)");
    }
    const Eigen::VectorXd step = lu.solve(-r);
    const double r0 = r.cwiseAbs().maxCoeff();
    bool stepped = false;
    for (double alpha = 1.0; alpha >= 1e-12; alpha *= 0.5) {
      const Eigen::VectorXd trial = u + alpha * step;
      if (residual(trial).cwiseAbs().maxCoeff() <= (1.0 - 1e-4 * alpha) * r0) {
        u = trial;
        stepped = true;
        break;
      }
    }
    if (!stepped) throw std::runtime_error("reluctance line search failed");
  }
  for (auto& b : branches_) {
    const int r1 = nodeIndex(b.n1), r2 = nodeIndex(b.n2);
    const double u1 = r1 >= 0 ? u(r1) : 0.0;
    const double u2 = r2 >= 0 ? u(r2) : 0.0;
    b.flux = branchPhi(b, u1 - u2, mmf(b));
  }
  potentials_ = std::vector<double>(u.data(), u.data() + u.size());
  solved_ = true;
}

double ReluctanceNetwork::branchFlux(const std::string& branch) const {
  if (!solved_) throw std::runtime_error("reluctance network not solved");
  auto it = branchByName_.find(branch);
  if (it == branchByName_.end()) throw std::runtime_error("unknown branch '" + branch + "'");
  return branches_[it->second].flux;
}

double ReluctanceNetwork::windingFlux(const std::string& name) const {
  if (!solved_) throw std::runtime_error("reluctance network not solved");
  auto it = windings_.find(name);
  if (it == windings_.end()) throw std::runtime_error("unknown winding '" + name + "'");
  return branchFlux(it->second.branch);
}

double ReluctanceNetwork::equivalentInductance(const std::string& name) {
  auto it = windings_.find(name);
  if (it == windings_.end()) throw std::runtime_error("unknown winding '" + name + "'");
  for (const auto& b : branches_) {
    if (b.saturable) {
      throw std::runtime_error("equivalentInductance needs a linear network (co-simulate instead)");
    }
  }
  // Save currents, energize with 1A, restore, mark stale.
  std::map<std::string, double> saved;
  for (const auto& [wn, w] : windings_) {
    (void)wn;
    saved[w.name] = w.current;
  }
  for (auto& [wn, w] : windings_) {
    (void)wn;
    w.current = (w.name == name) ? 1.0 : 0.0;
  }
  solve();
  const double lambda = it->second.turns * windingFlux(name);
  for (auto& [wn, w] : windings_) {
    (void)wn;
    w.current = saved[w.name];
  }
  solved_ = false;
  return lambda;  // per 1A excitation
}

}  // namespace magnetics
}  // namespace power_engine
