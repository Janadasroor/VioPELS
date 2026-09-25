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
constexpr double kMu0 = 4.0 * std::numbers::pi * 1e-7;

void reqPosFin(double v, const char* what) {
  if (!(v > 0.0) || !std::isfinite(v)) throw std::runtime_error(what);
}

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

double dowellFactor(int layers, double delta) {
  if (layers < 1) throw std::runtime_error("dowell needs layers >= 1");
  if (!(delta >= 0.0) || !std::isfinite(delta))
    throw std::runtime_error("dowell needs delta finite >= 0");
  if (delta < 1e-3) return 1.0;  // series limit (exact to ~1e-12)
  const double m = static_cast<double>(layers);
  const double t1 =
      (std::sinh(2.0 * delta) + std::sin(2.0 * delta)) / (std::cosh(2.0 * delta) - std::cos(2.0 * delta));
  const double t2 = (2.0 * (m * m - 1.0) / 3.0) *
                    (std::sinh(delta) - std::sin(delta)) / (std::cosh(delta) + std::cos(delta));
  return delta * (t1 + t2);
}

double eddyLossDensity(double rho, double thickness, double freqHz, double bPeak) {
  if (!(rho > 0.0) || !std::isfinite(rho)) {
    throw std::runtime_error("eddy resistivity must be positive finite");
  }
  if (!(thickness >= 0.0) || !std::isfinite(thickness)) {
    throw std::runtime_error("eddy thickness must be finite >= 0 (0 = no laminations)");
  }
  if (!(freqHz >= 0.0) || !std::isfinite(freqHz)) {
    throw std::runtime_error("eddy frequency must be finite >= 0");
  }
  if (!(bPeak >= 0.0) || !std::isfinite(bPeak)) {
    throw std::runtime_error("eddy Bpk must be finite >= 0");
  }
  return kPi * kPi / (6.0 * rho) * thickness * thickness * freqHz * freqHz * bPeak * bPeak;
}

double windingResistivityAtTemp(const WindingSpec& w, double tempC) {
  if (!std::isfinite(tempC)) throw std::runtime_error("winding temp must be finite [degC]");
  if (!(w.tempAlpha >= 0.0) || !std::isfinite(w.tempAlpha))
    throw std::runtime_error("winding spec needs tempAlpha finite >= 0");
  return w.resistivity * (1.0 + w.tempAlpha * (tempC - 20.0));
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

InductorDesign designGappedInductor(const InductorSpec& spec, const CoreGeometry& core,
                                    const CoreMaterial& mat, const WindingSpec& winding) {
  reqPosFin(spec.inductance, "inductor spec needs inductance > 0");
  reqPosFin(spec.iPeak, "inductor spec needs iPeak > 0");
  if (!(spec.iRms >= 0.0) || !std::isfinite(spec.iRms))
    throw std::runtime_error("inductor spec needs iRms finite >= 0");
  if (!(spec.iRipplePkPk >= 0.0) || !std::isfinite(spec.iRipplePkPk))
    throw std::runtime_error("inductor spec needs iRipplePkPk finite >= 0");
  reqPosFin(spec.freqHz, "inductor spec needs freqHz > 0");
  if (!(spec.bMaxMargin > 0.0) || !(spec.bMaxMargin < 1.0) || !std::isfinite(spec.bMaxMargin))
    throw std::runtime_error("inductor spec needs bMaxMargin in (0, 1)");
  if (!(spec.maxGapFraction > 0.0) || !(spec.maxGapFraction < 0.5) ||
      !std::isfinite(spec.maxGapFraction))
    throw std::runtime_error("inductor spec needs maxGapFraction in (0, 0.5)");
  if (!(spec.maxRolloff > 0.0) || !(spec.maxRolloff < 1.0) || !std::isfinite(spec.maxRolloff))
    throw std::runtime_error("inductor spec needs maxRolloff in (0, 1)");
  if (!(spec.lossBudgetW >= 0.0) || !std::isfinite(spec.lossBudgetW))
    throw std::runtime_error("inductor spec needs lossBudgetW finite >= 0");
  reqPosFin(core.ae, "core geometry needs ae > 0");
  reqPosFin(core.le, "core geometry needs le > 0");
  reqPosFin(core.ve, "core geometry needs ve > 0");
  reqPosFin(core.mlt, "core geometry needs mlt > 0");
  reqPosFin(core.windowArea, "core geometry needs windowArea > 0");
  reqPosFin(mat.bh.bs, "core material needs Bs > 0");
  reqPosFin(mat.bh.a, "core material needs shape field a > 0");
  if (!(mat.bh.hc >= 0.0) || !std::isfinite(mat.bh.hc))
    throw std::runtime_error("core material needs Hc finite >= 0");
  reqPosFin(mat.rho, "core material needs rho > 0");
  if (!(mat.lamThickness >= 0.0) || !std::isfinite(mat.lamThickness))
    throw std::runtime_error("core material needs lamThickness finite >= 0");
  reqPosFin(winding.wireAreaM2, "winding spec needs wireAreaM2 > 0");
  reqPosFin(winding.resistivity, "winding spec needs resistivity > 0");
  if (!(winding.maxFill > 0.0) || !(winding.maxFill < 1.0) || !std::isfinite(winding.maxFill))
    throw std::runtime_error("winding spec needs maxFill in (0, 1)");
  if (winding.layers < 1) throw std::runtime_error("winding spec needs layers >= 1");
  if (!std::isfinite(spec.tempC)) throw std::runtime_error("inductor spec needs tempC finite");
  if (!std::isfinite(mat.bsTempCoeff))
    throw std::runtime_error("core material needs bsTempCoeff finite");
  if (!std::isfinite(mat.rhoTempCoeff))
    throw std::runtime_error("core material needs rhoTempCoeff finite");

  // Hot operating point: copper rho(T), saturation bound Bs(T), core rho(T).
  // mu_i (Bs/a slope) deliberately stays at the 20C value — the saturation
  // bound is what temperature threatens. Throws if T pushes Bs/rho out.
  const double rhoCu = windingResistivityAtTemp(winding, spec.tempC);
  const double bsHot = mat.bh.bs * (1.0 + mat.bsTempCoeff * (spec.tempC - 20.0));
  if (!(bsHot > 0.0) || !std::isfinite(bsHot))
    throw std::runtime_error("inductor hot Bs non-positive: temp/coeff out of range");
  const double rhoCoreHot = mat.rho * (1.0 + mat.rhoTempCoeff * (spec.tempC - 20.0));
  if (!(rhoCoreHot > 0.0) || !std::isfinite(rhoCoreHot))
    throw std::runtime_error("inductor hot core rho non-positive: temp/coeff out of range");

  // Turns from the Bsat bound at worst-case current (peak + ripple/2).
  const double iMax = spec.iPeak + 0.5 * spec.iRipplePkPk;
  const double bMax = spec.bMaxMargin * bsHot;
  long long n = static_cast<long long>(
      std::ceil(spec.inductance * iMax / (bMax * core.ae)));
  if (n < 1) n = 1;
  // Core reluctance from the tanh initial slope (mu_i = Bs/a, exact).
  const double rCore = core.le / ((mat.bh.bs / mat.bh.a) * core.ae);
  // Raise N until the core fits inside N^2/L (gap reluctance >= 0).
  for (long long k = 0; k < 1000000; ++k) {
    const double rTot = static_cast<double>(n) * static_cast<double>(n) / spec.inductance;
    if (rTot >= rCore) break;
    ++n;
  }
  const double rTot = static_cast<double>(n) * static_cast<double>(n) / spec.inductance;
  // Gap from L = N^2/(Rcore + Rgap), Rgap = lg/(mu0*Ae*F),
  // F = 1 + lg/sqrt(Ae): closed form lg = K/(1 - K/s), K = dR*mu0*Ae.
  const double dR = rTot - rCore;
  const double s = std::sqrt(core.ae);
  const double kk = dR * kMu0 * core.ae;
  if (!(kk < s)) throw std::runtime_error("inductor spec infeasible: fringing swamps gap");
  const double lg = kk / (1.0 - kk / s);
  if (!(lg >= 0.0) || !std::isfinite(lg))
    throw std::runtime_error("inductor spec infeasible: negative gap");
  if (lg > spec.maxGapFraction * core.le)
    throw std::runtime_error("inductor spec infeasible: gap over limit");
  const double fr = 1.0 + lg / s;
  const double rGap = lg / (kMu0 * core.ae * fr);

  // Verify on a saturable-core + gap series network (the module's own model).
  ReluctanceNetwork net;
  net.addSaturableReluctance("core", 0, 1, core.le, core.ae, bsHot, mat.bh.a);
  net.addReluctance("gap", 1, 0, rGap > 0.0 ? rGap : 1e-12);
  net.addWinding("W", "core", static_cast<double>(n));
  auto lambdaAt = [&](double i) {
    net.setWindingCurrent("W", i);
    net.solve();
    return static_cast<double>(n) * net.windingFlux("W");
  };
  const double l0 = lambdaAt(1e-3) / 1e-3;
  const double lPk = lambdaAt(spec.iPeak) / spec.iPeak;
  const double bDc = net.branchFlux("core") / core.ae;
  const double bAcPk = (0.5 * spec.iRipplePkPk) * l0 / (static_cast<double>(n) * core.ae);
  const double bPeak = bDc + bAcPk;
  if (!(bPeak <= bsHot) || !std::isfinite(bPeak))
    throw std::runtime_error("inductor spec infeasible: Bpeak over Bs");
  const double rolloff = 1.0 - lPk / l0;
  if (!(rolloff <= spec.maxRolloff))
    throw std::runtime_error("inductor spec infeasible: roll-off over limit");

  // Minor-loop hysteresis loss on the ripple triangle (settle one cycle,
  // measure the second: deterministic minor-loop loss density).
  const double rippleRms = 0.5 * spec.iRipplePkPk / std::sqrt(3.0);
  const double iDc = std::sqrt(std::max(spec.iRms * spec.iRms - rippleRms * rippleRms, 0.0));
  const double hDc = static_cast<double>(n) * iDc / core.le;
  const double dH = static_cast<double>(n) * spec.iRipplePkPk / core.le;
  HysteresisCore hyst(mat.bh);
  hyst.update(hDc);
  constexpr int kTriSteps = 40;
  auto triangle = [&]() {
    for (int k = 1; k <= kTriSteps; ++k)
      hyst.update(hDc - 0.5 * dH + dH * static_cast<double>(k) / kTriSteps);
    for (int k = 1; k <= kTriSteps; ++k)
      hyst.update(hDc + 0.5 * dH - dH * static_cast<double>(k) / kTriSteps);
  };
  triangle();  // settle onto the minor loop
  const double before = hyst.loss();
  triangle();
  const double perCycle = hyst.loss() - before;
  const double hystW = perCycle * spec.freqHz * core.ve;
  const double eddyW =
      eddyLossDensity(rhoCoreHot, mat.lamThickness, spec.freqHz, bAcPk) * core.ve;
  // Window fill + DC copper (MLT*N*rho/Aw at Irms); budget covers total.
  const double fill =
      static_cast<double>(n) * winding.wireAreaM2 / core.windowArea;
  if (!(fill <= winding.maxFill))
    throw std::runtime_error("inductor spec infeasible: window fill over limit");
  const double rDc =
      core.mlt * static_cast<double>(n) * rhoCu / winding.wireAreaM2;
  const double copperW = rDc * spec.iRms * spec.iRms;
  // Dowell AC copper on the ripple (round wire -> square equivalent,
  // skin depth from hot winding resistivity, full layers).
  const double dWire = 2.0 * std::sqrt(winding.wireAreaM2 / kPi);
  const double hSq = 0.25 * kPi * dWire;
  const double skin = std::sqrt(rhoCu / (kPi * spec.freqHz * kMu0));
  const double iAc = 0.5 * spec.iRipplePkPk / std::sqrt(3.0);
  const double acCopperW = dowellFactor(winding.layers, hSq / skin) * rDc * iAc * iAc;
  if (spec.lossBudgetW > 0.0 && hystW + eddyW + copperW + acCopperW > spec.lossBudgetW)
    throw std::runtime_error("inductor spec infeasible: total loss over budget");
  return InductorDesign{static_cast<int>(n), lg,        bPeak,  l0,
                        lPk,               rolloff,    hystW,  eddyW,
                        copperW,           acCopperW,  fill,   rDc,
                        spec.tempC};
}

}  // namespace magnetics
}  // namespace power_engine
