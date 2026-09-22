#include "power_engine/solver.h"

#include <cmath>
#include <cstring>
#include <map>
#include <stdexcept>

namespace power_engine {

TransientSolver::TransientSolver(Circuit& circuit, double dt)
    : circuit_(circuit), dt_(dt) {
  if (!(dt_ > 0.0) || !std::isfinite(dt_)) throw std::runtime_error("dt must be positive finite");
  rebuildMaps();
}

void TransientSolver::setStep(double dt) {
  if (!(dt > 0.0) || !std::isfinite(dt)) throw std::runtime_error("dt must be positive finite");
  dt_ = dt;
}

void TransientSolver::setTime(double t) {
  if (!std::isfinite(t) || t < 0.0) throw std::runtime_error("solver time must be finite >= 0");
  t_ = t;
}

SolverState TransientSolver::saveState() const {
  SolverState s;
  s.t = t_;
  s.x = x_;
  s.devices = circuit_.devices();
  return s;
}

void TransientSolver::restoreState(const SolverState& s) {
  if (s.devices.size() != circuit_.devices().size()) {
    throw std::runtime_error("restoreState topology mismatch (device count changed)");
  }
  if (!std::isfinite(s.t) || s.t < 0.0) throw std::runtime_error("restoreState bad time");
  rebuildMaps();  // topology-fixed, but keeps maps consistent by construction
  if (s.x.size() != x_.size()) {
    throw std::runtime_error("restoreState topology mismatch (unknown count changed)");
  }
  t_ = s.t;
  x_ = s.x;
  circuit_.mutableDevices() = s.devices;
}

void TransientSolver::setAdaptive(double tol, double dtMin, double dtMax) {
  if (!(tol > 0.0) || !std::isfinite(tol)) {
    throw std::runtime_error("adaptive tol must be positive finite");
  }
  if (!(dtMin > 0.0) || !std::isfinite(dtMin) || !(dtMax >= dtMin) || !std::isfinite(dtMax)) {
    throw std::runtime_error("adaptive needs 0 < dtMin <= dtMax, finite");
  }
  adaptive_.enabled = true;
  adaptive_.tol = tol;
  adaptive_.dtMin = dtMin;
  adaptive_.dtMax = dtMax;
  if (dt_ < dtMin || dt_ > dtMax) dt_ = std::min(std::max(dt_, dtMin), dtMax);
}

void TransientSolver::rebuildMaps() {
  nodeList_ = circuit_.nodes();
  nodeIndex_.clear();
  hasNonlinear_ = false;
  hasDiodes_ = false;
  for (std::size_t i = 0; i < nodeList_.size(); ++i) nodeIndex_[nodeList_[i]] = static_cast<int>(i);
  extraRow_.clear();
  const auto& devs = circuit_.devices();
  rowA_.assign(devs.size(), -1);
  rowB_.assign(devs.size(), -1);
  rowC_.assign(devs.size(), -1);
  rowD_.assign(devs.size(), -1);
  rowE_.assign(devs.size(), -1);
  std::size_t k = 0;
  for (std::size_t i = 0; i < devs.size(); ++i) {
    const auto& d = devs[i];
    const int r1 = nodeRow(d.n1);
    const int r2 = nodeRow(d.n2);
    if (r1 == -2 || r2 == -2) throw std::runtime_error("internal error: stale node map");
    rowA_[i] = r1;
    rowB_[i] = r2;
    if (d.type == DeviceType::VoltageSource) {
      extraRow_[d.name] = nodeList_.size() + k;
      rowE_[i] = static_cast<int>(nodeList_.size() + k);
      k += 1;
    } else if (d.type == DeviceType::Transformer) {
      const int r3 = nodeRow(d.n3);
      const int r4 = nodeRow(d.n4);
      if (r3 == -2 || r4 == -2) throw std::runtime_error("internal error: stale node map");
      rowC_[i] = r3;
      rowD_[i] = r4;
      extraRow_[d.name] = nodeList_.size() + k;
      rowE_[i] = static_cast<int>(nodeList_.size() + k);
      k += 2;  // Ip row then Is row
    } else if (d.type == DeviceType::CoupledInductor) {
      const int r3 = nodeRow(d.n3);
      const int r4 = nodeRow(d.n4);
      if (r3 == -2 || r4 == -2) throw std::runtime_error("internal error: stale node map");
      rowC_[i] = r3;
      rowD_[i] = r4;
    }
    if (d.type == DeviceType::SatInductor) hasNonlinear_ = true;
    if (d.type == DeviceType::Diode) hasDiodes_ = true;
  }
  const auto n = static_cast<Eigen::Index>(nodeList_.size() + k);
  x_ = Eigen::VectorXd::Zero(n);
  workA_ = Eigen::MatrixXd::Zero(n, n);
  workZ_ = Eigen::VectorXd::Zero(n);
  if (n >= kSparseThreshold) {
    // Topology-fixed sparsity pattern: switch/diode states only change
    // values, so factor the symbolic pattern once here. (No status to
    // check: analyzePattern reports via the later factorize info.)
    assemble(true);
    workAs_ = workA_.sparseView();
    workSlu_[0].analyzePattern(workAs_);
    workSlu_[1].analyzePattern(workAs_);
  }
  cacheValid_[0] = false;  // maps/buffers rebuilt: drop cached factors
  cacheValid_[1] = false;
  resolveWindow_.clear();  // fresh topology: restart stiffness detection
  resolveWinPos_ = 0;
  resolveWinSum_ = 0;
  prevDx_.clear();
  flipRun_.clear();
  runPeak_.clear();
  havePrevAuto_ = false;
  cleanSteps_ = 0;
  lastEmbeddedErr_ = -1.0;
}

void TransientSolver::initialize() {
  rebuildMaps();
  t_ = 0.0;
  x_.setZero();
  stats_ = SolverStats{};
  for (auto& d : circuit_.mutableDevices()) {
    if (d.type == DeviceType::Capacitor) {
      d.v_prev = d.ic;
      d.i_prev = 0.0;
    } else if (d.type == DeviceType::Inductor) {
      d.i_prev = d.ic;
      d.v_prev = 0.0;
    } else if (d.type == DeviceType::Diode) {
      d.conducting = false;
      d.v_prev = 0.0;
      d.i_prev = 0.0;
    } else if (d.type == DeviceType::Transformer) {
      d.v_prev = 0.0;
      d.i_prev = 0.0;
      d.i2_prev = 0.0;
    } else if (d.type == DeviceType::CoupledInductor) {
      d.v_prev = 0.0;
      d.v2_prev = 0.0;
      d.i_prev = d.ic;
      d.i2_prev = d.ic2;
    } else if (d.type == DeviceType::SatInductor) {
      d.v_prev = 0.0;
      d.i_prev = d.ic;
      d.flux = satFlux(d.ic, d.value, d.lsat, d.isat);
      d.newton_ik = d.ic;
    } else {
      d.v_prev = 0.0;
      d.i_prev = 0.0;
    }
    d.recT = 0.0;
    d.recI = 0.0;
    d.recE = 0.0;
    if (d.type == DeviceType::Switch) {
      d.closedPrev = d.closed;
      d.transT = 0.0;
      d.transFrom = 0.0;
      d.transTo = 0.0;
    }
  }
}

// Row-precomputed stamps (-1 = ground, never -2 here: rebuildMaps validates).
static inline void stampG(Eigen::MatrixXd& A, int r1, int r2, double g) {
  if (r1 >= 0) A(r1, r1) += g;
  if (r2 >= 0) A(r2, r2) += g;
  if (r1 >= 0 && r2 >= 0) {
    A(r1, r2) -= g;
    A(r2, r1) -= g;
  }
}

static inline void stampI(Eigen::VectorXd& z, int r1, int r2, double i12) {
  // KCL: current leaving terminal 1, entering terminal 2.
  if (r1 >= 0) z(r1) -= i12;
  if (r2 >= 0) z(r2) += i12;
}

// Effective switch resistance: steady Ron/Roff, or the geometric-ramp
// value while a slew-limited transition is in flight: R(t) sweeps decades
// at a constant ratio per step (linear-in-R would jump 1000x on the first
// step of a turn-off and re-excite the very ringing the ramp removes).
// Single definition shared by assemble() and updateHistories().
static inline double switchResistance(const Device& d) {
  if (d.tsw > 0.0 && d.transT > 0.0 && d.transFrom != d.transTo && d.transFrom > 0.0 &&
      d.transTo > 0.0) {
    const double frac = 1.0 - d.transT / d.tsw;  // 0 at toggle -> 1 at end
    return d.transFrom * std::pow(d.transTo / d.transFrom, frac);
  }
  if (d.tsw > 0.0 && d.transT > 0.0) return d.transTo;  // degenerate: snap
  return d.closed ? d.ron : d.roff;
}

// Recovery/tail branch current in reference direction (n1->n2) at the
// current recovery timer value. Diode: triangular Irr*(recT/trr) with
// recI = -Irr. Switch: exponential tail recI*exp(-elapsed/ttail).
static inline double recoveryCurrent(const Device& d) {
  if (d.type == DeviceType::Diode) {
    return d.trr > 0.0 ? d.recI * (d.recT / d.trr) : 0.0;
  }
  if (d.ttail <= 0.0) return 0.0;
  const double elapsed = 5.0 * d.ttail - d.recT;
  return d.recI * std::exp(-elapsed / d.ttail);
}

void TransientSolver::assemble(bool withMatrix) const {
  Eigen::MatrixXd& A = workA_;
  Eigen::VectorXd& z = workZ_;
  if (withMatrix) A.setZero();
  z.setZero();
  // Slew-transition start (idempotent within a step): a gate toggle with
  // tsw>0 begins the ramp HERE so the toggle step itself already solves
  // with frac = 0 (old steady R) instead of a full ideal commutation.
  // Starting at commit would let all the violence happen first and defeat
  // the ramp. Skipped when already started (diode-iteration re-assembles).
  for (auto& d : circuit_.mutableDevices()) {
    if (d.type != DeviceType::Switch || d.tsw <= 0.0) continue;
    const bool rose = !d.closedPrev && d.closed;
    const bool fell = d.closedPrev && !d.closed;
    if (!(rose || fell)) continue;
    const double target = d.closed ? d.ron : d.roff;
    if (d.transT > 0.0 && d.transTo == target) continue;
    d.transFrom = (d.transT > 0.0) ? switchResistance(d) : (d.closedPrev ? d.ron : d.roff);
    d.transTo = target;
    d.transT = d.tsw;
  }

  // KCL coupling for a branch current flowing r1->r2 through extra row r.
  auto stampBranch = [&](int r1, int r2, Eigen::Index r) {
    if (r1 >= 0) {
      A(r1, r) += 1.0;
      A(r, r1) += 1.0;
    }
    if (r2 >= 0) {
      A(r2, r) -= 1.0;
      A(r, r2) -= 1.0;
    }
  };

  const auto& devs = circuit_.devices();
  for (std::size_t i = 0; i < devs.size(); ++i) {
    const auto& d = devs[i];
    const int r1 = rowA_[i];
    const int r2 = rowB_[i];
    switch (d.type) {
      case DeviceType::Resistor: {
        if (withMatrix) stampG(A, r1, r2, 1.0 / d.value);
        break;
      }
      case DeviceType::Capacitor: {
        // BDF2: i = (3C/dt)*v - (4C/dt)*vMid + (C/dt)*vPrev.
        const double g = bdf2Stage_ ? 3.0 * d.value / dt_ : 2.0 * d.value / dt_;
        const double iHist = bdf2Stage_ ? -g * ((4.0 / 3.0) * d.v_mid - d.v_prev / 3.0)
                                        : -g * d.v_prev - d.i_prev;
        if (withMatrix) stampG(A, r1, r2, g);
        stampI(z, r1, r2, iHist);
        break;
      }
      case DeviceType::Inductor: {
        // BDF2: i = (4/3)*iMid - (1/3)*iPrev + (dt/3L)*v (voltages enter
        // only through the new step — BDF2 differentiates the state).
        const double g = bdf2Stage_ ? dt_ / (3.0 * d.value) : dt_ / (2.0 * d.value);
        const double iHist = bdf2Stage_ ? (4.0 / 3.0) * d.i_mid - d.i_prev / 3.0
                                        : d.i_prev + g * d.v_prev;
        if (withMatrix) stampG(A, r1, r2, g);
        stampI(z, r1, r2, iHist);
        break;
      }
      case DeviceType::CurrentSource: {
        stampI(z, r1, r2, d.value);
        break;
      }
      case DeviceType::VoltageSource: {
        const auto row = static_cast<Eigen::Index>(rowE_[i]);
        if (withMatrix) stampBranch(r1, r2, row);
        z(row) = d.value;
        break;
      }
      case DeviceType::Switch: {
        // Ideal switch as Ron/Roff, slew ramp while transitioning
        // (+ parallel tail source while recovering).
        if (withMatrix) stampG(A, r1, r2, 1.0 / switchResistance(d));
        if (d.recT > 0.0 && !d.closed) stampI(z, r1, r2, recoveryCurrent(d));
        break;
      }
      case DeviceType::Diode: {
        if (d.recT > 0.0) {
          // Reverse recovery: impressed triangular current (branch current
          // is circuit-driven at low impedance; the source imposes it and
          // the node voltage floats to suit). No shunt: it would swallow
          // the imposed current through Ron under reverse bias.
          stampI(z, r1, r2, recoveryCurrent(d));
        } else if (d.conducting) {
          // Norton: G=1/Ron || current source G*Vf (cathode->anode).
          const double g = 1.0 / d.ron;
          if (withMatrix) stampG(A, r1, r2, g);
          stampI(z, r1, r2, -g * d.vf);
        } else {
          if (withMatrix) stampG(A, r1, r2, 1.0 / d.roff);
        }
        break;
      }
      case DeviceType::Transformer: {
        // Ideal transformer, ratio n:1 (Vp/Vs = n), unknowns Ip (n1->n2)
        // and Is (n3->n4). KCL coupling like two zero-volt sources, plus:
        //   row Ip: Vp - n*Vs = 0   (voltage constraint)
        //   row Is: n*Ip + Is = 0   (power conservation)
        const auto base = static_cast<Eigen::Index>(rowE_[i]);
        const auto rIp = base;
        const auto rIs = base + 1;
        const double n = d.ratio;
        const int r3 = rowC_[i];
        const int r4 = rowD_[i];
        if (withMatrix) stampBranch(r1, r2, rIp);
        if (withMatrix) stampBranch(r3, r4, rIs);
        // Row rIp currently reads V(n1)-V(n2)=0; extend with -n*Vs.
        if (withMatrix) {
          if (r3 >= 0) A(rIp, r3) -= n;
          if (r4 >= 0) A(rIp, r4) += n;
        }
        // Row rIs currently reads V(n3)-V(n4)=0; replace the ROW entries
        // (keep the COLUMN KCL coupling of Is) with n*Ip+Is=0.
        if (withMatrix) {
          if (r3 >= 0) A(rIs, r3) = 0.0;
          if (r4 >= 0) A(rIs, r4) = 0.0;
          A(rIs, rIp) += n;
          A(rIs, rIs) += 1.0;
        }
        z(rIp) = 0.0;
        z(rIs) = 0.0;
        break;
      }
      case DeviceType::CoupledInductor: {
        // Trapezoidal 2-port Norton from flux linkage L*i with
        // L = [[L1,M],[M,L2]]: i_{n+1} = G*v_{n+1} + hist,
        // G = (dt/2)*L^-1, hist = i_n + G*v_n. BDF2: G = (dt/3)*L^-1,
        // hist = (4/3)*iMid - (1/3)*i_n. Dots at winding starts.
        const double det = d.l1 * d.l2 - d.m * d.m;  // > 0 since k < 1
        const double f = (bdf2Stage_ ? 2.0 / 3.0 : 0.5) * dt_ / det;
        const double g11 = f * d.l2;
        const double g12 = -f * d.m;
        const double g22 = f * d.l1;
        const int r3 = rowC_[i];
        const int r4 = rowD_[i];
        const double v1p = d.v_prev;
        const double v2p = d.v2_prev;
        const double h1 = bdf2Stage_ ? (4.0 / 3.0) * d.i_mid - d.i_prev / 3.0
                                     : d.i_prev + g11 * v1p + g12 * v2p;
        const double h2 = bdf2Stage_ ? (4.0 / 3.0) * d.i2_mid - d.i2_prev / 3.0
                                     : d.i2_prev + g12 * v1p + g22 * v2p;
        // Conductance block.
        const int rows[4] = {r1, r2, r3, r4};
        const double gmat[4][4] = {{g11, -g11, g12, -g12},
                                   {-g11, g11, -g12, g12},
                                   {g12, -g12, g22, -g22},
                                   {-g12, g12, -g22, g22}};
        if (withMatrix) {
          for (int a = 0; a < 4; ++a) {
            if (rows[a] < 0) continue;
            for (int b = 0; b < 4; ++b) {
              if (rows[b] < 0) continue;
              A(rows[a], rows[b]) += gmat[a][b];
            }
          }
        }
        stampI(z, r1, r2, h1);
        stampI(z, r3, r4, h2);
        break;
      }
      case DeviceType::SatInductor: {
        // Newton-linearized companion at ik: G = dt/(2*Ld(ik)),
        // Ieq = ik - (λ(ik)-flux_n)/Ld + G*v_n. BDF2 stage: G = dt/(3Ld),
        // Ieq = ik + ((4/3)λMid - (1/3)flux_n - λ(ik))/Ld with the frozen
        // midpoint flux λMid = λ(ik_mid); voltages enter through G*v only.
        const double ld = satSlope(d.newton_ik, d.value, d.lsat, d.isat);
        const double g = bdf2Stage_ ? dt_ / (3.0 * ld) : dt_ / (2.0 * ld);
        const double lam = satFlux(d.newton_ik, d.value, d.lsat, d.isat);
        double ieq;
        if (bdf2Stage_) {
          const double lamMid = satFlux(d.ik_mid, d.value, d.lsat, d.isat);
          ieq = d.newton_ik + ((4.0 / 3.0) * lamMid - d.flux / 3.0 - lam) / ld;
        } else {
          ieq = d.newton_ik - (lam - d.flux) / ld + g * d.v_prev;
        }
        if (withMatrix) stampG(A, r1, r2, g);
        stampI(z, r1, r2, ieq);
        break;
      }
    }
  }
}

void TransientSolver::factorize() const {
  const int slot = bdf2Stage_ ? 1 : 0;
  if (workA_.rows() >= kSparseThreshold) {
    workAs_ = workA_.sparseView();
    workSlu_[slot].factorize(workAs_);
    if (workSlu_[slot].info() != Eigen::Success) {
      throw SingularError(
          "singular MNA matrix (check topology: floating node or V-source loop?)");
    }
    return;
  }
  workLu_[slot].compute(workA_);
  // Singularity guard: MNA structural singularities (floating nodes,
  // V-source loops) make a U pivot exactly (or relatively) zero.
  // Threshold mirrors FullPivLU::isInvertible semantics: relative to the
  // largest pivot, far below any legitimate stiffness (Ron/Roff ~ 2e8,
  // companion conductances similar scale).
  const double dmax = workLu_[slot].matrixLU().diagonal().cwiseAbs().maxCoeff();
  const double dmin = workLu_[slot].matrixLU().diagonal().cwiseAbs().minCoeff();
  constexpr double kPivotTol = 64.0 * Eigen::NumTraits<double>::epsilon();
  if (!(dmax > 0.0) || dmin <= dmax * kPivotTol) {
    throw SingularError(
        "singular MNA matrix (check topology: floating node or V-source loop?)");
  }
}

Eigen::VectorXd TransientSolver::solveFactors() const {
  const int slot = bdf2Stage_ ? 1 : 0;
  if (workA_.rows() >= kSparseThreshold) {
    ++stats_.sparseSolves;
    return workSlu_[slot].solve(workZ_);
  }
  return workLu_[slot].solve(workZ_);
}

Eigen::VectorXd TransientSolver::solveLinear() const {
  factorize();
  return solveFactors();
}

namespace {
// FNV-1a helpers for the topology signature (within-run use only:
// determinism on this machine, no cross-platform stability needed).
inline void hashWord(std::uint64_t& h, std::uint64_t w) {
  h ^= w;
  h *= 1099511628211ULL;
}
inline std::uint64_t dblBits(double v) {
  std::uint64_t w = 0;
  std::memcpy(&w, &v, sizeof(w));
  return w;
}
// Order-2 step controller factors shared by step-doubling and embedded
// TR-BDF2 control (exponent 1/3, safety 0.9, clamped).
inline double growFactor(double tol, double err) {
  return std::min(2.0, std::max(0.3, 0.9 * std::cbrt(tol / err)));
}
inline double shrinkFactor(double tol, double err) {
  return std::max(0.2, 0.9 * std::cbrt(tol / err));
}
}  // namespace

TransientSolver::MatrixSig TransientSolver::matrixSig() const {
  std::uint64_t h = 1469598103934665603ULL;
  hashWord(h, dblBits(dt_));
  hashWord(h, bdf2Stage_ ? 1ULL : 0ULL);  // stage 2 shares dt_ but not A
  const auto& devs = circuit_.devices();
  hashWord(h, static_cast<std::uint64_t>(devs.size()));
  for (const auto& d : devs) {
    switch (d.type) {
      case DeviceType::Resistor:
      case DeviceType::Capacitor:
      case DeviceType::Inductor:
        hashWord(h, dblBits(d.value));
        break;
      case DeviceType::CoupledInductor:
        hashWord(h, dblBits(d.l1));
        hashWord(h, dblBits(d.l2));
        hashWord(h, dblBits(d.m));
        break;
      case DeviceType::Switch:
        hashWord(h, d.closed ? 1ULL : 0ULL);
        hashWord(h, dblBits(d.ron));
        hashWord(h, dblBits(d.roff));
        // In-flight ramp: resistance varies every step (transT counts
        // down; endpoints fixed per transition — all hashed).
        hashWord(h, dblBits(d.transT));
        hashWord(h, dblBits(d.transFrom));
        hashWord(h, dblBits(d.transTo));
        break;
      case DeviceType::Diode:
        // Recovery (recT>0) replaces the Norton shunt with an impressed
        // source: matrix-affecting. recT's exact value is z-only.
        hashWord(h, d.conducting ? 1ULL : 0ULL);
        hashWord(h, d.recT > 0.0 ? 1ULL : 0ULL);
        hashWord(h, dblBits(d.ron));
        hashWord(h, dblBits(d.roff));
        break;
      case DeviceType::Transformer:
        hashWord(h, dblBits(d.ratio));
        break;
      case DeviceType::SatInductor:
        // Defensive: saturable circuits bypass the cache (Newton), but a
        // changing operating point must never alias a cached signature.
        hashWord(h, dblBits(d.value));
        hashWord(h, dblBits(d.newton_ik));
        break;
      case DeviceType::VoltageSource:
      case DeviceType::CurrentSource:
        break;  // values stamp z only; the +/-1 branch pattern is static
    }
  }
  return MatrixSig{h};
}

void TransientSolver::assembleCached() const {
  const MatrixSig s = matrixSig();
  const int slot = bdf2Stage_ ? 1 : 0;
  if (cacheValid_[slot] && s == cachedSig_[slot]) {
    assemble(false);
    ++stats_.factorSkips;
    return;
  }
  assemble(true);
  factorize();
  cachedSig_[slot] = s;
  cacheValid_[slot] = true;
}

bool TransientSolver::updateDiodeStates(const Eigen::VectorXd& x) {
  if (!hasDiodes_) return false;  // diode-free circuits skip the scan
  auto vRow = [&](int r) -> double { return r < 0 ? 0.0 : x(static_cast<Eigen::Index>(r)); };
  bool changed = false;
  auto& devs = circuit_.mutableDevices();
  for (std::size_t i = 0; i < devs.size(); ++i) {
    auto& d = devs[i];
    if (d.type != DeviceType::Diode) continue;
    if (d.recT > 0.0) continue;  // recovery in progress; timer runs in histories
    const double vd = vRow(rowA_[i]) - vRow(rowB_[i]);
    if (d.conducting) {
      // I(anode->cathode) = (Vd - Vf)/Ron; turn off when negative.
      const double id = (vd - d.vf) / d.ron;
      if (id < -kDiodeIhys) {
        if (d.qrr > 0.0 && d.i_prev > 1e-12) {
          // Forward turn-off: triangular recovery (Irr = 2*Qrr/trr).
          d.recI = -2.0 * d.qrr / d.trr;
          d.recT = d.trr;
          changed = true;  // re-solve with the recovery stamp
          ++stats_.diodeEvents;
        } else {
          d.conducting = false;
          changed = true;
          ++stats_.diodeEvents;
        }
      }
    } else {
      if (vd > d.vf + kDiodeVhys) {
        d.conducting = true;
        changed = true;
        ++stats_.diodeEvents;
      }
    }
  }
  return changed;
}

void TransientSolver::updateHistories(const Eigen::VectorXd& x, HistHow how) {
  const bool mid = (how == HistHow::TrapMid);
  const bool bdf2 = (how == HistHow::Bdf2Prev);
  auto vRow = [&](int r) -> double { return r < 0 ? 0.0 : x(static_cast<Eigen::Index>(r)); };
  auto& devs = circuit_.mutableDevices();
  for (std::size_t i = 0; i < devs.size(); ++i) {
    auto& d = devs[i];
    const double vNew = vRow(rowA_[i]) - vRow(rowB_[i]);
    double& vT = mid ? d.v_mid : d.v_prev;
    double& iT = mid ? d.i_mid : d.i_prev;
    switch (d.type) {
      case DeviceType::Capacitor: {
        double iNew;
        if (bdf2) {
          const double G = 3.0 * d.value / dt_;
          iNew = G * vNew - (4.0 / 3.0) * G * d.v_mid + G * d.v_prev / 3.0;
        } else {
          const double g = 2.0 * d.value / dt_;
          iNew = g * vNew - g * d.v_prev - d.i_prev;
        }
        vT = vNew;
        iT = iNew;
        break;
      }
      case DeviceType::Inductor: {
        double iNew;
        if (bdf2) {
          const double G = dt_ / (3.0 * d.value);
          iNew = (4.0 / 3.0) * d.i_mid - d.i_prev / 3.0 + G * vNew;
        } else {
          const double g = dt_ / (2.0 * d.value);
          iNew = d.i_prev + g * vNew + g * d.v_prev;
        }
        vT = vNew;
        iT = iNew;
        break;
      }
      case DeviceType::Resistor:
      case DeviceType::CurrentSource: {
        vT = vNew;
        iT = (d.type == DeviceType::Resistor) ? vNew / d.value : d.value;
        break;
      }
      case DeviceType::Switch: {
        // Branch current from the pre-update resistance/recovery state
        // (matches the historical evaluation order bit-for-bit when idle).
        const double rNow = switchResistance(d);
        const double iNew =
            vNew / rNow + ((d.recT > 0.0 && !d.closed) ? recoveryCurrent(d) : 0.0);
        if (!mid) {
          const double iBefore = d.i_prev;
          const bool fell = d.closedPrev && !d.closed;
          // Toggle-start lives in assemble() (the toggle step must already
          // solve pre-toggle); here only countdown, completion, and tail.
          if (d.transT > 0.0) {
            // In transition (either gate state): countdown only. A
            // turn-off tail (if any) starts at completion (post voltage
            // rise), not at the edge.
            d.transT -= dt_;
            if (d.transT <= 0.0) {
              d.transT = 0.0;
              if (!d.closed && d.ttail > 0.0 && d.tailk > 0.0 && iBefore != 0.0) {
                d.recT = 5.0 * d.ttail;
                d.recI = d.tailk * iBefore;
              }
            }
          }
          if (d.closed) {
            d.recT = 0.0;  // re-closing cancels any tail
            d.recI = 0.0;
          } else if (d.recT > 0.0) {
            d.recT -= dt_;
            if (d.recT <= 0.0) {
              d.recT = 0.0;
              d.recI = 0.0;
            }
          } else if (fell && d.tsw == 0.0 && d.ttail > 0.0 && d.tailk > 0.0 &&
                     iBefore != 0.0) {
            d.recT = 5.0 * d.ttail;
            d.recI = d.tailk * iBefore;
          }
          d.closedPrev = d.closed;
        }
        vT = vNew;
        iT = iNew;
        break;
      }
      case DeviceType::Diode: {
        if (!mid && d.recT > 0.0) {
          d.v_prev = vNew;
          d.i_prev = recoveryCurrent(d);  // impressed (see assemble)
          d.recT -= dt_;
          if (d.recT <= 0.0) {
            // Release: snap off, booking Qrr*|V| for Engine loss accounting.
            d.recT = 0.0;
            d.recI = 0.0;
            d.conducting = false;
            d.recE += d.qrr * std::abs(vNew);
          }
        } else if (!mid || d.recT <= 0.0) {
          // Mid-capture during recovery: freeze the impressed state into
          // the midpoint (timers untouched); step state handled above.
          vT = vNew;
          iT = (d.recT > 0.0) ? recoveryCurrent(d)
                              : (d.conducting ? (vNew - d.vf) / d.ron : vNew / d.roff);
        }
        break;
      }
      case DeviceType::VoltageSource:
      case DeviceType::Transformer:
        break;  // branch currents handled below from extra unknowns
      case DeviceType::CoupledInductor: {
        const double det = d.l1 * d.l2 - d.m * d.m;
        const double v2New = vRow(rowC_[i]) - vRow(rowD_[i]);
        double i1New, i2New;
        if (bdf2) {
          const double f = dt_ / (3.0 * det);
          const double g11 = f * d.l2;
          const double g12 = -f * d.m;
          const double g22 = f * d.l1;
          i1New = (4.0 / 3.0) * d.i_mid - d.i_prev / 3.0 + g11 * vNew + g12 * v2New;
          i2New = (4.0 / 3.0) * d.i2_mid - d.i2_prev / 3.0 + g12 * vNew + g22 * v2New;
        } else {
          const double f = 0.5 * dt_ / det;
          const double g11 = f * d.l2;
          const double g12 = -f * d.m;
          const double g22 = f * d.l1;
          i1New = g11 * vNew + g12 * v2New + d.i_prev + g11 * d.v_prev + g12 * d.v2_prev;
          i2New = g12 * vNew + g22 * v2New + d.i2_prev + g12 * d.v_prev + g22 * d.v2_prev;
        }
        vT = vNew;
        iT = i1New;
        double& v2T = mid ? d.v2_mid : d.v2_prev;
        double& i2T = mid ? d.i2_mid : d.i2_prev;
        v2T = v2New;
        i2T = i2New;
        break;
      }
      case DeviceType::SatInductor: {
        // At Newton convergence newton_ik is the branch current; refresh
        // flux linkage from it for the next step.
        const double ld = satSlope(d.newton_ik, d.value, d.lsat, d.isat);
        double iNew;
        if (bdf2) {
          const double g = dt_ / (3.0 * ld);
          const double lam = satFlux(d.newton_ik, d.value, d.lsat, d.isat);
          const double lamMid = satFlux(d.ik_mid, d.value, d.lsat, d.isat);
          const double ieq =
              d.newton_ik + ((4.0 / 3.0) * lamMid - d.flux / 3.0 - lam) / ld;
          iNew = g * vNew + ieq;
        } else {
          const double g = dt_ / (2.0 * ld);
          const double lam = satFlux(d.newton_ik, d.value, d.lsat, d.isat);
          const double ieq = d.newton_ik - (lam - d.flux) / ld + g * d.v_prev;
          iNew = g * vNew + ieq;
        }
        vT = vNew;
        iT = iNew;
        if (mid) {
          d.ik_mid = d.newton_ik;  // freeze stage-1 operating point
        } else {
          d.flux = satFlux(d.i_prev, d.value, d.lsat, d.isat);
          d.newton_ik = d.i_prev;
        }
        break;
      }
    }
  }
  if (mid) return;  // branch-current unknowns have no midpoint state
  for (std::size_t i = 0; i < devs.size(); ++i) {
    auto& d = devs[i];
    if (d.type == DeviceType::VoltageSource) {
      const auto row = static_cast<Eigen::Index>(rowE_[i]);
      d.i_prev = x(row);
      d.v_prev = d.value;
    } else if (d.type == DeviceType::Transformer) {
      const auto base = static_cast<Eigen::Index>(rowE_[i]);
      d.i_prev = x(base);        // Ip, primary n1->n2
      d.i2_prev = x(base + 1);   // Is, secondary n3->n4
      d.v_prev = vRow(rowA_[i]) - vRow(rowB_[i]);
    }
  }
}

struct TransientSolver::Snapshot {
  double t = 0.0;
  Eigen::VectorXd x;
  SolverStats stats;
  struct Dev {
    double v = 0.0, i = 0.0, v2 = 0.0, i2 = 0.0;
    bool cond = false;
    double flux = 0.0, ik = 0.0;
    double recT = 0.0, recI = 0.0, recE = 0.0;
    bool closedPrev = false;
    double transT = 0.0, transFrom = 0.0, transTo = 0.0;
  };
  std::vector<Dev> devs;
};

TransientSolver::Snapshot TransientSolver::snapshot() const {
  Snapshot s;
  s.t = t_;
  s.x = x_;
  s.stats = stats_;
  const auto& devs = circuit_.devices();
  s.devs.reserve(devs.size());
  for (const auto& d : devs) {
    s.devs.push_back({d.v_prev, d.i_prev, d.v2_prev, d.i2_prev, d.conducting, d.flux,
                      d.newton_ik, d.recT, d.recI, d.recE, d.closedPrev, d.transT,
                      d.transFrom, d.transTo});
  }
  return s;
}

void TransientSolver::restore(const Snapshot& s) {
  t_ = s.t;
  x_ = s.x;
  stats_ = s.stats;
  auto& devs = circuit_.mutableDevices();
  for (std::size_t i = 0; i < devs.size() && i < s.devs.size(); ++i) {
    devs[i].v_prev = s.devs[i].v;
    devs[i].i_prev = s.devs[i].i;
    devs[i].v2_prev = s.devs[i].v2;
    devs[i].i2_prev = s.devs[i].i2;
    devs[i].conducting = s.devs[i].cond;
    devs[i].flux = s.devs[i].flux;
    devs[i].newton_ik = s.devs[i].ik;
    devs[i].recT = s.devs[i].recT;
    devs[i].recI = s.devs[i].recI;
    devs[i].recE = s.devs[i].recE;
    devs[i].closedPrev = s.devs[i].closedPrev;
    devs[i].transT = s.devs[i].transT;
    devs[i].transFrom = s.devs[i].transFrom;
    devs[i].transTo = s.devs[i].transTo;
  }
}

bool TransientSolver::newtonUpdate(const Eigen::VectorXd& x) {
  auto vRow = [&](int r) -> double { return r < 0 ? 0.0 : x(static_cast<Eigen::Index>(r)); };
  auto& devs = circuit_.mutableDevices();
  bool converged = true;
  for (std::size_t i = 0; i < devs.size(); ++i) {
    auto& d = devs[i];
    if (d.type != DeviceType::SatInductor) continue;
    const double v = vRow(rowA_[i]) - vRow(rowB_[i]);
    const double ld = satSlope(d.newton_ik, d.value, d.lsat, d.isat);
    double inew;
    if (bdf2Stage_) {
      const double g = dt_ / (3.0 * ld);
      const double lam = satFlux(d.newton_ik, d.value, d.lsat, d.isat);
      const double lamMid = satFlux(d.ik_mid, d.value, d.lsat, d.isat);
      const double ieq =
          d.newton_ik + ((4.0 / 3.0) * lamMid - d.flux / 3.0 - lam) / ld;
      inew = g * v + ieq;
    } else {
      const double g = dt_ / (2.0 * ld);
      const double lam = satFlux(d.newton_ik, d.value, d.lsat, d.isat);
      const double ieq = d.newton_ik - (lam - d.flux) / ld + g * d.v_prev;
      inew = g * v + ieq;
    }
    if (std::abs(inew - d.newton_ik) > kNewtonAbsTol + kNewtonRelTol * std::abs(inew)) {
      converged = false;
    }
    d.newton_ik = inew;
  }
  return converged;
}

/// One converged solve at the current dt_/stage: Newton loop for saturable
/// circuits (full assemble, cache bypassed — the operating point moves),
/// cached path otherwise; diode iteration inside in both cases.
void TransientSolver::convergeStep() {
  if (hasNonlinear_) {
    for (auto& d : circuit_.mutableDevices())
      if (d.type == DeviceType::SatInductor) d.newton_ik = d.i_prev;
    for (int k = 0; k < kMaxNewtonIters; ++k) {
      assemble(true);
      x_ = solveLinear();
      for (int iter = 0; iter < kMaxDiodeIters; ++iter) {
        if (!updateDiodeStates(x_)) break;
        assemble(true);
        x_ = solveLinear();
        ++stats_.resolves;
      }
      ++stats_.newtonIters;
      if (newtonUpdate(x_)) break;
      if (k == kMaxNewtonIters - 1) {
        throw std::runtime_error("Newton loop did not converge (saturable inductor)");
      }
    }
  } else {
    assembleCached();
    x_ = solveFactors();
    for (int iter = 0; iter < kMaxDiodeIters; ++iter) {
      if (!updateDiodeStates(x_)) break;
      assembleCached();
      x_ = solveFactors();
      ++stats_.resolves;
    }
  }
}

void TransientSolver::trBdf2Step(double dtNew) {
  // Stage 1: trapezoidal half step; histories captured to midpoint state
  // (timers/flux/edge state untouched — the step hasn't committed).
  xStart_ = x_;
  dt_ = dtNew * 0.5;
  bdf2Stage_ = false;
  convergeStep();
  updateHistories(x_, HistHow::TrapMid);
  xMid_ = x_;
  // Stage 2: BDF2 full step from frozen midpoint + step-start state.
  dt_ = dtNew;
  bdf2Stage_ = true;
  convergeStep();
  bdf2Stage_ = false;
  t_ += dt_;
  updateHistories(x_, HistHow::Bdf2Prev);
  ++stats_.steps;
}

void TransientSolver::embeddedStep() {
  // Adaptive TR-BDF2 control with the embedded stage-difference estimate:
  // linear extrapolation from (start, mid) is first-order at t+dt while
  // the BDF2 solution is second-order, so their difference is O(dt^2) and
  // tracks the local error up to a constant the safety factor absorbs.
  // One converged step per attempt (vs three for step-doubling).
  int condFails = 0;  // consecutive ill-conditioned attempts (grow to escape)
  for (int attempt = 0; attempt < 50; ++attempt) {
    const double dtTry = dt_;
    const Snapshot s0 = snapshot();
    const SolverStats base = stats_;
    try {
      fixedStep(dtTry);  // trBdf2Step: sets xStart_/xMid_
    } catch (const SingularError&) {
      // Shrinking worsens companion spread (C~1/dt vs L~dt); growing
      // escapes conditioning holes. Truly singular topologies fail at
      // every dt and rethrow below after a bounded climb.
      restore(s0);
      if (++condFails > 5) throw;
      dt_ = std::min(adaptive_.dtMax, dtTry * 2.0);
      continue;
    }
    condFails = 0;
    const bool diodesFired = stats_.diodeEvents != base.diodeEvents;
    double err = 0.0;
    for (Eigen::Index i = 0; i < x_.size(); ++i) {
      const double denom = 1e-6 + std::abs(x_[i]);
      const double e = std::abs(x_[i] - (2.0 * xMid_[i] - xStart_[i])) / denom;
      if (e > err) err = e;
    }
    const bool atFloor = dtTry <= adaptive_.dtMin * (1.0 + 1e-9);
    if (diodesFired) {
      // Commutation inside the step: the error estimate is invalid across
      // discontinuities (measured: err pins at ~1 from dtMax to dtMin —
      // no dt resolves an event straddle, so shrink-spiralling only ends
      // at dtMin's ill-conditioned companions and a guard throw). Accept
      // at current dt unchanged (no shrink ratchet, no growth); accuracy
      // resumes when quiet. Same principle as SPICE LTE bypass on events.
      autoUpdate(static_cast<int>(stats_.newtonIters - base.newtonIters),
                 static_cast<int>(stats_.resolves - base.resolves));
      lastEmbeddedErr_ = err;  // may exceed tol: documented exception
      return;
    }
    if (err <= adaptive_.tol || atFloor) {
      autoUpdate(static_cast<int>(stats_.newtonIters - base.newtonIters),
                 static_cast<int>(stats_.resolves - base.resolves));
      lastEmbeddedErr_ = err;
      // Grow unless diodes fired (non-smooth) or the step was floor-forced
      // by excess error (atFloor with err > tol: hold, don't grow).
      if (!diodesFired && err > 0.0 && (err <= adaptive_.tol || !atFloor)) {
        dt_ = std::min(adaptive_.dtMax,
                       std::max(adaptive_.dtMin, dtTry * growFactor(adaptive_.tol, err)));
      }
      return;
    }
    restore(s0);  // reject: full revert, shrink and retry
    dt_ = std::max(adaptive_.dtMin, dtTry * shrinkFactor(adaptive_.tol, err));
  }
  // Extremely stiff spot: force-advance at dtMin rather than stall. If
  // dtMin itself is ill-conditioned (gray-zone companion spread), escalate
  // upward instead — then rethrow honestly if nothing solves.
  for (double dtForce = adaptive_.dtMin;;) {
    const long long r0 = stats_.resolves, n0 = stats_.newtonIters;
    try {
      fixedStep(dtForce);
    } catch (const SingularError&) {
      if (!(dtForce < adaptive_.dtMax)) throw;
      dtForce = std::min(adaptive_.dtMax, dtForce * 10.0);
      continue;
    }
    dt_ = dtForce;
    autoUpdate(static_cast<int>(stats_.newtonIters - n0),
               static_cast<int>(stats_.resolves - r0));
    lastEmbeddedErr_ = adaptive_.tol;
    break;
  }
}

void TransientSolver::fixedStep(double dtNew) {
  dt_ = dtNew;
  bdf2Stage_ = false;
  if (integ_ == Integrator::TrBdf2) {
    trBdf2Step(dtNew);
    return;
  }
  convergeStep();
  t_ += dt_;
  updateHistories(x_, HistHow::TrapPrev);
  ++stats_.steps;
}

void TransientSolver::autoUpdate(int newtonThisStep, int resolvesThisStep) {
  if (!auto_.enabled) return;
  if (static_cast<int>(resolveWindow_.size()) != auto_.window || auto_.window <= 0) {
    resolveWindow_.assign(std::max(1, auto_.window), 0);
    resolveWinPos_ = 0;
    resolveWinSum_ = 0;
  }
  resolveWinSum_ -= resolveWindow_[resolveWinPos_];
  resolveWindow_[resolveWinPos_] = resolvesThisStep;
  resolveWinSum_ += resolvesThisStep;
  resolveWinPos_ = (resolveWinPos_ + 1) % static_cast<int>(resolveWindow_.size());
  // Sustained Nyquist alternation: the same unknown flipping delta sign
  // every step. Physical ripple/commutation flips occasionally (safe);
  // only full-rate alternation reaches flipNeed.
  bool flipped = false;
  if (!havePrevAuto_ || xPrevAuto_.size() != x_.size()) {
    xPrevAuto_ = x_;
    prevDx_.assign(x_.size(), 0.0);
    flipRun_.assign(x_.size(), 0);
    runPeak_.assign(x_.size(), 0.0);
    havePrevAuto_ = true;
  } else {
    for (Eigen::Index k = 0; k < x_.size(); ++k) {
      const double dx = x_[k] - xPrevAuto_[k];
      const double ax = std::abs(dx);
      // Amplitude gate: only alternation comparable to the signal counts.
      // Sub-percent commutation bursts are harmless and must not trigger.
      const double gate = auto_.flipRel * std::max(1.0, std::abs(x_[k]));
      // Decay gate: a numerical limit-cycle holds amplitude; transient
      // bursts decay (measured: 1.45 -> 0.66 -> 0.13 across 3 steps).
      // The run continues only while |dx| stays within half of its peak.
      if (ax > gate && std::abs(prevDx_[k]) > gate && dx * prevDx_[k] < 0.0 &&
          (flipRun_[k] == 0 || ax >= 0.5 * runPeak_[k])) {
        ++flipRun_[k];
        if (ax > runPeak_[k]) runPeak_[k] = ax;
        if (flipRun_[k] >= auto_.flipNeed) flipped = true;
      } else {
        flipRun_[k] = 0;
        runPeak_[k] = ax;
      }
      prevDx_[k] = dx;
    }
    xPrevAuto_ = x_;
  }
  const bool stiff = resolveWinSum_ >= auto_.trigResolves ||
                     newtonThisStep >= auto_.trigNewton || flipped;

  if (stiff) {
    cleanSteps_ = 0;
    if (integ_ != Integrator::TrBdf2) {
      integ_ = Integrator::TrBdf2;
      ++stats_.integratorSwitches;
    }
  } else if (integ_ == Integrator::TrBdf2 && ++cleanSteps_ >= auto_.cleanBack) {
    integ_ = Integrator::Trapezoidal;
    ++stats_.integratorSwitches;
    cleanSteps_ = 0;
  }
}

void TransientSolver::step() {
  // Gate-driven switch states are already set on the circuit. Steps whose
  // topology signature (switch/diode states, values, dt) is unchanged
  // reuse the cached factorization (RHS-only assemble); anything else
  // re-factorizes. Diodes iterate to consistency within the step (no time
  // advance during iteration), then histories advance once.
  if (rowA_.size() != circuit_.devices().size()) {
    throw std::runtime_error("topology change mid-run: only device values may change");
  }
  if (!adaptive_.enabled) {
    const long long r0 = stats_.resolves, n0 = stats_.newtonIters;
    fixedStep(dt_);
    // Committed step: stiffness detection sees accepted work only.
    autoUpdate(static_cast<int>(stats_.newtonIters - n0),
               static_cast<int>(stats_.resolves - r0));
    return;
  }
  if (integ_ == Integrator::TrBdf2) {
    // Embedded stage-difference control (one converged step per attempt).
    embeddedStep();
    return;
  }
  // Adaptive step-doubling: full step vs two half steps on node voltages.
  // The accepted state is always the (more accurate) half-step one.
  int condFails = 0;  // consecutive ill-conditioned attempts (grow to escape)
  for (int attempt = 0; attempt < 50; ++attempt) {
    const double dtTry = dt_;
    const Snapshot s0 = snapshot();
    try {
      fixedStep(dtTry);
    } catch (const SingularError&) {
      restore(s0);
      if (++condFails > 5) throw;
      dt_ = std::min(adaptive_.dtMax, dtTry * 2.0);
      continue;
    }
    condFails = 0;
    const Eigen::VectorXd xFull = x_;
    restore(s0);
    const SolverStats base = stats_;
    try {
      fixedStep(dtTry * 0.5);
      fixedStep(dtTry * 0.5);
    } catch (const SingularError&) {
      restore(s0);
      if (++condFails > 5) throw;
      dt_ = std::min(adaptive_.dtMax, dtTry * 2.0);
      continue;
    }
    condFails = 0;
    dt_ = dtTry;  // fixedStep() leaves dt_/2 behind; controller owns dt_
    const Eigen::VectorXd xHalf = x_;
    const bool diodesFired = stats_.diodeEvents != base.diodeEvents;
    double err = 0.0;
    for (Eigen::Index i = 0; i < x_.size(); ++i) {
      const double denom = 1e-6 + std::abs(xHalf[i]);
      const double e = std::abs(xFull[i] - xHalf[i]) / denom;
      if (e > err) err = e;
    }
    const bool atFloor = dtTry <= adaptive_.dtMin * (1.0 + 1e-9);
    if (diodesFired) {
      // Commutation inside the step: the error estimate is invalid across
      // discontinuities (measured: err pins at ~1 from dtMax to dtMin —
      // no dt resolves an event straddle, so shrink-spiralling only ends
      // at dtMin's ill-conditioned companions and a guard throw). Accept
      // at current dt unchanged (no shrink ratchet, no growth); accuracy
      // resumes when quiet. Same principle as SPICE LTE bypass on events.
      autoUpdate(static_cast<int>(stats_.newtonIters - base.newtonIters),
                 static_cast<int>(stats_.resolves - base.resolves));
      lastEmbeddedErr_ = err;  // may exceed tol: documented exception
      stats_.steps = base.steps + 1;  // halves did two steps; count one
      return;
    }
    if (err <= adaptive_.tol || atFloor) {
      // Accept half-step state (already committed); fix step counters to
      // count one accepted step with the half-path resolves/events.
      stats_.steps = base.steps + 1;
      autoUpdate(static_cast<int>(stats_.newtonIters - base.newtonIters),
                 static_cast<int>(stats_.resolves - base.resolves));
      lastEmbeddedErr_ = err;
      // Grow unless diodes fired (non-smooth) or the step was floor-forced
      // by excess error (atFloor with err > tol: hold, don't grow).
      if (!diodesFired && err > 0.0 && (err <= adaptive_.tol || !atFloor)) {
        const double factor = growFactor(adaptive_.tol, err);
        dt_ = std::min(adaptive_.dtMax, std::max(adaptive_.dtMin, dtTry * factor));
      }
      return;
    }
    restore(s0);  // reject: full revert, shrink and retry
    const double factor = shrinkFactor(adaptive_.tol, err);
    dt_ = std::max(adaptive_.dtMin, dtTry * factor);
  }
  // Extremely stiff spot: force-advance at dtMin rather than stall (see
  // above: escalate on ill-conditioning, rethrow if nothing solves).
  for (double dtForce = adaptive_.dtMin;;) {
    const long long r0 = stats_.resolves, n0 = stats_.newtonIters;
    try {
      fixedStep(dtForce);
    } catch (const SingularError&) {
      if (!(dtForce < adaptive_.dtMax)) throw;
      dtForce = std::min(adaptive_.dtMax, dtForce * 10.0);
      continue;
    }
    dt_ = dtForce;
    autoUpdate(static_cast<int>(stats_.newtonIters - n0),
               static_cast<int>(stats_.resolves - r0));
    break;
  }
}

void TransientSolver::stepTo(double tLimit) {
  if (!adaptive_.enabled) throw std::runtime_error("stepTo needs adaptive mode");
  int guard = 0;
  const double eps = 1e-12 * (1.0 + std::abs(tLimit));
  while (t_ < tLimit - eps) {
    dt_ = std::min(dt_, tLimit - t_);
    step();  // advances exactly the entry dt_ (floor-accept guarantees it)
    if (++guard > 10000000) throw std::runtime_error("stepTo did not converge");
  }
}

double TransientSolver::nodeVoltage(int node) const {
  if (node == 0) return 0.0;
  auto it = nodeIndex_.find(node);
  if (it == nodeIndex_.end()) throw std::runtime_error("unknown node");
  return x_(static_cast<Eigen::Index>(it->second));
}

double TransientSolver::deviceCurrent(const std::string& name) const {
  return circuit_.devices()[circuit_.deviceIndex(name)].i_prev;
}

}  // namespace power_engine
