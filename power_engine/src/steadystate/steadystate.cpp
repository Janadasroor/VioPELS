#include "power_engine/steadystate.h"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <Eigen/Dense>

#include "power_engine/engine.h"

namespace power_engine {
namespace steadystate {
namespace {

// One continuous state slot: which device, which quantity.
struct StateSlot {
  std::size_t devIdx = 0;
  int kind = 0;  // 0: inductor i, 1: coupled i1, 2: coupled i2,
                 // 3: saturable i, 4: capacitor v
};

double getSlot(const Device& d, int kind) {
  switch (kind) {
    case 0:
    case 1:
    case 3:
      return d.i_prev;
    case 2:
      return d.i2_prev;
    case 4:
      return d.v_prev;
    default:
      break;
  }
  return 0.0;  // unreachable
}

void setSlot(Device& d, int kind, double v) {
  switch (kind) {
    case 0:
    case 1:
    case 3:
      d.i_prev = v;
      break;
    case 2:
      d.i2_prev = v;
      break;
    case 4:
      d.v_prev = v;
      break;
    default:
      break;
  }
  if (d.type == DeviceType::SatInductor) {
    // Keep derived quantities consistent with the operating current.
    d.flux = satFlux(d.i_prev, d.value, d.lsat, d.isat);
    d.newton_ik = d.i_prev;
  }
}

double slotNorm(const Eigen::VectorXd& r, const Eigen::VectorXd& x0,
                const ShootingConfig& cfg) {
  double nres = 0.0;
  for (Eigen::Index i = 0; i < r.size(); ++i) {
    const double scale = cfg.absTol + cfg.relTol * std::abs(x0[i]);
    const double q = std::abs(r[i]) / scale;
    if (q > nres) nres = q;
  }
  return nres;
}

}  // namespace

ShootingResult solvePeriodicSteadyState(Engine& eng, double t0, const ShootingConfig& cfg) {
  if (!(cfg.period > 0.0) || !std::isfinite(cfg.period)) {
    throw std::runtime_error("shooting needs a positive finite period");
  }
  if (cfg.maxIters <= 0) throw std::runtime_error("shooting needs maxIters > 0");
  if (!(cfg.relTol > 0.0) || !(cfg.absTol > 0.0) || !(cfg.fdStep > 0.0)) {
    throw std::runtime_error("shooting needs positive tols and fdStep");
  }
  if (!(t0 >= 0.0) || !std::isfinite(t0)) {
    throw std::runtime_error("shooting needs finite t0 >= 0");
  }
  if (eng.status() != SimulationStatus::Running) {
    throw std::runtime_error("shooting needs a started engine (call start() first)");
  }
  if (eng.stopTime() > 0.0) {
    throw std::runtime_error("shooting needs no stop time (clearStopTime first)");
  }
  if (eng.time() > t0 + 1e-12) {
    throw std::runtime_error("shooting needs t0 at or after current time; runUntil(t0) first if needed");
  }

  // Build the continuous-state map once (topology is fixed during shooting).
  std::vector<StateSlot> slots;
  {
    const auto& devs = eng.circuit().devices();
    for (std::size_t i = 0; i < devs.size(); ++i) {
      const auto& d = devs[i];
      switch (d.type) {
        case DeviceType::Inductor:
          slots.push_back({i, 0});
          break;
        case DeviceType::CoupledInductor:
          slots.push_back({i, 1});
          slots.push_back({i, 2});
          break;
        case DeviceType::SatInductor:
          slots.push_back({i, 3});
          break;
        case DeviceType::HystereticInductor:
          // Stateful (hS/hB memory) with no slot kind: silently skipping
          // would converge the wrong orbit. Reject like statespace does
          // for unsupported devices (transformers stay skippable: stateless).
          throw std::runtime_error("shooting needs L/C states: hysteretic inductor '" +
                                   d.name + "' has unmapped memory state");
          break;
        case DeviceType::Capacitor:
          slots.push_back({i, 4});
          break;
        default:
          break;
      }
    }
  }
  if (slots.empty()) throw std::runtime_error("shooting needs at least one L/C state");
  const auto n = static_cast<Eigen::Index>(slots.size());

  auto extract = [&](const SolverState& s) {
    Eigen::VectorXd x(n);
    for (Eigen::Index i = 0; i < n; ++i) x[i] = getSlot(s.devices[slots[i].devIdx], slots[i].kind);
    return x;
  };
  auto inject = [&](SolverState& s, const Eigen::VectorXd& x) {
    for (Eigen::Index i = 0; i < n; ++i) setSlot(s.devices[slots[i].devIdx], slots[i].kind, x[i]);
  };
  // One exact period run from a state blob; returns end continuous states.
  auto runPeriod = [&](const SolverState& s0) {
    eng.restoreSolverState(s0);
    eng.resetScheduledEvents();
    eng.resetAccumulators();
    eng.runUntil(t0 + cfg.period);
    return extract(eng.saveSolverState());
  };

  // Position at t0 if needed, then snapshot the orbit-start guess.
  if (eng.time() < t0 - 1e-12) {
    eng.resetScheduledEvents();
    eng.runUntil(t0);
  }
  eng.resetAccumulators();
  SolverState s0 = eng.saveSolverState();
  Eigen::VectorXd x0 = extract(s0);

  ShootingResult out;
  double best = std::numeric_limits<double>::infinity();
  SolverState bestEnd = s0;
  for (int it = 0; it < cfg.maxIters; ++it) {
    const Eigen::VectorXd x1 = runPeriod(s0);
    const Eigen::VectorXd r = x1 - x0;
    const double nres = slotNorm(r, x0, cfg);
    if (nres < best) {
      best = nres;
      bestEnd = eng.saveSolverState();
    }
    if (nres <= 1.0) {
      out.converged = true;
      out.iters = it + 1;
      out.residual = nres;
      eng.rewindTo(bestEnd, t0);
      return out;
    }
    // Finite-difference Jacobian of F (period map), then Newton on F(x)-x.
    Eigen::MatrixXd J(n, n);
    for (Eigen::Index j = 0; j < n; ++j) {
      const double h = cfg.fdStep * (1.0 + std::abs(x0[j]));
      SolverState sp = s0;
      Eigen::VectorXd xp = x0;
      xp[j] += h;
      inject(sp, xp);
      const Eigen::VectorXd x1p = runPeriod(sp);
      J.col(j) = (x1p - x1) / h;
    }
    Eigen::MatrixXd A = J;
    for (Eigen::Index i = 0; i < n; ++i) A(i, i) -= 1.0;  // d(F-x)/dx
    Eigen::VectorXd dx = A.fullPivLu().solve(-r);
    if (!dx.allFinite()) break;  // singular step: keep best effort
    // Backtracking: accept the first halving that improves the residual.
    bool improved = false;
    Eigen::VectorXd x0new = x0;
    for (int b = 0; b < 4; ++b) {
      x0new = x0 + dx;
      SolverState st = s0;
      inject(st, x0new);
      const Eigen::VectorXd x1t = runPeriod(st);
      Eigen::VectorXd rt = x1t - x0new;
      if (slotNorm(rt, x0new, cfg) < nres) {
        improved = true;
        break;
      }
      dx *= 0.5;
    }
    if (!improved) break;  // keep best effort
    x0 = x0new;
    inject(s0, x0);
  }
  out.converged = false;
  out.iters = cfg.maxIters;
  out.residual = best;
  eng.rewindTo(bestEnd, t0);
  return out;
}

}  // namespace steadystate
}  // namespace power_engine
