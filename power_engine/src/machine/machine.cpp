#include "power_engine/machine.h"

#include <cmath>
#include <complex>
#include <numbers>
#include <stdexcept>

namespace power_engine {
namespace machine {
namespace {

constexpr double kPi = std::numbers::pi;

}  // namespace

void stepMechanical(MechanicalState& st, double te, double tload,
                    const MechanicalParams& p, double dt) {
  if (!(p.j > 0.0) || !std::isfinite(p.j)) {
    throw std::runtime_error("mechanical J must be positive finite");
  }
  if (!(p.b >= 0.0) || !std::isfinite(p.b)) {
    throw std::runtime_error("mechanical B must be finite >= 0");
  }
  if (!(dt > 0.0) || !std::isfinite(dt)) throw std::runtime_error("mechanical dt must be positive");
  if (!std::isfinite(te) || !std::isfinite(tload) || !std::isfinite(st.omega) ||
      !std::isfinite(st.theta)) {
    throw std::runtime_error("mechanical state/torque must be finite");
  }
  const double tNet = te - tload;
  if (p.b == 0.0) {
    st.theta += st.omega * dt + 0.5 * tNet / p.j * dt * dt;
    st.omega += tNet / p.j * dt;
    return;
  }
  const double wInf = tNet / p.b;
  const double decay = std::exp(-p.b / p.j * dt);
  st.theta += wInf * dt + (st.omega - wInf) * p.j / p.b * (1.0 - decay);
  st.omega = wInf + (st.omega - wInf) * decay;
}

ThreePhase pmsmEmf(double theta, double omega, const PmsmParams& m) {
  if (m.polePairs < 1) throw std::runtime_error("PMSM needs polePairs >= 1");
  if (!(m.lambdaPm >= 0.0) || !std::isfinite(m.lambdaPm)) {
    throw std::runtime_error("PMSM lambdaPm must be finite >= 0");
  }
  const double thE = m.polePairs * theta;
  const double amp = m.polePairs * omega * m.lambdaPm;
  ThreePhase e;
  e.a = amp * std::sin(thE);
  e.b = amp * std::sin(thE - 2.0 * kPi / 3.0);
  e.c = amp * std::sin(thE + 2.0 * kPi / 3.0);
  return e;
}

void park(double ia, double ib, double ic, double thE, double& id, double& iq) {
  if (!std::isfinite(ia) || !std::isfinite(ib) || !std::isfinite(ic) ||
      !std::isfinite(thE)) {
    throw std::runtime_error("Park inputs must be finite");
  }
  // Amplitude-invariant, q from +sine row so that ia = I*sin(thE + offsets)
  // gives id = 0, iq = +I (motoring-positive with the EMF above).
  id = (2.0 / 3.0) *
       (ia * std::cos(thE) + ib * std::cos(thE - 2.0 * kPi / 3.0) +
        ic * std::cos(thE + 2.0 * kPi / 3.0));
  iq = (2.0 / 3.0) *
       (ia * std::sin(thE) + ib * std::sin(thE - 2.0 * kPi / 3.0) +
        ic * std::sin(thE + 2.0 * kPi / 3.0));
}

double pmsmTorque(double id, double iq, const PmsmParams& m) {
  if (!std::isfinite(id) || !std::isfinite(iq)) {
    throw std::runtime_error("PMSM torque inputs must be finite");
  }
  if (m.polePairs < 1) throw std::runtime_error("PMSM needs polePairs >= 1");
  return 1.5 * m.polePairs * (m.lambdaPm * iq + (m.ld - m.lq) * id * iq);
}

namespace {

struct InductionDeriv {
  double fds = 0.0, fqs = 0.0, fdr = 0.0, fqr = 0.0;
};

void checkInduction(const InductionParams& m) {
  if (!(m.rs >= 0.0) || !std::isfinite(m.rs) || !(m.rr > 0.0) ||
      !std::isfinite(m.rr) || !(m.ls > 0.0) || !std::isfinite(m.ls) ||
      !(m.lr > 0.0) || !std::isfinite(m.lr) || !(m.lm > 0.0) ||
      !std::isfinite(m.lm)) {
    throw std::runtime_error("induction R/L must be finite (Rr,Ls,Lr,Lm > 0)");
  }
  if (m.polePairs < 1) throw std::runtime_error("induction needs polePairs >= 1");
  if (!(m.lm * m.lm < m.ls * m.lr)) {
    throw std::runtime_error("induction needs Lm^2 < Ls*Lr (leakage > 0)");
  }
}

InductionDeriv inductionRhs(const InductionState& st, double vds, double vqs, double we,
                            double wr, const InductionParams& m) {
  const double det = m.ls * m.lr - m.lm * m.lm;
  const double ids = (m.lr * st.fds - m.lm * st.fdr) / det;
  const double iqs = (m.lr * st.fqs - m.lm * st.fqr) / det;
  const double idr = (m.ls * st.fdr - m.lm * st.fds) / det;
  const double iqr = (m.ls * st.fqr - m.lm * st.fqs) / det;
  const double wrSlip = we - wr;
  InductionDeriv d;
  d.fds = vds - m.rs * ids + we * st.fqs;
  d.fqs = vqs - m.rs * iqs - we * st.fds;
  d.fdr = -m.rr * idr + wrSlip * st.fqr;
  d.fqr = -m.rr * iqr - wrSlip * st.fdr;
  return d;
}

}  // namespace

void stepInduction(InductionState& st, double vds, double vqs, double we,
                   double wr, const InductionParams& m, double dt) {
  checkInduction(m);
  if (!(dt > 0.0) || !std::isfinite(dt)) throw std::runtime_error("induction dt must be positive");
  if (!std::isfinite(vds) || !std::isfinite(vqs) || !std::isfinite(we) ||
      !std::isfinite(wr) || !std::isfinite(st.fds) || !std::isfinite(st.fqs) ||
      !std::isfinite(st.fdr) || !std::isfinite(st.fqr)) {
    throw std::runtime_error("induction step inputs must be finite");
  }
  // Classical RK4 on the flux state (linear time-varying via wr/we).
  const InductionDeriv k1 = inductionRhs(st, vds, vqs, we, wr, m);
  InductionState s2{st.fds + 0.5 * dt * k1.fds, st.fqs + 0.5 * dt * k1.fqs,
                    st.fdr + 0.5 * dt * k1.fdr, st.fqr + 0.5 * dt * k1.fqr};
  const InductionDeriv k2 = inductionRhs(s2, vds, vqs, we, wr, m);
  InductionState s3{st.fds + 0.5 * dt * k2.fds, st.fqs + 0.5 * dt * k2.fqs,
                    st.fdr + 0.5 * dt * k2.fdr, st.fqr + 0.5 * dt * k2.fqr};
  const InductionDeriv k3 = inductionRhs(s3, vds, vqs, we, wr, m);
  InductionState s4{st.fds + dt * k3.fds, st.fqs + dt * k3.fqs, st.fdr + dt * k3.fdr,
                    st.fqr + dt * k3.fqr};
  const InductionDeriv k4 = inductionRhs(s4, vds, vqs, we, wr, m);
  st.fds += dt / 6.0 * (k1.fds + 2.0 * k2.fds + 2.0 * k3.fds + k4.fds);
  st.fqs += dt / 6.0 * (k1.fqs + 2.0 * k2.fqs + 2.0 * k3.fqs + k4.fqs);
  st.fdr += dt / 6.0 * (k1.fdr + 2.0 * k2.fdr + 2.0 * k3.fdr + k4.fdr);
  st.fqr += dt / 6.0 * (k1.fqr + 2.0 * k2.fqr + 2.0 * k3.fqr + k4.fqr);
}

void inductionCurrents(const InductionState& st, const InductionParams& m,
                       double& ids, double& iqs, double& idr, double& iqr) {
  checkInduction(m);
  const double det = m.ls * m.lr - m.lm * m.lm;
  ids = (m.lr * st.fds - m.lm * st.fdr) / det;
  iqs = (m.lr * st.fqs - m.lm * st.fqr) / det;
  idr = (m.ls * st.fdr - m.lm * st.fds) / det;
  iqr = (m.ls * st.fqr - m.lm * st.fqs) / det;
}

double inductionTorque(const InductionState& st, double ids, double iqs,
                       const InductionParams& m) {
  if (m.polePairs < 1) throw std::runtime_error("induction needs polePairs >= 1");
  return 1.5 * m.polePairs * (st.fds * iqs - st.fqs * ids);
}

double inductionSteadyTorque(const InductionParams& m, double vPhaseRms,
                             double freqHz, double slip) {
  checkInduction(m);
  if (!(vPhaseRms >= 0.0) || !std::isfinite(vPhaseRms) || !(freqHz > 0.0) ||
      !std::isfinite(freqHz) || !(slip > 0.0) || !(slip <= 1.0) ||
      !std::isfinite(slip)) {
    throw std::runtime_error("inductionSteadyTorque needs V>=0, f>0, 0<s<=1");
  }
  const double we = 2.0 * kPi * freqHz;
  const std::complex<double> js(0.0, 1.0);
  const std::complex<double> zs = m.rs + js * we * (m.ls - m.lm);
  const std::complex<double> zm = js * we * m.lm;
  const std::complex<double> zr = m.rr / slip + js * we * (m.lr - m.lm);
  const std::complex<double> zin = zs + zm * zr / (zm + zr);
  const std::complex<double> is = vPhaseRms / zin;
  const std::complex<double> i2 = is * zm / (zm + zr);
  return 3.0 * std::norm(i2) * (m.rr / slip) * (m.polePairs / we);
}

}  // namespace machine
}  // namespace power_engine
