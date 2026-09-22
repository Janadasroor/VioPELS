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

ThreePhase inversePark(double vd, double vq, double thE) {
  if (!std::isfinite(vd) || !std::isfinite(vq) || !std::isfinite(thE)) {
    throw std::runtime_error("Inverse Park inputs must be finite");
  }
  ThreePhase v;
  v.a = vd * std::cos(thE) + vq * std::sin(thE);
  v.b = vd * std::cos(thE - 2.0 * kPi / 3.0) + vq * std::sin(thE - 2.0 * kPi / 3.0);
  v.c = vd * std::cos(thE + 2.0 * kPi / 3.0) + vq * std::sin(thE + 2.0 * kPi / 3.0);
  return v;
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

FocController::FocController() : FocController(FocParams{}) {}

FocController::FocController(const FocParams& p)
    : p_(p),
      speedPi_(p.speedKp, p.speedKi, 0.0, p.speedMaxIq),
      pid_(2.0 * kPi * p.currentBandwidthHz * p.motor.ld,
           2.0 * kPi * p.currentBandwidthHz * p.motor.ld * p.motor.rs / p.motor.ld,
           -p.vdc / 2.0, p.vdc / 2.0),
      piq_(2.0 * kPi * p.currentBandwidthHz * p.motor.lq,
           2.0 * kPi * p.currentBandwidthHz * p.motor.lq * p.motor.rs / p.motor.lq,
           -p.vdc / 2.0, p.vdc / 2.0),
      pwm_{control::Pwm(p.carrierFreq, 0.0, 0.0, control::Carrier::Symmetric),
           control::Pwm(p.carrierFreq, 0.0, 0.0, control::Carrier::Symmetric),
           control::Pwm(p.carrierFreq, 0.0, 0.0, control::Carrier::Symmetric)} {}

FocController::Gates FocController::update(double t, double wRef, double accelFF, double w,
                                           const ThreePhase& i, double thE, double dt) {
  park(i.a, i.b, i.c, thE, id_, iq_);
  const double kTq = 1.5 * p_.motor.polePairs * p_.motor.lambdaPm;
  iqRef_ = std::min(std::max(speedPi_.update(wRef - w, dt) +
                                 (p_.motor.mech.j * accelFF + p_.motor.mech.b * wRef) / kTq,
                             0.0),
                    p_.speedMaxIq);
  const double we = p_.motor.polePairs * w;
  double vd = pid_.update(0.0 - id_, dt) - we * p_.motor.lq * iq_;
  double vq = piq_.update(iqRef_ - iq_, dt) + we * (p_.motor.ld * id_ + p_.motor.lambdaPm);
  // Preserve angle under the linear-modulation ceiling.
  const double vmax = p_.voltMargin * p_.vdc / 2.0;
  const double m = std::hypot(vd, vq);
  if (m > vmax && m > 0.0) {
    vd *= vmax / m;
    vq *= vmax / m;
  }
  vd_ = vd;
  vq_ = vq;
  const ThreePhase v = inversePark(vd, vq, thE);
  const double half = p_.vdc / 2.0;
  duties_.a = std::min(1.0, std::max(0.0, 0.5 * (1.0 + v.a / half)));
  duties_.b = std::min(1.0, std::max(0.0, 0.5 * (1.0 + v.b / half)));
  duties_.c = std::min(1.0, std::max(0.0, 0.5 * (1.0 + v.c / half)));
  pwm_[0].setDuty(duties_.a);
  pwm_[1].setDuty(duties_.b);
  pwm_[2].setDuty(duties_.c);
  return {pwm_[0].output(t), pwm_[1].output(t), pwm_[2].output(t)};
}

}  // namespace machine
}  // namespace power_engine
