#include "power_engine/machine.h"

#include <cmath>
#include <complex>
#include <numbers>
#include <stdexcept>

namespace power_engine {
namespace machine {
namespace {

constexpr double kPi = std::numbers::pi;

// Guarded positive value for divisor positions (also silences MSVC's
// potential-divide-by-zero heuristic on motor parameters).
double reqPos(double v, const char* what) {
  if (!(v > 0.0) || !std::isfinite(v)) throw std::runtime_error(what);
  return v;
}

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
      pid_(2.0 * kPi * reqPos(p.currentBandwidthHz, "FOC needs bandwidth > 0") *
               reqPos(p.motor.ld, "FOC needs Ld > 0"),
           2.0 * kPi * p.currentBandwidthHz * p.motor.rs,
           -p.vdc / 2.0, p.vdc / 2.0),
      piq_(2.0 * kPi * reqPos(p.currentBandwidthHz, "FOC needs bandwidth > 0") *
               reqPos(p.motor.lq, "FOC needs Lq > 0"),
           2.0 * kPi * p.currentBandwidthHz * p.motor.rs,
           -p.vdc / 2.0, p.vdc / 2.0),
      pwm_{control::Pwm(reqPos(p.carrierFreq, "FOC needs carrier freq > 0"), 0.0, 0.0,
                        control::Carrier::Symmetric),
           control::Pwm(p.carrierFreq, 0.0, 0.0, control::Carrier::Symmetric),
           control::Pwm(p.carrierFreq, 0.0, 0.0, control::Carrier::Symmetric)},
      svpwm_(p.carrierFreq) {  // validated above (pwm_) + in Svpwm ctor
  reqPos(p.vdc, "FOC needs Vdc > 0");
  if (p.motor.polePairs < 1) throw std::runtime_error("FOC needs polePairs >= 1");
  reqPos(p.motor.lambdaPm, "FOC needs lambdaPm > 0");
  if (!(p.motor.rs >= 0.0) || !std::isfinite(p.motor.rs))
    throw std::runtime_error("FOC needs finite Rs >= 0");
  if (!(p.speedMaxIq >= 0.0) || !std::isfinite(p.speedMaxIq))
    throw std::runtime_error("FOC needs finite speedMaxIq >= 0");
  if (!(p.maxCurrent >= 0.0) || !std::isfinite(p.maxCurrent))
    throw std::runtime_error("FOC needs finite maxCurrent >= 0");
  if (!(p.fwKi >= 0.0) || !std::isfinite(p.fwKi))
    throw std::runtime_error("FOC needs finite fwKi >= 0");
  if (!(p.voltMargin > 0.0) || !std::isfinite(p.voltMargin))
    throw std::runtime_error("FOC needs finite voltMargin > 0");
  kTq_ = 1.5 * p.motor.polePairs * p.motor.lambdaPm;  // > 0 by above
}

FocController::Gates FocController::update(double t, double wRef, double accelFF, double w,
                                           const ThreePhase& i, double thE, double dt) {
  park(i.a, i.b, i.c, thE, id_, iq_);
  iqRef_ = std::min(std::max(speedPi_.update(wRef - w, dt) +
                                 (p_.motor.mech.j * accelFF + p_.motor.mech.b * wRef) / kTq_,
                             0.0),
                    p_.speedMaxIq);
  const double we = p_.motor.polePairs * w;
  // Mode ceiling first: FW sizes id* against the same budget the final
  // clamp enforces (Vdc/2 carrier, Vdc/sqrt(3) SVPWM). Runtime sqrt:
  // MSVC-hostile only as constexpr (see Svpwm::sequence).
  const double half = p_.vdc / 2.0;
  const double vmax = p_.voltMargin *
                      (p_.modulation == FocParams::Modulation::Svpwm ? p_.vdc / std::sqrt(3.0)
                                                                    : half);
  // Field weakening + MTPA (opt-in; off => id* identically 0, untouched).
  // Feedforward + feedback split: the static Rs-inclusive voltage ellipse
  // (vd = Rs*id+we*Lq*iq, vq = Rs*iq-we*Ld*id+we*lam, vd^2+vq^2 <= vmax^2;
  // B here is the negative of the textbook form — same Park-convention
  // reason as the decoupling above) gives instant bounded bulk action via
  // the min-abs root, while a voltage-feedback trim (idfb_ on
  // |v_cmd|-0.97*vmax, +-5A authority) corrects residuals. FW drives id*
  // POSITIVE here: under this codebase's Park convention, +id lowers
  // terminal voltage (plant-ID proven to <1.2%; textbook demag-id*
  // conventions assume the opposite q-handing and fight the plant by
  // ~7.5V). Above base speed FW wins over MTPA (voltage constraint
  // dominates efficiency); below base MTPA alone applies.
  idRef_ = 0.0;
  if (p_.fieldWeakening) {
    const double lam = p_.motor.lambdaPm;
    const double ld = p_.motor.ld, lq = p_.motor.lq, rs = p_.motor.rs;
    // MTPA (IPM only; Ld == Lq gives exactly 0): id* for torque optimality.
    double idmtpa = 0.0;
    if (lq > ld) {
      idmtpa = (lam - std::sqrt(lam * lam + 8.0 * (lq - ld) * (lq - ld) * iqRef_ * iqRef_)) /
               (4.0 * (lq - ld));  // <= 0
    }
    // Feedforward: min-abs root of A*id^2+B*id+C (0 when id = 0 fits).
    const double awe = std::abs(we);
    const double a = rs * rs + awe * awe * ld * ld;
    const double b = 2.0 * awe * (rs * lq * iqRef_ - ld * (rs * iqRef_ + awe * lam));
    const double c = awe * awe * lq * lq * iqRef_ * iqRef_ +
                     (rs * iqRef_ + awe * lam) * (rs * iqRef_ + awe * lam) - vmax * vmax;
    double idff = 0.0;
    if (c > 0.0) {
      const double disc = b * b - 4.0 * a * c;
      if (disc >= 0.0) {
        const double sq = std::sqrt(disc);
        const double r1 = (-b + sq) / (2.0 * a);
        const double r2 = (-b - sq) / (2.0 * a);
        idff = (std::abs(r1) < std::abs(r2)) ? r1 : r2;
      }
    }
    // Feedback trim on residual over-modulation (conditional: rest at 0).
    // Positive direction: raises id* to pull terminal voltage down.
    constexpr double kFwTrimAuth = 5.0;  // [A] trim authority
    const double err = vmagPrev_ - 0.97 * vmax;  // > 0 means over-modulating
    if (idfb_ > 0.0 || err > 0.0) idfb_ += p_.fwKi * err * dt;
    if (idfb_ < 0.0) idfb_ = 0.0;
    if (idfb_ > kFwTrimAuth) idfb_ = kFwTrimAuth;
    // FW wins above base (idff/idfb_ nonzero); MTPA applies below base.
    double idFw = idff + idfb_;
    idRef_ = (idff != 0.0 || idfb_ != 0.0) ? idFw : idmtpa;
    // No MTPV iq management: beyond-capability operation saturates demand
    // honestly (follow-up). The yoke below bounds the demand step so a
    // railed d-PI can never pin vmag high forever.
    // Cascade demand yoke: never demand more than kFwLead above what the
    // d-axis actually delivers (keeps vd honest so vmag reflects need).
    constexpr double kFwLead = 2.0;  // [A]
    if (idRef_ > id_ + kFwLead) idRef_ = id_ + kFwLead;
    // Current circle shared by torque + demag (angle-preserving scale).
    const double lim = p_.maxCurrent > 0.0 ? p_.maxCurrent : p_.speedMaxIq;
    const double im = std::hypot(idRef_, iqRef_);
    if (im > lim && im > 0.0) {
      idRef_ *= lim / im;
      iqRef_ *= lim / im;
    }
  }
  double vd = pid_.update(idRef_ - id_, dt) + we * p_.motor.lq * iq_;
  double vq = piq_.update(iqRef_ - iq_, dt) + we * (p_.motor.lambdaPm - p_.motor.ld * id_);
  // Decoupling signs are load-bearing (refine R-deep-dive 2026-09-23): for
  // this codebase's park (q from the +sine row), the plant is
  // vd = Rs*id + we*Lq*iq, vq = Rs*iq - we*Ld*id + we*lam (proven by
  // finite-difference Park math + live open-loop plant ID to <1.2%),
  // i.e. OPPOSITE cross terms to Krause leading-q textbooks. Textbook
  // signs here fight the plant whenever id != 0 (7.5V error at the FW
  // point) while passing every id = 0 test invisibly (vq term vanishes,
  // vd error absorbed by the d-PI).
  // Final protection under the same ceiling FW sized against (above).
  const double m = std::hypot(vd, vq);
  vmagPrev_ = m;  // next step's FW feedback sees this step's demand
  if (m > vmax && m > 0.0) {
    vd *= vmax / m;
    vq *= vmax / m;
  }
  vd_ = vd;
  vq_ = vq;
  const ThreePhase v = inversePark(vd, vq, thE);
  if (p_.modulation == FocParams::Modulation::Svpwm) {
    // Space-vector sequence at the same switching frequency: Clarke the
    // commanded phase voltages, normalize to Vdc/2 (sequence() units),
    // and read the symmetric 7-segment states at the intra-period time.
    const control::AlphaBeta ab = control::clarke(v.a, v.b, v.c);
    const control::Svpwm::Sequence seq = svpwm_.sequence(ab.alpha / half, ab.beta / half);
    const double tp = std::fmod(t, svpwm_.period());  // t >= 0 -> [0, T)
    const control::Svpwm::PhaseState sw =
        svpwm_.switches(seq.sector, tp, seq.t1, seq.t2, seq.t0);
    duties_.a = std::min(1.0, std::max(0.0, 0.5 * (1.0 + v.a / half)));
    duties_.b = std::min(1.0, std::max(0.0, 0.5 * (1.0 + v.b / half)));
    duties_.c = std::min(1.0, std::max(0.0, 0.5 * (1.0 + v.c / half)));
    return {sw.a, sw.b, sw.c};
  }
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
