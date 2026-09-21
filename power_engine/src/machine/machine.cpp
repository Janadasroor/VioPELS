#include "power_engine/machine.h"

#include <cmath>
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

}  // namespace machine
}  // namespace power_engine
