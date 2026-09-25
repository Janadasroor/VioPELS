#include "power_engine/circuit.h"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

namespace power_engine {

void Circuit::checkNodes(int n1, int n2) {
  if (n1 < 0 || n2 < 0) throw std::runtime_error("node ids must be >= 0");
  if (n1 == 0 && n2 == 0) throw std::runtime_error("device shorted to ground on both ends");
  // Same non-ground node on both ends stamps nothing: the device is a silent
  // no-op (refine-roadmap R1). Reject at the boundary instead of running on.
  if (n1 == n2) throw std::runtime_error("device terminals must differ (same node on both ends)");
}

void Circuit::checkPositive(double v, const char* what) {
  if (!(v > 0.0) || !std::isfinite(v)) {
    throw std::runtime_error(std::string(what) + " must be positive finite");
  }
}

void Circuit::checkNameUnique(const std::vector<Device>& devs, const std::string& name) {
  for (const auto& d : devs) {
    if (d.name == name) throw std::runtime_error("duplicate device name: " + name);
  }
  if (name.empty()) throw std::runtime_error("device name must be non-empty");
}

static Device makeDevice(DeviceType t, const std::string& name, int n1, int n2, double value,
                         double ic = 0.0) {
  Device d;
  d.type = t;
  d.name = name;
  d.n1 = n1;
  d.n2 = n2;
  d.value = value;
  d.ic = ic;
  d.v_prev = (t == DeviceType::Capacitor) ? ic : 0.0;
  d.i_prev = (t == DeviceType::Inductor) ? ic : 0.0;
  return d;
}

void Circuit::addResistor(const std::string& name, int n1, int n2, double r) {
  checkNodes(n1, n2);
  checkPositive(r, "R");
  checkNameUnique(devices_, name);
  noteTopologyChange();
  devices_.push_back(makeDevice(DeviceType::Resistor, name, n1, n2, r));
}

void Circuit::addCapacitor(const std::string& name, int n1, int n2, double c, double vc0) {
  checkNodes(n1, n2);
  checkPositive(c, "C");
  if (!std::isfinite(vc0)) throw std::runtime_error("Vc0 must be finite");
  checkNameUnique(devices_, name);
  noteTopologyChange();
  devices_.push_back(makeDevice(DeviceType::Capacitor, name, n1, n2, c, vc0));
}

void Circuit::addInductor(const std::string& name, int n1, int n2, double l, double il0) {
  checkNodes(n1, n2);
  checkPositive(l, "L");
  if (!std::isfinite(il0)) throw std::runtime_error("Il0 must be finite");
  checkNameUnique(devices_, name);
  noteTopologyChange();
  devices_.push_back(makeDevice(DeviceType::Inductor, name, n1, n2, l, il0));
}

void Circuit::addVoltageSource(const std::string& name, int np, int nm, double v) {
  checkNodes(np, nm);
  if (!std::isfinite(v)) throw std::runtime_error("V must be finite");
  checkNameUnique(devices_, name);
  noteTopologyChange();
  devices_.push_back(makeDevice(DeviceType::VoltageSource, name, np, nm, v));
}

void Circuit::addCurrentSource(const std::string& name, int np, int nm, double i) {
  checkNodes(np, nm);
  if (!std::isfinite(i)) throw std::runtime_error("I must be finite");
  checkNameUnique(devices_, name);
  noteTopologyChange();
  devices_.push_back(makeDevice(DeviceType::CurrentSource, name, np, nm, i));
}

void Circuit::addSwitch(const std::string& name, int n1, int n2, double ron, double roff,
                        bool closed, double eon, double eoff, double ttail, double tailk,
                        double tsw) {
  checkNodes(n1, n2);
  checkPositive(ron, "Ron");
  checkPositive(roff, "Roff");
  if (!(roff > ron)) throw std::runtime_error("Roff must exceed Ron");
  if (!(eon >= 0.0) || !std::isfinite(eon)) throw std::runtime_error("Eon must be finite >= 0");
  if (!(eoff >= 0.0) || !std::isfinite(eoff)) {
    throw std::runtime_error("Eoff must be finite >= 0");
  }
  if (!(ttail >= 0.0) || !std::isfinite(ttail)) {
    throw std::runtime_error("Ttail must be finite >= 0");
  }
  if (!(tailk >= 0.0) || !std::isfinite(tailk)) {
    throw std::runtime_error("Tailk must be finite >= 0");
  }
  if (!(tsw >= 0.0) || !std::isfinite(tsw)) {
    throw std::runtime_error("Tsw must be finite >= 0");
  }
  checkNameUnique(devices_, name);
  noteTopologyChange();
  Device d = makeDevice(DeviceType::Switch, name, n1, n2, 0.0);
  d.ron = ron;
  d.roff = roff;
  d.closed = closed;
  d.closedPrev = closed;
  d.eon = eon;
  d.eoff = eoff;
  d.ttail = ttail;
  d.tailk = tailk;
  d.tsw = tsw;
  devices_.push_back(d);
}

void Circuit::addDiode(const std::string& name, int anode, int cathode, double vf, double ron,
                       double roff, double qrr, double trr) {
  checkNodes(anode, cathode);
  if (!std::isfinite(vf) || vf < 0.0) throw std::runtime_error("Vf must be finite non-negative");
  checkPositive(ron, "diode Ron");
  checkPositive(roff, "diode Roff");
  if (!(roff > ron)) throw std::runtime_error("diode Roff must exceed Ron");
  if (!(qrr >= 0.0) || !std::isfinite(qrr)) throw std::runtime_error("Qrr must be finite >= 0");
  if (!(trr >= 0.0) || !std::isfinite(trr)) throw std::runtime_error("Trr must be finite >= 0");
  if (qrr > 0.0 && !(trr > 0.0)) throw std::runtime_error("Qrr needs Trr > 0");
  checkNameUnique(devices_, name);
  noteTopologyChange();
  Device d = makeDevice(DeviceType::Diode, name, anode, cathode, 0.0);
  d.vf = vf;
  d.ron = ron;
  d.roff = roff;
  d.qrr = qrr;
  d.trr = trr;
  d.conducting = false;  // start blocking
  devices_.push_back(d);
}

void Circuit::addSwitchDiode(const std::string& name, int n1, int n2, double ron, double roff,
                             double vf, bool closed, double eon, double eoff, double qrr,
                             double trr, double ttail, double tailk, double tsw) {
  checkNodes(n1, n2);
  checkPositive(ron, "Ron");
  checkPositive(roff, "Roff");
  if (!(roff > ron)) throw std::runtime_error("Roff must exceed Ron");
  if (!std::isfinite(vf) || vf < 0.0) throw std::runtime_error("Vf must be finite non-negative");
  if (!(eon >= 0.0) || !std::isfinite(eon)) throw std::runtime_error("Eon must be finite >= 0");
  if (!(eoff >= 0.0) || !std::isfinite(eoff)) {
    throw std::runtime_error("Eoff must be finite >= 0");
  }
  if (!(qrr >= 0.0) || !std::isfinite(qrr)) throw std::runtime_error("Qrr must be finite >= 0");
  if (!(trr >= 0.0) || !std::isfinite(trr)) throw std::runtime_error("Trr must be finite >= 0");
  if (qrr > 0.0 && !(trr > 0.0)) throw std::runtime_error("Qrr needs Trr > 0");
  if (!(ttail >= 0.0) || !std::isfinite(ttail)) {
    throw std::runtime_error("Ttail must be finite >= 0");
  }
  if (!(tailk >= 0.0) || !std::isfinite(tailk)) {
    throw std::runtime_error("Tailk must be finite >= 0");
  }
  if (!(tsw >= 0.0) || !std::isfinite(tsw)) {
    throw std::runtime_error("Tsw must be finite >= 0");
  }
  checkNameUnique(devices_, name);
  noteTopologyChange();
  Device d = makeDevice(DeviceType::SwitchDiode, name, n1, n2, 0.0);
  d.ron = ron;
  d.roff = roff;
  d.vf = vf;
  d.closed = closed;
  d.closedPrev = closed;
  d.eon = eon;
  d.eoff = eoff;
  d.qrr = qrr;
  d.trr = trr;
  d.ttail = ttail;
  d.tailk = tailk;
  d.tsw = tsw;
  d.conducting = false;  // diode half starts blocking
  devices_.push_back(d);
}

void Circuit::addTransformer(const std::string& name, int np1, int nm1, int np2, int nm2,
                             double ratio) {
  checkNodes(np1, nm1);
  checkNodes(np2, nm2);
  if (!(ratio > 0.0) || !std::isfinite(ratio)) {
    throw std::runtime_error("transformer ratio must be positive finite");
  }
  checkNameUnique(devices_, name);
  noteTopologyChange();
  Device d = makeDevice(DeviceType::Transformer, name, np1, nm1, 0.0);
  d.n3 = np2;
  d.n4 = nm2;
  d.ratio = ratio;
  devices_.push_back(d);
}

void Circuit::addCenterTapTransformer(const std::string& name, int na, int nct, int nb,
                                      int nsp, int nsn, double ratio) {
  checkNodes(na, nct);
  checkNodes(nct, nb);
  checkNodes(nsp, nsn);
  if (!(ratio > 0.0) || !std::isfinite(ratio)) {
    throw std::runtime_error("center-tap transformer ratio must be positive finite");
  }
  checkNameUnique(devices_, name);
  noteTopologyChange();
  Device d = makeDevice(DeviceType::CenterTapTransformer, name, na, nct, 0.0);
  d.n3 = nb;
  d.n4 = nsp;
  d.n5 = nsn;
  d.ratio = ratio;
  devices_.push_back(d);
}

void Circuit::addCoupledInductors(const std::string& name, int n1a, int n1b, int n2a, int n2b,
                                 double l1, double l2, double k, double il10, double il20) {
  checkNodes(n1a, n1b);
  checkNodes(n2a, n2b);
  checkPositive(l1, "L1");
  checkPositive(l2, "L2");
  if (!(k > 0.0) || !(k < 1.0) || !std::isfinite(k)) {
    throw std::runtime_error("coupling k must satisfy 0 < k < 1 (k=1 is singular; use Transformer)");
  }
  if (!std::isfinite(il10) || !std::isfinite(il20)) {
    throw std::runtime_error("coupled initial currents must be finite");
  }
  checkNameUnique(devices_, name);
  noteTopologyChange();
  Device d = makeDevice(DeviceType::CoupledInductor, name, n1a, n1b, 0.0, il10);
  d.n3 = n2a;
  d.n4 = n2b;
  d.l1 = l1;
  d.l2 = l2;
  d.m = k * std::sqrt(l1 * l2);
  d.ic2 = il20;
  d.i2_prev = il20;
  devices_.push_back(d);
}

void Circuit::addSaturableInductor(const std::string& name, int n1, int n2, double lunsat,
                                   double lsat, double isat, double il0) {
  checkNodes(n1, n2);
  checkPositive(lunsat, "Lunsat");
  checkPositive(lsat, "Lsat");
  checkPositive(isat, "Isat");
  if (!(lsat <= lunsat)) throw std::runtime_error("Lsat must not exceed Lunsat");
  if (!std::isfinite(il0)) throw std::runtime_error("Il0 must be finite");
  checkNameUnique(devices_, name);
  noteTopologyChange();
  Device d = makeDevice(DeviceType::SatInductor, name, n1, n2, lunsat, il0);
  d.lsat = lsat;
  d.isat = isat;
  d.flux = satFlux(il0, lunsat, lsat, isat);
  d.newton_ik = il0;
  devices_.push_back(d);
}

void Circuit::addHystereticInductor(const std::string& name, int n1, int n2, double turns,
                                    double ae, double le, double ve, double bs, double a,
                                    double hc, double il0) {
  checkNodes(n1, n2);
  checkPositive(turns, "turns");
  checkPositive(ae, "Ae");
  checkPositive(le, "le");
  checkPositive(ve, "Ve");
  checkPositive(bs, "Bs");
  checkPositive(a, "shape field a");
  if (!(hc >= 0.0) || !std::isfinite(hc)) throw std::runtime_error("Hc must be finite >= 0");
  if (!std::isfinite(il0)) throw std::runtime_error("Il0 must be finite");
  checkNameUnique(devices_, name);
  noteTopologyChange();
  Device d = makeDevice(DeviceType::HystereticInductor, name, n1, n2, 0.0, il0);
  d.hTurns = turns;
  d.hAe = ae;
  d.hLe = le;
  d.hVe = ve;
  d.hBs = bs;
  d.hA = a;
  d.hHc = hc;
  d.i_prev = il0;
  const double h0 = turns * il0 / le;
  d.hH = h0;
  d.hS = 0.0;
  d.hB = bs * std::tanh(h0 / a);
  d.hLoss = 0.0;
  d.flux = turns * ae * d.hB;
  devices_.push_back(d);
}

Device& Circuit::findDevice(const std::string& name) {
  return devices_[deviceIndex(name)];
}

const Device& Circuit::findDevice(const std::string& name) const {
  return devices_[deviceIndex(name)];
}

std::size_t Circuit::deviceIndex(const std::string& name) const {
  if (index_.size() != devices_.size()) {
    index_.clear();
    for (std::size_t i = 0; i < devices_.size(); ++i) index_[devices_[i].name] = i;
  }
  const auto it = index_.find(name);
  if (it == index_.end()) throw std::runtime_error("unknown device: " + name);
  return it->second;
}

void Circuit::setSwitch(const std::string& name, bool closed) {
  Device& d = findDevice(name);
  if (d.type != DeviceType::Switch && d.type != DeviceType::SwitchDiode)
    throw std::runtime_error("setSwitch on non-switch: " + name);
  d.closed = closed;
}

bool Circuit::switchClosed(const std::string& name) const {
  const Device& d = findDevice(name);
  if (d.type != DeviceType::Switch && d.type != DeviceType::SwitchDiode)
    throw std::runtime_error("switchClosed on non-switch: " + name);
  return d.closed;
}

bool Circuit::diodeConducting(const std::string& name) const {
  const Device& d = findDevice(name);
  if (d.type != DeviceType::Diode) throw std::runtime_error("diodeConducting on non-diode: " + name);
  return d.conducting;
}

std::vector<int> Circuit::nodes() const {
  std::set<int> s;
  for (const auto& d : devices_) {
    if (d.n1 != 0) s.insert(d.n1);
    if (d.n2 != 0) s.insert(d.n2);
    if (d.type == DeviceType::Transformer || d.type == DeviceType::CoupledInductor) {
      if (d.n3 != 0) s.insert(d.n3);
      if (d.n4 != 0) s.insert(d.n4);
    } else if (d.type == DeviceType::CenterTapTransformer) {
      if (d.n3 != 0) s.insert(d.n3);
      if (d.n4 != 0) s.insert(d.n4);
      if (d.n5 != 0) s.insert(d.n5);
    }
  }
  return {s.begin(), s.end()};
}

std::size_t Circuit::numVoltageSources() const {
  std::size_t n = 0;
  for (const auto& d : devices_)
    if (d.type == DeviceType::VoltageSource) ++n;
  return n;
}

std::size_t Circuit::numExtraUnknowns() const {
  std::size_t n = 0;
  for (const auto& d : devices_) {
    if (d.type == DeviceType::VoltageSource) {
      n += 1;
    } else if (d.type == DeviceType::Transformer) {
      n += 2;  // primary + secondary branch currents
    } else if (d.type == DeviceType::CenterTapTransformer) {
      n += 3;  // IA + IB + IS branch currents
    }
  }
  return n;
}

}  // namespace power_engine
