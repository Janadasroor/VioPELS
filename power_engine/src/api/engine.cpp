#include "power_engine/engine.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>

#include "power_engine/control.h"

namespace power_engine {

namespace {
constexpr double kTimeEps = 1e-12;
}  // namespace

Engine::Engine() : solver_(circuit_, 1e-6) {}

void Engine::setTimeStep(double dt) { solver_.setStep(dt); }

void Engine::setStopTime(double tStop) {
  if (!(tStop > 0.0) || !std::isfinite(tStop)) throw std::runtime_error("tStop must be positive finite");
  tStop_ = tStop;
}

double Engine::time() const { return solver_.time(); }

void Engine::setSwitch(const std::string& name, bool closed) {
  circuit_.setSwitch(name, closed);  // takes effect on next step()
}

void Engine::scheduleSwitch(const std::string& name, bool closed, double at) {
  // Validate switch exists now (topology is fixed before start()).
  circuit_.setSwitch(name, circuit_.switchClosed(name));  // throws if not a switch
  if (!std::isfinite(at) || at < 0.0) throw std::runtime_error("event time must be finite >= 0");
  // Sorted insert (stable for equal times): callers usually append in time
  // order (per-period scheduling), which stays O(1) amortized; a full sort
  // per call would be O(N^2 log N) total and dominate long PWM runs.
  const SwitchEvent ev{at, name, closed, false};
  if (events_.empty() || !(ev.at < events_.back().at)) {
    events_.push_back(ev);
  } else {
    const auto it = std::upper_bound(
        events_.begin(), events_.end(), ev.at,
        [](double t, const SwitchEvent& e) { return t < e.at; });
    events_.insert(it, ev);
  }
}

void Engine::clearScheduledEvents() { events_.clear(); }

void Engine::applyPwmSpecs() {
  if (!hasNetlist() || net_.pwms.empty()) {
    throw std::runtime_error("applyPwmSpecs needs a loaded netlist with .control pwm");
  }
  if (!(tStop_ > 0.0)) {
    throw std::runtime_error("applyPwmSpecs needs a stop time (.tran tstop or setStopTime)");
  }
  for (const auto& spec : net_.pwms) {
    control::Pwm pwm(spec.freq, spec.duty, spec.phase,
                     spec.symmetric ? control::Carrier::Symmetric
                                    : control::Carrier::TrailingEdge);
    const double T = pwm.period();
    circuit_.setSwitch(spec.switchName, circuit_.switchClosed(spec.switchName));  // validate
    // Seed initial state at t=0 so the first sub-step is consistent.
    circuit_.setSwitch(spec.switchName, pwm.output(0.0));
    for (double edge = pwm.nextEdge(0.0); edge < tStop_; edge = pwm.nextEdge(edge)) {
      // Level just after the edge; nextEdge() is strictly increasing.
      scheduleSwitch(spec.switchName, pwm.output(edge + T * 1e-6), edge);
      if (!(pwm.nextEdge(edge) > edge)) break;  // paranoia against fp stall
    }
    if (!spec.complement.empty()) {
      if (!(spec.deadtime > 0.0)) {
        throw std::runtime_error("complementary pwm needs deadtime>0");
      }
      const double ton = spec.duty * T;
      if (!(spec.deadtime < ton && spec.deadtime < T - ton)) {
        throw std::runtime_error("deadtime too large for pwm duty (needs td < ton and td < toff)");
      }
      circuit_.setSwitch(spec.complement, circuit_.switchClosed(spec.complement));  // validate
      // Complementary edges: lo turns on td after main fall, off td after rise.
      for (double k = 0.0, rise = spec.phase; rise < tStop_; ++k, rise = spec.phase + k * T) {
        double onT, offT;
        if (!spec.symmetric) {
          onT = rise + ton + spec.deadtime;
          offT = rise + T + spec.deadtime;
        } else {
          const double start = (T - ton) * 0.5;
          onT = rise + start + ton + spec.deadtime;
          offT = rise + T + start + spec.deadtime;
        }
        if (onT < tStop_) scheduleSwitch(spec.complement, true, onT);
        if (offT < tStop_) scheduleSwitch(spec.complement, false, offT);
      }
      // Initial complementary state at t=0.
      circuit_.setSwitch(spec.complement, pwm.complementary(0.0, spec.deadtime).lo);
    }
  }
}

std::size_t Engine::pendingEventCount() const {
  std::size_t n = 0;
  for (const auto& e : events_) {
    if (!e.applied) ++n;
  }
  return n;
}

void Engine::loadNetlist(const std::string& text) {
  if (status_ == SimulationStatus::Running) {
    throw std::runtime_error("loadNetlist must be called before start()");
  }
  netlistSource_ = text;
  net_ = netlist::Parser().parse(netlistSource_, paramOverrides_);
  circuit_ = net_.circuit;  // same object: solver_ reference stays valid
  events_.clear();
  started_ = false;
  if (net_.tran.given) {
    solver_.setStep(net_.tran.dt);
    tStop_ = net_.tran.tstop;
    if (net_.tran.method == "AUTO")
      solver_.setIntegratorAuto(true);
    else
      solver_.setIntegrator(net_.tran.method == "TRBDF2" ? Integrator::TrBdf2
                                                         : Integrator::Trapezoidal);
  }
  solver_.initialize();  // rebuild node maps for the new topology
  refreshSolution();
}

void Engine::setParameter(const std::string& name, double value) {
  if (netlistSource_.empty()) {
    throw std::runtime_error("setParameter needs a loaded netlist (loadNetlist first)");
  }
  if (status_ == SimulationStatus::Running || started_) {
    throw std::runtime_error("setParameter must be called before start() (reload netlist to run again)");
  }
  if (!std::isfinite(value)) throw std::runtime_error("parameter value must be finite");
  std::string key = name;
  for (auto& c : key) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  if (key.empty()) throw std::runtime_error("parameter name must be non-empty");
  paramOverrides_[key] = value;
  net_ = netlist::Parser().parse(netlistSource_, paramOverrides_);
  circuit_ = net_.circuit;
  events_.clear();
  if (net_.tran.given) {
    solver_.setStep(net_.tran.dt);
    tStop_ = net_.tran.tstop;
    if (net_.tran.method == "AUTO")
      solver_.setIntegratorAuto(true);
    else
      solver_.setIntegrator(net_.tran.method == "TRBDF2" ? Integrator::TrBdf2
                                                         : Integrator::Trapezoidal);
  }
  solver_.initialize();  // rebuild node maps for the new topology
  refreshSolution();
}

double Engine::deviceCurrent(const std::string& name) const {
  return solver_.deviceCurrent(name);
}

void Engine::start() {
  solver_.initialize();
  status_ = SimulationStatus::Running;
  started_ = true;
  probeNodes_ = solver_.nodeList();
  probeKeys_.clear();
  probeKeys_.reserve(probeNodes_.size());
  solution_.probes.clear();  // drop stale keys if re-started on new topology
  solution_.states.clear();
  solution_.states.reserve(probeNodes_.size());
  for (int n : probeNodes_) probeKeys_.push_back("v:" + std::to_string(n));
  applyDueEvents(0.0);
  resetAccumulators();
  refreshSolution();
}

void Engine::resetScheduledEvents() {
  for (auto& e : events_) e.applied = false;
}

SolverState Engine::saveSolverState() const { return solver_.saveState(); }

void Engine::restoreSolverState(const SolverState& s) { solver_.restoreState(s); }

void Engine::rewindTo(const SolverState& state, double t0) {
  solver_.restoreState(state);
  solver_.setTime(t0);
  resetScheduledEvents();
  applyDueEvents(t0);
  resetAccumulators();
  refreshSolution();
  // R7: repositioning is for continued simulation — a rewind from a
  // Finished run must be steppable again (shooting never sets tStop, so
  // this is a no-op on its path).
  if (status_ == SimulationStatus::Finished) status_ = SimulationStatus::Running;
}

void Engine::resetAccumulators() {
  // (Re)build loss accumulators and gate snapshot for edge detection.
  losses_.clear();
  lastGate_.clear();
  for (const auto& d : circuit_.devices()) {
    if (d.type == DeviceType::Switch || d.type == DeviceType::Diode) {
      losses_[d.name] = thermal::DeviceLoss{};
    }
    if (d.type == DeviceType::Switch) lastGate_[d.name] = d.closed;
  }
  for (auto& [name, net] : thermals_) {
    (void)name;
    net.reset();
  }
}

void Engine::runUntil(double tEnd) {
  if (status_ != SimulationStatus::Running) throw std::runtime_error("Engine not running: call start()");
  if (!(tEnd > solver_.time()) || !std::isfinite(tEnd)) {
    throw std::runtime_error("runUntil needs a finite target after current time");
  }
  if (tStop_ > 0.0) {
    throw std::runtime_error("runUntil needs no stop time (clearStopTime first)");
  }
  while (solver_.time() < tEnd - kTimeEps) step();
}

void Engine::applyDueEvents(double tNow) {
  for (auto& e : events_) {
    if (!e.applied && e.at <= tNow + kTimeEps) {
      circuit_.setSwitch(e.name, e.closed);
      e.applied = true;
    }
  }
}

void Engine::step() {
  if (status_ != SimulationStatus::Running) throw std::runtime_error("Engine not running: call start()");
  const double baseDt = solver_.stepSize();
  double tTarget = solver_.time() + baseDt;
  if (tStop_ > 0.0 && tTarget > tStop_) tTarget = tStop_;
  try {
    applyDueEvents(solver_.time());
    while (solver_.time() < tTarget - kTimeEps) {
      const double tNow = solver_.time();
      // Next pending event strictly after now.
      double tNext = tTarget;
      for (const auto& e : events_) {
        if (!e.applied && e.at > tNow + kTimeEps && e.at < tNext) tNext = e.at;
      }
      if (tNext > tTarget) tNext = tTarget;
      preStepLossHooks();
      // Sliver guard: arbitrary duty edges land at arbitrary sub-step
      // alignments, and picosecond slivers make capacitor companions
      // (2C/dtSub) explode, tripping the solver's relative singularity
      // guard (false positive on a valid matrix). Two cases:
      // (a) event imminent within the sliver: fire it now (<=1ns early,
      //     negligible) and continue; terminates (consumes the event).
      // (b) no event, residual to tTarget within the sliver (a previous
      //     split landed just before the boundary/stop): extend past it
      //     (<=1ns overshoot) and solve normally with safe companions.
      //     The tNext < tTarget condition is load-bearing: skipping the
      //     solve with no event to consume spins forever.
      const double kMinSub = std::max(kTimeEps, 1e-3 * baseDt);
      if (tNext < tTarget && tNext - tNow <= kMinSub) {
        applyDueEvents(tNext);
        continue;
      }
      if (tNext >= tTarget && tTarget - tNow <= kMinSub) {
        tTarget += kMinSub;
        tNext = tTarget;
      }
      if (solver_.adaptive()) {
        const double tBefore = solver_.time();
        solver_.stepTo(tNext);  // error-controlled, lands exactly
        if (solver_.time() > tBefore) updateLosses(solver_.time() - tBefore);
      } else {
        // Note: no zero-step skip here on purpose (an old one could spin
        // forever when the residual has no event in it); dust-scale
        // residuals solve normally below.
        // Full uninterrupted frame: pass baseDt exactly. Recomputing
        // tNext-tNow as (t+dt)-t is NOT bitwise dt in floating point
        // (1-ulp jitter on ~0.2% of steps), which would needlessly
        // invalidate the solver's factorization cache and jitter companions.
        double dtSub = tNext - tNow;
        if (tNext == tTarget && tNext == tNow + baseDt) dtSub = baseDt;
        solver_.setStep(dtSub);
        solver_.step();  // includes diode event iteration
        updateLosses(dtSub);
      }
      applyDueEvents(solver_.time());
    }
    solver_.setStep(baseDt);  // restore nominal step
  } catch (...) {
    solver_.setStep(baseDt);
    status_ = SimulationStatus::Error;
    throw;
  }
  refreshSolution();
  if (callback_) callback_(solution_);
  if (tStop_ > 0.0 && solver_.time() >= tStop_ - kTimeEps) status_ = SimulationStatus::Finished;
}

void Engine::stop() {
  if (status_ == SimulationStatus::Running) status_ = SimulationStatus::Finished;
}

void Engine::refreshSolution() {
  solution_.t = solver_.time();
  solution_.states.clear();
  for (std::size_t k = 0; k < probeNodes_.size(); ++k) {
    const double v = solver_.nodeVoltage(probeNodes_[k]);
    solution_.probes[probeKeys_[k]] = v;  // in place: no clear/reinsert
    solution_.states.push_back(v);
  }
  for (const auto& [name, net] : thermals_) {
    solution_.probes["tj:" + name] = net.tj();
  }
}

void Engine::preStepLossHooks() {
  if (lossModels_.empty()) return;
  for (const auto& [name, model] : lossModels_) {
    Device& d = circuit_.findDevice(name);
    // Pre-solve branch quantities (histories are still pre-sub-step here).
    preStepVi_[name] = {d.v_prev, d.i_prev};
    // One-way explicit Tj coupling: refresh conduction params from the
    // current Tj before stamping (iteration is a later roadmap item).
    const double tj = deviceTemp(name);
    if (model.hasRon()) d.ron = model.ronTj.at({{"TJ", tj}});
    if (model.hasVf()) d.vf = model.vfTj.at({{"TJ", tj}});
  }
}

void Engine::updateLosses(double dtSub) {
  // Gate-edge detection (scheduled or manual setSwitch): newly closed ->
  // Eon, newly opened -> Eoff. Edge energy drives thermals as an impulse
  // spread over this sub-step (exact in energy).
  // Edge I/V sampling (histories are POST-sub-step, i.e. post-edge):
  // turn-on needs pre-edge blocking V + post-edge commutated I;
  // turn-off needs pre-edge on-current I + post-edge blocking V.
  // Pre-edge (v,i) come from the preStepVi_ snapshot taken before the solve.
  std::map<std::string, double> edgeEnergy;
  for (const auto& d : circuit_.devices()) {
    if (d.type != DeviceType::Switch) continue;
    auto it = lastGate_.find(d.name);
    const bool before = (it == lastGate_.end()) ? d.closed : it->second;
    if (d.closed != before) {
      double e = d.closed ? d.eon : d.eoff;
      auto mit = lossModels_.find(d.name);
      if (mit != lossModels_.end()) {
        const loss::Table& tab = d.closed ? mit->second.eon : mit->second.eoff;
        if (!tab.empty()) {
          const auto pit = preStepVi_.find(d.name);
          const double vPre = (pit == preStepVi_.end()) ? 0.0 : pit->second.first;
          const double iPre = (pit == preStepVi_.end()) ? 0.0 : pit->second.second;
          // Turn-on: V = pre-edge blocking, I = post-edge (d.i_prev);
          // turn-off: I = pre-edge, V = post-edge (d.v_prev).
          const double vEdge = d.closed ? vPre : d.v_prev;
          const double iEdge = d.closed ? d.i_prev : iPre;
          const double tj = deviceTemp(d.name);
          e = tab.at({{"I", std::abs(iEdge)},
                      {"V", std::abs(vEdge)},
                      {"TJ", tj}});
        }
      }
      losses_[d.name].esw += e;
      edgeEnergy[d.name] += e;
      lastGate_[d.name] = d.closed;
    }
  }
  for (auto& [name, loss] : losses_) {
    Device& d = circuit_.findDevice(name);
    // Passive-sign conduction power from solved branch quantities:
    // switch I^2*Ron/Roff (+ tail), diode Vf*I + I^2*Ron (or V^2/Roff blocking).
    const double pCond = d.v_prev * d.i_prev;
    loss.econd += pCond * dtSub;
    double pDrive = pCond;
    auto it = edgeEnergy.find(name);
    if (it != edgeEnergy.end()) pDrive += it->second / dtSub;
    if (d.recE != 0.0) {  // diode recovery release energy (Qrr*Vr)
      loss.esw += d.recE;
      pDrive += d.recE / dtSub;
      d.recE = 0.0;
    }
    auto th = thermals_.find(name);
    if (th != thermals_.end()) th->second.step(pDrive, dtSub);
  }
}

void Engine::attachLossModel(const std::string& device, loss::DeviceLossModel model) {
  const Device& d = circuit_.findDevice(device);  // throws if unknown
  if (d.type != DeviceType::Switch && d.type != DeviceType::Diode) {
    throw std::runtime_error("attachLossModel needs a switch or diode: " + device);
  }
  if (d.type == DeviceType::Diode && model.hasSwitching()) {
    throw std::runtime_error("attachLossModel: diodes use recovery (QRR), not EON/EOFF tables");
  }
  lossModels_.insert_or_assign(device, std::move(model));
}

bool Engine::hasLossModel(const std::string& device) const {
  return lossModels_.find(device) != lossModels_.end();
}

double Engine::deviceTemp(const std::string& device) const {
  auto ov = tjOverrides_.find(device);
  if (ov != tjOverrides_.end()) return ov->second;
  auto it = thermals_.find(device);
  return it == thermals_.end() ? 25.0 : it->second.tj();
}

void Engine::setJunctionTempOverride(const std::string& device, double tj) {
  circuit_.findDevice(device);  // throws if unknown
  if (!std::isfinite(tj)) throw std::runtime_error("override Tj must be finite");
  tjOverrides_.insert_or_assign(device, tj);
}

void Engine::clearJunctionTempOverride(const std::string& device) {
  tjOverrides_.erase(device);
}

void Engine::applyLossModels() {
  if (!hasNetlist()) {
    throw std::runtime_error("applyLossModels needs a loaded netlist with .etable");
  }
  for (const auto& d : circuit_.devices()) {
    if (d.type != DeviceType::Switch && d.type != DeviceType::Diode) continue;
    if (d.eonTable.empty() && d.eoffTable.empty() && d.ronTable.empty() &&
        d.vfTable.empty()) {
      continue;
    }
    auto resolve = [&](const std::string& ref) -> loss::Table {
      if (ref.empty()) return loss::Table{};
      auto it = net_.tables.find(ref);
      if (it == net_.tables.end()) {
        throw std::runtime_error("applyLossModels: unknown .etable '" + ref + "'");
      }
      return it->second;
    };
    if (d.type == DeviceType::Diode && (!d.eonTable.empty() || !d.eoffTable.empty())) {
      throw std::runtime_error("applyLossModels: diodes use recovery (QRR), not EON/EOFF tables");
    }
    loss::DeviceLossModel model;
    model.eon = resolve(d.eonTable);
    model.eoff = resolve(d.eoffTable);
    model.ronTj = resolve(d.ronTable);
    model.vfTj = resolve(d.vfTable);
    lossModels_.insert_or_assign(d.name, std::move(model));
  }
}

void Engine::attachThermal(const std::string& device, thermal::ThermalNetwork net) {
  const Device& d = circuit_.findDevice(device);  // throws if unknown
  if (d.type != DeviceType::Switch && d.type != DeviceType::Diode) {
    throw std::runtime_error("attachThermal needs a switch or diode: " + device);
  }
  thermals_.insert_or_assign(device, std::move(net));
}

void Engine::applyThermalSpecs() {
  if (!hasNetlist() || net_.thermals.empty()) {
    throw std::runtime_error("applyThermalSpecs needs a loaded netlist with .thermal");
  }
  for (const auto& spec : net_.thermals) {
    std::vector<thermal::Stage> stages;
    for (std::size_t i = 0; i < spec.r.size(); ++i) {
      stages.push_back({spec.r[i], spec.c[i]});
    }
    attachThermal(spec.device, spec.foster ? thermal::ThermalNetwork::foster(stages, spec.tamb)
                                           : thermal::ThermalNetwork::cauer(stages, spec.tamb));
  }
}

bool Engine::hasThermal(const std::string& device) const {
  return thermals_.find(device) != thermals_.end();
}

double Engine::junctionTemp(const std::string& device) const {
  auto it = thermals_.find(device);
  if (it == thermals_.end()) throw std::runtime_error("no thermal attached: " + device);
  return it->second.tj();
}

thermal::DeviceLoss Engine::deviceLoss(const std::string& device) const {
  const Device& d = circuit_.findDevice(device);
  if (d.type != DeviceType::Switch && d.type != DeviceType::Diode) {
    throw std::runtime_error("deviceLoss needs a switch or diode: " + device);
  }
  auto it = losses_.find(device);
  if (it == losses_.end()) return {};
  return it->second;
}

}  // namespace power_engine
