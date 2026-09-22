// VioPELS FMI 2.0 Co-Simulation wrapper (roadmap item 18).
//
// Compiled once per FMU with -DFMI2_FUNCTION_PREFIX=<ModelIdentifier>
// (see fmi_pack.py): the TU is model-agnostic — netlist, I/O map and GUID
// come from resources/ at instantiate time.
//
// Contract (also in ARCHITECTURE):
// - Inputs (VR 0..I-1, file order): voltage/current SOURCE devices; Set
//   writes .value (non-V/I targets and non-finite values are rejected).
//   Gate drives are NOT exposed (step-boundary control stays inside;
//   netlist .control pwm is the self-switching path).
// - Outputs (VR I..): node probes ("v:N"); read any time after init.
// - Horizons are master-driven: .tran tstop is cleared at init exit
//   (PWM edges are still expanded from it first). startTime must be 0.
// - No rollback (Get/SetFMUstate, derivatives, input derivatives:
//   fmi2Error). Steps are synchronous (CancelStep is a no-op OK).
// - Throws inside the engine surface as fmi2Error, never propagate.

#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "fmi/fmi2Functions.h"
#include "power_engine/engine.h"

namespace {

struct Instance {
  std::string name;
  std::unique_ptr<power_engine::Engine> eng;
  std::string netlistText;
  std::vector<std::string> inDevices;
  std::vector<std::string> outProbes;
  std::string guid;
  enum class State { Instantiated, InitMode, Stepped, Terminated } state =
      State::Instantiated;
  double time = 0.0;
  fmi2CallbackFunctions cb{};
  bool hasCb = false;
  bool logging = false;
};

void logMsg(Instance* m, fmi2Status s, const char* text) {
  if (m && m->hasCb && m->logging && m->cb.logger) {
    char buf[1024];
    std::snprintf(buf, sizeof(buf), "%s", text);
    m->cb.logger(m->cb.componentEnvironment, m->name.c_str(), s, "logAll", buf);
  }
}

bool readFile(const std::string& path, std::string& out) {
  std::ifstream f(path, std::ios::binary);
  if (!f) return false;
  std::ostringstream ss;
  ss << f.rdbuf();
  out = ss.str();
  return true;
}

std::string resourceDir(fmi2String uri) {
  // "file:///abs/path[/]" -> "/abs/path". Relative URIs rejected.
  std::string u = uri ? uri : "";
  const std::string pre = "file://";
  if (u.compare(0, pre.size(), pre) != 0) return {};
  u = u.substr(pre.size());
  if (!u.empty() && u.back() != '/') u += '/';
  return u;
}

bool isSource(power_engine::Engine& eng, const std::string& name) {
  try {
    const auto t = eng.circuit().findDevice(name).type;
    return t == power_engine::DeviceType::VoltageSource ||
           t == power_engine::DeviceType::CurrentSource;
  } catch (...) {
    return false;
  }
}

bool buildEngine(Instance* m) {
  try {
    m->eng = std::make_unique<power_engine::Engine>();
    m->eng->loadNetlist(m->netlistText);
    m->eng->applyPwmSpecs();
    m->eng->clearStopTime();  // horizons are master-driven
  } catch (const std::exception& e) {
    logMsg(m, fmi2Error, e.what());
    return false;
  }
  return true;
}

}  // namespace

const char* fmi2GetTypesPlatform(void) { return fmi2TypesPlatform; }
const char* fmi2GetVersion(void) { return fmi2Version; }

fmi2Status fmi2SetDebugLogging(fmi2Component c, fmi2Boolean loggingOn, size_t,
                               const fmi2String[]) {
  if (!c) return fmi2Error;
  static_cast<Instance*>(c)->logging = loggingOn != 0;
  return fmi2OK;
}

fmi2Component fmi2Instantiate(fmi2String instanceName, fmi2Type fmuType, fmi2String fmuGUID,
                              fmi2String fmuResourceLocation,
                              const fmi2CallbackFunctions* functions, fmi2Boolean visible,
                              fmi2Boolean loggingOn) {
  (void)visible;
  if (fmuType != fmi2CoSimulation) return nullptr;
  const std::string dir = resourceDir(fmuResourceLocation);
  if (dir.empty()) return nullptr;
  auto* m = new (std::nothrow) Instance();
  if (!m) return nullptr;
  m->name = instanceName ? instanceName : "";
  if (functions) {
    m->cb = *functions;
    m->hasCb = true;
  }
  m->logging = loggingOn != 0;
  std::string guidFile;
  if (!readFile(dir + "modelGuid.txt", guidFile)) {
    delete m;
    return nullptr;
  }
  while (!guidFile.empty() &&
         (guidFile.back() == '\n' || guidFile.back() == '\r' || guidFile.back() == ' ')) {
    guidFile.pop_back();
  }
  if (guidFile != (fmuGUID ? fmuGUID : "")) {
    delete m;
    return nullptr;  // standard GUID check
  }
  if (!readFile(dir + "model.netlist", m->netlistText)) {
    delete m;
    return nullptr;
  }
  std::string io;
  if (!readFile(dir + "io.txt", io)) {
    delete m;
    return nullptr;
  }
  std::istringstream lines(io);
  std::string kind, what;
  while (lines >> kind >> what) {
    if (kind == "input")
      m->inDevices.push_back(what);
    else if (kind == "output")
      m->outProbes.push_back(what);
    else {
      delete m;
      return nullptr;
    }
  }
  if (!buildEngine(m)) {
    delete m;
    return nullptr;
  }
  return m;
}

void fmi2FreeInstance(fmi2Component c) { delete static_cast<Instance*>(c); }

fmi2Status fmi2SetupExperiment(fmi2Component c, fmi2Boolean, fmi2Real, fmi2Real startTime,
                               fmi2Boolean, fmi2Real) {
  auto* m = static_cast<Instance*>(c);
  if (!m || m->state != Instance::State::Instantiated) return fmi2Error;
  if (startTime != 0.0) return fmi2Error;  // engine clock starts at 0
  return fmi2OK;
}

fmi2Status fmi2EnterInitializationMode(fmi2Component c) {
  auto* m = static_cast<Instance*>(c);
  if (!m || m->state != Instance::State::Instantiated) return fmi2Error;
  m->state = Instance::State::InitMode;
  return fmi2OK;
}

fmi2Status fmi2ExitInitializationMode(fmi2Component c) {
  auto* m = static_cast<Instance*>(c);
  if (!m || m->state != Instance::State::InitMode) return fmi2Error;
  try {
    m->eng->start();
  } catch (const std::exception& e) {
    logMsg(m, fmi2Error, e.what());
    return fmi2Error;
  }
  m->time = 0.0;
  m->state = Instance::State::Stepped;
  return fmi2OK;
}

fmi2Status fmi2Terminate(fmi2Component c) {
  auto* m = static_cast<Instance*>(c);
  if (!m || m->state == Instance::State::Instantiated) return fmi2Error;
  m->state = Instance::State::Terminated;
  return fmi2OK;
}

fmi2Status fmi2Reset(fmi2Component c) {
  auto* m = static_cast<Instance*>(c);
  if (!m) return fmi2Error;
  if (!buildEngine(m)) return fmi2Error;
  m->time = 0.0;
  m->state = Instance::State::Instantiated;
  return fmi2OK;
}

fmi2Status fmi2GetReal(fmi2Component c, const fmi2ValueReference vr[], size_t nvr,
                       fmi2Real value[]) {
  auto* m = static_cast<Instance*>(c);
  if (!m || m->state == Instance::State::Instantiated ||
      m->state == Instance::State::Terminated)
    return fmi2Error;
  try {
    for (size_t k = 0; k < nvr; ++k) {
      const unsigned i = vr[k];
      if (i < m->inDevices.size()) {
        value[k] = m->eng->circuit().findDevice(m->inDevices[i]).value;
      } else if (i - m->inDevices.size() < m->outProbes.size()) {
        value[k] = m->eng->currentSolution().probes.at(
            m->outProbes[i - m->inDevices.size()]);
      } else {
        return fmi2Error;
      }
    }
  } catch (const std::exception& e) {
    logMsg(m, fmi2Error, e.what());
    return fmi2Error;
  }
  return fmi2OK;
}

fmi2Status fmi2SetReal(fmi2Component c, const fmi2ValueReference vr[], size_t nvr,
                       const fmi2Real value[]) {
  auto* m = static_cast<Instance*>(c);
  if (!m || m->state == Instance::State::Terminated) return fmi2Error;
  try {
    for (size_t k = 0; k < nvr; ++k) {
      const unsigned i = vr[k];
      if (i >= m->inDevices.size()) return fmi2Error;  // outputs are read-only
      if (!std::isfinite(value[k])) return fmi2Error;
      if (!isSource(*m->eng, m->inDevices[i])) return fmi2Error;
      m->eng->circuit().findDevice(m->inDevices[i]).value = value[k];
    }
  } catch (const std::exception& e) {
    logMsg(m, fmi2Error, e.what());
    return fmi2Error;
  }
  return fmi2OK;
}

fmi2Status fmi2GetInteger(fmi2Component, const fmi2ValueReference[], size_t, fmi2Integer[]) {
  return fmi2Error;  // all-Reals FMU (matches modelDescription)
}
fmi2Status fmi2GetBoolean(fmi2Component, const fmi2ValueReference[], size_t, fmi2Boolean[]) {
  return fmi2Error;
}
fmi2Status fmi2GetString(fmi2Component, const fmi2ValueReference[], size_t, fmi2String[]) {
  return fmi2Error;
}
fmi2Status fmi2SetInteger(fmi2Component, const fmi2ValueReference[], size_t,
                          const fmi2Integer[]) {
  return fmi2Error;
}
fmi2Status fmi2SetBoolean(fmi2Component, const fmi2ValueReference[], size_t,
                          const fmi2Boolean[]) {
  return fmi2Error;
}
fmi2Status fmi2SetString(fmi2Component, const fmi2ValueReference[], size_t,
                         const fmi2String[]) {
  return fmi2Error;
}

fmi2Status fmi2GetFMUstate(fmi2Component, fmi2FMUstate*) { return fmi2Error; }
fmi2Status fmi2SetFMUstate(fmi2Component, fmi2FMUstate) { return fmi2Error; }
fmi2Status fmi2FreeFMUstate(fmi2Component, fmi2FMUstate*) { return fmi2Error; }
fmi2Status fmi2SerializedFMUstateSize(fmi2Component, fmi2FMUstate, size_t*) {
  return fmi2Error;
}
fmi2Status fmi2SerializeFMUstate(fmi2Component, fmi2FMUstate, fmi2Byte[], size_t) {
  return fmi2Error;
}
fmi2Status fmi2DeSerializeFMUstate(fmi2Component, const fmi2Byte[], size_t, fmi2FMUstate*) {
  return fmi2Error;
}
fmi2Status fmi2GetDirectionalDerivative(fmi2Component, const fmi2ValueReference[], size_t,
                                        const fmi2ValueReference[], size_t, const fmi2Real[],
                                        fmi2Real[]) {
  return fmi2Error;
}

fmi2Status fmi2SetRealInputDerivatives(fmi2Component, const fmi2ValueReference[], size_t,
                                       const fmi2Integer[], const fmi2Real[]) {
  return fmi2Error;
}
fmi2Status fmi2GetRealOutputDerivatives(fmi2Component, const fmi2ValueReference[], size_t,
                                        const fmi2Integer[], fmi2Real[]) {
  return fmi2Error;
}

fmi2Status fmi2DoStep(fmi2Component c, fmi2Real tComm, fmi2Real stepSize,
                      fmi2Boolean) {
  auto* m = static_cast<Instance*>(c);
  if (!m || m->state != Instance::State::Stepped) return fmi2Error;
  if (!(stepSize > 0.0) || !std::isfinite(stepSize) || !std::isfinite(tComm)) return fmi2Error;
  if (std::abs(tComm - m->time) > 1e-12 * (1.0 + std::abs(tComm))) return fmi2Error;
  try {
    m->eng->runUntil(m->time + stepSize);
  } catch (const std::exception& e) {
    logMsg(m, fmi2Error, e.what());
    return fmi2Error;
  }
  m->time += stepSize;
  return fmi2OK;
}

fmi2Status fmi2CancelStep(fmi2Component c) {
  if (!c) return fmi2Error;
  return fmi2OK;  // synchronous steps: nothing pending
}

fmi2Status fmi2GetStatus(fmi2Component c, const fmi2StatusKind s, fmi2Status* value) {
  auto* m = static_cast<Instance*>(c);
  if (!m || !value) return fmi2Error;
  switch (s) {
    case fmi2DoStepStatus:
    case fmi2PendingStatus:
      *value = fmi2OK;
      return fmi2OK;
    case fmi2Terminated:
      *value = (m->state == Instance::State::Terminated) ? fmi2OK : fmi2Error;
      return fmi2OK;
    default:
      return fmi2Error;
  }
}

fmi2Status fmi2GetRealStatus(fmi2Component c, const fmi2StatusKind s, fmi2Real* value) {
  auto* m = static_cast<Instance*>(c);
  if (!m || !value) return fmi2Error;
  if (s != fmi2LastSuccessfulTime) return fmi2Error;
  *value = m->time;
  return fmi2OK;
}

fmi2Status fmi2GetIntegerStatus(fmi2Component, const fmi2StatusKind, fmi2Integer*) {
  return fmi2Error;
}
fmi2Status fmi2GetBooleanStatus(fmi2Component, const fmi2StatusKind, fmi2Boolean*) {
  return fmi2Error;
}
fmi2Status fmi2GetStringStatus(fmi2Component, const fmi2StatusKind, fmi2String*) {
  return fmi2Error;
}
