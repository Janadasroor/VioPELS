// VioPELS FMI 3.0 Co-Simulation wrapper (roadmap item 18).
//
// Compiled once per FMU with -DFMI3_FUNCTION_PREFIX=<ModelIdentifier>
// (see fmi_pack.py --fmi-version 3): the TU is model-agnostic — netlist,
// I/O map and instantiation token come from resources/ at instantiate
// time. Mirrors fmi_wrapper.cpp (FMI 2.0) function for function.
//
// Contract (also in ARCHITECTURE):
// - Inputs (VR 0..I-1, file order): voltage/current SOURCE devices; Set
//   writes .value (non-V/I targets and non-finite values are rejected).
//   Gate drives are NOT exposed (step-boundary control stays inside;
//   netlist .control pwm is the self-switching path).
// - Outputs (VR I..): node probes ("v:N"); read any time after init.
// - Horizons are master-driven: .tran tstop is cleared at init exit
//   (PWM edges are still expanded from it first). startTime must be 0.
// - No event mode (eventModeUsed rejected at instantiate), no rollback
//   (Get/SetFMUstate: fmi3Error), steps are synchronous (earlyReturn
//   never set; requiredIntermediateVariables rejected).
// - Throws inside the engine surface as fmi3Error, never propagate.

#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "fmi/fmi3Functions.h"
#include "power_engine/engine.h"

namespace {

struct Instance {
  std::string name;
  std::unique_ptr<power_engine::Engine> eng;
  std::string netlistText;
  std::vector<std::string> inDevices;
  std::vector<std::string> outProbes;
  std::string token;
  enum class State { Instantiated, InitMode, Stepped, Terminated } state =
      State::Instantiated;
  double time = 0.0;
  fmi3LogMessageCallback logger = nullptr;
  fmi3InstanceEnvironment logEnv = nullptr;
  bool logging = false;
};

void logMsg(Instance* m, fmi3Status s, const char* text) {
  if (m && m->logging && m->logger) {
    char buf[1024];
    std::snprintf(buf, sizeof(buf), "%s", text);
    m->logger(m->logEnv, s, "logAll", buf);
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

void stripRight(std::string& s) {
  while (!s.empty() && (s.back() == '\n' || s.back() == '\r' || s.back() == ' ')) {
    s.pop_back();
  }
}

std::string resourceDir(fmi3String uri) {
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
    logMsg(m, fmi3Error, e.what());
    return false;
  }
  return true;
}

fmi3Status getFloat64Impl(Instance* m, const fmi3ValueReference vr[], size_t nvr,
                          fmi3Float64 value[], size_t nValues) {
  if (nValues != nvr) return fmi3Error;
  try {
    for (size_t k = 0; k < nvr; ++k) {
      const unsigned i = vr[k];
      if (i < m->inDevices.size()) {
        value[k] = m->eng->circuit().findDevice(m->inDevices[i]).value;
      } else if (i - m->inDevices.size() < m->outProbes.size()) {
        value[k] = m->eng->currentSolution().probes.at(
            m->outProbes[i - m->inDevices.size()]);
      } else {
        return fmi3Error;
      }
    }
  } catch (const std::exception& e) {
    logMsg(m, fmi3Error, e.what());
    return fmi3Error;
  }
  return fmi3OK;
}

}  // namespace

const char* fmi3GetVersion(void) { return fmi3Version; }

fmi3Status fmi3SetDebugLogging(fmi3Instance instance, fmi3Boolean loggingOn, size_t,
                               const fmi3String[]) {
  if (!instance) return fmi3Error;
  static_cast<Instance*>(instance)->logging = loggingOn;
  return fmi3OK;
}

fmi3Instance fmi3InstantiateCoSimulation(
    fmi3String instanceName, fmi3String instantiationToken, fmi3String resourcePath,
    fmi3Boolean visible, fmi3Boolean loggingOn, fmi3Boolean eventModeUsed,
    fmi3Boolean /*earlyReturnAllowed*/, const fmi3ValueReference[],
    size_t nRequiredIntermediateVariables, fmi3InstanceEnvironment instanceEnvironment,
    fmi3LogMessageCallback logMessage, fmi3IntermediateUpdateCallback) {
  (void)visible;
  if (eventModeUsed) return nullptr;  // CS without event mode only
  if (nRequiredIntermediateVariables > 0) return nullptr;  // no intermediate update
  const std::string dir = resourceDir(resourcePath);
  if (dir.empty()) return nullptr;
  auto* m = new (std::nothrow) Instance();
  if (!m) return nullptr;
  m->name = instanceName ? instanceName : "";
  m->logEnv = instanceEnvironment;
  m->logger = logMessage;
  m->logging = loggingOn;
  std::string tokenFile;
  if (!readFile(dir + "modelGuid.txt", tokenFile)) {
    delete m;
    return nullptr;
  }
  stripRight(tokenFile);
  if (tokenFile != (instantiationToken ? instantiationToken : "")) {
    delete m;
    return nullptr;  // standard instantiation-token check
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

void fmi3FreeInstance(fmi3Instance instance) { delete static_cast<Instance*>(instance); }

fmi3Status fmi3EnterInitializationMode(fmi3Instance instance, fmi3Boolean, fmi3Float64,
                                       fmi3Float64 startTime, fmi3Boolean, fmi3Float64) {
  auto* m = static_cast<Instance*>(instance);
  if (!m || m->state != Instance::State::Instantiated) return fmi3Error;
  if (startTime != 0.0) return fmi3Error;  // engine clock starts at 0
  m->state = Instance::State::InitMode;
  return fmi3OK;
}

fmi3Status fmi3ExitInitializationMode(fmi3Instance instance) {
  auto* m = static_cast<Instance*>(instance);
  if (!m || m->state != Instance::State::InitMode) return fmi3Error;
  try {
    m->eng->start();
  } catch (const std::exception& e) {
    logMsg(m, fmi3Error, e.what());
    return fmi3Error;
  }
  m->time = 0.0;
  m->state = Instance::State::Stepped;
  return fmi3OK;
}

fmi3Status fmi3EnterEventMode(fmi3Instance) { return fmi3Error; }  // CS only

fmi3Status fmi3Terminate(fmi3Instance instance) {
  auto* m = static_cast<Instance*>(instance);
  if (!m || m->state == Instance::State::Instantiated) return fmi3Error;
  m->state = Instance::State::Terminated;
  return fmi3OK;
}

fmi3Status fmi3Reset(fmi3Instance instance) {
  auto* m = static_cast<Instance*>(instance);
  if (!m) return fmi3Error;
  if (!buildEngine(m)) return fmi3Error;
  m->time = 0.0;
  m->state = Instance::State::Instantiated;
  return fmi3OK;
}

fmi3Status fmi3GetFloat64(fmi3Instance instance, const fmi3ValueReference vr[], size_t nvr,
                          fmi3Float64 value[], size_t nValues) {
  auto* m = static_cast<Instance*>(instance);
  if (!m || m->state == Instance::State::Instantiated ||
      m->state == Instance::State::Terminated)
    return fmi3Error;
  return getFloat64Impl(m, vr, nvr, value, nValues);
}

fmi3Status fmi3SetFloat64(fmi3Instance instance, const fmi3ValueReference vr[], size_t nvr,
                          const fmi3Float64 value[], size_t nValues) {
  auto* m = static_cast<Instance*>(instance);
  if (!m || m->state == Instance::State::Terminated) return fmi3Error;
  if (nValues != nvr) return fmi3Error;
  try {
    for (size_t k = 0; k < nvr; ++k) {
      const unsigned i = vr[k];
      if (i >= m->inDevices.size()) return fmi3Error;  // outputs are read-only
      if (!std::isfinite(value[k])) return fmi3Error;
      if (!isSource(*m->eng, m->inDevices[i])) return fmi3Error;
      m->eng->circuit().findDevice(m->inDevices[i]).value = value[k];
    }
  } catch (const std::exception& e) {
    logMsg(m, fmi3Error, e.what());
    return fmi3Error;
  }
  return fmi3OK;
}

fmi3Status fmi3DoStep(fmi3Instance instance, fmi3Float64 tComm, fmi3Float64 stepSize,
                      fmi3Boolean, fmi3Boolean* eventHandlingNeeded,
                      fmi3Boolean* terminateSimulation, fmi3Boolean* earlyReturn,
                      fmi3Float64* lastSuccessfulTime) {
  auto* m = static_cast<Instance*>(instance);
  if (!m || m->state != Instance::State::Stepped) return fmi3Error;
  if (!eventHandlingNeeded || !terminateSimulation || !earlyReturn || !lastSuccessfulTime)
    return fmi3Error;
  if (!(stepSize > 0.0) || !std::isfinite(stepSize) || !std::isfinite(tComm)) return fmi3Error;
  if (std::abs(tComm - m->time) > 1e-12 * (1.0 + std::abs(tComm))) return fmi3Error;
  try {
    m->eng->runUntil(m->time + stepSize);
  } catch (const std::exception& e) {
    logMsg(m, fmi3Error, e.what());
    return fmi3Error;
  }
  m->time += stepSize;
  *eventHandlingNeeded = false;
  *terminateSimulation = false;
  *earlyReturn = false;
  *lastSuccessfulTime = m->time;
  return fmi3OK;
}

// All other types: this FMU is Float64-only (matches modelDescription).
fmi3Status fmi3GetFloat32(fmi3Instance, const fmi3ValueReference[], size_t, fmi3Float32[],
                          size_t) {
  return fmi3Error;
}
fmi3Status fmi3GetInt8(fmi3Instance, const fmi3ValueReference[], size_t, fmi3Int8[], size_t) {
  return fmi3Error;
}
fmi3Status fmi3GetUInt8(fmi3Instance, const fmi3ValueReference[], size_t, fmi3UInt8[],
                        size_t) {
  return fmi3Error;
}
fmi3Status fmi3GetInt16(fmi3Instance, const fmi3ValueReference[], size_t, fmi3Int16[],
                        size_t) {
  return fmi3Error;
}
fmi3Status fmi3GetUInt16(fmi3Instance, const fmi3ValueReference[], size_t, fmi3UInt16[],
                         size_t) {
  return fmi3Error;
}
fmi3Status fmi3GetInt32(fmi3Instance, const fmi3ValueReference[], size_t, fmi3Int32[],
                        size_t) {
  return fmi3Error;
}
fmi3Status fmi3GetUInt32(fmi3Instance, const fmi3ValueReference[], size_t, fmi3UInt32[],
                         size_t) {
  return fmi3Error;
}
fmi3Status fmi3GetInt64(fmi3Instance, const fmi3ValueReference[], size_t, fmi3Int64[],
                        size_t) {
  return fmi3Error;
}
fmi3Status fmi3GetUInt64(fmi3Instance, const fmi3ValueReference[], size_t, fmi3UInt64[],
                         size_t) {
  return fmi3Error;
}
fmi3Status fmi3GetBoolean(fmi3Instance, const fmi3ValueReference[], size_t, fmi3Boolean[],
                          size_t) {
  return fmi3Error;
}
fmi3Status fmi3GetString(fmi3Instance, const fmi3ValueReference[], size_t, fmi3String[],
                         size_t) {
  return fmi3Error;
}
fmi3Status fmi3GetBinary(fmi3Instance, const fmi3ValueReference[], size_t, size_t[],
                         fmi3Binary[], size_t[]) {
  return fmi3Error;
}
fmi3Status fmi3GetClock(fmi3Instance, const fmi3ValueReference[], size_t, fmi3Clock[]) {
  return fmi3Error;
}
fmi3Status fmi3SetFloat32(fmi3Instance, const fmi3ValueReference[], size_t,
                          const fmi3Float32[], size_t) {
  return fmi3Error;
}
fmi3Status fmi3SetInt8(fmi3Instance, const fmi3ValueReference[], size_t, const fmi3Int8[],
                       size_t) {
  return fmi3Error;
}
fmi3Status fmi3SetUInt8(fmi3Instance, const fmi3ValueReference[], size_t, const fmi3UInt8[],
                        size_t) {
  return fmi3Error;
}
fmi3Status fmi3SetInt16(fmi3Instance, const fmi3ValueReference[], size_t, const fmi3Int16[],
                        size_t) {
  return fmi3Error;
}
fmi3Status fmi3SetUInt16(fmi3Instance, const fmi3ValueReference[], size_t, const fmi3UInt16[],
                         size_t) {
  return fmi3Error;
}
fmi3Status fmi3SetInt32(fmi3Instance, const fmi3ValueReference[], size_t, const fmi3Int32[],
                        size_t) {
  return fmi3Error;
}
fmi3Status fmi3SetUInt32(fmi3Instance, const fmi3ValueReference[], size_t, const fmi3UInt32[],
                         size_t) {
  return fmi3Error;
}
fmi3Status fmi3SetInt64(fmi3Instance, const fmi3ValueReference[], size_t, const fmi3Int64[],
                        size_t) {
  return fmi3Error;
}
fmi3Status fmi3SetUInt64(fmi3Instance, const fmi3ValueReference[], size_t, const fmi3UInt64[],
                         size_t) {
  return fmi3Error;
}
fmi3Status fmi3SetBoolean(fmi3Instance, const fmi3ValueReference[], size_t,
                          const fmi3Boolean[], size_t) {
  return fmi3Error;
}
fmi3Status fmi3SetString(fmi3Instance, const fmi3ValueReference[], size_t, const fmi3String[],
                         size_t) {
  return fmi3Error;
}
fmi3Status fmi3SetBinary(fmi3Instance, const fmi3ValueReference[], size_t, const size_t[],
                         const fmi3Binary[], size_t[]) {
  return fmi3Error;
}
fmi3Status fmi3SetClock(fmi3Instance, const fmi3ValueReference[], size_t, const fmi3Clock[]) {
  return fmi3Error;
}
fmi3Status fmi3GetNumberOfVariableDependencies(fmi3Instance, fmi3ValueReference, size_t*) {
  return fmi3Error;
}
fmi3Status fmi3GetVariableDependencies(fmi3Instance, fmi3ValueReference, size_t[], size_t[],
                                       size_t[], size_t) {
  return fmi3Error;
}
fmi3Status fmi3GetFMUState(fmi3Instance, fmi3FMUState*) { return fmi3Error; }
fmi3Status fmi3SetFMUState(fmi3Instance, fmi3FMUState) { return fmi3Error; }
fmi3Status fmi3FreeFMUState(fmi3Instance, fmi3FMUState*) { return fmi3Error; }
fmi3Status fmi3SerializedFMUStateSize(fmi3Instance, fmi3FMUState, size_t*) {
  return fmi3Error;
}
fmi3Status fmi3SerializeFMUState(fmi3Instance, fmi3FMUState, fmi3Byte[], size_t) {
  return fmi3Error;
}
fmi3Status fmi3DeserializeFMUState(fmi3Instance, const fmi3Byte[], size_t, fmi3FMUState*) {
  return fmi3Error;
}
fmi3Status fmi3GetDirectionalDerivative(fmi3Instance, const fmi3ValueReference[], size_t,
                                        const fmi3ValueReference[], size_t, const fmi3Float64[],
                                        size_t, fmi3Float64[], size_t) {
  return fmi3Error;
}
fmi3Status fmi3GetAdjointDerivative(fmi3Instance, const fmi3ValueReference[], size_t,
                                    const fmi3ValueReference[], size_t, const fmi3Float64[],
                                    size_t, fmi3Float64[], size_t) {
  return fmi3Error;
}
fmi3Status fmi3EnterConfigurationMode(fmi3Instance) { return fmi3Error; }
fmi3Status fmi3ExitConfigurationMode(fmi3Instance) { return fmi3Error; }
fmi3Status fmi3GetIntervalDecimal(fmi3Instance, const fmi3ValueReference[], size_t,
                                  fmi3Float64[], fmi3IntervalQualifier[]) {
  return fmi3Error;
}
fmi3Status fmi3GetIntervalFraction(fmi3Instance, const fmi3ValueReference[], size_t,
                                   fmi3UInt64[], fmi3UInt64[], fmi3IntervalQualifier[]) {
  return fmi3Error;
}
fmi3Status fmi3GetShiftDecimal(fmi3Instance, const fmi3ValueReference[], size_t, fmi3Float64[]) {
  return fmi3Error;
}
fmi3Status fmi3GetShiftFraction(fmi3Instance, const fmi3ValueReference[], size_t,
                                fmi3UInt64[], fmi3UInt64[]) {
  return fmi3Error;
}
fmi3Status fmi3SetIntervalDecimal(fmi3Instance, const fmi3ValueReference[], size_t,
                                  const fmi3Float64[]) {
  return fmi3Error;
}
fmi3Status fmi3SetIntervalFraction(fmi3Instance, const fmi3ValueReference[], size_t,
                                   const fmi3UInt64[], const fmi3UInt64[]) {
  return fmi3Error;
}
fmi3Status fmi3SetShiftDecimal(fmi3Instance, const fmi3ValueReference[], size_t,
                               const fmi3Float64[]) {
  return fmi3Error;
}
fmi3Status fmi3SetShiftFraction(fmi3Instance, const fmi3ValueReference[], size_t,
                                const fmi3UInt64[], const fmi3UInt64[]) {
  return fmi3Error;
}
fmi3Status fmi3EvaluateDiscreteStates(fmi3Instance) { return fmi3Error; }
fmi3Status fmi3UpdateDiscreteStates(fmi3Instance, fmi3Boolean*, fmi3Boolean*, fmi3Boolean*,
                                    fmi3Boolean*, fmi3Float64*) {
  return fmi3Error;
}
