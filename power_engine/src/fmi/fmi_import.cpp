// FMI 2.0 Co-Simulation import: hostile-ABI slave + engine coupling.
// Private fmi headers stay in this TU (pimpl in fmi_import.h).
#include "power_engine/fmi_import.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <sstream>

#include "power_engine/engine.h"

#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif

// MSVC trips on third-party macros the same way Eigen did; the vendored
// FMI headers are ours to guard at the include site.
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4200 4201 4204 4221)
#endif
#include "fmi/fmi2FunctionTypes.h"
#include "fmi/fmi2TypesPlatform.h"
#ifdef _MSC_VER
#pragma warning(pop)
#endif

namespace power_engine {
namespace fmi_import {
namespace {

std::string statusName(int s) {
  switch (s) {
    case fmi2OK: return "fmi2OK";
    case fmi2Warning: return "fmi2Warning";
    case fmi2Discard: return "fmi2Discard";
    case fmi2Error: return "fmi2Error";
    case fmi2Fatal: return "fmi2Fatal";
    case fmi2Pending: return "fmi2Pending";
    default: return "fmi2Status(" + std::to_string(s) + ")";
  }
}

std::string readFile(const std::string& path) {
  FILE* f = std::fopen(path.c_str(), "rb");
  if (!f) throw std::runtime_error("fmi import: cannot open " + path);
  std::string out;
  char buf[1 << 12];
  std::size_t n = 0;
  while ((n = std::fread(buf, 1, sizeof buf, f)) > 0) out.append(buf, n);
  std::fclose(f);
  return out;
}

std::string decodeEntities(std::string s) {
  const char* from[] = {"&lt;", "&gt;", "&amp;", "&quot;", "&apos;"};
  const char* to[] = {"<", ">", "&", "\"", "'"};
  for (int k = 0; k < 5; ++k) {
    std::string::size_type p = 0;
    while ((p = s.find(from[k], p)) != std::string::npos) {
      s.replace(p, std::strlen(from[k]), to[k]);
      p += std::strlen(to[k]);
    }
  }
  return s;
}

// Minimal tag scanner over machine-generated modelDescription.xml:
// skips prolog/comments/declarations, yields (name, attrs, selfClose)
// for element tags and "" for the matching close tag position.
struct Tag {
  std::string name;
  std::map<std::string, std::string> attrs;
  bool selfClose = false;
  bool close = false;  // </name>
  std::size_t end = 0;  // offset just past '>'
};

Tag parseTag(const std::string& xml, std::size_t lt) {
  // xml[lt] == '<', not a prolog/comment/declaration.
  std::size_t gt = xml.find('>', lt + 1);
  if (gt == std::string::npos) throw std::runtime_error("fmi import: unterminated tag");
  Tag t;
  t.end = gt + 1;
  std::string inner = xml.substr(lt + 1, gt - lt - 1);
  std::size_t p = 0;
  if (!inner.empty() && inner[0] == '/') {
    t.close = true;
    p = 1;
  }
  while (p < inner.size() && inner[p] != ' ' && inner[p] != '\t' &&
         inner[p] != '\n' && inner[p] != '\r' && inner[p] != '/')
    t.name += inner[p++];
  if (!t.close) {
    // Trailing '/' (allow space before it) marks self-close.
    std::size_t e = inner.size();
    while (e > 0 && (inner[e - 1] == ' ' || inner[e - 1] == '\t' ||
                     inner[e - 1] == '\n' || inner[e - 1] == '\r'))
      --e;
    if (e > 0 && inner[e - 1] == '/') {
      t.selfClose = true;
      inner.erase(e - 1, 1);
    }
    while (p < inner.size()) {
      while (p < inner.size() && (inner[p] == ' ' || inner[p] == '\t' ||
                                  inner[p] == '\n' || inner[p] == '\r'))
        ++p;
      if (p >= inner.size()) break;
      std::size_t ks = p;
      while (p < inner.size() && inner[p] != '=' && inner[p] != ' ' &&
             inner[p] != '\t' && inner[p] != '\n' && inner[p] != '\r')
        ++p;
      std::string key = inner.substr(ks, p - ks);
      while (p < inner.size() && inner[p] != '=' && inner[p] != '"' && inner[p] != '\'')
        ++p;
      if (p >= inner.size() || inner[p] != '=') break;
      ++p;
      while (p < inner.size() && inner[p] == ' ') ++p;
      if (p >= inner.size() || (inner[p] != '"' && inner[p] != '\'')) break;
      char q = inner[p++];
      std::size_t vs = p;
      while (p < inner.size() && inner[p] != q) ++p;
      t.attrs[decodeEntities(key)] = decodeEntities(inner.substr(vs, p - vs));
      if (p < inner.size()) ++p;
    }
  }
  return t;
}

// Next element tag at/after `pos`; transparently skips prolog, comments,
// and declarations. Returns false at end of input.
bool nextTag(const std::string& xml, std::size_t& pos, Tag& out) {
  while (true) {
    std::size_t lt = xml.find('<', pos);
    if (lt == std::string::npos) return false;
    if (xml.compare(lt, 2, "<?") == 0) {
      std::size_t e = xml.find("?>", lt + 2);
      if (e == std::string::npos) throw std::runtime_error("fmi import: unterminated prolog");
      pos = e + 2;
      continue;
    }
    if (xml.compare(lt, 4, "<!--") == 0) {
      std::size_t e = xml.find("-->", lt + 4);
      if (e == std::string::npos) throw std::runtime_error("fmi import: unterminated comment");
      pos = e + 3;
      continue;
    }
    if (xml.compare(lt, 2, "<!") == 0) {
      std::size_t e = xml.find('>', lt + 2);
      if (e == std::string::npos) throw std::runtime_error("fmi import: unterminated declaration");
      pos = e + 1;
      continue;
    }
    out = parseTag(xml, lt);
    pos = out.end;
    return true;
  }
}

}  // namespace

const FmuVariable& ModelDescription::find(const std::string& name) const {
  for (const auto& v : variables)
    if (v.name == name) return v;
  throw std::runtime_error("fmi import: unknown variable '" + name + "'");
}

ModelDescription parseModelDescription(const std::string& fmuDir) {
  const std::string xml = readFile(fmuDir + "/modelDescription.xml");
  ModelDescription md;
  std::size_t pos = 0;
  Tag t;
  bool inVars = false;
  while (nextTag(xml, pos, t)) {
    if (t.close) {
      if (t.name == "ModelVariables") inVars = false;
      continue;
    }
    if (t.name == "fmiModelDescription") {
      const auto it = t.attrs.find("fmiVersion");
      if (it == t.attrs.end() || it->second != "2.0")
        throw std::runtime_error("fmi import: only FMI 2.0 supported (fmiVersion=" +
                                 (it == t.attrs.end() ? "<missing>" : it->second) + ")");
      md.modelName = t.attrs.count("modelName") ? t.attrs.at("modelName") : "";
      md.guid = t.attrs.count("guid") ? t.attrs.at("guid") : "";
    } else if (t.name == "CoSimulation") {
      if (md.modelIdentifier.empty()) {
        const auto it = t.attrs.find("modelIdentifier");
        if (it == t.attrs.end() || it->second.empty())
          throw std::runtime_error("fmi import: <CoSimulation> without modelIdentifier");
        md.modelIdentifier = it->second;
      }
    } else if (t.name == "ModelVariables") {
      inVars = true;
    } else if (inVars && t.name == "ScalarVariable") {
      FmuVariable v;
      const auto req = [&](const char* k) -> std::string {
        const auto it = t.attrs.find(k);
        if (it == t.attrs.end())
          throw std::runtime_error(std::string("fmi import: ScalarVariable without ") + k);
        return it->second;
      };
      v.name = req("name");
      const std::string vr = req("valueReference");
      try {
        v.valueReference = static_cast<unsigned int>(std::stoull(vr));
      } catch (...) {
        throw std::runtime_error("fmi import: bad valueReference '" + vr + "'");
      }
      v.causality = t.attrs.count("causality") ? t.attrs.at("causality") : "local";
      v.variability = t.attrs.count("variability") ? t.attrs.at("variability") : "continuous";
      if (!t.selfClose) {
        // Inner type element (<Real start=".."/> etc.), then </ScalarVariable>.
        bool typed = false;
        Tag u;
        while (nextTag(xml, pos, u)) {
          if (u.close) break;
          if (u.name == "Real" || u.name == "Integer" || u.name == "Boolean" ||
              u.name == "String") {
            if (typed)
              throw std::runtime_error("fmi import: '" + v.name + "' has two types");
            typed = true;
            v.type = u.name[0];
            const auto si = u.attrs.find("start");
            if (si != u.attrs.end()) {
              try {
                if (v.type == 'R')
                  v.realStart = std::stod(si->second);
                else if (v.type == 'I')
                  v.intStart = std::stoll(si->second);
                else if (v.type == 'B')
                  v.boolStart = (si->second == "true" || si->second == "1");
                else
                  v.stringStart = si->second;
              } catch (...) {
                throw std::runtime_error("fmi import: bad start for '" + v.name + "'");
              }
            }
          }
        }
        if (!typed)
          throw std::runtime_error("fmi import: '" + v.name + "' has no type element");
      } else {
        throw std::runtime_error("fmi import: '" + v.name + "' has no type element");
      }
      md.variables.push_back(std::move(v));
    }
  }
  if (md.modelIdentifier.empty())
    throw std::runtime_error("fmi import: no <CoSimulation> element (2.0 CS required)");
  return md;
}

std::string platformLibPath(const std::string& fmuDir,
                            const std::string& modelIdentifier) {
#ifdef _WIN32
  const std::string plat = "win64", ext = ".dll";
#elif defined(__APPLE__)
  const std::string plat = "darwin64", ext = ".dylib";
#else
  const std::string plat = "linux64", ext = ".so";
#endif
  std::string p = fmuDir + "/binaries/" + plat + "/" + modelIdentifier + ext;
  if (!std::filesystem::exists(p)) {
    std::string have;
    const std::string bd = fmuDir + "/binaries/" + plat;
    if (std::filesystem::exists(bd))
      for (const auto& e : std::filesystem::directory_iterator(bd))
        have += " " + e.path().filename().string();
    throw std::runtime_error("fmi import: missing " + p + " (have:" + have + ")");
  }
  return p;
}

struct FmuSlave::Fns {
  fmi2InstantiateTYPE* instantiate = nullptr;
  fmi2SetupExperimentTYPE* setupExperiment = nullptr;
  fmi2EnterInitializationModeTYPE* enterInit = nullptr;
  fmi2ExitInitializationModeTYPE* exitInit = nullptr;
  fmi2TerminateTYPE* terminate = nullptr;
  fmi2FreeInstanceTYPE* freeInstance = nullptr;
  fmi2SetRealTYPE* setReal = nullptr;
  fmi2SetIntegerTYPE* setInteger = nullptr;
  fmi2SetBooleanTYPE* setBoolean = nullptr;
  fmi2GetRealTYPE* getReal = nullptr;
  fmi2GetIntegerTYPE* getInteger = nullptr;
  fmi2GetBooleanTYPE* getBoolean = nullptr;
  fmi2DoStepTYPE* doStep = nullptr;
  fmi2SetDebugLoggingTYPE* setDebugLogging = nullptr;
};

namespace {
void* libOpen(const std::string& path) {
#ifdef _WIN32
  return static_cast<void*>(LoadLibraryA(path.c_str()));
#else
  return dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
#endif
}
void* libSym(void* h, const char* name) {
#ifdef _WIN32
  return reinterpret_cast<void*>(GetProcAddress(static_cast<HMODULE>(h), name));
#else
  return dlsym(h, name);
#endif
}
void libClose(void* h) {
#ifdef _WIN32
  FreeLibrary(static_cast<HMODULE>(h));
#else
  dlclose(h);
#endif
}
std::string libError() {
#ifdef _WIN32
  return "error " + std::to_string(GetLastError());
#else
  const char* e = dlerror();
  return e ? e : "unknown";
#endif
}

void fmuLogger(fmi2ComponentEnvironment env, fmi2String, fmi2Status,
               fmi2String, fmi2String message, ...) {
  auto* self = static_cast<FmuSlave*>(env);
  if (self) self->appendLog(message);
}
void* fmuAlloc(std::size_t nobj, std::size_t size) { return std::calloc(nobj, size); }
void fmuStepDone(fmi2ComponentEnvironment, fmi2Status) {}
}  // namespace

FmuSlave::FmuSlave(const std::string& fmuDir)
    : desc_(parseModelDescription(fmuDir)), dir_(fmuDir), fns_(new Fns) {
  const std::string lib = platformLibPath(fmuDir, desc_.modelIdentifier);
  handle_ = libOpen(lib);
  if (!handle_) throw std::runtime_error("fmi import: cannot load " + lib + ": " + libError());
  const char* names[] = {"fmi2Instantiate",
                         "fmi2SetupExperiment",
                         "fmi2EnterInitializationMode",
                         "fmi2ExitInitializationMode",
                         "fmi2Terminate",
                         "fmi2FreeInstance",
                         "fmi2SetReal",
                         "fmi2SetInteger",
                         "fmi2SetBoolean",
                         "fmi2GetReal",
                         "fmi2GetInteger",
                         "fmi2GetBoolean",
                         "fmi2DoStep",
                         "fmi2SetDebugLogging"};
  void** slots[] = {reinterpret_cast<void**>(&fns_->instantiate),
                    reinterpret_cast<void**>(&fns_->setupExperiment),
                    reinterpret_cast<void**>(&fns_->enterInit),
                    reinterpret_cast<void**>(&fns_->exitInit),
                    reinterpret_cast<void**>(&fns_->terminate),
                    reinterpret_cast<void**>(&fns_->freeInstance),
                    reinterpret_cast<void**>(&fns_->setReal),
                    reinterpret_cast<void**>(&fns_->setInteger),
                    reinterpret_cast<void**>(&fns_->setBoolean),
                    reinterpret_cast<void**>(&fns_->getReal),
                    reinterpret_cast<void**>(&fns_->getInteger),
                    reinterpret_cast<void**>(&fns_->getBoolean),
                    reinterpret_cast<void**>(&fns_->doStep),
                    reinterpret_cast<void**>(&fns_->setDebugLogging)};
  for (int i = 0; i < 14; ++i) {
    *slots[i] = libSym(handle_, names[i]);
    if (!*slots[i]) {
      libClose(handle_);
      handle_ = nullptr;
      throw std::runtime_error("fmi import: " + lib + " lacks " + names[i]);
    }
  }
  fmi2CallbackFunctions cb{};
  cb.logger = fmuLogger;
  cb.allocateMemory = fmuAlloc;
  cb.stepFinished = fmuStepDone;
  cb.componentEnvironment = this;
  const std::string res = "file:///" + dir_ + "/resources/";
  comp_ = fns_->instantiate("power_engine", fmi2CoSimulation, desc_.guid.c_str(),
                            res.c_str(), &cb, fmi2False, fmi2True);
  if (!comp_) {
    libClose(handle_);
    handle_ = nullptr;
    throw std::runtime_error("fmi import: fmi2Instantiate failed (" + logTail_ + ")");
  }
  state_ = State::Instantiated;
}

FmuSlave::~FmuSlave() {
  if (comp_ && fns_) {
    try {
      if (state_ == State::Init || state_ == State::Stepping) fns_->terminate(comp_);
    } catch (...) {
    }
    try {
      fns_->freeInstance(comp_);
    } catch (...) {
    }
  }
  if (handle_) libClose(handle_);
}

void FmuSlave::require(State s, const char* what) const {
  if (state_ != s)
    throw std::runtime_error(std::string("fmi import: ") + what + " out of order");
}

void FmuSlave::fail(const std::string& fn, int status) const {
  throw std::runtime_error("fmi import: " + fn + " -> " + statusName(status) +
                           " [" + desc_.modelName + "] " + logTail_);
}

void FmuSlave::enterInit() {
  require(State::Instantiated, "enterInit");
  fmi2Status s = fns_->setupExperiment(comp_, fmi2False, 0.0, 0.0, fmi2False, 0.0);
  if (s != fmi2OK) fail("fmi2SetupExperiment", s);
  s = fns_->enterInit(comp_);
  if (s != fmi2OK) fail("fmi2EnterInitializationMode", s);
  state_ = State::Init;
}

void FmuSlave::exitInit() {
  require(State::Init, "exitInit");
  // Apply Real starts (inputs the FMU needs before initialization ends).
  for (const auto& v : desc_.variables)
    if (v.type == 'R' && v.causality == "input") setReal(v.name, v.realStart);
  fmi2Status s = fns_->exitInit(comp_);
  if (s != fmi2OK) fail("fmi2ExitInitializationMode", s);
  state_ = State::Stepping;
}

void FmuSlave::doStep(double commStep) {
  require(State::Stepping, "doStep");
  if (!(commStep > 0.0)) throw std::runtime_error("fmi import: commStep must be > 0");
  fmi2Status s = fns_->doStep(comp_, t_, commStep, fmi2True);
  if (s == fmi2Discard)
    throw std::runtime_error("fmi import: fmi2DoStep discarded @" + std::to_string(t_) +
                             " (fixed comm grid rejected) [" + logTail_ + "]");
  if (s != fmi2OK) fail("fmi2DoStep", s);
  t_ += commStep;
}

void FmuSlave::terminate() {
  require(State::Stepping, "terminate");
  fmi2Status s = fns_->terminate(comp_);
  if (s != fmi2OK) fail("fmi2Terminate", s);
  state_ = State::Terminated;
}

void FmuSlave::setReal(const std::string& name, double v) {
  if (state_ != State::Init && state_ != State::Stepping)
    throw std::runtime_error("fmi import: setReal out of order");
  const fmi2ValueReference vr = desc_.find(name).valueReference;
  fmi2Status s = fns_->setReal(comp_, &vr, 1, &v);
  if (s != fmi2OK) fail("fmi2SetReal(" + name + ")", s);
}

void FmuSlave::setBoolean(const std::string& name, bool v) {
  if (state_ != State::Init && state_ != State::Stepping)
    throw std::runtime_error("fmi import: setBoolean out of order");
  const fmi2ValueReference vr = desc_.find(name).valueReference;
  const fmi2Boolean b = v ? 1 : 0;
  fmi2Status s = fns_->setBoolean(comp_, &vr, 1, &b);
  if (s != fmi2OK) fail("fmi2SetBoolean(" + name + ")", s);
}

void FmuSlave::setInteger(const std::string& name, long long v) {
  if (state_ != State::Init && state_ != State::Stepping)
    throw std::runtime_error("fmi import: setInteger out of order");
  const fmi2ValueReference vr = desc_.find(name).valueReference;
  const fmi2Integer i = static_cast<fmi2Integer>(v);
  fmi2Status s = fns_->setInteger(comp_, &vr, 1, &i);
  if (s != fmi2OK) fail("fmi2SetInteger(" + name + ")", s);
}

double FmuSlave::getReal(const std::string& name) {
  require(State::Stepping, "getReal");
  const fmi2ValueReference vr = desc_.find(name).valueReference;
  fmi2Real v = 0;
  fmi2Status s = fns_->getReal(comp_, &vr, 1, &v);
  if (s != fmi2OK) fail("fmi2GetReal(" + name + ")", s);
  return v;
}

bool FmuSlave::getBoolean(const std::string& name) {
  require(State::Stepping, "getBoolean");
  const fmi2ValueReference vr = desc_.find(name).valueReference;
  fmi2Boolean b = 0;
  fmi2Status s = fns_->getBoolean(comp_, &vr, 1, &b);
  if (s != fmi2OK) fail("fmi2GetBoolean(" + name + ")", s);
  return b != 0;
}

long long FmuSlave::getInteger(const std::string& name) {
  require(State::Stepping, "getInteger");
  const fmi2ValueReference vr = desc_.find(name).valueReference;
  fmi2Integer i = 0;
  fmi2Status s = fns_->getInteger(comp_, &vr, 1, &i);
  if (s != fmi2OK) fail("fmi2GetInteger(" + name + ")", s);
  return i;
}

CoSim::CoSim(FmuSlave& slave, double commStep, std::vector<Input> inputs,
             std::vector<Output> outputs)
    : slave_(slave), commStep_(commStep), inputs_(std::move(inputs)), outputs_(std::move(outputs)) {
  if (!(commStep_ > 0.0)) throw std::runtime_error("fmi import: commStep must be > 0");
  for (const auto& in : inputs_) {
    const FmuVariable& v = slave_.description().find(in.fmuVar);
    if (v.causality != "input")
      throw std::runtime_error("fmi import: '" + in.fmuVar + "' is not an input");
    if (!in.isConst && in.probe.empty() && in.current.empty())
      throw std::runtime_error("fmi import: input '" + in.fmuVar + "' needs probe/current/const");
    if (v.type != 'R' && v.type != 'B' && v.type != 'I')
      throw std::runtime_error("fmi import: input '" + in.fmuVar + "' type unsupported (v1: R/I/B)");
  }
  for (const auto& out : outputs_) {
    const FmuVariable& v = slave_.description().find(out.fmuVar);
    if (v.causality != "output")
      throw std::runtime_error("fmi import: '" + out.fmuVar + "' is not an output");
    if (v.type != 'B' && v.type != 'R')
      throw std::runtime_error("fmi import: output '" + out.fmuVar + "' type unsupported (v1: B/R)");
  }
}

void CoSim::exchange(Engine& eng) {
  const double te = eng.time(), tf = slave_.time();
  if (std::abs(te - tf) > 1e-9 * (1.0 + std::abs(tf)))
    throw std::runtime_error("fmi import: engine/slave time skew (" + std::to_string(te) +
                             " vs " + std::to_string(tf) + ")");
  const auto& probes = eng.currentSolution().probes;
  for (const auto& in : inputs_) {
    const FmuVariable& v = slave_.description().find(in.fmuVar);
    double val = in.constant;
    if (!in.isConst) {
      if (!in.probe.empty()) {
        const auto it = probes.find(in.probe);
        if (it == probes.end())
          throw std::runtime_error("fmi import: unknown probe '" + in.probe + "'");
        val = it->second;
      } else {
        val = eng.deviceCurrent(in.current);
      }
    }
    if (v.type == 'R')
      slave_.setReal(in.fmuVar, val);
    else if (v.type == 'B')
      slave_.setBoolean(in.fmuVar, val > 0.5);
    else
      slave_.setInteger(in.fmuVar, static_cast<long long>(std::llround(val)));
  }
  slave_.doStep(commStep_);
  for (const auto& out : outputs_) {
    const FmuVariable& v = slave_.description().find(out.fmuVar);
    const bool gate = (v.type == 'B') ? slave_.getBoolean(out.fmuVar)
                                      : (slave_.getReal(out.fmuVar) > 0.5);
    eng.setSwitch(out.switchName, gate);
  }
}

}  // namespace fmi_import
}  // namespace power_engine
