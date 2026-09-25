#pragma once
// FMI 2.0 Co-Simulation IMPORT: drive an external FMU from our engine.
//
// Mirror of the export discipline (power_engine/fmi/fmi_wrapper.cpp):
// hostile ABI, explicit state machine, every non-OK status throws with
// the FMU's own last-error text. v1 scope (deliberate):
//   - FMI 2.0 Co-Simulation only (fmiVersion != "2.0" throws).
//   - Extracted FMU directories (binaries/<plat>/<modelIdentifier>.<ext>);
//     `.fmu` unzip stays user-side (`unzip -d dir model.fmu`).
//   - Controller coupling: FMU inputs sample engine probes / device
//     currents, FMU outputs drive ideal-switch gates. FMU-driven sources
//     are the fast follow (needs a mid-run source-value seam).
//   - No fmi2Reset / GetFMUstate (v1 always runs [0, tEnd] forward).
#include <cstddef>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace power_engine {

class Engine;  // power_engine::Engine (include engine.h to co-simulate)

namespace fmi_import {

/// One <ScalarVariable>: name, valueReference, causality, type + start.
struct FmuVariable {
  std::string name;
  unsigned int valueReference = 0;
  std::string causality;    // "input" | "output" | "local" | ...
  std::string variability;  // "continuous" | "discrete" | ...
  char type = '?';          // 'R' | 'I' | 'B' | 'S'
  double realStart = 0.0;
  long long intStart = 0;
  bool boolStart = false;
  std::string stringStart;
};

/// Parsed modelDescription.xml (hand-rolled parser: the modelDescription
/// grammar subset FMI 2.0 CS needs; no XML dependency by design).
struct ModelDescription {
  std::string modelName;
  std::string guid;
  std::string modelIdentifier;  // <CoSimulation modelIdentifier="..">
  std::vector<FmuVariable> variables;

  const FmuVariable& find(const std::string& name) const;  // throws
};

/// Parse <dir>/modelDescription.xml. Throws on missing file, malformed
/// XML-subset, fmiVersion != 2.0, or missing <CoSimulation>.
ModelDescription parseModelDescription(const std::string& fmuDir);

/// Resolved platform library path inside an extracted FMU dir.
std::string platformLibPath(const std::string& fmuDir,
                            const std::string& modelIdentifier);

/// An instantiated FMU slave: RAII over dlopen/LoadLibrary +
/// instantiate/setup/init/doStep/terminate/free. Every method enforces
/// the instantiate->init->step->terminated order (throws otherwise) and
/// every non-fmi2OK status throws with function + FMU log tail.
class FmuSlave {
 public:
  explicit FmuSlave(const std::string& fmuDir);
  ~FmuSlave();
  FmuSlave(const FmuSlave&) = delete;
  FmuSlave& operator=(const FmuSlave&) = delete;

  const ModelDescription& description() const { return desc_; }
  double time() const { return t_; }

  /// Append FMU logger text (called by the C logger callback; public so
  /// the free function can reach it without friendship tricks).
  void appendLog(const char* msg) {
    if (msg) {
      logTail_ += msg;
      logTail_ += '\n';
      if (logTail_.size() > 2000) logTail_.erase(0, logTail_.size() - 2000);
    }
  }

  void enterInit();   // setupExperiment + enterInitializationMode
  void exitInit();    // exitInitializationMode (applies Real starts)
  void doStep(double commStep);  // advances t_ by commStep
  void terminate();

  void setReal(const std::string& name, double v);
  void setBoolean(const std::string& name, bool v);
  void setInteger(const std::string& name, long long v);
  double getReal(const std::string& name);
  bool getBoolean(const std::string& name);
  long long getInteger(const std::string& name);

 private:
  enum class State { Loaded, Instantiated, Init, Stepping, Terminated };
  void require(State s, const char* what) const;
  [[noreturn]] void fail(const std::string& fn, int status) const;

  ModelDescription desc_;
  std::string dir_;
  void* handle_ = nullptr;
  void* comp_ = nullptr;
  State state_ = State::Loaded;
  double t_ = 0.0;
  std::string logTail_;
  struct Fns;  // fmi2 function pointers (pimpl: keeps fmi headers private)
  std::unique_ptr<Fns> fns_;
};

/// Engine <-> slave binding. Inputs sample the engine, outputs drive it:
///   input:  {fmuVar, probe "v:3"} | {fmuVar, current "L1"} | {fmuVar, const}
///   output: {switch "S1", fmuVar} (Boolean direct, Real thresholded at 0.5)
/// exchange() sets inputs, doSteps one commStep, applies outputs. Engine
/// time must equal slave time at entry (throws otherwise: no silent skew).
class CoSim {
 public:
  struct Input {
    std::string fmuVar;
    std::string probe;    // "v:3" style key, else ...
    std::string current;  // ... deviceCurrent(name), else ...
    double constant = 0.0;
    bool isConst = false;
  };
  struct Output {
    std::string switchName;
    std::string fmuVar;
  };
  CoSim(FmuSlave& slave, double commStep,
        std::vector<Input> inputs, std::vector<Output> outputs);

  double commStep() const { return commStep_; }
  void exchange(Engine& eng);

 private:
  FmuSlave& slave_;
  double commStep_;
  std::vector<Input> inputs_;
  std::vector<Output> outputs_;
};

}  // namespace fmi_import
}  // namespace power_engine
