#pragma once
#include <functional>
#include <map>
#include <string>
#include <vector>

#include "power_engine/engine.h"

namespace power_engine {
namespace sweep {

/// One sweep axis: netlist `.param` name + values (set via
/// Engine::setParameter, which re-elaborates per point).
struct SweepAxis {
  std::string param;
  std::vector<double> values;
};

/// One grid-point result: parameter values + measured outputs.
/// ok=false carries the error message instead of outputs.
struct SweepResult {
  std::map<std::string, double> params;
  std::map<std::string, double> outputs;
  bool ok = true;
  std::string error;
};

/// Full results table (cartesian product order: last axis fastest).
struct SweepTable {
  std::vector<SweepResult> rows;
  /// CSV with param columns then the union of output keys (in first-seen
  /// order); failed rows carry error= in an extra column.
  std::string csv() const;
};

/// Parameter sweep over a netlist (no new language: harness + callbacks).
///
/// Per grid point: load netlist, setParameter() each axis value,
/// setup(eng, point) for test-specific wiring (PWM/thermal/loss specs,
/// custom edges — all public Engine API), start, step to stop, then
/// measure(eng, point) computes outputs. Exceptions in setup/run/measure
/// are recorded per row unless stopOnError (then they propagate).
struct SweepConfig {
  std::string netlist;
  std::vector<SweepAxis> axes;
  std::function<void(Engine&, const std::map<std::string, double>&)> setup =
      [](Engine&, const std::map<std::string, double>&) {};
  std::function<std::map<std::string, double>(Engine&,
                                              const std::map<std::string, double>&)>
      measure;
  bool stopOnError = false;
};

SweepTable runSweep(const SweepConfig& cfg);

}  // namespace sweep
}  // namespace power_engine
