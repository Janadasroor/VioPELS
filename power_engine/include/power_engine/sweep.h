#pragma once
#include <cstddef>
#include <cstdint>
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
///
/// Parallelism: jobs==1 runs serially (default); jobs==0 uses
/// hardware_concurrency; jobs==N>1 runs N worker threads over disjoint
/// row ranges. Rows are written by index, so the table (and csv()) is
/// bitwise identical for any job count. Each point owns a local Engine —
/// no shared engine state — but setup/measure callbacks (and anything
/// they capture) MUST be thread-safe when jobs!=1; sharing one mutable
/// accumulator across points is a data race (keep per-point state in
/// locals or recompute in measure). With jobs>1 + stopOnError, the first
/// error is rethrown after in-flight points finish (vs immediate abort
/// when serial).
struct SweepConfig {
  std::string netlist;
  std::vector<SweepAxis> axes;
  std::function<void(Engine&, const std::map<std::string, double>&)> setup =
      [](Engine&, const std::map<std::string, double>&) {};
  std::function<std::map<std::string, double>(Engine&,
                                              const std::map<std::string, double>&)>
      measure;
  bool stopOnError = false;
  int jobs = 1;  ///< 1 serial, 0 = auto (hardware_concurrency), N = threads
};

SweepTable runSweep(const SweepConfig& cfg);

// --- Monte Carlo tolerance analysis (item 18). Axes take explicit value
// lists, so an MC run is just a sampled axis + runSweep (threaded) +
// statistics below. Sampling is deterministic (splitmix64, integer
// arithmetic — bitwise reproducible across platforms for a fixed seed).

/// n uniform samples in [lo, hi] (throws if n <= 0 or lo > hi).
std::vector<double> uniformSamples(int n, double lo, double hi,
                                   std::uint64_t seed = 0x9E3779B97F4A7C15ULL);
/// n Gaussian samples N(mean, sigma) via Box-Muller (sigma >= 0; throws
/// otherwise). Deterministic for a fixed seed.
std::vector<double> gaussianSamples(int n, double mean, double sigma,
                                    std::uint64_t seed = 0x6A09E667F3BCC909ULL);

/// Moments over one output column across ok rows (throws if empty).
struct ColumnStats {
  double mean = 0.0;
  double std = 0.0;  ///< population std
  double min = 0.0;
  double max = 0.0;
  std::size_t n = 0;
};
ColumnStats columnStats(const SweepTable& t, const std::string& output);
/// Fraction of ok rows with output in [lo, hi] (throws if no ok rows).
double yieldWithin(const SweepTable& t, const std::string& output, double lo,
                   double hi);

}  // namespace sweep
}  // namespace power_engine
