#pragma once
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace power_engine {

/// Time-domain solution snapshot at time t.
struct Solution {
  double t = 0.0;
  /// Node voltages keyed as "v:<node_id>", e.g. "v:1".
  std::map<std::string, double> probes;
  /// Typed boolean control signals keyed as "gate:<switch>", e.g.
  /// "gate:S1". Kept separate from `probes` so consumers can tell a
  /// logic signal (0/1) from a physical quantity without guessing. The
  /// CLI exposes them on request as `--probe gate:S1`.
  std::map<std::string, bool> gates;
  /// Full nodal voltages in solver order (index 0 -> first non-ground node).
  std::vector<double> states;
};

enum class SimulationStatus { Idle, Running, Finished, Error };

using SolutionCallback = std::function<void(const Solution&)>;

}  // namespace power_engine
