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
  /// Full nodal voltages in solver order (index 0 -> first non-ground node).
  std::vector<double> states;
};

enum class SimulationStatus { Idle, Running, Finished, Error };

using SolutionCallback = std::function<void(const Solution&)>;

}  // namespace power_engine
