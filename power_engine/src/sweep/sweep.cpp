#include "power_engine/sweep.h"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace power_engine {
namespace sweep {
namespace {

void checkAxis(const SweepAxis& ax) {
  if (ax.param.empty()) throw std::runtime_error("sweep axis needs a param name");
  if (ax.values.empty()) throw std::runtime_error("sweep axis needs >= 1 value");
  for (double v : ax.values) {
    if (!std::isfinite(v)) throw std::runtime_error("sweep axis values must be finite");
  }
}

}  // namespace

std::string SweepTable::csv() const {
  // Column order: params (first-seen), outputs (first-seen), then error.
  std::vector<std::string> pcols, ocols;
  for (const auto& r : rows) {
    for (const auto& [k, v] : r.params) {
      (void)v;
      if (std::find(pcols.begin(), pcols.end(), k) == pcols.end()) pcols.push_back(k);
    }
    for (const auto& [k, v] : r.outputs) {
      (void)v;
      if (std::find(ocols.begin(), ocols.end(), k) == ocols.end()) ocols.push_back(k);
    }
  }
  std::ostringstream os;
  for (std::size_t i = 0; i < pcols.size(); ++i) os << (i ? "," : "") << pcols[i];
  for (const auto& c : ocols) os << "," << c;
  os << ",error\n";
  for (const auto& r : rows) {
    bool first = true;
    for (const auto& c : pcols) {
      os << (first ? "" : ",");
      first = false;
      auto it = r.params.find(c);
      if (it != r.params.end()) os << it->second;
    }
    for (const auto& c : ocols) {
      os << ",";
      auto it = r.outputs.find(c);
      if (it != r.outputs.end()) os << it->second;
    }
    os << "," << (r.ok ? "" : r.error) << "\n";
  }
  return os.str();
}

SweepTable runSweep(const SweepConfig& cfg) {
  if (!cfg.measure) throw std::runtime_error("runSweep needs a measure callback");
  for (const auto& ax : cfg.axes) checkAxis(ax);
  // Cartesian product sizes (empty axes = single point).
  std::size_t total = 1;
  for (const auto& ax : cfg.axes) total *= ax.values.size();
  SweepTable table;
  std::vector<std::size_t> idx(cfg.axes.size(), 0);
  for (std::size_t n = 0; n < total; ++n) {
    // Odometer: last axis fastest.
    std::map<std::string, double> point;
    std::size_t rem = n;
    for (std::size_t a = cfg.axes.size(); a-- > 0;) {
      const std::size_t m = cfg.axes[a].values.size();
      idx[a] = rem % m;
      rem /= m;
      point[cfg.axes[a].param] = cfg.axes[a].values[idx[a]];
    }
    SweepResult row;
    row.params = point;
    try {
      Engine eng;
      eng.loadNetlist(cfg.netlist);
      for (const auto& [k, v] : point) eng.setParameter(k, v);
      cfg.setup(eng, point);
      eng.start();
      while (eng.status() == SimulationStatus::Running) eng.step();
      row.outputs = cfg.measure(eng, point);
    } catch (const std::exception& e) {
      if (cfg.stopOnError) throw;
      row.ok = false;
      row.error = e.what();
    }
    table.rows.push_back(std::move(row));
  }
  return table;
}

}  // namespace sweep
}  // namespace power_engine
