#include "power_engine/sweep.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>

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
  if (cfg.jobs < 0) throw std::runtime_error("runSweep jobs must be >= 0 (0 = auto)");
  for (const auto& ax : cfg.axes) checkAxis(ax);
  // Cartesian product sizes (empty axes = single point).
  std::size_t total = 1;
  for (const auto& ax : cfg.axes) total *= ax.values.size();
  // Point parameters by flat index (odometer: last axis fastest).
  auto pointAt = [&](std::size_t n) {
    std::map<std::string, double> point;
    std::size_t rem = n;
    for (std::size_t a = cfg.axes.size(); a-- > 0;) {
      const std::size_t m = cfg.axes[a].values.size();
      point[cfg.axes[a].param] = cfg.axes[a].values[rem % m];
      rem /= m;
    }
    return point;
  };
  auto runPoint = [&](std::size_t n) {
    SweepResult row;
    row.params = pointAt(n);
    try {
      Engine eng;
      eng.loadNetlist(cfg.netlist);
      for (const auto& [k, v] : row.params) eng.setParameter(k, v);
      cfg.setup(eng, row.params);
      eng.start();
      while (eng.status() == SimulationStatus::Running) eng.step();
      row.outputs = cfg.measure(eng, row.params);
    } catch (const std::exception& e) {
      if (cfg.stopOnError) throw;
      row.ok = false;
      row.error = e.what();
    }
    return row;
  };
  unsigned jobs = 1;
  if (cfg.jobs == 0) {
    const unsigned hw = std::thread::hardware_concurrency();
    jobs = hw == 0 ? 2u : hw;
  } else {
    jobs = static_cast<unsigned>(cfg.jobs);
  }
  SweepTable table;
  table.rows.resize(total);
  if (jobs <= 1 || total <= 1) {
    for (std::size_t n = 0; n < total; ++n) table.rows[n] = runPoint(n);
    return table;
  }
  // Workers pull flat indices atomically; rows are disjoint by construction.
  std::atomic<std::size_t> next{0};
  std::atomic<bool> stop{false};
  std::mutex errMutex;
  std::exception_ptr firstErr;
  const unsigned nWorkers = std::min<unsigned>(jobs, static_cast<unsigned>(total));
  std::vector<std::thread> workers;
  workers.reserve(nWorkers);
  for (unsigned w = 0; w < nWorkers; ++w) {
    workers.emplace_back([&] {
      for (;;) {
        if (stop.load(std::memory_order_relaxed)) return;
        const std::size_t n = next.fetch_add(1, std::memory_order_relaxed);
        if (n >= total) return;
        try {
          table.rows[n] = runPoint(n);
        } catch (...) {
          // stopOnError propagation (per-row std::exceptions are already
          // recorded inside runPoint): first error wins, rethrown on join.
          std::lock_guard<std::mutex> lock(errMutex);
          if (!firstErr) firstErr = std::current_exception();
          stop.store(true, std::memory_order_relaxed);
          return;
        }
      }
    });
  }
  for (auto& t : workers) t.join();
  if (firstErr) std::rethrow_exception(firstErr);
  return table;
}

}  // namespace sweep
}  // namespace power_engine
