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
// splitmix64: deterministic integer PRNG (no FP platform variance).
uint64_t splitmix64(uint64_t& s) {
  uint64_t z = (s += 0x9E3779B97F4A7C15ULL);
  z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
  z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
  return z ^ (z >> 31);
}
// Uniform [0,1): 53-bit mantissa from the top bits (never exactly 1.0,
// and > 0 unless the draw is exactly zero — guarded by callers).
double unit01(uint64_t& s) {
  return (static_cast<double>(splitmix64(s) >> 11)) * (1.0 / 9007199254740992.0);
}
}  // namespace

std::vector<double> uniformSamples(int n, double lo, double hi, std::uint64_t seed) {
  if (n <= 0) throw std::runtime_error("uniformSamples needs n > 0");
  if (!(lo <= hi) || !std::isfinite(lo) || !std::isfinite(hi))
    throw std::runtime_error("uniformSamples needs finite lo <= hi");
  std::vector<double> out;
  out.reserve(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) out.push_back(lo + (hi - lo) * unit01(seed));
  return out;
}

std::vector<double> gaussianSamples(int n, double mean, double sigma, std::uint64_t seed) {
  if (n <= 0) throw std::runtime_error("gaussianSamples needs n > 0");
  if (!(sigma >= 0.0) || !std::isfinite(sigma) || !std::isfinite(mean))
    throw std::runtime_error("gaussianSamples needs finite mean, sigma >= 0");
  std::vector<double> out;
  out.reserve(static_cast<std::size_t>(n));
  constexpr double kPi = 3.14159265358979323846;
  for (int i = 0; i < n;) {
    // Box-Muller pair; u1 in (0,1) — redraw on exact 0 (log domain).
    double u1 = 0.0;
    while (u1 <= 0.0) u1 = unit01(seed);
    const double u2 = unit01(seed);
    const double r = std::sqrt(-2.0 * std::log(u1));
    const double a = 2.0 * kPi * u2;
    out.push_back(mean + sigma * r * std::cos(a));
    if (++i < n) out.push_back(mean + sigma * r * std::sin(a));
    ++i;
  }
  return out;
}

ColumnStats columnStats(const SweepTable& t, const std::string& output) {
  ColumnStats s;
  double m2 = 0.0;
  bool first = true;
  for (const auto& r : t.rows) {
    if (!r.ok) continue;
    const auto it = r.outputs.find(output);
    if (it == r.outputs.end()) throw std::runtime_error("columnStats: missing output " + output);
    const double x = it->second;
    if (first) {
      s.mean = x;
      s.min = x;
      s.max = x;
      first = false;
    } else {
      const double d = x - s.mean;
      s.mean += d / static_cast<double>(s.n + 1);
      m2 += d * (x - s.mean);
      if (x < s.min) s.min = x;
      if (x > s.max) s.max = x;
    }
    ++s.n;
  }
  if (s.n == 0) throw std::runtime_error("columnStats: no ok rows for " + output);
  s.std = s.n > 1 ? std::sqrt(m2 / static_cast<double>(s.n)) : 0.0;
  return s;
}

double yieldWithin(const SweepTable& t, const std::string& output, double lo, double hi) {
  std::size_t ok = 0, in = 0;
  for (const auto& r : t.rows) {
    if (!r.ok) continue;
    const auto it = r.outputs.find(output);
    if (it == r.outputs.end()) throw std::runtime_error("yieldWithin: missing output " + output);
    ++ok;
    if (it->second >= lo && it->second <= hi) ++in;
  }
  if (ok == 0) throw std::runtime_error("yieldWithin: no ok rows for " + output);
  return static_cast<double>(in) / static_cast<double>(ok);
}

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
