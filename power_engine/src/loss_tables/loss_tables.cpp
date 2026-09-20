#include "power_engine/loss_tables.h"

#include <cmath>

namespace power_engine {
namespace loss {

Table::Table(std::vector<std::string> axes, std::vector<std::vector<double>> grids,
             std::vector<double> values, std::string tag)
    : axes_(std::move(axes)),
      grids_(std::move(grids)),
      values_(std::move(values)),
      tag_(std::move(tag)) {
  if (axes_.empty()) throw std::runtime_error("loss Table needs >= 1 axis");
  if (axes_.size() > 4) throw std::runtime_error("loss Table supports at most 4 axes");
  if (grids_.size() != axes_.size()) {
    throw std::runtime_error("loss Table: grids/axes size mismatch");
  }
  std::size_t n = 1;
  for (std::size_t a = 0; a < axes_.size(); ++a) {
    if (axes_[a].empty()) throw std::runtime_error("loss Table: empty axis name");
    if (grids_[a].empty()) throw std::runtime_error("loss Table: empty grid");
    for (double v : grids_[a]) {
      if (!std::isfinite(v)) throw std::runtime_error("loss Table: non-finite grid");
    }
    for (std::size_t i = 1; i < grids_[a].size(); ++i) {
      if (!(grids_[a][i] > grids_[a][i - 1])) {
        throw std::runtime_error("loss Table: grid must be strictly increasing");
      }
    }
    n *= grids_[a].size();
  }
  if (values_.size() != n) throw std::runtime_error("loss Table: values/grid size mismatch");
  for (double v : values_) {
    if (!std::isfinite(v)) throw std::runtime_error("loss Table: non-finite value");
  }
}

double Table::at(const std::map<std::string, double>& coords) const {
  if (empty()) throw std::runtime_error("loss Table::at on empty table");
  const std::size_t n = axes_.size();
  // Per-axis bracket [lo,hi] + fraction (clamped at edges; exact on nodes).
  std::vector<std::size_t> lo(n), hi(n);
  std::vector<double> frac(n, 0.0);
  for (std::size_t a = 0; a < n; ++a) {
    auto it = coords.find(axes_[a]);
    if (it == coords.end()) {
      throw std::runtime_error("loss Table::at missing axis '" + axes_[a] + "'");
    }
    if (!std::isfinite(it->second)) {
      throw std::runtime_error("loss Table::at non-finite coordinate");
    }
    const auto& g = grids_[a];
    if (g.size() == 1 || it->second <= g.front()) {
      lo[a] = hi[a] = 0;
    } else if (it->second >= g.back()) {
      lo[a] = hi[a] = g.size() - 1;
    } else {
      const auto up = std::upper_bound(g.begin(), g.end(), it->second);
      hi[a] = static_cast<std::size_t>(up - g.begin());
      lo[a] = hi[a] - 1;
      frac[a] = (it->second - g[lo[a]]) / (g[hi[a]] - g[lo[a]]);
    }
  }
  // Multilinear over the 2^n corners (row-major, last axis fastest).
  std::vector<std::size_t> stride(n, 1);
  for (std::size_t a = n; a-- > 0;) {
    if (a + 1 < n) stride[a] = stride[a + 1] * grids_[a + 1].size();
  }
  double acc = 0.0;
  const std::size_t corners = std::size_t{1} << n;
  for (std::size_t c = 0; c < corners; ++c) {
    double w = 1.0;
    std::size_t idx = 0;
    for (std::size_t a = 0; a < n; ++a) {
      const bool takeHi = (c >> (n - 1 - a)) & 1u;
      const std::size_t k = takeHi ? hi[a] : lo[a];
      w *= takeHi ? frac[a] : (1.0 - frac[a]);
      idx += k * stride[a];
    }
    acc += w * values_[idx];
  }
  return acc;
}

}  // namespace loss
}  // namespace power_engine
