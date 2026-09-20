#include "power_engine/thermal.h"

#include <cmath>
#include <stdexcept>

namespace power_engine {
namespace thermal {
namespace {

void checkStages(const std::vector<Stage>& stages) {
  if (stages.empty() || stages.size() > 32) {
    throw std::runtime_error("thermal network needs 1..32 stages");
  }
  for (const auto& s : stages) {
    if (!(s.r > 0.0) || !std::isfinite(s.r)) {
      throw std::runtime_error("thermal R must be positive finite");
    }
    if (!(s.c > 0.0) || !std::isfinite(s.c)) {
      throw std::runtime_error("thermal C must be positive finite");
    }
  }
}

}  // namespace

ThermalNetwork::ThermalNetwork(Kind kind, std::vector<Stage> stages, double tamb)
    : kind_(kind), stages_(std::move(stages)), tamb_(tamb), states_(stages_.size(), 0.0) {
  if (!std::isfinite(tamb)) throw std::runtime_error("Tamb must be finite");
}

ThermalNetwork ThermalNetwork::foster(std::vector<Stage> stages, double tamb) {
  checkStages(stages);
  return {Kind::Foster, std::move(stages), tamb};
}

ThermalNetwork ThermalNetwork::cauer(std::vector<Stage> stages, double tamb) {
  checkStages(stages);
  return {Kind::Cauer, std::move(stages), tamb};
}

void ThermalNetwork::reset() { states_.assign(stages_.size(), 0.0); }

void ThermalNetwork::step(double power, double dt) {
  if (!std::isfinite(power)) throw std::runtime_error("thermal power must be finite");
  if (!(dt > 0.0) || !std::isfinite(dt)) throw std::runtime_error("thermal dt must be positive");
  if (kind_ == Kind::Foster) {
    for (std::size_t i = 0; i < stages_.size(); ++i) {
      const double tau = stages_[i].r * stages_[i].c;
      const double target = power * stages_[i].r;
      states_[i] += (target - states_[i]) * (1.0 - std::exp(-dt / tau));
    }
    return;
  }
  // Cauer, explicit Euler on node temperatures (relative to Tamb).
  const std::size_t n = stages_.size();
  std::vector<double> dT(n, 0.0);
  for (std::size_t i = 0; i < n; ++i) {
    double in = 0.0, out = 0.0;
    if (i == 0) {
      in = power;
    } else {
      in = (states_[i - 1] - states_[i]) / stages_[i - 1].r;
    }
    if (i + 1 < n) {
      out = (states_[i] - states_[i + 1]) / stages_[i].r;
    } else {
      out = states_[i] / stages_[i].r;  // to ambient (relative temp)
    }
    dT[i] = (in - out) / stages_[i].c;
  }
  for (std::size_t i = 0; i < n; ++i) states_[i] += dT[i] * dt;
}

double ThermalNetwork::tj() const {
  double t = tamb_;
  if (kind_ == Kind::Foster) {
    for (double s : states_) t += s;
  } else if (!states_.empty()) {
    t += states_[0];  // junction-side node
  }
  return t;
}

}  // namespace thermal
}  // namespace power_engine
