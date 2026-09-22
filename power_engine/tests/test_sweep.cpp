#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <gtest/gtest.h>

#include "power_engine/engine.h"
#include "power_engine/sweep.h"

using power_engine::Engine;
using power_engine::sweep::runSweep;
using power_engine::sweep::SweepAxis;
using power_engine::sweep::SweepConfig;
using power_engine::sweep::SweepTable;

// --- Grid mechanics: cartesian product, odometer order, error capture.
TEST(SweepGrid, ProductOrderAndErrors) {
  SweepConfig cfg;
  cfg.netlist = "V1 1 0 1\nR1 1 0 10\n.tran 1u 10u\n.end\n";
  cfg.axes = {{"A", {1.0, 2.0}}, {"B", {10.0, 20.0, 30.0}}};
  cfg.measure = [](Engine&, const std::map<std::string, double>& p) {
    return std::map<std::string, double>{{"sum", p.at("A") + p.at("B")}};
  };
  const SweepTable t = runSweep(cfg);
  ASSERT_EQ(t.rows.size(), 6u);
  // Last axis fastest: (1,10) (1,20) (1,30) (2,10) ...
  EXPECT_EQ(t.rows[0].params.at("A"), 1.0);
  EXPECT_EQ(t.rows[0].params.at("B"), 10.0);
  EXPECT_EQ(t.rows[2].params.at("B"), 30.0);
  EXPECT_EQ(t.rows[3].params.at("A"), 2.0);
  EXPECT_EQ(t.rows[5].outputs.at("sum"), 32.0);
  for (const auto& r : t.rows) EXPECT_TRUE(r.ok);
  // CSV carries params, outputs, error column.
  const std::string csv = t.csv();
  EXPECT_NE(csv.find("A,B,sum,error"), std::string::npos);
  EXPECT_NE(csv.find("2,30,32,"), std::string::npos);

  // Empty axes = single point.
  SweepConfig single;
  single.netlist = cfg.netlist;
  single.measure = [](Engine&, const std::map<std::string, double>&) {
    return std::map<std::string, double>{{"x", 1.0}};
  };
  EXPECT_EQ(runSweep(single).rows.size(), 1u);

  // Measure throws: recorded per row, or propagated with stopOnError.
  SweepConfig bad = cfg;
  bad.measure = [](Engine&, const std::map<std::string, double>& p) {
    if (p.at("A") == 2.0) throw std::runtime_error("boom");
    return std::map<std::string, double>{};
  };
  const SweepTable t2 = runSweep(bad);
  EXPECT_TRUE(t2.rows[0].ok);
  EXPECT_FALSE(t2.rows[3].ok);
  EXPECT_NE(t2.rows[3].error.find("boom"), std::string::npos);
  bad.stopOnError = true;
  EXPECT_THROW(runSweep(bad), std::runtime_error);

  // Bad config rejected up front.
  SweepConfig noMeasure;
  noMeasure.netlist = cfg.netlist;
  EXPECT_THROW(runSweep(noMeasure), std::runtime_error);
  SweepConfig badAxis = cfg;
  badAxis.axes = {{"A", {}}};
  badAxis.measure = cfg.measure;
  EXPECT_THROW(runSweep(badAxis), std::runtime_error);
}

// --- Threaded sweep: bitwise-identical tables, error parity, speedup.
// NOTE: threaded points must use stateless setup/measure (no shared
// mutable accumulator across points — that pattern is a data race with
// jobs>1; the efficiency test above stays serial for that reason).
namespace {

SweepConfig buckGrid() {
  SweepConfig cfg;
  cfg.netlist = R"(
.param RLOAD 5
.param FSW 20k
.tran 0.5u 6m
V1 1 0 12
S1 1 2 RON=5m ROFF=1Meg EON=10u EOFF=15u INIT=ON
D1 0 2 VF=0.7 RON=10m
L1 2 3 200u
C1 3 0 200u
Rload 3 0 {RLOAD}
.control pwm switch=S1 freq={FSW} duty=0.5
.end
)";
  cfg.axes = {{"RLOAD", {2.5, 5.0, 10.0, 20.0}}};
  cfg.setup = [](Engine& eng, const std::map<std::string, double>&) {
    eng.applyPwmSpecs();
  };
  // Stateless: final Vout + edge-counted losses only (no windowed means).
  cfg.measure = [](Engine& eng, const std::map<std::string, double>&) {
    return std::map<std::string, double>{
        {"vout", eng.currentSolution().probes.at("v:3")},
        {"esw", eng.deviceLoss("S1").esw},
        {"econd", eng.deviceLoss("S1").econd + eng.deviceLoss("D1").econd}};
  };
  return cfg;
}

}  // namespace

TEST(SweepThreads, BitwiseIdenticalCsv) {
  SweepConfig serial = buckGrid();
  serial.jobs = 1;
  const std::string ref = runSweep(serial).csv();
  SweepConfig par = buckGrid();
  par.jobs = 4;
  EXPECT_EQ(runSweep(par).csv(), ref);
  SweepConfig autoJobs = buckGrid();
  autoJobs.jobs = 0;
  EXPECT_EQ(runSweep(autoJobs).csv(), ref);
  EXPECT_THROW({ SweepConfig bad = buckGrid(); bad.jobs = -1; runSweep(bad); },
               std::runtime_error);
}

TEST(SweepThreads, ErrorRowsMatch) {
  auto failing = [] {
    SweepConfig cfg = buckGrid();
    cfg.measure = [](Engine& eng, const std::map<std::string, double>& p) {
      if (p.at("RLOAD") == 10.0) throw std::runtime_error("bad load");
      return std::map<std::string, double>{
          {"vout", eng.currentSolution().probes.at("v:3")}};
    };
    return cfg;
  };
  SweepConfig s1 = failing();
  s1.jobs = 1;
  const SweepTable ref = runSweep(s1);
  SweepConfig s4 = failing();
  s4.jobs = 4;
  const SweepTable got = runSweep(s4);
  ASSERT_EQ(got.rows.size(), ref.rows.size());
  for (std::size_t i = 0; i < ref.rows.size(); ++i) {
    EXPECT_EQ(got.rows[i].ok, ref.rows[i].ok);
    EXPECT_EQ(got.rows[i].error, ref.rows[i].error);
    EXPECT_EQ(got.rows[i].outputs, ref.rows[i].outputs);
  }
  SweepConfig stop = failing();
  stop.jobs = 4;
  stop.stopOnError = true;
  EXPECT_THROW(runSweep(stop), std::runtime_error);
}

TEST(SweepThreads, ParallelFaster) {
  SweepConfig cfg = buckGrid();
  cfg.axes = {{"RLOAD", {2.5, 5.0, 10.0, 20.0}}, {"FSW", {10e3, 20e3}}};
  cfg.jobs = 1;
  const auto t0 = std::chrono::steady_clock::now();
  const std::string ref = runSweep(cfg).csv();
  const auto t1 = std::chrono::steady_clock::now();
  cfg.jobs = 4;
  const auto t2 = std::chrono::steady_clock::now();
  EXPECT_EQ(runSweep(cfg).csv(), ref);
  const auto t3 = std::chrono::steady_clock::now();
  const double serialMs =
      std::chrono::duration<double, std::milli>(t1 - t0).count();
  const double parMs =
      std::chrono::duration<double, std::milli>(t3 - t2).count();
  std::printf("[sweep] serial=%.0fms parallel(4)=%.0fms speedup=%.2f\n", serialMs,
              parMs, serialMs / parMs);
  EXPECT_LT(parMs, serialMs);
}
// Transient-free checks (edge-counted esw is exact; econd carries the
// L/C startup, tolerated by shape/monotonicity assertions) plus an
// analytic efficiency band from measured Vout (no ideal-Vout assumption).
// --- Acceptance: buck efficiency vs load over RLOAD grid.
TEST(SweepEfficiency, BuckVsLoadMatchesAnalytic) {
  const char* netlist = R"(
.param RLOAD 5
.tran 0.5u 6m
V1 1 0 12
S1 1 2 RON=5m ROFF=1Meg EON=10u EOFF=15u INIT=ON
D1 0 2 VF=0.7 RON=10m
L1 2 3 200u
C1 3 0 200u
Rload 3 0 {RLOAD}
.control pwm switch=S1 freq=20k duty=0.5
.end
)";
  struct Acc {
    double sum = 0.0, n = 0.0;
  };
  auto acc = std::make_shared<Acc>();
  SweepConfig cfg;
  cfg.netlist = netlist;
  cfg.axes = {{"RLOAD", {2.5, 5.0, 10.0, 20.0}}};
  cfg.setup = [acc](Engine& eng, const std::map<std::string, double>&) {
    acc->sum = 0.0;
    acc->n = 0.0;
    eng.applyPwmSpecs();
    eng.setCallback([acc](const power_engine::Solution& s) {
      if (s.t > 5e-3) {
        acc->sum += s.probes.at("v:3");
        acc->n += 1.0;
      }
    });
  };
  cfg.measure = [acc](Engine& eng, const std::map<std::string, double>&) {
    const double vout = acc->sum / acc->n;
    return std::map<std::string, double>{
        {"vout", vout},
        {"esw", eng.deviceLoss("S1").esw},
        {"econd", eng.deviceLoss("S1").econd + eng.deviceLoss("D1").econd}};
  };
  const SweepTable t = runSweep(cfg);
  ASSERT_EQ(t.rows.size(), 4u);
  double prevEta = 2.0, prevEcond = 1e18;
  for (const auto& r : t.rows) {
    ASSERT_TRUE(r.ok) << r.error;
    const double rl = r.params.at("RLOAD");
    const double vout = r.outputs.at("vout");
    // Edge accounting is transient-free: 119 on + 120 off per 6ms run.
    EXPECT_NEAR(r.outputs.at("esw"), 119.0 * 10e-6 + 120.0 * 15e-6, 25e-6)
        << "R=" << rl;
    // Conduction scales ~1/R^2 (I^2R): strictly decreasing with RLOAD.
    EXPECT_LT(r.outputs.at("econd"), prevEcond) << "R=" << rl;
    prevEcond = r.outputs.at("econd");
    // Efficiency from measured Vout vs analytic loss model (broad band:
    // analytic uses ideal I=Vout/R and ignores ripple + startup).
    const double iL = vout / rl;
    const double pOut = vout * vout / rl;
    const double pLossRef =
        25e-6 * 20e3 + iL * iL * (0.5 * 5e-3 + 0.5 * 10e-3) + 0.7 * iL * 0.5;
    const double pLossMeas = (r.outputs.at("esw") + r.outputs.at("econd")) / 6e-3;
    EXPECT_NEAR(pLossMeas, pLossRef, 0.25 * pLossRef) << "R=" << rl;
    const double eta = pOut / (pOut + pLossMeas);
    EXPECT_LT(eta, prevEta) << "R=" << rl;  // switching floor: lighter load = worse eta
    prevEta = eta;
  }
}
