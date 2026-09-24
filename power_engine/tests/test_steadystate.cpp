#include <cmath>
#include <stdexcept>
#include <gtest/gtest.h>

#include "power_engine/engine.h"
#include "power_engine/steadystate.h"

using power_engine::Engine;
using power_engine::steadystate::ShootingConfig;
using power_engine::steadystate::ShootingResult;
using power_engine::steadystate::solvePeriodicSteadyState;

namespace {

void buildCcmBuck(Engine& eng, double dt) {
  eng.setTimeStep(dt);
  eng.circuit().addVoltageSource("Vin", 1, 0, 12.0);
  eng.circuit().addSwitch("S1", 1, 2, 5e-3, 1e6, true);
  eng.circuit().addDiode("D1", 0, 2, 0.0, 10e-3, 1e6);
  eng.circuit().addInductor("L1", 2, 3, 200e-6, 0.0);
  eng.circuit().addCapacitor("C1", 3, 0, 200e-6, 0.0);
  eng.circuit().addResistor("Rload", 3, 0, 5.0);
}

// Average Vout over exactly one period starting now (T/dt must be integer).
double avgOnePeriod(Engine& eng, double period) {
  double sum = 0.0;
  long long n = 0;
  const double tEnd = eng.time() + period;
  while (eng.time() < tEnd - 1e-12) {
    eng.step();
    sum += eng.currentSolution().probes.at("v:3");
    ++n;
  }
  return sum / static_cast<double>(n);
}

}  // namespace

// CCM buck, shooting from cold start: converges, lands on the orbit, and
// matches a brute-force 6ms transient (120 exact periods => same phase).
TEST(SteadyState, CcmBuckMatchesBruteForce) {
  constexpr double kT = 50e-6;
  Engine eng;
  buildCcmBuck(eng, 1e-6);
  eng.scheduleSwitch("S1", true, 0.0);
  eng.scheduleSwitch("S1", false, 0.5 * kT);
  eng.start();

  ShootingConfig cfg;
  cfg.period = kT;
  const ShootingResult r = solvePeriodicSteadyState(eng, 0.0, cfg);
  EXPECT_TRUE(r.converged);
  EXPECT_LT(r.iters, 15);
  EXPECT_EQ(eng.time(), 0.0);
  EXPECT_EQ(eng.status(), power_engine::SimulationStatus::Running);

  // Orbit average matches duty theory; states match brute force.
  EXPECT_NEAR(avgOnePeriod(eng, kT), 6.0, 0.02 * 6.0);
  const double ilShoot = eng.deviceCurrent("L1");

  Engine ref;
  buildCcmBuck(ref, 1e-6);
  for (double t = 0.0; t < 12e-3; t += kT) {
    ref.scheduleSwitch("S1", true, t);
    ref.scheduleSwitch("S1", false, t + 0.5 * kT);
  }
  ref.setStopTime(12e-3);  // 6 LC-envelope taus: brute force truly settled
  ref.start();
  while (ref.status() == power_engine::SimulationStatus::Running) ref.step();
  const double ilRef = ref.deviceCurrent("L1");
  const double vcRef = ref.currentSolution().probes.at("v:3");
  // Reposition: run the shooting engine back to orbit start for comparison.
  // (avgOnePeriod above advanced it one period; orbit is periodic.)
  EXPECT_NEAR(ilShoot, ilRef, 0.02 * std::abs(ilRef));
  EXPECT_NEAR(eng.currentSolution().probes.at("v:3"), vcRef, 0.02 * std::abs(vcRef));
}

// DCM buck: diodes re-settle inside every period sim; must still converge.
TEST(SteadyState, DcmBuckConverges) {
  constexpr double kT = 50e-6;
  Engine eng;
  eng.setTimeStep(0.5e-6);
  eng.circuit().addVoltageSource("Vin", 1, 0, 12.0);
  eng.circuit().addSwitch("S1", 1, 2, 5e-3, 1e6, true);
  eng.circuit().addDiode("D1", 0, 2, 0.0, 10e-3, 1e6);
  eng.circuit().addInductor("L1", 2, 3, 20e-6, 0.0);
  eng.circuit().addCapacitor("C1", 3, 0, 100e-6, 0.0);
  eng.circuit().addResistor("Rload", 3, 0, 20.0);
  eng.scheduleSwitch("S1", true, 0.0);
  eng.scheduleSwitch("S1", false, 0.5 * kT);
  eng.start();

  ShootingConfig cfg;
  cfg.period = kT;
  cfg.maxIters = 40;
  const ShootingResult r = solvePeriodicSteadyState(eng, 0.0, cfg);
  EXPECT_TRUE(r.converged);
  EXPECT_NEAR(avgOnePeriod(eng, kT), 10.53, 0.05 * 10.53);
}

TEST(SteadyState, BadConfigThrows) {
  Engine eng;
  buildCcmBuck(eng, 1e-6);
  ShootingConfig cfg;
  cfg.period = 50e-6;
  EXPECT_THROW(solvePeriodicSteadyState(eng, 0.0, cfg), std::runtime_error);  // not started
  eng.start();
  ShootingConfig bad = cfg;
  bad.period = 0.0;
  EXPECT_THROW(solvePeriodicSteadyState(eng, 0.0, bad), std::runtime_error);
  bad = cfg;
  bad.maxIters = 0;
  EXPECT_THROW(solvePeriodicSteadyState(eng, 0.0, bad), std::runtime_error);
  eng.setStopTime(1e-3);
  EXPECT_THROW(solvePeriodicSteadyState(eng, 0.0, cfg), std::runtime_error);  // stop set
  eng.clearStopTime();

  Engine res;
  res.setTimeStep(1e-6);
  res.circuit().addVoltageSource("V1", 1, 0, 5.0);
  res.circuit().addResistor("R1", 1, 0, 100.0);
  res.start();
  EXPECT_THROW(solvePeriodicSteadyState(res, 0.0, cfg), std::runtime_error);  // no L/C
}

// Refine-roadmap R7: rewindTo() is the sole legitimate setTime() path
// (histories + clock + events + accumulators realigned together). Rewinding
// mid-run and re-stepping must reproduce the uninterrupted trajectory
// bitwise — including across scheduled gate edges (event re-arm path).
TEST(SteadyState, RewindToReproducesUninterruptedRun) {
  constexpr double kT = 50e-6, kDt = 1e-6;
  auto build = [](Engine& eng) {
    buildCcmBuck(eng, kDt);
    for (double t = 0.0; t < 100e-6; t += kT) {
      eng.scheduleSwitch("S1", true, t);
      eng.scheduleSwitch("S1", false, t + 0.5 * kT);
    }
    eng.setStopTime(100e-6);
    eng.start();
  };
  auto traceTo = [](Engine& eng, double tEnd) {
    std::vector<double> v, il;
    while (eng.time() < tEnd - 1e-12) {
      eng.step();
      v.push_back(eng.currentSolution().probes.at("v:3"));
      il.push_back(eng.deviceCurrent("L1"));
    }
    return std::make_pair(v, il);
  };
  Engine ref;
  build(ref);
  const auto refTail = traceTo(ref, 100e-6);

  Engine rw;
  build(rw);
  (void)traceTo(rw, 60e-6);
  const auto s60 = rw.saveSolverState();
  const auto firstTail = traceTo(rw, 100e-6);
  // Sanity: both runs agreed before any rewind (tails cover steps 60..100).
  ASSERT_EQ(firstTail.first.size(), 40u);
  ASSERT_EQ(refTail.first.size(), 100u);
  for (std::size_t i = 0; i < firstTail.first.size(); ++i) {
    const std::size_t j = i + 60;
    EXPECT_EQ(firstTail.first[i], refTail.first[j]);
    EXPECT_EQ(firstTail.second[i], refTail.second[j]);
  }
  // Rewind to 60us (across the 50us/75us edges) and replay: bitwise match.
  rw.rewindTo(s60, 60e-6);
  EXPECT_EQ(rw.time(), 60e-6);
  const auto replayTail = traceTo(rw, 100e-6);
  ASSERT_EQ(replayTail.first.size(), 40u);  // 40 steps re-stepped
  for (std::size_t i = 0; i < replayTail.first.size(); ++i) {
    const std::size_t j = i + 60;  // replay covers reference steps 60..100
    EXPECT_EQ(replayTail.first[i], refTail.first[j]) << "v step " << i;
    EXPECT_EQ(replayTail.second[i], refTail.second[j]) << "il step " << i;
  }
  EXPECT_NEAR(rw.time(), 100e-6, 1e-12);  // 1ulp summation order, not drift
}

// Shooting through coupled-inductor states (kinds 1+2): PWM-driven RL
// with a loaded secondary reaches a periodic orbit; linear map so Newton
// is exact after the first residual.
TEST(SteadyState, CoupledInductorConverges) {
  constexpr double kT = 50e-6;
  Engine eng;
  eng.setTimeStep(1e-6);
  eng.circuit().addVoltageSource("V1", 1, 0, 10.0);
  eng.circuit().addSwitch("S1", 1, 2, 5e-3, 1e6, true);
  eng.circuit().addCoupledInductors("W1", 2, 0, 3, 0, 200e-6, 200e-6, 0.9);
  eng.circuit().addResistor("Rprim", 2, 0, 50.0);
  eng.circuit().addResistor("Rsec", 3, 0, 50.0);
  eng.scheduleSwitch("S1", true, 0.0);
  eng.scheduleSwitch("S1", false, 0.5 * kT);
  eng.start();
  ShootingConfig cfg;
  cfg.period = kT;
  cfg.maxIters = 20;
  const ShootingResult r = solvePeriodicSteadyState(eng, 0.0, cfg);
  EXPECT_TRUE(r.converged);
  EXPECT_GT(eng.solverStats().steps, 0);
}

// Shooting through a saturable inductor (kind 3 + flux/newton_ik refresh):
// gentle buck (below the knee) converges.
TEST(SteadyState, SaturableInductorConverges) {
  constexpr double kT = 50e-6;
  Engine eng;
  eng.setTimeStep(1e-6);
  eng.circuit().addVoltageSource("Vin", 1, 0, 12.0);
  eng.circuit().addSwitch("S1", 1, 2, 5e-3, 1e6, true);
  eng.circuit().addDiode("D1", 0, 2, 0.0, 10e-3, 1e6);
  eng.circuit().addSaturableInductor("Y1", 2, 3, 200e-6, 20e-6, 2.0, 0.0);
  eng.circuit().addCapacitor("C1", 3, 0, 200e-6, 0.0);
  eng.circuit().addResistor("Rload", 3, 0, 5.0);
  eng.scheduleSwitch("S1", true, 0.0);
  eng.scheduleSwitch("S1", false, 0.5 * kT);
  eng.start();
  ShootingConfig cfg;
  cfg.period = kT;
  cfg.maxIters = 30;
  const ShootingResult r = solvePeriodicSteadyState(eng, 0.0, cfg);
  EXPECT_TRUE(r.converged);
  EXPECT_NEAR(avgOnePeriod(eng, kT), 6.0, 0.05 * 6.0);
}

// Orbit start ahead of now: positions forward with runUntil first.
TEST(SteadyState, FutureT0PositionsForward) {
  constexpr double kT = 50e-6;
  Engine eng;
  buildCcmBuck(eng, 1e-6);
  for (double t = 0.0; t < 4.0 * kT; t += kT) {
    eng.scheduleSwitch("S1", true, t);
    eng.scheduleSwitch("S1", false, t + 0.5 * kT);
  }
  eng.start();
  ShootingConfig cfg;
  cfg.period = kT;
  const ShootingResult r = solvePeriodicSteadyState(eng, 2.0 * kT, cfg);
  EXPECT_TRUE(r.converged);
  EXPECT_NEAR(eng.time(), 2.0 * kT, 1e-9);
  EXPECT_NEAR(avgOnePeriod(eng, kT), 6.0, 0.02 * 6.0);
}

// Graceful non-convergence: one iteration from cold start cannot close
// the orbit; best-effort state rewinds to t0 honestly (no throw).
TEST(SteadyState, NonConvergenceIsHonest) {
  constexpr double kT = 50e-6;
  Engine eng;
  buildCcmBuck(eng, 1e-6);
  eng.scheduleSwitch("S1", true, 0.0);
  eng.scheduleSwitch("S1", false, 0.5 * kT);
  eng.start();
  ShootingConfig cfg;
  cfg.period = kT;
  cfg.maxIters = 1;
  const ShootingResult r = solvePeriodicSteadyState(eng, 0.0, cfg);
  EXPECT_FALSE(r.converged);
  EXPECT_EQ(r.iters, 1);
  EXPECT_TRUE(std::isfinite(r.residual));
  EXPECT_EQ(eng.time(), 0.0);
}

// Validation edges: bad tols, bad t0, t0 behind now.
TEST(SteadyState, MoreBadConfigThrows) {
  Engine eng;
  buildCcmBuck(eng, 1e-6);
  eng.scheduleSwitch("S1", true, 0.0);
  eng.scheduleSwitch("S1", false, 0.5 * 50e-6);
  eng.start();
  ShootingConfig cfg;
  cfg.period = 50e-6;
  ShootingConfig bad = cfg;
  bad.relTol = 0.0;
  EXPECT_THROW(solvePeriodicSteadyState(eng, 0.0, bad), std::runtime_error);
  bad = cfg;
  bad.fdStep = 0.0;
  EXPECT_THROW(solvePeriodicSteadyState(eng, 0.0, bad), std::runtime_error);
  EXPECT_THROW(solvePeriodicSteadyState(eng, -1.0, cfg), std::runtime_error);
  for (int k = 0; k < 100; ++k) eng.step();
  ASSERT_GT(eng.time(), 0.0);
  EXPECT_THROW(solvePeriodicSteadyState(eng, 0.0, cfg), std::runtime_error);
}

// Deep-saturation shooting: Newton overshoots from cold start, forcing
// backtracking halves (and possibly an honest non-converge). Either way
// the line-search machinery is exercised, never a throw.
TEST(SteadyState, DeepSaturationBacktracks) {
  constexpr double kT = 50e-6;
  Engine eng;
  eng.setTimeStep(1e-6);
  eng.circuit().addVoltageSource("Vin", 1, 0, 12.0);
  eng.circuit().addSwitch("S1", 1, 2, 5e-3, 1e6, true);
  eng.circuit().addDiode("D1", 0, 2, 0.0, 10e-3, 1e6);
  eng.circuit().addSaturableInductor("Y1", 2, 3, 200e-6, 20e-6, 0.3, 0.0);
  eng.circuit().addCapacitor("C1", 3, 0, 200e-6, 0.0);
  eng.circuit().addResistor("Rload", 3, 0, 5.0);
  eng.scheduleSwitch("S1", true, 0.0);
  eng.scheduleSwitch("S1", false, 0.5 * kT);
  eng.start();
  ShootingConfig cfg;
  cfg.period = kT;
  cfg.maxIters = 6;
  const ShootingResult r = solvePeriodicSteadyState(eng, 0.0, cfg);
  EXPECT_TRUE(std::isfinite(r.residual));
  EXPECT_EQ(eng.time(), 0.0);
}
