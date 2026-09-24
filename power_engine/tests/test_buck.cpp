#include <cmath>
#include <gtest/gtest.h>

#include "power_engine/engine.h"

using power_engine::Engine;

// Open-loop buck (CCM): Vin=12V, f=20kHz (T=50us), D=0.5,
// S1 high-side switch, D1 freewheel (Vf=0 ideal), L=200uH, C=200uF, R=5ohm.
// Nodes: 1=Vin+, 2=switch node (L left), 3=Vout.
// Expected steady-state Vout = D*Vin within 2% (ideal, tiny Ron drops).
TEST(BuckOpenLoop, SteadyStateMatchesDutyTheory) {
  constexpr double Vin = 12.0;
  constexpr double freq = 20e3;
  constexpr double period = 1.0 / freq;
  constexpr double duty = 0.5;
  constexpr double dt = 0.5e-6;  // 100 steps/period
  constexpr double tStop = 6e-3;  // 120 periods, >> tau

  Engine eng;
  eng.setTimeStep(dt);
  eng.circuit().addVoltageSource("Vin", 1, 0, Vin);
  eng.circuit().addSwitch("S1", 1, 2, 5e-3, 1e6, true);
  eng.circuit().addDiode("D1", 0, 2, 0.0, 10e-3, 1e6);  // anode gnd, cathode node2
  eng.circuit().addInductor("L1", 2, 3, 200e-6, 0.0);
  eng.circuit().addCapacitor("C1", 3, 0, 200e-6, 0.0);
  eng.circuit().addResistor("Rload", 3, 0, 5.0);
  eng.setStopTime(tStop);
  eng.start();

  // Gate drive aligned to step boundaries: closed for D*T at each period start.
  double voutSum = 0.0;
  long long voutCount = 0;
  double voutMin = 1e9, voutMax = -1e9;
  while (eng.status() == power_engine::SimulationStatus::Running) {
    const double tNext = eng.currentSolution().t + dt;
    const double phase = std::fmod(tNext, period);
    eng.setSwitch("S1", phase < duty * period);
    eng.step();
    const double t = eng.currentSolution().t;
    const double vout = eng.currentSolution().probes.at("v:3");
    ASSERT_TRUE(std::isfinite(vout)) << "t=" << t;
    if (t > tStop - period) {  // average over last switching period
      voutSum += vout;
      ++voutCount;
      if (vout < voutMin) voutMin = vout;
      if (vout > voutMax) voutMax = vout;
    }
  }
  ASSERT_GT(voutCount, 0);
  const double voutAvg = voutSum / static_cast<double>(voutCount);
  const double expected = duty * Vin;  // 6V
  EXPECT_NEAR(voutAvg, expected, 0.02 * expected)
      << "avg=" << voutAvg << " expected=" << expected;

  // Sanity: CCM ripple small, inductor current stays positive.
  EXPECT_LT(voutMax - voutMin, 0.15 * expected) << "excessive ripple";
  EXPECT_GT(eng.solverStats().diodeEvents, 0) << "diode never commutated";
  EXPECT_GT(eng.solverStats().steps, 1000);
}

// Exact-time scheduled edges with a step that does NOT divide the PWM period:
// dt=3us vs T=50us. All gate edges are scheduled upfront; step() must split
// into sub-steps so transitions land exactly (no step quantization).
TEST(BuckScheduledEvents, NonAlignedDtMatchesDutyTheory) {
  constexpr double Vin = 12.0;
  constexpr double freq = 20e3;
  constexpr double period = 1.0 / freq;
  constexpr double duty = 0.5;
  constexpr double dt = 3e-6;  // does not divide 50us
  constexpr double tStop = 6e-3;

  Engine eng;
  eng.setTimeStep(dt);
  eng.circuit().addVoltageSource("Vin", 1, 0, Vin);
  eng.circuit().addSwitch("S1", 1, 2, 5e-3, 1e6, true);
  eng.circuit().addDiode("D1", 0, 2, 0.0, 10e-3, 1e6);
  eng.circuit().addInductor("L1", 2, 3, 200e-6, 0.0);
  eng.circuit().addCapacitor("C1", 3, 0, 200e-6, 0.0);
  eng.circuit().addResistor("Rload", 3, 0, 5.0);
  eng.setStopTime(tStop);
  // Schedule the full PWM train upfront (exact period boundaries).
  for (double t = 0.0; t < tStop; t += period) {
    eng.scheduleSwitch("S1", true, t);
    if (t + duty * period < tStop) eng.scheduleSwitch("S1", false, t + duty * period);
  }
  const std::size_t nEvents = eng.pendingEventCount();
  ASSERT_GT(nEvents, 100u);
  eng.start();

  double voutSum = 0.0;
  long long voutCount = 0;
  while (eng.status() == power_engine::SimulationStatus::Running) {
    eng.step();  // no manual setSwitch: edges fire at exact times inside step()
    const double t = eng.currentSolution().t;
    const double vout = eng.currentSolution().probes.at("v:3");
    ASSERT_TRUE(std::isfinite(vout)) << "t=" << t;
    if (t > tStop - period) {
      voutSum += vout;
      ++voutCount;
    }
  }
  ASSERT_GT(voutCount, 0);
  EXPECT_EQ(eng.pendingEventCount(), 0u) << "missed scheduled switching events";
  const double voutAvg = voutSum / static_cast<double>(voutCount);
  const double expected = duty * Vin;
  EXPECT_NEAR(voutAvg, expected, 0.02 * expected)
      << "avg=" << voutAvg << " expected=" << expected;
  EXPECT_GT(eng.solverStats().diodeEvents, 0);
}

// Event cursor rewind (item 14d): an edge scheduled mid-run earlier than
// the next pending edge (backdated by a slow controller) rewinds the
// cursor and fires as due. Without the rewind it would sit unapplied
// forever (stale cursor skips it), leaving a pending edge behind.
TEST(BuckScheduledEvents, BackdatedMidRunScheduleFires) {
  Engine eng;
  eng.setTimeStep(3e-6);
  eng.circuit().addVoltageSource("Vin", 1, 0, 12.0);
  eng.circuit().addSwitch("S1", 1, 2, 5e-3, 1e6, true);
  eng.circuit().addDiode("D1", 0, 2, 0.0, 10e-3, 1e6);
  eng.circuit().addInductor("L1", 2, 3, 200e-6, 0.0);
  eng.circuit().addCapacitor("C1", 3, 0, 200e-6, 0.0);
  eng.circuit().addResistor("Rload", 3, 0, 5.0);
  eng.scheduleSwitch("S1", false, 0.2e-3);
  eng.scheduleSwitch("S1", true, 0.4e-3);
  eng.scheduleSwitch("S1", false, 1.0e-3);
  eng.setStopTime(2e-3);
  eng.start();
  while (eng.status() == power_engine::SimulationStatus::Running && eng.time() < 0.5e-3)
    eng.step();
  ASSERT_TRUE(eng.status() == power_engine::SimulationStatus::Running);
  ASSERT_TRUE(eng.circuit().switchClosed("S1"));  // 0.4ms ON edge fired
  eng.scheduleSwitch("S1", false, 0.35e-3);  // backdated past fired edges: rewinds cursor
  eng.step();
  EXPECT_FALSE(eng.circuit().switchClosed("S1"));  // fired immediately as due
  while (eng.status() == power_engine::SimulationStatus::Running) eng.step();
  EXPECT_EQ(eng.pendingEventCount(), 0u);
  EXPECT_FALSE(eng.circuit().switchClosed("S1"));
}
