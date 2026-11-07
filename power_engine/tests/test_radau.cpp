// Radau IIA (Phase 2 core, first slice): 3-stage order-5 collocation on
// the MNA DAE. Fixed-step, linear-stamped subset (R/C/L + V/I sources +
// switches/diodes). These tests pin the two claims that matter for the
// slice: (a) the stage plumbing is correct (order-5 accuracy on smooth
// linear circuits, with inductor currents promoted to explicit unknowns),
// and (b) unsupported devices are rejected honestly instead of being
// silently approximated.
#include <cmath>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "power_engine/engine.h"

using power_engine::Engine;
using power_engine::Integrator;

namespace {

// RC charge to 1 V: exact v(t) = 1 - exp(-t/tau), tau = R*C = 1 ms.
double rcError(Integrator method, double dt, double tstop) {
  Engine eng;
  eng.setTimeStep(dt);
  eng.setIntegrator(method);
  eng.circuit().addVoltageSource("V1", 1, 0, 1.0);
  eng.circuit().addResistor("R1", 1, 2, 1000.0);
  eng.circuit().addCapacitor("C1", 2, 0, 1e-6, 0.0);
  eng.setStopTime(tstop);
  eng.start();
  while (eng.status() == power_engine::SimulationStatus::Running) eng.step();
  const double tau = 1e-3;
  const double exact = 1.0 - std::exp(-tstop / tau);
  return std::abs(eng.currentSolution().probes.at("v:2") - exact);
}

}  // namespace

// Order-5 collocation on a smooth linear DAE: at h/tau = 0.02 the error
// is orders below trapezoidal's O(h^2). The 100x margin keeps the test
// about the method, not about constants.
TEST(Radau, BeatsTrapezoidalAtCoarseStep) {
  const double eTrap = rcError(Integrator::Trapezoidal, 2e-5, 1e-3);
  const double eRadau = rcError(Integrator::RadauIIA, 2e-5, 1e-3);
  EXPECT_GT(eTrap, 0.0);
  EXPECT_LT(eRadau, 0.01 * eTrap) << "trap=" << eTrap << " radau=" << eRadau;
}

// Fine-step RC accuracy against the analytic curve (the 5th-order claim
// is already covered above; this pins the absolute value).
TEST(Radau, RcMatchesAnalytic) {
  EXPECT_LT(rcError(Integrator::RadauIIA, 1e-5, 5e-3), 1e-6);
}

// Convergence order: halving dt must shrink the error by ~2^5 = 32x.
// Measured 30.6x; the >16x bound rejects accidental second-order behavior
// while tolerating the first-step startup error.
TEST(Radau, IsFifthOrder) {
  const double e1 = rcError(Integrator::RadauIIA, 8e-5, 1e-3);
  const double e2 = rcError(Integrator::RadauIIA, 4e-5, 1e-3);
  ASSERT_GT(e2, 0.0);
  EXPECT_GT(e1 / e2, 16.0) << "e1=" << e1 << " e2=" << e2;
}

// Inductor current is carried as an explicit DAE unknown in this slice:
// RL step i(t) = 1 - exp(-t/tau), tau = L/R = 1 ms.
TEST(Radau, InductorCurrentMatchesAnalytic) {
  Engine eng;
  eng.setTimeStep(1e-5);
  eng.setIntegrator(Integrator::RadauIIA);
  eng.circuit().addVoltageSource("V1", 1, 0, 1.0);
  eng.circuit().addResistor("R1", 1, 2, 1.0);
  eng.circuit().addInductor("L1", 2, 0, 1e-3, 0.0);
  eng.setStopTime(5e-3);
  eng.start();
  while (eng.status() == power_engine::SimulationStatus::Running) eng.step();
  const double exact = 1.0 - std::exp(-5.0);
  EXPECT_NEAR(eng.deviceCurrent("L1"), exact, 1e-6);
}

// Switched circuit end-to-end: open-loop buck with an ideal diode. The
// gate edges split the run into piecewise-linear intervals; each interval
// gets a Radau step. Output must converge to the CCM average Vin*D.
TEST(Radau, BuckConvergesToDutyAverage) {
  Engine eng;
  eng.loadNetlist(R"(
.model SW mosfet_ideal RON=5m ROFF=1Meg
.model DD diode_ideal VF=0.0 RON=10m
V1 1 0 12
S1 1 2 MODEL=SW
D1 0 2 MODEL=DD
L1 2 3 200u
C1 3 0 200u
Rload 3 0 5
.control pwm switch=S1 freq=20k duty=0.5
.tran 1u 3m
.end
)");
  eng.applyPwmSpecs();
  eng.setIntegrator(Integrator::RadauIIA);
  eng.start();
  while (eng.status() == power_engine::SimulationStatus::Running) eng.step();
  const double vout = eng.currentSolution().probes.at("v:3");
  EXPECT_TRUE(std::isfinite(vout));
  EXPECT_NEAR(vout, 6.0, 1.0);  // CCM average Vin*D with losses
}

// The slice boundary is explicit: saturable magnetics are not silently
// approximated — selecting Radau throws with the device name.
TEST(Radau, RejectsSaturableInductor) {
  Engine eng;
  eng.setTimeStep(1e-6);
  eng.setIntegrator(Integrator::RadauIIA);
  eng.circuit().addVoltageSource("V1", 1, 0, 1.0);
  eng.circuit().addResistor("R1", 1, 2, 10.0);
  eng.circuit().addSaturableInductor("L1", 2, 0, 100e-3, 5e-3, 2.0, 0.0);
  eng.setStopTime(1e-4);
  eng.start();
  try {
    eng.step();
    FAIL() << "expected RadauIIA to reject a saturable inductor";
  } catch (const std::runtime_error& e) {
    EXPECT_NE(std::string(e.what()).find("L1"), std::string::npos);
    EXPECT_NE(std::string(e.what()).find("SatInductor"), std::string::npos);
  }
}

// Adaptive stepping is not implemented for Radau in this slice: fail
// loudly rather than pretend error control exists.
TEST(Radau, RejectsAdaptiveRequest) {
  Engine eng;
  eng.setTimeStep(1e-6);
  eng.setIntegrator(Integrator::RadauIIA);
  eng.circuit().addVoltageSource("V1", 1, 0, 1.0);
  eng.circuit().addResistor("R1", 1, 2, 1000.0);
  eng.circuit().addCapacitor("C1", 2, 0, 1e-6, 0.0);
  eng.start();
  eng.setAdaptive(1e-4, 1e-9, 1e-3);
  EXPECT_THROW(eng.step(), std::runtime_error);
}

// Netlist plumbing: `.tran ... RADAU` selects the integrator through the
// parser (and the engine keeps it after start()).
TEST(Radau, NetlistKeywordSelectsRadau) {
  Engine eng;
  eng.loadNetlist(R"(
V1 1 0 1
R1 1 2 1000
C1 2 0 1u
.tran 10u 1m RADAU
.end
)");
  EXPECT_EQ(eng.integrator(), Integrator::RadauIIA);
  eng.start();
  while (eng.status() == power_engine::SimulationStatus::Running) eng.step();
  EXPECT_NEAR(eng.currentSolution().probes.at("v:2"), 1.0 - std::exp(-1.0), 1e-5);
}
