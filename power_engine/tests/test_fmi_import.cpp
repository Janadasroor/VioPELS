// FMI 2.0 Co-Simulation import: XML parse, slave lifecycle, engine coupling.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "power_engine/engine.h"
#include "power_engine/fmi_import.h"

#ifndef FMU_STUB_DIR
#error "FMU_STUB_DIR compile definition missing"
#endif

namespace {

using power_engine::Engine;
using namespace power_engine::fmi_import;

void writeFile(const std::string& path, const std::string& content) {
  std::ofstream f(path, std::ios::binary | std::ios::trunc);
  if (!f) throw std::runtime_error("cannot write " + path);
  f.write(content.data(), static_cast<std::streamsize>(content.size()));
}

// Buck with MANUAL gates (no .control pwm: the imported controller drives S1).
const char* kBuckManual = R"(
.model SW mosfet_ideal RON=5m ROFF=1Meg
.model DD diode_ideal VF=0.0 RON=10m
V1 1 0 12
S1 1 2 MODEL=SW
D1 0 2 MODEL=DD
L1 2 3 200u
C1 3 0 200u
Rload 3 0 5
.tran 0.5u 6m
.end
)";

TEST(FmiImport, ParseStubXml) {
  const ModelDescription md = parseModelDescription(FMU_STUB_DIR);
  EXPECT_EQ(md.modelName, "BangBang");
  EXPECT_EQ(md.modelIdentifier, "fmu_stub");
  EXPECT_EQ(md.variables.size(), 4u);
  const FmuVariable& vout = md.find("vout");
  EXPECT_EQ(vout.valueReference, 0u);
  EXPECT_EQ(vout.causality, "input");
  EXPECT_EQ(vout.type, 'R');
  EXPECT_DOUBLE_EQ(vout.realStart, 0.0);
  EXPECT_EQ(md.find("vref").causality, "parameter");
  EXPECT_DOUBLE_EQ(md.find("vref").realStart, 6.0);
  EXPECT_EQ(md.find("err").causality, "output");
  EXPECT_EQ(md.find("gate").type, 'B');
  EXPECT_THROW(md.find("nope"), std::runtime_error);
}

TEST(FmiImport, ParseRejects) {
  writeFile("fmu_badver_tmp.xml",
            "<fmiModelDescription fmiVersion=\"3.0\"/>");
  // parseModelDescription appends the filename itself; exercise through a
  // scratch FMU dir instead (missing file + wrong version both throw).
  EXPECT_THROW(parseModelDescription("/no/such/dir"), std::runtime_error);
  std::filesystem::create_directories("fmu_bad_tmp");
  writeFile("fmu_bad_tmp/modelDescription.xml",
            "<?xml version=\"1.0\"?>\n<fmiModelDescription fmiVersion=\"3.0\"/>\n");
  EXPECT_THROW(parseModelDescription("fmu_bad_tmp"), std::runtime_error);
  writeFile("fmu_bad_tmp/modelDescription.xml",
            "<fmiModelDescription fmiVersion=\"2.0\"/>\n");
  EXPECT_THROW(parseModelDescription("fmu_bad_tmp"), std::runtime_error);  // no CS
  writeFile("fmu_bad_tmp/modelDescription.xml",
            "<fmiModelDescription fmiVersion=\"2.0\">\n"
            "<CoSimulation modelIdentifier=\"x\"/>\n"
            "<ModelVariables>\n"
            "<ScalarVariable name=\"a\" valueReference=\"0\"/>\n"  // no type
            "</ModelVariables>\n</fmiModelDescription>\n");
  EXPECT_THROW(parseModelDescription("fmu_bad_tmp"), std::runtime_error);
}

TEST(FmiImport, SlaveLifecycle) {
  FmuSlave s(FMU_STUB_DIR);
  EXPECT_THROW(s.doStep(1e-3), std::runtime_error);  // before init
  EXPECT_THROW(s.getReal("err"), std::runtime_error);
  s.enterInit();
  s.setReal("vref", 7.0);
  s.setReal("vout", 0.0);
  s.exitInit();
  EXPECT_DOUBLE_EQ(s.getReal("err"), 7.0);
  EXPECT_TRUE(s.getBoolean("gate"));  // 0 < 7
  s.setReal("vout", 9.0);
  s.doStep(1e-3);
  EXPECT_DOUBLE_EQ(s.time(), 1e-3);
  EXPECT_DOUBLE_EQ(s.getReal("err"), -2.0);
  EXPECT_FALSE(s.getBoolean("gate"));
  EXPECT_THROW(s.setReal("nope", 1.0), std::runtime_error);
  EXPECT_THROW(s.doStep(0.0), std::runtime_error);
  s.terminate();
  EXPECT_THROW(s.doStep(1e-3), std::runtime_error);  // after terminate
}

TEST(FmiImport, SlaveMissingLib) {
  std::filesystem::create_directories("fmu_nolib_tmp");
  writeFile("fmu_nolib_tmp/modelDescription.xml",
            "<fmiModelDescription fmiVersion=\"2.0\" modelName=\"X\" guid=\"g\">\n"
            "<CoSimulation modelIdentifier=\"ghost\"/>\n"
            "</fmiModelDescription>\n");
  EXPECT_THROW(FmuSlave("fmu_nolib_tmp"), std::runtime_error);
}

TEST(FmiImport, CoSimValidatesBinding) {
  FmuSlave s(FMU_STUB_DIR);
  CoSim::Input in;
  in.fmuVar = "vout";
  in.probe = "v:3";
  CoSim::Output bad;
  bad.switchName = "S1";
  bad.fmuVar = "vout";  // input, not output
  EXPECT_THROW(CoSim(s, 1e-3, {in}, {bad}), std::runtime_error);
  CoSim::Input badIn;
  badIn.fmuVar = "err";  // output, not input
  badIn.probe = "v:3";
  CoSim::Output out;
  out.switchName = "S1";
  out.fmuVar = "gate";
  EXPECT_THROW(CoSim(s, 1e-3, {badIn}, {out}), std::runtime_error);
  EXPECT_THROW(CoSim(s, 0.0, {in}, {out}), std::runtime_error);
}

TEST(FmiImport, ExchangeRejectsSkewAndBadProbe) {
  Engine eng;
  eng.loadNetlist(kBuckManual);
  eng.clearStopTime();
  eng.start();
  FmuSlave s(FMU_STUB_DIR);
  s.enterInit();
  s.exitInit();
  CoSim::Input in;
  in.fmuVar = "vout";
  in.probe = "v:3";
  CoSim::Output out;
  out.switchName = "S1";
  out.fmuVar = "gate";
  CoSim co(s, 50e-6, {in}, {out});
  s.doStep(50e-6);  // slave runs ahead: skew must throw, not silently couple
  EXPECT_THROW(co.exchange(eng), std::runtime_error);
}

TEST(FmiImport, ClosedLoopBuckRegulates) {
  Engine eng;
  eng.loadNetlist(kBuckManual);
  eng.clearStopTime();
  std::vector<std::pair<double, double>> vout, vsw;
  eng.setCallback([&](const power_engine::Solution& sol) {
    vout.emplace_back(sol.t, sol.probes.at("v:3"));
    vsw.emplace_back(sol.t, sol.probes.at("v:2"));
  });
  eng.start();
  FmuSlave s(FMU_STUB_DIR);
  s.enterInit();
  s.exitInit();  // vref = 6.0 start
  CoSim::Input in;
  in.fmuVar = "vout";
  in.probe = "v:3";
  CoSim::Output out;
  out.switchName = "S1";
  out.fmuVar = "gate";
  CoSim co(s, 50e-6, {in}, {out});
  const double tEnd = 6e-3;
  while (s.time() < tEnd) {
    co.exchange(eng);      // samples v:3, steps slave, drives S1
    eng.runUntil(s.time());
  }
  s.terminate();
  // Regulation band over the last 2ms (bang-bang around 6V, LC-filtered).
  double sum = 0;
  int n = 0;
  double swMin = 1e9, swMax = -1e9;
  for (const auto& [t, v] : vout)
    if (t >= 4e-3) {
      sum += v;
      ++n;
    }
  for (const auto& [t, v] : vsw)
    if (t >= 4e-3) {
      swMin = std::min(swMin, v);
      swMax = std::max(swMax, v);
    }
  ASSERT_GT(n, 0);
  EXPECT_GT(sum / n, 4.5);
  EXPECT_LT(sum / n, 7.5);
  EXPECT_LT(swMin, 1.0);   // gates actually toggled (diode freewheel seen)
  EXPECT_GT(swMax, 11.0);  // ... and the 12V rail
}

}  // namespace
