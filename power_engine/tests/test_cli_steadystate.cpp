// Tests for pe steady-state (in-process via runSteadyState; deterministic
// fixtures, no spawned processes).
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cmd_steadystate.h"
#include "pe.h"

namespace {

const char* kBuck = R"(
.model SW mosfet_ideal RON=5m ROFF=1Meg
.model DD diode_ideal VF=0.0 RON=10m
V1 1 0 12
S1 1 2 MODEL=SW
D1 0 2 MODEL=DD
L1 2 3 200u
C1 3 0 200u
Rload 3 0 5
.control pwm switch=S1 freq=20k duty=0.5
.tran 0.5u 3m
.end
)";

// Autonomous: constant-source series RLC, no switches, no pwms.
const char* kAutonomous = R"(
V1 1 0 12
L1 1 2 100u
C1 2 3 100u
R1 3 0 10
.tran 1u 10u
.end
)";

const std::string kBuckPath = "pe_cli_steady_buck_tmp.net";
const std::string kAutoPath = "pe_cli_steady_auto_tmp.net";
const std::string kOrbitPath = "pe_cli_steady_orbit_tmp.csv";

void writeFile(const std::string& path, const std::string& content) {
  std::ofstream f(path, std::ios::binary | std::ios::trunc);
  if (!f) throw std::runtime_error("cannot open " + path);
  f.write(content.data(), static_cast<std::streamsize>(content.size()));
}

void removeFile(const std::string& path) { std::remove(path.c_str()); }

std::string readFile(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error("cannot open " + path);
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

int run(std::vector<std::string> argv, std::string& outText, std::string& errText) {
  std::ostringstream out, err;
  const int rc = pe::runSteadyState(pe::parseArgs(argv), out, err);
  outText = out.str();
  errText = err.str();
  return rc;
}

std::vector<std::vector<std::string>> parseCsv(const std::string& csv) {
  std::vector<std::vector<std::string>> rows;
  std::istringstream in(csv);
  std::string line;
  while (std::getline(in, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line.empty()) continue;
    std::vector<std::string> fields;
    std::istringstream row(line);
    std::string cell;
    while (std::getline(row, cell, ',')) fields.push_back(cell);
    rows.push_back(fields);
  }
  return rows;
}

}  // namespace

TEST(CliSteadyState, BuckConvergesAndDefaultsFromPwm) {
  writeFile(kBuckPath, kBuck);
  std::string out, err;
  const int rc = run({"--netlist", kBuckPath}, out, err);
  EXPECT_EQ(rc, 0) << err;
  EXPECT_NE(out.find("converged=1"), std::string::npos) << out;
  EXPECT_NE(out.find("iters="), std::string::npos) << out;
  removeFile(kBuckPath);
}

TEST(CliSteadyState, OrbitCsvCoversExactlyOnePeriod) {
  writeFile(kBuckPath, kBuck);
  std::string out, err;
  const int rc = run({"--netlist", kBuckPath, "--out", kOrbitPath}, out, err);
  ASSERT_EQ(rc, 0) << err;
  ASSERT_FALSE(readFile(kOrbitPath).empty());
  const auto rows = parseCsv(readFile(kOrbitPath));
  ASSERT_GE(rows.size(), 10u);
  // Header: time + probe columns.
  EXPECT_EQ(rows.front().front(), "time");
  std::vector<double> t;
  double sumV3 = 0.0;
  int idxV3 = -1;
  for (size_t c = 0; c < rows.front().size(); ++c) {
    if (rows.front()[c] == "v:3") idxV3 = static_cast<int>(c);
  }
  ASSERT_GE(idxV3, 0);
  for (size_t i = 1; i < rows.size(); ++i) {
    t.push_back(std::stod(rows[i][0]));
    sumV3 += std::stod(rows[i][idxV3]);
  }
  // One period at 20 kHz from an orbit that ends at pop=0: [0, 50e-6).
  EXPECT_LT(t.front(), 1e-6);
  EXPECT_NEAR(t.back(), 50e-6, 0.5e-6);
  // CCM buck: cycle average of v:3 must sit near Vduty = Vin * D = 6 V.
  EXPECT_NEAR(sumV3 / static_cast<double>(t.size()), 6.0, 0.6);
  removeFile(kBuckPath);
  removeFile(kOrbitPath);
}

TEST(CliSteadyState, AutoDerivesPeriodFromSinglePwm) {
  writeFile(kBuckPath, kBuck);
  std::string out, err;
  const int rc = run({"--netlist", kBuckPath, "--auto"}, out, err);
  EXPECT_EQ(rc, 0) << err;
  EXPECT_NE(out.find("converged=1"), std::string::npos) << out;
  removeFile(kBuckPath);
}

TEST(CliSteadyState, AutonomousNetNeedsPeriodAndConverges) {
  writeFile(kAutoPath, kAutonomous);
  std::string out, err;
  int rc = run({"--netlist", kAutoPath}, out, err);
  EXPECT_EQ(rc, 2);  // no pwm, no --period: usage error
  rc = run({"--netlist", kAutoPath, "--period", "1e-5"}, out, err);
  EXPECT_EQ(rc, 0) << err;
  EXPECT_NE(out.find("converged=1"), std::string::npos) << out;
  removeFile(kAutoPath);
}

TEST(CliSteadyState, UsageErrors) {
  writeFile(kBuckPath, kBuck);
  std::string out, err;
  EXPECT_EQ(run({"--period", "10e-6", "--auto", "--netlist", kBuckPath}, out, err), 2);
  EXPECT_EQ(run({"--netlist", kBuckPath, "--pop", "-1"}, out, err), 2);
  EXPECT_EQ(run({"--netlist", kBuckPath, "--period", "0"}, out, err), 2);
  EXPECT_EQ(run({"--netlist", kBuckPath, "--method", "bogus"}, out, err), 2);
  EXPECT_EQ(run({"--netlist", "does_not_exist.net"}, out, err), 2);
  EXPECT_EQ(run({"--netlist", kBuckPath, "--probe", "v:99", "--out", kOrbitPath}, out, err), 2);
  removeFile(kBuckPath);
  removeFile(kOrbitPath);
}
