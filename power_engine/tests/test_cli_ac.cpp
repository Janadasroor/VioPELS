// Tests for pe ac (in-process via runAcCommand; deterministic fixtures,
// no spawned processes). Bode accuracy is checked against analytic plants,
// the way test_ac.cpp does it at the library level.
#include <cmath>
#include <cstdio>
#include <fstream>
#include <numbers>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cmd_ac.h"
#include "pe.h"

namespace {

const char* kRc = R"(
V1 1 0 1.0
R1 1 2 1k
C1 2 0 1u
.tran 5u 10m
.end
)";

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

const std::string kRcPath = "pe_cli_ac_rc_tmp.net";
const std::string kBuckPath = "pe_cli_ac_buck_tmp.net";

void writeFile(const std::string& path, const std::string& content) {
  std::ofstream f(path, std::ios::binary | std::ios::trunc);
  if (!f) throw std::runtime_error("cannot open " + path);
  f.write(content.data(), static_cast<std::streamsize>(content.size()));
}

void removeFile(const std::string& path) { std::remove(path.c_str()); }

int run(std::vector<std::string> argv, std::string& outText, std::string& errText) {
  std::ostringstream out, err;
  const int rc = pe::runAc(pe::parseArgs(argv), out, err);
  outText = out.str();
  errText = err.str();
  return rc;
}

struct BodeRow {
  double freq;
  double magDb;
  double phaseDeg;
};

std::vector<BodeRow> parseBode(const std::string& csv, const std::string& probe) {
  std::istringstream in(csv);
  std::string line;
  if (!std::getline(in, line)) return {};
  std::istringstream head(line);
  std::string cell;
  std::vector<std::string> cols;
  while (std::getline(head, cell, ',')) cols.push_back(cell);
  const std::string magCol = probe + "_mag_db", phCol = probe + "_phase_deg";
  int iMag = -1, iPh = -1;
  for (size_t c = 0; c < cols.size(); ++c) {
    if (cols[c] == magCol) iMag = static_cast<int>(c);
    if (cols[c] == phCol) iPh = static_cast<int>(c);
  }
  std::vector<BodeRow> rows;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    std::istringstream row(line);
    std::vector<std::string> f;
    while (std::getline(row, cell, ',')) f.push_back(cell);
    if (iMag < 0 || iPh < 0 || static_cast<int>(f.size()) <= std::max(iMag, iPh)) continue;
    rows.push_back({std::stod(f[0]), std::stod(f[iMag]), std::stod(f[iPh])});
  }
  return rows;
}

}  // namespace

TEST(CliAc, RcBodeMatchesAnalytical) {
  writeFile(kRcPath, kRc);
  std::string out, err;
  const int rc = run({"--netlist", kRcPath, "--probe", "v:2", "--from", "100",
                      "--to", "10000", "--points", "9", "--settle", "6e-3",
                      "--dt", "1e-6"},
                     out, err);
  ASSERT_EQ(rc, 0) << err;
  const auto rows = parseBode(out, "v:2");
  ASSERT_EQ(rows.size(), 9u);
  ASSERT_TRUE(rows.front().freq < rows.back().freq);
  constexpr double kFc = 1.0 / (2.0 * std::numbers::pi * 1e3 * 1e-6);
  for (const auto& r : rows) {
    const double mag = 1.0 / std::sqrt(1.0 + std::pow(r.freq / kFc, 2.0));
    const double phase = -std::atan(r.freq / kFc) * 180.0 / std::numbers::pi;
    EXPECT_NEAR(r.magDb, 20.0 * std::log10(mag), 1.5) << "f=" << r.freq;
    EXPECT_NEAR(r.phaseDeg, phase, 8.0) << "f=" << r.freq;
  }
  removeFile(kRcPath);
}

TEST(CliAc, BuckDutyBodeShowsResonance) {
  writeFile(kBuckPath, kBuck);
  std::string out, err;
  const int rc = run({"--netlist", kBuckPath, "--inject", "duty:S1", "--probe", "v:3",
                      "--from", "100", "--to", "5000", "--points", "9",
                      "--settle", "5e-3"},
                     out, err);
  ASSERT_EQ(rc, 0) << err;
  const auto rows = parseBode(out, "v:3");
  ASSERT_EQ(rows.size(), 9u);
  // Analytic plant Gvd = Vin/(1 + sL/R + s^2 LC): DC ~ 21.7 dB, peak
  // ~27.5 dB near 571 Hz (lightly damped).
  ASSERT_TRUE(rows.front().freq < 110.0);
  EXPECT_NEAR(rows.front().magDb, 20.0 * std::log10(12.0), 2.0);
  double peak = 0.0;
  for (const auto& r : rows) peak = std::max(peak, r.magDb);
  EXPECT_GT(peak, 24.0);  // resonance peak above the flat gain
  // Deep attenuation beyond 3 kHz.
  double magHigh = 0.0;
  for (const auto& r : rows)
    if (r.freq > 2500.0) magHigh = std::min(magHigh == 0.0 ? 1e9 : magHigh, r.magDb);
  EXPECT_LT(magHigh, 0.0);
  removeFile(kBuckPath);
}

TEST(CliAc, MultitoneTracksStepped) {
  writeFile(kRcPath, kRc);
  std::string outStep, errStep, outMt, errMt;
  const int rcS = run({"--netlist", kRcPath, "--probe", "v:2", "--from", "10",
                       "--to", "1000", "--points", "6", "--settle", "6e-3",
                       "--method", "stepped"},
                      outStep, errStep);
  ASSERT_EQ(rcS, 0) << errStep;
  const int rcM = run({"--netlist", kRcPath, "--probe", "v:2", "--from", "10",
                       "--to", "1000", "--points", "6", "--settle", "6e-3",
                       "--method", "multitone"},
                      outMt, errMt);
  ASSERT_EQ(rcM, 0) << errMt;
  const auto stepRows = parseBode(outStep, "v:2");
  const auto mtRows = parseBode(outMt, "v:2");
  ASSERT_EQ(stepRows.size(), mtRows.size());
  for (size_t i = 0; i < mtRows.size(); ++i) {
    // Multitone snaps onto the bin grid; compare at shared frequencies.
    EXPECT_NEAR(stepRows[i].magDb, mtRows[i].magDb, 2.0)
        << "bin i=" << i;
  }
  removeFile(kRcPath);
}

TEST(CliAc, DutyInjectionRequiresPwmAndAliasHeadroom) {
  // Autonomous net (no .control pwm): duty injection must be refused.
  const char* kAuto = R"(
V1 1 0 12
L1 1 2 100u
C1 2 3 100u
R1 3 0 10
.tran 1u 10u
.end
)";
  writeFile(kRcPath, kAuto);
  std::string out, err;
  EXPECT_EQ(run({"--netlist", kRcPath, "--inject", "duty:S1"}, out, err), 2);
  removeFile(kRcPath);
  // Pwm net: tones at or above pwm/2 must be refused.
  writeFile(kBuckPath, kBuck);
  EXPECT_EQ(run({"--netlist", kBuckPath, "--inject", "duty:S1", "--from", "9000",
                 "--to", "20000", "--points", "3"},
                out, err),
            2);
  removeFile(kBuckPath);
}

TEST(CliAc, UsageErrors) {
  writeFile(kRcPath, kRc);
  std::string out, err;
  EXPECT_EQ(run({"--netlist", kRcPath, "--to", "5"}, out, err), 2);          // from>=to
  EXPECT_EQ(run({"--netlist", kRcPath, "--points", "1"}, out, err), 2);       // points<2
  EXPECT_EQ(run({"--netlist", kRcPath, "--probe", "v:99"}, out, err), 2);     // unknown probe
  EXPECT_EQ(run({"--netlist", kRcPath, "--inject", "src:X9"}, out, err), 2);  // unknown device
  EXPECT_EQ(run({"--netlist", kRcPath, "--inject", "bogus:V1"}, out, err), 2);  // bad mode
  EXPECT_EQ(run({"--netlist", kRcPath, "--method", "flat"}, out, err), 2);    // bad method
  EXPECT_EQ(run({"--netlist", "pe_cli_missing.net"}, out, err), 2);           // missing file
  removeFile(kRcPath);
}
