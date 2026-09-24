// Tests for pe sweep (in-process via runSweepCommand; deterministic DC
// divider fixture settles in one step, so grids stay instant).
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cmd_sweep.h"
#include "pe.h"

namespace {
const char* kDiv = R"(
.param R1V 10
V1 1 0 12
R1 1 2 {R1V}
R2 2 0 10
.tran 1u 20u
.end
)";
const std::string kDivPath = "pe_cli_sweep_div_tmp.net";

void writeFile(const std::string& path, const std::string& content) {
  std::ofstream f(path, std::ios::binary | std::ios::trunc);
  if (!f) throw std::runtime_error("cannot open " + path);
  f.write(content.data(), static_cast<std::streamsize>(content.size()));
}

std::string runSweep(std::vector<std::string> argv, int& rc, std::string& errText) {
  std::ostringstream out, err;
  rc = pe::runSweepCommand(pe::parseArgs(argv), out, err);
  errText = err.str();
  return out.str();
}

double colMean(const std::string& csv, const std::string& col) {
  std::istringstream in(csv);
  std::string line;
  std::getline(in, line);
  std::istringstream head(line);
  std::string cell;
  int idx = -1, c = 0;
  while (std::getline(head, cell, ',')) {
    if (cell == col) idx = c;
    ++c;
  }
  if (idx < 0) throw std::runtime_error("no column " + col);
  double sum = 0.0;
  long long n = 0;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    std::istringstream row(line);
    int k = 0;
    while (std::getline(row, cell, ',')) {
      if (k == idx && !cell.empty()) {
        sum += std::stod(cell);
        ++n;
      }
      ++k;
    }
  }
  if (n == 0) throw std::runtime_error("no data in " + col);
  return sum / n;
}

}  // namespace

TEST(CliSweep, DividerGridExact) {
  writeFile(kDivPath, kDiv);
  int rc = -1;
  std::string err;
  // v2 = 12*10/(R1+10): 6, 4, 3.
  const std::string csv = runSweep({"--netlist", kDivPath, "--axis", "R1V=10,20,30",
                                    "--measure", "last(v:2)"},
                                   rc, err);
  EXPECT_EQ(rc, 0) << err;
  EXPECT_NEAR(colMean(csv, "R1V"), 20.0, 1e-9);
  std::istringstream in(csv);
  std::string line;
  std::getline(in, line);  // header
  std::vector<double> got;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    got.push_back(std::stod(line.substr(line.find(',') + 1)));
  }
  ASSERT_EQ(got.size(), 3u);
  EXPECT_NEAR(got[0], 6.0, 1e-9);
  EXPECT_NEAR(got[1], 4.0, 1e-9);
  EXPECT_NEAR(got[2], 3.0, 1e-9);
  std::remove(kDivPath.c_str());
}

TEST(CliSweep, RangeAndNamedMeasure) {
  writeFile(kDivPath, kDiv);
  int rc = -1;
  std::string err;
  const std::string csv =
      runSweep({"--netlist", kDivPath, "--axis", "R1V=10:10:30", "--measure",
                "mean(v:2) as vout", "--measure", "min(v:2)", "--measure", "max(v:2)"},
               rc, err);
  EXPECT_EQ(rc, 0) << err;
  EXPECT_NE(csv.find("vout"), std::string::npos);
  EXPECT_NE(csv.find("min_v:2"), std::string::npos);
  EXPECT_NE(csv.find("max_v:2"), std::string::npos);
  EXPECT_NEAR(colMean(csv, "vout"), (6.0 + 4.0 + 3.0) / 3.0, 1e-9);
  std::remove(kDivPath.c_str());
}

TEST(CliSweep, JobsBitwiseIdentical) {
  writeFile(kDivPath, kDiv);
  int rc1 = -1, rc2 = -1;
  std::string e1, e2;
  const std::string s1 = runSweep({"--netlist", kDivPath, "--axis", "R1V=10,15,20,25,30",
                                   "--measure", "last(v:2)", "--jobs", "1"},
                                  rc1, e1);
  const std::string s2 = runSweep({"--netlist", kDivPath, "--axis", "R1V=10,15,20,25,30",
                                   "--measure", "last(v:2)", "--jobs", "2"},
                                  rc2, e2);
  EXPECT_EQ(rc1, 0) << e1;
  EXPECT_EQ(rc2, 0) << e2;
  EXPECT_EQ(s1, s2);  // harness bitwise guarantee, end to end
  std::remove(kDivPath.c_str());
}

TEST(CliSweep, MonteCarloDeterministic) {
  writeFile(kDivPath, kDiv);
  int rc1 = -1, rc2 = -1;
  std::string e1, e2;
  const std::vector<std::string> base = {"--netlist", kDivPath, "--axis",
                                         "R1V=uniform(4,10,40)", "--measure", "last(v:2)"};
  const std::string s1 = runSweep(base, rc1, e1);
  const std::string s2 = runSweep(base, rc2, e2);
  EXPECT_EQ(rc1, 0) << e1;
  EXPECT_EQ(rc2, 0) << e2;
  EXPECT_EQ(s1, s2);
  // Differing seeds differ (streams actually used).
  std::vector<std::string> seeded = base;
  seeded.push_back("--seed");
  seeded.push_back("12345");
  int rc3 = -1;
  std::string e3;
  const std::string s3 = runSweep(seeded, rc3, e3);
  EXPECT_EQ(rc3, 0) << e3;
  EXPECT_NE(s1, s3);
  std::remove(kDivPath.c_str());
}

TEST(CliSweep, RowErrorRecordedUnlessStopOnError) {
  writeFile(kDivPath, kDiv);
  int rc = -1;
  std::string err;
  // R1V=0: zero resistor throws at re-elaboration.
  const std::string csv = runSweep({"--netlist", kDivPath, "--axis", "R1V=10,0",
                                    "--measure", "last(v:2)"},
                                   rc, err);
  EXPECT_EQ(rc, 0);
  EXPECT_NE(csv.find("error"), std::string::npos);
  EXPECT_NE(csv.find("10,"), std::string::npos);  // good row present
  int rc2 = -1;
  std::string err2;
  runSweep({"--netlist", kDivPath, "--axis", "R1V=10,0", "--measure", "last(v:2)",
            "--stop-on-error"},
           rc2, err2);
  EXPECT_EQ(rc2, 1);
  EXPECT_FALSE(err2.empty());
  std::remove(kDivPath.c_str());
}

TEST(CliSweep, YieldAndStatsToStderr) {
  writeFile(kDivPath, kDiv);
  int rc = -1;
  std::string err;
  const std::string csv =
      runSweep({"--netlist", kDivPath, "--axis", "R1V=10,20,30", "--measure", "last(v:2)",
                "--yield", "last_v:2,3.5,6.5", "--stats", "last_v:2"},
               rc, err);
  EXPECT_EQ(rc, 0) << err;
  EXPECT_NE(err.find("yield[last_v:2 in 3.5,6.5] = 0.666667"), std::string::npos);
  EXPECT_NE(err.find("stats[last_v:2]:"), std::string::npos);
  EXPECT_NE(err.find("n=3"), std::string::npos);
  EXPECT_EQ(csv.find("yield"), std::string::npos);  // stdout stays pure CSV
  std::remove(kDivPath.c_str());
}

TEST(CliSweep, UsageErrorsAreExit2) {
  std::ostringstream out, err;
  EXPECT_EQ(pe::runSweepCommand(pe::parseArgs({}), out, err), 2);
  EXPECT_EQ(pe::runSweepCommand(pe::parseArgs({"--netlist", "/no/such.net"}), out, err), 2);
  writeFile(kDivPath, kDiv);
  auto rc = [&](std::vector<std::string> a) {
    a.insert(a.begin(), {"--netlist", kDivPath});
    std::ostringstream o, e;
    return pe::runSweepCommand(pe::parseArgs(a), o, e);
  };
  EXPECT_EQ(rc({"--axis", "R1V"}), 2);
  EXPECT_EQ(rc({"--axis", "R1V=1,abc"}), 2);
  EXPECT_EQ(rc({"--axis", "R1V=1:0:5"}), 2);
  EXPECT_EQ(rc({"--axis", "R1V=nope(1,2,3)"}), 2);
  EXPECT_EQ(rc({"--axis", "R1V=uniform(0,1,2)"}), 2);
  EXPECT_EQ(rc({"--measure", "avg(v:2)"}), 2);
  EXPECT_EQ(rc({"--measure", "last()"}), 2);
  EXPECT_EQ(rc({"--measure", "last(v:99)"}), 2);
  EXPECT_EQ(rc({"--jobs", "-1"}), 2);
  EXPECT_EQ(rc({"--jobs", "257"}), 2);
  EXPECT_EQ(rc({"--yield", "last_v:2,5"}), 2);
  EXPECT_EQ(rc({"--method", "rk4"}), 2);
  EXPECT_EQ(rc({"--param", "X"}), 2);
  // No stop time anywhere.
  writeFile("pe_cli_nostop_tmp.net", "V1 1 0 5\nR1 1 0 10\n.end\n");
  EXPECT_EQ(pe::runSweepCommand(
                pe::parseArgs({"--netlist", "pe_cli_nostop_tmp.net", "--axis", "X=1,2"}), out,
                err),
            2);
  std::remove(kDivPath.c_str());
  std::remove("pe_cli_nostop_tmp.net");
}

TEST(CliSweep, DispatchHelp) {
  pe::registerSweepCommand();
  std::ostringstream out, err;
  EXPECT_EQ(pe::dispatch({"sweep", "--help"}, out, err), 0);
  EXPECT_NE(out.str().find("Usage:"), std::string::npos);
}
