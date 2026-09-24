// Tests for the pe CLI framework + run command (in-process via the
// exported runNetlist seam: return codes and CSV content, no processes).
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cmd_run.h"
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

void writeFile(const std::string& path, const std::string& content) {
  std::ofstream f(path, std::ios::binary | std::ios::trunc);
  if (!f) throw std::runtime_error("cannot open " + path);
  f.write(content.data(), static_cast<std::streamsize>(content.size()));
  f.close();
  if (!f) throw std::runtime_error("cannot write " + path);
}

std::string readFile(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error("cannot open " + path);
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

void removeFile(const std::string& path) { std::remove(path.c_str()); }

pe::ParsedArgs argsFor(std::vector<std::string> argv) { return pe::parseArgs(argv); }

// Split CSV body into rows of fields (header + data).
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

TEST(CliParse, OptionsFlagsPositionals) {
  const auto a = argsFor({"--netlist", "f.net", "--dt=1e-6", "--verbose", "--param", "R=5",
                          "--param", "C=1u", "pos", "--", "--not-an-opt"});
  EXPECT_EQ(a.get("netlist"), "f.net");
  EXPECT_EQ(a.get("dt"), "1e-6");
  EXPECT_TRUE(a.has("verbose"));
  ASSERT_EQ(a.opts.at("param").size(), 2u);
  EXPECT_EQ(a.opts.at("param")[0], "R=5");
  EXPECT_EQ(a.opts.at("param")[1], "C=1u");
  ASSERT_EQ(a.positionals.size(), 2u);
  EXPECT_EQ(a.positionals[0], "pos");
  EXPECT_EQ(a.positionals[1], "--not-an-opt");
  EXPECT_EQ(a.get("missing", "dflt"), "dflt");
  EXPECT_FALSE(a.has("nope"));
}

TEST(CliParse, MalformedThrows) { EXPECT_THROW(argsFor({"--=x"}), std::runtime_error); }

TEST(CliRegistry, RunIsRegistered) {
  const auto& cmds = pe::commands();
  const auto it = cmds.find("run");
  ASSERT_TRUE(it != cmds.end());
  EXPECT_FALSE(it->second.summary.empty());
  EXPECT_FALSE(std::string(pe::runHelp()).empty());
}

TEST(CliRun, BuckNetlistToCsv) {
  const std::string path = "pe_cli_buck_tmp.net";
  writeFile(path, kBuck);
  std::ostringstream out, err;
  const int rc = pe::runNetlist(argsFor({"--netlist", path}), out, err);
  removeFile(path);
  EXPECT_EQ(rc, 0) << err.str();
  const auto rows = parseCsv(out.str());
  ASSERT_GE(rows.size(), 3u);  // header + settled rows
  EXPECT_EQ(rows[0][0], "time");
  // v:3 column present; settled mean near D*Vin = 6V.
  int vcol = -1;
  for (std::size_t i = 0; i < rows[0].size(); ++i)
    if (rows[0][i] == "v:3") vcol = static_cast<int>(i);
  ASSERT_GE(vcol, 0);
  double sum = 0.0;
  long long n = 0;
  for (std::size_t r = rows.size() / 2; r < rows.size(); ++r) {
    sum += std::stod(rows[r][vcol]);
    ++n;
  }
  EXPECT_NEAR(sum / n, 6.0, 0.05 * 6.0);
}

TEST(CliRun, ProbeFilterAndOutFile) {
  const std::string path = "pe_cli_buck_tmp.net";
  const std::string csvPath = "pe_cli_out_tmp.csv";
  writeFile(path, kBuck);
  std::ostringstream out, err;
  const int rc = pe::runNetlist(
      argsFor({"--netlist", path, "--probe", "v:3", "--out", csvPath, "--tstop", "0.0002"}),
      out, err);
  EXPECT_EQ(rc, 0) << err.str();
  EXPECT_TRUE(out.str().empty());  // file mode: nothing on stdout
  const std::string content = readFile(csvPath);
  removeFile(path.c_str());
  removeFile(csvPath.c_str());
  const auto rows = parseCsv(content);
  ASSERT_GE(rows.size(), 2u);
  EXPECT_EQ(rows[0].size(), 2u);  // time + v:3 only
  EXPECT_EQ(rows[0][1], "v:3");
}

TEST(CliRun, UsageErrorsAreExit2) {
  std::ostringstream out, err;
  EXPECT_EQ(pe::runNetlist(argsFor({}), out, err), 2);  // no --netlist
  EXPECT_EQ(pe::runNetlist(argsFor({"--netlist", "/no/such/file.net"}), out, err), 2);
  const std::string path = "pe_cli_buck_tmp.net";
  writeFile(path, kBuck);
  EXPECT_EQ(pe::runNetlist(argsFor({"--netlist", path, "stray"}), out, err), 2);
  EXPECT_EQ(pe::runNetlist(argsFor({"--netlist", path, "--dt", "abc"}), out, err), 2);
  EXPECT_EQ(pe::runNetlist(argsFor({"--netlist", path, "--dt", "0"}), out, err), 2);
  EXPECT_EQ(pe::runNetlist(argsFor({"--netlist", path, "--method", "rk4"}), out, err), 2);
  EXPECT_EQ(pe::runNetlist(argsFor({"--netlist", path, "--param", "R"}), out, err), 2);
  EXPECT_EQ(pe::runNetlist(argsFor({"--netlist", path, "--param", "R=abc"}), out, err), 2);
  // No stop time anywhere (.tran stripped, no --tstop): incomplete input.
  const std::string path2 = "pe_cli_nostop_tmp.net";
  writeFile(path2, "V1 1 0 5\nR1 1 0 10\n.end\n");
  EXPECT_EQ(pe::runNetlist(argsFor({"--netlist", path2}), out, err), 2);
  removeFile(path);
  removeFile(path2);
}

TEST(CliRun, SimErrorsAreExit1) {
  const std::string path = "pe_cli_buck_tmp.net";
  writeFile(path, kBuck);
  std::ostringstream out, err;
  // Unknown probe (validated against the first solution).
  EXPECT_EQ(pe::runNetlist(argsFor({"--netlist", path, "--probe", "v:99"}), out, err), 1);
  // Unparseable netlist (unknown device letter).
  const std::string path3 = "pe_cli_bad_tmp.net";
  writeFile(path3, "Q1 1 0 0\n");
  EXPECT_EQ(pe::runNetlist(argsFor({"--netlist", path3}), out, err), 1);
  removeFile(path);
  removeFile(path3);
}

TEST(CliRun, ParamOverrideAndMethod) {
  const std::string path = "pe_cli_buck_tmp.net";
  writeFile(path, kBuck);
  std::ostringstream out, err;
  // .tran tstop overridden down; TR-BDF2 integrator selected.
  const int rc = pe::runNetlist(
      argsFor({"--netlist", path, "--tstop", "0.0002", "--method", "trbdf2"}), out, err);
  EXPECT_EQ(rc, 0) << err.str();
  const auto rows = parseCsv(out.str());
  ASSERT_GE(rows.size(), 2u);
  const double tLast = std::stod(rows.back()[0]);
  EXPECT_NEAR(tLast, 0.0002, 1e-9);
  removeFile(path);
}
