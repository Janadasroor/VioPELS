// pe run: netlist in, CSV out (ngspice -b style).
//
//   pe run --netlist <file> [--out <csv>] [--tstop <s>] [--dt <s>]
//          [--method trap|trbdf2|auto] [--param K=V ...] [--probe <key> ...]
//
// Loads the netlist, applies .param overrides, optional .tran overrides,
// expands .control pwm / .thermal / .etable specs found in the file,
// runs to the stop time, and writes time + probes as CSV (stdout default).
// Exit codes: 0 ok, 1 sim/runtime error, 2 usage or unreadable input.
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "pe.h"
#include "cmd_run.h"
#include "power_engine/engine.h"

namespace {

const char kHelp[] = R"(pe run: simulate a netlist, print CSV.
Usage: pe run --netlist <file> [options]

Options:
  --netlist <file>   netlist to simulate (required)
  --out <csv>        write CSV here (default: stdout)
  --tstop <s>        stop time, overrides .tran tstop
  --dt <s>           time step, overrides .tran dt
  --method <m>       trap (default) | trbdf2 | auto
  --param K=V        .param override (repeatable)
  --probe <key>      probe column like v:3 (repeatable; default: all probes)
  --help             this text

CSV columns: time,<probes in sorted order>. Exit codes: 0 ok,
1 sim/runtime error, 2 usage error or unreadable input.
)";

bool readFile(const std::string& path, std::string& out, std::string& why) {
  std::ifstream f(path, std::ios::binary);
  if (!f) {
    why = "cannot open file '" + path + "'";
    return false;
  }
  std::ostringstream ss;
  ss << f.rdbuf();
  if (f.bad()) {
    why = "error reading file '" + path + "'";
    return false;
  }
  out = ss.str();
  return true;
}

bool parseDouble(const std::string& s, double& v) {
  try {
    std::size_t n = 0;
    v = std::stod(s, &n);
    if (n != s.size() || !std::isfinite(v)) return false;
  } catch (...) {
    return false;
  }
  return true;
}

int runNetlistImpl(const pe::ParsedArgs& args, std::ostream& out, std::ostream& err) {
  if (!args.has("netlist") || args.positionals.size() > 0) {
    err << "pe run: --netlist <file> is required (no positionals)\n";
    return 2;
  }
  const std::string path = args.get("netlist");
  std::string text, why;
  if (!readFile(path, text, why)) {
    err << "pe run: " << why << "\n";
    return 2;
  }
  double dt = 0.0, tstop = 0.0;
  if (args.has("dt") && (!parseDouble(args.get("dt"), dt) || !(dt > 0.0))) {
    err << "pe run: bad --dt '" << args.get("dt") << "' (finite > 0)\n";
    return 2;
  }
  if (args.has("tstop") && (!parseDouble(args.get("tstop"), tstop) || !(tstop > 0.0))) {
    err << "pe run: bad --tstop '" << args.get("tstop") << "' (finite > 0)\n";
    return 2;
  }
  const std::string method = args.get("method", "trap");
  if (method != "trap" && method != "trbdf2" && method != "auto") {
    err << "pe run: bad --method '" << method << "' (trap|trbdf2|auto)\n";
    return 2;
  }
  // --param K=V pairs (split on first '='; V parsed now for exit-2 errors).
  std::vector<std::pair<std::string, double>> params;
  auto pit = args.opts.find("param");
  if (pit != args.opts.end()) {
    for (const auto& p : pit->second) {
      const auto eq = p.find('=');
      double v = 0.0;
      if (eq == std::string::npos || eq == 0 || !parseDouble(p.substr(eq + 1), v) ||
          !(std::isfinite(v))) {
        err << "pe run: bad --param '" << p << "' (want K=finite-number)\n";
        return 2;
      }
      params.emplace_back(p.substr(0, eq), v);
    }
  }
  const std::vector<std::string> wantProbes =
      args.has("probe") ? args.opts.at("probe") : std::vector<std::string>{};

  try {
    power_engine::Engine eng;
    eng.loadNetlist(text);
    for (const auto& [k, v] : params) eng.setParameter(k, v);
    if (dt > 0.0) eng.setTimeStep(dt);
    if (tstop > 0.0) eng.setStopTime(tstop);
    if (!(eng.stopTime() > 0.0)) {
      err << "pe run: no stop time (.tran tstop or --tstop)\n";
      return 2;
    }
    if (method == "trbdf2") {
      eng.setIntegrator(power_engine::Integrator::TrBdf2);
    } else if (method == "auto") {
      eng.setIntegratorAuto(true);
    }
    if (!eng.netlist().pwms.empty()) eng.applyPwmSpecs();
    if (!eng.netlist().thermals.empty()) eng.applyThermalSpecs();
    eng.applyLossModels();
    eng.start();
    // Probe columns: requested subset, else everything (sorted: std::map).
    std::vector<std::string> cols;
    bool headerDone = false;
    std::string csv;
    csv.reserve(1 << 20);
    char buf[64];
    while (eng.status() == power_engine::SimulationStatus::Running) {
      eng.step();
      const auto& sol = eng.currentSolution();
      if (!headerDone) {
        if (!wantProbes.empty()) {
          for (const auto& p : wantProbes) {
            if (sol.probes.find(p) == sol.probes.end()) {
              err << "pe run: unknown probe '" << p << "' (have";
              for (const auto& [k, v] : sol.probes) {
                (void)v;
                err << " " << k;
              }
              err << ")\n";
              return 1;
            }
          }
          cols = wantProbes;
        } else {
          for (const auto& [k, v] : sol.probes) {
            (void)v;
            cols.push_back(k);
          }
        }
        csv += "time";
        for (const auto& c : cols) {
          csv += ",";
          csv += c;
        }
        csv += "\n";
        headerDone = true;
      }
      std::snprintf(buf, sizeof(buf), "%.9f", sol.t);
      csv += buf;
      for (const auto& c : cols) {
        std::snprintf(buf, sizeof(buf), "%.10g", sol.probes.at(c));
        csv += ",";
        csv += buf;
      }
      csv += "\n";
    }
    const std::string outPath = args.get("out");
    if (!outPath.empty()) {
      std::ofstream f(outPath, std::ios::binary | std::ios::trunc);
      if (!f) {
        err << "pe run: cannot open --out '" << outPath + "'\n";
        return 1;
      }
      f << csv;
      if (!f) {
        err << "pe run: error writing --out '" << outPath + "'\n";
        return 1;
      }
    } else {
      out << csv;
    }
  } catch (const std::exception& e) {
    err << "pe run: " << e.what() << "\n";
    return 1;
  }
  return 0;
}

[[maybe_unused]] const bool kRegistered = ([] {
  pe::Command c;
  c.name = "run";
  c.summary = "simulate a netlist, print CSV";
  c.help = kHelp;
  c.run = [](const pe::ParsedArgs& args, std::ostream& o, std::ostream& e) {
    return runNetlistImpl(args, o, e);
  };
  pe::registerCommand(std::move(c));
  return true;
})();

}  // namespace

namespace pe {

const char* runHelp() { return kHelp; }

int runNetlist(const ParsedArgs& args, std::ostream& out, std::ostream& err) {
  return runNetlistImpl(args, out, err);
}

void registerRunCommand() {
  // Referencing kRegistered forces the TU (and its static registration)
  // into the link; without this, archive member selection drops it.
  (void)kRegistered;
}

}  // namespace pe
