// pe steady-state: periodic steady state of a switching netlist.
//
//   pe steady-state --netlist <file> [--period <s> | --auto]
//                     [--pop N] [--dt <s>] [--method trap|trbdf2|auto]
//                     [--param K=V ...] [--max-iters N] [--rel-tol x]
//                     [--abs-tol x] [--probe <key> ...] [--out <csv>]
//
// Newton on the period map (see power_engine/steadystate.h). The netlist
// needs at least one inductor or capacitor (the continuous states Newton
// solves for); diodes re-settle inside every period simulation. Gate
// edges come from `.control pwm` (expand over the analysis horizon with
// --pop); a netlist with no PWM is an autonomous circuit and keeps its
// initial switch state. `.tran tstop` is ignored: this command sets its
// own horizons: POP pre-run over [0, pop*period], orbit recorded over
// [pop*period, (pop+1)*period].
//
// stdout: `converged=<0|1> iters=<n> residual=<r>` (always). --out writes
// the steady orbit as time,<probes> CSV (default: all probes, sorted).
// Exit codes: 0 converged, 1 sim/runtime error or Newton non-convergence
// (best-effort orbit is what --out records; check converged=0), 2 usage.
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "cmd_steadystate.h"
#include "pe.h"
#include "power_engine/engine.h"
#include "power_engine/steadystate.h"

namespace {

const char kHelp[] = R"(pe steady-state: periodic steady state (Newton shooting).
Usage: pe steady-state --netlist <file> [options]

Options:
  --netlist <file>   netlist to analyze (required)
  --period <s>       orbit period [s], must be > 0
  --auto             derive period from .control pwm (all pwms must share
                     one frequency; T = 1/f). Conflicts with --period.
  --pop N            run N cold-start periods before Newton (pre-charge;
                     default 0). Edges are expanded over [0, (N+1)*T).
  --dt <s>           time step, overrides .tran dt
  --method <m>       trap (default) | trbdf2 | auto
  --param K=V        .param override (repeatable)
  --max-iters N      Newton iteration cap (default 25)
  --rel-tol x        per-component relative tolerance (default 1e-6)
  --abs-tol x        per-component absolute tolerance (default 1e-9)
  --probe <key>      probe column like v:3 (repeatable; default: all)
  --out <csv>        write the steady orbit CSV here (time + probes)
  --help             this text

stdout: converged=<0|1> iters=<n> residual=<r>
Exit codes: 0 converged, 1 sim/runtime error or non-convergence,
2 usage error or unreadable input.

Notes: hysteretic-inductor nets are rejected (memory has no state slot).
Thermal networks are frozen over one period (timescale-separated); use
pe run / .thermal for electro-thermal soak.
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

bool parseLong(const std::string& s, long long& v) {
  try {
    std::size_t n = 0;
    v = std::stoll(s, &n);
    if (n != s.size()) return false;
  } catch (...) {
    return false;
  }
  return true;
}

int steadyStateImpl(const pe::ParsedArgs& args, std::ostream& out, std::ostream& err) {
  if (!args.has("netlist") || args.positionals.size() > 0) {
    err << "pe steady-state: --netlist <file> is required (no positionals)\n";
    return 2;
  }
  const std::string path = args.get("netlist");
  std::string text, why;
  if (!readFile(path, text, why)) {
    err << "pe steady-state: " << why << "\n";
    return 2;
  }
  double dt = 0.0;
  if (args.has("dt") && (!parseDouble(args.get("dt"), dt) || !(dt > 0.0))) {
    err << "pe steady-state: bad --dt '" << args.get("dt") << "' (finite > 0)\n";
    return 2;
  }
  double period = 0.0;
  const bool havePeriod = args.has("period");
  if (havePeriod && (!parseDouble(args.get("period"), period) || !(period > 0.0))) {
    err << "pe steady-state: bad --period '" << args.get("period") << "' (finite > 0)\n";
    return 2;
  }
  const bool autoPeriod = args.has("auto");
  if (havePeriod && autoPeriod) {
    err << "pe steady-state: --period and --auto conflict\n";
    return 2;
  }
  long long pop = 0;
  if (args.has("pop") && (!parseLong(args.get("pop"), pop) || pop < 0)) {
    err << "pe steady-state: bad --pop '" << args.get("pop") << "' (integer >= 0)\n";
    return 2;
  }
  double relTol = 1e-6, absTol = 1e-9;
  long long maxIters = 25;
  if (args.has("rel-tol") && (!parseDouble(args.get("rel-tol"), relTol) || !(relTol > 0.0))) {
    err << "pe steady-state: bad --rel-tol '" << args.get("rel-tol") << "'\n";
    return 2;
  }
  if (args.has("abs-tol") && (!parseDouble(args.get("abs-tol"), absTol) || !(absTol > 0.0))) {
    err << "pe steady-state: bad --abs-tol '" << args.get("abs-tol") << "'\n";
    return 2;
  }
  if (args.has("max-iters") && (!parseLong(args.get("max-iters"), maxIters) || maxIters <= 0)) {
    err << "pe steady-state: bad --max-iters '" << args.get("max-iters") << "' (integer > 0)\n";
    return 2;
  }
  const std::string method = args.get("method", "trap");
  if (method != "trap" && method != "trbdf2" && method != "auto") {
    err << "pe steady-state: bad --method '" << method
        << "' (trap|trbdf2|auto)\n";
    return 2;
  }
  std::vector<std::pair<std::string, double>> params;
  auto pit = args.opts.find("param");
  if (pit != args.opts.end()) {
    for (const auto& p : pit->second) {
      const auto eq = p.find('=');
      double v = 0.0;
      if (eq == std::string::npos || eq == 0 || !parseDouble(p.substr(eq + 1), v)) {
        err << "pe steady-state: bad --param '" << p << "' (want K=finite-number)\n";
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
    // dt must be known before start; the netlist .tran is the default.
    if (dt > 0.0) eng.setTimeStep(dt);
    if (!(dt > 0.0) && !eng.netlist().tran.given) {
      err << "pe steady-state: no time step (.tran dt or --dt)\n";
      return 2;
    }
    // Period resolution.
    if (!havePeriod && !autoPeriod) {
      // Single-frequency .control pwm nets can default to --auto behavior;
      // everything else must say --period.
      if (!eng.netlist().pwms.empty()) {
        const double f0 = eng.netlist().pwms.front().freq;
        bool same = true;
        for (const auto& p : eng.netlist().pwms) {
          if (std::fabs(p.freq - f0) > 1e-9 * f0) same = false;
        }
        if (same && f0 > 0.0) {
          period = 1.0 / f0;
        } else {
          err << "pe steady-state: pwm switches disagree on freq; pass --period\n";
          return 2;
        }
      } else {
        err << "pe steady-state: need --period (or --auto for a single-freq .control pwm net)\n";
        return 2;
      }
    } else if (autoPeriod) {
      if (eng.netlist().pwms.empty()) {
        err << "pe steady-state: --auto needs a .control pwm net (none found)\n";
        return 2;
      }
      const double f0 = eng.netlist().pwms.front().freq;
      for (const auto& p : eng.netlist().pwms) {
        if (std::fabs(p.freq - f0) > 1e-9 * f0) {
          err << "pe steady-state: --auto needs one pwm freq; pass --period\n";
          return 2;
        }
      }
      if (!(f0 > 0.0)) {
        err << "pe steady-state: bad pwm freq in netlist\n";
        return 2;
      }
      period = 1.0 / f0;
    }
    const double t0 = static_cast<double>(pop) * period;
    // Gate edges must exist for [t0, t0+period]; applyPwmSpecs expands over
    // [0, tStop), so set a temporary horizon covering the POP pre-run.
    eng.setStopTime(static_cast<double>(pop + 1) * period);
    if (method == "trbdf2") {
      eng.setIntegrator(power_engine::Integrator::TrBdf2);
    } else if (method == "auto") {
      eng.setIntegratorAuto(true);
    }
    if (!eng.netlist().pwms.empty()) eng.applyPwmSpecs();
    if (!eng.netlist().thermals.empty()) eng.applyThermalSpecs();
    eng.applyLossModels();
    eng.clearStopTime();  // shooting needs no stop time
    eng.start();

    power_engine::steadystate::ShootingConfig cfg;
    cfg.period = period;
    cfg.maxIters = static_cast<int>(maxIters);
    cfg.relTol = relTol;
    cfg.absTol = absTol;
    const auto r = power_engine::steadystate::solvePeriodicSteadyState(eng, t0, cfg);
    out << "converged=" << (r.converged ? 1 : 0) << " iters=" << r.iters
        << " residual=" << r.residual << "\n";

    const std::string outPath = args.get("out");
    if (!outPath.empty()) {
      // Record the steady orbit over [t0, t0+period] via the step callback.
      struct Row {
        double t;
        std::map<std::string, double> probes;
      };
      std::vector<Row> rows;
      eng.setCallback([&rows](const power_engine::Solution& s) {
        rows.push_back({s.t, s.probes});
      });
      eng.runUntil(t0 + period);
      // Columns: requested subset (validated), else all probes, sorted.
      std::vector<std::string> cols = wantProbes;
      if (!cols.empty()) {
        for (const auto& p : cols) {
          if (!rows.empty() && rows.front().probes.find(p) == rows.front().probes.end()) {
            err << "pe steady-state: unknown probe '" << p << "'\n";
            return 2;
          }
        }
      } else if (!rows.empty()) {
        for (const auto& [k, v] : rows.front().probes) {
          (void)v;
          cols.push_back(k);
        }
      }
      std::string csv = "time";
      for (const auto& c : cols) csv += "," + c;
      csv += "\n";
      char buf[64];
      for (const auto& row : rows) {
        std::snprintf(buf, sizeof(buf), "%.9f", row.t);
        csv += buf;
        for (const auto& c : cols) {
          std::snprintf(buf, sizeof(buf), "%.10g", row.probes.at(c));
          csv += ",";
          csv += buf;
        }
        csv += "\n";
      }
      std::ofstream f(outPath, std::ios::binary | std::ios::trunc);
      if (!f) {
        err << "pe steady-state: cannot open --out '" << outPath << "'\n";
        return 1;
      }
      f << csv;
      if (!f) {
        err << "pe steady-state: error writing --out '" << outPath << "'\n";
        return 1;
      }
    }
    return r.converged ? 0 : 1;
  } catch (const std::exception& e) {
    err << "pe steady-state: " << e.what() << "\n";
    return 1;
  }
}

[[maybe_unused]] const bool kRegistered = ([] {
  pe::Command c;
  c.name = "steady-state";
  c.summary = "periodic steady state (Newton shooting)";
  c.help = kHelp;
  c.run = [](const pe::ParsedArgs& args, std::ostream& o, std::ostream& e) {
    return steadyStateImpl(args, o, e);
  };
  pe::registerCommand(std::move(c));
  return true;
})();

}  // namespace

namespace pe {

const char* steadyStateHelp() { return kHelp; }

int runSteadyState(const ParsedArgs& args, std::ostream& out, std::ostream& err) {
  return steadyStateImpl(args, out, err);
}

void registerSteadyStateCommand() {
  // Referencing kRegistered forces the TU (and its static registration)
  // into the link; without this, archive member selection drops it.
  (void)kRegistered;
}

}  // namespace pe
