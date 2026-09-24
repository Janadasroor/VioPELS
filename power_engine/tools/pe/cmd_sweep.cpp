// pe sweep: parameter grid over a netlist, CSV table out.
//
//   pe sweep --netlist <file> --axis <spec> [--axis ...] [--out <csv>]
//            [--tstop <s>] [--dt <s>] [--method trap|trbdf2|auto]
//            [--param K=V ...] [--measure <spec> ...] [--jobs N]
//            [--stop-on-error] [--seed U] [--stats OUT ...] [--yield OUT,LO,HI ...]
//
// Axis specs: K=v1,v2,.. (list) | K=lo:step:hi (range) |
// K=uniform(n,lo,hi) | K=gauss(n,mean,sigma) (deterministic MC; --seed
// overrides both sampler seeds). Measure specs: last|mean|min|max(PROBE)
// [as NAME] over the per-point trajectory (default: last() of every
// probe). --stats/--yield report to stderr (stdout stays pure CSV).
// Exit codes: 0 table produced (row errors are data in the error column),
// 1 propagated runtime error (stopOnError, empty stats), 2 usage error.
#include <cmath>
#include <cstdint>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "cmd_sweep.h"
#include "pe.h"
#include "power_engine/engine.h"
#include "power_engine/sweep.h"

namespace {

const char kHelp[] = R"(pe sweep: parameter grid over a netlist, CSV table out.
Usage: pe sweep --netlist <file> --axis <spec> [options]

Options:
  --netlist <file>   netlist to sweep (required)
  --axis <spec>      K=v1,v2,.. | K=lo:step:hi | K=uniform(n,lo,hi) |
                     K=gauss(n,mean,sigma) (repeatable; empty = one point)
  --out <csv>        write table here (default: stdout)
  --tstop <s>        stop time, overrides .tran tstop
  --dt <s>           time step, overrides .tran dt
  --method <m>       trap (default) | trbdf2 | auto
  --param K=V        extra .param (wins over axis values)
  --measure <spec>   last|mean|min|max(PROBE) [as NAME] (repeatable;
                     default: last() of every probe)
  --jobs N           1 serial (default), 0 = auto, N = threads (cap 256)
                     (bitwise-identical table for any job count)
  --stop-on-error    rethrow first point error instead of recording
  --seed U           sampler seed for uniform/gauss axes
  --stats OUT        column stats (mean/std/min/max/n) to stderr
  --yield OUT,LO,HI  fraction of rows with OUT in [LO,HI], to stderr
  --help             this text
)";

constexpr std::size_t kMaxPoints = 100000;  // grid-size sanity cap

std::string trim(std::string s) {
  const auto ws = " \t\r\n";
  s.erase(0, s.find_first_not_of(ws));
  if (!s.empty()) s.erase(s.find_last_not_of(ws) + 1);
  return s;
}

bool parseInt(const std::string& s, long long& v) {
  try {
    std::size_t n = 0;
    v = std::stoll(s, &n);
    if (n != s.size()) return false;
  } catch (...) {
    return false;
  }
  return true;
}

bool parseULL(const std::string& s, unsigned long long& v) {
  try {
    std::size_t n = 0;
    const long long sv = std::stoll(s, &n);
    if (n != s.size() || sv < 0) return false;
    v = static_cast<unsigned long long>(sv);
  } catch (...) {
    return false;
  }
  return true;
}

std::vector<std::string> splitComma(const std::string& s) {
  std::vector<std::string> parts;
  std::istringstream in(s);
  std::string cell;
  while (std::getline(in, cell, ',')) parts.push_back(trim(cell));
  return parts;
}

// Axis values for one --axis K=spec. Throws runtime_error (usage -> exit 2).
// Without --seed the library default streams are used (uniform and gauss
// have different defaults; forcing one seed on both would correlate them).
std::pair<std::string, std::vector<double>> parseAxis(const std::string& spec, bool seedGiven,
                                                      unsigned long long seed) {
  const auto eq = spec.find('=');
  if (eq == std::string::npos || eq == 0)
    throw std::runtime_error("bad --axis '" + spec + "' (want K=values)");
  const std::string key = trim(spec.substr(0, eq));
  const std::string body = trim(spec.substr(eq + 1));
  if (key.empty() || body.empty())
    throw std::runtime_error("bad --axis '" + spec + "' (want K=values)");
  const auto paren = body.find('(');
  if (paren != std::string::npos) {
    if (body.back() != ')')
      throw std::runtime_error("bad --axis '" + spec + "' (unbalanced parens)");
    const std::string fn = trim(body.substr(0, paren));
    const auto argv = splitComma(body.substr(paren + 1, body.size() - paren - 2));
    if (argv.size() != 3)
      throw std::runtime_error("bad --axis '" + spec + "' (want fn(n,a,b))");
    long long n = 0;
    double a = 0.0, b = 0.0;
    if (!parseInt(argv[0], n) || n <= 0)
      throw std::runtime_error("bad --axis '" + spec + "' (n must be > 0)");
    if (!pe::parseDoubleStrict(argv[1], a) || !pe::parseDoubleStrict(argv[2], b))
      throw std::runtime_error("bad --axis '" + spec + "' (numbers)");
    if (fn == "uniform") {
      if (!(a <= b)) throw std::runtime_error("bad --axis '" + spec + "' (lo > hi)");
      return {key, seedGiven
                       ? power_engine::sweep::uniformSamples(static_cast<int>(n), a, b, seed)
                       : power_engine::sweep::uniformSamples(static_cast<int>(n), a, b)};
    }
    if (fn == "gauss") {
      if (!(b >= 0.0)) throw std::runtime_error("bad --axis '" + spec + "' (sigma < 0)");
      return {key, seedGiven
                       ? power_engine::sweep::gaussianSamples(static_cast<int>(n), a, b, seed)
                       : power_engine::sweep::gaussianSamples(static_cast<int>(n), a, b)};
    }
    throw std::runtime_error("bad --axis '" + spec + "' (want uniform|gauss)");
  }
  if (body.find(':') != std::string::npos) {
    const auto parts = splitComma(body);
    if (parts.size() != 1)
      throw std::runtime_error("bad --axis '" + spec + "' (range has no commas)");
    std::vector<std::string> seg;
    std::istringstream in(parts[0]);
    std::string cell;
    while (std::getline(in, cell, ':')) seg.push_back(trim(cell));
    if (seg.size() != 3)
      throw std::runtime_error("bad --axis '" + spec + "' (want lo:step:hi)");
    double lo = 0.0, step = 0.0, hi = 0.0;
    if (!pe::parseDoubleStrict(seg[0], lo) || !pe::parseDoubleStrict(seg[1], step) ||
        !pe::parseDoubleStrict(seg[2], hi) || !(step > 0.0) || !(hi >= lo))
      throw std::runtime_error("bad --axis '" + spec + "' (want lo:step:hi)");
    std::vector<double> vals;
    for (double v = lo; v <= hi + step * 1e-9; v += step) vals.push_back(v);
    if (vals.empty()) throw std::runtime_error("bad --axis '" + spec + "' (empty range)");
    return {key, vals};
  }
  std::vector<double> vals;
  for (const auto& cell : splitComma(body)) {
    double v = 0.0;
    if (!pe::parseDoubleStrict(cell, v))
      throw std::runtime_error("bad --axis '" + spec + "' (numbers)");
    vals.push_back(v);
  }
  if (vals.empty()) throw std::runtime_error("bad --axis '" + spec + "' (empty list)");
  return {key, vals};
}

struct Measure {
  enum class Op { Last, Mean, Min, Max } op = Op::Last;
  std::string probe;
  std::string outName;
};

// last|mean|min|max(PROBE) [as NAME]. Throws runtime_error (usage -> exit 2).
Measure parseMeasure(const std::string& spec) {
  Measure m;
  std::string body = spec, name;
  const auto as = spec.find(" as ");
  if (as != std::string::npos) {
    body = trim(spec.substr(0, as));
    name = trim(spec.substr(as + 4));
    if (name.empty() || name.find_first_of(", \t") != std::string::npos)
      throw std::runtime_error("bad --measure '" + spec + "' (bad NAME)");
  }
  const auto open = body.find('(');
  if (open == std::string::npos || body.back() != ')')
    throw std::runtime_error("bad --measure '" + spec + "' (want OP(PROBE))");
  const std::string fn = trim(body.substr(0, open));
  const std::string probe = trim(body.substr(open + 1, body.size() - open - 2));
  if (probe.empty()) throw std::runtime_error("bad --measure '" + spec + "' (empty PROBE)");
  if (fn == "last")
    m.op = Measure::Op::Last;
  else if (fn == "mean")
    m.op = Measure::Op::Mean;
  else if (fn == "min")
    m.op = Measure::Op::Min;
  else if (fn == "max")
    m.op = Measure::Op::Max;
  else
    throw std::runtime_error("bad --measure '" + spec + "' (want last|mean|min|max)");
  m.probe = probe;
  m.outName = name.empty() ? fn + "_" + probe : name;
  return m;
}

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

// Per-point trajectory recorder (thread-local: the harness runs points on
// worker threads; R2 pattern — thread-confined, deterministic).
struct Trace {
  std::vector<std::map<std::string, double>> rows;
};
thread_local Trace tlTrace;

int runSweepImpl(const pe::ParsedArgs& args, std::ostream& out, std::ostream& err) {
  if (!args.has("netlist") || args.positionals.size() > 0) {
    err << "pe sweep: --netlist <file> is required (no positionals)\n";
    return 2;
  }
  const std::string path = args.get("netlist");
  std::string text, why;
  if (!readFile(path, text, why)) {
    err << "pe sweep: " << why << "\n";
    return 2;
  }
  double dt = 0.0, tstop = 0.0;
  if (args.has("dt") && (!pe::parseDoubleStrict(args.get("dt"), dt) || !(dt > 0.0))) {
    err << "pe sweep: bad --dt '" << args.get("dt") << "' (finite > 0)\n";
    return 2;
  }
  if (args.has("tstop") && (!pe::parseDoubleStrict(args.get("tstop"), tstop) || !(tstop > 0.0))) {
    err << "pe sweep: bad --tstop '" << args.get("tstop") << "' (finite > 0)\n";
    return 2;
  }
  const std::string method = args.get("method", "trap");
  if (method != "trap" && method != "trbdf2" && method != "auto") {
    err << "pe sweep: bad --method '" << method << "' (trap|trbdf2|auto)\n";
    return 2;
  }
  long long jobs = 1;
  if (args.has("jobs")) {
    if (!parseInt(args.get("jobs"), jobs) || jobs < 0 || jobs > 256) {
      err << "pe sweep: bad --jobs '" << args.get("jobs") << "' (0-256)\n";
      return 2;
    }
  }
  unsigned long long seed = 0;
  bool seedGiven = false;
  if (args.has("seed")) {
    if (!parseULL(args.get("seed"), seed)) {
      err << "pe sweep: bad --seed '" << args.get("seed") << "'\n";
      return 2;
    }
    seedGiven = true;
  }
  // Axes (usage errors -> exit 2, before any simulation).
  std::vector<power_engine::sweep::SweepAxis> axes;
  {
    auto ait = args.opts.find("axis");
    if (ait != args.opts.end()) {
      for (const auto& spec : ait->second) {
        power_engine::sweep::SweepAxis ax;
        try {
          auto [key, vals] = parseAxis(spec, seedGiven, seed);
          ax.param = key;
          ax.values = vals;
        } catch (const std::exception& e) {
          err << "pe sweep: " << e.what() << "\n";
          return 2;
        }
        axes.push_back(ax);
      }
    }
  }
  std::size_t total = 1;
  for (const auto& ax : axes) total *= ax.values.size();
  if (total > kMaxPoints) {
    err << "pe sweep: grid has " << total << " points (cap " << kMaxPoints << ")\n";
    return 2;
  }
  // Measures (usage errors -> exit 2).
  std::vector<Measure> measures;
  {
    auto mit = args.opts.find("measure");
    if (mit != args.opts.end()) {
      for (const auto& spec : mit->second) {
        try {
          measures.push_back(parseMeasure(spec));
        } catch (const std::exception& e) {
          err << "pe sweep: " << e.what() << "\n";
          return 2;
        }
      }
    }
  }
  // --param K=V extras (override axis values; parsed now for exit-2 errors).
  std::vector<std::pair<std::string, double>> extras;
  {
    auto pit = args.opts.find("param");
    if (pit != args.opts.end()) {
      for (const auto& p : pit->second) {
        const auto eq = p.find('=');
        double v = 0.0;
        if (eq == std::string::npos || eq == 0 || !pe::parseDoubleStrict(p.substr(eq + 1), v)) {
          err << "pe sweep: bad --param '" << p << "' (want K=finite-number)\n";
          return 2;
        }
        extras.emplace_back(p.substr(0, eq), v);
      }
    }
  }
  // --yield OUT,LO,HI triples + --stats outputs (parsed now, evaluated later).
  struct YieldReq {
    std::string out;
    double lo = 0.0, hi = 0.0;
  };
  std::vector<YieldReq> yields;
  {
    auto yit = args.opts.find("yield");
    if (yit != args.opts.end()) {
      for (const auto& spec : yit->second) {
        const auto parts = splitComma(spec);
        double lo = 0.0, hi = 0.0;
        if (parts.size() != 3 || parts[0].empty() || !pe::parseDoubleStrict(parts[1], lo) ||
            !pe::parseDoubleStrict(parts[2], hi) || !(hi >= lo)) {
          err << "pe sweep: bad --yield '" << spec << "' (want OUT,LO,HI)\n";
          return 2;
        }
        yields.push_back({parts[0], lo, hi});
      }
    }
  }
  std::vector<std::string> statsOuts;
  {
    auto sit = args.opts.find("stats");
    if (sit != args.opts.end())
      for (const auto& s : sit->second) {
        if (trim(s).empty()) {
          err << "pe sweep: bad --stats '" << s << "' (want OUT)\n";
          return 2;
        }
        statsOuts.push_back(trim(s));
      }
  }

  try {
    // Upfront validation on a scratch engine: parse must succeed (else the
    // harness would record the same error on every row); a stop time is
    // required (.tran or --tstop); measure probes must exist (thermal
    // specs applied: tj: probes appear only with them attached).
    {
      power_engine::Engine scratch;
      scratch.loadNetlist(text);
      if (tstop > 0.0) scratch.setStopTime(tstop);
      if (!(scratch.stopTime() > 0.0)) {
        err << "pe sweep: no stop time (.tran tstop or --tstop)\n";
        return 2;
      }
      if (!scratch.netlist().thermals.empty()) scratch.applyThermalSpecs();
      scratch.start();
      scratch.step();
      if (!measures.empty()) {
        const auto& probes = scratch.currentSolution().probes;
        for (const auto& m : measures) {
          if (probes.find(m.probe) == probes.end()) {
            err << "pe sweep: unknown probe '" << m.probe << "'\n";
            return 2;
          }
        }
      }
    }
    power_engine::sweep::SweepConfig cfg;
    cfg.netlist = text;
    cfg.axes = axes;
    cfg.stopOnError = args.has("stop-on-error");
    cfg.jobs = static_cast<int>(jobs);
    cfg.setup = [&](power_engine::Engine& eng,
                    const std::map<std::string, double>& /*point*/) {
      tlTrace.rows.clear();
      if (dt > 0.0) eng.setTimeStep(dt);
      if (tstop > 0.0) eng.setStopTime(tstop);
      if (method == "trbdf2") {
        eng.setIntegrator(power_engine::Integrator::TrBdf2);
      } else if (method == "auto") {
        eng.setIntegratorAuto(true);
      }
      for (const auto& [k, v] : extras) eng.setParameter(k, v);
      if (!eng.netlist().pwms.empty()) eng.applyPwmSpecs();
      if (!eng.netlist().thermals.empty()) eng.applyThermalSpecs();
      eng.applyLossModels();
      eng.setCallback([&](const power_engine::Solution& s) { tlTrace.rows.push_back(s.probes); });
    };
    cfg.measure = [&](power_engine::Engine& eng,
                      const std::map<std::string, double>& /*point*/) {
      std::map<std::string, double> mOut;
      const auto& final = eng.currentSolution().probes;
      auto series = [&](const std::string& p) {
        std::vector<double> v;
        v.reserve(tlTrace.rows.size());
        for (const auto& row : tlTrace.rows) v.push_back(row.at(p));
        return v;
      };
      if (measures.empty()) {
        for (const auto& [k, v] : final) mOut["last_" + k] = v;
        return mOut;
      }
      for (const auto& m : measures) {
        const std::vector<double> v = series(m.probe);
        if (v.empty()) throw std::runtime_error("pe sweep: no samples for '" + m.probe + "'");
        double r = v.back();
        if (m.op == Measure::Op::Mean) {
          double s = 0.0;
          for (double x : v) s += x;
          r = s / static_cast<double>(v.size());
        } else if (m.op == Measure::Op::Min) {
          for (double x : v) r = std::min(r, x);
        } else if (m.op == Measure::Op::Max) {
          for (double x : v) r = std::max(r, x);
        }
        mOut[m.outName] = r;
      }
      return mOut;
    };
    const power_engine::sweep::SweepTable table = power_engine::sweep::runSweep(cfg);
    const std::string csv = table.csv();
    const std::string outPath = args.get("out");
    if (!outPath.empty()) {
      std::ofstream f(outPath, std::ios::binary | std::ios::trunc);
      if (!f) {
        err << "pe sweep: cannot open --out '" << outPath + "'\n";
        return 1;
      }
      f << csv;
      if (!f) {
        err << "pe sweep: error writing --out '" << outPath + "'\n";
        return 1;
      }
    } else {
      out << csv;
    }
    for (const auto& s : statsOuts) {
      const auto st = power_engine::sweep::columnStats(table, s);
      char buf[256];
      std::snprintf(buf, sizeof(buf), "stats[%s]: n=%zu mean=%.10g std=%.10g min=%.10g max=%.10g\n",
                    s.c_str(), st.n, st.mean, st.std, st.min, st.max);
      err << buf;
    }
    for (const auto& y : yields) {
      const double yw = power_engine::sweep::yieldWithin(table, y.out, y.lo, y.hi);
      char buf[256];
      std::snprintf(buf, sizeof(buf), "yield[%s in %.10g,%.10g] = %.6f\n", y.out.c_str(), y.lo,
                    y.hi, yw);
      err << buf;
    }
  } catch (const std::exception& e) {
    err << "pe sweep: " << e.what() << "\n";
    return 1;
  }
  return 0;
}

[[maybe_unused]] const bool kRegistered = ([] {
  pe::Command c;
  c.name = "sweep";
  c.summary = "parameter grid over a netlist, CSV table";
  c.help = kHelp;
  c.run = [](const pe::ParsedArgs& args, std::ostream& o, std::ostream& e) {
    return runSweepImpl(args, o, e);
  };
  pe::registerCommand(std::move(c));
  return true;
})();

}  // namespace

namespace pe {

const char* sweepHelp() { return kHelp; }

int runSweepCommand(const ParsedArgs& args, std::ostream& out, std::ostream& err) {
  return runSweepImpl(args, out, err);
}

void registerSweepCommand() { (void)kRegistered; }

}  // namespace pe
