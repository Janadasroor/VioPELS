// pe ac: small-signal frequency response (Bode) of a netlist.
//
//   pe ac --netlist <file> [--inject src:<name> | duty:<switch>]
//         [--probe v:N ...] [--from <Hz> --to <Hz> --points N]
//         [--perturb <a>] [--settle <s>] [--measure-periods N] [--ramp <s>]
//         [--method stepped|multitone] [--dt <s>] [--param K=V ...] [--out <csv>]
//
// Injection: a raised-cosine-ramped sine on top of the DC operating point,
// driven into either an independent source (`--inject src:V1`, u = source
// value) or a `.control pwm` switch's carrier duty (`--inject duty:S1`,
// u = gate; edges scheduled from the analytic duty law, duty constant per
// carrier period, exact gate transitions). y = probe value, interval-
// midpoint sampled. The kernel e^{-jwt} is integrated EXACTLY per interval
// with exact DC rejection (ac::FourierMeter), so small AC signals on large
// DC operating points stay accurate. Duty-injected nets alias above half
// the pwm frequency — tones at or above pwm/2 are rejected.
//
// Methods: `stepped` (default) runs one simulation per frequency (exact);
// `multitone` runs ONE simulation with all tones (integer multiples of
// --from, Schroeder phases, integer window covering every tone — no
// harmonic overlap; the printed frequencies are the snapped ones).
// CSV columns: freq,<probe>_mag_db,<probe>_phase_deg[,...]
// Exit codes: 0 ok, 1 sim/runtime error, 2 usage or unreadable input.
//
// Accuracy: choose --settle > 5x the largest circuit time constant (the
// startup transient drowns small signals; there is no generic auto-
// estimate). Per-step ZOH drive carries a ~f*dt footprint — keep f*dt
// <= ~0.01 at the highest frequency (.tran dt / --dt). `.tran tstop` is
// ignored (this command sets its own horizons).
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <numbers>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "cmd_ac.h"
#include "pe.h"
#include "power_engine/ac.h"
#include "power_engine/engine.h"
#include "power_engine/multitone.h"

namespace {

const char kHelp[] = R"(pe ac: small-signal frequency response (Bode).
Usage: pe ac --netlist <file> [options]

Options:
  --netlist <file>   netlist to analyze (required)
  --inject <mode>    src:<name> | duty:<switch>. Default: src:<name> of the
                     only independent V/I source in the netlist. duty:
                     requires a .control pwm on that switch (its freq sets
                     the carrier; the netlist DC duty sets the op point).
  --probe <key>      probe column like v:3 (repeatable; default: all)
  --from <Hz>        sweep start in Hz, default 10 (log grid)
  --to <Hz>          sweep end in Hz, default 1e5 (< pwm/2 for duty:)
  --points N         sweep points, default 41
  --perturb <a>      peak injection amplitude (source units or duty
                     fraction). Default: 5% of |DC value| (src:) / 0.05 (duty:)
  --settle <s>       pre-measurement settle time, default 2e-3
                     (choose > 5x the largest circuit time constant)
  --measure-periods N  integer periods measured per frequency, default 3
  --ramp <s>         raised-cosine injection envelope, default
                     min(1e-3, settle/4); 0 disables
  --dt <s>           time step, overrides .tran dt
  --method <m>       stepped (default) | multitone
  --param K=V        .param override (repeatable)
  --out <csv>        write CSV here (default: stdout)
  --help             this text

CSV columns: freq,<probe>_mag_db,<probe>_phase_deg[,...]
Exit codes: 0 ok, 1 sim/runtime error, 2 usage error or unreadable input.
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

constexpr double kPi = std::numbers::pi;

// Raised-cosine envelope: 0 at t<=0, 1 for t >= ramp (ramp<=0 -> always 1).
double envelope(double t, double ramp) {
  if (ramp <= 0.0 || t >= ramp) return 1.0;
  const double x = t / ramp;
  return 0.5 * (1.0 - std::cos(kPi * x));
}

// Log-spaced grid: from..to with n points, inclusive of both ends.
std::vector<double> logFreqs(double from, double to, long long n) {
  std::vector<double> f;
  if (n < 2) return f;
  const double ratio = std::pow(to / from, 1.0 / static_cast<double>(n - 1));
  f.reserve(static_cast<std::size_t>(n));
  for (long long i = 0; i < n; ++i)
    f.push_back(from * std::pow(ratio, static_cast<double>(i)));
  return f;
}

enum class InjectKind { Source, Duty };

struct Injection {
  InjectKind kind = InjectKind::Source;
  std::string devName;
  double base = 0.0;  // DC source value (src) or carrier duty (duty)
  double amp = 0.0;   // peak perturbation amplitude
};

}  // namespace

namespace pe {

const char* acHelp() { return kHelp; }

}  // namespace pe

namespace {

// Build the engine, pre-schedule duty edges (analytic law), run settle +
// measurement window, and return per-tone gains (probe -> point).
std::vector<std::map<std::string, power_engine::ac::AcPoint>> measure(
    const std::string& text,
    const std::vector<std::pair<std::string, double>>& params, double dtOverride,
    const Injection& inject, double pwmPeriod, double ramp, double settle,
    double window, const std::vector<double>& toneFreqs,
    const std::vector<double>& toneAmps, const std::vector<double>& tonePhases,
    const std::vector<std::string>& probes) {
  power_engine::Engine eng;
  eng.loadNetlist(text);
  // The netlist .tran sets its own tStop — ignore it here, this command
  // drives its own horizons (settle + integer measurement window).
  eng.clearStopTime();
  for (const auto& [k, v] : params) eng.setParameter(k, v);
  if (dtOverride > 0.0) eng.setTimeStep(dtOverride);
  const double tEnd = settle + window;
  if (inject.kind == InjectKind::Duty) {
    eng.circuit().setSwitch(inject.devName, inject.base > 0.0);
    const long long kMax =
        static_cast<long long>(std::llround(tEnd / pwmPeriod)) + 1;
    for (long long k = 0; k < kMax; ++k) {
      const double tk = static_cast<double>(k) * pwmPeriod;
      double d = inject.base;
      for (std::size_t i = 0; i < toneFreqs.size(); ++i) {
        d += toneAmps[i] * envelope(tk, ramp) *
             std::sin(2.0 * kPi * toneFreqs[i] * tk + tonePhases[i]);
      }
      d = std::min(1.0, std::max(0.0, d));
      eng.scheduleSwitch(inject.devName, true, tk);
      eng.scheduleSwitch(inject.devName, false, tk + d * pwmPeriod);
    }
  }
  const std::size_t nProbes = probes.size();
  auto drive = [&](double t) {
    if (inject.kind != InjectKind::Source) return;
    double v = inject.base;
    for (std::size_t i = 0; i < toneFreqs.size(); ++i) {
      v += toneAmps[i] * envelope(t, ramp) *
           std::sin(2.0 * kPi * toneFreqs[i] * t + tonePhases[i]);
    }
    eng.circuit().findDevice(inject.devName).value = v;
  };
  eng.start();
  while (eng.time() < settle) {
    drive(eng.time());
    eng.step();
  }
  const std::size_t nTones = toneFreqs.size();
  // meters[tone][probe]
  std::vector<std::vector<power_engine::ac::FourierMeter>> meters(
      nTones, std::vector<power_engine::ac::FourierMeter>(nProbes));
  for (std::size_t i = 0; i < nTones; ++i)
    for (std::size_t p = 0; p < nProbes; ++p) meters[i][p].begin(toneFreqs[i]);
  double tPrev = eng.time();
  std::vector<double> yPrev(nProbes, 0.0);
  for (std::size_t p = 0; p < nProbes; ++p)
    yPrev[p] = eng.currentSolution().probes.at(probes[p]);
  while (eng.time() < tEnd) {
    drive(eng.time());
    eng.step();
    const double t = eng.time();
    double u = 0.0;
    if (inject.kind == InjectKind::Source) {
      u = eng.circuit().findDevice(inject.devName).value;
    } else {
      u = eng.circuit().switchClosed(inject.devName) ? 1.0 : 0.0;
    }
    for (std::size_t p = 0; p < nProbes; ++p) {
      const double y = eng.currentSolution().probes.at(probes[p]);
      const double yMid = 0.5 * (yPrev[p] + y);
      for (std::size_t i = 0; i < nTones; ++i)
        meters[i][p].sample(tPrev, t, u, yMid);
      yPrev[p] = y;
    }
    tPrev = t;
  }
  eng.stop();
  std::vector<std::map<std::string, power_engine::ac::AcPoint>> rows(nTones);
  for (std::size_t i = 0; i < nTones; ++i)
    for (std::size_t p = 0; p < nProbes; ++p) rows[i][probes[p]] = meters[i][p].result();
  return rows;
}

}  // namespace

namespace {

int runAcImpl(const pe::ParsedArgs& args, std::ostream& out, std::ostream& err) {
  if (!args.has("netlist") || args.positionals.size() > 0) {
    err << "pe ac: --netlist <file> is required (no positionals)\n";
    return 2;
  }
  const std::string path = args.get("netlist");
  std::string text, why;
  if (!readFile(path, text, why)) {
    err << "pe ac: " << why << "\n";
    return 2;
  }
  double from = 10.0, to = 1e5;
  long long points = 41;
  if (args.has("from") && (!parseDouble(args.get("from"), from) || !(from > 0.0))) {
    err << "pe ac: bad --from '" << args.get("from") << "' (finite > 0)\n";
    return 2;
  }
  if (args.has("to") && (!parseDouble(args.get("to"), to) || !(to > 0.0))) {
    err << "pe ac: bad --to '" << args.get("to") << "' (finite > 0)\n";
    return 2;
  }
  if (!(to > from)) {
    err << "pe ac: --to must exceed --from\n";
    return 2;
  }
  if (args.has("points") &&
      (!parseLong(args.get("points"), points) || points < 2 || points > 1000)) {
    err << "pe ac: bad --points '" << args.get("points") << "' (2..1000)\n";
    return 2;
  }
  const std::vector<std::string> wantProbes =
      args.has("probe") ? args.opts.at("probe") : std::vector<std::string>{};
  double dt = 0.0;
  if (args.has("dt") && (!parseDouble(args.get("dt"), dt) || !(dt > 0.0))) {
    err << "pe ac: bad --dt '" << args.get("dt") << "' (finite > 0)\n";
    return 2;
  }
  double settle = 2e-3, perturb = -1.0, ramp = -1.0;
  long long measurePeriods = 3;
  if (args.has("settle") && (!parseDouble(args.get("settle"), settle) || !(settle > 0.0))) {
    err << "pe ac: bad --settle '" << args.get("settle") << "'\n";
    return 2;
  }
  if (args.has("perturb") && (!parseDouble(args.get("perturb"), perturb) || !(perturb > 0.0))) {
    err << "pe ac: bad --perturb '" << args.get("perturb") << "' (finite > 0)\n";
    return 2;
  }
  if (args.has("ramp") && (!parseDouble(args.get("ramp"), ramp) || ramp < 0.0)) {
    err << "pe ac: bad --ramp '" << args.get("ramp") << "' (>= 0)\n";
    return 2;
  }
  if (args.has("measure-periods") &&
      (!parseLong(args.get("measure-periods"), measurePeriods) || measurePeriods < 1)) {
    err << "pe ac: bad --measure-periods '" << args.get("measure-periods")
        << "' (integer >= 1)\n";
    return 2;
  }
  const std::string method = args.get("method", "stepped");
  if (method != "stepped" && method != "multitone") {
    err << "pe ac: bad --method '" << method << "' (stepped|multitone)\n";
    return 2;
  }
  std::vector<std::pair<std::string, double>> params;
  auto pit = args.opts.find("param");
  if (pit != args.opts.end()) {
    for (const auto& p : pit->second) {
      const auto eq = p.find('=');
      double v = 0.0;
      if (eq == std::string::npos || eq == 0 || !parseDouble(p.substr(eq + 1), v)) {
        err << "pe ac: bad --param '" << p << "' (want K=finite-number)\n";
        return 2;
      }
      params.emplace_back(p.substr(0, eq), v);
    }
  }

  try {
    // --- validate + probe listing on a throwaway engine ------------------
    power_engine::Engine probeEng;
    probeEng.loadNetlist(text);
    for (const auto& [k, v] : params) probeEng.setParameter(k, v);
    if (dt > 0.0) probeEng.setTimeStep(dt);
    if (!(dt > 0.0) && !probeEng.netlist().tran.given) {
      err << "pe ac: no time step (.tran dt or --dt)\n";
      return 2;
    }
    probeEng.start();
    std::vector<std::string> allProbes;
    for (const auto& [k, v] : probeEng.currentSolution().probes) {
      (void)v;
      allProbes.push_back(k);
    }
    probeEng.stop();
    std::vector<std::string> probes = wantProbes.empty() ? allProbes : wantProbes;
    for (const auto& p : probes) {
      bool found = false;
      for (const auto& k : allProbes) found = found || (k == p);
      if (!found) {
        err << "pe ac: unknown probe '" << p << "' (have";
        for (const auto& k : allProbes) err << " " << k;
        err << ")\n";
        return 2;
      }
    }
    if (probes.empty()) {
      err << "pe ac: netlist produced no probes\n";
      return 2;
    }
    // --- resolve the injection ------------------------------------------
    Injection inject;
    const std::string injectSpec = args.get("inject");
    if (injectSpec.empty()) {
      std::vector<std::string> srcs;
      for (const auto& d : probeEng.circuit().devices())
        if (d.type == power_engine::DeviceType::VoltageSource ||
            d.type == power_engine::DeviceType::CurrentSource)
          srcs.push_back(d.name);
      if (srcs.size() != 1) {
        err << "pe ac: cannot default --inject (netlist has " << srcs.size()
            << " independent sources; pass --inject src:<name> or duty:<switch>)\n";
        return 2;
      }
      inject.kind = InjectKind::Source;
      inject.devName = srcs.front();
    } else {
      const auto colon = injectSpec.find(':');
      if (colon == std::string::npos) {
        err << "pe ac: bad --inject '" << injectSpec
            << "' (want src:<name>|duty:<name>)\n";
        return 2;
      }
      const std::string mode = injectSpec.substr(0, colon);
      inject.devName = injectSpec.substr(colon + 1);
      if (mode == "src") {
        inject.kind = InjectKind::Source;
      } else if (mode == "duty") {
        inject.kind = InjectKind::Duty;
      } else {
        err << "pe ac: bad --inject mode '" << mode << "' (src|duty)\n";
        return 2;
      }
    }
    double pwmFreq = 0.0, pwmPeriod = 0.0;
    if (inject.kind == InjectKind::Source) {
      try {
        const auto& dev = probeEng.circuit().findDevice(inject.devName);
        if (dev.type != power_engine::DeviceType::VoltageSource &&
            dev.type != power_engine::DeviceType::CurrentSource) {
          err << "pe ac: --inject src:" << inject.devName
              << " is not an independent source\n";
          return 2;
        }
        inject.base = dev.value;
      } catch (...) {
        err << "pe ac: unknown device '" << inject.devName << "'\n";
        return 2;
      }
      if (!(perturb > 0.0)) perturb = 0.05 * std::abs(inject.base);
      if (!(perturb > 0.0)) perturb = 0.05;  // base == 0: absolute fallback
    } else {
      try {
        const auto& dev = probeEng.circuit().findDevice(inject.devName);
        if (dev.type != power_engine::DeviceType::Switch &&
            dev.type != power_engine::DeviceType::SwitchDiode) {
          err << "pe ac: --inject duty:" << inject.devName << " is not a switch\n";
          return 2;
        }
      } catch (...) {
        err << "pe ac: unknown device '" << inject.devName << "'\n";
        return 2;
      }
      double duty = -1.0;
      for (const auto& p : probeEng.netlist().pwms) {
        if (p.switchName == inject.devName) {
          pwmFreq = p.freq;
          duty = p.duty;
        }
      }
      if (!(pwmFreq > 0.0) || duty < 0.0) {
        err << "pe ac: --inject duty:" << inject.devName
            << " needs a .control pwm on that switch\n";
        return 2;
      }
      pwmPeriod = 1.0 / pwmFreq;
      inject.base = duty;
      if (!(perturb > 0.0)) perturb = 0.05;
      if (duty + perturb > 1.0 || duty - perturb < 0.0) {
        err << "pe ac: perturbation exceeds duty headroom (base=" << duty
            << ", amp=" << perturb << ")\n";
        return 2;
      }
    }
    const std::vector<double> freqs = logFreqs(from, to, points);
    if (inject.kind == InjectKind::Duty) {
      for (double f : freqs) {
        if (!(f < 0.5 * pwmFreq)) {
          err << "pe ac: tone at " << f << " Hz aliases (must be < pwm/2 = "
              << 0.5 * pwmFreq
              << " Hz for duty: injection; narrow the sweep instead)\n";
          return 2;
        }
      }
    }
    // --- tone sets -------------------------------------------------------
    const bool multitone = (method == "multitone");
    const double rampUsed = ramp >= 0.0 ? ramp : std::min(1e-3, settle / 4.0);
    std::vector<std::vector<double>> toneGroups, phaseGroups, ampGroups;
    std::vector<double> windowGroups;
    std::vector<double> csvFreqs;
    if (multitone) {
      // Snap every requested tone onto the bin grid (bin = --from, so the
      // window = 1/bin contains an integer number of every tone's periods).
      const double bin = freqs.front();
      std::vector<double> snapped;
      for (double f : freqs) {
        const long long k = std::max<long long>(1, std::llround(f / bin));
        const double sf = static_cast<double>(k) * bin;
        if (snapped.empty() || sf > snapped.back()) snapped.push_back(sf);
      }
      std::vector<double> amps(snapped.size(), perturb);
      const power_engine::multitone::MultitoneSignal sig =
          power_engine::multitone::MultitoneSignal::schroeder(snapped, amps);
      std::vector<double> phases;
      for (const auto& t : sig.tones()) phases.push_back(t.phaseRad);
      toneGroups.push_back(snapped);
      phaseGroups.push_back(std::move(phases));
      ampGroups.push_back(std::move(amps));
      windowGroups.push_back(1.0 / bin);
    } else {
      for (double f : freqs) {
        toneGroups.push_back({f});
        phaseGroups.push_back({0.0});
        ampGroups.push_back({perturb});
        windowGroups.push_back(static_cast<double>(measurePeriods) / f);
      }
    }
    // --- measure ---------------------------------------------------------
    std::vector<std::map<std::string, power_engine::ac::AcPoint>> csvRows;
    for (std::size_t gi = 0; gi < toneGroups.size(); ++gi) {
      try {
        auto rows = measure(text, params, dt, inject, pwmPeriod, rampUsed, settle,
                            windowGroups[gi], toneGroups[gi], ampGroups[gi],
                            phaseGroups[gi], probes);
        for (std::size_t i = 0; i < toneGroups[gi].size(); ++i) {
          csvFreqs.push_back(toneGroups[gi][i]);
          csvRows.push_back(std::move(rows[i]));
        }
      } catch (const std::exception& e) {
        err << "pe ac: measurement failed: " << e.what() << "\n";
        return 1;
      }
    }
    // --- CSV --------------------------------------------------------------
    std::string csv = "freq";
    for (const auto& p : probes) csv += "," + p + "_mag_db," + p + "_phase_deg";
    csv += "\n";
    char buf[64];
    for (std::size_t r = 0; r < csvFreqs.size(); ++r) {
      std::snprintf(buf, sizeof(buf), "%.10g", csvFreqs[r]);
      csv += buf;
      for (const auto& p : probes) {
        const auto& pt = csvRows[r][p];
        std::snprintf(buf, sizeof(buf), "%.10g", pt.magDb);
        csv += ",";
        csv += buf;
        std::snprintf(buf, sizeof(buf), "%.10g", pt.phaseDeg);
        csv += ",";
        csv += buf;
      }
      csv += "\n";
    }
    const std::string outPath = args.get("out");
    if (!outPath.empty()) {
      std::ofstream f(outPath, std::ios::binary | std::ios::trunc);
      if (!f) {
        err << "pe ac: cannot open --out '" << outPath << "'\n";
        return 1;
      }
      f << csv;
      if (!f) {
        err << "pe ac: error writing --out '" << outPath << "'\n";
        return 1;
      }
      err << "pe ac: wrote " << csvFreqs.size() << " points to " << outPath << "\n";
    } else {
      out << csv;
    }
    return 0;
  } catch (const std::exception& e) {
    err << "pe ac: " << e.what() << "\n";
    return 1;
  }
}

[[maybe_unused]] const bool kRegistered = ([] {
  pe::Command c;
  c.name = "ac";
  c.summary = "small-signal frequency response (Bode)";
  c.help = kHelp;
  c.run = [](const pe::ParsedArgs& args, std::ostream& o, std::ostream& e) {
    return runAcImpl(args, o, e);
  };
  pe::registerCommand(std::move(c));
  return true;
})();

}  // namespace

namespace pe {

int runAc(const ParsedArgs& args, std::ostream& out, std::ostream& err) {
  return runAcImpl(args, out, err);
}

void registerAcCommand() {
  // Referencing kRegistered forces the TU (and its static registration)
  // into the link; without this, archive member selection drops it.
  (void)kRegistered;
}

}  // namespace pe
