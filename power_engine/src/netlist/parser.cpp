#include "power_engine/netlist.h"

#include <cctype>
#include <cmath>
#include <cstdio>
#include <memory>
#include <sstream>
#include <stdexcept>

namespace power_engine {
namespace netlist {
namespace {

// Refine-roadmap R3: resource guards for untrusted netlist text. Real
// netlists are kilobytes; these caps are orders of magnitude above any
// legitimate input and only bite on adversarial/accidental blowups.
constexpr std::size_t kMaxNetlistBytes = 1u << 20;  // 1 MiB total input
constexpr int kMaxExprDepth = 64;                   // expression nesting
constexpr std::size_t kMaxExpandedLines = 100000;   // subckt expansion

// ---------- small string helpers ----------

std::string trim(const std::string& s) {
  std::size_t a = 0;
  while (a < s.size() && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
  std::size_t b = s.size();
  while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
  return s.substr(a, b - a);
}

std::string upper(std::string s) {
  for (auto& c : s) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  return s;
}

[[noreturn]] void fail(int line, const std::string& msg) {
  throw std::runtime_error("netlist line " + std::to_string(line) + ": " + msg);
}

std::vector<std::string> split(const std::string& s) {
  std::vector<std::string> out;
  std::istringstream iss(s);
  std::string t;
  while (iss >> t) out.push_back(t);
  return out;
}

bool ieq(const std::string& a, const std::string& b) { return upper(a) == upper(b); }

// Strip `;` comments (brace-aware so `{a;b}` would survive; `;` inside
// braces is not legal in our expressions but be safe anyway).
std::string stripInlineComment(const std::string& s) {
  int depth = 0;
  for (std::size_t i = 0; i < s.size(); ++i) {
    if (s[i] == '{') ++depth;
    if (s[i] == '}') --depth;
    if (s[i] == ';' && depth == 0) return s.substr(0, i);
  }
  return s;
}

// ---------- expression evaluator (recursive descent) ----------

struct ExprTok {
  enum Kind { Num, Ident, Op, LPar, RPar, End } kind = End;
  double num = 0.0;
  std::string text;
};

double suffixScale(const std::string& s, std::size_t& i) {
  // Longest match first: Meg/MEG/meg, then single letters.
  if (i + 3 <= s.size()) {
    std::string w = s.substr(i, 3);
    if (ieq(w, "MEG")) {
      i += 3;
      return 1e6;
    }
  }
  if (i < s.size()) {
    switch (std::tolower(static_cast<unsigned char>(s[i]))) {
      case 'f': ++i; return 1e-15;
      case 'p': ++i; return 1e-12;
      case 'n': ++i; return 1e-9;
      case 'u': ++i; return 1e-6;
      case 'm': ++i; return 1e-3;
      case 'k': ++i; return 1e3;
      case 'g': ++i; return 1e9;
      case 't': ++i; return 1e12;
      default: break;
    }
  }
  return 1.0;
}

class ExprLexer {
 public:
  explicit ExprLexer(const std::string& s) : s_(s) {}
  ExprTok next() {
    while (i_ < s_.size() && std::isspace(static_cast<unsigned char>(s_[i_]))) ++i_;
    ExprTok t;
    if (i_ >= s_.size()) return t;
    const char c = s_[i_];
    if (c == '(') {
      ++i_;
      t.kind = ExprTok::LPar;
      return t;
    }
    if (c == ')') {
      ++i_;
      t.kind = ExprTok::RPar;
      return t;
    }
    if (c == '+' || c == '-' || c == '*' || c == '/' || c == '^') {
      ++i_;
      t.kind = ExprTok::Op;
      t.text = std::string(1, c);
      return t;
    }
    if (std::isdigit(static_cast<unsigned char>(c)) || c == '.') {
      // Scan the numeric extent first: stod() on substr(i_) would copy the
      // whole remaining expression per number token (quadratic on long
      // `{1+1+...}` chains). The scan mirrors strtod shape (no sign; the
      // lexer splits +/- as operators) so stod sees an exact-size copy.
      std::size_t j = i_;
      while (j < s_.size() && std::isdigit(static_cast<unsigned char>(s_[j]))) ++j;
      if (j == i_ + 1 && s_[i_] == '0' && j + 1 < s_.size() &&
          (s_[j] == 'x' || s_[j] == 'X') &&
          std::isxdigit(static_cast<unsigned char>(s_[j + 1]))) {
        // Hex float prefix (stod/strtod accept it; keep behavior identical).
        j += 2;
        while (j < s_.size() && std::isxdigit(static_cast<unsigned char>(s_[j]))) ++j;
      } else {
        if (j < s_.size() && s_[j] == '.') {
          ++j;
          while (j < s_.size() && std::isdigit(static_cast<unsigned char>(s_[j]))) ++j;
        }
        if (j < s_.size() && (s_[j] == 'e' || s_[j] == 'E')) {
          std::size_t k = j + 1;
          if (k < s_.size() && (s_[k] == '+' || s_[k] == '-')) ++k;
          if (k < s_.size() && std::isdigit(static_cast<unsigned char>(s_[k]))) {
            j = k + 1;
            while (j < s_.size() && std::isdigit(static_cast<unsigned char>(s_[j]))) ++j;
          }
        }
      }
      std::size_t len = 0;
      double v = 0.0;
      try {
        v = std::stod(s_.substr(i_, j - i_), &len);
      } catch (...) {
        throw std::runtime_error("bad number in expression near '" + s_.substr(i_, j - i_) + "'");
      }
      i_ += len;
      v *= suffixScale(s_, i_);
      t.kind = ExprTok::Num;
      t.num = v;
      return t;
    }
    if (std::isalpha(static_cast<unsigned char>(c)) || c == '_') {
      std::size_t j = i_;
      while (j < s_.size() &&
             (std::isalnum(static_cast<unsigned char>(s_[j])) || s_[j] == '_')) {
        ++j;
      }
      t.kind = ExprTok::Ident;
      t.text = s_.substr(i_, j - i_);
      i_ = j;
      return t;
    }
    throw std::runtime_error(std::string("bad character in expression: '") + c + "'");
  }

 private:
  const std::string& s_;
  std::size_t i_ = 0;
};

class ExprParser {
 public:
  ExprParser(const std::string& s, const std::map<std::string, double>& params)
      : lex_(s), params_(params) {
    cur_ = lex_.next();
  }
  double run() {
    const double v = expr();
    if (cur_.kind != ExprTok::End) throw std::runtime_error("trailing tokens in expression");
    return v;
  }

 private:
  void eat() { cur_ = lex_.next(); }
  // Depth guard: unary +/- , right-assoc ^ and parens all recurse through
  // factor(). Iterative chains (1+1+..., 1*1*...) loop in expr()/term() and
  // never accumulate depth. RAII restores on all return paths; a throw
  // aborts the whole parse anyway.
  struct DepthGuard {
    int& d;
    explicit DepthGuard(int& d) : d(d) {
      if (++d > kMaxExprDepth)
        throw std::runtime_error("expression nesting too deep (limit 64)");
    }
    ~DepthGuard() { --d; }
  };
  double expr() {
    double v = term();
    while (cur_.kind == ExprTok::Op && (cur_.text == "+" || cur_.text == "-")) {
      std::string op = cur_.text;
      eat();
      const double r = term();
      v = (op == "+") ? v + r : v - r;
    }
    return v;
  }
  double term() {
    double v = factor();
    while (cur_.kind == ExprTok::Op && (cur_.text == "*" || cur_.text == "/")) {
      std::string op = cur_.text;
      eat();
      const double r = factor();
      v = (op == "*") ? v * r : v / r;
    }
    return v;
  }
  double factor() {
    DepthGuard g(depth_);
    if (cur_.kind == ExprTok::Op && (cur_.text == "-" || cur_.text == "+")) {
      std::string op = cur_.text;
      eat();
      const double v = factor();
      return (op == "-") ? -v : v;
    }
    double v = primary();
    if (cur_.kind == ExprTok::Op && cur_.text == "^") {
      eat();
      v = std::pow(v, factor());
    }
    return v;
  }
  double primary() {
    if (cur_.kind == ExprTok::Num) {
      const double v = cur_.num;
      eat();
      return v;
    }
    if (cur_.kind == ExprTok::Ident) {
      std::string key = cur_.text;
      eat();
      std::string up = upper(key);
      for (const auto& [k, val] : params_) {
        if (upper(k) == up) return val;
      }
      throw std::runtime_error("unknown parameter '" + key + "'");
    }
    if (cur_.kind == ExprTok::LPar) {
      eat();
      const double v = expr();
      if (cur_.kind != ExprTok::RPar) throw std::runtime_error("missing ')'");
      eat();
      return v;
    }
    throw std::runtime_error("unexpected token in expression");
  }
  ExprLexer lex_;
  const std::map<std::string, double>& params_;
  ExprTok cur_;
  int depth_ = 0;
};

double evalRaw(const std::string& s, const std::map<std::string, double>& params) {
  const std::string t = trim(s);
  if (t.size() >= 2 && t.front() == '{' && t.back() == '}') {
    return ExprParser(trim(t.substr(1, t.size() - 2)), params).run();
  }
  return ExprParser(t, params).run();
}

// ---------- elaboration context ----------

struct SubcktDef {
  std::string name;
  std::vector<std::string> ports;
  struct RawLine {
    int lineNo = 0;
    std::string text;
  };
  std::vector<RawLine> body;
};

struct Elaborator {
  NetlistResult& out;
  const std::map<std::string, SubcktDef>& subckts;
  std::map<std::string, double>& params;
  const std::map<std::string, double>& locked;  // setParameter overrides win
  std::map<std::string, int> nodeIds;  // upper(node name) -> int id
  int nextNode = 1;
  std::string prefix;                            // e.g. "X1:" for hierarchy
  std::map<std::string, int> portBind;           // upper(port) -> node id (inside subckt)
  std::string lastSwitchOrDiode;                 // for bare .thermal
  int depth = 0;
  // Shared subckt-expansion budget: X-instantiation multiplies body lines
  // (self-recursive subckts fan out exponentially until the depth cap).
  // Counts every elaborated line across all scopes; shared_ptr so child
  // Elaborators draw from the same budget.
  std::shared_ptr<std::size_t> expanded;

  int nodeId(const std::string& tok, int line) {
    const std::string up = upper(tok);
    if (up == "0" || up == "GND" || up == "GROUND") return 0;
    // Inside a subckt, ports bind to the caller's nodes.
    auto pb = portBind.find(up);
    if (pb != portBind.end()) return pb->second;
    const std::string key = prefix + up;
    auto it = nodeIds.find(key);
    if (it != nodeIds.end()) return it->second;
    if (nextNode > 1000000) fail(line, "too many nodes");
    nodeIds[key] = nextNode;
    return nextNode++;
  }

  std::string scoped(const std::string& name) const { return prefix + name; }

  double value(const std::string& tok, int line) {
    std::string err;
    try {
      return evalRaw(tok, params);
    } catch (const std::exception& e) {
      err = e.what();
    }
    fail(line, std::string("bad value '") + tok + "': " + err);
  }

  // Split trailing KEY=VAL tokens; returns {positionals, kv map}.
  static void splitKv(const std::vector<std::string>& toks, std::size_t from,
                      std::vector<std::string>& pos, std::map<std::string, std::string>& kv,
                      int line) {
    for (std::size_t i = from; i < toks.size(); ++i) {
      const auto eq = toks[i].find('=');
      if (eq == std::string::npos) {
        pos.push_back(toks[i]);
      } else {
        std::string k = upper(trim(toks[i].substr(0, eq)));
        std::string v = trim(toks[i].substr(eq + 1));
        if (k.empty() || v.empty()) fail(line, "malformed KEY=VALUE token '" + toks[i] + "'");
        kv[k] = v;
      }
    }
  }

  double kvNum(const std::map<std::string, std::string>& kv, const std::string& key,
               double dflt, int line, bool* present = nullptr) {
    auto it = kv.find(key);
    if (it == kv.end()) {
      if (present) *present = false;
      return dflt;
    }
    if (present) *present = true;
    return value(it->second, line);
  }

  void addDeviceLine(const std::vector<std::string>& toks, int line) {
    if (toks.empty()) return;
    const std::string name = scoped(toks[0]);
    const char kind = static_cast<char>(std::toupper(static_cast<unsigned char>(toks[0][0])));
    std::vector<std::string> pos;
    std::map<std::string, std::string> kv;
    splitKv(toks, 1, pos, kv, line);
    auto& c = out.circuit;

    switch (kind) {
      case 'R': {
        if (pos.size() != 3) fail(line, "R needs: Rname n1 n2 value");
        c.addResistor(name, nodeId(pos[0], line), nodeId(pos[1], line), value(pos[2], line));
        break;
      }
      case 'C': {
        if (pos.size() != 3) fail(line, "C needs: Cname n1 n2 value [IC=v0]");
        c.addCapacitor(name, nodeId(pos[0], line), nodeId(pos[1], line), value(pos[2], line),
                       kvNum(kv, "IC", 0.0, line));
        break;
      }
      case 'L': {
        if (pos.size() != 3) fail(line, "L needs: Lname n1 n2 value [IC=i0]");
        c.addInductor(name, nodeId(pos[0], line), nodeId(pos[1], line), value(pos[2], line),
                      kvNum(kv, "IC", 0.0, line));
        break;
      }
      case 'V':
      case 'I': {
        // [DC] value : optional DC keyword.
        std::vector<std::string> p = pos;
        if (!p.empty() && ieq(p[0], "DC")) p.erase(p.begin());
        // Also allow leading DC before nodes? No: strict N+ N- value.
        if (p.size() != 3) {
          fail(line, std::string(kind == 'V' ? "V" : "I") + " needs: Xname n+ n- value");
        }
        const double v = value(p[2], line);
        if (kind == 'V') {
          c.addVoltageSource(name, nodeId(p[0], line), nodeId(p[1], line), v);
        } else {
          c.addCurrentSource(name, nodeId(p[0], line), nodeId(p[1], line), v);
        }
        break;
      }
      case 'S':
      case 'M':
      case 'Q': {
        if (pos.size() != 2) fail(line, "switch needs: Sname n1 n2 [MODEL=..] ..");
        double ron = 5e-3, roff = 1e6, eon = 0.0, eoff = 0.0, ttail = 0.0, tailk = 0.1;
        double tsw = 0.0;
        bool closed = false;
        std::string eonTab, eoffTab, ronTab;  // .etable refs (UPPER)
        auto mit = kv.find("MODEL");
        if (mit != kv.end()) {
          auto m = out.models.find(upper(mit->second));
          if (m == out.models.end()) fail(line, "unknown .model '" + mit->second + "'");
          const std::string mk = upper(m->second.kind);
          const bool isFet = (kind == 'S') || (kind == 'M');
          if (isFet && mk != "MOSFET_IDEAL" && mk != "IGBT_IDEAL") {
            fail(line, "model '" + mit->second + "' is not a switch model");
          }
          if (kind == 'Q' && mk != "IGBT_IDEAL" && mk != "MOSFET_IDEAL") {
            fail(line, "model '" + mit->second + "' is not a switch model");
          }
          auto g = [&](const char* k, double d) {
            auto it = m->second.num.find(k);
            return it == m->second.num.end() ? d : it->second;
          };
          auto gs = [&](const char* k) {
            auto it = m->second.str.find(k);
            return it == m->second.str.end() ? std::string{} : it->second;
          };
          ron = g("RON", ron);
          roff = g("ROFF", roff);
          eon = g("EON", 0.0);
          eoff = g("EOFF", 0.0);
          ttail = g("TTAIL", 0.0);
          tailk = g("TAILK", 0.1);
          tsw = g("TSW", 0.0);
          eonTab = gs("EON_TABLE");
          eoffTab = gs("EOFF_TABLE");
          ronTab = gs("RON_TABLE");
        }
        // Device-level loss-table refs win over MODEL (stored UPPER-cased).
        {
          auto dv = [&](const char* k, std::string dflt) {
            auto it2 = kv.find(k);
            return it2 == kv.end() ? dflt : upper(it2->second);
          };
          eonTab = dv("EON_TABLE", upper(eonTab));
          eoffTab = dv("EOFF_TABLE", upper(eoffTab));
          ronTab = dv("RON_TABLE", upper(ronTab));
          if (kv.find("VF_TABLE") != kv.end()) fail(line, "switches use RON_TABLE, not VF_TABLE");
        }
        bool hasRon = false, hasRoff = false;
        ron = kvNum(kv, "RON", ron, line, &hasRon);
        roff = kvNum(kv, "ROFF", roff, line, &hasRoff);
        (void)hasRon;
        (void)hasRoff;
        eon = kvNum(kv, "EON", eon, line);
        eoff = kvNum(kv, "EOFF", eoff, line);
        if (!(eon >= 0.0) || !std::isfinite(eon)) fail(line, "EON must be finite >= 0");
        if (!(eoff >= 0.0) || !std::isfinite(eoff)) fail(line, "EOFF must be finite >= 0");
        ttail = kvNum(kv, "TTAIL", ttail, line);
        tailk = kvNum(kv, "TAILK", tailk, line);
        if (!(ttail >= 0.0) || !std::isfinite(ttail)) fail(line, "TTAIL must be finite >= 0");
        if (!(tailk >= 0.0) || !std::isfinite(tailk)) fail(line, "TAILK must be finite >= 0");
        tsw = kvNum(kv, "TSW", tsw, line);
        if (!(tsw >= 0.0) || !std::isfinite(tsw)) fail(line, "TSW must be finite >= 0");
        auto iit = kv.find("INIT");
        if (iit != kv.end()) {
          if (ieq(iit->second, "ON") || ieq(iit->second, "CLOSED") || iit->second == "1") {
            closed = true;
          } else if (ieq(iit->second, "OFF") || ieq(iit->second, "OPEN") ||
                     iit->second == "0") {
            closed = false;
          } else {
            fail(line, "INIT must be ON or OFF");
          }
        }
        c.addSwitch(name, nodeId(pos[0], line), nodeId(pos[1], line), ron, roff, closed, eon,
                    eoff, ttail, tailk, tsw);
        Device& ds = c.findDevice(name);
        ds.eonTable = eonTab;
        ds.eoffTable = eoffTab;
        ds.ronTable = ronTab;
        lastSwitchOrDiode = name;
        break;
      }
      case 'D': {
        if (pos.size() != 2) fail(line, "diode needs: Dname anode cathode [MODEL=..] ..");
        double vf = 0.0, ron = 10e-3, roff = 1e6, qrr = 0.0, trr = 0.0;
        std::string ronTab, vfTab;  // .etable refs (UPPER)
        auto mit = kv.find("MODEL");
        if (mit != kv.end()) {
          auto m = out.models.find(upper(mit->second));
          if (m == out.models.end()) fail(line, "unknown .model '" + mit->second + "'");
          if (upper(m->second.kind) != "DIODE_IDEAL") {
            fail(line, "model '" + mit->second + "' is not a diode model");
          }
          auto g = [&](const char* k, double d) {
            auto it = m->second.num.find(k);
            return it == m->second.num.end() ? d : it->second;
          };
          auto gs = [&](const char* k) {
            auto it = m->second.str.find(k);
            return it == m->second.str.end() ? std::string{} : it->second;
          };
          vf = g("VF", vf);
          ron = g("RON", ron);
          roff = g("ROFF", roff);
          qrr = g("QRR", 0.0);
          trr = g("TRR", 0.0);
          ronTab = gs("RON_TABLE");
          vfTab = gs("VF_TABLE");
          if (m->second.str.count("EON_TABLE") != 0u ||
              m->second.str.count("EOFF_TABLE") != 0u) {
            fail(line, "diodes use recovery (QRR), not EON/EOFF tables");
          }
        }
        vf = kvNum(kv, "VF", vf, line);
        ron = kvNum(kv, "RON", ron, line);
        roff = kvNum(kv, "ROFF", roff, line);
        qrr = kvNum(kv, "QRR", qrr, line);
        trr = kvNum(kv, "TRR", trr, line);
        {
          auto dv = [&](const char* k, std::string dflt) {
            auto it2 = kv.find(k);
            return it2 == kv.end() ? dflt : upper(it2->second);
          };
          ronTab = dv("RON_TABLE", upper(ronTab));
          vfTab = dv("VF_TABLE", upper(vfTab));
          if (kv.find("EON_TABLE") != kv.end() || kv.find("EOFF_TABLE") != kv.end()) {
            fail(line, "diodes use recovery (QRR), not EON/EOFF tables");
          }
        }
        c.addDiode(name, nodeId(pos[0], line), nodeId(pos[1], line), vf, ron, roff, qrr, trr);
        Device& dd = c.findDevice(name);
        dd.ronTable = ronTab;
        dd.vfTable = vfTab;
        lastSwitchOrDiode = name;
        break;
      }
      case 'T': {
        if (pos.size() != 4 && pos.size() != 5 && pos.size() != 6) {
          fail(line, "transformer needs: Tname n1 n2 n3 n4 [RATIO=n] or "
                      "Tname nA nCT nB nS+ nS- RATIO=n (center-tap)");
        }
        double ratio = 2.0;
        if (pos.size() == 6) {
          // Center-tap with positional ratio.
          ratio = value(pos[5], line);
        } else if (pos.size() == 5 && kv.find("RATIO") == kv.end()) {
          ratio = value(pos[4], line);  // legacy 4-node positional ratio
        } else {
          ratio = kvNum(kv, "RATIO", ratio, line);
        }
        if (!(ratio > 0.0) || !std::isfinite(ratio)) fail(line, "RATIO must be positive");
        if (pos.size() >= 5 && kv.find("RATIO") != kv.end()) {
          // 5 nodes + RATIO key: shared-core center-tap.
          c.addCenterTapTransformer(name, nodeId(pos[0], line), nodeId(pos[1], line),
                                    nodeId(pos[2], line), nodeId(pos[3], line),
                                    nodeId(pos[4], line), ratio);
          break;
        }
        if (pos.size() == 6) {
          c.addCenterTapTransformer(name, nodeId(pos[0], line), nodeId(pos[1], line),
                                    nodeId(pos[2], line), nodeId(pos[3], line),
                                    nodeId(pos[4], line), ratio);
          break;
        }
        c.addTransformer(name, nodeId(pos[0], line), nodeId(pos[1], line),
                         nodeId(pos[2], line), nodeId(pos[3], line), ratio);
        break;
      }
      case 'W': {
        // Coupled inductors: Wname n1 n2 n3 n4 L1=.. L2=.. (K=.. | M=..).
        if (pos.size() != 4) fail(line, "coupled inductors need: Wname n1 n2 n3 n4 L1=.. L2=.. K=..");
        const double l1 = kvNum(kv, "L1", 0.0, line);
        const double l2 = kvNum(kv, "L2", 0.0, line);
        if (!(l1 > 0.0) || !(l2 > 0.0)) fail(line, "coupled inductors need L1>0 and L2>0");
        bool hasK = false, hasM = false;
        const double k = kvNum(kv, "K", 0.0, line, &hasK);
        const double mm = kvNum(kv, "M", 0.0, line, &hasM);
        double kk = 0.0;
        if (hasK == hasM) fail(line, "coupled inductors need exactly one of K= or M=");
        if (hasK) {
          kk = k;
        } else {
          if (!(mm > 0.0)) fail(line, "M must be positive");
          if (!(mm * mm < l1 * l2)) fail(line, "M must satisfy M^2 < L1*L2");
          kk = mm / std::sqrt(l1 * l2);
        }
        const double il1 = kvNum(kv, "IC1", 0.0, line);
        const double il2 = kvNum(kv, "IC2", 0.0, line);
        c.addCoupledInductors(name, nodeId(pos[0], line), nodeId(pos[1], line),
                              nodeId(pos[2], line), nodeId(pos[3], line), l1, l2, kk, il1,
                              il2);
        break;
      }
      case 'Y': {
        // Saturable inductor: Yname n1 n2 LUNSAT=.. LSAT=.. ISAT=.. [IC=..].
        if (pos.size() != 2) fail(line, "saturable inductor needs: Yname n1 n2 LUNSAT=.. LSAT=.. ISAT=..");
        const double lu = kvNum(kv, "LUNSAT", 0.0, line);
        const double ls = kvNum(kv, "LSAT", 0.0, line);
        const double is = kvNum(kv, "ISAT", 0.0, line);
        const double ic = kvNum(kv, "IC", 0.0, line);
        c.addSaturableInductor(name, nodeId(pos[0], line), nodeId(pos[1], line), lu, ls,
                               is, ic);
        break;
      }
      case 'H': {
        // Hysteretic inductor: Hname n1 n2 N=.. AE=.. LE=.. VE=.. BS=.. A=..
        // [HC=..] [IC=..].
        if (pos.size() != 2) fail(line, "hysteretic inductor needs: Hname n1 n2 N=.. AE=.. LE=.. VE=.. BS=.. A=..");
        c.addHystereticInductor(
            name, nodeId(pos[0], line), nodeId(pos[1], line), kvNum(kv, "N", 0.0, line),
            kvNum(kv, "AE", 0.0, line), kvNum(kv, "LE", 0.0, line),
            kvNum(kv, "VE", 0.0, line), kvNum(kv, "BS", 0.0, line),
            kvNum(kv, "A", 0.0, line), kvNum(kv, "HC", 0.0, line),
            kvNum(kv, "IC", 0.0, line));
        break;
      }
      case 'X': {
        if (depth >= 16) fail(line, "subcircuit nesting too deep");
        if (pos.empty()) fail(line, "X needs: Xname nodes... subckt");
        const std::string subName = upper(pos.back());
        auto sit = subckts.find(subName);
        if (sit == subckts.end()) fail(line, "unknown subcircuit '" + pos.back() + "'");
        const SubcktDef& sub = sit->second;
        if (pos.size() - 1 != sub.ports.size()) {
          fail(line, "subcircuit '" + sub.name + "' expects " +
                         std::to_string(sub.ports.size()) + " nodes, got " +
                         std::to_string(pos.size() - 1));
        }
        Elaborator child{out, subckts, params, locked, nodeIds, nextNode,
                          prefix + toks[0] + ":", {}, lastSwitchOrDiode, depth + 1,
                          expanded};
        // Copy shared node table so sibling scopes stay consistent.
        child.nodeIds = nodeIds;
        child.nextNode = nextNode;
        for (std::size_t i = 0; i < sub.ports.size(); ++i) {
          child.portBind[upper(sub.ports[i])] = nodeId(pos[i], line);
        }
        for (const auto& rl : sub.body) child.processLine(rl.text, rl.lineNo);
        nodeIds = child.nodeIds;
        nextNode = child.nextNode;
        lastSwitchOrDiode = child.lastSwitchOrDiode;
        break;
      }
      default:
        fail(line, std::string("unknown device '") + toks[0] + "'");
    }
  }

  void processDirective(const std::vector<std::string>& toks, int line) {
    const std::string dir = upper(toks[0]);
    if (dir == ".PARAM") {
      auto setParam = [&](const std::string& k, double v) {
        if (locked.count(upper(k)) != 0u) return;  // override wins
        params[upper(k)] = v;
      };
      if ((toks.size() - 1) % 2 != 0 && toks.size() > 1) {
        // Allow KEY=VAL form mixed in: re-split.
        for (std::size_t i = 1; i < toks.size(); ++i) {
          const auto eq = toks[i].find('=');
          if (eq == std::string::npos) fail(line, ".param needs NAME value pairs");
          setParam(trim(toks[i].substr(0, eq)), value(trim(toks[i].substr(eq + 1)), line));
        }
        return;
      }
      for (std::size_t i = 1; i + 1 < toks.size(); i += 2) {
        setParam(toks[i], value(toks[i + 1], line));
      }
      if (toks.size() <= 1) fail(line, ".param needs NAME value pairs");
      return;
    }
    if (dir == ".MODEL") {
      if (toks.size() < 3) fail(line, ".model needs: .model NAME KIND k=v ...");
      ModelDef m;
      m.name = toks[1];
      m.kind = toks[2];
      const std::string k = upper(m.kind);
      if (k != "MOSFET_IDEAL" && k != "IGBT_IDEAL" && k != "DIODE_IDEAL") {
        fail(line, "unsupported .model kind '" + m.kind + "'");
      }
      for (std::size_t i = 3; i < toks.size(); ++i) {
        const auto eq = toks[i].find('=');
        if (eq == std::string::npos) fail(line, "model params must be KEY=VALUE");
        std::string key = upper(trim(toks[i].substr(0, eq)));
        std::string val = trim(toks[i].substr(eq + 1));
        try {
          m.num[key] = evalRaw(val, params);
        } catch (...) {
          m.str[key] = val;  // keep non-numeric as string
        }
      }
      out.models[upper(m.name)] = m;
      return;
    }
    if (dir == ".TRAN") {
      if (toks.size() != 3 && toks.size() != 4)
        fail(line, ".tran needs: .tran dt tstop [TRAP|TRBDF2|AUTO]");
      out.tran.given = true;
      out.tran.dt = value(toks[1], line);
      out.tran.tstop = value(toks[2], line);
      if (!(out.tran.dt > 0.0) || !(out.tran.tstop > 0.0)) {
        fail(line, ".tran dt and tstop must be positive");
      }
      out.tran.method.clear();
      if (toks.size() == 4) {
        const std::string m = upper(toks[3]);
        if (m != "TRAP" && m != "TRBDF2" && m != "AUTO")
          fail(line, ".tran method must be TRAP, TRBDF2 or AUTO");
        if (m != "TRAP") out.tran.method = m;
      }
      return;
    }
    if (dir == ".CONTROL") {
      if (toks.size() < 2 || !ieq(toks[1], "PWM")) {
        fail(line, ".control only supports: .control pwm switch=.. freq=.. ...");
      }
      std::vector<std::string> pos;
      std::map<std::string, std::string> kv;
      splitKv(toks, 2, pos, kv, line);
      PwmSpec p;
      auto req = [&](const char* k) -> std::string {
        auto it = kv.find(k);
        if (it == kv.end()) fail(line, std::string(".control pwm needs ") + k + "=..");
        return it->second;
      };
      p.switchName =ScopedNameFix(req("SWITCH"));
      p.freq = value(req("FREQ"), line);
      p.duty = kvNum(kv, "DUTY", 0.5, line);
      p.deadtime = kvNum(kv, "DEADTIME", 0.0, line);
      p.phase = kvNum(kv, "PHASE", 0.0, line);
      auto cit = kv.find("COMPLEMENT");
      if (cit != kv.end()) p.complement = ScopedNameFix(cit->second);
      auto car = kv.find("CARRIER");
      if (car != kv.end()) {
        if (ieq(car->second, "SYMMETRIC") || ieq(car->second, "TRIANGLE")) {
          p.symmetric = true;
        } else if (ieq(car->second, "TRAILING") || ieq(car->second, "SAWTOOTH")) {
          p.symmetric = false;
        } else {
          fail(line, "pwm carrier must be trailing or symmetric");
        }
      }
      if (!(p.freq > 0.0)) fail(line, "pwm freq must be positive");
      if (p.duty < 0.0 || p.duty > 1.0) fail(line, "pwm duty must be in [0,1]");
      if (p.deadtime < 0.0) fail(line, "pwm deadtime must be >= 0");
      out.pwms.push_back(p);
      return;
    }
    if (dir == ".THERMAL") {
      if (toks.size() < 2) fail(line, ".thermal needs: .thermal foster|cauer ...");
      ThermalSpec t;
      const std::string net = upper(toks[1]);
      if (net == "FOSTER") {
        t.foster = true;
      } else if (net == "CAUER") {
        t.foster = false;
      } else {
        fail(line, "thermal network must be foster or cauer");
      }
      std::vector<std::string> pos;
      std::map<std::string, std::string> kv;
      splitKv(toks, 2, pos, kv, line);
      auto dit = kv.find("DEVICE");
      if (dit == kv.end()) dit = kv.find("DEV");
      if (dit != kv.end()) {
        t.device = ScopedNameFix(dit->second);
      } else if (!lastSwitchOrDiode.empty()) {
        t.device = lastSwitchOrDiode;  // attach to most recent switch/diode
      } else {
        fail(line, ".thermal needs device= (no preceding switch/diode)");
      }
      t.tamb = kvNum(kv, "TAMB", 25.0, line);
      // Collect R1 C1 R2 C2 ... pairs (order-independent).
      for (int i = 1; i <= 32; ++i) {
        char rb[16], cb[16];
        std::snprintf(rb, sizeof(rb), "R%d", i);
        std::snprintf(cb, sizeof(cb), "C%d", i);
        auto rit = kv.find(rb);
        auto cit = kv.find(cb);
        if (rit == kv.end() && cit == kv.end()) break;
        if (rit == kv.end() || cit == kv.end()) {
          fail(line, "thermal stage " + std::to_string(i) + " needs both R and C");
        }
        t.r.push_back(value(rit->second, line));
        t.c.push_back(value(cit->second, line));
      }
      if (t.r.empty()) fail(line, ".thermal needs at least one R/C stage");
      for (double v : t.r) {
        if (!(v > 0.0)) fail(line, "thermal R must be positive");
      }
      for (double v : t.c) {
        if (!(v > 0.0)) fail(line, "thermal C must be positive");
      }
      out.thermals.push_back(t);
      return;
    }
    if (dir == ".ETABLE") {
      // Datasheet loss table:
      // .etable NAME [I=a,b,..] [V=..] [TJ=..] (E=.. | R=.. | VF=..)
      // Exactly one value key: E = switching energy [J], R = resistance
      // [Ohm], VF = forward drop [V]. Values are row-major with the LAST
      // listed axis fastest ((i*V+j)*TJ+k for I,V,TJ). Single-point axes
      // allowed (constant along them). Out-of-range lookups clamp.
      if (toks.size() < 3) fail(line, ".etable needs: .etable NAME AXES.. E|R|VF=..");
      std::vector<std::string> pos;
      std::map<std::string, std::string> kv;
      splitKv(toks, 2, pos, kv, line);
      auto csv = [&](const char* k) {
        std::vector<double> vs;
        auto it = kv.find(k);
        if (it == kv.end()) return vs;
        std::string s = it->second;
        std::size_t a = 0;
        while (a <= s.size()) {
          const auto c = s.find(',', a);
          const std::string tok = trim(s.substr(a, c == std::string::npos ? c : c - a));
          if (tok.empty()) fail(line, "empty entry in .etable " + std::string(k));
          vs.push_back(value(tok, line));
          if (c == std::string::npos) break;
          a = c + 1;
        }
        return vs;
      };
      const std::vector<double> gi = csv("I");
      const std::vector<double> gv = csv("V");
      const std::vector<double> gt = csv("TJ");
      const bool hasE = kv.count("E") != 0u;
      const bool hasR = kv.count("R") != 0u;
      const bool hasVf = kv.count("VF") != 0u;
      if (static_cast<int>(hasE) + static_cast<int>(hasR) + static_cast<int>(hasVf) != 1) {
        fail(line, ".etable needs exactly one of E=, R=, VF=");
      }
      const std::vector<double> vals =
          csv(hasE ? "E" : (hasR ? "R" : "VF"));
      std::vector<std::string> axes;
      std::vector<std::vector<double>> grids;
      if (!gi.empty()) {
        axes.emplace_back("I");
        grids.push_back(gi);
      }
      if (!gv.empty()) {
        axes.emplace_back("V");
        grids.push_back(gv);
      }
      if (!gt.empty()) {
        axes.emplace_back("TJ");
        grids.push_back(gt);
      }
      if (axes.empty()) fail(line, ".etable needs at least one axis (I=, V=, TJ=)");
      try {
        out.tables[upper(toks[1])] = loss::Table(
            std::move(axes), std::move(grids), vals, hasE ? "E" : (hasR ? "R" : "VF"));
      } catch (const std::exception& e) {
        fail(line, std::string(".etable '") + toks[1] + "': " + e.what());
      }
      return;
    }
    if (dir == ".ENDS" || dir == ".END") return;  // handled by outer parse loop
    fail(line, "unknown directive '" + toks[0] + "'");
  }

  std::string ScopedNameFix(const std::string& n) const { return prefix + n; }

  void processLine(const std::string& text, int line) {
    if (++(*expanded) > kMaxExpandedLines)
      fail(line, "netlist expansion too large (subckt fan-out limit 100000 lines)");
    const auto toks = split(text);
    if (toks.empty()) return;
    if (!toks[0].empty() && toks[0][0] == '.') {
      processDirective(toks, line);
    } else {
      addDeviceLine(toks, line);
    }
  }
};

}  // namespace

double evalExpression(const std::string& expr,
                      const std::map<std::string, double>& params) {
  return evalRaw(expr, params);
}

NetlistResult Parser::parse(const std::string& text,
                             const std::map<std::string, double>& overrides) const {
  if (text.size() > kMaxNetlistBytes)
    throw std::runtime_error("netlist input exceeds 1 MiB limit");
  // 1. Split into logical lines (continuation with leading '+').
  struct L {
    int no = 0;
    std::string text;
  };
  std::vector<L> lines;
  {
    std::istringstream iss(text);
    std::string raw;
    int no = 0;
    std::string cur;
    int curNo = 0;
    auto flush = [&]() {
      if (!trim(cur).empty()) lines.push_back({curNo, cur});
      cur.clear();
    };
    while (std::getline(iss, raw)) {
      ++no;
      if (!raw.empty() && raw.back() == '\r') raw.pop_back();
      std::string t = trim(stripInlineComment(raw));
      if (t.empty() || t[0] == '*') continue;
      if (!t.empty() && t[0] == '+') {
        if (cur.empty()) fail(no, "continuation '+' with no previous line");
        cur += " " + trim(t.substr(1));
      } else {
        flush();
        cur = t;
        curNo = no;
      }
    }
    flush();
  }

  // 2. Collect subcircuit definitions; top-level statements inline.
  std::map<std::string, SubcktDef> subckts;
  struct Top {
    int no = 0;
    std::string text;
  };
  std::vector<Top> top;
  for (std::size_t i = 0; i < lines.size(); ++i) {
    auto toks = split(lines[i].text);
    if (toks.empty()) continue;
    if (ieq(toks[0], ".SUBCKT")) {
      if (toks.size() < 2) fail(lines[i].no, ".subckt needs a name");
      SubcktDef sub;
      sub.name = toks[1];
      for (std::size_t k = 2; k < toks.size(); ++k) sub.ports.push_back(toks[k]);
      if (sub.ports.empty()) fail(lines[i].no, ".subckt needs port nodes");
      ++i;
      bool closed = false;
      for (; i < lines.size(); ++i) {
        auto bt = split(lines[i].text);
        if (!bt.empty() && ieq(bt[0], ".ENDS")) {
          closed = true;
          break;
        }
        if (!bt.empty() && ieq(bt[0], ".SUBCKT")) {
          fail(lines[i].no, "nested .subckt definitions are not supported");
        }
        sub.body.push_back({lines[i].no, lines[i].text});
      }
      if (!closed) fail(lines[i - 1].no, ".subckt '" + sub.name + "' missing .ends");
      const std::string key = upper(sub.name);
      if (subckts.count(key)) fail(lines[i].no, "duplicate .subckt '" + sub.name + "'");
      subckts[key] = sub;
    } else if (ieq(toks[0], ".END")) {
      break;  // ignore everything after .end
    } else {
      top.push_back({lines[i].no, lines[i].text});
    }
  }

  // 3. Elaborate top level.
  NetlistResult out;
  std::map<std::string, double> params;
  std::map<std::string, double> lockedUpper;
  for (const auto& [k, v] : overrides) {
    params[upper(k)] = v;
    lockedUpper[upper(k)] = v;
  }
  Elaborator el{out, subckts, params, lockedUpper, {}, 1, "", {}, "", 0,
                  std::make_shared<std::size_t>(0)};
  for (const auto& t : top) el.processLine(t.text, t.no);
  out.params = params;

  // 4. Cross-checks: pwm/thermal reference existing switches/diodes.
  auto isSwitch = [&](const std::string& n) {
    try {
      return out.circuit.findDevice(n).type == DeviceType::Switch;
    } catch (...) {
      return false;
    }
  };
  for (const auto& p : out.pwms) {
    if (!isSwitch(p.switchName)) {
      throw std::runtime_error("netlist: .control pwm switch '" + p.switchName +
                               "' is not a defined switch");
    }
    if (!p.complement.empty() && !isSwitch(p.complement)) {
      throw std::runtime_error("netlist: .control pwm complement '" + p.complement +
                               "' is not a defined switch");
    }
    if (!p.complement.empty() && p.deadtime <= 0.0) {
      throw std::runtime_error("netlist: complementary pwm needs deadtime>0");
    }
  }
  for (const auto& t : out.thermals) {
    try {
      const auto& d = out.circuit.findDevice(t.device);
      if (d.type != DeviceType::Switch && d.type != DeviceType::Diode) {
        throw std::runtime_error("netlist: .thermal device '" + t.device +
                                 "' is not a switch/diode");
      }
    } catch (const std::runtime_error& e) {
      std::string m = e.what();
      if (m.find("unknown device") != std::string::npos) {
        throw std::runtime_error("netlist: .thermal device '" + t.device + "' not found");
      }
      throw;
    }
  }
  if (out.circuit.devices().empty()) throw std::runtime_error("netlist: no devices defined");
  return out;
}

std::string NetlistResult::serialize() const {
  std::ostringstream os;
  os << "* serialized by power_engine\n";
  for (const auto& [k, v] : params) os << ".param " << k << " " << v << "\n";
  for (const auto& [k, m] : models) {
    os << ".model " << m.name << " " << m.kind;
    for (const auto& [pk, pv] : m.num) os << " " << pk << "=" << pv;
    for (const auto& [pk, pv] : m.str) os << " " << pk << "=" << pv;
    os << "\n";
  }
  for (const auto& [k, t] : tables) {
    os << ".etable " << k;
    for (std::size_t i = 0; i < t.axes().size(); ++i) {
      os << " " << t.axes()[i] << "=";
      for (std::size_t j = 0; j < t.grids()[i].size(); ++j) {
        if (j != 0) os << ",";
        os << t.grids()[i][j];
      }
    }
    os << " " << (t.tag().empty() ? "E" : t.tag()) << "=";
    for (std::size_t j = 0; j < t.values().size(); ++j) {
      if (j != 0) os << ",";
      os << t.values()[j];
    }
    os << "\n";
  }
  auto nid = [](int n) { return n == 0 ? std::string("0") : "N" + std::to_string(n); };
  for (const auto& d : circuit.devices()) {
    switch (d.type) {
      case DeviceType::Resistor:
        os << d.name << " " << nid(d.n1) << " " << nid(d.n2) << " " << d.value << "\n";
        break;
      case DeviceType::Capacitor:
        os << d.name << " " << nid(d.n1) << " " << nid(d.n2) << " " << d.value << " IC=" << d.ic
           << "\n";
        break;
      case DeviceType::Inductor:
        os << d.name << " " << nid(d.n1) << " " << nid(d.n2) << " " << d.value << " IC=" << d.ic
           << "\n";
        break;
      case DeviceType::VoltageSource:
        os << d.name << " " << nid(d.n1) << " " << nid(d.n2) << " " << d.value << "\n";
        break;
      case DeviceType::CurrentSource:
        os << d.name << " " << nid(d.n1) << " " << nid(d.n2) << " " << d.value << "\n";
        break;
      case DeviceType::Switch:
        os << d.name << " " << nid(d.n1) << " " << nid(d.n2) << " RON=" << d.ron
           << " ROFF=" << d.roff << " INIT=" << (d.closed ? "ON" : "OFF");
        if (d.eon > 0.0) os << " EON=" << d.eon;
        if (d.eoff > 0.0) os << " EOFF=" << d.eoff;
        if (d.ttail > 0.0) os << " TTAIL=" << d.ttail << " TAILK=" << d.tailk;
        if (d.tsw > 0.0) os << " TSW=" << d.tsw;
        if (!d.eonTable.empty()) os << " EON_TABLE=" << d.eonTable;
        if (!d.eoffTable.empty()) os << " EOFF_TABLE=" << d.eoffTable;
        if (!d.ronTable.empty()) os << " RON_TABLE=" << d.ronTable;
        os << "\n";
        break;
      case DeviceType::Diode:
        os << d.name << " " << nid(d.n1) << " " << nid(d.n2) << " VF=" << d.vf << " RON=" << d.ron
           << " ROFF=" << d.roff;
        if (d.qrr > 0.0) os << " QRR=" << d.qrr << " TRR=" << d.trr;
        if (!d.ronTable.empty()) os << " RON_TABLE=" << d.ronTable;
        if (!d.vfTable.empty()) os << " VF_TABLE=" << d.vfTable;
        os << "\n";
        break;
      case DeviceType::Transformer:
        os << d.name << " " << nid(d.n1) << " " << nid(d.n2) << " " << nid(d.n3) << " "
           << nid(d.n4) << " RATIO=" << d.ratio << "\n";
        break;
      case DeviceType::CenterTapTransformer:
        os << d.name << " " << nid(d.n1) << " " << nid(d.n2) << " " << nid(d.n3) << " "
           << nid(d.n4) << " " << nid(d.n5) << " RATIO=" << d.ratio << "\n";
        break;
      case DeviceType::CoupledInductor: {
        const double k = d.m / std::sqrt(d.l1 * d.l2);
        os << d.name << " " << nid(d.n1) << " " << nid(d.n2) << " " << nid(d.n3) << " "
           << nid(d.n4) << " L1=" << d.l1 << " L2=" << d.l2 << " K=" << k << "\n";
        break;
      }
      case DeviceType::SatInductor:
        os << d.name << " " << nid(d.n1) << " " << nid(d.n2) << " LUNSAT=" << d.value
           << " LSAT=" << d.lsat << " ISAT=" << d.isat << " IC=" << d.ic << "\n";
        break;
      case DeviceType::HystereticInductor:
        os << d.name << " " << nid(d.n1) << " " << nid(d.n2) << " N=" << d.hTurns
           << " AE=" << d.hAe << " LE=" << d.hLe << " VE=" << d.hVe << " BS=" << d.hBs
           << " A=" << d.hA << " HC=" << d.hHc << " IC=" << d.ic << "\n";
        break;
    }
  }
  if (tran.given) {
    os << ".tran " << tran.dt << " " << tran.tstop;
    if (!tran.method.empty()) os << " " << tran.method;
    os << "\n";
  }
  for (const auto& p : pwms) {
    os << ".control pwm switch=" << p.switchName << " freq=" << p.freq << " duty=" << p.duty;
    if (p.deadtime > 0.0) os << " deadtime=" << p.deadtime;
    if (p.phase != 0.0) os << " phase=" << p.phase;
    if (!p.complement.empty()) os << " complement=" << p.complement;
    if (p.symmetric) os << " carrier=symmetric";
    os << "\n";
  }
  for (const auto& t : thermals) {
    os << ".thermal " << (t.foster ? "foster" : "cauer") << " device=" << t.device;
    for (std::size_t i = 0; i < t.r.size(); ++i) {
      os << " R" << (i + 1) << "=" << t.r[i] << " C" << (i + 1) << "=" << t.c[i];
    }
    os << " Tamb=" << t.tamb << "\n";
  }
  os << ".end\n";
  return os.str();
}

}  // namespace netlist
}  // namespace power_engine
