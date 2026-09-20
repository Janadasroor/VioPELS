#pragma once
#include <map>
#include <string>
#include <vector>

#include "power_engine/circuit.h"

namespace power_engine {
namespace netlist {

/// A `.model` definition: kind (e.g. "mosfet_ideal") + numeric/string params.
struct ModelDef {
  std::string name;
  std::string kind;
  std::map<std::string, double> num;     ///< RON, ROFF, VTO, VF, EON, ...
  std::map<std::string, std::string> str;
};

/// A `.tran dt tstop` directive.
struct TranSpec {
  bool given = false;
  double dt = 1e-6;
  double tstop = 0.0;
};

/// A `.control pwm ...` directive: open-loop gate drive for one switch
/// (Phase 4 consumes this to generate exact scheduled edges).
struct PwmSpec {
  std::string switchName;
  double freq = 20e3;
  double duty = 0.5;
  double deadtime = 0.0;
  double phase = 0.0;                 ///< phase shift [s]
  std::string complement;             ///< optional complementary switch
  bool symmetric = false;             ///< false = trailing-edge, true = centered
};

/// A `.thermal foster|cauer ...` directive (Phase 5 consumes this).
struct ThermalSpec {
  std::string device;                 ///< switch/diode name (resolved at parse)
  bool foster = true;                 ///< false = cauer
  std::vector<double> r;              ///< stage resistances [K/W]
  std::vector<double> c;              ///< stage capacitances [J/K]
  double tamb = 25.0;                 ///< ambient [degC]
};

/// Fully elaborated netlist: circuit + simulation directives.
struct NetlistResult {
  Circuit circuit;
  std::map<std::string, double> params;
  std::map<std::string, ModelDef> models;
  TranSpec tran;
  std::vector<PwmSpec> pwms;
  std::vector<ThermalSpec> thermals;

  /// Canonical re-serialization (round-trip: serialize -> parse -> equal).
  std::string serialize() const;
};

/// SPICE-like system-level netlist parser.
///
/// Supported statements (case-insensitive keywords, `*` full-line and `;`
/// inline comments, `+` continuation lines):
/// ```
/// .param NAME value [NAME value ...]      (values may be expressions)
/// .model NAME KIND k=v ...                (mosfet_ideal, igbt_ideal, diode_ideal)
/// Rname n1 n2 value
/// Cname n1 n2 value [IC=v0]
/// Lname n1 n2 value [IC=i0]
/// Vname np nm value | Vname np nm DC value
/// Iname np nm value | Iname np nm DC value
/// Sname n1 n2 [MODEL=m] [RON=..] [ROFF=..] [INIT=ON|OFF]
/// Mname d s MODEL=m ...  (ideal MOSFET, logic gate: alias of S)
/// Qname c e MODEL=m ...  (ideal IGBT, logic gate: alias of S)
/// Dname anode cathode [MODEL=m] [VF=..] [RON=..] [ROFF=..]
/// Tname n1 n2 n3 n4 [RATIO=n]   (ideal transformer, n:1)
/// Wname n1 n2 n3 n4 L1=.. L2=.. (K=..|M=..) [IC1=..] [IC2=..]
/// Yname n1 n2 LUNSAT=.. LSAT=.. ISAT=.. [IC=..]  (saturable inductor)
/// Xname nodes... subcktName      (subcircuit instance)
/// .subckt NAME ports... / .ends
/// .tran dt tstop
/// .control pwm switch=S freq=.. duty=.. [deadtime=..] [complement=S2] [phase=..]
/// .thermal foster|cauer [device=D] R1=.. C1=.. ... [Tamb=..]
/// .end
/// ```
/// Values accept SI suffixes (f p n u m k Meg G T) and `{expressions}` over
/// `.param` names. Nodes: `0`, `GND`, `GROUND` = ground; other names map to
/// integer ids. MOSFET/IGBT gates are logic-level, driven via `.control pwm`
/// (documented system-level idealization: VTO is recorded, not simulated).
/// A bare `.thermal` without `device=` attaches to the most recently defined
/// switch/diode. Unknown directives or devices throw with line numbers.
class Parser {
 public:
  /// Parse + elaborate. `overrides` (UPPER-name -> value) pre-seed the
  /// parameter table and win over in-file `.param` lines (used by
  /// Engine::setParameter for exact re-elaboration).
  NetlistResult parse(const std::string& text,
                      const std::map<std::string, double>& overrides = {}) const;
};

/// Evaluate an arithmetic expression over params (used by tests/tools).
double evalExpression(const std::string& expr,
                      const std::map<std::string, double>& params);

}  // namespace netlist
}  // namespace power_engine
