// pe argument parsing + command registry (see pe.h).
#include "pe.h"

#include <ostream>
#include <stdexcept>

namespace pe {

ParsedArgs parseArgs(const std::vector<std::string>& argv) {
  ParsedArgs out;
  bool optsOpen = true;
  for (std::size_t i = 0; i < argv.size(); ++i) {
    const std::string& a = argv[i];
    if (optsOpen && a == "--") {
      optsOpen = false;
      continue;
    }
    if (optsOpen && a.size() > 2 && a[0] == '-' && a[1] == '-') {
      const auto eq = a.find('=', 2);
      if (eq != std::string::npos) {
        const std::string key = a.substr(2, eq - 2);
        if (key.empty()) throw std::runtime_error("empty option name in '" + a + "'");
        out.opts[key].push_back(a.substr(eq + 1));
      } else {
        const std::string key = a.substr(2);
        if (key.empty()) throw std::runtime_error("empty option name in '" + a + "'");
        if (i + 1 < argv.size() && !(argv[i + 1].size() > 1 && argv[i + 1][0] == '-')) {
          out.opts[key].push_back(argv[++i]);
        } else {
          out.opts[key];  // bare --flag (empty value list = present)
        }
      }
      continue;
    }
    out.positionals.push_back(a);
  }
  return out;
}

namespace {
std::map<std::string, Command>& registry() {
  static std::map<std::string, Command> r;
  return r;
}
}  // namespace

void registerCommand(Command cmd) { registry()[cmd.name] = std::move(cmd); }

const std::map<std::string, Command>& commands() { return registry(); }

}  // namespace pe
