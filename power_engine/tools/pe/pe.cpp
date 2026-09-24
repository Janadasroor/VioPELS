// pe argument parsing + command registry (see pe.h).
#include "pe.h"

#ifndef PE_VERSION
#define PE_VERSION "dev"
#endif

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

void printGlobalHelp(std::ostream& o, const char* version) {
  o << "pe: VioPELS power-electronics CLI (version " << version << ")\n\nCommands:\n";
  for (const auto& [name, cmd] : commands()) o << "  " << name << " — " << cmd.summary << "\n";
  o << "\nRun `pe <command> --help` for command options.\n";
}

int dispatch(const std::vector<std::string>& args, std::ostream& out, std::ostream& err) {
  if (args.empty() || args[0] == "--help" || args[0] == "-h") {
    const bool usageError = args.empty();
    printGlobalHelp(usageError ? err : out, PE_VERSION);
    return usageError ? 2 : 0;
  }
  if (args[0] == "--version" || args[0] == "-V") {
    out << "pe " << PE_VERSION << "\n";
    return 0;
  }
  const auto& cmds = commands();
  const auto it = cmds.find(args[0]);
  if (it == cmds.end()) {
    err << "pe: unknown command '" << args[0] << "'\n\n";
    printGlobalHelp(err, PE_VERSION);
    return 2;
  }
  std::vector<std::string> rest(args.begin() + 1, args.end());
  for (const auto& a : rest) {
    if (a == "--") break;  // explicit end of options wins
    if (a == "--help" || a == "-h") {
      out << it->second.help;
      return 0;
    }
  }
  ParsedArgs parsed;
  try {
    parsed = parseArgs(rest);
  } catch (const std::exception& e) {
    err << "pe " << it->first << ": " << e.what() << "\n";
    return 2;
  }
  try {
    return it->second.run(parsed, out, err);
  } catch (const std::exception& e) {
    err << "pe " << it->first << ": internal error: " << e.what() << "\n";
    return 1;
  }
}

}  // namespace pe
