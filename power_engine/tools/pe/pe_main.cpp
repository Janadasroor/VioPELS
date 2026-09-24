// pe: VioPELS command line (subcommand dispatch; see pe.h).
//
//   pe [--help | --version | <command> [--help] [options]]
//
// New features register via registerCommand() (one line per command TU).
#ifndef PE_VERSION
#define PE_VERSION "dev"
#endif

#include <iostream>
#include <string>
#include <vector>

#include "pe.h"
#include "cmd_run.h"

namespace {
void printGlobalHelp(std::ostream& o) {
  o << "pe: VioPELS power-electronics CLI (version " << PE_VERSION << ")\n\nCommands:\n";
  for (const auto& [name, cmd] : pe::commands()) o << "  " << name << " — " << cmd.summary << "\n";
  o << "\nRun `pe <command> --help` for command options.\n";
}
}  // namespace

int main(int argc, char** argv) {
  pe::registerRunCommand();
  std::vector<std::string> args(argv + 1, argv + argc);
  if (args.empty() || args[0] == "--help" || args[0] == "-h") {
    const bool usageError = args.empty();
    printGlobalHelp(usageError ? std::cerr : std::cout);
    return usageError ? 2 : 0;
  }
  if (args[0] == "--version" || args[0] == "-V") {
    std::cout << "pe " << PE_VERSION << "\n";
    return 0;
  }
  const auto& cmds = pe::commands();
  const auto it = cmds.find(args[0]);
  if (it == cmds.end()) {
    std::cerr << "pe: unknown command '" << args[0] << "'\n\n";
    printGlobalHelp(std::cerr);
    return 2;
  }
  std::vector<std::string> rest(args.begin() + 1, args.end());
  for (const auto& a : rest) {
    if (a == "--") break;  // explicit end of options wins
    if (a == "--help" || a == "-h") {
      std::cout << it->second.help;
      return 0;
    }
  }
  pe::ParsedArgs parsed;
  try {
    parsed = pe::parseArgs(rest);
  } catch (const std::exception& e) {
    std::cerr << "pe " << it->first << ": " << e.what() << "\n";
    return 2;
  }
  try {
    return it->second.run(parsed, std::cout, std::cerr);
  } catch (const std::exception& e) {
    std::cerr << "pe " << it->first << ": internal error: " << e.what() << "\n";
    return 1;
  }
}
