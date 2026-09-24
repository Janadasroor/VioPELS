// pe command framework: subcommand registry + shared argument parsing.
//
// Adding a feature = new cmd_<name>.cpp defining a Command + a
// register<Name>Command() declaration + one call in pe_main (explicit:
// linkers drop unreferenced archive members, so pure static-init
// registration silently vanishes from the binary).
//
// Exit codes (match scripts/preflight.py): 0 ok, 1 runtime/sim error,
// 2 usage error or unreadable input file.
#pragma once
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace pe {

// Parsed `--key value` / `--key=value` options (repeatable keys keep all
// values in order), bare `--flag` switches, and positional arguments.
// A `--` token ends option parsing (rest is positional).
struct ParsedArgs {
  std::map<std::string, std::vector<std::string>> opts;
  std::vector<std::string> positionals;
  bool has(const std::string& key) const {
    return opts.find(key) != opts.end();  // bare --flags count as present
  }
  // Last value (or default when absent).
  std::string get(const std::string& key, const std::string& dflt = "") const {
    const auto it = opts.find(key);
    if (it == opts.end() || it->second.empty()) return dflt;
    return it->second.back();
  }
};

// Parses argv (without argv[0]). Throws std::runtime_error on malformed
// input (dangling --key, empty key); caller maps to exit 2.
ParsedArgs parseArgs(const std::vector<std::string>& argv);

struct Command {
  std::string name;     // subcommand word, e.g. "run"
  std::string summary;  // one line for global help
  // Full help text (options, examples). Printed by `pe <cmd> --help`.
  std::string help;
  // Execute with command argv (without the subcommand word).
  // Returns process exit code (0/1/2).
  std::function<int(const ParsedArgs& args, std::ostream& out, std::ostream& err)> run;
};

void registerCommand(Command cmd);
const std::map<std::string, Command>& commands();

}  // namespace pe
