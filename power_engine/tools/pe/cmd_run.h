// pe run command entry (also the test seam: tests drive runNetlist()
// in-process and assert return codes + CSV, no spawned processes).
#pragma once
#include <ostream>
#include <string>

#include "pe.h"

namespace pe {
// Executes `run` (see kHelp in cmd_run.cpp). Returns 0/1/2.
int runNetlist(const ParsedArgs& args, std::ostream& out, std::ostream& err);
const char* runHelp();
// Registers `run` (called once from pe_main; explicit beats static-init
// roulette: linkers drop unreferenced archive members, silently
// unregistering commands).
void registerRunCommand();
}  // namespace pe
