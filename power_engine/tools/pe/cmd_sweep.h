// pe sweep command entry (also the test seam, like runNetlist).
#pragma once
#include <ostream>
#include <string>

#include "pe.h"

namespace pe {
// Executes `sweep` (see kHelp in cmd_sweep.cpp). Returns 0/1/2.
int runSweepCommand(const ParsedArgs& args, std::ostream& out, std::ostream& err);
const char* sweepHelp();
void registerSweepCommand();
}  // namespace pe
