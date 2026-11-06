// pe steady-state command entry (also the test seam, like runNetlist).
#pragma once
#include <ostream>
#include <string>

#include "pe.h"

namespace pe {
// Executes `steady-state` (see kHelp in cmd_steadystate.cpp). Returns 0/1/2.
int runSteadyState(const ParsedArgs& args, std::ostream& out, std::ostream& err);
const char* steadyStateHelp();
// Registers `steady-state` (called once from pe_main; explicit beats
// static-init roulette).
void registerSteadyStateCommand();
}  // namespace pe
