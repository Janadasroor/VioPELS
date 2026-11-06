// pe ac command entry (also the test seam, like runNetlist).
#pragma once
#include <ostream>
#include <string>

#include "pe.h"

namespace pe {
// Executes `ac` (see kHelp in cmd_ac.cpp). Returns 0/1/2.
int runAc(const ParsedArgs& args, std::ostream& out, std::ostream& err);
const char* acHelp();
// Registers `ac` (called once from pe_main).
void registerAcCommand();
}  // namespace pe
