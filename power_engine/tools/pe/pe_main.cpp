// pe: VioPELS command line (thin dispatch wrapper; see pe.h).
//
//   pe [--help | --version | <command> [--help] [options]]
//
// New features register via register<Name>Command() (one call below per
// command TU) + runNetlist-style test seams for behavior.
#include <iostream>
#include <string>
#include <vector>

#include "cmd_run.h"
#include "cmd_sweep.h"
#include "pe.h"

int main(int argc, char** argv) {
  pe::registerRunCommand();
  pe::registerSweepCommand();
  return pe::dispatch(std::vector<std::string>(argv + 1, argv + argc), std::cout, std::cerr);
}
