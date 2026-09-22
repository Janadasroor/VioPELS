// libFuzzer target for the SPICE-like netlist parser (roadmap item 15).
// Throws are correct behavior (invalid input rejected); the fuzzer hunts
// crashes, hangs, OOMs and sanitizer violations. Also exercises the
// serialize() round-trip on accepted inputs.
#include <cstddef>
#include <cstdint>
#include <string>

#include "power_engine/netlist.h"

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size) {
  if (size == 0 || size > 4096) return 0;
  const std::string text(reinterpret_cast<const char*>(data), size);
  try {
    power_engine::netlist::Parser parser;
    const auto net = parser.parse(text, {});
    // Round-trip accepted inputs (may throw on unprintable corners — fine).
    try {
      (void)net.serialize();
    } catch (...) {
    }
  } catch (...) {
    // Rejection is the documented contract for invalid netlists.
  }
  return 0;
}
