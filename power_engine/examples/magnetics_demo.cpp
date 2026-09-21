#include <cmath>
#include <cstdio>

#include "power_engine/magnetics.h"

// Hysteresis loop demo: Bs=1T, a=100A/m, Hc=50A/m driven +-1000A/m.
// Prints CSV h,b over two cycles.
int main() {
  power_engine::magnetics::HysteresisCore core({1.0, 100.0, 50.0});
  std::printf("h,b\n");
  for (int k = 0; k < 800; ++k) {
    const double h = 1000.0 * std::sin(2.0 * 3.141592653589793 * k / 400.0);
    std::printf("%.9f,%.9f\n", h, core.update(h));
  }
  return 0;
}
