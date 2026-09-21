#include <cstdio>

#include "power_engine/magnetics.h"

// Saturable-toroid lambda-I demo: N=100 on a 3-segment core loop.
// Prints CSV i,lambda (shows saturation knee).
int main() {
  power_engine::magnetics::ReluctanceNetwork net;
  net.addSaturableReluctance("coreA", 0, 2, 0.1, 1e-4, 1.5, 100.0);
  net.addSaturableReluctance("coreB", 2, 3, 0.1, 1e-4, 1.5, 100.0);
  net.addSaturableReluctance("coreC", 3, 0, 0.1, 1e-4, 1.5, 100.0);
  net.addWinding("W1", "coreA", 100.0);
  std::printf("current,lambda\n");
  for (double i = 0.02; i <= 5.01; i *= 1.5) {
    net.setWindingCurrent("W1", i);
    net.solve();
    std::printf("%.9f,%.9f\n", i, 100.0 * net.windingFlux("W1"));
  }
  return 0;
}
