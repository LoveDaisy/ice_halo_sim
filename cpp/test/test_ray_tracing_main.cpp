#include <chrono>

#include "context/context.h"
#include "core/simulation.h"

using namespace icehalo;

int main(int argc, char* argv[]) {
  if (argc != 2) {
    printf("USAGE: %s <config-file>\n", argv[0]);
    return -1;
  }

  auto start = std::chrono::system_clock::now();
  ProjectContextPtr context = ProjectContext::CreateFromFile(argv[1]);
  Simulator simulator(context);

  auto t = std::chrono::system_clock::now();
  std::chrono::duration<float, std::ratio<1, 1000>> diff = t - start;
  printf("Initialization: %.2fms\n", diff.count());

  const auto& wavelengths = context->wavelengths_;
  for (decltype(wavelengths.size()) i = 0; i < wavelengths.size(); i++) {
    const auto& wl = wavelengths[i];
    printf("starting at wavelength: %d\n", wl.wavelength);
    simulator.SetCurrentWavelengthIndex(i);

    auto t0 = std::chrono::system_clock::now();
    simulator.Run();
    auto t1 = std::chrono::system_clock::now();
    diff = t1 - t0;
    printf("Ray tracing: %.2fms\n", diff.count());
  }
  context->PrintCrystalInfo();
  simulator.PrintRayInfo();

  auto end = std::chrono::system_clock::now();
  diff = end - start;
  printf("Total: %.3fs\n", diff.count() / 1e3);

  return 0;
}
