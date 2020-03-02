#include <chrono>

#include "context/context.hpp"
#include "core/simulation.hpp"
#include "util/log.hpp"

using namespace icehalo;

int main(int argc, char* argv[]) {
  if (argc != 2) {
    printf("USAGE: %s <config-file>\n", argv[0]);
    return -1;
  }

  ENABLE_LOG_SEVERITY;
  DISABLE_LOG_THREAD_ID;
  auto start = std::chrono::system_clock::now();
  ProjectContextPtr context = ProjectContext::CreateFromFile(argv[1]);
  Simulator simulator(context);

  auto t = std::chrono::system_clock::now();
  std::chrono::duration<float, std::ratio<1, 1000>> diff = t - start;
  LOG_INFO("Initialization: %.2fms", diff.count());

  const auto& wavelengths = context->wavelengths_;
  for (decltype(wavelengths.size()) i = 0; i < wavelengths.size(); i++) {
    const auto& wl = wavelengths[i];
    LOG_INFO("starting at wavelength: %d", wl.wavelength);
    simulator.SetCurrentWavelengthIndex(i);

    auto t0 = std::chrono::system_clock::now();
    simulator.Run();
    auto t1 = std::chrono::system_clock::now();
    diff = t1 - t0;
    LOG_INFO("Ray tracing: %.2fms", diff.count());
  }
  context->PrintCrystalInfo();
  simulator.PrintRayInfo();

  auto end = std::chrono::system_clock::now();
  diff = end - start;
  LOG_INFO("Total: %.3fs", diff.count() / 1e3);

  return 0;
}
