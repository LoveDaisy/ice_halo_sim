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

  auto start = std::chrono::system_clock::now();
  ProjectContextPtr context = ProjectContext::CreateFromFile(argv[1]);
  Simulator simulator(context);

  auto t = std::chrono::system_clock::now();
  std::chrono::duration<float, std::milli> diff = t - start;
  LOG_INFO("Initialization: %.2fms", diff.count());

  char filename[256];
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

    t0 = std::chrono::system_clock::now();
    std::sprintf(filename, "directions_%d_%lli.bin", wl.wavelength, t0.time_since_epoch().count());
    icehalo::File file(context->GetDataDirectory().c_str(), filename);
    file.Open(icehalo::FileOpenMode::kWrite);
    simulator.GetSimulationRayData().Serialize(file, true);

    t1 = std::chrono::system_clock::now();
    diff = t1 - t0;
    LOG_INFO("Saving: %.2fms", diff.count());
  }

  auto end = std::chrono::system_clock::now();
  diff = end - start;
  LOG_INFO("Total: %.3fs", diff.count() / 1e3);

  return 0;
}
