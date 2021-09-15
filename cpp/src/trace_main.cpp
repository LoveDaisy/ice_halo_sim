#include <chrono>

#include "context/context.hpp"
#include "process/simulation.hpp"
#include "util/arg_parser.hpp"
#include "util/log.hpp"


int main(int argc, char* argv[]) {
  icehalo::ArgParser parser;
  parser.AddArgument("-v", 0, "verbose", "make output verbose");
  parser.AddArgument("-f", 1, "config-file", "config file");
  icehalo::ArgParseResult arg_parse_result;
  try {
    arg_parse_result = parser.Parse(argc, argv);
  } catch (...) {
    return -1;
  }

  const char* config_filename = arg_parse_result.at("-f")[0].c_str();
  if (arg_parse_result.count("-v")) {
    icehalo::LogFilterPtr stdout_filter = icehalo::LogFilter::MakeLevelFilter({ icehalo::LogLevel::kVerbose });
    icehalo::LogDestPtr stdout_dest = icehalo::LogStdOutDest::GetInstance();
    icehalo::Logger::GetInstance()->AddDestination(stdout_filter, stdout_dest);
  }

  auto start = std::chrono::system_clock::now();
  icehalo::ProjectContextPtr context = icehalo::ProjectContext::CreateFromFile(config_filename);
  icehalo::Simulator simulator(context);

  auto t = std::chrono::system_clock::now();
  std::chrono::duration<float, std::milli> diff = t - start;
  LOG_INFO("Initialization: %.2fms", diff.count());

  char filename[256];
  const auto& wavelengths = context->wavelengths_;
  for (size_t i = 0; i < wavelengths.size(); i++) {
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
