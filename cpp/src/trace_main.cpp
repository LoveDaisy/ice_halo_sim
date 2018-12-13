#include <chrono>

#include "optics.h"
#include "context.h"

using namespace IceHalo;

int main(int argc, char *argv[]) {
  if (argc != 2) {
    printf("USAGE: %s <config-file>\n", argv[0]);
    return -1;
  }

  auto start = std::chrono::system_clock::now();

  ContextParser *parser = ContextParser::createFileParser(argv[1]);
  SimulationContext context;
  try {
    parser->parseSimulationSettings(context);
  } catch (std::invalid_argument &e) {
    fprintf(stderr, "Parsing error! Exit!\n Message: %s\n", e.what());
    return -1;
  }
  context.applySettings();
  delete parser;

  auto t = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = t - start;
  printf("Initialization: %.2fms\n", diff.count() * 1.0e3);

  char filename[256];
  for (auto wl : context.getWavelengths()) {
    printf("starting at wavelength: %.1f\n", wl);

    context.setCurrentWavelength(wl);

    auto t0 = std::chrono::system_clock::now();
    Optics::traceRays(context);
    auto t1 = std::chrono::system_clock::now();
    diff = t1 - t0;
    printf("Ray tracing: %.2fms\n", diff.count() * 1.0e3);

    t0 = std::chrono::system_clock::now();
    std::sprintf(filename, "directions_%.1f_%lli.bin", wl, t0.time_since_epoch().count());
    context.writeFinalDirections(filename);

    // std::sprintf(filename, "paths_%.1f_%lli.bin", wl, t0.time_since_epoch().count());
    // context.writeRayInfo(filename, 116.5f*Math::PI/180.0f, -17.7f*Math::PI/180.0f, 0.5f/57.0f);
    t1 = std::chrono::system_clock::now();
    diff = t1 - t0;
    printf("Writing: %.2fms\n", diff.count() * 1.0e3);
  }
  context.printCrystalInfo();

  auto end = std::chrono::system_clock::now();
  diff = end - start;
  printf("Total: %.3fs\n", diff.count());

  return 0;
}
