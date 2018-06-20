#include <chrono>

#include "optics.h"
#include "context.h"
#include "render.h"

using namespace IceHalo;

int main(int argc, char *argv[])
{
    if (argc != 2) {
        printf("USAGE: %s <config-file>\n", argv[0]);
        return -1;
    }

    auto start = std::chrono::system_clock::now();

    ContextParser *parser = ContextParser::createFileParser(argv[1]);
    SimulationContext context;
    try {
        parser->parseSettings(context);
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
    for (float wl = 560.0f; wl < 655.0f;) {
        printf("starting at wavelength: %.1f\n", wl);

        context.setWavelength(wl);

        auto t0 = std::chrono::system_clock::now();
        Optics::traceRays(context);
        auto t1 = std::chrono::system_clock::now();
        diff = t1 - t0;
        printf("Ray tracing: %.2fms\n", diff.count() * 1.0e3);

        t0 = std::chrono::system_clock::now();
        std::sprintf(filename, "directions_%.1f_%lli.bin", wl, t0.time_since_epoch().count());
        context.writeFinalDirections(filename);

        // std::sprintf(filename, "paths_%.1f_%lli.bin", wl, t0.time_since_epoch().count());
        // context.writeRayInfo(filename, -104.5f*Crystal::PI/180.0f, -30.7f*Crystal::PI/180.0f, 0.5f/57.0f);
        t1 = std::chrono::system_clock::now();
        diff = t1 - t0;
        printf("Writing: %.2fms\n", diff.count() * 1.0e3);

        wl += 30.0f;
    }
    context.printCrystalInfo();

    // float camRot[] = { 90.0f, 0.0f, 0.0f };
    // float dir[] = { 0.0f, 0.9455f, 0.3256f };
    // int imgXY[2];
    // EquiAreaCameraProjection proj;
    // proj.project(camRot, 120, 1, dir, 1801, 1801, imgXY);

    // printf("%d, %d\n", imgXY[0], imgXY[1]); 

    auto end = std::chrono::system_clock::now();
    diff = end - start;
    printf("Total: %.3fs\n", diff.count());

    return 0;
}
