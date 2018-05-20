#include <vector>
#include <chrono>
#include <thread>

#include "geometry.h"
#include "optics.h"
#include "processhelper.h"


int main(int argc, char *argv[])
{
    auto start = std::chrono::system_clock::now();

    SimulationContext context;
    context.setTotalRayNum(1000000);
    context.setMaxRecursionNum(9);
    context.envCtx->setSunPosition(90.0f*Geometry::PI/180.0f, 1.0f*Geometry::PI/180.0f);
    context.crystalCtx->addGeometry(Geometry::createHexPyramid(1.47f, 0.0, 1.47f), 2.0f,
                                    OrientationGenerator::AxisDistribution::AX_ZENITHAL_GAUSS, 30.0f*Geometry::PI/180.f,
                                    OrientationGenerator::RollDistribution::ROLL_UNIFORM, 0.0f);
    context.crystalCtx->addGeometry(Geometry::createHexPyramid(0.31f, 0.f, 1.30), 3.0f,
                                    OrientationGenerator::AxisDistribution::AX_ZENITHAL_GAUSS, 3.4f,
                                    OrientationGenerator::RollDistribution::ROLL_UNIFORM, 0.0f);
    context.crystalCtx->addGeometry(Geometry::createCubicPyramid(0.6f, 1.0f), 2.0f,
                                    OrientationGenerator::AxisDistribution::AX_HOR_GAUSS, 14.0f*Geometry::PI/180.f,
                                    OrientationGenerator::RollDistribution::ROLL_HOR_GAUSS, 2.6f*Geometry::PI/180.0f);
    context.applySettings();

    auto t = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = t - start;
    printf("Initialization: %.2fms\n", diff.count() * 1.0e3);

    char filename[256];
    for (float wl = 440.0f; wl < 655.0f;) {
        printf("starting at wavelength: %.1f\n", wl);

        context.envCtx->setWavelength(wl);

        auto t0 = std::chrono::system_clock::now();
        Optics::traceRays(context);
        auto t1 = std::chrono::system_clock::now();
        diff = t1 - t0;
        printf("Ray tracing: %.2fms\n", diff.count() * 1.0e3);

        t0 = std::chrono::system_clock::now();
        std::sprintf(filename, "directions_%.1f_%lli.bin", wl, t0.time_since_epoch().count());
        context.writeFinalDirections(filename);

        // std::sprintf(filename, "paths_%.1f_%lli.bin", wl, t0.time_since_epoch().count());
        // context.writeRayInfo(filename, -104.5f*Geometry::PI/180.0f, -30.7f*Geometry::PI/180.0f, 0.5f/57.0f);
        t1 = std::chrono::system_clock::now();
        diff = t1 - t0;
        printf("Writing: %.2fms\n", diff.count() * 1.0e3);

        wl += 30.0f;
    }

    auto end = std::chrono::system_clock::now();
    diff = end - start;
    printf("Total: %.3fs\n", diff.count());

    return 0;
}
