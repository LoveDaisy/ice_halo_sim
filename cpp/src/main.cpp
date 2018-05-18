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
    context.setTotalRayNum(10000);
    context.setMaxRecursionNum(9);
    context.envCtx->setSunPosition(90.0f*Geometry::PI/180.0f, 42.0f*Geometry::PI/180.0f);
    context.crystalCtx->addGeometry(Geometry::createHexCylindar(3.0f), 2.0f,
                                    OrientationGenerator::AxisDistribution::AX_HOR_GAUSS, 0.2f*Geometry::PI/180.f,
                                    OrientationGenerator::RollDistribution::ROLL_UNIFORM, 0.0f);
    context.crystalCtx->addGeometry(Geometry::createHexCylindar(3.0f), 1.0f,
                                    OrientationGenerator::AxisDistribution::AX_SPH_UNIFORM, 0.0f,
                                    OrientationGenerator::RollDistribution::ROLL_UNIFORM, 0.0f);
    context.applySettings();

    auto t = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = t - start;
    printf("Initialization: %.2fms\n", diff.count() * 1.0e3);

    char filename[256];
    for (float wl = 440.0f; ;) {
        if (wl > 655) {
            break;
        }
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
