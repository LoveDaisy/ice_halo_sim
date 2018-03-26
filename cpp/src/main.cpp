#include <vector>
#include <chrono>

#include "geometry.h"
#include "optics.h"
#include "testhelper.h"

int main(int argc, char *argv[])
{
    auto start = std::chrono::system_clock::now();

    RayTracingContext context;
    context.setIncDirNum(300000);
    context.setRaysPerDirection(20);
    context.setSunPosition(90.0f*Geometry::PI/180.0f, 10.0f*Geometry::PI/180.0f);
    context.setGeometry(Geometry::createHexCylindar(2.0f));

    context.oriGen.setAxisOrientation(OrientationGenerator::AxisDistribution::AX_SPH_UNIFORM, 0.0f);
    context.oriGen.setAxisRoll(OrientationGenerator::RollDistribution::ROLL_UNIFORM, 0.0f);

    context.applySettings();

    auto t = std::chrono::system_clock::now();
    printf("Initialization: %.2fms\n", (t - start).count() / 1.0e3);

    for (float wl = 440.0f; ;) {
        if (wl > 655) {
            break;
        }
        printf("starting at wavelength: %.1f\n", wl);
        context.setWavelength(wl);

        auto t0 = std::chrono::system_clock::now();
        context.clearRays();
        auto t1 = std::chrono::system_clock::now();
        printf("Clearing: %.2fms\n", (t1 - t0).count() / 1.0e3);

        t0 = std::chrono::system_clock::now();
        Optics::traceRays(context);
        t1 = std::chrono::system_clock::now();
        printf("Ray tracing: %.2fms\n", (t1 - t0).count() / 1.0e3);

        t0 = std::chrono::system_clock::now();
        context.writeFinalDirections();
        t1 = std::chrono::system_clock::now();
        printf("Writing: %.2fms\n", (t1 - t0).count() / 1.0e3);

        wl += 30.0f;
    }
    auto end = std::chrono::system_clock::now();
    printf("Total: %.3fs\n", (end - start).count() / 1.0e6);

    return 0;
}
