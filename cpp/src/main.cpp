#include <vector>
#include <chrono>
#include <thread>

#include "geometry.h"
#include "optics.h"
#include "processhelper.h"

constexpr auto CRYSTAL_NUM = 2;


int main(int argc, char *argv[])
{
    auto start = std::chrono::system_clock::now();

    RayTracingContext contexts[CRYSTAL_NUM];
    float crystalNumRatio[CRYSTAL_NUM] = { 2.0f, 1.0f };
    float total = 0.0f;
    for (float r : crystalNumRatio) {
        total += r;
    }
    for (float &r : crystalNumRatio) {
        r /= total;
    }
    int totalIncDirNum = 100000;

    /* Crystal 1 */
    contexts[0].setIncDirNum(static_cast<int>(totalIncDirNum * crystalNumRatio[0]));
    contexts[0].setRaysPerDirection(10);
    contexts[0].setSunPosition(90.0f*Geometry::PI/180.0f, 42.0f*Geometry::PI/180.0f);
    contexts[0].setGeometry(Geometry::createHexCylindar(3.0f));

    contexts[0].oriGen.setAxisOrientation(OrientationGenerator::AxisDistribution::AX_HOR_GAUSS, 0.2f*3.1416f/180.f);
    contexts[0].oriGen.setAxisRoll(OrientationGenerator::RollDistribution::ROLL_UNIFORM, 0.0f);

    contexts[0].applySettings();

    /* Crystal 2 */
    contexts[1].setIncDirNum(static_cast<int>(totalIncDirNum * crystalNumRatio[1]));
    contexts[1].setRaysPerDirection(10);
    contexts[1].setSunPosition(90.0f*Geometry::PI/180.0f, 42.0f*Geometry::PI/180.0f);
    contexts[1].setGeometry(Geometry::createHexCylindar(3.0f));

    contexts[1].oriGen.setAxisOrientation(OrientationGenerator::AxisDistribution::AX_SPH_UNIFORM, 0.2f*3.1416f/180.f);
    contexts[1].oriGen.setAxisRoll(OrientationGenerator::RollDistribution::ROLL_UNIFORM, 0.0f);

    contexts[1].applySettings();

    auto t = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = t - start;
    printf("Initialization: %.2fms\n", diff.count() * 1.0e3);

    for (float wl = 440.0f; ;) {
        if (wl > 655) {
            break;
        }
        printf("starting at wavelength: %.1f\n", wl);

        for (auto &ctx : contexts) {
            ctx.setWavelength(wl);

            auto t0 = std::chrono::system_clock::now();
            ctx.clearRays();
            auto t1 = std::chrono::system_clock::now();
            diff = t1 - t0;
            printf("Clearing: %.2fms\n", diff.count() * 1.0e3);

            t0 = std::chrono::system_clock::now();
            Optics::traceRays(ctx);
            t1 = std::chrono::system_clock::now();
            diff = t1 - t0;
            printf("Ray tracing: %.2fms\n", diff.count() * 1.0e3);

            char filename[128];
            t0 = std::chrono::system_clock::now();
            std::sprintf(filename, "directions_%.1f_%lli.bin", wl, t0.time_since_epoch().count());
            ctx.writeFinalDirections(filename);
            t1 = std::chrono::system_clock::now();
            diff = t1 - t0;
            printf("Writing: %.2fms\n", diff.count() * 1.0e3);
        }

        wl += 30.0f;
    }

    auto end = std::chrono::system_clock::now();
    diff = end - start;
    printf("Total: %.3fs\n", diff.count());

    return 0;
}
