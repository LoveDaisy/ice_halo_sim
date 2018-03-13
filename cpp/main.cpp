#include <vector>
#include <chrono>
#include <random>
#include <cmath>
#include <cstdio>

#include "geometry.h"
#include "optics.h"
#include "linearalgebra.h"
#include "testhelper.h"

int main(int argc, char *argv[])
{
    TestContext context;

    std::vector<Ray*> rays;

    auto t0 = std::chrono::system_clock::now();
    Optics::traceRays(context.incDirNum, context.rayDir, context.param, context.g, rays);
    auto t1 = std::chrono::system_clock::now();
    printf("%.2fms\n", (t1 - t0).count() / 1000.0);

    context.writeFinalDirections(rays);
    // context.writeGeometryInfo();
    // context.writeRayPaths(rays);

    return 0;
}
