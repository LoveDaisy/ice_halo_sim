#include <vector>
#include <chrono>
#include <random>
#include <cmath>
#include <cstdio>

#include "geometry.h"
#include "optics.h"
#include "linearalgebra.h"

int main(int argc, char *argv[])
{
    float lon = 30.0f * 3.14159f / 180.0f;
    float lat = -15.0f * 3.14159f / 180.0f;
    float roll = 0.3f;

    float rayDir[3] = {0.3f, 0.2f, 1.0f};
    LinearAlgebra::normalize3(rayDir);
    printf("[%.4f,%.4f,%.4f]\n", rayDir[0], rayDir[1], rayDir[2]);
    
    LinearAlgebra::rotateZ(lon, lat, roll, 1, rayDir);

    printf("[%.4f,%.4f,%.4f]\n", rayDir[0], rayDir[1], rayDir[2]);

    LinearAlgebra::rotateZBack(lon, lat, roll, 1, rayDir);
    printf("[%.4f,%.4f,%.4f]\n", rayDir[0], rayDir[1], rayDir[2]);

    return 0;
}
