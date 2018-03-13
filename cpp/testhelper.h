#ifndef TESTHELPER_H
#define TESTHELPER_H

#include "geometry.h"
#include "optics.h"

#include <vector>

class TestContext
{
public:
    TestContext();
    ~TestContext();

    void writeFinalDirections(const std::vector<Ray*> &rays);
    void writeGeometryInfo();
    void writeRayPaths(const std::vector<Ray*> &rays);
    
    RayTracingParam param;
    Geometry *g;

    const int incDirNum;
    float *rayDir;
    float *mainAxRot;
};

#endif // TESTHELPER_H