#ifndef TESTHELPER_H
#define TESTHELPER_H

#include "geometry.h"
#include "optics.h"

#include <vector>
#include <random>


class OrientationGenerator
{
public:
    enum class AxisDistribution
    {
        AX_SPH_UNIFORM,
        AX_ZENITHAL_GAUSS,
        AX_HOR_GAUSS
    };

    enum class RollDistribution
    {
        ROLL_UNIFORM,
        ROLL_HOR
    };

public:
    OrientationGenerator(float axStd, float rollStd,
        AxisDistribution ax = AxisDistribution::AX_SPH_UNIFORM,
        RollDistribution roll = RollDistribution::ROLL_UNIFORM);
    ~OrientationGenerator() = default;

    void fillData(const float *sunDir, int num, float *rayDir, float *mainAxRot);
    void setAxisOrientation(AxisDistribution ax, float axStd);
    void setAxisRoll(RollDistribution roll, float rollStd);

private:
    std::mt19937 generator;
    std::normal_distribution<float> gaussDistribution;
    std::uniform_real_distribution<float> uniformDistribution;

    AxisDistribution axDist;
    RollDistribution rollDist;

    float axStd;
    float rollStd;
};



class TestContext
{
public:
    TestContext();
    ~TestContext();

    const Geometry * getGeometry() const;
    const float * getRayDirections() const;
    const float * getMainAxisRotations() const;
    int getIncDirNum() const;

    void setWavelength(float wl);
    void setIncDirNum(int incDirNum);
    void setSunPosition(float lon, float lat);
    void setGeometry(Geometry *g);
    
    bool isSettingsApplied();
    void applySettings();

    void writeFinalDirections(const std::vector<Ray*> &rays);
    void writeGeometryInfo();
    void writeRayPaths(const std::vector<Ray*> &rays);
    
    RayTracingParam param;
    OrientationGenerator oriGen;
    Geometry *g;

private:
    int incDirNum;
    float *rayDir;
    float *mainAxRot;
    float sunDir[3];

    float wl;

    bool initialized;
};

#endif // TESTHELPER_H