#ifndef TESTHELPER_H
#define TESTHELPER_H

#include "geometry.h"
#include "optics.h"

#include <vector>
#include <set>
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
        ROLL_HOR_GAUSS
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


class RaySegmentFactory
{
public:
    ~RaySegmentFactory();

    static RaySegmentFactory * getInstance();
    
    RaySegment * getRaySegment(float *pt, float *dir, float w, int faceId);
    void clear();

private:
    RaySegmentFactory();

    static RaySegmentFactory *instance;
    static const uint32_t chunkSize = 1024*64;
    
    std::vector<RaySegment*> segments;
    uint32_t nextUnusedId;
    uint32_t currentChunkId;
};


class RayTracingContext
{
public:
    RayTracingContext();
    ~RayTracingContext();

    const Geometry * getGeometry() const;
    const float * getRayDirections() const;
    const float * getMainAxisRotations() const;
    int getIncDirNum() const;
    int getRaysPerDirection() const;
    int getMaxRecursion() const;

    void setRaysPerDirection(int raysPerDirection);
    void setWavelength(float wl);
    void setIncDirNum(int incDirNum);
    void setSunPosition(float lon, float lat);
    void setGeometry(Geometry *g);
    
    bool isSettingsApplied();
    void applySettings();

    void clearRays();

    void writeFinalDirections();
    // void writeGeometryInfo();
    // void writeRayPaths(const std::vector<Ray*> &rays);
    
    OrientationGenerator oriGen;
    Geometry *g;
    std::vector<Ray*> rays;

private:
    int incDirNum;
    float *rayDir;
    float *mainAxRot;
    float sunDir[3];

    int raysPerDirection;
    int maxRecursion;

    float wl;

    bool initialized;
};

#endif // TESTHELPER_H