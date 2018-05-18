#ifndef TESTHELPER_H
#define TESTHELPER_H

#include "geometry.h"
#include "optics.h"

#include <vector>
#include <set>
#include <random>


class SimulationContext;

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
    OrientationGenerator();
    OrientationGenerator(float axStd, float rollStd,
        AxisDistribution ax = AxisDistribution::AX_SPH_UNIFORM,
        RollDistribution roll = RollDistribution::ROLL_UNIFORM);
    ~OrientationGenerator() = default;

    void fillData(const float *sunDir, int num, float *rayDir, float *mainAxRot);

    // void setAxisDistribution(AxisDistribution axisDist, float std);
    // void setRollDistribution(RollDistribution rollDist, float std);

    // void setAxisOrientation(AxisDistribution ax, float axStd);
    // void setAxisRoll(RollDistribution roll, float rollStd);

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
    static const uint32_t chunkSize = 1024 * 64;
    
    std::vector<RaySegment*> segments;
    uint32_t nextUnusedId;
    uint32_t currentChunkId;
};


class EnvironmentContext
{
friend SimulationContext;
public:
    EnvironmentContext() = default;
    ~EnvironmentContext() = default;

    void setWavelength(float wavelength);
    float getWavelength();

    void setSunPosition(float lon, float lat);

private:
    float wavelength;
    float sunDir[3];
};


class RayTracingContext
{
friend SimulationContext;
public:
    RayTracingContext() = default;
    ~RayTracingContext() = default;

    void clearRays();
    void pushBackRay(Ray *ray);
    
private:
    std::vector<Ray*> rays;
};


class CrystalContext
{
friend SimulationContext;
public:
    CrystalContext() = default;
    ~CrystalContext();

    void addGeometry(Geometry *g, float populationWeight,
                     OrientationGenerator::AxisDistribution axisDist, float axisStd,
                     OrientationGenerator::RollDistribution rollDist, float rollStd);

    Geometry * getCrystal(int i);
    int getRayNum(int i);
    RayTracingContext *getRayTracingCtx(int i);

    size_t popSize();

private:
    std::vector<Geometry *> crystals;
    std::vector<float> populationWeights;
    std::vector<OrientationGenerator> oriGens;
    std::vector<int> rayNums;
    std::vector<RayTracingContext *> rayTracingCtxs;
};


class SimulationContext
{
public:
    SimulationContext();
    ~SimulationContext();

    void setTotalRayNum(uint64_t num);
    void setMaxRecursionNum(int num);
    int getMaxRecursionNum();

    const float* getRayDirections(int i);

    void applySettings();

    void writeFinalDirections(const char *filename);

    CrystalContext *crystalCtx;
    EnvironmentContext *envCtx;
    
private:
    uint64_t totalRayNum;
    int maxRecursionNum;

    float *rayDir;
    float *mainAxRot;
    int *crystalId;
};


#endif // TESTHELPER_H