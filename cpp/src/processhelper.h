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
    enum class Distribution
    {
        UNIFORM,
        GAUSS
    };

public:
    // OrientationGenerator();
    OrientationGenerator(Distribution axDist, float axMean, float axStd, 
        Distribution rollDist, float rollMean, float rollStd);
    ~OrientationGenerator() = default;

    void fillData(const float *sunDir, int num, float *rayDir, float *mainAxRot);

private:
    std::mt19937 generator;
    std::normal_distribution<float> gaussDistribution;
    std::uniform_real_distribution<float> uniformDistribution;

    Distribution axDist;
    float axMean;
    float axStd;

    Distribution rollDist;
    float rollMean;
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
                     OrientationGenerator::Distribution axisDist, float axisMean, float axisStd,
                     OrientationGenerator::Distribution rollDist, float rollMean, float rollStd);

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
    void writeRayInfo(const char *filename, float lon, float lat, float delta);
    void writeRayInfo(std::FILE *file, Ray *r);
    void writeCrystalInfo(const char* filename);

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