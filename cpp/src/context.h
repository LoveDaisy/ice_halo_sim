#ifndef TESTHELPER_H
#define TESTHELPER_H

#include "geometry.h"
#include "optics.h"
#include "rapidjson/document.h"

#include <vector>
#include <random>


class SimulationContext;


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
    void printCrystalInfo();

    CrystalContext *crystalCtx;
    EnvironmentContext *envCtx;
    
private:
    void writeRayInfo(std::FILE *file, Ray *r);

    uint64_t totalRayNum;
    int maxRecursionNum;

    float *rayDir;
    float *mainAxRot;
    int *crystalId;
};


class ContextParser
{
public:
    ~ContextParser() = default;

    void parseSettings(SimulationContext &ctx);

    static ContextParser * getFileParser(const char* filename);

private:
    explicit ContextParser(rapidjson::Document &d);

    void parseRayNumber(SimulationContext &ctx);
    void parseMaxRecursion(SimulationContext &ctx);
    void parseSunSetting(SimulationContext &ctx);
    void parseCrystalSetting(SimulationContext &ctx, const rapidjson::Value &c, int ci);
    void parseCrystalType(SimulationContext &ctx, const rapidjson::Value &c, int ci,
        float population,
        OrientationGenerator::Distribution axisDist, float axisMean, float axisStd,
        OrientationGenerator::Distribution rollDist, float rollMean, float rollStd);

    rapidjson::Document d;
};


#endif // TESTHELPER_H