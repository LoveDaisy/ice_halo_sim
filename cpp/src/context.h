#ifndef TESTHELPER_H
#define TESTHELPER_H

#include "geometry.h"
#include "optics.h"
#include "rapidjson/document.h"

#include <vector>
#include <random>
#include <string>
#include <cstdio>


class SimulationContext;
class ContextParser;


class EnvironmentContext
{
friend SimulationContext;
public:
    explicit EnvironmentContext(SimulationContext *ctx);
    ~EnvironmentContext() = default;

    void setWavelength(float wavelength);
    float getWavelength() const;

    void setSunPosition(float lon, float lat);
    const float * getSunDirection() const;

private:
    SimulationContext *simCtx;

    float wavelength;
    float sunDir[3];
};


class CrystalContext
{
friend SimulationContext;
friend Optics;
public:
    explicit CrystalContext(SimulationContext *ctx);
    ~CrystalContext();

    void setCrystal(Crystal *g, float populationRatio,
                    OrientationGenerator::Distribution axisDist, float axisMean, float axisStd,
                    OrientationGenerator::Distribution rollDist, float rollMean, float rollStd);
    Crystal * getCrystal();

    void fillDir(const float *incDir, float *rayDir, float *mainAxRot, int num = 1);

private:
    SimulationContext *simCtx;

    float populationRatio;
    Crystal *crystal;
    OrientationGenerator oriGen;
};


class RayTracingContext
{
friend SimulationContext;
friend Optics;
public:
    explicit RayTracingContext(SimulationContext *ctx);
    ~RayTracingContext();

    void setRayNum(int rayNum);

    void initRays(CrystalContext *ctx);
    void initRays(int rayNum, const float *dir, const float *w, CrystalContext *ctx);
    void clearRays();
    void commitHitResult();
    void commitPropagateResult(CrystalContext *ctx);
    bool isFinished();

private:
    void deleteArrays();
    int chooseFace(const float *faces, int faceNum, const float *rayDir);
    void fillDir(const float *incDir, float *rayDir, float *axRot, CrystalContext *ctx);
    void fillPts(const float *faces, int idx, float *rayPts);

    SimulationContext *simCtx;

    int initRayNum;
    int currentRayNum;
    std::vector<Ray *> rays;
    std::vector<RaySegment *> activeRaySeg;

    std::default_random_engine gen;
    std::uniform_real_distribution<float> dis;

    float *mainAxRot;

    float *rayDir;
    float *rayPts;
    float *faceNorm;
    int *faceId;

    float *rayDir2;
    float *rayDir3;
    float *rayPts2;
    float *rayW2;
    int *faceId2;
};


class SimulationContext
{
friend ContextParser;
public:
    SimulationContext();
    ~SimulationContext();

    int getMaxRecursionNum() const;
    int getCrystalNum() const;
    void setWavelength(float wavelength);
    float getWavelength();
    const float * getSunDir() const;

    void applySettings();

    CrystalContext * getCrystalContext(int i);
    RayTracingContext * getRayTracingContext(int i);

    /* For output */
    void writeFinalDirections(const char *filename);
    void writeRayInfo(const char *filename, float lon, float lat, float delta);
    void printCrystalInfo();

private:
    // Helper function
    void writeRayInfo(std::FILE *file, Ray *r);

    EnvironmentContext *envCtx;
    std::vector<CrystalContext *> crystalCtxs;
    std::vector<RayTracingContext *> rayTracingCtxs;

    uint64_t totalRayNum;
    int maxRecursionNum;
};


class ContextParser
{
public:
    ~ContextParser() = default;

    void parseSettings(SimulationContext &ctx);

    static ContextParser * createFileParser(const char *filename);

private:
    explicit ContextParser(rapidjson::Document &d, const char *filename);

    void parseRayNumber(SimulationContext &ctx);
    void parseMaxRecursion(SimulationContext &ctx);
    void parseSunSetting(SimulationContext &ctx);
    void parseCrystalSetting(SimulationContext &ctx, const rapidjson::Value &c, int ci);
    void parseCrystalType(SimulationContext &ctx, const rapidjson::Value &c, int ci,
        float population,
        OrientationGenerator::Distribution axisDist, float axisMean, float axisStd,
        OrientationGenerator::Distribution rollDist, float rollMean, float rollStd);
    Crystal * parseCustomCrystal(std::FILE *file);

    rapidjson::Document d;

    std::string filename;
};


#endif // TESTHELPER_H