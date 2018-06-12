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

    void initRays(CrystalContext *ctx, int rayNum, const float *dir, const float *w, RaySegment **prevRaySeg = nullptr);
    void clearRays();
    void commitHitResult();
    void commitPropagateResult(CrystalContext *ctx);
    bool isFinished();

    size_t copyFinishedRaySegments(RaySegment **segs, float *dir, float prob = 1.0f);

private:
    static constexpr float PROP_MIN_W = 1e-6;
    static constexpr float SCAT_MIN_W = 1e-3;
    
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

    uint64_t getTotalInitRays() const;
    int getMaxRecursionNum() const;
    
    int getMultiScatterNum() const;
    float getMultiScatterProb() const;
    
    int getCrystalNum() const;
    
    void setWavelength(float wavelength);
    float getWavelength();
    
    const float * getSunDir() const;
    void setSunPosition(float lon, float lat);

    void applySettings();
    void allocateCrystalRayNum(int scatterIdx, uint64_t totalRayNum);

    CrystalContext * getCrystalContext(int i);
    RayTracingContext * getRayTracingContext(int scatterIndx, int crystalIdx);

    /* For output */
    void writeFinalDirections(const char *filename);
    void writeRayInfo(const char *filename, float lon, float lat, float delta);
    void printCrystalInfo();

private:
    void writeRayInfo(std::FILE *file, Ray *r);     // Helper function

    std::vector<CrystalContext *> crystalCtxs;
    std::vector<std::vector<RayTracingContext *> > rayTracingCtxs;

    uint64_t totalRayNum;
    int maxRecursionNum;
    
    int multiScatterNum;
    float multiScatterProb;

    float wavelength;
    float sunDir[3];

    std::mt19937 generator;
    std::uniform_real_distribution<float> uniformDistribution;
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
    void parseMultiScatterSetting(SimulationContext &ctx);
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