#include "processhelper.h"
#include "linearalgebra.h"

#include <unordered_set>


OrientationGenerator::OrientationGenerator(float axStd, float rollStd,
        AxisDistribution ax, RollDistribution roll) :
    axDist(ax), rollDist(roll),
    axStd(axStd), rollStd(rollStd)
{
    unsigned int seed = static_cast<unsigned int>(std::chrono::system_clock::now().time_since_epoch().count());
    // unsigned int seed = 2345;
    generator.seed(seed);
}


OrientationGenerator::OrientationGenerator() :
    OrientationGenerator(0.0f, 0.0f)
{ }


// void OrientationGenerator::setAxisDistribution(AxisDistribution axisDist, float std)
// {
//     this->axDist = axisDist;
//     this->axStd = std;
// }


// void OrientationGenerator::setRollDistribution(RollDistribution rollDist, float std)
// {
//     this->rollDist = rollDist;
//     this->rollStd = rollStd;
// }


void OrientationGenerator::fillData(const float *sunDir, int num, float *rayDir, float *mainAxRot)
{
    float tmpSunDir[3] = { 0.0f };
    float sunRotation[3] = {
        std::atan2(-sunDir[1], -sunDir[0]),
        std::asin(-sunDir[2]),
        0.0f
    };
    float h = 1.0f - std::cos(0.25f * Geometry::PI / 180.0f);

    // printf("SUN_ROT:%+.4f,%+.4f,%+.4f\n", sunRotation[0], sunRotation[1], sunRotation[2]);

    for (int i = 0; i < num; i++) {
        float lon, lat, roll;

        switch (axDist) {
            case AxisDistribution::AX_SPH_UNIFORM : {
                float v[3] = {gaussDistribution(generator),
                              gaussDistribution(generator),
                              gaussDistribution(generator)};
                LinearAlgebra::normalize3(v);
                lon = std::atan2(v[1], v[0]);
                lat = std::asin(v[2] / LinearAlgebra::norm3(v));
            }
                break;
            case AxisDistribution::AX_HOR_GAUSS :
                lon = uniformDistribution(generator) * 2 * Geometry::PI;
                lat = gaussDistribution(generator) * axStd;
                break;
            case AxisDistribution::AX_ZENITHAL_GAUSS :
                // TODO this implementation may be NOT right
                lon = uniformDistribution(generator) * 2 * Geometry::PI;
                lat = Geometry::PI / 2 - std::abs(gaussDistribution(generator) * axStd);
                break;
        }

        switch (rollDist) {
            case RollDistribution::ROLL_HOR_GAUSS :
                roll = gaussDistribution(generator) * rollStd;
                break;
            case RollDistribution::ROLL_UNIFORM :
                roll = uniformDistribution(generator) * 2 * Geometry::PI;
                break;
        }

        mainAxRot[i*3+0] = lon;
        mainAxRot[i*3+1] = lat;
        mainAxRot[i*3+2] = roll;

        float z = 1.0f - uniformDistribution(generator) * h;
        float q = uniformDistribution(generator) * 2 * Geometry::PI;
        tmpSunDir[0] = std::sqrt(1.0f - z * z) * std::cos(q);
        tmpSunDir[1] = std::sqrt(1.0f - z * z) * std::sin(q);
        tmpSunDir[2] = z;
        LinearAlgebra::rotateZBack(sunRotation, tmpSunDir);

        // printf("SUN_DIR:%+.4f,%+.4f,%+.4f,%+.4f,%+.4f,%+.4f\n", 
        //     -sunDir[0], -sunDir[1], -sunDir[2],
        //     tmpSunDir[0], tmpSunDir[1], tmpSunDir[2]);

        memcpy(rayDir+i*3, sunDir, 3*sizeof(float));
        LinearAlgebra::rotateZ(mainAxRot+i*3, rayDir+i*3);
    }
}


// void OrientationGenerator::setAxisOrientation(AxisDistribution ax, float axStd)
// {
//     this->axDist = ax;
//     this->axStd = axStd;
// }


// void OrientationGenerator::setAxisRoll(RollDistribution roll, float rollStd)
// {
//     this->rollDist = roll;
//     this->rollStd = rollStd;
// }


RaySegmentFactory * RaySegmentFactory::instance = nullptr;

RaySegmentFactory::RaySegmentFactory()
{
    auto *raySegPool = new RaySegment[chunkSize];
    segments.push_back(raySegPool);
    nextUnusedId = 0;
    currentChunkId = 0;
}

RaySegmentFactory::~RaySegmentFactory()
{
    for (auto seg : segments) {
        delete[] seg;
    }
    segments.clear();
}

RaySegmentFactory * RaySegmentFactory::getInstance()
{
    if (instance == nullptr) {
        instance = new RaySegmentFactory();
    }
    return instance;
}

RaySegment * RaySegmentFactory::getRaySegment(float *pt, float *dir, float w, int faceId)
{
    RaySegment *seg;
    RaySegment *currentChunk;

    if (nextUnusedId < chunkSize) {
        currentChunk = segments[currentChunkId];
    } else {
        currentChunkId++;
        if (currentChunkId >= segments.size()) {
            auto *raySegPool = new RaySegment[chunkSize];
            segments.push_back(raySegPool);
        }
        nextUnusedId = 0;
        currentChunk = segments[currentChunkId];
    }
    
    seg = &currentChunk[nextUnusedId++];
    seg->reset();

    seg->pt.val(pt);
    seg->dir.val(dir);
    seg->w = w;
    seg->faceId = faceId;

    return seg;
}

void RaySegmentFactory::clear()
{
    nextUnusedId = 0;
    currentChunkId = 0;
}


void EnvironmentContext::setWavelength(float wavelength)
{
    this->wavelength = wavelength;
}


float EnvironmentContext::getWavelength()
{
    return wavelength;
}


void EnvironmentContext::setSunPosition(float lon, float lat)
{
    float x = -std::cos(lat) * std::cos(lon);
    float y = -std::cos(lat) * std::sin(lon);
    float z = -std::sin(lat);

    this->sunDir[0] = x;
    this->sunDir[1] = y;
    this->sunDir[2] = z;
}


void RayTracingContext::clearRays()
{
    rays.clear();
}


void RayTracingContext::pushBackRay(Ray *ray)
{
    rays.push_back(ray);
}


CrystalContext::~CrystalContext()
{
    for (auto &p : rayTracingCtxs) {
        delete p;
    }
}


void CrystalContext::addGeometry(
    Geometry *g, float populationWeight,
    OrientationGenerator::AxisDistribution axisDist, float axisStd,
    OrientationGenerator::RollDistribution rollDist, float rollStd)
{
    crystals.push_back(g);
    populationWeights.push_back(populationWeight);
    oriGens.emplace_back(axisStd, rollStd, axisDist, rollDist);
    rayNums.push_back(0);
    rayTracingCtxs.push_back(new RayTracingContext());
}


Geometry * CrystalContext::getCrystal(int i)
{
    return crystals[i];
}


int CrystalContext::getRayNum(int i)
{
    return rayNums[i];
}


RayTracingContext * CrystalContext::getRayTracingCtx(int i)
{
    return rayTracingCtxs[i];
}


size_t CrystalContext::popSize()
{
    return crystals.size();
}


SimulationContext::SimulationContext() :
    totalRayNum(0), maxRecursionNum(9),
    rayDir(nullptr), mainAxRot(nullptr), crystalId(nullptr)
{
    crystalCtx = new CrystalContext();
    envCtx = new EnvironmentContext();
}


SimulationContext::~SimulationContext()
{
    delete envCtx;
    delete crystalCtx;

    if (rayDir)
        delete[] rayDir;
    if (mainAxRot)
        delete[] mainAxRot;
    if (crystalId)
        delete[] crystalId;
}


void SimulationContext::setTotalRayNum(uint64_t num)
{
    this->totalRayNum = num;
    
    if (rayDir)
        delete[] rayDir;
    rayDir = new float[num * 3];

    if (mainAxRot)
        delete[] mainAxRot;
    mainAxRot = new float[num * 3];

    if (crystalId)
        delete[] crystalId;
    crystalId = new int[num];

    for (int i = 0; i < num; i++) {
        rayDir[i] = 0.0f;
        mainAxRot[i] = 0.0f;
        crystalId[i] = 0;
    }
}


void SimulationContext::setMaxRecursionNum(int num)
{
    maxRecursionNum = num;
}


int SimulationContext::getMaxRecursionNum()
{
    return maxRecursionNum;
}


const float* SimulationContext::getRayDirections(int i)
{
    float *p = rayDir;
    for (int x = 0; x < i; x++) {
        p += crystalCtx->rayNums[x] * 3;
    }
    return p;
}


// void SimulationContext::reset()
// {
//     if (rayDir) delete[] rayDir;
//     if (mainAxRot) delete[] mainAxRot;
//     if (crystalId) delete[] crystalId;

//     rayDir = nullptr;
//     mainAxRot = nullptr;
//     crystalId = nullptr;

//     totalRayNum = 0;
//     maxRecursionNum = 9;
// }


void SimulationContext::applySettings()
{
    if (totalRayNum <= 0) return;

    size_t popSize = crystalCtx->popSize();
    float popWeightSum = 0.0f;
    for (float pw : crystalCtx->populationWeights) {
        popWeightSum += pw;
    }
    for (float &pw : crystalCtx->populationWeights) {
        pw /= popWeightSum;
    }
    
    size_t currentIdx = 0;
    const float * sunDir = envCtx->sunDir;
    for (int i = 0; i < popSize-1; i++) {
        auto tmpPopSize = static_cast<int>(crystalCtx->populationWeights[i] * totalRayNum);
        crystalCtx->oriGens[i].fillData(sunDir, tmpPopSize, rayDir + currentIdx*3, mainAxRot + currentIdx*3);
        crystalCtx->rayNums[i] = tmpPopSize;
        for (int j = 0; j < tmpPopSize; j++) {
            crystalId[currentIdx + j] = i;
        }

        // for (int k = 0; k < tmpPopSize; k++) {
        //     printf("DIR%d:%+.4f,%+.4f,%+.4f\n", i, rayDir[currentIdx*3+k+0], rayDir[currentIdx*3+k+1], rayDir[currentIdx*3+k+2]);
        //     printf("MA%d:%+.4f,%+.4f,%+.4f\n", i, mainAxRot[currentIdx*3+k+0], mainAxRot[currentIdx*3+k+1], mainAxRot[currentIdx*3+k+2]);
        // }
        currentIdx += tmpPopSize;
    }
    auto tmpPopSize = static_cast<int>(totalRayNum - currentIdx);
    crystalCtx->oriGens[popSize-1].fillData(sunDir, tmpPopSize, rayDir + currentIdx*3, mainAxRot + currentIdx*3);
    crystalCtx->rayNums[popSize-1] = tmpPopSize;
    for (int j = 0; j < tmpPopSize; j++) {
        crystalId[currentIdx + j] = static_cast<int>(popSize - 1);
    }

    // for (int k = 0; k < tmpPopSize; k++) {
    //     printf("DIR%d:%+.4f,%+.4f,%+.4f\n", popSize-1, rayDir[currentIdx*3+k+0], rayDir[currentIdx*3+k+1], rayDir[currentIdx*3+k+2]);
    //     printf("MA%d:%+.4f,%+.4f,%+.4f\n", popSize-1, mainAxRot[currentIdx*3+k+0], mainAxRot[currentIdx*3+k+1], mainAxRot[currentIdx*3+k+2]);
    // }
}


void SimulationContext::writeFinalDirections(const char *filename)
{
    std::FILE* file = std::fopen(filename, "wb");
    if (!file) return;

    uint64_t totalRaySegNum = 0;
    for (auto &rc : crystalCtx->rayTracingCtxs) {
        for (auto &r : rc->rays) {
            totalRaySegNum += r->totalNum();
        }
    }
    fwrite(&totalRaySegNum, sizeof(uint64_t), 1, file);
    
    size_t currentIdx = 0;
    std::vector<RaySegment*> v;
    int k = 0;
    for (auto &rc : crystalCtx->rayTracingCtxs) {
        for (auto r : rc->rays) {
            v.clear();
            v.push_back(r->firstRaySeg);

            while (!v.empty()) {
                RaySegment *p = v.back();
                v.pop_back();
                if (p->nextReflect) {
                    v.push_back(p->nextReflect);
                }
                if (p->nextRefract) {
                    v.push_back(p->nextRefract);
                }
                if (p->isValidEnd() && LinearAlgebra::dot3(p->dir.val(), r->firstRaySeg->dir.val()) < 1.0 - 1e-5) {
                    float finalDir[3];
                    memcpy(finalDir, p->dir.val(), sizeof(float)*3);
                    LinearAlgebra::rotateZBack(mainAxRot+currentIdx*3, finalDir);
                    fwrite(finalDir, sizeof(float), 3, file);
                    fwrite(&p->w, sizeof(float), 1, file);

                    // printf("LAST_MA%d:%+.4f,%+.4f,%+.4f\n", k,
                    //     mainAxRot[currentIdx*3+0], mainAxRot[currentIdx*3+1], mainAxRot[currentIdx*3+2]);
                    // printf("OUT%d:%+.4f,%+.4f,%+.4f\n", k,
                    //     p->dir.x(), p->dir.y(), p->dir.z());
                    // printf("FINAL%d:%+.4f,%+.4f,%+.4f\n", k,
                    //     finalDir[0], finalDir[1], finalDir[2]);
                }
            }
            currentIdx++;
        }
        k++;
    }

    std::fclose(file);
}


void SimulationContext::writeRayInfo(const char *filename, float lon, float lat, float delta)
{
    std::FILE* file = std::fopen(filename, "wb");
    if (!file) return;

    float targetDir[3] = {
        std::cos(lat) * std::cos(lon),
        std::cos(lat) * std::sin(lon),
        std::sin(lat)
    };
    float cosDelta = std::cos(delta);

    size_t currentIdx = 0;
    std::vector<RaySegment*> v;
    for (auto &rc : crystalCtx->rayTracingCtxs) {
        for (auto r : rc->rays) {
            v.clear();
            v.push_back(r->firstRaySeg);
            bool targetOn = false;

            while (!v.empty()) {
                RaySegment *p = v.back();
                v.pop_back();
                if (p->nextReflect) {
                    v.push_back(p->nextReflect);
                }
                if (p->nextRefract) {
                    v.push_back(p->nextRefract);
                }
                if (p->isValidEnd()) {
                    float finalDir[3];
                    memcpy(finalDir, p->dir.val(), sizeof(float)*3);
                    LinearAlgebra::rotateZBack(mainAxRot+currentIdx*3, finalDir);
                    if (LinearAlgebra::dot3(targetDir, finalDir) > cosDelta) {
                        targetOn = true;
                        break;
                    }
                }
            }
            currentIdx++;

            if (targetOn) {
                writeRayInfo(file, r);
            }
        }
    }

    fclose(file);
}


void SimulationContext::writeRayInfo(std::FILE *file, Ray *sr)
{
    std::vector<RaySegment*> v;
    v.push_back(sr->firstRaySeg);

    std::unordered_set<RaySegment*> checked;
    float tmp[7];
    while (!v.empty()) {
        RaySegment *p = v.back();
        if (checked.find(p) != checked.end()) {
            v.pop_back();
            continue;
        }

        if (p->nextReflect && checked.find(p->nextReflect) == checked.end()) {
            v.push_back(p->nextReflect);
            continue;
        }
        if (p->nextRefract && checked.find(p->nextRefract) == checked.end()) {
            v.push_back(p->nextRefract);
            continue;
        }
        if (p->nextReflect == nullptr && p->nextRefract == nullptr && p->isValidEnd()) {
            // checked.insert(p);
            tmp[6] = -1; tmp[0] = v.size();
            fwrite(tmp, sizeof(float), 7, file);
            // printf("%lu,0,0,0,0,0,-1;\n", v.size());
            for (auto r : v) {
                memcpy(tmp, r->pt.val(), 3*sizeof(float));
                memcpy(tmp+3, r->dir.val(), 3*sizeof(float));
                tmp[6] = r->w;
                fwrite(tmp, sizeof(float), 7, file);

                // printf("%+.4f,%+.4f,%+.4f,%+.4f,%+.4f,%+.4f,%+.4f;\n",
                //     tmp[0], tmp[1], tmp[2], tmp[3], tmp[4], tmp[5], tmp[6]);
            }
        }
        checked.insert(p);
    }
}




