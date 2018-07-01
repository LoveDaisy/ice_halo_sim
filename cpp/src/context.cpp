#include "context.h"
#include "threadingpool.h"

#include "rapidjson/pointer.h"
#include "rapidjson/error/en.h"
#include "rapidjson/filereadstream.h"

#include <unordered_set>


namespace IceHalo {

CrystalContext::CrystalContext(SimulationContext *ctx) :
    simCtx(ctx)
{ }


CrystalContext::~CrystalContext()
{
    delete crystal;
}


void CrystalContext::setCrystal(
    Crystal *g, float populationWeight,
    Math::OrientationGenerator::Distribution axisDist, float axisMean, float axisStd,
    Math::OrientationGenerator::Distribution rollDist, float rollMean, float rollStd )
{
    this->crystal = g;
    this->populationRatio = populationWeight;
    this->oriGen = Math::OrientationGenerator(
        axisDist, axisMean * Crystal::PI / 180.0f, axisStd * Crystal::PI / 180.0f,
        rollDist, rollMean * Crystal::PI / 180.0f, rollStd * Crystal::PI / 180.0f
    );
}


Crystal * CrystalContext::getCrystal()
{
    return this->crystal;
}


void CrystalContext::fillDir(const float *incDir, float *rayDir, float *mainAxRot, int num)
{
    oriGen.fillData(incDir, num, rayDir, mainAxRot);
}


RayTracingContext::RayTracingContext(SimulationContext *ctx) :
    simCtx(ctx), initRayNum(0), currentRayNum(0), activeRaySegNum(0),
    rays(nullptr), activeRaySeg(nullptr),
    gen(), dis(0.0f, 1.0f),
    mainAxRot(nullptr),
    rayDir(nullptr), rayPts(nullptr), faceNorm(nullptr), faceId(nullptr),
    rayDir2(nullptr), rayDir3(nullptr), rayPts2(nullptr), rayW2(nullptr), faceId2(nullptr)
{ }


RayTracingContext::~RayTracingContext()
{
    deleteArrays();
}


void RayTracingContext::setRayNum(int num)
{
    initRayNum = num;
    currentRayNum = num;
    int maxRayNum = initRayNum * simCtx->getMaxRecursionNum() * 3;

    deleteArrays();

    mainAxRot = new float[initRayNum * 3];
    rayDir = new float[maxRayNum * 3];
    rayPts = new float[maxRayNum * 3];
    faceNorm = new float[maxRayNum * 3];
    faceId = new int[maxRayNum];

    rayDir2 = new float[maxRayNum * 3];
    rayDir3 = new float[maxRayNum * 3];
    rayPts2 = new float[maxRayNum * 3];
    rayW2 = new float[maxRayNum];
    faceId2 = new int[maxRayNum];

    activeRaySeg = new RaySegment*[maxRayNum * 3];
    rays = new Ray*[initRayNum];
}


void RayTracingContext::initRays(CrystalContext *ctx, int rayNum, const float *dir, const float *w, RaySegment **prevRaySeg)
{
    setRayNum(rayNum);

    Crystal * crystal = ctx->getCrystal();
    int faceNum = crystal->faceNum();
    auto *faces = new float[faceNum * 9];
    crystal->copyFaceData(faces);

    activeRaySegNum = 0;
    Pool *pool = Pool::getInstance();
    int step = initRayNum / 80;
    for (int startIdx = 0; startIdx < initRayNum; startIdx += step) {
        int endIdx = std::min(startIdx + step, initRayNum);
        pool->addJob(std::bind(&RayTracingContext::initRaysRange, this,
            ctx, dir, faces, startIdx, endIdx));
    }
    pool->waitFinish();

    for (int i = 0; i < initRayNum; i++) {
        auto *r = new Ray(rayPts+i*3, rayDir+i*3, w[i], faceId[i]);
        if (prevRaySeg && prevRaySeg[i]) {
            prevRaySeg[i]->nextRefract = r->firstRaySeg;
        }
        rays[i] = r;
        activeRaySeg[i] = r->firstRaySeg;
    }
    activeRaySegNum = initRayNum;

    delete[] faces;
}


void RayTracingContext::initRaysRange(CrystalContext *ctx, const float *dir, 
    const float *faces,
    int startIdx, int endIdx)
{
    int faceNum = ctx->getCrystal()->faceNum();
    for (int i = startIdx; i < endIdx; i++) {
        fillDir(dir+i*3, rayDir+i*3, mainAxRot+i*3, ctx);

        int idx = chooseFace(faces, faceNum, rayDir+i*3);
        faceId[i] = idx;

        fillPts(faces, idx, rayPts+i*3);
        ctx->getCrystal()->copyNormalData(idx, faceNorm+i*3);

        // auto *r = new Ray(rayPts+i*3, rayDir+i*3, w[i], faceId[i]);
        // if (prevRaySeg && prevRaySeg[i]) {
        //     prevRaySeg[i]->nextRefract = r->firstRaySeg;
        // }
        // rays[i] = r;
        // activeRaySeg[i] = r->firstRaySeg;
    }
}


void RayTracingContext::commitHitResult()
{
    RaySegmentFactory *raySegPool = RaySegmentFactory::getInstance();
    for (int i = 0; i < currentRayNum; i++) {
        RaySegment *lastSeg = activeRaySeg[i];
        RaySegment *seg = raySegPool->getRaySegment(rayPts + i*3, rayDir2 + i*3,
            lastSeg->w * rayW2[i], faceId[i]);
        lastSeg->nextReflect = seg;
        seg->prev = lastSeg;
        activeRaySeg[i] = seg;

        seg = raySegPool->getRaySegment(rayPts + i*3, rayDir3 + i*3,
            lastSeg->w * (1.0f - rayW2[i]), faceId[i]);
        lastSeg->nextRefract = seg;
        seg->prev = lastSeg;
        activeRaySeg[currentRayNum + i] = seg;
    }

    memcpy(rayDir, rayDir2, currentRayNum * 3 * sizeof(float));
    memcpy(rayDir + currentRayNum*3, rayDir3, currentRayNum * 3 * sizeof(float));
    memcpy(rayPts + currentRayNum*3, rayPts, currentRayNum * 3 * sizeof(float));
    memcpy(faceId + currentRayNum, faceId, currentRayNum * sizeof(int));

    currentRayNum *= 2;
    activeRaySegNum = currentRayNum;
}


void RayTracingContext::commitPropagateResult(CrystalContext *ctx)
{
    int k = 0;
    activeRaySegNum = 0;
    for (int i = 0; i < currentRayNum; i++) {
        if (faceId2[i] >= 0 && activeRaySeg[i]->w > PROP_MIN_W) {
            activeRaySeg[activeRaySegNum++] = activeRaySeg[i];
            memcpy(rayPts + k*3, rayPts2 + i*3, 3 * sizeof(float));
            memcpy(rayDir + k*3, rayDir + i*3, 3 * sizeof(float));
            faceId[k] = faceId2[i];
            ctx->getCrystal()->copyNormalData(faceId2[i], faceNorm + k*3);
            k++;
        } else if (faceId2[i] < 0) {
            activeRaySeg[i]->isFinished = true;
        }
    }
    currentRayNum = k;
}


bool RayTracingContext::isFinished()
{
    return activeRaySegNum <= 0;
}


size_t RayTracingContext::copyFinishedRaySegments(RaySegment **segs, float *dir, float prob)
{
    std::atomic_uint64_t k(0);
    
    Pool *pool = Pool::getInstance();
    int step = initRayNum / 80;
    for (int startIdx = 0; startIdx < initRayNum; startIdx += step) {
        int endIdx = std::min(startIdx + step, initRayNum);
        pool->addJob([segs, dir, prob, &k, startIdx, endIdx, this](){ 
            copyFinishedRaySegmentsRange(segs, dir, prob, k, startIdx, endIdx);
        });
    }
    pool->waitFinish();

    return k.load();
}


void RayTracingContext::copyFinishedRaySegmentsRange(RaySegment **segs, float *dir, float prob,
    std::atomic_uint64_t &k, int startIdx, int endIdx)
{
    std::default_random_engine randomEngine;
    std::uniform_real_distribution<double> uniformDist(0.0, 1.0);

    std::vector<RaySegment *> v;
    for (int rayIdx = startIdx; rayIdx < endIdx; rayIdx++) {
        v.clear();
        v.push_back(rays[rayIdx]->firstRaySeg);

        while (!v.empty()) {
            RaySegment *p = v.back();
            v.pop_back();
            if (p->nextReflect && !p->isFinished) {
                v.push_back(p->nextReflect);
            }
            if (p->nextRefract && !p->isFinished) {
                v.push_back(p->nextRefract);
            }
            if (p->w > SCAT_MIN_W && p->faceId >= 0 && p->isFinished 
                    && uniformDist(randomEngine) < prob) {
                auto tmpk = k++;
                float *finalDir = dir + tmpk * 3;
                memcpy(finalDir, p->dir.val(), sizeof(float) * 3);
                Math::rotateZBack(mainAxRot + rayIdx * 3, finalDir);
                segs[tmpk] = p;
            }
        }
    }
}


void RayTracingContext::deleteArrays()
{
    delete[] mainAxRot;
    delete[] rayDir;
    delete[] rayPts;
    delete[] faceNorm;
    delete[] faceId;

    delete[] rayDir2;
    delete[] rayDir3;
    delete[] rayPts2;
    delete[] rayW2;
    delete[] faceId2;

    delete[] activeRaySeg;
    delete[] rays;
}


int RayTracingContext::chooseFace(const float *faces, int faceNum, const float *rayDir)
{
    auto *frac = new float[faceNum];

    for (int i = 0; i < faceNum; i++) {
        float v1[3], v2[3];
        Math::vec3FromTo(faces + i*9, faces + i*9 + 3, v1);
        Math::vec3FromTo(faces + i*9, faces + i*9 + 6, v2);
        float norm[3];
        Math::cross3(v1, v2, norm);
        float c = Math::dot3(rayDir, norm);
        frac[i] = c < 0 ? (-c) : 0;
    }
    for (int i = 1; i < faceNum; i++) {
        frac[i] += frac[i-1];
    }
    for (int i = 0; i < faceNum; i++) {
        frac[i] /= frac[faceNum-1];
    }
    frac[faceNum-1] = 1.0f;    // Force to 1.0f

    int idx = -1;
    float p = dis(gen);
    for (int j = 0; j < faceNum; j++) {
        if (p <= frac[j]) {
            idx = j;
            break;
        }
    }

    delete[] frac;

    return idx;
}


void RayTracingContext::fillDir(const float *incDir, float *rayDir, float *axRot, CrystalContext *ctx)
{
    ctx->fillDir(incDir, rayDir, axRot);
}


void RayTracingContext::fillPts(const float *faces, int idx, float *rayPts)
{
    float a = dis(gen);
    float b = dis(gen);
    if (a + b > 1.0f) {
        a = 1.0f - a;
        b = 1.0f - b;
    }
    rayPts[0] = faces[idx*9+0] + a * (faces[idx*9+3] - faces[idx*9+0]) +
        b * (faces[idx*9+6] - faces[idx*9+0]);
    rayPts[1] = faces[idx*9+1] + a * (faces[idx*9+4] - faces[idx*9+1]) +
        b * (faces[idx*9+7] - faces[idx*9+1]);
    rayPts[2] = faces[idx*9+2] + a * (faces[idx*9+5] - faces[idx*9+2]) +
        b * (faces[idx*9+8] - faces[idx*9+2]);
}


SimulationContext::SimulationContext() :
    totalRayNum(0), maxRecursionNum(9), 
    multiScatterNum(1), multiScatterProb(1.0f),
    wavelength(550.0f), sunDiameter(0.5f)
{
    // envCtx = new EnvironmentContext(this);
}


SimulationContext::~SimulationContext()
{
    // delete envCtx;
    for (auto p : crystalCtxs) {
        delete p;
    }
    for (auto &rcs : rayTracingCtxs) {
        for (auto rc : rcs) {
            delete rc;
        }
    }
}


uint64_t SimulationContext::getTotalInitRays() const
{
    return totalRayNum;
}


int SimulationContext::getMaxRecursionNum() const
{
    return maxRecursionNum;
}


int SimulationContext::getCrystalNum() const
{
    return static_cast<int>(crystalCtxs.size());
}


int SimulationContext::getMultiScatterNum() const
{
    return multiScatterNum;
}

float SimulationContext::getMultiScatterProb() const
{
    return multiScatterProb;
}


void SimulationContext::setWavelength(float wavelength)
{
    this->wavelength = wavelength;
}


float SimulationContext::getWavelength()
{
    return wavelength;
}


void SimulationContext::fillSunDir(float *dir, int num)
{
    float sunLon = std::atan2(sunDir[1], sunDir[0]);
    float sunLat = std::asin(sunDir[2] / Math::norm3(sunDir));
    float sunRot[3] = { sunLon, sunLat, 0 };

    float dz = 1.0f - std::cos(sunDiameter / 360 * Crystal::PI);
    for (int i = 0; i < num; i++) {
        float z = 1.0f - uniformDistribution(generator) * dz;
        float r = std::sqrt(1.0f - z * z);
        float q = uniformDistribution(generator) * 2 * Crystal::PI;
        float x = std::cos(q) * r;
        float y = std::sin(q) * r;

        dir[i*3 + 0] = x;
        dir[i*3 + 1] = y;
        dir[i*3 + 2] = z;
    }
    Math::rotateZBack(sunRot, dir, num);
}


void SimulationContext::setSunPosition(float lon, float lat)
{
    float x = -std::cos(lat * Crystal::PI / 180.0f) * std::cos(lon * Crystal::PI / 180.0f);
    float y = -std::cos(lat * Crystal::PI / 180.0f) * std::sin(lon * Crystal::PI / 180.0f);
    float z = -std::sin(lat * Crystal::PI / 180.0f);

    this->sunDir[0] = x;
    this->sunDir[1] = y;
    this->sunDir[2] = z;
}


void SimulationContext::applySettings()
{
    if (totalRayNum <= 0) return;

    float popWeightSum = 0.0f;
    for (auto c : crystalCtxs) {
        popWeightSum += c->populationRatio;
    }
    for (auto c : crystalCtxs) {
        c->populationRatio /= popWeightSum;
    }

    setCrystalRayNum(0, totalRayNum);
}


void SimulationContext::setCrystalRayNum(int scatterIdx, uint64_t totalRayNum)
{
    if (scatterIdx >= static_cast<int>(rayTracingCtxs.size())) {
        return;
    }

    size_t popSize = crystalCtxs.size();
    size_t currentIdx = 0;
    for (decltype(popSize) i = 0; i < popSize-1; i++) {
        auto tmpPopSize = static_cast<int>(crystalCtxs[i]->populationRatio * totalRayNum);
        rayTracingCtxs[scatterIdx][i]->setRayNum(tmpPopSize);
        currentIdx += tmpPopSize;
    }
    auto tmpPopSize = static_cast<int>(totalRayNum - currentIdx);
    rayTracingCtxs[scatterIdx][popSize-1]->setRayNum(tmpPopSize);
}


CrystalContext * SimulationContext::getCrystalContext(int i)
{
    return crystalCtxs[i];
}


RayTracingContext * SimulationContext::getRayTracingContext(int scatterIdx, int crystalIdx)
{
    return rayTracingCtxs[scatterIdx][crystalIdx];
}


void SimulationContext::writeFinalDirections(const char *filename)
{
    std::FILE* file = std::fopen(filename, "wb");
    if (!file) return;

    // uint64_t totalRaySegNum = 0;
    // for (auto &rc : rayTracingCtxs) {
    //     for (auto &r : rc->rays) {
    //         totalRaySegNum += r->totalNum();
    //     }
    // }
    // fwrite(&totalRaySegNum, sizeof(uint64_t), 1, file);

    std::vector<RaySegment *> v;
    for (auto &rcs : rayTracingCtxs) {
        for (auto rc : rcs) {
            size_t currentIdx = 0;
            for (int i = 0; i < rc->initRayNum; i++) {
                auto r = rc->rays[i];
                v.clear();
                v.push_back(r->firstRaySeg);

                while (!v.empty()) {
                    RaySegment *p = v.back();
                    v.pop_back();
                    if (p->nextReflect && !p->isFinished) {
                        v.push_back(p->nextReflect);
                    }
                    if (p->nextRefract && !p->isFinished) {
                        v.push_back(p->nextRefract);
                    }
                    if (!p->nextReflect && !p->nextRefract &&
                            p->isValidEnd() && Math::dot3(p->dir.val(), r->firstRaySeg->dir.val()) < 1.0 - 1e-5) {
                        float finalDir[3];
                        memcpy(finalDir, p->dir.val(), sizeof(float) * 3);
                        Math::rotateZBack(rc->mainAxRot + currentIdx * 3, finalDir);
                        fwrite(finalDir, sizeof(float), 3, file);
                        fwrite(&p->w, sizeof(float), 1, file);
                    }
                }
                currentIdx++;
            }
        }
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

    std::vector<RaySegment*> v;
    for (auto &rcs : rayTracingCtxs) {
        size_t currentIdx = 0;
        for (auto rc : rcs) {
            for (int i = 0; i < rc->initRayNum; i++) {
                auto r = rc->rays[i];
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
                        memcpy(finalDir, p->dir.val(), sizeof(float) * 3);
                        Math::rotateZBack(rc->mainAxRot + currentIdx * 3, finalDir);
                        if (Math::dot3(targetDir, finalDir) > cosDelta) {
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


void SimulationContext::printCrystalInfo()
{
    for (auto c : crystalCtxs) {
        auto g = c->crystal;
        printf("--\n");
        for (auto &v : g->getVertexes()) {
            printf("V:%+.4f,%+.4f,%+.4f;\n", v.x(), v.y(), v.z());
        }
        for (auto &f : g->getFaces()) {
            printf("F:%d,%d,%d;\n", f.id1(), f.id2(), f.id3());
        }
    }
}

ContextParser::ContextParser(rapidjson::Document &d, const char *filename) :
    d(std::move(d)), filename(filename)
{ }


ContextParser * ContextParser::createFileParser(const char *filename)
{
    using namespace rapidjson;

    printf("Reading config from: %s\n", filename);

    FILE* fp = fopen(filename, "rb");
    if (!fp) {
        printf("ERROR: file %s cannot be open!\n", filename);
        return nullptr;
    }

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    Document d;
    if (d.ParseStream(is).HasParseError()) {
        fprintf(stderr, "\nError(offset %u): %s\n",
            (unsigned)d.GetErrorOffset(),
        GetParseError_En(d.GetParseError()));
        fclose(fp);
        return nullptr;
    }

    fclose(fp);

    return new ContextParser(d, filename);
}


void ContextParser::parseRayNumber(SimulationContext &ctx)
{
    using namespace rapidjson;

    // Parsing ray number
    uint64_t rayNumber = 10000;
    auto *p = Pointer("/ray_number").Get(d);
    if (p == nullptr) {
        fprintf(stderr, "\nWARNING! Config missing <ray_number>, using default 10000!\n");
    } else if (!p->IsUint()) {
        fprintf(stderr, "\nWARNING! config <ray_number> is not unsigned int, using default 10000!\n");
    } else {
        rayNumber = p->GetUint();
    }
    ctx.totalRayNum = rayNumber;
}


void ContextParser::parseMaxRecursion(SimulationContext &ctx)
{
    using namespace rapidjson;

    int maxRecursion = 9;
    auto *p = Pointer("/max_recursion").Get(d);
    if (p == nullptr) {
        fprintf(stderr, "\nWARNING! Config missing <max_recursion>, using default 9!\n");
    } else if (!p->IsInt()) {
        fprintf(stderr, "\nWARNING! config <max_recursion> is not a integer, using default 9!\n");
    } else {
        maxRecursion = p->GetInt();
    }
    ctx.maxRecursionNum = maxRecursion;
}


void ContextParser::parseMultiScatterSetting(SimulationContext &ctx)
{
    using namespace rapidjson;

    int multiScattering = 1;
    auto *p = Pointer("/multi_scatter/repeat").Get(d);
    if (p == nullptr) {
        fprintf(stderr, "\nWARNING! Config missing <multi_scatter.repeat>, using default value 1!\n");
    } else if (!p->IsInt()) {
        fprintf(stderr, "\nWARNING! Config <multi_scatter.repeat> is not a integer, using default 1!\n");
    } else {
        multiScattering = p->GetInt();
    }
    ctx.multiScatterNum = multiScattering;

    float prob = 1.0f;
    p = Pointer("/multi_scatter/probability").Get(d);
    if (p == nullptr) {
        fprintf(stderr, "\nWARNING! Config missing <multi_scatter.probability>, using default value 1.0!\n");
    } else if (!p->IsNumber()) {
        fprintf(stderr, "\nWARNING! Config <multi_scatter.probability> is not a number, using default 1.0!\n");
    } else {
        prob = static_cast<float>(p->GetDouble());
        prob = std::fmax(std::fmin(prob, 1.0f), 0.0f);
    }
    ctx.multiScatterProb = prob;

    for (int i = 0; i < multiScattering; i++) {
        ctx.rayTracingCtxs.emplace_back(std::vector<RayTracingContext *>());
    }
}


void ContextParser::parseSunSetting(SimulationContext &ctx)
{
    using namespace rapidjson;

    // Parsing sun altitude
    float sunAltitude = 0.0f;
    auto *p = Pointer("/sun/altitude").Get(d);
    if (p == nullptr) {
        fprintf(stderr, "\nWARNING! Config missing <sun.altitude>, using default 0.0!\n");
    } else if (!p->IsNumber()) {
        fprintf(stderr, "\nWARNING! config <sun.altitude> is not a number, using default 0.0!\n");
    } else {
        sunAltitude = static_cast<float>(p->GetDouble());
    }
    ctx.setSunPosition(90.0f, sunAltitude);

    // Parsing sun diameter
    float sunDiameter = 0.5f;
    p = Pointer("/sun/diameter").Get(d);
    if (p == nullptr) {
        fprintf(stderr, "\nWARNING! Config missing <sun.diameter>, using default 0.5!\n");
    } else if (!p->IsNumber()) {
        fprintf(stderr, "\nWARNING! Config <sun.diameter> is not a number, using default 0.5!\n");
    } else {
        sunDiameter = static_cast<float>(p->GetDouble());
    }
    ctx.sunDiameter = sunDiameter;
}


void ContextParser::parseCrystalSetting(SimulationContext &ctx, const rapidjson::Value &c, int ci)
{
    using namespace rapidjson;
    using namespace Math;

    char msgBuffer[512];

    auto *p = Pointer("/enable").Get(c);
    if (p == nullptr || !p->IsBool()) {
        sprintf(msgBuffer, "<crystal[%d].enable> cannot recgonize!", ci);
        throw std::invalid_argument(msgBuffer);
    } else if (!p->GetBool()) {
        return;
    }

    OrientationGenerator::Distribution axisDist, rollDist;
    float axisMean, rollMean;
    float axisStd, rollStd;

    p = Pointer("/axis/type").Get(c);
    if (p == nullptr || !p->IsString()) {
        sprintf(msgBuffer, "<crystal[%d].axis.type> cannot recgonize!", ci);
        throw std::invalid_argument(msgBuffer);
    } else if (*p == "Gauss") {
        axisDist = OrientationGenerator::Distribution::GAUSS;
    } else if (*p == "Uniform") {
        axisDist = OrientationGenerator::Distribution::UNIFORM;
    } else {
        sprintf(msgBuffer, "<crystal[%d].axis.type> cannot recgonize!", ci);
        throw std::invalid_argument(msgBuffer);
    }

    p = Pointer("/roll/type").Get(c);
    if (p == nullptr || !p->IsString()) {
        sprintf(msgBuffer, "<crystal[%d].roll.type> cannot recgonize!", ci);
        throw std::invalid_argument(msgBuffer);
    } else if (*p == "Gauss") {
        rollDist = OrientationGenerator::Distribution::GAUSS;
    } else if (*p == "Uniform") {
        rollDist = OrientationGenerator::Distribution::UNIFORM;
    } else {
        sprintf(msgBuffer, "<crystal[%d].roll.type> cannot recgonize!", ci);
        throw std::invalid_argument(msgBuffer);
    }

    p = Pointer("/axis/mean").Get(c);
    if (p == nullptr || !p->IsNumber()) {
        sprintf(msgBuffer, "<crystal[%d].axis.mean> cannot recgonize!", ci);
        throw std::invalid_argument(msgBuffer);
    } else {
        axisMean = static_cast<float>(90 - p->GetDouble());
    }

    p = Pointer("/axis/std").Get(c);
    if (p == nullptr || !p->IsNumber()) {
        sprintf(msgBuffer, "<crystal[%d].axis.std> cannot recgonize!", ci);
        throw std::invalid_argument(msgBuffer);
    } else {
        axisStd = static_cast<float>(p->GetDouble());
    }

    p = Pointer("/roll/mean").Get(c);
    if (p == nullptr || !p->IsNumber()) {
        sprintf(msgBuffer, "<crystal[%d].roll.mean> cannot recgonize!", ci);
        throw std::invalid_argument(msgBuffer);
    } else {
        rollMean = static_cast<float>(p->GetDouble());
    }

    p = Pointer("/roll/std").Get(c);
    if (p == nullptr || !p->IsNumber()) {
        sprintf(msgBuffer, "<crystal[%d].roll.std> cannot recgonize!", ci);
        throw std::invalid_argument(msgBuffer);
    } else {
        rollStd = static_cast<float>(p->GetDouble());
    }

    float population = 1.0;
    p = Pointer("/population").Get(c);
    if (p == nullptr || !p->IsNumber()) {
        fprintf(stderr, "\nWARNING! <crystal[%d].population> cannot recgonize, using default 1.0!\n", ci);
    } else {
        population = static_cast<float>(p->GetDouble());
    }

    p = Pointer("/type").Get(c);
    if (p == nullptr || !p->IsString()) {
        sprintf(msgBuffer, "<crystal[%d].type> cannot recgonize!", ci);
        throw std::invalid_argument(msgBuffer);
    } else {
        parseCrystalType(ctx, c, ci, population, axisDist, axisMean, axisStd, rollDist, rollMean, rollStd);
    }

    for (auto &rtc : ctx.rayTracingCtxs) {
        rtc.push_back(new RayTracingContext(&ctx));
    }
}

void ContextParser::parseCrystalType(SimulationContext &ctx, const rapidjson::Value &c, int ci,
    float population,
    Math::OrientationGenerator::Distribution axisDist, float axisMean, float axisStd,
    Math::OrientationGenerator::Distribution rollDist, float rollMean, float rollStd)
{
    using namespace rapidjson;

    char msgBuffer[512];

    auto *cryCtx = new CrystalContext(&ctx);
    const auto *p = Pointer("/parameter").Get(c);
    if (c["type"] == "HexCylinder") {
        if (p == nullptr || !p->IsNumber()) {
            sprintf(msgBuffer, "<crystal[%d].parameter> cannot recgonize!", ci);
            throw std::invalid_argument(msgBuffer);
        }
        auto h = static_cast<float>(p->GetDouble());
        cryCtx->setCrystal(Crystal::createHexCylinder(h), population,
            axisDist, axisMean, axisStd,
            rollDist, rollMean, rollStd);
    } else if (c["type"] == "HexPyramid") {
        if (p == nullptr || !p->IsArray()) {
            sprintf(msgBuffer, "<crystal[%d].parameter> cannot recgonize!", ci);
            throw std::invalid_argument(msgBuffer);
        } else if (p->Size() == 3) {
            auto h1 = static_cast<float>((*p)[0].GetDouble());
            auto h2 = static_cast<float>((*p)[1].GetDouble());
            auto h3 = static_cast<float>((*p)[2].GetDouble());
            cryCtx->setCrystal(Crystal::createHexPyramid(h1, h2, h3), population,
                axisDist, axisMean, axisStd,
                rollDist, rollMean, rollStd);
        } else if (p->Size() == 5) {
            int i1 = (*p)[0].GetInt();
            int i2 = (*p)[1].GetInt();
            auto h1 = static_cast<float>((*p)[2].GetDouble());
            auto h2 = static_cast<float>((*p)[3].GetDouble());
            auto h3 = static_cast<float>((*p)[4].GetDouble());
            cryCtx->setCrystal(Crystal::createHexPyramid(i1, i2, h1, h2, h3), population,
                axisDist, axisMean, axisStd,
                rollDist, rollMean, rollStd);
        } else if (p->Size() == 7) {
            int upperIdx1 = (*p)[0].GetInt();
            int upperIdx2 = (*p)[1].GetInt();
            int lowerIdx1 = (*p)[2].GetInt();
            int lowerIdx2 = (*p)[3].GetInt();
            auto h1 = static_cast<float>((*p)[4].GetDouble());
            auto h2 = static_cast<float>((*p)[5].GetDouble());
            auto h3 = static_cast<float>((*p)[6].GetDouble());
            cryCtx->setCrystal(Crystal::createHexPyramid(upperIdx1, upperIdx2, lowerIdx1,
                lowerIdx2, h1, h2, h3), population,
                axisDist, axisMean, axisStd,
                rollDist, rollMean, rollStd);
        } else {
            sprintf(msgBuffer, "<crystal[%d].parameter> number doesn't match!", ci);
            throw std::invalid_argument(msgBuffer);
        }
    } else if (c["type"] == "HexPyramidStackHalf") {
        if (p == nullptr || !p->IsArray()) {
            sprintf(msgBuffer, "<crystal[%d].parameter> cannot recgonize!", ci);
            throw std::invalid_argument(msgBuffer);
        } else if (p->Size() == 7) {
            int upperIdx1 = (*p)[0].GetInt();
            int upperIdx2 = (*p)[1].GetInt();
            int lowerIdx1 = (*p)[2].GetInt();
            int lowerIdx2 = (*p)[3].GetInt();
            auto h1 = static_cast<float>((*p)[4].GetDouble());
            auto h2 = static_cast<float>((*p)[5].GetDouble());
            auto h3 = static_cast<float>((*p)[6].GetDouble());
            cryCtx->setCrystal(Crystal::createHexPyramidStackHalf(upperIdx1, upperIdx2, lowerIdx1,
                lowerIdx2, h1, h2, h3), population,
                axisDist, axisMean, axisStd,
                rollDist, rollMean, rollStd);
        } else {
            sprintf(msgBuffer, "<crystal[%d].parameter> number doesn't match!", ci);
            throw std::invalid_argument(msgBuffer);
        }
    } else if (c["type"] == "TriPyramid") {
        if (p == nullptr || !p->IsArray()) {
            sprintf(msgBuffer, "<crystal[%d].parameter> cannot recgonize!", ci);
            throw std::invalid_argument(msgBuffer);
        } else if (p->Size() == 5) {
            int i1 = (*p)[0].GetInt();
            int i2 = (*p)[1].GetInt();
            auto h1 = static_cast<float>((*p)[2].GetDouble());
            auto h2 = static_cast<float>((*p)[3].GetDouble());
            auto h3 = static_cast<float>((*p)[4].GetDouble());
            cryCtx->setCrystal(Crystal::createTriPyramid(i1, i2, h1, h2, h3), population,
                axisDist, axisMean, axisStd,
                rollDist, rollMean, rollStd);
        } else {
            sprintf(msgBuffer, "<crystal[%d].parameter> number doesn't match!", ci);
            throw std::invalid_argument(msgBuffer);
        }
    } else if (c["type"] == "CubicPyramid") {
        if (p == nullptr || !p->IsArray()) {
            sprintf(msgBuffer, "<crystal[%d].parameter> cannot recgonize!", ci);
            throw std::invalid_argument(msgBuffer);
        } else if (p->Size() == 2) {
            auto h1 = static_cast<float>((*p)[0].GetDouble());
            auto h2 = static_cast<float>((*p)[1].GetDouble());
            cryCtx->setCrystal(Crystal::createCubicPyramid(h1, h2), population,
                axisDist, axisMean, axisStd,
                rollDist, rollMean, rollStd);
        } else {
            sprintf(msgBuffer, "<crystal[%d].parameter> number doesn't match!", ci);
            throw std::invalid_argument(msgBuffer);
        }
    } else if (c["type"] == "Custom") {
        if (p == nullptr || !p->IsString()) {
            sprintf(msgBuffer, "<crystal[%d].parameter> cannot recgonize!", ci);
            throw std::invalid_argument(msgBuffer);
        } else {
            char modelFileNameBuffer[512] = { 0 };
            auto n = filename.rfind('/');
            if (n == std::string::npos) {
                sprintf(modelFileNameBuffer, "models/%s", p->GetString());
            } else {
                sprintf(modelFileNameBuffer, "%s/models/%s", filename.substr(0, n).c_str(), p->GetString());
            }
            std::FILE *file = fopen(modelFileNameBuffer, "r");
            if (!file) {
                sprintf(msgBuffer, "<crystal[%d].parameter> cannot open model file!", ci);
                throw std::invalid_argument(msgBuffer);
            }
            Crystal *crystal = parseCustomCrystal(file);
            cryCtx->setCrystal(crystal, population,
                axisDist, axisMean, axisStd,
                rollDist, rollMean, rollStd);
            fclose(file);
        }
    } else {
        sprintf(msgBuffer, "<crystal[%d].type> cannot recgonize!", ci);
        throw std::invalid_argument(msgBuffer);
    }
    ctx.crystalCtxs.push_back(cryCtx);
}


Crystal * ContextParser::parseCustomCrystal(std::FILE *file)
{
    std::vector<Math::Vec3f> vertexes;
    std::vector<Math::TriangleIdx> faces;
    float vbuf[3];
    int fbuf[3];
    int c;
    while ((c = std::fgetc(file)) != EOF) {
        switch (c) {
            case 'v':
            case 'V':
                std::fscanf(file, "%f %f %f", vbuf+0, vbuf+1, vbuf+2);
                vertexes.emplace_back(Math::Vec3f(vbuf));
                break;
            case 'f':
            case 'F':
                std::fscanf(file, "%d %d %d", fbuf+0, fbuf+1, fbuf+2);
                faces.emplace_back(Math::TriangleIdx(fbuf[0]-1, fbuf[1]-1, fbuf[2]-1));
                break;
            default:
                break;
        }
    }
    return new Crystal(vertexes, faces);
}


void ContextParser::parseSettings(SimulationContext &ctx)
{
    using namespace rapidjson;

    char msgBuffer[512];

    parseRayNumber(ctx);
    parseMaxRecursion(ctx);
    parseSunSetting(ctx);
    parseMultiScatterSetting(ctx);

    const auto *p = Pointer("/crystal").Get(d);
    if (p == nullptr || !p->IsArray()) {
        sprintf(msgBuffer, "Missing <crystal>. Parsing fail!");
        throw std::invalid_argument(msgBuffer);
    }

    int ci = 0;
    for (const Value &c : p->GetArray()) {
        parseCrystalSetting(ctx, c, ci);
        ci++;
    }
}

}   // namespace IceHalo