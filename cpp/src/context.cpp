#include "context.h"
#include "linearalgebra.h"

#include "rapidjson/pointer.h"
#include "rapidjson/error/en.h"
#include "rapidjson/filereadstream.h"

#include <unordered_set>


EnvironmentContext::EnvironmentContext(SimulationContext *ctx) :
    simCtx(ctx)
{ }


void EnvironmentContext::setWavelength(float wavelength)
{
    this->wavelength = wavelength;
}


float EnvironmentContext::getWavelength() const
{
    return wavelength;
}


void EnvironmentContext::setSunPosition(float lon, float lat)
{
    float x = -std::cos(lat * Crystal::PI / 180.0f) * std::cos(lon * Crystal::PI / 180.0f);
    float y = -std::cos(lat * Crystal::PI / 180.0f) * std::sin(lon * Crystal::PI / 180.0f);
    float z = -std::sin(lat * Crystal::PI / 180.0f);

    this->sunDir[0] = x;
    this->sunDir[1] = y;
    this->sunDir[2] = z;
}


const float * EnvironmentContext::getSunDirection() const
{
    return this->sunDir;
}


CrystalContext::CrystalContext(SimulationContext *ctx) :
    simCtx(ctx)
{ }


void CrystalContext::setCrystal(
    Crystal *g, float populationWeight,
    OrientationGenerator::Distribution axisDist, float axisMean, float axisStd,
    OrientationGenerator::Distribution rollDist, float rollMean, float rollStd )
{
    this->crystal = g;
    this->populationRatio = populationWeight;
    this->oriGen = OrientationGenerator(
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
    simCtx(ctx), initRayNum(0), currentRayNum(0),
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
    int maxRayNum = initRayNum * simCtx->getMaxRecursionNum();
    
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
}


void RayTracingContext::initRays(CrystalContext *ctx)
{
    Crystal * crystal = ctx->getCrystal();
    int faceNum = crystal->faceNum();
    auto *faces = new float[faceNum * 9];
    crystal->copyFaceData(faces);

    const float *sunDir = simCtx->getSunDir();

    rays.clear();
    activeRaySeg.clear();
    for (int i = 0; i < initRayNum; i++) {
        fillDir(sunDir, rayDir+i*3, mainAxRot+i*3, ctx);
        
        int idx = chooseFace(faces, faceNum, rayDir+i*3);
        faceId[i] = idx;

        fillPts(faces, idx, rayPts+i*3);
        crystal->copyNormalData(idx, faceNorm+i*3);

        auto *r = new Ray(rayPts+i*3, rayDir+i*3, 1.0f, faceId[i]);
        rays.push_back(r);
        activeRaySeg.push_back(r->firstRaySeg);

        // printf("PTS:%+.4f,%+.4f,%+.4f\n", rayPts[i*3+0], rayPts[i*3+1], rayPts[i*3+2]);
        // printf("DIR:%+.4f,%+.4f,%+.4f\n", rayDir[i*3+0], rayDir[i*3+1], rayDir[i*3+2]);
        // printf("AX:%+.4f,%+.4f,%+.4f\n", mainAxRot[i*3+0], mainAxRot[i*3+1], mainAxRot[i*3+2]);
    }

    delete[] faces;
}


void RayTracingContext::initRays(int rayNum, const float *dir, CrystalContext *ctx)
{
    setRayNum(rayNum);

    Crystal * crystal = ctx->getCrystal();
    int faceNum = crystal->faceNum();
    auto *faces = new float[faceNum * 9];
    crystal->copyFaceData(faces);

    rays.clear();
    activeRaySeg.clear();
    for (int i = 0; i < initRayNum; i++) {
        fillDir(dir+i*3, rayDir+i*3, mainAxRot+i*3, ctx);
        
        int idx = chooseFace(faces, faceNum, rayDir+i*3);
        faceId[i] = idx;

        fillPts(faces, idx, rayPts+i*3);
        crystal->copyNormalData(idx, faceNorm+i*3);

        auto *r = new Ray(rayPts+i*3, rayDir+i*3, 1.0f, faceId[i]);
        rays.push_back(r);
        activeRaySeg.push_back(r->firstRaySeg);

        printf("DIR:%+.4f,%+.4f,%+.4f\n", rayPts[i*3+0], rayPts[i*3+1], rayPts[i*3+2]);
    }

    delete[] faces;
}


void RayTracingContext::clearRays()
{
    rays.clear();
}


void RayTracingContext::commitHitResult()
{
    RaySegmentFactory *raySegPool = RaySegmentFactory::getInstance();
    std::vector<RaySegment *> tmpRaySegs;
    for (int i = 0; i < currentRayNum; i++) {
        RaySegment *lastSeg = activeRaySeg[i];
        RaySegment *seg = raySegPool->getRaySegment(rayPts + i*3, rayDir2 + i*3,
            lastSeg->w * rayW2[i], faceId[i]);
        lastSeg->nextReflect = seg;
        seg->prev = lastSeg;
        tmpRaySegs.push_back(seg);

        seg = raySegPool->getRaySegment(rayPts + i*3, rayDir3 + i*3,
            lastSeg->w * (1.0f - rayW2[i]), faceId[i]);
        lastSeg->nextRefract = seg;
        seg->prev = lastSeg;
        tmpRaySegs.push_back(seg);
    }
    activeRaySeg.clear();
    for (int i = 0; i < tmpRaySegs.size(); i += 2) {
        activeRaySeg.push_back(tmpRaySegs[i]);
    }
    for (int i = 1; i < tmpRaySegs.size(); i += 2) {
        activeRaySeg.push_back(tmpRaySegs[i]);
    }
    tmpRaySegs.clear();

    memcpy(rayDir, rayDir2, currentRayNum * 3 * sizeof(float));
    memcpy(rayDir + currentRayNum*3, rayDir3, currentRayNum * 3 * sizeof(float));
    memcpy(rayPts + currentRayNum*3, rayPts, currentRayNum * 3 * sizeof(float));
    memcpy(faceId + currentRayNum, faceId, currentRayNum * sizeof(int));

    currentRayNum *= 2;
}


void RayTracingContext::commitPropagateResult(CrystalContext *ctx)
{
    std::vector<RaySegment *> tmpRaySegs;

    int k = 0;
    for (int i = 0; i < currentRayNum; i++) {
        if (faceId2[i] >= 0 && activeRaySeg[i]->w > 1e-3) {
            tmpRaySegs.push_back(activeRaySeg[i]);
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
    activeRaySeg.swap(tmpRaySegs);
    tmpRaySegs.clear();
}


bool RayTracingContext::isFinished()
{
    return activeRaySeg.empty();
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
}


int RayTracingContext::chooseFace(const float *faces, int faceNum, const float *rayDir)
{
    auto *frac = new float[faceNum];

    for (int i = 0; i < faceNum; i++) {
        float v1[3], v2[3];
        LinearAlgebra::vec3FromTo(faces + i*9, faces + i*9 + 3, v1);
        LinearAlgebra::vec3FromTo(faces + i*9, faces + i*9 + 6, v2);
        float norm[3];
        LinearAlgebra::cross3(v1, v2, norm);
        float c = LinearAlgebra::dot3(rayDir, norm);
        frac[i] = c < 0 ? LinearAlgebra::norm3(norm) / 2 * (-c) : 0;
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
    totalRayNum(0), maxRecursionNum(9)
{
    envCtx = new EnvironmentContext(this);
}


SimulationContext::~SimulationContext()
{
    delete envCtx;
    for (auto p : crystalCtxs) {
        delete p;
    }
    for (auto p : rayTracingCtxs) {
        delete p;
    }
}


int SimulationContext::getMaxRecursionNum() const
{
    return maxRecursionNum;
}


int SimulationContext::getCrystalNum() const
{
    return static_cast<int>(crystalCtxs.size());
}


void SimulationContext::setWavelength(float wavelength)
{
    envCtx->setWavelength(wavelength);
}


float SimulationContext::getWavelength()
{
    return envCtx->getWavelength();
}


const float * SimulationContext::getSunDir() const
{
    return envCtx->sunDir;
}


void SimulationContext::applySettings()
{
    if (totalRayNum <= 0) return;

    size_t popSize = crystalCtxs.size();
    float popWeightSum = 0.0f;
    for (auto c : crystalCtxs) {
        popWeightSum += c->populationRatio;
    }
    for (auto c : crystalCtxs) {
        c->populationRatio /= popWeightSum;
    }
    
    size_t currentIdx = 0;
    for (int i = 0; i < popSize-1; i++) {
        auto tmpPopSize = static_cast<int>(crystalCtxs[i]->populationRatio * totalRayNum);
        rayTracingCtxs[i]->setRayNum(tmpPopSize);
        currentIdx += tmpPopSize;
    }
    auto tmpPopSize = static_cast<int>(totalRayNum - currentIdx);
    rayTracingCtxs[popSize-1]->setRayNum(tmpPopSize);
}


CrystalContext * SimulationContext::getCrystalContext(int i)
{
    return crystalCtxs[i];
}


RayTracingContext * SimulationContext::getRayTracingContext(int i)
{
    return rayTracingCtxs[i];
}


void SimulationContext::writeFinalDirections(const char *filename)
{
    std::FILE* file = std::fopen(filename, "wb");
    if (!file) return;

    uint64_t totalRaySegNum = 0;
    for (auto &rc : rayTracingCtxs) {
        for (auto &r : rc->rays) {
            totalRaySegNum += r->totalNum();
        }
    }
    fwrite(&totalRaySegNum, sizeof(uint64_t), 1, file);
    
    size_t currentIdx = 0;
    std::vector<RaySegment*> v;
    int k = 0;
    for (auto &rc : rayTracingCtxs) {
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
                    LinearAlgebra::rotateZBack(rc->mainAxRot+currentIdx*3, finalDir);
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
    for (auto &rc : rayTracingCtxs) {
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
                    LinearAlgebra::rotateZBack(rc->mainAxRot+currentIdx*3, finalDir);
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

ContextParser::ContextParser(rapidjson::Document &d) : d(std::move(d))
{ }


ContextParser * ContextParser::createFileParser(const char *filename)
{
    using namespace rapidjson;

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

    return new ContextParser(d);
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
    ctx.envCtx->setSunPosition(90.0f, sunAltitude);
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
    ctx.envCtx->setSunPosition(90.0f, sunAltitude);
}


void ContextParser::parseCrystalSetting(SimulationContext &ctx, const rapidjson::Value &c, int ci)
{
    using namespace rapidjson;

    OrientationGenerator::Distribution axisDist, rollDist;
    float axisMean, rollMean;
    float axisStd, rollStd;

    char msgBuffer[512];

    auto *p = Pointer("/axis/type").Get(c);
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

    ctx.rayTracingCtxs.push_back(new RayTracingContext(&ctx));
}

void ContextParser::parseCrystalType(SimulationContext &ctx, const rapidjson::Value &c, int ci,
    float population,
    OrientationGenerator::Distribution axisDist, float axisMean, float axisStd,
    OrientationGenerator::Distribution rollDist, float rollMean, float rollStd)
{
    using namespace rapidjson;

    char msgBuffer[512];

    const auto *p = Pointer("/parameter").Get(c);
    if (c["type"] == "HexCylinder") {
        if (p == nullptr || !p->IsNumber()) {
            sprintf(msgBuffer, "<crystal[%d].parameter> cannot recgonize!", ci);
            throw std::invalid_argument(msgBuffer);
        }
        auto h = static_cast<float>(p->GetDouble());
        auto *cryCtx = new CrystalContext(&ctx);
        cryCtx->setCrystal(Crystal::createHexCylinder(h), population,
            axisDist, axisMean, axisStd,
            rollDist, rollMean, rollStd);
        ctx.crystalCtxs.push_back(cryCtx);
    } else if (c["type"] == "HexPyramid") {
        if (p == nullptr || !p->IsArray()) {
            sprintf(msgBuffer, "<crystal[%d].parameter> cannot recgonize!", ci);
            throw std::invalid_argument(msgBuffer);
        } else if (p->Size() == 3) {
            auto h1 = static_cast<float>((*p)[0].GetDouble());
            auto h2 = static_cast<float>((*p)[1].GetDouble());
            auto h3 = static_cast<float>((*p)[2].GetDouble());
            auto *cryCtx = new CrystalContext(&ctx);
            cryCtx->setCrystal(Crystal::createHexPyramid(h1, h2, h3), population,
                axisDist, axisMean, axisStd,
                rollDist, rollMean, rollStd);
            ctx.crystalCtxs.push_back(cryCtx);
        } else if (p->Size() == 5) {
            int i1 = (*p)[0].GetInt();
            int i2 = (*p)[1].GetInt();
            auto h1 = static_cast<float>((*p)[2].GetDouble());
            auto h2 = static_cast<float>((*p)[3].GetDouble());
            auto h3 = static_cast<float>((*p)[4].GetDouble());
            auto *cryCtx = new CrystalContext(&ctx);
            cryCtx->setCrystal(Crystal::createHexPyramid(i1, i2, h1, h2, h3), population,
                axisDist, axisMean, axisStd,
                rollDist, rollMean, rollStd);
            ctx.crystalCtxs.push_back(cryCtx);
        } else if (p->Size() == 7) {
            int upperIdx1 = (*p)[0].GetInt();
            int upperIdx2 = (*p)[1].GetInt();
            int lowerIdx1 = (*p)[2].GetInt();
            int lowerIdx2 = (*p)[3].GetInt();
            auto h1 = static_cast<float>((*p)[4].GetDouble());
            auto h2 = static_cast<float>((*p)[5].GetDouble());
            auto h3 = static_cast<float>((*p)[6].GetDouble());
            auto *cryCtx = new CrystalContext(&ctx);
            cryCtx->setCrystal(Crystal::createHexPyramid(upperIdx1, upperIdx2, lowerIdx1,
                lowerIdx2, h1, h2, h3), population,
                axisDist, axisMean, axisStd,
                rollDist, rollMean, rollStd);
            ctx.crystalCtxs.push_back(cryCtx);
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
            auto *cryCtx = new CrystalContext(&ctx);
            cryCtx->setCrystal(Crystal::createHexPyramidStackHalf(upperIdx1, upperIdx2, lowerIdx1,
                lowerIdx2, h1, h2, h3), population,
                axisDist, axisMean, axisStd,
                rollDist, rollMean, rollStd);
            ctx.crystalCtxs.push_back(cryCtx);
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
            auto *cryCtx = new CrystalContext(&ctx);
            cryCtx->setCrystal(Crystal::createTriPyramid(i1, i2, h1, h2, h3), population,
                axisDist, axisMean, axisStd,
                rollDist, rollMean, rollStd);
            ctx.crystalCtxs.push_back(cryCtx);
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
            auto *cryCtx = new CrystalContext(&ctx);
            cryCtx->setCrystal(Crystal::createCubicPyramid(h1, h2), population,
                axisDist, axisMean, axisStd,
                rollDist, rollMean, rollStd);
            ctx.crystalCtxs.push_back(cryCtx);
        } else {
            sprintf(msgBuffer, "<crystal[%d].parameter> number doesn't match!", ci);
            throw std::invalid_argument(msgBuffer);
        }
    } else {
        sprintf(msgBuffer, "<crystal[%d].type> cannot recgonize!", ci);
        throw std::invalid_argument(msgBuffer);
    }
}


void ContextParser::parseSettings(SimulationContext &ctx)
{
    using namespace rapidjson;

    char msgBuffer[512];

    parseRayNumber(ctx);
    parseMaxRecursion(ctx);
    parseSunSetting(ctx);

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
