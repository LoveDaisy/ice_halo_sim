#include "context.h"
#include "linearalgebra.h"

#include "rapidjson/pointer.h"
#include "rapidjson/error/en.h"
#include "rapidjson/filereadstream.h"

#include <unordered_set>


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
    float x = -std::cos(lat * Crystal::PI / 180.0f) * std::cos(lon * Crystal::PI / 180.0f);
    float y = -std::cos(lat * Crystal::PI / 180.0f) * std::sin(lon * Crystal::PI / 180.0f);
    float z = -std::sin(lat * Crystal::PI / 180.0f);

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
    Crystal *g, float populationWeight,
    OrientationGenerator::Distribution axisDist, float axisMean, float axisStd,
    OrientationGenerator::Distribution rollDist, float rollMean, float rollStd)
{
    crystals.push_back(g);
    populationWeights.push_back(populationWeight);
    oriGens.emplace_back(axisDist, axisMean * Crystal::PI / 180.0f, axisStd * Crystal::PI / 180.0f,
                         rollDist, rollMean * Crystal::PI / 180.0f, rollStd * Crystal::PI / 180.0f);
    rayNums.push_back(0);
    rayTracingCtxs.push_back(new RayTracingContext());
}


Crystal * CrystalContext::getCrystal(int i)
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

    delete[] rayDir;
    delete[] mainAxRot;
    delete[] crystalId;
}


void SimulationContext::setTotalRayNum(uint64_t num)
{
    this->totalRayNum = num;
    
    delete[] rayDir;
    rayDir = new float[num * 3];

    delete[] mainAxRot;
    mainAxRot = new float[num * 3];

    delete[] crystalId;
    crystalId = new int[num];

    for (auto i = 0; i < num; i++) {
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


void SimulationContext::printCrystalInfo()
{
    for (auto g : crystalCtx->crystals) {
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


ContextParser * ContextParser::getFileParser(const char* filename)
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
    ctx.setTotalRayNum(rayNumber);
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
        float h = static_cast<float>(p->GetDouble());
        ctx.crystalCtx->addGeometry(Crystal::createHexCylinder(h), population,
            axisDist, axisMean, axisStd,
            rollDist, rollMean, rollStd);
    } else if (c["type"] == "HexPyramid") {
        if (p == nullptr || !p->IsArray()) {
            sprintf(msgBuffer, "<crystal[%d].parameter> cannot recgonize!", ci);
            throw std::invalid_argument(msgBuffer);
        } else if (p->Size() == 3) {
            float h1 = static_cast<float>((*p)[0].GetDouble());
            float h2 = static_cast<float>((*p)[1].GetDouble());
            float h3 = static_cast<float>((*p)[2].GetDouble());
            ctx.crystalCtx->addGeometry(Crystal::createHexPyramid(h1, h2, h3), population,
                axisDist, axisMean, axisStd,
                rollDist, rollMean, rollStd);
        } else if (p->Size() == 5) {
            int i1 = (*p)[0].GetInt();
            int i2 = (*p)[1].GetInt();
            float h1 = static_cast<float>((*p)[2].GetDouble());
            float h2 = static_cast<float>((*p)[3].GetDouble());
            float h3 = static_cast<float>((*p)[4].GetDouble());
            ctx.crystalCtx->addGeometry(Crystal::createHexPyramid(i1, i2, h1, h2, h3), population,
                axisDist, axisMean, axisStd,
                rollDist, rollMean, rollStd);
        } else if (p->Size() == 7) {
            int upperIdx1 = (*p)[0].GetInt();
            int upperIdx2 = (*p)[1].GetInt();
            int lowerIdx1 = (*p)[2].GetInt();
            int lowerIdx2 = (*p)[3].GetInt();
            float h1 = static_cast<float>((*p)[4].GetDouble());
            float h2 = static_cast<float>((*p)[5].GetDouble());
            float h3 = static_cast<float>((*p)[6].GetDouble());
            ctx.crystalCtx->addGeometry(Crystal::createHexPyramid(upperIdx1, upperIdx2, lowerIdx1,
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
            float h1 = static_cast<float>((*p)[4].GetDouble());
            float h2 = static_cast<float>((*p)[5].GetDouble());
            float h3 = static_cast<float>((*p)[6].GetDouble());
            ctx.crystalCtx->addGeometry(Crystal::createHexPyramidStackHalf(upperIdx1, upperIdx2, lowerIdx1,
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
            float h1 = static_cast<float>((*p)[2].GetDouble());
            float h2 = static_cast<float>((*p)[3].GetDouble());
            float h3 = static_cast<float>((*p)[4].GetDouble());
            ctx.crystalCtx->addGeometry(Crystal::createTriPyramid(i1, i2, h1, h2, h3), population,
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
            float h1 = static_cast<float>((*p)[0].GetDouble());
            float h2 = static_cast<float>((*p)[1].GetDouble());
            ctx.crystalCtx->addGeometry(Crystal::createCubicPyramid(h1, h2), population,
                axisDist, axisMean, axisStd,
                rollDist, rollMean, rollStd);
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
