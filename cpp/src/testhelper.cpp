#include "testhelper.h"
#include "linearalgebra.h"

RayTracingContext::RayTracingContext() :
    oriGen(Geometry::PI*2, Geometry::PI*2),
    incDirNum(100000), wl(0)
{
    this->g = Geometry::createHexCylindar(5.0f);

    float sunLon = -90.0f * Geometry::PI / 180.0f;
    float sunLat = 27.0f * Geometry::PI / 180.0f;
    setSunPosition(sunLon, sunLat);

    rayDir = new float[incDirNum * 3];
    mainAxRot = new float[incDirNum * 3];

    maxRecursion = 9;
    raysPerDirection = 20;
}

RayTracingContext::~RayTracingContext()
{
    delete[] rayDir;
    delete[] mainAxRot;
}

const Geometry * RayTracingContext::getGeometry() const
{
    return this->g;
}

const float * RayTracingContext::getRayDirections() const
{
    return this->rayDir;
}

const float * RayTracingContext::getMainAxisRotations() const
{
    return this->mainAxRot;
}

void RayTracingContext::setWavelength(float wl)
{
    float n = IceRefractiveIndex::n(wl);
    g->setRefractiveIndex(n);
    this->wl = wl;
}

int RayTracingContext::getIncDirNum() const
{
    return incDirNum;
}

int RayTracingContext::getRaysPerDirection() const
{
    return raysPerDirection;
}

int RayTracingContext::getMaxRecursion() const
{
    return maxRecursion;
}

void RayTracingContext::setRaysPerDirection(int raysPerDirection)
{
    this->raysPerDirection = raysPerDirection;
}

void RayTracingContext::setIncDirNum(int incDirNum)
{
    this->incDirNum = incDirNum;
    this->initialized = false;
}

void RayTracingContext::setSunPosition(float lon, float lat)
{
    sunDir[0] = -std::cos(lat) * std::cos(lon);
    sunDir[1] = -std::cos(lat) * std::sin(lon);
    sunDir[2] = -std::sin(lat);
    this->initialized = false;
}

void RayTracingContext::setGeometry(Geometry *g)
{
    delete this->g;
    this->g = g;
    this->initialized = false;
}

bool RayTracingContext::isSettingsApplied()
{
    return initialized;
}

void RayTracingContext::applySettings()
{
    if (rayDir) {
        delete[] rayDir;
        rayDir = new float[3 * incDirNum];
    }
    if (mainAxRot) {
        delete[] mainAxRot;
        mainAxRot = new float[3 * incDirNum];
    }

    memset(rayDir, 0, incDirNum*3*sizeof(float));
    memset(mainAxRot, 0, incDirNum*3*sizeof(float));
    oriGen.fillData(sunDir, incDirNum, rayDir, mainAxRot);

    this->initialized = true;
}

void RayTracingContext::writeFinalDirections()
{
    size_t totalDir = 0;
    for (auto r : rays) {
        totalDir += r->totalNum();
    }

    printf("rays.size(): %lu, incDirNum: %d, totalDir: %ld\n", rays.size(), incDirNum, totalDir);

    auto *finalDir = new float[totalDir * 4];
    int k = 0;
    std::vector<RaySegment *> v;
    for (int i = 0; i < rays.size(); i++) {
        auto r = rays[i];
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
            if (p->isValidEnd() &&
                    LinearAlgebra::dot3(p->dir.val(), r->firstRaySeg->dir.val()) < 1.0f - 1e-5 ) {
                memcpy(finalDir + k*4, p->dir.val(), 3*sizeof(float));
                LinearAlgebra::rotateZBack(mainAxRot+i/raysPerDirection*3, 1, finalDir+k*4);
                finalDir[k*4+3] = p->w;
                k++;
            }
        }
    }

    char filename[128];
    std::sprintf(filename, "directions_%.1f.bin", wl);
    FILE * pFile;
    pFile = fopen(filename, "wb");
    fwrite(&totalDir, sizeof(int), 1, pFile);
    fwrite(finalDir, sizeof(float), totalDir * 4, pFile);
    fclose(pFile);

    delete[] finalDir;
}

void RayTracingContext::clearRays()
{
    RaySegmentFactory::getInstance()->clear();
    rays.clear();
}


// void RayTracingContext::writeGeometryInfo()
// {
//     int vtxNum = g->vtxNum();
//     int faceNum = g->faceNum();

//     auto *vtx_buffer = new float[vtxNum * 3];
//     g->copyVertexData(vtx_buffer);

//     auto *face_buffer = new int[faceNum * 3];
//     g->copyFaceIdxData(face_buffer);

//     FILE * pFile;
//     pFile = fopen("geometry.bin", "wb");
//     fwrite(&vtxNum, sizeof(int), 1, pFile);
//     fwrite(vtx_buffer, sizeof(float), vtxNum * 3, pFile);
//     fwrite(&faceNum, sizeof(int), 1, pFile);
//     fwrite(face_buffer, sizeof(int), faceNum * 3, pFile);
//     fclose(pFile);
// }


// void RayTracingContext::writeRayPaths(const std::vector<Ray*> &rays)
// {
//     FILE * pFile;
//     pFile = fopen("rays.bin", "wb");

//     int totalRays = 0;
//     for (auto r : rays) {
//         totalRays += r->totalNum();
//     }
//     fwrite(&totalRays, sizeof(int), 1, pFile);

//     for (auto r : rays) {
//         std::vector<RaySegment *> v;
//         v.push_back(r->firstRaySeg);
//         while (!v.empty()) {
//             RaySegment *p = v.back();
//             v.pop_back();
//             if (p->nextReflect) {
//                 v.push_back(p->nextReflect);
//             }
//             if (p->nextRefract) {
//                 v.push_back(p->nextRefract);
//             }
//             if (p->isValidEnd()
//                 && LinearAlgebra::dot3(p->dir.val(), r->firstRaySeg->dir.val()) < 1.0f - 1e-5 ) {
//                 std::vector<RaySegment *> tmpV;
//                 RaySegment *q = p;
//                 while (q->prev) {
//                     tmpV.push_back(q);
//                     q = q->prev;
//                 }
//                 tmpV.push_back(q);

//                 int tmpNum = tmpV.size();
//                 fwrite(&tmpNum, sizeof(int), 1, pFile);

//                 std::reverse(tmpV.begin(), tmpV.end());
//                 for (auto tmp_r : tmpV) {
//                     float tmpRayData[7] = {tmp_r->pt.x(), tmp_r->pt.y(), tmp_r->pt.z(),
//                         tmp_r->dir.x(), tmp_r->dir.y(), tmp_r->dir.z(),
//                         tmp_r->w};
//                     auto faceId = static_cast<float>(tmp_r->faceId);

//                     fwrite(tmpRayData, sizeof(float), 7, pFile);
//                     fwrite(&faceId, sizeof(float), 1, pFile);

//                 }
//             }
//         }
//     }
//     fclose(pFile);
// }


OrientationGenerator::OrientationGenerator(float axStd, float rollStd,
        AxisDistribution ax, RollDistribution roll) :
    axDist(ax), rollDist(roll),
    axStd(axStd), rollStd(rollStd)
{
    unsigned int seed = static_cast<unsigned int>(std::chrono::system_clock::now().time_since_epoch().count());
    // unsigned int seed = 2345;
    generator.seed(seed);
}


void OrientationGenerator::fillData(const float *sunDir, int num, float *rayDir, float *mainAxRot)
{
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

        memcpy(rayDir+i*3, sunDir, 3*sizeof(float));
        LinearAlgebra::rotateZ(mainAxRot+i*3, rayDir+i*3);
    }
}


void OrientationGenerator::setAxisOrientation(AxisDistribution ax, float axStd)
{
    this->axDist = ax;
    this->axStd = axStd;
}


void OrientationGenerator::setAxisRoll(RollDistribution roll, float rollStd)
{
    this->rollDist = roll;
    this->rollStd = rollStd;
}


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


