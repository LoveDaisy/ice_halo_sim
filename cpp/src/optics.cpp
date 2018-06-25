#include <limits>

#include "optics.h"
#include "linearalgebra.h"
#include "context.h"
#include "threadingpool.h"


namespace IceHalo {

RaySegment::RaySegment() :
    nextReflect(nullptr),
    nextRefract(nullptr),
    prev(nullptr),
    pt(0, 0, 0), dir(0, 0, 0), w(0), faceId(-1), isFinished(false)
{ }

RaySegment::RaySegment(const float *pt, const float *dir, float w, int faceId) :
    nextReflect(nullptr),
    nextRefract(nullptr),
    prev(nullptr),
    pt(pt), dir(dir), w(w), faceId(faceId), isFinished(false)
{ }

bool RaySegment::isValidEnd()
{
    return w > 0 && faceId >= 0 && isFinished;
}

void RaySegment::reset()
{
    nextReflect = nullptr;
    nextRefract = nullptr;
    prev = nullptr;
    pt.val(0, 0, 0);
    dir.val(0, 0, 0);
    faceId = -1;
    isFinished = false;
}


Ray::Ray(const float *pt, const float *dir, float w, int faceId)
{
    firstRaySeg = RaySegmentFactory::getInstance()->getRaySegment(pt, dir, w, faceId);
}


Ray::Ray(RaySegment *seg) : firstRaySeg(seg)
{ }


Ray::~Ray()
{
    firstRaySeg = nullptr;
}

size_t Ray::totalNum()
{
    if (firstRaySeg == nullptr) {
        return 0;
    }

    std::vector<RaySegment*> v;
    v.push_back(firstRaySeg);

    size_t n = 0;
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
            n++;
        }
    }
    return n;
}


RaySegmentFactory * RaySegmentFactory::instance = nullptr;


void Optics::hitSurface(float n, RayTracingContext *rayCtx)
{
    Pool *pool = Pool::getInstance();
    int step = rayCtx->currentRayNum / 200;
    for (int startIdx = 0; startIdx < rayCtx->currentRayNum; startIdx += step) {
        int endIdx = std::min(startIdx + step, rayCtx->currentRayNum);
        pool->addJob(std::bind(&Optics::hitSurfaceRange, n, rayCtx, startIdx, endIdx));
    }
    pool->waitFinish();
}


void Optics::hitSurfaceRange(float n, RayTracingContext *rayCtx, int startIdx, int endIdx)
{
    for (int i = startIdx; i < endIdx; i++) {
        const float *tmp_dir = rayCtx->rayDir + i*3;
        const float *tmp_norm = rayCtx->faceNorm + i*3;

        float cos_theta = LinearAlgebra::dot3(tmp_dir, tmp_norm);

        float rr = cos_theta > 0 ? n : 1.0f / n;

        rayCtx->rayW2[i] = getReflectRatio(cos_theta, rr);

        for (int j = 0; j < 3; ++j) {
            rayCtx->rayDir2[i*3+j] = tmp_dir[j] - 2 * cos_theta * tmp_norm[j];
        }

        float d = (1.0f - rr * rr) / (cos_theta * cos_theta) + rr * rr;
        for (int j = 0; j < 3; j++) {
            rayCtx->rayDir3[i*3+j] = d <= 0.0f ? rayCtx->rayDir2[i*3+j] :
            rr * tmp_dir[j] - (rr - std::sqrt(d)) * cos_theta * tmp_norm[j];
        }
    }
}

void Optics::propagate(RayTracingContext *rayCtx, CrystalContext *cryCtx)
{
    auto faceNum = cryCtx->crystal->faceNum();
    auto *faces = new float[faceNum * 9];
    cryCtx->crystal->copyFaceData(faces);

    Pool *pool = Pool::getInstance();
    int step = rayCtx->currentRayNum / 200;
    for (int startIdx = 0; startIdx < rayCtx->currentRayNum; startIdx += step) {
        int endIdx = std::min(startIdx + step, rayCtx->currentRayNum);
        pool->addJob(std::bind(&Optics::propagateRange, rayCtx, faceNum, faces, startIdx, endIdx));
    }
    pool->waitFinish();
}


void Optics::propagateRange(RayTracingContext *rayCtx, int faceNum, float *faces, int startIdx, int endIdx)
{
    for (int i = startIdx; i < endIdx; i++) {
        const float *tmp_pt = rayCtx->rayPts + i*3;
        const float *tmp_dir = rayCtx->rayDir + i*3;

        float min_t = std::numeric_limits<float>::max();

        int currFaceId = rayCtx->faceId[i];
        rayCtx->faceId2[i] = -1;
        for (int j = 0; j < faceNum; j++) {
            if (j == currFaceId) {
                continue;
            }
            const float *tmp_face = faces + j*9;
            
            float p[3];
            float t, alpha, beta;

            intersectLineFace(tmp_pt, tmp_dir, tmp_face, &p[0], &t, &alpha, &beta);
            if (t > 1e-6 && t < min_t && alpha >= 0 && beta >= 0 && alpha + beta <= 1) {
                min_t = t;

                rayCtx->rayPts2[i*3+0] = p[0];
                rayCtx->rayPts2[i*3+1] = p[1];
                rayCtx->rayPts2[i*3+2] = p[2];

                rayCtx->faceId2[i] = j;
            }
        }
    }
}


void Optics::traceRays(SimulationContext &context)
{
    RaySegmentFactory::getInstance()->clear();

    auto totalRays = context.getTotalInitRays();
    auto maxRecursion = context.getMaxRecursionNum();
    auto multiScatterNum = context.getMultiScatterNum();
    auto maxNum = totalRays * maxRecursion * multiScatterNum * 3;

    auto *dirStore = new float[maxNum * 3], *dirStore2 = new float[maxNum *3];
    auto *wStore = new float[maxNum];
    auto **raySegStore = new RaySegment *[maxNum], **raySegStore2 = new RaySegment *[maxNum];

    for (decltype(maxNum) i = 0; i < maxNum; i++) {
        context.fillSunDir(dirStore + i*3);
        wStore[i] = 1.0f;
        raySegStore[i] = nullptr;
        raySegStore2[i] = nullptr;
    }

    std::default_random_engine randomEngine;
    std::uniform_real_distribution<double> uniformDist(0.0, 1.0);
    std::vector<RaySegment *> v;
    for (int scatterIdx = 0; scatterIdx < multiScatterNum; scatterIdx++) {
        size_t inputOffset = 0, outputOffset = 0;
        for (int crystalIdx = 0; crystalIdx < context.getCrystalNum(); crystalIdx++) {
            auto crystalCtx = context.getCrystalContext(crystalIdx);
            auto rayTracingCtx = context.getRayTracingContext(scatterIdx, crystalIdx);

            rayTracingCtx->clearRays();
            rayTracingCtx->initRays(crystalCtx, rayTracingCtx->initRayNum, 
                dirStore + inputOffset * 3, wStore + inputOffset, raySegStore + inputOffset);

            // Start loop
            float index = IceRefractiveIndex::n(context.getWavelength());
            int recursion = 0;
            while (!rayTracingCtx->isFinished() && recursion < maxRecursion) {
                // hitSurfaceHalide(index, rayTracingCtx);
                hitSurface(index, rayTracingCtx);
                rayTracingCtx->commitHitResult();
                // propagateHalide(rayTracingCtx, crystalCtx);
                propagate(rayTracingCtx, crystalCtx);
                rayTracingCtx->commitPropagateResult(crystalCtx);
                recursion++;
            }

            inputOffset += rayTracingCtx->initRayNum;

            v.clear();
            auto tmpRaySegs = rayTracingCtx->copyFinishedRaySegments(raySegStore2 + outputOffset,
                dirStore2 + outputOffset * 3, context.getMultiScatterProb());
            outputOffset += tmpRaySegs;
        }

        if (scatterIdx < multiScatterNum - 1) {
            for (int i = outputOffset - 1; i >= 0; i--) {
                auto j = static_cast<size_t>(uniformDist(randomEngine) * i);
                raySegStore[i] = raySegStore2[j];
                raySegStore[j] = raySegStore2[i];
                memcpy(dirStore + j*3, dirStore2 + i*3, sizeof(float)*3);
                memcpy(dirStore + i*3, dirStore2 + j*3, sizeof(float)*3);
                wStore[i] = raySegStore[i]->w;
            }
            context.setCrystalRayNum(scatterIdx + 1, outputOffset);
        }
    }

    delete[] dirStore;
    delete[] wStore;
    
}


float Optics::getReflectRatio(float cos_angle, float rr)
{
    float s = std::sqrt(1.0f - cos_angle * cos_angle);
    float c = std::abs(cos_angle);
    float d = std::fmax(1.0f - (rr * s) * (rr * s), 0.0f);
    float d_sqrt = std::sqrt(d);

    float Rs = (rr * c - d_sqrt) / (rr * c + d_sqrt);
    Rs *= Rs;
    float Rp = (rr * d_sqrt - c) / (rr * d_sqrt + c);
    Rp *= Rp;

    return (Rs + Rp) / 2;
}

void Optics::intersectLineFace(const float *pt, const float *dir, const float *face,
    float *p, float *t, float *alpha, float *beta)
{
    const float *face_point = face;
    float face_base[6];
    LinearAlgebra::vec3FromTo(&face[0], &face[3], &face_base[0]);
    LinearAlgebra::vec3FromTo(&face[0], &face[6], &face_base[3]);

    float a = face_base[0]*face_base[4]*face_point[2] + face_base[1]*face_base[5]*face_point[0] +
        face_base[2]*face_base[3]*face_point[1] - face_base[2]*face_base[4]*face_point[0] -
        face_base[1]*face_base[3]*face_point[2] - face_base[0]*face_base[5]*face_point[1];
    float b = pt[0]*face_base[1]*face_base[5] + pt[1]*face_base[2]*face_base[3] +
        pt[2]*face_base[0]*face_base[4] - pt[0]*face_base[2]*face_base[4] -
        pt[1]*face_base[0]*face_base[5] - pt[2]*face_base[1]*face_base[3];
    float c = dir[0]*face_base[1]*face_base[5] + dir[1]*face_base[2]*face_base[3] +
        dir[2]*face_base[0]*face_base[4] - dir[0]*face_base[2]*face_base[4] -
        dir[1]*face_base[0]*face_base[5] - dir[2]*face_base[1]*face_base[3];
    bool flag = std::abs(c) > 1e-6;
    *t = flag ? (a - b) / c : -1;

    a = dir[0]*pt[1]*face_base[5] + dir[1]*pt[2]*face_base[3] +
        dir[2]*pt[0]*face_base[4] - dir[0]*pt[2]*face_base[4] -
        dir[1]*pt[0]*face_base[5] - dir[2]*pt[1]*face_base[3];
    b = dir[0]*face_base[4]*face_point[2] + dir[1]*face_base[5]*face_point[0] +
        dir[2]*face_base[3]*face_point[1] - dir[0]*face_base[5]*face_point[1] -
        dir[1]*face_base[3]*face_point[2] - dir[2]*face_base[4]*face_point[0];
    *alpha = flag ? (a + b) / c : -1;

    a = dir[0]*pt[1]*face_base[2] + dir[1]*pt[2]*face_base[0] +
        dir[2]*pt[0]*face_base[1] - dir[0]*pt[2]*face_base[1] -
        dir[1]*pt[0]*face_base[2] - dir[2]*pt[1]*face_base[0];
    b = dir[0]*face_base[1]*face_point[2] + dir[1]*face_base[2]*face_point[0] +
        dir[2]*face_base[0]*face_point[1] - dir[0]*face_base[2]*face_point[1] -
        dir[1]*face_base[0]*face_point[2] - dir[2]*face_base[1]*face_point[0];
    *beta = flag ? -(a + b) / c : -1;

    p[0] = pt[0] + *t * dir[0];
    p[1] = pt[1] + *t * dir[1];
    p[2] = pt[2] + *t * dir[2];
}



constexpr float IceRefractiveIndex::_wl[];
constexpr float IceRefractiveIndex::_n[];

float IceRefractiveIndex::n(float waveLength)
{
    if (waveLength < _wl[0]) {
        return 1.0f;
    }

    float nn = 1.0f;
    for (decltype(sizeof(_wl)) i = 0; i < sizeof(_wl) / sizeof(float); i++) {
        if (waveLength < _wl[i]) {
            float w1 = _wl[i-1];
            float w2 = _wl[i];
            float n1 = _n[i-1];
            float n2 = _n[i];

            nn = n1 + (n2 - n1) / (w2 - w1) * (waveLength - w1);
            break;
        }
    }

    return nn;
}

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

RaySegment * RaySegmentFactory::getRaySegment(const float *pt, const float *dir, float w, int faceId)
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


}   // namespace IceHalo
