#include <limits>

#include "optics.h"
#include "linearalgebra.h"
#include "context.h"

#include "ray_hit.h"
#include "ray_prop.h"
#include "HalideBuffer.h"


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
    // return nextReflect == nullptr && nextRefract == nullptr 
    //         && w > 0 && faceId >= 0 && isFinished;
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


// void Optics::hitSurface(float n, int num, const float *dir, const float *norm,
//         float *reflect_dir, float *refract_dir, float *reflect_w)
// {
//      for (int i = 0; i < num; ++i) {
//          const float *tmp_dir = dir + i*3;
//          const float *tmp_norm = norm + i*3;
        
//          float cos_theta = LinearAlgebra::dot3(tmp_dir, tmp_norm);
//          float inc_angle = std::acos(std::abs(cos_theta));

//          bool is_inner = cos_theta > 0;
//          float tmp_n1 = is_inner ? n : 1;
//          float tmp_n2 = is_inner ? 1 : n;

//          reflect_w[i] = getReflectRatio(inc_angle, tmp_n1, tmp_n2);

//          for (int j = 0; j < 3; ++j) {
//              reflect_dir[i*3+j] = tmp_dir[j] - 2 * cos_theta * tmp_norm[j];
//          }

//          float rr = tmp_n1 / tmp_n2;
//          float d = (1.0f - rr * rr) / (cos_theta * cos_theta) + rr * rr;
//          for (int j = 0; j < 3; j++) {
//              refract_dir[i*3+j] = d <= 0.0f ? reflect_dir[i*3+j] :
//                  rr * tmp_dir[j] - (rr - std::sqrt(d)) * cos_theta * tmp_norm[j];
//          }
//      }
// }


void Optics::hitSurfaceHalide(float n, RayTracingContext *rayCtx)
{
    using namespace Halide::Runtime;

    Buffer<float> dir_buffer(rayCtx->rayDir, 3, rayCtx->currentRayNum);
    Buffer<float> norm_buffer(rayCtx->faceNorm, 3, rayCtx->currentRayNum);
    Buffer<float> reflect_dir_buffer(rayCtx->rayDir2, 3, rayCtx->currentRayNum);
    Buffer<float> refract_dir_buffer(rayCtx->rayDir3, 3, rayCtx->currentRayNum);
    Buffer<float> reflect_w_buffer(rayCtx->rayW2, rayCtx->currentRayNum);

    ray_hit(dir_buffer, norm_buffer, n, reflect_dir_buffer, refract_dir_buffer, reflect_w_buffer);
}

// void Optics::propagate(int num, const float *pt, const float *dir, int face_num, const float *faces,
//         float *new_pt, int *new_face_id)
// {
//     for (int i = 0; i < num; ++i) {
//         const float *tmp_pt = pt + i*3;
//         const float *tmp_dir = dir + i*3;

//         float min_t = std::numeric_limits<float>::max();
//         new_face_id[i] = -1;
//         for (int j = 0; j < face_num; j++) {
//             const float *tmp_face = faces + j*9;
            
//             float p[3];
//             float t, alpha, beta;

//             intersectLineFace(tmp_pt, tmp_dir, tmp_face, &p[0], &t, &alpha, &beta);
//             if (t > 1e-6 && t < min_t && alpha >= 0 && beta >= 0 && alpha + beta <= 1) {
//                 min_t = t;

//                 new_pt[i*3+0] = p[0];
//                 new_pt[i*3+1] = p[1];
//                 new_pt[i*3+2] = p[2];

//                 new_face_id[i] = j;
//             }
//         }
//     }
// }


void Optics::propagateHalide(RayTracingContext *rayCtx, CrystalContext *cryCtx)
{
    using namespace Halide::Runtime;

    auto *faces = new float[cryCtx->crystal->faceNum() * 9];
    cryCtx->crystal->copyFaceData(faces);

    Buffer<float> pt_buffer(rayCtx->rayPts, 3, rayCtx->currentRayNum);
    Buffer<float> dir_buffer(rayCtx->rayDir, 3, rayCtx->currentRayNum);
    Buffer<float> face_buffer(faces, 9, cryCtx->crystal->faceNum());
    Buffer<int> face_id_buffer(rayCtx->faceId, rayCtx->currentRayNum);
    Buffer<float> new_pt_buffer(rayCtx->rayPts2, 3, rayCtx->currentRayNum);
    Buffer<int> new_face_id_buffer(rayCtx->faceId2, rayCtx->currentRayNum);

    ray_prop(dir_buffer, pt_buffer, face_buffer, face_id_buffer, new_pt_buffer, new_face_id_buffer);

    delete[] faces;
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
                hitSurfaceHalide(index, rayTracingCtx);
                rayTracingCtx->commitHitResult();
                propagateHalide(rayTracingCtx, crystalCtx);
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


// float Optics::getReflectRatio(float inc_angle, float n1, float n2)
// {
//     float c = std::cos(inc_angle);
//     float s = std::sin(inc_angle);
//     float d = 1.0f - (n1 / n2 * s) * (n1 / n2 * s);
//     d = d <= 0.0f ? 0.0f : d;
//     float d_sqrt = std::sqrt(d);

//     float Rs = (n1 * c - n2 * d_sqrt) / (n1 * c + n2 * d_sqrt);
//     Rs *= Rs;
//     float Rp = (n1 * d_sqrt - n2 * c) / (n1 * d_sqrt + n2 * c);
//     Rp *= Rp;

//     return (Rs + Rp) / 2;
// }

// void Optics::intersectLineFace(const float *pt, const float *dir, const float *face,
//     float *p, float *t, float *alpha, float *beta)
// {
//     const float *face_point = face;
//     float face_base[6];
//     LinearAlgebra::vec3FromTo(&face[0], &face[3], &face_base[0]);
//     LinearAlgebra::vec3FromTo(&face[0], &face[6], &face_base[3]);

//     float a = face_base[0]*face_base[4]*face_point[2] + face_base[1]*face_base[5]*face_point[0] +
//         face_base[2]*face_base[3]*face_point[1] - face_base[2]*face_base[4]*face_point[0] -
//         face_base[1]*face_base[3]*face_point[2] - face_base[0]*face_base[5]*face_point[1];
//     float b = pt[0]*face_base[1]*face_base[5] + pt[1]*face_base[2]*face_base[3] +
//         pt[2]*face_base[0]*face_base[4] - pt[0]*face_base[2]*face_base[4] -
//         pt[1]*face_base[0]*face_base[5] - pt[2]*face_base[1]*face_base[3];
//     float c = dir[0]*face_base[1]*face_base[5] + dir[1]*face_base[2]*face_base[3] +
//         dir[2]*face_base[0]*face_base[4] - dir[0]*face_base[2]*face_base[4] -
//         dir[1]*face_base[0]*face_base[5] - dir[2]*face_base[1]*face_base[3];
//     *t = (a - b) / c;

//     a = dir[0]*pt[1]*face_base[5] + dir[1]*pt[2]*face_base[3] +
//         dir[2]*pt[0]*face_base[4] - dir[0]*pt[2]*face_base[4] -
//         dir[1]*pt[0]*face_base[5] - dir[2]*pt[1]*face_base[3];
//     b = dir[0]*face_base[4]*face_point[2] + dir[1]*face_base[5]*face_point[0] +
//         dir[2]*face_base[3]*face_point[1] - dir[0]*face_base[5]*face_point[1] -
//         dir[1]*face_base[3]*face_point[2] - dir[2]*face_base[4]*face_point[0];
//     *alpha = (a + b) / c;

//     a = dir[0]*pt[1]*face_base[2] + dir[1]*pt[2]*face_base[0] +
//         dir[2]*pt[0]*face_base[1] - dir[0]*pt[2]*face_base[1] -
//         dir[1]*pt[0]*face_base[2] - dir[2]*pt[1]*face_base[0];
//     b = dir[0]*face_base[1]*face_point[2] + dir[1]*face_base[2]*face_point[0] +
//         dir[2]*face_base[0]*face_point[1] - dir[0]*face_base[2]*face_point[1] -
//         dir[1]*face_base[0]*face_point[2] - dir[2]*face_base[1]*face_point[0];
//     *beta = -(a + b) / c;

//     p[0] = pt[0] + *t * dir[0];
//     p[1] = pt[1] + *t * dir[1];
//     p[2] = pt[2] + *t * dir[2];
// }



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
