#include <limits>
#include <cmath>

#include "optics.h"
#include "mymath.h"
#include "context.h"
#include "threadingpool.h"


namespace IceHalo {

RaySegment::RaySegment()
    : nextReflect(nullptr), nextRefract(nullptr), prev(nullptr),
      pt(0, 0, 0), dir(0, 0, 0), w(0),
      faceId(-1), isFinished(false) {}

RaySegment::RaySegment(const float* pt, const float* dir, float w, int faceId)
    : nextReflect(nullptr), nextRefract(nullptr),
      prev(nullptr), pt(pt), dir(dir), w(w), faceId(faceId), isFinished(false) {}

bool RaySegment::isValidEnd() {
  return w > 0 && faceId >= 0 && isFinished;
}

void RaySegment::reset() {
  nextReflect = nullptr;
  nextRefract = nullptr;
  prev = nullptr;
  pt.val(0, 0, 0);
  dir.val(0, 0, 0);
  faceId = -1;
  isFinished = false;
}


Ray::Ray(const float* pt, const float* dir, float w, int faceId) {
  firstRaySeg = RaySegmentPool::getInstance().getRaySegment(pt, dir, w, faceId);
}


Ray::Ray(RaySegment* seg) : firstRaySeg(seg) {}


Ray::~Ray() {
  firstRaySeg = nullptr;
}

size_t Ray::totalNum() {
  if (firstRaySeg == nullptr) {
    return 0;
  }

  std::vector<RaySegment*> v;
  v.push_back(firstRaySeg);

  size_t n = 0;
  while (!v.empty()) {
    RaySegment* p = v.back();
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



void Optics::hitSurface(float n, std::shared_ptr<RayTracingContext> rayCtx) {
  Pool* pool = Pool::getInstance();
  int step = rayCtx->currentRayNum / 80;
  for (int startIdx = 0; startIdx < rayCtx->currentRayNum; startIdx += step) {
    int endIdx = std::min(startIdx + step, rayCtx->currentRayNum);
    pool->addJob(std::bind(&Optics::hitSurfaceRange, n, rayCtx, startIdx, endIdx));
  }
  pool->waitFinish();
}


void Optics::hitSurfaceRange(float n, std::shared_ptr<RayTracingContext> rayCtx, int startIdx, int endIdx) {
  for (int i = startIdx; i < endIdx; i++) {
    const float* tmp_dir = rayCtx->rayDir + i * 3;
    const float* tmp_norm = rayCtx->faceNorm + i * 3;

    float cos_theta = Math::dot3(tmp_dir, tmp_norm);

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

void Optics::propagate(std::shared_ptr<RayTracingContext> rayCtx, std::shared_ptr<CrystalContext> cryCtx) {
  auto faceNum = cryCtx->getCrystal()->faceNum();
  auto* faces = new float[faceNum * 9];
  cryCtx->getCrystal()->copyFaceData(faces);

  Pool* pool = Pool::getInstance();
  int step = rayCtx->currentRayNum / 80;
  for (int startIdx = 0; startIdx < rayCtx->currentRayNum; startIdx += step) {
    int endIdx = std::min(startIdx + step, rayCtx->currentRayNum);
    pool->addJob(std::bind(&Optics::propagateRange, rayCtx, faceNum, faces, startIdx, endIdx));
  }
  pool->waitFinish();

  delete[] faces;
}


void Optics::propagateRange(std::shared_ptr<RayTracingContext> rayCtx, int faceNum, float* faces, int startIdx, int endIdx) {
  for (int i = startIdx; i < endIdx; i++) {
    const float* tmp_pt = rayCtx->rayPts + i*3;
    const float* tmp_dir = rayCtx->rayDir + i*3;

    float min_t = std::numeric_limits<float>::max();

    int currFaceId = rayCtx->faceId[i];
    rayCtx->faceId2[i] = -1;
    for (int j = 0; j < faceNum; j++) {
      if (j == currFaceId) {
        continue;
      }
      const float* tmp_face = faces + j*9;

      float p[3];
      float t = -1, alpha = -1, beta = -1;

      intersectLineTriangle(tmp_pt, tmp_dir, tmp_face, &p[0], &t, &alpha, &beta);
      if (t > Math::kFloatEps && t < min_t && alpha >= 0 && beta >= 0 && alpha + beta <= 1) {
        min_t = t;
        std::memcpy(rayCtx->rayPts2 + i*3, p, sizeof(float) * 3);
        rayCtx->faceId2[i] = j;
      }
    }
  }
}


void Optics::traceRays(std::shared_ptr<SimulationContext> context) {
  RaySegmentPool::getInstance().clear();

  auto totalRays = context->getTotalInitRays();
  auto maxRecursion = context->getMaxRecursionNum();
  auto multiScatterNum = context->getMultiScatterNum();
  auto maxNum = totalRays * maxRecursion * multiScatterNum * 3;

  auto* dirStore = new float[maxNum * 3];
  auto* dirStore2 = new float[maxNum * 3];
  auto* wStore = new float[maxNum];
  auto** raySegStore = new RaySegment*[maxNum];
  auto** raySegStore2 = new RaySegment*[maxNum];

  context->fillSunDir(dirStore, totalRays);

  for (decltype(maxNum) i = 0; i < maxNum; i++) {
    wStore[i] = 1.0f;
    raySegStore[i] = nullptr;
    raySegStore2[i] = nullptr;
  }

  std::default_random_engine randomEngine;
  std::uniform_real_distribution<double> uniformDist(0.0, 1.0);
  for (int scatterIdx = 0; scatterIdx < multiScatterNum; scatterIdx++) {
    size_t inputOffset = 0, outputOffset = 0;
    for (int crystalIdx = 0; crystalIdx < context->getCrystalNum(); crystalIdx++) {
      auto crystalCtx = context->getCrystalContext(crystalIdx);
      auto rayTracingCtx = context->getRayTracingContext(scatterIdx, crystalIdx);

      rayTracingCtx->initRays(crystalCtx, rayTracingCtx->initRayNum,
        dirStore + inputOffset * 3, wStore + inputOffset, raySegStore + inputOffset);

      // Start loop
      float index = IceRefractiveIndex::n(context->getCurrentWavelength());
      for (int recursion = 0; !rayTracingCtx->isFinished() && recursion < maxRecursion; recursion++) {
        hitSurface(index, rayTracingCtx);
        rayTracingCtx->commitHitResult();
        propagate(rayTracingCtx, crystalCtx);
        rayTracingCtx->commitPropagateResult(crystalCtx);
      }

      inputOffset += rayTracingCtx->initRayNum;

      auto tmpRaySegs = rayTracingCtx->copyFinishedRaySegments(raySegStore2 + outputOffset,
        dirStore2 + outputOffset * 3, context->getMultiScatterProb());
      outputOffset += tmpRaySegs;
    }

    if (scatterIdx < multiScatterNum - 1) {
      for (auto i = static_cast<int>(outputOffset - 1); i >= 0; i--) {
        auto j = static_cast<size_t>(uniformDist(randomEngine) * i);
        raySegStore[i] = raySegStore2[j];
        raySegStore[j] = raySegStore2[i];
        std::memcpy(dirStore + j*3, dirStore2 + i*3, sizeof(float)*3);
        std::memcpy(dirStore + i*3, dirStore2 + j*3, sizeof(float)*3);
        wStore[i] = raySegStore[i]->w;
      }
      context->setCrystalRayNum(scatterIdx + 1, outputOffset);
    }
  }

  delete[] dirStore;
  delete[] dirStore2;
  delete[] wStore;
  delete[] raySegStore;
  delete[] raySegStore2;

}


float Optics::getReflectRatio(float cos_angle, float rr) {
  float s = std::sqrt(1.0f - cos_angle * cos_angle);
  float c = std::abs(cos_angle);
  float d = std::max(1.0f - (rr * s) * (rr * s), 0.0f);
  float d_sqrt = std::sqrt(d);

  float Rs = (rr * c - d_sqrt) / (rr * c + d_sqrt);
  Rs *= Rs;
  float Rp = (rr * d_sqrt - c) / (rr * d_sqrt + c);
  Rp *= Rp;

  return (Rs + Rp) / 2;
}

void Optics::intersectLineTriangle(const float* pt, const float* dir, const float* face,
                                   float* p, float* t, float* alpha, float* beta) {
  const float* face_point = face;
  float face_base[6];
  Math::vec3FromTo(&face[0], &face[3], &face_base[0]);
  Math::vec3FromTo(&face[0], &face[6], &face_base[3]);

  float c = dir[0]*face_base[1]*face_base[5] + dir[1]*face_base[2]*face_base[3] +
    dir[2]*face_base[0]*face_base[4] - dir[0]*face_base[2]*face_base[4] -
    dir[1]*face_base[0]*face_base[5] - dir[2]*face_base[1]*face_base[3];
  if (Math::floatEqual(c, 0)) {
    *t = -1;
    return;
  }

  float a = face_base[0]*face_base[4]*face_point[2] + face_base[1]*face_base[5]*face_point[0] +
    face_base[2]*face_base[3]*face_point[1] - face_base[2]*face_base[4]*face_point[0] -
    face_base[1]*face_base[3]*face_point[2] - face_base[0]*face_base[5]*face_point[1];
  float b = pt[0]*face_base[1]*face_base[5] + pt[1]*face_base[2]*face_base[3] +
    pt[2]*face_base[0]*face_base[4] - pt[0]*face_base[2]*face_base[4] -
    pt[1]*face_base[0]*face_base[5] - pt[2]*face_base[1]*face_base[3];
  *t = (a - b) / c;
  if (*t < 0) {
    return;
  }

  a = dir[0]*pt[1]*face_base[5] + dir[1]*pt[2]*face_base[3] +
    dir[2]*pt[0]*face_base[4] - dir[0]*pt[2]*face_base[4] -
    dir[1]*pt[0]*face_base[5] - dir[2]*pt[1]*face_base[3];
  b = dir[0]*face_base[4]*face_point[2] + dir[1]*face_base[5]*face_point[0] +
    dir[2]*face_base[3]*face_point[1] - dir[0]*face_base[5]*face_point[1] -
    dir[1]*face_base[3]*face_point[2] - dir[2]*face_base[4]*face_point[0];
  *alpha = (a + b) / c;

  a = dir[0]*pt[1]*face_base[2] + dir[1]*pt[2]*face_base[0] +
    dir[2]*pt[0]*face_base[1] - dir[0]*pt[2]*face_base[1] -
    dir[1]*pt[0]*face_base[2] - dir[2]*pt[1]*face_base[0];
  b = dir[0]*face_base[1]*face_point[2] + dir[1]*face_base[2]*face_point[0] +
    dir[2]*face_base[0]*face_point[1] - dir[0]*face_base[2]*face_point[1] -
    dir[1]*face_base[0]*face_point[2] - dir[2]*face_base[1]*face_point[0];
  *beta = -(a + b) / c;

  p[0] = pt[0] + *t * dir[0];
  p[1] = pt[1] + *t * dir[1];
  p[2] = pt[2] + *t * dir[2];
}



constexpr float IceRefractiveIndex::_wl[];
constexpr float IceRefractiveIndex::_n[];

float IceRefractiveIndex::n(float waveLength) {
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

RaySegmentPool::RaySegmentPool() {
  auto* raySegPool = new RaySegment[chunkSize];
  segments.push_back(raySegPool);
  nextUnusedId = 0;
  currentChunkId = 0;
}

RaySegmentPool::~RaySegmentPool() {
  for (auto seg : segments) {
    delete[] seg;
  }
  segments.clear();
}

RaySegmentPool& RaySegmentPool::getInstance() {
  static RaySegmentPool instance;
  return instance;
}

RaySegment* RaySegmentPool::getRaySegment(const float* pt, const float* dir, float w, int faceId) {
  RaySegment* seg;
  RaySegment* currentChunk;

  if (nextUnusedId >= chunkSize) {
    auto segSize = segments.size();
    if (currentChunkId >= segSize - 1) {
      auto* raySegPool = new RaySegment[chunkSize];
      segments.push_back(raySegPool);
      currentChunkId.store(segSize);
    } else {
      currentChunkId++;
    }
    nextUnusedId = 0;
  }
  currentChunk = segments[currentChunkId];

  seg = currentChunk + nextUnusedId;
  nextUnusedId++;
  seg->reset();

  seg->pt.val(pt);
  seg->dir.val(dir);
  seg->w = w;
  seg->faceId = faceId;

  return seg;
}

void RaySegmentPool::clear() {
  nextUnusedId = 0;
  currentChunkId = 0;
}


}   // namespace IceHalo
