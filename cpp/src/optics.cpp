#include "optics.h"
#include "mymath.h"
#include "context.h"
#include "threadingpool.h"

#include <limits>
#include <cmath>
#include <algorithm>


namespace IceHalo {

RaySegment::RaySegment()
    : nextReflect(nullptr), nextRefract(nullptr), prev(nullptr), root(nullptr),
      pt(0, 0, 0), dir(0, 0, 0), w(0),
      faceId(-1), isFinished(false) {}


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


Ray::Ray(RaySegment *seg, const float main_axis_rot[3])
    : firstRaySeg(seg), main_axis_rot(main_axis_rot) {}


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


void Optics::HitSurface(const IceHalo::CrystalPtr& crystal, float n, size_t num,
                        const float* dir_in, const int* face_id_in, const float* w_in,
                        float* dir_out, float* w_out) {
  auto face_num = crystal->totalFaces();
  auto face_norm = new float[face_num * 3];
  crystal->copyNormData(face_norm);

  for (decltype(num) i = 0; i < num; i++) {
    const float* tmp_dir = dir_in + i * 3;
    const float* tmp_norm = face_norm + face_id_in[i] * 3;

    float cos_theta = Math::dot3(tmp_dir, tmp_norm);
    float rr = cos_theta > 0 ? n : 1.0f / n;
    float d = (1.0f - rr * rr) / (cos_theta * cos_theta) + rr * rr;

    w_out[2 * i + 0] = getReflectRatio(cos_theta, rr) * w_in[i];
    w_out[2 * i + 1] = w_in[i] - w_out[2 * i + 0];

    float* tmp_dir_reflection = dir_out + (i * 2 + 0) * 3;
    float* tmp_dir_refraction = dir_out + (i * 2 + 1) * 3;
    for (int j = 0; j < 3; j++) {
      tmp_dir_reflection[j] = tmp_dir[j] - 2 * cos_theta * tmp_norm[j];  // Reflection
      tmp_dir_refraction[j] = d <= 0.0f ? tmp_dir_reflection[j] :
                              rr * tmp_dir[j] - (rr - std::sqrt(d)) * cos_theta * tmp_norm[j];  // Refraction
    }
  }

  delete[] face_norm;
}


void Optics::Propagate(const IceHalo::CrystalPtr& crystal, size_t num,
                       const float* pt_in, const float* dir_in, const float* w_in,
                       float* pt_out, int* face_id_out) {
  auto face_num = crystal->totalFaces();
  auto faces = new float[face_num * 9];
  auto face_bases = new float[face_num * 6];

  crystal->copyFaceData(faces);
  for (int i = 0; i < face_num; i++) {
    Math::vec3FromTo(faces + i * 9 + 0, faces + i * 9 + 3, face_bases + i * 6 + 0);
    Math::vec3FromTo(faces + i * 9 + 0, faces + i * 9 + 6, face_bases + i * 6 + 3);
  }

  for (decltype(num) i = 0; i < num; i++) {
    face_id_out[i] = -1;
  }

  for (decltype(num) i = 0; i < num; i++) {
    if (w_in[i] < SimulationContext::kPropMinW) {
      continue;
    }
    intersectLineWithTriangles(pt_in + i / 2 * 3, dir_in + i * 3,
                               face_bases, faces, face_num,
                               pt_out + i * 3, face_id_out + i);
  }

  delete[] faces;
  delete[] face_bases;
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


void Optics::intersectLineWithTriangles(const float* pt, const float* dir,
                                        const float* faceBases, const float* facePoints, int faceNum,
                                        float* p, int* idx) {
  float min_t = std::numeric_limits<float>::max();

  for (int i = 0; i < faceNum; i++) {
    const float* curr_face_point = facePoints + i * 9;
    const float* curr_face_base = faceBases + i * 6;

    float ff04 = curr_face_base[0] * curr_face_base[4];
    float ff05 = curr_face_base[0] * curr_face_base[5];
    float ff13 = curr_face_base[1] * curr_face_base[3];
    float ff15 = curr_face_base[1] * curr_face_base[5];
    float ff23 = curr_face_base[2] * curr_face_base[3];
    float ff24 = curr_face_base[2] * curr_face_base[4];

    float c = dir[0] * ff15 + dir[1] * ff23 + dir[2] * ff04 -
              dir[0] * ff24 - dir[1] * ff05 - dir[2] * ff13;
    if (Math::floatEqual(c, 0)) {
      continue;
    }


    float a = ff15 * curr_face_point[0] + ff23 * curr_face_point[1] + ff04 * curr_face_point[2] -
              ff24 * curr_face_point[0] - ff05 * curr_face_point[1] - ff13 * curr_face_point[2];
    float b = pt[0] * ff15 + pt[1] * ff23 + pt[2] * ff04 -
              pt[0] * ff24 - pt[1] * ff05 - pt[2] * ff13;
    float t = (a - b) / c;
    if (t <= Math::kFloatEps) {
      continue;
    }

    float dp01 = dir[0] * pt[1];
    float dp02 = dir[0] * pt[2];
    float dp10 = dir[1] * pt[0];
    float dp12 = dir[1] * pt[2];
    float dp20 = dir[2] * pt[0];
    float dp21 = dir[2] * pt[1];

    a = dp12 * curr_face_base[3] + dp20 * curr_face_base[4] + dp01 * curr_face_base[5] -
        dp21 * curr_face_base[3] - dp02 * curr_face_base[4] - dp10 * curr_face_base[5];
    b = dir[0] * curr_face_base[4] * curr_face_point[2] + dir[1] * curr_face_base[5] * curr_face_point[0] +
        dir[2] * curr_face_base[3] * curr_face_point[1] - dir[0] * curr_face_base[5] * curr_face_point[1] -
        dir[1] * curr_face_base[3] * curr_face_point[2] - dir[2] * curr_face_base[4] * curr_face_point[0];
    float alpha = (a + b) / c;
    if (alpha < 0 || alpha > 1) {
      continue;
    }

    a = dp12 * curr_face_base[0] + dp20 * curr_face_base[1] + dp01 * curr_face_base[2] -
        dp21 * curr_face_base[0] - dp02 * curr_face_base[1] - dp10 * curr_face_base[2];
    b = dir[0] * curr_face_base[1] * curr_face_point[2] + dir[1] * curr_face_base[2] * curr_face_point[0] +
        dir[2] * curr_face_base[0] * curr_face_point[1] - dir[0] * curr_face_base[2] * curr_face_point[1] -
        dir[1] * curr_face_base[0] * curr_face_point[2] - dir[2] * curr_face_base[1] * curr_face_point[0];
    float beta = -(a + b) / c;

    if (t < min_t && alpha >= 0 && beta >= 0 && alpha + beta <= 1) {
      min_t = t;
      p[0] = pt[0] + t * dir[0];
      p[1] = pt[1] + t * dir[1];
      p[2] = pt[2] + t * dir[2];
      *idx = i;
    }
  }

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
  auto* raySegPool = new RaySegment[kChunkSize];
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

  if (nextUnusedId >= kChunkSize) {
    auto segSize = segments.size();
    if (currentChunkId >= segSize - 1) {
      auto* raySegPool = new RaySegment[kChunkSize];
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
