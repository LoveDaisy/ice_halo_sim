#include "optics.h"

#include <immintrin.h>
#include <smmintrin.h>
#include <xmmintrin.h>

#include <algorithm>
#include <cmath>
#include <limits>

#include "context.h"
#include "mymath.h"
#include "threadingpool.h"


namespace IceHalo {

RaySegment::RaySegment()
    : next_reflect(nullptr), next_refract(nullptr), prev(nullptr), root_ctx(nullptr), pt(0, 0, 0), dir(0, 0, 0), w(0),
      face_id(-1), is_finished(false) {}


void RaySegment::ResetWith(const float* pt, const float* dir, float w, int face_id) {
  next_reflect = nullptr;
  next_refract = nullptr;
  prev = nullptr;
  root_ctx = nullptr;

  this->pt.val(pt);
  this->dir.val(dir);
  this->w = w;
  this->face_id = face_id;

  is_finished = false;
}


void Optics::HitSurface(const IceHalo::CrystalPtr& crystal, float n, size_t num,        // input
                        const float* dir_in, const int* face_id_in, const float* w_in,  // input
                        float* dir_out, float* w_out) {                                 // output
  auto face_norm = crystal->GetFaceNorm();

  for (decltype(num) i = 0; i < num; i++) {
    const float* tmp_dir = dir_in + i * 3;
    const float* tmp_norm = face_norm + face_id_in[i] * 3;

    float cos_theta = Math::Dot3(tmp_dir, tmp_norm);
    float rr = cos_theta > 0 ? n : 1.0f / n;
    float d = (1.0f - rr * rr) / (cos_theta * cos_theta) + rr * rr;

    bool is_total_reflected = d <= 0.0f;

    w_out[2 * i + 0] = GetReflectRatio(cos_theta, rr) * w_in[i];
    w_out[2 * i + 1] = is_total_reflected ? -1 : w_in[i] - w_out[2 * i + 0];

    float* tmp_dir_reflection = dir_out + (i * 2 + 0) * 3;
    float* tmp_dir_refraction = dir_out + (i * 2 + 1) * 3;
    for (int j = 0; j < 3; j++) {
      tmp_dir_reflection[j] = tmp_dir[j] - 2 * cos_theta * tmp_norm[j];  // Reflection
      tmp_dir_refraction[j] = is_total_reflected ?
                                  tmp_dir_reflection[j] :
                                  rr * tmp_dir[j] - (rr - std::sqrt(d)) * cos_theta * tmp_norm[j];  // Refraction
    }
  }
}


void Optics::Propagate(const IceHalo::CrystalPtr& crystal, size_t num,                                     // input
                       const float* pt_in, const float* dir_in, const float* w_in, const int* face_id_in,  // input
                       float* pt_out, int* face_id_out) {                                                  // output
  for (decltype(num) i = 0; i < num; i++) {
    face_id_out[i] = -1;
  }

  auto total_faces = crystal->TotalFaces();
  auto face_bases = crystal->GetFaceBaseVector();
  auto face_vertexes = crystal->GetFaceVertex();
  auto face_norms = crystal->GetFaceNorm();
  for (decltype(num) i = 0; i < num; i++) {
    if (w_in[i] < SimulationContext::kPropMinW) {
      continue;
    }
#if defined(__SSE4_1__) && defined(__AVX__)
    IntersectLineWithTrianglesSimd(pt_in + i / 2 * 3, dir_in + i * 3, face_id_in[i / 2], total_faces,  //
                                   face_bases, face_vertexes, face_norms,                              //
                                   pt_out + i * 3, face_id_out + i);                                   // output
#else
    IntersectLineWithTriangles(pt_in + i / 2 * 3, dir_in + i * 3, face_id_in[i / 2], total_faces,  //
                               face_bases, face_vertexes, face_norms,                              //
                               pt_out + i * 3, face_id_out + i);                                   // output
#endif
  }
}


float Optics::GetReflectRatio(float cos_angle, float rr) {
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


void Optics::IntersectLineWithTriangles(const float* pt, const float* dir,  // input
                                        int face_id, int face_num,          // input
                                        const float* face_bases,            // input (pt1 - pt2, pt1 - pt3)
                                        const float* face_points,           // input (pt1, pt2, pt3)
                                        const float* face_norm,             // input
                                        float* p, int* idx) {               // output
  float min_t = std::numeric_limits<float>::max();
  const float* norm_in = face_norm + face_id * 3;
  float flag_in = Math::Dot3(dir, norm_in);

  for (int i = 0; i < face_num; i++) {
    const float* curr_face_point = face_points + i * 9;
    const float* curr_face_base = face_bases + i * 6;
    const float* curr_face_norm = face_norm + i * 3;

    if (Math::Dot3(dir, curr_face_norm) * flag_in >= 0) {
      continue;
    }

    float ff04 = curr_face_base[0] * curr_face_base[4];
    float ff05 = curr_face_base[0] * curr_face_base[5];
    float ff13 = curr_face_base[1] * curr_face_base[3];
    float ff15 = curr_face_base[1] * curr_face_base[5];
    float ff23 = curr_face_base[2] * curr_face_base[3];
    float ff24 = curr_face_base[2] * curr_face_base[4];

    float c = dir[0] * ff15 + dir[1] * ff23 + dir[2] * ff04 - dir[0] * ff24 - dir[1] * ff05 - dir[2] * ff13;
    if (Math::FloatEqualZero(c)) {
      continue;
    }


    float a = ff15 * curr_face_point[0] + ff23 * curr_face_point[1] + ff04 * curr_face_point[2] -
              ff24 * curr_face_point[0] - ff05 * curr_face_point[1] - ff13 * curr_face_point[2];
    float b = pt[0] * ff15 + pt[1] * ff23 + pt[2] * ff04 - pt[0] * ff24 - pt[1] * ff05 - pt[2] * ff13;
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

    a = dp12 * curr_face_base[3] + dp20 * curr_face_base[4] + dp01 * curr_face_base[5] - dp21 * curr_face_base[3] -
        dp02 * curr_face_base[4] - dp10 * curr_face_base[5];
    b = dir[0] * curr_face_base[4] * curr_face_point[2] + dir[1] * curr_face_base[5] * curr_face_point[0] +
        dir[2] * curr_face_base[3] * curr_face_point[1] - dir[0] * curr_face_base[5] * curr_face_point[1] -
        dir[1] * curr_face_base[3] * curr_face_point[2] - dir[2] * curr_face_base[4] * curr_face_point[0];
    float alpha = (a + b) / c;
    if (alpha < 0 || alpha > 1) {
      continue;
    }

    a = dp12 * curr_face_base[0] + dp20 * curr_face_base[1] + dp01 * curr_face_base[2] - dp21 * curr_face_base[0] -
        dp02 * curr_face_base[1] - dp10 * curr_face_base[2];
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


void Optics::IntersectLineWithTrianglesSimd(const float* pt, const float* dir,  // input
                                            int face_id, int face_num,          // input
                                            const float* face_bases,            // input
                                            const float* face_points,           // input
                                            const float* face_norm,             // input
                                            float* p, int* idx) {               // output
  float min_t = std::numeric_limits<float>::max();
  const float* norm_in = face_norm + face_id * 3;

  __m128 DIR = _mm_loadu_ps(dir);
  __m128 PT = _mm_loadu_ps(pt);
  __m128 NORM_IN = _mm_loadu_ps(norm_in);
  __m128 DN_IN = _mm_dp_ps(DIR, NORM_IN, 0x71);

  for (int i = 0; i < face_num; i++) {
    const float* curr_face_point = face_points + i * 9;
    const float* curr_face_base = face_bases + i * 6;
    const float* curr_face_norm = face_norm + i * 3;

    __m128 CURR_FACE_NORM = _mm_loadu_ps(curr_face_norm);

    __m128 DN_CURR = _mm_dp_ps(DIR, CURR_FACE_NORM, 0x71);
    __m128 FLAG = _mm_mul_ps(DN_IN, DN_CURR);

    if (FLAG[0] >= 0) {
      continue;
    }

    /* Here use permute to get the production of curr_face_base.
     * The original plain code is:
     *
     *   float ff04 = curr_face_base[0] * curr_face_base[4];
     *   float ff05 = curr_face_base[0] * curr_face_base[5];
     *   float ff13 = curr_face_base[1] * curr_face_base[3];
     *   float ff15 = curr_face_base[1] * curr_face_base[5];
     *   float ff23 = curr_face_base[2] * curr_face_base[3];
     *   float ff24 = curr_face_base[2] * curr_face_base[4];
     *
     *        |<-- high -- low -->|
     * index: 5   4   3   2   1   0
     * index: 1   0   2   4   3   5
     */
    __m128 CURR_FB0 = _mm_loadu_ps(curr_face_base + 0);
    __m128 CURR_FB1 = _mm_loadu_ps(curr_face_base + 3);
    __m128 FF0 = _mm_mul_ps(CURR_FB0, _mm_permute_ps(CURR_FB1, 0xD2));
    __m128 FF1 = _mm_mul_ps(CURR_FB1, _mm_permute_ps(CURR_FB0, 0xD2));

    /*
     * The original plain code is:
     *
     *   float c = dir[0] * ff15 + dir[1] * ff23 + dir[2] * ff04 -
     *             dir[0] * ff24 - dir[1] * ff05 - dir[2] * ff13;
     *
     *      |<-- high ------ low -->|
     * dir: 2   1   0   -   2   1   0
     * ff1: 1   0   2  ff0: 1   0   2
     */
    __m128 SUB_PERM_FF = _mm_permute_ps(_mm_sub_ps(FF1, FF0), 0xD2);
    __m128 C = _mm_dp_ps(DIR, SUB_PERM_FF, 0x71);

    if (Math::FloatEqualZero(C[0])) {
      continue;
    }


    /*
     *   float a = ff15 * curr_face_point[0] + ff23 * curr_face_point[1] + ff04 * curr_face_point[2] -
     *             ff24 * curr_face_point[0] - ff05 * curr_face_point[1] - ff13 * curr_face_point[2];
     *   float b = pt[0] * ff15 + pt[1] * ff23 + pt[2] * ff04 -
     *             pt[0] * ff24 - pt[1] * ff05 - pt[2] * ff13;
     *
     *        |<-- high ------ low -->|
     * fp/pt: 2   1   0   -   2   1   0
     * ff1  : 1   0   2  ff0: 1   0   2
     */
    __m128 CURR_FACE_POINT = _mm_loadu_ps(curr_face_point);
    __m128 A = _mm_dp_ps(SUB_PERM_FF, CURR_FACE_POINT, 0x71);
    __m128 B = _mm_dp_ps(SUB_PERM_FF, PT, 0x71);
    float t = (A[0] - B[0]) / C[0];
    if (t <= Math::kFloatEps) {
      continue;
    }

    /*
     *   float dp01 = dir[0] * pt[1];
     *   float dp02 = dir[0] * pt[2];
     *   float dp10 = dir[1] * pt[0];
     *   float dp12 = dir[1] * pt[2];
     *   float dp20 = dir[2] * pt[0];
     *   float dp21 = dir[2] * pt[1];
     *
     *      |<-- high -|-- low -->|
     * dir: 2   1   0  |  2   1   0
     * pt : 1   0   2  |  0   2   1
     */
    __m128 DP0 = _mm_mul_ps(DIR, _mm_permute_ps(PT, 0xC9));
    __m128 DP1 = _mm_mul_ps(DIR, _mm_permute_ps(PT, 0xD2));
    __m128 SUB_PERM_DP = _mm_sub_ps(_mm_permute_ps(DP0, 0xC9), _mm_permute_ps(DP1, 0xD2));

    /*
     *   a = dp12 * curr_face_base[3] + dp20 * curr_face_base[4] + dp01 * curr_face_base[5] -
     *       dp21 * curr_face_base[3] - dp02 * curr_face_base[4] - dp10 * curr_face_base[5];
     *   b = dir[0] * curr_face_base[4] * curr_face_point[2] + dir[1] * curr_face_base[5] * curr_face_point[0] +
     *       dir[2] * curr_face_base[3] * curr_face_point[1] - dir[0] * curr_face_base[5] * curr_face_point[1] -
     *       dir[1] * curr_face_base[3] * curr_face_point[2] - dir[2] * curr_face_base[4] * curr_face_point[0];
     *
     * a:
     *      |<-- high ---|--- low -->|
     * fb1: 5   4   3  - |   5   4   3
     * dp0: 0   2   1  dp1:  1   0   2
     *
     * b:
     *      |<-- high --|--- low -->|
     * dir: 2   1   0   |   2   1   0
     * fb1: 3   5   4   |   4   3   5
     * fp : 1   0   2   |   0   2   1
     */
    A = _mm_dp_ps(CURR_FB1, SUB_PERM_DP, 0x71);
    B = _mm_dp_ps(DIR,
                  _mm_sub_ps(_mm_mul_ps(_mm_permute_ps(CURR_FB1, 0xC9), _mm_permute_ps(CURR_FACE_POINT, 0xD2)),
                             _mm_mul_ps(_mm_permute_ps(CURR_FB1, 0xD2), _mm_permute_ps(CURR_FACE_POINT, 0xC9))),
                  0x71);
    float alpha = (A[0] + B[0]) / C[0];
    if (alpha < 0 || alpha > 1) {
      continue;
    }

    /*
     *   a = dp12 * curr_face_base[0] + dp20 * curr_face_base[1] + dp01 * curr_face_base[2] -
     *       dp21 * curr_face_base[0] - dp02 * curr_face_base[1] - dp10 * curr_face_base[2];
     *   b = dir[0] * curr_face_base[1] * curr_face_point[2] + dir[1] * curr_face_base[2] * curr_face_point[0] +
     *       dir[2] * curr_face_base[0] * curr_face_point[1] - dir[0] * curr_face_base[2] * curr_face_point[1] -
     *       dir[1] * curr_face_base[0] * curr_face_point[2] - dir[2] * curr_face_base[1] * curr_face_point[0];
     *
     * a:
     *      |<-- high ---|--- low -->|
     * fb0: 2   1   0    |   2   1   0
     * dp1: 0   2   1  dp0:  1   0   2
     *
     * b:
     *      |<-- high --|--- low -->|
     * dir: 2   1   0   |   2   1   0
     * fb0: 0   2   1   |   1   0   2
     * fp : 1   0   2   |   0   2   1
     */
    A = _mm_dp_ps(CURR_FB0, SUB_PERM_DP, 0x71);
    B = _mm_dp_ps(DIR,
                  _mm_sub_ps(_mm_mul_ps(_mm_permute_ps(CURR_FB0, 0xC9), _mm_permute_ps(CURR_FACE_POINT, 0xD2)),
                             _mm_mul_ps(_mm_permute_ps(CURR_FB0, 0xD2), _mm_permute_ps(CURR_FACE_POINT, 0xC9))),
                  0x71);
    float beta = -(A[0] + B[0]) / C[0];

    if (t < min_t && alpha >= 0 && beta >= 0 && alpha + beta <= 1) {
      min_t = t;
      p[0] = pt[0] + t * dir[0];
      p[1] = pt[1] + t * dir[1];
      p[2] = pt[2] + t * dir[2];
      *idx = i;
    }
  }
}


constexpr float IceRefractiveIndex::kCoefAvr[];
constexpr float IceRefractiveIndex::kCoefO[];
constexpr float IceRefractiveIndex::kCoefE[];

float IceRefractiveIndex::Get(float wave_length) {
  /* Shellmeier's equation:
   *
   *   n^2 = 1 + B1 * lambda^2 / (lambda^2 - C1)
   *           + B2 * lambda^2 / (lambda^2 - C2)
   *   lambda in micrometer, B * 1e-2, C * 1e2
   */
  if (wave_length < kMinWaveLength || wave_length > kMaxWaveLength) {
    return 1.0f;
  }

  wave_length /= 1e3;

  float n = 1.0f;
  n += kCoefAvr[0] / (1 - kCoefAvr[2] * 1e-2f / wave_length / wave_length);
  n += kCoefAvr[1] / (1 - kCoefAvr[3] * 1e2f / wave_length / wave_length);

  return std::sqrt(n);
}


RaySegmentPool* RaySegmentPool::instance_ = nullptr;

RaySegmentPool::RaySegmentPool() : current_chunk_id_(0), next_unused_id_(0) {
  auto* raySegPool = new RaySegment[kChunkSize];
  segments_.push_back(raySegPool);
}

RaySegmentPool::~RaySegmentPool() {
  for (auto seg : segments_) {
    delete[] seg;
  }
  segments_.clear();
}

RaySegmentPool* RaySegmentPool::GetInstance() {
  if (!instance_) {
    instance_ = new RaySegmentPool();
  }
  return instance_;
}

RaySegment* RaySegmentPool::GetRaySegment(const float* pt, const float* dir, float w, int faceId) {
  RaySegment* seg;

  auto id = next_unused_id_.fetch_add(1);
  if (id >= kChunkSize) {
    std::unique_lock<std::mutex> lock(id_mutex_);
    id = next_unused_id_;
    if (id > kChunkSize) {
      auto seg_size = segments_.size();
      if (current_chunk_id_ >= seg_size - 1) {
        auto* ray_seg_pool = new RaySegment[kChunkSize];
        segments_.push_back(ray_seg_pool);
        current_chunk_id_ = seg_size;
      } else {
        current_chunk_id_++;
      }
      id = 0;
      next_unused_id_ = 0;
    }
  }

  seg = segments_[current_chunk_id_] + id;
  seg->ResetWith(pt, dir, w, faceId);

  return seg;
}

void RaySegmentPool::Clear() {
  next_unused_id_ = 0;
  current_chunk_id_ = 0;
}


}  // namespace IceHalo
