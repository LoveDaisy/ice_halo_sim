#include "optics.hpp"

#ifdef USE_SIMD
#include <immintrin.h>
#endif

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <limits>
#include <memory>

#include "context/context.hpp"
#include "core/math.hpp"
#include "util/obj_pool.hpp"


namespace icehalo {

RayPathRecorder::RayPathRecorder() : hash_(0), offset_(0) {}


size_t RayPathRecorder::Hash() const noexcept {
  if (offset_ < 0) {
    size_t reverse_hash = hash_;
    endian::ByteSwap::Swap(&reverse_hash);
    auto* p = reinterpret_cast<uint8_t*>(&reverse_hash);
    for (size_t i = 0; i < sizeof(reverse_hash); i++) {
      ReverseBits(p + i);
    }
    return reverse_hash;
  } else {
    return hash_;
  }
}


void RayPathRecorder::Clear() noexcept {
  hash_ = 0;
  offset_ = 0;
}


size_t RayPathRecorder::Hash(const RayPath& ray_path) noexcept {
  RayPathRecorder recorder{};
  for (const auto& fn : ray_path) {
    recorder << fn;
  }
  return recorder.Hash();
}


void RayPathRecorder::Serialize(File& file, bool with_boi) const {
  if (with_boi) {
    file.Write(ISerializable::kDefaultBoi);
  }
  file.Write(hash_);
  file.Write(offset_);
}


void RayPathRecorder::Deserialize(File& file, endian::Endianness endianness) {
  endianness = CheckEndianness(file, endianness);
  bool need_swap = (endianness != endian::kCompileEndian);

  file.Read(&hash_);
  file.Read(&offset_);
  if (need_swap) {
    endian::ByteSwap::Swap(&hash_);
    endian::ByteSwap::Swap(&offset_);
  }
}


RayPathRecorder& RayPathRecorder::operator<<(ShortIdType id) {
  hash_ ^= LeftCircularShift(id, offset_);
  offset_ += kStep;
  offset_ %= kTotalBits;
  return *this;
}


RayPathRecorder& RayPathRecorder::operator<<(const RayPathRecorder& second) {
  hash_ ^= LeftCircularShift(second.hash_, offset_);
  offset_ += second.offset_;
  offset_ %= kTotalBits;
  return *this;
}


RayPathRecorder& RayPathRecorder::operator>>(RayPathRecorder& second) {
  second.hash_ = hash_ ^ LeftCircularShift(second.hash_, offset_);
  second.offset_ += offset_;
  second.offset_ %= kTotalBits;
  return second;
}


RayPathRecorder &RayPathRecorder::operator,(ShortIdType id) {
  return operator<<(id);
}


void RayPathRecorder::ReverseBits(uint8_t* n) {
  static uint8_t lookup[16] = {
    0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe, 0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf,
  };
  *n = (lookup[(*n) & 0xf] << 4) | lookup[(*n) >> 4];
}


size_t RayPathRecorder::RightCircularShift(size_t val, unsigned int n) {
  n %= kTotalBits;
  return (val >> n) | (val << (kTotalBits - n));
}


size_t RayPathRecorder::LeftCircularShift(size_t val, unsigned int n) {
  n %= kTotalBits;
  return (val << n) | (val >> (kTotalBits - n));
}


RaySegment::RaySegment()
    : next_reflect(nullptr), next_refract(nullptr), prev(nullptr), root_ctx(nullptr), pt(0, 0, 0), dir(0, 0, 0), w(0),
      face_id(-1), state(RaySegmentState::kOnGoing), recorder{} {}


RaySegment::RaySegment(const float* pt, const float* dir, float w, int face_id)
    : next_reflect(nullptr), next_refract(nullptr), prev(nullptr), root_ctx(nullptr), pt(pt), dir(dir), w(w),
      face_id(face_id), state(RaySegmentState::kOnGoing), recorder{} {}


void RaySegment::Serialize(File& file, bool with_boi) const {
  if (with_boi) {
    file.Write(ISerializable::kDefaultBoi);
  }

  uint32_t chunk_id = 0;
  uint32_t obj_id = 0;
  std::tie(chunk_id, obj_id) = RaySegmentPool::GetInstance()->GetObjectSerializeIndex(next_reflect);
  file.Write(chunk_id);
  file.Write(obj_id);
  std::tie(chunk_id, obj_id) = RaySegmentPool::GetInstance()->GetObjectSerializeIndex(next_refract);
  file.Write(chunk_id);
  file.Write(obj_id);
  std::tie(chunk_id, obj_id) = RaySegmentPool::GetInstance()->GetObjectSerializeIndex(prev);
  file.Write(chunk_id);
  file.Write(obj_id);
  std::tie(chunk_id, obj_id) = RayInfoPool::GetInstance()->GetObjectSerializeIndex(root_ctx);
  file.Write(chunk_id);
  file.Write(obj_id);

  file.Write(pt.val(), 3);
  file.Write(dir.val(), 3);
  file.Write(w);
  file.Write(static_cast<int16_t>(face_id));
  file.Write(static_cast<uint8_t>(state));

  recorder.Serialize(file, false);
}


void RaySegment::Deserialize(File& file, endian::Endianness endianness) {
  endianness = CheckEndianness(file, endianness);
  bool need_swap = (endianness != endian::kCompileEndian);

  uint32_t chunk_id = 0;
  uint32_t obj_id = 0;
  file.Read(&chunk_id);
  file.Read(&obj_id);
  if (need_swap) {
    endian::ByteSwap::Swap(&chunk_id);
    endian::ByteSwap::Swap(&obj_id);
  }
  next_reflect = reinterpret_cast<RaySegment*>(CombineU32AsPointer(chunk_id, obj_id));

  file.Read(&chunk_id);
  file.Read(&obj_id);
  if (need_swap) {
    endian::ByteSwap::Swap(&chunk_id);
    endian::ByteSwap::Swap(&obj_id);
  }
  next_refract = reinterpret_cast<RaySegment*>(CombineU32AsPointer(chunk_id, obj_id));

  file.Read(&chunk_id);
  file.Read(&obj_id);
  if (need_swap) {
    endian::ByteSwap::Swap(&chunk_id);
    endian::ByteSwap::Swap(&obj_id);
  }
  prev = reinterpret_cast<RaySegment*>(CombineU32AsPointer(chunk_id, obj_id));

  file.Read(&chunk_id);
  file.Read(&obj_id);
  if (need_swap) {
    endian::ByteSwap::Swap(&chunk_id);
    endian::ByteSwap::Swap(&obj_id);
  }
  root_ctx = reinterpret_cast<RayInfo*>(CombineU32AsPointer(chunk_id, obj_id));

  float vec3f_buf[3];
  file.Read(vec3f_buf, 3);
  if (need_swap) {
    endian::ByteSwap::Swap(vec3f_buf, 3);
  }
  pt.val(vec3f_buf);

  file.Read(vec3f_buf, 3);
  if (need_swap) {
    endian::ByteSwap::Swap(vec3f_buf, 3);
  }
  dir.val(vec3f_buf);

  file.Read(&w);
  if (need_swap) {
    endian::ByteSwap::Swap(&w);
  }

  int16_t face_id_data = 0;
  file.Read(&face_id_data);
  if (need_swap) {
    endian::ByteSwap::Swap(&face_id_data);
  }
  face_id = face_id_data;

  uint8_t state_data = 0;
  file.Read(&state_data);
  state = static_cast<RaySegmentState>(state_data);

  recorder.Deserialize(file, endianness);
}


namespace v3 {

float GetReflectRatio(float delta, float rr) {
  float d_sqrt = std::sqrt(delta);

  float Rs = (rr - d_sqrt) / (rr + d_sqrt);
  Rs *= Rs;
  float Rp = (1 - rr * d_sqrt) / (1 + rr * d_sqrt);
  Rp *= Rp;

  return (Rs + Rp) / 2;
}

void HitSurface_Normal(const Crystal& crystal, float n, size_t num,                          // input
                       const float_bf_t d_in, const float_bf_t w_in, const int_bf_t fid_in,  // input
                       float_bf_t d_out, float_bf_t w_out) {                                 // output
  const auto* face_norm = crystal.GetFaceNorm();

  for (size_t i = 0; i < num; i++) {
    const float* tmp_dir = d_in.Ptr(i);
    const float* tmp_norm = face_norm + fid_in[i] * 3;

    float cos_theta = Dot3(tmp_dir, tmp_norm);
    float rr = cos_theta > 0 ? n : 1.0f / n;
    float d = (1.0f - rr * rr) / (cos_theta * cos_theta) + rr * rr;

    bool is_total_reflected = d <= 0.0f;

    w_out[2 * i + 0] = GetReflectRatio(std::max(d, 0.0f), rr) * w_in[i];
    w_out[2 * i + 1] = is_total_reflected ? -1 : w_in[i] - w_out[2 * i + 0];

    float* tmp_dir_reflection = d_out.Ptr(i * 2 + 0);
    float* tmp_dir_refraction = d_out.Ptr(i * 2 + 1);
    for (int j = 0; j < 3; j++) {
      tmp_dir_reflection[j] = tmp_dir[j] - 2 * cos_theta * tmp_norm[j];  // Reflection
      tmp_dir_refraction[j] = is_total_reflected ?
                                  tmp_dir_reflection[j] :
                                  rr * tmp_dir[j] - (rr - std::sqrt(d)) * cos_theta * tmp_norm[j];  // Refraction
    }
  }
}

#if defined(USE_SIMD) && defined(__AVX__) && defined(__SSE__)
void HitSurface_Simd(const Crystal& crystal, float n,   // input
                     size_t num, const RaySeg* ray_in,  // input
                     RaySeg* ray_out) {                 // output
  const auto* face_norm = crystal.GetFaceNorm();
  __m128 zero_ = _mm_set_ps1(0.0f);
  __m128 one_ = _mm_set_ps1(1.0f);
  __m128 minus_one_ = _mm_set_ps1(-1.0f);
  __m128 half_ = _mm_set_ps1(0.5f);
  __m128 two_ = _mm_set_ps1(2.0f);
  __m128 n_ = _mm_set_ps1(n);
  __m128 n_inv_ = _mm_set_ps1(1.0f / n);

  float d[24];  // x1*4, y1*4, z1*4, x2*4, y2*4, z2*4
  float w[8];   // w1*4, w2*4

  __m128 dir_[3], dir_L_[3], dir_R_[3];  // dx_, dy_, dz_
  __m128 norm_[3];                       // nx_, ny_, nz_

  size_t i = 0;
  for (; i + 3 < num; i += 4) {
    int ids[4] = { ray_in[i].fid_, ray_in[i + 1].fid_, ray_in[i + 2].fid_, ray_in[i + 3].fid_ };

    for (int j = 0; j < 3; j++) {
      dir_[j] = _mm_set_ps(ray_in[i + 3].d_[j], ray_in[i + 2].d_[j], ray_in[i + 1].d_[j], ray_in[i].d_[j]);
      norm_[j] = _mm_set_ps(face_norm[ids[3] * 3 + j], face_norm[ids[2] * 3 + j], face_norm[ids[1] * 3 + j],
                            face_norm[ids[0] * 3 + j]);
    }
    __m128 w_ = _mm_set_ps(ray_in[i + 3].w_, ray_in[i + 2].w_, ray_in[i + 1].w_, ray_in[i].w_);

    auto c_ = _mm_add_ps(_mm_add_ps(_mm_mul_ps(dir_[0], norm_[0]), _mm_mul_ps(dir_[1], norm_[1])),
                         _mm_mul_ps(dir_[2], norm_[2]));
    auto rr_ = _mm_blendv_ps(n_inv_, n_, _mm_cmp_ps(c_, zero_, 14));  // 14: GT, greater than
    auto rr2_ = _mm_mul_ps(rr_, rr_);
    auto d_ = _mm_add_ps(_mm_div_ps(_mm_sub_ps(one_, rr2_), _mm_mul_ps(c_, c_)), rr2_);
    auto d_sqrt_ = _mm_sqrt_ps(_mm_max_ps(d_, zero_));
    auto is_total_ref_ = _mm_cmp_ps(d_, zero_, 1);  // 1: LT, less than

    auto Rs_ = _mm_div_ps(_mm_sub_ps(rr_, d_sqrt_), _mm_add_ps(rr_, d_sqrt_));
    auto Rs2_ = _mm_mul_ps(Rs_, Rs_);
    auto Rp_ = _mm_div_ps(_mm_sub_ps(one_, _mm_mul_ps(rr_, d_sqrt_)), _mm_add_ps(one_, _mm_mul_ps(rr_, d_sqrt_)));
    auto Rp2_ = _mm_mul_ps(Rp_, Rp_);
    auto R_ = _mm_mul_ps(_mm_add_ps(Rs2_, Rp2_), half_);

    auto w_L_ = _mm_mul_ps(R_, w_);
    auto w_R_ = _mm_blendv_ps(_mm_sub_ps(w_, w_L_), minus_one_, is_total_ref_);

    for (int j = 0; j < 3; j++) {
      dir_L_[j] = _mm_sub_ps(dir_[j], _mm_mul_ps(two_, _mm_mul_ps(c_, norm_[j])));
      dir_R_[j] = _mm_blendv_ps(
          _mm_sub_ps(_mm_mul_ps(rr_, dir_[j]), _mm_mul_ps(_mm_mul_ps(_mm_sub_ps(rr_, d_sqrt_), c_), norm_[j])),
          dir_L_[j], is_total_ref_);
    }

    _mm_store_ps(w, w_L_);
    _mm_store_ps(w + 4, w_R_);

    for (int j = 0; j < 3; j++) {
      _mm_store_ps(d + j * 4, dir_L_[j]);
      _mm_store_ps(d + j * 4 + 12, dir_R_[j]);
    }

    for (int j = 0; j < 4; j++) {
      ray_out[(i + j) * 2 + 0].w_ = w[j];
      ray_out[(i + j) * 2 + 1].w_ = w[j + 4];

      ray_out[(i + j) * 2 + 0].d_[0] = d[j];
      ray_out[(i + j) * 2 + 0].d_[1] = d[j + 4];
      ray_out[(i + j) * 2 + 0].d_[2] = d[j + 8];
      ray_out[(i + j) * 2 + 1].d_[0] = d[j + 12];
      ray_out[(i + j) * 2 + 1].d_[1] = d[j + 16];
      ray_out[(i + j) * 2 + 1].d_[2] = d[j + 20];
    }
  }

  HitSurface_Normal(crystal, n,           // input
                    num - i, ray_in + i,  // input
                    ray_out + i * 2);     // output
}
#endif


void HitSurface(const Crystal& crystal, float n, size_t num,                          // input
                const float_bf_t d_in, const float_bf_t w_in, const int_bf_t fid_in,  // input
                float_bf_t d_out, float_bf_t w_out) {                                 // output
#if defined(USE_SIMD) && defined(__SSE__) && defined(__AVX__)
  HitSurface_Simd(crystal, n, num, ray_in, ray_out);
#else
  HitSurface_Normal(crystal, n, num, d_in, w_in, fid_in, d_out, w_out);
#endif
}


void RayTriangleBW(const float* ray_pt, const float* ray_dir,  // input
                   int face_num, const float* face_transform,  // input
                   float* out_pt, int* out_face_id) {
  float min_t = -1.0f;
  *out_face_id = -1;
  std::memcpy(out_pt, ray_pt, 3 * sizeof(float));

  float d[3];
  float p[3];

  for (int i = 0; i < face_num; i++) {
    const float* tf = face_transform + i * 12;
    p[2] = Dot3(ray_pt, tf + 8) + tf[11];
    d[2] = Dot3(ray_dir, tf + 8);

    if (FloatEqualZero(d[2])) {
      continue;  // Parallel to this triangle
    }

    auto t = -p[2] / d[2];
    if (t < math::kFloatEps) {
      continue;
    }

    p[0] = Dot3(ray_pt, tf + 0) + tf[3];
    d[0] = Dot3(ray_dir, tf + 0);

    auto u = p[0] + t * d[0];
    if (u < -math::kFloatEps || u > 1.0f) {
      continue;  // out of this triangle
    }

    p[1] = Dot3(ray_pt, tf + 4) + tf[7];
    d[1] = Dot3(ray_dir, tf + 4);

    auto v = p[1] + t * d[1];
    if (v < -math::kFloatEps || v > 1.0f) {
      continue;  // out of this triangle
    }

    if (u + v > 1.0f) {
      continue;  // out of this triangle
    }

    if (min_t < -math::kFloatEps || (t < min_t && min_t > 0.0f)) {
      min_t = t;
      *out_face_id = i;
      for (int j = 0; j < 3; j++) {
        out_pt[j] += t * ray_dir[j];
      }
    }
  }
}


void Propagate(const Crystal& crystal, size_t num, size_t step,                      // input
               const float_bf_t d_in, const float_bf_t p_in, const float_bf_t w_in,  // input, d, p, w
               float_bf_t p_out, int_bf_t fid_out) {                                 // output, p, fid
  auto face_num = crystal.TotalFaces();
  const auto* face_transform = crystal.GetFaceCoordTf();

  // Do main work
  for (size_t i = 0; i < num; i++) {
    if (w_in[i] < 0) {  // Total reflection
      continue;
    }
    RayTriangleBW(p_in.Ptr(i / step), d_in.Ptr(i),  // input
                  face_num, face_transform,         // input
                  p_out.Ptr(i), fid_out.Ptr(i));    // output
  }
}

}  // namespace v3


void Optics::HitSurface(const Crystal* crystal, float n, size_t num,                    // input
                        const float* dir_in, const int* face_id_in, const float* w_in,  // input
                        float* dir_out, float* w_out) {                                 // output
  const auto* face_norm = crystal->GetFaceNorm();

  for (size_t i = 0; i < num; i++) {
    const float* tmp_dir = dir_in + i * 3;
    const float* tmp_norm = face_norm + face_id_in[i] * 3;

    float cos_theta = Dot3(tmp_dir, tmp_norm);
    float rr = cos_theta > 0 ? n : 1.0f / n;
    float d = (1.0f - rr * rr) / (cos_theta * cos_theta) + rr * rr;

    bool is_total_reflected = d <= 0.0f;

    w_out[2 * i + 0] = GetReflectRatio(std::max(d, 0.0f), rr) * w_in[i];
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


RayInfo::RayInfo() : first_ray_segment(nullptr), prev_ray_segment(nullptr), crystal_id(-1), main_axis{ 0, 0, 0 } {}


RayInfo::RayInfo(RaySegment* seg, int crystal_id, const float* main_axis)
    : first_ray_segment(seg), prev_ray_segment(nullptr), crystal_id(crystal_id), main_axis(main_axis) {}


void RayInfo::Serialize(File& file, bool with_boi) const {
  if (with_boi) {
    file.Write(ISerializable::kDefaultBoi);
  }

  uint32_t chunk_id = 0;
  uint32_t obj_id = 0;
  std::tie(chunk_id, obj_id) = RaySegmentPool::GetInstance()->GetObjectSerializeIndex(first_ray_segment);
  file.Write(chunk_id);
  file.Write(obj_id);

  std::tie(chunk_id, obj_id) = RaySegmentPool::GetInstance()->GetObjectSerializeIndex(prev_ray_segment);
  file.Write(chunk_id);
  file.Write(obj_id);

  file.Write(crystal_id);

  file.Write(main_axis.val(), 3);
}


void RayInfo::Deserialize(File& file, endian::Endianness endianness) {
  endianness = CheckEndianness(file, endianness);
  bool need_swap = (endianness != endian::kCompileEndian);

  uint32_t chunk_id = 0;
  uint32_t obj_id = 0;
  file.Read(&chunk_id);
  file.Read(&obj_id);
  if (need_swap) {
    endian::ByteSwap::Swap(&chunk_id);
    endian::ByteSwap::Swap(&obj_id);
  }
  first_ray_segment = reinterpret_cast<RaySegment*>(CombineU32AsPointer(chunk_id, obj_id));

  file.Read(&chunk_id);
  file.Read(&obj_id);
  if (need_swap) {
    endian::ByteSwap::Swap(&chunk_id);
    endian::ByteSwap::Swap(&obj_id);
  }
  prev_ray_segment = reinterpret_cast<RaySegment*>(CombineU32AsPointer(chunk_id, obj_id));

  file.Read(&crystal_id);
  if (need_swap) {
    endian::ByteSwap::Swap(&crystal_id);
  }

  float vec3f_buf[3];
  file.Read(vec3f_buf, 3);
  if (need_swap) {
    endian::ByteSwap::Swap(vec3f_buf, 3);
  }
  main_axis.val(vec3f_buf);
}


void Optics::Propagate(const Crystal* crystal, size_t num,                                                 // input
                       const float* pt_in, const float* dir_in, const float* w_in, const int* face_id_in,  // input
                       float* pt_out, int* face_id_out) {                                                  // output
  for (size_t i = 0; i < num; i++) {
    face_id_out[i] = -1;
  }

  auto total_faces = crystal->TotalFaces();
  const auto* face_bases = crystal->GetFaceBaseVector();
  const auto* face_vertexes = crystal->GetFaceVertex();
  const auto* face_norms = crystal->GetFaceNorm();
  for (size_t i = 0; i < num; i++) {
    if (w_in[i] < ProjectContext::kPropMinW) {
      continue;
    }
#if defined(USE_SIMD) && defined(__SSE4_1__) && defined(__AVX__)
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


float Optics::GetReflectRatio(float delta, float rr) {
  float d_sqrt = std::sqrt(delta);

  float Rs = (rr - d_sqrt) / (rr + d_sqrt);
  Rs *= Rs;
  float Rp = (1 - rr * d_sqrt) / (1 + rr * d_sqrt);
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
  float flag_in = Dot3(dir, norm_in);

  for (int i = 0; i < face_num; i++) {
    const float* curr_face_point = face_points + i * 9;
    const float* curr_face_base = face_bases + i * 6;
    const float* curr_face_norm = face_norm + i * 3;

    if (Dot3(dir, curr_face_norm) * flag_in >= 0) {
      continue;
    }

    float ff04 = curr_face_base[0] * curr_face_base[4];
    float ff05 = curr_face_base[0] * curr_face_base[5];
    float ff13 = curr_face_base[1] * curr_face_base[3];
    float ff15 = curr_face_base[1] * curr_face_base[5];
    float ff23 = curr_face_base[2] * curr_face_base[3];
    float ff24 = curr_face_base[2] * curr_face_base[4];

    float c = dir[0] * ff15 + dir[1] * ff23 + dir[2] * ff04 - dir[0] * ff24 - dir[1] * ff05 - dir[2] * ff13;
    if (FloatEqualZero(c)) {
      continue;
    }


    float a = ff15 * curr_face_point[0] + ff23 * curr_face_point[1] + ff04 * curr_face_point[2] -
              ff24 * curr_face_point[0] - ff05 * curr_face_point[1] - ff13 * curr_face_point[2];
    float b = pt[0] * ff15 + pt[1] * ff23 + pt[2] * ff04 - pt[0] * ff24 - pt[1] * ff05 - pt[2] * ff13;
    float t = (a - b) / c;
    if (t <= math::kFloatEps) {
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


#if defined(USE_SIMD) && defined(__SSE__) && defined(__AVX__)
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
  auto DN_IN = _mm_dp_ps(DIR, NORM_IN, 0x71);

  for (int i = 0; i < face_num; i++) {
    const float* curr_face_point = face_points + i * 9;
    const float* curr_face_base = face_bases + i * 6;
    const float* curr_face_norm = face_norm + i * 3;

    __m128 CURR_FACE_NORM = _mm_loadu_ps(curr_face_norm);

    auto DN_CURR = _mm_dp_ps(DIR, CURR_FACE_NORM, 0x71);
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
    auto SUB_PERM_FF = _mm_permute_ps(_mm_sub_ps(FF1, FF0), 0xD2);
    auto C = _mm_dp_ps(DIR, SUB_PERM_FF, 0x71);

    if (FloatEqualZero(C[0])) {
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
    auto A = _mm_dp_ps(SUB_PERM_FF, CURR_FACE_POINT, 0x71);
    auto B = _mm_dp_ps(SUB_PERM_FF, PT, 0x71);
    float t = (A[0] - B[0]) / C[0];
    if (t <= math::kFloatEps) {
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
#endif


constexpr float IceRefractiveIndex::kCoefAvr[];
constexpr float IceRefractiveIndex::kCoefO[];
constexpr float IceRefractiveIndex::kCoefE[];

double IceRefractiveIndex::Get(double wave_length) {
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

  double n = 1.0;
  n += kCoefAvr[0] / (1 - kCoefAvr[2] * 1e-2f / wave_length / wave_length);
  n += kCoefAvr[1] / (1 - kCoefAvr[3] * 1e2f / wave_length / wave_length);

  return std::sqrt(n);
}


}  // namespace icehalo
