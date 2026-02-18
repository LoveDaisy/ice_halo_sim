#include "optics.hpp"

#ifdef USE_SIMD
#include <immintrin.h>
#endif

#include <algorithm>
#include <cmath>
#include <cstring>

#include "core/math.hpp"


namespace lumice {

float GetReflectRatio(float delta, float rr) {
  float d_sqrt = std::sqrt(delta);

  float Rs = (rr - d_sqrt) / (rr + d_sqrt);  // NOLINT(readability-identifier-naming) Fresnel notation
  Rs *= Rs;
  float Rp = (1 - rr * d_sqrt) / (1 + rr * d_sqrt);  // NOLINT(readability-identifier-naming) Fresnel notation
  Rp *= Rp;

  return (Rs + Rp) / 2;
}

// NOLINTNEXTLINE(readability-function-size,readability-identifier-naming)
void HitSurface_Normal(const Crystal& crystal, float n, size_t num,                          // input
                       const float_bf_t d_in, const float_bf_t w_in, const int_bf_t fid_in,  // input
                       float_bf_t d_out, float_bf_t w_out) {                                 // output
  const auto* face_norm = crystal.GetTriangleNormal();

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


// NOLINTNEXTLINE(readability-function-size)
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


// NOLINTNEXTLINE(readability-function-size)
void Propagate(const Crystal& crystal, size_t num, size_t step,                      // input
               const float_bf_t d_in, const float_bf_t p_in, const float_bf_t w_in,  // input, d, p, w
               float_bf_t p_out, int_bf_t fid_out) {                                 // output, p, fid
  auto face_num = crystal.TotalTriangles();
  const auto* face_transform = crystal.GetTriangleCoordTf();

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

}  // namespace lumice
