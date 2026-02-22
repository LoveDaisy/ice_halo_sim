#include "optics.hpp"

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

// NOLINTNEXTLINE(readability-function-size)
void HitSurface(const Crystal& crystal, float n, size_t num,                          // input
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


// NOLINTNEXTLINE(readability-function-size,readability-function-cognitive-complexity)
void Propagate(const Crystal& crystal, size_t num, size_t step,                      // input
               const float_bf_t d_in, const float_bf_t p_in, const float_bf_t w_in,  // input, d, p, w
               float_bf_t p_out, int_bf_t fid_out) {                                 // output, p, fid
  auto face_num = crystal.TotalTriangles();
  const auto* face_transform = crystal.GetTriangleCoordTf();

  for (size_t i = 0; i < num; i++) {
    if (w_in[i] < 0) {  // Total reflection
      continue;
    }

    // --- Inlined RayTriangleBW: find nearest triangle intersection ---
    const float* ray_pt = p_in.Ptr(i / step);
    const float* ray_dir = d_in.Ptr(i);
    float* out_pt = p_out.Ptr(i);
    int* out_face_id = fid_out.Ptr(i);

    float min_t = -1.0f;
    *out_face_id = -1;
    std::memcpy(out_pt, ray_pt, 3 * sizeof(float));

    float d[3];
    float p[3];

    for (int fi = 0; fi < static_cast<int>(face_num); fi++) {
      const float* tf = face_transform + fi * 12;
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
        *out_face_id = fi;
        for (int j = 0; j < 3; j++) {
          out_pt[j] += t * ray_dir[j];
        }
      }
    }
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
