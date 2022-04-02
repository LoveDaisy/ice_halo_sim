#include <benchmark/benchmark.h>
#include <immintrin.h>

#include <memory>
#include <string>

#include "context/context.hpp"
#include "core/crystal.hpp"
#include "core/optics.hpp"

// ========== Define the test fixture ==========
class TestOptics : public ::benchmark::Fixture {
 public:
  void SetUp(::benchmark::State& state) override {
    crystal_ = icehalo::Crystal::CreateHexPrism(1.2f);

    auto face_num = crystal_->TotalFaces();
    auto face_base = crystal_->GetFaceBaseVector();
    auto face_point = crystal_->GetFaceVertex();

    auto ray_num = state.range(0);
    dir_in_.reset(new float[ray_num * 3]{});
    pt_in_.reset(new float[ray_num * 3]{});
    w_in_.reset(new float[ray_num]{});
    face_id_in_.reset(new int[ray_num]{});

    pt_out_.reset(new float[ray_num * 3]);
    face_id_out_.reset(new int[ray_num]);

    // Fill in data
    auto* rng = icehalo::RandomNumberGenerator::GetInstance();
    for (int64_t i = 0; i < ray_num; i++) {
      // 1. Randomly choose a face
      auto face_id = static_cast<int>(rng->GetUniform() * face_num);
      face_id_in_[i] = face_id;

      // 2. Randomly sample a point
      auto u = rng->GetUniform();
      auto v = rng->GetUniform();
      if (u + v > 1.0f) {
        u = 1.0f - u;
        v = 1.0f - v;
      }
      for (int j = 0; j < 3; j++) {
        pt_in_[i * 3 + j] = face_point[face_id * 9 + j] +  // point_0
                            face_base[face_id * 6 + j] * u +
                            face_base[face_id * 6 + 3 + j] * v;  // u * base_0 + v * base_1
      }

      // 3. Randomly choose a direction
      icehalo::Vec3f d{ rng->GetGaussian(), rng->GetGaussian(), rng->GetGaussian() };
      d.Normalize();
      std::memcpy(dir_in_.get() + i * 3, d.val(), 3 * sizeof(float));

      // 4. Randomly assgin an intensity
      w_in_[i] = rng->GetUniform();
    }
  }

 protected:
  icehalo::CrystalPtrU crystal_;
  std::unique_ptr<float[]> dir_in_;
  std::unique_ptr<float[]> pt_in_;
  std::unique_ptr<float[]> w_in_;
  std::unique_ptr<int[]> face_id_in_;

  std::unique_ptr<float[]> pt_out_;
  std::unique_ptr<int[]> face_id_out_;
};

// ========== HitSurface ==========
BENCHMARK_DEFINE_F(TestOptics, HitSurface)(::benchmark::State& st) {
  auto ray_num = st.range(0);

  std::unique_ptr<float[]> dir_out{ new float[ray_num * 6] };
  std::unique_ptr<float[]> w_out{ new float[ray_num * 2] };

  for (auto _ : st) {
    icehalo::Optics::HitSurface(crystal_.get(), 1.31f, ray_num,                 // input
                                dir_in_.get(), face_id_in_.get(), w_in_.get(),  // input
                                dir_out.get(), w_out.get());                    // output
  }
}


// ========== HitSurface_new ==========
BENCHMARK_DEFINE_F(TestOptics, HitSurface_new)(::benchmark::State& st) {
  auto ray_num = st.range(0);

  std::unique_ptr<float[]> dir_out{ new float[ray_num * 6] };
  std::unique_ptr<float[]> w_out{ new float[ray_num * 2] };

  for (auto _ : st) {
    icehalo::optics::HitSurface(crystal_.get(), 1.31f, ray_num,                 // input
                                dir_in_.get(), face_id_in_.get(), w_in_.get(),  // input
                                dir_out.get(), w_out.get());                    // output
  }
}


// ========== RayTriangle ==========
BENCHMARK_DEFINE_F(TestOptics, RayTriangle)(::benchmark::State& st) {
  auto face_num = crystal_->TotalFaces();
  auto face_norm = crystal_->GetFaceNorm();
  auto face_base = crystal_->GetFaceBaseVector();
  auto face_point = crystal_->GetFaceVertex();

  auto ray_num = st.range(0);

  using icehalo::Optics;
  for (auto _ : st) {
    const auto* pt_in = pt_in_.get();
    const auto* dir_in = dir_in_.get();
    const auto* id_in = face_id_in_.get();
    float* pt_out = pt_out_.get();
    int* id_out = face_id_out_.get();
    for (int64_t i = 0; i < ray_num; i++) {
      Optics::IntersectLineWithTriangles(pt_in + i * 3, dir_in + i * 3, id_in[i], face_num,  // input
                                         face_base, face_point, face_norm,                   // input
                                         pt_out + i * 3, id_out + i);                        // output
    }
  }
}


// ========== RayTriangle_simd ==========
BENCHMARK_DEFINE_F(TestOptics, RayTriangle_simd)(::benchmark::State& st) {
  auto face_num = crystal_->TotalFaces();
  auto face_norm = crystal_->GetFaceNorm();
  auto face_base = crystal_->GetFaceBaseVector();
  auto face_point = crystal_->GetFaceVertex();

  auto ray_num = st.range(0);
  using icehalo::Optics;
  for (auto _ : st) {
    const auto* pt_in = pt_in_.get();
    const auto* dir_in = dir_in_.get();
    const auto* id_in = face_id_in_.get();
    float* pt_out = pt_out_.get();
    int* id_out = face_id_out_.get();
    for (int64_t i = 0; i < ray_num; i++) {
      Optics::IntersectLineWithTrianglesSimd(pt_in + i * 3, dir_in + i * 3, id_in[i], face_num,  // input
                                             face_base, face_point, face_norm,                   // input
                                             pt_out + i * 3, id_out + i);                        // output
    }
  }
}


// ========== RayTriangle_MT ==========
// See wiki: https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
void RayTriangleMT(const float* ray_pt, const float* ray_dir, int face_id,                                  // input
                   int face_num, const float* face_edge, const float* face_pt, const float* /*face_norm*/,  // input
                   float* out_pt, int* out_face_id) {                                                       // output
  float min_t = -1.0f;
  *out_face_id = -1;

  float h[3]{};
  float s[3]{};
  for (int i = 0; i < face_num; i++) {
    if (i == face_id) {
      continue;
    }

    const float* vtx = face_pt + i * 9;
    const float* edge = face_edge + i * 6;

    icehalo::Cross3(ray_dir, edge + 3, h);
    float a = icehalo::Dot3(edge, h);
    if (icehalo::FloatEqualZero(a)) {
      continue;  // ray parallel to triangle
    }

    float t = icehalo::Dot3(edge + 3, h) / a;
    if (t < 0.0f) {
      continue;
    }

    icehalo::Vec3FromTo(vtx, ray_pt, s);
    float u = icehalo::Dot3(s, h) / a;
    if (u < 0.0f || u > 1.0f) {
      continue;  // out of this triangle
    }

    icehalo::Cross3(s, edge, h);
    float v = icehalo::Dot3(ray_dir, h) / a;
    if (v < 0.0f || v > 1.0f) {
      continue;  // out of this triangle
    }

    if (u + v > 1.0f) {
      continue;  // out of this triangle
    }

    if (min_t < 0.0f || (t < min_t && min_t > 0.0f)) {
      min_t = t;
      *out_face_id = i;
      for (int j = 0; j < 3; j++) {
        out_pt[j] = ray_pt[j] + t * ray_dir[j];
      }
    }
  }
}

BENCHMARK_DEFINE_F(TestOptics, RayTriangle_MT)(::benchmark::State& st) {
  auto face_num = crystal_->TotalFaces();
  auto face_norm = crystal_->GetFaceNorm();
  auto face_base = crystal_->GetFaceBaseVector();
  auto face_point = crystal_->GetFaceVertex();

  auto ray_num = st.range(0);

  using icehalo::Optics;
  for (auto _ : st) {
    const auto* pt_in = pt_in_.get();
    const auto* dir_in = dir_in_.get();
    const auto* id_in = face_id_in_.get();
    float* pt_out = pt_out_.get();
    int* id_out = face_id_out_.get();
    for (int64_t i = 0; i < ray_num; i++) {
      RayTriangleMT(pt_in + i * 3, dir_in + i * 3, id_in[i], face_num,  // input
                    face_base, face_point, face_norm,                   // input
                    pt_out + i * 3, id_out + i);                        // output
    }
  }
}


// ========== RayTriangle_BW ==========
void RayTriangleBW(const float* ray_pt, const float* ray_dir, int face_id,  // input
                   int face_num, const float* face_transform,               // input
                   float* out_pt, int* out_face_id) {
  float min_t = -1.0f;
  *out_face_id = -1;

  float d[3];
  float p[3];

  for (int i = 0; i < face_num; i++) {
    if (i == face_id) {
      continue;
    }

    const float* tf = face_transform + i * 12;
    p[0] = icehalo::Dot3(ray_pt, tf + 0) + tf[3];
    p[1] = icehalo::Dot3(ray_pt, tf + 4) + tf[7];
    p[2] = icehalo::Dot3(ray_pt, tf + 8) + tf[11];

    d[0] = icehalo::Dot3(ray_dir, tf + 0) + tf[3];
    d[1] = icehalo::Dot3(ray_dir, tf + 4) + tf[7];
    d[2] = icehalo::Dot3(ray_dir, tf + 8) + tf[11];

    if (icehalo::FloatEqualZero(d[2])) {
      continue;  // Parallel to this triangle
    }

    auto t = -p[2] / d[2];
    if (t < 0.0f) {
      continue;
    }

    auto u = p[0] + t * d[0];
    if (u < 0.0f || u > 1.0f) {
      continue;  // out of this triangle
    }

    auto v = p[1] + t * d[1];
    if (v < 0.0f || v > 1.0f) {
      continue;  // out of this triangle
    }

    if (u + v > 1.0f) {
      continue;  // out of this triangle
    }

    if (min_t < 0.0f || (t < min_t && min_t > 0.0f)) {
      min_t = t;
      *out_face_id = i;
      for (int j = 0; j < 3; j++) {
        out_pt[j] = ray_pt[j] + t * ray_dir[j];
      }
    }
  }
}

BENCHMARK_DEFINE_F(TestOptics, RayTriangle_BW)(::benchmark::State& st) {
  auto face_num = crystal_->TotalFaces();
  auto face_norm = crystal_->GetFaceNorm();
  auto face_base = crystal_->GetFaceBaseVector();
  auto face_point = crystal_->GetFaceVertex();

  auto ray_num = st.range(0);
  std::unique_ptr<float[]> face_transform{ new float[face_num * 12] };
  float m[3];
  for (int i = 0; i < face_num; i++) {
    icehalo::Cross3(face_base + i * 6 + 3, face_norm + i * 3, m);
    auto a = icehalo::Dot3(face_base + i * 6, m);
    face_transform[i * 12 + 0] =
        (face_base[i * 6 + 4] * face_norm[i * 3 + 2] - face_base[i * 6 + 5] * face_norm[i * 3 + 1]) / a;
    face_transform[i * 12 + 1] =
        (face_base[i * 6 + 5] * face_norm[i * 3 + 0] - face_base[i * 6 + 3] * face_norm[i * 3 + 2]) / a;
    face_transform[i * 12 + 2] =
        (face_base[i * 6 + 3] * face_norm[i * 3 + 1] - face_base[i * 6 + 4] * face_norm[i * 3 + 0]) / a;

    face_transform[i * 12 + 4] =
        (face_base[i * 6 + 2] * face_norm[i * 3 + 1] - face_base[i * 6 + 1] * face_norm[i * 3 + 2]) / a;
    face_transform[i * 12 + 5] =
        (face_base[i * 6 + 0] * face_norm[i * 3 + 2] - face_base[i * 6 + 2] * face_norm[i * 3 + 0]) / a;
    face_transform[i * 12 + 6] =
        (face_base[i * 6 + 1] * face_norm[i * 3 + 0] - face_base[i * 6 + 0] * face_norm[i * 3 + 1]) / a;

    face_transform[i * 12 + 8] =
        (face_base[i * 6 + 1] * face_base[i * 6 + 5] - face_base[i * 6 + 2] * face_base[i * 6 + 4]) / a;
    face_transform[i * 12 + 9] =
        (face_base[i * 6 + 2] * face_base[i * 6 + 3] - face_base[i * 6 + 0] * face_base[i * 6 + 5]) / a;
    face_transform[i * 12 + 10] =
        (face_base[i * 6 + 0] * face_base[i * 6 + 4] - face_base[i * 6 + 1] * face_base[i * 6 + 3]) / a;

    face_transform[i * 12 + 3] = -icehalo::Dot3(face_transform.get() + i * 12 + 0, face_point + i * 9);
    face_transform[i * 12 + 7] = -icehalo::Dot3(face_transform.get() + i * 12 + 4, face_point + i * 9);
    face_transform[i * 12 + 11] = -icehalo::Dot3(face_transform.get() + i * 12 + 8, face_point + i * 9);
  }

  using icehalo::Optics;
  for (auto _ : st) {
    const auto* pt_in = pt_in_.get();
    const auto* dir_in = dir_in_.get();
    const auto* id_in = face_id_in_.get();
    float* pt_out = pt_out_.get();
    int* id_out = face_id_out_.get();
    for (int64_t i = 0; i < ray_num; i++) {
      RayTriangleBW(pt_in + i * 3, dir_in + i * 3, id_in[i],  // input
                    face_num, face_transform.get(),           // input
                    pt_out + i * 3, id_out + i);              // output
    }
  }
}


// ========== RayTriangle_BW_simd ==========
void RayTriangleBWSimd(const float* ray_pt, const float* ray_dir, int face_id,  // input
                       int face_num, const float* face_transform,               // input
                       float* out_pt, int* out_face_id) {
  float min_t = -1.0f;
  *out_face_id = -1;

  float tmp_dz[4];
  float tmp_t[4];
  float tmp_u[4];
  float tmp_v[4];

  __m128 tf_[12];
  __m128 ray_p_[3];
  __m128 ray_d_[3];
  for (int j = 0; j < 3; j++) {
    ray_p_[j] = _mm_set_ps1(ray_pt[j]);
    ray_d_[j] = _mm_set_ps1(ray_dir[j]);
  }

  int i = 0;
  for (; i + 3 < face_num; i += 4) {
    const float* tf = face_transform + i * 12;

    for (int j = 0; j < 12; j++) {
      tf_[j] = _mm_set_ps(tf[36 + j], tf[24 + j], tf[12 + j], tf[j]);
    }

    auto px_ = _mm_add_ps(_mm_add_ps(_mm_mul_ps(ray_p_[0], tf_[0]), _mm_mul_ps(ray_p_[1], tf_[1])),
                          _mm_fmadd_ps(ray_p_[2], tf_[2], tf_[3]));
    auto py_ = _mm_add_ps(_mm_add_ps(_mm_mul_ps(ray_p_[0], tf_[4]), _mm_mul_ps(ray_p_[1], tf_[5])),
                          _mm_fmadd_ps(ray_p_[2], tf_[6], tf_[7]));
    auto pz_ = _mm_add_ps(_mm_add_ps(_mm_mul_ps(ray_p_[0], tf_[8]), _mm_mul_ps(ray_p_[1], tf_[9])),
                          _mm_fmadd_ps(ray_p_[2], tf_[10], tf_[11]));

    auto dx_ = _mm_add_ps(_mm_add_ps(_mm_mul_ps(ray_d_[0], tf_[0]), _mm_mul_ps(ray_d_[1], tf_[1])),
                          _mm_fmadd_ps(ray_d_[2], tf_[2], tf_[3]));
    auto dy_ = _mm_add_ps(_mm_add_ps(_mm_mul_ps(ray_d_[0], tf_[4]), _mm_mul_ps(ray_d_[1], tf_[5])),
                          _mm_fmadd_ps(ray_d_[2], tf_[6], tf_[7]));
    auto dz_ = _mm_add_ps(_mm_add_ps(_mm_mul_ps(ray_d_[0], tf_[8]), _mm_mul_ps(ray_d_[1], tf_[9])),
                          _mm_fmadd_ps(ray_d_[2], tf_[10], tf_[11]));

    auto t_ = _mm_div_ps(-pz_, dz_);
    auto u_ = _mm_fmadd_ps(t_, dx_, px_);
    auto v_ = _mm_fmadd_ps(t_, dy_, py_);

    _mm_storeu_ps(tmp_dz, dz_);
    _mm_storeu_ps(tmp_t, t_);
    _mm_storeu_ps(tmp_u, u_);
    _mm_storeu_ps(tmp_v, v_);

    for (int j = 0; j < 4; j++) {
      if (i + j == face_id) {
        continue;
      }
      if (icehalo::FloatEqualZero(tmp_dz[j])) {
        continue;
      }
      if (tmp_t[j] < 0.0f || tmp_u[j] < 0.0f || tmp_v[j] < 0.0f || tmp_u[j] + tmp_v[j] > 1.0f) {
        continue;
      }
      if (min_t < 0.0f || (tmp_t[j] < min_t && min_t > 0.0f)) {
        min_t = tmp_t[j];
        *out_face_id = i + j;
        for (int k = 0; k < 3; k++) {
          out_pt[k] = ray_pt[k] + tmp_t[j] * ray_dir[k];
        }
      }
    }
  }
}

BENCHMARK_DEFINE_F(TestOptics, RayTriangle_BW_simd)(::benchmark::State& st) {
  auto face_num = crystal_->TotalFaces();
  auto face_norm = crystal_->GetFaceNorm();
  auto face_base = crystal_->GetFaceBaseVector();
  auto face_point = crystal_->GetFaceVertex();

  auto ray_num = st.range(0);
  std::unique_ptr<float[]> face_transform{ new float[face_num * 12] };
  float m[3];
  for (int i = 0; i < face_num; i++) {
    icehalo::Cross3(face_base + i * 6 + 3, face_norm + i * 3, m);
    auto a = icehalo::Dot3(face_base + i * 6, m);
    face_transform[i * 12 + 0] =
        (face_base[i * 6 + 4] * face_norm[i * 3 + 2] - face_base[i * 6 + 5] * face_norm[i * 3 + 1]) / a;
    face_transform[i * 12 + 1] =
        (face_base[i * 6 + 5] * face_norm[i * 3 + 0] - face_base[i * 6 + 3] * face_norm[i * 3 + 2]) / a;
    face_transform[i * 12 + 2] =
        (face_base[i * 6 + 3] * face_norm[i * 3 + 1] - face_base[i * 6 + 4] * face_norm[i * 3 + 0]) / a;

    face_transform[i * 12 + 4] =
        (face_base[i * 6 + 2] * face_norm[i * 3 + 1] - face_base[i * 6 + 1] * face_norm[i * 3 + 2]) / a;
    face_transform[i * 12 + 5] =
        (face_base[i * 6 + 0] * face_norm[i * 3 + 2] - face_base[i * 6 + 2] * face_norm[i * 3 + 0]) / a;
    face_transform[i * 12 + 6] =
        (face_base[i * 6 + 1] * face_norm[i * 3 + 0] - face_base[i * 6 + 0] * face_norm[i * 3 + 1]) / a;

    face_transform[i * 12 + 8] =
        (face_base[i * 6 + 1] * face_base[i * 6 + 5] - face_base[i * 6 + 2] * face_base[i * 6 + 4]) / a;
    face_transform[i * 12 + 9] =
        (face_base[i * 6 + 2] * face_base[i * 6 + 3] - face_base[i * 6 + 0] * face_base[i * 6 + 5]) / a;
    face_transform[i * 12 + 10] =
        (face_base[i * 6 + 0] * face_base[i * 6 + 4] - face_base[i * 6 + 1] * face_base[i * 6 + 3]) / a;

    face_transform[i * 12 + 3] = -icehalo::Dot3(face_transform.get() + i * 12 + 0, face_point + i * 9);
    face_transform[i * 12 + 7] = -icehalo::Dot3(face_transform.get() + i * 12 + 4, face_point + i * 9);
    face_transform[i * 12 + 11] = -icehalo::Dot3(face_transform.get() + i * 12 + 8, face_point + i * 9);
  }

  using icehalo::Optics;
  for (auto _ : st) {
    const auto* pt_in = pt_in_.get();
    const auto* dir_in = dir_in_.get();
    const auto* id_in = face_id_in_.get();
    float* pt_out = pt_out_.get();
    int* id_out = face_id_out_.get();
    for (int64_t i = 0; i < ray_num; i++) {
      RayTriangleBWSimd(pt_in + i * 3, dir_in + i * 3, id_in[i],  // input
                        face_num, face_transform.get(),           // input
                        pt_out + i * 3, id_out + i);              // output
    }
  }
}


///////////////////////// Register bechmarks //////////////////////////////////////
BENCHMARK_REGISTER_F(TestOptics, HitSurface)->Arg(10)->Arg(100)->Arg(1000);
BENCHMARK_REGISTER_F(TestOptics, HitSurface_new)->Arg(10)->Arg(100)->Arg(1000);

constexpr int kStepMultiplier = 2;
#define RAY_TRIANGLE_RANGE Range(10, 10000)
BENCHMARK_REGISTER_F(TestOptics, RayTriangle)->RangeMultiplier(kStepMultiplier)->RAY_TRIANGLE_RANGE;
BENCHMARK_REGISTER_F(TestOptics, RayTriangle_MT)->RangeMultiplier(kStepMultiplier)->RAY_TRIANGLE_RANGE;
BENCHMARK_REGISTER_F(TestOptics, RayTriangle_BW)->RangeMultiplier(kStepMultiplier)->RAY_TRIANGLE_RANGE;
BENCHMARK_REGISTER_F(TestOptics, RayTriangle_BW_simd)->RangeMultiplier(kStepMultiplier)->RAY_TRIANGLE_RANGE;
BENCHMARK_REGISTER_F(TestOptics, RayTriangle_simd)->RangeMultiplier(kStepMultiplier)->RAY_TRIANGLE_RANGE;