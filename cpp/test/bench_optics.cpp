#include <benchmark/benchmark.h>

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
};

// ========== HitSurface ==========
BENCHMARK_DEFINE_F(TestOptics, TestHitSurface)(::benchmark::State& st) {
  auto ray_num = st.range(0);

  std::unique_ptr<float[]> dir_out{ new float[ray_num * 6] };
  std::unique_ptr<float[]> w_out{ new float[ray_num * 2] };

  for (auto _ : st) {
    icehalo::Optics::HitSurface(crystal_.get(), 1.31f, ray_num,                 // input
                                dir_in_.get(), face_id_in_.get(), w_in_.get(),  // input
                                dir_out.get(), w_out.get());                    // output
  }
}
BENCHMARK_REGISTER_F(TestOptics, TestHitSurface)->Arg(10)->Arg(100)->Arg(1000);


// ========== HitSurfaceNew ==========
BENCHMARK_DEFINE_F(TestOptics, TestHitSurfaceNew)(::benchmark::State& st) {
  auto ray_num = st.range(0);

  std::unique_ptr<float[]> dir_out{ new float[ray_num * 6] };
  std::unique_ptr<float[]> w_out{ new float[ray_num * 2] };

  for (auto _ : st) {
    icehalo::optics::HitSurface(crystal_.get(), 1.31f, ray_num,                 // input
                                dir_in_.get(), face_id_in_.get(), w_in_.get(),  // input
                                dir_out.get(), w_out.get());                    // output
  }
}
BENCHMARK_REGISTER_F(TestOptics, TestHitSurfaceNew)->Arg(10)->Arg(100)->Arg(1000);


// ========== IntersectLineWithTriangles ==========
BENCHMARK_DEFINE_F(TestOptics, TestIntersectLineWithTriangles)(::benchmark::State& st) {
  auto face_num = crystal_->TotalFaces();
  auto face_norm = crystal_->GetFaceNorm();
  auto face_base = crystal_->GetFaceBaseVector();
  auto face_point = crystal_->GetFaceVertex();

  auto ray_num = st.range(0);
  float out_pt_normal[]{ 0, 0, 0 };
  int out_id_normal = -1;

  using icehalo::Optics;
  for (auto _ : st) {
    const auto* pt_in = pt_in_.get();
    const auto* dir_in = dir_in_.get();
    const auto* id_in = face_id_in_.get();
    for (int64_t i = 0; i < ray_num; i++) {
      Optics::IntersectLineWithTriangles(pt_in + i * 3, dir_in + i * 3, id_in[i],
                                         face_num,                          // input
                                         face_base, face_point, face_norm,  // input
                                         out_pt_normal, &out_id_normal);    // output
    }
  }
}
BENCHMARK_REGISTER_F(TestOptics, TestIntersectLineWithTriangles)->Arg(10)->Arg(100)->Arg(1000);


// ========== IntersectLineWithTrianglesSimd ==========
BENCHMARK_DEFINE_F(TestOptics, TestIntersectLineWithTrianglesSimd)(::benchmark::State& st) {
  auto face_num = crystal_->TotalFaces();
  auto face_norm = crystal_->GetFaceNorm();
  auto face_base = crystal_->GetFaceBaseVector();
  auto face_point = crystal_->GetFaceVertex();

  auto ray_num = st.range(0);
  float out_pt_normal[]{ 0, 0, 0 };
  int out_id_normal = -1;

  using icehalo::Optics;
  for (auto _ : st) {
    const auto* pt_in = pt_in_.get();
    const auto* dir_in = dir_in_.get();
    const auto* id_in = face_id_in_.get();
    for (int64_t i = 0; i < ray_num; i++) {
      Optics::IntersectLineWithTrianglesSimd(pt_in + i * 3, dir_in + i * 3, id_in[i],
                                             face_num,                          // input
                                             face_base, face_point, face_norm,  // input
                                             out_pt_normal, &out_id_normal);    // output
    }
  }
}
BENCHMARK_REGISTER_F(TestOptics, TestIntersectLineWithTrianglesSimd)->Arg(10)->Arg(100)->Arg(1000);