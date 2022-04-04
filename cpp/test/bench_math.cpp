#include <benchmark/benchmark.h>
#include <immintrin.h>

#include <memory>

#include "core/math.hpp"

// ========== Define the test fixture ==========
class TestMath : public ::benchmark::Fixture {
 public:
  void SetUp(::benchmark::State& state) override {
    auto num = state.range(0);

    v3f_input1_.reset(new float[num * 3]{});
    v3f_input2_.reset(new float[num * 3]{});
    v9f_input1_.reset(new float[num * 9]{});

    v1f_output1_.reset(new float[num]{});
    v3f_output1_.reset(new float[num * 3]{});

    auto* rng = icehalo::RandomNumberGenerator::GetInstance();
    for (int i = 0; i < num * 3; i++) {
      v3f_input1_[i] = rng->GetGaussian();
      v3f_input2_[i] = rng->GetGaussian();
    }
    for (int i = 0; i < num * 9; i++) {
      v9f_input1_[i] = rng->GetGaussian();
    }
  }

 protected:
  std::unique_ptr<float[]> v3f_input1_;
  std::unique_ptr<float[]> v3f_input2_;
  std::unique_ptr<float[]> v9f_input1_;
  std::unique_ptr<float[]> v1f_output1_;
  std::unique_ptr<float[]> v3f_output1_;
};


//////////////////// Dot //////////////////////////////////////////////
// ========== Naive ==========
BENCHMARK_DEFINE_F(TestMath, DotV3_naive)(::benchmark::State& st) {
  auto num = st.range(0);

  for (auto _ : st) {
    for (int i = 0; i < num; i++) {
      v1f_output1_[i] = v3f_input1_[i * 3 + 0] * v3f_input2_[i * 3 + 0] +
                        v3f_input1_[i * 3 + 1] * v3f_input2_[i * 3 + 1] +
                        v3f_input1_[i * 3 + 2] * v3f_input2_[i * 3 + 2];
    }
  }
}


// ========== Vec3f ==========
BENCHMARK_DEFINE_F(TestMath, DotV3_vec3f)(::benchmark::State& st) {
  auto num = st.range(0);

  for (auto _ : st) {
    const float* pa = v3f_input1_.get();
    const float* pb = v3f_input2_.get();
    for (int i = 0; i < num; i++) {
      v1f_output1_[i] = icehalo::Vec3f::Dot(icehalo::Vec3f(pa + i * 3), icehalo::Vec3f(pb + i * 3));
    }
  }
}


// ========== SIMD ==========
#if defined(__SSE4_1__)
inline float Dot3fSimd(const float* a, const float* b) {
  float res[4];
  __m128 a_ = _mm_loadu_ps(a);
  __m128 b_ = _mm_loadu_ps(b);
  auto c_ = _mm_dp_ps(a_, b_, 0xff);
  _mm_storeu_ps(res, c_);
  return res[0];
}
#endif

BENCHMARK_DEFINE_F(TestMath, DotV3_simd)(::benchmark::State& st) {
  auto num = st.range(0);

  for (auto _ : st) {
    const float* pa = v3f_input1_.get();
    const float* pb = v3f_input2_.get();
    for (int i = 0; i < num; i++) {
      v1f_output1_[i] = Dot3fSimd(pa + i * 3, pb + i * 3);
    }
  }
}


//////////////////// Matrix multiply //////////////////////////////////////////////
// ========== Naive ==========
BENCHMARK_DEFINE_F(TestMath, Mat3x3MulV3_naive)(::benchmark::State& st) {
  auto num = st.range(0);

  for (auto _ : st) {
    for (int i = 0; i < num; i++) {
      v3f_output1_[i * 3 + 0] = v9f_input1_[i * 9 + 0] * v3f_input1_[i * 3 + 0] +
                                v9f_input1_[i * 9 + 1] * v3f_input1_[i * 3 + 1] +
                                v9f_input1_[i * 9 + 2] * v3f_input1_[i * 3 + 2];
      v3f_output1_[i * 3 + 1] = v9f_input1_[i * 9 + 3] * v3f_input1_[i * 3 + 0] +
                                v9f_input1_[i * 9 + 4] * v3f_input1_[i * 3 + 1] +
                                v9f_input1_[i * 9 + 5] * v3f_input1_[i * 3 + 2];
      v3f_output1_[i * 3 + 2] = v9f_input1_[i * 9 + 6] * v3f_input1_[i * 3 + 0] +
                                v9f_input1_[i * 9 + 7] * v3f_input1_[i * 3 + 1] +
                                v9f_input1_[i * 9 + 8] * v3f_input1_[i * 3 + 2];
    }
  }
}


//////////////////// Random number generator ///////////////////////////////////////////
// ========== Legacy ==========
BENCHMARK_DEFINE_F(TestMath, Rng_legacy)(::benchmark::State& st) {
  auto num = st.range(0);
  icehalo::RandomNumberGenerator* rng = nullptr;
  if (st.thread_index() == 0) {
    rng = icehalo::RandomNumberGenerator::GetInstance();
  }

  for (auto _ : st) {
    rng = icehalo::RandomNumberGenerator::GetInstance();
    for (int i = 0; i < num; i++) {
      v1f_output1_[i] = rng->GetGaussian();
    }
  }
}


// ========== Register ==========
BENCHMARK_REGISTER_F(TestMath, DotV3_naive)->RangeMultiplier(10)->Range(100, 1e5);
BENCHMARK_REGISTER_F(TestMath, DotV3_vec3f)->RangeMultiplier(10)->Range(100, 1e5);
BENCHMARK_REGISTER_F(TestMath, DotV3_simd)->RangeMultiplier(10)->Range(100, 1e5);

BENCHMARK_REGISTER_F(TestMath, Mat3x3MulV3_naive)->RangeMultiplier(10)->Range(100, 1e4);

BENCHMARK_REGISTER_F(TestMath, Rng_legacy)->RangeMultiplier(2)->Range(1 << 6, 1 << 16);