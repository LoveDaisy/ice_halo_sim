#include <gtest/gtest.h>

#include <algorithm>
#include <memory>

#include "core/crystal.hpp"
#include "core/math.hpp"
#include "util/threading_pool.hpp"

namespace {

class RngTest : public ::testing::Test {
 protected:
  static constexpr size_t kCheckSize = 1024;
  static constexpr double kFloatEps = 1e-7;
};


TEST_F(RngTest, GaussianTest) {
  // Prepare buffers to store the random values.
  std::unique_ptr<float[]> values1(new float[kCheckSize]);
  std::unique_ptr<float[]> values2(new float[kCheckSize]);

  auto* rng = icehalo::RandomNumberGenerator::GetInstance();

  // Fill the buffer with Gaussian-distributed random values.
  rng->Reset();
  for (size_t i = 0; i < kCheckSize; i++) {
    values1[i] = rng->GetGaussian();
  }

  // Fill the buffer in multi-threading mode.
  rng->Reset();
  auto thread_pool = icehalo::ThreadingPool::CreatePool();
  thread_pool->CommitRangeStepJobsAndWait(
      0, kCheckSize, [&values2, &rng](int /* thread_id */, int i) { values2[i] = rng->GetGaussian(); });

  // Compare the two buffers. They should be the same after sorting.
  std::sort(values1.get(), values1.get() + kCheckSize);
  std::sort(values2.get(), values2.get() + kCheckSize);
  for (size_t i = 0; i < kCheckSize; i++) {
    ASSERT_NEAR(values1[i], values2[i], kFloatEps);
  }
}


TEST_F(RngTest, UniformTest) {
  // Prepare buffers to store the random values.
  std::unique_ptr<float[]> values1(new float[kCheckSize]);
  std::unique_ptr<float[]> values2(new float[kCheckSize]);

  auto* rng = icehalo::RandomNumberGenerator::GetInstance();

  // Fill the buffer with uniform-distributed random values.
  rng->Reset();
  for (size_t i = 0; i < kCheckSize; i++) {
    values1[i] = rng->GetUniform();
  }

  // Fill the buffer in multi-threading mode.
  rng->Reset();
  auto thread_pool = icehalo::ThreadingPool::CreatePool();
  thread_pool->CommitRangeStepJobsAndWait(
      0, kCheckSize, [&values2, &rng](int /* thread_id */, int i) { values2[i] = rng->GetUniform(); });

  // Compare the two buffers. They should be the same after sorting.
  std::sort(values1.get(), values1.get() + kCheckSize);
  std::sort(values2.get(), values2.get() + kCheckSize);
  for (size_t i = 0; i < kCheckSize; i++) {
    ASSERT_NEAR(values1[i], values2[i], kFloatEps);
  }
}


TEST_F(RngTest, TriangleSample) {
  auto crystal = icehalo::v3::Crystal::CreatePrism(0.2f);

  float p[3];
  float v[3];
  int fid = 0;
  const float* face_vtx = crystal.GetTriangleVtx() + fid * 9;
  icehalo::v3::SampleTrianglePoint(face_vtx, p);
  for (int k = 0; k < 3; k++) {
    v[k] = p[k] - face_vtx[k];
  }
  float dot = icehalo::Dot3(v, crystal.GetTriangleNormal() + fid * 3);
  EXPECT_NEAR(dot, 0.0f, kFloatEps);
}

}  // namespace
