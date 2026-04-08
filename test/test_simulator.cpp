#include <gtest/gtest.h>

#include <cstddef>
#include <vector>

#include "core/simulator.hpp"

namespace lumice {
namespace {

size_t SumAlloc(const size_t* alloc, size_t n) {
  size_t sum = 0;
  for (size_t i = 0; i < n; i++) {
    sum += alloc[i];
  }
  return sum;
}

// Case 1: Equal proportions
TEST(PartitionCrystalRayNum, EqualProportions) {
  std::vector<float> proportions = { 0.5f, 0.5f };
  auto result = PartitionCrystalRayNum(proportions, 100);

  EXPECT_EQ(result[0], 50u);
  EXPECT_EQ(result[1], 50u);
  EXPECT_EQ(SumAlloc(result.get(), 2), 100u);
}

// Case 2: Many crystals with small proportions — each should get >=1 when ray_num is large enough
TEST(PartitionCrystalRayNum, SmallProportionsLargeRayNum) {
  // 9 crystals at 0.01, 1 crystal at 0.91 → sum=1.0
  std::vector<float> proportions(9, 0.01f);
  proportions.push_back(0.91f);

  auto result = PartitionCrystalRayNum(proportions, 1000);

  // Each 0.01 crystal: 0.01 * 1000 = 10 → should get >=1
  for (size_t i = 0; i < 9; i++) {
    EXPECT_GE(result[i], 1u) << "Crystal " << i << " with proportion 0.01 got 0 rays";
  }
  EXPECT_EQ(SumAlloc(result.get(), 10), 1000u);
}

// Case 3: Three crystals with small proportions
TEST(PartitionCrystalRayNum, ThreeCrystalsSmallProportions) {
  std::vector<float> proportions = { 0.01f, 0.01f, 0.98f };
  auto result = PartitionCrystalRayNum(proportions, 1000);

  EXPECT_GE(result[0], 1u);
  EXPECT_GE(result[1], 1u);
  EXPECT_EQ(SumAlloc(result.get(), 3), 1000u);
}

// Case 4: Single crystal
TEST(PartitionCrystalRayNum, SingleCrystal) {
  std::vector<float> proportions = { 1.0f };
  auto result = PartitionCrystalRayNum(proportions, 1000);

  EXPECT_EQ(result[0], 1000u);
}

// Case 5: ray_num = 0
TEST(PartitionCrystalRayNum, ZeroRays) {
  std::vector<float> proportions = { 0.5f, 0.5f };
  auto result = PartitionCrystalRayNum(proportions, 0);

  EXPECT_EQ(result[0], 0u);
  EXPECT_EQ(result[1], 0u);
}

// Case 6: Empty proportions list
TEST(PartitionCrystalRayNum, EmptyProportions) {
  std::vector<float> proportions;
  auto result = PartitionCrystalRayNum(proportions, 100);

  // Should not crash, result is a valid (empty) unique_ptr
  EXPECT_NE(result, nullptr);
}

// Case 7: Boundary — proportion * ray_num < 1, some crystals legitimately get 0
TEST(PartitionCrystalRayNum, BoundarySmallProportionSmallRayNum) {
  // 9 crystals at 0.001, 1 crystal at 0.991 → sum=1.0
  std::vector<float> proportions(9, 0.001f);
  proportions.push_back(0.991f);

  auto result = PartitionCrystalRayNum(proportions, 100);

  // Total must be exact
  EXPECT_EQ(SumAlloc(result.get(), 10), 100u);

  // Last crystal should get the bulk
  EXPECT_GE(result[9], 90u);
}

// Case 8: All-zero proportions — should not crash (total_prop == 0)
TEST(PartitionCrystalRayNum, AllZeroProportions) {
  std::vector<float> proportions = { 0.0f, 0.0f, 0.0f };
  auto result = PartitionCrystalRayNum(proportions, 100);

  // All zero proportions → all allocations zero (total_prop guard)
  EXPECT_EQ(SumAlloc(result.get(), 3), 0u);
}

// Determinism: same input should always produce same output
TEST(PartitionCrystalRayNum, Deterministic) {
  std::vector<float> proportions = { 0.3f, 0.5f, 0.2f };

  auto r1 = PartitionCrystalRayNum(proportions, 1000);
  auto r2 = PartitionCrystalRayNum(proportions, 1000);

  EXPECT_EQ(r1[0], r2[0]);
  EXPECT_EQ(r1[1], r2[1]);
  EXPECT_EQ(r1[2], r2[2]);
}

}  // namespace
}  // namespace lumice
