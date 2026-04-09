#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <numeric>
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
  std::vector<double> carry(2, 0.0);
  auto result = PartitionCrystalRayNum(proportions, 100, carry);

  EXPECT_EQ(result[0], 50u);
  EXPECT_EQ(result[1], 50u);
  EXPECT_EQ(SumAlloc(result.get(), 2), 100u);

  // Carry should be zero for exact division
  EXPECT_DOUBLE_EQ(carry[0], 0.0);
  EXPECT_DOUBLE_EQ(carry[1], 0.0);
}

// Case 2: Many crystals with small proportions — each should get >=1 when ray_num is large enough
TEST(PartitionCrystalRayNum, SmallProportionsLargeRayNum) {
  // 9 crystals at 0.01, 1 crystal at 0.91 → sum=1.0
  std::vector<float> proportions(9, 0.01f);
  proportions.push_back(0.91f);
  std::vector<double> carry(10, 0.0);

  auto result = PartitionCrystalRayNum(proportions, 1000, carry);

  // Each 0.01 crystal: 0.01 * 1000 = 10 → should get >=1
  for (size_t i = 0; i < 9; i++) {
    EXPECT_GE(result[i], 1u) << "Crystal " << i << " with proportion 0.01 got 0 rays";
  }
  EXPECT_EQ(SumAlloc(result.get(), 10), 1000u);
}

// Case 3: Three crystals with small proportions
TEST(PartitionCrystalRayNum, ThreeCrystalsSmallProportions) {
  std::vector<float> proportions = { 0.01f, 0.01f, 0.98f };
  std::vector<double> carry(3, 0.0);
  auto result = PartitionCrystalRayNum(proportions, 1000, carry);

  EXPECT_GE(result[0], 1u);
  EXPECT_GE(result[1], 1u);
  EXPECT_EQ(SumAlloc(result.get(), 3), 1000u);
}

// Case 4: Single crystal
TEST(PartitionCrystalRayNum, SingleCrystal) {
  std::vector<float> proportions = { 1.0f };
  std::vector<double> carry(1, 0.0);
  auto result = PartitionCrystalRayNum(proportions, 1000, carry);

  EXPECT_EQ(result[0], 1000u);
}

// Case 5: ray_num = 0
TEST(PartitionCrystalRayNum, ZeroRays) {
  std::vector<float> proportions = { 0.5f, 0.5f };
  std::vector<double> carry(2, 0.0);
  auto result = PartitionCrystalRayNum(proportions, 0, carry);

  EXPECT_EQ(result[0], 0u);
  EXPECT_EQ(result[1], 0u);
}

// Case 6: Empty proportions list
TEST(PartitionCrystalRayNum, EmptyProportions) {
  std::vector<float> proportions;
  std::vector<double> carry;
  auto result = PartitionCrystalRayNum(proportions, 100, carry);

  // Should not crash, result is a valid (empty) unique_ptr
  EXPECT_NE(result, nullptr);
}

// Case 7: Boundary — proportion * ray_num < 1, some crystals legitimately get 0
TEST(PartitionCrystalRayNum, BoundarySmallProportionSmallRayNum) {
  // 9 crystals at 0.001, 1 crystal at 0.991 → sum=1.0
  std::vector<float> proportions(9, 0.001f);
  proportions.push_back(0.991f);
  std::vector<double> carry(10, 0.0);

  auto result = PartitionCrystalRayNum(proportions, 100, carry);

  // Total must be exact
  EXPECT_EQ(SumAlloc(result.get(), 10), 100u);

  // Last crystal should get the bulk
  EXPECT_GE(result[9], 90u);
}

// Case 8: All-zero proportions — should not crash (total_prop == 0)
TEST(PartitionCrystalRayNum, AllZeroProportions) {
  std::vector<float> proportions = { 0.0f, 0.0f, 0.0f };
  std::vector<double> carry(3, 0.0);
  auto result = PartitionCrystalRayNum(proportions, 100, carry);

  // All zero proportions → all allocations zero (total_prop guard)
  EXPECT_EQ(SumAlloc(result.get(), 3), 0u);
}

// Determinism: same input should always produce same output
TEST(PartitionCrystalRayNum, Deterministic) {
  std::vector<float> proportions = { 0.3f, 0.5f, 0.2f };

  std::vector<double> carry1(3, 0.0);
  std::vector<double> carry2(3, 0.0);
  auto r1 = PartitionCrystalRayNum(proportions, 1000, carry1);
  auto r2 = PartitionCrystalRayNum(proportions, 1000, carry2);

  EXPECT_EQ(r1[0], r2[0]);
  EXPECT_EQ(r1[1], r2[1]);
  EXPECT_EQ(r1[2], r2[2]);
}

// Verify carry output values after a single call (lock down carry semantics)
TEST(PartitionCrystalRayNum, CarryOutputValues) {
  std::vector<float> proportions = { 0.3f, 0.5f, 0.2f };
  std::vector<double> carry(3, 0.0);
  auto result = PartitionCrystalRayNum(proportions, 100, carry);

  EXPECT_EQ(SumAlloc(result.get(), 3), 100u);

  // All carry values must be in [0, 1)
  for (size_t i = 0; i < 3; i++) {
    EXPECT_GE(carry[i], 0.0) << "carry[" << i << "] is negative";
    EXPECT_LT(carry[i], 1.0) << "carry[" << i << "] >= 1.0";
  }
}

// Cross-batch fairness: 0.5% crystals must get rays within 10 batches
TEST(PartitionCrystalRayNum, CrossBatchFairness) {
  // 9 crystals at 0.5%, 1 crystal at 95.5%
  std::vector<float> proportions(9, 0.005f);
  proportions.push_back(0.955f);
  std::vector<double> carry(10, 0.0);

  std::vector<size_t> total_alloc(10, 0);
  for (int batch = 0; batch < 10; batch++) {
    auto result = PartitionCrystalRayNum(proportions, 128, carry);
    EXPECT_EQ(SumAlloc(result.get(), 10), 128u) << "Batch " << batch << " total mismatch";
    for (size_t i = 0; i < 10; i++) {
      total_alloc[i] += result[i];
    }
  }

  // After 10 batches (1280 total rays), each 0.5% crystal should have > 0 rays
  for (size_t i = 0; i < 9; i++) {
    EXPECT_GT(total_alloc[i], 0u) << "Crystal " << i << " (0.5%) got 0 rays after 10 batches";
  }
}

// Same-proportion crystals should have allocation difference <= 1 after N batches
TEST(PartitionCrystalRayNum, PositionIndependence) {
  // 9 crystals all at 0.5%, 1 at 95.5%
  std::vector<float> proportions(9, 0.005f);
  proportions.push_back(0.955f);
  std::vector<double> carry(10, 0.0);

  std::vector<size_t> total_alloc(10, 0);
  for (int batch = 0; batch < 20; batch++) {
    auto result = PartitionCrystalRayNum(proportions, 128, carry);
    for (size_t i = 0; i < 10; i++) {
      total_alloc[i] += result[i];
    }
  }

  // Among the 9 equal-proportion crystals, max-min difference should be <= 1
  size_t min_alloc = *std::min_element(total_alloc.begin(), total_alloc.begin() + 9);
  size_t max_alloc = *std::max_element(total_alloc.begin(), total_alloc.begin() + 9);
  EXPECT_LE(max_alloc - min_alloc, 1u);
}

// Carry reset: clearing and re-initializing should behave like first call
TEST(PartitionCrystalRayNum, CarryReset) {
  std::vector<float> proportions = { 0.3f, 0.5f, 0.2f };

  // Run a few batches to accumulate carry
  std::vector<double> carry(3, 0.0);
  for (int i = 0; i < 5; i++) {
    PartitionCrystalRayNum(proportions, 100, carry);
  }

  // Reset carry
  carry.assign(3, 0.0);
  auto result_after_reset = PartitionCrystalRayNum(proportions, 100, carry);

  // Should be identical to a fresh call
  std::vector<double> fresh_carry(3, 0.0);
  auto result_fresh = PartitionCrystalRayNum(proportions, 100, fresh_carry);

  for (size_t i = 0; i < 3; i++) {
    EXPECT_EQ(result_after_reset[i], result_fresh[i]);
    EXPECT_DOUBLE_EQ(carry[i], fresh_carry[i]);
  }
}

// Variable ray_num across batches: carry should still work correctly
TEST(PartitionCrystalRayNum, VariableRayNum) {
  std::vector<float> proportions = { 0.005f, 0.995f };
  std::vector<double> carry(2, 0.0);

  size_t total_alloc_0 = 0;
  size_t total_rays = 0;
  std::vector<size_t> ray_nums = { 128, 64, 200, 10, 300, 50, 128, 128, 128, 128 };
  for (auto rn : ray_nums) {
    auto result = PartitionCrystalRayNum(proportions, rn, carry);
    EXPECT_EQ(SumAlloc(result.get(), 2), rn);
    total_alloc_0 += result[0];
    total_rays += rn;
  }

  // After 1264 total rays, 0.5% crystal should have gotten some
  EXPECT_GT(total_alloc_0, 0u);
  // And the ratio should be approximately correct
  double actual_ratio = static_cast<double>(total_alloc_0) / total_rays;
  EXPECT_NEAR(actual_ratio, 0.005, 0.005);  // Within 0.5% absolute
}

// ray_num=0 should not change carry
TEST(PartitionCrystalRayNum, ZeroRayNumPreservesCarry) {
  std::vector<float> proportions = { 0.3f, 0.7f };
  std::vector<double> carry(2, 0.0);

  // Accumulate some carry
  PartitionCrystalRayNum(proportions, 100, carry);
  auto carry_before = carry;

  // Call with ray_num=0
  auto result = PartitionCrystalRayNum(proportions, 0, carry);

  // Carry should be unchanged
  EXPECT_DOUBLE_EQ(carry[0], carry_before[0]);
  EXPECT_DOUBLE_EQ(carry[1], carry_before[1]);
  EXPECT_EQ(result[0], 0u);
  EXPECT_EQ(result[1], 0u);
}

}  // namespace
}  // namespace lumice
