#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <limits>
#include <numeric>
#include <vector>

#include "config/filter_config.hpp"
#include "config/proj_config.hpp"
#include "config/sim_data.hpp"
#include "core/filter_spec.hpp"
#include "core/geo3d.hpp"
#include "core/math.hpp"
#include "core/raypath.hpp"
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

  // After a single call with fresh carry and no correction needed,
  // carry values should be in [0, 1). After deficit/surplus correction,
  // carry can temporarily be outside this range (self-correcting over batches).
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

// ============================================================================
// BuildCrystalRotation chain math verification
//
// Validates the 4 chain cases listed in
//   scratchpad/scrum-coordinate-system-overhaul/explore-clarify-coordinate-convention/
//   coordinate_convention_v1.md, Appendix A.
//
// Chain under test: R = Rz(az - pi) * Ry(-zenith) * Rz(roll), with
//                   zenith = pi/2 - latitude.
// Verifies the world directions of crystal local basis vectors:
//   N1 (= local +z, the c-axis) and N3 (= local +x).
// ============================================================================

namespace {

constexpr float kChainTolDot = 1.0f - 1e-5f;

void ApplyRotation(const Rotation& r, float v[3]) {
  r.Apply(v);
}

float Dot(const float a[3], const float b[3]) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

Rotation BuildFromDeg(float az_deg, float zenith_deg, float roll_deg) {
  float az_rad = az_deg * math::kDegreeToRad;
  float lat_rad = (90.0f - zenith_deg) * math::kDegreeToRad;
  float roll_rad = roll_deg * math::kDegreeToRad;
  return BuildCrystalRotation(az_rad, lat_rad, roll_rad);
}

void ExpectAlignedWith(const float actual[3], const float expected[3], const char* label) {
  EXPECT_GT(Dot(actual, expected), kChainTolDot)
      << label << ": (" << actual[0] << ", " << actual[1] << ", " << actual[2] << ") vs expected (" << expected[0]
      << ", " << expected[1] << ", " << expected[2] << ")";
}

}  // namespace

// Case A: (az=0, zenith=0, roll=0)
// Validates the az - 180 degree offset term: with no other rotation
// in play, N3 is rotated by Rz(-pi) and ends up at world -x.
TEST(BuildCrystalRotation, CaseA_AzOffsetOnly) {
  Rotation r = BuildFromDeg(0.0f, 0.0f, 0.0f);
  float n1[3] = { 0, 0, 1 };
  float n3[3] = { 1, 0, 0 };
  ApplyRotation(r, n1);
  ApplyRotation(r, n3);
  constexpr float kExpN1[3] = { 0, 0, 1 };
  constexpr float kExpN3[3] = { -1, 0, 0 };
  ExpectAlignedWith(n1, kExpN1, "Case A N1 (expect +z)");
  ExpectAlignedWith(n3, kExpN3, "Case A N3 (expect -x)");
}

// Case B: (az=0, zenith=90, roll=0)
// Validates the Ry(-zenith) sign: this is the "Parry default" pose
// where N3 should point to world +z.
TEST(BuildCrystalRotation, CaseB_ParryLikePose) {
  Rotation r = BuildFromDeg(0.0f, 90.0f, 0.0f);
  float n1[3] = { 0, 0, 1 };
  float n3[3] = { 1, 0, 0 };
  ApplyRotation(r, n1);
  ApplyRotation(r, n3);
  constexpr float kExpN1[3] = { 1, 0, 0 };
  constexpr float kExpN3[3] = { 0, 0, 1 };
  ExpectAlignedWith(n1, kExpN1, "Case B N1 (expect +x)");
  ExpectAlignedWith(n3, kExpN3, "Case B N3 (expect +z)");
}

// Case C: (az=90, zenith=90, roll=0)
// Validates the Rz(az - pi) term with non-trivial azimuth: N1 lands on
// world +y, while N3 still points to world +z (independent of az when
// zenith=90).
TEST(BuildCrystalRotation, CaseC_NonTrivialAz) {
  Rotation r = BuildFromDeg(90.0f, 90.0f, 0.0f);
  float n1[3] = { 0, 0, 1 };
  float n3[3] = { 1, 0, 0 };
  ApplyRotation(r, n1);
  ApplyRotation(r, n3);
  constexpr float kExpN1[3] = { 0, 1, 0 };
  constexpr float kExpN3[3] = { 0, 0, 1 };
  ExpectAlignedWith(n1, kExpN1, "Case C N1 (expect +y)");
  ExpectAlignedWith(n3, kExpN3, "Case C N3 (expect +z)");
}

// Case D: (az=0, zenith=0, roll=90)
// Validates the Rz(roll) term: roll rotates around the local c-axis,
// keeping N1 unchanged but moving N3 around.
TEST(BuildCrystalRotation, CaseD_RollAroundCAxis) {
  Rotation r = BuildFromDeg(0.0f, 0.0f, 90.0f);
  float n1[3] = { 0, 0, 1 };
  float n3[3] = { 1, 0, 0 };
  ApplyRotation(r, n1);
  ApplyRotation(r, n3);
  constexpr float kExpN1[3] = { 0, 0, 1 };
  constexpr float kExpN3[3] = { 0, -1, 0 };
  ExpectAlignedWith(n1, kExpN1, "Case D N1 (expect +z)");
  ExpectAlignedWith(n3, kExpN3, "Case D N3 (expect -y)");
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

// ============================================================================
// CollectData filter dispatch (Design A: simulator-side emit-gate)
//
// Confirms that filter-fail rays are dropped at the simulator boundary
// (IsFilterDropped()=true, IsOutgoing()=false) so they never reach the
// outgoing buffer or downstream consumers. See doc/filter-architecture.md §2
// and task-revert-filter-to-simulator-side AC-1.
// ============================================================================

// kFilterIn (default) + Match() returning false → Check() == false. Emulates a
// "filter-fail" ray without depending on FilterSpec orbit setup.
class AlwaysRejectSpec : public FilterSpec {
 public:
  bool Match(const RaySeg& /*ray*/) const override { return false; }
};

class AlwaysAcceptSpec : public FilterSpec {
 public:
  bool Match(const RaySeg& /*ray*/) const override { return true; }
};

namespace {

RaySeg MakeOutgoingCandidate() {
  RaySeg r{};
  r.d_[0] = 1.0f;
  r.d_[1] = 0.0f;
  r.d_[2] = 0.0f;
  r.p_[0] = 0.0f;
  r.p_[1] = 0.0f;
  r.p_[2] = 0.0f;
  r.w_ = 0.5f;  // positive (not TIR)
  r.from_face_ = kInvalidId;
  r.to_face_ = kInvalidId;  // outgoing candidate marker
  r.crystal_rot_ = Rotation{};
  return r;
}

}  // namespace

TEST(CollectDataFilterDispatch, FilterFailDroppedWithZeroProb) {
  RandomNumberGenerator rng(42);
  AlwaysRejectSpec filter;
  MsInfo ms_info;
  // Design A: filter is checked before prob. AlwaysRejectSpec → ray is
  // marked IsFilterDropped(), regardless of prob value.
  ms_info.prob_ = 0.0f;

  RayBuffer buffer_data[2];
  RayBuffer init_data[2];
  buffer_data[0].Reset(8);
  buffer_data[1].Reset(8);
  init_data[0].Reset(8);
  init_data[1].Reset(8);

  buffer_data[1].EmplaceBack(MakeOutgoingCandidate());

  CollectData(rng, ms_info, &filter, buffer_data, init_data);

  ASSERT_EQ(buffer_data[1].size_, 1u);
  const auto& r = buffer_data[1].rays_[0];
  EXPECT_TRUE(r.IsFilterDropped()) << "filter-fail ray must be IsFilterDropped()";
  EXPECT_FALSE(r.IsOutgoing()) << "filter-fail ray must not reach outgoing";
  EXPECT_FALSE(r.IsContinue());
  EXPECT_EQ(init_data[1].size_, 0u);
}

TEST(CollectDataFilterDispatch, FilterFailDroppedWithNonZeroProb) {
  // prob_=1.0f: still drops because filter is checked first under Design A.
  RandomNumberGenerator rng(42);
  AlwaysRejectSpec filter;
  MsInfo ms_info;
  ms_info.prob_ = 1.0f;

  RayBuffer buffer_data[2];
  RayBuffer init_data[2];
  buffer_data[0].Reset(8);
  buffer_data[1].Reset(8);
  init_data[0].Reset(8);
  init_data[1].Reset(8);

  buffer_data[1].EmplaceBack(MakeOutgoingCandidate());

  CollectData(rng, ms_info, &filter, buffer_data, init_data);

  ASSERT_EQ(buffer_data[1].size_, 1u);
  const auto& r = buffer_data[1].rays_[0];
  EXPECT_TRUE(r.IsFilterDropped()) << "filter-fail ray must be IsFilterDropped() under Design A";
  EXPECT_FALSE(r.IsOutgoing());
  EXPECT_FALSE(r.IsContinue());
  EXPECT_EQ(init_data[1].size_, 0u);
}

TEST(CollectDataFilterDispatch, FilterPassWithProbContinues) {
  RandomNumberGenerator rng(42);
  AlwaysAcceptSpec filter;
  MsInfo ms_info;
  ms_info.prob_ = 1.0f;  // always branch when filter passes

  RayBuffer buffer_data[2];
  RayBuffer init_data[2];
  buffer_data[0].Reset(8);
  buffer_data[1].Reset(8);
  init_data[0].Reset(8);
  init_data[1].Reset(8);

  buffer_data[1].EmplaceBack(MakeOutgoingCandidate());

  CollectData(rng, ms_info, &filter, buffer_data, init_data);

  ASSERT_EQ(buffer_data[1].size_, 1u);
  EXPECT_TRUE(buffer_data[1].rays_[0].IsContinue());
  EXPECT_EQ(init_data[1].size_, 1u);
}

TEST(CollectDataFilterDispatch, FilterPassNoProbEmitsOutgoing) {
  RandomNumberGenerator rng(42);
  AlwaysAcceptSpec filter;
  MsInfo ms_info;
  ms_info.prob_ = 0.0f;  // no branching → outgoing

  RayBuffer buffer_data[2];
  RayBuffer init_data[2];
  buffer_data[0].Reset(8);
  buffer_data[1].Reset(8);
  init_data[0].Reset(8);
  init_data[1].Reset(8);

  buffer_data[1].EmplaceBack(MakeOutgoingCandidate());

  CollectData(rng, ms_info, &filter, buffer_data, init_data);

  ASSERT_EQ(buffer_data[1].size_, 1u);
  EXPECT_TRUE(buffer_data[1].rays_[0].IsOutgoing());
  EXPECT_EQ(init_data[1].size_, 0u);
}

// ---- AC-5: derived-helper unit tests for RaySeg segment-kind predicates ----

namespace {

RaySeg MakeRaySegBase() {
  RaySeg r{};
  r.d_[0] = 1.0f;
  r.d_[1] = 0.0f;
  r.d_[2] = 0.0f;
  r.p_[0] = 0.0f;
  r.p_[1] = 0.0f;
  r.p_[2] = 0.0f;
  r.from_face_ = kInvalidId;
  r.crystal_rot_ = Rotation{};
  return r;
}

}  // namespace

// TC-1: to_face_ != kInvalidId && w_ >= 0  →  IsNormal()
TEST(RaySegDerivedKind, NormalWhenHasFaceAndPositiveWeight) {
  RaySeg r = MakeRaySegBase();
  r.w_ = 0.5f;
  r.to_face_ = 3;  // any non-sentinel face id
  r.is_continue_ = false;

  EXPECT_TRUE(r.IsNormal());
  EXPECT_FALSE(r.IsTir());
  EXPECT_FALSE(r.IsOutgoing());
  EXPECT_FALSE(r.IsContinue());
}

// TC-2: w_ < 0  →  IsTir()  (overrides to_face_ value)
TEST(RaySegDerivedKind, TirWhenNegativeWeight) {
  RaySeg r = MakeRaySegBase();
  r.w_ = -1.0f;
  r.to_face_ = kInvalidId;  // also exercises that w_<0 wins over to_face_ branches
  r.is_continue_ = false;

  EXPECT_TRUE(r.IsTir());
  EXPECT_FALSE(r.IsNormal());
  EXPECT_FALSE(r.IsOutgoing());
  EXPECT_FALSE(r.IsContinue());
}

// TC-3: to_face_ == kInvalidId && w_ >= 0 && !is_continue_  →  IsOutgoing()
TEST(RaySegDerivedKind, OutgoingWhenNoFaceAndNotContinue) {
  RaySeg r = MakeRaySegBase();
  r.w_ = 0.5f;
  r.to_face_ = kInvalidId;
  r.is_continue_ = false;

  EXPECT_TRUE(r.IsOutgoing());
  EXPECT_FALSE(r.IsNormal());
  EXPECT_FALSE(r.IsTir());
  EXPECT_FALSE(r.IsContinue());
}

// TC-4: to_face_ == kInvalidId && w_ >= 0 && is_continue_  →  IsContinue() (not IsOutgoing)
TEST(RaySegDerivedKind, ContinueWhenOutgoingCandidateBranchGated) {
  RaySeg r = MakeRaySegBase();
  r.w_ = 0.5f;
  r.to_face_ = kInvalidId;
  r.is_continue_ = true;

  EXPECT_TRUE(r.IsContinue());
  EXPECT_FALSE(r.IsOutgoing());  // is_continue_ excludes IsOutgoing
  EXPECT_FALSE(r.IsNormal());
  EXPECT_FALSE(r.IsTir());
}

// ---- AC-3: RaySeg::IsValidComplete() — N4 construction-time invariants ----

namespace {

// Build a fully N4-compliant RaySeg as the positive baseline. Each negative
// test below mutates one field to exercise a single invariant in isolation.
RaySeg MakeValidRaySeg() {
  RaySeg r = MakeRaySegBase();
  r.w_ = 0.5f;
  r.to_face_ = 3;
  r.is_continue_ = false;
  r.crystal_idx_ = 0;
  return r;
}

}  // namespace

TEST(RaySegValidate, ValidRayPassesAllChecks) {
  RaySeg r = MakeValidRaySeg();
  EXPECT_TRUE(r.IsValidComplete());
}

TEST(RaySegValidate, TirSentinelWeightPasses) {
  // w_ == -1.0f exactly is the TIR sentinel and must be accepted by N4-2.
  RaySeg r = MakeValidRaySeg();
  r.w_ = -1.0f;
  r.to_face_ = kInvalidId;
  EXPECT_TRUE(r.IsValidComplete());
}

TEST(RaySegValidate, InvalidWeightNotMinusOne) {
  // N4-2 violation: negative weight other than the TIR sentinel.
  RaySeg r = MakeValidRaySeg();
  r.w_ = -0.5f;
  EXPECT_FALSE(r.IsValidComplete());
}

TEST(RaySegValidate, ContinueWithValidFace) {
  // N4-3 violation: is_continue_ ray must have to_face_ == kInvalidId.
  RaySeg r = MakeValidRaySeg();
  r.is_continue_ = true;
  r.to_face_ = 3;
  EXPECT_FALSE(r.IsValidComplete());
}

TEST(RaySegValidate, CrystalIdxOutOfRange) {
  // N4-4 violation: crystal_idx_ must be < kMaxCrystalNum or == kInvalidId.
  RaySeg r = MakeValidRaySeg();
  r.crystal_idx_ = static_cast<IdType>(kMaxCrystalNum);  // == 16, out of [0, 15]
  EXPECT_FALSE(r.IsValidComplete());
}

TEST(RaySegValidate, DirectionNaN) {
  // N4-5 violation: direction component is NaN.
  RaySeg r = MakeValidRaySeg();
  r.d_[0] = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(r.IsValidComplete());
}

TEST(RaySegValidate, PositionInf) {
  // N4-5 violation: position component is Inf.
  RaySeg r = MakeValidRaySeg();
  r.p_[1] = std::numeric_limits<float>::infinity();
  EXPECT_FALSE(r.IsValidComplete());
}

}  // namespace
}  // namespace lumice
