#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numeric>
#include <set>
#include <utility>
#include <vector>

#include "config/filter_config.hpp"
#include "config/proj_config.hpp"
#include "config/sim_data.hpp"
#include "core/crystal.hpp"
#include "core/filter_spec.hpp"
#include "core/geo3d.hpp"
#include "core/math.hpp"
#include "core/raypath.hpp"
#include "core/simulator.hpp"
#include "core/trace_ops.hpp"
#include "util/queue.hpp"

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
// CollectData filter dispatch (Design A — filter-fail = ray kill)
//
// Design A branch table for outgoing candidates:
//   filter-pass + prob-pass → IsContinue() (next MS scatter)
//   filter-pass + prob-fail → IsOutgoing() (emit)
//   filter-fail             → ray terminates (w_ set negative; neither outgoing nor continue)
// ============================================================================

class AlwaysRejectSpec : public FilterSpec {
 public:
  bool Match(const RaySeg& /*ray*/, const RaypathRecorder& /*rec*/, const uint8_t* /*arena*/) const override {
    return false;
  }
};

class AlwaysAcceptSpec : public FilterSpec {
 public:
  bool Match(const RaySeg& /*ray*/, const RaypathRecorder& /*rec*/, const uint8_t* /*arena*/) const override {
    return true;
  }
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

TEST(CollectDataFilterDispatch, FilterFailTerminatesRay) {
  RandomNumberGenerator rng(42);
  AlwaysRejectSpec filter;
  MsInfo ms_info;
  ms_info.prob_ = 0.0f;

  RayBuffer buffer_data[2];
  RayBuffer init_data[2];
  buffer_data[0].Reset(8);
  buffer_data[1].Reset(8);
  init_data[0].Reset(8);
  init_data[1].Reset(8);

  buffer_data[1].EmplaceBack(MakeOutgoingCandidate(), RaypathRecorder{});

  CollectData(rng, ms_info, &filter, buffer_data, init_data);

  ASSERT_EQ(buffer_data[1].size_, 1u);
  const auto& r = buffer_data[1].rays_[0];
  EXPECT_FALSE(r.IsOutgoing()) << "filter-fail must not reach outgoing";
  EXPECT_FALSE(r.IsContinue()) << "filter-fail must not continue";
  EXPECT_LT(r.w_, 0.0f) << "filter-fail terminates ray (w_ set negative)";
  EXPECT_EQ(init_data[1].size_, 0u);
}

TEST(CollectDataFilterDispatch, FilterFailTerminatesEvenWithProbPass) {
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

  buffer_data[1].EmplaceBack(MakeOutgoingCandidate(), RaypathRecorder{});

  CollectData(rng, ms_info, &filter, buffer_data, init_data);

  ASSERT_EQ(buffer_data[1].size_, 1u);
  const auto& r = buffer_data[1].rays_[0];
  EXPECT_FALSE(r.IsContinue()) << "Design A: filter-fail must not continue even with prob=1";
  EXPECT_FALSE(r.IsOutgoing());
  EXPECT_LT(r.w_, 0.0f);
  EXPECT_EQ(init_data[1].size_, 0u);
}

TEST(CollectDataFilterDispatch, FilterPassWithProbContinues) {
  RandomNumberGenerator rng(42);
  AlwaysAcceptSpec filter;
  MsInfo ms_info;
  ms_info.prob_ = 1.0f;

  RayBuffer buffer_data[2];
  RayBuffer init_data[2];
  buffer_data[0].Reset(8);
  buffer_data[1].Reset(8);
  init_data[0].Reset(8);
  init_data[1].Reset(8);

  buffer_data[1].EmplaceBack(MakeOutgoingCandidate(), RaypathRecorder{});

  CollectData(rng, ms_info, &filter, buffer_data, init_data);

  ASSERT_EQ(buffer_data[1].size_, 1u);
  EXPECT_TRUE(buffer_data[1].rays_[0].IsContinue());
  EXPECT_EQ(init_data[1].size_, 1u);
}

TEST(CollectDataFilterDispatch, FilterPassNoProbEmitsOutgoing) {
  RandomNumberGenerator rng(42);
  AlwaysAcceptSpec filter;
  MsInfo ms_info;
  ms_info.prob_ = 0.0f;

  RayBuffer buffer_data[2];
  RayBuffer init_data[2];
  buffer_data[0].Reset(8);
  buffer_data[1].Reset(8);
  init_data[0].Reset(8);
  init_data[1].Reset(8);

  buffer_data[1].EmplaceBack(MakeOutgoingCandidate(), RaypathRecorder{});

  CollectData(rng, ms_info, &filter, buffer_data, init_data);

  ASSERT_EQ(buffer_data[1].size_, 1u);
  EXPECT_TRUE(buffer_data[1].rays_[0].IsOutgoing());
  EXPECT_EQ(init_data[1].size_, 0u);
}

// ============================================================================
// ComponentMaskPropagation (task-331.1): T1 transport-only coverage for the
// per-ray component mask added to RayBuffer. All assertions here are about
// *transport* (reset / fan-out / pass-through), not production — the mask
// value is always either 0 or a hand-injected test marker; T2 will start
// setting real bits and the CollectData assertions below will need updating
// (see the inline notes at each one).
// ============================================================================

TEST(ComponentMaskPropagation, InitRayFirstMsZeroesComponentSlots) {
  Crystal crystal = Crystal::CreatePrism(1.0f);
  RandomNumberGenerator rng(42);
  SunParam sun{ 90.0f, 0.0f, 0.5f };
  WlParam wl{ 550.0f, 1.0f };
  AxisDistribution axis;

  RayBuffer buffer_data[2];
  buffer_data[0].Reset(8);
  buffer_data[1].Reset(8);
  RayBuffer all_data;
  all_data.Reset(16);

  // Pre-poison every slot so the "zeroed by InitRayFirstMs" assertion below is
  // load-bearing (not vacuously true because of a fresh-allocation zero-init).
  for (size_t i = 0; i < 8; i++) {
    buffer_data[0].SetComponent(i, 0xFFFFFFFFFFFFFFFFull);
  }

  constexpr size_t kRayNum = 4;
  InitRayFirstMs(rng, sun, wl, kRayNum, crystal, /*curr_crystal_id=*/0, axis, buffer_data, all_data);

  ASSERT_EQ(buffer_data[0].size_, kRayNum);
  for (size_t i = 0; i < kRayNum; i++) {
    EXPECT_EQ(buffer_data[0].ComponentAt(i), 0u) << "slot " << i << " not zeroed by InitRayFirstMs";
  }
}

TEST(ComponentMaskPropagation, TraceRayBasicInfoFanOutInheritsMask) {
  Crystal crystal = Crystal::CreatePrism(1.0f);
  float refractive_index = crystal.GetRefractiveIndex(550.0f);
  RandomNumberGenerator rng(7);
  SunParam sun{ 90.0f, 0.0f, 0.5f };
  WlParam wl{ 550.0f, 1.0f };
  AxisDistribution axis;

  RayBuffer buffer_data[2];
  buffer_data[0].Reset(4);
  buffer_data[1].Reset(4);
  RayBuffer all_data;
  all_data.Reset(16);

  constexpr size_t kRayNum = 1;
  InitRayFirstMs(rng, sun, wl, kRayNum, crystal, /*curr_crystal_id=*/0, axis, buffer_data, all_data);
  ASSERT_EQ(buffer_data[0].size_, kRayNum);

  // Inject a nonzero test value AFTER InitRayFirstMs (which resets to 0) —
  // this is the marker the fan-out below must propagate.
  constexpr uint64_t kMarker = 0x123456789abcdef0ull;
  buffer_data[0].SetComponent(0, kMarker);

  TraceRayBasicInfo(crystal, refractive_index, kRayNum, buffer_data);

  ASSERT_EQ(buffer_data[1].size_, kRayNum * 2);
  EXPECT_EQ(buffer_data[1].ComponentAt(0), kMarker) << "reflect child did not inherit parent mask";
  EXPECT_EQ(buffer_data[1].ComponentAt(1), kMarker) << "refract child did not inherit parent mask";
}

namespace {

// Build a RaySeg shaped like MakeOutgoingCandidate() but with to_face_ set to
// a real (non-sentinel) face id, so IsNormal() is true (to_face_ !=
// kInvalidId && w_ >= 0) instead of IsOutgoing()/IsContinue().
RaySeg MakeNormalCandidate() {
  RaySeg r{};
  r.d_[0] = 1.0f;
  r.d_[1] = 0.0f;
  r.d_[2] = 0.0f;
  r.p_[0] = 0.0f;
  r.p_[1] = 0.0f;
  r.p_[2] = 0.0f;
  r.w_ = 0.5f;  // positive (not TIR)
  r.from_face_ = kInvalidId;
  r.to_face_ = 3;  // non-sentinel face id -> IsNormal() candidate
  r.crystal_rot_ = Rotation{};
  return r;
}

}  // namespace

// T1 pass-through contract: CollectData copies buffer_data[1]'s mask verbatim
// into buffer_data[0] for IsNormal() rays (no OR-combine yet). T2 will change
// this to "new = old OR produced-bit" — when that lands, this assertion is
// EXPECTED to need updating (from plain equality to the OR relationship); a
// mismatch here after T2 lands is not, by itself, a regression.
TEST(ComponentMaskPropagation, CollectDataNormalBranchPassesThroughUnchanged) {
  RandomNumberGenerator rng(42);
  AlwaysAcceptSpec filter;
  MsInfo ms_info;
  ms_info.prob_ = 0.0f;

  RayBuffer buffer_data[2];
  RayBuffer init_data[2];
  buffer_data[0].Reset(8);
  buffer_data[1].Reset(8);
  init_data[0].Reset(8);
  init_data[1].Reset(8);

  buffer_data[1].EmplaceBack(MakeNormalCandidate(), RaypathRecorder{});
  constexpr uint64_t kMarker = 0x00ff00ff00ff00ffull;
  buffer_data[1].SetComponent(0, kMarker);

  CollectData(rng, ms_info, &filter, buffer_data, init_data);

  ASSERT_EQ(buffer_data[0].size_, 1u);
  ASSERT_TRUE(buffer_data[0].rays_[0].IsNormal());
  EXPECT_EQ(buffer_data[0].ComponentAt(0), kMarker)
      << "T1 pass-through: destination mask must equal source verbatim (no OR yet)";
}

// Mirror of the IsNormal() case above for the IsContinue() branch — same T1
// pass-through contract, same "will become OR-combine under T2" caveat.
TEST(ComponentMaskPropagation, CollectDataContinueBranchPassesThroughUnchanged) {
  RandomNumberGenerator rng(42);
  AlwaysAcceptSpec filter;
  MsInfo ms_info;
  ms_info.prob_ = 1.0f;  // force filter-pass + prob-pass -> IsContinue()

  RayBuffer buffer_data[2];
  RayBuffer init_data[2];
  buffer_data[0].Reset(8);
  buffer_data[1].Reset(8);
  init_data[0].Reset(8);
  init_data[1].Reset(8);

  buffer_data[1].EmplaceBack(MakeOutgoingCandidate(), RaypathRecorder{});
  constexpr uint64_t kMarker = 0xaaaabbbbccccddddull;
  buffer_data[1].SetComponent(0, kMarker);

  CollectData(rng, ms_info, &filter, buffer_data, init_data);

  ASSERT_EQ(init_data[1].size_, 1u);
  ASSERT_TRUE(buffer_data[1].rays_[0].IsContinue());
  EXPECT_EQ(init_data[1].ComponentAt(0), kMarker)
      << "T1 pass-through: destination mask must equal source verbatim (no OR yet)";
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

// --- Task 260.6: Simulator::effective_seed_ ----------------------------- //
// When the user-facing seed is 0 (default multi-worker random mode), the
// Simulator must hand the backend a non-zero, per-instance, pairwise-distinct
// seed so the Metal device-gen path activates (gated on `gen_seed_ != 0`).

TEST(SimulatorEffectiveSeed, ZeroSeedYieldsNonZero) {
  auto config_queue = std::make_shared<Queue<SimBatch>>();
  auto data_queue = std::make_shared<Queue<SimData>>();
  Simulator sim(config_queue, data_queue, /*seed=*/0);
  uint32_t derived = sim.GetEffectiveSeed();
  EXPECT_NE(derived, 0u);
  // Stable across repeated reads — required so backend's `seeded_` idempotency
  // gate stays consistent across BeginSession calls.
  EXPECT_EQ(derived, sim.GetEffectiveSeed());
}

TEST(SimulatorEffectiveSeed, FixedSeedPreserved) {
  auto config_queue = std::make_shared<Queue<SimBatch>>();
  auto data_queue = std::make_shared<Queue<SimData>>();
  Simulator sim(config_queue, data_queue, /*seed=*/42);
  EXPECT_EQ(sim.GetEffectiveSeed(), 42u);
}

TEST(SimulatorEffectiveSeed, TwoZeroSeedInstancesDistinct) {
  // Global atomic counter monotonically increments per construction; pairwise
  // distinctness is deterministic, not probabilistic.
  auto config_queue = std::make_shared<Queue<SimBatch>>();
  auto data_queue = std::make_shared<Queue<SimData>>();
  Simulator a(config_queue, data_queue, /*seed=*/0);
  Simulator b(config_queue, data_queue, /*seed=*/0);
  EXPECT_NE(a.GetEffectiveSeed(), b.GetEffectiveSeed());
  EXPECT_NE(a.GetEffectiveSeed(), 0u);
  EXPECT_NE(b.GetEffectiveSeed(), 0u);
}

// Regression sentinel that started life as a guard against the
// PolygonFaceOfTri argmax fix (a first-match `dot > 1-1e-3` collapsed ≥2
// upper-pyramid faces onto the lowest polygon index on ~≥87.4° wedge because
// adjacent upper-face normals dot to ~0.9994). Step 4 of the closed-form
// representation swap replaced the whole argmax reversal with a parametric
// slot→poly-face id carried directly by each fan sub-triangle
// (detail::EntrySubTri::face_id, derived from cf_geom_ presence). The original
// failure mode is *structurally* unreachable through the closed-form path —
// the map is not rediscovered per call. This sentinel is kept as a
// higher-level end-to-end assertion (extreme-wedge crystals expose 6 distinct
// upper-pyramid polygon faces to the CPU consumer surface) so a future
// refactor that regresses either (a) parametric face-number assignment or
// (b) the slot→poly-face table under move/copy still fails loudly.
//
// Identification: upper-pyramid faces carry Fn ∈ {13..18} (per
// crystal.cpp::CreatePyramid Miller-axis numbering).
constexpr IdType kUpperPyramidFnLo = 13;
constexpr IdType kUpperPyramidFnHi = 18;

size_t CountDistinctUpperPolyFaces(const Crystal& crystal, bool* any_invalid_out = nullptr) {
  // Rebuild the fan sub-triangles from cf_geom_ (the same helper the CPU/GPU
  // entry samplers use); each sub-triangle carries the compact present-face id
  // it belongs to (== the old PolygonFaceOfTri result).
  const CrystalGeom& cf = crystal.CfGeom();
  std::vector<detail::EntrySubTri> sub(detail::CountEntrySubTris(cf));
  if (!sub.empty()) {
    detail::BuildEntrySubTris(cf, sub.data());
  }
  std::set<IdType> polys;
  bool any_invalid = false;
  for (size_t t = 0; t < sub.size(); t++) {
    IdType poly = sub[t].face_id;
    if (poly == kInvalidId) {
      any_invalid = true;
      continue;
    }
    IdType fn = crystal.GetFn(poly);
    if (fn < kUpperPyramidFnLo || fn > kUpperPyramidFnHi) {
      continue;
    }
    polys.insert(poly);
  }
  if (any_invalid_out) {
    *any_invalid_out = any_invalid;
  }
  return polys.size();
}

TEST(PolygonFaceOfTriArgmax, ExtremeWedge88SixDistinctUpperFaces) {
  // Extreme wedge 88° — six distinct upper-pyramid polygon faces must be
  // exposed at the CPU consumer surface (parametric layout guarantee).
  auto crystal = Crystal::CreatePyramid(88.0f, 88.0f, 1.0f, 0.0f, 1.0f);
  bool any_invalid = false;
  size_t distinct = CountDistinctUpperPolyFaces(crystal, &any_invalid);
  EXPECT_EQ(distinct, 6u) << "extreme-wedge 88°: upper-pyramid triangles must map to 6 distinct "
                             "polygon faces (parametric slot→poly-face table)";
  EXPECT_FALSE(any_invalid) << "no upper-pyramid triangle should return kInvalidId on a "
                               "closed-form-built crystal";
}

TEST(PolygonFaceOfTriArgmax, NormalWedge87SixDistinctUpperFaces) {
  auto crystal = Crystal::CreatePyramid(87.0f, 87.0f, 1.0f, 0.0f, 1.0f);
  bool any_invalid = false;
  size_t distinct = CountDistinctUpperPolyFaces(crystal, &any_invalid);
  EXPECT_EQ(distinct, 6u);
  EXPECT_FALSE(any_invalid) << "no upper-pyramid triangle should return kInvalidId at wedge 87°";
}

// ---- InitRay_p_fid polygon-granularity sampler ----
//
// White-box self-proof of the compact present-face numbering that the rewritten
// InitRay_p_fid assigns to RaySeg::to_face_. The oracle suite
// (test/golden-analytic/core/test_incidence_sampling_polygon_oracle.cpp) already
// gates the sampled *distribution*; these are faster, direct checks that the
// compact id the sampler writes is the same numbering GetFn/poly_face_n_ use —
// the failure mode risk-4 warns about (a numbering drift silently indexes the
// wrong face normal downstream). Expectations are derived independently from
// CfGeom (slot ascending, skip absent), not from the sampler under test.
namespace {

// Drive the real production sampler for `n` rays, all with crystal-local
// direction `d`. Mirrors the oracle's DriveEntrySampling but kept local so this
// unit test does not depend on the golden-analytic support header.
std::vector<IdType> DriveToFaces(const Crystal& crystal, const float d[3], size_t n, uint32_t seed,
                                 std::vector<std::array<float, 3>>* points_out = nullptr) {
  RandomNumberGenerator::GetInstance().SetSeed(seed);
  RayBuffer buf(n);
  buf.size_ = n;
  for (size_t i = 0; i < n; i++) {
    RaySeg& r = buf[i];
    r.d_[0] = d[0];
    r.d_[1] = d[1];
    r.d_[2] = d[2];
    r.w_ = 1.0f;
    r.from_face_ = kInvalidId;
    r.to_face_ = kInvalidId;
  }
  InitRay_p_fid(crystal, &buf);
  std::vector<IdType> faces(n);
  if (points_out) {
    points_out->resize(n);
  }
  for (size_t i = 0; i < n; i++) {
    faces[i] = buf[i].to_face_;
    if (points_out) {
      (*points_out)[i] = { buf[i].p_[0], buf[i].p_[1], buf[i].p_[2] };
    }
  }
  return faces;
}

}  // namespace

// The compact numbering InitRay_p_fid assigns (slot ascending, skip absent) must
// agree with the numbering PopulateFromCfGeom fed into GetFn: for every present
// slot, GetFn(compact_id) must equal that slot's parametric face_number.
TEST(InitRayPolygonSampling, CompactFaceIdMatchesGetFn) {
  const float kUnitDist[6] = { 1, 1, 1, 1, 1, 1 };
  const float kDrop[6] = { 1.0f, 0.4f, 1.0f, 0.5f, 1.0f, 1.0f };  // known face-drop pyramid
  std::vector<std::pair<const char*, Crystal>> fixtures;
  fixtures.emplace_back("prism_h1.2", Crystal::CreatePrism(1.2f));
  fixtures.emplace_back("pyr_shoulder", Crystal::CreatePyramid(28.0f, 28.0f, 0.6f, 1.0f, 0.6f, kUnitDist));
  fixtures.emplace_back("pyr_drop", Crystal::CreatePyramid(30.0f, 30.0f, 1.0f, 0.5f, 1.0f, kDrop));

  for (auto& [label, crystal] : fixtures) {
    const CrystalGeom& cf = crystal.CfGeom();
    IdType compact = 0;
    for (int slot = 0; slot < cf.face_cnt; slot++) {
      if (!cf.face_present[slot]) {
        continue;
      }
      EXPECT_EQ(crystal.GetFn(compact), static_cast<IdType>(cf.face_number[slot]))
          << label << ": compact id " << compact << " (slot " << slot << ") face-number mismatch";
      compact++;
    }
    EXPECT_EQ(static_cast<size_t>(compact), crystal.PolygonFaceCount()) << label << ": present-count mismatch";
  }
}

// A direction that illuminates exactly one face: every ray must land on that
// face's compact id (in range, single-valued), the point must lie on that face's
// plane, and GetFn(to_face_) must equal the illuminated face's parametric number.
TEST(InitRayPolygonSampling, SingleFaceDirectionHitsExpectedFace) {
  Crystal crystal = Crystal::CreatePrism(1.2f);
  const CrystalGeom& cf = crystal.CfGeom();

  // Independently pick the "top basal" face: present slot whose outward normal
  // has the most-positive z. A ray travelling -z illuminates only it.
  int top_slot = -1;
  IdType top_compact = kInvalidId;
  IdType compact = 0;
  float best_nz = -2.0f;
  for (int slot = 0; slot < cf.face_cnt; slot++) {
    if (!cf.face_present[slot]) {
      continue;
    }
    const float nz = cf.face_normal[slot * 3 + 2];
    if (nz > best_nz) {
      best_nz = nz;
      top_slot = slot;
      top_compact = compact;
    }
    compact++;
  }
  ASSERT_GE(top_slot, 0);
  ASSERT_GT(best_nz, 0.9f) << "expected a basal face with near-+z normal";

  // Expected z of the top face (all its corners share it for a flat basal face).
  const float expect_z = cf.face_vtx[static_cast<size_t>(top_slot) * kCrystalGeomMaxVtxPerFace * 3 + 2];

  const float d[3] = { 0.0f, 0.0f, -1.0f };
  std::vector<std::array<float, 3>> pts;
  const std::vector<IdType> faces = DriveToFaces(crystal, d, 4000, 12345, &pts);

  for (size_t i = 0; i < faces.size(); i++) {
    ASSERT_NE(faces[i], kInvalidId);
    ASSERT_LT(faces[i], static_cast<IdType>(crystal.PolygonFaceCount()));
    EXPECT_EQ(faces[i], top_compact) << "ray " << i << " selected a non-illuminated face";
    EXPECT_NEAR(pts[i][2], expect_z, 1e-4f) << "sampled point off the top-basal plane";
  }
  EXPECT_EQ(crystal.GetFn(top_compact), static_cast<IdType>(cf.face_number[top_slot]));
}

// Across several directions and fixtures, every sampled to_face_ must be in range
// and correspond to a genuinely front-facing face (outward normal·d < 0) — a
// zero-/back-weight face must never be selected. Directly catches a numbering
// drift that would point to_face_ at the wrong (e.g. back) face.
TEST(InitRayPolygonSampling, SelectedFacesAreFrontFacing) {
  const float kUnitDist[6] = { 1, 1, 1, 1, 1, 1 };
  std::vector<std::pair<const char*, Crystal>> fixtures;
  fixtures.emplace_back("prism_h1.2", Crystal::CreatePrism(1.2f));
  fixtures.emplace_back("pyr_shoulder", Crystal::CreatePyramid(28.0f, 28.0f, 0.6f, 1.0f, 0.6f, kUnitDist));

  const float s = 1.0f / std::sqrt(3.0f);
  const std::vector<std::array<float, 3>> dirs = {
    { 0, 0, -1 }, { 0, 0, 1 }, { 1, 0, 0 }, { s, s, s }, { -s, s, -s },
  };

  for (auto& [label, crystal] : fixtures) {
    const CrystalGeom& cf = crystal.CfGeom();
    // Build compact-id → slot map (same rule the sampler uses).
    std::vector<int> compact_to_slot;
    for (int slot = 0; slot < cf.face_cnt; slot++) {
      if (cf.face_present[slot]) {
        compact_to_slot.push_back(slot);
      }
    }
    for (const auto& d : dirs) {
      const std::vector<IdType> faces = DriveToFaces(crystal, d.data(), 2000, 777, nullptr);
      for (IdType f : faces) {
        ASSERT_NE(f, kInvalidId) << label;
        ASSERT_LT(static_cast<size_t>(f), compact_to_slot.size()) << label;
        const int slot = compact_to_slot[f];
        const float* n = cf.face_normal + slot * 3;
        const float n_dot_d = n[0] * d[0] + n[1] * d[1] + n[2] * d[2];
        // Front-facing = outward normal opposes the ray direction. Allow a tiny
        // grazing margin (fan sub-tri normals of near-planar faces jitter ~1e-3).
        EXPECT_LT(n_dot_d, 1e-2f) << label << ": selected a back-facing face (compact " << f << ", slot " << slot
                                  << ", n·d=" << n_dot_d << ")";
      }
    }
  }
}

// A face with vtx_cnt>=3 (not the whole-face degenerate case already handled
// by CountEntrySubTris/BuildEntrySubTris skipping vtx_cnt<3) can still contain
// one collapsed fan sub-triangle if two of its corners coincide. Before the
// fix, Cross3 on the collapsed pair gave a zero vector, area correctly came
// out 0, but Normalize3 on that same zero vector produced NaN — poisoning the
// per-ray projected-weight table even though the sub-tri's area already
// should have zeroed its selection weight. Regression guard: the degenerate
// sub-tri's normal must be a finite zero, not NaN, and must not corrupt its
// non-degenerate neighbor.
TEST(InitRayPolygonSampling, DegenerateSubTriProducesFiniteZeroNotNaN) {
  CrystalGeom cf{};
  cf.face_cnt = 1;
  cf.face_present[0] = true;
  cf.face_vtx_cnt[0] = 4;
  // Corners 0 and 1 coincide -> fan sub-tri (0,1,2) collapses to zero area.
  // Corners 0,2,3 form a valid non-degenerate triangle.
  const float corners[4][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 1, 0, 0 }, { 1, 1, 0 } };
  for (int k = 0; k < 4; k++) {
    std::memcpy(cf.face_vtx + static_cast<size_t>(k) * 3, corners[k], 3 * sizeof(float));
  }

  const size_t subtri_cnt = detail::CountEntrySubTris(cf);
  ASSERT_EQ(subtri_cnt, 2u);
  std::vector<detail::EntrySubTri> subtri(subtri_cnt);
  detail::BuildEntrySubTris(cf, subtri.data());

  // Degenerate sub-tri (0,1,2): area exactly 0, normal finite zero (not NaN).
  EXPECT_FLOAT_EQ(subtri[0].area, 0.0f);
  for (float c : subtri[0].n) {
    ASSERT_TRUE(std::isfinite(c)) << "degenerate sub-tri produced a non-finite normal component";
    EXPECT_FLOAT_EQ(c, 0.0f);
  }
  EXPECT_EQ(subtri[0].face_id, 0);

  // Its non-degenerate neighbor (0,2,3) must be unaffected: positive area,
  // finite unit normal.
  EXPECT_GT(subtri[1].area, 0.0f);
  for (float c : subtri[1].n) {
    EXPECT_TRUE(std::isfinite(c));
  }
  EXPECT_EQ(subtri[1].face_id, 0);

  // The per-ray weight InitRay_p_fid actually computes from this sub-tri must
  // come out exactly 0.0f (not NaN) for any direction, so collapsed geometry
  // is silently discarded via zero weight rather than corrupting the sampler.
  const float d[3] = { 0.0f, 0.0f, -1.0f };
  const float w0 = std::max(-Dot3(d, subtri[0].n) * subtri[0].area, 0.0f);
  ASSERT_TRUE(std::isfinite(w0));
  EXPECT_FLOAT_EQ(w0, 0.0f);
}

}  // namespace
}  // namespace lumice
