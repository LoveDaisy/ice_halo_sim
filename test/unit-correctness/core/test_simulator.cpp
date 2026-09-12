#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <set>
#include <thread>
#include <utility>
#include <vector>

#include "config/filter_config.hpp"
#include "config/light_config.hpp"
#include "config/proj_config.hpp"
#include "config/sim_data.hpp"
#include "core/crystal.hpp"
#include "core/filter_spec.hpp"
#include "core/geo3d.hpp"
#include "core/math.hpp"
#include "core/raypath.hpp"
#include "core/shared/lat_path_selection.hpp"
#include "core/simulator.hpp"
#include "core/trace_ops.hpp"
#include "util/illuminant.hpp"
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

// ---- ResetHitLoopBuffers: the hit-loop buffer-pair capacity contract ----
//
// buffer_data[0] holds a hit's input rays; TraceRayBasicInfo fans each of them
// into two slots of buffer_data[1]. RayBuffer::EmplaceBack's `size_ + 1 <
// capacity_` guard is what bounds buffer_data[0].size_, so the contract that
// keeps the fan-out in bounds is
//
//     buffer_data[1].capacity_ >= 2 * (buffer_data[0].capacity_ - 1)
//
// stated against the CAPACITIES, not against the ray_num of the current batch.
// These cases are the deterministic half of the guard: the e2e sentinel over the
// reported scene can only trip when a ray actually violates the "at most one
// normal child per parent" assumption, which is a stochastic event, whereas the
// contract itself is checkable directly and always.
size_t MaxFanOutSlots(const RayBuffer buffer_data[2]) {
  // EmplaceBack fills at most capacity_ - 1 slots; each of those fans out to 2.
  return 2 * (buffer_data[0].capacity_ - 1);
}

TEST(ResetHitLoopBuffers, SecondBufferAbsorbsFullFanOut) {
  for (size_t ray_num : { size_t{ 1 }, size_t{ 2 }, size_t{ 8 }, size_t{ 24 }, size_t{ 32 }, size_t{ 40 },
                          size_t{ 128 }, size_t{ 1024 } }) {
    RayBuffer buffer_data[2];
    ResetHitLoopBuffers(buffer_data, ray_num);
    EXPECT_GE(buffer_data[1].capacity_, MaxFanOutSlots(buffer_data))
        << "ray_num=" << ray_num << ": buffer_data[1] cannot absorb a full buffer_data[0] fan-out";
  }
}

TEST(ResetHitLoopBuffers, ContractHoldsAcrossShrinkingBatches) {
  // RayBuffer::Reset is grow-never-shrink, so a workspace recycled from a large
  // batch keeps the large capacity_. The contract must therefore be stated
  // against buffer_data[0]'s ACTUAL capacity, not against the current batch's
  // ray_num — a formula that recomputes `ray_num * 2` for both buffers leaves
  // buffer_data[0] oversized from the earlier batch while buffer_data[1] is
  // sized for the small one.
  RayBuffer buffer_data[2];
  ResetHitLoopBuffers(buffer_data, 1024);
  const size_t grown_capacity = buffer_data[0].capacity_;
  ResetHitLoopBuffers(buffer_data, 8);
  EXPECT_EQ(buffer_data[0].capacity_, grown_capacity) << "Reset is expected to be grow-never-shrink";
  EXPECT_GE(buffer_data[1].capacity_, MaxFanOutSlots(buffer_data));
}

TEST(ResetHitLoopBuffers, InputBufferStillHoldsTheBatch) {
  // Guard against "fix" the contract by shrinking buffer_data[0]: it must still
  // be able to hold the batch it is given (with the one slot EmplaceBack keeps
  // free), or rays would be dropped at the hit-loop entry instead.
  for (size_t ray_num : { size_t{ 1 }, size_t{ 32 }, size_t{ 128 } }) {
    RayBuffer buffer_data[2];
    ResetHitLoopBuffers(buffer_data, ray_num);
    EXPECT_GT(buffer_data[0].capacity_, ray_num) << "ray_num=" << ray_num;
  }
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

// --- Orientation-count predicate: behavioral pin + config aggregation ---

// Sample `n` orientations for `axis` through the production path and return
// the raw 3x3 matrices, so a caller can compare them bit-for-bit.
std::vector<std::array<float, 9>> SampleOrientations(const AxisDistribution& axis, size_t n, uint32_t seed) {
  RandomNumberGenerator rng(seed);
  RayBuffer buffer_data[2];
  buffer_data[0].Reset(n);
  buffer_data[1].Reset(n);
  buffer_data[0].size_ = n;
  InitRay_rot(rng, axis, buffer_data);

  std::vector<std::array<float, 9>> out;
  out.reserve(n);
  for (size_t i = 0; i < n; i++) {
    std::array<float, 9> m{};
    const float* mat = buffer_data[0].rays_[i].crystal_rot_.GetMat();
    std::copy(mat, mat + 9, m.begin());
    out.push_back(m);
  }
  return out;
}

// The pin AxisDistribution::IsAxisDeterministic's doc comment names. It binds
// the predicate to what InitRay_rot ACTUALLY PRODUCES, not to another pure
// predicate: an assertion like "a full-sphere axis is not axis-deterministic"
// reads the same dist.type fields both predicates read, so it is true by
// construction and cannot fail.
//
// Scope, established by mutation rather than assumed. This catches a change that
// makes kNoRandom stop meaning "no draw" at runtime — jittering the kNoRandom
// latitude branch turns the first case red while all five truth-table cases in
// test_math.cpp stay green, which is the gap this test exists to fill. It does
// NOT catch InitRay_rot's full_sphere branch being decoupled from
// IsFullSphereUniform.
//
// This comment used to justify that last sentence by asserting the two sampler
// paths were "statistically equivalent since the unified area-measure LatLut, so
// nothing observable changes", and told the reader not to weaken these
// assertions into a distributional check because "there is no difference to
// detect". That was wrong. The difference is real and lives in roll: the
// full_sphere fast path never applies the pole-crossing `roll += pi` that the
// LatLut path applies on a flip. The measurement the old claim rested on — the
// fraction of samples with |world z| < 0.5 — reads the sampled axis direction
// only, so it is structurally blind to roll and would have reported agreement no
// matter how far apart the two paths were (a16: the yardstick has to reach the
// quantity under test). RollFlipDivergence.* below measures roll itself.
TEST(AxisDeterminismMatchesRuntimeOrientation, DeterministicAxisYieldsOneFixedRotation) {
  AxisDistribution axis;  // default ctor: all three kNoRandom
  ASSERT_TRUE(axis.IsAxisDeterministic());

  constexpr size_t kN = 16;
  auto mats = SampleOrientations(axis, kN, /*seed=*/1234);
  ASSERT_EQ(mats.size(), kN);
  for (size_t i = 1; i < kN; i++) {
    EXPECT_EQ(mats[i], mats[0]) << "ray " << i << " got a different rotation from a deterministic axis";
  }
}

TEST(AxisDeterminismMatchesRuntimeOrientation, FullSphereAxisIsStochasticAndVaries) {
  // roll stays kNoRandom on purpose: this is the combination most likely to be
  // misjudged by a predicate that stops short of all three fields. It used to be
  // misjudged — this assertion was ASSERT_TRUE, which is exactly how the defect
  // this test now guards was pinned as expected behavior. An axis whose roll is
  // fixed cannot take the fast path: that path drops the pole-crossing
  // `roll += pi`, so on the ~50% of rays that cross a pole it produces a crystal
  // rotated a further 180 deg about its own c axis relative to what the config
  // asks for. See RollFlipDivergence.* below for the measurement.
  AxisDistribution axis;
  axis.azimuth_dist = { DistributionType::kUniform, 0.0f, 360.0f };
  axis.latitude_dist = { DistributionType::kUniform, 90.0f, 360.0f };
  ASSERT_FALSE(axis.IsFullSphereUniform());
  EXPECT_FALSE(axis.IsAxisDeterministic());

  constexpr size_t kN = 16;
  auto mats = SampleOrientations(axis, kN, /*seed=*/1234);
  ASSERT_EQ(mats.size(), kN);
  size_t distinct_from_first = 0;
  for (size_t i = 1; i < kN; i++) {
    if (mats[i] != mats[0]) {
      distinct_from_first++;
    }
  }
  EXPECT_EQ(distinct_from_first, kN - 1) << "predicate says stochastic but InitRay_rot repeated a rotation";
}

// --- RollFlipDivergence: the yardstick that can see roll ---
//
// Why this suite exists. IsFullSphereUniform dispatches InitRay_rot to a fast
// path that samples the axis direction directly and never applies the
// pole-crossing correction detail::NormalizeLatitude reports; the general
// kLutInverseCdf path adds pi to BOTH azimuth and roll whenever it flips. The
// two therefore agree on the axis DIRECTION under any roll, and disagree on the
// crystal's rotation about that axis unless roll's own distribution is
// unchanged by a +180 deg shift. A criterion built on the sampled direction
// (the fraction of |world z| < 0.5 that the comments here used to cite) cannot
// separate them at all. These cases read roll itself.
//
// Recovering roll. BuildCrystalRotation composes
//   M = Rz(azimuth - pi) * Ry(latitude - pi/2) * Rz(roll),
// a ZYZ Euler triple whose third row is
//   (-sin(b)*cos(roll), sin(b)*sin(roll), cos(b)),  b = latitude - pi/2.
// So atan2(M[2][1], -M[2][0]) recovers roll up to a constant pi offset --
// b ranges over [-pi, 0], where sin(b) <= 0, so the sign never varies mid-run.
// Every assertion below is about how the recovered values CLUSTER, never about
// an absolute angle, so that constant offset is immaterial.
//
// Near the poles sin(b) -> 0 and the recovery is ill-conditioned (roll and
// azimuth become the same rotation there). Samples inside that band are
// dropped; the retained fraction is asserted so a change that pushed most
// samples to the poles could not quietly empty the measurement.
constexpr float kPoleGuardSinB = 0.05f;

// Recovered roll angles for `n` production-sampled orientations, one entry per
// ray outside the pole guard band.
std::vector<float> SampleRecoveredRolls(const AxisDistribution& axis, size_t n, uint32_t seed) {
  auto mats = SampleOrientations(axis, n, seed);
  std::vector<float> out;
  out.reserve(mats.size());
  for (const auto& m : mats) {
    const float m20 = m[6];
    const float m21 = m[7];
    if (std::hypot(m20, m21) <= kPoleGuardSinB) {
      continue;
    }
    out.push_back(std::atan2(m21, -m20));
  }
  return out;
}

// The defect the user reported, at the smallest scale that still shows it: an
// axis whose azimuth and latitude span the full sphere but whose roll is fixed.
// Before IsFullSphereUniform grew its roll conjunct this axis took the fast
// path, and both assertions below failed -- the path was kFullSphere, and every
// ray came back with the same roll instead of the half-and-half split the config
// actually describes. Latitude is uniform over 360 deg, i.e. it spans the far
// side of the sphere too, so detail::NormalizeLatitude reflects roughly half the
// draws back and the general path adds pi to their roll.
TEST(RollFlipDivergence, FixedRollAxisTakesLutPathAndSplitsRollAcrossHalfTurn) {
  AxisDistribution axis;
  axis.azimuth_dist = { DistributionType::kUniform, 0.0f, 360.0f };
  axis.latitude_dist = { DistributionType::kUniform, 90.0f, 360.0f };
  // roll_dist stays the kNoRandom default: one fixed roll for every ray.

  ASSERT_EQ(lat_path::SelectLatPath(axis).kind, lat_path::LatPathKind::kLutInverseCdf)
      << "an axis with a fixed roll must not take the fast path: that path drops the pole-crossing roll += pi";

  constexpr size_t kN = 2000;
  auto rolls = SampleRecoveredRolls(axis, kN, /*seed=*/20260829);
  ASSERT_GT(rolls.size(), kN * 95 / 100) << "pole guard dropped an implausible share of the samples";

  // rng.Get on a kNoRandom roll returns the same constant every time, so the
  // only thing that can move roll is the flip: the recovered angles must sit on
  // exactly two points half a turn apart, not spread between them.
  size_t upper = 0;
  size_t lower = 0;
  size_t off_cluster = 0;
  float worst_c = 1.0f;
  for (float roll : rolls) {
    const float c = std::cos(roll);
    if (std::abs(c) <= 0.99f) {
      off_cluster++;
      worst_c = std::min(worst_c, std::abs(c));
    }
    (c > 0.0f ? upper : lower)++;
  }
  EXPECT_EQ(off_cluster, 0u) << off_cluster << " recovered rolls sit between the two discrete values (worst |cos| "
                             << worst_c << ")";
  const double upper_frac = static_cast<double>(upper) / static_cast<double>(rolls.size());
  // A latitude uniform over the full 360 deg crosses a pole on half its draws,
  // so the split is 50/50. The window is ~4.5 sigma wide at this sample count.
  EXPECT_GT(upper_frac, 0.45) << "roll flip fires too rarely (" << upper << " of " << rolls.size() << ")";
  EXPECT_LT(upper_frac, 0.55) << "roll flip fires too often (" << upper << " of " << rolls.size() << ")";
  EXPECT_EQ(upper + lower, rolls.size());
}

// The positive half of the same proposition, and the reason the fast path is
// kept rather than deleted: when roll is uniform over a full turn, adding pi to
// half the rays is invisible in distribution, so dropping the correction costs
// nothing and IsFullSphereUniform still routes here.
TEST(RollFlipDivergence, UniformRollAxisKeepsFastPathAndRollStaysUniform) {
  AxisDistribution axis;
  axis.azimuth_dist = { DistributionType::kUniform, 0.0f, 360.0f };
  axis.latitude_dist = { DistributionType::kUniform, 90.0f, 360.0f };
  axis.roll_dist = { DistributionType::kUniform, 0.0f, 360.0f };

  ASSERT_TRUE(axis.IsFullSphereUniform());
  ASSERT_EQ(lat_path::SelectLatPath(axis).kind, lat_path::LatPathKind::kFullSphere);

  constexpr size_t kN = 4000;
  auto rolls = SampleRecoveredRolls(axis, kN, /*seed=*/20260829);
  ASSERT_GT(rolls.size(), kN * 95 / 100) << "pole guard dropped an implausible share of the samples";

  constexpr size_t kBins = 8;
  std::array<size_t, kBins> hist{};
  for (float roll : rolls) {
    float t = roll;
    while (t < 0.0f) {
      t += 2.0f * math::kPi;
    }
    auto bin = static_cast<size_t>(t / (2.0f * math::kPi) * kBins);
    hist[std::min(bin, kBins - 1)]++;
  }
  // Uniform over 8 bins is 0.125 each; sigma is ~0.005 at this sample count, so
  // the window below is wide enough to be stable and narrow enough that a roll
  // pinned to one or two values (the failure mode this guards) blows straight
  // through it.
  for (size_t i = 0; i < kBins; i++) {
    const double frac = static_cast<double>(hist[i]) / static_cast<double>(rolls.size());
    EXPECT_GT(frac, 0.10) << "roll bin " << i << " underfilled: " << hist[i];
    EXPECT_LT(frac, 0.15) << "roll bin " << i << " overfilled: " << hist[i];
  }
}

TEST(DeterministicOrientationCountTest, CountsAxisDeterministicSlotsAcrossLayers) {
  auto make_setting = [](bool axis_random) {
    ScatteringSetting s;
    s.crystal_.id_ = 0;
    s.crystal_.param_ = PrismCrystalParam{};
    if (axis_random) {
      s.crystal_.axis_.azimuth_dist = { DistributionType::kUniform, 0.0f, 360.0f };
    }
    s.crystal_proportion_ = 1.0f;
    return s;
  };

  SceneConfig config{};
  MsInfo layer0;
  layer0.prob_ = 0.0f;
  layer0.setting_.push_back(make_setting(/*axis_random=*/false));
  layer0.setting_.push_back(make_setting(/*axis_random=*/true));
  layer0.setting_.push_back(make_setting(/*axis_random=*/false));
  MsInfo layer1;
  layer1.prob_ = 0.0f;
  layer1.setting_.push_back(make_setting(/*axis_random=*/true));
  layer1.setting_.push_back(make_setting(/*axis_random=*/false));
  config.ms_.push_back(std::move(layer0));
  config.ms_.push_back(std::move(layer1));

  // 3 of the 5 (layer, ci) slots carry a deterministic axis, across BOTH layers.
  EXPECT_EQ(DeterministicOrientationCount(config), 3u);
}

TEST(DeterministicOrientationCountTest, IsIndependentOfTheShapePredicate) {
  // The task in one assertion: the commonest halo setup is deterministic on the
  // shape axis and stochastic on the orientation axis. Wiring the orientation
  // count to IsDeterministic(CrystalParam) would make these two counts equal.
  ScatteringSetting s;
  s.crystal_.id_ = 0;
  s.crystal_.param_ = PrismCrystalParam{};  // fixed shape
  s.crystal_.axis_.azimuth_dist = { DistributionType::kUniform, 0.0f, 360.0f };
  s.crystal_proportion_ = 1.0f;

  SceneConfig config{};
  MsInfo layer;
  layer.prob_ = 0.0f;
  layer.setting_.push_back(std::move(s));
  config.ms_.push_back(std::move(layer));

  EXPECT_EQ(DeterministicCrystalCount(config), 1u) << "shape is fixed";
  EXPECT_EQ(DeterministicOrientationCount(config), 0u) << "axis is random";
}

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

TEST(UpperPyramidPolyFaceCoverage, ExtremeWedge88SixDistinctUpperFaces) {
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

TEST(UpperPyramidPolyFaceCoverage, NormalWedge87SixDistinctUpperFaces) {
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

// --- Emitted energy: the absolute-normalization denominator ---------------- //
//
// Simulator is the sole producer of SimData::emitted_energy_, and it writes it
// on four different code paths. The end-to-end oracle downstream can only say
// the total came out wrong, not which path wrote it — so drive Run() directly
// and check the total against a weight sum that is known before the simulation
// starts. It is knowable in advance precisely because emitted energy is an a
// priori quantity: it depends on the source and the ray budget, never on what
// the rays go on to do.
//
// A discrete two-wavelength spectrum is the case that has teeth: the two
// WlParams carry different weights, so a path that charged the wrong wavelength
// (or charged one twice) lands on a different total than the right one, which a
// single-wavelength scene could not distinguish.

namespace {

SceneConfig MakeTwoWavelengthScene(const std::vector<WlParam>& spectrum) {
  SceneConfig scene;
  scene.ray_num_ = 0;
  scene.max_hits_ = 4;
  scene.light_source_.param_ = SunParam{ 30.0f, 0.0f, 0.5f };
  scene.light_source_.spectrum_ = spectrum;

  MsInfo ms;
  ms.prob_ = 0.0f;
  ScatteringSetting s;
  s.crystal_.id_ = 0;
  PrismCrystalParam prism;
  prism.h_ = Distribution{ DistributionType::kNoRandom, 1.0f, 0.0f };
  for (auto& d : prism.d_) {
    d = Distribution{ DistributionType::kNoRandom, 1.0f, 0.0f };
  }
  s.crystal_.param_ = prism;
  s.crystal_proportion_ = 1.0f;
  ms.setting_.push_back(std::move(s));
  scene.ms_.push_back(std::move(ms));
  return scene;
}

// Drive Run() over `batches` batches of `ray_num` rays each and return the sum
// of emitted_energy_ (and of root_ray_count_, its unweighted sibling) over every
// SimData that came out.
struct EmittedTotals {
  double emitted = 0.0;
  size_t root_rays = 0;
  size_t sim_data_count = 0;
};

EmittedTotals RunAndSumEmitted(const SceneConfig& scene, size_t ray_num, size_t batches) {
  auto config_queue = std::make_shared<Queue<SimBatch>>();
  auto data_queue = std::make_shared<Queue<SimData>>();
  Simulator sim(config_queue, data_queue, /*seed=*/1234);

  auto shared_scene = std::make_shared<const SceneConfig>(scene);
  for (size_t b = 0; b < batches; ++b) {
    SimBatch batch;
    batch.ray_num_ = ray_num;
    batch.scene_ = shared_scene;
    batch.generation_ = 1;
    config_queue->Emplace(std::move(batch));
  }
  config_queue->Emplace(SimBatch{});  // ray_num_ == 0 → Run() exits

  std::thread runner([&] { sim.Run(); });
  runner.join();

  // Drain by emptiness, NOT by shutting the queue down first: Queue::Shutdown
  // discards whatever is still queued, which here is the entire measurement.
  EmittedTotals totals;
  while (!data_queue->Empty()) {
    auto data = data_queue->Get();
    totals.emitted += data.emitted_energy_;
    totals.root_rays += data.root_ray_count_;
    totals.sim_data_count++;
  }
  return totals;
}

}  // namespace

TEST(SimulatorEmittedEnergy, DiscreteSpectrumChargesEachWavelengthItsOwnWeight) {
  // Run() calls SimulateOneWavelength once per WlParam per batch, so over
  // `batches` batches of `ray_num` rays the source emitted
  //     Σ_wavelengths weight · ray_num · batches
  // and nothing about the crystals, the filter, or the geometry may change it.
  constexpr size_t kRayNum = 512;
  constexpr size_t kBatches = 3;
  const std::vector<WlParam> kSpectrum = { { 450.0f, 0.25f }, { 650.0f, 1.75f } };

  auto totals = RunAndSumEmitted(MakeTwoWavelengthScene(kSpectrum), kRayNum, kBatches);

  double expected = 0.0;
  for (const auto& wl : kSpectrum) {
    expected += static_cast<double>(wl.weight_) * static_cast<double>(kRayNum) * static_cast<double>(kBatches);
  }
  ASSERT_GT(totals.sim_data_count, 0u) << "no SimData produced — the probe measured nothing";
  EXPECT_NEAR(totals.emitted, expected, expected * 1e-5);

  // Its unweighted sibling must agree on the ray count, which is what pins the
  // two to the same aggregation grain. If a future path accumulates one and
  // overwrites the other, the ratio moves off the weight mean and this fails.
  EXPECT_EQ(totals.root_rays, kRayNum * kBatches * kSpectrum.size());
}

TEST(SimulatorEmittedEnergy, EmittedEnergyIsProportionalToTheRayBudget) {
  // Doubling the rays doubles the energy emitted. Stated as a ratio so it holds
  // without knowing the absolute weights — the guard against a path that writes
  // a per-batch constant instead of weight × ray_num.
  const std::vector<WlParam> kSpectrum = { { 550.0f, 1.0f } };
  auto small = RunAndSumEmitted(MakeTwoWavelengthScene(kSpectrum), 256, 1);
  auto large = RunAndSumEmitted(MakeTwoWavelengthScene(kSpectrum), 512, 1);

  ASSERT_GT(small.emitted, 0.0);
  EXPECT_NEAR(large.emitted / small.emitted, 2.0, 1e-4);
}

TEST(SimulatorEmittedEnergy, IlluminantChargesTheBandExpectationNotTheSampledWeight) {
  // An illuminant batch draws its wavelength at random, and the SPD weight of
  // that draw varies by ~20% batch to batch. The charge must be the band
  // expectation instead, so that the same config at two different seeds
  // normalizes identically — otherwise "absolute" would still wobble with the
  // seed. Assert the exact expectation, which also pins that the sampled weight
  // is NOT what is being summed (it would land somewhere else with probability
  // essentially one).
  constexpr size_t kRayNum = 256;
  constexpr size_t kBatches = 4;
  auto scene = MakeTwoWavelengthScene({});
  scene.light_source_.spectrum_ = IlluminantType::kD65;

  auto totals = RunAndSumEmitted(scene, kRayNum, kBatches);

  const double expected = static_cast<double>(MeanIlluminantWeight(IlluminantType::kD65)) *
                          static_cast<double>(kRayNum) * static_cast<double>(kBatches);
  ASSERT_GT(totals.sim_data_count, 0u);
  EXPECT_NEAR(totals.emitted, expected, expected * 1e-5);
}

}  // namespace

// --- Ray allocation: dealing by q while charging by p ---------------------- //
//
// Under scene.ray_allocation = adaptive a layer deals its rays by the delivered
// q_i (ScatteringSetting::crystal_ray_alloc_weight_) instead of by p_i, and every
// ray born into entry i is scaled by (p_i/ΣP)/(q_i/ΣQ) so that the expected image
// is unchanged. Three propositions are pinned here, all on the legacy CPU path
// (Simulator::Run with no renders_, so no backend can be selected):
//   1. the correction formula itself (normalized shares, not raw ratios);
//   2. Σ_i n_i · w · correction_i == N · w up to the partition's ±1-ray rounding,
//      read off the RAW root segments (every ray's w_ at birth) AND off the
//      emitted_energy_ the batch charges — the two must agree, and both must
//      equal the a-priori budget;
//   3. q == p under adaptive is BIT-IDENTICAL to proportional — the strongest
//      "the new path has no hidden side effect" statement available without a
//      pilot to deliver a real q, and the one the mode's default rests on.
// The multi-layer case pins that a continuation layer's correction is its own
// (AC4) and that continuation hops are not charged as emissions (AC3).

TEST(RayAllocationCorrection, EqualVectorsAtAnyScaleAreIdentity) {
  // q == p exactly.
  auto c = ComputeRayAllocationCorrection({ 100.0f, 100.0f, 0.2f }, { 100.0f, 100.0f, 0.2f });
  ASSERT_EQ(c.size(), 3u);
  for (float v : c) {
    EXPECT_EQ(v, 1.0f);
  }
  // Same shares, different scale: still identity — the shares are what count.
  c = ComputeRayAllocationCorrection({ 1.0f, 1.0f }, { 10.0f, 10.0f });
  EXPECT_EQ(c[0], 1.0f);
  EXPECT_EQ(c[1], 1.0f);
}

TEST(RayAllocationCorrection, ComparesNormalizedSharesNotRawRatios) {
  // p shares (1/2, 1/2); q shares (3/4, 1/4) → corrections (2/3, 2). A raw
  // p_i/q_i would read (1/30, 1/10) here: the q vector is deliberately on a
  // different scale from p, which is exactly the input a pilot might deliver.
  auto c = ComputeRayAllocationCorrection({ 1.0f, 1.0f }, { 30.0f, 10.0f });
  ASSERT_EQ(c.size(), 2u);
  EXPECT_NEAR(c[0], 2.0f / 3.0f, 1e-6f);
  EXPECT_NEAR(c[1], 2.0f, 1e-6f);
  // Σ share_q · correction == 1: the weighted mean of the corrections over the
  // dealt shares is what makes the total emitted weight come out at N·w.
  EXPECT_NEAR(0.75 * c[0] + 0.25 * c[1], 1.0, 1e-6);
}

TEST(RayAllocationCorrection, UndealtEntriesStayFiniteAndNegativesAreClamped) {
  // q_1 == 0 with p_1 > 0: the entry is dealt no rays, so the slot is never read
  // — it must simply not be NaN. p_0 == 0 with q_0 > 0 is the mirror case (a
  // switched-off crystal handed a weight): finite, zero, harmless.
  auto c = ComputeRayAllocationCorrection({ 0.0f, 1.0f, 1.0f }, { 1.0f, 0.0f, 1.0f });
  ASSERT_EQ(c.size(), 3u);
  EXPECT_TRUE(std::isfinite(c[0]));
  EXPECT_EQ(c[0], 0.0f);
  EXPECT_EQ(c[1], 1.0f);
  // Entry 2: p share 1/2 (of the clamped ΣP = 2), q share 1/2 (ΣQ = 2) → 1.
  EXPECT_TRUE(std::isfinite(c[2]));
  EXPECT_NEAR(c[2], 1.0f, 1e-6f);
  // Negative entries are clamped to 0 on both sides, matching PartitionCrystalRayNum.
  c = ComputeRayAllocationCorrection({ -5.0f, 1.0f }, { -5.0f, 1.0f });
  EXPECT_EQ(c[0], 1.0f);
  EXPECT_EQ(c[1], 1.0f);
  // Degenerate: nothing dealt at all. Finite, no throw.
  c = ComputeRayAllocationCorrection({ 0.0f, 0.0f }, { 0.0f, 0.0f });
  EXPECT_EQ(c[0], 1.0f);
  EXPECT_EQ(c[1], 1.0f);
}

TEST(RayAllocationCorrection, AllZeroEnergyShareWithDealtRaysIsZeroNotIdentity) {
  // total_p <= 0 while total_q > 0: every entry's energy share is zero (a
  // proportional-fallback layer whose crystal_proportion_ is entirely 0) yet
  // some entries were dealt real rays (a 537.3 q epsilon floor can raise q
  // above 0 without touching p). Those dealt entries must contribute zero
  // energy — the same "p_i == 0 ⇒ correction == 0" rule already pinned above
  // for the single-entry case — not the 1.0f identity, which would inject the
  // nominal weight for entries the scene declared as carrying none.
  auto c = ComputeRayAllocationCorrection({ 0.0f, 0.0f }, { 1.0f, 3.0f });
  ASSERT_EQ(c.size(), 2u);
  EXPECT_EQ(c[0], 0.0f);
  EXPECT_EQ(c[1], 0.0f);
}

namespace {

// Deterministic hexagonal prism of height `h`; two different heights make two
// distinguishable crystals without any of them being stochastic.
ScatteringSetting MakePrismEntry(IdType id, float h, float proportion, float alloc_weight) {
  ScatteringSetting s;
  // FilterConfig has no default member initializers: leave it default-initialized and
  // action_ is stack garbage, which can read as kFilterOut and silently terminate every ray.
  s.filter_ = FilterConfig{ kInvalidId, FilterConfig::kSymNone, FilterConfig::kFilterIn, NoneFilterParam{} };
  s.crystal_.id_ = id;
  PrismCrystalParam prism;
  prism.h_ = Distribution{ DistributionType::kNoRandom, h, 0.0f };
  for (auto& d : prism.d_) {
    d = Distribution{ DistributionType::kNoRandom, 1.0f, 0.0f };
  }
  s.crystal_.param_ = prism;
  s.crystal_proportion_ = proportion;
  s.crystal_ray_alloc_weight_ = alloc_weight;
  return s;
}

// Two-entry single layer: p = (1, 1), q as given (-1 = not delivered).
SceneConfig MakeTwoEntryScene(SceneConfig::RayAllocationMode mode, float q0, float q1, float prob = 0.0f) {
  SceneConfig scene;
  scene.ray_num_ = 0;
  scene.max_hits_ = 4;
  scene.ray_allocation_ = mode;
  scene.light_source_.param_ = SunParam{ 30.0f, 0.0f, 0.5f };
  scene.light_source_.spectrum_ = std::vector<WlParam>{ { 550.0f, 1.0f } };
  MsInfo ms;
  ms.prob_ = prob;
  ms.setting_.push_back(MakePrismEntry(0, 1.0f, 1.0f, q0));
  ms.setting_.push_back(MakePrismEntry(1, 0.3f, 1.0f, q1));
  scene.ms_.push_back(std::move(ms));
  return scene;
}

struct AllocRunOutput {
  std::vector<RayBuffer> all_data;  // one snapshot per batch
  std::vector<SimData> batches;
};

void SnapshotAllDataForAlloc(void* ctx, const RayBuffer& all_data) {
  static_cast<std::vector<RayBuffer>*>(ctx)->emplace_back(all_data);
}

AllocRunOutput RunLegacy(const SceneConfig& scene, size_t ray_num, size_t batches, uint32_t seed) {
  auto config_queue = std::make_shared<Queue<SimBatch>>();
  auto data_queue = std::make_shared<Queue<SimData>>();
  Simulator sim(config_queue, data_queue, seed);
  AllocRunOutput out;
  sim.SetAllDataObserverForTest(&SnapshotAllDataForAlloc, &out.all_data);
  auto shared_scene = std::make_shared<const SceneConfig>(scene);
  for (size_t b = 0; b < batches; ++b) {
    SimBatch batch;
    batch.ray_num_ = ray_num;
    batch.scene_ = shared_scene;
    batch.generation_ = 1;
    config_queue->Emplace(std::move(batch));
  }
  config_queue->Emplace(SimBatch{});  // ray_num_ == 0 → Run() exits
  std::thread runner([&] { sim.Run(); });
  runner.join();
  while (!data_queue->Empty()) {
    out.batches.push_back(data_queue->Get());
  }
  return out;
}

// A root segment is the one InitRay_p_fid stamped with no source face; every
// traced child carries its parent's hit face as from_face_.
bool IsRootSegment(const RaySeg& r) {
  return r.from_face_ == kInvalidId && !r.is_continue_ && r.w_ >= 0.0f;
}

struct RootTally {
  std::map<IdType, size_t> count;   // by crystal_idx_ (== per-(layer,ci) crystal instance here)
  std::map<IdType, double> weight;  // Σ w_ at birth, by the same key
};

RootTally TallyRoots(const RayBuffer& all_data) {
  RootTally t;
  for (size_t i = 0; i < all_data.size_; i++) {
    const auto& r = all_data[i];
    if (IsRootSegment(r)) {
      t.count[r.crystal_idx_]++;
      t.weight[r.crystal_idx_] += r.w_;
    }
  }
  return t;
}

}  // namespace

TEST(RayAllocationLegacyPath, AdaptiveDealsByQAndChargesByP) {
  // p = (1, 1), q = (1, 2): the partition must deal 1000 rays as (333, 667) —
  // the partition's own rounding of (333.3, 666.7) — and the corrections are
  // (0.5/(1/3), 0.5/(2/3)) = (1.5, 0.75), so Σ n_i·c_i = 499.5 + 500.25 = 999.75:
  // N up to one ray at the larger correction, which is the stated tolerance.
  constexpr size_t kN = 1000;
  const float kW = 1.0f;
  auto out = RunLegacy(MakeTwoEntryScene(SceneConfig::RayAllocationMode::kAdaptive, 1.0f, 2.0f), kN, 1, 7);
  ASSERT_EQ(out.batches.size(), 1u);
  ASSERT_EQ(out.all_data.size(), 1u);

  const auto tally = TallyRoots(out.all_data[0]);
  ASSERT_EQ(tally.count.size(), 2u) << "both entries must be dealt rays";
  // Dealt by q, not by p (proportional would deal 500/500).
  EXPECT_EQ(tally.count.at(0), 333u);
  EXPECT_EQ(tally.count.at(1), 667u);
  // Every root of entry i was born at w · c_i: the per-entry weight sum is
  // n_i · w · c_i to float rounding.
  EXPECT_NEAR(tally.weight.at(0), 333.0 * kW * 1.5, 1e-3);
  EXPECT_NEAR(tally.weight.at(1), 667.0 * kW * 0.75, 1e-3);
  // AC6: Σ n_i · w · c_i == N · w within one ray at the largest correction.
  const double total_root_weight = tally.weight.at(0) + tally.weight.at(1);
  EXPECT_NEAR(total_root_weight, static_cast<double>(kN) * kW, 1.5);
  // And the batch charges exactly what it emitted — not `w · N`.
  EXPECT_NEAR(out.batches[0].emitted_energy_, total_root_weight, 1e-3);
  EXPECT_EQ(out.batches[0].root_ray_count_, kN);
}

TEST(RayAllocationLegacyPath, QEqualToPIsBitIdenticalToProportional) {
  // The default's zero-regression claim, stated as the strongest thing it can
  // be: same seed, same scene, adaptive with q == p vs proportional — every
  // root weight, every traced segment, every outgoing ray, and the charged
  // energy are the same bits. Both arms deal (500, 500) and multiply by 1.0f.
  constexpr size_t kN = 1000;
  auto prop = RunLegacy(MakeTwoEntryScene(SceneConfig::RayAllocationMode::kProportional, -1.0f, -1.0f), kN, 2, 11);
  auto adap = RunLegacy(MakeTwoEntryScene(SceneConfig::RayAllocationMode::kAdaptive, 1.0f, 1.0f), kN, 2, 11);
  ASSERT_EQ(prop.batches.size(), 2u);
  ASSERT_EQ(adap.batches.size(), 2u);
  ASSERT_EQ(prop.all_data.size(), adap.all_data.size());
  // Segment-level mismatches are counted, not asserted per row: the first
  // differing segment would otherwise hide every one after it.
  for (size_t b = 0; b < prop.batches.size(); b++) {
    EXPECT_EQ(prop.batches[b].emitted_energy_, adap.batches[b].emitted_energy_);
    EXPECT_EQ(prop.batches[b].outgoing_w_, adap.batches[b].outgoing_w_);
    EXPECT_EQ(prop.batches[b].outgoing_d_, adap.batches[b].outgoing_d_);
    const auto& a = prop.all_data[b];
    const auto& c = adap.all_data[b];
    EXPECT_EQ(a.size_, c.size_);
    size_t mismatched = 0;
    for (size_t i = 0; i < std::min(a.size_, c.size_); i++) {
      if (a[i].w_ != c[i].w_ || a[i].crystal_idx_ != c[i].crystal_idx_) {
        mismatched++;
      }
    }
    EXPECT_EQ(mismatched, 0u) << "batch " << b;
  }
  // Positive control on the comparison itself: q != p does move the bits.
  auto skew = RunLegacy(MakeTwoEntryScene(SceneConfig::RayAllocationMode::kAdaptive, 1.0f, 3.0f), kN, 1, 11);
  EXPECT_NE(TallyRoots(skew.all_data[0]).count.at(0), TallyRoots(prop.all_data[0]).count.at(0));
}

TEST(RayAllocationLegacyPath, UndeliveredWeightMakesTheLayerProportional) {
  // adaptive with one entry still at the -1 sentinel: the layer deals by p,
  // whole — not "by q where delivered". Pinned by the dealt counts (500/500,
  // not q-shaped) and by every correction being exactly 1.0f (root weights == w).
  constexpr size_t kN = 1000;
  auto out = RunLegacy(MakeTwoEntryScene(SceneConfig::RayAllocationMode::kAdaptive, 3.0f, -1.0f), kN, 1, 5);
  const auto tally = TallyRoots(out.all_data[0]);
  EXPECT_EQ(tally.count.at(0), 500u);
  EXPECT_EQ(tally.count.at(1), 500u);
  size_t roots_off_nominal = 0;
  for (size_t i = 0; i < out.all_data[0].size_; i++) {
    const auto& r = out.all_data[0][i];
    if (IsRootSegment(r) && r.w_ != 1.0f) {
      roots_off_nominal++;
    }
  }
  EXPECT_EQ(roots_off_nominal, 0u);
  EXPECT_EQ(out.batches[0].emitted_energy_, static_cast<float>(kN));
}

TEST(RayAllocationLegacyPath, ContinuationLayerUsesItsOwnCorrectionAndIsNotCharged) {
  // Two layers, both adaptive with DIFFERENT q: layer 0 q = (1, 3), layer 1
  // q = (4, 1). prob = 1 on layer 0 so every filter-pass exit continues.
  //   AC3: emitted_energy_ is layer 0's Σ n_i·c_i·w only — the continuation
  //        hops (which re-deal the SAME rays) add nothing.
  //   AC4: layer 1's roots are the continuation segments re-dealt by ITS q and
  //        scaled by ITS corrections. Each layer-1 root is one continuation
  //        segment times c_1[ci], so Σ_{layer-1 roots} w_/c_1[ci] equals
  //        Σ_{continuation segments} w_ EXACTLY (to summation rounding),
  //        whatever random subset each entry was dealt. Detection power: with
  //        layer 1 left uncorrected the left side reads ≈1.36× the right; with
  //        layer 0's (2, 2/3) applied instead of layer 1's (0.625, 2.5) it reads
  //        ≈2.6× — both far outside the 1e-4 tolerance.
  constexpr size_t kN = 1000;
  auto scene = MakeTwoEntryScene(SceneConfig::RayAllocationMode::kAdaptive, 1.0f, 3.0f, /*prob=*/1.0f);
  MsInfo second;
  second.prob_ = 0.0f;
  second.setting_.push_back(MakePrismEntry(2, 1.0f, 1.0f, 4.0f));
  second.setting_.push_back(MakePrismEntry(3, 0.3f, 1.0f, 1.0f));
  scene.ms_.push_back(std::move(second));

  auto out = RunLegacy(scene, kN, 1, 3);
  ASSERT_EQ(out.batches.size(), 1u);
  const auto& all_data = out.all_data[0];

  // Layer-0 crystals are instances 0 and 1 (created in ci order on the first
  // layer); layer-1 crystals are 2 and 3. Corrections per layer:
  //   layer 0: shares p (1/2, 1/2) vs q (1/4, 3/4) → (2, 2/3)
  //   layer 1: shares p (1/2, 1/2) vs q (4/5, 1/5) → (0.625, 2.5)
  const auto c0 = ComputeRayAllocationCorrection({ 1.0f, 1.0f }, { 1.0f, 3.0f });
  const auto c1 = ComputeRayAllocationCorrection({ 1.0f, 1.0f }, { 4.0f, 1.0f });
  const auto tally = TallyRoots(all_data);
  ASSERT_EQ(tally.count.size(), 4u) << "every (layer, entry) must be dealt rays";
  EXPECT_EQ(tally.count.at(0), 250u);
  EXPECT_EQ(tally.count.at(1), 750u);

  // AC3: charged = layer 0 only.
  const double layer0_weight = tally.weight.at(0) + tally.weight.at(1);
  EXPECT_NEAR(layer0_weight, 250.0 * c0[0] + 750.0 * c0[1], 1e-3);
  EXPECT_NEAR(out.batches[0].emitted_energy_, layer0_weight, 1e-3);
  EXPECT_NEAR(out.batches[0].emitted_energy_, static_cast<double>(kN), 2.0);

  // AC4: layer 1 re-deals the continuation weight by its own corrections.
  double continuation_weight = 0.0;
  for (size_t i = 0; i < all_data.size_; i++) {
    if (all_data[i].IsContinue()) {
      continuation_weight += all_data[i].w_;
    }
  }
  ASSERT_GT(continuation_weight, 0.0) << "prob=1 on layer 0 must produce continuations";
  const size_t layer1_roots = tally.count.at(2) + tally.count.at(3);
  // Layer 1 deals its (continuation count) rays as (4/5, 1/5) of that count.
  EXPECT_NEAR(static_cast<double>(tally.count.at(2)), 0.8 * static_cast<double>(layer1_roots), 1.0);
  const double undone = tally.weight.at(2) / c1[0] + tally.weight.at(3) / c1[1];
  EXPECT_NEAR(undone, continuation_weight, continuation_weight * 1e-4);
}


// --- Ray allocation: the pilot that delivers q ------------------------------ //
//
// The pieces the pilot is made of, each pinned on its own, then the pass as a whole:
//   1. the tally hook inside SimulateOneWavelength counts what the partition dealt
//      and sums what truly exited — measured against a render of the same scene;
//   2. ComputeAdaptiveRayAllocationWeights: p_i = 0 stays 0 whatever the tally says,
//      a zero-exit live entry lands on the floor, the floor's K counts live entries
//      only, and the design's calibration numbers reproduce its .30/.50/.20;
//   3. RayAllocationEntryResolved's three branches;
//   4. RunRayAllocationPilot on a scene with a real energy skew moves q in the
//      Neyman direction, delivers every layer, and its bounded doubling loop both
//      fires and stops;
//   5. the pass leaves a subsequent render bit-identical (no shared state touched);
//   6. AC6, on the delivered q: dealing by it and correcting by p charges N;
//   7. AC8, the starvation fixed point: without the floor a zero-exit entry is dealt
//      nothing forever, with it the entry keeps its 1%/K.

namespace {

// A raypath filter on a prism: `path` in the user's 1-based face numbering.
FilterConfig RaypathFilterIn(const std::vector<IdType>& path) {
  return FilterConfig{ kInvalidId, FilterConfig::kSymP | FilterConfig::kSymB | FilterConfig::kSymD,
                       FilterConfig::kFilterIn, RaypathFilterParam{ path } };
}

// Entry A: every raypath (p = 1); entry B: only the 3-5-7-3 path (a small fraction of
// B's rays exit at all, so E[e²]_B per dealt ray is far below A's). Under Neyman the
// rare-exit entry's share must drop below its energy share, and a small-p copy of the
// unfiltered crystal must rise above its energy share — the two directions the user's
// 120° halo scene turns on (there the rare entry carried 246× the energy per exit).
SceneConfig MakeSkewedScene(float p_a, float p_b, float p_c) {
  SceneConfig scene;
  scene.ray_num_ = 0;
  scene.max_hits_ = 6;
  scene.ray_allocation_ = SceneConfig::RayAllocationMode::kAdaptive;
  scene.light_source_.param_ = SunParam{ 20.0f, 0.0f, 0.5f };
  scene.light_source_.spectrum_ = std::vector<WlParam>{ { 550.0f, 1.0f } };
  MsInfo ms;
  ms.prob_ = 0.0f;
  ms.setting_.push_back(MakePrismEntry(0, 1.0f, p_a, -1.0f));
  auto b = MakePrismEntry(1, 1.0f, p_b, -1.0f);
  b.filter_ = RaypathFilterIn({ 3, 5, 7, 3 });
  ms.setting_.push_back(std::move(b));
  ms.setting_.push_back(MakePrismEntry(2, 1.0f, p_c, -1.0f));
  scene.ms_.push_back(std::move(ms));
  return scene;
}

double LayerSumW(const std::vector<RayAllocationPilotStats::Entry>& layer) {
  double s = 0.0;
  for (const auto& e : layer) {
    s += e.sum_w;
  }
  return s;
}

}  // namespace

TEST(RayAllocationPilotHook, TalliesTheDealtRaysAndTheTrueExits) {
  // Deal counts are the partition's own numbers, exact; the exit energy is what a
  // render of the same scene hands its consumer, to sampling error. The pilot and
  // the render draw different random streams (the pilot's chunking differs from a
  // batch), so the energy is compared per emitted ray at 5% over 40k rays each —
  // the per-ray mean of a hexagonal prism's landed energy is stable to ~1% there.
  auto scene = MakeTwoEntryScene(SceneConfig::RayAllocationMode::kProportional, -1.0f, -1.0f);
  constexpr size_t kRays = 40'000;
  RayAllocationPilotBudget budget;
  budget.initial_rays = kRays;
  budget.max_doublings = 0;
  auto report = Simulator::RunRayAllocationPilot(scene, budget);
  ASSERT_EQ(report.stats.layers.size(), 1u);
  ASSERT_EQ(report.stats.layers[0].size(), 2u);
  EXPECT_EQ(report.total_rays, kRays);
  EXPECT_EQ(report.stats.layers[0][0].rays, kRays / 2);
  EXPECT_EQ(report.stats.layers[0][1].rays, kRays / 2);
  EXPECT_GT(report.stats.layers[0][0].exits, 0u);
  EXPECT_GT(report.stats.layers[0][1].exits, 0u);
  // Σw⁴ ≥ (Σw²)²/exits by Cauchy–Schwarz: the fourth moment is accumulated from the
  // same rays as the second, not from a different set.
  const auto& e0 = report.stats.layers[0][0];
  EXPECT_GE(e0.sum_w4 * static_cast<double>(e0.exits), e0.sum_w2 * e0.sum_w2 * (1.0 - 1e-9));

  auto render = RunLegacy(MakeTwoEntryScene(SceneConfig::RayAllocationMode::kProportional, -1.0f, -1.0f), kRays, 1,
                          /*seed=*/91);
  ASSERT_EQ(render.batches.size(), 1u);
  double render_w = 0.0;
  for (float w : render.batches[0].outgoing_w_) {
    render_w += w;
  }
  ASSERT_GT(render_w, 0.0);
  const double pilot_w = LayerSumW(report.stats.layers[0]);
  EXPECT_NEAR(pilot_w / static_cast<double>(kRays), render_w / static_cast<double>(kRays),
              0.05 * render_w / static_cast<double>(kRays));
}

TEST(RayAllocationPilotHook, DeliversEveryLayerAndTalliesContinuationsUnderTheirOwnEntries) {
  // Two layers, prob = 1 on the first so every exit continues: layer 1's dealt
  // count is exactly layer 0's exit count (the survivors, re-dealt), and every
  // entry on both layers is delivered a weight >= 0 — the all-or-nothing rule of
  // ResolveLayerRayAllocation needs all of them.
  auto scene = MakeTwoEntryScene(SceneConfig::RayAllocationMode::kAdaptive, -1.0f, -1.0f, /*prob=*/1.0f);
  MsInfo second;
  second.prob_ = 0.0f;
  second.setting_.push_back(MakePrismEntry(2, 1.0f, 1.0f, -1.0f));
  second.setting_.push_back(MakePrismEntry(3, 0.3f, 1.0f, -1.0f));
  scene.ms_.push_back(std::move(second));
  RayAllocationPilotBudget budget;
  budget.initial_rays = 20'000;
  budget.max_doublings = 0;
  auto report = Simulator::RunRayAllocationPilot(scene, budget);
  ASSERT_EQ(report.stats.layers.size(), 2u);
  // With prob = 1 every layer-0 candidate exit continues, so layer 0 tallies NO
  // true exits and layer 1 is dealt every one of them.
  EXPECT_EQ(report.stats.layers[0][0].exits + report.stats.layers[0][1].exits, 0u);
  const size_t layer1_dealt = report.stats.layers[1][0].rays + report.stats.layers[1][1].rays;
  EXPECT_GT(layer1_dealt, 0u);
  EXPECT_GT(report.stats.layers[1][0].exits + report.stats.layers[1][1].exits, 0u);
  for (const auto& layer : scene.ms_) {
    for (const auto& s : layer.setting_) {
      EXPECT_GE(s.crystal_ray_alloc_weight_, 0.0f);
    }
    EXPECT_TRUE(ResolveLayerRayAllocation(scene.ray_allocation_, layer).adaptive);
  }
  // Layer 0 saw no exit at all, so both live entries sit on the floor: dealt
  // uniformly, which is what "nothing measured" has to mean.
  EXPECT_FLOAT_EQ(scene.ms_[0].setting_[0].crystal_ray_alloc_weight_, 0.005f);
  EXPECT_FLOAT_EQ(scene.ms_[0].setting_[1].crystal_ray_alloc_weight_, 0.005f);
}

TEST(AdaptiveRayAllocationWeights, ASwitchedOffEntryStaysOffWhateverTheTallySays) {
  // p_0 = 0 with a fat tally (a misuse the code must be indifferent to): q_0 == 0
  // exactly, no floor. K counts the two live entries, so the floor is 0.005.
  std::vector<RayAllocationPilotStats::Entry> stats(3);
  stats[0] = { 100.0, 100.0, 100.0, 1000, 1000 };
  stats[1] = { 10.0, 10.0, 10.0, 1000, 100 };
  stats[2] = { 10.0, 10.0, 10.0, 1000, 100 };
  auto q = ComputeAdaptiveRayAllocationWeights({ 0.0f, 0.5f, 0.5f }, stats);
  ASSERT_EQ(q.size(), 3u);
  EXPECT_EQ(q[0], 0.0f);
  EXPECT_NEAR(q[1], 0.5f, 1e-6f);
  EXPECT_NEAR(q[2], 0.5f, 1e-6f);
}

TEST(AdaptiveRayAllocationWeights, AZeroExitLiveEntryLandsOnTheFloorNotOnZero) {
  // Entry 1 was dealt rays and none exited: raw = 0, q = 0.01/K = 0.005 (K = 2).
  // Entry 2 was dealt nothing at all (a deep layer no ray reached): same floor —
  // nothing measured is not "measured to be zero", and both are dealt something.
  std::vector<RayAllocationPilotStats::Entry> stats(3);
  stats[0] = { 10.0, 10.0, 10.0, 1000, 100 };
  stats[1] = { 0.0, 0.0, 0.0, 1000, 0 };
  stats[2] = {};
  auto q = ComputeAdaptiveRayAllocationWeights({ 0.5f, 0.5f, 0.0f }, stats);
  EXPECT_NEAR(q[0], 1.0f, 1e-6f);
  EXPECT_FLOAT_EQ(q[1], 0.005f);
  EXPECT_EQ(q[2], 0.0f);
  auto q3 = ComputeAdaptiveRayAllocationWeights({ 0.5f, 0.5f, 0.5f }, stats);
  EXPECT_NEAR(q3[0], 1.0f, 1e-6f);
  EXPECT_NEAR(q3[1], 0.01f / 3.0f, 1e-7f);
  EXPECT_NEAR(q3[2], 0.01f / 3.0f, 1e-7f);
}

TEST(AdaptiveRayAllocationWeights, ReproducesTheDesignCalibration) {
  // The three entries the design was calibrated on (E[e²] per emitted ray from
  // 5×20k-ray probes of the user's scene): p = (.4995, .4995, .001), E[e²] =
  // (1.0e-7, 2.8e-7, 1.1e-2) → Neyman q = (.30, .50, .20). The rare entry's share
  // rises 200× over its energy share; that is the number the whole scrum is for.
  std::vector<RayAllocationPilotStats::Entry> stats(3);
  const double e2[3] = { 1.0e-7, 2.8e-7, 1.1e-2 };
  for (size_t i = 0; i < 3; i++) {
    stats[i].rays = 100'000;
    stats[i].sum_w2 = e2[i] * 100'000.0;
    stats[i].exits = 1;
  }
  auto q = ComputeAdaptiveRayAllocationWeights({ 0.4995f, 0.4995f, 0.001f }, stats);
  const float total = q[0] + q[1] + q[2];
  EXPECT_NEAR(q[0] / total, 0.30f, 0.01f);
  EXPECT_NEAR(q[1] / total, 0.50f, 0.01f);
  EXPECT_NEAR(q[2] / total, 0.20f, 0.01f);
}

TEST(RayAllocationEntryResolved, ThreeBranches) {
  RayAllocationPilotStats::Entry none;
  EXPECT_FALSE(RayAllocationEntryResolved(none, 0.2, 100));
  // Zero exits: resolved by the dealt count alone.
  RayAllocationPilotStats::Entry silent;
  silent.rays = 99;
  EXPECT_FALSE(RayAllocationEntryResolved(silent, 0.2, 100));
  silent.rays = 100;
  EXPECT_TRUE(RayAllocationEntryResolved(silent, 0.2, 100));
  // Every dealt ray exited at the same w: var(w²) = 0, resolved at any n.
  RayAllocationPilotStats::Entry constant;
  constant.rays = 4;
  constant.exits = 4;
  constant.sum_w2 = 4.0;
  constant.sum_w4 = 4.0;
  EXPECT_TRUE(RayAllocationEntryResolved(constant, 0.2, 100));
  // One exit among many silent rays: mean(w²) = 1/n, E[w⁴] = 1/n, so
  // var ≈ 1/n and rel_err(√mean) = 0.5·√(1/n·n)/1 = 0.5 — never resolved by the
  // variance branch, however many rays, until more exits arrive.
  RayAllocationPilotStats::Entry rare;
  rare.rays = 1'000'000;
  rare.exits = 1;
  rare.sum_w2 = 1.0;
  rare.sum_w4 = 1.0;
  EXPECT_FALSE(RayAllocationEntryResolved(rare, 0.2, 100));
  // A hundred such exits: rel_err = 0.5·√(1/100) = 0.05 ≤ 0.2.
  rare.exits = 100;
  rare.sum_w2 = 100.0;
  rare.sum_w4 = 100.0;
  EXPECT_TRUE(RayAllocationEntryResolved(rare, 0.2, 100));
  EXPECT_FALSE(RayAllocationEntryResolved(rare, 0.04, 100));
}

TEST(RayAllocationPilot, MovesQInTheNeymanDirectionOnARealEnergySkew) {
  // Energy shares p = (1, 1, 0.05). Entry B keeps one raypath of A's crystal, so
  // per dealt ray it lands a small fraction of A's energy: its q share must fall
  // below its p share. Entry C is A's crystal at 1/20 the share: per ray it lands
  // what A does, so Neyman keeps q_C/q_A == p_C/p_A — the small share alone is not
  // a reason to move (only a per-ray energy difference is; the user's scene had
  // both, which is why its rare entry rose 200×).
  auto scene = MakeSkewedScene(1.0f, 1.0f, 0.05f);
  RayAllocationPilotBudget budget;
  budget.initial_rays = 60'000;
  auto report = Simulator::RunRayAllocationPilot(scene, budget);
  const auto& s = scene.ms_[0].setting_;
  ASSERT_GE(s[0].crystal_ray_alloc_weight_, 0.0f);
  ASSERT_GE(s[1].crystal_ray_alloc_weight_, 0.0f);
  ASSERT_GE(s[2].crystal_ray_alloc_weight_, 0.0f);
  const float qa = s[0].crystal_ray_alloc_weight_;
  const float qb = s[1].crystal_ray_alloc_weight_;
  const float qc = s[2].crystal_ray_alloc_weight_;
  EXPECT_LT(qb / qa, 0.5f) << "the filtered entry lands far less per ray; Neyman must deal it less than p does";
  EXPECT_GT(qb, 0.0f) << "but never nothing: the floor is what keeps a rare entry alive";
  EXPECT_NEAR(qc / qa, 0.05f, 0.015f) << "same crystal, same per-ray energy: the share is p's";
  // The tally behind it, as the design's own diagnostic: B exits rarely.
  const auto& tb = report.stats.layers[0][1];
  const auto& ta = report.stats.layers[0][0];
  EXPECT_LT(static_cast<double>(tb.exits) / static_cast<double>(tb.rays),
            0.5 * static_cast<double>(ta.exits) / static_cast<double>(ta.rays));
}

TEST(RayAllocationPilot, DoublesABudgetThatDidNotConvergeAndStopsAtTheCap) {
  // Force the doubling loop: a target no entry can meet on the budget (5% relative
  // error on √E[e²] needs at least 100 exits even at constant w, and 60 rays split
  // three ways deal 20 each), then a cap of 2 doublings so the loop is seen both to
  // fire and to stop. total_rays is exactly initial × 2^doublings: each round
  // re-traces the cumulative count.
  auto scene = MakeSkewedScene(1.0f, 1.0f, 1.0f);
  RayAllocationPilotBudget budget;
  budget.initial_rays = 60;
  budget.max_doublings = 2;
  budget.target_rel_error = 0.05;
  budget.zero_hit_resolve_rays = 1'000'000;  // a silent entry cannot resolve by count here
  auto report = Simulator::RunRayAllocationPilot(scene, budget);
  EXPECT_EQ(report.doublings, 2);
  EXPECT_EQ(report.total_rays, 60u * 4u);
  EXPECT_FALSE(report.converged);
  // Every entry is still delivered — an unconverged pilot is a coarser q, not no q.
  for (const auto& s : scene.ms_[0].setting_) {
    EXPECT_GE(s.crystal_ray_alloc_weight_, 0.0f);
  }
  // Positive control on the loop's exit condition: with the budget the design
  // ships (large enough for the unfiltered entries) the first check passes for
  // them, and the filtered one resolves within the doublings.
  auto scene2 = MakeSkewedScene(1.0f, 1.0f, 1.0f);
  RayAllocationPilotBudget ample;
  ample.initial_rays = 60'000;
  auto report2 = Simulator::RunRayAllocationPilot(scene2, ample);
  EXPECT_TRUE(report2.converged);
  EXPECT_LE(report2.doublings, ample.max_doublings);
}

TEST(RayAllocationPilot, LeavesASubsequentRenderBitIdentical) {
  // The pass seeds this thread's RandomNumberGenerator and its own Simulator; a
  // render started afterwards on its own thread with the user's seed must be the
  // same bits as one started with no pilot before it. RunLegacy's Run() re-seeds
  // the worker thread's instance on entry, so this is what pins that the pilot
  // touched nothing a render reads.
  const auto reference =
      RunLegacy(MakeTwoEntryScene(SceneConfig::RayAllocationMode::kProportional, -1.0f, -1.0f), 2000, 2, /*seed=*/17);
  auto pilot_scene = MakeSkewedScene(1.0f, 1.0f, 0.05f);
  RayAllocationPilotBudget budget;
  budget.initial_rays = 10'000;
  budget.max_doublings = 0;
  (void)Simulator::RunRayAllocationPilot(pilot_scene, budget);
  const auto after =
      RunLegacy(MakeTwoEntryScene(SceneConfig::RayAllocationMode::kProportional, -1.0f, -1.0f), 2000, 2, /*seed=*/17);
  ASSERT_EQ(reference.batches.size(), after.batches.size());
  for (size_t b = 0; b < reference.batches.size(); b++) {
    EXPECT_EQ(reference.batches[b].outgoing_w_, after.batches[b].outgoing_w_);
    EXPECT_EQ(reference.batches[b].outgoing_d_, after.batches[b].outgoing_d_);
    EXPECT_EQ(reference.batches[b].emitted_energy_, after.batches[b].emitted_energy_);
  }
  // And the pilot itself is reproducible: same scene, same q, whatever ran before.
  auto again = MakeSkewedScene(1.0f, 1.0f, 0.05f);
  (void)Simulator::RunRayAllocationPilot(again, budget);
  for (size_t ci = 0; ci < 3; ci++) {
    EXPECT_EQ(pilot_scene.ms_[0].setting_[ci].crystal_ray_alloc_weight_,
              again.ms_[0].setting_[ci].crystal_ray_alloc_weight_);
  }
}

TEST(RayAllocationPilot, DeliveredQDealsByQAndChargesN) {
  // AC6 on the delivered vector rather than a hand-picked one: with q from a real
  // pilot (floor included), a render of the scene deals by it and the root weights
  // sum to N — the expected image is unchanged by construction of the corrections,
  // and this is the partition-level statement of that with the real q in hand.
  auto scene = MakeSkewedScene(1.0f, 1.0f, 0.05f);
  RayAllocationPilotBudget budget;
  budget.initial_rays = 30'000;
  budget.max_doublings = 0;
  (void)Simulator::RunRayAllocationPilot(scene, budget);
  constexpr size_t kN = 4000;
  auto out = RunLegacy(scene, kN, 1, 23);
  ASSERT_EQ(out.all_data.size(), 1u);
  const auto tally = TallyRoots(out.all_data[0]);
  double total_root_weight = 0.0;
  for (const auto& [id, w] : tally.weight) {
    total_root_weight += w;
  }
  // Within one ray at the largest correction (p share / q share; the floored
  // entry's correction is the largest and is bounded by (p_i/ΣP)/(0.01/K)).
  std::vector<float> p;
  std::vector<float> q;
  for (const auto& s : scene.ms_[0].setting_) {
    p.push_back(s.crystal_proportion_);
    q.push_back(s.crystal_ray_alloc_weight_);
  }
  const auto c = ComputeRayAllocationCorrection(p, q);
  const double c_max = *std::max_element(c.begin(), c.end());
  EXPECT_NEAR(total_root_weight, static_cast<double>(kN), c_max);
  EXPECT_NEAR(out.batches[0].emitted_energy_, total_root_weight, 1e-2);
  // And it really dealt by q: the filtered entry got fewer rays than its p share.
  const auto alloc = ResolveLayerRayAllocation(scene.ray_allocation_, scene.ms_[0]);
  ASSERT_TRUE(alloc.adaptive);
  EXPECT_LT(tally.count.at(1), kN / 3);
}

TEST(RayAllocationPilot, TheFloorIsWhatKeepsAZeroExitEntryFromStarvingForever) {
  // AC8. A tally-then-reallocate loop without a floor has an absorbing state: an
  // entry the pilot saw no exit from gets q = 0, is dealt nothing next round, and
  // so records no exit next round either — Σw²/n = 0 → q = 0, forever. Modelled
  // here exactly as the design's Monte Carlo did: three rounds of "deal by q,
  // tally, recompute q" on a synthetic entry whose exits arrive at rate h per dealt
  // ray, once with the floor (the shipped ComputeAdaptiveRayAllocationWeights) and
  // once with the floor stripped (the same normalization with ε = 0).
  const float p[3] = { 0.5f, 0.5f, 0.001f };
  const double h[3] = { 0.2, 0.2, 0.05 };  // exit rate per dealt ray; entry 2 is rare
  auto no_floor = [](const std::vector<float>& pv, const std::vector<RayAllocationPilotStats::Entry>& st) {
    std::vector<float> q(3, 0.0f);
    double total = 0.0;
    double raw[3] = { 0.0, 0.0, 0.0 };
    for (size_t i = 0; i < 3; i++) {
      if (pv[i] > 0.0f && st[i].rays > 0 && st[i].sum_w2 > 0.0) {
        raw[i] = pv[i] * std::sqrt(st[i].sum_w2 / static_cast<double>(st[i].rays));
        total += raw[i];
      }
    }
    for (size_t i = 0; i < 3; i++) {
      q[i] = total > 0.0 ? static_cast<float>(raw[i] / total) : 0.0f;
    }
    return q;
  };
  auto simulate = [&](bool with_floor) {
    // Round 0: a pilot so small the rare entry (p = 0.001 → 1 ray of 1000) sees
    // no exit. Deterministic tally: exits = floor(h · rays), each at w = 1.
    std::vector<float> q = { p[0], p[1], p[2] };
    std::vector<size_t> dealt_last(3, 0);
    for (int round = 0; round < 3; round++) {
      std::vector<double> carry(3, 0.0);
      auto n = PartitionCrystalRayNum(q, 1000, carry);
      std::vector<RayAllocationPilotStats::Entry> st(3);
      for (size_t i = 0; i < 3; i++) {
        st[i].rays = n[i];
        st[i].exits = static_cast<size_t>(h[i] * static_cast<double>(n[i]));
        st[i].sum_w = st[i].sum_w2 = st[i].sum_w4 = static_cast<double>(st[i].exits);
        dealt_last[i] = n[i];
      }
      q = with_floor ? ComputeAdaptiveRayAllocationWeights({ p[0], p[1], p[2] }, st) :
                       no_floor({ p[0], p[1], p[2] }, st);
    }
    return std::make_pair(q, dealt_last);
  };
  auto [q_none, dealt_none] = simulate(false);
  EXPECT_EQ(q_none[2], 0.0f) << "without a floor the rare entry is absorbed at q = 0";
  EXPECT_EQ(dealt_none[2], 0u) << "and dealt nothing — the fixed point";
  auto [q_floor, dealt_floor] = simulate(true);
  EXPECT_FLOAT_EQ(q_floor[2], 0.01f / 3.0f) << "the floor holds the rare entry at 1%/K";
  EXPECT_GT(dealt_floor[2], 0u) << "so it keeps being dealt rays, and can be measured";
}

}  // namespace lumice
