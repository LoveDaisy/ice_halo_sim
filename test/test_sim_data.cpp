// Tests for src/config/sim_data.hpp — RayBuffer and SimData.
//
// SimData has 4 hand-written special member functions (copy/move ctor + assign).
// History: learnings.md records "manual copy/move easily miss new fields".
// When adding a field to SimData, you MUST update:
//   1. sim_data.cpp's static_assert(sizeof(SimData) == ...) — already exists
//   2. The 4 special member functions in sim_data.cpp
//   3. CopyConstructDeepCopy / CopyAssignmentDeepCopy / MoveConstruct* tests below
// The sizeof static_assert at the top of this file is a redundant mirror of
// (1), gated to the platform where the value was authored. This is NOT a true
// double-safeguard on other platforms — the source-side assert in sim_data.cpp
// is unconditional and remains the primary guarantee.

#include <gtest/gtest.h>

#include <utility>

#include "config/sim_data.hpp"
#include "core/def.hpp"
#include "core/raypath.hpp"

namespace {

using lumice::kInfSize;
using lumice::RayBuffer;
using lumice::RaySeg;
using lumice::SimData;

// Redundant mirror of sim_data.cpp's static_assert, gated to the platform
// where the original assert was authored (macOS Apple Silicon, 64-bit).
// Other 64-bit ABIs may differ; the unconditional assert in sim_data.cpp is
// the primary guarantee — this mirror just adds an extra reminder on the
// authoring platform that this test file's per-field assertions need updating.
#if defined(__APPLE__) && defined(__aarch64__)
static_assert(sizeof(void*) == 8, "SimData layout assumes 64-bit pointers");
static_assert(sizeof(SimData) == 144,
              "SimData layout changed — update test_sim_data.cpp DeepCopy/Move assertions "
              "and sim_data.cpp's static_assert.");
#endif

// Helper: build a RaySeg with an identifiable marker for assertions.
RaySeg MakeRay(int marker) {
  RaySeg r{};
  r.w_ = static_cast<float>(marker);
  r.fid_ = marker;
  return r;
}

// Helper: build a fully populated SimData with rays_.size_ < rays_.capacity_
// (capacity=8, size=5). The size != capacity input shape is load-bearing —
// it would catch a future "memcpy size bytes instead of capacity bytes"
// refactor that silently truncates trailing data.
SimData MakePopulatedSimData() {
  SimData s(8);
  s.curr_wl_ = 550.0f;
  s.total_intensity_ = 1.5f;
  s.generation_ = 42;
  s.root_ray_count_ = 7;
  for (int i = 0; i < 5; i++) {
    s.rays_.EmplaceBack(MakeRay(i));
  }
  s.outgoing_indices_ = { 0, 2, 4 };
  s.outgoing_d_ = { 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f };
  s.outgoing_w_ = { 0.5f, 0.7f };
  s.crystals_.emplace_back();
  return s;
}

}  // namespace


// =============== RayBuffer Tests ===============

TEST(RayBufferTest, DefaultConstruction) {
  RayBuffer buf;
  EXPECT_EQ(buf.capacity_, 0u);
  EXPECT_EQ(buf.size_, 0u);
  EXPECT_TRUE(buf.Empty());
  EXPECT_EQ(buf.rays(), nullptr);
}


TEST(RayBufferTest, CapacityConstruction) {
  RayBuffer buf(10);
  EXPECT_EQ(buf.capacity_, 10u);
  EXPECT_EQ(buf.size_, 0u);
  EXPECT_TRUE(buf.Empty());
  EXPECT_NE(buf.rays(), nullptr);
}


// EmplaceBack(RaySeg) has a strict-less-than guard: `if (size_ + 1 < capacity_)`.
// This means a buffer of capacity N can only hold N-1 elements (the last slot is
// never written). This test pins that quirk in place by name and assertion.
TEST(RayBufferTest, EmplaceBackLosesOneSlot) {
  RayBuffer buf(5);

  for (int i = 0; i < 4; i++) {
    buf.EmplaceBack(MakeRay(i));
  }
  EXPECT_EQ(buf.size_, 4u);
  EXPECT_FALSE(buf.Empty());
  EXPECT_FLOAT_EQ(buf[0].w_, 0.0f);
  EXPECT_FLOAT_EQ(buf[1].w_, 1.0f);
  EXPECT_FLOAT_EQ(buf[2].w_, 2.0f);
  EXPECT_FLOAT_EQ(buf[3].w_, 3.0f);

  // Guard rejects further writes; size and existing data must remain unchanged.
  buf.EmplaceBack(MakeRay(99));
  EXPECT_EQ(buf.size_, 4u);
  EXPECT_FLOAT_EQ(buf[0].w_, 0.0f);
  EXPECT_FLOAT_EQ(buf[3].w_, 3.0f);

  // Repeated guard hits should also leave existing data intact.
  for (int i = 0; i < 5; i++) {
    buf.EmplaceBack(MakeRay(88));
  }
  EXPECT_EQ(buf.size_, 4u);
  EXPECT_FLOAT_EQ(buf[0].w_, 0.0f);
  EXPECT_FLOAT_EQ(buf[3].w_, 3.0f);
}


TEST(RayBufferTest, EmplaceBackBatchOnEmptyDst) {
  RayBuffer src(10);
  for (int i = 0; i < 5; i++) {
    src.EmplaceBack(MakeRay(i));
  }
  EXPECT_EQ(src.size_, 5u);

  RayBuffer dst(10);
  dst.EmplaceBack(src, 1, 3);
  EXPECT_EQ(dst.size_, 3u);
  EXPECT_FLOAT_EQ(dst[0].w_, 1.0f);
  EXPECT_FLOAT_EQ(dst[1].w_, 2.0f);
  EXPECT_FLOAT_EQ(dst[2].w_, 3.0f);

  // Calling with kInfSize (size_t::max) must be clamped by buffer.size_,
  // not propagate as an overflow.
  RayBuffer dst2(10);
  dst2.EmplaceBack(src, 0, kInfSize);
  EXPECT_EQ(dst2.size_, 5u);  // clamped to src.size_
  EXPECT_FLOAT_EQ(dst2[0].w_, 0.0f);
  EXPECT_FLOAT_EQ(dst2[4].w_, 4.0f);
}


// Contract-locking test for a known quirk in batch EmplaceBack:
//   end = std::min({ start + len, capacity_, buffer.size_ });
// uses `capacity_` as the upper bound rather than `capacity_ - size_`. When
// `dst` already contains data, this can cause writes past the allocated end.
//
// This test is DISABLED_ — GoogleTest will skip it but emit a "YOU HAVE 1
// DISABLED TEST" reminder. It serves as a regression anchor: when the quirk
// is fixed in a future task, removing the DISABLED_ prefix will validate
// the fix. DO NOT delete this test; it documents the current behavior and
// the pending fix.
TEST(RayBufferTest, DISABLED_EmplaceBackBatchOnNonEmptyDstQuirk) {
  // Intentionally not executed by default. To run manually:
  //   ./unit_test --gtest_also_run_disabled_tests --gtest_filter='*Quirk*'
  //
  // TODO: when sim_data.cpp's batch EmplaceBack is fixed to use
  //       `capacity_ - size_` as the upper bound, remove the DISABLED_ prefix
  //       and replace the FAIL with the assertions sketched below.
  //
  // Target post-fix scenario:
  //   RayBuffer src(10);
  //   for (int i = 0; i < 10; i++) src.EmplaceBack(MakeRay(i));
  //   // src can hold 9 (strict-less-than guard); src.size_ == 9.
  //
  //   RayBuffer dst(10);
  //   for (int i = 0; i < 6; i++) dst.EmplaceBack(MakeRay(100 + i));
  //   // dst.size_ == 6 (also constrained to capacity-1 = 9).
  //
  //   dst.EmplaceBack(src, 0, /*len=*/kInfSize);
  //   // After fix, end = min(start+len, capacity_-size_, src.size_) = min(_, 4, 9) = 4
  //   // i.e. only 4 elements are copied (filling dst to capacity-1 = 9 minus 1 reserved slot).
  //
  // Target assertions (uncomment when fix lands and DISABLED_ is removed):
  //   EXPECT_EQ(dst.size_, 9u);  // pre-existing 6 + 3 new (or whatever the
  //                              //   fixed bound permits without overflow)
  //   EXPECT_FLOAT_EQ(dst[5].w_, 105.0f);  // last pre-existing element intact
  //   EXPECT_FLOAT_EQ(dst[6].w_, 0.0f);    // first appended element from src
  //
  // Until the fix lands, this test fails loudly when run manually so nobody
  // mistakes the DISABLED state for "passing".
  FAIL() << "Quirk anchor: batch EmplaceBack on non-empty dst writes past "
            "capacity. Awaiting fix in a future task. See target assertions "
            "in the comment block above.";
}


TEST(RayBufferTest, ResetGrowsButNeverShrinks) {
  RayBuffer buf(5);
  for (int i = 0; i < 3; i++) {
    buf.EmplaceBack(MakeRay(i));
  }
  EXPECT_EQ(buf.size_, 3u);

  // Same capacity: size resets, capacity unchanged.
  buf.Reset(5);
  EXPECT_EQ(buf.size_, 0u);
  EXPECT_EQ(buf.capacity_, 5u);
  EXPECT_TRUE(buf.Empty());

  // Larger capacity: grows.
  buf.Reset(20);
  EXPECT_EQ(buf.size_, 0u);
  EXPECT_EQ(buf.capacity_, 20u);
  EXPECT_NE(buf.rays(), nullptr);

  // Smaller capacity: ignored, capacity stays at 20 (only-grows semantics).
  buf.Reset(3);
  EXPECT_EQ(buf.size_, 0u);
  EXPECT_EQ(buf.capacity_, 20u);
  EXPECT_TRUE(buf.Empty());

  // From a default-constructed (capacity=0) buffer, Reset(N) must allocate.
  RayBuffer empty_buf;
  EXPECT_EQ(empty_buf.capacity_, 0u);
  empty_buf.Reset(10);
  EXPECT_EQ(empty_buf.capacity_, 10u);
  EXPECT_EQ(empty_buf.size_, 0u);
  EXPECT_NE(empty_buf.rays(), nullptr);
}


TEST(RayBufferTest, IterationAndIndexing) {
  RayBuffer buf(5);
  for (int i = 0; i < 4; i++) {
    buf.EmplaceBack(MakeRay(i));
  }

  // begin/end pointer arithmetic equals size.
  EXPECT_EQ(buf.end() - buf.begin(), 4);

  // Range-based iteration sees all elements in order.
  float sum = 0.0f;
  for (const auto& r : buf) {
    sum += r.w_;
  }
  EXPECT_FLOAT_EQ(sum, 6.0f);  // 0+1+2+3

  // operator[] direct access matches iterated values.
  EXPECT_FLOAT_EQ(buf[0].w_, 0.0f);
  EXPECT_FLOAT_EQ(buf[3].w_, 3.0f);
}


// =============== SimData Tests ===============

TEST(SimDataTest, CopyConstructDeepCopy) {
  auto original = MakePopulatedSimData();
  SimData copy(original);

  // Scalar field assertions, each with a failure message identifying which
  // field was missed (helps catch the "manual copy missed a field" bug class).
  EXPECT_FLOAT_EQ(copy.curr_wl_, 550.0f) << "curr_wl_ not copied";
  EXPECT_FLOAT_EQ(copy.total_intensity_, 1.5f) << "total_intensity_ not copied";
  EXPECT_EQ(copy.generation_, 42u) << "generation_ not copied";
  EXPECT_EQ(copy.root_ray_count_, 7u) << "root_ray_count_ not copied";

  // rays_ field assertions.
  EXPECT_EQ(copy.rays_.size_, 5u);
  EXPECT_EQ(copy.rays_.capacity_, 8u);
  for (int i = 0; i < 5; i++) {
    EXPECT_FLOAT_EQ(copy.rays_[i].w_, original.rays_[i].w_);
  }

  // Vector field equality.
  EXPECT_EQ(copy.outgoing_indices_, original.outgoing_indices_);
  EXPECT_EQ(copy.outgoing_d_, original.outgoing_d_);
  EXPECT_EQ(copy.outgoing_w_, original.outgoing_w_);
  EXPECT_EQ(copy.crystals_.size(), original.crystals_.size());

  // Deep copy independence — each pointer/container field independently.
  copy.rays_[0].w_ = 999.0f;
  EXPECT_FLOAT_EQ(original.rays_[0].w_, 0.0f) << "rays_ not deep-copied";

  copy.outgoing_indices_.push_back(99);
  EXPECT_EQ(original.outgoing_indices_.size(), 3u) << "outgoing_indices_ not deep-copied";

  copy.outgoing_d_.clear();
  EXPECT_EQ(original.outgoing_d_.size(), 6u) << "outgoing_d_ not deep-copied";

  copy.outgoing_w_.clear();
  EXPECT_EQ(original.outgoing_w_.size(), 2u) << "outgoing_w_ not deep-copied";

  copy.crystals_.clear();
  EXPECT_EQ(original.crystals_.size(), 1u) << "crystals_ not deep-copied";
}


TEST(SimDataTest, CopyAssignmentDeepCopy) {
  auto original = MakePopulatedSimData();
  SimData target(3);  // Different initial capacity, exercises rays_ realloc path.
  target = original;

  // Field completeness (mirror of copy ctor test).
  EXPECT_FLOAT_EQ(target.curr_wl_, 550.0f) << "curr_wl_ not assigned";
  EXPECT_FLOAT_EQ(target.total_intensity_, 1.5f) << "total_intensity_ not assigned";
  EXPECT_EQ(target.generation_, 42u) << "generation_ not assigned";
  EXPECT_EQ(target.root_ray_count_, 7u) << "root_ray_count_ not assigned";
  EXPECT_EQ(target.rays_.size_, 5u);
  EXPECT_EQ(target.rays_.capacity_, 8u);
  for (int i = 0; i < 5; i++) {
    EXPECT_FLOAT_EQ(target.rays_[i].w_, original.rays_[i].w_);
  }
  EXPECT_EQ(target.outgoing_indices_, original.outgoing_indices_);
  EXPECT_EQ(target.outgoing_d_, original.outgoing_d_);
  EXPECT_EQ(target.outgoing_w_, original.outgoing_w_);
  EXPECT_EQ(target.crystals_.size(), 1u);

  // Deep copy independence.
  target.rays_[0].w_ = 999.0f;
  EXPECT_FLOAT_EQ(original.rays_[0].w_, 0.0f) << "rays_ not deep-assigned";
  target.outgoing_indices_.push_back(99);
  EXPECT_EQ(original.outgoing_indices_.size(), 3u) << "outgoing_indices_ not deep-assigned";
  target.crystals_.clear();
  EXPECT_EQ(original.crystals_.size(), 1u) << "crystals_ not deep-assigned";

  // Self-assignment must preserve all fields (source code has &other == this guard).
  // Snapshot → self-assign → assert preservation (NOT assert clearing).
  // Use an alias reference to bypass -Wself-assign-overloaded.
  auto orig = MakePopulatedSimData();
  const float kSnapCurrWl = orig.curr_wl_;
  const float kSnapTotalInt = orig.total_intensity_;
  const uint64_t kSnapGen = orig.generation_;
  const size_t kSnapRoot = orig.root_ray_count_;
  const size_t kSnapRaysSize = orig.rays_.size_;
  const size_t kSnapRaysCap = orig.rays_.capacity_;
  const float kSnapRay0W = orig.rays_[0].w_;
  const auto kSnapOutgoingIdx = orig.outgoing_indices_;
  const size_t kSnapCrystalsSize = orig.crystals_.size();

  SimData& self_alias = orig;
  self_alias = orig;

  EXPECT_FLOAT_EQ(orig.curr_wl_, kSnapCurrWl);
  EXPECT_FLOAT_EQ(orig.total_intensity_, kSnapTotalInt);
  EXPECT_EQ(orig.generation_, kSnapGen);
  EXPECT_EQ(orig.root_ray_count_, kSnapRoot);
  EXPECT_EQ(orig.rays_.size_, kSnapRaysSize);
  EXPECT_EQ(orig.rays_.capacity_, kSnapRaysCap);
  EXPECT_FLOAT_EQ(orig.rays_[0].w_, kSnapRay0W);
  EXPECT_EQ(orig.outgoing_indices_, kSnapOutgoingIdx);
  EXPECT_EQ(orig.crystals_.size(), kSnapCrystalsSize);
}


TEST(SimDataTest, MoveConstructTransfersOwnership) {
  auto original = MakePopulatedSimData();
  RaySeg* original_ptr = original.rays_.rays_.get();

  SimData moved(std::move(original));

  // Moved object: pointer ownership transfer (no memcpy), all fields valid.
  EXPECT_EQ(moved.rays_.rays_.get(), original_ptr) << "ownership transfer expected";
  EXPECT_EQ(moved.rays_.size_, 5u);
  EXPECT_EQ(moved.rays_.capacity_, 8u);
  EXPECT_FLOAT_EQ(moved.curr_wl_, 550.0f);
  EXPECT_FLOAT_EQ(moved.total_intensity_, 1.5f);
  EXPECT_EQ(moved.generation_, 42u);
  EXPECT_EQ(moved.root_ray_count_, 7u);
  EXPECT_EQ(moved.outgoing_indices_.size(), 3u);
  EXPECT_EQ(moved.outgoing_d_.size(), 6u);
  EXPECT_EQ(moved.outgoing_w_.size(), 2u);
  EXPECT_EQ(moved.crystals_.size(), 1u);

  // Moved-from source contract — three categories:
  // (a) rays_ pointer ownership transferred → nullptr + zeroed size/capacity.
  EXPECT_EQ(original.rays_.rays_, nullptr);
  EXPECT_EQ(original.rays_.size_, 0u);
  EXPECT_EQ(original.rays_.capacity_, 0u);

  // (b) std::vector members are moved-from. The C++ standard only guarantees
  // "valid but unspecified" state, but libc++/libstdc++ both leave them empty.
  // This assertion locks the current observed behavior; if migrating to MSVC
  // STL or a future libc++ change, this may need to be relaxed.
  EXPECT_TRUE(original.crystals_.empty());
  EXPECT_TRUE(original.outgoing_indices_.empty());
  EXPECT_TRUE(original.outgoing_d_.empty());
  EXPECT_TRUE(original.outgoing_w_.empty());

  // (c) POD scalar fields are NOT reset on move — this is the current
  // contract. We deliberately do NOT assert curr_wl_/generation_/etc. to be
  // zero. Future refactors that change this behavior should update both
  // sim_data.cpp and this comment.
}


TEST(SimDataTest, MoveAssignAndSelfMove) {
  // Normal move-assignment.
  auto src = MakePopulatedSimData();
  RaySeg* src_ptr = src.rays_.rays_.get();
  SimData dst(3);  // Different initial capacity.
  dst = std::move(src);

  EXPECT_EQ(dst.rays_.rays_.get(), src_ptr);
  EXPECT_EQ(dst.rays_.size_, 5u);
  EXPECT_EQ(dst.rays_.capacity_, 8u);
  EXPECT_FLOAT_EQ(dst.curr_wl_, 550.0f);
  EXPECT_EQ(dst.generation_, 42u);
  EXPECT_EQ(dst.outgoing_indices_.size(), 3u);
  EXPECT_EQ(dst.crystals_.size(), 1u);

  // Source moved-from state.
  EXPECT_EQ(src.rays_.rays_, nullptr);
  EXPECT_EQ(src.rays_.size_, 0u);
  EXPECT_EQ(src.rays_.capacity_, 0u);
  EXPECT_TRUE(src.crystals_.empty());
  EXPECT_TRUE(src.outgoing_indices_.empty());

  // Self-move-assignment must preserve all fields (source code has
  // &other == this guard). Snapshot → self-move → assert preservation.
  auto self = MakePopulatedSimData();
  const float kSnapCurrWl = self.curr_wl_;
  const uint64_t kSnapGen = self.generation_;
  const size_t kSnapRaysSize = self.rays_.size_;
  RaySeg* snap_rays_ptr = self.rays_.rays_.get();
  const size_t kSnapOutgoingIdxSize = self.outgoing_indices_.size();
  const size_t kSnapCrystalsSize = self.crystals_.size();

  // Use an alias reference to bypass -Wself-move warning.
  SimData& self_ref = self;
  self_ref = std::move(self);

  EXPECT_FLOAT_EQ(self.curr_wl_, kSnapCurrWl);
  EXPECT_EQ(self.generation_, kSnapGen);
  EXPECT_EQ(self.rays_.size_, kSnapRaysSize);
  EXPECT_EQ(self.rays_.rays_.get(), snap_rays_ptr);
  EXPECT_EQ(self.outgoing_indices_.size(), kSnapOutgoingIdxSize);
  EXPECT_EQ(self.crystals_.size(), kSnapCrystalsSize);
}
