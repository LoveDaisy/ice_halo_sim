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

#include <algorithm>
#include <cstring>
#include <type_traits>
#include <utility>

#include "config/sim_data.hpp"
#include "core/def.hpp"
#include "core/exit_seam.hpp"
#include "core/raypath.hpp"

namespace {

using lumice::kInfSize;
using lumice::RayBuffer;
using lumice::RaypathRecorder;
using lumice::RaySeg;
using lumice::SimData;

// Redundant mirror of sim_data.cpp's static_assert, gated to the platform
// where the original assert was authored (macOS Apple Silicon, 64-bit).
// Other 64-bit ABIs may differ; the unconditional assert in sim_data.cpp is
// the primary guarantee — this mirror just adds an extra reminder on the
// authoring platform that this test file's per-field assertions need updating.
#if defined(__APPLE__) && defined(__aarch64__)
static_assert(sizeof(void*) == 8, "SimData layout assumes 64-bit pointers");
// Task 252.3 added backend_xyz_/backend_total_intensity_/is_backend_path_
// (28B + 7B padding) bumping 192 → 232 for the image-seam path. scrum-258.1
// Step 4 removes those fields (exit seam is the canonical out path),
// shrinking back to 192. scrum-258.2 adds exit_records_
// (vector<ExitRayRecord>, 24B), bumping 192 → 216. scrum-268.8 (DR-3) adds
// outgoing_wl_ (vector<float>, 24B) for per-ray wavelength, bumping 216 → 240.
// chore-292 (A2) removes the vestigial outgoing_indices_ (vector<size_t>, 24B)
// — content never read; count = outgoing_w_.size() — shrinking 240 → 216.
// S1 device-fused: adds xyz_pixel_data_ (vector<float>, 24B) + xyz_landed_weight_
// (float, 4B) + 4B padding = 32B, bumping 216 → 248. task-exit-seam-crystal-count
// adds crystal_count_ (size_t, 8B) for exit-seam stats, bumping 248 → 256.
// scrum-312 adds sim_scene_credit_ (size_t, 8B) for third-clock drain counter
// balance, bumping 256 → 264.
static_assert(sizeof(SimData) == 264,
              "SimData layout changed — update test_sim_data.cpp DeepCopy/Move assertions "
              "and sim_data.cpp's static_assert.");
#endif

// Helper: build a RaySeg with an identifiable marker for assertions.
RaySeg MakeRay(int marker) {
  RaySeg r{};
  r.w_ = static_cast<float>(marker);
  // No fid-equivalent in the new RaySeg; keep both polygon-face fields at
  // kInvalidId. Test assertions only inspect w_, so this is sufficient.
  r.from_face_ = lumice::kInvalidId;
  r.to_face_ = lumice::kInvalidId;
  return r;
}

// Helper: build a fully populated SimData with rays_.size_ < rays_.capacity_
// (capacity=8, size=5). The size != capacity input shape is load-bearing —
// it would catch a future "memcpy size bytes instead of capacity bytes"
// refactor that silently truncates trailing data.
SimData MakePopulatedSimData() {
  SimData s(8);
  s.curr_wl_ = 550.0f;
  s.generation_ = 42;
  s.root_ray_count_ = 7;
  for (int i = 0; i < 5; i++) {
    s.rays_.EmplaceBack(MakeRay(i), RaypathRecorder{});
  }
  s.outgoing_d_ = { 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f };
  s.outgoing_w_ = { 0.5f, 0.7f };
  // scrum-268.8 (DR-3): outgoing_wl_ deep-copy / move coverage.
  s.outgoing_wl_ = { 550.0f, 600.0f };
  // scrum-258.2: exit_records_ — 2 distinct rich records to exercise the
  // deep-copy / move paths added to SimData's special members.
  s.exit_records_.resize(2);
  s.exit_records_[0].dir[0] = 1.0f;
  s.exit_records_[0].dir[1] = 2.0f;
  s.exit_records_[0].dir[2] = 3.0f;
  s.exit_records_[0].weight = 0.5f;
  s.exit_records_[0].crystal_id = 7;
  s.exit_records_[0].ms_layer_idx = 0;
  s.exit_records_[0].path.size_ = 2;
  s.exit_records_[0].path.data_[0] = 3;
  s.exit_records_[0].path.data_[1] = 5;
  s.exit_records_[1].dir[0] = 4.0f;
  s.exit_records_[1].dir[1] = 5.0f;
  s.exit_records_[1].dir[2] = 6.0f;
  s.exit_records_[1].weight = 0.7f;
  s.exit_records_[1].crystal_id = 9;
  s.exit_records_[1].ms_layer_idx = 1;
  s.exit_records_[1].path.size_ = 1;
  s.exit_records_[1].path.data_[0] = 11;
  // S1 device-fused: xyz_pixel_data_ + xyz_landed_weight_ coverage.
  s.xyz_pixel_data_ = { 0.1f, 0.2f, 0.3f };
  s.xyz_landed_weight_ = 1.5f;
  s.crystals_.emplace_back();
  // task-exit-seam-crystal-count: propagation coverage for the new field.
  s.crystal_count_ = 4;
  // scrum-312: third-clock drain counter-balance credit propagation coverage.
  s.sim_scene_credit_ = 11;
  return s;
}

}  // namespace


// =============== RaypathRecorder POD / RayBuffer overflow Tests ===============
//
// Round 2 (#247.4) restructured overflow as a per-RayBuffer arena keyed by
// recorders_[idx].overflow_idx_. RaypathRecorder itself is POD; standalone
// operator<< only services inline-cap recorders (asserts on overflow). All
// overflow coverage therefore lives at the RayBuffer level below.

TEST(RaypathRecorderTest, IsTriviallyCopyableAndCompact) {
  static_assert(std::is_trivially_copyable_v<RaypathRecorder>,
                "RaypathRecorder must be POD/trivially-copyable so fan-out is memcpy");
  static_assert(sizeof(RaypathRecorder) == 18,
                "RaypathRecorder stride must stay at 18B; regressing here means the "
                "fan-out memcpy footprint grew");
  // Inline append fills data_ contiguously and never touches the overflow index.
  RaypathRecorder rec;
  for (uint8_t i = 0; i < RaypathRecorder::kInlineCap; i++) {
    rec << static_cast<lumice::IdType>(i + 1);
  }
  EXPECT_EQ(rec.size_, RaypathRecorder::kInlineCap);
  EXPECT_FALSE(rec.HasOverflow());
  for (uint8_t i = 0; i < RaypathRecorder::kInlineCap; i++) {
    EXPECT_EQ(rec.data_[i], static_cast<uint8_t>(i + 1));
  }
}


TEST(RayBufferRecorderTest, AppendOverflowMigratesInlineSlot) {
  // The buffer-level append path is the only way to push past kInlineCap.
  // First 15 bytes stay inline; the 16th forces arena allocation and
  // migrates data_[0..14] into the slot before writing the new byte.
  RayBuffer buf(4);
  for (uint8_t i = 0; i < RaypathRecorder::kInlineCap; i++) {
    buf.RecorderAppend(0, static_cast<lumice::IdType>(i + 1));
  }
  ASSERT_EQ(buf.RecorderAt(0).size_, RaypathRecorder::kInlineCap);
  EXPECT_FALSE(buf.RecorderAt(0).HasOverflow()) << "still inline at exactly kInlineCap hits";
  EXPECT_EQ(buf.RecorderDataPtr(0), buf.RecorderAt(0).data_);

  // 16th hit triggers the migration.
  buf.RecorderAppend(0, static_cast<lumice::IdType>(16));

  EXPECT_EQ(buf.RecorderAt(0).size_, static_cast<uint8_t>(RaypathRecorder::kInlineCap + 1));
  ASSERT_TRUE(buf.RecorderAt(0).HasOverflow());
  EXPECT_NE(buf.OverflowArena(), nullptr);

  const uint8_t* p = buf.RecorderDataPtr(0);
  ASSERT_NE(p, buf.RecorderAt(0).data_) << "DataPtr must follow the arena slot once overflow";
  for (uint8_t i = 0; i < RaypathRecorder::kInlineCap; i++) {
    EXPECT_EQ(p[i], i + 1) << "migrated inline byte " << static_cast<int>(i) << " corrupted";
  }
  EXPECT_EQ(p[RaypathRecorder::kInlineCap], static_cast<uint8_t>(16));
}


TEST(RayBufferRecorderTest, AppendBeyondMaxHitsIsNoop) {
  // size_ saturates at kMaxHits; further appends are silently dropped.
  RayBuffer buf(4);
  for (uint8_t i = 0; i < lumice::kMaxHits; i++) {
    buf.RecorderAppend(0, static_cast<lumice::IdType>(i + 1));
  }
  EXPECT_EQ(buf.RecorderAt(0).size_, static_cast<uint8_t>(lumice::kMaxHits));
  ASSERT_TRUE(buf.RecorderAt(0).HasOverflow());

  // Snapshot the arena slot, push one more, expect no change.
  uint8_t snapshot[lumice::kMaxHits];
  std::memcpy(snapshot, buf.RecorderDataPtr(0), lumice::kMaxHits);

  buf.RecorderAppend(0, static_cast<lumice::IdType>(99));
  EXPECT_EQ(buf.RecorderAt(0).size_, static_cast<uint8_t>(lumice::kMaxHits));
  EXPECT_EQ(std::memcmp(buf.RecorderDataPtr(0), snapshot, lumice::kMaxHits), 0)
      << "saturation must not mutate stored bytes";
}


TEST(RayBufferRecorderTest, ClearResetsToInlinePath) {
  RayBuffer buf(4);
  for (uint8_t i = 0; i < RaypathRecorder::kInlineCap + 2; i++) {
    buf.RecorderAppend(0, static_cast<lumice::IdType>(i + 1));
  }
  ASSERT_TRUE(buf.RecorderAt(0).HasOverflow());

  buf.RecorderClear(0);

  EXPECT_EQ(buf.RecorderAt(0).size_, static_cast<uint8_t>(0));
  EXPECT_FALSE(buf.RecorderAt(0).HasOverflow());
  EXPECT_EQ(buf.RecorderDataPtr(0), buf.RecorderAt(0).data_);

  // Small follow-up push stays inline; the arena slot is NOT reclaimed (bump
  // allocator semantics — Reset() rewinds overflow_used_), but that slot is
  // unobservable from this recorder.
  buf.RecorderAppend(0, static_cast<lumice::IdType>(42));
  EXPECT_EQ(buf.RecorderAt(0).size_, static_cast<uint8_t>(1));
  EXPECT_FALSE(buf.RecorderAt(0).HasOverflow());
  EXPECT_EQ(buf.RecorderDataPtr(0)[0], static_cast<uint8_t>(42));
}


TEST(RayBufferRecorderTest, FanOutDuplicatesOverflowSlotIntoDstArena) {
  RayBuffer src(4);
  RayBuffer dst(8);

  // Populate src[1] with an overflow recorder of 20 bytes (inline 15 +
  // overflow 5).
  for (uint8_t i = 0; i < 20; i++) {
    src.RecorderAppend(1, static_cast<lumice::IdType>(i + 1));
  }
  ASSERT_TRUE(src.RecorderAt(1).HasOverflow());

  // Pretend src has size 2 so FanOut bookkeeping is consistent; in production
  // the caller (simulator.cpp) sets buffer_data[0].size_ before fan-out.
  src.size_ = 2;
  dst.size_ = 0;

  dst.RecorderFanOut(src, /*src_idx=*/1, /*dst0=*/2, /*dst1=*/3);

  // Both dst slots must reflect the same 20 bytes as src[1].
  ASSERT_TRUE(dst.RecorderAt(2).HasOverflow());
  ASSERT_TRUE(dst.RecorderAt(3).HasOverflow());
  EXPECT_EQ(dst.RecorderAt(2).size_, static_cast<uint8_t>(20));
  EXPECT_EQ(dst.RecorderAt(3).size_, static_cast<uint8_t>(20));

  const uint8_t* a = dst.RecorderDataPtr(2);
  const uint8_t* b = dst.RecorderDataPtr(3);
  const uint8_t* s = src.RecorderDataPtr(1);
  ASSERT_NE(a, s) << "dst arena must be independent of src arena";
  ASSERT_NE(b, s);
  ASSERT_NE(a, b) << "dst0 and dst1 must occupy distinct arena slots";
  for (uint8_t i = 0; i < 20; i++) {
    EXPECT_EQ(a[i], static_cast<uint8_t>(i + 1)) << "dst0 byte " << static_cast<int>(i);
    EXPECT_EQ(b[i], static_cast<uint8_t>(i + 1)) << "dst1 byte " << static_cast<int>(i);
  }

  // Mutating dst0 must NOT bleed into src or dst1.
  uint8_t snapshot_src[lumice::kMaxHits];
  uint8_t snapshot_b[lumice::kMaxHits];
  std::memcpy(snapshot_src, s, lumice::kMaxHits);
  std::memcpy(snapshot_b, b, lumice::kMaxHits);
  uint8_t* a_mut = dst.RecorderDataPtr(2);
  a_mut[0] = 0xAA;
  EXPECT_EQ(std::memcmp(s, snapshot_src, lumice::kMaxHits), 0) << "src arena leaked";
  EXPECT_EQ(std::memcmp(b, snapshot_b, lumice::kMaxHits), 0) << "dst1 arena aliased dst0";
}


// task-284 regression: single-ray EmplaceBack(r, rec, arena_src) must clone
// the overflow slot into *this* buffer's arena. Pre-fix CollectData used the
// 2-arg EmplaceBack on overflow recorders, leaving overflow_idx_ pointing into
// the source buffer's arena; the next RecorderFanOut/DupOverflowSlot then
// dereferenced this->overflow_arena_ (null) → SIGSEGV on max_hits>kInlineCap.
TEST(RayBufferRecorderTest, EmplaceBackSingleRayWithOverflowDupsArena) {
  RayBuffer src(4);
  // Populate src[1] with an overflow recorder of 20 bytes.
  for (uint8_t i = 0; i < 20; i++) {
    src.RecorderAppend(1, static_cast<lumice::IdType>(i + 1));
  }
  ASSERT_TRUE(src.RecorderAt(1).HasOverflow());
  src.size_ = 2;  // mirror production caller bookkeeping

  RayBuffer dst(8);
  dst.EmplaceBack(MakeRay(7), src.RecorderAt(1), src);

  EXPECT_EQ(dst.size_, 1u);
  EXPECT_FLOAT_EQ(dst[0].w_, 7.0f);
  ASSERT_TRUE(dst.RecorderAt(0).HasOverflow());
  EXPECT_EQ(dst.RecorderAt(0).size_, static_cast<uint8_t>(20));

  // dst arena must be independent of src arena.
  ASSERT_NE(dst.OverflowArena(), nullptr) << "dst arena must be lazily allocated by DupOverflowSlot";
  const uint8_t* dst_slot = dst.RecorderDataPtr(0);
  const uint8_t* src_slot = src.RecorderDataPtr(1);
  ASSERT_NE(dst_slot, src_slot);
  for (uint8_t i = 0; i < 20; i++) {
    EXPECT_EQ(dst_slot[i], static_cast<uint8_t>(i + 1)) << "dst byte " << static_cast<int>(i);
  }

  // Mutating src arena must not bleed into dst.
  uint8_t snapshot_dst[lumice::kMaxHits];
  std::memcpy(snapshot_dst, dst_slot, lumice::kMaxHits);
  src.RecorderDataPtr(1)[0] = 0xAA;
  EXPECT_EQ(std::memcmp(dst_slot, snapshot_dst, lumice::kMaxHits), 0) << "dst arena aliased src arena";
}


// task-284 no-truncation contract: the exit-seam record (ExitFaceSeq) must
// carry the FULL recorded path up to kMaxHits, NOT silently cap it at the old
// kInlineCap=15. This mirrors the cpu_trace_backend.cpp exit-record build
// (seq_len = min(recorder.size_, ExitFaceSeq::kCap); memcpy(path.data_, ...))
// against a >15-hit overflow recorder, asserting bytes past index 15 survive.
// Owner-added per code-review Suggestion (direct structural assertion is
// stronger than the PSNR-noise-floor proxy used in AC2).
TEST(RayBufferRecorderTest, ExitFaceSeqCarriesFullPathNoTruncationPast15) {
  // ExitFaceSeq must be able to hold the full path; the whole point of the
  // task-284 fix is decoupling kCap from kInlineCap.
  static_assert(lumice::ExitFaceSeq::kCap == static_cast<uint8_t>(lumice::kMaxHits),
                "ExitFaceSeq::kCap must equal kMaxHits so exit paths are never truncated");
  ASSERT_GT(lumice::ExitFaceSeq::kCap, lumice::RaypathRecorder::kInlineCap);

  RayBuffer buf(4);
  constexpr uint8_t kPathLen = 20;  // > kInlineCap (15), exercises the arena
  for (uint8_t i = 0; i < kPathLen; i++) {
    buf.RecorderAppend(0, static_cast<lumice::IdType>(i + 1));
  }
  const RaypathRecorder& rp = buf.RecorderAt(0);
  ASSERT_TRUE(rp.HasOverflow());
  ASSERT_EQ(rp.size_, kPathLen);

  // Replicate the exit-record builder's copy (cpu_trace_backend.cpp).
  lumice::ExitFaceSeq seq{};
  const uint8_t* src = buf.RecorderDataPtr(0);
  auto seq_len = static_cast<uint8_t>(std::min<size_t>(rp.size_, lumice::ExitFaceSeq::kCap));
  seq.size_ = seq_len;
  std::memcpy(seq.data_, src, seq_len);

  // The 20-byte path must survive intact — no cap at 15.
  EXPECT_EQ(seq.size_, kPathLen) << "exit-seam path truncated; kCap regression";
  for (uint8_t i = 0; i < kPathLen; i++) {
    EXPECT_EQ(seq.data_[i], static_cast<uint8_t>(i + 1)) << "byte " << static_cast<int>(i);
  }
  // Specifically assert the bytes that the OLD kCap=15 design would have dropped.
  EXPECT_EQ(seq.data_[15], static_cast<uint8_t>(16));
  EXPECT_EQ(seq.data_[19], static_cast<uint8_t>(20));
}


// task-284 inline-recorder path must remain a trivial copy via the 3-arg
// overload (no arena allocation, identical observable behaviour to the 2-arg
// overload).
TEST(RayBufferRecorderTest, EmplaceBackSingleRayInlineSkipsArena) {
  RayBuffer src(4);
  src.RecorderAppend(1, static_cast<lumice::IdType>(42));
  ASSERT_FALSE(src.RecorderAt(1).HasOverflow());
  src.size_ = 2;

  RayBuffer dst(8);
  dst.EmplaceBack(MakeRay(3), src.RecorderAt(1), src);

  EXPECT_EQ(dst.size_, 1u);
  EXPECT_FALSE(dst.RecorderAt(0).HasOverflow());
  EXPECT_EQ(dst.RecorderAt(0).size_, static_cast<uint8_t>(1));
  EXPECT_EQ(dst.RecorderAt(0).data_[0], static_cast<uint8_t>(42));
  EXPECT_EQ(dst.OverflowArena(), nullptr) << "inline path must keep dst arena unallocated";
}


TEST(RayBufferRecorderTest, FanOutInlineRecorderTakesTrivialPath) {
  // Inline recorders must not allocate an arena slot during fan-out (this is
  // the hot path under bench(max_hits=8)).
  RayBuffer src(4);
  RayBuffer dst(8);
  for (uint8_t i = 0; i < 5; i++) {
    src.RecorderAppend(1, static_cast<lumice::IdType>(i + 1));
  }
  ASSERT_FALSE(src.RecorderAt(1).HasOverflow());
  src.size_ = 2;

  dst.RecorderFanOut(src, 1, 2, 3);

  EXPECT_FALSE(dst.RecorderAt(2).HasOverflow()) << "inline fan-out must not touch the arena";
  EXPECT_FALSE(dst.RecorderAt(3).HasOverflow());
  EXPECT_EQ(dst.OverflowArena(), nullptr) << "dst arena allocation must stay lazy";
  for (uint8_t i = 0; i < 5; i++) {
    EXPECT_EQ(dst.RecorderAt(2).data_[i], static_cast<uint8_t>(i + 1));
    EXPECT_EQ(dst.RecorderAt(3).data_[i], static_cast<uint8_t>(i + 1));
  }
}


TEST(RayBufferRecorderTest, ResetRewindsArena) {
  RayBuffer buf(4);
  // Fill three overflow recorders.
  for (size_t slot : { 0u, 1u, 2u }) {
    for (uint8_t i = 0; i < 20; i++) {
      buf.RecorderAppend(slot, static_cast<lumice::IdType>(i + 1));
    }
  }
  ASSERT_GT(buf.OverflowUsed(), 0u);
  uint16_t cap_before = buf.OverflowCap();

  buf.Reset(buf.capacity_);

  EXPECT_EQ(buf.OverflowUsed(), 0u) << "Reset must rewind the bump cursor";
  EXPECT_EQ(buf.OverflowCap(), cap_before) << "Reset retains arena capacity";

  // Reset() rewinds the arena cursor but does NOT zero recorders_ in place —
  // that mirrors production where InitRay_other_info calls RecorderClear per
  // ray index at the start of the next MS layer. Emulate that contract:
  buf.RecorderClear(0);
  buf.RecorderAppend(0, static_cast<lumice::IdType>(42));
  EXPECT_FALSE(buf.RecorderAt(0).HasOverflow()) << "small follow-up stays inline";
  EXPECT_EQ(buf.RecorderAt(0).size_, static_cast<uint8_t>(1));
}


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
    buf.EmplaceBack(MakeRay(i), RaypathRecorder{});
  }
  EXPECT_EQ(buf.size_, 4u);
  EXPECT_FALSE(buf.Empty());
  EXPECT_FLOAT_EQ(buf[0].w_, 0.0f);
  EXPECT_FLOAT_EQ(buf[1].w_, 1.0f);
  EXPECT_FLOAT_EQ(buf[2].w_, 2.0f);
  EXPECT_FLOAT_EQ(buf[3].w_, 3.0f);

  // Guard rejects further writes; size and existing data must remain unchanged.
  buf.EmplaceBack(MakeRay(99), RaypathRecorder{});
  EXPECT_EQ(buf.size_, 4u);
  EXPECT_FLOAT_EQ(buf[0].w_, 0.0f);
  EXPECT_FLOAT_EQ(buf[3].w_, 3.0f);

  // Repeated guard hits should also leave existing data intact.
  for (int i = 0; i < 5; i++) {
    buf.EmplaceBack(MakeRay(88), RaypathRecorder{});
  }
  EXPECT_EQ(buf.size_, 4u);
  EXPECT_FLOAT_EQ(buf[0].w_, 0.0f);
  EXPECT_FLOAT_EQ(buf[3].w_, 3.0f);
}


TEST(RayBufferTest, EmplaceBackBatchOnEmptyDst) {
  RayBuffer src(10);
  for (int i = 0; i < 5; i++) {
    // Identifiable recorder per src element: face id = i+1 so the batch-copy
    // codepath has functional coverage — if recorders_[size_] = buffer.recorders_[i]
    // is silently dropped, RecorderAt(j).size_ stays 0 on dst and assertions fire.
    RaypathRecorder rec;
    rec << static_cast<lumice::IdType>(i + 1);
    src.EmplaceBack(MakeRay(i), rec);
  }
  EXPECT_EQ(src.size_, 5u);

  RayBuffer dst(10);
  dst.EmplaceBack(src, 1, 3);
  EXPECT_EQ(dst.size_, 3u);
  EXPECT_FLOAT_EQ(dst[0].w_, 1.0f);
  EXPECT_FLOAT_EQ(dst[1].w_, 2.0f);
  EXPECT_FLOAT_EQ(dst[2].w_, 3.0f);

  // Recorder content: dst[0..2] originate from src[1..3] → face ids 2/3/4.
  EXPECT_EQ(dst.RecorderAt(0).size_, 1u);
  EXPECT_EQ(dst.RecorderAt(0).data_[0], static_cast<uint8_t>(2));
  EXPECT_EQ(dst.RecorderAt(1).size_, 1u);
  EXPECT_EQ(dst.RecorderAt(1).data_[0], static_cast<uint8_t>(3));
  EXPECT_EQ(dst.RecorderAt(2).size_, 1u);
  EXPECT_EQ(dst.RecorderAt(2).data_[0], static_cast<uint8_t>(4));

  // Calling with kInfSize (size_t::max) must be clamped by buffer.size_,
  // not propagate as an overflow.
  RayBuffer dst2(10);
  dst2.EmplaceBack(src, 0, kInfSize);
  EXPECT_EQ(dst2.size_, 5u);  // clamped to src.size_
  EXPECT_FLOAT_EQ(dst2[0].w_, 0.0f);
  EXPECT_FLOAT_EQ(dst2[4].w_, 4.0f);

  // Recorder content: dst2[0..4] mirror src[0..4] → face ids 1..5.
  EXPECT_EQ(dst2.RecorderAt(0).size_, 1u);
  EXPECT_EQ(dst2.RecorderAt(0).data_[0], static_cast<uint8_t>(1));
  EXPECT_EQ(dst2.RecorderAt(4).size_, 1u);
  EXPECT_EQ(dst2.RecorderAt(4).data_[0], static_cast<uint8_t>(5));
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
    buf.EmplaceBack(MakeRay(i), RaypathRecorder{});
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
    buf.EmplaceBack(MakeRay(i), RaypathRecorder{});
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


// =============== RayBuffer copy/move (self-owned value semantics) ===============
//
// These tests pin the contract that SimData's four special members now delegate
// to (see sim_data.cpp). Removing or weakening any of them would re-introduce
// the hand-syncing surface that the #246.2 / #247.1 split fixed.

namespace {

// Helper: build a populated RayBuffer with size < capacity, identifiable
// rays + recorders. capacity=6 / size=4 mirrors the size != capacity shape
// used by MakePopulatedSimData so copy-by-capacity is exercised.
RayBuffer MakePopulatedRayBuffer() {
  RayBuffer buf(6);
  for (int i = 0; i < 4; i++) {
    RaypathRecorder rec;
    rec << static_cast<lumice::IdType>(i + 1);
    buf.EmplaceBack(MakeRay(i), rec);
  }
  return buf;
}

}  // namespace


TEST(RayBufferTest, CopyConstructDeepCopy) {
  auto src = MakePopulatedRayBuffer();
  RayBuffer dst(src);

  EXPECT_EQ(dst.capacity_, 6u);
  EXPECT_EQ(dst.size_, 4u);
  ASSERT_NE(dst.rays(), nullptr);
  EXPECT_NE(dst.rays(), src.rays()) << "deep copy expected: pointers must differ";

  for (size_t i = 0; i < 4; i++) {
    EXPECT_FLOAT_EQ(dst[i].w_, src[i].w_);
    EXPECT_EQ(dst.RecorderAt(i).size_, 1u);
    EXPECT_EQ(dst.RecorderAt(i).data_[0], static_cast<uint8_t>(i + 1));
  }

  // Deep copy independence: mutating dst must not affect src.
  dst[0].w_ = 999.0f;
  EXPECT_FLOAT_EQ(src[0].w_, 0.0f);

  // Edge case: copy of a default-constructed (capacity=0) buffer must not
  // dereference nullptr in the memcpy path.
  RayBuffer empty;
  // NOLINTNEXTLINE(performance-unnecessary-copy-initialization) — the copy IS the test subject
  RayBuffer dst_empty(empty);
  EXPECT_EQ(dst_empty.capacity_, 0u);
  EXPECT_EQ(dst_empty.size_, 0u);
  EXPECT_EQ(dst_empty.rays(), nullptr);
}


TEST(RayBufferTest, CopyAssignDeepCopy) {
  auto src = MakePopulatedRayBuffer();
  RayBuffer dst(3);  // Different initial capacity — exercises realloc path.
  dst = src;

  EXPECT_EQ(dst.capacity_, 6u);
  EXPECT_EQ(dst.size_, 4u);
  EXPECT_NE(dst.rays(), src.rays());
  for (size_t i = 0; i < 4; i++) {
    EXPECT_FLOAT_EQ(dst[i].w_, src[i].w_);
    EXPECT_EQ(dst.RecorderAt(i).data_[0], static_cast<uint8_t>(i + 1));
  }

  // Self-assignment guard — alias via reference to bypass -Wself-assign-overloaded.
  const size_t kSrcCap = src.capacity_;
  const size_t kSrcSize = src.size_;
  const float kRay0W = src[0].w_;
  RayBuffer& self_alias = src;
  self_alias = src;
  EXPECT_EQ(src.capacity_, kSrcCap);
  EXPECT_EQ(src.size_, kSrcSize);
  EXPECT_FLOAT_EQ(src[0].w_, kRay0W);
}


TEST(RayBufferTest, CopyAssignSelfAssignWithOverflow) {
  // Regression guard: copy-assign self with overflow arena must leave data intact.
  RayBuffer buf(4);
  RaypathRecorder empty_rec;
  empty_rec.Clear();
  buf.EmplaceBack(MakeRay(1), empty_rec);
  for (uint8_t i = 0; i < 20; i++) {
    buf.RecorderAppend(0, static_cast<lumice::IdType>(i + 1));
  }
  ASSERT_TRUE(buf.RecorderAt(0).HasOverflow());
  const uint16_t kUsedBefore = buf.OverflowUsed();
  const uint8_t kSizeBefore = buf.RecorderAt(0).size_;
  const std::vector<uint8_t> kDataBefore(buf.RecorderDataPtr(0), buf.RecorderDataPtr(0) + kSizeBefore);

  RayBuffer& self_alias = buf;
  self_alias = buf;

  EXPECT_EQ(buf.OverflowUsed(), kUsedBefore) << "self-assign must not alter overflow_used_";
  EXPECT_EQ(buf.RecorderAt(0).size_, kSizeBefore) << "self-assign must preserve recorder size_";
  EXPECT_EQ(std::vector<uint8_t>(buf.RecorderDataPtr(0), buf.RecorderDataPtr(0) + kSizeBefore), kDataBefore)
      << "self-assign must preserve overflow arena data";
}

TEST(RayBufferTest, MoveConstructTransfersOwnership) {
  auto src = MakePopulatedRayBuffer();
  RaySeg* src_rays_ptr = src.rays();
  const RaypathRecorder* src_rec_ptr = &src.RecorderAt(0);

  RayBuffer dst(std::move(src));

  EXPECT_EQ(dst.capacity_, 6u);
  EXPECT_EQ(dst.size_, 4u);
  EXPECT_EQ(dst.rays(), src_rays_ptr) << "ownership transfer expected (no copy)";
  EXPECT_EQ(&dst.RecorderAt(0), src_rec_ptr) << "recorders_ ownership transfer expected";

  // Moved-from source zeroed by RayBuffer move ctor.
  EXPECT_EQ(src.capacity_, 0u);
  EXPECT_EQ(src.size_, 0u);
  EXPECT_EQ(src.rays_, nullptr);
  EXPECT_EQ(src.recorders_, nullptr);
}


TEST(RayBufferTest, MoveAssignTransfersOwnership) {
  auto src = MakePopulatedRayBuffer();
  RaySeg* src_rays_ptr = src.rays();
  RayBuffer dst(3);
  dst = std::move(src);

  EXPECT_EQ(dst.capacity_, 6u);
  EXPECT_EQ(dst.size_, 4u);
  EXPECT_EQ(dst.rays(), src_rays_ptr);
  EXPECT_EQ(src.capacity_, 0u);
  EXPECT_EQ(src.size_, 0u);
  EXPECT_EQ(src.rays_, nullptr);

  // Self-move-assignment must preserve all fields (source code has &other == this guard).
  auto self = MakePopulatedRayBuffer();
  const size_t kCap = self.capacity_;
  const size_t kSize = self.size_;
  RaySeg* snap_ptr = self.rays();
  const float kRay0W = self[0].w_;

  RayBuffer& self_ref = self;
  self_ref = std::move(self);

  EXPECT_EQ(self.capacity_, kCap);
  EXPECT_EQ(self.size_, kSize);
  EXPECT_EQ(self.rays(), snap_ptr);
  EXPECT_FLOAT_EQ(self[0].w_, kRay0W);
}


TEST(RayBufferTest, EmplaceBackOverflowDeepCopy) {
  // Build src: add one ray with an inline recorder, then overflow it via RecorderAppend.
  RayBuffer src(4);
  RaypathRecorder empty_rec;
  empty_rec.Clear();
  src.EmplaceBack(MakeRay(7), empty_rec);
  for (uint8_t i = 0; i < 20; i++) {
    src.RecorderAppend(0, static_cast<lumice::IdType>(i + 1));
  }
  ASSERT_EQ(src.size_, 1u);
  ASSERT_TRUE(src.RecorderAt(0).HasOverflow()) << "precondition: src recorder must overflow";

  // EmplaceBack via the buffer-to-buffer overload copies slot 0 into dst.
  RayBuffer dst(4);
  dst.EmplaceBack(src, 0, 1);

  ASSERT_EQ(dst.size_, 1u);
  const RaypathRecorder& dst_rec = dst.RecorderAt(0);
  const RaypathRecorder& src_rec = src.RecorderAt(0);
  EXPECT_EQ(dst_rec.size_, src_rec.size_) << "size must match after EmplaceBack";
  EXPECT_NE(dst.RecorderDataPtr(0), src.RecorderDataPtr(0)) << "dst arena must be independent of src";
  EXPECT_EQ(std::memcmp(dst.RecorderDataPtr(0), src.RecorderDataPtr(0), src_rec.size_), 0)
      << "overflow data must be faithfully copied";
}

// =============== SimData Tests ===============

TEST(SimDataTest, CopyConstructDeepCopy) {
  auto original = MakePopulatedSimData();
  SimData copy(original);

  // Scalar field assertions, each with a failure message identifying which
  // field was missed (helps catch the "manual copy missed a field" bug class).
  EXPECT_FLOAT_EQ(copy.curr_wl_, 550.0f) << "curr_wl_ not copied";
  EXPECT_EQ(copy.generation_, 42u) << "generation_ not copied";
  EXPECT_EQ(copy.root_ray_count_, 7u) << "root_ray_count_ not copied";

  // rays_ field assertions.
  EXPECT_EQ(copy.rays_.size_, 5u);
  EXPECT_EQ(copy.rays_.capacity_, 8u);
  for (int i = 0; i < 5; i++) {
    EXPECT_FLOAT_EQ(copy.rays_[i].w_, original.rays_[i].w_);
  }

  // Vector field equality.
  EXPECT_EQ(copy.outgoing_d_, original.outgoing_d_);
  EXPECT_EQ(copy.outgoing_w_, original.outgoing_w_);
  EXPECT_EQ(copy.outgoing_wl_, original.outgoing_wl_);  // scrum-268.8 (DR-3)
  EXPECT_EQ(copy.crystals_.size(), original.crystals_.size());
  ASSERT_EQ(copy.exit_records_.size(), original.exit_records_.size());
  EXPECT_EQ(copy.exit_records_[0].crystal_id, 7u);
  EXPECT_EQ(copy.exit_records_[0].path.size_, 2u);
  EXPECT_EQ(copy.exit_records_[0].path.data_[1], 5u);
  EXPECT_EQ(copy.exit_records_[1].ms_layer_idx, 1u);
  EXPECT_EQ(copy.xyz_pixel_data_, original.xyz_pixel_data_);  // S1 device-fused
  EXPECT_FLOAT_EQ(copy.xyz_landed_weight_, 1.5f);
  EXPECT_EQ(copy.crystal_count_, 4u) << "crystal_count_ not copied";         // task-exit-seam-crystal-count
  EXPECT_EQ(copy.sim_scene_credit_, 11u) << "sim_scene_credit_ not copied";  // scrum-312

  // Deep copy independence — each pointer/container field independently.
  copy.rays_[0].w_ = 999.0f;
  EXPECT_FLOAT_EQ(original.rays_[0].w_, 0.0f) << "rays_ not deep-copied";

  copy.outgoing_d_.clear();
  EXPECT_EQ(original.outgoing_d_.size(), 6u) << "outgoing_d_ not deep-copied";

  copy.outgoing_w_.clear();
  EXPECT_EQ(original.outgoing_w_.size(), 2u) << "outgoing_w_ not deep-copied";

  copy.outgoing_wl_.clear();
  EXPECT_EQ(original.outgoing_wl_.size(), 2u) << "outgoing_wl_ not deep-copied";

  copy.exit_records_.clear();
  EXPECT_EQ(original.exit_records_.size(), 2u) << "exit_records_ not deep-copied";

  copy.crystals_.clear();
  EXPECT_EQ(original.crystals_.size(), 1u) << "crystals_ not deep-copied";
}


TEST(SimDataTest, CopyAssignmentDeepCopy) {
  auto original = MakePopulatedSimData();
  SimData target(3);  // Different initial capacity, exercises rays_ realloc path.
  target = original;

  // Field completeness (mirror of copy ctor test).
  EXPECT_FLOAT_EQ(target.curr_wl_, 550.0f) << "curr_wl_ not assigned";
  EXPECT_EQ(target.generation_, 42u) << "generation_ not assigned";
  EXPECT_EQ(target.root_ray_count_, 7u) << "root_ray_count_ not assigned";
  EXPECT_EQ(target.rays_.size_, 5u);
  EXPECT_EQ(target.rays_.capacity_, 8u);
  for (int i = 0; i < 5; i++) {
    EXPECT_FLOAT_EQ(target.rays_[i].w_, original.rays_[i].w_);
  }
  EXPECT_EQ(target.outgoing_d_, original.outgoing_d_);
  EXPECT_EQ(target.outgoing_w_, original.outgoing_w_);
  EXPECT_EQ(target.outgoing_wl_, original.outgoing_wl_);  // scrum-268.8 (DR-3)
  EXPECT_EQ(target.crystals_.size(), 1u);
  ASSERT_EQ(target.exit_records_.size(), 2u);
  EXPECT_EQ(target.exit_records_[0].crystal_id, 7u);
  EXPECT_EQ(target.xyz_pixel_data_, original.xyz_pixel_data_);  // S1 device-fused
  EXPECT_FLOAT_EQ(target.xyz_landed_weight_, 1.5f);
  EXPECT_EQ(target.crystal_count_, 4u) << "crystal_count_ not assigned";         // task-exit-seam-crystal-count
  EXPECT_EQ(target.sim_scene_credit_, 11u) << "sim_scene_credit_ not assigned";  // scrum-312

  // Deep copy independence.
  target.rays_[0].w_ = 999.0f;
  EXPECT_FLOAT_EQ(original.rays_[0].w_, 0.0f) << "rays_ not deep-assigned";
  target.exit_records_.clear();
  EXPECT_EQ(original.exit_records_.size(), 2u) << "exit_records_ not deep-assigned";
  target.crystals_.clear();
  EXPECT_EQ(original.crystals_.size(), 1u) << "crystals_ not deep-assigned";

  // Self-assignment must preserve all fields (source code has &other == this guard).
  // Snapshot → self-assign → assert preservation (NOT assert clearing).
  // Use an alias reference to bypass -Wself-assign-overloaded.
  auto orig = MakePopulatedSimData();
  const float kSnapCurrWl = orig.curr_wl_;
  const uint64_t kSnapGen = orig.generation_;
  const size_t kSnapRoot = orig.root_ray_count_;
  const size_t kSnapRaysSize = orig.rays_.size_;
  const size_t kSnapRaysCap = orig.rays_.capacity_;
  const float kSnapRay0W = orig.rays_[0].w_;
  const size_t kSnapCrystalsSize = orig.crystals_.size();

  SimData& self_alias = orig;
  self_alias = orig;

  EXPECT_FLOAT_EQ(orig.curr_wl_, kSnapCurrWl);
  EXPECT_EQ(orig.generation_, kSnapGen);
  EXPECT_EQ(orig.root_ray_count_, kSnapRoot);
  EXPECT_EQ(orig.rays_.size_, kSnapRaysSize);
  EXPECT_EQ(orig.rays_.capacity_, kSnapRaysCap);
  EXPECT_FLOAT_EQ(orig.rays_[0].w_, kSnapRay0W);
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
  EXPECT_EQ(moved.generation_, 42u);
  EXPECT_EQ(moved.root_ray_count_, 7u);
  EXPECT_EQ(moved.outgoing_d_.size(), 6u);
  EXPECT_EQ(moved.outgoing_w_.size(), 2u);
  EXPECT_EQ(moved.exit_records_.size(), 2u);
  EXPECT_EQ(moved.crystals_.size(), 1u);
  EXPECT_EQ(moved.xyz_pixel_data_.size(), 3u);  // S1 device-fused
  EXPECT_FLOAT_EQ(moved.xyz_landed_weight_, 1.5f);
  EXPECT_EQ(moved.crystal_count_, 4u) << "crystal_count_ not moved";         // task-exit-seam-crystal-count
  EXPECT_EQ(moved.sim_scene_credit_, 11u) << "sim_scene_credit_ not moved";  // scrum-312

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
  EXPECT_TRUE(original.outgoing_d_.empty());
  EXPECT_TRUE(original.outgoing_w_.empty());
  EXPECT_TRUE(original.exit_records_.empty());

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
  EXPECT_EQ(dst.crystals_.size(), 1u);
  EXPECT_EQ(dst.crystal_count_, 4u) << "crystal_count_ not move-assigned";         // task-exit-seam-crystal-count
  EXPECT_EQ(dst.sim_scene_credit_, 11u) << "sim_scene_credit_ not move-assigned";  // scrum-312

  // Source moved-from state.
  EXPECT_EQ(src.rays_.rays_, nullptr);
  EXPECT_EQ(src.rays_.size_, 0u);
  EXPECT_EQ(src.rays_.capacity_, 0u);
  EXPECT_TRUE(src.crystals_.empty());

  // Self-move-assignment must preserve all fields (source code has
  // &other == this guard). Snapshot → self-move → assert preservation.
  auto self = MakePopulatedSimData();
  const float kSnapCurrWl = self.curr_wl_;
  const uint64_t kSnapGen = self.generation_;
  const size_t kSnapRaysSize = self.rays_.size_;
  RaySeg* snap_rays_ptr = self.rays_.rays_.get();
  const size_t kSnapOutgoingWSize = self.outgoing_w_.size();
  const size_t kSnapCrystalsSize = self.crystals_.size();

  // Use an alias reference to bypass -Wself-move warning.
  SimData& self_ref = self;
  self_ref = std::move(self);

  EXPECT_FLOAT_EQ(self.curr_wl_, kSnapCurrWl);
  EXPECT_EQ(self.generation_, kSnapGen);
  EXPECT_EQ(self.rays_.size_, kSnapRaysSize);
  EXPECT_EQ(self.rays_.rays_.get(), snap_rays_ptr);
  EXPECT_EQ(self.outgoing_w_.size(), kSnapOutgoingWSize);
  EXPECT_EQ(self.crystals_.size(), kSnapCrystalsSize);
}
