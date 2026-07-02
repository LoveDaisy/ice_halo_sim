// Unit test for the 64-bit ray-index infrastructure that replaces
// `lumice::NarrowPcgRayBase` (task-gpu-rng-ray-index-uint64):
//
//   - `lm_pcg::pcg_advance_hi` / `lm_pcg::pcg_seed_with_high` in
//     src/core/shared/pcg_shared.h — device-visible carry-detect + conditional
//     high-word XOR-mix. hi==0 (session under 2^32 rays) is a pure no-op so
//     the hot-path PCG stream stays bit-exact with the pre-fix behavior in the
//     entire in-range regime; hi!=0 diverges across epochs so two rays whose
//     (base_lo + tid) coincide after a uint32 wrap DO NOT collapse onto the
//     same PCG stream (the scrum-267.3 silent-under-sampling failure mode).
//
//   - `lumice::SplitPcgRayBase` in src/core/backend/trace_backend.hpp —
//     host-side lo/hi split of a size_t ray-base. Non-throwing (replaces the
//     4.29e9-cap `NarrowPcgRayBase` guard that used to force a CPU fallback).
//
// The plan (§ Step 1 test points) mandates three CPU-only assertions:
//   1. hi==0 bit-exact vs the pre-fix seed derivation (no silent parity
//      regression in the in-range regime — this is issue.md AC "hash mixing
//      must be bit-identical for high==0").
//   2. Cross-2^32 rays constructed via lo overflow do NOT collide (a base with
//      lo near UINT32_MAX + a tid range that spans the wrap yields per-ray
//      pcg_uniform draws that are all distinct within a small sample).
//   3. Different hi values produce different mixed seeds (defends against a
//      hypothetical pcg_hash(hi) collision at small hi; not a re-proof of
//      pcg_hash avalanche, just a sanity trip-wire).
//
// The full uint64 fix supersedes the "cap at 4.29e9 rays" guard: past 2^32
// rays there is no more silent stream collapse, so no runtime error is needed.

#include <gtest/gtest.h>

#include <cstdint>
#include <set>

#include "core/backend/trace_backend.hpp"
#include "core/shared/pcg_shared.h"

namespace {

constexpr uint32_t kU32Max = UINT32_MAX;

// The pre-fix seed derivation used inside `pcg_uniform`: h = pcg_hash(seed ^
// pcg_hash(global_idx * 1000003u + slot)). Recomputes a single draw under the
// old rules so the hi==0 branch can be diff-tested against it. The reference
// is intentionally inlined here instead of imported so a future refactor of
// the shared header cannot silently drift this baseline.
uint32_t PreFixHash(uint32_t seed, uint32_t global_idx, uint32_t slot) {
  return lm_pcg::pcg_hash(seed ^ lm_pcg::pcg_hash(global_idx * 1000003u + slot));
}

// Emulate a single device draw under the new rules: hi is mixed into the
// stream seed via pcg_seed_with_high; global_idx is base_lo + tid (allowed to
// wrap, matching the device-side uint32 arithmetic).
uint32_t PostFixHash(uint32_t seed, uint32_t base_lo, uint32_t base_hi, uint32_t tid, uint32_t slot) {
  uint32_t hi = lm_pcg::pcg_advance_hi(base_lo, base_hi, tid);
  uint32_t mixed_seed = lm_pcg::pcg_seed_with_high(seed, hi);
  uint32_t global_idx = base_lo + tid;  // wrap-allowed
  return lm_pcg::pcg_hash(mixed_seed ^ lm_pcg::pcg_hash(global_idx * 1000003u + slot));
}

// ---------------------------------------------------------------------------
// SplitPcgRayBase: lo/hi bit-shuffle correctness.
// ---------------------------------------------------------------------------

TEST(SplitPcgRayBase, ZeroAndSmall) {
  auto lh = lumice::SplitPcgRayBase(0u);
  EXPECT_EQ(lh.lo, 0u);
  EXPECT_EQ(lh.hi, 0u);
  lh = lumice::SplitPcgRayBase(12345u);
  EXPECT_EQ(lh.lo, 12345u);
  EXPECT_EQ(lh.hi, 0u);
}

TEST(SplitPcgRayBase, JustUnderAndAtU32Max) {
  auto lh = lumice::SplitPcgRayBase(static_cast<size_t>(kU32Max));
  EXPECT_EQ(lh.lo, kU32Max);
  EXPECT_EQ(lh.hi, 0u);
  lh = lumice::SplitPcgRayBase(static_cast<size_t>(kU32Max) + 1u);
  EXPECT_EQ(lh.lo, 0u);
  EXPECT_EQ(lh.hi, 1u);
}

TEST(SplitPcgRayBase, MultiEpoch) {
  // 5e9 is well past 2^32 (~4.29e9), lands in the hi=1 epoch. Recompose to
  // verify: (hi << 32) | lo == the original ray_base.
  size_t rb = 5'000'000'000ull;
  auto lh = lumice::SplitPcgRayBase(rb);
  EXPECT_EQ(lh.hi, 1u);
  EXPECT_EQ(lh.lo, static_cast<uint32_t>(rb - (1ull << 32)));
  size_t recomposed = (static_cast<size_t>(lh.hi) << 32) | static_cast<size_t>(lh.lo);
  EXPECT_EQ(recomposed, rb);
}

// ---------------------------------------------------------------------------
// pcg_advance_hi + pcg_seed_with_high: hi==0 bit-exact identity.
// ---------------------------------------------------------------------------

TEST(PcgAdvanceHi, ZeroBaseZeroHi) {
  // No wrap, hi input == 0 → advance returns 0.
  EXPECT_EQ(lm_pcg::pcg_advance_hi(0u, 0u, 0u), 0u);
  EXPECT_EQ(lm_pcg::pcg_advance_hi(0u, 0u, 1u), 0u);
  EXPECT_EQ(lm_pcg::pcg_advance_hi(12345u, 0u, 32768u), 0u);
}

TEST(PcgAdvanceHi, CarryAcrossU32Max) {
  // base_lo near UINT32_MAX + a tid that wraps → carry into hi.
  uint32_t base_lo = kU32Max - 5u;  // room for tid 0..5 without wrap
  EXPECT_EQ(lm_pcg::pcg_advance_hi(base_lo, 3u, 0u), 3u);
  EXPECT_EQ(lm_pcg::pcg_advance_hi(base_lo, 3u, 5u), 3u);  // lo=UINT32_MAX, no wrap
  EXPECT_EQ(lm_pcg::pcg_advance_hi(base_lo, 3u, 6u), 4u);  // wraps once → hi+1
  EXPECT_EQ(lm_pcg::pcg_advance_hi(base_lo, 3u, 100u), 4u);
}

TEST(PcgSeedWithHigh, HiZeroIsIdentity) {
  // The critical AC assertion: hi==0 is a pure no-op, so the entire
  // in-range (session < 2^32 rays) PCG hot path is bit-exact with the
  // pre-fix behavior. Sweep across a range of seeds so a hypothetical
  // hi==0 branch that accidentally XORs something nonzero is caught.
  for (uint32_t seed : { 0u, 1u, 12345u, 0xDEADBEEFu, 0xFFFFFFFFu }) {
    EXPECT_EQ(lm_pcg::pcg_seed_with_high(seed, 0u), seed);
  }
}

TEST(PcgSeedWithHigh, HiNonZeroDiverges) {
  // Small hi values yield distinct mixed seeds → pcg_hash(hi) is not
  // producing a trivial collision at small integers (defence trip-wire,
  // not a re-proof of pcg_hash avalanche).
  uint32_t seed = 0xC0FFEE00u;
  std::set<uint32_t> mixed_seeds;
  for (uint32_t hi = 1u; hi <= 8u; ++hi) {
    mixed_seeds.insert(lm_pcg::pcg_seed_with_high(seed, hi));
  }
  EXPECT_EQ(mixed_seeds.size(), 8u);
  // And each mixed seed must differ from the hi==0 (unmixed) seed too.
  for (auto s : mixed_seeds) {
    EXPECT_NE(s, seed);
  }
}

// ---------------------------------------------------------------------------
// In-range parity: draws under the new rules are bit-identical to pre-fix
// draws for every (seed, base_lo, tid, slot) that stays under 2^32 rays.
// This is the direct code-level proof of the issue.md AC:
//   "hash mixing changes to include high bits, and high==0 (in-range <2^32)
//    branch must be BIT-IDENTICAL to the current behavior."
// ---------------------------------------------------------------------------

TEST(InRangeParity, BitExactVsPreFix) {
  // Sweep a small grid of seeds/bases/tids/slots. The pcg_uniform hot path
  // reduces to PreFixHash(seed, base_lo + tid, slot); PostFixHash under the
  // new rules is that same function whenever base_lo + tid does NOT wrap
  // (hi==0), so this is a mechanical equality across the whole grid.
  uint32_t seeds[] = { 0u, 1u, 0x12345678u, 0xDEADBEEFu };
  uint32_t bases[] = { 0u, 100u, 65536u, kU32Max / 2u };
  uint32_t tids[] = { 0u, 1u, 32u, 128u };
  uint32_t slots[] = { 0u, 1u, 7u, 20u };
  for (uint32_t seed : seeds) {
    for (uint32_t base : bases) {
      for (uint32_t tid : tids) {
        for (uint32_t slot : slots) {
          uint32_t ref = PreFixHash(seed, base + tid, slot);
          uint32_t got = PostFixHash(seed, base, /*base_hi=*/0u, tid, slot);
          EXPECT_EQ(ref, got) << "seed=" << seed << " base_lo=" << base << " tid=" << tid << " slot=" << slot;
        }
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Cross-2^32 non-collision: this is the AC "new stress unit test that
// verifies constructing a large ray_base (directly injected, not by running
// 4B rays) does not produce identical random streams for two rays whose
// global_idx would collide under the pre-fix uint32 wrap".
// ---------------------------------------------------------------------------

TEST(CrossU32, WrapDoesNotCollapseStreams) {
  // Base sits N below UINT32_MAX; tid range spans 0..2N so tids [0..N-1]
  // stay in the hi=H epoch (lo = base_lo + tid) while tids [N..2N-1] wrap
  // into the hi=H+1 epoch. The critical rays are: (base_lo, hi=H, tid=N+k)
  // wraps to lo = k in the hi=H+1 epoch; under the pre-fix rules that same
  // lo = k would also have appeared in the hi=H epoch as (base_lo=0, tid=k)
  // — the SAME PCG stream. Under the new rules the mixed seed differs
  // between hi=H and hi=H+1 so the wrapped rays do NOT collide.
  uint32_t base_hi = 3u;  // arbitrary non-zero epoch
  uint32_t base_lo = kU32Max - 5u;
  uint32_t tids_span = 12u;  // 0..5 → no wrap, 6..11 → wrap
  uint32_t seed = 0x1234ABCDu;
  uint32_t slot = 0u;

  std::set<uint32_t> draws;
  for (uint32_t tid = 0u; tid < tids_span; ++tid) {
    draws.insert(PostFixHash(seed, base_lo, base_hi, tid, slot));
  }
  // Each ray must produce a distinct hash: 12 rays → 12 distinct draws.
  // If the hi-mix were missing, tids 6..11 (wrapped lo=0..5) would collide
  // with the hi=H stream of some other (base, tid) combo — but more importantly
  // the SPECIFIC AC-relevant collision is between a hi=H ray at global_idx=X
  // and a hi=H+1 ray at global_idx=X (same lo). We test that directly below.
  EXPECT_EQ(draws.size(), tids_span);

  // Direct AC collision test: same lo, different hi → different stream.
  // Construct two rays whose (base_lo + tid) lands on identical uint32 values
  // but whose hi differs by 1 (they belong to different epochs). Under the
  // pre-fix rules these would be a silent stream collapse.
  uint32_t target_lo = 42u;  // pick any small value
  // Ray A: hi=H, base_lo=0, tid=target_lo → lo = 42, hi = H (no wrap).
  uint32_t hash_a = PostFixHash(seed, /*base_lo=*/0u, /*base_hi=*/base_hi, target_lo, slot);
  // Ray B: hi=H+1, force wrap: base_lo = UINT32_MAX + 1 - target_lo (in uint32
  // that's -target_lo), tid = 2*target_lo → base_lo + tid = target_lo, wraps
  // once → hi = H+1. Simpler: pick base_lo = UINT32_MAX - target_lo + 1, tid = 2*target_lo
  // so base_lo + tid = target_lo + UINT32_MAX + 1 = target_lo (wrapped).
  uint32_t base_lo_b = kU32Max - target_lo + 1u;
  uint32_t tid_b = 2u * target_lo;
  ASSERT_EQ(base_lo_b + tid_b, target_lo);  // sanity: they collide in the low 32 bits
  ASSERT_EQ(lm_pcg::pcg_advance_hi(base_lo_b, base_hi, tid_b), base_hi + 1u);
  uint32_t hash_b = PostFixHash(seed, base_lo_b, base_hi, tid_b, slot);
  // Different hi → different mixed seed → different draw. This is the failure
  // mode that pre-fix code silently allowed to collide.
  EXPECT_NE(hash_a, hash_b);
}

}  // namespace
