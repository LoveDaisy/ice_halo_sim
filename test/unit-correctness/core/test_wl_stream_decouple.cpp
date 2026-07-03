// CPU-only unit test for lm_pcg::BuildWlStream and kWlStreamNonce
// (task-gpu-wl-stream-decouple-green-tint / issue.md).
//
// The device-side bug fixed by this task was: `gen_root_kernel` sampled the
// per-ray wavelength on `slot = 20u` while reusing the same `mixed_seed` as
// the orientation stream, betting that orientation never consumed 20 slots.
// Under a near-pole GenericReject path (laplacian mean=0), orientation CAN
// consume ≥20 slots on ~10% of rays, colliding wl with an orientation draw
// → wl correlates with crystal orientation → red end suppressed → green tint.
//
// The fix: BuildWlStream XORs `kWlStreamNonce` into the seed, so the wl draw
// lives in a completely independent PCG seed domain. This test provides a
// second-per-scale mechanical trip-wire (no Metal/CUDA device needed) that:
//   (a) the seed is actually mixed (XOR applied);
//   (b) the nonce is non-zero AND distinct from every other stream nonce in
//       use anywhere in the codebase (so the "distinct seed domain" invariant
//       is a whole-project fact, not just a two-nonce check);
//   (c) the first pcg_uniform draw from BuildWlStream does not equal any of
//       the first 21 draws from the orientation stream at the same
//       (mixed_seed, global_idx) — i.e. the seed-domain isolation actually
//       decouples the wl draw from the historically-colliding slot 20 and
//       every other slot the orientation stream could consume.

#include <gtest/gtest.h>

#include <cstdint>
#include <set>

#include "core/shared/pcg_shared.h"

namespace {

// Manually-maintained static SNAPSHOT of the other stream nonces (their real
// definitions live in metal_trace_backend.mm / cuda_trace_backend.cu, which
// this host-only test cannot #include). This is a snapshot, NOT a live
// cross-file guard: it only proves kWlStreamNonce differs from these recorded
// values. If someone later changes a real nonce in its defining file without
// updating this copy, the test will keep passing against the stale value — so
// any nonce change in those files must also update this list (and the
// clearinghouse comment in pcg_shared.h). The proper drift-proof fix would be
// to hoist all nonces into pcg_shared.h as shared symbols; deferred as it
// touches both backend files and is out of this task's scope.
constexpr uint32_t kTransitNonceValue = 0xA5A5A5A5u;
constexpr uint32_t kCudaGateNonceValue = 0x5A5A5A5Au;
constexpr uint32_t kCudaGenNonceValue = 0x3C9A7F11u;
constexpr uint32_t kMetalShuffleNonceValue = 0xB17CA3D9u;
constexpr uint32_t kCudaDrainNonceValue = 0xD5A1B3C7u;

TEST(WlStreamDecouple, NonceIsNonZeroAndDistinct) {
  // (a) non-zero: a zero nonce would silently degrade to the pre-fix behavior
  // (wl_stream.seed == mixed_seed), reintroducing the green-tint bug.
  EXPECT_NE(lm_pcg::kWlStreamNonce, 0u);
  // (b) pairwise distinct vs all other stream nonces in the codebase. Using
  // std::set to catch any two-way collision in one shot.
  std::set<uint32_t> all_nonces = {
    lm_pcg::kWlStreamNonce, kTransitNonceValue,      kCudaGateNonceValue,
    kCudaGenNonceValue,     kMetalShuffleNonceValue, kCudaDrainNonceValue,
  };
  EXPECT_EQ(all_nonces.size(), 6u);
}

TEST(WlStreamDecouple, SeedIsXorMixed) {
  // BuildWlStream must actually apply the XOR — a silent off-by-one that
  // dropped it (e.g. `s.seed = mixed_seed;`) would recreate the bug.
  uint32_t mixed_seed = 0xC0FFEE00u;
  uint32_t global_idx = 42u;
  lm_pcg::PcgStream s = lm_pcg::BuildWlStream(mixed_seed, global_idx);
  EXPECT_EQ(s.seed, mixed_seed ^ lm_pcg::kWlStreamNonce);
  EXPECT_NE(s.seed, mixed_seed);
  EXPECT_EQ(s.global_idx, global_idx);
  EXPECT_EQ(s.slot, 0u);
}

TEST(WlStreamDecouple, FirstDrawDiffersFromAllOrientationSlots) {
  // The direct code-level proof that seed-domain isolation works: the FIRST
  // wl draw must not equal the first draw at any of the slots the
  // orientation stream could have consumed at the same (mixed_seed,
  // global_idx). We sweep slots 0..20 inclusive to cover the historical wl
  // slot (20) plus every slot the near-pole GenericReject loop could have
  // reached under the pre-fix design.
  //
  // Sweeping several (mixed_seed, global_idx) pairs so a single lucky hash
  // collision on one pair cannot mask a real regression.
  uint32_t seeds[] = { 0u, 1u, 0xDEADBEEFu, 0xC0FFEE01u, 0x12345678u };
  uint32_t gidxs[] = { 0u, 42u, 1000u, 65535u, 0x80000000u };
  for (uint32_t seed : seeds) {
    for (uint32_t gidx : gidxs) {
      lm_pcg::PcgStream wl = lm_pcg::BuildWlStream(seed, gidx);
      float wl_draw = lm_pcg::pcg_uniform(wl);
      for (uint32_t slot = 0u; slot <= 20u; ++slot) {
        lm_pcg::PcgStream orient;
        orient.seed = seed;
        orient.global_idx = gidx;
        orient.slot = slot;
        float orient_draw = lm_pcg::pcg_uniform(orient);
        EXPECT_NE(wl_draw, orient_draw) << "wl collides with orientation slot=" << slot << " at seed=" << seed
                                        << " gidx=" << gidx;
      }
    }
  }
}

}  // namespace
