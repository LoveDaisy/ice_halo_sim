// Unit test for `lumice::NarrowPcgRayBase` (src/core/backend/trace_backend.hpp):
// the runtime guard that narrows a per-session PCG ray-index base (size_t) to the
// uint32 the device kernels consume as `global_idx = base + tid`.
//
// AC: scrum-cuda-backend-complete 296.6 task-cuda-single-engine Step B — replaces
// the per-backend no-op-in-Release asserts. Past 2^32 rays in one session the
// uint32 global_idx wraps and two rays collide on the same PCG stream → silent
// under-sampling (scrum-267.3 failure mode). The guard must THROW (not silently
// truncate) before `base + (dispatch_count-1)` would wrap. This validates the
// boundary without running 4.29e9 rays — we set the counter at the threshold
// directly, the approach the plan called for. The full uint64 fix is backlogged
// ("device-gen ray index 32-bit truncation").

#include <gtest/gtest.h>

#include <cstdint>

#include "core/backend/trace_backend.hpp"

namespace {

constexpr size_t kU32Max = static_cast<size_t>(UINT32_MAX);

TEST(NarrowPcgRayBase, PassesThroughInRangeBases) {
  EXPECT_EQ(lumice::NarrowPcgRayBase(0u, 1u, "t"), 0u);
  EXPECT_EQ(lumice::NarrowPcgRayBase(12345u, 32768u, "t"), 12345u);
  // Largest base that still leaves room for the whole dispatch: base + count-1
  // must be <= UINT32_MAX, i.e. base <= UINT32_MAX - count.
  size_t count = 32768u;
  size_t max_ok = kU32Max - count;
  EXPECT_EQ(lumice::NarrowPcgRayBase(max_ok, count, "t"), static_cast<uint32_t>(max_ok));
}

TEST(NarrowPcgRayBase, ThrowsWhenBasePlusDispatchWouldWrap) {
  size_t count = 32768u;
  // One past the safe boundary: base + (count-1) would exceed UINT32_MAX.
  size_t over_by_one = kU32Max - count + 1u;
  EXPECT_THROW(lumice::NarrowPcgRayBase(over_by_one, count, "t"), lumice::BackendUnavailableError);
  // A base already past UINT32_MAX (e.g. a 5-billion-ray session) must throw.
  EXPECT_THROW(lumice::NarrowPcgRayBase(static_cast<size_t>(5'000'000'000ull), 1u, "t"),
               lumice::BackendUnavailableError);
}

TEST(NarrowPcgRayBase, BoundaryIsDispatchAware) {
  // The guard accounts for dispatch_count: the same base passes with a small
  // dispatch but throws with a larger one that would push the last tid past 2^32.
  size_t base = kU32Max - 10u;
  EXPECT_EQ(lumice::NarrowPcgRayBase(base, 10u, "t"), static_cast<uint32_t>(base));  // base+9 == UINT32_MAX-1, ok
  EXPECT_THROW(lumice::NarrowPcgRayBase(base, 100u, "t"), lumice::BackendUnavailableError);
}

}  // namespace
