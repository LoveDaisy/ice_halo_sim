// Regression sentinel for the RenderConsumer::Consume empty-batch contract.
//
// Background: the assertion in RenderConsumer::Consume once read
//   assert(!data.outgoing_w_.empty() || data.rays_.Empty());
// which encodes "rays_ non-empty => outgoing_w_ non-empty". That property is
// NOT something Design A promises: the simulator-side filter is an emit-gate, so
// an entire Consume() batch having every candidate ray rejected (outgoing_*
// empty while rays_ is non-empty) is a legitimate tail event, not a malformed
// state. A low pass-rate filter tripped it in Debug builds (SIGABRT). The
// assertion was rewritten to the real Design A invariant —
// outgoing_d_.size() == 3 * outgoing_w_.size() (the parallel-array sizing
// documented in sim_data.hpp). See doc/filter-architecture.md §4
// "Empty-batch contract".
//
// Coverage:
//   A. AllRaysFilteredOutDoesNotAbort — a non-empty rays_ with fully-empty
//      outgoing_* must Consume() cleanly and leave snapshot_intensity_ at 0.
//      Runs in every build type (does not depend on assert firing); this is the
//      positive regression lock for the fix itself.
//   B. MismatchedOutgoingSizesTripsAssert (Debug-only, #ifndef NDEBUG) — a
//      deliberate outgoing_d_/outgoing_w_ size mismatch must still trip the new
//      assertion, proving it is not a dead assert. Compiled out under NDEBUG
//      where the assert is a no-op, so it never fires as a false "pass" in a
//      Release CI run.

#include <gtest/gtest.h>

#include <vector>

#include "config/color_class_table.hpp"
#include "config/proj_config.hpp"
#include "config/render_config.hpp"
#include "config/sim_data.hpp"
#include "server/render.hpp"

namespace lumice {
namespace {

// Minimal render config: 64x64 equal-area fisheye looking straight up. Kept
// local (not shared with test_render_consumer_component_lanes.cpp's
// file-private MakeLaneRenderConfig) — cross-file reuse would need a refactor
// whose cost outweighs a small copy here.
RenderConfig MakeMinimalRenderConfig() {
  RenderConfig cfg;
  cfg.id_ = 0;
  cfg.lens_.type_ = LensParam::kFisheyeEqualArea;
  cfg.lens_.fov_ = 180.0f;
  cfg.resolution_[0] = 64;
  cfg.resolution_[1] = 64;
  cfg.view_.az_ = 0.0f;
  cfg.view_.el_ = 90.0f;
  cfg.view_.ro_ = 0.0f;
  cfg.visible_ = RenderConfig::kUpper;
  return cfg;
}

// -----------------------------------------------------------------------------
// A. Whole batch filtered out (outgoing_* empty, rays_ non-empty) → no abort.
// -----------------------------------------------------------------------------
TEST(RenderConsumerEmptyBatch, AllRaysFilteredOutDoesNotAbort) {
  // Model "this batch had N candidate rays, all rejected by the filter": rays_
  // carries a non-zero size while outgoing_d_/outgoing_w_ stay default-empty.
  constexpr size_t kN = 128;
  SimData data;
  data.curr_wl_ = 550.0f;
  data.rays_.Reset(kN);
  data.rays_.size_ = kN;
  // outgoing_d_/outgoing_w_/outgoing_component_ left empty on purpose.

  RenderConfig cfg = MakeMinimalRenderConfig();
  RenderConsumer rc(cfg);

  EXPECT_NO_FATAL_FAILURE(rc.Consume(data));
  rc.PrepareSnapshot();
  EXPECT_FLOAT_EQ(rc.GetRawXyzResult().snapshot_intensity_, 0.0f);
}

// -----------------------------------------------------------------------------
// B. The rewritten assertion is still live (Debug-only).
// -----------------------------------------------------------------------------
#ifndef NDEBUG
TEST(RenderConsumerEmptyBatchDeathTest, MismatchedOutgoingSizesTripsAssert) {
  RenderConfig cfg = MakeMinimalRenderConfig();
  RenderConsumer rc(cfg);

  SimData data;
  data.curr_wl_ = 550.0f;
  data.outgoing_w_ = { 0.5f, 0.7f };         // 2 weights
  data.outgoing_d_ = { 0.0f, 0.0f, -1.0f };  // only 1 ray's worth of direction

  EXPECT_DEATH(rc.Consume(data), "");
}
#else
TEST(RenderConsumerEmptyBatchDeathTest, MismatchedOutgoingSizesTripsAssert) {
  GTEST_SKIP() << "assert() is a no-op under NDEBUG (Release); the death test is "
                  "meaningful only in a Debug build";
}
#endif

}  // namespace
}  // namespace lumice
