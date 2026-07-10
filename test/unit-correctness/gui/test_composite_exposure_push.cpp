#include <gtest/gtest.h>

#include <limits>

#include "gui/composite_exposure_push.hpp"

namespace lumice::gui {
namespace {

constexpr float kEpsilon = 1e-4f;
constexpr float kNan = std::numeric_limits<float>::quiet_NaN();

// task-345.3 code-review round 1 Major #1: regression coverage for the four
// branches of the display-time composite-exposure push guard, extracted from
// app_panels.cpp so it's reachable without a full ImGui frame.

TEST(CompositeExposurePush, OffToOffNeverPushes) {
  EXPECT_FALSE(ShouldPushCompositeExposure(/*composite_active=*/false, /*last_composite_active=*/false,
                                           /*ev_total=*/1.0f, /*last_pushed_ev=*/1.0f, kEpsilon));
  EXPECT_FALSE(ShouldPushCompositeExposure(false, false, 2.0f, kNan, kEpsilon));
}

TEST(CompositeExposurePush, OffToOnAlwaysForcesPushEvenWithSameEv) {
  // plan-review Minor #2: composite just went live; last_pushed_ev happens to
  // equal ev_total (a stale value from a prior composite-off period) — the
  // edge condition alone must still force the push.
  EXPECT_TRUE(ShouldPushCompositeExposure(/*composite_active=*/true, /*last_composite_active=*/false,
                                          /*ev_total=*/1.0f, /*last_pushed_ev=*/1.0f, kEpsilon));
}

TEST(CompositeExposurePush, OnToOnPushesWhenValueChanges) {
  EXPECT_TRUE(ShouldPushCompositeExposure(/*composite_active=*/true, /*last_composite_active=*/true,
                                          /*ev_total=*/1.5f, /*last_pushed_ev=*/1.0f, kEpsilon));
}

TEST(CompositeExposurePush, OnToOnSuppressesWhenValueUnchanged) {
  EXPECT_FALSE(ShouldPushCompositeExposure(/*composite_active=*/true, /*last_composite_active=*/true,
                                           /*ev_total=*/1.0f, /*last_pushed_ev=*/1.0f, kEpsilon));
  // Within epsilon of the last pushed value still counts as unchanged.
  EXPECT_FALSE(ShouldPushCompositeExposure(true, true, 1.0f + kEpsilon * 0.5f, 1.0f, kEpsilon));
}

TEST(CompositeExposurePush, OnToOnPushesOnFirstEverPushRegardlessOfEv) {
  // last_pushed_ev == NaN models "no push has happened yet" — must push even
  // when composite was already active on a prior frame (e.g. edge already
  // consumed) so the very first bake isn't silently skipped.
  EXPECT_TRUE(ShouldPushCompositeExposure(/*composite_active=*/true, /*last_composite_active=*/true,
                                          /*ev_total=*/0.0f, /*last_pushed_ev=*/kNan, kEpsilon));
}

}  // namespace
}  // namespace lumice::gui
