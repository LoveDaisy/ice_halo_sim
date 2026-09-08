// The "GPU acceleration stopped partway through this run" modal fires once
// per fallback, not once per frame.
//
// Why an edge and not a level: SetGuiWarning dedups on message IDENTITY only while the
// message is still in flight. Once the user dismisses the modal the in-flight message is
// cleared, so a caller that re-asserts the same condition every frame reopens the modal
// the frame after it was closed — a dialog the user cannot get rid of while the run
// continues. The sibling color-degrade poll avoids this by keying its dedup on the
// committed epoch; backend fallback has no such key (it survives CommitConfig within one
// run and clears only when the next Start() re-resolves the backend), which is why it
// carries a plain latch instead.
//
// No window, no GL context, no server — the gate is a pure function of the flag and the
// latch, which is the whole reason it was pulled out of SyncFromPoller.

#include <gtest/gtest.h>

#include "gui/app.hpp"

namespace {

using lumice::gui::BackendFallbackWarningEdge;

TEST(BackendFallbackWarningEdge, QuietWhileNoFallbackHasHappened) {
  bool latch = false;
  EXPECT_FALSE(BackendFallbackWarningEdge(false, latch));
  EXPECT_FALSE(latch) << "latch armed without a fallback; the first real one would be swallowed";
}

TEST(BackendFallbackWarningEdge, FiresOnceOnTheRisingEdge) {
  bool latch = false;
  EXPECT_TRUE(BackendFallbackWarningEdge(true, latch));
  EXPECT_TRUE(latch);
}

TEST(BackendFallbackWarningEdge, StaysQuietWhileTheSameFallbackPersists) {
  bool latch = false;
  ASSERT_TRUE(BackendFallbackWarningEdge(true, latch));
  // The flag stays true for the rest of the run, and SyncFromPoller polls every frame.
  for (int frame = 0; frame < 100; frame++) {
    EXPECT_FALSE(BackendFallbackWarningEdge(true, latch)) << "re-opened the modal on frame " << frame;
  }
}

TEST(BackendFallbackWarningEdge, ReArmsSoTheNextRunCanWarnAgain) {
  bool latch = false;
  ASSERT_TRUE(BackendFallbackWarningEdge(true, latch));
  EXPECT_FALSE(BackendFallbackWarningEdge(false, latch)) << "the falling edge is not itself a warning";
  EXPECT_FALSE(latch) << "latch stayed armed, so a second fallback would go unreported";
  EXPECT_TRUE(BackendFallbackWarningEdge(true, latch)) << "a fallback in the next run must warn again";
}

}  // namespace
