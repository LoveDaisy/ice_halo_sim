// Default-document smoke test — migrated from gui_test's `gui_smoke/default_state`.
//
// The original case read the live `gui::g_state` after the harness had booted the app. There is no
// app here, so it asserts on `gui::MakeNewDocumentState()` instead: that is the one function
// main.cpp / DoNew() / DoOpen() all call to produce a fresh document, so it — not the global — is
// what "the default state" actually means. The global is just wherever the result was last stored.
//
// The override directory is supplied explicitly and freshly emptied, so the assertions describe the
// factory document rather than whatever personal defaults exist on the machine running the test.

#include <gtest/gtest.h>

#include "gui/gui_state.hpp"
#include "gui/user_defaults.hpp"
#include "support/user_defaults_test_env.hpp"

namespace {

using lumice::test_user_defaults::FreshOverlayDir;

}  // namespace

TEST(GuiSmoke, default_state) {
  // Verify default state: 1 layer with 1 entry, embedded renderer at default values
  const gui::GuiState state = gui::MakeNewDocumentState(FreshOverlayDir("smoke_default_state"));
  // ASSERT, not EXPECT: the next line indexes layers[0]. IM_CHECK_EQ aborted the case on failure,
  // so stopping here is the faithful translation, not an added guard.
  ASSERT_EQ(static_cast<int>(state.layers.size()), 1);
  EXPECT_EQ(static_cast<int>(state.layers[0].entries.size()), 1);
  EXPECT_EQ(state.renderer.lens_type, 0);
  EXPECT_EQ(state.dirty, false);
  EXPECT_EQ(state.sim_state, gui::GuiState::SimState::kIdle);
}
