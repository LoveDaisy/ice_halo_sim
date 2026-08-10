// Composition chain: what a document switch and a revert leave behind.
//
// Units in the chain: app × gui_state × server_poller.
//
// What the collaboration produces that is observable: after opening another document, nothing on
// screen still describes the previous one. The status bar's ray / crystal / sampling-density
// readout describes the run of the document in front of you, and its display gate is simply "is the
// ray count non-zero" — a leftover value from the previous document satisfies that gate perfectly.
// No poll follows a document switch to correct it either: the poller's stats gate filters INCOMING
// updates and never clears what is already displayed. So the numbers just sit there, describing a
// run of a file the user has closed, and they look exactly like real numbers.
//
// Revert is the deliberate carve-out and is pinned here beside the others rather than omitted for
// symmetry: it restores configuration into the SAME document and does not end the run whose numbers
// are on screen, so those numbers still describe what is displayed. Without an assertion saying so,
// a later "let's clear stats on every reset reason" change would blank a still-accurate status bar
// and nothing would object.
//
// Derived from the src call graph: app.cpp -> server_poller is 10 call sites, and the document
// entry points are where most of them cluster.

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <system_error>

#include "gui/app.hpp"
#include "gui/file_io.hpp"
#include "gui/gui_state.hpp"

namespace lumice::gui {
namespace {

// Non-zero on all four counters, so a partial reset cannot pass by clearing only the gate field.
void SeedStatusBarStats() {
  g_state.stats_ray_seg_num = 4321;
  g_state.stats_sim_ray_num = 200000;
  g_state.stats_crystal_num = 77;
  g_state.stats_orientation_num = 88;
}

void ExpectStatusBarCleared(const char* which) {
  SCOPED_TRACE(which);
  EXPECT_EQ(g_state.stats_ray_seg_num, 0u);
  EXPECT_EQ(g_state.stats_sim_ray_num, 0u);
  EXPECT_EQ(g_state.stats_crystal_num, 0u);
  EXPECT_EQ(g_state.stats_orientation_num, 0u);
}

// Both document-entry points, then the carve-out.
//
// This drives the real DoNew / DoOpen rather than calling the reset helper directly, because what
// is being pinned is the end-to-end property, not any one implementation of it. Today the property
// holds structurally — every switch path replaces the whole GuiState before the reset runs, and a
// fresh GuiState zeroes all four counters — so an explicit reset inside the helper would be
// unreachable. A future change that stops replacing the whole state on open is exactly what this
// case is here to catch.
TEST(DocumentSwitchChain, OpeningAnotherDocumentLeavesNoStatusBarNumbersBehind) {
  SeedStatusBarStats();
  DoNew();
  ExpectStatusBarCleared("DoNew");

  // The .lmc path is the one worth spending a file on: unlike DoNew and the .json import, it hands
  // the LIVE state to the loader rather than building a separate one, so it is where a leftover
  // field would actually survive if the deserializer stopped clearing.
  const std::filesystem::path path = std::filesystem::temp_directory_path() / "lumice_document_switch_chain.lmc";
  struct Cleanup {
    std::filesystem::path path;
    ~Cleanup() {
      std::error_code ec;
      std::filesystem::remove(path, ec);  // best-effort: a teardown failure must not fail the case
    }
  } cleanup{ path };
  ASSERT_TRUE(SaveLmcFile(path, g_state, g_preview, /*save_texture=*/false));
  SeedStatusBarStats();
  DoOpen(path);
  ExpectStatusBarCleared("DoOpen(.lmc)");

  // Revert: same document, run not ended, numbers still true.
  g_state.last_committed_state = GuiState::ConfigSnapshot::From(g_state);
  SeedStatusBarStats();
  DoRevert();
  EXPECT_EQ(g_state.stats_ray_seg_num, 4321u) << "Revert blanked a status bar that was still accurate";
  EXPECT_EQ(g_state.stats_sim_ray_num, 200000u);
  EXPECT_EQ(g_state.stats_crystal_num, 77u);
  EXPECT_EQ(g_state.stats_orientation_num, 88u);
}

// Revert's field scope must equal the scope of the "did the configuration change" predicate, which
// is this project's declared single source of truth for that question. The two directions below are
// pinned as a pair because either one alone is satisfiable by a wrong fix.
//
//   (A) A view field the predicate deliberately EXCLUDES — camera elevation, a pure client-side
//       reprojection — must SURVIVE a Revert. Changing it never counts as a change, so the Revert
//       button never lights up for it; discarding it anyway is Revert undoing something the UI
//       never claimed was pending. The crystal assertion beside it is what stops "just stop
//       restoring the renderer at all" from passing.
//   (B) A field the predicate DOES judge — the simulation resolution, which changes the render grid
//       — must still be restored. Without this, narrowing the Revert baseline too far reads as a
//       pass.
TEST(DocumentSwitchChain, RevertRestoresExactlyWhatCountsAsAChange) {
  DoNew();
  if (g_state.crystals.empty()) {
    // Do not depend on the default document happening to contain a crystal: a later change to the
    // seeded contents would turn this case's premise into a no-op rather than fail it.
    g_state.crystals.emplace_back();
  }
  g_state.last_committed_state = GuiState::ConfigSnapshot::From(g_state);
  const float committed_height = g_state.crystals[0].height.center;
  const int committed_resolution = g_state.renderer.sim_resolution_index;

  ASSERT_FLOAT_EQ(g_state.renderer.elevation, 0.0f) << "premise: the drag below is a change from the committed value";
  g_state.renderer.elevation = 30.0f;                                // (A) never counted as a change
  g_state.renderer.sim_resolution_index = committed_resolution + 1;  // (B) counted
  g_state.crystals[0].height.center = committed_height + 1.0f;       // (B) counted

  DoRevert();

  EXPECT_FLOAT_EQ(g_state.renderer.elevation, 30.0f) << "Revert discarded a view change it never offered to undo";
  EXPECT_EQ(g_state.renderer.sim_resolution_index, committed_resolution);
  EXPECT_FLOAT_EQ(g_state.crystals[0].height.center, committed_height);
}

// Revert also has to make the next reconcile re-push the restored display payload, or the restored
// configuration sits in the state while the screen keeps showing what was there before. The
// mechanism is the display baseline being cleared; a run through the server is not needed to prove
// it, only that the clearing still happens on this path.
TEST(DocumentSwitchChain, RevertClearsTheDisplayBaselineSoTheRestoredStateIsPushed) {
  DoNew();
  g_state.last_committed_state = GuiState::ConfigSnapshot::From(g_state);
  g_state.last_pushed_display_state = GuiState::DisplayStateBaseline{};
  ASSERT_TRUE(g_state.last_pushed_display_state.has_value()) << "premise: there is a baseline to clear";

  DoRevert();

  EXPECT_FALSE(g_state.last_pushed_display_state.has_value())
      << "the restored display state would never be pushed, so the screen keeps the pre-Revert picture";
}

// The reset itself has one owner, and this reads the source to prove it.
//
// A behavioural test can only show that resetting works when it is reached; it cannot show that no
// OTHER place also resets, which is the failure being guarded — the primitives were once scattered
// across command handlers, so each new command had to remember them and one eventually would not.
// Counting occurrences in the tracked source is the only form of that statement available, and it
// fails at build-and-test time rather than waiting for an on-screen regression.
TEST(DocumentSwitchChain, TheResetPrimitivesHaveExactlyOneCallSite) {
  std::ifstream in(LUMICE_GUI_APP_CPP_PATH);
  ASSERT_TRUE(in.is_open()) << "cannot read " << LUMICE_GUI_APP_CPP_PATH;
  const std::string src((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  ASSERT_FALSE(src.empty());

  const auto occurrences = [&src](const std::string& needle) {
    int count = 0;
    for (size_t pos = src.find(needle); pos != std::string::npos; pos = src.find(needle, pos + needle.size())) {
      ++count;
    }
    return count;
  };

  // The needles are the owner's actual code shape; a zero here means the primitive was renamed and
  // this case has quietly stopped checking anything, which is why zero fails as loudly as two.
  EXPECT_EQ(occurrences("g_server_poller.InvalidateStagedTexture()"), 1);
  EXPECT_EQ(occurrences("g_crystal_mesh_hash = 0"), 1);
}

}  // namespace
}  // namespace lumice::gui
