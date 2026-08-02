// The interaction cases that never drive a widget.
//
// test_gui_interaction.cpp is the repo's one genuinely UI-driven test file — 169 of its cases
// click, type and drag through ImGuiTestContext, and stay there. These five only call production
// functions and assert on GuiState / the warning channel:
//   Interaction.mark_dirty — MarkDirty sets the flag, DoNew clears it
//   the four p2_filter_type overflow cases — a filter whose predicate set exceeds
//     ComponentTable::kMaxBits must surface a warning, and a persistent overflow re-detected on
//     every auto-commit tick must NOT respawn the modal (dedup)
//
// Note on what did NOT move: the whole p2_linked group (12 cases) stays in gui_test by owner
// decision even though 6 of them would pass the criterion — the group tests one shared-pool
// semantics whose seam is on the UI side, and splitting it would leave that seam spanning two
// targets with no owner. p0_file/new and p0_file/save_open_roundtrip look pure at a glance but
// both read gui::g_preview (a GL texture), so they stay too.
//
// Each case starts from gui::DoNew() rather than gui_test's ResetTestState(): a fresh document is
// the only thing they took from that helper, and DoNew() is what it delegates to for exactly that.

#include <gtest/gtest.h>

#include <string>

#include "gui/app.hpp"
#include "gui/gui_state.hpp"
#include "gui/panels.hpp"
#include "lumice.h"

namespace gui = lumice::gui;

// P2: MarkDirty
TEST(Interaction, mark_dirty) {
  gui::DoNew();

  EXPECT_EQ(gui::g_state.dirty, false);

  // Mark dirty
  gui::g_state.MarkDirty();
  EXPECT_EQ(gui::g_state.dirty, true);

  // DoNew resets dirty
  gui::DoNew();
  EXPECT_EQ(gui::g_state.dirty, false);
}

// AC1: the end-to-end degrade-warning
// wire. Big-OR filter (host-side ABI-legal) + a color config with > 64
// distinct predicates on one placement — the ABI check passes (commit is
// NOT rejected), the CORE drops the excess predicates (kNoBit), and DoRun
// surfaces the "coloring degraded" modal via SetGuiWarning with a message
// string DIFFERENT from the ABI-overflow message (identity-dedup safety).
TEST(Interaction, big_or_filter_with_color_overflow_surfaces_warning) {
  gui::DoNew();
  gui::ClearGuiWarning();
  gui::g_server = LUMICE_CreateServer();
  EXPECT_TRUE(gui::g_server != nullptr);

  // Populate raypath_color across 3 classes × 22 refs = 66 unique
  // raypath predicates on the (layer 0, crystal 1) placement. Each ref
  // uses a distinct 2-face raypath text ("f1-f2") so structural dedup
  // does not collapse them across classes; face numbers stay in the
  // valid prism range 1..8 (kMaxHits is 64, well above our lengths).
  // ABI caps allow 32 refs/class and 64 classes; splitting across
  // classes is the only way to get > 64 predicates through the ABI to
  // the CORE, where BuildColorGateTable dedupes across classes and hits
  // ComponentTable::kMaxBits=64 → 66-64 = 2 predicates dropped.
  gui::g_state.raypath_color.clear();
  constexpr int kNumClasses = 3;
  constexpr int kRefsPerClass = 22;
  static_assert(kNumClasses * kRefsPerClass > 64, "must exceed ComponentTable::kMaxBits");
  int uid = 0;  // index into a 64-combo (f1,f2) grid; overflow refs (>=64) use 3-face raypaths
  for (int c = 0; c < kNumClasses; ++c) {
    gui::ColorClassConfig cls;
    cls.color[0] = 1.0f - c * 0.2f;
    cls.color[1] = 0.5f;
    cls.color[2] = 0.0f + c * 0.2f;
    cls.combine = 0;
    cls.visible = true;
    cls.solo = false;
    for (int k = 0; k < kRefsPerClass; ++k, ++uid) {
      gui::ColorClassRefConfig ref;
      ref.layer_idx = 0;
      ref.crystal_pool_id = 0;  // maps to CrystalConfig::id_ = 1 in a new document
      ref.match_all = false;
      if (uid < 64) {
        const int f1 = 1 + (uid % 8);
        const int f2 = 1 + (uid / 8);
        ref.predicate_text = std::to_string(f1) + "-" + std::to_string(f2);
      } else {
        // Two extra 3-face raypaths past the 64-combo grid — structurally
        // distinct from all length-2 predicates above so total unique
        // predicates = 66 → 2 overflow past kMaxBits.
        const int tail = uid - 63;  // 1, 2
        ref.predicate_text = "1-1-" + std::to_string(tail);
      }
      cls.match.push_back(ref);
    }
    gui::g_state.raypath_color.push_back(cls);
  }

  // Sim ray count small so the run finishes quickly if it starts.
  gui::g_state.sim.infinite = false;
  gui::g_state.sim.ray_num_millions = 0.001f;

  gui::DoRun(/*user_initiated=*/true);

  // Commit MUST succeed (ABI accepts the config); the drop is a
  // display-layer degradation only.
  const std::string warning = gui::PeekGuiWarning();
  EXPECT_TRUE(!warning.empty());
  EXPECT_TRUE(warning.find("color") != std::string::npos || warning.find("Color") != std::string::npos);
  // The message MUST be distinct from the two existing ABI-overflow msgs
  // (filter cap / color-class cap), else SetGuiWarning's identity-dedup
  // would silently collapse them (regression anchor per plan §7 risk 3).
  EXPECT_TRUE(warning.find("This raypath color configuration exceeds its predicate") != std::string::npos);
  EXPECT_TRUE(warning.find("Simplify the color configuration") != std::string::npos);

  // Precise count lock: 66 unique predicates - kMaxBits(64) = 2
  // dropped. Ties the end-to-end GUI test to the same exact number the Step 5/7 unit tests
  // assert, rather than only the message substring.
  LUMICE_ColorOverflowInfo color_over{};
  EXPECT_TRUE(LUMICE_GetColorOverflowInfo(gui::g_server, &color_over) == LUMICE_OK);
  EXPECT_TRUE(color_over.component_overflow_count == 2);

  // Cleanup.
  gui::ClearGuiWarning();
  gui::g_state.raypath_color.clear();
  gui::g_server_poller.Stop();
  LUMICE_StopServer(gui::g_server);
  LUMICE_DestroyServer(gui::g_server);
  gui::g_server = nullptr;
  gui::g_state.run_intent = gui::RunIntent::kNone;
  gui::g_state.committed_epoch = 0;
  gui::g_state.dirty = false;
}

// Regression anchor: a persistent
// color-overflow condition (unlike the BuildScene-REJECT branch covered by the sibling
// `overflow_auto_commit_dedup` test) goes through the commit-SUCCEEDED branch of DoRun. That
// branch used to call ClearGuiWarning() unconditionally before checking for a color overflow,
// which zeroed the identity-dedup state ahead of the comparison and made every auto-commit
// tick (user_initiated=false) reopen the modal even though the SAME overflow persisted —
// reproducing the "70ms slider drag freezes the UI" regression Step 2 fixed for the ABI-reject
// branch. This test drives two auto-commit ticks against the same 66-predicate overflow setup
// as `big_or_filter_with_color_overflow_surfaces_warning` and asserts the second tick does NOT
// respawn the modal.
TEST(Interaction, color_overflow_auto_commit_dedup) {
  gui::DoNew();
  gui::ClearGuiWarning();
  gui::g_server = LUMICE_CreateServer();
  EXPECT_TRUE(gui::g_server != nullptr);

  gui::g_state.raypath_color.clear();
  constexpr int kNumClasses = 3;
  constexpr int kRefsPerClass = 22;
  static_assert(kNumClasses * kRefsPerClass > 64, "must exceed ComponentTable::kMaxBits");
  int uid = 0;
  for (int c = 0; c < kNumClasses; ++c) {
    gui::ColorClassConfig cls;
    cls.color[0] = 1.0f - c * 0.2f;
    cls.color[1] = 0.5f;
    cls.color[2] = 0.0f + c * 0.2f;
    cls.combine = 0;
    cls.visible = true;
    cls.solo = false;
    for (int k = 0; k < kRefsPerClass; ++k, ++uid) {
      gui::ColorClassRefConfig ref;
      ref.layer_idx = 0;
      ref.crystal_pool_id = 0;
      ref.match_all = false;
      if (uid < 64) {
        const int f1 = 1 + (uid % 8);
        const int f2 = 1 + (uid / 8);
        ref.predicate_text = std::to_string(f1) + "-" + std::to_string(f2);
      } else {
        const int tail = uid - 63;
        ref.predicate_text = "1-1-" + std::to_string(tail);
      }
      cls.match.push_back(ref);
    }
    gui::g_state.raypath_color.push_back(cls);
  }

  gui::g_state.sim.infinite = false;
  gui::g_state.sim.ray_num_millions = 0.001f;

  // First auto-commit tick: overflow persists → commit succeeds, color-degrade warning set.
  gui::DoRun(/*user_initiated=*/false);
  EXPECT_TRUE(!gui::PeekGuiWarning().empty());
  EXPECT_TRUE(gui::IsGuiWarningPending());
  const std::string first_msg = gui::PeekGuiWarning();

  // Frame consumes OpenPopup; trigger cleared, message retained.
  gui::internal_test::ConsumeGuiWarningPending();
  EXPECT_TRUE(!gui::IsGuiWarningPending());

  // Second auto-commit tick with the SAME overflow: dedup MUST hold (no modal respawn).
  gui::DoRun(/*user_initiated=*/false);
  EXPECT_STREQ(gui::PeekGuiWarning().c_str(), first_msg.c_str());
  EXPECT_TRUE(!gui::IsGuiWarningPending());

  // Cleanup.
  gui::ClearGuiWarning();
  gui::g_state.raypath_color.clear();
  gui::g_server_poller.Stop();
  LUMICE_StopServer(gui::g_server);
  LUMICE_DestroyServer(gui::g_server);
  gui::g_server = nullptr;
  gui::g_state.run_intent = gui::RunIntent::kNone;
  gui::g_state.committed_epoch = 0;
  gui::g_state.dirty = false;
}

// AC3: a user-clicked Run always
// re-opens the warning modal when the overflow condition persists, even
// after the user dismissed the previous popup with OK. DoRun(true) calls
// ClearGuiWarning() before SetGuiWarning() so the identity-dedup does NOT
// swallow the second Run. Uses the same 4-factor × 9-alt raypath (6561
// would-be clauses > LUMICE_MAX_CONFIG_CLAUSES=4096) as the export_json
// overflow tests so the overflow trigger stays a single source of truth.
//
// We assert via IsGuiWarningPending() — the internal "OpenPopup pending"
// flag — rather than driving a full frame + clicking "OK", because:
//   1) The dedup semantics live entirely in SetGuiWarning/ClearGuiWarning
//      and their observable is precisely that flag transition.
//   2) Running frames while a modal is open makes the harness fight the
//      popup's input capture, obscuring the invariant this test is here
//      to lock.
TEST(Interaction, overflow_user_initiated_run_refires) {
  gui::DoNew();
  gui::ClearGuiWarning();
  gui::g_server = LUMICE_CreateServer();
  EXPECT_TRUE(gui::g_server != nullptr);

  // Same over-cap recipe as import_export/export_json_rejects_overflow_filter:
  // 4 raypath factors × 9 alternatives = 6561 clauses > 4096.
  gui::SummandText row;
  row.text = "1;2;3;4;5;6;7;8;3-4 & 1;2;3;4;5;6;7;8;3-4 & 1;2;3;4;5;6;7;8;3-4 & 1;2;3;4;5;6;7;8;3-4";
  row.factors = {
    gui::Factor{ gui::RaypathParams{ "1;2;3;4;5;6;7;8;3-4" } },
    gui::Factor{ gui::RaypathParams{ "1;2;3;4;5;6;7;8;3-4" } },
    gui::Factor{ gui::RaypathParams{ "1;2;3;4;5;6;7;8;3-4" } },
    gui::Factor{ gui::RaypathParams{ "1;2;3;4;5;6;7;8;3-4" } },
  };
  gui::FilterConfig f;
  f.name = "OverflowFilter";
  f.param = gui::SumOfProducts{ row };
  gui::g_state.filters.push_back(f);
  gui::SetFilter(gui::g_state, gui::g_state.layers[0].entries[0], gui::g_state.filters.back());

  // First user Run: overflow → warning set + trigger set.
  gui::DoRun(/*user_initiated=*/true);
  EXPECT_TRUE(!gui::PeekGuiWarning().empty());
  EXPECT_TRUE(gui::IsGuiWarningPending());
  const std::string first_msg = gui::PeekGuiWarning();

  // Simulate the frame that consumes OpenPopup (RenderGuiWarningPopup
  // clears the trigger after calling OpenPopup) without touching the
  // in-flight message — matches what happens after a real frame renders.
  gui::internal_test::ConsumeGuiWarningPending();
  EXPECT_TRUE(!gui::PeekGuiWarning().empty());
  EXPECT_TRUE(!gui::IsGuiWarningPending());

  // Second user Run with the same overflow: MUST re-open the modal.
  gui::DoRun(/*user_initiated=*/true);
  EXPECT_STREQ(gui::PeekGuiWarning().c_str(), first_msg.c_str());
  EXPECT_TRUE(gui::IsGuiWarningPending());

  // Cleanup.
  gui::ClearGuiWarning();
  LUMICE_StopServer(gui::g_server);
  LUMICE_DestroyServer(gui::g_server);
  gui::g_server = nullptr;
}

// AC3: main-loop auto-commit
// (DoRun(user_initiated=false)) MUST preserve dedup so a persistent
// overflow condition re-detected every 70ms tick does not respawn the
// modal (which would freeze user interaction the moment a slider drag
// pushes them into overflow). Same overflow setup as the sibling test.
TEST(Interaction, overflow_auto_commit_dedup) {
  gui::DoNew();
  gui::ClearGuiWarning();
  gui::g_server = LUMICE_CreateServer();
  EXPECT_TRUE(gui::g_server != nullptr);

  gui::SummandText row;
  row.text = "1;2;3;4;5;6;7;8;3-4 & 1;2;3;4;5;6;7;8;3-4 & 1;2;3;4;5;6;7;8;3-4 & 1;2;3;4;5;6;7;8;3-4";
  row.factors = {
    gui::Factor{ gui::RaypathParams{ "1;2;3;4;5;6;7;8;3-4" } },
    gui::Factor{ gui::RaypathParams{ "1;2;3;4;5;6;7;8;3-4" } },
    gui::Factor{ gui::RaypathParams{ "1;2;3;4;5;6;7;8;3-4" } },
    gui::Factor{ gui::RaypathParams{ "1;2;3;4;5;6;7;8;3-4" } },
  };
  gui::FilterConfig f;
  f.name = "OverflowFilter";
  f.param = gui::SumOfProducts{ row };
  gui::g_state.filters.push_back(f);
  gui::SetFilter(gui::g_state, gui::g_state.layers[0].entries[0], gui::g_state.filters.back());

  // First auto-commit tick: overflow → warning set + trigger set.
  gui::DoRun(/*user_initiated=*/false);
  EXPECT_TRUE(!gui::PeekGuiWarning().empty());
  EXPECT_TRUE(gui::IsGuiWarningPending());

  // Frame consumes OpenPopup; trigger cleared, message retained.
  gui::internal_test::ConsumeGuiWarningPending();
  EXPECT_TRUE(!gui::IsGuiWarningPending());

  // Second auto-commit tick with the SAME overflow: dedup MUST hold.
  gui::DoRun(/*user_initiated=*/false);
  EXPECT_TRUE(!gui::IsGuiWarningPending());  // no modal respawn

  gui::ClearGuiWarning();
  LUMICE_StopServer(gui::g_server);
  LUMICE_DestroyServer(gui::g_server);
  gui::g_server = nullptr;
}
