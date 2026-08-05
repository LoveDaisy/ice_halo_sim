// task-342.3 Step 10: gui_test coverage for the Colors window's pure helpers.
//
// These tests exercise the exposed helpers in color_window.hpp directly (no
// ImGui UI drive). Together they nail down:
//   - plan §3 decision 1: z_order and physical vector index stay decoupled —
//     SwapZOrder swaps scalars, never vector entries.
//   - CompactZOrder rebuilds a permutation after delete-class leaves a gap.
//   - plan §3 decision 3: ValidateSingleAtomText rejects multi-factor and
//     multi-alternative text (AND-inside-atom or ; -union).
//   - AC5 decoupling: BuildClassFromFilter clones filter rows into fresh
//     refs by value (no shared filter_id), skipping multi-factor AND rows.
//
// The UI-driven paths (color swatch → LUMICE_SetRaypathColors call, empty-arc
// warning icon rendering, ItemClick-style flows) are left for the AC6 owner
// on-screen manual pass; direct helpers cover the invariants that a UI test
// would only reach indirectly and much more slowly.

#include <chrono>
#include <nlohmann/json.hpp>
#include <thread>

#include "IconsFontAwesome6.h"  // ICON_FA_* selectors for locating icon-prefixed buttons.
#include "gui/app.hpp"          // ShouldTintColorsButton — Step 1 predicate for AC2.
#include "gui/color_window.hpp"
#include "gui/file_io.hpp"  // BuildScene — pipeline assertion for AC4 default flow-through.
#include "gui/gui_state.hpp"
#include "gui/raypath_segments.hpp"
#include "support/scoped_result_frame.hpp"
#include "test_gui_shared.hpp"

namespace {

// Minimal single-class raypath_color config so a real server has exactly one
// active color class to commit against. Mirrors kColorConfig in
// test_gui_composite_preview.cpp; recreated here (not shared) because that
// fixture lives in a TU-private anon namespace and is not linkable across the
// gui_test target (same rationale as that file's own header comment).
const char* kSingleClassConfig = R"({
  "crystal": [{
    "id": 1, "type": "prism",
    "shape": {"height": 1.5},
    "axis": {"zenith": {"type": "gauss", "mean": 90.0, "std": 10.0},
             "azimuth": {"type": "uniform", "mean": 0.0, "std": 180.0},
             "roll": {"type": "uniform", "mean": 0.0, "std": 180.0}}
  }],
  "filter": [],
  "scene": {
    "light_source": {"type": "sun", "altitude": 20.0, "azimuth": 0.0,
                     "diameter": 0.5, "spectrum": "D65"},
    "ray_num": 200000,
    "max_hits": 8,
    "scattering": [{"prob": 0.0, "entries": [{"crystal": 1, "proportion": 1.0}]}]
  },
  "render": [{
    "id": 1,
    "lens": {"type": "dual_fisheye_equal_area", "fov": 180.0},
    "resolution": [128, 64],
    "view": {"elevation": 0, "azimuth": 0, "roll": 0},
    "visible": "full", "background": [0, 0, 0],
    "opacity": 1.0, "intensity_factor": 1.0
  }],
  "raypath_color": {
    "mode": "dominant",
    "classes": [
      {"color": [1.0, 0.0, 0.0], "match": [{"layer": 0, "crystal": 1}]}
    ]
  }
})";

// JSON -> Scene handle -> commit: the only commit path since v4.12 removed the JSON-string
// entry points. The handle is a local — LUMICE_CommitScene reads it as const and keeps no
// reference — so it is destroyed as soon as the commit returns.
LUMICE_ErrorCode CommitJsonConfig(LUMICE_Server* server, const char* json) {
  LUMICE_Scene* scene = nullptr;
  if (auto err = LUMICE_SceneFromJson(json, &scene); err != LUMICE_OK) {
    return err;
  }
  const auto err = LUMICE_CommitScene(server, scene, /*out_reused=*/nullptr);
  LUMICE_SceneDestroy(scene);
  return err;
}

bool RunToIdleWithData(LUMICE_Server* server, const char* json) {
  if (CommitJsonConfig(server, json) != LUMICE_OK) {
    return false;
  }
  for (int waited = 0; waited < 5000; waited += 10) {
    LUMICE_ServerState st = LUMICE_SERVER_RUNNING;
    LUMICE_QueryServerState(server, &st);
    if (st == LUMICE_SERVER_IDLE) {
      LUMICE_RawXyzResult xyz[1]{};
      lumice::test::ScopedResultFrame frame_xyz(server);
      LUMICE_FrameGetRawXyz(frame_xyz.get(), xyz, 1);
      if (xyz[0].has_valid_data) {
        return true;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

}  // namespace

void RegisterColorWindowTests(ImGuiTestEngine* engine) {
  // The two tests above drive PollColorClassSignal with server=nullptr, which
  // returns before ever calling LUMICE_GetColorClassSignal —
  // they exercise the resize path but never the "server rejects class_count"
  // branch (issue.md ② root cause 2: the 500 ms throttled poll hits the server's
  // strict class_count check during the settling window between a GUI-side
  // push_back and the server picking up the committed class table). This test
  // drives that branch for real: commit a 1-class config to a real server, then
  // grow g_state.raypath_color to 2 WITHOUT committing, so PollColorClassSignal's
  // `n` (2) mismatches the server's active class count (1) and
  // LUMICE_GetColorClassSignal returns LUMICE_ERR_INVALID_CONFIG.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "color_window", "poll_signal_preserves_on_real_server_class_count_mismatch");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetTestState();
      LUMICE_Server* server = LUMICE_CreateServer();
      IM_CHECK(server != nullptr);
      const bool ok = RunToIdleWithData(server, kSingleClassConfig);
      IM_CHECK(ok);
      if (!ok) {
        LUMICE_DestroyServer(server);
        return;
      }

      // Local state grows to 2 classes; the server still only knows about 1
      // (no commit happened), so this mirrors the real add-class settling window.
      gui::ColorClassConfig c;
      gui::g_state.raypath_color.push_back(c);
      gui::g_state.raypath_color.push_back(c);

      // Sentinel values distinguishable from both a real signal (0/1) and the
      // resize-appended default (1): if the mismatch branch is skipped and the
      // real poll result gets moved into out_flags, these would be overwritten.
      std::vector<int> flags = { 7, 7 };
      gui::PollColorClassSignal(gui::g_state, server, flags);

      IM_CHECK_EQ(static_cast<int>(flags.size()), 2);
      IM_CHECK_EQ(flags[0], 7);
      IM_CHECK_EQ(flags[1], 7);

      LUMICE_DestroyServer(server);
    };
  }

  // task-cleanup-hardening S5 (AC2): the RefreshColorClassSignals cache is keyed
  // by (server, committed_epoch). A change in either — backend swap (CPU<->GPU)
  // or any struct commit that bumps the epoch — must invalidate the cache so the
  // next call bypasses the 500 ms throttle and re-polls the fresh domain
  // immediately. Prior to S5, the cache retained the old server's signal for up
  // to one throttle interval after the swap, which read to the user as "the pip
  // still says matched" for half a second on a fresh backend. The two tests below
  // verify (a) invalidation on epoch bump and (b) invalidation on server pointer
  // change; a third verifies steady-state (no domain change → throttle honored)
  // so the invalidation logic doesn't force-poll every frame.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "refresh_signals_invalidates_on_committed_epoch_bump");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetTestState();
      gui::ColorClassConfig c;
      gui::g_state.raypath_color.push_back(c);
      gui::g_state.committed_epoch = 0;

      // First call: primes the cache and updates keys from (nullptr, 0) → (nullptr, 0)
      // (unchanged), forces the initial poll (last_poll_time=-1000 < now).
      gui::RefreshColorClassSignals(gui::g_state, nullptr);
      LUMICE_Server* srv_after1 = nullptr;
      uint64_t epoch_after1 = 0;
      size_t size_after1 = 0;
      float t1 = 0.0f;
      gui::GetColorClassSignalCacheKeysForTest(&srv_after1, &epoch_after1, &size_after1, &t1);
      IM_CHECK_EQ(srv_after1, static_cast<LUMICE_Server*>(nullptr));
      IM_CHECK_EQ(static_cast<int>(epoch_after1), 0);
      IM_CHECK_EQ(static_cast<int>(size_after1), 1);
      IM_CHECK(t1 > -1000.0f);  // poll fired: timer updated

      // Bump the epoch and call again immediately (well within the 500 ms throttle).
      // Without S5 invalidation this call would be a throttle-hit no-op that leaves
      // last_poll_time unchanged; with S5 the cache clears + last_poll_time resets
      // to -1000 forcing an immediate re-poll (last_poll_time updates to `now`).
      gui::g_state.committed_epoch = 5;
      gui::RefreshColorClassSignals(gui::g_state, nullptr);
      LUMICE_Server* srv_after2 = nullptr;
      uint64_t epoch_after2 = 0;
      size_t size_after2 = 0;
      float t2 = 0.0f;
      gui::GetColorClassSignalCacheKeysForTest(&srv_after2, &epoch_after2, &size_after2, &t2);
      IM_CHECK_EQ(static_cast<int>(epoch_after2), 5);  // key updated to new epoch
      IM_CHECK(t2 >= t1);                              // poll fired again (timer advanced or same-frame equal)
    };
  }

  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "refresh_signals_invalidates_on_server_pointer_change");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetTestState();
      gui::ColorClassConfig c;
      gui::g_state.raypath_color.push_back(c);

      // Two real servers so the cache has valid pointers to swap between —
      // avoids fake-pointer UB when RefreshColorClassSignals eventually calls
      // LUMICE_GetColorClassSignal. Neither needs to be committed; the poll
      // return path is not the point — we're inspecting the cache keys.
      LUMICE_Server* srv_a = LUMICE_CreateServer();
      LUMICE_Server* srv_b = LUMICE_CreateServer();
      IM_CHECK(srv_a != nullptr);
      IM_CHECK(srv_b != nullptr);
      IM_CHECK(srv_a != srv_b);

      gui::RefreshColorClassSignals(gui::g_state, srv_a);
      LUMICE_Server* srv_after1 = nullptr;
      uint64_t epoch_after1 = 0;
      gui::GetColorClassSignalCacheKeysForTest(&srv_after1, &epoch_after1, nullptr, nullptr);
      IM_CHECK_EQ(srv_after1, srv_a);

      // Immediate call with a different server pointer must swap the cached key
      // and force a re-poll rather than serve srv_a's stale flags.
      gui::RefreshColorClassSignals(gui::g_state, srv_b);
      LUMICE_Server* srv_after2 = nullptr;
      gui::GetColorClassSignalCacheKeysForTest(&srv_after2, nullptr, nullptr, nullptr);
      IM_CHECK_EQ(srv_after2, srv_b);

      LUMICE_DestroyServer(srv_a);
      LUMICE_DestroyServer(srv_b);
    };
  }

  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "refresh_signals_steady_state_honors_throttle");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetTestState();
      gui::ColorClassConfig c;
      gui::g_state.raypath_color.push_back(c);
      gui::g_state.committed_epoch = 42;

      gui::RefreshColorClassSignals(gui::g_state, nullptr);
      float t1 = 0.0f;
      gui::GetColorClassSignalCacheKeysForTest(nullptr, nullptr, nullptr, &t1);

      // Same (server, epoch) — this must be a throttle-hit no-op; last_poll_time
      // is NOT reset to -1000 (no bypass), so t2 == t1 (or, at most, epsilon
      // above from GetTime drift within the same yield). Anti-regression: guards
      // against a future refactor that would force-poll every call and defeat
      // the 500 ms debounce contract in lumice.h.
      gui::RefreshColorClassSignals(gui::g_state, nullptr);
      float t2 = 0.0f;
      gui::GetColorClassSignalCacheKeysForTest(nullptr, nullptr, nullptr, &t2);
      IM_CHECK_EQ(t2, t1);
    };
  }

  // task-349.2 Step 1 (AC1 white-box lockdown): structural color-class edits on
  // top of a completed simulation must surface as SimState::kModified through
  // the shared dirty → ReconcileSimState pipeline (the same route the main-
  // scene "changed since last run" ⚠ + Revert ride on). Driven by REAL UI
  // clicks (not a hand poke of state.dirty) so the test can catch a future
  // rewrite that quietly bypasses MarkStructHardDirty on any color-window control.
  //
  // Seed pattern (kDone + committed_epoch>0 + run_intent=kLoaded + dirty=false)
  // mirrors p1_edit/ok_no_change_preserves_state so the reconcile base pins to
  // kDone and the subsequent dirty edit flips it to kModified.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "add_class_via_ui_marks_modified");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // T1: the reconciler diffs against last_committed_state; without a baseline it is a no-op
      // before the first commit (mirrors real Run-then-edit flow). Seed the baseline BEFORE the
      // edit so the subsequent Add Class becomes a diff — under the OLD widget-side
      // MarkStructHardDirty call this seeding was unnecessary; T1 makes the baseline a first-class
      // input. (toggle_whole_via_ui_marks_modified below already followed this pattern.)
      gui::g_state.last_committed_state = gui::GuiState::ConfigSnapshot::From(gui::g_state);
      gui::g_state.run_intent = gui::RunIntent::kLoaded;
      gui::g_state.sim_state = gui::GuiState::SimState::kDone;
      gui::g_state.committed_epoch = 5;
      gui::g_state.display_epoch_floor = 0;
      gui::g_state.dirty = false;
      gui::g_state.color_window_open = true;
      ctx->Yield(4);

      // Click the "+ Add Class" button in the Colors window. `**/` wildcard
      // walks any window path, so we don't need to hardcode the window ID.
      ctx->ItemClick("**/" ICON_FA_PLUS " Add Class");
      ctx->Yield(2);

      IM_CHECK_EQ(static_cast<int>(gui::g_state.raypath_color.size()), 1);
      IM_CHECK(gui::g_state.dirty);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(gui::GuiState::SimState::kModified));
      // The top-bar Revert affordance is the user-visible surface of kModified;
      // presence here proves the ReconcileSimState → RenderTopBar wire is live
      // for a color-driven dirty (not just a main-scene edit).
      IM_CHECK(ctx->ItemExists("##TopBar/Revert"));

      // Close the Colors window so it does not overlay the next test's clicks.
      gui::g_state.color_window_open = false;
      ctx->Yield(2);
    };
  }

  // task-color-window-controls-polish (A5): the "Remove All" button clears
  // raypath_color in one click and rides the same T1 structural hard-reset
  // channel as single-row delete (vector cardinality change → reconciler ->
  // hard-reset lane). This test mirrors add_class_via_ui_marks_modified's
  // seed shape so we can compare the two channels symmetrically: N add clicks
  // then one Remove All must land raypath_color at 0 and sim_state at
  // kModified (structural change against the committed baseline).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "remove_all_clears_raypath_color");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Same reconcile baseline seed as add_class_via_ui_marks_modified: pin
      // to kDone so structural edits below flip sim_state to kModified.
      gui::g_state.last_committed_state = gui::GuiState::ConfigSnapshot::From(gui::g_state);
      gui::g_state.run_intent = gui::RunIntent::kLoaded;
      gui::g_state.sim_state = gui::GuiState::SimState::kDone;
      gui::g_state.committed_epoch = 5;
      gui::g_state.display_epoch_floor = 0;
      gui::g_state.dirty = false;
      gui::g_state.color_window_open = true;
      ctx->Yield(4);

      // Add three classes so the "Remove All" click has non-trivial work.
      ctx->ItemClick("**/" ICON_FA_PLUS " Add Class");
      ctx->Yield(2);
      ctx->ItemClick("**/" ICON_FA_PLUS " Add Class");
      ctx->Yield(2);
      ctx->ItemClick("**/" ICON_FA_PLUS " Add Class");
      ctx->Yield(2);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.raypath_color.size()), 3);

      // Click Remove All. Vector cardinality drops to 0; the reconciler routes
      // this as a structural change (same lane as three individual erases),
      // so sim_state stays at kModified against the empty committed baseline.
      ctx->ItemClick("**/" ICON_FA_TRASH " Remove All");
      ctx->Yield(2);

      IM_CHECK_EQ(static_cast<int>(gui::g_state.raypath_color.size()), 0);
      // The Add Class path (add_class_via_ui_marks_modified above) already
      // proved sim_state → kModified when raypath_color grows; the mirror
      // guarantee for shrinkage is that Remove All is routed through the same
      // structural channel (not a display-only mutation that would be
      // silently swallowed). sim_state must NOT be kDone/kIdle here — if the
      // reconciler misrouted the clear onto need_display_push, the 3 Add
      // Class kModified flag would silently clear and the UI would claim
      // "nothing to commit" while the vector shrank from 3 to 0.
      IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(gui::GuiState::SimState::kModified));

      // The button still renders (BeginDisabled greys it out but ItemInfo can
      // locate it) — this proves the "+ Add Class" row keeps the Remove All
      // affordance visible even when nothing is left to remove.
      IM_CHECK(ctx->ItemExists("**/" ICON_FA_TRASH " Remove All"));

      gui::g_state.color_window_open = false;
      ctx->Yield(2);
    };
  }

  // Whole-crystal checkbox is the #2 case from issue.md: a display-affecting
  // structural edit whose current implementation is `SetRefMatchAll(ref, ...);
  // state.MarkStructHardDirty();` — must surface as kModified for the same reason
  // as Add Class.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "toggle_whole_via_ui_marks_modified");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Seed one color class with one ref (match_all=false) so the "whole"
      // checkbox is unchecked and toggling it flips to true. Do this BEFORE
      // pinning kDone so the seeding does not itself dirty the reconcile input.
      gui::ColorClassConfig cls;
      cls.color[0] = 1.0f;
      cls.visible = true;
      gui::ColorClassRefConfig ref;
      ref.layer_idx = 0;
      ref.crystal_pool_id = gui::g_state.layers[0].entries[0].crystal_id;
      ref.match_all = false;
      cls.match.push_back(ref);
      gui::g_state.raypath_color.push_back(cls);
      // Baseline snapshot for the config-vs-snapshot dirty check inside
      // ReconcileSimState (dirty is set explicitly below anyway; snapshot just
      // ensures we start from a clean reconciled kDone before the UI edit).
      gui::g_state.last_committed_state = gui::GuiState::ConfigSnapshot::From(gui::g_state);

      gui::g_state.run_intent = gui::RunIntent::kLoaded;
      gui::g_state.sim_state = gui::GuiState::SimState::kDone;
      gui::g_state.committed_epoch = 5;
      gui::g_state.display_epoch_floor = 0;
      gui::g_state.dirty = false;
      gui::g_state.color_window_open = true;
      ctx->Yield(4);

      // Expand every tree node in the Colors window so RenderRefRow (and its
      // "whole" checkbox) becomes part of the frame's item table. ItemOpenAll
      // walks the window's item tree; wildcard-locating `##body` under the
      // class's PushID(phys=int) is fragile because PushID(int) hashes an
      // integer, not the string "0".
      ctx->ItemOpenAll("//" ICON_FA_PALETTE " Colors");
      ctx->Yield(2);

      ctx->SetRef("//" ICON_FA_PALETTE " Colors");
      ctx->ItemClick("**/whole");
      ctx->Yield(2);
      ctx->SetRef("");

      IM_CHECK(gui::g_state.raypath_color[0].match[0].match_all);
      IM_CHECK(gui::g_state.dirty);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(gui::GuiState::SimState::kModified));
      IM_CHECK(ctx->ItemExists("##TopBar/Revert"));

      // Close the Colors window so the next test's frame does not still show
      // an overlay on top of top-bar controls it needs to click.
      gui::g_state.color_window_open = false;
      ctx->Yield(2);
    };
  }

  // task-356.3 AC2 — clicking any of P/B/D on a per-ref row must land on the
  // structural-dirty path (same route as toggling whole / editing predicate
  // text). The three clicks are asserted independently in a single test so a
  // regression in any one operator== field surfaces here.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "toggle_pbd_symmetry_via_ui_marks_modified");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Seed one color class with one non-match-all ref so P/B/D are editable
      // (not BeginDisabled-frozen).
      gui::ColorClassConfig cls;
      cls.color[0] = 1.0f;
      cls.visible = true;
      gui::ColorClassRefConfig ref;
      ref.layer_idx = 0;
      ref.crystal_pool_id = gui::g_state.layers[0].entries[0].crystal_id;
      ref.match_all = false;
      ref.predicate_text = "3-5";  // valid single atom so IM_CHECK on state.dirty later
                                   // reflects OUR symmetry clicks, not a re-edit noise.
      cls.match.push_back(ref);
      gui::g_state.raypath_color.push_back(cls);
      gui::g_state.last_committed_state = gui::GuiState::ConfigSnapshot::From(gui::g_state);

      gui::g_state.run_intent = gui::RunIntent::kLoaded;
      gui::g_state.sim_state = gui::GuiState::SimState::kDone;
      gui::g_state.committed_epoch = 5;
      gui::g_state.display_epoch_floor = 0;
      gui::g_state.dirty = false;
      gui::g_state.color_window_open = true;
      ctx->Yield(4);

      ctx->ItemOpenAll("//" ICON_FA_PALETTE " Colors");
      ctx->Yield(2);
      ctx->SetRef("//" ICON_FA_PALETTE " Colors");

      // Click P. All three symmetry fields are wired identically at the widget
      // layer, so a single click already exercises the reconciler-dirty path.
      // The subsequent B/D clicks are the plan-review Minor #3 belt-and-braces
      // coverage: each bit must independently participate in operator== (else
      // the reconciler diff would not observe the second/third click as a
      // change AT ALL).
      ctx->ItemClick("**/P##color_ref");
      ctx->Yield(2);
      IM_CHECK(gui::g_state.raypath_color[0].match[0].sym_p);
      IM_CHECK(gui::g_state.dirty);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(gui::GuiState::SimState::kModified));

      ctx->ItemClick("**/B##color_ref");
      ctx->Yield(2);
      IM_CHECK(gui::g_state.raypath_color[0].match[0].sym_b);
      IM_CHECK(gui::g_state.dirty);

      ctx->ItemClick("**/D##color_ref");
      ctx->Yield(2);
      IM_CHECK(gui::g_state.raypath_color[0].match[0].sym_d);
      IM_CHECK(gui::g_state.dirty);

      // Symmetry is per-field independent — P and B remain set after clicking D.
      IM_CHECK(gui::g_state.raypath_color[0].match[0].sym_p);
      IM_CHECK(gui::g_state.raypath_color[0].match[0].sym_b);

      // Reset ref before probing an absolute-path item, mirroring
      // toggle_whole_via_ui_marks_modified.
      ctx->SetRef("");
      IM_CHECK(ctx->ItemExists("##TopBar/Revert"));

      gui::g_state.color_window_open = false;
      ctx->Yield(2);
    };
  }

  // task-color-default-pbd (A4) AC1 + AC4 — clicking "+ Add Ref" on an existing
  // class seeds the new ref with sym_p/sym_b/sym_d = true (owner-preferred PBD
  // default), and this default propagates through BuildScene into the committed
  // scene's per-ref symmetry as the "PBD" string (bits 1|2|4 == 7).
  // AC3's dual (deserialization NOT re-labeled to PBD) lives in
  // test_gui_import_export.cpp — a struct-default assertion, orthogonal to
  // this call-site test.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "add_ref_defaults_to_pbd_symmetry");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Seed one empty class so "+ Add Ref" has a target class to append into.
      // Do it BEFORE the reconcile baseline snapshot so the seeding itself is
      // part of the "committed" state and the subsequent Add Ref click is the
      // only diff observed by the reconciler.
      gui::ColorClassConfig cls;
      cls.color[0] = 1.0f;
      cls.visible = true;
      gui::g_state.raypath_color.push_back(cls);
      gui::g_state.last_committed_state = gui::GuiState::ConfigSnapshot::From(gui::g_state);

      gui::g_state.run_intent = gui::RunIntent::kLoaded;
      gui::g_state.sim_state = gui::GuiState::SimState::kDone;
      gui::g_state.committed_epoch = 5;
      gui::g_state.display_epoch_floor = 0;
      gui::g_state.dirty = false;
      gui::g_state.color_window_open = true;
      ctx->Yield(4);

      // Expand tree nodes so the "+ Add Ref" SmallButton (nested under the
      // per-class TreeNode "##body") is part of the frame's item table. Same
      // pattern as toggle_whole_via_ui_marks_modified and
      // toggle_pbd_symmetry_via_ui_marks_modified above.
      ctx->ItemOpenAll("//" ICON_FA_PALETTE " Colors");
      ctx->Yield(2);

      ctx->SetRef("//" ICON_FA_PALETTE " Colors");
      ctx->ItemClick("**/" ICON_FA_PLUS " Add Ref");
      ctx->Yield(2);
      ctx->SetRef("");

      // AC1: the new ref must default to P|B|D all true (call-site owner —
      // struct default in ColorClassRefConfig stays false).
      IM_CHECK_EQ(static_cast<int>(gui::g_state.raypath_color[0].match.size()), 1);
      IM_CHECK(gui::g_state.raypath_color[0].match[0].sym_p);
      IM_CHECK(gui::g_state.raypath_color[0].match[0].sym_b);
      IM_CHECK(gui::g_state.raypath_color[0].match[0].sym_d);

      // AC4: the default flows through BuildScene into the committed scene's
      // per-ref symmetry (P|B|D, emitted as "PBD"). Proves the new default
      // reaches the per-ref symmetry pipeline without any renderer-side changes.
      gui::ScenePtr scene = gui::BuildScene(gui::g_state, gui::SceneIntent::kSimCommit);
      IM_CHECK(scene != nullptr);
      size_t json_len = 0;
      IM_CHECK_EQ(LUMICE_SceneToJson(scene.get(), nullptr, 0, &json_len), LUMICE_OK);
      std::string scene_buf(json_len + 1, '\0');
      IM_CHECK_EQ(LUMICE_SceneToJson(scene.get(), scene_buf.data(), scene_buf.size(), nullptr), LUMICE_OK);
      scene_buf.resize(json_len);
      const auto scene_j = nlohmann::json::parse(scene_buf);
      IM_CHECK_STR_EQ(scene_j["raypath_color"]["classes"][0]["match"][0]["symmetry"].get<std::string>().c_str(), "PBD");

      gui::g_state.color_window_open = false;
      ctx->Yield(2);
    };
  }

  // task-349.2 Step 2 (§3.4 Revert-completeness fix): after +Add Class flips
  // sim_state to kModified, clicking the top-bar Revert button must restore
  // raypath_color to its pre-edit size AND settle sim_state back to a
  // non-Modified state. Both are required: pre-fix, ConfigSnapshot silently
  // omitted raypath_color, so Revert cleared `dirty` (settling sim_state) but
  // left the added class behind — a check on sim_state alone would have
  // reported "green" while the color edit stealthily remained.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "add_class_then_revert_restores_color_state");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Seed a single, non-default color class as the "last-committed" baseline
      // (with fingerprint fields the assertion can distinguish from a default-
      // constructed junk class if the round-trip drops content silently).
      gui::ColorClassConfig baseline;
      baseline.color[0] = 0.25f;
      baseline.color[1] = 0.5f;
      baseline.color[2] = 0.75f;
      baseline.combine = 1;
      baseline.visible = false;
      gui::g_state.raypath_color.push_back(baseline);
      // Snapshot the baseline INTO last_committed_state, mirroring what
      // BuildScene + LUMICE_CommitScene do after a successful commit.
      gui::g_state.last_committed_state = gui::GuiState::ConfigSnapshot::From(gui::g_state);

      // Pin reconcile base to kDone so a subsequent dirty edit surfaces as
      // kModified. Same seed shape as the two tests above.
      gui::g_state.run_intent = gui::RunIntent::kLoaded;
      gui::g_state.sim_state = gui::GuiState::SimState::kDone;
      gui::g_state.committed_epoch = 5;
      gui::g_state.display_epoch_floor = 0;
      gui::g_state.dirty = false;
      gui::g_state.color_window_open = true;
      ctx->Yield(4);

      // Structural edit: +Add Class → raypath_color grows to 2, sim_state → kModified.
      ctx->ItemClick("**/" ICON_FA_PLUS " Add Class");
      ctx->Yield(2);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.raypath_color.size()), 2);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(gui::GuiState::SimState::kModified));

      // Click Revert. Post-fix: raypath_color shrinks back to 1 (the baseline
      // is restored from last_committed_state), sim_state settles back to a
      // non-Modified state (kDone under the current reconcile). Pre-fix, only
      // the sim_state assertion would pass and the size assertion would fail —
      // that is the entire point of this regression guard.
      ctx->ItemClick("##TopBar/Revert");
      ctx->Yield(2);

      IM_CHECK_EQ(static_cast<int>(gui::g_state.raypath_color.size()), 1);
      IM_CHECK_NE(static_cast<int>(gui::g_state.sim_state), static_cast<int>(gui::GuiState::SimState::kModified));
      // Content-level check: the baseline's fingerprint survived — proves
      // ApplyTo copied the raypath_color vector by value rather than default-
      // initializing an empty replacement (belt + suspenders with the
      // unit-correctness test on ConfigSnapshot itself).
      IM_CHECK_EQ(gui::g_state.raypath_color[0].combine, 1);
      IM_CHECK_EQ(gui::g_state.raypath_color[0].visible, false);
      IM_CHECK_EQ(gui::g_state.raypath_color[0].color[1], 0.5f);

      gui::g_state.color_window_open = false;
      ctx->Yield(2);
    };
  }

  // task-349.2 Step 3 (#6, AC2 machine gate): when there are configured color
  // classes with matches, the top-bar Colored button + Colors-window "Enable
  // colors" checkbox must NOT be BeginDisabled(). This is the "wiring is
  // present" check — 7 pure unit tests above already cover the predicate
  // `NoVisibleMatchedColorClass`; here we drive the real UI with a
  // configured class + non-empty signal and confirm the two controls stay
  // enabled. The complementary "predicate=true → disabled" state depends on
  // driving the internal 500 ms signal cache to zeros, which requires a real
  // 0-match server config (plan §7 risk 4 called this out as potentially
  // flaky); we cover it via the pure predicate unit tests + owner on-screen
  // AC4 pass, not a fragile end-to-end here.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "color_window", "enable_controls_stay_enabled_when_predicate_reports_matches");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Seed one configured class (non-empty match) so the top-bar Colored
      // button (which is gated on !raypath_color.empty()) actually renders.
      gui::ColorClassConfig cls;
      cls.color[0] = 1.0f;
      gui::ColorClassRefConfig ref;
      ref.layer_idx = 0;
      ref.crystal_pool_id = 0;
      ref.match_all = true;
      cls.match.push_back(ref);
      gui::g_state.raypath_color.push_back(cls);
      gui::g_state.color_window_open = true;
      ctx->Yield(4);

      // With server=nullptr, RefreshColorClassSignals resizes the cache to
      // (n, 1) — all classes report "matched / unknown", predicate returns
      // false, controls stay enabled. After 349.3 (#4) the top-bar toggle is a
      // plain-text Checkbox again, so the widget id is label-dependent
      // ("Colored" vs "Full Spectrum"). ResetTestState() / DoNew() leave
      // last_uploaded_as_composite at its default `false`, so the initial label
      // is "Full Spectrum".
      auto top_info = ctx->ItemInfo("##TopBar/Full Spectrum##CompositePreviewToggle");
      IM_CHECK((top_info.ItemFlags & ImGuiItemFlags_Disabled) == 0);

      ctx->SetRef("//" ICON_FA_PALETTE " Colors");
      auto win_info = ctx->ItemInfo("**/Enable colors");
      IM_CHECK((win_info.ItemFlags & ImGuiItemFlags_Disabled) == 0);
      ctx->SetRef("");

      gui::g_state.color_window_open = false;
      ctx->Yield(2);
    };
  }

  // task-fix-color-window-visibility-consistency AC1: with multiple configured+
  // matched classes, hiding every matched class (visible=false, no solo) must
  // grey out BOTH the top-bar Colored toggle AND the Colors-window Enable
  // checkbox. Pre-fix, only "all classes have signal=0" would disable them;
  // the "matched but all hidden" branch (composite genuinely empty) left the
  // controls enabled but non-responsive because last_uploaded_as_composite
  // could not flip on. This is the merged-predicate wiring gate — the pure
  // unit tests above cover the semantics; this drives the real UI to confirm
  // both mirror sites (app_panels.cpp + color_window.cpp) route through it.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window",
                                    "enable_controls_disabled_when_matched_classes_hidden_via_visible_false");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Two configured classes, both hidden by the user (visible=false, no
      // solo). RefreshColorClassSignals with server=nullptr resizes cache to
      // (n, 1) = "matched" for both. NoVisibleMatchedColorClass then sees
      // matched+hidden across the pool → true → BeginDisabled fires on both.
      gui::ColorClassRefConfig ref;
      ref.layer_idx = 0;
      ref.crystal_pool_id = 0;
      ref.match_all = true;
      for (int i = 0; i < 2; i++) {
        gui::ColorClassConfig cls;
        cls.match.push_back(ref);
        cls.visible = false;
        cls.solo = false;
        gui::g_state.raypath_color.push_back(cls);
      }
      gui::g_state.color_window_open = true;
      ctx->Yield(4);

      auto top_info = ctx->ItemInfo("##TopBar/Full Spectrum##CompositePreviewToggle");
      IM_CHECK((top_info.ItemFlags & ImGuiItemFlags_Disabled) != 0);

      ctx->SetRef("//" ICON_FA_PALETTE " Colors");
      auto win_info = ctx->ItemInfo("**/Enable colors");
      IM_CHECK((win_info.ItemFlags & ImGuiItemFlags_Disabled) != 0);
      ctx->SetRef("");

      // Flipping ONE class back to visible releases the disable — the composite
      // is once again non-empty. This is the direct AC1 recovery path.
      gui::g_state.raypath_color[0].visible = true;
      ctx->Yield(4);

      top_info = ctx->ItemInfo("##TopBar/Full Spectrum##CompositePreviewToggle");
      IM_CHECK((top_info.ItemFlags & ImGuiItemFlags_Disabled) == 0);
      ctx->SetRef("//" ICON_FA_PALETTE " Colors");
      win_info = ctx->ItemInfo("**/Enable colors");
      IM_CHECK((win_info.ItemFlags & ImGuiItemFlags_Disabled) == 0);
      ctx->SetRef("");

      gui::g_state.color_window_open = false;
      ctx->Yield(2);
    };
  }

  // task-fix-color-window-visibility-consistency AC2: after Alt+click solos a
  // class, peer classes' eye icons must render ICON_FA_EYE_SLASH (not
  // ICON_FA_EYE) — the storage layer keeps peer.visible=true (HandleEyeClick
  // storage semantics are unchanged), but the compositor hides them via the
  // any_solo branch, and the UI now reads through EffectiveVisible to agree.
  // We drive HandleEyeClick directly (state seed) so this test is decoupled
  // from the eye button's widget path (the button label itself IS what we're
  // testing here), then inspect the class rows' widget IDs. Both eye buttons
  // in the Colors window live under PushID(phys) — literal $$<int> per the
  // integer-PushID selector convention (learnings/gui-test-integer-pushid).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "eye_icon_shows_slash_on_peers_when_another_class_solo");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      for (int i = 0; i < 2; i++) {
        gui::ColorClassConfig cls;
        cls.visible = true;
        cls.solo = false;
        gui::g_state.raypath_color.push_back(cls);
      }
      // Alt+click class 1 → solo it. Class 0 keeps visible=true / solo=false —
      // its eye must now show EYE_SLASH via EffectiveVisible(any_solo=true).
      gui::HandleEyeClick(gui::g_state.raypath_color, /*phys=*/1, /*alt_down=*/true);
      IM_CHECK(gui::g_state.raypath_color[0].visible);
      IM_CHECK(!gui::g_state.raypath_color[0].solo);
      IM_CHECK(gui::g_state.raypath_color[1].solo);

      gui::g_state.color_window_open = true;
      ctx->Yield(4);

      ctx->SetRef("//" ICON_FA_PALETTE " Colors");
      // Class 0 (peer): eye must be EYE_SLASH; the plain EYE variant must NOT
      // exist under $$0. Class 1 (solo'd): eye must be EYE; the SLASH variant
      // must NOT exist under $$1. Two-sided assertion pins the widget label —
      // a regression that ignored any_solo would show EYE on $$0 (pre-fix).
      IM_CHECK(ctx->ItemExists("**/$$0/" ICON_FA_EYE_SLASH));
      IM_CHECK(!ctx->ItemExists("**/$$0/" ICON_FA_EYE));
      IM_CHECK(ctx->ItemExists("**/$$1/" ICON_FA_EYE));
      IM_CHECK(!ctx->ItemExists("**/$$1/" ICON_FA_EYE_SLASH));
      ctx->SetRef("");

      // Clear the solo (second alt+click on the solo'd class) — icons must
      // revert: both back to EYE since visible is still true on both.
      gui::HandleEyeClick(gui::g_state.raypath_color, /*phys=*/1, /*alt_down=*/true);
      IM_CHECK(!gui::g_state.raypath_color[0].solo);
      IM_CHECK(!gui::g_state.raypath_color[1].solo);
      ctx->Yield(4);

      ctx->SetRef("//" ICON_FA_PALETTE " Colors");
      IM_CHECK(ctx->ItemExists("**/$$0/" ICON_FA_EYE));
      IM_CHECK(!ctx->ItemExists("**/$$0/" ICON_FA_EYE_SLASH));
      IM_CHECK(ctx->ItemExists("**/$$1/" ICON_FA_EYE));
      IM_CHECK(!ctx->ItemExists("**/$$1/" ICON_FA_EYE_SLASH));
      ctx->SetRef("");

      gui::g_state.color_window_open = false;
      ctx->Yield(2);
    };
  }

  // task-gui-feedback-affordances Step 1 (AC2): end-to-end wire — after
  // clicking + Add Class the raypath_color vector grows, the predicate flips
  // true, and the Colors button remains rendered/clickable in the tinted
  // state. This exercises the real ImGui frame path (top-bar renders the
  // button, PushStyleColor stack balances, layout intact), which is the
  // "GUI test 实际验证" leg of AC5.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "topbar_colors_button_renders_in_both_states");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      // Empty state: predicate false, button exists.
      IM_CHECK(gui::g_state.raypath_color.empty());
      IM_CHECK(!gui::ShouldTintColorsButton(gui::g_state.raypath_color.empty()));
      IM_CHECK(ctx->ItemExists("##TopBar/" ICON_FA_PALETTE " Colors"));

      // Populate raypath_color via the Colors window (real UI path).
      gui::g_state.last_committed_state = gui::GuiState::ConfigSnapshot::From(gui::g_state);
      gui::g_state.run_intent = gui::RunIntent::kLoaded;
      gui::g_state.sim_state = gui::GuiState::SimState::kDone;
      gui::g_state.committed_epoch = 5;
      gui::g_state.display_epoch_floor = 0;
      gui::g_state.dirty = false;
      gui::g_state.color_window_open = true;
      ctx->Yield(4);
      ctx->ItemClick("**/" ICON_FA_PLUS " Add Class");
      ctx->Yield(2);

      // Non-empty state: predicate true, button still exists and remains
      // clickable (PushStyleColor stack balanced ⇒ no ImGui assertion).
      IM_CHECK(!gui::g_state.raypath_color.empty());
      IM_CHECK(gui::ShouldTintColorsButton(gui::g_state.raypath_color.empty()));
      IM_CHECK(ctx->ItemExists("##TopBar/" ICON_FA_PALETTE " Colors"));

      gui::g_state.color_window_open = false;
      ctx->Yield(2);
    };
  }
}
