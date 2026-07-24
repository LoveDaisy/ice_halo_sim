#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <thread>

#include "IconsFontAwesome6.h"  // ICON_FA_* selectors used to match new icon-prefixed button labels
#include "gui/log_sink.hpp"     // ImGuiLogSink (for log_panel_above_left_panel test sink injection)
// imgui_internal.h is generally an anti-pattern, but z-order assertions need
// direct ImGuiContext::Windows access. The relied-on semantics
// (BringWindowToDisplayFront splices to g.Windows back; creation with
// NoBringToFrontOnFocus does push_front = bottom) are documented in the
// convention block at the top of src/gui/app_panels.cpp. Any ImGui upgrade
// that alters either rule must update both that comment and the
// p1_layout / p1_edit_modal z-order assertions below.
#include "gui/panels.hpp"         // FilterSummary declaration (also re-exposes gui_state.hpp transitively)
#include "gui/server_poller.hpp"  // LUMICE_CreateServer/StopServer/DestroyServer (real-commit tests)
#include "imgui_internal.h"
#include "test_gui_shared.hpp"  // declares g_enable_log_panel (toggle gate for RenderLogPanel)

// ========== Helpers for interaction tests ==========

// Polls gui::g_state.texture_upload_count until it reaches baseline + 1 (or higher).
// Returns true on success, false on timeout.
static bool WaitForSimRestartAtLeast(ImGuiTestContext* ctx, unsigned long baseline_upload_count,
                                     int timeout_ms = 1500) {
  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    ctx->Yield();  // allow main thread to call SyncFromPoller()
    if (gui::g_state.texture_upload_count >= baseline_upload_count + 1) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

// AC2 core deliverable evidence (task-classic-params-migration, code-review-01.md merged Major
// #1): plan.md §4 Step 6 calls out "same filter edit via CommitAllBuffers (Staged OK) vs
// CommitAllBuffersImmediate (Immediate) produces identical GuiEffects" as the task's core
// observable proof that S6 (staged/immediate unification) is real. Drives ONE logical
// filter-presence edit through either commit path from an identical "finite rays done" baseline
// and returns the effect-observable triple for cross-path comparison. A free function (not a
// capturing lambda) because ImGuiTest::TestFunc is a raw function pointer typedef and cannot bind
// captures.
struct Ac2Outcome {
  bool dirty;
  gui::GuiState::SimState sim_state;
  unsigned long long display_epoch_floor;
};

static Ac2Outcome RunFilterPresenceToggleScenario(ImGuiTestContext* ctx, bool start_with_filter, bool immediate) {
  ResetTestState();
  gui::g_state.modal_immediate_mode = immediate;
  if (start_with_filter) {
    gui::FilterConfig f;
    f.SetRaypath(gui::RaypathParams{ "3-1-5" });
    gui::SetFilter(gui::g_state, gui::g_state.layers[0].entries[0], f);
  }
  gui::g_state.run_intent = gui::RunIntent::kLoaded;
  gui::g_state.sim_state = gui::GuiState::SimState::kDone;
  gui::g_state.snapshot_intensity = 0.5f;
  gui::g_state.committed_epoch = 5;
  gui::g_state.display_epoch_floor = 0;
  gui::g_state.dirty = false;
  gui::g_state.last_committed_state = gui::GuiState::ConfigSnapshot::From(gui::g_state);
  ctx->Yield(2);

  ctx->ItemClick("**/Edit##fi");
  ctx->Yield(4);
  ctx->ItemClick("**/###filter_tab");
  ctx->Yield(4);
  if (start_with_filter) {
    ctx->ItemClick("**/Remove Filter##filter");
  } else {
    ctx->ItemInputValue("**/##row_text_0", "3-1-5");
  }
  ctx->Yield(2);
  if (immediate) {
    ctx->ItemClick("**/Close##edit_modal");
  } else {
    ctx->ItemClick("**/" ICON_FA_CHECK " OK##edit_modal");
  }
  ctx->Yield(2);

  const Ac2Outcome out{ gui::g_state.dirty, gui::g_state.sim_state, gui::g_state.display_epoch_floor };
  if (immediate) {
    gui::g_state.modal_immediate_mode = false;
  }
  return out;
}

// P0 tests
void RegisterP0Tests(ImGuiTestEngine* engine) {
  // P0: New
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p0_file", "new");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      ResetTestState();

      // Modify state: add an extra entry to first layer, set dirty
      gui::EntryCard extra;
      gui::g_state.layers[0].entries.push_back(extra);
      gui::g_state.dirty = true;

      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers[0].entries.size()), 2);
      IM_CHECK_EQ(gui::g_state.dirty, true);

      // DoNew resets
      gui::DoNew();

      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers.size()), 1);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers[0].entries.size()), 1);
      IM_CHECK_EQ(gui::g_state.dirty, false);
      IM_CHECK_EQ(gui::g_preview.HasTexture(), false);
    };
  }

  // P0: Save/Open roundtrip
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p0_file", "save_open_roundtrip");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      ResetTestState();

      // Modify state via layers model
      auto& entry0 = gui::g_state.layers[0].entries[0];
      gui::CrystalOf(gui::g_state, entry0).type = gui::CrystalType::kPyramid;
      gui::CrystalOf(gui::g_state, entry0).prism_h = 2.0f;
      gui::CrystalOf(gui::g_state, entry0).upper_h = 0.3f;
      gui::CrystalOf(gui::g_state, entry0).lower_h = 0.4f;
      gui::g_state.sun.altitude = 30.0f;
      gui::g_state.sim.max_hits = 12;

      // Save
      const char* tmp_path = "/tmp/lumice_gui_test.lmc";
      bool save_ok = gui::SaveLmcFile(tmp_path, gui::g_state, gui::g_preview, false);
      IM_CHECK(save_ok);

      // Reset
      gui::DoNew();
      IM_CHECK_EQ(gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id].type, gui::CrystalType::kPrism);

      // Load
      std::vector<unsigned char> tex_data;
      int tex_w = 0;
      int tex_h = 0;
      bool load_ok = gui::LoadLmcFile(tmp_path, gui::g_state, tex_data, tex_w, tex_h);
      IM_CHECK(load_ok);

      // Verify roundtrip
      auto& loaded_entry = gui::g_state.layers[0].entries[0];
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, loaded_entry).type, gui::CrystalType::kPyramid);
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, loaded_entry).prism_h, 2.0f);
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, loaded_entry).upper_h, 0.3f);
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, loaded_entry).lower_h, 0.4f);
      IM_CHECK_EQ(gui::g_state.sun.altitude, 30.0f);
      IM_CHECK_EQ(gui::g_state.sim.max_hits, 12);
      IM_CHECK(tex_data.empty());  // save_texture=false

      // Cleanup
      std::remove(tmp_path);
    };
  }

  // P0: Run/Stop UI state
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p0_sim", "run_stop_ui");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();

      // sim_state is DERIVED each frame by ReconcileSimState (the harness main loop calls
      // SyncFromPoller every Yield), so these scenes drive the INTENT inputs, not sim_state
      // directly. With no live server observation the reconcile maps intent → sim_state
      // deterministically (kNone→kIdle, kRunning→kSimulating, kStopped→kDone, kStopped+dirty→kModified).

      // Scene 1: kIdle — "Run" button should exist
      gui::g_state.run_intent = gui::RunIntent::kNone;
      gui::g_state.dirty = false;
      ctx->Yield();
      IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(gui::GuiState::SimState::kIdle));
      IM_CHECK(ctx->ItemExists("##TopBar/" ICON_FA_PLAY " Run"));

      // Scene 2: kSimulating — "Stop" button should exist
      gui::g_state.run_intent = gui::RunIntent::kRunning;
      ctx->Yield();
      IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(gui::GuiState::SimState::kSimulating));
      IM_CHECK(ctx->ItemExists("##TopBar/" ICON_FA_STOP " Stop"));

      // Scene 3: kDone
      gui::g_state.run_intent = gui::RunIntent::kStopped;
      ctx->Yield();
      IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(gui::GuiState::SimState::kDone));
      IM_CHECK(ctx->ItemExists("##TopBar/" ICON_FA_PLAY " Run"));  // Back to Run

      // Scene 4: kModified — Revert button should appear (dirty edit on a completed result)
      gui::g_state.dirty = true;
      ctx->Yield();
      IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(gui::GuiState::SimState::kModified));
      IM_CHECK(ctx->ItemExists("##TopBar/Revert"));
      gui::g_state.dirty = false;
    };
  }
}

// P1 tests
void RegisterP1Tests(ImGuiTestEngine* engine) {
  // P1: Entry Add/Delete (card-based UI, copy model)
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_entry", "add_delete");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Verify initial state: 1 layer with 1 entry
      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers.size()), 1);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers[0].entries.size()), 1);

      // Add an entry via the "+ Crystal" button in layer 0
      ctx->ItemClick("**/+ Crystal##layer_0");
      ctx->Yield();
      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers[0].entries.size()), 2);

      // Delete the new entry (×##del_0_1 for layer 0, entry 1). The hover button
      // is always in the ImGui tree (alpha=0 when not hovered) so test engine
      // can click it by ID even if the card is not hovered.
      ctx->ItemClick("**/" ICON_FA_XMARK "##del_0_1");
      ctx->Yield();
      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers[0].entries.size()), 1);
    };
  }

  // P1: Filter via modal (copy model — filter is optional per entry)
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_filter", "modal_set_clear");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Verify initial: entry has no filter
      IM_CHECK(!gui::g_state.layers[0].entries[0].filter_id.has_value());

      // Programmatically set a filter so we can test clearing it
      gui::FilterConfig f;
      f.SetRaypath(gui::RaypathParams{ "3-1-5" });
      gui::SetFilter(gui::g_state, gui::g_state.layers[0].entries[0], f);

      // Verify filter is set
      IM_CHECK(gui::g_state.layers[0].entries[0].filter_id.has_value());

      // Open filter modal, click Remove (buffered) then OK to commit removal.
      ctx->ItemClick("**/Edit##fi");
      ctx->Yield(4);  // popup open + Filter tab activation (SetSelected first-frame)
      ctx->ItemClick("**/Remove Filter##filter");
      ctx->Yield(2);
      ctx->ItemClick("**/" ICON_FA_CHECK " OK##edit_modal");
      ctx->Yield(2);

      // Verify filter is cleared
      IM_CHECK(!gui::g_state.layers[0].entries[0].filter_id.has_value());
    };
  }

  // P1: scrum-gui-polish-v13 161.2 (H5-adapted for 333.4) — Remove Filter arms
  // the intent flag and dirties the tab. Rewritten from the pre-H5 form which
  // asserted "Remove disabled after clear" — under H5 Remove is always
  // enabled (intent flag, not derived from row emptiness).
  //
  // Observables under H5:
  //   - Row 0 InputText remains editable.
  //   - Filter tab dirty mark " *" lights up (rows differ from snapshot).
  //   - Row 0 buffer content cleared (checked by re-typing then clicking OK
  //     — with intent set, the retyped text is ignored and filter_id
  //     collapses to nullopt regardless — direct proof intent overrides
  //     everything, indirect proof clear-then-write is coherent).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_filter", "remove_clears_textbox_staged");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      gui::FilterConfig f;
      f.SetRaypath(gui::RaypathParams{ "3-1-5" });
      gui::SetFilter(gui::g_state, gui::g_state.layers[0].entries[0], f);

      ctx->ItemClick("**/Edit##fi");
      ctx->Yield(4);
      ctx->ItemClick("**/Remove Filter##filter");
      ctx->Yield(2);

      // Row 0 InputText remains editable (no "disabled while pending remove"
      // banner — H5 dropped that concept).
      auto rp_info = ctx->ItemInfo("**/##row_text_0");
      IM_CHECK((rp_info.ItemFlags & ImGuiItemFlags_Disabled) == 0);

      // AC#4: Filter tab label carries a trailing " *" dirty mark.
      auto tab_info = ctx->ItemInfo("**/###filter_tab");
      IM_CHECK(tab_info.ID != 0);
      IM_CHECK(std::strstr(tab_info.DebugLabel, "*") != nullptr);

      // Cancel so we don't leave state polluted for later tests.
      ctx->ItemClick("**/" ICON_FA_XMARK " Cancel##edit_modal");
      ctx->Yield(2);
    };
  }

  // (`remove_preserves_sym_on_retype` removed in 333.4 H5 migration. The
  // pre-H5 raypath contract was "Remove clears buffer; typing re-enables the
  // filter with sym_* preserved". H5 unifies Remove to the pre-H5 EE-style
  // session-level intent flag (plan §3 point 4): Remove → OK writes nullopt
  // regardless of subsequent typing. The "typing re-creates filter" semantic
  // this test encoded no longer exists — use Cancel instead of Remove if the
  // user wants to abort a Remove. sym_* preservation across a "no-op" edit
  // cycle is now covered by `remove_then_cancel_restores_filter` above.)

  // P1: scrum-gui-polish-v13 161.2 — Staged Remove + OK / Cancel path contract
  // test (AC#5 Cancel branch): Remove then Cancel must restore the filter
  // (including raypath & sym_*) from the entry — buffer-discard semantics.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_filter", "remove_then_cancel_restores_filter");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      gui::FilterConfig f;
      f.SetRaypath(gui::RaypathParams{ "3-1-5" });
      f.sym_p = true;
      f.sym_b = false;
      f.sym_d = true;
      gui::SetFilter(gui::g_state, gui::g_state.layers[0].entries[0], f);
      ctx->Yield();

      ctx->ItemClick("**/Edit##fi");
      ctx->Yield(4);
      ctx->ItemClick("**/Remove Filter##filter");
      ctx->Yield(2);
      ctx->ItemClick("**/" ICON_FA_XMARK " Cancel##edit_modal");
      ctx->Yield(4);

      IM_CHECK(gui::g_state.layers[0].entries[0].filter_id.has_value());
      IM_CHECK_STR_EQ(gui::g_state.filters[*gui::g_state.layers[0].entries[0].filter_id].RaypathText().c_str(),
                      "3-1-5");
      IM_CHECK(gui::g_state.filters[*gui::g_state.layers[0].entries[0].filter_id].sym_p == true);
      IM_CHECK(gui::g_state.filters[*gui::g_state.layers[0].entries[0].filter_id].sym_b == false);
      IM_CHECK(gui::g_state.filters[*gui::g_state.layers[0].entries[0].filter_id].sym_d == true);
    };
  }

  // (`remove_then_retype_commits_new_raypath` removed in 333.4 H5 migration.
  // The pre-H5 raypath Remove was a buffer-clear (not an intent flag), so
  // typing after Remove naturally re-created the filter on OK. H5 unifies
  // Remove to the pre-H5 EE-style session-level intent (plan §3 point 4):
  // OK writes nullopt regardless of subsequent typing. The new semantic is
  // covered by `remove_filter_clears_on_ok` in p2_filter_type.)

  // (`remove_button_disabled_when_empty` removed in 333.4 H5 migration: the
  // pre-H5 Remove Filter button was disabled when the raypath backing buffer
  // was empty. Under H5 Remove Filter is a session-level intent flag, always
  // enabled — the concept it tested no longer exists. New Remove semantics
  // are covered by `remove_filter_clears_on_ok` in p2_filter_type.)

  // P1: scrum-gui-polish-v13 161.2 — In Immediate mode, Remove applies on the
  // next frame (ApplyBuffersToEntry sees empty raypath and writes nullopt).
  // Dual observable: entry.filter_id = nullopt (direct) + display_epoch_floor
  // (MarkStructHardDirty side effect — raises the floor to committed_epoch). Seed a
  // non-zero committed_epoch with the floor at 0 so the bump is observable, and the
  // pre-condition asserts the floor starts below committed_epoch to avoid tautologies.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_filter", "immediate_remove_applies_next_frame");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      gui::FilterConfig f;
      f.SetRaypath(gui::RaypathParams{ "3-1-5" });
      gui::SetFilter(gui::g_state, gui::g_state.layers[0].entries[0], f);
      gui::g_state.committed_epoch = 5;
      gui::g_state.display_epoch_floor = 0;
      // task-classic-params-migration: seed last_committed_state after the filter is
      // present so removing it later trips AnyEntryFilterPresenceChanged in the reconciler.
      // Pre-migration this test worked because CommitAllBuffersImmediate manually fired
      // MarkStructHardDirty regardless of baseline; post-migration the effect flows through the
      // reconciler and requires a real baseline to diff against.
      gui::g_state.last_committed_state = gui::GuiState::ConfigSnapshot::From(gui::g_state);
      ctx->Yield();

      // Pre-condition guard: MarkStructHardDirty has not fired yet (floor still below committed).
      IM_CHECK_EQ(gui::g_state.display_epoch_floor, 0u);

      ctx->ItemClick("**/Edit##fi");
      ctx->Yield(4);
      // Enter Immediate mode (close+reopen cycle).
      ctx->ItemClick("**/Immediate##edit_modal");
      ctx->Yield(6);
      ctx->ItemClick("**/###filter_tab");
      ctx->Yield(2);

      ctx->ItemClick("**/Remove Filter##filter");
      ctx->Yield(2);

      IM_CHECK(!gui::g_state.layers[0].entries[0].filter_id.has_value());
      IM_CHECK_EQ(gui::g_state.display_epoch_floor, gui::g_state.committed_epoch);

      // Close modal (Immediate uses Close button, not Cancel).
      ctx->ItemClick("**/Close##edit_modal");
      ctx->Yield(2);
      gui::g_state.modal_immediate_mode = false;
    };
  }

  // P1: scrum-gui-polish-v7 152.6 — Edit modal tab titles carry a trailing
  // " *" marker when the in-flight buffer diverges from its open-time
  // snapshot. Tab IDs are anchored to stable "###<slug>_tab" suffixes so the
  // label change does not reset SelectedTabId; the test reads the truncated
  // display string from ItemInfo().DebugLabel. Covers AC #1 / #2 / #3 / #4 / #5.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_edit_modal", "tab_dirty_mark");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      auto is_dirty = [&](const char* tab_ref) -> bool {
        auto info = ctx->ItemInfo(tab_ref);
        return info.ID != 0 && std::strstr(info.DebugLabel, "*") != nullptr;
      };

      ResetTestState();
      ctx->Yield(2);
      const float orig_h = gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id].height.center;

      // AC #1: on modal open, no tab carries a dirty marker.
      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(4);
      IM_CHECK(!is_dirty("**/###crystal_tab"));
      IM_CHECK(!is_dirty("**/###axis_tab"));
      IM_CHECK(!is_dirty("**/###filter_tab"));

      // AC #2: editing Crystal's Height only dirties the Crystal tab.
      ctx->ItemInputValue("**/##Height##modal_cr_input", orig_h + 1.0f);
      ctx->Yield(2);
      IM_CHECK(is_dirty("**/###crystal_tab"));
      IM_CHECK(!is_dirty("**/###axis_tab"));
      IM_CHECK(!is_dirty("**/###filter_tab"));

      // AC #3: restoring the original value clears the dirty marker.
      ctx->ItemInputValue("**/##Height##modal_cr_input", orig_h);
      ctx->Yield(2);
      IM_CHECK(!is_dirty("**/###crystal_tab"));

      // AC #5: Cancel discards buffer; reopening shows no marker.
      ctx->ItemInputValue("**/##Height##modal_cr_input", orig_h + 2.0f);
      ctx->Yield(2);
      ctx->ItemClick("**/" ICON_FA_XMARK " Cancel##edit_modal");
      ctx->Yield(2);
      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(4);
      IM_CHECK(!is_dirty("**/###crystal_tab"));

      // AC #4: OK commits new baseline; reopening shows no marker.
      ctx->ItemInputValue("**/##Height##modal_cr_input", orig_h + 3.0f);
      ctx->Yield(2);
      ctx->ItemClick("**/" ICON_FA_CHECK " OK##edit_modal");
      ctx->Yield(2);
      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(4);
      IM_CHECK(!is_dirty("**/###crystal_tab"));
      ctx->ItemClick("**/" ICON_FA_XMARK " Cancel##edit_modal");
      ctx->Yield(2);
    };
  }

  // P1: scrum-gui-polish-v12 / task-modal-preview-cross-tab — the persistent
  // crystal preview pane (##modal_left_pane) must be reachable from every tab,
  // and the right TabBar pane (##modal_right_pane) must render with non-zero
  // width on the first frame (harden against the 0.0f right-width fallback
  // discussed in plan §3.2).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_edit_modal", "preview_visible_across_tabs");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Open Edit Entry modal via the Crystal edit shortcut.
      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(4);

      // Left pane anchor: preview InvisibleButton must exist anywhere under
      // the modal (glob walks into BeginChild windows too).
      IM_CHECK(ctx->ItemExists("**/##modal_preview_interact"));
      // Right pane anchor: Crystal tab header must exist — proves the right
      // BeginChild resolved to non-zero width so TabBar rendered.
      IM_CHECK(ctx->ItemExists("**/###crystal_tab"));

      // Switch to Axis tab — left preview should still be present.
      ctx->ItemClick("**/###axis_tab");
      ctx->Yield(2);
      IM_CHECK(ctx->ItemExists("**/##modal_preview_interact"));

      // Switch to Filter tab — same invariant.
      ctx->ItemClick("**/###filter_tab");
      ctx->Yield(2);
      IM_CHECK(ctx->ItemExists("**/##modal_preview_interact"));

      // Close modal (cleanup).
      ctx->ItemClick("**/" ICON_FA_XMARK " Cancel##edit_modal");
      ctx->Yield(2);
    };
  }

  // P1: scrum-gui-polish-v12 / task-modal-preview-cross-tab — trackball
  // rotation applied on the preview must survive tab switches (the preview
  // is persistent, so ImGui must not reinitialize its child state on
  // tab-active change).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_edit_modal", "preview_rotation_persists_across_tabs");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(4);

      // Baseline rotation matrix before any drag.
      float before[16];
      std::memcpy(before, gui::g_crystal_rotation, sizeof before);

      // Drag on the preview to apply a non-zero trackball rotation.
      ctx->ItemDragWithDelta("**/##modal_preview_interact", ImVec2(60.0f, 0.0f));
      ctx->Yield(2);
      IM_CHECK(std::memcmp(before, gui::g_crystal_rotation, sizeof before) != 0);

      // Freeze post-drag state as the invariant baseline for the tab switch.
      float drag_state[16];
      std::memcpy(drag_state, gui::g_crystal_rotation, sizeof drag_state);

      // Switch tabs (Axis then back to Crystal) — rotation must not reset.
      ctx->ItemClick("**/###axis_tab");
      ctx->Yield(2);
      ctx->ItemClick("**/###crystal_tab");
      ctx->Yield(2);
      IM_CHECK(std::memcmp(drag_state, gui::g_crystal_rotation, sizeof drag_state) == 0);

      ctx->ItemClick("**/" ICON_FA_XMARK " Cancel##edit_modal");
      ctx->Yield(2);
    };
  }

  // P1: task-fix-crystal-preview-thumbnail Bug 2 — opening the Edit modal on a
  // different crystal card must snap the preview pose to the target crystal's
  // axis default (equivalent to auto-clicking Reset View). Opening the SAME
  // crystal_id twice in a row must preserve the trackball pose (so a user who
  // dragged, closed, then re-opened the same entry sees the same view). The
  // switch judge is crystal_id, not (layer_idx, entry_idx), so two entries
  // sharing one pool crystal (Link scenario) share trackball history.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_edit_modal", "preview_pose_follows_crystal_switch");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Seed a second crystal in the pool with Column-preset axis params, then
      // add a second entry bound to it. Entry 0 keeps the default (Random)
      // crystal that DoNew() emits.
      const gui::AxisDist az_full{ gui::AxisDistType::kUniform, 0.0f, 360.0f };
      const gui::AxisDist roll_full{ gui::AxisDistType::kUniform, 0.0f, 360.0f };
      const gui::AxisDist zenith_column{ gui::AxisDistType::kGauss, 90.0f, 1.0f };

      gui::CrystalConfig c_column;
      c_column.zenith = zenith_column;
      c_column.azimuth = az_full;
      c_column.roll = roll_full;
      IM_CHECK_EQ(static_cast<int>(gui::ClassifyAxisPreset(c_column.zenith, c_column.azimuth, c_column.roll)),
                  static_cast<int>(gui::AxisPreset::kColumn));

      gui::EntryCard e_column;
      e_column.crystal_id = static_cast<int>(gui::g_state.crystals.size());
      gui::g_state.crystals.push_back(c_column);
      gui::g_state.layers[0].entries.push_back(e_column);
      ctx->Yield(2);

      // Expected default poses for each entry's crystal.
      const auto& cr0 = gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id];
      gui::AxisDist params0[3] = { cr0.zenith, cr0.azimuth, cr0.roll };
      float expected0[16] = { 0 };
      gui::DefaultPreviewRotation(gui::ClassifyAxisPreset(cr0.zenith, cr0.azimuth, cr0.roll), params0, expected0);

      gui::AxisDist params1[3] = { c_column.zenith, c_column.azimuth, c_column.roll };
      float expected1[16] = { 0 };
      gui::DefaultPreviewRotation(gui::AxisPreset::kColumn, params1, expected1);

      // Open entry 0 (Random) via OpenEditModal (bypasses the Edit button so
      // the assertion isolates OpenEditModal's own reset logic rather than the
      // card-router indirection). The preview must snap to entry 0's default.
      gui::EditRequest req0{ gui::EditTarget::kCrystal, /*layer_idx=*/0, /*entry_idx=*/0 };
      gui::OpenEditModal(req0, gui::g_state);
      ctx->Yield(2);
      for (int i = 0; i < 16; ++i) {
        if (gui::g_crystal_rotation[i] != expected0[i]) {
          IM_ERRORF("entry0 (Random) idx=%d actual=%f expected=%f", i, static_cast<double>(gui::g_crystal_rotation[i]),
                    static_cast<double>(expected0[i]));
        }
      }

      // Simulate user trackball drag on entry 0's pose so we can prove entry 1
      // opens with a DIFFERENT (target-specific) rotation, not the leftover.
      gui::ApplyTrackballRotation(60.0f, 0.0f);
      IM_CHECK(std::memcmp(gui::g_crystal_rotation, expected0, sizeof(expected0)) != 0);

      // Direct card-to-card switch WITHOUT Cancel/OK — this is the exact user
      // path the bug reproduces. Opening entry 1 must overwrite g_crystal_rotation
      // with the Column default, discarding the dragged pose.
      gui::EditRequest req1{ gui::EditTarget::kCrystal, /*layer_idx=*/0, /*entry_idx=*/1 };
      gui::OpenEditModal(req1, gui::g_state);
      ctx->Yield(2);
      for (int i = 0; i < 16; ++i) {
        if (gui::g_crystal_rotation[i] != expected1[i]) {
          IM_ERRORF("entry1 (Column) idx=%d actual=%f expected=%f", i, static_cast<double>(gui::g_crystal_rotation[i]),
                    static_cast<double>(expected1[i]));
        }
      }

      // Re-open the same entry 1 after dragging: crystal_id matches the loaded
      // one, so the trackball must survive (idempotent re-open). This is the
      // invariant that makes the crystal_id judge distinct from a
      // "always reset" fix — and protects the "same-entry Cancel then reopen"
      // UX and the linked-entries share-history semantics.
      gui::ApplyTrackballRotation(30.0f, 20.0f);
      float after_drag[16];
      std::memcpy(after_drag, gui::g_crystal_rotation, sizeof(after_drag));
      IM_CHECK(std::memcmp(after_drag, expected1, sizeof(expected1)) != 0);
      gui::OpenEditModal(req1, gui::g_state);
      ctx->Yield(2);
      IM_CHECK(std::memcmp(gui::g_crystal_rotation, after_drag, sizeof(after_drag)) == 0);

      ctx->ItemClick("**/" ICON_FA_XMARK " Cancel##edit_modal");
      ctx->Yield(2);
    };
  }

  // P1: task-fix-modal-edit-state-leak (AC1, Filter tab) — in Immediate mode,
  // switching to another entry while a filter InputText is mid-edit (cursor
  // never left the box) must NOT carry the uncommitted text into the new entry.
  // Root cause: crystal-card clicks are a raw hit-test (never a real ImGui
  // widget), so the mid-edit InputText is never deactivated the normal way;
  // SetRowsFromSop resets the row uid to 0 so the next entry's first row reuses
  // the same InputText ID ("##row_text_0"), and ImGui replays the previous
  // entry's text into it via g.InputTextDeactivatedState (deferred deactivation-
  // writeback). Fix = per-(layer,entry) PushID scope around each tab's content
  // in RenderModalTabBar, so the new entry's widget gets a DIFFERENT ID and none
  // of ImGui's per-ID caches match. (ClearActiveID() does NOT fix this — it is
  // what populates InputTextDeactivatedState.) Reproduced pre-fix (progress.md
  // bisect); guards the fix from regressing.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_edit_modal", "filter_edit_not_leaked_on_entry_switch");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      gui::g_state.modal_immediate_mode = true;  // bug is Immediate-mode only

      // Add a second entry bound to a DISTINCT crystal. This is load-bearing:
      // entries that SHARE a crystal_id are an intentional "linked group" that
      // shares filters atomically (ApplyBuffersToEntry::propagate_filter_id_to_
      // linked), so a shared crystal would propagate entry 0's filter to entry 1
      // by design and mask the actual bug. The reported bug is two DISTINCT
      // crystals, where no such linking applies.
      gui::CrystalConfig c_second;
      c_second.height = 5.0f;
      gui::EntryCard e_second;
      e_second.crystal_id = static_cast<int>(gui::g_state.crystals.size());
      gui::g_state.crystals.push_back(c_second);
      gui::g_state.layers[0].entries.push_back(e_second);
      ctx->Yield(2);
      IM_CHECK(!gui::g_state.layers[0].entries[0].filter_id.has_value());
      IM_CHECK(!gui::g_state.layers[0].entries[1].filter_id.has_value());

      // Open entry 0's Filter tab and type into the first summand row WITHOUT
      // committing (KeyCharsAppend does not send Enter, so the box stays active).
      gui::EditRequest req0{ gui::EditTarget::kFilter, /*layer_idx=*/0, /*entry_idx=*/0 };
      gui::OpenEditModal(req0, gui::g_state);
      ctx->Yield(4);
      ctx->ItemClick("**/##row_text_0");
      ctx->KeyCharsAppend("3-5");
      ctx->Yield(2);

      // L1 precondition A: Immediate mode commits every frame, so entry 0 now
      // actually carries the "3-5" filter (proves the input landed).
      IM_CHECK(gui::g_state.layers[0].entries[0].filter_id.has_value());
      IM_CHECK_STR_EQ(gui::g_state.filters[*gui::g_state.layers[0].entries[0].filter_id].RaypathText().c_str(), "3-5");
      // L1 precondition B: the InputText is still the active item (cursor未离开).
      IM_CHECK(ImGui::GetActiveID() != 0);

      // Switch to entry 1's Filter tab without leaving the box — the exact user
      // path (clicking another crystal routes through OpenEditModal).
      gui::EditRequest req1{ gui::EditTarget::kFilter, /*layer_idx=*/0, /*entry_idx=*/1 };
      gui::OpenEditModal(req1, gui::g_state);
      ctx->Yield(4);

      // Core AC1: entry 1 must stay filter-less — "3-5" must not have bled in.
      IM_CHECK(!gui::g_state.layers[0].entries[1].filter_id.has_value());

      ctx->ItemClick("**/Close##edit_modal");  // Immediate mode: single Close button
      ctx->Yield(2);
    };
  }

  // P1: task-fix-modal-edit-state-leak (AC2, Crystal tab) — the same leak is a
  // mechanism-level defect (not filter-specific): Crystal/Axis tab inputs use
  // fixed, entry-agnostic widget IDs (e.g. "##Height##modal_cr_input"), so an
  // uncommitted Height edit also carries across an entry switch via the same
  // g.InputTextDeactivatedState replay. The per-(layer,entry) PushID scope is a
  // single mechanism-level fix that covers all three tabs. This test proves
  // AC2's "统一性" (unified coverage beyond the Filter tab).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_edit_modal", "crystal_edit_not_leaked_on_entry_switch");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      gui::g_state.modal_immediate_mode = true;

      // Seed a second crystal with a distinct height so "unchanged" is a strong
      // assertion, and bind a second entry to it.
      gui::CrystalConfig c_second;
      c_second.height = 5.0f;  // distinct from entry 0's default (1.0)
      gui::EntryCard e_second;
      e_second.crystal_id = static_cast<int>(gui::g_state.crystals.size());
      gui::g_state.crystals.push_back(c_second);
      gui::g_state.layers[0].entries.push_back(e_second);
      ctx->Yield(2);
      const int cid1 = gui::g_state.layers[0].entries[1].crystal_id;
      const float orig_h1 = gui::g_state.crystals[cid1].height.center;
      IM_CHECK_EQ(orig_h1, 5.0f);

      // Open entry 0's Crystal tab, focus Height and type without committing.
      gui::EditRequest req0{ gui::EditTarget::kCrystal, /*layer_idx=*/0, /*entry_idx=*/0 };
      gui::OpenEditModal(req0, gui::g_state);
      ctx->Yield(4);
      ctx->ItemClick("**/##Height##modal_cr_input");
      ctx->KeyCharsAppend("9");
      ctx->Yield(2);
      // Precondition: the Height input SPECIFICALLY is the active edit target, so
      // the "9" keystroke actually landed in the leak source (an InputFloat
      // commits on deactivation, not live, so we can't assert entry 0's height
      // changed yet — instead pin the active item to Height's resolved ID). If
      // the click/type ever stops landing, this fails loudly rather than letting
      // the core assertion below pass vacuously.
      const ImGuiID height_id = ctx->ItemInfo("**/##Height##modal_cr_input").ID;
      IM_CHECK(height_id != 0);
      IM_CHECK_EQ(ImGui::GetActiveID(), height_id);

      // Switch to entry 1's Crystal tab, then deactivate the (pre-fix still
      // active) Height input via Enter to force the InputFloat writeback. With
      // the fix, entry 1's Height input has a different ID than entry 0's pending
      // edit, so InputTextDeactivatedState never matches and Enter is a no-op.
      gui::EditRequest req1{ gui::EditTarget::kCrystal, /*layer_idx=*/0, /*entry_idx=*/1 };
      gui::OpenEditModal(req1, gui::g_state);
      ctx->Yield(4);
      ctx->KeyPress(ImGuiKey_Enter);
      ctx->Yield(2);

      // Core AC2: entry 1's crystal height must be unchanged (not polluted by
      // the "9" typed into entry 0's Height field).
      IM_CHECK_EQ(gui::g_state.crystals[cid1].height, orig_h1);

      ctx->ItemClick("**/Close##edit_modal");  // Immediate mode: single Close button
      ctx->Yield(2);
    };
  }

  // P1: Edit modal OK without any change must NOT clear the rendered preview
  // or arm Revert. Regression guard for scrum-gui-polish-v7 152.2: previously
  // CommitAllBuffers unconditionally MarkStructHardDirty()'d after any OK,
  // wiping snapshot_intensity + flipping sim_state to kModified.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_edit", "ok_no_change_preserves_state");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Simulate "finite rays just finished": sim_state == kDone with a
      // non-zero snapshot intensity that the diff-gate must preserve. run_intent=kLoaded pins the
      // reconcile base to kDone (the harness main loop reconciles sim_state every frame, so a bare
      // sim_state write would be overwritten — the intent is what makes it stick).
      gui::g_state.run_intent = gui::RunIntent::kLoaded;
      gui::g_state.sim_state = gui::GuiState::SimState::kDone;
      gui::g_state.snapshot_intensity = 0.5f;
      gui::g_state.committed_epoch = 5;
      gui::g_state.display_epoch_floor = 0;
      gui::g_state.dirty = false;
      ctx->Yield();

      // Open Edit Entry modal and immediately click OK (no field touched).
      ctx->ItemClick("**/Edit##fi");
      ctx->Yield(4);
      ctx->ItemClick("**/" ICON_FA_CHECK " OK##edit_modal");
      ctx->Yield(2);

      // Render-invalidation must not have fired:
      //   - sim_state still kDone (no kModified ⇒ Revert button stays hidden)
      //   - snapshot_intensity preserved (>0 rather than ==0.5 so future
      //     normalization changes don't break the semantic assertion)
      //   - display_epoch_floor NOT raised (no filter fence armed)
      //   - dirty still false (no spurious unsaved-changes flag)
      IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(gui::GuiState::SimState::kDone));
      IM_CHECK_GT(gui::g_state.snapshot_intensity, 0.0f);
      IM_CHECK_EQ(gui::g_state.display_epoch_floor, 0u);
      IM_CHECK(!gui::g_state.dirty);
    };
  }

  // P1: Positive-path counterpart of ok_no_change_preserves_state — when the
  // OK actually changes the entry (here: Remove Filter on an entry that has
  // one), render-invalidation MUST fire. Guards against a regressed diff-gate
  // where operator== wrongly returns true.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_edit", "ok_with_change_invalidates");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Stage a filter on the entry so Remove Filter is a real change.
      gui::FilterConfig f;
      f.SetRaypath(gui::RaypathParams{ "3-1-5" });
      gui::SetFilter(gui::g_state, gui::g_state.layers[0].entries[0], f);
      // "finite rays just finished" snapshot state. run_intent=kLoaded pins the reconcile base to
      // kDone so the subsequent dirty edit surfaces as kModified (see companion test above).
      gui::g_state.run_intent = gui::RunIntent::kLoaded;
      gui::g_state.sim_state = gui::GuiState::SimState::kDone;
      gui::g_state.snapshot_intensity = 0.5f;
      gui::g_state.committed_epoch = 5;
      gui::g_state.display_epoch_floor = 0;
      gui::g_state.dirty = false;
      // task-classic-params-migration: seed last_committed_state at the "filter present"
      // baseline so ReconcileGuiEffects fires need_resim/hard-reset when Remove Filter takes
      // effect. Pre-migration this test worked because CommitAllBuffers manually fired
      // MarkDirty+MarkStructHardDirty unconditionally; post-migration the effect requires a real
      // baseline diff.
      gui::g_state.last_committed_state = gui::GuiState::ConfigSnapshot::From(gui::g_state);
      ctx->Yield();

      ctx->ItemClick("**/Edit##fi");
      ctx->Yield(4);
      ctx->ItemClick("**/Remove Filter##filter");
      ctx->Yield(2);
      ctx->ItemClick("**/" ICON_FA_CHECK " OK##edit_modal");
      ctx->Yield(2);

      // Render-invalidation MUST have fired (all four effects):
      IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(gui::GuiState::SimState::kModified));
      IM_CHECK_EQ(gui::g_state.snapshot_intensity, 0.0f);
      IM_CHECK_EQ(gui::g_state.display_epoch_floor, gui::g_state.committed_epoch);
      IM_CHECK(gui::g_state.dirty);
    };
  }

  // AC2 case 1/2: filter ADD (nullopt→"3-1-5") — Staged vs Immediate must agree.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_edit_modal", "ac2_staged_vs_immediate_filter_add");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      const Ac2Outcome staged = RunFilterPresenceToggleScenario(ctx, /*start_with_filter=*/false, /*immediate=*/false);
      const Ac2Outcome immediate =
          RunFilterPresenceToggleScenario(ctx, /*start_with_filter=*/false, /*immediate=*/true);
      IM_CHECK_EQ(staged.dirty, immediate.dirty);
      IM_CHECK_EQ(static_cast<int>(staged.sim_state), static_cast<int>(immediate.sim_state));
      IM_CHECK_EQ(staged.display_epoch_floor, immediate.display_epoch_floor);
      // Non-vacuous witness: both paths actually fired the hard reset (not a "both no-op" pass).
      IM_CHECK(staged.dirty);
      IM_CHECK_EQ(static_cast<int>(staged.sim_state), static_cast<int>(gui::GuiState::SimState::kModified));
    };
  }

  // AC2 case 2/2: filter REMOVE ("3-1-5"→nullopt) — Staged vs Immediate must agree.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_edit_modal", "ac2_staged_vs_immediate_filter_remove");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      const Ac2Outcome staged = RunFilterPresenceToggleScenario(ctx, /*start_with_filter=*/true, /*immediate=*/false);
      const Ac2Outcome immediate = RunFilterPresenceToggleScenario(ctx, /*start_with_filter=*/true, /*immediate=*/true);
      IM_CHECK_EQ(staged.dirty, immediate.dirty);
      IM_CHECK_EQ(static_cast<int>(staged.sim_state), static_cast<int>(immediate.sim_state));
      IM_CHECK_EQ(staged.display_epoch_floor, immediate.display_epoch_floor);
      IM_CHECK(staged.dirty);
      IM_CHECK_EQ(static_cast<int>(staged.sim_state), static_cast<int>(gui::GuiState::SimState::kModified));
    };
  }

  // [code-review-01.md merged Major #1 companion ask] Duplicate hard-reset widget-level
  // regression: cloning an entry that carries a filter appends a new `filters` pool slot, which
  // the field's kStructHard auto-diff picks up unconditionally on the next reconcile (see
  // progress.md Step 3-6 entry — this is pre-existing production behavior via T0's
  // main.cpp:367 ApplyGuiEffects, not new to this task; the removed hand-written MarkDirty was
  // redundant). Pins the observable effect end-to-end through the real Duplicate button.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_linked", "duplicate_with_filter_triggers_hard_reset");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      gui::FilterConfig f;
      f.SetRaypath(gui::RaypathParams{ "3-1-5" });
      gui::SetFilter(gui::g_state, gui::g_state.layers[0].entries[0], f);
      gui::g_state.run_intent = gui::RunIntent::kLoaded;
      gui::g_state.sim_state = gui::GuiState::SimState::kDone;
      gui::g_state.snapshot_intensity = 0.5f;
      gui::g_state.committed_epoch = 5;
      gui::g_state.display_epoch_floor = 0;
      gui::g_state.dirty = false;
      gui::g_state.last_committed_state = gui::GuiState::ConfigSnapshot::From(gui::g_state);
      ctx->Yield(2);

      ctx->ItemClick("**/" ICON_FA_COPY "##dup_0_0");
      ctx->Yield(2);

      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers[0].entries.size()), 2);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(gui::GuiState::SimState::kModified));
      IM_CHECK_EQ(gui::g_state.snapshot_intensity, 0.0f);
      IM_CHECK_EQ(gui::g_state.display_epoch_floor, gui::g_state.committed_epoch);
      IM_CHECK(gui::g_state.dirty);
    };
  }

  // P1: scrum-gui-polish-v11 / task-modal-immediate-mode M2 gate — in
  // Immediate mode, crystal-only buffer edits must MarkDirty but NOT
  // MarkStructHardDirty. This is what keeps infinite-rays accumulation alive
  // while the user drags a Crystal slider. Filter edits still fire
  // MarkStructHardDirty (identical to Staged OK semantics for filter changes).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_edit_modal", "immediate_crystal_does_not_clear_display");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      gui::g_state.modal_immediate_mode = false;
      ctx->Yield(2);

      // Seed "finite rays just finished" state — an accumulated preview the
      // Immediate mode is supposed to preserve while the user tweaks Crystal.
      gui::g_state.run_intent = gui::RunIntent::kLoaded;
      gui::g_state.sim_state = gui::GuiState::SimState::kDone;
      gui::g_state.snapshot_intensity = 0.5f;
      gui::g_state.committed_epoch = 5;
      gui::g_state.display_epoch_floor = 0;
      gui::g_state.dirty = false;
      // task-classic-params-migration: seed baseline so Crystal edits produce a real
      // `crystals` diff in ReconcileGuiEffects. Pre-migration this test worked because
      // CommitAllBuffersImmediate manually MarkDirty'd unconditionally; post-migration the
      // dirty flag is only set by the reconciler when it sees a diff against the baseline.
      gui::g_state.last_committed_state = gui::GuiState::ConfigSnapshot::From(gui::g_state);
      ctx->Yield();

      const float orig_h = gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id].height.center;

      // Open Edit modal (Staged), then toggle Immediate — triggers close+reopen.
      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(4);
      ctx->ItemClick("**/Immediate##edit_modal");
      ctx->Yield(6);
      // After close+reopen the Crystal tab may need an explicit SetSelected —
      // the original OpenEditModal path set g_pending_tab_select, but mode
      // switch reuses the existing buffer without re-entering OpenEditModal.
      ctx->ItemClick("**/###crystal_tab");
      ctx->Yield(2);  // 1 frame for CloseCurrentPopup, 1 for HandlePopupClosed,
                      // 1 for OpenPopup, plus a little slack for tab SetSelected.

      // Crystal-only edit: change Height via the Crystal tab input. Must
      // MarkDirty (state.dirty == true) but must NOT MarkStructHardDirty
      // (snapshot_intensity preserved, display_epoch_floor NOT raised).
      ctx->ItemInputValue("**/##Height##modal_cr_input", orig_h + 1.0f);
      ctx->Yield(2);
      IM_CHECK(gui::g_state.dirty);
      IM_CHECK_GT(gui::g_state.snapshot_intensity, 0.0f);
      IM_CHECK_EQ(gui::g_state.display_epoch_floor, 0u);

      // Filter edit: switch to the Filter tab and type a raypath. Because the
      // entry had no filter at modal open (initial_present=false), the first
      // raypath edit is itself the filter creation — filter_changed == true
      // in ApplyBuffersToEntry → MarkStructHardDirty fires → display clears.
      ctx->ItemClick("**/###filter_tab");
      ctx->Yield(4);
      ctx->ItemInputValue("**/##row_text_0", "3-1-5");
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.snapshot_intensity, 0.0f);
      IM_CHECK_EQ(gui::g_state.display_epoch_floor, gui::g_state.committed_epoch);

      // Close the Immediate popup and restore Staged default for later tests.
      ctx->ItemClick("**/Close##edit_modal");
      ctx->Yield(2);
      gui::g_state.modal_immediate_mode = false;
    };
  }

  // P1: scrum-gui-polish-v14 / task-filter-invalid-raypath-no-apply —
  // Immediate mode: invalid / incomplete raypath must not reach entry.filter.
  // Model-layer invariant enforced at ApplyBuffersToEntry: FilterConfig never
  // holds a non-kValid raypath, regardless of commit mode. Staged mode's OK
  // button already enforces via disabled gate; this test covers the Immediate
  // per-frame commit path — kInvalid ("abc"), kIncomplete ("3-") both skip,
  // while kValid ("3-1-5") and empty (→ nullopt) continue to commit normally.
  //
  // Three-layer assertion pattern (avoids false-positive tautologies):
  //   L1 precondition — verify the buffer actually received the typed text by
  //      checking the derived "Remove Filter" enable state (disabled when the
  //      InputText is empty), ruling out a silent ItemInputValue no-op.
  //   L2 guard effect — assert entry.filter.raypath_text stays at the last
  //      valid commit (the guard intercepted the invalid write).
  //   L3 alt-path negation — assert entry.filter remains .has_value() (guard
  //      did not mistakenly clear to nullopt).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_edit_modal", "immediate_invalid_raypath_does_not_commit");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      gui::g_state.modal_immediate_mode = true;
      ctx->Yield(2);

      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(4);
      ctx->ItemClick("**/###filter_tab");
      ctx->Yield(4);

      // Establish a valid baseline commit so subsequent guard-intercepted
      // frames have a "last known good" to preserve.
      ctx->ItemInputValue("**/##row_text_0", "3-1");
      ctx->Yield(2);
      IM_CHECK(gui::g_state.layers[0].entries[0].filter_id.has_value());
      IM_CHECK_STR_EQ(gui::g_state.filters[*gui::g_state.layers[0].entries[0].filter_id].RaypathText().c_str(), "3-1");

      // kInvalid: pure non-numeric "abc" — syntactically invalid.
      // (H5 anti-tautology witness: the earlier "type '3-1' → entry.filter
      // becomes has_value" assertion above already proved ItemInputValue
      // actually reaches g_summand_rows[0]. Under H5 Remove Filter is a
      // session-level intent flag (always enabled), so the pre-H5 "Remove
      // enabled iff textbox non-empty" probe no longer distinguishes silent
      // no-ops — dropped.)
      ctx->ItemInputValue("**/##row_text_0", "abc");
      ctx->Yield(2);
      // Guard intercepted — entry.filter preserved at last valid.
      IM_CHECK(gui::g_state.layers[0].entries[0].filter_id.has_value());
      IM_CHECK_STR_EQ(gui::g_state.filters[*gui::g_state.layers[0].entries[0].filter_id].RaypathText().c_str(), "3-1");

      // kIncomplete: trailing separator "3-" — syntactically incomplete per
      // ValidateSummandText rules. Guard treats same as kInvalid (Staged OK
      // disjunction `v.state != kValid`).
      ctx->ItemInputValue("**/##row_text_0", "3-");
      ctx->Yield(2);
      IM_CHECK(gui::g_state.layers[0].entries[0].filter_id.has_value());
      IM_CHECK_STR_EQ(gui::g_state.filters[*gui::g_state.layers[0].entries[0].filter_id].RaypathText().c_str(), "3-1");

      // Valid recovery: a fresh valid raypath must commit normally, proving
      // the guard does not wedge the entry permanently after a rejection.
      ctx->ItemInputValue("**/##row_text_0", "3-1-5");
      ctx->Yield(2);
      IM_CHECK(gui::g_state.layers[0].entries[0].filter_id.has_value());
      IM_CHECK_STR_EQ(gui::g_state.filters[*gui::g_state.layers[0].entries[0].filter_id].RaypathText().c_str(),
                      "3-1-5");

      // Empty → nullopt (161.2 rule unchanged by this task).
      ctx->ItemInputValue("**/##row_text_0", "");
      ctx->Yield(2);
      IM_CHECK(!gui::g_state.layers[0].entries[0].filter_id.has_value());

      ctx->ItemClick("**/Close##edit_modal");
      ctx->Yield(2);
      gui::g_state.modal_immediate_mode = false;
    };
  }

  // P1: scrum-gui-polish-v11 / task-modal-immediate-mode Test 1 — Immediate
  // mode: slider edits are committed to state immediately (no OK needed).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_edit_modal", "immediate_slider_commits_immediately");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      // Open the modal directly in Immediate mode (skip the close+reopen
      // mode-switch path — that is covered by Test 5).
      gui::g_state.modal_immediate_mode = true;
      ctx->Yield(2);
      const float orig_h = gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id].height.center;

      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(4);

      // Change Height — must reach the entry on the same frame (Immediate
      // commits every frame via CommitAllBuffersImmediate).
      ctx->ItemInputValue("**/##Height##modal_cr_input", orig_h + 2.5f);
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id].height, orig_h + 2.5f);

      ctx->ItemClick("**/Close##edit_modal");
      ctx->Yield(2);
      gui::g_state.modal_immediate_mode = false;
    };
  }

  // P1: scrum-gui-polish-v11 / task-modal-immediate-mode Test 2 — Immediate
  // mode: tab titles never carry the " *" dirty-mark even after edits
  // (Immediate has no staged-vs-committed distinction).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_edit_modal", "immediate_no_dirty_mark");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      auto is_dirty = [&](const char* tab_ref) -> bool {
        auto info = ctx->ItemInfo(tab_ref);
        return info.ID != 0 && std::strstr(info.DebugLabel, "*") != nullptr;
      };

      ResetTestState();
      gui::g_state.modal_immediate_mode = true;
      ctx->Yield(2);
      const float orig_h = gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id].height.center;

      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(4);

      ctx->ItemInputValue("**/##Height##modal_cr_input", orig_h + 3.0f);
      ctx->Yield(2);
      IM_CHECK(!is_dirty("**/###crystal_tab"));
      IM_CHECK(!is_dirty("**/###axis_tab"));
      IM_CHECK(!is_dirty("**/###filter_tab"));

      ctx->ItemClick("**/Close##edit_modal");
      ctx->Yield(2);
      gui::g_state.modal_immediate_mode = false;
    };
  }

  // P1: scrum-gui-polish-v11 / task-modal-immediate-mode Test 3 — Immediate
  // Close preserves all edits (no Cancel semantics in Immediate mode).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_edit_modal", "immediate_close_preserves_changes");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      gui::g_state.modal_immediate_mode = true;
      ctx->Yield(2);
      const float orig_h = gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id].height.center;

      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(4);

      ctx->ItemInputValue("**/##Height##modal_cr_input", orig_h + 4.0f);
      ctx->Yield(2);
      ctx->ItemClick("**/Close##edit_modal");
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id].height, orig_h + 4.0f);

      // Reopen to confirm the value truly persisted through close (guards
      // against any path that might silently revert buffer→entry mapping).
      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(4);
      IM_CHECK_EQ(gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id].height, orig_h + 4.0f);
      ctx->ItemClick("**/Close##edit_modal");
      ctx->Yield(2);
      gui::g_state.modal_immediate_mode = false;
    };
  }

  // P1: scrum-gui-polish-v11 / task-modal-immediate-mode Test 4 — Immediate
  // Esc closes the popup while keeping edits intact (ImGui default popup
  // Esc-close behavior; no Cancel-like revert because Immediate has no
  // staged buffer to discard).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_edit_modal", "immediate_esc_closes_without_revert");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      gui::g_state.modal_immediate_mode = true;
      ctx->Yield(2);
      const float orig_h = gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id].height.center;

      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(4);

      ctx->ItemInputValue("**/##Height##modal_cr_input", orig_h + 5.0f);
      ctx->Yield(2);

      // ImGui's default popup handling closes the top popup on Escape.
      ctx->KeyPress(ImGuiKey_Escape);
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id].height, orig_h + 5.0f);

      gui::g_state.modal_immediate_mode = false;
    };
  }

  // P1: scrum-gui-polish-v11 / task-modal-immediate-mode Test 5 — mode switch
  // close+reopen preserves the Filter tab selection + buffered edits. Uses the
  // Filter tab because its controls (Raypath InputText) have simple wildcard
  // paths; the preservation contract is the same as for any tab.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_edit_modal", "mode_switch_preserves_tab_and_buffer");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      gui::g_state.modal_immediate_mode = false;
      ctx->Yield(2);

      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(4);
      // Switch to Filter tab in Staged mode.
      ctx->ItemClick("**/###filter_tab");
      ctx->Yield(4);
      // Stage a raypath in the Filter tab buffer. Staged mode: entry still
      // has no filter until OK.
      ctx->ItemInputValue("**/##row_text_0", "3-1-5");
      ctx->Yield(2);
      IM_CHECK(!gui::g_state.layers[0].entries[0].filter_id.has_value());

      // Toggle Immediate: switch commits the buffer via CommitAllBuffersImmediate,
      // then close+reopen preserves tab selection + buffer.
      ctx->ItemClick("**/Immediate##edit_modal");
      ctx->Yield(8);

      // Filter is now populated on the entry (committed by the switch).
      IM_CHECK(gui::g_state.layers[0].entries[0].filter_id.has_value());
      IM_CHECK_STR_EQ(gui::g_state.filters[*gui::g_state.layers[0].entries[0].filter_id].RaypathText().c_str(),
                      "3-1-5");
      // Filter tab controls still accessible — tab selection survived.
      IM_CHECK(ctx->ItemExists("**/##row_text_0"));

      ctx->ItemClick("**/Close##edit_modal");
      ctx->Yield(2);
      gui::g_state.modal_immediate_mode = false;
    };
  }

  // P1: scrum-gui-polish-v12 / task-immediate-modal-passthrough — Immediate
  // modal external-click passthrough: clicks outside the Edit modal must
  // reach background UI (e.g. topbar menus open) and must NOT close the
  // modal. Asserts both the passthrough direction (external button responds)
  // and the persistence direction (modal stays visible).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_edit_modal", "immediate_passthrough_external_click");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      gui::g_state.modal_immediate_mode = false;
      ctx->Yield(2);

      // Open the Edit modal in Staged mode, then switch to Immediate (covers
      // the Staged → Immediate mode-switch consume path as a side benefit).
      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(4);
      ctx->ItemClick("**/Immediate##edit_modal");
      ctx->Yield(8);

      // Modal is visible.
      IM_CHECK(ctx->ItemExists("**/###crystal_tab"));

      // Click an external topbar button. ##TopBar/Save opens the Save menu;
      // picked over ##TopBar/New because the default Immediate window origin
      // (ImGui default (60, 60) since no position management this task) may
      // overlap the upper-left area, and over Scene-group inputs that can be
      // geometrically covered by the AlwaysAutoResize window extending
      // downward.
      ctx->ItemClick("##TopBar/Save");
      ctx->Yield(2);

      // Passthrough direction: the Save click reached the background button —
      // the Save menu opened (Save Copy appears as a child). Depends on the
      // topbar Save menu containing a "Save Copy" entry; rename the menu item
      // and this assertion must be updated (see p1_file/save_menu_structure
      // for the canonical menu contract).
      IM_CHECK(ctx->ItemExists("**/Save Copy"));
      // Persistence direction: the external click did NOT close the modal.
      IM_CHECK(ctx->ItemExists("**/###crystal_tab"));

      // Dismiss the Save menu, then close the modal cleanly.
      ctx->KeyPress(ImGuiKey_Escape);
      ctx->Yield(2);
      ctx->ItemClick("**/Close##edit_modal");
      ctx->Yield(4);
      // Close-button contract: modal is gone after Close. Duplicates the
      // core check from immediate_close_button_closes but keeps this test
      // self-contained so a Close regression does not silently pass by
      // relying on the next test's ResetTestState() to mask it.
      IM_CHECK(!ctx->ItemExists("**/###crystal_tab"));
      gui::g_state.modal_immediate_mode = false;
    };
  }

  // P1: scrum-gui-polish-v12 / task-immediate-modal-passthrough — Immediate
  // modal explicit close: the bottom Close button resets g_active_modal,
  // and the next frame's Begin(p_open=false) → !window_open path tears the
  // window down. Asserts the modal disappears after a single Close click.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_edit_modal", "immediate_close_button_closes");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      gui::g_state.modal_immediate_mode = true;
      ctx->Yield(2);

      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(4);
      IM_CHECK(ctx->ItemExists("**/###crystal_tab"));

      ctx->ItemClick("**/Close##edit_modal");
      ctx->Yield(4);
      IM_CHECK(!ctx->ItemExists("**/###crystal_tab"));

      gui::g_state.modal_immediate_mode = false;
    };
  }

  // P1: scrum-gui-polish-v13 / task-immediate-modal-full-close — Close button
  // path: after clicking Close, the Edit Entry window must fully disappear
  // (WasActive == false or window destroyed). Existing test
  // immediate_close_button_closes only asserts body items are gone; this test
  // asserts the window title bar also disappears — regression for the
  // Immediate-mode bug where ImGui::Begin keeps rendering a tomb-stone title
  // bar after *p_open=false (docking/viewport retention).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_edit_modal", "immediate_close_button_hides_window");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      gui::g_state.modal_immediate_mode = true;
      ctx->Yield(2);

      // Arrange: open the modal and assert precondition (window truly visible).
      // Using ctx->GetWindowByRef is Test Engine's thread-safe lookup; direct
      // ImGui::FindWindowByName is avoided per learnings on TestFunc/GuiFunc
      // threading.
      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(4);
      ImGuiWindow* w_open = ctx->GetWindowByRef("Edit Entry");
      IM_CHECK(w_open != nullptr);
      IM_CHECK(w_open->WasActive == true);

      // Act: click Close button (label "Close##edit_modal", verified from
      // edit_modals.cpp:968).
      ctx->ItemClick("**/Close##edit_modal");
      ctx->Yield(3);

      // Assert: window fully hidden.
      ImGuiWindow* w_after = ctx->GetWindowByRef("Edit Entry");
      IM_CHECK(w_after == nullptr || w_after->WasActive == false);

      gui::g_state.modal_immediate_mode = false;
    };
  }

  // P1: scrum-gui-polish-v13 / task-immediate-modal-full-close — Title-bar ×
  // path: clicking the × in the title bar must also fully hide the window.
  // Uses Test Engine's WindowClose API (imgui_te_context.h:312), which
  // triggers the same internal close that clicking × would.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_edit_modal", "immediate_title_x_hides_window");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      gui::g_state.modal_immediate_mode = true;
      ctx->Yield(2);

      // Arrange: precondition — window visible.
      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(4);
      ImGuiWindow* w_open = ctx->GetWindowByRef("Edit Entry");
      IM_CHECK(w_open != nullptr);
      IM_CHECK(w_open->WasActive == true);

      // Act: close via title-bar × (WindowClose triggers the ImGui internal
      // close path, equivalent to user clicking the × glyph).
      ctx->WindowClose("Edit Entry");
      ctx->Yield(3);

      // Assert: window fully hidden.
      ImGuiWindow* w_after = ctx->GetWindowByRef("Edit Entry");
      IM_CHECK(w_after == nullptr || w_after->WasActive == false);

      gui::g_state.modal_immediate_mode = false;
    };
  }

  // P1: scrum-gui-polish-v13 / task-immediate-modal-full-close — Reopen
  // regression: after closing the Immediate modal, reopening must yield the
  // same WasActive transition as the first open. Guards against any flag
  // leakage (g_pending_open / g_pending_mode_switch / g_pending_tab_select)
  // that could make the second session behave differently.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_edit_modal", "immediate_reopen_after_close");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      gui::g_state.modal_immediate_mode = true;
      ctx->Yield(2);

      // First open+close cycle.
      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(4);
      ImGuiWindow* w1 = ctx->GetWindowByRef("Edit Entry");
      IM_CHECK(w1 != nullptr && w1->WasActive);
      ctx->ItemClick("**/Close##edit_modal");
      ctx->Yield(3);
      ImGuiWindow* w1_after = ctx->GetWindowByRef("Edit Entry");
      IM_CHECK(w1_after == nullptr || !w1_after->WasActive);

      // Second open+close cycle — behavior must match first.
      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(4);
      ImGuiWindow* w2 = ctx->GetWindowByRef("Edit Entry");
      IM_CHECK(w2 != nullptr && w2->WasActive);
      ctx->ItemClick("**/Close##edit_modal");
      ctx->Yield(3);
      ImGuiWindow* w2_after = ctx->GetWindowByRef("Edit Entry");
      IM_CHECK(w2_after == nullptr || !w2_after->WasActive);

      gui::g_state.modal_immediate_mode = false;
    };
  }

  // P1: scrum-gui-polish-v12 / task-gui-window-zorder — Symptom A guard.
  // After T3 changed the Immediate Edit modal to ImGui::Begin (regular window),
  // background panels lacking NoBringToFrontOnFocus would be raised on click and
  // occlude the modal. Step 2a/2b added the flag to all 6 background panels.
  // This test focuses ##LeftPanel programmatically (no business side effect, no
  // popup) and asserts the Edit Entry stays at g.Windows.back() — i.e. topmost.
  // Reverse-validation: removing NoBringToFrontOnFocus from ##LeftPanel must
  // make the z-order assertion fail (g.Windows.back() becomes ##LeftPanel).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_edit_modal", "immediate_modal_above_background_panel");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      gui::g_state.modal_immediate_mode = true;
      ctx->Yield(2);

      // Open the Immediate Edit modal.
      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(4);
      IM_CHECK(ctx->ItemExists("**/###crystal_tab"));

      // Focus ##LeftPanel via the Test Engine's WindowFocus API (no business
      // side effect; goes through ImGui::FocusWindow). Without
      // NoBringToFrontOnFocus this would splice ##LeftPanel to g.Windows.back().
      ctx->WindowFocus("##LeftPanel");
      ctx->Yield(2);

      // Core assertion: Edit Entry is still the topmost root window.
      // BringWindowToDisplayFront splices to back of g.Windows; since
      // scrum-gui-polish-v12/task-modal-preview-cross-tab the modal contains
      // BeginChild panes (##modal_left_pane / ##modal_right_pane) that appear
      // after their parent in g.Windows with ImGuiWindowFlags_ChildWindow set;
      // walk backwards to find the topmost non-child window for the invariant.
      ImGuiContext* g = ImGui::GetCurrentContext();
      IM_CHECK(g->Windows.Size > 0);
      ImGuiWindow* topmost_root = nullptr;
      for (int i = g->Windows.Size - 1; i >= 0; i--) {
        ImGuiWindow* w = g->Windows[i];
        if ((w->Flags & ImGuiWindowFlags_ChildWindow) == 0) {
          topmost_root = w;
          break;
        }
      }
      IM_CHECK(topmost_root != nullptr);
      IM_CHECK_STR_EQ(topmost_root->Name, "Edit Entry");

      // Behavior fallback: Close still works (sanity that the modal is
      // actually responding to clicks, not just visually layered).
      ctx->ItemClick("**/Close##edit_modal");
      ctx->Yield(4);
      IM_CHECK(!ctx->ItemExists("**/###crystal_tab"));

      // ResetTestState() (calls ResetModalState() which restores
      // modal_immediate_mode = false). Belt-and-braces against IM_CHECK early-
      // return: even if an assertion above fires, the next test's first action
      // is its own ResetTestState() — but call it here too for clarity.
      ResetTestState();
    };
  }

  // P1: scrum-gui-polish-v15 / task-detachable-modal-impl — Immediate Edit Entry
  // must retain ImGuiWindowFlags_NoDocking so the multi-viewport mechanism only
  // creates an independent OS viewport when the user drags it outside the main
  // window, never docks it into the main window. Runtime detach behavior itself
  // is validated by macOS manual QA in plan Step 5 — this test is the structural
  // guard against accidental NoDocking removal.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_edit_modal", "immediate_modal_viewport_flags_preserved");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      gui::g_state.modal_immediate_mode = true;
      ctx->Yield(2);

      // Note: runtime flipping of ImGuiConfigFlags_ViewportsEnable inside the
      // hidden test GLFW window crashes the backend (platform window creation
      // fails under the test harness). Do not enable the flag here — this
      // test only asserts the structural contract that is required for
      // production (main.cpp sets the flag at init) to work correctly; actual
      // detach behavior is validated by macOS manual QA (plan Step 5).

      // Open Edit Entry Immediate mode.
      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(4);
      ImGuiWindow* w = ctx->GetWindowByRef("Edit Entry");
      IM_CHECK(w != nullptr);
      IM_CHECK(w->WasActive);

      // Contract: NoDocking must be on the Immediate branch ImGui::Begin
      // flags. With ViewportsEnable (enabled in production), NoDocking lets
      // the window become an independent OS viewport when dragged outside
      // the main window without accidentally docking into a main-window
      // split target.
      IM_CHECK((w->Flags & ImGuiWindowFlags_NoDocking) != 0);

      // On first open, the window is placed in the main viewport (gated by
      // NoSavedSettings semantics via io.IniFilename = nullptr in main.cpp).
      // Independent viewport creation is triggered by a user drag and is not
      // exercised in the test harness.
      IM_CHECK(w->Viewport == ImGui::GetMainViewport());

      // Cleanup.
      ctx->ItemClick("**/Close##edit_modal");
      ctx->Yield(3);
      gui::g_state.modal_immediate_mode = false;
    };
  }

  // P1: scrum-gui-polish-v12 / task-gui-window-zorder — z-order invariant
  // between LogPanel (Layer 3, no flag, push_back -> top) and LeftPanel
  // (background cluster, NoBringToFrontOnFocus, push_front -> bottom).
  // Asserts the index of ##LogPanel in g.Windows is greater than that of
  // ##LeftPanel after we explicitly try to raise ##LeftPanel. See the
  // z-order convention block at the top of src/gui/app_panels.cpp.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_layout", "log_panel_above_left_panel");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();

      // RenderLogPanel is gated twice in the test harness:
      //   1. test_gui_main.cpp's main loop only calls it if g_enable_log_panel
      //   2. RenderLogPanel itself early-returns if g_imgui_log_sink is null
      //      (the sink is normally initialized only when --log-panel is on).
      // RAII guard restores both gates regardless of whether IM_CHECK fires —
      // ResetTestState() doesn't touch these test-harness CLI globals, so a
      // mid-test assertion failure would otherwise leak the log panel into
      // every subsequent test in this binary.
      struct LogPanelGateGuard {
        bool prev_enable;
        std::shared_ptr<gui::ImGuiLogSink> prev_sink;
        LogPanelGateGuard() : prev_enable(g_enable_log_panel), prev_sink(gui::g_imgui_log_sink) {
          g_enable_log_panel = true;
          if (!gui::g_imgui_log_sink) {
            gui::g_imgui_log_sink = std::make_shared<gui::ImGuiLogSink>();
          }
        }
        ~LogPanelGateGuard() {
          gui::g_imgui_log_sink = prev_sink;
          g_enable_log_panel = prev_enable;
          gui::g_state.log_panel_open = false;
        }
      } gate_guard;

      gui::g_state.log_panel_open = true;
      ctx->Yield(4);

      // Try to raise ##LeftPanel. Without NoBringToFrontOnFocus it would
      // splice ##LeftPanel to the back of g.Windows.
      ctx->WindowFocus("##LeftPanel");
      ctx->Yield(2);

      // ImGui creates windows without NoBringToFrontOnFocus via
      // g.Windows.push_back (= visual top); windows with the flag via
      // push_front (= visual bottom). LogPanel intentionally has no flag
      // (Layer 3 floating); LeftPanel does (background cluster). Therefore
      // LogPanel's index in g.Windows must be greater than LeftPanel's.
      ImGuiContext* g = ImGui::GetCurrentContext();
      IM_CHECK(g != nullptr);
      int log_idx = -1;
      int left_idx = -1;
      for (int i = 0; i < g->Windows.Size; ++i) {
        if (strcmp(g->Windows[i]->Name, "##LogPanel") == 0) {
          log_idx = i;
        } else if (strcmp(g->Windows[i]->Name, "##LeftPanel") == 0) {
          left_idx = i;
        }
      }
      IM_CHECK(log_idx >= 0);
      IM_CHECK(left_idx >= 0);
      IM_CHECK_GT(log_idx, left_idx);

      // gate_guard restores log-panel CLI globals + log_panel_open here.
    };
  }

  // task-345.5 (⑥): Colors button moved from status bar into a new
  // "feature button" group on the top bar, right of Save. Verifies:
  //   - new top-bar item exists at the expected path,
  //   - old status-bar item is gone,
  //   - clicking toggles color_window_open (both directions).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_layout", "colors_button_relocated_to_topbar");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      IM_CHECK(ctx->ItemExists("##TopBar/" ICON_FA_PALETTE " Colors"));
      IM_CHECK(!ctx->ItemExists("##StatusBar/" ICON_FA_PALETTE " Colors"));

      const bool initial = gui::g_state.color_window_open;
      ctx->ItemClick("##TopBar/" ICON_FA_PALETTE " Colors");
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.color_window_open, !initial);

      ctx->ItemClick("##TopBar/" ICON_FA_PALETTE " Colors");
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.color_window_open, initial);
    };
  }

  // task-349.3 (#4): the colored/full-spectrum display-time toggle evolved as
  //   346.3 Checkbox → 348.3 icon-only Button → 349.3 plain-text Checkbox
  // (owner: two adjacent PALETTE-icon controls confused which was the toggle
  // vs. the window opener). Widget path is once again label-dependent:
  //   "##TopBar/Colored##CompositePreviewToggle"       (composite_now == true)
  //   "##TopBar/Full Spectrum##CompositePreviewToggle" (composite_now == false)
  // The four semantic assertions pinned here mirror the 346.3 originals — the
  // 348.3 icon-Button widget path and the 346.3 icon-plus-label Checkbox path
  // are both retained as legacy negative regressions.
  //   - AC4: no raypath_color class ⇒ no toggle anywhere (any new/legacy path,
  //     top bar or status bar).
  //   - AC1: with one color class, the toggle exists at the appropriate new
  //     top-bar Checkbox path and is gone from every legacy path.
  //   - AC3 (346.3, ≠ 348.3 AC3): the toggle stays visible while
  //     color_window_open is false — persistent marker independent of the
  //     Colors window's own render call.
  //   - AC2 (346.3): clicking flips g_state.show_composite_preview and leaves
  //     raypath_color / dirty untouched (display-time only, no re-simulation).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_layout", "colored_toggle_topbar_checkbox");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Compute the current Checkbox path from ground truth — label swaps with
      // last_uploaded_as_composite, so the expected id must be recomputed on
      // every assertion rather than cached as a static string (349.3 review
      // Minor #3: converge to a single "recompute on each check" pattern).
      auto current_path = []() {
        return gui::g_state.last_uploaded_as_composite ? std::string("##TopBar/Colored##CompositePreviewToggle") :
                                                         std::string("##TopBar/Full Spectrum##CompositePreviewToggle");
      };
      const char* legacy_icon_only = "##TopBar/" ICON_FA_PALETTE "##CompositePreviewToggle";
      const char* legacy_icon_colored = "##TopBar/" ICON_FA_PALETTE " Colored##CompositePreviewToggle";
      const char* legacy_icon_full = "##TopBar/" ICON_FA_PALETTE " Full Spectrum##CompositePreviewToggle";
      const char* legacy_status_colored = "##StatusBar/" ICON_FA_PALETTE " Colored##CompositePreviewToggle";
      const char* legacy_status_full = "##StatusBar/" ICON_FA_PALETTE " Full Spectrum##CompositePreviewToggle";

      // AC4 — no color classes ⇒ nothing rendered at any known path (new
      // Checkbox path in either mode + every legacy path).
      IM_CHECK(gui::g_state.raypath_color.empty());
      IM_CHECK(!ctx->ItemExists("##TopBar/Colored##CompositePreviewToggle"));
      IM_CHECK(!ctx->ItemExists("##TopBar/Full Spectrum##CompositePreviewToggle"));
      IM_CHECK(!ctx->ItemExists(legacy_icon_only));
      IM_CHECK(!ctx->ItemExists(legacy_icon_colored));
      IM_CHECK(!ctx->ItemExists(legacy_icon_full));
      IM_CHECK(!ctx->ItemExists(legacy_status_colored));
      IM_CHECK(!ctx->ItemExists(legacy_status_full));

      // Install one color class + pin the ground truth to "not composite" so
      // the initial label is "Full Spectrum".
      gui::ColorClassConfig c;
      gui::g_state.raypath_color.push_back(c);
      gui::g_state.last_uploaded_as_composite = false;
      ctx->Yield(2);

      // AC1 — new Checkbox path exists at the mode-appropriate label; all
      // legacy paths (icon-only Button, icon-plus-label Checkbox, and both
      // status-bar variants) are gone.
      IM_CHECK(ctx->ItemExists(current_path().c_str()));
      IM_CHECK(!ctx->ItemExists(legacy_icon_only));
      IM_CHECK(!ctx->ItemExists(legacy_icon_colored));
      IM_CHECK(!ctx->ItemExists(legacy_icon_full));
      IM_CHECK(!ctx->ItemExists(legacy_status_colored));
      IM_CHECK(!ctx->ItemExists(legacy_status_full));

      // AC3 (346.3) — closing the Colors window does not hide the toggle.
      gui::g_state.color_window_open = false;
      ctx->Yield(2);
      IM_CHECK(ctx->ItemExists(current_path().c_str()));

      // AC2 (346.3) — click flips show_composite_preview only; raypath_color
      // / dirty untouched (display-time toggle, no re-simulation). Recompute
      // the path before every click so we survive the label swap that follows
      // once ground truth propagates.
      const bool pref_before = gui::g_state.show_composite_preview;
      const size_t classes_before = gui::g_state.raypath_color.size();
      const bool dirty_before = gui::g_state.dirty;
      ctx->ItemClick(current_path().c_str());
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.show_composite_preview, !pref_before);
      IM_CHECK_EQ(gui::g_state.raypath_color.size(), classes_before);
      IM_CHECK_EQ(gui::g_state.dirty, dirty_before);

      // Symmetric second click — recompute the path from current ground truth
      // to survive the label swap.
      ctx->ItemClick(current_path().c_str());
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.show_composite_preview, pref_before);
    };
  }

  // p1_layout/collapse_strip_click_works_when_unoccluded — AC3 regression for
  // task-color-window-mouse-capture: OverlayButton's `!io.WantCaptureMouse` gate
  // (app_panels.cpp:308) must not suppress the collapse strip's own click when no
  // floating window covers it (code-review round 1, Major finding on the missing
  // no-occlusion verification for this gate).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_layout", "collapse_strip_click_works_when_unoccluded");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      gui::g_state.left_panel_collapsed = true;
      ctx->Yield(2);

      // Mirror RenderCollapsedStrip/OverlayButton geometry (app_panels.cpp:291-333): the
      // strip spans [kTopBarHeight, window_height - kStatusBarHeight) and the button (local
      // anonymous-namespace constexpr kCollapseBtnSize=20.0f, not exported outside
      // app_panels.cpp) sits vertically centered within it.
      const ImGuiViewport* vp = ImGui::GetMainViewport();
      constexpr float kCollapseBtnSize = 20.0f;  // mirrors app_panels.cpp:291
      float strip_h = vp->Size.y - gui::kTopBarHeight - gui::kStatusBarHeight;
      float btn_y = gui::kTopBarHeight + (strip_h - kCollapseBtnSize) * 0.5f;
      ImVec2 click_pos(vp->Pos.x + kCollapseBtnSize * 0.5f, vp->Pos.y + btn_y + kCollapseBtnSize * 0.5f);

      ctx->MouseMoveToPos(click_pos);
      ctx->MouseClick(0);
      ctx->Yield(2);

      // Core AC3 assertion: with nothing occluding the strip, io.WantCaptureMouse is false
      // at this position, so the click still expands the panel as before this task's change.
      IM_CHECK_EQ(gui::g_state.left_panel_collapsed, false);
    };
  }

  // P1: Unsaved Changes Popup
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_file", "unsaved_popup");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Set dirty
      gui::g_state.dirty = true;
      ctx->Yield();

      // Click New — should trigger unsaved popup
      ctx->ItemClick("##TopBar/New");
      ctx->Yield(2);

      // Click "Don't Save" in the popup
      ctx->ItemClick("Unsaved Changes/Don't Save");

      // Verify state was reset
      IM_CHECK_EQ(gui::g_state.dirty, false);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers.size()), 1);
    };
  }

  // P1: scrum-gui-polish-v9 155.2 + 155.4 — Save popup structure and simulating gating.
  // 155.4 renamed "Panorama..." → "Dual Fisheye Equal Area..." and added "Equirectangular...".
  // Asserts the single Save dropdown hosts all 6 actions + Include Texture/Overlay checkboxes,
  // and that Save/Save Copy disable under simulating while read-only exports stay live.
  // Uses ItemInfo rather than clicking MenuItem("Save") because DoSave triggers the
  // native file dialog (blocking, not test-engine-driveable).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_file", "save_menu_structure");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // --- Case 1: not simulating — Save/Save Copy enabled ---
      // sim_state is reconcile-derived; drive intent (kNone→kIdle) instead of writing sim_state.
      gui::g_state.run_intent = gui::RunIntent::kNone;
      ctx->Yield();
      ctx->ItemClick("##TopBar/Save");
      ctx->Yield(2);

      // All menu items + checkbox present under the popup
      IM_CHECK(ctx->ItemExists("**/Save Copy"));
      IM_CHECK(ctx->ItemExists("**/Screenshot..."));
      IM_CHECK(ctx->ItemExists("**/Dual Fisheye Equal Area..."));
      IM_CHECK(ctx->ItemExists("**/Equirectangular..."));
      IM_CHECK(ctx->ItemExists("**/Config JSON..."));
      IM_CHECK(ctx->ItemExists("**/Include Texture in .lmc"));

      auto copy_info = ctx->ItemInfo("**/Save Copy");
      IM_CHECK((copy_info.ItemFlags & ImGuiItemFlags_Disabled) == 0);

      ctx->KeyPress(ImGuiKey_Escape);
      ctx->Yield(2);

      // --- Case 2: simulating — Save/Save Copy disabled, Config JSON still enabled ---
      gui::g_state.run_intent = gui::RunIntent::kRunning;
      ctx->Yield();
      ctx->ItemClick("##TopBar/Save");
      ctx->Yield(2);

      auto copy_info2 = ctx->ItemInfo("**/Save Copy");
      IM_CHECK((copy_info2.ItemFlags & ImGuiItemFlags_Disabled) != 0);
      auto cfg_info = ctx->ItemInfo("**/Config JSON...");
      IM_CHECK((cfg_info.ItemFlags & ImGuiItemFlags_Disabled) == 0);

      ctx->KeyPress(ImGuiKey_Escape);
      ctx->Yield(2);
      gui::g_state.run_intent = gui::RunIntent::kNone;
      ctx->Yield();
    };
  }

  // P1: task-pbd-gui-gate — D tooltip (i) not shown when axis config meets D conditions.
  // az = Uniform 360°, roll mean = 0 (default). IsDApplicableGuiAxis → true → no (i).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_filter", "d_tooltip_not_shown_when_applicable");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Default crystal: azimuth = Uniform 360°, roll mean = 0 — D is applicable.
      IM_CHECK_EQ(gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id].azimuth.type,
                  gui::AxisDistType::kUniform);
      IM_CHECK_EQ(gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id].azimuth.std, 360.0f);
      IM_CHECK_EQ(gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id].roll.mean, 0.0f);

      ctx->ItemClick("**/Edit##fi");
      ctx->Yield(4);

      // (i) should NOT be shown: modal's d_applicable should be true.
      IM_CHECK_EQ(gui::IsCurrentModalDApplicable(), true);

      ctx->ItemClick("**/" ICON_FA_XMARK " Cancel##edit_modal");
      ctx->Yield(2);
    };
  }

  // P1: task-pbd-gui-gate — D tooltip (i) shown when roll mean is not a multiple of 30.
  // az = Uniform 360°, roll mean = 15 → IsDApplicableGuiAxis → false → (i) shown.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_filter", "d_tooltip_shown_roll_not_multiple_of_30");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id].azimuth = { gui::AxisDistType::kUniform, 0.0f,
                                                                                      360.0f };
      gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id].roll = { gui::AxisDistType::kUniform, 15.0f,
                                                                                   360.0f };
      ctx->Yield();

      ctx->ItemClick("**/Edit##fi");
      ctx->Yield(4);

      // (i) should be shown: modal's d_applicable should be false.
      IM_CHECK_EQ(gui::IsCurrentModalDApplicable(), false);

      ctx->ItemClick("**/" ICON_FA_XMARK " Cancel##edit_modal");
      ctx->Yield(2);
    };
  }

  // P1: task-pbd-gui-gate — D tooltip (i) shown when azimuth is not Uniform.
  // az = kGauss (non-uniform), roll mean = 0 → IsDApplicableGuiAxis → false → (i) shown.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_filter", "d_tooltip_shown_az_not_uniform");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id].azimuth = { gui::AxisDistType::kGauss, 0.0f,
                                                                                      30.0f };
      gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id].roll = { gui::AxisDistType::kUniform, 0.0f,
                                                                                   360.0f };
      ctx->Yield();

      ctx->ItemClick("**/Edit##fi");
      ctx->Yield(4);

      // (i) should be shown: modal's d_applicable should be false.
      IM_CHECK_EQ(gui::IsCurrentModalDApplicable(), false);

      ctx->ItemClick("**/" ICON_FA_XMARK " Cancel##edit_modal");
      ctx->Yield(2);
    };
  }

  // P1: scrum-gui-polish-v9 155.3 — screenshot_include_overlay toggle and default value.
  // Does NOT exercise the actual PNG write — that requires driving the main loop between
  // ImGui_ImplOpenGL3_RenderDrawData and glfwSwapBuffers, which the Test Engine's Yield
  // timing does not cleanly expose. Pixel-level verification is covered by the manual
  // smoke documented in plan.md M2.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_file", "screenshot_overlay_toggle");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Default is false (keeps current behaviour: no overlay in Screenshot).
      IM_CHECK_EQ(gui::g_state.screenshot_include_overlay, false);

      // Open Save menu → toggle Include Overlay on.
      ctx->ItemClick("##TopBar/Save");
      ctx->Yield(2);
      ctx->ItemClick("**/Include Overlay in Screenshot");
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.screenshot_include_overlay, true);

      // Reopen and toggle back off.
      ctx->ItemClick("##TopBar/Save");
      ctx->Yield(2);
      ctx->ItemClick("**/Include Overlay in Screenshot");
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.screenshot_include_overlay, false);
    };
  }

  // AC3: new crystal defaults to non-random; enabling a shape field's randomization defaults its
  // type to uniform (owner-defined), and disabling collapses it back to NO_RANDOM with zero spread.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_edit_modal", "shape_dist_default_and_enable");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      // Immediate mode so checkbox edits commit to g_state every frame (mirrors the
      // immediate_slider_commits_immediately pattern); ResetTestState clears the flag to false.
      gui::g_state.modal_immediate_mode = true;
      ctx->Yield(2);
      auto crystal = []() -> gui::CrystalConfig& {
        return gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id];
      };
      // AC3: default not random for every shape field.
      IM_CHECK_EQ(crystal().height.type, gui::ShapeDistType::kNoRandom);
      for (int i = 0; i < 6; i++) {
        IM_CHECK_EQ(crystal().face_distance[i].type, gui::ShapeDistType::kNoRandom);
      }

      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(4);
      // AC3: enabling randomization defaults type to uniform.
      ctx->ItemClick("**/Randomize##rnd_Height##modal_cr");
      ctx->Yield(2);
      IM_CHECK_EQ(crystal().height.type, gui::ShapeDistType::kUniform);
      IM_CHECK_GT(crystal().height.spread, 0.0f);  // default spread = 0.2 * center (center default 1.0)

      // Disabling collapses back to NO_RANDOM and zeroes the (now meaningless) spread.
      ctx->ItemClick("**/Randomize##rnd_Height##modal_cr");
      ctx->Yield(2);
      IM_CHECK_EQ(crystal().height.type, gui::ShapeDistType::kNoRandom);
      IM_CHECK_EQ(crystal().height.spread, 0.0f);

      ctx->ItemClick("**/Close##edit_modal");
      ctx->Yield(2);
      gui::g_state.modal_immediate_mode = false;
    };
  }

  // AC2: face_distance unified view broadcasts type/spread to all 6 faces, and the per-face advanced
  // view can diverge a single face WITHOUT the unified view later silently overwriting the others
  // (plan §7 risk 2 — the very "silent data loss" class this scrum fixes).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_edit_modal", "shape_dist_face_unified_and_perface");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      gui::g_state.modal_immediate_mode = true;  // commit checkbox edits to g_state each frame
      ctx->Yield(2);
      auto crystal = []() -> gui::CrystalConfig& {
        return gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id];
      };
      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(4);
      ctx->ItemClick("**/Face Distance##modal");  // expand the Face Distance tree
      ctx->Yield(2);

      // Diverge two face centers BEFORE enabling unified randomization (via the always-visible
      // per-face center sliders — a legitimate, common operation path that does not require
      // randomization to be on). Regression coverage for code-review round 1: the "Randomize (all
      // faces)" enable branch must broadcast ONE shared spread value derived once, not compute
      // spread per-face from each face's own (now-divergent) center — the latter would silently
      // give faces different spreads under a control labelled "all faces".
      ctx->ItemInputValue("**/##Face 3##modal_fd_input", 1.2f);
      ctx->Yield(2);
      ctx->ItemInputValue("**/##Face 4##modal_fd_input", 0.6f);
      ctx->Yield(2);

      // AC2: unified "Randomize (all faces)" broadcasts uniform to every face.
      ctx->ItemClick("**/Randomize (all faces)##modal_fd_uni");
      ctx->Yield(2);
      for (int i = 0; i < 6; i++) {
        IM_CHECK_EQ(crystal().face_distance[i].type, gui::ShapeDistType::kUniform);
      }
      // Regression: with divergent centers, all 6 faces must still get the SAME broadcast spread.
      const float uni_spread = crystal().face_distance[0].spread;
      for (int i = 1; i < 6; i++) {
        IM_CHECK_EQ(crystal().face_distance[i].spread, uni_spread);
      }

      // Diverge face 0 via the per-face advanced view (its checkbox is labelled "Face 3").
      ctx->ItemClick("**/Per-face randomization##modal_fd_adv");
      ctx->Yield(2);
      ctx->ItemClick("**/Face 3##fd_adv_ck_0");
      ctx->Yield(2);
      IM_CHECK_EQ(crystal().face_distance[0].type, gui::ShapeDistType::kNoRandom);

      // Risk 2: passively re-rendering the (now-mixed) unified view across several frames must NOT
      // broadcast — faces 1..5 stay uniform, face 0 stays NO_RANDOM.
      ctx->Yield(6);
      IM_CHECK_EQ(crystal().face_distance[0].type, gui::ShapeDistType::kNoRandom);
      for (int i = 1; i < 6; i++) {
        IM_CHECK_EQ(crystal().face_distance[i].type, gui::ShapeDistType::kUniform);
      }

      // Collapse the two TreeNodes we expanded. ImGui persists TreeNode open/closed state per
      // window across tests (ResetTestState resets g_state but not ImGui storage), so leaving them
      // open would change the crystal-modal layout for later tests that assume the default-collapsed
      // Face Distance section.
      ctx->ItemClick("**/Per-face randomization##modal_fd_adv");
      ctx->ItemClick("**/Face Distance##modal");
      ctx->Yield(2);
      ctx->ItemClick("**/Close##edit_modal");
      ctx->Yield(2);
      gui::g_state.modal_immediate_mode = false;
    };
  }
}

// P2 tests
void RegisterP2Tests(ImGuiTestEngine* engine) {
  // P2: MarkDirty
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_state", "mark_dirty");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      ResetTestState();

      IM_CHECK_EQ(gui::g_state.dirty, false);

      // Mark dirty
      gui::g_state.MarkDirty();
      IM_CHECK_EQ(gui::g_state.dirty, true);

      // DoNew resets dirty
      gui::DoNew();
      IM_CHECK_EQ(gui::g_state.dirty, false);
    };
  }

  // P2: Lens switch — full-sky resets elevation/azimuth/roll
  // Renderer invariants run every frame in RenderPreviewPanel
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_render", "lens_switch");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Set non-zero view params
      gui::g_state.renderer.elevation = 45.0f;
      gui::g_state.renderer.azimuth = -30.0f;
      gui::g_state.renderer.roll = 10.0f;

      // Switch to full-sky lens type (index 4 = Dual Fisheye Equal Area)
      gui::g_state.renderer.lens_type = 4;

      // Renderer invariants in RenderPreviewPanel reset view angles for full-sky lenses
      ctx->Yield(3);

      IM_CHECK_EQ(gui::g_state.renderer.elevation, 0.0f);
      IM_CHECK_EQ(gui::g_state.renderer.azimuth, 0.0f);
      IM_CHECK_EQ(gui::g_state.renderer.roll, 0.0f);
    };
  }
}

// ========== task-test-gui-interaction: P1 Crystal/Filter CRUD + reference handling ==========

void RegisterP1InteractionTests(ImGuiTestEngine* engine) {
  // Copy model: crystal/axis/filter are embedded in each entry — no ID references.
  // Old "delete crystal reassigns scatter entry" tests are no longer applicable.

  // p1_card/layer_add_delete — add and remove a layer
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_card", "layer_add_delete");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers.size()), 1);

      // Add a layer
      ctx->ItemClick("**/+ Layer");
      ctx->Yield();
      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers.size()), 2);

      // Delete layer 1 via its per-layer header "x" button
      ctx->ItemClick("**/" ICON_FA_XMARK "##layer_1");
      ctx->Yield();
      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers.size()), 1);
    };
  }

  // p1_prob_footguns — task-gui-ms-prob-footguns: cover the four-state prob
  // slider and the "+ Layer" continuation-prob promotion. Guards two edit-time
  // footguns without changing core rendering semantics.

  // last_layer_prob_zero_disables_slider: default single-layer (prob=0) → the
  // Prob slider must be disabled. Same test asserts the degenerate single-layer
  // case remains covered (was `single_layer` in old code).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_prob_footguns", "last_layer_prob_zero_disables_slider");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers.size()), 1);
      IM_CHECK(gui::IsProbZero(gui::g_state.layers[0].probability));
      auto info = ctx->ItemInfo("**/##Prob.##layer_0_input");
      IM_CHECK((info.ItemFlags & ImGuiItemFlags_Disabled) != 0);
    };
  }

  // last_layer_prob_nonzero_enables_slider: a hand-written config's last layer
  // with prob>0 → slider must be ENABLED (user can drag it back to 0). Guards
  // against silently locking the user out of a loaded footgun value.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_prob_footguns", "last_layer_prob_nonzero_enables_slider");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      gui::g_state.layers[0].probability = 0.5f;
      ctx->Yield(2);
      auto info = ctx->ItemInfo("**/##Prob.##layer_0_input");
      IM_CHECK((info.ItemFlags & ImGuiItemFlags_Disabled) == 0);
    };
  }

  // intermediate_layer_prob_zero_enables_slider: after "+ Layer", the old last
  // layer (now intermediate) with prob=0 must be ENABLED (user needs to fix
  // the dead-next-layer footgun). This exercises the (c) state.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_prob_footguns", "intermediate_layer_prob_zero_enables_slider");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      // Manually add a layer without going through "+ Layer" (which would auto-
      // promote layer[0].probability): construct a two-layer state where the
      // first layer still has prob=0. This is the state a hand-written config
      // could produce (footgun #2 from the CLI side).
      gui::Layer new_layer;
      gui::EntryCard e;
      e.crystal_id = 0;
      new_layer.entries.push_back(e);
      gui::g_state.layers.push_back(std::move(new_layer));
      gui::g_state.layers[0].probability = 0.0f;
      ctx->Yield(2);
      auto info = ctx->ItemInfo("**/##Prob.##layer_0_input");
      IM_CHECK((info.ItemFlags & ImGuiItemFlags_Disabled) == 0);
    };
  }

  // add_layer_promotes_prev_last_prob: "+ Layer" must promote the old last
  // layer's prob from 0 to kDefaultContinuationProb so the new layer receives
  // rays. Root fix for footgun #2.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_prob_footguns", "add_layer_promotes_prev_last_prob");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers.size()), 1);
      IM_CHECK_EQ(gui::g_state.layers[0].probability, 0.0f);

      ctx->ItemClick("**/+ Layer");
      ctx->Yield();

      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers.size()), 2);
      IM_CHECK_EQ(gui::g_state.layers[0].probability, gui::kDefaultContinuationProb);
      IM_CHECK(gui::IsProbZero(gui::g_state.layers[1].probability));  // new last stays 0
    };
  }

  // add_layer_promotes_near_zero_prev_last (R2 Major regression): slider drags
  // can leave layer.probability at a sub-step non-zero (e.g. 1e-7). Promotion
  // must use the shared IsProbZero epsilon — a strict == 0.0f would sneak this
  // past the promotion and reproduce footgun #2. This test pins that the
  // gui_state.hpp IsProbZero helper is actually used by "+ Layer".
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_prob_footguns", "add_layer_promotes_near_zero_prev_last");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      gui::g_state.layers[0].probability = 1e-7f;  // < kProbZeroEps
      ctx->Yield(2);

      ctx->ItemClick("**/+ Layer");
      ctx->Yield();

      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers.size()), 2);
      IM_CHECK_EQ(gui::g_state.layers[0].probability, gui::kDefaultContinuationProb);
    };
  }

  // add_layer_preserves_nonzero_prev_last: "+ Layer" must NOT overwrite a
  // previously non-zero last-layer prob (already fine — user set it on
  // purpose). Guards against over-eager promotion.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_prob_footguns", "add_layer_preserves_nonzero_prev_last");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      gui::g_state.layers[0].probability = 0.6f;
      ctx->Yield(2);

      ctx->ItemClick("**/+ Layer");
      ctx->Yield();

      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers.size()), 2);
      IM_CHECK_EQ(gui::g_state.layers[0].probability, 0.6f);  // preserved
    };
  }

  // add_delete_layer_last_falls_back_with_warning (Suggestion-1): "+ Layer"
  // promotes the old last to kDefaultContinuationProb, then deleting the new
  // last returns the old last to last-layer status — now with prob>0. Slider
  // must be ENABLED (state (b)); the reactive four-state logic in panels.cpp
  // must handle this without any delete-path special-casing.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_prob_footguns", "add_delete_layer_last_falls_back_with_warning");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      ctx->ItemClick("**/+ Layer");
      ctx->Yield();
      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers.size()), 2);
      IM_CHECK_EQ(gui::g_state.layers[0].probability, gui::kDefaultContinuationProb);

      // Delete layer 1 via the per-layer header "x" button. Layer 0 becomes
      // the last layer again — this time with prob=kDefaultContinuationProb>0.
      ctx->ItemClick("**/" ICON_FA_XMARK "##layer_1");
      ctx->Yield();
      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers.size()), 1);
      IM_CHECK_EQ(gui::g_state.layers[0].probability, gui::kDefaultContinuationProb);
      // Layer 0 is now last with prob>0 → slider must be ENABLED so the user
      // can fix it (state (b), not the disabled state (a)).
      auto info = ctx->ItemInfo("**/##Prob.##layer_0_input");
      IM_CHECK((info.ItemFlags & ImGuiItemFlags_Disabled) == 0);
    };
  }

  // intermediate_layer_prob_nonzero_normal (code-review Minor-4): the "normal"
  // fourth state — an intermediate (non-last) layer with prob>0 must have an
  // ENABLED slider and NO warning (neither the disabled last-layer-zero state
  // (a) nor a warning state (b)/(c)). Completes the four-state coverage.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_prob_footguns", "intermediate_layer_prob_nonzero_normal");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      // Two layers; layer 0 (intermediate) with prob>0 → state (d) normal.
      gui::Layer new_layer;
      gui::EntryCard e;
      e.crystal_id = 0;
      new_layer.entries.push_back(e);
      gui::g_state.layers.push_back(std::move(new_layer));
      gui::g_state.layers[0].probability = 0.5f;  // intermediate, non-zero
      ctx->Yield(2);
      // Slider enabled (state (d) normal: not the disabled last-layer-zero
      // state (a)). We assert the disabled dimension only — the warning icon is
      // a plain ImGui::TextColored with no queryable ID, so its absence cannot
      // be asserted via ItemInfo (would always report "not found" and give a
      // false pass). The show_warning_icon predicate is deterministic in
      // panels.cpp; states (b)/(c) exercise its enable branch elsewhere.
      auto info = ctx->ItemInfo("**/##Prob.##layer_0_input");
      IM_CHECK((info.ItemFlags & ImGuiItemFlags_Disabled) == 0);
    };
  }

  // p1_card/entry_copy — copy an entry card
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_card", "entry_copy");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Modify the default entry's crystal type
      gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id].type = gui::CrystalType::kPyramid;
      gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id].prism_h = 2.5f;
      ctx->Yield();

      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers[0].entries.size()), 1);

      // Duplicate the entry (hover button; ID is always addressable even at alpha=0).
      ctx->ItemClick("**/" ICON_FA_COPY "##dup_0_0");
      ctx->Yield();
      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers[0].entries.size()), 2);

      // Verify the copy has same crystal params
      auto& copy = gui::g_state.layers[0].entries[1];
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, copy).type, gui::CrystalType::kPyramid);
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, copy).prism_h, 2.5f);
    };
  }

  // p1_card/proportion_direct_write — modify entry proportion programmatically
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_card", "proportion_direct_write");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Default proportion is 100.0; change to 42.0
      IM_CHECK_EQ(gui::g_state.layers[0].entries[0].proportion, 100.0f);
      gui::g_state.layers[0].entries[0].proportion = 42.0f;
      gui::g_state.MarkDirty();
      ctx->Yield();

      IM_CHECK_EQ(gui::g_state.layers[0].entries[0].proportion, 42.0f);
      IM_CHECK_EQ(gui::g_state.dirty, true);
    };
  }

  // p1_card/card_wide_click_opens_modal — clicking card blank area opens the edit modal
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_card", "card_wide_click_opens_modal");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      IM_CHECK_EQ(gui::IsEditModalOpen(), false);

      // Derive a thumbnail click position from the Edit##cr button anchor.
      // The Edit button sits in the right column at thumb_display_size + text_w + 2*spacing from
      // card left (~200 px in the default test window). The thumbnail occupies the leftmost
      // ~88 px (4 * frame_height_with_spacing − spacing_y), so card_left + 30 stays in the
      // thumbnail and is clear of every ImGui item (IsAnyItemHovered == false).
      auto edit_info = ctx->ItemInfo("**/Edit##cr");
      IM_CHECK(edit_info.ID != 0);
      float card_left = edit_info.RectFull.Min.x - 200.0f;
      float card_top = edit_info.RectFull.Min.y;
      ImVec2 thumb_center(card_left + 30.0f, card_top + 20.0f);

      ctx->MouseMoveToPos(thumb_center);
      ctx->MouseClick(0);
      ctx->Yield(4);

      IM_CHECK_EQ(gui::IsEditModalOpen(), true);
      auto tgt = gui::GetEditModalTarget();
      IM_CHECK_EQ(tgt.layer_idx, 0);
      IM_CHECK_EQ(tgt.entry_idx, 0);

      // Cleanup: close modal
      ctx->ItemClick("**/" ICON_FA_XMARK " Cancel##edit_modal");
      ctx->Yield(2);
    };
  }

  // p1_card/card_click_blocked_when_covered_by_colors_window — AC1 regression
  // for task-color-window-mouse-capture: the Colors window is a floating
  // ImGui::Begin above the LeftPanel; when it covers a crystal card the manual
  // click detection in RenderEntryCard must not fire (issue.md AC1).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_card", "card_click_blocked_when_covered_by_colors_window");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      IM_CHECK_EQ(gui::IsEditModalOpen(), false);

      // Reuse the anchor logic from card_wide_click_opens_modal so both tests
      // move together if the LeftPanel layout shifts.
      auto edit_info = ctx->ItemInfo("**/Edit##cr");
      IM_CHECK(edit_info.ID != 0);
      float card_left = edit_info.RectFull.Min.x - 200.0f;
      float card_top = edit_info.RectFull.Min.y;
      ImVec2 thumb_center(card_left + 30.0f, card_top + 20.0f);

      // Open the Colors window and move it so its client area covers
      // thumb_center. The default size is 720x480 (color_window.cpp:505); by
      // anchoring the top-left ~300px above and ~300px left of thumb_center,
      // thumb_center lands ~mid-window, well past the header / composite-mode
      // combo / Add Class button row / separator (~70px from top).
      // raypath_color is empty after DoNew(), so no color-class row extends
      // down into that region — the click point falls on the empty class table
      // area, which contains no ImGui item.
      gui::g_state.color_window_open = true;
      ctx->Yield(2);

      ImVec2 colors_pos(thumb_center.x - 300.0f, thumb_center.y - 240.0f);
      ctx->WindowMove(ICON_FA_PALETTE " Colors", colors_pos);
      ctx->Yield(2);

      ctx->MouseMoveToPos(thumb_center);
      ctx->MouseClick(0);
      ctx->Yield(4);

      // Core AC1 assertion: the covered click must NOT open the edit modal.
      IM_CHECK_EQ(gui::IsEditModalOpen(), false);

      // Cleanup
      gui::g_state.color_window_open = false;
      ctx->Yield(2);
    };
  }

  // p1_card/card_wide_click_noop_same_card — card click on already-open card is no-op
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_card", "card_wide_click_noop_same_card");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Open modal on entry 0 via Edit button
      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(4);
      IM_CHECK_EQ(gui::IsEditModalOpen(), true);
      auto tgt_before = gui::GetEditModalTarget();
      IM_CHECK_EQ(tgt_before.layer_idx, 0);
      IM_CHECK_EQ(tgt_before.entry_idx, 0);

      // Set the Height field through the modal UI — the buffer now differs from g_state.
      // If a spurious OpenEditModal call resets the buffer, the field reverts to g_state.height.
      const float orig_h = gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id].height.center;
      ctx->ItemInputValue("**/##Height##modal_cr_input", orig_h + 33.0f);
      ctx->Yield();

      // Click the thumbnail area of the same card (same anchor logic as card_wide_click_opens_modal)
      auto edit_info = ctx->ItemInfo("**/Edit##cr");
      IM_CHECK(edit_info.ID != 0);
      float card_left = edit_info.RectFull.Min.x - 200.0f;
      float card_top = edit_info.RectFull.Min.y;
      ImVec2 thumb_center(card_left + 30.0f, card_top + 20.0f);

      ctx->MouseMoveToPos(thumb_center);
      ctx->MouseClick(0);
      ctx->Yield(4);

      // Modal still open on same card — no-op
      IM_CHECK_EQ(gui::IsEditModalOpen(), true);
      auto tgt_after = gui::GetEditModalTarget();
      IM_CHECK_EQ(tgt_after.layer_idx, 0);
      IM_CHECK_EQ(tgt_after.entry_idx, 0);

      // Commit buffer via OK: if no-op held, g_state.height becomes orig_h + 33.0f;
      // if OpenEditModal was spuriously called, buffer was reset to orig_h.
      ctx->ItemClick("**/" ICON_FA_CHECK " OK##edit_modal");
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id].height, orig_h + 33.0f);
    };
  }
}

// ========== task-test-gui-interaction: P1 Slider boundary tests ==========


void RegisterP1SliderBoundaryTests(ImGuiTestEngine* engine) {
  // p1_slider/ray_count_min_max — linear scale, exact boundary values expected.
  // SliderWithInput("Rays(M)", ...) now lives in RenderSceneControls (right panel Scene group).
  // Internal widget ID is "##Rays(M)_input" via PrepareSliderLayout.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_slider", "ray_count_min_max");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      // Scene is now a CollapsingHeader in the right panel, open by default
      ctx->Yield(2);

      IM_CHECK_EQ(gui::g_state.sim.infinite, false);  // slider active when not infinite

      ctx->ItemInputValue("**/##Rays(M)_input", 0.1f);
      ctx->Yield();
      IM_CHECK_EQ(gui::g_state.sim.ray_num_millions, 0.1f);

      ctx->ItemInputValue("**/##Rays(M)_input", 100.0f);
      ctx->Yield();
      IM_CHECK_EQ(gui::g_state.sim.ray_num_millions, 100.0f);
    };
  }

  // p1_slider/ray_count_infinite_toggle — toggle preserves slider value across on/off
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_slider", "ray_count_infinite_toggle");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      // Scene is now a CollapsingHeader in the right panel, open by default
      ctx->Yield(2);

      ctx->ItemInputValue("**/##Rays(M)_input", 5.0f);
      ctx->Yield();
      IM_CHECK_EQ(gui::g_state.sim.ray_num_millions, 5.0f);

      ctx->ItemClick("**/Infinite rays");
      ctx->Yield();
      IM_CHECK_EQ(gui::g_state.sim.infinite, true);

      ctx->ItemClick("**/Infinite rays");
      ctx->Yield();
      IM_CHECK_EQ(gui::g_state.sim.infinite, false);
      // Slider value preserved across toggle
      IM_CHECK_EQ(gui::g_state.sim.ray_num_millions, 5.0f);
    };
  }

  // Crystal slider boundary tests — via SliderWithInput in Edit Crystal modal.
  // Open modal, use ctx->ItemInputValue to write boundary values through the widget,
  // click OK, then verify the clamped values in g_state.
  //
  // Three-H-mapping conventions (see gui/slider_mapping.hpp):
  //   Prism Height: [0.01, 100] kLog
  //   Pyramid prism_h: [0, 100] kLogLinear
  //   Pyramid upper_h / lower_h: [0, 1] kLinear
  //   Face Distance: [0, 2] kLinear

  // p1_slider/height_clamp_via_modal — kLog clamps height to [0.01, 100]
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_slider", "height_clamp_via_modal");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      IM_CHECK_EQ(gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id].type, gui::CrystalType::kPrism);

      // Open crystal modal
      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(3);

      // Write 0 via the input widget — should be clamped to min (0.01)
      ctx->ItemInputValue("**/##Height##modal_cr_input", 0.0f);
      ctx->Yield();

      // Click OK to commit
      ctx->ItemClick("**/" ICON_FA_CHECK " OK##edit_modal");
      ctx->Yield(2);

      // Height should be clamped to >= 0.01 (kLog with min=0.01)
      IM_CHECK_GE(gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id].height.center, 0.01f - 1e-6f);
    };
  }

  // p1_slider/pyramid_h_allows_zero_via_modal — Upper H and Lower H allow 0
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_slider", "pyramid_h_allows_zero_via_modal");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Switch to Pyramid type programmatically
      gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id].type = gui::CrystalType::kPyramid;
      ctx->Yield();

      // Open crystal modal
      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(3);

      // Write 0 to Upper H — should be allowed (min=0.0 for kLinear)
      ctx->ItemInputValue("**/##Upper H##modal_cr_input", 0.0f);
      ctx->Yield();

      // Click OK
      ctx->ItemClick("**/" ICON_FA_CHECK " OK##edit_modal");
      ctx->Yield(2);

      IM_CHECK_EQ(gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id].upper_h, 0.0f);
    };
  }

  // p1_slider/pyramid_h_clamp_upper_via_modal — Upper H clamped at new [0,1] upper bound
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_slider", "pyramid_h_clamp_upper_via_modal");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id].type = gui::CrystalType::kPyramid;
      ctx->Yield();

      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(3);

      // Write 1.5 — should be clamped to 1.0 (kLinear with max=1.0)
      ctx->ItemInputValue("**/##Upper H##modal_cr_input", 1.5f);
      ctx->Yield();

      ctx->ItemClick("**/" ICON_FA_CHECK " OK##edit_modal");
      ctx->Yield(2);

      IM_CHECK_EQ(gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id].upper_h, 1.0f);
    };
  }

  // p1_slider/pyramid_h_linear_readback_via_modal — input-box linear identity
  // within the declared [0, 1] range. This goes through the _input field path,
  // which only applies std::clamp; it does NOT cover the slider-drag nonlinear
  // mapping path. Scale-configuration regressions (e.g. reverting to kLogLinear)
  // are covered by pyramid_h_clamp_upper_via_modal via the max-bound assertion.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_slider", "pyramid_h_linear_readback_via_modal");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id].type = gui::CrystalType::kPyramid;
      ctx->Yield();

      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(3);

      ctx->ItemInputValue("**/##Upper H##modal_cr_input", 0.5f);
      ctx->Yield();

      ctx->ItemClick("**/" ICON_FA_CHECK " OK##edit_modal");
      ctx->Yield(2);

      IM_CHECK_EQ(gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id].upper_h, 0.5f);
    };
  }

  // p1_slider/altitude_negative_boundaries — sun altitude ±90°
  // SliderWithInput("Altitude", ...) now in RenderSceneControls (right panel Scene group)
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_slider", "altitude_negative_boundaries");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      // Scene is now a CollapsingHeader in the right panel, open by default
      ctx->Yield(2);

      ctx->ItemInputValue("**/##Altitude_input", -90.0f);
      ctx->Yield();
      IM_CHECK_EQ(gui::g_state.sun.altitude, -90.0f);

      ctx->ItemInputValue("**/##Altitude_input", 90.0f);
      ctx->Yield();
      IM_CHECK_EQ(gui::g_state.sun.altitude, 90.0f);
    };
  }

  // p1_slider/altitude_edit_after_commit_marks_dirty — AC2 end-to-end regression
  // (scrum-gui-state-reconcile T0, plan §4 Step 4 point 3 / §7 risk 1). Drives the real Altitude
  // widget after a real DoRun() commit has populated last_committed_state, then asserts dirty
  // becomes true via the production frame-tail ReconcileGuiEffects() path — the legacy DIRTY_IF
  // wrapper was retired at this call site (panels.cpp RenderSceneControls), so this is the only
  // test that exercises sun.altitude's post-migration dirty derivation end-to-end rather than by
  // constructing a GuiState by hand (see test_gui_state_reconcile.cpp for the pure variant).
  //
  // M6 same-frame contract: the effects reconcile runs at frame TAIL (after all Render*() calls,
  // before ImGui::Render()) in both prod (main.cpp) and this harness (test_gui_main.cpp). That
  // means the LAST frame of an ItemInputValue interaction — which writes state.sun.altitude
  // during widget rendering — reaches its own frame-tail reconcile before returning to us. So we
  // assert dirty immediately, with no intervening ctx->Yield(). The absence of that Yield is the
  // structural evidence that dirty was set in the SAME frame as the edit (Round 3 Major #1 fix).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_slider", "altitude_edit_after_commit_marks_dirty");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      gui::g_server = LUMICE_CreateServer();
      IM_CHECK(gui::g_server != nullptr);
      gui::g_state.sim.infinite = false;
      gui::g_state.sim.ray_num_millions = 0.5f;
      gui::DoRun(/*user_initiated=*/true);  // real Run path: synchronous commit populates last_committed_state.
      IM_CHECK(gui::g_state.last_committed_state.has_value());
      gui::g_state.dirty = false;  // DoRun doesn't touch dirty; pin a known pre-edit baseline.
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.dirty, false);

      ctx->ItemInputValue("**/##Altitude_input", gui::g_state.sun.altitude + 5.0f);
      // Deliberately NO Yield() here — see block comment above: with the M6 frame-tail placement,
      // dirty must be true by the end of the frame that received the widget edit. An intervening
      // Yield would silently accept next-frame semantics and let a future regression to top-of-
      // frame reconcile placement slip through.
      IM_CHECK(gui::g_state.dirty);

      // Cleanup: leave a clean global state for subsequent tests.
      gui::g_server_poller.Stop();
      LUMICE_StopServer(gui::g_server);
      LUMICE_DestroyServer(gui::g_server);
      gui::g_server = nullptr;
      gui::g_state.run_intent = gui::RunIntent::kNone;
      gui::g_state.committed_epoch = 0;
      gui::g_state.dirty = false;
    };
  }

  // p1_slider/altitude_out_of_range_no_dirty — out-of-range input at upper boundary does not trigger dirty
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_slider", "altitude_out_of_range_no_dirty");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      gui::g_state.sun.altitude = 90.0f;
      gui::g_state.dirty = false;
      ctx->Yield(2);

      ctx->ItemInputValue("**/##Altitude_input", 200.0f);
      ctx->Yield();
      IM_CHECK_EQ(gui::g_state.sun.altitude, 90.0f);
      IM_CHECK_EQ(gui::g_state.dirty, false);
    };
  }

  // p1_slider/altitude_out_of_range_negative_no_dirty — out-of-range negative input at lower boundary
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_slider", "altitude_out_of_range_negative_no_dirty");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      gui::g_state.sun.altitude = -90.0f;
      gui::g_state.dirty = false;
      ctx->Yield(2);

      ctx->ItemInputValue("**/##Altitude_input", -200.0f);
      ctx->Yield();
      IM_CHECK_EQ(gui::g_state.sun.altitude, -90.0f);
      IM_CHECK_EQ(gui::g_state.dirty, false);
    };
  }
}

// ========== task-test-gui-interaction: P2 Render (lens switch / overlay) ==========


void RegisterP2InteractionRenderTests(ImGuiTestEngine* engine) {
  // p2_render/lens_switch_fov_clamp — switching lens type clamps fov to new lens's MaxFov
  // See app_panels.cpp:610-611: fov = std::min(fov, MaxFov(lens_type)) applied per frame.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_render", "lens_switch_fov_clamp");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Set up a full-sky lens (lens_type=4 Dual Fisheye EA) with max fov 360
      gui::g_state.renderer.lens_type = 4;
      gui::g_state.renderer.fov = 360.0f;
      ctx->Yield(3);
      IM_CHECK_EQ(gui::g_state.renderer.fov, 360.0f);

      // Switch to Linear (lens_type=0) — its MaxFov is typically much smaller
      gui::g_state.renderer.lens_type = 0;
      ctx->Yield(3);

      // fov should be clamped to Linear's max (some value < 360)
      IM_CHECK_LT(gui::g_state.renderer.fov, 360.0f);
    };
  }

  // p2_render/lens_orthographic_selection — selecting Orthographic drives FOV max to 180.
  // See task-lens-orthographic scrum-gui-polish-v15 for context.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_render", "lens_orthographic_selection");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Start with Dual Fisheye EA (fov=360) to prove the subsequent clamp is real.
      gui::g_state.renderer.lens_type = gui::kLensTypeDualFisheyeEqualArea;
      gui::g_state.renderer.fov = 360.0f;
      ctx->Yield(3);
      IM_CHECK_EQ(gui::g_state.renderer.fov, 360.0f);

      // Switch to Fisheye Orthographic (MaxFov=180). fov must drop to 180.
      gui::g_state.renderer.lens_type = gui::kLensTypeFisheyeOrthographic;
      ctx->Yield(3);
      IM_CHECK_LE(gui::g_state.renderer.fov, 180.0f);
      IM_CHECK_GT(gui::g_state.renderer.fov, 0.0f);

      // Push fov past the cap; the per-frame clamp pulls it back to 180.
      gui::g_state.renderer.fov = 200.0f;
      ctx->Yield(3);
      IM_CHECK_LE(gui::g_state.renderer.fov, 180.0f);
    };
  }

  // p2_render/lens_dual_orthographic_selection — dual orthographic also clamps to 180.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_render", "lens_dual_orthographic_selection");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      gui::g_state.renderer.lens_type = gui::kLensTypeDualFisheyeEqualArea;
      gui::g_state.renderer.fov = 360.0f;
      ctx->Yield(3);

      gui::g_state.renderer.lens_type = gui::kLensTypeDualFisheyeOrthographic;
      ctx->Yield(3);
      IM_CHECK_LE(gui::g_state.renderer.fov, 180.0f);
      IM_CHECK_GT(gui::g_state.renderer.fov, 0.0f);

      gui::g_state.renderer.fov = 250.0f;
      ctx->Yield(3);
      IM_CHECK_LE(gui::g_state.renderer.fov, 180.0f);
    };
  }

  // p2_render/lens_orthographic_view_controls — single orthographic must allow
  // elevation/azimuth/roll edits and NOT be force-zeroed (issue subpoint 2),
  // and FOV slider must remain effective up to its 180° clamp (issue subpoint 3).
  // Regression guard for the LensIsFullSky() refactor (task-orthographic-followup
  // Step 1): the previous `lens_type >= 4` heuristic incorrectly flagged single
  // orthographic (lens=8) as full-sky, disabling all view controls.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_render", "lens_orthographic_view_controls");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      gui::g_state.renderer.lens_type = gui::kLensTypeFisheyeOrthographic;
      gui::g_state.renderer.fov = 120.0f;
      gui::g_state.renderer.elevation = 30.0f;
      gui::g_state.renderer.azimuth = 45.0f;
      gui::g_state.renderer.roll = 10.0f;
      ctx->Yield(3);

      // Single orthographic must NOT be force-zeroed by the per-frame full-sky guard.
      IM_CHECK_EQ(gui::g_state.renderer.elevation, 30.0f);
      IM_CHECK_EQ(gui::g_state.renderer.azimuth, 45.0f);
      IM_CHECK_EQ(gui::g_state.renderer.roll, 10.0f);

      // FOV slider must accept values up to 180° without being clamped lower.
      gui::g_state.renderer.fov = 170.0f;
      ctx->Yield(3);
      IM_CHECK_EQ(gui::g_state.renderer.fov, 170.0f);

      // Pushing past 180° is clamped (already covered by lens_orthographic_selection,
      // re-asserted here to keep the "FOV adjustable + clamped" invariant explicit).
      gui::g_state.renderer.fov = 220.0f;
      ctx->Yield(3);
      IM_CHECK_LE(gui::g_state.renderer.fov, 180.0f);
    };
  }

  // p2_render/lens_full_sky_view_controls_disabled — guards every entry in
  // gui::kFullSkyLensTypes from accidentally gaining view controls. The loop
  // iterates the SSOT array directly (not a local copy) so any future
  // addition / removal in kFullSkyLensTypes automatically updates the test
  // coverage, closing the loop with the static_assert in gui_constants.hpp.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_render", "lens_full_sky_view_controls_disabled");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      for (int lens : gui::kFullSkyLensTypes) {
        ResetTestState();
        ctx->Yield(2);

        gui::g_state.renderer.lens_type = lens;
        gui::g_state.renderer.elevation = 30.0f;
        gui::g_state.renderer.azimuth = 45.0f;
        gui::g_state.renderer.roll = 10.0f;
        ctx->Yield(3);

        // Per-frame full-sky guard must zero these for every lens in the set.
        IM_CHECK_EQ(gui::g_state.renderer.elevation, 0.0f);
        IM_CHECK_EQ(gui::g_state.renderer.azimuth, 0.0f);
        IM_CHECK_EQ(gui::g_state.renderer.roll, 0.0f);
      }
    };
  }

  // ===== task-tests-and-baseline (scrum-outside-in-globe-view 173.4) =====
  // Globe lens (lens=10) GUI interaction coverage. Verifies the lens-type
  // selection / per-lens view controls / trackball drag handler / unified
  // Reset button / cross-lens roll preservation, all of which were exercised
  // only by manual GUI in scrum 173.3 due to headless-runner limits.

  // p2_render/lens_globe_selection — selecting Globe drives FOV max to 90
  // (MaxFov(kGlobe) = 90, src/config/render_config.cpp:138).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_render", "lens_globe_selection");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Start with Dual Fisheye EA (fov=360) to prove the subsequent clamp is real.
      gui::g_state.renderer.lens_type = gui::kLensTypeDualFisheyeEqualArea;
      gui::g_state.renderer.fov = 360.0f;
      ctx->Yield(3);

      // Switch to Globe (MaxFov=90). The per-frame clamp pulls fov to 90.
      gui::g_state.renderer.lens_type = gui::kLensTypeGlobe;
      ctx->Yield(3);
      IM_CHECK_EQ(gui::g_state.renderer.fov, 90.0f);

      // Push fov past the cap; the per-frame clamp pulls it back to <= 90.
      gui::g_state.renderer.fov = 200.0f;
      ctx->Yield(3);
      IM_CHECK_LE(gui::g_state.renderer.fov, 90.0f);
    };
  }

  // p2_render/lens_globe_view_controls — Globe is NOT in kFullSkyLensTypes,
  // so elevation/azimuth must NOT be force-zeroed, the stored roll value
  // must be preserved (Globe disables the roll slider via BeginDisabled
  // without writing back), and the FOV cap (90) must hold.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_render", "lens_globe_view_controls");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      gui::g_state.renderer.lens_type = gui::kLensTypeGlobe;
      gui::g_state.renderer.fov = 60.0f;
      gui::g_state.renderer.elevation = 30.0f;
      gui::g_state.renderer.azimuth = 45.0f;
      // Guard: Globe activation must NOT reset stored roll field
      // (EffectiveRollForLens reads it without writing back).
      gui::g_state.renderer.roll = 10.0f;
      ctx->Yield(3);

      IM_CHECK_EQ(gui::g_state.renderer.elevation, 30.0f);
      IM_CHECK_EQ(gui::g_state.renderer.azimuth, 45.0f);
      IM_CHECK_EQ(gui::g_state.renderer.roll, 10.0f);

      // FOV slider in Globe mode caps at 90.
      gui::g_state.renderer.fov = 80.0f;
      ctx->Yield(3);
      IM_CHECK_EQ(gui::g_state.renderer.fov, 80.0f);

      gui::g_state.renderer.fov = 120.0f;
      ctx->Yield(3);
      IM_CHECK_LE(gui::g_state.renderer.fov, 90.0f);
    };
  }

  // p2_render/lens_*trackball — Path A real-drag coverage upgrade (scrum 175.2).
  // Five contract tests over the drag handler at app_panels.cpp:776-793:
  //   T1 lens_globe_trackball              — Globe az direction (+dx → +az)
  //   T2 lens_fisheye_trackball            — non-Globe az direction (+dx → -az)
  //   T3 lens_globe_trackball_az_wrap      — az wrap across ±180°
  //   T4 lens_globe_trackball_el_clamp     — Globe el clamp at +89°
  //   T5 lens_fisheye_trackball_el_clamp   — non-Globe el clamp at +90°
  //
  // The main-viewport "##preview_interact" InvisibleButton is rendered only
  // when g_preview.HasTexture() || g_preview.HasBackground() is true (see
  // app_panels.cpp:676). The shared GuiFunc below uploads a synth texture so
  // the button becomes addressable; this mirrors the BgOverlayGuiFunc pattern
  // in test_gui_bg.cpp:8-14. The local static upload-done flag isolates this
  // group from g_export_test, avoiding cross-file shared-state coupling.
  static bool s_trackball_upload_done = false;
  auto trackball_gui_func = [](ImGuiTestContext* /*ctx*/) {
    if (!s_trackball_upload_done) {
      InitSynthTexture();
      gui::g_preview.UploadTexture(g_synth_tex.data(), kSynthTexW, kSynthTexH);
      s_trackball_upload_done = true;
    }
  };

  // T1: Globe drag direction. dx=+60 px @ 0.3 sensitivity → az ≈ +18°.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_render", "lens_globe_trackball");
    t->GuiFunc = trackball_gui_func;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      s_trackball_upload_done = false;
      ctx->Yield(3);
      IM_CHECK(gui::g_preview.HasTexture());

      gui::g_state.renderer.lens_type = gui::kLensTypeGlobe;
      gui::g_state.renderer.fov = 60.0f;
      gui::g_state.renderer.azimuth = 0.0f;
      gui::g_state.renderer.elevation = 0.0f;
      gui::g_state.renderer.roll = 0.0f;
      ctx->Yield(2);

      ctx->ItemDragWithDelta("**/##preview_interact", ImVec2(60.0f, 0.0f));
      ctx->Yield(2);

      IM_CHECK_GT(gui::g_state.renderer.azimuth, 17.0f);
      IM_CHECK_LT(gui::g_state.renderer.azimuth, 19.0f);
      IM_CHECK_LT(std::fabs(gui::g_state.renderer.elevation), 0.5f);
      IM_CHECK_LT(std::fabs(gui::g_state.renderer.roll), 0.001f);
    };
  }

  // T2: non-Globe drag direction is opposite (sign flip in app_panels.cpp:780).
  // dx=+60 px → az ≈ -18° on FisheyeEquidist.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_render", "lens_fisheye_trackball");
    t->GuiFunc = trackball_gui_func;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      s_trackball_upload_done = false;
      ctx->Yield(3);
      IM_CHECK(gui::g_preview.HasTexture());

      gui::g_state.renderer.lens_type = gui::kLensTypeFisheyeEquidist;
      gui::g_state.renderer.fov = 60.0f;
      gui::g_state.renderer.azimuth = 0.0f;
      gui::g_state.renderer.elevation = 0.0f;
      gui::g_state.renderer.roll = 0.0f;
      ctx->Yield(2);

      ctx->ItemDragWithDelta("**/##preview_interact", ImVec2(60.0f, 0.0f));
      ctx->Yield(2);

      IM_CHECK_GT(gui::g_state.renderer.azimuth, -19.0f);
      IM_CHECK_LT(gui::g_state.renderer.azimuth, -17.0f);
      IM_CHECK_LT(std::fabs(gui::g_state.renderer.elevation), 0.5f);
      IM_CHECK_LT(std::fabs(gui::g_state.renderer.roll), 0.001f);
    };
  }

  // T3: az wrap across +180°. Start az=170, drag dx=+200 → 170 + 60 = 230 →
  // wrap by -360 → -130. Validates app_panels.cpp:783-789.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_render", "lens_globe_trackball_az_wrap");
    t->GuiFunc = trackball_gui_func;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      s_trackball_upload_done = false;
      ctx->Yield(3);
      IM_CHECK(gui::g_preview.HasTexture());

      gui::g_state.renderer.lens_type = gui::kLensTypeGlobe;
      gui::g_state.renderer.fov = 60.0f;
      gui::g_state.renderer.azimuth = 170.0f;
      gui::g_state.renderer.elevation = 0.0f;
      gui::g_state.renderer.roll = 0.0f;
      ctx->Yield(2);

      ctx->ItemDragWithDelta("**/##preview_interact", ImVec2(200.0f, 0.0f));
      ctx->Yield(2);

      IM_CHECK_GT(gui::g_state.renderer.azimuth, -131.0f);
      IM_CHECK_LT(gui::g_state.renderer.azimuth, -129.0f);
    };
  }

  // T4: Globe el clamps at ±89° (tighter than non-Globe ±90°). Globe formula
  // is `el -= dy * 0.3` (app_panels.cpp:778), so dy=-1000 drives +300 toward
  // the upper limit — far past +89, which std::min collapses to exactly 89.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_render", "lens_globe_trackball_el_clamp");
    t->GuiFunc = trackball_gui_func;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      s_trackball_upload_done = false;
      ctx->Yield(3);
      IM_CHECK(gui::g_preview.HasTexture());

      gui::g_state.renderer.lens_type = gui::kLensTypeGlobe;
      gui::g_state.renderer.fov = 60.0f;
      gui::g_state.renderer.azimuth = 0.0f;
      gui::g_state.renderer.elevation = 0.0f;
      gui::g_state.renderer.roll = 0.0f;
      ctx->Yield(2);

      ctx->ItemDragWithDelta("**/##preview_interact", ImVec2(0.0f, -1000.0f));
      ctx->Yield(2);

      IM_CHECK_GE(gui::g_state.renderer.elevation, 89.0f);
      IM_CHECK_LT(gui::g_state.renderer.elevation, 89.5f);
    };
  }

  // T5: non-Globe el clamps at ±90° (looser limit branch in app_panels.cpp:792).
  // Non-Globe formula is `el += dy * 0.3` (app_panels.cpp:781), so dy=+1000
  // drives +300 toward +90.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_render", "lens_fisheye_trackball_el_clamp");
    t->GuiFunc = trackball_gui_func;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      s_trackball_upload_done = false;
      ctx->Yield(3);
      IM_CHECK(gui::g_preview.HasTexture());

      gui::g_state.renderer.lens_type = gui::kLensTypeFisheyeEquidist;
      gui::g_state.renderer.fov = 60.0f;
      gui::g_state.renderer.azimuth = 0.0f;
      gui::g_state.renderer.elevation = 0.0f;
      gui::g_state.renderer.roll = 0.0f;
      ctx->Yield(2);

      ctx->ItemDragWithDelta("**/##preview_interact", ImVec2(0.0f, 1000.0f));
      ctx->Yield(2);

      IM_CHECK_GE(gui::g_state.renderer.elevation, 90.0f);
      IM_CHECK_LT(gui::g_state.renderer.elevation, 90.5f);
    };
  }

  // p2_render/view_reset_button — the unified View `Reset` button (added in
  // scrum 173.3) must restore all four resettable fields to
  // DefaultViewParamsFor(lens) for ANY current lens, while leaving lens_type
  // and visible untouched. Two lenses cover both the 180° fov default
  // (fisheye_equidist) and the Globe-specific 30° default + roll branch.
  // Per issue.md AC: "至少 2 个 lens" — exhaustive enumeration is unnecessary
  // since DefaultViewParamsFor coverage is already established at the unit
  // level + reviewed in scrum 173.3.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_render", "view_reset_button");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      // Sanity: the Reset button must be addressable on the default lens.
      IM_CHECK(ctx->ItemExists("**/Reset##view"));

      int lens_set[] = { gui::kLensTypeFisheyeEquidist, gui::kLensTypeGlobe };
      for (int lens : lens_set) {
        ResetTestState();
        ctx->Yield(2);

        gui::g_state.renderer.lens_type = lens;
        ctx->Yield(3);
        // Capture visible AFTER the lens-switch frame settles, so the snapshot
        // reflects any per-frame normalization the new lens may apply.
        int visible_before = gui::g_state.renderer.visible;

        // User edits all four resettable fields.
        gui::g_state.renderer.fov = 42.0f;
        gui::g_state.renderer.elevation = 33.0f;
        gui::g_state.renderer.azimuth = -77.0f;
        gui::g_state.renderer.roll = 11.0f;
        ctx->Yield(3);

        ctx->ItemClick("**/Reset##view");
        ctx->Yield(3);

        gui::ViewDefaults def = gui::DefaultViewParamsFor(lens);
        IM_CHECK_EQ(gui::g_state.renderer.fov, def.fov);
        IM_CHECK_EQ(gui::g_state.renderer.elevation, def.elevation);
        IM_CHECK_EQ(gui::g_state.renderer.azimuth, def.azimuth);
        IM_CHECK_EQ(gui::g_state.renderer.roll, def.roll);
        // Reset must NOT alter lens_type or the visibility mode.
        IM_CHECK_EQ(gui::g_state.renderer.lens_type, lens);
        IM_CHECK_EQ(gui::g_state.renderer.visible, visible_before);
      }
    };
  }

  // p2_render/lens_globe_roll_preserved_on_lens_switch — switching to Globe
  // and back must preserve the stored RenderConfig.roll value: Globe forces
  // roll=0 only at render time via EffectiveRollForLens (gui_state.hpp:199),
  // never writing back to the stored field. Regression guard for the
  // 173.3 SUMMARY §3 design contract.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_render", "lens_globe_roll_preserved_on_lens_switch");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      gui::g_state.renderer.lens_type = gui::kLensTypeFisheyeEquidist;
      gui::g_state.renderer.roll = 15.0f;
      ctx->Yield(3);
      IM_CHECK_EQ(gui::g_state.renderer.roll, 15.0f);

      // Switch to Globe — stored roll must remain 15° (display path masks it).
      gui::g_state.renderer.lens_type = gui::kLensTypeGlobe;
      ctx->Yield(3);
      IM_CHECK_EQ(gui::g_state.renderer.roll, 15.0f);

      // Switch back — stored roll still 15°.
      gui::g_state.renderer.lens_type = gui::kLensTypeFisheyeEquidist;
      ctx->Yield(3);
      IM_CHECK_EQ(gui::g_state.renderer.roll, 15.0f);
    };
  }

  // p2_render/lens_globe_switch_transform_other_to_globe — switching to Globe via
  // UI combo applies az+180 / el-negate transform (supersedes ba841e4 az-reset).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_render", "lens_globe_switch_transform_other_to_globe");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      gui::g_state.renderer.lens_type = gui::kLensTypeFisheyeEquidist;
      gui::g_state.renderer.azimuth = 90.0f;
      gui::g_state.renderer.elevation = 20.0f;
      ctx->Yield(3);

      // BeginCombo doesn't call IMGUI_TEST_ENGINE_ITEM_INFO so wildcard search fails for
      // the combo button; use SetRef + ComboClick which finds the button by ID and the
      // popup item via a window-scoped wildcard that scrolls to reveal clipped items.
      // Globe is item 11/11 in kLensTypePresentationOrder; the default popup height is 8,
      // so Globe is initially clipped and only reachable after popup scroll.
      ctx->SetRef("//##RightPanel");
      ctx->ComboClick("Lens Type##view/Globe");
      ctx->SetRef("");
      ctx->Yield(3);

      IM_CHECK_EQ(gui::g_state.renderer.azimuth, -90.0f);
      IM_CHECK_EQ(gui::g_state.renderer.elevation, -20.0f);
    };
  }

  // p2_render/lens_globe_switch_transform_globe_to_other — switching from Globe via
  // UI combo applies the same self-inverse transform (az+180 / el-negate).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_render", "lens_globe_switch_transform_globe_to_other");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      gui::g_state.renderer.lens_type = gui::kLensTypeGlobe;
      gui::g_state.renderer.azimuth = -90.0f;
      gui::g_state.renderer.elevation = -20.0f;
      ctx->Yield(3);

      ctx->SetRef("//##RightPanel");
      ctx->ComboClick("Lens Type##view/Fisheye Equidistant");
      ctx->SetRef("");
      ctx->Yield(3);

      IM_CHECK_EQ(gui::g_state.renderer.azimuth, 90.0f);
      IM_CHECK_EQ(gui::g_state.renderer.elevation, 20.0f);
    };
  }

  // p2_render/lens_globe_switch_transform_az_zero_boundary — az=0 wraps to 180
  // (not -180): the condition is "> 180.0f" (strict), so 0+180=180 is not reduced.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_render", "lens_globe_switch_transform_az_zero_boundary");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      gui::g_state.renderer.lens_type = gui::kLensTypeFisheyeEquidist;
      gui::g_state.renderer.azimuth = 0.0f;
      gui::g_state.renderer.elevation = 0.0f;
      ctx->Yield(3);

      ctx->SetRef("//##RightPanel");
      ctx->ComboClick("Lens Type##view/Globe");
      ctx->SetRef("");
      ctx->Yield(3);

      IM_CHECK_EQ(gui::g_state.renderer.azimuth, 180.0f);
      IM_CHECK_EQ(gui::g_state.renderer.elevation, 0.0f);
    };
  }

  // p2_render/lens_globe_switch_transform_az180_wrap — az=180 switching to Globe
  // applies az+180=360, which is > 180 and wraps to 0 (not staying at 360).
  // Regression guard for the strict "> 180.0f" wrap condition.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_render", "lens_globe_switch_transform_az180_wrap");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      gui::g_state.renderer.lens_type = gui::kLensTypeFisheyeEquidist;
      gui::g_state.renderer.azimuth = 180.0f;
      gui::g_state.renderer.elevation = 0.0f;
      ctx->Yield(3);

      ctx->SetRef("//##RightPanel");
      ctx->ComboClick("Lens Type##view/Globe");
      ctx->SetRef("");
      ctx->Yield(3);

      IM_CHECK_EQ(gui::g_state.renderer.azimuth, 0.0f);
      IM_CHECK_EQ(gui::g_state.renderer.elevation, 0.0f);
    };
  }

  // p2_render/lens_globe_switch_transform_el_clamp — el=91° switching to Globe
  // negates to -91°, then clamps to -89° (Globe el limit).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_render", "lens_globe_switch_transform_el_clamp");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      gui::g_state.renderer.lens_type = gui::kLensTypeFisheyeEquidist;
      gui::g_state.renderer.azimuth = 0.0f;
      gui::g_state.renderer.elevation = 91.0f;
      ctx->Yield(3);

      ctx->SetRef("//##RightPanel");
      ctx->ComboClick("Lens Type##view/Globe");
      ctx->SetRef("");
      ctx->Yield(3);

      IM_CHECK_EQ(gui::g_state.renderer.azimuth, 180.0f);
      IM_CHECK_EQ(gui::g_state.renderer.elevation, -89.0f);
    };
  }

  // p2_render/modal_layout_toggle_bit — switching modal_layout_vertical is safe
  // (view preference state-level test; layout dispatch is exercised only indirectly
  // through subsequent modal open, which would require a live popup — omitted to
  // keep the test fast and deterministic; plan Minor #5 noted the limitation).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_render", "modal_layout_toggle_bit");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      // ResetTestState pins modal_layout_vertical to legacy false; production
      // default is true (V) since gui-polish-v15 round 2.
      IM_CHECK_EQ(gui::g_state.modal_layout_vertical, false);

      gui::g_state.modal_layout_vertical = true;
      ctx->Yield(3);
      IM_CHECK_EQ(gui::g_state.modal_layout_vertical, true);

      gui::g_state.modal_layout_vertical = false;
      ctx->Yield(3);
      IM_CHECK_EQ(gui::g_state.modal_layout_vertical, false);
    };
  }

  // p2_render/lens_switch_preserves_overlay — overlay flags survive lens type changes
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_render", "lens_switch_preserves_overlay");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      gui::g_state.show_horizon_line = true;
      gui::g_state.show_horizon_label = true;
      gui::g_state.show_grid_line = true;
      gui::g_state.show_grid_label = true;
      gui::g_state.show_sun_circles_line = true;
      gui::g_state.show_sun_circles_label = true;
      ctx->Yield();

      gui::g_state.renderer.lens_type = 4;  // Dual Fisheye EA (full-sky)
      ctx->Yield(3);
      IM_CHECK_EQ(gui::g_state.show_horizon_line, true);
      IM_CHECK_EQ(gui::g_state.show_horizon_label, true);
      IM_CHECK_EQ(gui::g_state.show_grid_line, true);
      IM_CHECK_EQ(gui::g_state.show_grid_label, true);
      IM_CHECK_EQ(gui::g_state.show_sun_circles_line, true);
      IM_CHECK_EQ(gui::g_state.show_sun_circles_label, true);

      gui::g_state.renderer.lens_type = 0;  // Linear
      ctx->Yield(3);
      IM_CHECK_EQ(gui::g_state.show_horizon_line, true);
      IM_CHECK_EQ(gui::g_state.show_horizon_label, true);
      IM_CHECK_EQ(gui::g_state.show_grid_line, true);
      IM_CHECK_EQ(gui::g_state.show_grid_label, true);
      IM_CHECK_EQ(gui::g_state.show_sun_circles_line, true);
      IM_CHECK_EQ(gui::g_state.show_sun_circles_label, true);
    };
  }

  // ===== task-visible-lens-adapt (scrum-globe-view-v3 176.2) =====
  // lens × visibility adapter matrix: full-sky lenses disable the entire
  // Visibility section; Globe disables only the Front radio button.

  // p2_render/visibility_globe_front_disabled — Globe lens: Upper/Lower/Full
  // remain interactive, Front is greyed out (no 2-hemisphere symmetry).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_render", "visibility_globe_front_disabled");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      gui::g_state.renderer.lens_type = gui::kLensTypeGlobe;
      ctx->Yield(3);
      auto info_upper = ctx->ItemInfo("**/Upper##visible");
      IM_CHECK((info_upper.ItemFlags & ImGuiItemFlags_Disabled) == 0);
      auto info_full = ctx->ItemInfo("**/Full##visible");
      IM_CHECK((info_full.ItemFlags & ImGuiItemFlags_Disabled) == 0);
      auto info_lower = ctx->ItemInfo("**/Lower##visible");
      IM_CHECK((info_lower.ItemFlags & ImGuiItemFlags_Disabled) == 0);
      auto info_front = ctx->ItemInfo("**/Front##visible");
      IM_CHECK((info_front.ItemFlags & ImGuiItemFlags_Disabled) != 0);
    };
  }

  // p2_render/visibility_dual_fisheye_all_disabled — dual-fisheye (full-sky)
  // disables all four Visibility radio buttons.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_render", "visibility_dual_fisheye_all_disabled");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      gui::g_state.renderer.lens_type = gui::kLensTypeDualFisheyeEqualArea;
      ctx->Yield(3);
      auto info_upper = ctx->ItemInfo("**/Upper##visible");
      IM_CHECK((info_upper.ItemFlags & ImGuiItemFlags_Disabled) != 0);
      auto info_full = ctx->ItemInfo("**/Full##visible");
      IM_CHECK((info_full.ItemFlags & ImGuiItemFlags_Disabled) != 0);
      auto info_lower = ctx->ItemInfo("**/Lower##visible");
      IM_CHECK((info_lower.ItemFlags & ImGuiItemFlags_Disabled) != 0);
      auto info_front = ctx->ItemInfo("**/Front##visible");
      IM_CHECK((info_front.ItemFlags & ImGuiItemFlags_Disabled) != 0);
    };
  }

  // p2_render/visibility_rectangular_all_disabled — rectangular (full-sky)
  // disables all four Visibility radio buttons.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_render", "visibility_rectangular_all_disabled");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      gui::g_state.renderer.lens_type = gui::kLensTypeRectangular;
      ctx->Yield(3);
      auto info_upper = ctx->ItemInfo("**/Upper##visible");
      IM_CHECK((info_upper.ItemFlags & ImGuiItemFlags_Disabled) != 0);
      auto info_full = ctx->ItemInfo("**/Full##visible");
      IM_CHECK((info_full.ItemFlags & ImGuiItemFlags_Disabled) != 0);
      auto info_lower = ctx->ItemInfo("**/Lower##visible");
      IM_CHECK((info_lower.ItemFlags & ImGuiItemFlags_Disabled) != 0);
      auto info_front = ctx->ItemInfo("**/Front##visible");
      IM_CHECK((info_front.ItemFlags & ImGuiItemFlags_Disabled) != 0);
    };
  }

  // p2_render/visibility_dual_ortho_all_disabled — dual orthographic (full-sky)
  // disables all four Visibility radio buttons.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_render", "visibility_dual_ortho_all_disabled");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      gui::g_state.renderer.lens_type = gui::kLensTypeDualFisheyeOrthographic;
      ctx->Yield(3);
      auto info_upper = ctx->ItemInfo("**/Upper##visible");
      IM_CHECK((info_upper.ItemFlags & ImGuiItemFlags_Disabled) != 0);
      auto info_full = ctx->ItemInfo("**/Full##visible");
      IM_CHECK((info_full.ItemFlags & ImGuiItemFlags_Disabled) != 0);
      auto info_lower = ctx->ItemInfo("**/Lower##visible");
      IM_CHECK((info_lower.ItemFlags & ImGuiItemFlags_Disabled) != 0);
      auto info_front = ctx->ItemInfo("**/Front##visible");
      IM_CHECK((info_front.ItemFlags & ImGuiItemFlags_Disabled) != 0);
    };
  }

  // p2_render/visibility_fisheye_all_enabled — single fisheye (baseline):
  // all four Visibility radio buttons remain interactive.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_render", "visibility_fisheye_all_enabled");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      gui::g_state.renderer.lens_type = gui::kLensTypeFisheyeEqualArea;
      ctx->Yield(3);
      auto info_upper = ctx->ItemInfo("**/Upper##visible");
      IM_CHECK((info_upper.ItemFlags & ImGuiItemFlags_Disabled) == 0);
      auto info_full = ctx->ItemInfo("**/Full##visible");
      IM_CHECK((info_full.ItemFlags & ImGuiItemFlags_Disabled) == 0);
      auto info_lower = ctx->ItemInfo("**/Lower##visible");
      IM_CHECK((info_lower.ItemFlags & ImGuiItemFlags_Disabled) == 0);
      auto info_front = ctx->ItemInfo("**/Front##visible");
      IM_CHECK((info_front.ItemFlags & ImGuiItemFlags_Disabled) == 0);
    };
  }

  // p2_render/visibility_fisheye_ortho_all_enabled — single orthographic fisheye
  // (kLensTypeFisheyeOrthographic, enum 8) is NOT in kFullSkyLensTypes: all four
  // Visibility radio buttons remain interactive (orthographic research conclusion).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_render", "visibility_fisheye_ortho_all_enabled");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      gui::g_state.renderer.lens_type = gui::kLensTypeFisheyeOrthographic;
      ctx->Yield(3);
      auto info_upper = ctx->ItemInfo("**/Upper##visible");
      IM_CHECK((info_upper.ItemFlags & ImGuiItemFlags_Disabled) == 0);
      auto info_full = ctx->ItemInfo("**/Full##visible");
      IM_CHECK((info_full.ItemFlags & ImGuiItemFlags_Disabled) == 0);
      auto info_lower = ctx->ItemInfo("**/Lower##visible");
      IM_CHECK((info_lower.ItemFlags & ImGuiItemFlags_Disabled) == 0);
      auto info_front = ctx->ItemInfo("**/Front##visible");
      IM_CHECK((info_front.ItemFlags & ImGuiItemFlags_Disabled) == 0);
    };
  }

  // ===== task-visible-hemisphere-combo =====
  // Verify visibility controls: radio buttons (base hemisphere) + checkbox (front).

  // p2_render/visibility_click_hit — fisheye lens; ItemClick on Lower radio and Front
  // checkbox commits the correct values.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_render", "visibility_click_hit");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      gui::g_state.renderer.lens_type = gui::kLensTypeFisheyeEqualArea;
      ctx->Yield(3);
      ctx->ItemClick("**/Lower##visible");
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.renderer.visible, gui::kVisibleLower);
      IM_CHECK_EQ(gui::g_state.renderer.front, false);
      ctx->ItemClick("**/Front##visible");
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.renderer.visible, gui::kVisibleLower);
      IM_CHECK_EQ(gui::g_state.renderer.front, true);
      ctx->ItemClick("**/Front##visible");
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.renderer.front, false);
    };
  }

  // p2_render/visibility_same_row — fisheye lens; Upper radio and Front checkbox
  // must share the same screen row.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_render", "visibility_same_row");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      gui::g_state.renderer.lens_type = gui::kLensTypeFisheyeEqualArea;
      ctx->Yield(3);
      auto info_upper = ctx->ItemInfo("**/Upper##visible");
      auto info_front = ctx->ItemInfo("**/Front##visible");
      IM_CHECK_EQ(info_upper.RectFull.Min.y, info_front.RectFull.Min.y);
      IM_CHECK(info_front.RectFull.Min.x > info_upper.RectFull.Min.x);
    };
  }

  // p2_render/visibility_globe_disables_front — Globe lens disables the Front checkbox.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_render", "visibility_globe_disables_front");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      gui::g_state.renderer.lens_type = gui::kLensTypeGlobe;
      ctx->Yield(3);
      auto info_front = ctx->ItemInfo("**/Front##visible");
      IM_CHECK(info_front.ItemFlags & ImGuiItemFlags_Disabled);
    };
  }

  // p2_render/visibility_full_sky_disables_all — Full-sky lens disables the radio buttons
  // and Front checkbox (outer BeginDisabled path, acceptance criterion #5).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_render", "visibility_full_sky_disables_all");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      gui::g_state.renderer.lens_type = gui::kFullSkyLensTypes[0];
      ctx->Yield(3);
      auto info_upper = ctx->ItemInfo("**/Upper##visible");
      auto info_front = ctx->ItemInfo("**/Front##visible");
      IM_CHECK(info_upper.ItemFlags & ImGuiItemFlags_Disabled);
      IM_CHECK(info_front.ItemFlags & ImGuiItemFlags_Disabled);
    };
  }
}

// ========== task-test-gui-interaction: P1 Running simulation restart tests ==========

// These tests drive a real sim via StartPerfSimulation()/StopPerfSimulation() so that
// DoRun() has a live g_server to commit to and SyncFromPoller() can produce texture uploads.
// Restart detection uses the monotonic texture_upload_count as the "AtLeast +1" signal.


void RegisterP1RunningTests(ImGuiTestEngine* engine) {
  // p1_running/crystal_change_triggers_restart (prototype — see plan.md Step 5 ROI decision)
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_running", "crystal_change_triggers_restart");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      StartPerfSimulation();

      // Wait for first batch of sim data
      auto ray_timeout = std::chrono::steady_clock::now() + std::chrono::seconds(10);
      while (gui::g_state.stats_sim_ray_num == 0) {
        ctx->Yield();
        if (std::chrono::steady_clock::now() > ray_timeout) {
          fprintf(stderr, "[TEST] crystal_change_triggers_restart: timeout waiting for first sim data\n");
          StopPerfSimulation();
          IM_CHECK(false);
          return;
        }
      }

      // Wait for first texture upload (so baseline is stable and represents a completed cycle)
      auto upload_timeout = std::chrono::steady_clock::now() + std::chrono::seconds(5);
      while (gui::g_state.texture_upload_count == 0) {
        ctx->Yield();
        if (std::chrono::steady_clock::now() > upload_timeout) {
          break;  // proceed with baseline=0 — WaitForSimRestartAtLeast will still succeed on the first upload
        }
      }

      // Capture baseline — must NOT yield between this and the DoRun call below
      auto baseline = gui::g_state.texture_upload_count;

      // Act: change a crystal parameter, mark dirty, commit on test thread
      gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id].height = 2.5f;
      gui::g_state.dirty = true;
      gui::DoRun(/*user_initiated=*/true);

      // Assert: a new texture upload happens within timeout
      bool ok = WaitForSimRestartAtLeast(ctx, baseline, 1500);

      StopPerfSimulation();
      IM_CHECK(ok);
    };
  }

  // p1_running/filter_change_triggers_restart (extension)
  // Filter changes call MarkStructHardDirty() which raises display_epoch_floor to committed_epoch,
  // fencing the old generation's textures until DoRun re-commits and mints a newer epoch that
  // clears the floor. End-to-end exercise of the epoch-keyed anti-flicker gate.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_running", "filter_change_triggers_restart");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();

      // Pre-register a filter on the first entry so StartPerfSimulation's DoRun commits with it
      gui::FilterConfig f;
      f.SetRaypath(gui::RaypathParams{ "3-1-5" });
      gui::SetFilter(gui::g_state, gui::g_state.layers[0].entries[0], f);

      StartPerfSimulation();

      // Wait for first sim data
      auto ray_timeout = std::chrono::steady_clock::now() + std::chrono::seconds(10);
      while (gui::g_state.stats_sim_ray_num == 0) {
        ctx->Yield();
        if (std::chrono::steady_clock::now() > ray_timeout) {
          fprintf(stderr, "[TEST] filter_change_triggers_restart: timeout waiting for first sim data\n");
          StopPerfSimulation();
          IM_CHECK(false);
          return;
        }
      }

      // Wait for first texture upload
      auto upload_timeout = std::chrono::steady_clock::now() + std::chrono::seconds(5);
      while (gui::g_state.texture_upload_count == 0) {
        ctx->Yield();
        if (std::chrono::steady_clock::now() > upload_timeout) {
          break;
        }
      }

      auto baseline = gui::g_state.texture_upload_count;

      // Act: change filter raypath, mark filter dirty, commit
      gui::g_state.filters[*gui::g_state.layers[0].entries[0].filter_id].MutableRaypathText() = "3-1-5-7";
      gui::g_state.MarkStructHardDirty();
      gui::DoRun(/*user_initiated=*/true);

      // Filter changes may take slightly longer (epoch-floor fence until re-commit); use 2000ms
      bool ok = WaitForSimRestartAtLeast(ctx, baseline, 2000);

      StopPerfSimulation();
      IM_CHECK(ok);
    };
  }

  // p1_running/ray_count_infinite_toggle_while_running (extension)
  // Toggle Infinite off → fixed ray count triggers a full restart.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_running", "ray_count_infinite_toggle_while_running");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      StartPerfSimulation();
      // StartPerfSimulation sets sim.infinite=true; verify
      IM_CHECK_EQ(gui::g_state.sim.infinite, true);

      // Wait for initial data + upload
      auto ray_timeout = std::chrono::steady_clock::now() + std::chrono::seconds(10);
      while (gui::g_state.stats_sim_ray_num == 0) {
        ctx->Yield();
        if (std::chrono::steady_clock::now() > ray_timeout) {
          fprintf(stderr, "[TEST] infinite_toggle_while_running: timeout waiting for first sim data\n");
          StopPerfSimulation();
          IM_CHECK(false);
          return;
        }
      }
      auto upload_timeout = std::chrono::steady_clock::now() + std::chrono::seconds(5);
      while (gui::g_state.texture_upload_count == 0) {
        ctx->Yield();
        if (std::chrono::steady_clock::now() > upload_timeout) {
          break;
        }
      }

      // Drain any pending commits before capturing baseline (guard against race M7)
      ctx->Yield(3);

      auto baseline = gui::g_state.texture_upload_count;

      // Act: switch off Infinite → fixed ray count
      gui::g_state.sim.infinite = false;
      gui::g_state.sim.ray_num_millions = 0.5f;  // small enough that the run can finish
      gui::g_state.dirty = true;
      gui::DoRun(/*user_initiated=*/true);

      bool ok = WaitForSimRestartAtLeast(ctx, baseline, 2000);

      StopPerfSimulation();
      IM_CHECK(ok);
    };
  }
}

// ========== task-test-gui-interaction: P2 Modal interaction flows ==========


void RegisterP2InteractionModalTests(ImGuiTestEngine* engine) {
  // p2_modal/unsaved_changes_cancel_path — clicking Cancel preserves state
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_modal", "unsaved_changes_cancel_path");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      // Left panel now renders cards directly (no tabs)

      // Add an entry so state differs from default
      gui::g_state.layers[0].entries.push_back(gui::EntryCard{});
      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers[0].entries.size()), 2);
      gui::g_state.dirty = true;
      ctx->Yield();

      // Click New → triggers unsaved popup
      ctx->ItemClick("##TopBar/New");
      ctx->Yield(2);

      // Click Cancel
      ctx->ItemClick("Unsaved Changes/Cancel");
      ctx->Yield();

      // State preserved: still 2 entries, still dirty, pending action cleared
      IM_CHECK_EQ(gui::g_state.dirty, true);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers[0].entries.size()), 2);
      IM_CHECK_EQ(static_cast<int>(gui::g_pending_action), static_cast<int>(gui::PendingAction::kNone));
    };
  }

  // p2_modal/unsaved_changes_save_path — clicking Save writes file, resets state, performs New
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_modal", "unsaved_changes_save_path");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Set current_file_path so DoSave() doesn't pop a dialog
      const char* tmp_path = "/tmp/lumice_unsaved_save.lmc";
      std::remove(tmp_path);  // clean up any stale file from a previous failed run
      gui::g_state.current_file_path = tmp_path;
      // Modify state in a verifiable way
      gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id].type = gui::CrystalType::kPyramid;
      gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id].prism_h = 3.5f;
      gui::g_state.dirty = true;
      ctx->Yield();

      // Click New → triggers popup → Save
      ctx->ItemClick("##TopBar/New");
      ctx->Yield(2);
      ctx->ItemClick("Unsaved Changes/Save");
      ctx->Yield(2);

      // After Save path: DoSave wrote the file (dirty=false), DoNew() resets state, action cleared
      IM_CHECK_EQ(gui::g_state.dirty, false);
      IM_CHECK_EQ(static_cast<int>(gui::g_pending_action), static_cast<int>(gui::PendingAction::kNone));

      // Roundtrip check: the saved file reflects the pre-New prism_h=3.5 pyramid state
      gui::GuiState loaded;
      std::vector<unsigned char> tex;
      int tw = 0;
      int th = 0;
      bool load_ok = gui::LoadLmcFile(tmp_path, loaded, tex, tw, th);
      IM_CHECK(load_ok);
      IM_CHECK_EQ(gui::CrystalOf(loaded, loaded.layers[0].entries[0]).type, gui::CrystalType::kPyramid);
      IM_CHECK_EQ(gui::CrystalOf(loaded, loaded.layers[0].entries[0]).prism_h, 3.5f);

      std::remove(tmp_path);
    };
  }

  // task-cleanup-hardening AC4: DoSave under sim_state == kModified must NOT
  // silently write the stale-preview .lmc + clear Modified. It must front a
  // popup ("Config modified — Run first / Save anyway / Cancel"). This test
  // pins the "gate + no side-effect" contract at the call-level (DoSave
  // returns without touching the file); a second test below covers the
  // "Save anyway resumes serialization" branch, and a third pins that the
  // non-kModified path stays silent (regression guard for the pre-353.5
  // direct-serialize semantics).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_modal", "save_on_kmodified_opens_prompt_no_file_write");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      const char* tmp_path = "/tmp/lumice_savemod_gate.lmc";
      std::remove(tmp_path);
      gui::g_state.current_file_path = tmp_path;
      // Force sim_state == kModified via the reconciled base (kDone) + dirty
      // (any struct edit lands here through the field-tier reconciler).
      gui::g_state.sim_state = gui::GuiState::SimState::kModified;
      gui::g_state.dirty = true;

      // Precondition: no popup pending, no kind pending.
      IM_CHECK(!gui::g_show_save_modified_popup);
      IM_CHECK_EQ(static_cast<int>(gui::g_pending_save_kind), static_cast<int>(gui::PendingSaveKind::kNone));

      gui::DoSave();

      // The gate opened the popup + queued kSave, and the .lmc was NOT touched
      // (silent serialization would leave the file on disk).
      IM_CHECK(gui::g_show_save_modified_popup);
      IM_CHECK_EQ(static_cast<int>(gui::g_pending_save_kind), static_cast<int>(gui::PendingSaveKind::kSave));
      std::ifstream f(tmp_path);
      IM_CHECK(!f.good());

      // dirty stays true (the gate must not clear it — that only happens on
      // successful serialization).
      IM_CHECK(gui::g_state.dirty);

      // Cleanup — pretend the popup was resolved by Cancel.
      gui::g_show_save_modified_popup = false;
      gui::g_pending_save_kind = gui::PendingSaveKind::kNone;
    };
  }

  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_modal", "save_anyway_resumes_serialization");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      const char* tmp_path = "/tmp/lumice_savemod_anyway.lmc";
      std::remove(tmp_path);
      gui::g_state.current_file_path = tmp_path;
      gui::g_state.sim_state = gui::GuiState::SimState::kModified;
      gui::g_state.dirty = true;

      // Gate opens the popup; then simulate the user pressing "Save anyway"
      // by calling PerformSave directly (the popup's Save-anyway button body).
      gui::DoSave();
      IM_CHECK(gui::g_show_save_modified_popup);
      gui::PerformSave();

      // File written; dirty cleared. sim_state stays kModified — the popup
      // deliberately does NOT silently clear it (that would erase the
      // user-visible "config differs from render" cue for future edits).
      std::ifstream f(tmp_path);
      IM_CHECK(f.good());
      IM_CHECK(!gui::g_state.dirty);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(gui::GuiState::SimState::kModified));

      gui::g_show_save_modified_popup = false;
      gui::g_pending_save_kind = gui::PendingSaveKind::kNone;
      std::remove(tmp_path);
    };
  }

  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_modal", "save_bypasses_prompt_when_not_kmodified");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      const char* tmp_path = "/tmp/lumice_savemod_bypass.lmc";
      std::remove(tmp_path);
      gui::g_state.current_file_path = tmp_path;
      // kIdle: no run has happened yet — nothing "modified" to warn about.
      gui::g_state.sim_state = gui::GuiState::SimState::kIdle;
      gui::g_state.dirty = true;

      gui::DoSave();

      // No popup queued; the file was written directly.
      IM_CHECK(!gui::g_show_save_modified_popup);
      IM_CHECK_EQ(static_cast<int>(gui::g_pending_save_kind), static_cast<int>(gui::PendingSaveKind::kNone));
      std::ifstream f(tmp_path);
      IM_CHECK(f.good());
      IM_CHECK(!gui::g_state.dirty);

      std::remove(tmp_path);
    };
  }

  // code-review-01 M1: RenderUnsavedPopup's "Save" button must route through
  // the kModified gate (DoSave) rather than bypassing it — clicking Save
  // while both dirty and kModified must chain to RenderSaveModifiedPopup, not
  // silently serialize the stale preview. This drives the real button click
  // path (unlike save_on_kmodified_opens_prompt_no_file_write, which calls
  // DoSave() directly), pinning the last-mile button wiring the Pragmatist
  // review flagged as untested: Unsaved-Save → Save-Modified popup opens →
  // click "Save anyway" → file written AND the deferred New actually runs.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_modal", "unsaved_save_chains_to_save_modified_popup");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      const char* tmp_path = "/tmp/lumice_unsaved_chain_anyway.lmc";
      std::remove(tmp_path);
      gui::g_state.current_file_path = tmp_path;
      gui::g_state.layers[0].entries.push_back(gui::EntryCard{});
      // sim_state is re-derived every frame by ReconcileSimState (I2) from
      // run_intent, so a direct sim_state write would not survive the Yield()
      // calls below. kLoaded is the "static loaded result" intent (base=kDone
      // regardless of server/committed_epoch) — combined with dirty=true it
      // reconciles to kModified each frame, same as loading a .lmc then editing.
      gui::g_state.run_intent = gui::RunIntent::kLoaded;
      gui::g_state.dirty = true;
      ctx->Yield();

      // Click New → Unsaved popup → Save. Since sim_state == kModified, this
      // must NOT write the file yet — it must open Save-Modified instead.
      ctx->ItemClick("##TopBar/New");
      ctx->Yield(2);
      ctx->ItemClick("Unsaved Changes/Save");
      // DoSave() (invoked by the click handler above) sets
      // g_show_save_modified_popup mid-frame; RenderSaveModifiedPopup —
      // called right after RenderUnsavedPopup in both main.cpp and this
      // harness's GuiFunc — consumes it and opens the popup the same frame.
      ctx->Yield(3);

      IM_CHECK(ImGui::IsPopupOpen("Save Modified Config"));

      std::ifstream not_yet(tmp_path);
      IM_CHECK(!not_yet.good());
      // New must not have run yet — the entry added above is still present.
      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers[0].entries.size()), 2);

      // Click "Save anyway" — the deferred save runs, and so does the New
      // that was queued by the original Unsaved-Save click.
      ctx->ItemClick("Save Modified Config/Save anyway");
      ctx->Yield(2);

      std::ifstream f(tmp_path);
      IM_CHECK(f.good());
      IM_CHECK_EQ(static_cast<int>(gui::g_pending_action), static_cast<int>(gui::PendingAction::kNone));
      // DoNew() reset the document, so the pushed-back entry is gone.
      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers[0].entries.size()), 1);

      std::remove(tmp_path);
    };
  }

  // code-review-01 M1 companion: "Cancel" on the chained Save-Modified popup
  // must abort the deferred New too (not silently proceed without a save).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_modal", "unsaved_save_chain_cancel_aborts_pending_action");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      const char* tmp_path = "/tmp/lumice_unsaved_chain_cancel.lmc";
      std::remove(tmp_path);
      gui::g_state.current_file_path = tmp_path;
      gui::g_state.layers[0].entries.push_back(gui::EntryCard{});
      // See unsaved_save_chains_to_save_modified_popup above for why kLoaded
      // (not a direct sim_state write) is needed to survive the frame Yield.
      gui::g_state.run_intent = gui::RunIntent::kLoaded;
      gui::g_state.dirty = true;
      ctx->Yield();

      ctx->ItemClick("##TopBar/New");
      ctx->Yield(2);
      ctx->ItemClick("Unsaved Changes/Save");
      // See the sibling test above for why this hop needs 3 frames.
      ctx->Yield(3);

      ctx->ItemClick("Save Modified Config/Cancel");
      ctx->Yield(2);

      // Neither the save nor the New happened.
      std::ifstream f(tmp_path);
      IM_CHECK(!f.good());
      IM_CHECK(gui::g_state.dirty);
      IM_CHECK_EQ(static_cast<int>(gui::g_pending_action), static_cast<int>(gui::PendingAction::kNone));
      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers[0].entries.size()), 2);
    };
  }

  // code-review-02 M1 investigated whether RenderSaveModifiedPopup can be
  // dismissed via Escape, bypassing all three button branches and leaving
  // g_pending_action / g_pending_save_kind stale for a later, unrelated Save
  // to misfire on. White-box mechanism check (see the doc comment on
  // RenderSaveModifiedPopup) found Dear ImGui's NavUpdateCancelRequest()
  // never routes Escape to ClosePopupToLevel() for a window with
  // ImGuiWindowFlags_Modal set — which BeginPopupModal always sets — so this
  // popup (like every modal in this app) cannot be dismissed via Escape at
  // all. This test drives the real key press against the live popup to pin
  // that verified fact as a regression guard: if a future Dear ImGui upgrade
  // ever changes this, the popup will start closing here and the test fails,
  // flagging that the pending-sentinel leak this review worried about has
  // become reachable and needs the edge-detect fix reviewers originally asked
  // for.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_modal", "save_modified_popup_escape_is_a_noop");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      const char* tmp_path = "/tmp/lumice_savemod_escape.lmc";
      std::remove(tmp_path);
      gui::g_state.current_file_path = tmp_path;
      gui::g_state.layers[0].entries.push_back(gui::EntryCard{});
      // See unsaved_save_chains_to_save_modified_popup above for why kLoaded
      // (not a direct sim_state write) is needed to survive the frame Yield.
      gui::g_state.run_intent = gui::RunIntent::kLoaded;
      gui::g_state.dirty = true;
      ctx->Yield();

      ctx->ItemClick("##TopBar/New");
      ctx->Yield(2);
      ctx->ItemClick("Unsaved Changes/Save");
      // See the sibling test above for why this hop needs 3 frames.
      ctx->Yield(3);
      IM_CHECK(ImGui::IsPopupOpen("Save Modified Config"));
      IM_CHECK_EQ(static_cast<int>(gui::g_pending_action), static_cast<int>(gui::PendingAction::kNew));

      // Escape must NOT dismiss this modal (see doc comment above).
      ctx->KeyPress(ImGuiKey_Escape);
      ctx->Yield(2);
      IM_CHECK(ImGui::IsPopupOpen("Save Modified Config"));

      // Nothing fired, and the queued intent is untouched — consistent with
      // the popup never having closed.
      std::ifstream f(tmp_path);
      IM_CHECK(!f.good());
      IM_CHECK(gui::g_state.dirty);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers[0].entries.size()), 2);
      IM_CHECK_EQ(static_cast<int>(gui::g_pending_action), static_cast<int>(gui::PendingAction::kNew));
      IM_CHECK_EQ(static_cast<int>(gui::g_pending_save_kind), static_cast<int>(gui::PendingSaveKind::kSave));

      // The popup is still open — resolve it via a real button so the test
      // doesn't leak an open modal into the next test.
      ctx->ItemClick("Save Modified Config/Save anyway");
      ctx->Yield(2);
      std::ifstream f2(tmp_path);
      IM_CHECK(f2.good());
      IM_CHECK_EQ(static_cast<int>(gui::g_pending_action), static_cast<int>(gui::PendingAction::kNone));

      std::remove(tmp_path);
    };
  }

  // p2_modal/crystal_modal_open_cancel — open crystal modal, cancel, verify no state change
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_modal", "crystal_modal_open_cancel");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      auto& cr = gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id];
      auto type_before = cr.type;
      float height_before = cr.height.center;

      // Click Edit Crystal button on the first entry card
      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(3);

      // Click Cancel in the modal
      ctx->ItemClick("**/" ICON_FA_XMARK " Cancel##edit_modal");
      ctx->Yield(2);

      // State unchanged
      IM_CHECK_EQ(cr.type, type_before);
      IM_CHECK_EQ(cr.height, height_before);
    };
  }

  // p2_modal/crystal_modal_edit_confirm — edit crystal height, click OK, verify state updated
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_modal", "crystal_modal_edit_confirm");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Verify initial height
      float initial_height = gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id].height.center;

      // Open crystal modal
      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(3);

      // Modify height via the input widget in the modal
      ctx->ItemInputValue("**/##Height##modal_cr_input", 5.0f);
      ctx->Yield();

      // Click OK to commit buffer to state
      ctx->ItemClick("**/" ICON_FA_CHECK " OK##edit_modal");
      ctx->Yield(2);

      // Height should be updated to 5.0
      IM_CHECK_EQ(gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id].height, 5.0f);
    };
  }

  // p2_modal/crystal_modal_reset_all_resets_shape_params — modify shape params,
  // click Reset All in Crystal tab, OK to commit; verify all 7 shape fields
  // restored to CrystalConfig{} defaults while name/type/axis are preserved.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_modal", "crystal_modal_reset_all_resets_shape_params");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Step 1: bring entry crystal to a non-default state (height=5.0) and
      // record axis/name/type baseline. We need a non-default baseline so the
      // post-Reset assertion has something to differ from.
      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(3);
      ctx->ItemInputValue("**/##Height##modal_cr_input", 5.0f);
      ctx->Yield();
      ctx->ItemClick("**/" ICON_FA_CHECK " OK##edit_modal");
      ctx->Yield(2);

      auto& entry = gui::g_state.layers[0].entries[0];
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, entry).height, 5.0f);
      gui::AxisDist axis_zenith_baseline = gui::CrystalOf(gui::g_state, entry).zenith;
      gui::AxisDist axis_azimuth_baseline = gui::CrystalOf(gui::g_state, entry).azimuth;
      gui::AxisDist axis_roll_baseline = gui::CrystalOf(gui::g_state, entry).roll;
      std::string name_baseline = gui::CrystalOf(gui::g_state, entry).name;
      gui::CrystalType type_baseline = gui::CrystalOf(gui::g_state, entry).type;

      // Step 2: open modal again, modify height to another non-default (2.0),
      // click Reset All, then OK to commit the reset.
      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(3);
      ctx->ItemInputValue("**/##Height##modal_cr_input", 2.0f);
      ctx->Yield();
      ctx->ItemClick("**/Reset All##modal_cr");
      ctx->Yield();
      ctx->ItemClick("**/" ICON_FA_CHECK " OK##edit_modal");
      ctx->Yield(2);

      // Step 3: assert all 7 shape fields back to defaults.
      gui::CrystalConfig defaults;
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, entry).height, defaults.height);
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, entry).prism_h, defaults.prism_h);
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, entry).upper_h, defaults.upper_h);
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, entry).lower_h, defaults.lower_h);
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, entry).upper_alpha, defaults.upper_alpha);
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, entry).lower_alpha, defaults.lower_alpha);
      for (int i = 0; i < 6; ++i) {
        IM_CHECK_EQ(gui::CrystalOf(gui::g_state, entry).face_distance[i], defaults.face_distance[i]);
      }

      // Step 4: assert name/type/axis preserved (Reset All must not touch them).
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, entry).name, name_baseline);
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, entry).type, type_baseline);
      IM_CHECK(gui::CrystalOf(gui::g_state, entry).zenith == axis_zenith_baseline);
      IM_CHECK(gui::CrystalOf(gui::g_state, entry).azimuth == axis_azimuth_baseline);
      IM_CHECK(gui::CrystalOf(gui::g_state, entry).roll == axis_roll_baseline);
    };
  }

  // p2_modal/crystal_modal_reset_all_then_cancel_keeps_entry — Reset All in
  // edit buffer, then Cancel; entry must keep its pre-modal state (Reset All
  // only touches g_crystal_buf, never the entry until OK).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_modal", "crystal_modal_reset_all_then_cancel_keeps_entry");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Establish a non-default baseline (height=5.0) via OK commit.
      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(3);
      ctx->ItemInputValue("**/##Height##modal_cr_input", 5.0f);
      ctx->Yield();
      ctx->ItemClick("**/" ICON_FA_CHECK " OK##edit_modal");
      ctx->Yield(2);

      auto& entry = gui::g_state.layers[0].entries[0];
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, entry).height, 5.0f);

      // Open modal, Reset All, Cancel — entry must remain at 5.0.
      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(3);
      ctx->ItemClick("**/Reset All##modal_cr");
      ctx->Yield();
      ctx->ItemClick("**/" ICON_FA_XMARK " Cancel##edit_modal");
      ctx->Yield(2);

      // Entry unchanged: Cancel discarded the Reset All effect from the buffer.
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, entry).height, 5.0f);
    };
  }

  // p2_modal/spectrum_modal_reset_resets_to_preset_seed — inject a non-default custom
  // spectrum baseline, open the modal, click Reset, OK to commit; verify the committed
  // custom_spectrum matches the uniform 9-point 400–720nm/weight=1.0 preset seed
  // (asserted against the construction formula, not BuildPresetSeed() itself, to keep
  // the test independent of that private helper).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_modal", "spectrum_modal_reset_resets_to_preset_seed");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Step 1: establish a non-default custom-spectrum baseline via direct state
      // injection (same technique as test_gui_import_export.cpp:262-263). Direct
      // injection matches the "user already has a custom spectrum and clicks Edit
      // to reopen it" scenario without depending on the Combo/button chain.
      gui::g_state.sun.spectrum_index = gui::kCustomSpectrumIndex;
      gui::g_state.sun.custom_spectrum = { { 450.0f, 0.5f }, { 550.0f, 1.0f }, { 650.0f, 0.7f } };

      // Step 2: open the modal directly. See progress.md DECISION 2026-07-05: this
      // is functionally equivalent to clicking "Edit spectrum...##spectrum_edit"
      // (panels.cpp:965) but doesn't rely on the outer Combo showing that button.
      gui::OpenSpectrumModal(gui::g_state);
      ctx->Yield(3);

      // Step 3: Reset + OK.
      ctx->ItemClick("**/Reset##spec_reset");
      ctx->Yield();
      ctx->ItemClick("**/" ICON_FA_CHECK " OK##spec_ok");
      ctx->Yield(2);

      // Step 4: assert the committed spectrum matches the preset seed formula
      // (9 points, 400–720 nm equidistant, weight=1.0). Independent of
      // BuildPresetSeed() so a refactor of that helper can't silently drift this.
      const auto& out = gui::g_state.sun.custom_spectrum;
      IM_CHECK_EQ(gui::g_state.sun.spectrum_index, gui::kCustomSpectrumIndex);
      IM_CHECK_EQ(out.size(), (size_t)9);
      IM_CHECK_EQ(out.front().wavelength, 400.0f);
      IM_CHECK_EQ(out.back().wavelength, 720.0f);
      for (size_t i = 0; i < out.size(); ++i) {
        const float expected_wl = 400.0f + static_cast<float>(i) * 40.0f;
        IM_CHECK_EQ(out[i].wavelength, expected_wl);
        IM_CHECK_EQ(out[i].weight, 1.0f);
      }
    };
  }

  // p2_modal/spectrum_modal_reset_then_cancel_keeps_state — Reset in the edit buffer,
  // then Cancel; committed custom_spectrum and spectrum_index must stay at their
  // pre-Reset values (transactional contract from task-323: OK is the sole commit).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_modal", "spectrum_modal_reset_then_cancel_keeps_state");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Baseline: a non-default committed custom spectrum.
      const std::vector<gui::WlWeight> baseline = { { 450.0f, 0.5f }, { 550.0f, 1.0f }, { 650.0f, 0.7f } };
      gui::g_state.sun.spectrum_index = gui::kCustomSpectrumIndex;
      gui::g_state.sun.custom_spectrum = baseline;
      const int baseline_index = gui::g_state.sun.spectrum_index;

      // Open modal, Reset (mutates only the edit buffer), Cancel.
      gui::OpenSpectrumModal(gui::g_state);
      ctx->Yield(3);
      ctx->ItemClick("**/Reset##spec_reset");
      ctx->Yield();
      ctx->ItemClick("**/" ICON_FA_XMARK " Cancel##spec_cancel");
      ctx->Yield(2);

      // Committed state must be untouched.
      IM_CHECK_EQ(gui::g_state.sun.spectrum_index, baseline_index);
      IM_CHECK_EQ(gui::g_state.sun.custom_spectrum.size(), baseline.size());
      for (size_t i = 0; i < baseline.size(); ++i) {
        IM_CHECK_EQ(gui::g_state.sun.custom_spectrum[i].wavelength, baseline[i].wavelength);
        IM_CHECK_EQ(gui::g_state.sun.custom_spectrum[i].weight, baseline[i].weight);
      }
    };
  }

  // p2_modal/filter_modal_ok_no_change_keeps_nullopt — open filter modal on empty
  // entry, click OK without touching anything: entry must stay nullopt (otherwise
  // a default-constructed filter "* In PBD" silently blocks all rays).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_modal", "filter_modal_ok_no_change_keeps_nullopt");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Entry starts with no filter.
      IM_CHECK(!gui::g_state.layers[0].entries[0].filter_id.has_value());

      // Open filter modal, click OK immediately without modifying anything.
      ctx->ItemClick("**/Edit##fi");
      ctx->Yield(3);
      ctx->ItemClick("**/" ICON_FA_CHECK " OK##edit_modal");
      ctx->Yield(2);

      // Filter must still be nullopt.
      IM_CHECK(!gui::g_state.layers[0].entries[0].filter_id.has_value());
    };
  }

  // p2_modal/filter_modal_ok_change_creates_filter — open filter modal on empty
  // entry, modify a field (action), click OK: entry.filter must be created.
  // Covers the buf_changed=true && !initial_present branch of CommitAllBuffers.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_modal", "filter_modal_ok_change_creates_filter");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Entry starts with no filter.
      IM_CHECK(!gui::g_state.layers[0].entries[0].filter_id.has_value());

      // Open filter modal, type a raypath (touches g_filter_buf via g_raypath_buf
      // sync in CommitAllBuffers), click OK.
      ctx->ItemClick("**/Edit##fi");
      ctx->Yield(3);
      ctx->ItemInputValue("**/##row_text_0", "1-3");
      ctx->Yield();
      ctx->ItemClick("**/" ICON_FA_CHECK " OK##edit_modal");
      ctx->Yield(2);

      // Filter must now exist with the typed raypath.
      IM_CHECK(gui::g_state.layers[0].entries[0].filter_id.has_value());
      IM_CHECK_EQ(gui::g_state.filters[*gui::g_state.layers[0].entries[0].filter_id].RaypathText(), std::string("1-3"));
    };
  }

  // p2_modal/crystal_modal_reset_view_matches_thumbnail — Modal Reset View
  // must derive the rotation matrix from the same single source
  // (DefaultPreviewRotation) used by the entry-card thumbnail. Setting the
  // entry to a canonical preset config, opening the modal, and clicking Reset
  // View must leave g_crystal_rotation element-wise equal to
  // DefaultPreviewRotation(<expected preset>). This is a contract test that
  // will fail loudly if either path drifts from the shared source.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_modal", "crystal_modal_reset_view_matches_thumbnail");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      // Helper: install canonical AxisDist tuples for a given preset, open
      // the Crystal modal, click Reset View, then assert g_crystal_rotation
      // equals the matrix produced by DefaultPreviewRotation(preset).
      auto verify_preset = [&](const char* label, gui::AxisPreset preset, gui::AxisDist zenith, gui::AxisDist azimuth,
                               gui::AxisDist roll) {
        ResetTestState();
        ctx->Yield(2);

        auto& cr = gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id];
        cr.zenith = zenith;
        cr.azimuth = azimuth;
        cr.roll = roll;

        // Sanity: the canonical config must classify as the preset we expect.
        IM_CHECK_EQ(static_cast<int>(gui::ClassifyAxisPreset(cr.zenith, cr.azimuth, cr.roll)),
                    static_cast<int>(preset));

        // Open Edit Crystal modal (copies entry.crystal → modal buffer).
        ctx->ItemClick("**/Edit##cr");
        ctx->Yield(3);

        // Click Reset View in the modal preview pane.
        ctx->ItemClick("**/Reset View##modal");
        ctx->Yield(2);

        // Snapshot the rotation now — Cancel restores g_crystal_rotation from
        // g_saved_rotation (edit_modals.cpp:287/767), so reading after Cancel
        // would observe the pre-Open state instead of the Reset View result.
        float observed[16];
        std::memcpy(observed, gui::g_crystal_rotation, sizeof(observed));

        // Close modal cleanly (Cancel discards any buffer changes; trackball
        // restore is incidental and does not affect the snapshot above).
        ctx->ItemClick("**/" ICON_FA_XMARK " Cancel##edit_modal");
        ctx->Yield(2);

        // Build the same params[3] that ResetCrystalView passes (zenith,
        // azimuth, roll) so kCustom routes through the chain-formula path.
        gui::AxisDist params[3] = { zenith, azimuth, roll };
        float expected[16] = { 0 };
        gui::DefaultPreviewRotation(preset, params, expected);
        for (int i = 0; i < 16; ++i) {
          if (observed[i] != expected[i]) {
            IM_ERRORF("preset=%s index=%d actual=%f expected=%f", label, i, static_cast<double>(observed[i]),
                      static_cast<double>(expected[i]));
          }
        }
      };

      const gui::AxisDist az_full{ gui::AxisDistType::kUniform, 0.0f, 360.0f };
      const gui::AxisDist roll_full{ gui::AxisDistType::kUniform, 0.0f, 360.0f };
      const gui::AxisDist roll_locked{ gui::AxisDistType::kGauss, 0.0f, 1.0f };
      const gui::AxisDist zenith_full{ gui::AxisDistType::kUniform, 0.0f, 360.0f };

      // Plate canonical: zenith Gauss(mean=0, std=1).
      verify_preset("Plate", gui::AxisPreset::kPlate, gui::AxisDist{ gui::AxisDistType::kGauss, 0.0f, 1.0f }, az_full,
                    roll_full);

      // Column canonical: zenith Gauss(mean=90, std=1) with roll uniform-full.
      verify_preset("Column", gui::AxisPreset::kColumn, gui::AxisDist{ gui::AxisDistType::kGauss, 90.0f, 1.0f },
                    az_full, roll_full);

      // Parry canonical: same zenith as Column but roll locked.
      verify_preset("Parry", gui::AxisPreset::kParry, gui::AxisDist{ gui::AxisDistType::kGauss, 90.0f, 1.0f }, az_full,
                    roll_locked);

      // Lowitz canonical: zenith Gauss(mean=0, std=40) with roll locked.
      verify_preset("Lowitz", gui::AxisPreset::kLowitz, gui::AxisDist{ gui::AxisDistType::kGauss, 0.0f, 40.0f },
                    az_full, roll_locked);

      // Random canonical: all three axes Uniform(0, 360°). Locks the
      // ClassifyAxisPreset → DefaultPreviewRotation chain end-to-end so that
      // any future special-case for kRandom in either path is caught.
      verify_preset("Random", gui::AxisPreset::kRandom, zenith_full, az_full, roll_full);

      // Custom: anything that doesn't match the four presets — gauss with
      // moderate spread, roll free.
      verify_preset("Custom", gui::AxisPreset::kCustom, gui::AxisDist{ gui::AxisDistType::kGauss, 90.0f, 20.0f },
                    az_full, gui::AxisDist{ gui::AxisDistType::kGauss, 0.0f, 20.0f });
    };
  }

  // p2_modal/axis_preset_button_resets_modal_preview — Clicking a preset
  // button inside the Axis tab must drive g_crystal_rotation to the default
  // view derived from that preset (button writes per-preset distribution into
  // g_axis_buf, then ResetCrystalView consumes it as params, so kCustom also
  // lands on the chain-derived matrix). Closes the bug where switching preset
  // only updated the outer card thumbnail.
  //
  // Note: expected params are inlined here from edit_modals.cpp::kAxisPresets
  // (file-scope static, not exported). If kAxisPresets defaults change, this
  // test must be updated to match.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_modal", "axis_preset_button_resets_modal_preview");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      const gui::AxisDist az_full{ gui::AxisDistType::kUniform, 0.0f, 360.0f };
      const gui::AxisDist roll_free{ gui::AxisDistType::kUniform, 0.0f, 360.0f };
      const gui::AxisDist roll_locked{ gui::AxisDistType::kGauss, 0.0f, 1.0f };

      auto verify_preset_button = [&](const char* preset_label, gui::AxisPreset expected_preset, gui::AxisDist zenith,
                                      gui::AxisDist azimuth, gui::AxisDist roll) {
        ResetTestState();
        ctx->Yield(2);
        ctx->ItemClick("**/Edit##cr");
        ctx->Yield(3);
        ctx->ItemClick("**/###axis_tab");
        ctx->Yield(2);
        ctx->ItemClick(("**/" + std::string(preset_label)).c_str());
        ctx->Yield(2);

        float observed[16];
        std::memcpy(observed, gui::g_crystal_rotation, sizeof(observed));

        // Build the expected matrix from the same params kAxisPresets writes.
        gui::AxisDist params[3] = { zenith, azimuth, roll };
        float expected[16] = { 0 };
        gui::DefaultPreviewRotation(expected_preset, params, expected);

        ctx->ItemClick("**/" ICON_FA_XMARK " Cancel##edit_modal");
        ctx->Yield(2);

        for (int i = 0; i < 16; ++i) {
          if (observed[i] != expected[i]) {
            IM_ERRORF("preset=%s index=%d actual=%f expected=%f", preset_label, i, static_cast<double>(observed[i]),
                      static_cast<double>(expected[i]));
          }
        }
      };

      verify_preset_button("Column", gui::AxisPreset::kColumn, gui::AxisDist{ gui::AxisDistType::kGauss, 90.0f, 1.0f },
                           az_full, roll_free);
      verify_preset_button("Plate", gui::AxisPreset::kPlate, gui::AxisDist{ gui::AxisDistType::kGauss, 0.0f, 1.0f },
                           az_full, roll_free);
      verify_preset_button("Parry", gui::AxisPreset::kParry, gui::AxisDist{ gui::AxisDistType::kGauss, 90.0f, 1.0f },
                           az_full, roll_locked);
      verify_preset_button("Lowitz", gui::AxisPreset::kLowitz, gui::AxisDist{ gui::AxisDistType::kGauss, 0.0f, 40.0f },
                           az_full, roll_locked);
      verify_preset_button("Random", gui::AxisPreset::kRandom, az_full, az_full, roll_free);
      verify_preset_button("Custom", gui::AxisPreset::kCustom, gui::AxisDist{ gui::AxisDistType::kGauss, 90.0f, 20.0f },
                           az_full, gui::AxisDist{ gui::AxisDistType::kGauss, 0.0f, 20.0f });
    };
  }

  // p2_modal/axis_slider_drag_does_not_reset_modal_preview — Negative contract
  // (review-02 Minor #1, AC#7): editing an axis-distribution slider must NOT
  // reset the modal preview rotation. Only preset-button clicks and Reset View
  // overwrite g_crystal_rotation. Guards against accidentally wiring slider
  // edits to ResetCrystalView during future refactors.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_modal", "axis_slider_drag_does_not_reset_modal_preview");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(3);

      // Mutate g_crystal_rotation via trackball drag (synthetic, world-coord
      // semantics). Snapshot taken AFTER the drag so the comparison baseline
      // is the dragged state, not the modal-open default.
      gui::ApplyTrackballRotation(20.0f, 0.0f);
      ctx->Yield(1);

      float before[16];
      std::memcpy(before, gui::g_crystal_rotation, sizeof(before));

      // Switch to Axis tab and edit the zenith Mean. Both ItemInputValue (text
      // half) and slider-drag (drag half) of SliderWithInput route through the
      // same write target — the bound `g_axis_buf[0].mean` field — so neither
      // path can call ResetCrystalView without the other doing so too. The text
      // input is used here because it's deterministic under the test harness;
      // the contract being verified is "no axis-edit path resets g_crystal_rotation",
      // and the only places that call ResetCrystalView are the preset buttons
      // (edit_modals.cpp:486-499) and the Reset View button (:387) — both
      // separate from the slider control.
      ctx->ItemClick("**/###axis_tab");
      ctx->Yield(2);
      ctx->ItemInputValue("**/Zenith/##Mean_input", 45.0f);
      ctx->Yield(2);

      // Snapshot AGAIN before Cancel — Cancel restores g_crystal_rotation from
      // the open-time saved state, so reading after Cancel would observe the
      // pre-drag matrix and produce a false positive.
      float after[16];
      std::memcpy(after, gui::g_crystal_rotation, sizeof(after));

      ctx->ItemClick("**/" ICON_FA_XMARK " Cancel##edit_modal");
      ctx->Yield(2);

      for (int i = 0; i < 16; ++i) {
        if (before[i] != after[i]) {
          IM_ERRORF("slider edit unexpectedly mutated g_crystal_rotation at index %d: before=%f after=%f", i,
                    static_cast<double>(before[i]), static_cast<double>(after[i]));
        }
      }
    };
  }

  // p2_modal/world_coord_trackball_drag_axis — Verifies the world-coordinate
  // trackball semantics expressed in the GUI mesh frame (which has a Y-Z swap
  // vs. core/world):
  //   dx>0 → rotation around mesh +y (= world +z, vertical spin)
  //   dy>0 → rotation around mesh +x (= world +x, camera-right tilt)
  // Both compose as left-multiply on g_crystal_rotation. Guards Step 5 of #165.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_modal", "world_coord_trackball_drag_axis");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      auto check_axis = [](const char* label, float dx, float dy, int axis_kind) {
        // axis_kind: 0 = mesh +x (Rx), 1 = mesh +y (Ry).
        // Reset to identity before each sub-case.
        for (int i = 0; i < 16; ++i) {
          gui::g_crystal_rotation[i] = (i % 5 == 0) ? 1.0f : 0.0f;
        }
        gui::ApplyTrackballRotation(dx, dy);

        float mag = std::sqrt(dx * dx + dy * dy);
        float angle = mag * 0.01f;
        float c = std::cos(angle);
        float s = std::sin(angle);
        // Build the expected Rodrigues column-major matrix for the single axis.
        float exp_r[16] = { 0 };
        exp_r[15] = 1.0f;
        if (axis_kind == 0) {
          // Rx(angle): e1 unchanged; e2→(0,c,s); e3→(0,-s,c).
          exp_r[0] = 1.0f;
          exp_r[5] = c;
          exp_r[6] = s;
          exp_r[9] = -s;
          exp_r[10] = c;
        } else {
          // Ry(angle): e1→(c,0,-s); e2 unchanged; e3→(s,0,c).
          exp_r[0] = c;
          exp_r[2] = -s;
          exp_r[5] = 1.0f;
          exp_r[8] = s;
          exp_r[10] = c;
        }
        for (int i = 0; i < 16; ++i) {
          float diff = std::fabs(gui::g_crystal_rotation[i] - exp_r[i]);
          if (diff > 1e-5f) {
            IM_ERRORF("%s: index %d actual=%f expected=%f", label, i, static_cast<double>(gui::g_crystal_rotation[i]),
                      static_cast<double>(exp_r[i]));
          }
        }
      };

      ResetTestState();
      ctx->Yield(1);
      check_axis("dx>0 → mesh +y (world +z)", 10.0f, 0.0f, /*axis_kind=*/1);
      check_axis("dy>0 → mesh +x (world +x)", 0.0f, 10.0f, /*axis_kind=*/0);
    };
  }

  // P1: scrum-176 / task-crystal-card-highlight-active — the open edit modal
  // must report its bound (layer_idx, entry_idx) so RenderEntryCard can
  // visually highlight the source card. The lifecycle covers four close paths:
  // OK / Cancel / auto-close-on-entry-deletion / and the multi-entry routing
  // case where the second entry's modal must not bleed onto the first card.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_edit_modal", "active_card_target_lifecycle");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Initial: no modal, target is invalid.
      IM_CHECK_EQ(gui::IsEditModalOpen(), false);
      auto t0 = gui::GetEditModalTarget();
      IM_CHECK_EQ(t0.layer_idx, -1);
      IM_CHECK_EQ(t0.entry_idx, -1);

      // Open the Crystal modal on layer 0 entry 0 via the Edit button.
      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(4);
      IM_CHECK_EQ(gui::IsEditModalOpen(), true);
      auto t1 = gui::GetEditModalTarget();
      IM_CHECK_EQ(t1.layer_idx, 0);
      IM_CHECK_EQ(t1.entry_idx, 0);

      // Close via Cancel: target reset, modal closed.
      ctx->ItemClick("**/" ICON_FA_XMARK " Cancel##edit_modal");
      ctx->Yield(2);
      IM_CHECK_EQ(gui::IsEditModalOpen(), false);
      auto t2 = gui::GetEditModalTarget();
      IM_CHECK_EQ(t2.layer_idx, -1);
      IM_CHECK_EQ(t2.entry_idx, -1);

      // Reopen and close via OK: same lifecycle invariant.
      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(4);
      IM_CHECK_EQ(gui::IsEditModalOpen(), true);
      ctx->ItemClick("**/" ICON_FA_CHECK " OK##edit_modal");
      ctx->Yield(2);
      IM_CHECK_EQ(gui::IsEditModalOpen(), false);
      auto t3 = gui::GetEditModalTarget();
      IM_CHECK_EQ(t3.layer_idx, -1);
      IM_CHECK_EQ(t3.entry_idx, -1);

      // Multi-entry routing: open the modal on entry 1 and check the target
      // points to {0, 1}, NOT {0, 0}. We drive the modal via OpenEditModal()
      // directly (with a fully-formed EditRequest) so the integration path
      // OpenEditModal → g_active_modal/g_modal_*_idx still runs end-to-end —
      // bypassing the field assignment would degrade this AC into the
      // "assign X read X" tautology forbidden by code-quality/test-design.
      gui::g_state.layers[0].entries.emplace_back();
      ctx->Yield(2);
      gui::EditRequest req{ gui::EditTarget::kCrystal, /*layer_idx=*/0, /*entry_idx=*/1 };
      gui::OpenEditModal(req, gui::g_state);
      ctx->Yield(2);
      IM_CHECK_EQ(gui::IsEditModalOpen(), true);
      auto t4 = gui::GetEditModalTarget();
      IM_CHECK_EQ(t4.layer_idx, 0);
      IM_CHECK_EQ(t4.entry_idx, 1);

      // Auto-close on entry deletion (DA-4 / F-3 闭环): erase the bound
      // entry while the modal is open. The index-validity guard inside
      // RenderEditModals (see edit_modals.cpp:879) sets g_active_modal back
      // to kNone, and the next IsEditModalOpen() must return false.
      // We deliberately skip g_thumbnail_cache.OnLayerStructureChanged() here:
      // the line-879 guard only checks index bounds, not cache state, and
      // ResetTestState() / DoNew() at test entry already covers cache cleanup
      // for the next test, so the omission stays local.
      gui::g_state.layers[0].entries.erase(gui::g_state.layers[0].entries.begin() + 1);
      gui::g_thumbnail_cache.OnLayerStructureChanged();
      ctx->Yield(4);
      IM_CHECK_EQ(gui::IsEditModalOpen(), false);
      auto t5 = gui::GetEditModalTarget();
      IM_CHECK_EQ(t5.layer_idx, -1);
      IM_CHECK_EQ(t5.entry_idx, -1);
    };
  }

  // P1: scrum-176 / task-crystal-card-highlight-active — style-probe assertion
  // backing plan §5 M3 unattended-equivalent acceptance item 2 (pixel-color
  // delta). RenderEntryCard pushes ImGuiCol_Border ← style[ImGuiCol_NavHighlight]
  // and ImGuiStyleVar_ChildBorderSize ← kActiveCardBorder when active. We can't
  // reliably capture the left-panel framebuffer in headless mode (hidden GLFW
  // + retina non-determinism, learnings/tools-workflow.md), so we assert the
  // visual contract at the style-token layer instead: NavHighlight vs Border
  // must differ by ≥ 60 (sRGB L1) and kActiveCardBorder must exceed the
  // default ChildBorderSize. Both are necessary conditions for the highlight
  // being visible to the user; if either fails, the active branch in
  // RenderEntryCard is a no-op and the lifecycle test alone would still pass.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_edit_modal", "active_card_style_probe");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      const ImGuiStyle& style = ImGui::GetStyle();
      ImVec4 nav = style.Colors[ImGuiCol_NavHighlight];
      ImVec4 border = style.Colors[ImGuiCol_Border];
      auto to8 = [](float v) { return static_cast<int>(v * 255.0f + 0.5f); };
      int dr = std::abs(to8(nav.x) - to8(border.x));
      int dg = std::abs(to8(nav.y) - to8(border.y));
      int db = std::abs(to8(nav.z) - to8(border.z));
      int l1 = dr + dg + db;
      // sRGB L1 ≥ 60 is the threshold from plan §5 M3. NavHighlight in the
      // default ImGui dark theme is a strong yellow-orange (255, 170, 0) and
      // Border is dim grey (110, 110, 128, alpha 0.5), giving L1 ≈ 460 — way
      // above threshold. If a future theme change makes the two converge,
      // this assertion will catch the regression.
      IM_CHECK_GT(l1, 60);

      // Border thickness must be strictly greater than the default to be
      // perceivable. ImGui default ChildBorderSize is 1.0f; kActiveCardBorder
      // is 2.0f.
      IM_CHECK_GT(gui::kActiveCardBorder, style.ChildBorderSize);
    };
  }

  // ============================================================
  // p2_filter_type — task-composition-editor-ui (333.4, H5 SoP row editor)
  //
  // Filter modal is a sum-of-products row list:
  //   - Each row is one OR summand expressed in the small AND grammar
  //     (`3-5 & entry:2 & len:2-3`) validated by ValidateSummandText.
  //   - The pre-H5 FilterEditType discriminator + Raypath / Entry-Exit
  //     sub-panels are gone; every row can be raypath / entry-exit / an
  //     AND mix independently.
  //   - Remove Filter is a single intent flag (always enabled inside the
  //     modal; on OK it writes `filter_id = nullopt` regardless of typed
  //     content).
  //
  // Historical tests (`filter_type_radio_visible`,
  // `filter_type_switch_dispatches_subpanel`, `filter_per_type_buffer_isolated`,
  // `entry_exit_per_type_buffer_isolated`) tested the pre-H5 type
  // discriminator that no longer exists. Their coverage intent is subsumed
  // by the new "each row is independent" test below plus the AC1 round-trip
  // tests further down.
  // ============================================================

  // T1 — Action radio commits: typing a raypath in row 0, selecting
  // "Filter Out", OK → entry.filter.action == 1.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_filter_type", "filter_action_radio_commits_value");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      IM_CHECK(!gui::g_state.layers[0].entries[0].filter_id.has_value());

      ctx->ItemClick("**/Edit##fi");
      ctx->Yield(4);
      ctx->ItemInputValue("**/##row_text_0", "3-1-5");
      ctx->Yield();
      ctx->ItemClick("**/Filter Out##filter_action");
      ctx->Yield();
      ctx->ItemClick("**/" ICON_FA_CHECK " OK##edit_modal");
      ctx->Yield(2);

      IM_CHECK(gui::g_state.layers[0].entries[0].filter_id.has_value());
      IM_CHECK_EQ(gui::g_state.filters[*gui::g_state.layers[0].entries[0].filter_id].action, 1);
    };
  }

  // T2 — Multi-raypath OR across two rows: the H5 canonical way to express
  // "(3-5) OR (1-3)" is one summand row per alternative ("3-5" then
  // "+ Add OR row" then "1-3"), NOT the pre-H5 single-row ';' sugar (which
  // ValidateSummandText now rejects inside a row). Pins that the two rows
  // commit to a 2-summand SoP with each row's text preserved verbatim.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_filter_type", "filter_multi_raypath_or_e2e_via_modal");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      IM_CHECK(!gui::g_state.layers[0].entries[0].filter_id.has_value());

      ctx->ItemClick("**/Edit##fi");
      ctx->Yield(4);
      // Row 0 accepts a single-token raypath. This test uses two separate
      // rows to exercise the "+ Add OR row" path. (Post-334.3 H-A, a single
      // row `1-3;3-5` produces an equivalent serialized composition — see
      // semicolon_multi_raypath_single_row_commits + the import-export
      // equivalence tests.)
      ctx->ItemInputValue("**/##row_text_0", "3-5");
      ctx->Yield(1);
      ctx->ItemClick("**/+ Add OR row##summand_add");
      ctx->Yield(2);
      ctx->ItemInputValue("**/##row_text_1", "1-3");
      ctx->Yield(2);

      // OK must be enabled (both rows validate kValid).
      auto info_ok = ctx->ItemInfo("**/" ICON_FA_CHECK " OK##edit_modal");
      IM_CHECK((info_ok.ItemFlags & ImGuiItemFlags_Disabled) == 0);

      ctx->ItemClick("**/" ICON_FA_CHECK " OK##edit_modal");
      ctx->Yield(2);

      IM_CHECK(gui::g_state.layers[0].entries[0].filter_id.has_value());
      const auto& f = gui::g_state.filters[*gui::g_state.layers[0].entries[0].filter_id];
      IM_CHECK_EQ(f.param.size(), static_cast<size_t>(2));
      IM_CHECK_STR_EQ(f.param[0].text.c_str(), "3-5");
      IM_CHECK_STR_EQ(f.param[1].text.c_str(), "1-3");
    };
  }

  // T3 — Entry-Exit single-row: type "entry:2 & exit:5" in one row → the
  // committed SoP is a degenerate single-factor EE filter (equivalent to
  // pre-H5 EE type). Verifies AND grammar → EntryExitParams reconstruction.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_filter_type", "entry_exit_single_row_commits");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      IM_CHECK(!gui::g_state.layers[0].entries[0].filter_id.has_value());

      ctx->ItemClick("**/Edit##fi");
      ctx->Yield(4);
      ctx->ItemInputValue("**/##row_text_0", "entry:2 & exit:5");
      ctx->Yield(2);
      ctx->ItemClick("**/Filter Out##filter_action");
      ctx->Yield(2);

      auto info_ok = ctx->ItemInfo("**/" ICON_FA_CHECK " OK##edit_modal");
      IM_CHECK((info_ok.ItemFlags & ImGuiItemFlags_Disabled) == 0);
      ctx->ItemClick("**/" ICON_FA_CHECK " OK##edit_modal");
      ctx->Yield(2);

      IM_CHECK(gui::g_state.layers[0].entries[0].filter_id.has_value());
      const auto& f = gui::g_state.filters[*gui::g_state.layers[0].entries[0].filter_id];
      IM_CHECK(f.IsEntryExit());
      const auto& ee = f.EntryExitParamsValue();
      IM_CHECK_EQ(ee.entry_text, std::string("2"));
      IM_CHECK_EQ(ee.exit_text, std::string("5"));
      IM_CHECK_EQ(f.action, 1);
      IM_CHECK(f.sym_p);
      IM_CHECK(f.sym_b);
      IM_CHECK(f.sym_d);
    };
  }

  // T4 — EE multi-value + length range in AND grammar: type
  // "entry:3,4 & exit:5 & len:2-3" in one row → factor holds entry list
  // "3,4", exit "5", length_mode=3, [2,3].
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_filter_type", "entry_exit_uplift_multivalue_list");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      ctx->ItemClick("**/Edit##fi");
      ctx->Yield(4);
      ctx->ItemInputValue("**/##row_text_0", "entry:3,4 & exit:5 & len:2-3");
      ctx->Yield(2);

      auto info_ok = ctx->ItemInfo("**/" ICON_FA_CHECK " OK##edit_modal");
      IM_CHECK((info_ok.ItemFlags & ImGuiItemFlags_Disabled) == 0);
      ctx->ItemClick("**/" ICON_FA_CHECK " OK##edit_modal");
      ctx->Yield(2);

      IM_CHECK(gui::g_state.layers[0].entries[0].filter_id.has_value());
      const auto& f = gui::g_state.filters[*gui::g_state.layers[0].entries[0].filter_id];
      IM_CHECK(f.IsEntryExit());
      const auto& ee = f.EntryExitParamsValue();
      IM_CHECK_EQ(ee.entry_text, std::string("3,4"));
      IM_CHECK_EQ(ee.exit_text, std::string("5"));
      IM_CHECK_EQ(ee.length_mode, 3);
      IM_CHECK_EQ(ee.min_len, 2);
      IM_CHECK_EQ(ee.max_len, 3);
    };
  }

  // T4b — FilterSummary length-mode formatting (pure state, no ImGui).
  // Pins the four format strings the summary should emit for a
  // programmatically-constructed EE filter.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_filter_type", "entry_exit_uplift_length_summary_formats");
    t->TestFunc = [](ImGuiTestContext* /*ctx*/) {
      auto make_summary = [](int mode, int min_v, int max_v) {
        gui::FilterConfig fc;
        gui::EntryExitParams ee;
        ee.entry_text = "3";
        ee.exit_text = "5";
        ee.length_mode = mode;
        ee.min_len = min_v;
        ee.max_len = max_v;
        fc.SetEntryExit(ee);
        return gui::FilterSummary(std::optional<gui::FilterConfig>{ fc });
      };
      IM_CHECK_STR_EQ(make_summary(0, 1, 1).c_str(), "EE:3-5 In PBD");
      IM_CHECK_STR_EQ(make_summary(1, 2, 2).c_str(), "EE:3-5 L=2 In PBD");
      IM_CHECK_STR_EQ(make_summary(2, 1, 4).c_str(), "EE:3-5 L<=4 In PBD");
      IM_CHECK_STR_EQ(make_summary(3, 2, 5).c_str(), "EE:3-5 L=[2,5] In PBD");
    };
  }

  // T5 — Remove Filter (H5 unified button): clicking it arms the intent flag;
  // OK writes filter_id = nullopt even if the row buffers are non-empty
  // (session-level single direction, mirrors pre-H5 EE Remove semantics).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_filter_type", "remove_filter_clears_on_ok");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Pre-populate entry.filter so Remove has something to clear.
      gui::FilterConfig f;
      f.SetRaypath(gui::RaypathParams{ "3-1-5" });
      gui::SetFilter(gui::g_state, gui::g_state.layers[0].entries[0], f);
      ctx->Yield();
      IM_CHECK(gui::g_state.layers[0].entries[0].filter_id.has_value());

      ctx->ItemClick("**/Edit##fi");
      ctx->Yield(4);

      // Remove Filter is always enabled in H5 (intent flag; no derivation
      // from row emptiness).
      auto info_remove = ctx->ItemInfo("**/Remove Filter##filter");
      IM_CHECK((info_remove.ItemFlags & ImGuiItemFlags_Disabled) == 0);
      ctx->ItemClick("**/Remove Filter##filter");
      ctx->Yield(2);

      // OK is enabled even after Remove: the intent bypasses row validation.
      auto info_ok = ctx->ItemInfo("**/" ICON_FA_CHECK " OK##edit_modal");
      IM_CHECK((info_ok.ItemFlags & ImGuiItemFlags_Disabled) == 0);
      ctx->ItemClick("**/" ICON_FA_CHECK " OK##edit_modal");
      ctx->Yield(2);

      IM_CHECK(!gui::g_state.layers[0].entries[0].filter_id.has_value());
    };
  }

  // T6 — Row list add/remove: clicking "+ Add OR row" grows the SoP; the
  // per-row Remove button shrinks it. With one row remaining, per-row Remove
  // must be disabled (>=1 row invariant).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_filter_type", "row_add_remove_visible");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      ctx->ItemClick("**/Edit##fi");
      ctx->Yield(4);

      // On modal open, single blank row exists.
      IM_CHECK(ctx->ItemExists("**/##row_text_0"));
      IM_CHECK(!ctx->ItemExists("**/##row_text_1"));

      // Per-row Remove is disabled with only one row (≥1 row invariant).
      auto info_del0 = ctx->ItemInfo("**/" ICON_FA_XMARK "##row_delete_0");
      IM_CHECK((info_del0.ItemFlags & ImGuiItemFlags_Disabled) != 0);

      // Add a row: new row has uid=1.
      ctx->ItemClick("**/+ Add OR row##summand_add");
      ctx->Yield(2);
      IM_CHECK(ctx->ItemExists("**/##row_text_1"));

      // Both per-row Remove buttons are enabled now.
      auto info_del1 = ctx->ItemInfo("**/" ICON_FA_XMARK "##row_delete_1");
      IM_CHECK((info_del1.ItemFlags & ImGuiItemFlags_Disabled) == 0);

      // Delete row 1: only row 0 remains.
      ctx->ItemClick("**/" ICON_FA_XMARK "##row_delete_1");
      ctx->Yield(2);
      IM_CHECK(!ctx->ItemExists("**/##row_text_1"));
      IM_CHECK(ctx->ItemExists("**/##row_text_0"));

      ctx->ItemClick("**/" ICON_FA_XMARK " Cancel##edit_modal");
      ctx->Yield(2);
    };
  }

  // T6b — AC4 (scrum-369.1 host-abi-cpu-caps): the "+ Add OR row" soft cap was
  // raised from 16 to kMaxSummandRows=256, so the button must stay ENABLED past
  // the old 16-row limit. Click it up to 20 rows and assert (a) the button never
  // reports Disabled below the cap and (b) row uid 16 (the 17th row) exists —
  // proving growth past 16. First-hand runtime verification of AC4 via the real
  // GUI widget path (owner decision 2026-07-15, in lieu of a manual click).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_filter_type", "add_rows_past_16_enabled");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      ctx->ItemClick("**/Edit##fi");
      ctx->Yield(4);

      // Fresh modal starts with one blank row (uid 0). Add 19 more → uids 0..19.
      for (int i = 0; i < 19; ++i) {
        auto info_add = ctx->ItemInfo("**/+ Add OR row##summand_add");
        IM_CHECK((info_add.ItemFlags & ImGuiItemFlags_Disabled) == 0);
        ctx->ItemClick("**/+ Add OR row##summand_add");
        ctx->Yield(2);
      }

      // Row uid 16 = the 17th row: the button admitted growth well past 16.
      IM_CHECK(ctx->ItemExists("**/##row_text_16"));
      IM_CHECK(ctx->ItemExists("**/##row_text_19"));
      // Still below kMaxSummandRows=256, so the button remains enabled.
      auto info_add_final = ctx->ItemInfo("**/+ Add OR row##summand_add");
      IM_CHECK((info_add_final.ItemFlags & ImGuiItemFlags_Disabled) == 0);

      ctx->ItemClick("**/" ICON_FA_XMARK " Cancel##edit_modal");
      ctx->Yield(2);
    };
  }

  // T7 — Multi-row commit + per-row edit isolation (subsumes the pre-H5
  // per-type buffer isolation contract). Add three rows with distinct
  // grammar shapes (pure raypath / EE / AND mix), OK, verify the SoP has
  // three summands with the expected text preserved verbatim per row.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_filter_type", "multi_row_commits_sop");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      ctx->ItemClick("**/Edit##fi");
      ctx->Yield(4);

      ctx->ItemInputValue("**/##row_text_0", "3-5");
      ctx->Yield(1);
      ctx->ItemClick("**/+ Add OR row##summand_add");
      ctx->Yield(2);
      ctx->ItemInputValue("**/##row_text_1", "entry:2 & exit:4");
      ctx->Yield(1);
      ctx->ItemClick("**/+ Add OR row##summand_add");
      ctx->Yield(2);
      ctx->ItemInputValue("**/##row_text_2", "3-5 & entry:2");
      ctx->Yield(2);

      auto info_ok = ctx->ItemInfo("**/" ICON_FA_CHECK " OK##edit_modal");
      IM_CHECK((info_ok.ItemFlags & ImGuiItemFlags_Disabled) == 0);
      ctx->ItemClick("**/" ICON_FA_CHECK " OK##edit_modal");
      ctx->Yield(2);

      IM_CHECK(gui::g_state.layers[0].entries[0].filter_id.has_value());
      const auto& f = gui::g_state.filters[*gui::g_state.layers[0].entries[0].filter_id];
      IM_CHECK_EQ(f.param.size(), static_cast<size_t>(3));
      IM_CHECK_STR_EQ(f.param[0].text.c_str(), "3-5");
      IM_CHECK_STR_EQ(f.param[1].text.c_str(), "entry:2 & exit:4");
      IM_CHECK_STR_EQ(f.param[2].text.c_str(), "3-5 & entry:2");
    };
  }

  // T8 — AC1 end-to-end witness: three rows edited via GUI → SaveLmcFile →
  // LoadLmcFile → the reloaded FilterConfig equals the original (operator==
  // on SoP text). Complements 333.3's `sop_lmc_roundtrip` (which builds the
  // SoP programmatically) by closing the loop through the actual GUI editor.
  // The same three-row scenario as `multi_row_commits_sop` — reused so the two
  // tests together form one AC1 chain:
  //   [GUI edit → correct SoP] (T7) + [SoP → save+reload → identical SoP] (T8)
  //   ≡ [GUI edit → save+reload → same SoP that will drive the simulator].
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_filter_type", "sop_roundtrip_via_gui_editor");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      ctx->ItemClick("**/Edit##fi");
      ctx->Yield(4);
      ctx->ItemInputValue("**/##row_text_0", "3-5");
      ctx->Yield(1);
      ctx->ItemClick("**/+ Add OR row##summand_add");
      ctx->Yield(2);
      ctx->ItemInputValue("**/##row_text_1", "entry:2 & exit:4");
      ctx->Yield(1);
      ctx->ItemClick("**/+ Add OR row##summand_add");
      ctx->Yield(2);
      ctx->ItemInputValue("**/##row_text_2", "3-5 & entry:2");
      ctx->Yield(2);
      ctx->ItemClick("**/" ICON_FA_CHECK " OK##edit_modal");
      ctx->Yield(2);

      IM_CHECK(gui::g_state.layers[0].entries[0].filter_id.has_value());
      const auto original = gui::g_state.filters[*gui::g_state.layers[0].entries[0].filter_id];

      const char* tmp_path = "/tmp/lumice_gui_sop_roundtrip.lmc";
      const bool save_ok = gui::SaveLmcFile(tmp_path, gui::g_state, gui::g_preview, false);
      IM_CHECK(save_ok);

      gui::DoNew();
      std::vector<unsigned char> tex_data;
      int tex_w = 0;
      int tex_h = 0;
      const bool load_ok = gui::LoadLmcFile(tmp_path, gui::g_state, tex_data, tex_w, tex_h);
      IM_CHECK(load_ok);

      IM_CHECK(gui::g_state.layers[0].entries[0].filter_id.has_value());
      const auto& reloaded = gui::g_state.filters[*gui::g_state.layers[0].entries[0].filter_id];
      IM_CHECK(reloaded == original);
      IM_CHECK_EQ(reloaded.param.size(), static_cast<size_t>(3));
      IM_CHECK_STR_EQ(reloaded.param[0].text.c_str(), "3-5");
      IM_CHECK_STR_EQ(reloaded.param[1].text.c_str(), "entry:2 & exit:4");
      IM_CHECK_STR_EQ(reloaded.param[2].text.c_str(), "3-5 & entry:2");
      std::remove(tmp_path);
    };
  }

  // T9 — AC3 stress: multiple rounds of add / type / delete-middle-row.
  // Pins the plan §7 risk 1 mitigation (uid-encoded IDs prevent widget/buffer
  // tearing when middle rows are removed). Each round:
  //   1) Add a row (uid increases; existing rows keep their uid).
  //   2) Type into the new row via its uid-derived ID.
  //   3) Delete a middle row and verify the surviving rows' texts are still
  //      addressable by their pre-delete uid — i.e. no ID collision or
  //      buffer contents jumping to the wrong row (327 tearing symptom).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_filter_type", "row_dynamics_stress");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      ctx->ItemClick("**/Edit##fi");
      ctx->Yield(4);

      // Row 0 has uid=0 (SetRowsFromSop resets the uid counter on open).
      ctx->ItemInputValue("**/##row_text_0", "1-2");
      ctx->Yield(1);

      // Round 1: add row 1 (uid=1), type; expect both rows addressable.
      ctx->ItemClick("**/+ Add OR row##summand_add");
      ctx->Yield(2);
      IM_CHECK(ctx->ItemExists("**/##row_text_1"));
      ctx->ItemInputValue("**/##row_text_1", "3-4");
      ctx->Yield(1);

      // Round 2: add row 2 (uid=2), type.
      ctx->ItemClick("**/+ Add OR row##summand_add");
      ctx->Yield(2);
      IM_CHECK(ctx->ItemExists("**/##row_text_2"));
      ctx->ItemInputValue("**/##row_text_2", "entry:1 & exit:2");
      ctx->Yield(1);

      // Round 3: delete the middle row (uid=1). Row 0 (uid=0) and the former
      // row 2 (uid=2) survive; the uid=2 row's ID must NOT shift to
      // ##row_text_1 (that would prove the ID stack collided).
      ctx->ItemClick("**/" ICON_FA_XMARK "##row_delete_1");
      ctx->Yield(2);
      IM_CHECK(ctx->ItemExists("**/##row_text_0"));
      IM_CHECK(!ctx->ItemExists("**/##row_text_1"));
      IM_CHECK(ctx->ItemExists("**/##row_text_2"));

      // Round 4: add another row (uid=3); type; delete row 2 (uid=2).
      ctx->ItemClick("**/+ Add OR row##summand_add");
      ctx->Yield(2);
      IM_CHECK(ctx->ItemExists("**/##row_text_3"));
      ctx->ItemInputValue("**/##row_text_3", "5-6");
      ctx->Yield(1);
      ctx->ItemClick("**/" ICON_FA_XMARK "##row_delete_2");
      ctx->Yield(2);
      IM_CHECK(ctx->ItemExists("**/##row_text_0"));
      IM_CHECK(!ctx->ItemExists("**/##row_text_2"));
      IM_CHECK(ctx->ItemExists("**/##row_text_3"));

      // Round 5: commit; expect 2 rows, uid=0 text "1-2" and uid=3 text "5-6".
      // Row order matches iteration order in g_summand_rows (uid=0 then uid=3).
      ctx->ItemClick("**/" ICON_FA_CHECK " OK##edit_modal");
      ctx->Yield(2);

      IM_CHECK(gui::g_state.layers[0].entries[0].filter_id.has_value());
      const auto& f = gui::g_state.filters[*gui::g_state.layers[0].entries[0].filter_id];
      IM_CHECK_EQ(f.param.size(), static_cast<size_t>(2));
      IM_CHECK_STR_EQ(f.param[0].text.c_str(), "1-2");
      IM_CHECK_STR_EQ(f.param[1].text.c_str(), "5-6");
    };
  }

  // T10 — code-review-01 Major M1 regression: a blank OR row must NOT lower to
  // a match-all clause. A forgotten empty row beside a real one used to
  // materialize [{"3-5"}, {""}] → ExpandSopToClauses turns the factor-less row
  // into a match-all term → OR(3-5, match-all): a silent no-op under filter_in,
  // an all-black render under filter_out. The commit path now strips blank /
  // whitespace-only rows before materializing; if none survive → no filter.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_filter_type", "blank_row_dropped_not_matchall");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Author a single real row "3-5", then add a blank OR row and commit.
      ctx->ItemClick("**/Edit##fi");
      ctx->Yield(4);
      ctx->ItemInputValue("**/##row_text_0", "3-5");
      ctx->Yield(1);
      ctx->ItemClick("**/+ Add OR row##summand_add");
      ctx->Yield(2);
      // Leave row 1 blank on purpose (the M1 footgun).
      ctx->ItemClick("**/" ICON_FA_CHECK " OK##edit_modal");
      ctx->Yield(2);

      // The blank row is stripped: exactly one summand "3-5", NOT two, and no
      // match-all clause was injected.
      IM_CHECK(gui::g_state.layers[0].entries[0].filter_id.has_value());
      const auto& f = gui::g_state.filters[*gui::g_state.layers[0].entries[0].filter_id];
      IM_CHECK_EQ(f.param.size(), static_cast<size_t>(1));
      IM_CHECK_STR_EQ(f.param[0].text.c_str(), "3-5");

      // Reopen, blank out the sole real row's text too → all rows blank ≡ no
      // filter (filter_id cleared, not a match-all filter left behind).
      ctx->ItemClick("**/Edit##fi");
      ctx->Yield(4);
      ctx->ItemInputValue("**/##row_text_0", "");
      ctx->Yield(1);
      ctx->ItemClick("**/" ICON_FA_CHECK " OK##edit_modal");
      ctx->Yield(2);
      IM_CHECK(!gui::g_state.layers[0].entries[0].filter_id.has_value());
    };
  }

  // T11 (scrum-334.3 H-A) — a single OR row carrying inline ';' alternatives
  // ("1-3;3-5") commits without OK being disabled and preserves the whole
  // token verbatim in the row's raypath_text. Fan-out to multiple summands
  // happens at serialization time (see semicolon_row_equals_two_rows_composition
  // in test_gui_import_export.cpp for the end-to-end equivalence), NOT at
  // commit — so the on-disk row count stays at 1.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_filter_type", "semicolon_multi_raypath_single_row_commits");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      IM_CHECK(!gui::g_state.layers[0].entries[0].filter_id.has_value());

      ctx->ItemClick("**/Edit##fi");
      ctx->Yield(4);
      ctx->ItemInputValue("**/##row_text_0", "1-3;3-5");
      ctx->Yield(2);

      // OK must be enabled — ValidateSummandText now delegates to
      // ValidateRaypathTextMultiSegment for raypath tokens, so inner ';' is
      // accepted.
      auto info_ok = ctx->ItemInfo("**/" ICON_FA_CHECK " OK##edit_modal");
      IM_CHECK((info_ok.ItemFlags & ImGuiItemFlags_Disabled) == 0);

      ctx->ItemClick("**/" ICON_FA_CHECK " OK##edit_modal");
      ctx->Yield(2);

      IM_CHECK(gui::g_state.layers[0].entries[0].filter_id.has_value());
      const auto& f = gui::g_state.filters[*gui::g_state.layers[0].entries[0].filter_id];
      IM_CHECK_EQ(f.param.size(), static_cast<size_t>(1));
      IM_CHECK_STR_EQ(f.param[0].text.c_str(), "1-3;3-5");
      // The row parses into a single raypath factor (the ';' is inside the
      // token, not across factor boundaries).
      IM_CHECK_EQ(f.param[0].factors.size(), static_cast<size_t>(1));
    };
  }
}

// ========== task-gui-linked-entries: pick-mode / unlink / duplicate / badge ==========
// AC-9 coverage: link / unlink / duplicate / cancel / delete-cleanup /
// co-shared-highlight / legacy-partial-sharing seven paths.

void RegisterLinkedEntriesTests(ImGuiTestEngine* engine) {
  // link: ApplyPickLink copies source (crystal_id, filter_id) to target entry.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_linked", "link");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      ResetTestState();
      // Add a second entry with its own pool slot.
      gui::CrystalConfig c2;
      c2.type = gui::CrystalType::kPyramid;
      c2.prism_h = 2.0f;
      gui::EntryCard e2;
      e2.crystal_id = static_cast<int>(gui::g_state.crystals.size());
      gui::g_state.crystals.push_back(c2);
      gui::g_state.layers[0].entries.push_back(e2);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers[0].entries.size()), 2);

      const int src_cid = gui::g_state.layers[0].entries[0].crystal_id;
      IM_CHECK_NE(gui::g_state.layers[0].entries[1].crystal_id, src_cid);
      // Apply pick: entry 0 → entry 1
      bool ok = gui::ApplyPickLink(gui::g_state, { 0, 0 }, { 0, 1 });
      IM_CHECK(ok);
      IM_CHECK_EQ(gui::g_state.layers[0].entries[1].crystal_id, src_cid);
      IM_CHECK_EQ(gui::g_state.layers[0].entries[1].filter_id, gui::g_state.layers[0].entries[0].filter_id);
      IM_CHECK_EQ(gui::CountEntriesSharing(gui::g_state, src_cid, gui::g_state.layers[0].entries[0].filter_id), 2);
    };
  }

  // unlink: forking a shared entry creates a fresh pool slot.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_linked", "unlink");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      ResetTestState();
      // Two entries sharing crystal_id = 0.
      gui::EntryCard sibling;
      sibling.crystal_id = 0;
      gui::g_state.layers[0].entries.push_back(sibling);
      IM_CHECK_EQ(gui::CountEntriesSharing(gui::g_state, 0, std::nullopt), 2);
      const size_t crystals_before = gui::g_state.crystals.size();

      bool forked = gui::UnlinkEntryFromPool(gui::g_state, 0, 1);
      IM_CHECK(forked);
      IM_CHECK_EQ(gui::g_state.crystals.size(), crystals_before + 1);
      IM_CHECK_NE(gui::g_state.layers[0].entries[1].crystal_id, 0);
      IM_CHECK_EQ(gui::CountEntriesSharing(gui::g_state, 0, std::nullopt), 1);
    };
  }

  // duplicate: clone-to-pool produces two new ids; mutating the copy does not
  // affect the original. Validates the panels.cpp Duplicate handler.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_linked", "duplicate");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id].type = gui::CrystalType::kPyramid;
      gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id].prism_h = 1.5f;
      ctx->Yield();

      const int orig_cid = gui::g_state.layers[0].entries[0].crystal_id;
      ctx->ItemClick("**/" ICON_FA_COPY "##dup_0_0");
      ctx->Yield();
      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers[0].entries.size()), 2);
      const int dup_cid = gui::g_state.layers[0].entries[1].crystal_id;
      IM_CHECK_NE(dup_cid, orig_cid);

      // Mutate the duplicated entry's pool slot — original must remain intact.
      gui::g_state.crystals[dup_cid].prism_h = 3.7f;
      IM_CHECK_EQ(gui::g_state.crystals[orig_cid].prism_h, 1.5f);
      IM_CHECK_EQ(gui::g_state.crystals[dup_cid].prism_h, 3.7f);
    };
  }

  // cancel: pick_link_source cleared without applying ids.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_linked", "cancel");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      ResetTestState();
      gui::EntryCard sibling;
      gui::CrystalConfig c2;
      c2.type = gui::CrystalType::kPyramid;
      sibling.crystal_id = static_cast<int>(gui::g_state.crystals.size());
      gui::g_state.crystals.push_back(c2);
      gui::g_state.layers[0].entries.push_back(sibling);

      const int e0_cid = gui::g_state.layers[0].entries[0].crystal_id;
      const int e1_cid = gui::g_state.layers[0].entries[1].crystal_id;

      // Simulate Link to... press: set pick_link_source on source = entry 0.
      gui::g_state.pick_link_source = gui::GuiState::EntryRef{ 0, 0 };
      // Cancel without applying.
      gui::g_state.pick_link_source.reset();
      IM_CHECK_EQ(gui::g_state.layers[0].entries[0].crystal_id, e0_cid);
      IM_CHECK_EQ(gui::g_state.layers[0].entries[1].crystal_id, e1_cid);
      IM_CHECK(!gui::g_state.pick_link_source.has_value());
    };
  }

  // delete-cleanup: deleting an entry leaves orphan pool slots untouched (no
  // refcount-driven compaction) — pool grows monotonically within a session.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_linked", "delete_cleanup");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      ResetTestState();
      gui::CrystalConfig c2;
      c2.type = gui::CrystalType::kPyramid;
      gui::EntryCard e2;
      e2.crystal_id = static_cast<int>(gui::g_state.crystals.size());
      gui::g_state.crystals.push_back(c2);
      gui::g_state.layers[0].entries.push_back(e2);
      const size_t pool_before = gui::g_state.crystals.size();

      // Delete entry 1 (the pyramid one).
      gui::g_state.layers[0].entries.erase(gui::g_state.layers[0].entries.begin() + 1);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers[0].entries.size()), 1);
      // Pool slot is orphan but still present — guarantees other entries don't
      // see their ids shift.
      IM_CHECK_EQ(gui::g_state.crystals.size(), pool_before);
      // The surviving entry still points to its original slot.
      IM_CHECK_EQ(gui::g_state.layers[0].entries[0].crystal_id, 0);
    };
  }

  // co-shared-highlight: when two entries share (crystal_id, filter_id), the
  // CountEntriesSharing predicate is ≥ 2 and drives the fa-link badge +
  // co-shared border highlight. Tested at the data-layer (rendering is GL).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_linked", "co_shared_highlight");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      ResetTestState();
      gui::EntryCard sibling;
      sibling.crystal_id = 0;  // share with default entry
      gui::g_state.layers[0].entries.push_back(sibling);
      IM_CHECK_EQ(gui::CountEntriesSharing(gui::g_state, 0, std::nullopt), 2);
      // After unlink, badge predicate drops back to 1.
      gui::UnlinkEntryFromPool(gui::g_state, 0, 1);
      IM_CHECK_EQ(gui::CountEntriesSharing(gui::g_state, 0, std::nullopt), 1);
    };
  }

  // add_entry_independent: clicking "+ Crystal" in a layer must seed the new
  // entry with a fresh crystal pool slot (regression: implicit Link to slot 0).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_linked", "add_entry_independent");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      const int orig_cid = gui::g_state.layers[0].entries[0].crystal_id;
      const std::size_t crystals_before = gui::g_state.crystals.size();

      ctx->ItemClick("**/+ Crystal##layer_0");
      ctx->Yield();

      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers[0].entries.size()), 2);
      IM_CHECK_EQ(gui::g_state.crystals.size(), crystals_before + 1);
      const int new_cid = gui::g_state.layers[0].entries[1].crystal_id;
      IM_CHECK_NE(new_cid, orig_cid);
      // Pair must NOT be considered shared — fa-link badge must stay off.
      IM_CHECK_EQ(gui::CountEntriesSharing(gui::g_state, new_cid, std::nullopt), 1);
    };
  }

  // add_layer_independent: clicking "+ Layer" must seed the new layer's entry
  // with a fresh crystal pool slot (regression: implicit Link to slot 0).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_linked", "add_layer_independent");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      const int orig_cid = gui::g_state.layers[0].entries[0].crystal_id;
      const std::size_t crystals_before = gui::g_state.crystals.size();
      const std::size_t layers_before = gui::g_state.layers.size();

      ctx->ItemClick("**/+ Layer");
      ctx->Yield();

      IM_CHECK_EQ(gui::g_state.layers.size(), layers_before + 1);
      IM_CHECK_EQ(gui::g_state.crystals.size(), crystals_before + 1);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers.back().entries.size()), 1);
      const int new_cid = gui::g_state.layers.back().entries[0].crystal_id;
      IM_CHECK_NE(new_cid, orig_cid);
      IM_CHECK_EQ(gui::CountEntriesSharing(gui::g_state, new_cid, std::nullopt), 1);
    };
  }

  // filter_add_propagates_to_linked: adding a filter to one entry of a
  // linked group must also bind the linked siblings to the new filter pool
  // slot so the group stays coherent (fa-link badge remains visible).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_linked", "filter_add_propagates_to_linked");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      // Two entries linked at (crystal_id=0, filter_id=None).
      gui::EntryCard sibling;
      sibling.crystal_id = 0;
      gui::g_state.layers[0].entries.push_back(sibling);
      ctx->Yield(2);
      IM_CHECK_EQ(gui::CountEntriesSharing(gui::g_state, 0, std::nullopt), 2);

      // Open Filter modal on entry 0, type a raypath, commit via OK.
      ctx->ItemClick("**/Edit##fi");
      ctx->Yield(4);
      ctx->ItemInputValue("**/##row_text_0", "3-5");
      ctx->Yield(2);
      ctx->ItemClick("**/" ICON_FA_CHECK " OK##edit_modal");
      ctx->Yield(2);

      // Both entries must now share the new filter_id (linked group stays
      // coherent — badge predicate still matches).
      IM_CHECK(gui::g_state.layers[0].entries[0].filter_id.has_value());
      IM_CHECK_EQ(gui::g_state.layers[0].entries[1].filter_id, gui::g_state.layers[0].entries[0].filter_id);
      IM_CHECK_EQ(gui::CountEntriesSharing(gui::g_state, 0, gui::g_state.layers[0].entries[0].filter_id), 2);
    };
  }

  // filter_remove_propagates_to_linked: clearing an entry's filter via the
  // Remove Filter button must also clear linked siblings' filter_id so the
  // group remains coherent.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_linked", "filter_remove_propagates_to_linked");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      // Two entries linked at (crystal_id=0, filter_id=<set>).
      gui::FilterConfig f;
      f.SetRaypath(gui::RaypathParams{ "3-5" });
      gui::SetFilter(gui::g_state, gui::g_state.layers[0].entries[0], f);
      gui::EntryCard sibling;
      sibling.crystal_id = 0;
      sibling.filter_id = gui::g_state.layers[0].entries[0].filter_id;
      gui::g_state.layers[0].entries.push_back(sibling);
      ctx->Yield(2);
      IM_CHECK_EQ(gui::CountEntriesSharing(gui::g_state, 0, gui::g_state.layers[0].entries[0].filter_id), 2);

      // Open Filter modal on entry 0, click Remove Filter, commit via OK.
      ctx->ItemClick("**/Edit##fi");
      ctx->Yield(4);
      ctx->ItemClick("**/Remove Filter##filter");
      ctx->Yield(2);
      ctx->ItemClick("**/" ICON_FA_CHECK " OK##edit_modal");
      ctx->Yield(2);

      IM_CHECK(!gui::g_state.layers[0].entries[0].filter_id.has_value());
      IM_CHECK(!gui::g_state.layers[0].entries[1].filter_id.has_value());
      IM_CHECK_EQ(gui::CountEntriesSharing(gui::g_state, 0, std::nullopt), 2);
    };
  }

  // legacy-partial-sharing: same crystal_id but different filter_id is NOT
  // considered "shared" by the badge predicate (card is atomic share unit).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_linked", "legacy_partial_sharing");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      ResetTestState();
      // Entry 0: no filter.
      // Entry 1: shares crystal_id but adds a filter — different (cid, fid).
      gui::EntryCard sibling;
      sibling.crystal_id = 0;
      gui::FilterConfig f;
      f.SetRaypath(gui::RaypathParams{ "3-5" });
      gui::SetFilter(gui::g_state, sibling, f);
      gui::g_state.layers[0].entries.push_back(sibling);

      // Pair (0, nullopt) is unique to entry 0.
      IM_CHECK_EQ(gui::CountEntriesSharing(gui::g_state, 0, std::nullopt), 1);
      // Pair (0, 0) is unique to entry 1.
      IM_CHECK_EQ(gui::CountEntriesSharing(gui::g_state, 0, std::optional<int>{ 0 }), 1);
    };
  }

  // task-gui-feedback-affordances Step 7 (AC1): the end-to-end degrade-warning
  // wire. Big-OR filter (host-side ABI-legal) + a color config with > 64
  // distinct predicates on one placement — the ABI check passes (commit is
  // NOT rejected), the CORE drops the excess predicates (kNoBit), and DoRun
  // surfaces the "coloring degraded" modal via SetGuiWarning with a message
  // string DIFFERENT from the ABI-overflow message (identity-dedup safety).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_filter_type", "big_or_filter_with_color_overflow_surfaces_warning");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetTestState();
      gui::ClearGuiWarning();
      gui::g_server = LUMICE_CreateServer();
      IM_CHECK(gui::g_server != nullptr);

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
          ref.crystal_pool_id = 0;  // maps to CrystalConfig::id_ = 1 in ResetTestState()
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
      IM_CHECK(!warning.empty());
      IM_CHECK(warning.find("color") != std::string::npos || warning.find("Color") != std::string::npos);
      // The message MUST be distinct from the two existing ABI-overflow msgs
      // (filter cap / color-class cap), else SetGuiWarning's identity-dedup
      // would silently collapse them (regression anchor per plan §7 risk 3).
      IM_CHECK(warning.find("This raypath color configuration exceeds its predicate") != std::string::npos);
      IM_CHECK(warning.find("Simplify the color configuration") != std::string::npos);

      // Precise count lock (code-review-01 Suggestion): 66 unique predicates - kMaxBits(64) = 2
      // dropped. Ties the end-to-end GUI test to the same exact number the Step 5/7 unit tests
      // assert, rather than only the message substring.
      LUMICE_ColorOverflowInfo color_over{};
      IM_CHECK(LUMICE_GetColorOverflowInfo(gui::g_server, &color_over) == LUMICE_OK);
      IM_CHECK(color_over.component_overflow_count == 2);

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
    };
  }

  // task-color-degrade-gui-surfacing: end-to-end proof that an ASYNC GPU color
  // degrade (device buffer-layout cap exceeded on the worker's first batch)
  // surfaces a GUI modal. Unlike the component overflow above (synchronous, set
  // in CommitConfig), the color-class cap (kMaxColorClassesDevice=16) fires only
  // when the Metal backend actually runs, so the modal comes from SyncFromPoller
  // polling LUMICE_GetColorOverflowInfo — NOT from DoRun. Also guards the R1
  // reset (task-366 class): after switching to a clean config the count must
  // return to 0 and stop warning (no cross-config leak).
  {
    // Dedicated group so build.sh routes this to the REAL-TIMING pool: it drives
    // a live GPU sim and WaitForSimRestartAtLeast waits on wall-clock batch
    // accumulation, which --fixed-dt (the correctness pool) starves. See
    // scratchpad/task-gui-test-fixed-dt + build.sh two-pool split.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_gpu_color_degrade", "gpu_color_class_overflow_surfaces_async_warning");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      gui::ClearGuiWarning();
      gui::g_server = LUMICE_CreateServer();
      IM_CHECK(gui::g_server != nullptr);
      // Force the GPU (Metal) backend — the three color-degrade caps are
      // GPU-only. DoRun's MaybeReconstructServerForBackend rebuilds for GPU.
      gui::g_state.use_gpu_backend = true;

      // 20 color classes > kMaxColorClassesDevice=16 → BeginSession drops the
      // excess 4. Each class carries one whole-crystal (match_all) ref so it is
      // a real, non-empty class that reaches the core class table.
      gui::g_state.raypath_color.clear();
      constexpr int kNumClasses = 20;
      for (int c = 0; c < kNumClasses; ++c) {
        gui::ColorClassConfig cls;
        cls.color[0] = (c % 3 == 0) ? 1.0f : 0.0f;
        cls.color[1] = (c % 3 == 1) ? 1.0f : 0.0f;
        cls.color[2] = (c % 3 == 2) ? 1.0f : 0.0f;
        cls.combine = 0;
        cls.visible = true;
        cls.solo = false;
        gui::ColorClassRefConfig ref;
        ref.layer_idx = 0;
        ref.crystal_pool_id = 0;  // CrystalConfig::id_ = 1 in ResetTestState()
        ref.match_all = true;
        cls.match.push_back(ref);
        gui::g_state.raypath_color.push_back(cls);
      }

      // Small finite ray count so the GPU run finishes fast.
      gui::g_state.sim.infinite = false;
      gui::g_state.sim.ray_num_millions = 0.02f;

      const unsigned long baseline_uploads = gui::g_state.texture_upload_count;
      gui::DoRun(/*user_initiated=*/true);
      // Pump frames until the GPU produces a batch; SyncFromPoller polls the
      // async tally each Yield and fires the modal once it is populated.
      const bool ran = WaitForSimRestartAtLeast(ctx, baseline_uploads, /*timeout_ms=*/8000);
      IM_CHECK(ran);
      // Give SyncFromPoller a few extra ticks to observe the populated count.
      for (int i = 0; i < 20 && gui::PeekGuiWarning().empty(); ++i) {
        ctx->Yield();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }

      // Core AC: the async modal fired with a color-degrade message.
      const std::string warning = gui::PeekGuiWarning();
      IM_CHECK(!warning.empty());
      IM_CHECK(warning.find("color") != std::string::npos || warning.find("Color") != std::string::npos);
      IM_CHECK(warning.find("degraded") != std::string::npos);
      IM_CHECK(warning.find("color class") != std::string::npos);

      // Exact count lock: 20 - kMaxColorClassesDevice(16) = 4 dropped, via the
      // async poll path (component count stays 0 — this is not a predicate drop).
      LUMICE_ColorOverflowInfo color_over{};
      IM_CHECK(LUMICE_GetColorOverflowInfo(gui::g_server, &color_over) == LUMICE_OK);
      IM_CHECK(color_over.color_class_overflow_count == 4);
      IM_CHECK(color_over.component_overflow_count == 0);

      // R1 reset regression (task-366 class): switch to a CLEAN color config and
      // re-run; the tally must reset to 0 and the degrade modal must not re-fire.
      gui::ClearGuiWarning();
      gui::g_state.raypath_color.clear();
      const unsigned long baseline_uploads2 = gui::g_state.texture_upload_count;
      gui::DoRun(/*user_initiated=*/true);
      const bool ran2 = WaitForSimRestartAtLeast(ctx, baseline_uploads2, /*timeout_ms=*/8000);
      IM_CHECK(ran2);
      for (int i = 0; i < 20; ++i) {
        ctx->Yield();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      LUMICE_ColorOverflowInfo color_over2{};
      IM_CHECK(LUMICE_GetColorOverflowInfo(gui::g_server, &color_over2) == LUMICE_OK);
      IM_CHECK(color_over2.color_class_overflow_count == 0);
      IM_CHECK(gui::PeekGuiWarning().empty());

      // Cleanup.
      gui::ClearGuiWarning();
      gui::g_state.raypath_color.clear();
      gui::g_state.use_gpu_backend = false;
      gui::g_server_poller.Stop();
      LUMICE_StopServer(gui::g_server);
      LUMICE_DestroyServer(gui::g_server);
      gui::g_server = nullptr;
      gui::g_state.run_intent = gui::RunIntent::kNone;
      gui::g_state.committed_epoch = 0;
      gui::g_state.dirty = false;
    };
  }

  // task-gui-feedback-affordances code-review-01 Critical 1 regression anchor: a persistent
  // color-overflow condition (unlike the FillLumiceConfig-REJECT branch covered by the sibling
  // `overflow_auto_commit_dedup` test) goes through the commit-SUCCEEDED branch of DoRun. That
  // branch used to call ClearGuiWarning() unconditionally before checking for a color overflow,
  // which zeroed the identity-dedup state ahead of the comparison and made every auto-commit
  // tick (user_initiated=false) reopen the modal even though the SAME overflow persisted —
  // reproducing the "70ms slider drag freezes the UI" regression Step 2 fixed for the ABI-reject
  // branch. This test drives two auto-commit ticks against the same 66-predicate overflow setup
  // as `big_or_filter_with_color_overflow_surfaces_warning` and asserts the second tick does NOT
  // respawn the modal.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_filter_type", "color_overflow_auto_commit_dedup");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetTestState();
      gui::ClearGuiWarning();
      gui::g_server = LUMICE_CreateServer();
      IM_CHECK(gui::g_server != nullptr);

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
      IM_CHECK(!gui::PeekGuiWarning().empty());
      IM_CHECK(gui::IsGuiWarningPending());
      const std::string first_msg = gui::PeekGuiWarning();

      // Frame consumes OpenPopup; trigger cleared, message retained.
      gui::internal_test::ConsumeGuiWarningPending();
      IM_CHECK(!gui::IsGuiWarningPending());

      // Second auto-commit tick with the SAME overflow: dedup MUST hold (no modal respawn).
      gui::DoRun(/*user_initiated=*/false);
      IM_CHECK_STR_EQ(gui::PeekGuiWarning().c_str(), first_msg.c_str());
      IM_CHECK(!gui::IsGuiWarningPending());

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
    };
  }

  // task-gui-feedback-affordances Step 3 (AC4): the filter Edit modal live
  // preview must render its "Clauses: N / <limit>" line for both the normal
  // case and the overflow case (red-styled). This test opens the Edit modal,
  // types an over-cap row (4 × 9-alt = 6561 clauses > 4096), yields frames so
  // the immediate-mode preview runs — exercising the new PushStyleColor /
  // PopStyleColor branch — then Cancels out. The primary assertion is that no
  // ImGui style-stack assertion fires (branch balance is correct) and that
  // the modal remains navigable. The precise clause-count value is locked by
  // the sibling summarize_sop_expansion_delegates_to_commit_path unit test.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_filter_type", "live_preview_clause_count_overflow_no_crash");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      ctx->ItemClick("**/Edit##fi");
      ctx->Yield(4);

      // Non-overflow first: a simple row keeps the preview in the disabled-text
      // branch. Yield extra frames so ImGui commits at least one full render
      // pass through the preview code.
      ctx->ItemInputValue("**/##row_text_0", "3-5");
      ctx->Yield(3);

      // Now push into overflow territory: 4 factors × 9 alternatives each →
      // 6561 clauses, well above LUMICE_MAX_CONFIG_CLAUSES = 4096.
      ctx->ItemInputValue("**/##row_text_0",
                          "1;2;3;4;5;6;7;8;3-4 & 1;2;3;4;5;6;7;8;3-4 & 1;2;3;4;5;6;7;8;3-4 & 1;2;3;4;5;6;7;8;3-4");
      ctx->Yield(3);

      // The Edit modal is still open and its Cancel item is still clickable —
      // proving the preview render did not throw an ImGui assertion (which
      // would tear down the modal / test).
      IM_CHECK(ctx->ItemExists("**/" ICON_FA_XMARK " Cancel##edit_modal"));
      ctx->ItemClick("**/" ICON_FA_XMARK " Cancel##edit_modal");
      ctx->Yield(2);
    };
  }

  // task-gui-feedback-affordances Step 2 (AC3): a user-clicked Run always
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
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_filter_type", "overflow_user_initiated_run_refires");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetTestState();
      gui::ClearGuiWarning();
      gui::g_server = LUMICE_CreateServer();
      IM_CHECK(gui::g_server != nullptr);

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
      IM_CHECK(!gui::PeekGuiWarning().empty());
      IM_CHECK(gui::IsGuiWarningPending());
      const std::string first_msg = gui::PeekGuiWarning();

      // Simulate the frame that consumes OpenPopup (RenderGuiWarningPopup
      // clears the trigger after calling OpenPopup) without touching the
      // in-flight message — matches what happens after a real frame renders.
      gui::internal_test::ConsumeGuiWarningPending();
      IM_CHECK(!gui::PeekGuiWarning().empty());
      IM_CHECK(!gui::IsGuiWarningPending());

      // Second user Run with the same overflow: MUST re-open the modal.
      gui::DoRun(/*user_initiated=*/true);
      IM_CHECK_STR_EQ(gui::PeekGuiWarning().c_str(), first_msg.c_str());
      IM_CHECK(gui::IsGuiWarningPending());

      // Cleanup.
      gui::ClearGuiWarning();
      LUMICE_StopServer(gui::g_server);
      LUMICE_DestroyServer(gui::g_server);
      gui::g_server = nullptr;
    };
  }

  // task-gui-feedback-affordances Step 2 (AC3): main-loop auto-commit
  // (DoRun(user_initiated=false)) MUST preserve dedup so a persistent
  // overflow condition re-detected every 70ms tick does not respawn the
  // modal (which would freeze user interaction the moment a slider drag
  // pushes them into overflow). Same overflow setup as the sibling test.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_filter_type", "overflow_auto_commit_dedup");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetTestState();
      gui::ClearGuiWarning();
      gui::g_server = LUMICE_CreateServer();
      IM_CHECK(gui::g_server != nullptr);

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
      IM_CHECK(!gui::PeekGuiWarning().empty());
      IM_CHECK(gui::IsGuiWarningPending());

      // Frame consumes OpenPopup; trigger cleared, message retained.
      gui::internal_test::ConsumeGuiWarningPending();
      IM_CHECK(!gui::IsGuiWarningPending());

      // Second auto-commit tick with the SAME overflow: dedup MUST hold.
      gui::DoRun(/*user_initiated=*/false);
      IM_CHECK(!gui::IsGuiWarningPending());  // no modal respawn

      gui::ClearGuiWarning();
      LUMICE_StopServer(gui::g_server);
      LUMICE_DestroyServer(gui::g_server);
      gui::g_server = nullptr;
    };
  }
}
