#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>

#include "gui/log_sink.hpp"  // ImGuiLogSink (for log_panel_above_left_panel test sink injection)
// imgui_internal.h is generally an anti-pattern, but z-order assertions need
// direct ImGuiContext::Windows access. The relied-on semantics
// (BringWindowToDisplayFront splices to g.Windows back; creation with
// NoBringToFrontOnFocus does push_front = bottom) are documented in the
// convention block at the top of src/gui/app_panels.cpp. Any ImGui upgrade
// that alters either rule must update both that comment and the
// p1_layout / p1_edit_modal z-order assertions below.
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
      entry0.crystal.type = gui::CrystalType::kPyramid;
      entry0.crystal.prism_h = 2.0f;
      entry0.crystal.upper_h = 0.3f;
      entry0.crystal.lower_h = 0.4f;
      gui::g_state.sun.altitude = 30.0f;
      gui::g_state.sim.max_hits = 12;

      // Save
      const char* tmp_path = "/tmp/lumice_gui_test.lmc";
      bool save_ok = gui::SaveLmcFile(tmp_path, gui::g_state, gui::g_preview, false);
      IM_CHECK(save_ok);

      // Reset
      gui::DoNew();
      IM_CHECK_EQ(gui::g_state.layers[0].entries[0].crystal.type, gui::CrystalType::kPrism);

      // Load
      std::vector<unsigned char> tex_data;
      int tex_w = 0;
      int tex_h = 0;
      bool load_ok = gui::LoadLmcFile(tmp_path, gui::g_state, tex_data, tex_w, tex_h);
      IM_CHECK(load_ok);

      // Verify roundtrip
      auto& loaded_entry = gui::g_state.layers[0].entries[0];
      IM_CHECK_EQ(loaded_entry.crystal.type, gui::CrystalType::kPyramid);
      IM_CHECK_EQ(loaded_entry.crystal.prism_h, 2.0f);
      IM_CHECK_EQ(loaded_entry.crystal.upper_h, 0.3f);
      IM_CHECK_EQ(loaded_entry.crystal.lower_h, 0.4f);
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

      // Scene 1: kIdle — "Run" button should exist
      gui::g_state.sim_state = gui::GuiState::SimState::kIdle;
      ctx->Yield();
      IM_CHECK(ctx->ItemExists("##TopBar/Run"));

      // Scene 2: kSimulating — "Stop" button should exist
      gui::g_state.sim_state = gui::GuiState::SimState::kSimulating;
      ctx->Yield();
      IM_CHECK(ctx->ItemExists("##TopBar/Stop"));

      // Scene 3: kDone
      gui::g_state.sim_state = gui::GuiState::SimState::kDone;
      ctx->Yield();
      IM_CHECK(ctx->ItemExists("##TopBar/Run"));  // Back to Run

      // Scene 4: kModified — Revert button should appear
      gui::g_state.sim_state = gui::GuiState::SimState::kModified;
      ctx->Yield();
      IM_CHECK(ctx->ItemExists("##TopBar/Revert"));
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
      ctx->ItemClick("**/\xC3\x97##del_0_1");
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
      IM_CHECK(!gui::g_state.layers[0].entries[0].filter.has_value());

      // Programmatically set a filter so we can test clearing it
      gui::FilterConfig f;
      f.raypath_text = "3-1-5";
      gui::g_state.layers[0].entries[0].filter = f;

      // Verify filter is set
      IM_CHECK(gui::g_state.layers[0].entries[0].filter.has_value());

      // Open filter modal, click Remove (buffered) then OK to commit removal.
      ctx->ItemClick("**/Edit##fi");
      ctx->Yield(4);  // popup open + Filter tab activation (SetSelected first-frame)
      ctx->ItemClick("**/Remove Filter##filter");
      ctx->Yield(2);
      ctx->ItemClick("**/OK##edit_modal");
      ctx->Yield(2);

      // Verify filter is cleared
      IM_CHECK(!gui::g_state.layers[0].entries[0].filter.has_value());
    };
  }

  // P1: scrum-gui-polish-v13 161.2 — Remove Filter collapses to a plain edit
  // action (clear raypath; keep editability + sym_*). Observables:
  //   - Raypath InputText enabled after click (no more Staged "disabled while
  //     removed" banner)
  //   - Remove Filter button becomes disabled (derived from raypath now empty)
  //   - Filter tab dirty mark " *" lights up (AC#4)
  // sym_* preservation is covered by the dedicated re-type contract test.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_filter", "remove_clears_textbox_staged");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      gui::FilterConfig f;
      f.raypath_text = "3-1-5";
      gui::g_state.layers[0].entries[0].filter = f;

      ctx->ItemClick("**/Edit##fi");
      ctx->Yield(4);
      ctx->ItemClick("**/Remove Filter##filter");
      ctx->Yield(2);

      // InputText stays editable (no "disabled while pending remove" banner).
      auto rp_info = ctx->ItemInfo("**/Raypath##filter_modal");
      IM_CHECK((rp_info.ItemFlags & ImGuiItemFlags_Disabled) == 0);

      // Remove button itself becomes disabled (derived from raypath now empty)
      // — this is also proof the clear took effect.
      auto rm_info = ctx->ItemInfo("**/Remove Filter##filter");
      IM_CHECK((rm_info.ItemFlags & ImGuiItemFlags_Disabled) != 0);

      // AC#4: Filter tab label carries a trailing " *" dirty mark.
      auto tab_info = ctx->ItemInfo("**/###filter_tab");
      IM_CHECK(tab_info.ID != 0);
      IM_CHECK(std::strstr(tab_info.DebugLabel, "*") != nullptr);

      // Cancel so we don't leave state polluted for later tests.
      ctx->ItemClick("**/Cancel##edit_modal");
      ctx->Yield(2);
    };
  }

  // P1: scrum-gui-polish-v13 161.2 — Remove preserves sym_* across the buffer
  // cycle. Contract: Remove == backspace all chars → type same raypath again
  // → OK must write the filter back with sym_* identical to the Open-time
  // snapshot (not reset to defaults). Use a non-default sym_b=false so a
  // silent reset would be observable on entry.filter after OK.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_filter", "remove_preserves_sym_on_retype");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      gui::FilterConfig f;
      f.raypath_text = "3-1-5";
      f.sym_p = true;
      f.sym_b = false;  // <-- non-default; reset would flip it to true.
      f.sym_d = true;
      gui::g_state.layers[0].entries[0].filter = f;
      ctx->Yield();

      ctx->ItemClick("**/Edit##fi");
      ctx->Yield(4);
      ctx->ItemClick("**/Remove Filter##filter");
      ctx->Yield(2);
      ctx->ItemInputValue("**/Raypath##filter_modal", "3-1-5");
      ctx->Yield(2);
      ctx->ItemClick("**/OK##edit_modal");
      ctx->Yield(2);

      IM_CHECK(gui::g_state.layers[0].entries[0].filter.has_value());
      IM_CHECK_STR_EQ(gui::g_state.layers[0].entries[0].filter->raypath_text.c_str(), "3-1-5");
      IM_CHECK(gui::g_state.layers[0].entries[0].filter->sym_p == true);
      IM_CHECK(gui::g_state.layers[0].entries[0].filter->sym_b == false);
      IM_CHECK(gui::g_state.layers[0].entries[0].filter->sym_d == true);
    };
  }

  // P1: scrum-gui-polish-v13 161.2 — Staged Remove + OK / Cancel path contract
  // test (AC#5 Cancel branch): Remove then Cancel must restore the filter
  // (including raypath & sym_*) from the entry — buffer-discard semantics.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_filter", "remove_then_cancel_restores_filter");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      gui::FilterConfig f;
      f.raypath_text = "3-1-5";
      f.sym_p = true;
      f.sym_b = false;
      f.sym_d = true;
      gui::g_state.layers[0].entries[0].filter = f;
      ctx->Yield();

      ctx->ItemClick("**/Edit##fi");
      ctx->Yield(4);
      ctx->ItemClick("**/Remove Filter##filter");
      ctx->Yield(2);
      ctx->ItemClick("**/Cancel##edit_modal");
      ctx->Yield(4);

      IM_CHECK(gui::g_state.layers[0].entries[0].filter.has_value());
      IM_CHECK_STR_EQ(gui::g_state.layers[0].entries[0].filter->raypath_text.c_str(), "3-1-5");
      IM_CHECK(gui::g_state.layers[0].entries[0].filter->sym_p == true);
      IM_CHECK(gui::g_state.layers[0].entries[0].filter->sym_b == false);
      IM_CHECK(gui::g_state.layers[0].entries[0].filter->sym_d == true);
    };
  }

  // P1: scrum-gui-polish-v13 161.2 — After Remove the user can type a new
  // raypath; OK commits the new value (the cleared buffer did not leave the
  // tab in a terminal "removed" state).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_filter", "remove_then_retype_commits_new_raypath");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      gui::FilterConfig f;
      f.raypath_text = "1-3";
      gui::g_state.layers[0].entries[0].filter = f;
      ctx->Yield();

      ctx->ItemClick("**/Edit##fi");
      ctx->Yield(4);
      ctx->ItemClick("**/Remove Filter##filter");
      ctx->Yield(2);
      ctx->ItemInputValue("**/Raypath##filter_modal", "3-1-5");
      ctx->Yield(2);
      ctx->ItemClick("**/OK##edit_modal");
      ctx->Yield(2);

      IM_CHECK(gui::g_state.layers[0].entries[0].filter.has_value());
      IM_CHECK_STR_EQ(gui::g_state.layers[0].entries[0].filter->raypath_text.c_str(), "3-1-5");
    };
  }

  // P1: scrum-gui-polish-v13 161.2 — Remove button is disabled when the
  // raypath textbox is empty; typing through the InputText widget re-enables
  // it. Test must NOT write g_raypath_buf directly — we control the derived
  // state via the real ImGui widget to avoid tautological assertions.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_filter", "remove_button_disabled_when_empty");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Entry has no filter initially → raypath empty → Remove disabled.
      IM_CHECK(!gui::g_state.layers[0].entries[0].filter.has_value());

      ctx->ItemClick("**/Edit##fi");
      ctx->Yield(4);

      auto info_empty = ctx->ItemInfo("**/Remove Filter##filter");
      IM_CHECK((info_empty.ItemFlags & ImGuiItemFlags_Disabled) != 0);

      // Type through the real InputText widget; Remove must enable next frame.
      ctx->ItemInputValue("**/Raypath##filter_modal", "3");
      ctx->Yield(1);

      auto info_filled = ctx->ItemInfo("**/Remove Filter##filter");
      IM_CHECK((info_filled.ItemFlags & ImGuiItemFlags_Disabled) == 0);

      ctx->ItemClick("**/Cancel##edit_modal");
      ctx->Yield(2);
    };
  }

  // P1: scrum-gui-polish-v13 161.2 — In Immediate mode, Remove applies on the
  // next frame (ApplyBuffersToEntry sees empty raypath and writes nullopt).
  // Dual observable: entry.filter == nullopt (direct) + intensity_locked
  // (MarkFilterDirty side effect at gui_state.hpp:264-268). Pre-condition
  // asserts intensity_locked starts false to avoid tautologies.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_filter", "immediate_remove_applies_next_frame");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      gui::FilterConfig f;
      f.raypath_text = "3-1-5";
      gui::g_state.layers[0].entries[0].filter = f;
      gui::g_state.intensity_locked = false;
      ctx->Yield();

      // Pre-condition guard: MarkFilterDirty has not fired yet.
      IM_CHECK(gui::g_state.intensity_locked == false);

      ctx->ItemClick("**/Edit##fi");
      ctx->Yield(4);
      // Enter Immediate mode (close+reopen cycle).
      ctx->ItemClick("**/Immediate##edit_modal");
      ctx->Yield(6);
      ctx->ItemClick("**/###filter_tab");
      ctx->Yield(2);

      ctx->ItemClick("**/Remove Filter##filter");
      ctx->Yield(2);

      IM_CHECK(!gui::g_state.layers[0].entries[0].filter.has_value());
      IM_CHECK(gui::g_state.intensity_locked == true);

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
      const float orig_h = gui::g_state.layers[0].entries[0].crystal.height;

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
      ctx->ItemClick("**/Cancel##edit_modal");
      ctx->Yield(2);
      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(4);
      IM_CHECK(!is_dirty("**/###crystal_tab"));

      // AC #4: OK commits new baseline; reopening shows no marker.
      ctx->ItemInputValue("**/##Height##modal_cr_input", orig_h + 3.0f);
      ctx->Yield(2);
      ctx->ItemClick("**/OK##edit_modal");
      ctx->Yield(2);
      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(4);
      IM_CHECK(!is_dirty("**/###crystal_tab"));
      ctx->ItemClick("**/Cancel##edit_modal");
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
      ctx->ItemClick("**/Cancel##edit_modal");
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

      ctx->ItemClick("**/Cancel##edit_modal");
      ctx->Yield(2);
    };
  }

  // P1: Edit modal OK without any change must NOT clear the rendered preview
  // or arm Revert. Regression guard for scrum-gui-polish-v7 152.2: previously
  // CommitAllBuffers unconditionally MarkFilterDirty()'d after any OK,
  // wiping snapshot_intensity + flipping sim_state to kModified.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_edit", "ok_no_change_preserves_state");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Simulate "finite rays just finished": sim_state == kDone with a
      // non-zero snapshot intensity that the diff-gate must preserve.
      gui::g_state.sim_state = gui::GuiState::SimState::kDone;
      gui::g_state.snapshot_intensity = 0.5f;
      gui::g_state.intensity_locked = false;
      gui::g_state.dirty = false;
      ctx->Yield();

      // Open Edit Entry modal and immediately click OK (no field touched).
      ctx->ItemClick("**/Edit##fi");
      ctx->Yield(4);
      ctx->ItemClick("**/OK##edit_modal");
      ctx->Yield(2);

      // Render-invalidation must not have fired:
      //   - sim_state still kDone (no kModified ⇒ Revert button stays hidden)
      //   - snapshot_intensity preserved (>0 rather than ==0.5 so future
      //     normalization changes don't break the semantic assertion)
      //   - intensity_locked still false (no upload lock armed)
      //   - dirty still false (no spurious unsaved-changes flag)
      IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(gui::GuiState::SimState::kDone));
      IM_CHECK_GT(gui::g_state.snapshot_intensity, 0.0f);
      IM_CHECK(!gui::g_state.intensity_locked);
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
      f.raypath_text = "3-1-5";
      gui::g_state.layers[0].entries[0].filter = f;
      // "finite rays just finished" snapshot state.
      gui::g_state.sim_state = gui::GuiState::SimState::kDone;
      gui::g_state.snapshot_intensity = 0.5f;
      gui::g_state.intensity_locked = false;
      gui::g_state.dirty = false;
      ctx->Yield();

      ctx->ItemClick("**/Edit##fi");
      ctx->Yield(4);
      ctx->ItemClick("**/Remove Filter##filter");
      ctx->Yield(2);
      ctx->ItemClick("**/OK##edit_modal");
      ctx->Yield(2);

      // Render-invalidation MUST have fired (all four effects):
      IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(gui::GuiState::SimState::kModified));
      IM_CHECK_EQ(gui::g_state.snapshot_intensity, 0.0f);
      IM_CHECK(gui::g_state.intensity_locked);
      IM_CHECK(gui::g_state.dirty);
    };
  }

  // P1: scrum-gui-polish-v11 / task-modal-immediate-mode M2 gate — in
  // Immediate mode, crystal-only buffer edits must MarkDirty but NOT
  // MarkFilterDirty. This is what keeps infinite-rays accumulation alive
  // while the user drags a Crystal slider. Filter edits still fire
  // MarkFilterDirty (identical to Staged OK semantics for filter changes).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_edit_modal", "immediate_crystal_does_not_clear_display");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      gui::g_state.modal_immediate_mode = false;
      ctx->Yield(2);

      // Seed "finite rays just finished" state — an accumulated preview the
      // Immediate mode is supposed to preserve while the user tweaks Crystal.
      gui::g_state.sim_state = gui::GuiState::SimState::kDone;
      gui::g_state.snapshot_intensity = 0.5f;
      gui::g_state.intensity_locked = false;
      gui::g_state.dirty = false;
      ctx->Yield();

      const float orig_h = gui::g_state.layers[0].entries[0].crystal.height;

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
      // MarkDirty (state.dirty == true) but must NOT MarkFilterDirty
      // (snapshot_intensity preserved, intensity_locked still false).
      ctx->ItemInputValue("**/##Height##modal_cr_input", orig_h + 1.0f);
      ctx->Yield(2);
      IM_CHECK(gui::g_state.dirty);
      IM_CHECK_GT(gui::g_state.snapshot_intensity, 0.0f);
      IM_CHECK(!gui::g_state.intensity_locked);

      // Filter edit: switch to the Filter tab and type a raypath. Because the
      // entry had no filter at modal open (initial_present=false), the first
      // raypath edit is itself the filter creation — filter_changed == true
      // in ApplyBuffersToEntry → MarkFilterDirty fires → display clears.
      ctx->ItemClick("**/###filter_tab");
      ctx->Yield(4);
      ctx->ItemInputValue("**/Raypath##filter_modal", "3-1-5");
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.snapshot_intensity, 0.0f);
      IM_CHECK(gui::g_state.intensity_locked);

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
      ctx->ItemInputValue("**/Raypath##filter_modal", "3-1");
      ctx->Yield(2);
      IM_CHECK(gui::g_state.layers[0].entries[0].filter.has_value());
      IM_CHECK_STR_EQ(gui::g_state.layers[0].entries[0].filter->raypath_text.c_str(), "3-1");

      // kInvalid: pure non-numeric "abc" — syntactically invalid.
      ctx->ItemInputValue("**/Raypath##filter_modal", "abc");
      ctx->Yield(2);
      // L1: Remove Filter is disabled iff raypath textbox is empty — its gate
      //     (edit_modals.cpp: `raypath_empty = (g_raypath_buf[0] == '\0')`)
      //     is buffer-driven, independent of entry.filter model state. An
      //     enabled Remove Filter thus proves g_raypath_buf actually holds
      //     "abc" and rules out a silent ItemInputValue no-op (without which
      //     L2/L3 could reduce to a tautology by simply re-reading the last
      //     commit).
      {
        auto info_rm = ctx->ItemInfo("**/Remove Filter##filter");
        IM_CHECK((info_rm.ItemFlags & ImGuiItemFlags_Disabled) == 0);
      }
      // L2 + L3: guard intercepted — entry.filter preserved at last valid.
      IM_CHECK(gui::g_state.layers[0].entries[0].filter.has_value());
      IM_CHECK_STR_EQ(gui::g_state.layers[0].entries[0].filter->raypath_text.c_str(), "3-1");

      // kIncomplete: trailing separator "3-" — syntactically incomplete per
      // ValidateRaypathText rules. Guard treats same as kInvalid (Staged OK
      // disjunction `v.state != kValid`). Apply the same three-layer pattern
      // symmetrically so this subcase cannot degrade into a tautology when
      // ItemInputValue silently no-ops (which would leave the previous "abc"
      // → same guarded state as "3-", making L2/L3 trivially pass).
      ctx->ItemInputValue("**/Raypath##filter_modal", "3-");
      ctx->Yield(2);
      {
        auto info_rm = ctx->ItemInfo("**/Remove Filter##filter");
        IM_CHECK((info_rm.ItemFlags & ImGuiItemFlags_Disabled) == 0);
      }
      IM_CHECK(gui::g_state.layers[0].entries[0].filter.has_value());
      IM_CHECK_STR_EQ(gui::g_state.layers[0].entries[0].filter->raypath_text.c_str(), "3-1");

      // Valid recovery: a fresh valid raypath must commit normally, proving
      // the guard does not wedge the entry permanently after a rejection.
      ctx->ItemInputValue("**/Raypath##filter_modal", "3-1-5");
      ctx->Yield(2);
      IM_CHECK(gui::g_state.layers[0].entries[0].filter.has_value());
      IM_CHECK_STR_EQ(gui::g_state.layers[0].entries[0].filter->raypath_text.c_str(), "3-1-5");

      // Empty → nullopt (161.2 rule unchanged by this task).
      ctx->ItemInputValue("**/Raypath##filter_modal", "");
      ctx->Yield(2);
      IM_CHECK(!gui::g_state.layers[0].entries[0].filter.has_value());

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
      const float orig_h = gui::g_state.layers[0].entries[0].crystal.height;

      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(4);

      // Change Height — must reach the entry on the same frame (Immediate
      // commits every frame via CommitAllBuffersImmediate).
      ctx->ItemInputValue("**/##Height##modal_cr_input", orig_h + 2.5f);
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.layers[0].entries[0].crystal.height, orig_h + 2.5f);

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
      const float orig_h = gui::g_state.layers[0].entries[0].crystal.height;

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
      const float orig_h = gui::g_state.layers[0].entries[0].crystal.height;

      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(4);

      ctx->ItemInputValue("**/##Height##modal_cr_input", orig_h + 4.0f);
      ctx->Yield(2);
      ctx->ItemClick("**/Close##edit_modal");
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.layers[0].entries[0].crystal.height, orig_h + 4.0f);

      // Reopen to confirm the value truly persisted through close (guards
      // against any path that might silently revert buffer→entry mapping).
      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(4);
      IM_CHECK_EQ(gui::g_state.layers[0].entries[0].crystal.height, orig_h + 4.0f);
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
      const float orig_h = gui::g_state.layers[0].entries[0].crystal.height;

      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(4);

      ctx->ItemInputValue("**/##Height##modal_cr_input", orig_h + 5.0f);
      ctx->Yield(2);

      // ImGui's default popup handling closes the top popup on Escape.
      ctx->KeyPress(ImGuiKey_Escape);
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.layers[0].entries[0].crystal.height, orig_h + 5.0f);

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
      ctx->ItemInputValue("**/Raypath##filter_modal", "3-1-5");
      ctx->Yield(2);
      IM_CHECK(!gui::g_state.layers[0].entries[0].filter.has_value());

      // Toggle Immediate: switch commits the buffer via CommitAllBuffersImmediate,
      // then close+reopen preserves tab selection + buffer.
      ctx->ItemClick("**/Immediate##edit_modal");
      ctx->Yield(8);

      // Filter is now populated on the entry (committed by the switch).
      IM_CHECK(gui::g_state.layers[0].entries[0].filter.has_value());
      IM_CHECK_STR_EQ(gui::g_state.layers[0].entries[0].filter->raypath_text.c_str(), "3-1-5");
      // Filter tab controls still accessible — tab selection survived.
      IM_CHECK(ctx->ItemExists("**/Raypath##filter_modal"));

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
      gui::g_state.sim_state = gui::GuiState::SimState::kIdle;
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
      gui::g_state.sim_state = gui::GuiState::SimState::kSimulating;
      ctx->Yield();
      ctx->ItemClick("##TopBar/Save");
      ctx->Yield(2);

      auto copy_info2 = ctx->ItemInfo("**/Save Copy");
      IM_CHECK((copy_info2.ItemFlags & ImGuiItemFlags_Disabled) != 0);
      auto cfg_info = ctx->ItemInfo("**/Config JSON...");
      IM_CHECK((cfg_info.ItemFlags & ImGuiItemFlags_Disabled) == 0);

      ctx->KeyPress(ImGuiKey_Escape);
      ctx->Yield(2);
      gui::g_state.sim_state = gui::GuiState::SimState::kIdle;
      ctx->Yield();
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
      ctx->ItemClick("**/x##layer_1");
      ctx->Yield();
      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers.size()), 1);
    };
  }

  // p1_card/entry_copy — copy an entry card
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_card", "entry_copy");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Modify the default entry's crystal type
      gui::g_state.layers[0].entries[0].crystal.type = gui::CrystalType::kPyramid;
      gui::g_state.layers[0].entries[0].crystal.prism_h = 2.5f;
      ctx->Yield();

      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers[0].entries.size()), 1);

      // Duplicate the entry (hover button; ID is always addressable even at alpha=0).
      ctx->ItemClick("**/D##dup_0_0");
      ctx->Yield();
      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers[0].entries.size()), 2);

      // Verify the copy has same crystal params
      auto& copy = gui::g_state.layers[0].entries[1];
      IM_CHECK_EQ(copy.crystal.type, gui::CrystalType::kPyramid);
      IM_CHECK_EQ(copy.crystal.prism_h, 2.5f);
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

      IM_CHECK_EQ(gui::g_state.layers[0].entries[0].crystal.type, gui::CrystalType::kPrism);

      // Open crystal modal
      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(3);

      // Write 0 via the input widget — should be clamped to min (0.01)
      ctx->ItemInputValue("**/##Height##modal_cr_input", 0.0f);
      ctx->Yield();

      // Click OK to commit
      ctx->ItemClick("**/OK##edit_modal");
      ctx->Yield(2);

      // Height should be clamped to >= 0.01 (kLog with min=0.01)
      IM_CHECK_GE(gui::g_state.layers[0].entries[0].crystal.height, 0.01f - 1e-6f);
    };
  }

  // p1_slider/pyramid_h_allows_zero_via_modal — Upper H and Lower H allow 0
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_slider", "pyramid_h_allows_zero_via_modal");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Switch to Pyramid type programmatically
      gui::g_state.layers[0].entries[0].crystal.type = gui::CrystalType::kPyramid;
      ctx->Yield();

      // Open crystal modal
      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(3);

      // Write 0 to Upper H — should be allowed (min=0.0 for kLinear)
      ctx->ItemInputValue("**/##Upper H##modal_cr_input", 0.0f);
      ctx->Yield();

      // Click OK
      ctx->ItemClick("**/OK##edit_modal");
      ctx->Yield(2);

      IM_CHECK_EQ(gui::g_state.layers[0].entries[0].crystal.upper_h, 0.0f);
    };
  }

  // p1_slider/pyramid_h_clamp_upper_via_modal — Upper H clamped at new [0,1] upper bound
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_slider", "pyramid_h_clamp_upper_via_modal");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      gui::g_state.layers[0].entries[0].crystal.type = gui::CrystalType::kPyramid;
      ctx->Yield();

      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(3);

      // Write 1.5 — should be clamped to 1.0 (kLinear with max=1.0)
      ctx->ItemInputValue("**/##Upper H##modal_cr_input", 1.5f);
      ctx->Yield();

      ctx->ItemClick("**/OK##edit_modal");
      ctx->Yield(2);

      IM_CHECK_EQ(gui::g_state.layers[0].entries[0].crystal.upper_h, 1.0f);
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

      gui::g_state.layers[0].entries[0].crystal.type = gui::CrystalType::kPyramid;
      ctx->Yield();

      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(3);

      ctx->ItemInputValue("**/##Upper H##modal_cr_input", 0.5f);
      ctx->Yield();

      ctx->ItemClick("**/OK##edit_modal");
      ctx->Yield(2);

      IM_CHECK_EQ(gui::g_state.layers[0].entries[0].crystal.upper_h, 0.5f);
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
      gui::g_state.layers[0].entries[0].crystal.height = 2.5f;
      gui::g_state.dirty = true;
      gui::DoRun();

      // Assert: a new texture upload happens within timeout
      bool ok = WaitForSimRestartAtLeast(ctx, baseline, 1500);

      StopPerfSimulation();
      IM_CHECK(ok);
    };
  }

  // p1_running/filter_change_triggers_restart (extension)
  // Filter changes call MarkFilterDirty() which sets intensity_locked=true to block uploads
  // until the CommitConfig restart completes. DoRun triggers the restart which unlocks.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_running", "filter_change_triggers_restart");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();

      // Pre-register a filter on the first entry so StartPerfSimulation's DoRun commits with it
      gui::FilterConfig f;
      f.raypath_text = "3-1-5";
      gui::g_state.layers[0].entries[0].filter = f;

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
      gui::g_state.layers[0].entries[0].filter->raypath_text = "3-1-5-7";
      gui::g_state.MarkFilterDirty();
      gui::DoRun();

      // Filter changes may take slightly longer (intensity_locked delay); use 2000ms
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
      gui::DoRun();

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
      gui::g_state.layers[0].entries[0].crystal.type = gui::CrystalType::kPyramid;
      gui::g_state.layers[0].entries[0].crystal.prism_h = 3.5f;
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
      IM_CHECK_EQ(loaded.layers[0].entries[0].crystal.type, gui::CrystalType::kPyramid);
      IM_CHECK_EQ(loaded.layers[0].entries[0].crystal.prism_h, 3.5f);

      std::remove(tmp_path);
    };
  }

  // p2_modal/crystal_modal_open_cancel — open crystal modal, cancel, verify no state change
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_modal", "crystal_modal_open_cancel");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      auto& cr = gui::g_state.layers[0].entries[0].crystal;
      auto type_before = cr.type;
      float height_before = cr.height;

      // Click Edit Crystal button on the first entry card
      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(3);

      // Click Cancel in the modal
      ctx->ItemClick("**/Cancel##edit_modal");
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
      float initial_height = gui::g_state.layers[0].entries[0].crystal.height;

      // Open crystal modal
      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(3);

      // Modify height via the input widget in the modal
      ctx->ItemInputValue("**/##Height##modal_cr_input", 5.0f);
      ctx->Yield();

      // Click OK to commit buffer to state
      ctx->ItemClick("**/OK##edit_modal");
      ctx->Yield(2);

      // Height should be updated to 5.0
      IM_CHECK_EQ(gui::g_state.layers[0].entries[0].crystal.height, 5.0f);
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
      ctx->ItemClick("**/OK##edit_modal");
      ctx->Yield(2);

      auto& entry = gui::g_state.layers[0].entries[0];
      IM_CHECK_EQ(entry.crystal.height, 5.0f);
      gui::AxisDist axis_zenith_baseline = entry.crystal.zenith;
      gui::AxisDist axis_azimuth_baseline = entry.crystal.azimuth;
      gui::AxisDist axis_roll_baseline = entry.crystal.roll;
      std::string name_baseline = entry.crystal.name;
      gui::CrystalType type_baseline = entry.crystal.type;

      // Step 2: open modal again, modify height to another non-default (2.0),
      // click Reset All, then OK to commit the reset.
      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(3);
      ctx->ItemInputValue("**/##Height##modal_cr_input", 2.0f);
      ctx->Yield();
      ctx->ItemClick("**/Reset All##modal_cr");
      ctx->Yield();
      ctx->ItemClick("**/OK##edit_modal");
      ctx->Yield(2);

      // Step 3: assert all 7 shape fields back to defaults.
      gui::CrystalConfig defaults;
      IM_CHECK_EQ(entry.crystal.height, defaults.height);
      IM_CHECK_EQ(entry.crystal.prism_h, defaults.prism_h);
      IM_CHECK_EQ(entry.crystal.upper_h, defaults.upper_h);
      IM_CHECK_EQ(entry.crystal.lower_h, defaults.lower_h);
      IM_CHECK_EQ(entry.crystal.upper_alpha, defaults.upper_alpha);
      IM_CHECK_EQ(entry.crystal.lower_alpha, defaults.lower_alpha);
      for (int i = 0; i < 6; ++i) {
        IM_CHECK_EQ(entry.crystal.face_distance[i], defaults.face_distance[i]);
      }

      // Step 4: assert name/type/axis preserved (Reset All must not touch them).
      IM_CHECK_EQ(entry.crystal.name, name_baseline);
      IM_CHECK_EQ(entry.crystal.type, type_baseline);
      IM_CHECK(entry.crystal.zenith == axis_zenith_baseline);
      IM_CHECK(entry.crystal.azimuth == axis_azimuth_baseline);
      IM_CHECK(entry.crystal.roll == axis_roll_baseline);
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
      ctx->ItemClick("**/OK##edit_modal");
      ctx->Yield(2);

      auto& entry = gui::g_state.layers[0].entries[0];
      IM_CHECK_EQ(entry.crystal.height, 5.0f);

      // Open modal, Reset All, Cancel — entry must remain at 5.0.
      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(3);
      ctx->ItemClick("**/Reset All##modal_cr");
      ctx->Yield();
      ctx->ItemClick("**/Cancel##edit_modal");
      ctx->Yield(2);

      // Entry unchanged: Cancel discarded the Reset All effect from the buffer.
      IM_CHECK_EQ(entry.crystal.height, 5.0f);
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
      IM_CHECK(!gui::g_state.layers[0].entries[0].filter.has_value());

      // Open filter modal, click OK immediately without modifying anything.
      ctx->ItemClick("**/Edit##fi");
      ctx->Yield(3);
      ctx->ItemClick("**/OK##edit_modal");
      ctx->Yield(2);

      // Filter must still be nullopt.
      IM_CHECK(!gui::g_state.layers[0].entries[0].filter.has_value());
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
      IM_CHECK(!gui::g_state.layers[0].entries[0].filter.has_value());

      // Open filter modal, type a raypath (touches g_filter_buf via g_raypath_buf
      // sync in CommitAllBuffers), click OK.
      ctx->ItemClick("**/Edit##fi");
      ctx->Yield(3);
      ctx->ItemInputValue("**/Raypath##filter_modal", "1-3");
      ctx->Yield();
      ctx->ItemClick("**/OK##edit_modal");
      ctx->Yield(2);

      // Filter must now exist with the typed raypath.
      IM_CHECK(gui::g_state.layers[0].entries[0].filter.has_value());
      IM_CHECK_EQ(gui::g_state.layers[0].entries[0].filter->raypath_text, std::string("1-3"));
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

        auto& cr = gui::g_state.layers[0].entries[0].crystal;
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
        ctx->ItemClick("**/Cancel##edit_modal");
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

        ctx->ItemClick("**/Cancel##edit_modal");
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

      ctx->ItemClick("**/Cancel##edit_modal");
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
}
