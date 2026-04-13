#include <chrono>
#include <cstdio>
#include <thread>

#include "test_gui_shared.hpp"

// ========== Helpers for interaction tests ==========

// ========== Helpers for task-test-gui-interaction ==========

// Polls gui::g_state.texture_upload_count until it reaches baseline + 1 (or higher).
// Returns true on success, false on timeout.
//
// IMPORTANT: caller must NOT call ctx->Yield() between reading baseline and calling this
// helper; otherwise pending SyncFromPoller updates may be consumed concurrently, invalidating
// the baseline. Keep the Arrange → baseline capture → Act (dirty + DoRun) → wait sequence
// tight within a single test body.
//
// Uses ctx->Yield() (frame-rate limited, typically 16ms/frame) + 10ms explicit sleep between
// checks, giving robust detection on both macOS (1ms timer) and Windows (1ms with
// timeBeginPeriod, which the test main() sets). If a third call site appears, migrate to
// test_helpers.hpp.
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

// Triggers the two-stage delete for a referenced crystal via the Delete Crystal? modal.
// Caller must have set state.selected_crystal to the target crystal index.
static void ClickDeleteCrystalViaModal(ImGuiTestContext* ctx) {
  ctx->ItemClick("**/Del##crystal");
  ctx->Yield(2);
  ctx->ItemClick("Delete Crystal?/Delete");
  ctx->Yield();
}

// Opens the Delete Crystal? modal and clicks Cancel, leaving state unchanged.
static void ClickDeleteCrystalCancelModal(ImGuiTestContext* ctx) {
  ctx->ItemClick("**/Del##crystal");
  ctx->Yield(2);
  ctx->ItemClick("Delete Crystal?/Cancel");
  ctx->Yield();
}

// Triggers the two-stage delete for a referenced filter via the Delete Filter? modal.
// Caller must have set state.selected_filter to the target filter index.
static void ClickDeleteFilterViaModal(ImGuiTestContext* ctx) {
  ctx->ItemClick("**/Del##filter");
  ctx->Yield(2);
  ctx->ItemClick("Delete Filter?/Delete");
  ctx->Yield();
}

// P0 tests
void RegisterP0Tests(ImGuiTestEngine* engine) {
  // P0: New
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p0_file", "new");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      ResetTestState();

      // Modify state: add crystal, set dirty
      gui::CrystalConfig c;
      c.id = gui::g_state.next_crystal_id++;
      gui::g_state.crystals.push_back(c);
      gui::g_state.dirty = true;

      IM_CHECK_EQ(static_cast<int>(gui::g_state.crystals.size()), 2);
      IM_CHECK_EQ(gui::g_state.dirty, true);

      // DoNew resets
      gui::DoNew();

      IM_CHECK_EQ(static_cast<int>(gui::g_state.crystals.size()), 1);
      IM_CHECK_EQ(gui::g_state.dirty, false);
      IM_CHECK_EQ(gui::g_state.selected_crystal, 0);
      IM_CHECK_EQ(gui::g_preview.HasTexture(), false);
    };
  }

  // P0: Save/Open roundtrip
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p0_file", "save_open_roundtrip");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      ResetTestState();

      // Modify state
      gui::g_state.crystals[0].type = gui::CrystalType::kPyramid;
      gui::g_state.crystals[0].prism_h = 2.0f;
      gui::g_state.crystals[0].upper_h = 0.3f;
      gui::g_state.crystals[0].lower_h = 0.4f;
      gui::g_state.sun.altitude = 30.0f;
      gui::g_state.sim.max_hits = 12;

      // Save
      const char* tmp_path = "/tmp/lumice_gui_test.lmc";
      bool save_ok = gui::SaveLmcFile(tmp_path, gui::g_state, gui::g_preview, false);
      IM_CHECK(save_ok);

      // Reset
      gui::DoNew();
      IM_CHECK_EQ(gui::g_state.crystals[0].type, gui::CrystalType::kPrism);

      // Load
      std::vector<unsigned char> tex_data;
      int tex_w = 0;
      int tex_h = 0;
      bool load_ok = gui::LoadLmcFile(tmp_path, gui::g_state, tex_data, tex_w, tex_h);
      IM_CHECK(load_ok);

      // Verify roundtrip
      IM_CHECK_EQ(gui::g_state.crystals[0].type, gui::CrystalType::kPyramid);
      IM_CHECK_EQ(gui::g_state.crystals[0].prism_h, 2.0f);
      IM_CHECK_EQ(gui::g_state.crystals[0].upper_h, 0.3f);
      IM_CHECK_EQ(gui::g_state.crystals[0].lower_h, 0.4f);
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
  // P1: Crystal Add/Delete
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_crystal", "add_delete");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);  // Let GUI render

      // Verify initial state: 1 crystal
      IM_CHECK_EQ(static_cast<int>(gui::g_state.crystals.size()), 1);

      // Scenario A: Add a crystal, then delete the new (unreferenced) one
      ctx->ItemClick("**/Add##crystal");
      IM_CHECK_EQ(static_cast<int>(gui::g_state.crystals.size()), 2);
      IM_CHECK_EQ(gui::g_state.selected_crystal, 1);

      // Delete the new crystal (not referenced by scattering) — should delete directly
      ctx->ItemClick("**/Del##crystal");
      IM_CHECK_EQ(static_cast<int>(gui::g_state.crystals.size()), 1);

      // Scenario B: Try to delete the referenced crystal
      // Need at least 2 crystals for Del to be enabled
      ctx->ItemClick("**/Add##crystal");
      IM_CHECK_EQ(static_cast<int>(gui::g_state.crystals.size()), 2);

      // Select the first crystal (referenced by scattering)
      gui::g_state.selected_crystal = 0;
      ctx->Yield();

      // Click Del — should open confirmation popup
      ctx->ItemClick("**/Del##crystal");
      ctx->Yield(2);

      // Click Delete in the popup to confirm
      ctx->ItemClick("Delete Crystal?/Delete");
      IM_CHECK_EQ(static_cast<int>(gui::g_state.crystals.size()), 1);
    };
  }

  // P1: Filter Add/Delete
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_filter", "add_delete");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Verify initial: no filters
      IM_CHECK_EQ(static_cast<int>(gui::g_state.filters.size()), 0);

      // Switch to Filter tab
      ctx->ItemClick("##LeftPanel/ConfigTabs/Filter");
      ctx->Yield();

      // Add a filter
      ctx->ItemClick("**/Add##filter");
      IM_CHECK_EQ(static_cast<int>(gui::g_state.filters.size()), 1);
      IM_CHECK_EQ(gui::g_state.selected_filter, 0);

      // Delete the filter (unreferenced) — direct delete
      ctx->ItemClick("**/Del##filter");
      IM_CHECK_EQ(static_cast<int>(gui::g_state.filters.size()), 0);
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
      IM_CHECK_EQ(static_cast<int>(gui::g_state.crystals.size()), 1);
    };
  }

  // P1: Mouse wheel over crystal preview should zoom only, not scroll parent panel
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_crystal", "preview_scroll_isolation");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Ensure Crystal tab is selected (previous tests may have switched tabs)
      ctx->ItemClick("##LeftPanel/ConfigTabs/Crystal");
      ctx->Yield(3);  // Let layout settle: tab switch → mesh rebuild → FBO render

      ImGuiWindow* panel = ctx->GetWindowByRef("##LeftPanel");
      IM_CHECK(panel != nullptr);

      FILE* diag = fopen("/tmp/lumice_scroll_test.log", "w");

      // Default state: scroll at top (0). Panel should have scrollable content.
      float scroll_max = panel->ScrollMax.y;
      if (diag) {
        fprintf(diag, "scroll_max=%.1f content=%.1f panel_h=%.1f scroll=%.1f\n", scroll_max, panel->ContentSize.y,
                panel->Size.y, panel->Scroll.y);
      }
      IM_CHECK(scroll_max > 0.0f);  // Panel content must overflow for this test

      // Panel scroll is at 0 (top), so scrolling DOWN is possible.
      // The 3D Preview is partially visible near the bottom of the visible area.
      float scroll_before = panel->Scroll.y;
      float zoom_before = gui::g_crystal_zoom;

      // Move mouse into the visible portion of the crystal preview.
      // Preview starts at ~(content_h - preview_size) from content top.
      // At scroll=0, visible range is [0, panel_h]. Preview top is at
      // content_h - scroll_max - preview_size, approximately.
      // Use 85% of visible panel height to target the preview area.
      ImVec2 panel_pos = panel->Pos;
      float panel_w = panel->Size.x;
      ImVec2 preview_center(panel_pos.x + panel_w * 0.5f, panel_pos.y + panel->Size.y * 0.85f);
      if (diag) {
        fprintf(diag, "mouse=(%.1f,%.1f) scroll_before=%.1f zoom_before=%.3f\n", preview_center.x, preview_center.y,
                scroll_before, zoom_before);
      }

      ctx->MouseMoveToPos(preview_center);
      ctx->Yield();

      // Scroll DOWN (negative delta = scroll content down in ImGui)
      ctx->MouseWheelY(-3.0f);
      ctx->Yield(2);

      float scroll_after = panel->Scroll.y;
      float zoom_after = gui::g_crystal_zoom;
      if (diag) {
        fprintf(diag, "scroll: %.1f -> %.1f, zoom: %.3f -> %.3f\n", scroll_before, scroll_after, zoom_before,
                zoom_after);
        fclose(diag);
      }

      // Zoom should have changed (mouse was over preview)
      IM_CHECK(zoom_after != zoom_before);
      // Panel scroll should NOT have changed
      IM_CHECK_EQ(scroll_after, scroll_before);
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
      gui::g_state.renderers[0].elevation = 45.0f;
      gui::g_state.renderers[0].azimuth = -30.0f;
      gui::g_state.renderers[0].roll = 10.0f;

      // Switch to full-sky lens type (index 4 = Dual Fisheye Equal Area)
      gui::g_state.renderers[0].lens_type = 4;

      // Renderer invariants in RenderPreviewPanel reset view angles for full-sky lenses
      ctx->Yield(3);

      IM_CHECK_EQ(gui::g_state.renderers[0].elevation, 0.0f);
      IM_CHECK_EQ(gui::g_state.renderers[0].azimuth, 0.0f);
      IM_CHECK_EQ(gui::g_state.renderers[0].roll, 0.0f);
    };
  }
}

// ========== task-test-gui-interaction: P1 Crystal/Filter CRUD + reference handling ==========

void RegisterP1InteractionTests(ImGuiTestEngine* engine) {
  // p1_crystal/delete_referenced_reassigns_sole_entry
  // Contract: deleting a crystal referenced by a sole-entry scatter layer reassigns the
  // entry's crystal_id to the first non-deleted crystal's id (HandleDeletedCrystalRefs in
  // panels.cpp:260-282, kReassignOnly branch).
  // NOTE: ImGui tab selection is not in g_state; click Crystal tab explicitly because
  // earlier tests may have switched tabs.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_crystal", "delete_referenced_reassigns_sole_entry");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      ctx->ItemClick("##LeftPanel/ConfigTabs/Crystal");
      ctx->Yield(2);

      // Arrange: default has 1 crystal (id=1), 1 scatter layer with sole entry → id=1
      IM_CHECK_EQ(static_cast<int>(gui::g_state.crystals.size()), 1);
      IM_CHECK_EQ(gui::g_state.crystals[0].id, 1);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.scattering[0].entries.size()), 1);
      IM_CHECK_EQ(gui::g_state.scattering[0].entries[0].crystal_id, 1);

      // Add a second crystal (id=2), unreferenced
      ctx->ItemClick("**/Add##crystal");
      IM_CHECK_EQ(static_cast<int>(gui::g_state.crystals.size()), 2);
      IM_CHECK_EQ(gui::g_state.crystals[1].id, 2);

      // Act: select the first (referenced) crystal and delete via modal
      gui::g_state.selected_crystal = 0;
      ctx->Yield();
      ClickDeleteCrystalViaModal(ctx);

      // Assert: sole-entry layer reassigned to fallback (id=2)
      IM_CHECK_EQ(static_cast<int>(gui::g_state.crystals.size()), 1);
      IM_CHECK_EQ(gui::g_state.crystals[0].id, 2);
      IM_CHECK_EQ(gui::g_state.scattering[0].entries[0].crystal_id, 2);
    };
  }

  // p1_crystal/delete_referenced_removes_multi_entry
  // Contract: in a multi-entry scatter layer, deleting a referenced crystal erases matching
  // entries (HandleDeletedCrystalRefs kRemoveOnly branch), leaving the layer's other entries
  // intact.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_crystal", "delete_referenced_removes_multi_entry");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      ctx->ItemClick("##LeftPanel/ConfigTabs/Crystal");
      ctx->Yield(2);

      // Add second crystal (id=2)
      ctx->ItemClick("**/Add##crystal");
      IM_CHECK_EQ(static_cast<int>(gui::g_state.crystals.size()), 2);
      IM_CHECK_EQ(gui::g_state.crystals[1].id, 2);

      // Programmatically append a second entry to the default layer referencing crystal id=2
      gui::ScatterEntry extra;
      extra.crystal_id = gui::g_state.crystals[1].id;  // id=2
      extra.proportion = 50.0f;
      gui::g_state.scattering[0].entries.push_back(extra);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.scattering[0].entries.size()), 2);

      // Act: select crystal[1] (id=2) and delete — multi-entry layer → kRemoveOnly
      gui::g_state.selected_crystal = 1;
      ctx->Yield();
      ClickDeleteCrystalViaModal(ctx);

      // Assert: crystal[1] removed, layer now has only the original entry (crystal_id=1)
      IM_CHECK_EQ(static_cast<int>(gui::g_state.crystals.size()), 1);
      IM_CHECK_EQ(gui::g_state.crystals[0].id, 1);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.scattering[0].entries.size()), 1);
      IM_CHECK_EQ(gui::g_state.scattering[0].entries[0].crystal_id, 1);
    };
  }

  // p1_crystal/delete_modal_cancel_preserves_state
  // Contract: clicking Cancel in Delete Crystal? modal leaves state unchanged and clears
  // g_pending_delete_crystal_idx (panels.cpp:509-511).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_crystal", "delete_modal_cancel_preserves_state");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      ctx->ItemClick("##LeftPanel/ConfigTabs/Crystal");
      ctx->Yield(2);

      // Need 2 crystals so Del button is enabled
      ctx->ItemClick("**/Add##crystal");
      IM_CHECK_EQ(static_cast<int>(gui::g_state.crystals.size()), 2);

      // Act: select referenced crystal, open modal, cancel
      gui::g_state.selected_crystal = 0;
      ctx->Yield();
      ClickDeleteCrystalCancelModal(ctx);

      // Assert: both crystals preserved
      IM_CHECK_EQ(static_cast<int>(gui::g_state.crystals.size()), 2);
    };
  }

  // p1_crystal/delete_all_then_add_new_id
  // Contract: next_crystal_id is monotonically increasing and never reuses deleted ids
  // (gui_state.hpp:166, InitDefaultState starts at 1).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_crystal", "delete_all_then_add_new_id");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      ctx->ItemClick("##LeftPanel/ConfigTabs/Crystal");
      ctx->Yield(2);

      // Default: crystals[0].id=1, next_crystal_id=2 (after InitDefaultState)
      IM_CHECK_EQ(gui::g_state.crystals[0].id, 1);
      IM_CHECK_EQ(gui::g_state.next_crystal_id, 2);

      // Add crystal (id=2)
      ctx->ItemClick("**/Add##crystal");
      IM_CHECK_EQ(gui::g_state.crystals[1].id, 2);
      IM_CHECK_EQ(gui::g_state.next_crystal_id, 3);

      // Delete the first (referenced) via modal → sole entry reassigned to id=2
      gui::g_state.selected_crystal = 0;
      ctx->Yield();
      ClickDeleteCrystalViaModal(ctx);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.crystals.size()), 1);
      IM_CHECK_EQ(gui::g_state.crystals[0].id, 2);

      // Add a new crystal — must get id=3, not reuse id=1
      ctx->ItemClick("**/Add##crystal");
      IM_CHECK_EQ(static_cast<int>(gui::g_state.crystals.size()), 2);
      IM_CHECK_EQ(gui::g_state.crystals[1].id, 3);
      IM_CHECK_EQ(gui::g_state.next_crystal_id, 4);
    };
  }

  // p1_filter/delete_referenced_clears_scatter_filter_id
  // Contract: deleting a filter referenced by scatter entries clears those entries'
  // filter_id to -1 (ClearFilterReferences in panels.cpp:308-315, called from
  // Delete Filter? modal confirm path).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_filter", "delete_referenced_clears_scatter_filter_id");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Add a filter via the UI
      ctx->ItemClick("##LeftPanel/ConfigTabs/Filter");
      ctx->Yield();
      ctx->ItemClick("**/Add##filter");
      IM_CHECK_EQ(static_cast<int>(gui::g_state.filters.size()), 1);
      const int filter_id = gui::g_state.filters[0].id;

      // Programmatically reference this filter from the default scatter entry
      gui::g_state.scattering[0].entries[0].filter_id = filter_id;
      ctx->Yield();

      // Act: delete via modal (referenced filter → modal path)
      ClickDeleteFilterViaModal(ctx);

      // Assert: filter removed, scatter entry's filter_id cleared
      IM_CHECK_EQ(static_cast<int>(gui::g_state.filters.size()), 0);
      IM_CHECK_EQ(gui::g_state.scattering[0].entries[0].filter_id, -1);
    };
  }
}

// ========== task-test-gui-interaction: P1 Slider boundary tests ==========


void RegisterP1SliderBoundaryTests(ImGuiTestEngine* engine) {
  // p1_slider/ray_count_min_max — linear scale, exact boundary values expected.
  // SliderWithInput("Rays(M)", ...) lives in RenderSceneTab (panels.cpp:609), so we must
  // switch to the Scene tab first. Internal widget ID is "##Rays(M)_input" via
  // PrepareSliderLayout (panels.cpp:33).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_slider", "ray_count_min_max");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      ctx->ItemClick("##LeftPanel/ConfigTabs/Scene");
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
      ctx->ItemClick("##LeftPanel/ConfigTabs/Scene");
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

  // p1_slider/height_min_clamped_to_0_01 — kLog scale clamps write(0) to min
  // SliderWithInput("Height", ..., kLog) → "##Height_slider" / "##Height_input".
  // kLog branch uses *value = std::max(*value, min_val) to defend against 0/negative
  // from InputFloat (panels.cpp:72).
  // NOTE: ImGui tab selection is not part of g_state — previous tests may have switched
  // tabs, so we must explicitly click Crystal tab here.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_slider", "height_min_clamped_to_0_01");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      ctx->ItemClick("##LeftPanel/ConfigTabs/Crystal");
      ctx->Yield(2);

      IM_CHECK_EQ(gui::g_state.crystals[0].type, gui::CrystalType::kPrism);

      // Write 0 → expected clamp to 0.01 (slider min, kLog scale cannot reach 0)
      ctx->ItemInputValue("**/##Height_input", 0.0f);
      ctx->Yield();
      IM_CHECK_GE(gui::g_state.crystals[0].height, 0.01f - 1e-6f);
      IM_CHECK_LT(gui::g_state.crystals[0].height, 0.02f);
    };
  }

  // p1_slider/pyramid_h1_h3_allow_zero — Upper/Lower H linear 0-1, 0 is a legal value
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_slider", "pyramid_h1_h3_allow_zero");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      ctx->ItemClick("##LeftPanel/ConfigTabs/Crystal");
      ctx->Yield(2);

      // Switch crystal type to pyramid (Upper H / Lower H sliders appear)
      gui::g_state.crystals[0].type = gui::CrystalType::kPyramid;
      ctx->Yield(3);

      ctx->ItemInputValue("**/##Upper H_input", 0.0f);
      ctx->Yield();
      IM_CHECK_EQ(gui::g_state.crystals[0].upper_h, 0.0f);

      ctx->ItemInputValue("**/##Lower H_input", 0.0f);
      ctx->Yield();
      IM_CHECK_EQ(gui::g_state.crystals[0].lower_h, 0.0f);
    };
  }

  // p1_slider/pyramid_prism_h_loglinear_zero — kLogLinear scale allows prism_h to reach 0
  // SliderWithInput("Prism H", ..., kLogLinear) → "##Prism H_input".
  // kLogLinear hybrid mapping: linear near zero, log above x0=0.01.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_slider", "pyramid_prism_h_loglinear_zero");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      ctx->ItemClick("##LeftPanel/ConfigTabs/Crystal");
      ctx->Yield(2);

      // Switch to pyramid type
      gui::g_state.crystals[0].type = gui::CrystalType::kPyramid;
      ctx->Yield(3);

      // Write 0 → kLogLinear allows reaching exactly 0
      ctx->ItemInputValue("**/##Prism H_input", 0.0f);
      ctx->Yield();
      IM_CHECK_EQ(gui::g_state.crystals[0].prism_h, 0.0f);

      // Write max value → should clamp to 100
      ctx->ItemInputValue("**/##Prism H_input", 100.0f);
      ctx->Yield();
      IM_CHECK_EQ(gui::g_state.crystals[0].prism_h, 100.0f);

      // Write small value in linear region → should preserve exactly
      ctx->ItemInputValue("**/##Prism H_input", 0.005f);
      ctx->Yield();
      IM_CHECK_GE(gui::g_state.crystals[0].prism_h, 0.0049f);
      IM_CHECK_LE(gui::g_state.crystals[0].prism_h, 0.0051f);
    };
  }

  // p1_slider/altitude_negative_boundaries — sun altitude ±90°
  // SliderWithInput("Altitude", ...) is also in RenderSceneTab (panels.cpp:598)
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_slider", "altitude_negative_boundaries");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      ctx->ItemClick("##LeftPanel/ConfigTabs/Scene");
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
      gui::g_state.renderers[0].lens_type = 4;
      gui::g_state.renderers[0].fov = 360.0f;
      ctx->Yield(3);
      IM_CHECK_EQ(gui::g_state.renderers[0].fov, 360.0f);

      // Switch to Linear (lens_type=0) — its MaxFov is typically much smaller
      gui::g_state.renderers[0].lens_type = 0;
      ctx->Yield(3);

      // fov should be clamped to Linear's max (some value < 360)
      IM_CHECK_LT(gui::g_state.renderers[0].fov, 360.0f);
    };
  }

  // p2_render/lens_switch_preserves_overlay — overlay flags survive lens type changes
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_render", "lens_switch_preserves_overlay");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      gui::g_state.show_horizon = true;
      gui::g_state.show_grid = true;
      gui::g_state.show_sun_circles = true;
      ctx->Yield();

      gui::g_state.renderers[0].lens_type = 4;  // Dual Fisheye EA (full-sky)
      ctx->Yield(3);
      IM_CHECK_EQ(gui::g_state.show_horizon, true);
      IM_CHECK_EQ(gui::g_state.show_grid, true);
      IM_CHECK_EQ(gui::g_state.show_sun_circles, true);

      gui::g_state.renderers[0].lens_type = 0;  // Linear
      ctx->Yield(3);
      IM_CHECK_EQ(gui::g_state.show_horizon, true);
      IM_CHECK_EQ(gui::g_state.show_grid, true);
      IM_CHECK_EQ(gui::g_state.show_sun_circles, true);
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
      gui::g_state.crystals[0].height = 2.5f;
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

      // Pre-register a filter on g_state so StartPerfSimulation's DoRun commits with it
      gui::FilterConfig f;
      f.id = gui::g_state.next_filter_id++;
      f.raypath_text = "3-1-5";
      gui::g_state.filters.push_back(f);
      gui::g_state.selected_filter = 0;

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
      gui::g_state.filters[0].raypath_text = "3-1-5-7";
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
      ctx->ItemClick("##LeftPanel/ConfigTabs/Crystal");
      ctx->Yield(2);

      // Add a crystal so state differs from default
      ctx->ItemClick("**/Add##crystal");
      IM_CHECK_EQ(static_cast<int>(gui::g_state.crystals.size()), 2);
      gui::g_state.dirty = true;
      ctx->Yield();

      // Click New → triggers unsaved popup
      ctx->ItemClick("##TopBar/New");
      ctx->Yield(2);

      // Click Cancel
      ctx->ItemClick("Unsaved Changes/Cancel");
      ctx->Yield();

      // State preserved: still 2 crystals, still dirty, pending action cleared
      IM_CHECK_EQ(gui::g_state.dirty, true);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.crystals.size()), 2);
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
      gui::g_state.crystals[0].type = gui::CrystalType::kPyramid;
      gui::g_state.crystals[0].prism_h = 3.5f;
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
      IM_CHECK_EQ(loaded.crystals[0].type, gui::CrystalType::kPyramid);
      IM_CHECK_EQ(loaded.crystals[0].prism_h, 3.5f);

      std::remove(tmp_path);
    };
  }
}
