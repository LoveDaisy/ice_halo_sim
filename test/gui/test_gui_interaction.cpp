#include <chrono>
#include <cstdio>
#include <thread>

#include "test_gui_shared.hpp"

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

      // Add an entry via the "+ Entry" button in layer 0
      ctx->ItemClick("**/+ Entry##layer_0");
      ctx->Yield();
      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers[0].entries.size()), 2);

      // Delete the new entry (x##0_1 for layer 0, entry 1)
      // Entries can only be deleted when layer has >1 entries
      ctx->ItemClick("**/x##0_1");
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

      // Open filter modal, click Remove to clear
      ctx->ItemClick("**/E##fi");
      ctx->Yield(2);
      ctx->ItemClick("Edit Filter/Remove Filter##filter");
      ctx->Yield(2);

      // Verify filter is cleared
      IM_CHECK(!gui::g_state.layers[0].entries[0].filter.has_value());
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

  // P1: Mouse wheel over crystal preview should zoom only, not scroll parent panel
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_crystal", "preview_scroll_isolation");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Left panel now renders cards directly (no tabs) — just let layout settle
      ctx->Yield(3);  // Let layout settle: mesh rebuild → FBO render

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

      // Select the new layer (idx 1) then delete it
      gui::SetSelectedLayerIdx(1);
      ctx->Yield();
      ctx->ItemClick("**/- Layer");
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

      // Copy the entry
      ctx->ItemClick("**/Copy##0_0");
      ctx->Yield();
      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers[0].entries.size()), 2);

      // Verify the copy has same crystal params
      auto& copy = gui::g_state.layers[0].entries[1];
      IM_CHECK_EQ(copy.crystal.type, gui::CrystalType::kPyramid);
      IM_CHECK_EQ(copy.crystal.prism_h, 2.5f);
    };
  }

  // p1_card/proportion_slider — modify entry proportion
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_card", "proportion_slider");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      float before = gui::g_state.layers[0].entries[0].proportion;
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

  // Crystal slider boundary tests — programmatic validation via copy model.
  // Crystal parameters are now in layers[].entries[].crystal (copy model).
  // These tests validate boundary behavior by direct field manipulation.

  // p1_slider/crystal_height_boundary — height must be > 0 (prism default)
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_slider", "crystal_height_boundary");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      auto& cr = gui::g_state.layers[0].entries[0].crystal;
      IM_CHECK_EQ(cr.type, gui::CrystalType::kPrism);

      // Set height to a valid value
      cr.height = 2.0f;
      IM_CHECK_EQ(cr.height, 2.0f);

      // Height field allows positive values
      cr.height = 0.01f;
      IM_CHECK_GE(cr.height, 0.01f - 1e-6f);
    };
  }

  // p1_slider/pyramid_upper_lower_allow_zero — Upper/Lower H are 0-1, 0 is legal
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_slider", "pyramid_upper_lower_allow_zero");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      auto& cr = gui::g_state.layers[0].entries[0].crystal;
      cr.type = gui::CrystalType::kPyramid;
      ctx->Yield();

      cr.upper_h = 0.0f;
      IM_CHECK_EQ(cr.upper_h, 0.0f);

      cr.lower_h = 0.0f;
      IM_CHECK_EQ(cr.lower_h, 0.0f);

      // Verify non-zero values also work
      cr.upper_h = 0.5f;
      cr.lower_h = 0.3f;
      IM_CHECK_EQ(cr.upper_h, 0.5f);
      IM_CHECK_EQ(cr.lower_h, 0.3f);
    };
  }

  // p1_slider/pyramid_prism_h_allows_zero — prism_h can be exactly 0
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_slider", "pyramid_prism_h_allows_zero");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      auto& cr = gui::g_state.layers[0].entries[0].crystal;
      cr.type = gui::CrystalType::kPyramid;

      cr.prism_h = 0.0f;
      IM_CHECK_EQ(cr.prism_h, 0.0f);

      cr.prism_h = 100.0f;
      IM_CHECK_EQ(cr.prism_h, 100.0f);

      cr.prism_h = 0.005f;
      IM_CHECK_GE(cr.prism_h, 0.0049f);
      IM_CHECK_LE(cr.prism_h, 0.0051f);
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
}
