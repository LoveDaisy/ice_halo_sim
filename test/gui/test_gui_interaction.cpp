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

  // P1: scrum-gui-polish-v7 152.5 — Remove Filter marks the edit controls as
  // disabled (Raypath InputText) so the "will be removed on OK" banner is
  // consistent with the rest of the Filter tab.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_filter", "remove_disables_controls");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      gui::FilterConfig f;
      f.raypath_text = "3-1-5";
      gui::g_state.layers[0].entries[0].filter = f;

      ctx->ItemClick("**/Edit##fi");
      ctx->Yield(4);
      // Before Remove: Raypath control is enabled.
      auto info_before = ctx->ItemInfo("**/Raypath##filter_modal");
      IM_CHECK((info_before.ItemFlags & ImGuiItemFlags_Disabled) == 0);

      ctx->ItemClick("**/Remove Filter##filter");
      ctx->Yield(2);
      // After Remove: Raypath control is disabled.
      auto info_after = ctx->ItemInfo("**/Raypath##filter_modal");
      IM_CHECK((info_after.ItemFlags & ImGuiItemFlags_Disabled) != 0);

      // Cancel so we don't leave state polluted for later tests.
      ctx->ItemClick("**/Cancel##edit_modal");
      ctx->Yield(2);
    };
  }

  // P1: scrum-gui-polish-v7 152.5 — Undo Remove restores editability and
  // committing OK keeps the filter present (the buffered Remove intent was
  // reverted before commit).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_filter", "undo_remove_restores_controls");
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
      ctx->ItemClick("**/Undo Remove##filter_undo");
      ctx->Yield(2);

      // Controls restored to enabled state.
      auto info = ctx->ItemInfo("**/Raypath##filter_modal");
      IM_CHECK((info.ItemFlags & ImGuiItemFlags_Disabled) == 0);

      ctx->ItemClick("**/OK##edit_modal");
      ctx->Yield(2);
      // Filter must survive the round-trip since Remove was undone.
      IM_CHECK(gui::g_state.layers[0].entries[0].filter.has_value());
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
  // SliderWithInput("Height##modal_cr", ..., 0.01f, 100.0f, kLogLinear) — height ≥ 0.01
  // SliderWithInput("Upper H##modal_cr", ..., 0.0f, 100.0f, kLogLinear) — allows 0
  // SliderWithInput("Lower H##modal_cr", ..., 0.0f, 100.0f, kLogLinear) — allows 0
  // SliderWithInput("Prism H##modal_cr", ..., 0.0f, 100.0f, kLogLinear) — allows 0

  // p1_slider/height_clamp_via_modal — kLogLinear clamps height to [0.01, 100]
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

      // Height should be clamped to >= 0.01 (kLogLinear with min=0.01)
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

      // Write 0 to Upper H — should be allowed (min=0.0 for kLogLinear)
      ctx->ItemInputValue("**/##Upper H##modal_cr_input", 0.0f);
      ctx->Yield();

      // Click OK
      ctx->ItemClick("**/OK##edit_modal");
      ctx->Yield(2);

      IM_CHECK_EQ(gui::g_state.layers[0].entries[0].crystal.upper_h, 0.0f);
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

      gui::g_state.renderer.lens_type = 4;  // Dual Fisheye EA (full-sky)
      ctx->Yield(3);
      IM_CHECK_EQ(gui::g_state.show_horizon, true);
      IM_CHECK_EQ(gui::g_state.show_grid, true);
      IM_CHECK_EQ(gui::g_state.show_sun_circles, true);

      gui::g_state.renderer.lens_type = 0;  // Linear
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
}
