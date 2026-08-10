// The right panel's Scene group — what the simulation is asked to compute, as opposed to how the
// result is shown.
//
// What this suite is for. `RenderSceneControls` (src/gui/panels.cpp) draws the sun, the ray budget
// and the spectrum picker. Two things about it can only be settled by a real frame. The first is
// the clamp: every slider here reads its domain from the field editor registry, and what a user
// typing a number actually lands on is a property of the call site, not of the registry — asking
// the registry what to expect would compare one line of code against itself, so every bound below
// is a literal. The second is the spectrum picker's transaction: choosing "Custom..." must open an
// editor WITHOUT committing the custom index, and only the editor's OK may advance it, so that
// dismissing the editor leaves the combo where it was.
//
// Deliberately NOT here, with where each lives instead. Whether the registry's own table is right
// is unit-correctness/gui/test_gui_widget_rules.cpp; whether a custom spectrum survives a save and
// reload is composition-correctness/gui/test_document_roundtrip_chain.cpp; what the reconciler does
// with a dirty document once it is dirty is composition-correctness/gui/test_run_lifecycle_chain.cpp.
//
// What a user sees when these break: a number they typed silently becoming a different number, a
// document that reports unsaved changes after they typed something the control rejected anyway, or
// a spectrum they spent five minutes editing being replaced by a preset they only meant to look at.

#include <string>
#include <vector>

#include "IconsFontAwesome6.h"
#include "gui/server_poller.hpp"  // LUMICE_CreateServer / StopServer / DestroyServer
#include "test_gui_shared.hpp"

namespace {

const char* const kAltitude = "**/##Altitude_input";
const char* const kDiameter = "**/##Diameter_input";
const char* const kRays = "**/##Rays(M)_input";
const char* const kMaxHits = "**/##Max hits_input";

bool IsDisabled(const ImGuiTestItemInfo& info) {
  return (info.ItemFlags & ImGuiItemFlags_Disabled) != 0;
}

}  // namespace

void RegisterSceneControlTests(ImGuiTestEngine* engine) {
  // P78. The two sun sliders, at both ends of each declared domain. Literals throughout: the call
  // site and the registry are one piece of code now, so re-asking the registry would prove nothing.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "scene_controls", "the_sun_sliders_clamp_typed_values_to_their_domains");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      struct Bound {
        const char* input;
        float typed;
        float expected;
        const float* slot;
      };
      ResetTestState();
      ctx->Yield(2);

      const Bound kBounds[] = {
        { kAltitude, -90.0f, -90.0f, &gui::g_state.sun.altitude },  // in range: identity
        { kAltitude, 90.0f, 90.0f, &gui::g_state.sun.altitude },
        { kDiameter, 40.0f, 5.0f, &gui::g_state.sun.diameter },  // out of range: clamped
        { kDiameter, -1.0f, 0.1f, &gui::g_state.sun.diameter },
      };
      for (const Bound& b : kBounds) {
        ctx->ItemInputValue(b.input, b.typed);
        ctx->Yield();
        // Reported rather than asserted fatally: which bound moved is the diagnostic, and a fatal
        // assert would return out of the case and take the remaining bounds with it.
        if (*b.slot != b.expected) {
          IM_ERRORF("%s: typed %f, expected %f, got %f", b.input, static_cast<double>(b.typed),
                    static_cast<double>(b.expected), static_cast<double>(*b.slot));
        }
      }
    };
  }

  // A value the control rejects is not an edit. Typing 200 into a slider whose maximum is 90 leaves
  // the field at 90 — where it already was — so nothing changed and the document must not start
  // claiming unsaved work. The regression this guards produced a permanently dirty document from a
  // keystroke that had no effect at all.
  //
  // Both ends are driven, because the clamp is two separate comparisons.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "scene_controls", "a_rejected_out_of_range_value_does_not_dirty_anything");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      struct Rejected {
        float seed;
        float typed;
      };
      const Rejected kCases[] = { { 90.0f, 200.0f }, { -90.0f, -200.0f } };
      for (const Rejected& c : kCases) {
        ResetTestState();
        ctx->Yield(2);
        gui::g_state.sun.altitude = c.seed;
        gui::g_state.dirty = false;
        ctx->Yield(2);

        ctx->ItemInputValue(kAltitude, c.typed);
        ctx->Yield();
        if (gui::g_state.sun.altitude != c.seed) {
          IM_ERRORF("typing %f moved altitude off %f to %f", static_cast<double>(c.typed), static_cast<double>(c.seed),
                    static_cast<double>(gui::g_state.sun.altitude));
        }
        if (gui::g_state.dirty) {
          IM_ERRORF("typing the out-of-range %f dirtied the document", static_cast<double>(c.typed));
        }
      }
    };
  }

  // P78's other half, end to end through the production reconcile path rather than through a
  // hand-built GuiState: a real Run mints the committed baseline, and an edit afterwards has to be
  // seen as a difference from it.
  //
  // The absence of a Yield after the edit is the assertion, not an oversight. The effects reconcile
  // runs at frame TAIL — after every Render*() call, before ImGui::Render() — in production and in
  // this harness alike, so the last frame of an ItemInputValue reaches its own tail before
  // returning. An intervening Yield would silently accept next-frame semantics and let a future
  // move of the reconcile to the top of the frame pass unnoticed.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "scene_controls", "an_edit_after_a_run_dirties_the_document_that_same_frame");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      gui::g_server = LUMICE_CreateServer();
      IM_CHECK(gui::g_server != nullptr);
      gui::g_state.sim.infinite = false;
      gui::g_state.sim.ray_num_millions = 0.5f;
      gui::DoRun(/*user_initiated=*/true);  // the synchronous commit is what populates the baseline
      IM_CHECK(gui::g_state.last_committed_state.has_value());
      gui::g_state.dirty = false;  // DoRun does not touch dirty; pin a known pre-edit baseline
      ctx->Yield(2);
      IM_CHECK(!gui::g_state.dirty);

      ctx->ItemInputValue(kAltitude, gui::g_state.sun.altitude + 5.0f);
      IM_CHECK(gui::g_state.dirty);

      gui::g_server_poller.Stop();
      LUMICE_StopServer(gui::g_server);
      LUMICE_DestroyServer(gui::g_server);
      gui::g_server = nullptr;
      gui::g_state.run_intent = gui::RunIntent::kNone;
      gui::g_state.committed_epoch = 0;
      gui::g_state.dirty = false;
    };
  }

  // P82. The ray total is ONE control greyed by the registry, not two call sites picked between by
  // an if. Both states are asserted on the same widget id, which is what makes "there is only one"
  // observable: a reintroduced second call site under `infinite` would be a different item, and the
  // id that exists in one state would have to stop existing in the other.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "scene_controls", "the_ray_total_is_one_control_the_registry_greys");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      IM_CHECK(!gui::g_state.sim.infinite);
      IM_CHECK(!IsDisabled(ctx->ItemInfo(kRays)));

      // The declared domain, as literals.
      ctx->ItemInputValue(kRays, 0.1f);
      ctx->Yield();
      IM_CHECK_EQ(gui::g_state.sim.ray_num_millions, 0.1f);
      ctx->ItemInputValue(kRays, 100.0f);
      ctx->Yield();
      IM_CHECK_EQ(gui::g_state.sim.ray_num_millions, 100.0f);

      gui::g_state.sim.infinite = true;
      ctx->Yield(3);
      IM_CHECK(ctx->ItemExists(kRays));  // still the same item, and still submitted
      IM_CHECK(IsDisabled(ctx->ItemInfo(kRays)));

      gui::g_state.sim.infinite = false;
      ctx->Yield(3);
      IM_CHECK(!IsDisabled(ctx->ItemInfo(kRays)));
    };
  }

  // P82's user-visible consequence: turning the budget off and on again must give it back. A
  // checkbox that zeroed the total on the way past would be indistinguishable from this one until
  // the user turned it off.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "scene_controls", "toggling_infinite_rays_gives_the_ray_total_back");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      ctx->ItemInputValue(kRays, 5.0f);
      ctx->Yield();
      IM_CHECK_EQ(gui::g_state.sim.ray_num_millions, 5.0f);

      ctx->ItemClick("**/Infinite rays");
      ctx->Yield();
      IM_CHECK(gui::g_state.sim.infinite);
      ctx->ItemClick("**/Infinite rays");
      ctx->Yield();
      IM_CHECK(!gui::g_state.sim.infinite);
      IM_CHECK_EQ(gui::g_state.sim.ray_num_millions, 5.0f);
    };
  }

  // P83. The int slider is a different widget family from the float ones above and reads its bounds
  // from the same registry, so it gets the same treatment: literals, at both ends.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "scene_controls", "max_hits_clamps_typed_values_to_its_domain");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      ctx->ItemInputValue(kMaxHits, 12);
      ctx->Yield();
      IM_CHECK_EQ(gui::g_state.sim.max_hits, 12);
      ctx->ItemInputValue(kMaxHits, 100000);
      ctx->Yield();
      const int at_max = gui::g_state.sim.max_hits;
      IM_CHECK_LT(at_max, 100000);  // it was clamped by something
      ctx->ItemInputValue(kMaxHits, -5);
      ctx->Yield();
      IM_CHECK_GT(gui::g_state.sim.max_hits, 0);
      IM_CHECK_LT(gui::g_state.sim.max_hits, at_max);
    };
  }

  // P84. The GPU toggle is not a control that greys out on a machine without a GPU — it is not
  // drawn at all, because a checkbox whose only outcome is a silent fallback to the CPU is worse
  // than no checkbox. Which branch is under test is decided by the same probe the panel uses, so
  // this case says something true on a machine with a backend and on one without; asserting only
  // the branch this developer's machine happens to take would make it a no-op elsewhere.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "scene_controls", "the_gpu_toggle_is_absent_rather_than_greyed_without_a_backend");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      const bool have_backend =
          LUMICE_IsBackendAvailable(LUMICE_BACKEND_METAL) || LUMICE_IsBackendAvailable(LUMICE_BACKEND_CUDA);
      if (!have_backend) {
        IM_CHECK(!ctx->ItemExists("**/Use GPU"));
        return;
      }

      IM_CHECK(ctx->ItemExists("**/Use GPU"));
      IM_CHECK(!IsDisabled(ctx->ItemInfo("**/Use GPU")));
      const bool before = gui::g_state.use_gpu_backend;
      gui::g_state.dirty = false;
      ctx->ItemClick("**/Use GPU");
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.use_gpu_backend, !before);
      // The backend choice rebuilds the server on the next Run, so it has to read as a change.
      IM_CHECK(gui::g_state.dirty);

      // ...and it is unreachable while a run is in flight, since an in-flight stop still holds the
      // server the switch would rebuild. run_intent is what makes the state stick: the harness main
      // loop re-derives sim_state every frame, so a bare sim_state write is gone by the next one.
      gui::g_state.run_intent = gui::RunIntent::kRunning;
      ctx->Yield(3);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state),
                  static_cast<int>(gui::GuiState::SimState::kSimulating));  // the premise held
      IM_CHECK(IsDisabled(ctx->ItemInfo("**/Use GPU")));
      gui::g_state.run_intent = gui::RunIntent::kNone;

      ResetTestState();
      ctx->Yield(2);
    };
  }

  // P79 / P121. Picking "Custom..." is an INTENT, not a commit: it opens the editor and leaves
  // spectrum_index where it was, so a user who opens the editor to look at it and dismisses it
  // still has the preset they started from. The combo is bound to a local copy for exactly this
  // reason, and the only place the custom index is written is the editor's OK.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "scene_controls", "picking_custom_opens_the_editor_without_committing_it");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      const int preset_before = gui::g_state.sun.spectrum_index;
      IM_CHECK_NE(preset_before, gui::kCustomSpectrumIndex);

      ctx->SetRef("//##RightPanel");
      ctx->ComboClick("Spectrum/Custom...");
      ctx->SetRef("");
      ctx->Yield(3);

      // The editor is up, and the committed index has NOT moved.
      IM_CHECK(ctx->ItemExists("**/" ICON_FA_CHECK " OK##spec_ok"));
      IM_CHECK_EQ(gui::g_state.sun.spectrum_index, preset_before);

      // Cancel leaves it there too — the combo goes on showing the preset.
      ctx->ItemClick("**/" ICON_FA_XMARK " Cancel##spec_cancel");
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.sun.spectrum_index, preset_before);

      // OK is the sole commit point.
      ctx->SetRef("//##RightPanel");
      ctx->ComboClick("Spectrum/Custom...");
      ctx->SetRef("");
      ctx->Yield(3);
      ctx->ItemClick("**/" ICON_FA_CHECK " OK##spec_ok");
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.sun.spectrum_index, gui::kCustomSpectrumIndex);
    };
  }

  // P80. The re-open button exists only once there IS a custom spectrum to re-open — otherwise it
  // would be a second, redundant way to do what the combo already does.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "scene_controls", "the_edit_spectrum_button_appears_only_once_custom_is_committed");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      IM_CHECK(!ctx->ItemExists("**/Edit spectrum...##spectrum_edit"));

      gui::g_state.sun.spectrum_index = gui::kCustomSpectrumIndex;
      ctx->Yield(2);
      IM_CHECK(ctx->ItemExists("**/Edit spectrum...##spectrum_edit"));

      ctx->ItemClick("**/Edit spectrum...##spectrum_edit");
      ctx->Yield(3);
      IM_CHECK(ctx->ItemExists("**/" ICON_FA_CHECK " OK##spec_ok"));
      ctx->ItemClick("**/" ICON_FA_XMARK " Cancel##spec_cancel");
      ctx->Yield(2);
    };
  }

  // P81. custom_spectrum is only READ when the index selects it, so switching to a preset must
  // leave the list alone rather than clear it — a user comparing their spectrum against D65 has not
  // asked for their work to be discarded.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "scene_controls", "a_preset_detour_does_not_discard_the_custom_spectrum");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      const std::vector<gui::WlWeight> authored = { { 450.0f, 0.5f }, { 550.0f, 1.0f }, { 650.0f, 0.7f } };
      gui::g_state.sun.spectrum_index = gui::kCustomSpectrumIndex;
      gui::g_state.sun.custom_spectrum = authored;
      ctx->Yield(2);

      // Away to a preset, through the real combo...
      ctx->SetRef("//##RightPanel");
      ctx->ComboClick("Spectrum/D65");
      ctx->SetRef("");
      ctx->Yield(3);
      IM_CHECK_NE(gui::g_state.sun.spectrum_index, gui::kCustomSpectrumIndex);
      IM_CHECK_EQ(gui::g_state.sun.custom_spectrum.size(), authored.size());

      // ...and back. The editor opens on the list that was there, not on a fresh seed.
      ctx->SetRef("//##RightPanel");
      ctx->ComboClick("Spectrum/Custom...");
      ctx->SetRef("");
      ctx->Yield(3);
      ctx->ItemClick("**/" ICON_FA_CHECK " OK##spec_ok");
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.sun.spectrum_index, gui::kCustomSpectrumIndex);
      IM_CHECK_EQ(gui::g_state.sun.custom_spectrum.size(), authored.size());
      for (size_t i = 0; i < authored.size(); ++i) {
        if (gui::g_state.sun.custom_spectrum[i].wavelength != authored[i].wavelength ||
            gui::g_state.sun.custom_spectrum[i].weight != authored[i].weight) {
          IM_ERRORF("row %d came back as (%f, %f), authored (%f, %f)", static_cast<int>(i),
                    static_cast<double>(gui::g_state.sun.custom_spectrum[i].wavelength),
                    static_cast<double>(gui::g_state.sun.custom_spectrum[i].weight),
                    static_cast<double>(authored[i].wavelength), static_cast<double>(authored[i].weight));
        }
      }
    };
  }

  // P119 / P120 / P121. The row editor itself: values typed into a row reach the committed list,
  // Add row appends, and the per-row × removes THAT row rather than the last one. Driven in one
  // walkthrough because a row list is only interesting once the rows can be told apart.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "scene_controls", "spectrum_rows_can_be_edited_added_and_removed");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      gui::g_state.sun.spectrum_index = gui::kCustomSpectrumIndex;
      gui::g_state.sun.custom_spectrum = { { 450.0f, 0.5f }, { 550.0f, 1.0f }, { 650.0f, 0.7f } };
      gui::OpenSpectrumModal(gui::g_state);
      ctx->Yield(3);

      // Rows are drawn inside PushID(i), so "$$N" reproduces that scope by id.
      ctx->ItemInputValue("**/$$1/wl(nm)##wl", 500.0f);
      ctx->Yield();
      ctx->ItemInputValue("**/$$1/weight##wt", 0.25f);
      ctx->Yield();
      ctx->ItemClick("**/" ICON_FA_PLUS " Add row");
      ctx->Yield(2);
      // Remove the FIRST row, so "the right row went" is distinguishable from "a row went".
      ctx->ItemClick("**/$$0/" ICON_FA_XMARK "##rm");
      ctx->Yield(2);
      ctx->ItemClick("**/" ICON_FA_CHECK " OK##spec_ok");
      ctx->Yield(2);

      const auto& out = gui::g_state.sun.custom_spectrum;
      IM_CHECK_EQ(out.size(), (size_t)3);  // 3 seeded, +1 added, -1 removed
      // The edited row was row 1 and row 0 was removed, so it is now first.
      IM_CHECK_EQ(out[0].wavelength, 500.0f);
      IM_CHECK_EQ(out[0].weight, 0.25f);
      IM_CHECK_EQ(out[1].wavelength, 650.0f);  // the untouched third row moved up by one
    };
  }

  // P122. Reset restores the editor's default list; Cancel afterwards must still leave the
  // committed spectrum alone, because OK is the only commit point. Both halves in one case: they
  // are the same sentence, and separating them once let a Reset-then-Cancel land on disk.
  //
  // The expected list is spelled out from the construction rule (9 points, 400-720 nm, weight 1)
  // rather than read back from the private helper that builds it, so a refactor of that helper
  // cannot silently drift this.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "scene_controls", "spectrum_reset_seeds_a_uniform_grid_and_cancel_still_discards");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      const std::vector<gui::WlWeight> baseline = { { 450.0f, 0.5f }, { 550.0f, 1.0f }, { 650.0f, 0.7f } };
      gui::g_state.sun.spectrum_index = gui::kCustomSpectrumIndex;
      gui::g_state.sun.custom_spectrum = baseline;

      // Reset then Cancel: nothing reaches the committed state.
      gui::OpenSpectrumModal(gui::g_state);
      ctx->Yield(3);
      ctx->ItemClick("**/Reset##spec_reset");
      ctx->Yield();
      ctx->ItemClick("**/" ICON_FA_XMARK " Cancel##spec_cancel");
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.sun.spectrum_index, gui::kCustomSpectrumIndex);
      IM_CHECK_EQ(gui::g_state.sun.custom_spectrum.size(), baseline.size());

      // Reset then OK: the uniform seed is what lands.
      gui::OpenSpectrumModal(gui::g_state);
      ctx->Yield(3);
      ctx->ItemClick("**/Reset##spec_reset");
      ctx->Yield();
      ctx->ItemClick("**/" ICON_FA_CHECK " OK##spec_ok");
      ctx->Yield(2);

      const auto& out = gui::g_state.sun.custom_spectrum;
      IM_CHECK_EQ(gui::g_state.sun.spectrum_index, gui::kCustomSpectrumIndex);
      IM_CHECK_EQ(out.size(), (size_t)9);
      for (size_t i = 0; i < out.size(); ++i) {
        const float expected_wl = 400.0f + static_cast<float>(i) * 40.0f;
        if (out[i].wavelength != expected_wl || out[i].weight != 1.0f) {
          IM_ERRORF("seed row %d is (%f, %f), expected (%f, 1.0)", static_cast<int>(i),
                    static_cast<double>(out[i].wavelength), static_cast<double>(out[i].weight),
                    static_cast<double>(expected_wl));
        }
      }
    };
  }
}
