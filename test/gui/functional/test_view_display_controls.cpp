// The right panel's View and Display groups — the controls that decide what the preview shows and
// at what shape, as opposed to what the simulation computes.
//
// What this suite is for. These are the panel's own controls (`##RightPanel`), and what can only
// be answered by a real frame is whether a piece of state the panel derives actually reaches the
// screen. The aspect-clamp warning below is the sharpest instance: it is the only feedback a user
// gets when the window they asked for did not fit the monitor, and it is drawn from a signal
// written on a completely different code path (the GLFW resize in ApplyAspectRatio), so "the flag
// was set" and "the user was told" are two separate claims.
//
// The rest of the group is here too: the lens-type combo and the pose it re-gates on every switch,
// the FOV and visible-hemisphere gates, Reset, and the background row. What they have in common is
// that the lens decides what applies — the same slider is a live control under one projection and a
// greyed placeholder under another — so almost every case below is a claim quantified over lenses
// rather than a claim about one.
//
// Deliberately NOT here, with where each lives instead. Whether ResolveAspectFit computes
// was_clamped correctly is arithmetic over integers and is asserted over its whole domain in
// unit-correctness/gui/test_gui_widget_rules.cpp; which preset options are disabled with and
// without a background image is asserted over its whole domain in the same file; whether the aspect
// fields survive a document round trip is composition-correctness/gui/test_document_roundtrip_chain.cpp;
// what a DRAG on the preview does with the pose these controls set is
// functional/test_preview_viewport.cpp; the Overlay group below this one is
// functional/test_overlay_controls.cpp. Nothing below restates them.
//
// Three propositions in this group are recorded rather than covered. The Resolution combo's tooltip
// and its orange frame (P27) and the Globe row's "(?)" tooltip (P24) hang off ImGui::Text* items,
// which submit with id == 0 and never reach the test engine's registry — the same mechanism
// recorded for the entry card's summary and the colour rows' warning triangle. Applying an aspect
// preset (P28/P29) resizes the HOST WINDOW, and this suite's committed pixel references are
// captured at the harness's default framebuffer size, so a case that applied one would be charged
// to them; the preset rules themselves are covered over their whole domain at the unit layer, and
// the panel's response to the clamp signal they produce is the three cases below.
//
// What a user sees when these break: they pick 2:1 on a laptop screen that cannot hold it, get a
// window that is not 2:1, and nothing on screen explains why — or the opposite, a permanent
// "Screen too small" banner that no preset makes go away.

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <set>
#include <string>
#include <vector>

#include "gui/app.hpp"  // g_state / g_server / g_preview_vp / DoRun — the exposure-mode case
#include "gui/defaults_diff.hpp"
#include "gui/file_io.hpp"
#include "gui/gui_state.hpp"
#include "gui/mono_exposure_scale.hpp"
#include "gui/user_defaults.hpp"
#include "support/user_defaults_test_env.hpp"
#include "test_gui_shared.hpp"

namespace {

// The warning's addressable half. app_panels.cpp draws it as a DISABLED Selectable rather than a
// TextColored precisely so it has a real ImGui id: a Text* widget is emitted with id==0 and the
// test engine's item registry never sees it, so this string is the only handle a test has on
// "the user was told". The ratio detail printed on the following line is a plain Text and is
// therefore NOT assertable here — that is a property of ImGui's item registry, not an omission.
constexpr const char* kClampWarning = "**/Screen too small for this aspect";

// Open the Display group's exposure Mode combo and pick an entry.
//
// Hand-rolled rather than ctx->ComboClick() for the same two reasons the colour window's picker
// is: ImGui::BeginCombo() never calls IMGUI_TEST_ENGINE_ITEM_INFO(), so the combo has no debug
// label for a "**/" wildcard to match and must be addressed by its literal path; and the popup it
// opens is a separate window, which only SetRef("//$FOCUSED") can point at — ImHashDecoratedPath
// does not understand the $FOCUSED variable inside a longer path.
void PickExposureMode(ImGuiTestContext* ctx, const char* entry) {
  ctx->ItemClick("//##RightPanel/Mode##display");
  ctx->SetRef("//$FOCUSED");
  ctx->ItemClick((std::string("**/") + entry).c_str());
  ctx->SetRef("");
}

// Install a clamp signal without going through ApplyAspectRatio.
//
// ApplyAspectRatio is the only producer of this signal, and it produces it by asking GLFW for a
// window size against the real monitor work area and recording what it got (src/gui/app.cpp). A
// test cannot make the monitor smaller, and calling it would resize the harness window for every
// case that runs after this one — the suite's committed pixel references are captured at the
// harness's default framebuffer size, so a stray resize here would be charged to them. The signal
// is therefore written directly and the panel is asked what it does with it, which is the half of
// the contract this layer owns.
void InstallClampSignal(bool was_clamped) {
  gui::g_state.aspect_clamp.was_clamped = was_clamped;
  gui::g_state.aspect_clamp.requested_preview_ratio = 2.0f;
  gui::g_state.aspect_clamp.achieved_preview_ratio = was_clamped ? 1.01f : 2.0f;
}

// Set by the background-alpha case's TestFunc, consumed by its GuiFunc on the main thread.
bool g_bg_upload_requested = false;

// Hands the shared preview's background back when the case leaves, by whichever exit.
//
// gui_test is one process with one g_preview, and ResetTestState does not clear a loaded
// background — which is why the case that loads one has to put it back. Written as a guard rather
// than as statements at the end of the body because a fatal assert expands to `return`: the
// teardown would then run only on the passing path, and a real regression would leave a background
// image and bg_show set for every case that follows, including this suite's committed pixel
// references.
struct ScopedBackground {
  ~ScopedBackground() {
    gui::g_preview.ClearBackground();
    gui::g_state.bg_show = false;
  }
};

// The four View sliders, in the order the panel draws them.
const char* const kViewInputs[] = { "**/##FOV##view_input", "**/##Elevation##view_input", "**/##Azimuth##view_input",
                                    "**/##Roll##view_input" };

// The Look At entry. A "###" id, so the icon in its label is not part of it.
const char* const kLookAtButton = "**/###view_look_at";

// Open the Look At menu and pick one entry by its visible name.
//
// Two steps, for the reason test_color_window.cpp records at its own popup helper: a wildcard path
// resolves its prefix with ImHashDecoratedPath, which does not understand "$FOCUSED" — only
// SetRef() does — so the popup has to BECOME the ref before the wildcard search rather than inside
// it. Reports nothing itself; the caller checks the state that was supposed to change.
void PickLookAt(ImGuiTestContext* ctx, const char* entry) {
  ctx->SetRef("//##RightPanel");
  ctx->ItemClick(kLookAtButton);
  ctx->Yield(2);
  ctx->SetRef("//$FOCUSED");
  ctx->ItemClick((std::string("**/") + entry).c_str());
  ctx->Yield(2);
  ctx->SetRef("");
}

}  // namespace

void RegisterViewDisplayControlTests(ImGuiTestEngine* engine) {
  // The positive branch: a clamped window on a real preset says so.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "view_display_controls", "a_clamped_aspect_tells_the_user_the_screen_is_too_small");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      gui::g_state.aspect_preset = gui::AspectPreset::k2x1;
      InstallClampSignal(true);
      ctx->Yield(2);

      IM_CHECK(ctx->ItemExists(kClampWarning));
    };
  }

  // The Free preset re-check. app_panels.cpp tests the preset a SECOND time here, after
  // ApplyAspectRatio has already cleared the signal on the Free path — a deliberate belt-and-braces
  // guard against a stale signal arriving from a callback path that missed the clear. That makes it
  // a branch with no other guard: delete the re-check and every green test still passes, because
  // the only way to reach it is a state the producer is not supposed to leave behind.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "view_display_controls", "a_stale_clamp_signal_stays_silent_on_the_free_preset");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      gui::g_state.aspect_preset = gui::AspectPreset::kFree;
      InstallClampSignal(true);
      ctx->Yield(2);

      IM_CHECK(!ctx->ItemExists(kClampWarning));
    };
  }

  // The negative branch: same preset as the positive case, signal off. Paired with it deliberately
  // — a warning that is always drawn would pass the positive case on its own.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "view_display_controls", "an_aspect_that_fit_says_nothing");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      gui::g_state.aspect_preset = gui::AspectPreset::k2x1;
      InstallClampSignal(false);
      ctx->Yield(2);

      IM_CHECK(!ctx->ItemExists(kClampWarning));
    };
  }

  // P19. The four groups fold independently and all four start open, so a user who collapses Scene
  // to get at Overlay does not find Display gone too. Asserted through an item inside each group,
  // because "the header is closed" and "its contents are not submitted" are the same claim and the
  // second is the one that matters.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "view_display_controls", "the_four_groups_fold_independently_and_start_open");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      struct Group {
        const char* header;
        const char* member;  // an item that exists only while that group is expanded
      };
      const Group kGroups[] = {
        { "**/Scene", "**/##Altitude_input" },
        { "**/View", "**/##FOV##view_input" },
        { "**/Display", "**/##EV##display_input" },
        { "**/Overlay", "**/##horizon_line" },
      };

      ResetTestState();
      ctx->Yield(3);
      for (const Group& g : kGroups) {
        if (!ctx->ItemExists(g.member)) {
          IM_ERRORF("%s does not start expanded: %s is missing", g.header, g.member);
        }

        if (ctx->IsError()) {
          break;
        }
      }

      // Collapse each in turn and check that only that one closed.
      for (const Group& target : kGroups) {
        ctx->ItemClose(target.header);
        ctx->Yield(3);
        for (const Group& g : kGroups) {
          const bool present = ctx->ItemExists(g.member);
          const bool expected = &g != &target;
          if (present != expected) {
            IM_ERRORF("with %s collapsed, %s is %s", target.header, g.member, present ? "still there" : "gone too");
          }

          if (ctx->IsError()) {
            break;
          }
        }
        ctx->ItemOpen(target.header);
        ctx->Yield(3);

        if (ctx->IsError()) {
          break;
        }
      }
    };
  }

  // P20. The combo lists the lenses in a presentation order that is deliberately NOT the enum
  // order — the enum grew by addition, the list is grouped by what the projections do. Asserted by
  // reading where the items landed on screen, which is the only place the order is observable.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "view_display_controls", "the_lens_combo_lists_lenses_in_presentation_order");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(3);

      ctx->SetRef("//##RightPanel");
      ctx->ItemClick("Lens Type##view");  // opens the popup; BeginCombo reports no info of its own
      ctx->SetRef("");
      ctx->Yield(3);

      // The popup scrolls, so items past the fold have no rect to compare. Walk the prefix that is
      // visible and require it to be a prefix of the declared order — enough to catch a list that
      // fell back to enum order, since the two disagree from the second entry on.
      float previous_y = -1.0f;
      int checked = 0;
      for (const int lens : gui::kLensTypePresentationOrder) {
        const ImGuiTestItemInfo item =
            ctx->ItemInfo((std::string("**/") + gui::kLensTypeNames[lens]).c_str(), ImGuiTestOpFlags_NoError);
        if (item.ID == 0) {
          break;  // below the popup's fold
        }
        if (item.RectFull.Min.y <= previous_y) {
          IM_ERRORF("%s is drawn at y=%.1f, above the entry that should precede it (y=%.1f)", gui::kLensTypeNames[lens],
                    static_cast<double>(item.RectFull.Min.y), static_cast<double>(previous_y));
        }
        previous_y = item.RectFull.Min.y;
        ++checked;

        if (ctx->IsError()) {
          break;
        }
      }
      IM_CHECK_GT(checked, 2);  // a popup that never opened would pass the loop vacuously

      ctx->KeyPress(ImGuiKey_Escape);
      ctx->Yield(2);
    };
  }

  // P21 / P22. Every lens has a maximum field of view and the panel pulls the value into it on
  // every frame, not only on the frame the lens changed — a config loaded with a 360-degree value
  // and a rectilinear lens has to come back into range on its own.
  //
  // Quantified over every lens the enum has rather than over the handful someone thought to write
  // down: the four cases this replaces covered five of eleven lenses between them.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "view_display_controls", "every_lens_pulls_the_fov_into_its_own_maximum");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      for (int lens = 0; lens < gui::kLensTypeCount; ++lens) {
        gui::g_state.renderer.lens_type = lens;
        gui::g_state.renderer.fov = 1000.0f;  // above every lens's maximum
        ctx->Yield(3);

        const float max_fov = LUMICE_MaxFov(static_cast<LUMICE_LensType>(lens));
        const float got = gui::g_state.renderer.fov;
        if (got > max_fov || got <= 0.0f) {
          IM_ERRORF("lens %d settled at fov=%f, outside (0, %f]", lens, static_cast<double>(got),
                    static_cast<double>(max_fov));
        }

        if (ctx->IsError()) {
          break;
        }
      }
    };
  }

  // P25. Globe locks roll to zero at render time WITHOUT writing that back, so a user who switches
  // to the globe and away again finds the roll they set still there. The stored field and the
  // effective one are two different things, and this is the case that says so.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "view_display_controls", "the_globe_masks_roll_without_discarding_it");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      gui::g_state.renderer.lens_type = gui::kLensTypeFisheyeEquidist;
      gui::g_state.renderer.roll = 15.0f;
      ctx->Yield(3);
      IM_CHECK_EQ(gui::g_state.renderer.roll, 15.0f);
      IM_CHECK(!IsDisabled(ctx->ItemInfo("**/##Roll##view_input")));

      gui::g_state.renderer.lens_type = gui::kLensTypeGlobe;
      ctx->Yield(3);
      IM_CHECK(IsDisabled(ctx->ItemInfo("**/##Roll##view_input")));  // the control says so
      IM_CHECK_EQ(gui::g_state.renderer.roll, 15.0f);                // ...but the value survives

      gui::g_state.renderer.lens_type = gui::kLensTypeFisheyeEquidist;
      ctx->Yield(3);
      IM_CHECK_EQ(gui::g_state.renderer.roll, 15.0f);
      IM_CHECK(!IsDisabled(ctx->ItemInfo("**/##Roll##view_input")));
    };
  }

  // P21. Switching lens through the combo applies a pose correction, because the globe's azimuth
  // convention is the inverse of every other lens's — the user is orbiting a sphere rather than
  // turning their head, so the same sky direction is a different pair of numbers. The transform is
  // its own inverse, and the interesting rows are the seams: 0 must become 180 rather than -180,
  // 180 must wrap to 0 rather than stay at 360, and a beyond-limit elevation must land inside the
  // globe's tighter clamp.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "view_display_controls", "switching_lens_through_the_combo_transforms_the_pose");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      struct Row {
        const char* name;
        int from_lens;
        float az_before;
        float el_before;
        const char* pick;  // the lens NAME to click in the combo
        float az_after;
        float el_after;
      };
      const Row kRows[] = {
        { "into the globe", gui::kLensTypeFisheyeEquidist, 90.0f, 20.0f, "Globe", -90.0f, -20.0f },
        { "out of the globe", gui::kLensTypeGlobe, -90.0f, -20.0f, "Fisheye Equidistant", 90.0f, 20.0f },
        { "azimuth zero is a seam", gui::kLensTypeFisheyeEquidist, 0.0f, 0.0f, "Globe", 180.0f, 0.0f },
        { "azimuth 180 wraps", gui::kLensTypeFisheyeEquidist, 180.0f, 0.0f, "Globe", 0.0f, 0.0f },
        { "elevation clamps into the globe's range", gui::kLensTypeFisheyeEquidist, 0.0f, 91.0f, "Globe", 180.0f,
          -89.0f },
      };

      for (const Row& r : kRows) {
        ResetTestState();
        ctx->Yield(2);
        gui::g_state.renderer.lens_type = r.from_lens;
        gui::g_state.renderer.azimuth = r.az_before;
        gui::g_state.renderer.elevation = r.el_before;
        ctx->Yield(3);

        // The combo button is not in the item registry; ComboClick resolves it by id and scrolls
        // the popup to reveal an entry that starts clipped.
        ctx->SetRef("//##RightPanel");
        ctx->ComboClick((std::string("Lens Type##view/") + r.pick).c_str());
        ctx->SetRef("");
        ctx->Yield(3);

        const auto& v = gui::g_state.renderer;
        if (v.azimuth != r.az_after || v.elevation != r.el_after) {
          IM_ERRORF("%s: landed on az=%f el=%f, expected az=%f el=%f", r.name, static_cast<double>(v.azimuth),
                    static_cast<double>(v.elevation), static_cast<double>(r.az_after), static_cast<double>(r.el_after));
        }

        if (ctx->IsError()) {
          break;
        }
      }
    };
  }

  // P23. The visible-hemisphere radio group and the Front checkbox are gated by different rules,
  // and the whole (lens x widget) grid is one universally quantified claim rather than the handful
  // of cells someone thought to write down. Its sibling in
  // unit-correctness/gui/test_gui_widget_rules.cpp pins the RULE and runs windowless on all three
  // platforms; this half needs a live frame to read the item flags.
  //
  // Expected values are re-derived from LensIsFullSky / kLensTypeGlobe rather than from the field
  // registry the call site now reads, for the reason spelled out above: asking the registry what to
  // expect would compare one line of code against itself.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "view_display_controls", "the_visibility_row_is_gated_per_lens_by_two_rules");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      // One field drawn as three widgets, so all three must move together — which is why they are
      // listed individually rather than sampled.
      const char* const kHemisphere[] = { "**/Upper##visible", "**/Full##visible", "**/Lower##visible" };

      ResetTestState();
      ctx->Yield(2);
      for (int lens = 0; lens < gui::kLensTypeCount; ++lens) {
        gui::g_state.renderer.lens_type = lens;
        ctx->Yield(3);

        const bool full_sky = gui::LensIsFullSky(lens);
        // Front's gate is the wider one: it does not apply under the globe either.
        const bool front_applies = !(full_sky || lens == gui::kLensTypeGlobe);

        for (const char* ref : kHemisphere) {
          if (IsDisabled(ctx->ItemInfo(ref)) != full_sky) {
            IM_ERRORF("lens %d: %s is %s", lens, ref, full_sky ? "enabled" : "disabled");
          }

          if (ctx->IsError()) {
            break;
          }
        }
        if (IsDisabled(ctx->ItemInfo("**/Front##visible")) == front_applies) {
          IM_ERRORF("lens %d: Front is %s", lens, front_applies ? "disabled" : "enabled");
        }

        if (ctx->IsError()) {
          break;
        }
      }
    };
  }

  // P23's other half: the row commits what it says it does, and the checkbox sits on the same line
  // as the radios rather than wrapping to its own — the two are one decision presented as one row.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "view_display_controls", "the_visibility_row_commits_and_stays_on_one_line");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      gui::g_state.renderer.lens_type = gui::kLensTypeFisheyeEqualArea;
      ctx->Yield(3);

      ctx->ItemClick("**/Lower##visible");
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.renderer.visible, gui::kVisibleLower);
      IM_CHECK(!gui::g_state.renderer.front);

      // Front is independent of the hemisphere: setting it must not disturb the radio group.
      ctx->ItemClick("**/Front##visible");
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.renderer.visible, gui::kVisibleLower);
      IM_CHECK(gui::g_state.renderer.front);
      ctx->ItemClick("**/Front##visible");
      ctx->Yield(2);
      IM_CHECK(!gui::g_state.renderer.front);

      const ImGuiTestItemInfo upper = ctx->ItemInfo("**/Upper##visible");
      const ImGuiTestItemInfo front = ctx->ItemInfo("**/Front##visible");
      IM_CHECK_EQ(upper.RectFull.Min.y, front.RectFull.Min.y);
      IM_CHECK_GT(front.RectFull.Min.x, upper.RectFull.Min.x);
    };
  }

  // P25 / P57. The four View sliders at both ends of their domains, including the one bound that is
  // a function of the state: elevation stops one degree short of the pole under the globe and at
  // the pole everywhere else. Driven at BOTH lenses, since a registry that had gone back to a
  // constant would satisfy either one alone.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "view_display_controls", "the_view_sliders_clamp_and_elevation_tightens_on_globe");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      gui::g_state.renderer.lens_type = gui::kLensTypeLinear;
      ctx->Yield(3);

      ctx->ItemInputValue("**/##Elevation##view_input", 200.0f);
      ctx->Yield();
      IM_CHECK_EQ(gui::g_state.renderer.elevation, 90.0f);
      ctx->ItemInputValue("**/##Elevation##view_input", -200.0f);
      ctx->Yield();
      IM_CHECK_EQ(gui::g_state.renderer.elevation, -90.0f);

      ctx->ItemInputValue("**/##Azimuth##view_input", 400.0f);
      ctx->Yield();
      IM_CHECK_EQ(gui::g_state.renderer.azimuth, 180.0f);
      ctx->ItemInputValue("**/##Azimuth##view_input", -400.0f);
      ctx->Yield();
      IM_CHECK_EQ(gui::g_state.renderer.azimuth, -180.0f);

      ctx->ItemInputValue("**/##Roll##view_input", 400.0f);
      ctx->Yield();
      IM_CHECK_EQ(gui::g_state.renderer.roll, 180.0f);
      ctx->ItemInputValue("**/##Roll##view_input", -400.0f);
      ctx->Yield();
      IM_CHECK_EQ(gui::g_state.renderer.roll, -180.0f);

      // Switching to the globe tightens elevation's bound under a value that was legal one frame
      // ago — the slider pulls it in on its own, with nothing else in the frame touching elevation.
      gui::g_state.renderer.lens_type = gui::kLensTypeGlobe;
      ctx->Yield(3);
      IM_CHECK_EQ(gui::g_state.renderer.elevation, -89.0f);
      ctx->ItemInputValue("**/##Elevation##view_input", 200.0f);
      ctx->Yield();
      IM_CHECK_EQ(gui::g_state.renderer.elevation, 89.0f);
    };
  }

  // P22 / P25. Which of the four applies, per lens — the half of the gating that is not a number.
  // The three configurations differ from each other in exactly one field's answer (the globe greys
  // roll and nothing else), so a gate that had collapsed to a single condition fails here.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "view_display_controls", "which_view_sliders_apply_depends_on_the_lens");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      gui::g_state.renderer.lens_type = gui::kLensTypeLinear;
      ctx->Yield(3);
      for (const char* ref : kViewInputs) {
        if (IsDisabled(ctx->ItemInfo(ref))) {
          IM_ERRORF("a rectilinear lens greyed %s", ref);
        }

        if (ctx->IsError()) {
          break;
        }
      }

      gui::g_state.renderer.lens_type = gui::kLensTypeGlobe;
      ctx->Yield(3);
      IM_CHECK(!IsDisabled(ctx->ItemInfo("**/##FOV##view_input")));
      IM_CHECK(!IsDisabled(ctx->ItemInfo("**/##Elevation##view_input")));
      IM_CHECK(!IsDisabled(ctx->ItemInfo("**/##Azimuth##view_input")));
      IM_CHECK(IsDisabled(ctx->ItemInfo("**/##Roll##view_input")));

      // Full sky: no view angle applies at all, and the angles already stored are zeroed rather
      // than merely greyed. The second half is load-bearing outside this suite:
      // field_editor_registry.hpp's argument for why the defaults panel cannot save an unreachable
      // (roll, full-sky lens) pair rests on RenderPreviewPanel doing this on the very next frame,
      // before any Save can observe the field — that comment names this case. Greying alone would
      // not settle it, since the panel writes the field directly rather than through the widget.
      //
      // Iterates the same constant the product does, so a lens added to the set is covered here
      // without anyone adding a row.
      for (const int lens : gui::kFullSkyLensTypes) {
        gui::g_state.renderer.roll = 15.0f;
        gui::g_state.renderer.elevation = 20.0f;
        gui::g_state.renderer.lens_type = lens;
        ctx->Yield(3);
        if (gui::g_state.renderer.roll != 0.0f || gui::g_state.renderer.elevation != 0.0f) {
          IM_ERRORF("full-sky lens %d kept roll=%f elevation=%f instead of zeroing them", lens,
                    static_cast<double>(gui::g_state.renderer.roll),
                    static_cast<double>(gui::g_state.renderer.elevation));
        }
        for (const char* ref : kViewInputs) {
          if (!IsDisabled(ctx->ItemInfo(ref))) {
            IM_ERRORF("full-sky lens %d left %s enabled", lens, ref);
          }

          if (ctx->IsError()) {
            break;
          }
        }

        if (ctx->IsError()) {
          break;
        }
      }
    };
  }

  // The same invariant as the case above, quantified over the three ways a document ARRIVES rather
  // than over the ways it is edited. It matters separately because the loaders are transcripts:
  // ParseRendererFromGuiJson (.lmc), DeserializeFromJson (CLI JSON import) and the personal-defaults
  // overlay each write renderer.elevation/azimuth/roll straight out of the file with no full-sky
  // rule of their own, and all four keys are default-eligible, so a hand-edited file really can put
  // a full-sky lens and a non-zero pose into GuiState together.
  //
  // Enforcement is one place and one place only — the renderer-invariant block at the top of
  // RenderPreviewPanel, which runs every frame — so what this asserts is that the block covers the
  // arrival paths and not just the widgets. The second half of each check is the part that says why
  // anyone should care: the exported CLI config, the one artifact whose pose changes what core
  // renders, must come out zeroed too. Since core's rectangular projection follows the full camera
  // pose (2026-09-02), an export that carried elevation 20 here would render a tilted all-sky map
  // that the preview never showed.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "view_display_controls", "an_arriving_document_cannot_smuggle_a_full_sky_pose");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();

      // The document every installer below carries.
      gui::GuiState doctored = gui::InitDefaultState();
      doctored.renderer.lens_type = gui::kLensTypeRectangular;
      doctored.renderer.elevation = 20.0f;
      doctored.renderer.azimuth = 35.0f;
      doctored.renderer.roll = -15.0f;

      // Post-arrival check, shared by the three installers: one frame, then the pose is gone from
      // the state AND from what the state would export.
      auto settled_pose_is_zero = [](ImGuiTestContext* c, const char* via) {
        c->Yield(3);
        const auto& r = gui::g_state.renderer;
        if (r.elevation != 0.0f || r.azimuth != 0.0f || r.roll != 0.0f) {
          IM_ERRORF("%s: the arriving pose survived (el=%f az=%f ro=%f)", via, static_cast<double>(r.elevation),
                    static_cast<double>(r.azimuth), static_cast<double>(r.roll));
          return;
        }
        if (!gui::LensIsFullSky(r.lens_type)) {
          IM_ERRORF("%s: the lens did not arrive; the pose check above proves nothing", via);
          return;
        }
        std::string exported;
        if (!gui::BuildExportJsonOrWarn(gui::g_state, &exported, nullptr)) {
          IM_ERRORF("%s: the exported config could not be built", via);
          return;
        }
        const nlohmann::json doc = nlohmann::json::parse(exported, nullptr, false);
        if (doc.is_discarded() || !doc.contains("render") || !doc["render"].is_array() || doc["render"].empty()) {
          IM_ERRORF("%s: the exported config has no render block to read", via);
          return;
        }
        const nlohmann::json view = doc["render"][0].value("view", nlohmann::json::object());
        if (view.value("elevation", 0.0) != 0.0 || view.value("azimuth", 0.0) != 0.0 ||
            view.value("roll", 0.0) != 0.0) {
          IM_ERRORF("%s: the exported config carries a full-sky pose: %s", via, view.dump().c_str());
        }
      };

      // 1. .lmc — the GUI's own document format.
      {
        const std::string path = GuiTestTempPath("full_sky_pose.lmc").string();
        IM_CHECK(gui::SaveLmcFile(path, doctored, gui::g_preview, /*save_texture=*/false));
        gui::DoOpen(path);
        settled_pose_is_zero(ctx, ".lmc load");
        std::remove(path.c_str());
      }

      // 2. CLI JSON import. The vehicle is the export arm itself: it transcribes the pose, which is
      // exactly what makes a config written by an older build (or by hand) able to carry one back in.
      if (!ctx->IsError()) {
        ResetTestState();
        std::string json;
        IM_CHECK(gui::BuildExportJsonOrWarn(doctored, &json, nullptr));
        const std::string path = GuiTestTempPath("full_sky_pose.json").string();
        IM_CHECK(gui::ExportConfigJson(path, json));
        gui::DoOpen(path);
        settled_pose_is_zero(ctx, "CLI JSON import");
        std::remove(path.c_str());
      }

      // 3. Personal defaults. Not reachable through the settings panel — it greys these rows under a
      // full-sky lens — but the override file is user-editable JSON, and MakeNewDocumentState applies
      // whatever it finds. This is the path field_editor_registry.hpp's argument leans on.
      if (!ctx->IsError()) {
        ResetTestState();
        const std::filesystem::path dir = lumice::test_user_defaults::FreshOverlayDir("full_sky_pose");
        nlohmann::json overlay = nlohmann::json::object();
        const std::vector<gui::DefaultDiffRow> rows = gui::BuildDefaultDiffRows(doctored, overlay);
        IM_CHECK(gui::ApplyCheckedRowsToDoc(
            overlay, rows, { "renderer.lens_type", "renderer.elevation", "renderer.azimuth", "renderer.roll" },
            doctored));
        IM_CHECK(gui::WriteUserDefaultsFile(dir, overlay));
        gui::g_state = gui::MakeNewDocumentState(dir);
        settled_pose_is_zero(ctx, "personal defaults");
        lumice::test_user_defaults::ResetUserDefaultsChannels();
      }

      ResetTestState();
    };
  }

  // P26. Reset returns the four view fields to the defaults OF THE CURRENT LENS, which are not the
  // same defaults for every lens — and it must leave the lens itself and the visibility mode alone,
  // or it stops being a view reset and becomes a "start over".
  //
  // Two lenses rather than all eleven: what is under test is that the button consults the lens at
  // all, and DefaultViewParamsFor's own table is covered at the unit layer.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "view_display_controls", "reset_returns_the_view_to_the_lenses_own_defaults");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      IM_CHECK(ctx->ItemExists("**/Reset##view"));

      const int kLenses[] = { gui::kLensTypeFisheyeEquidist, gui::kLensTypeGlobe };
      for (const int lens : kLenses) {
        ResetTestState();
        ctx->Yield(2);
        gui::g_state.renderer.lens_type = lens;
        ctx->Yield(3);
        // Captured after the lens-switch frame settles, so it reflects any per-frame normalization
        // the new lens applies rather than the value from before the switch.
        const int visible_before = gui::g_state.renderer.visible;

        gui::g_state.renderer.fov = 42.0f;
        gui::g_state.renderer.elevation = 33.0f;
        gui::g_state.renderer.azimuth = -77.0f;
        gui::g_state.renderer.roll = 11.0f;
        ctx->Yield(3);

        ctx->ItemClick("**/Reset##view");
        ctx->Yield(3);

        const gui::ViewDefaults def = gui::DefaultViewParamsFor(lens);
        const auto& v = gui::g_state.renderer;
        if (v.fov != def.fov || v.elevation != def.elevation || v.azimuth != def.azimuth || v.roll != def.roll) {
          IM_ERRORF("lens %d: reset left fov=%f el=%f az=%f roll=%f, expected %f/%f/%f/%f", lens,
                    static_cast<double>(v.fov), static_cast<double>(v.elevation), static_cast<double>(v.azimuth),
                    static_cast<double>(v.roll), static_cast<double>(def.fov), static_cast<double>(def.elevation),
                    static_cast<double>(def.azimuth), static_cast<double>(def.roll));
        }
        if (v.lens_type != lens || v.visible != visible_before) {
          IM_ERRORF("lens %d: reset also changed the lens or the visibility mode", lens);
        }

        if (ctx->IsError()) {
          break;
        }
      }
    };
  }

  // The Display group's exposure slider, at both ends. Literals, for the same reason as everywhere
  // else in this file.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "view_display_controls", "the_exposure_slider_clamps_to_its_declared_domain");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      ctx->ItemInputValue("**/##EV##display_input", 40.0f);
      ctx->Yield();
      IM_CHECK_EQ(gui::g_state.renderer.exposure_offset, 16.0f);
      ctx->ItemInputValue("**/##EV##display_input", -40.0f);
      ctx->Yield();
      IM_CHECK_EQ(gui::g_state.renderer.exposure_offset, -8.0f);
    };
  }

  // P31. The background alpha is the one migrated field whose gate has two conditions — no image
  // loaded, OR an image that is loaded but hidden.
  //
  // KNOWN LIMIT, so the next reader does not take this for more than it is: the "no image loaded"
  // half is NOT distinguishable from here. That slider sits inside an outer disabled scope which
  // also wraps the Show checkbox and therefore stays, and which greys the slider on its own whether
  // or not the field's own gate agrees. What this case DOES separate is the other half — with an
  // image loaded, greyed follows the Show toggle, and that is the condition the call site used to
  // compute for itself.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "view_display_controls", "the_background_alpha_follows_the_show_toggle");
    t->GuiFunc = [](ImGuiTestContext*) {
      // UploadBgTexture is a GL call, so it has to happen on the main thread rather than in the
      // test coroutine.
      if (g_bg_upload_requested) {
        static const unsigned char kPixels[2 * 2 * 3] = { 255, 0, 0, 0, 255, 0, 0, 0, 255, 255, 255, 255 };
        gui::g_preview.UploadBgTexture(kPixels, 2, 2);
        g_bg_upload_requested = false;
      }
    };
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ScopedBackground restore_background;
      ctx->Yield(2);
      IM_CHECK(!gui::g_preview.HasBackground());

      g_bg_upload_requested = true;
      ctx->Yield(3);
      IM_CHECK(gui::g_preview.HasBackground());

      gui::g_state.bg_show = false;
      ctx->Yield(3);
      IM_CHECK(IsDisabled(ctx->ItemInfo("**/##Alpha##display_input")));

      gui::g_state.bg_show = true;
      ctx->Yield(3);
      IM_CHECK(!IsDisabled(ctx->ItemInfo("**/##Alpha##display_input")));

      ctx->ItemInputValue("**/##Alpha##display_input", 7.5f);
      ctx->Yield();
      IM_CHECK_EQ(gui::g_state.bg_alpha, 1.0f);
      ctx->ItemInputValue("**/##Alpha##display_input", -3.0f);
      ctx->Yield();
      IM_CHECK_EQ(gui::g_state.bg_alpha, 0.0f);
      // The background is handed back by ScopedBackground; see it for why not here.
    };
  }

  // The pan/zoom sliders: the OTHER end of the same three fields the canvas gesture writes.
  //
  // The point of a case here, given functional/test_preview_viewport.cpp already drives the
  // gesture, is that these two entry points are one owner and not two. A shadow copy committed on
  // mouse-up would satisfy every assertion over there and still leave the slider showing a stale
  // number — so what is checked is that typing lands on the same `g_state` float the gesture
  // moves, and that the domain the slider advertises is the one actually enforced.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "view_display_controls",
                                    "the_background_pan_and_zoom_sliders_gate_and_clamp_like_the_alpha");
    t->GuiFunc = [](ImGuiTestContext*) {
      if (g_bg_upload_requested) {
        static const unsigned char kPixels[2 * 2 * 3] = { 255, 0, 0, 0, 255, 0, 0, 0, 255, 255, 255, 255 };
        gui::g_preview.UploadBgTexture(kPixels, 2, 2);
        g_bg_upload_requested = false;
      }
    };
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ScopedBackground restore_background;
      ctx->Yield(2);

      g_bg_upload_requested = true;
      ctx->Yield(3);
      IM_CHECK(gui::g_preview.HasBackground());

      // Addressed through the right panel rather than with a `**/` wildcard: a wildcard resolves a
      // LABEL and a clipped item is registered by id alone, and these three sit low enough in the
      // Background group to be past the fold at the harness window size. Window-relative ids
      // resolve and scroll into view. Same reasoning as the grid alpha in test_defaults_panel.cpp.
      ctx->SetRef("##RightPanel");

      // Written out rather than looped: IM_CHECK expands to a `return`, so a loop would report the
      // first greyed-out row and silently skip the rest — the shape check_loop_fatal_asserts.py
      // exists to reject. Three rows do not need the loop anyway.
      gui::g_state.bg_show = false;
      ctx->Yield(3);
      IM_CHECK(IsDisabled(ctx->ItemInfo("##Offset X##display_bg_input")));
      IM_CHECK(IsDisabled(ctx->ItemInfo("##Offset Y##display_bg_input")));
      IM_CHECK(IsDisabled(ctx->ItemInfo("##Zoom##display_bg_input")));

      gui::g_state.bg_show = true;
      ctx->Yield(3);
      IM_CHECK(!IsDisabled(ctx->ItemInfo("##Offset X##display_bg_input")));
      IM_CHECK(!IsDisabled(ctx->ItemInfo("##Offset Y##display_bg_input")));
      IM_CHECK(!IsDisabled(ctx->ItemInfo("##Zoom##display_bg_input")));

      // Both ends of each domain, driven past them. The reported value is the field itself, so a
      // clamp that lived only in the widget's display would not satisfy this.
      ctx->ItemInputValue("##Offset X##display_bg_input", 9.0f);
      ctx->Yield();
      IM_CHECK_EQ(gui::g_state.bg_offset_x, 2.0f);
      ctx->ItemInputValue("##Offset X##display_bg_input", -9.0f);
      ctx->Yield();
      IM_CHECK_EQ(gui::g_state.bg_offset_x, -2.0f);

      ctx->ItemInputValue("##Offset Y##display_bg_input", 9.0f);
      ctx->Yield();
      IM_CHECK_EQ(gui::g_state.bg_offset_y, 2.0f);
      ctx->ItemInputValue("##Offset Y##display_bg_input", -9.0f);
      ctx->Yield();
      IM_CHECK_EQ(gui::g_state.bg_offset_y, -2.0f);

      // Zoom is multiplicative and its domain is asymmetric, which is why it is spelled out rather
      // than folded into the loop above: 0 is not merely out of range, it is the value that would
      // make the UV transform divide by zero if it ever reached the renderer.
      ctx->ItemInputValue("##Zoom##display_bg_input", 40.0f);
      ctx->Yield();
      IM_CHECK_EQ(gui::g_state.bg_scale, 5.0f);
      ctx->ItemInputValue("##Zoom##display_bg_input", 0.0f);
      ctx->Yield();
      IM_CHECK_EQ(gui::g_state.bg_scale, 0.2f);

      ctx->SetRef("");
      // The background is handed back by ScopedBackground.
    };
  }

  // The View and Display groups' half of the trailing-label column, stated per group because each
  // group is its own PushItemWidth scope and the two could drift apart without either one being
  // internally inconsistent. Both families are present in each: hand-built [slider][input] rows
  // from panels.cpp, and ImGui Combos that place their own label at a spacing constant hardcoded
  // inside BeginCombo. Control right edges are asserted alongside label left edges — squaring up
  // the labels by widening one family's controls only would trade one visible misalignment for
  // another.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "view_display_controls", "the_trailing_label_column_is_one_line_across_row_families");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(3);

      // Collected before anything is asserted: every ImGuiTestContext action opens with
      // `if (IsError()) return;`, so an assertion between two ItemInfo calls would leave the rest
      // of the rows unmeasured and report the first offender as if it were the only one.
      // Addressed under the panel window rather than through "**/": a wildcard resolves by the
      // debug label an item registers, and BeginCombo registers none — the three combo rows would
      // be unreachable, which are the rows the invariant is about.
      ctx->SetRef("//##RightPanel");
      std::vector<LabelColumnRow> view;
      view.push_back(MeasureComboRow(ctx, "Lens Type", "Lens Type##view"));
      view.push_back(MeasureWidgetRow(ctx, "FOV", "##FOV##view_input", "##FOV##view_label"));
      view.push_back(MeasureWidgetRow(ctx, "Elevation", "##Elevation##view_input", "##Elevation##view_label"));
      view.push_back(MeasureWidgetRow(ctx, "Azimuth", "##Azimuth##view_input", "##Azimuth##view_label"));
      view.push_back(MeasureWidgetRow(ctx, "Roll", "##Roll##view_input", "##Roll##view_label"));

      std::vector<LabelColumnRow> display;
      display.push_back(MeasureComboRow(ctx, "Resolution", "Resolution##display"));
      display.push_back(MeasureComboRow(ctx, "Mode", "Mode##display"));
      display.push_back(MeasureWidgetRow(ctx, "EV", "##EV##display_input", "##EV##display_label"));
      display.push_back(MeasureComboRow(ctx, "Preset", "Preset##display_aspect"));
      display.push_back(MeasureWidgetRow(ctx, "Alpha", "##Alpha##display_input", "##Alpha##display_label"));
      display.push_back(
          MeasureWidgetRow(ctx, "Offset X", "##Offset X##display_bg_input", "##Offset X##display_bg_label"));
      display.push_back(
          MeasureWidgetRow(ctx, "Offset Y", "##Offset Y##display_bg_input", "##Offset Y##display_bg_label"));
      display.push_back(MeasureWidgetRow(ctx, "Zoom", "##Zoom##display_bg_input", "##Zoom##display_bg_label"));
      ctx->SetRef("");

      CheckLabelColumn("view", view);
      CheckLabelColumn("display", display);
    };
  }

  // The exposure Mode combo, and the two claims that make it worth having.
  //
  // (1) It re-lights the picture in the SAME frame. That is not decoration: it is the entire
  //     difference between this control and the simulation-semantics toggle that was removed
  //     years ago. This one changes which anchor the client divides an already-simulated frame
  //     by, so it costs nothing; that one changed what was simulated and cost a second run. A
  //     case that only asserted `g_state.renderer.ev_mode == 1` would pass just as well against
  //     an implementation that quietly needed a re-run to show anything, which is the exact
  //     regression worth guarding.
  // (2) It does not disturb the finished run. sim_state stays kDone and run_intent stays
  //     kRunCompleted — a mode switch must not offer to re-simulate its way to the same rays.
  //
  // Needs a real sim because the preview only computes an exposure when it has a texture, and
  // absolute mode divides by emitted energy, which only a real snapshot carries.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "view_display_controls",
                                    "the_exposure_mode_combo_relights_the_same_frame_without_re_running");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      gui::g_server_poller.Stop();
      gui::g_server = LUMICE_CreateServer();
      IM_CHECK(gui::g_server != nullptr);
      gui::g_server_is_gpu = false;
      gui::g_state = gui::InitDefaultState();
      gui::g_state.sim.infinite = false;
      gui::g_state.sim.ray_num_millions = 0.2f;
      gui::g_state.sim.max_hits = 8;

      gui::DoRun(/*user_initiated=*/true);
      const auto start = std::chrono::steady_clock::now();
      while (gui::g_state.sim_state != gui::GuiState::SimState::kDone ||
             gui::g_state.run_intent != gui::RunIntent::kRunCompleted) {
        ctx->Yield();
        if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start).count() > 20) {
          break;
        }
      }
      IM_CHECK_EQ(gui::g_state.sim_state, gui::GuiState::SimState::kDone);
      // The premise of everything below: absolute mode's denominator actually arrived from the
      // C API. Without it the two modes would agree on a scale of zero and the case would pass
      // while proving nothing.
      IM_CHECK_GT(gui::g_state.snapshot_emitted_energy, 0.0f);
      IM_CHECK_GT(gui::g_state.snapshot_intensity, 0.0f);

      ctx->Yield();
      const float scale_relative = gui::g_preview_vp.params.exposure.intensity_scale;
      IM_CHECK_GT(scale_relative, 0.0f);

      // One combo pick, then exactly one frame.
      PickExposureMode(ctx, "Absolute");
      IM_CHECK_EQ(gui::g_state.renderer.ev_mode, 1);
      const float scale_before_next_frame = gui::g_preview_vp.params.exposure.intensity_scale;
      ctx->Yield();
      const float scale_absolute = gui::g_preview_vp.params.exposure.intensity_scale;

      // Same-frame, stated as a comparison rather than as a bare inequality: the uniform the
      // preview will draw with now equals what the absolute formula says it should be, computed
      // here from the same GuiState the panel read.
      lumice::gui::MonoExposureInput expected_in;
      expected_in.exposure_offset = gui::g_state.renderer.exposure_offset;
      expected_in.ev_auto = gui::g_state.ev_auto;
      expected_in.snapshot_intensity = gui::g_state.snapshot_intensity;
      expected_in.snapshot_emitted_energy = gui::g_state.snapshot_emitted_energy;
      expected_in.total_pixels = gui::g_preview.GetTextureWidth() * gui::g_preview.GetTextureHeight();
      const float expected =
          lumice::gui::ComputeMonoExposure(lumice::gui::MonoEvMode::kAbsolute, expected_in).intensity_scale;
      IM_CHECK_GT(expected, 0.0f);
      IM_CHECK_LT(std::abs(scale_absolute - expected), expected * 1e-4f);
      // And it really moved — otherwise "matches the absolute formula" would be satisfied by the
      // two formulas happening to agree on this scene.
      IM_CHECK(std::abs(scale_absolute - scale_relative) > scale_relative * 1e-3f);
      // The value captured before the yield is reported, not asserted: the panel may or may not
      // have already been drawn this frame when the popup closed, so demanding it here would be
      // asserting a frame-ordering detail rather than the same-frame promise. What the promise
      // means is that no poll, commit or run stood between the click and the new exposure — which
      // the run-state checks below are what actually pin.
      ctx->LogInfo("scale before yield %.6f, after %.6f", scale_before_next_frame, scale_absolute);

      // No re-run was asked for, and none happened.
      IM_CHECK_EQ(gui::g_state.sim_state, gui::GuiState::SimState::kDone);
      IM_CHECK_EQ(gui::g_state.run_intent, gui::RunIntent::kRunCompleted);
      // The unsaved marker does NOT move, and that is measured here rather than assumed, because
      // it is the one place this control's behaviour is worth stating out loud: `dirty` in this
      // app is raised only by ApplyGuiEffects, off the resim / hard-reset lanes, so no display-time
      // field raises it. `exposure_offset` has behaved this way for as long as it has existed, and
      // ev_mode is deliberately in the same class (gui_state.hpp: it is excluded from
      // RenderConfigResimFields for the same reason EV is). Both are serialized into the .lmc, so
      // there IS a gap — change either one, close without saving, and nothing prompts — but it is
      // one gap belonging to a whole class of fields, not a property of this control, and closing
      // it for ev_mode alone would make the two halves of the exposure row disagree.
      //
      // The EV slider is driven right below so the claim is a measured parity rather than a
      // reading of the reconciler: whatever `dirty` does for EV, it does for the mode.
      IM_CHECK_EQ(gui::g_state.dirty, false);
      const bool dirty_after_mode = gui::g_state.dirty;
      ctx->ItemInputValue("//##RightPanel/##EV##display_input", 1.5f);
      ctx->Yield();
      IM_CHECK_EQ(gui::g_state.renderer.exposure_offset, 1.5f);  // the drag really landed
      IM_CHECK_EQ(gui::g_state.dirty, dirty_after_mode);
      gui::g_state.renderer.exposure_offset = 0.0f;
      ctx->Yield();

      // Back to Relative, and the scale comes back with it.
      PickExposureMode(ctx, "Relative");
      ctx->Yield();
      IM_CHECK_EQ(gui::g_state.renderer.ev_mode, 0);
      IM_CHECK_LT(std::abs(gui::g_preview_vp.params.exposure.intensity_scale - scale_relative), scale_relative * 1e-3f);

      gui::g_server_poller.Stop();
      LUMICE_StopServer(gui::g_server);
      LUMICE_DestroyServer(gui::g_server);
      gui::g_server = nullptr;
      gui::g_state.run_intent = gui::RunIntent::kNone;
      gui::g_state.committed_epoch = 0;
      gui::g_state.display_epoch_floor = 0;
    };
  }

  // The permanent EV readout under the slider.
  //
  // It is the answer to a mismatch this GUI structurally cannot detect: exposure_offset is saved
  // per document, so in absolute mode two files sit at two heights on one shared scale and a
  // single-document app has nothing to compare them against. It cannot warn; what it can do is
  // never leave the current height implicit, so the height is on screen at all times rather than
  // inside a tooltip nobody hovers.
  //
  // Asserted through the item's own label, which IS the rendered text: the readout is a disabled
  // Selectable whose "##" id folds the whole label in, so locating it by the exact string the
  // formatter produces is a claim about the text on screen and not merely about a widget existing.
  // ("###" would have discarded everything before it — ImHashStr resets on the triple form — and
  // the two modes' labels would then be indistinguishable to a lookup.) Composing the expected
  // string from FormatMonoEvReadout rather than retyping it keeps this from becoming a second copy
  // of the format.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "view_display_controls", "the_ev_readout_states_the_mode_and_height_on_screen");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      ctx->SetRef("//##RightPanel");

      // The two phases deliberately use DIFFERENT numbers, so no label string is looked up twice
      // across the case: ItemInfo registers a task keyed by the resolved path and keeps the answer
      // for the rest of the run, which would make a second lookup of a now-absent label report the
      // id it found the first time.
      //
      // Phase 1 — absolute, with a sparse-scene auto anchor sitting far from zero. That anchor is
      // exactly the number the old readout showed drifting while the picture did not move, so the
      // claim here is as much about what is NOT said as about what is.
      gui::g_state.renderer.exposure_offset = 2.0f;
      gui::g_state.ev_auto = -5.66f;
      gui::g_state.renderer.ev_mode = 1;
      ctx->Yield();
      const std::string absolute_label = lumice::gui::FormatMonoEvReadout(1, 2.0f, -5.66f) + "##display_ev_readout";
      const std::string wrong_relative = lumice::gui::FormatMonoEvReadout(0, 2.0f, -5.66f) + "##display_ev_readout";
      IM_CHECK(absolute_label != wrong_relative);
      IM_CHECK(ctx->ItemInfo(absolute_label.c_str(), ImGuiTestOpFlags_NoError).ID != 0);
      // And the reason a bare "does it exist" would not do: the relative wording must be ABSENT
      // here. A readout that ignored the mode and always printed the relative form — auto folded
      // in, the misleading reading this replaces — would satisfy the check above on its own.
      IM_CHECK_EQ(ctx->ItemInfo(wrong_relative.c_str(), ImGuiTestOpFlags_NoError).ID, (ImGuiID)0);

      // Phase 2 — relative, different numbers, both directions again.
      gui::g_state.renderer.exposure_offset = -1.5f;
      gui::g_state.ev_auto = 3.25f;
      gui::g_state.renderer.ev_mode = 0;
      ctx->Yield();
      const std::string relative_label = lumice::gui::FormatMonoEvReadout(0, -1.5f, 3.25f) + "##display_ev_readout";
      const std::string wrong_absolute = lumice::gui::FormatMonoEvReadout(1, -1.5f, 3.25f) + "##display_ev_readout";
      IM_CHECK(ctx->ItemInfo(relative_label.c_str(), ImGuiTestOpFlags_NoError).ID != 0);
      IM_CHECK_EQ(ctx->ItemInfo(wrong_absolute.c_str(), ImGuiTestOpFlags_NoError).ID, (ImGuiID)0);

      // ... and it is not clipped, which "the item exists" does not cover and which the first
      // draft of this readout got wrong: the relative wording spelled out "manual"/"effective",
      // measured 308px against the Display group's 284px of content width, and drew with its last
      // number cut off — a permanent readout that exists precisely so a number is never implicit,
      // with the number missing.
      //
      // Read off the item rather than recomputed from the string: a Selectable's box is
      // max(label width, available width) extended by half the item spacing on each side
      // (imgui_widgets.cpp), so RectFull always runs a little past the window's work rect and the
      // clip rect always takes that little back. What only happens when the text does NOT fit is
      // the clip rect biting deeper than that allowance. Measured on the pre-fix wording, the
      // difference was 37px against an 8px allowance; on the shipped wording it is 0px.
      const ImGuiTestItemInfo readout_info = ctx->ItemInfo(relative_label.c_str());
      IM_CHECK_LE(readout_info.RectFull.GetWidth() - readout_info.RectClipped.GetWidth(),
                  ImGui::GetStyle().ItemSpacing.x);

      ctx->SetRef("");
    };
  }

  // ---- Look At presets ----------------------------------------------------------------------
  //
  // What a user sees when these break: they pick "Antisolar" and the camera turns somewhere else,
  // or it turns to the right place and quietly loses the roll and field of view they had set, or
  // Globe hands them a pose its own slider cannot express so the next touch of that slider snaps
  // the view away.
  //
  // Where the ARITHMETIC is judged: unit-correctness/gui/test_view_look_at.cpp, against analytic
  // relations (the antisolar direction is 180 degrees from the sun, and so on) with its own
  // independently written oracle. Nothing below restates that. These cases are about the parts only
  // a real frame can answer — that the entry is gated by the same rule as the sliders it writes,
  // that what reaches g_state is the CLAMPED value, that the fields it must not touch are
  // untouched, and that the menu shows the same words the Overlay list does.
  //
  // Deliberately NOT covered here: the tooltip TEXT on the disabled entry. It is drawn through
  // ImGui::SetTooltip, i.e. TextUnformatted with id == 0, which the test engine's item registry
  // never sees — the same mechanical limit this file already records for the Resolution tooltip and
  // test_color_window.cpp records for its own. That the registry has a reason to show at all is
  // asserted one layer down, in test_view_look_at.cpp.

  // P-LookAt-1 / AC1. The gate is the sliders' gate. Asserted across the lens space rather than at
  // one lens, because "same rule" is a claim about where the two answers agree AND disagree: a
  // hand-written `lens == globe` condition would pass a globe-only check and fail here.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "view_display_controls", "the_look_at_entry_is_gated_exactly_like_the_view_angles");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Every lens the product offers, iterated from the product's own list so a lens added later
      // is covered here without anyone adding a row.
      for (const int lens : gui::kLensTypePresentationOrder) {
        gui::g_state.renderer.lens_type = lens;
        ctx->Yield(3);
        const ImGuiTestItemInfo look_at = ctx->ItemInfo(kLookAtButton, ImGuiTestOpFlags_NoError);
        const ImGuiTestItemInfo elevation = ctx->ItemInfo("**/##Elevation##view_input", ImGuiTestOpFlags_NoError);
        if (look_at.ID == 0) {
          IM_ERRORF("lens %d does not offer the Look At entry at all", lens);
        } else if (IsDisabled(look_at) != IsDisabled(elevation)) {
          IM_ERRORF("lens %d: Look At disabled=%d but the Elevation slider disabled=%d", lens,
                    static_cast<int>(IsDisabled(look_at)), static_cast<int>(IsDisabled(elevation)));
        }

        if (ctx->IsError()) {
          break;
        }
      }
    };
  }

  // P-LookAt-2 / AC2. The value that lands in g_state is the CLAMPED one, and Globe is where that
  // is visible: its elevation stops one degree short of the pole, so an unclamped Zenith writes
  // 90 into a field whose slider tops out at 89. The re-read after further frames is the second
  // half — a preset that wrote 90 and let the slider pull it back to 89 next frame would satisfy a
  // "the final value is 89" check on its own, while having spent a frame in a pose the panel says
  // is impossible.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "view_display_controls", "a_preset_writes_the_pose_the_sliders_own_bounds_allow");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      const ScopedPopups popup_guard(ctx);
      ctx->Yield(2);

      gui::g_state.renderer.lens_type = gui::kLensTypeGlobe;
      ctx->Yield(3);
      PickLookAt(ctx, "Zenith");
      IM_CHECK_EQ(gui::g_state.renderer.elevation, 89.0f);
      ctx->Yield(4);
      IM_CHECK_EQ(gui::g_state.renderer.elevation, 89.0f);  // nothing pulled it afterwards

      PickLookAt(ctx, "Nadir");
      IM_CHECK_EQ(gui::g_state.renderer.elevation, -89.0f);
      ctx->Yield(4);
      IM_CHECK_EQ(gui::g_state.renderer.elevation, -89.0f);

      // The same two presets under a lens whose bound is the pole itself: 90, not 89. Without this
      // half, an implementation that hard-coded 89 would pass everything above.
      gui::g_state.renderer.lens_type = gui::kLensTypeLinear;
      ctx->Yield(3);
      PickLookAt(ctx, "Zenith");
      IM_CHECK_EQ(gui::g_state.renderer.elevation, 90.0f);
      PickLookAt(ctx, "Nadir");
      IM_CHECK_EQ(gui::g_state.renderer.elevation, -90.0f);
    };
  }

  // P-LookAt-3 / AC3. Roll and field of view are the user's framing; "look at that" does not
  // include "and throw away how I had it framed". Bitwise, not approximate — the claim is that the
  // fields were not written, and any tolerance would let a write that happened to be small pass.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "view_display_controls", "a_preset_leaves_roll_and_fov_untouched");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      const ScopedPopups popup_guard(ctx);
      ctx->Yield(2);

      gui::g_state.renderer.lens_type = gui::kLensTypeLinear;
      ctx->Yield(3);
      gui::g_state.renderer.roll = 33.0f;
      gui::g_state.renderer.fov = 47.0f;
      gui::g_state.sun.altitude = 30.0f;
      ctx->Yield(3);
      const float roll_before = gui::g_state.renderer.roll;
      const float fov_before = gui::g_state.renderer.fov;

      // Every entry, not one: the fields the presets share are written in one place, but the loop
      // is what says no single entry grew its own side effect.
      const char* const kEntries[] = {
        "Zenith", "Nadir", "Sun", "Subsun", "Anthelion", "Antisolar", "Sun-side horizon"
      };
      for (const char* entry : kEntries) {
        PickLookAt(ctx, entry);
        if (gui::g_state.renderer.roll != roll_before || gui::g_state.renderer.fov != fov_before) {
          IM_ERRORF("%s changed roll %f->%f or fov %f->%f", entry, static_cast<double>(roll_before),
                    static_cast<double>(gui::g_state.renderer.roll), static_cast<double>(fov_before),
                    static_cast<double>(gui::g_state.renderer.fov));
        }

        if (ctx->IsError()) {
          break;
        }
      }
    };
  }

  // P-LookAt-4. The wiring, end to end through the real widget: the preset reads the CURRENT sun
  // altitude out of g_state and the angles it computes reach the renderer fields. Two altitudes,
  // one of them below the horizon, because a call site that passed a constant — or read the wrong
  // field — would satisfy a single-altitude check.
  //
  // The relations themselves (elevation equals the sun's altitude; the antisolar point is a half
  // turn away and mirrored) are the same ones the unit layer proves against its own oracle. What is
  // new here is only that this menu, in this panel, is connected to them.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "view_display_controls", "the_sun_presets_follow_the_current_sun_altitude");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      const ScopedPopups popup_guard(ctx);
      ctx->Yield(2);
      gui::g_state.renderer.lens_type = gui::kLensTypeLinear;
      ctx->Yield(3);

      const auto check_altitude = [ctx](float altitude) {
        gui::g_state.sun.altitude = altitude;
        ctx->Yield(3);

        PickLookAt(ctx, "Sun");
        IM_CHECK_LT(std::fabs(gui::g_state.renderer.elevation - altitude), 1e-3f);
        IM_CHECK_LT(std::fabs(gui::g_state.renderer.azimuth), 1e-3f);

        PickLookAt(ctx, "Antisolar");
        IM_CHECK_LT(std::fabs(gui::g_state.renderer.elevation + altitude), 1e-3f);
        IM_CHECK_LT(std::fabs(std::fabs(gui::g_state.renderer.azimuth) - 180.0f), 1e-3f);

        PickLookAt(ctx, "Sun-side horizon");
        IM_CHECK_LT(std::fabs(gui::g_state.renderer.elevation), 1e-3f);
        IM_CHECK_LT(std::fabs(gui::g_state.renderer.azimuth), 1e-3f);
      };
      check_altitude(30.0f);
      check_altitude(-12.0f);  // the sun has set; its subsun and anthelion have not
    };
  }

  // P-LookAt-5 / AC6. One direction, one name. The six marker entries are looked up by the strings
  // the Overlay list itself is built from, so a rename that reached only one of the two lists
  // leaves this case unable to find the entry.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "view_display_controls", "the_menu_names_the_directions_the_overlay_does");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      const ScopedPopups popup_guard(ctx);
      ctx->Yield(2);
      gui::g_state.renderer.lens_type = gui::kLensTypeLinear;
      ctx->Yield(3);

      ctx->SetRef("//##RightPanel");
      ctx->ItemClick(kLookAtButton);
      ctx->Yield(2);
      ctx->SetRef("//$FOCUSED");
      for (int i = 0; i < LUMICE_ANNOTATION_MARKER_COUNT; ++i) {
        const std::string entry = std::string("**/") + gui::kMarkerDisplayNames[i];
        if (!ctx->ItemExists(entry.c_str())) {
          IM_ERRORF("the menu has no entry named '%s'", gui::kMarkerDisplayNames[i]);
        }

        if (ctx->IsError()) {
          break;
        }
      }
      IM_CHECK(ctx->ItemExists("**/Sun-side horizon"));
      ctx->SetRef("");
      // Close the menu this case opened by hand. Left up, it covers the button the next step has to
      // click, and the failure reads as "the button is not hoverable" rather than as the leak it is.
      ctx->KeyPress(ImGuiKey_Escape);
      ctx->Yield(2);

      // And picking one closes the menu — a preset is an action, not a mode, so the list must not
      // stay up waiting for a second choice.
      PickLookAt(ctx, "Sun");
      ctx->SetRef("//$FOCUSED");
      IM_CHECK(!ctx->ItemExists("**/Anthelion"));
      ctx->SetRef("");
    };
  }
}
