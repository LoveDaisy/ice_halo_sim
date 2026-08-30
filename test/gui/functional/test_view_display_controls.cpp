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
#include <string>

#include "gui/gui_state.hpp"
#include "test_gui_shared.hpp"

namespace {

// The warning's addressable half. app_panels.cpp draws it as a DISABLED Selectable rather than a
// TextColored precisely so it has a real ImGui id: a Text* widget is emitted with id==0 and the
// test engine's item registry never sees it, so this string is the only handle a test has on
// "the user was told". The ratio detail printed on the following line is a plain Text and is
// therefore NOT assertable here — that is a property of ImGui's item registry, not an omission.
constexpr const char* kClampWarning = "**/Screen too small for this aspect";

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

      ctx->ItemInputValue("**/##EV##display_input", 20.0f);
      ctx->Yield();
      IM_CHECK_EQ(gui::g_state.renderer.exposure_offset, 6.0f);
      ctx->ItemInputValue("**/##EV##display_input", -20.0f);
      ctx->Yield();
      IM_CHECK_EQ(gui::g_state.renderer.exposure_offset, -6.0f);
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
}
