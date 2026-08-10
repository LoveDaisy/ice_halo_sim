// The preview panel itself: the invariants it enforces every frame, and what a drag on it does.
//
// What this suite is for. `RenderPreviewPanel` (src/gui/app_panels.cpp) is the only place the
// renderer's pose is both displayed and edited, and it re-derives that pose on every frame rather
// than only on the frames a control was touched — which is why a lens whose projection has no
// notion of "which way am I facing" can zero the angles out from under a value the user set two
// frames ago. Nothing about that is visible without a real frame, and nothing about the drag is
// visible without a real input event.
//
// The drag's gain is the sharp one. It is not a constant: the same pixel distance has to turn the
// view by more when the field of view is wide and less when it is narrow, or a 1-degree telephoto
// view becomes unusable while a 179-degree view barely moves. It used to be a constant, and the
// cases below carry the shape of that bug — they source the expected magnitude from the same
// function the handler calls, and assert the SIGN, the WRAP and the CLAMP, which are the handler's
// own and not the gain law's.
//
// Deliberately NOT here, with where each lives instead. The gain law itself is asserted against the
// projection pipeline in unit-correctness/gui/test_preview_renderer.cpp; which view controls are
// greyed per lens is functional/test_view_display_controls.cpp; what the preview's pixels look like
// is visual/test_preview_pixels.cpp and the committed lens-projection reference group.
//
// What a user sees when these break: a view that snaps back to level whenever they let go, a drag
// that spins the sky at a speed unrelated to how zoomed in they are, or an azimuth that jams at
// due south instead of carrying on round.

#include <cmath>

#include "gui/gui_state.hpp"
#include "test_gui_shared.hpp"

namespace {

const char* const kInteract = "**/##preview_interact";

// The interaction surface is only submitted when there is something to interact with — a texture or
// a background image — so every drag case has to put one there first. The upload is a GL call and
// therefore has to happen on the main thread rather than in the test coroutine, which is what this
// GuiFunc is for.
bool g_upload_done = false;

void UploadSynthTexture(ImGuiTestContext*) {
  if (!g_upload_done) {
    InitSynthTexture();
    gui::g_preview.UploadTexture(g_synth_tex.data(), kSynthTexW, kSynthTexH);
    g_upload_done = true;
  }
}

// Degrees the handler should produce for a given pixel delta, read off the live viewport.
//
// Sourced from the same function the handler calls rather than restated as a literal: the literals
// these cases used to carry (18 degrees for a 60 px drag) were the old fixed 0.3 deg/px in
// disguise, which is why making the gain field-of-view-aware turned four of them red — they were
// pinning a constant none of them was about.
float ExpectedDeg(int lens_type, float fov, float pixels) {
  return pixels * gui::ComputeDragGainDegPerPixel(lens_type, fov, gui::g_preview_vp.vp_w, gui::g_preview_vp.vp_h);
}

void SeedPose(int lens_type, float fov, float az, float el) {
  gui::g_state.renderer.lens_type = lens_type;
  gui::g_state.renderer.fov = fov;
  gui::g_state.renderer.azimuth = az;
  gui::g_state.renderer.elevation = el;
  gui::g_state.renderer.roll = 0.0f;
}

}  // namespace

void RegisterPreviewViewportTests(ImGuiTestEngine* engine) {
  // P33. With nothing to show, the panel says so and reports no viewport — the second half is what
  // stops the rest of the GUI from computing drag gains and overlay placements against a rectangle
  // that is not being drawn into.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "preview_viewport", "an_empty_preview_says_so_and_claims_no_viewport");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(3);

      IM_CHECK(!gui::g_preview.HasTexture());
      IM_CHECK(!gui::g_preview.HasBackground());
      IM_CHECK(!gui::g_preview_vp.active);
      // No interaction surface either: there is nothing to drag.
      IM_CHECK(!ctx->ItemExists(kInteract));
    };
  }

  // The other side of the same claim, so the case above cannot pass by the panel being broken:
  // once a texture arrives the viewport becomes active and the surface appears.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "preview_viewport", "a_rendered_preview_claims_a_viewport_to_drag");
    t->GuiFunc = UploadSynthTexture;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      g_upload_done = false;
      ctx->Yield(4);

      IM_CHECK(gui::g_preview.HasTexture());
      IM_CHECK(gui::g_preview_vp.active);
      IM_CHECK_GT(gui::g_preview_vp.vp_w, 0);
      IM_CHECK_GT(gui::g_preview_vp.vp_h, 0);
      IM_CHECK(ctx->ItemExists(kInteract));
    };
  }

  // P34. A full-sky projection shows the whole sky at once, so there is no direction to be facing —
  // and the panel enforces that every frame rather than only on the frame the lens changed. The
  // angles are set FIRST and the lens second, so what is asserted is the per-frame guard rather
  // than a one-off reset in the lens-change handler.
  //
  // The set is iterated from the constant the product uses, so a lens added to it is covered here
  // without anyone remembering to add a row.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "preview_viewport", "a_full_sky_lens_zeroes_the_pose_every_frame");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      for (int lens : gui::kFullSkyLensTypes) {
        gui::g_state.renderer.lens_type = lens;
        gui::g_state.renderer.elevation = 30.0f;
        gui::g_state.renderer.azimuth = 45.0f;
        gui::g_state.renderer.roll = 10.0f;
        ctx->Yield(3);
        const auto& r = gui::g_state.renderer;
        if (r.elevation != 0.0f || r.azimuth != 0.0f || r.roll != 0.0f) {
          IM_ERRORF("lens %d left the pose at el=%f az=%f roll=%f", lens, static_cast<double>(r.elevation),
                    static_cast<double>(r.azimuth), static_cast<double>(r.roll));
        }

        if (ctx->IsError()) {
          break;
        }
      }
    };
  }

  // P35 / P36. A rightward drag turns the view, and which WAY depends on the projection: under the
  // globe the user is spinning a sphere in front of them, everywhere else they are turning their
  // own head, so the two signs are opposite. One case, because the claim is the difference — a
  // handler that had lost the branch would agree with one half and not the other.
  //
  // The magnitude is bounded rather than pinned: what is under test is the wiring, and the gain
  // that produces the number has its own coverage at the unit layer.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "preview_viewport", "a_rightward_drag_turns_opposite_ways_on_globe_and_elsewhere");
    t->GuiFunc = UploadSynthTexture;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      struct Case {
        const char* name;
        int lens;
        float sign;
      };
      const Case kCases[] = {
        { "globe", gui::kLensTypeGlobe, +1.0f },
        { "fisheye", gui::kLensTypeFisheyeEquidist, -1.0f },
      };

      // One lens per call rather than one loop body: the texture precondition and the
      // non-degenerate-viewport guard are fatal (without either, everything below is vacuous), and a
      // fatal assert inside a loop would return out of the case and take the second lens with it.
      auto drag_on = [ctx](const Case& c) {
        ResetTestState();
        g_upload_done = false;
        ctx->Yield(4);
        IM_CHECK(gui::g_preview.HasTexture());
        SeedPose(c.lens, 60.0f, 0.0f, 0.0f);
        ctx->Yield(2);

        const float expected = c.sign * ExpectedDeg(c.lens, 60.0f, 60.0f);
        // A degenerate viewport would make every comparison below vacuously true.
        IM_CHECK_GT(std::fabs(expected), 1.0f);
        ctx->ItemDragWithDelta(kInteract, ImVec2(60.0f, 0.0f));
        ctx->Yield(2);

        const auto& r = gui::g_state.renderer;
        if (std::fabs(r.azimuth - expected) > 1.0f) {
          IM_ERRORF("%s: azimuth landed on %f, expected about %f", c.name, static_cast<double>(r.azimuth),
                    static_cast<double>(expected));
        }
        // A horizontal drag is horizontal: nothing else moved.
        if (std::fabs(r.elevation) > 0.5f || std::fabs(r.roll) > 0.001f) {
          IM_ERRORF("%s: a horizontal drag also moved el=%f roll=%f", c.name, static_cast<double>(r.elevation),
                    static_cast<double>(r.roll));
        }
      };

      for (const Case& c : kCases) {
        drag_on(c);

        if (ctx->IsError()) {
          break;
        }
      }
    };
  }

  // P35's sharp half, and the reason the gain is a function rather than a constant: the same drag
  // has to turn the view further at a wide field of view than at a narrow one. A constant gain
  // satisfies every other case in this file.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "preview_viewport", "the_same_drag_turns_further_at_a_wider_field_of_view");
    t->GuiFunc = UploadSynthTexture;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      g_upload_done = false;
      ctx->Yield(4);
      IM_CHECK(gui::g_preview.HasTexture());

      SeedPose(gui::kLensTypeFisheyeEquidist, 10.0f, 0.0f, 0.0f);
      ctx->Yield(2);
      ctx->ItemDragWithDelta(kInteract, ImVec2(60.0f, 0.0f));
      ctx->Yield(2);
      const float narrow = std::fabs(gui::g_state.renderer.azimuth);

      SeedPose(gui::kLensTypeFisheyeEquidist, 160.0f, 0.0f, 0.0f);
      ctx->Yield(2);
      ctx->ItemDragWithDelta(kInteract, ImVec2(60.0f, 0.0f));
      ctx->Yield(2);
      const float wide = std::fabs(gui::g_state.renderer.azimuth);

      IM_CHECK_GT(narrow, 0.0f);  // the premise: both drags did something
      IM_CHECK_GT(wide, narrow);
    };
  }

  // P37. Azimuth is a circle, so a drag past due south has to come out the other side rather than
  // stop there — a pinned value would leave the user unable to turn any further in the direction
  // they were already turning.
  //
  // The overshoot is asserted rather than assumed: if the gain ever shrinks enough that 200 px no
  // longer clears the remaining 10 degrees, this case would stop exercising the wrap and go on
  // passing quietly on an unwrapped value.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "preview_viewport", "the_azimuth_wraps_round_instead_of_pinning_at_the_seam");
    t->GuiFunc = UploadSynthTexture;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      g_upload_done = false;
      ctx->Yield(4);
      IM_CHECK(gui::g_preview.HasTexture());
      SeedPose(gui::kLensTypeGlobe, 60.0f, 170.0f, 0.0f);
      ctx->Yield(2);

      const float unwrapped = 170.0f + ExpectedDeg(gui::kLensTypeGlobe, 60.0f, 200.0f);
      IM_CHECK_GT(unwrapped, 180.0f);  // precondition: the drag really does cross the seam
      ctx->ItemDragWithDelta(kInteract, ImVec2(200.0f, 0.0f));
      ctx->Yield(2);

      IM_CHECK_LT(gui::g_state.renderer.azimuth, 0.0f);  // wrapped, not pinned at +180
      IM_CHECK_LT(std::fabs(gui::g_state.renderer.azimuth - (unwrapped - 360.0f)), 1.0f);
    };
  }

  // P36 / P38. Elevation is NOT a circle — past the pole the view would be upside down — so it
  // clamps, and the globe clamps one degree short of the other lenses because its orbit degenerates
  // exactly at the pole. Both limits in one case, since the claim is that they differ.
  //
  // Each row asserts the drag overshot the limit before asserting the limit held, so neither can
  // decay into "never reached it, therefore never violated it". The field of view differs per row
  // for that reason alone: the gain scales with it, and at 60 degrees a 1000 px drag never reaches
  // the looser limit at all.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "preview_viewport", "elevation_clamps_one_degree_short_of_the_pole_on_globe");
    t->GuiFunc = UploadSynthTexture;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      struct Limit {
        const char* name;
        int lens;
        float fov;
        float dy;  // globe drives elevation up on an upward drag, the others on a downward one
        float limit;
      };
      const Limit kLimits[] = {
        { "globe", gui::kLensTypeGlobe, 60.0f, -1000.0f, 89.0f },
        { "fisheye", gui::kLensTypeFisheyeEquidist, 360.0f, 1000.0f, 90.0f },
      };

      // A lambda per row, for the same reason as the sign case above: the texture precondition is
      // fatal and would otherwise take the second row with it.
      auto drive = [ctx](const Limit& l) {
        ResetTestState();
        g_upload_done = false;
        ctx->Yield(4);
        IM_CHECK(gui::g_preview.HasTexture());
        SeedPose(l.lens, l.fov, 0.0f, 0.0f);
        ctx->Yield(2);

        const float reach = ExpectedDeg(l.lens, l.fov, 1000.0f);
        if (reach <= l.limit) {
          IM_ERRORF("%s: a 1000 px drag only reaches %f degrees, which never touches the %f limit", l.name,
                    static_cast<double>(reach), static_cast<double>(l.limit));
          return;
        }
        ctx->ItemDragWithDelta(kInteract, ImVec2(0.0f, l.dy));
        ctx->Yield(2);

        const float el = gui::g_state.renderer.elevation;
        if (el < l.limit || el >= l.limit + 0.5f) {
          IM_ERRORF("%s: elevation settled at %f, expected the %f limit", l.name, static_cast<double>(el),
                    static_cast<double>(l.limit));
        }
      };

      for (const Limit& l : kLimits) {
        drive(l);

        if (ctx->IsError()) {
          break;
        }
      }
    };
  }

  // P38. A full-sky projection has nothing to aim, so neither the drag nor the wheel may move
  // anything — and the surface is still submitted, so "nothing happened" has to be the handler's
  // decision rather than the absence of a target to click.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "preview_viewport", "a_full_sky_lens_ignores_the_drag_it_still_accepts");
    t->GuiFunc = UploadSynthTexture;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      g_upload_done = false;
      ctx->Yield(4);
      IM_CHECK(gui::g_preview.HasTexture());

      gui::g_state.renderer.lens_type = gui::kLensTypeDualFisheyeEqualArea;
      ctx->Yield(3);
      IM_CHECK(ctx->ItemExists(kInteract));  // the premise: there IS something to drag on
      const float fov_before = gui::g_state.renderer.fov;

      ctx->ItemDragWithDelta(kInteract, ImVec2(120.0f, 80.0f));
      ctx->Yield(3);
      IM_CHECK_EQ(gui::g_state.renderer.azimuth, 0.0f);
      IM_CHECK_EQ(gui::g_state.renderer.elevation, 0.0f);
      IM_CHECK_EQ(gui::g_state.renderer.fov, fov_before);
    };
  }
}
