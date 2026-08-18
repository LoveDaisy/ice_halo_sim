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

// Frames to withhold the upload for, so a case can watch the empty state first and the result
// afterwards within one document. Counted down here rather than gated by a bool a case has to clear
// again: a case that fails an assert never runs its own cleanup, and a counter that drains on its
// own cannot hand a later case a preview that stays empty forever.
int g_upload_delay_frames = 0;

void UploadSynthTexture(ImGuiTestContext*) {
  if (g_upload_delay_frames > 0) {
    g_upload_delay_frames--;
    return;
  }
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
  // P33. With nothing rendered and nothing loaded, the panel is not empty — it is aimed. The
  // document already says which way the camera points, how wide the lens is and where the sun will
  // be, so the panel draws that sky, dimmed, instead of a black rectangle. What it still does NOT
  // do is offer a drag surface: aiming a view that has no content is a separate question from
  // previewing one, and the panel keeps "the surface exists only when there is something to
  // interact with".
  //
  // Which lines appear is asserted in BOTH directions in the same case on purpose. The empty
  // state's sky coordinate system is the three the blueprint enumerates — horizon, angular-distance
  // circles, sun marker (doc/gui-layout-architecture.md §4) — and those are forced on with every
  // user toggle deliberately left OFF here, which is the only setting under which "forced" is
  // distinguishable from "happened to follow a toggle that was on". The grid is NOT enumerated and
  // so still follows its toggle; asserting only the forced half would pass just as well if the
  // forcing had been applied to every line indiscriminately, and asserting only the grid half would
  // pass on the pre-456.7 behaviour where nothing was forced at all.
  //
  // The toggles are then re-read AFTER the frame: forcing is a presentation-layer override on the
  // per-frame decoration, and a version of it that reached back into GuiState would still satisfy
  // every assertion above while silently rewriting what the user sees once a result arrives.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "preview_viewport", "an_empty_preview_frames_the_sky_it_has_not_rendered");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      gui::g_state.show_horizon_line = false;
      gui::g_state.show_grid_line = false;
      gui::g_state.show_sun_circles_line = false;
      ctx->Yield(3);

      IM_CHECK(!gui::g_preview.HasTexture());
      IM_CHECK(!gui::g_preview.HasBackground());
      // A viewport IS claimed now: the empty state renders through the same shader as a result.
      IM_CHECK(gui::g_preview_vp.active);
      IM_CHECK_GT(gui::g_preview_vp.vp_w, 0);
      IM_CHECK_GT(gui::g_preview_vp.vp_h, 0);

      const auto& ov = gui::g_preview_vp.params.overlay;
      IM_CHECK(ov.show_sun_marker);
      IM_CHECK_NE(ov.sun_marker_screen_pos[1], gui::kOverlaySentinel);
      // Enumerated by the blueprint => drawn even though the user's toggle says otherwise.
      IM_CHECK(ov.show_horizon);
      IM_CHECK(ov.show_sun_circles);
      // Not enumerated => still the user's call, and the user said no.
      IM_CHECK(!ov.show_grid);
      // ...at half the intensity they were set at, which is what separates "framed and waiting"
      // from "rendered".
      IM_CHECK_LT(std::fabs(ov.horizon_alpha - gui::g_state.horizon_alpha * 0.5f), 1e-5f);
      IM_CHECK_LT(std::fabs(ov.grid_alpha - gui::g_state.grid_alpha * 0.5f), 1e-5f);
      IM_CHECK_LT(std::fabs(ov.zenith_nadir_alpha - gui::g_state.zenith_nadir_alpha * 0.5f), 1e-5f);
      // The circles and the marker are the two the SHADER no longer draws: forced on (asserted
      // above) at zero shader alpha, because the empty state draws its own dashed ring and
      // cross-hair on the CPU instead (ApplyEmptyStatePresentation's handover). Both halves are
      // asserted here rather than in two cases, since either alone reads as a different feature —
      // alpha 0 with the flag off would just be "not drawn", and the flag on at half alpha would
      // be the pre-457.6 shader-drawn form.
      IM_CHECK_LT(std::fabs(ov.sun_circles_alpha), 1e-6f);
      IM_CHECK_LT(std::fabs(ov.sun_marker_alpha), 1e-6f);
      IM_CHECK(gui::g_preview_vp.empty_state.drawn);
      // The stored toggles came through the forced frame untouched.
      IM_CHECK(!gui::g_state.show_horizon_line);
      IM_CHECK(!gui::g_state.show_sun_circles_line);
      IM_CHECK(!gui::g_state.show_grid_line);
      // No interaction surface: there is nothing to drag.
      IM_CHECK(!ctx->ItemExists(kInteract));
    };
  }

  // The four marks the empty state draws ITSELF, on the CPU, because the shader cannot: the
  // angular-distance rings are dashed rather than solid, each carries its angle as text, the sun is
  // a cross-hair rather than a filled dot, and the horizon carries a HORIZON label
  // (doc/gui-layout-architecture.md §4, and the prototype this task worked from).
  //
  // Why these counters exist to be asserted at all: an ImGui draw-list call leaves nothing behind,
  // so without them the only evidence that any of this happened is a screenshot, and the committed
  // reference image for this frame is re-shot on any layout change — it cannot also be the thing
  // that says the rings are still dashed. The counters are written as the marks are drawn.
  //
  // The pairing with the shader side is the point of asserting both here: the same frame that
  // reports four CPU-drawn marks reports sun_circles_alpha == 0 on the decoration (the case above),
  // which is the handover, not a contradiction.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "preview_viewport", "an_empty_preview_draws_its_own_instrument_marks");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      SeedPose(gui::kLensTypeLinear, 90.0f, 0.0f, 0.0f);
      gui::g_state.sun.altitude = 20.0f;
      ctx->Yield(3);

      IM_CHECK(!gui::g_preview.HasTexture());  // the premise: this is the empty state
      const auto& inst = gui::g_preview_vp.empty_state;
      IM_CHECK(inst.drawn);
      // Every ring the document asks for, not a fixed two: the angles are user-editable
      // (GuiState::sun_circle_angles), and a hardcoded 22/46 here would pass on an implementation
      // that ignored the list.
      IM_CHECK_EQ(inst.dashed_circles, static_cast<int>(gui::g_state.sun_circle_angles.size()));
      IM_CHECK_EQ(inst.degree_labels, inst.dashed_circles);
      IM_CHECK(inst.horizon_label);
      IM_CHECK(inst.sun_cross);
      IM_CHECK_NE(inst.sun_cross_pos[0], gui::kOverlaySentinel);
    };
  }

  // The ring geometry itself, pinned at one configuration rather than left to the screenshot.
  //
  // With the camera aimed straight at the sun through the linear lens, the sun lands at the centre
  // of the frame and each ring becomes a circle around it whose pixel radius is focal·tan(theta).
  // The focal length and the display scale are both unknown to this case — the viewport is in
  // framebuffer pixels, the marks are in ImGui screen pixels, and the ratio between them is
  // whatever the machine's DPI says. So the assertion is their RATIO, which both unknowns cancel
  // out of: two rings' label anchors sit above the cross in the proportion tan(46°)/tan(22°),
  // whatever the screen. A basis vector pointing the wrong way, a missing cos(theta) term, or a
  // ring built at the wrong angular distance all break this; a DPI change does not.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "preview_viewport", "an_empty_previews_rings_stand_at_their_own_angles");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      gui::g_state.sun_circle_angles = { 22.0f, 46.0f };
      gui::g_state.sun.altitude = 20.0f;
      // Elevation = the sun's altitude puts the sun on the view axis. The field of view has to
      // clear the outer ring at its TOP, where the label anchor is: the linear lens puts a ring at
      // theta at focal·tan(theta) from the centre, and focal is half the short edge over
      // tan(fov/2), so the 46° ring only fits inside a half-height above fov ≈ 92°. At 90° the
      // anchor falls off the top edge and the label walks sideways around the ring — correct
      // behaviour, and the reason this case cannot use the same pose as the one above.
      SeedPose(gui::kLensTypeLinear, 120.0f, 0.0f, 20.0f);
      ctx->Yield(3);

      const auto& inst = gui::g_preview_vp.empty_state;
      IM_CHECK(inst.drawn);
      IM_CHECK_EQ(inst.degree_labels, 2);
      IM_CHECK(inst.sun_cross);

      // Both labels sit straight above the cross: the anchor is the ring's t = 90° point, which is
      // up the meridian from the sun.
      const float dx_22 = inst.degree_label_pos[0][0] - inst.sun_cross_pos[0];
      const float dx_46 = inst.degree_label_pos[1][0] - inst.sun_cross_pos[0];
      IM_CHECK_LT(std::fabs(dx_22), 1.0f);
      IM_CHECK_LT(std::fabs(dx_46), 1.0f);

      // ...at radii in the lens's own proportion. Screen y grows downward, so both offsets are
      // negative and the ratio is taken on their magnitudes.
      const float dy_22 = inst.sun_cross_pos[1] - inst.degree_label_pos[0][1];
      const float dy_46 = inst.sun_cross_pos[1] - inst.degree_label_pos[1][1];
      IM_CHECK_GT(dy_22, 1.0f);
      IM_CHECK_GT(dy_46, dy_22);
      const float kDeg2Rad = 3.14159265358979323846f / 180.0f;
      const float expected_ratio = std::tan(46.0f * kDeg2Rad) / std::tan(22.0f * kDeg2Rad);
      const float actual_ratio = dy_46 / dy_22;
      // 2% covers the ring's 96-segment sampling (the anchor is a vertex, not a point on the true
      // circle) and single-precision projection noise; it does not cover a wrong angle, the
      // nearest of which (a ring built at 45° instead of 46°) moves this ratio by ~4%.
      IM_CHECK_LT(std::fabs(actual_ratio - expected_ratio) / expected_ratio, 0.02f);
    };
  }

  // The grid half of the same claim, read from the side the case above cannot reach. There, every
  // toggle was off, so "the grid follows its toggle" and "the grid is forced off" produce identical
  // pixels; only a grid switched ON separates them. It is switched on while the other two stay off,
  // so the frame carries the two rules disagreeing about the same document — a forcing that had
  // been widened to cover the grid, or narrowed to cover nothing, fails here rather than in review.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "preview_viewport", "an_empty_preview_still_obeys_the_grid_toggle");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      gui::g_state.show_horizon_line = false;
      gui::g_state.show_sun_circles_line = false;
      gui::g_state.show_grid_line = true;
      ctx->Yield(3);

      IM_CHECK(!gui::g_preview.HasTexture());  // the premise: this is the empty state
      const auto& ov = gui::g_preview_vp.params.overlay;
      IM_CHECK(ov.show_grid);         // the user asked for it
      IM_CHECK(ov.show_horizon);      // the blueprint asks for it
      IM_CHECK(ov.show_sun_circles);  // likewise
      IM_CHECK(!gui::g_state.show_horizon_line);
      IM_CHECK(!gui::g_state.show_sun_circles_line);
      IM_CHECK(gui::g_state.show_grid_line);
    };
  }

  // The round trip, which is where the "presentation layer only" rule is worth something to a user:
  // the empty state's forcing has to leave nothing behind once a result arrives. Asserted on the
  // DECORATION after the texture upload, not merely on GuiState — a forcing that wrote back to
  // GuiState and a forcing that leaked into the post-empty decoration are different defects, and
  // only the second is what the user would actually see.
  //
  // The three toggles are given three different values, all non-default in the sense that matters
  // (each disagrees with what the empty state would have forced), so a decoration rebuilt from a
  // single wrong constant cannot match by luck.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "preview_viewport", "leaving_the_empty_state_restores_the_users_own_toggles");
    t->GuiFunc = UploadSynthTexture;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      g_upload_done = false;
      // Hold the upload back long enough to read the empty state, then let it land. Without this
      // the GuiFunc uploads on the very next frame and the "before" half of the round trip is never
      // on screen to be left.
      g_upload_delay_frames = 4;
      gui::g_state.show_horizon_line = false;
      gui::g_state.show_grid_line = true;
      gui::g_state.show_sun_circles_line = false;
      ctx->Yield(3);

      IM_CHECK(!gui::g_preview.HasTexture());  // the premise: the empty state came first
      {
        const auto& empty_ov = gui::g_preview_vp.params.overlay;
        IM_CHECK(empty_ov.show_horizon);  // forced, against the toggle
        IM_CHECK(empty_ov.show_sun_circles);
      }
      IM_CHECK(!gui::g_state.show_horizon_line);
      IM_CHECK(gui::g_state.show_grid_line);
      IM_CHECK(!gui::g_state.show_sun_circles_line);

      ctx->Yield(6);
      IM_CHECK(gui::g_preview.HasTexture());  // the premise: this is no longer the empty state
      const auto& ov = gui::g_preview_vp.params.overlay;
      IM_CHECK(!ov.show_horizon);
      IM_CHECK(ov.show_grid);
      IM_CHECK(!ov.show_sun_circles);
      IM_CHECK(!gui::g_state.show_horizon_line);
      IM_CHECK(gui::g_state.show_grid_line);
      IM_CHECK(!gui::g_state.show_sun_circles_line);
    };
  }

  // The other side of the same claim, so the case above cannot pass by the panel being broken:
  // once a texture arrives the surface appears, the overlay goes back to the user's own intensity,
  // and the sun marker — whose whole job is to stand in for an image that is not there yet — turns
  // itself off rather than sitting on top of the sun it was predicting.
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

      const auto& ov = gui::g_preview_vp.params.overlay;
      IM_CHECK(!ov.show_sun_marker);
      IM_CHECK_LT(std::fabs(ov.horizon_alpha - gui::g_state.horizon_alpha), 1e-6f);
      IM_CHECK_LT(std::fabs(ov.grid_alpha - gui::g_state.grid_alpha), 1e-6f);
      // ...and the empty state's own marks are gone with it. Asserted on a frame that had no empty
      // state at all rather than on one that left it: the counters are rebuilt every frame, so a
      // version that only ever accumulated would read as "still drawing dashed rings over the
      // result" here — which is what the user would see.
      const auto& inst = gui::g_preview_vp.empty_state;
      IM_CHECK(!inst.drawn);
      IM_CHECK_EQ(inst.dashed_circles, 0);
      IM_CHECK(!inst.sun_cross);
    };
  }

  // The empty state is not a picture of a default sky, it is a picture of THIS document: raising
  // the sun or narrowing the lens moves the marker on the very next frame, with no run in between.
  // That is the whole "already framed, press Run" claim — a static placeholder would satisfy every
  // assertion in the case above.
  //
  // Both moves are asserted by direction rather than by magnitude: a higher sun is higher in the
  // frame, and a narrower lens pushes the same sun further from the centre. The magnitudes belong
  // to the projection laws, which have their own coverage at the unit layer.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "preview_viewport", "an_empty_previews_sky_tracks_the_document_live");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      SeedPose(gui::kLensTypeLinear, 90.0f, 0.0f, 0.0f);
      gui::g_state.sun.altitude = 5.0f;
      ctx->Yield(3);
      IM_CHECK(!gui::g_preview.HasTexture());  // the premise: this is the empty state
      const float low = gui::g_preview_vp.params.overlay.sun_marker_screen_pos[1];
      const float low_dir_z = gui::g_preview_vp.params.overlay.sun_dir[2];
      IM_CHECK_NE(low, gui::kOverlaySentinel);

      gui::g_state.sun.altitude = 20.0f;
      ctx->Yield(2);
      const float high = gui::g_preview_vp.params.overlay.sun_marker_screen_pos[1];
      IM_CHECK_NE(high, gui::kOverlaySentinel);
      IM_CHECK_GT(high, low + 1.0f);
      // The direction fed to the angular-distance circles moved with it, so the circles stay
      // centred on the marker rather than the two disagreeing about where the sun is.
      IM_CHECK_LT(gui::g_preview_vp.params.overlay.sun_dir[2], low_dir_z);

      gui::g_state.sun.altitude = 5.0f;
      gui::g_state.renderer.fov = 45.0f;
      ctx->Yield(2);
      const float zoomed = gui::g_preview_vp.params.overlay.sun_marker_screen_pos[1];
      IM_CHECK_NE(zoomed, gui::kOverlaySentinel);
      IM_CHECK_GT(zoomed, low + 1.0f);
    };
  }

  // The same liveness, read at the moment it is worth something: a result IS on screen but the
  // document has moved on since it was produced. The image stays exactly as it was rendered while
  // the coordinate system follows the edit, and the gap between them is the "this result is out of
  // date" signal — the same statement as the top bar's dirty chip, made in pixels.
  //
  // Asserted without reference to sim_state on purpose: the overlay never reads the simulation
  // lifecycle, and a test that checked kModified here would be pinning a coupling that must not
  // exist.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "preview_viewport", "a_stale_result_keeps_its_pixels_while_the_sky_moves_on");
    t->GuiFunc = UploadSynthTexture;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      g_upload_done = false;
      SeedPose(gui::kLensTypeLinear, 90.0f, 0.0f, 0.0f);
      gui::g_state.sun.altitude = 5.0f;
      ctx->Yield(4);
      IM_CHECK(gui::g_preview.HasTexture());
      const int tex_w = gui::g_preview.GetTextureWidth();
      const int tex_h = gui::g_preview.GetTextureHeight();
      const float before = gui::g_preview_vp.params.overlay.sun_marker_screen_pos[1];
      IM_CHECK_NE(before, gui::kOverlaySentinel);

      // Edit the document without re-running. Nothing re-uploads a texture, so the pixels on screen
      // are still the ones produced under the old sun.
      gui::g_state.sun.altitude = 20.0f;
      ctx->Yield(3);

      IM_CHECK_GT(gui::g_preview_vp.params.overlay.sun_marker_screen_pos[1], before + 1.0f);
      IM_CHECK(gui::g_preview.HasTexture());
      IM_CHECK_EQ(gui::g_preview.GetTextureWidth(), tex_w);
      IM_CHECK_EQ(gui::g_preview.GetTextureHeight(), tex_h);
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
