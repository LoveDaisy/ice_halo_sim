// AnnotationAnchors' contract as a per-frame, multi-family anchor query.
//
// Two propositions are worth pinning from the outside. First, HasResult() says a computation
// succeeded, NOT that every family produced anchors: a result can hold the grid's labels and not
// the circles', because one call serves every family and a family whose list is empty is simply
// absent. Second, Compute is a CALL AND NOT A CACHE: every call answers for the key it is given
// and nothing survives from the previous one — the marker that was requested last frame and not
// this one must be gone, not held. That is what lets the live preview call it on every frame of a
// drag and read that frame's anchors.

#include <gtest/gtest.h>

#include <vector>

#include "gui/annotation_anchors.hpp"
#include "gui/app.hpp"
#include "gui/gui_constants.hpp"
#include "gui/preview_renderer.hpp"  // kOverlaySentinel

namespace gui = lumice::gui;

namespace {

// A plain 120 deg fisheye at 96x96 — big enough that every requested family lands anchors, small
// enough that a case costs microseconds.
gui::AnnotationViewInput MakeView() {
  gui::AnnotationViewInput in;
  in.lens_type = gui::kLensTypeFisheyeEqualArea;
  in.fov = 120.0f;
  in.visible = gui::kVisibleFull;
  in.sun_altitude_deg = 30.0f;
  return in;
}

gui::AnnotationAnchors::ViewKey KeyFor(const gui::AnnotationViewInput& in) {
  return gui::MakeAnnotationViewKey(in, 96, 96);
}

}  // namespace

TEST(AnnotationAnchors, AResultCanHoldOneFamilyAndNotAnother) {
  // Grid only. The circles' labels must come back EMPTY, not as some default anchor.
  gui::AnnotationViewInput grid_only = MakeView();
  grid_only.elevation_deg = gui::ComputeGridElevationAngles(gui::ComputeGridStep(grid_only.fov));
  grid_only.longitude_deg = gui::ComputeGridLongitudeAngles(gui::ComputeGridStep(grid_only.fov));
  gui::AnnotationAnchors anchors;
  anchors.Compute(KeyFor(grid_only));
  ASSERT_TRUE(anchors.HasResult());
  EXPECT_EQ(anchors.Width(), 96);
  EXPECT_EQ(anchors.Height(), 96);
  EXPECT_FALSE(anchors.GridLabels().empty());
  EXPECT_TRUE(anchors.AngularDistLabels().empty()) << "a family that was not requested must not produce anchors";

  // The mirror image, on the SAME instance: switching to circles-only must clear the grid's
  // labels rather than leave the previous result's behind. This is the half a fresh instance
  // cannot see, and it is the half that bites.
  gui::AnnotationViewInput circles_only = MakeView();
  circles_only.angular_dist_deg = { 22.0f, 46.0f };
  anchors.Compute(KeyFor(circles_only));
  ASSERT_TRUE(anchors.HasResult());
  EXPECT_FALSE(anchors.AngularDistLabels().empty());
  EXPECT_TRUE(anchors.GridLabels().empty()) << "the previous key's grid labels survived into a result that did "
                                               "not ask for them";
}

TEST(AnnotationAnchors, OneCallServesEveryFamily) {
  gui::AnnotationViewInput all = MakeView();
  all.horizon = true;
  all.angular_dist_deg = { 22.0f, 46.0f };
  all.elevation_deg = gui::ComputeGridElevationAngles(gui::ComputeGridStep(all.fov));
  all.longitude_deg = gui::ComputeGridLongitudeAngles(gui::ComputeGridStep(all.fov));
  all.marker_ids = { LUMICE_ANNOTATION_MARKER_SUN };

  gui::AnnotationAnchors anchors;
  anchors.Compute(KeyFor(all));
  ASSERT_TRUE(anchors.HasResult());
  EXPECT_FALSE(anchors.HorizonLabels().empty());
  EXPECT_FALSE(anchors.AngularDistLabels().empty());
  EXPECT_FALSE(anchors.GridLabels().empty());
  EXPECT_TRUE(anchors.MarkerPoint(LUMICE_ANNOTATION_MARKER_SUN).valid)
      << "a 120 deg fisheye on the horizon images a sun at altitude 30";
}

TEST(AnnotationAnchors, EveryCallAnswersForItsOwnKeyAndNothingElse) {
  // The per-frame contract, stated as the thing a cache would get wrong: two calls with two views
  // must give two answers, and neither may lean on the other. The horizon's anchor moves with the
  // elevation, which is what a drag changes on every frame.
  gui::AnnotationViewInput level = MakeView();
  level.horizon = true;
  gui::AnnotationViewInput tilted = level;
  tilted.elevation = 20.0f;

  gui::AnnotationAnchors anchors;
  anchors.Compute(KeyFor(level));
  ASSERT_TRUE(anchors.HasResult());
  ASSERT_FALSE(anchors.HorizonLabels().empty());
  const float level_py = anchors.HorizonLabels()[0].py;

  anchors.Compute(KeyFor(tilted));
  ASSERT_TRUE(anchors.HasResult());
  ASSERT_FALSE(anchors.HorizonLabels().empty());
  const float tilted_py = anchors.HorizonLabels()[0].py;
  EXPECT_NE(level_py, tilted_py) << "the anchor did not move with the view";

  // And back: the answer for the first key again, not a residue of the second.
  anchors.Compute(KeyFor(level));
  ASSERT_TRUE(anchors.HasResult());
  ASSERT_FALSE(anchors.HorizonLabels().empty());
  EXPECT_FLOAT_EQ(anchors.HorizonLabels()[0].py, level_py);
}

TEST(AnnotationAnchors, AKeyThatAsksForNothingClearsTheResult) {
  gui::AnnotationViewInput something = MakeView();
  something.horizon = true;
  gui::AnnotationAnchors anchors;
  anchors.Compute(KeyFor(something));
  ASSERT_TRUE(anchors.HasResult());

  anchors.Compute(gui::AnnotationAnchors::ViewKey{});
  EXPECT_FALSE(anchors.HasResult());
  EXPECT_TRUE(anchors.HorizonLabels().empty());
  EXPECT_EQ(anchors.Width(), 0);
}

// --- The reference-point markers: a FOURTH thing the one call serves ---
//
// They are a list (marker_ids), like the three line families — but the early-out guard in
// Compute() still has to test it separately from the other three, and that is the place where
// forgetting is silent.

TEST(AnnotationAnchors, MarkersAloneStillReachCore) {
  // Markers on, every angle list empty — a user who turned the grid and the circles off. A guard
  // that tested only the three lists would return before calling core at all: no result, no
  // points, no warning.
  gui::AnnotationViewInput markers_only = MakeView();
  markers_only.marker_ids = { LUMICE_ANNOTATION_MARKER_ZENITH, LUMICE_ANNOTATION_MARKER_NADIR };
  gui::AnnotationAnchors anchors;
  anchors.Compute(KeyFor(markers_only));
  ASSERT_TRUE(anchors.HasResult()) << "a request carrying only the markers must still be computed";
  EXPECT_TRUE(anchors.GridLabels().empty());
  // This view is a 120 deg fisheye looking at the horizon's default (elevation 0), so it images
  // neither pole; what the case pins is that core was CALLED, which HasResult() reports.
  EXPECT_FALSE(anchors.MarkerPoint(LUMICE_ANNOTATION_MARKER_ZENITH).valid);
  EXPECT_FALSE(anchors.MarkerPoint(LUMICE_ANNOTATION_MARKER_NADIR).valid);
}

TEST(AnnotationAnchors, MarkerPointsAreReportedAndClearedWithTheKey) {
  // A view that images the zenith: straight up, over the upper hemisphere.
  gui::AnnotationViewInput up = MakeView();
  up.elevation = 90.0f;
  up.visible = gui::kVisibleUpper;
  up.marker_ids = { LUMICE_ANNOTATION_MARKER_ZENITH, LUMICE_ANNOTATION_MARKER_NADIR };

  gui::AnnotationAnchors anchors;
  anchors.Compute(KeyFor(up));
  ASSERT_TRUE(anchors.HasResult());
  ASSERT_TRUE(anchors.MarkerPoint(LUMICE_ANNOTATION_MARKER_ZENITH).valid) << "the fixture must image the zenith";
  EXPECT_NEAR(anchors.MarkerPoint(LUMICE_ANNOTATION_MARKER_ZENITH).px, 48.0f, 1.5f)
      << "looking straight up puts the zenith at the canvas centre";
  EXPECT_NEAR(anchors.MarkerPoint(LUMICE_ANNOTATION_MARKER_ZENITH).py, 48.0f, 1.5f);
  // Its opposite is behind the camera and in the excluded hemisphere, so it must be reported as a
  // miss rather than as some default coordinate.
  EXPECT_FALSE(anchors.MarkerPoint(LUMICE_ANNOTATION_MARKER_NADIR).valid);

  // Same instance, markers no longer requested: the held point must go, not linger. A consumer
  // reading a stale valid point would draw a ring for a request nobody made.
  gui::AnnotationViewInput no_markers = up;
  no_markers.marker_ids.clear();
  no_markers.elevation_deg = { 30.0f };
  anchors.Compute(KeyFor(no_markers));
  ASSERT_TRUE(anchors.HasResult());
  EXPECT_FALSE(anchors.MarkerPoint(LUMICE_ANNOTATION_MARKER_ZENITH).valid)
      << "a result that did not ask for the markers must hold no point";
}

// --- The canvas -> shader coordinate conversion ---
//
// Two independent changes at once (origin corner -> centre, y down -> y up), which is exactly the
// shape a hand-written conversion gets half right. A sign error here is invisible to every
// compile-time and structural check and shows up only as a ring in a mirrored position.
//
// And the y half is CONDITIONAL, which is the third thing to get wrong: the shader hands
// overlayAuxLines a y-DOWN position for the full-sky family (kFullSkyLensTypes — the flip that
// makes the GUI's picture match the CLI's), so those five lenses must NOT take the second flip.

TEST(AnnotationAnchors, CanvasPointToShaderScreenPosFlipsYAndRecentres) {
  float out[2] = { 0.0f, 0.0f };
  // A view-matrix lens: the overlay's space is y-UP, so the conversion flips.
  const int kUp = gui::kLensTypeFisheyeEqualArea;
  // The canvas centre is the shader's origin.
  CanvasPointToShaderScreenPos(gui::AnnotationAnchors::Point{ 50.0f, 30.0f, true }, kUp, 100, 60, out);
  EXPECT_FLOAT_EQ(out[0], 0.0f);
  EXPECT_FLOAT_EQ(out[1], 0.0f);

  // Top-left corner of the canvas: shader-left and shader-UP, i.e. negative x and POSITIVE y. A
  // conversion that forgot the flip would answer -30 here.
  CanvasPointToShaderScreenPos(gui::AnnotationAnchors::Point{ 0.0f, 0.0f, true }, kUp, 100, 60, out);
  EXPECT_FLOAT_EQ(out[0], -50.0f);
  EXPECT_FLOAT_EQ(out[1], 30.0f);

  // Bottom-right corner: the opposite sign on both axes.
  CanvasPointToShaderScreenPos(gui::AnnotationAnchors::Point{ 100.0f, 60.0f, true }, kUp, 100, 60, out);
  EXPECT_FLOAT_EQ(out[0], 50.0f);
  EXPECT_FLOAT_EQ(out[1], -30.0f);
}

TEST(AnnotationAnchors, CanvasPointToShaderScreenPosLeavesYAloneForTheFullSkyFamily) {
  // The same canvas point, on the two kinds of lens. x is unaffected — only the y convention
  // differs — so a conversion that flipped unconditionally answers the NEGATED y here, which puts
  // every off-centre marker in a mirrored position.
  //
  // It stayed invisible while the family had two members: on a full-sky lens the zenith and the
  // nadir both land on the canvas's horizontal centre line, where the flip is the identity.
  // Rectangular is the exception that always could have shown it, and the four sun-relative markers
  // show it everywhere.
  float up[2] = { 0.0f, 0.0f };
  float down[2] = { 0.0f, 0.0f };
  const gui::AnnotationAnchors::Point p{ 10.0f, 10.0f, true };
  CanvasPointToShaderScreenPos(p, gui::kLensTypeFisheyeEqualArea, 100, 60, up);
  CanvasPointToShaderScreenPos(p, gui::kLensTypeDualFisheyeEqualArea, 100, 60, down);
  EXPECT_FLOAT_EQ(up[0], down[0]);
  EXPECT_FLOAT_EQ(up[1], 20.0f);
  EXPECT_FLOAT_EQ(down[1], -20.0f);

  // Every member of the family, so the branch is the classifier and not a hard-coded id or two.
  for (int lens : gui::kFullSkyLensTypes) {
    float v[2] = { 0.0f, 0.0f };
    CanvasPointToShaderScreenPos(p, lens, 100, 60, v);
    EXPECT_FLOAT_EQ(v[1], -20.0f) << "lens type " << lens;
  }
  // ...and the globe, which is full-sky in the everyday sense but is NOT in that list: its shader
  // branch applies no y flip, so it converts like the view-matrix lenses.
  float globe[2] = { 0.0f, 0.0f };
  CanvasPointToShaderScreenPos(p, gui::kLensTypeGlobe, 100, 60, globe);
  EXPECT_FLOAT_EQ(globe[1], 20.0f);
}

TEST(AnnotationAnchors, CanvasPointToShaderScreenPosUsesTheCanvasItIsGiven) {
  // The same canvas point is a DIFFERENT shader position on a different canvas, which is why the
  // off-screen export must convert at its own size rather than inherit the preview's answer.
  float preview[2] = { 0.0f, 0.0f };
  float export_canvas[2] = { 0.0f, 0.0f };
  const gui::AnnotationAnchors::Point p{ 10.0f, 10.0f, true };
  CanvasPointToShaderScreenPos(p, gui::kLensTypeFisheyeEqualArea, 100, 100, preview);
  CanvasPointToShaderScreenPos(p, gui::kLensTypeFisheyeEqualArea, 400, 200, export_canvas);
  EXPECT_FLOAT_EQ(preview[0], -40.0f);
  EXPECT_FLOAT_EQ(export_canvas[0], -190.0f);
  EXPECT_NE(preview[1], export_canvas[1]);
}

TEST(AnnotationAnchors, CanvasPointToShaderScreenPosSendsAMissToTheSentinel) {
  // An unimaged direction must land where the shader's distance test rejects it. Writing its
  // (0, 0) default through the conversion instead would draw a ring at the canvas corner — the
  // ordinary single-lens case, since one of the two poles is almost always off screen.
  float out[2] = { 0.0f, 0.0f };
  CanvasPointToShaderScreenPos(gui::AnnotationAnchors::Point{ 0.0f, 0.0f, false }, gui::kLensTypeFisheyeEqualArea, 100,
                               60, out);
  EXPECT_FLOAT_EQ(out[0], gui::kOverlaySentinel);
  EXPECT_FLOAT_EQ(out[1], gui::kOverlaySentinel);

  // A degenerate canvas is a miss too: half of zero is zero, so the arithmetic would silently
  // answer the point's own coordinates.
  CanvasPointToShaderScreenPos(gui::AnnotationAnchors::Point{ 5.0f, 5.0f, true }, gui::kLensTypeFisheyeEqualArea, 0, 0,
                               out);
  EXPECT_FLOAT_EQ(out[0], gui::kOverlaySentinel);
}
