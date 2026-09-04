// AnnotationOverlayCache's contract as a MULTI-FAMILY result holder, which is what it became when
// the coordinate grid joined the angular-distance circles in its one request.
//
// The proposition worth pinning is the one that is easy to get wrong from the outside:
// HasResult() says a computation succeeded, NOT that every family produced a mask. A caller that
// reads HasResult() and then hands AngularDistMask().data() to an uploader alongside
// Width()/Height() uploads W*H bytes from a pointer to nothing whenever the circles were the
// family that was off — and it is a plain vector, so the read is silent rather than a crash. Both
// production consumers (app_panels.cpp's per-frame fill and file_io.cpp's off-screen export) check
// the individual mask instead, and these cases are what say the situation they check for is real.

#include <gtest/gtest.h>

#include <vector>

#include "gui/annotation_overlay_cache.hpp"
#include "gui/app.hpp"
#include "gui/gui_constants.hpp"
#include "gui/preview_renderer.hpp"  // kOverlaySentinel

namespace gui = lumice::gui;

namespace {

// A plain 120 deg fisheye at 96x96 — big enough that every requested family lands pixels, small
// enough that a case costs milliseconds.
gui::AnnotationViewInput MakeView() {
  gui::AnnotationViewInput in;
  in.lens_type = gui::kLensTypeFisheyeEqualArea;
  in.fov = 120.0f;
  in.visible = gui::kVisibleFull;
  in.sun_altitude_deg = 30.0f;
  return in;
}

gui::AnnotationOverlayCache::ViewKey KeyFor(const gui::AnnotationViewInput& in) {
  return gui::MakeAnnotationViewKey(in, 96, 96);
}

}  // namespace

TEST(AnnotationOverlayCache, AResultCanHoldOneFamilyAndNotAnother) {
  // Grid only. The circles' mask must come back EMPTY rather than as a buffer of zeros, because
  // empty is the only state a caller can distinguish from "a mask that happens to be all zero".
  gui::AnnotationViewInput grid_only = MakeView();
  grid_only.elevation_deg = gui::ComputeGridElevationAngles(gui::ComputeGridStep(grid_only.fov));
  grid_only.longitude_deg = gui::ComputeGridLongitudeAngles(gui::ComputeGridStep(grid_only.fov));
  gui::AnnotationOverlayCache cache;
  cache.Refresh(KeyFor(grid_only));
  ASSERT_TRUE(cache.HasResult());
  EXPECT_FALSE(cache.GridMask().empty());
  EXPECT_TRUE(cache.AngularDistMask().empty()) << "a family that was not requested must not produce a mask";
  EXPECT_FALSE(cache.GridLabels().empty());
  EXPECT_TRUE(cache.AngularDistLabels().empty());

  // The mirror image, on the SAME cache instance: switching to circles-only must clear the grid's
  // mask rather than leave the previous result's behind. This is the half a fresh instance cannot
  // see, and it is the half that bites — Recompute() clears the vectors, and a cleared vector's
  // data() still points at its old buffer.
  gui::AnnotationViewInput circles_only = MakeView();
  circles_only.angular_dist_deg = { 22.0f, 46.0f };
  cache.Refresh(KeyFor(circles_only));
  ASSERT_TRUE(cache.HasResult());
  EXPECT_FALSE(cache.AngularDistMask().empty());
  EXPECT_TRUE(cache.GridMask().empty()) << "the previous key's grid mask survived into a result that did not ask "
                                           "for it";
  EXPECT_FALSE(cache.AngularDistLabels().empty());
  EXPECT_TRUE(cache.GridLabels().empty());
}

TEST(AnnotationOverlayCache, OneCallServesAllThreeFamilies) {
  // The class's stated design (ONE CALL SERVES BOTH CONSUMERS, now three families): asking for
  // everything at once must return everything, and the generation must advance exactly once — a
  // per-family call would show up here as three.
  gui::AnnotationViewInput all = MakeView();
  all.angular_dist_deg = { 22.0f, 46.0f };
  all.elevation_deg = gui::ComputeGridElevationAngles(gui::ComputeGridStep(all.fov));
  all.longitude_deg = gui::ComputeGridLongitudeAngles(gui::ComputeGridStep(all.fov));

  gui::AnnotationOverlayCache cache;
  const uint64_t before = cache.Generation();
  cache.Refresh(KeyFor(all));
  ASSERT_TRUE(cache.HasResult());
  EXPECT_FALSE(cache.AngularDistMask().empty());
  EXPECT_FALSE(cache.GridMask().empty());
  EXPECT_EQ(cache.Width(), 96);
  EXPECT_EQ(cache.Height(), 96);
  EXPECT_GT(cache.Generation(), before);

  // Refreshing the same key must NOT recompute: the generation is what PreviewRenderer compares to
  // decide whether to re-upload a texture, so an advancing one on an unchanged view would put a
  // W*H upload back into every frame.
  const uint64_t settled = cache.Generation();
  cache.Refresh(KeyFor(all));
  EXPECT_EQ(cache.Generation(), settled);
}

TEST(AnnotationOverlayCache, TheGridMaskIsTheUnionOfTheTwoFamilies) {
  // The GUI has one colour and one alpha for the whole grid, so the cache merges the parallels and
  // the meridians into one mask. Merged, not replaced: a bug that kept only the last family
  // assigned would still return a plausible non-empty mask.
  gui::AnnotationViewInput parallels = MakeView();
  parallels.elevation_deg = { 30.0f, 60.0f };
  gui::AnnotationOverlayCache a;
  a.Refresh(KeyFor(parallels));
  ASSERT_TRUE(a.HasResult());
  const std::vector<unsigned char> only_parallels = a.GridMask();

  gui::AnnotationViewInput meridians = MakeView();
  meridians.longitude_deg = { 0.0f, 90.0f };
  gui::AnnotationOverlayCache b;
  b.Refresh(KeyFor(meridians));
  ASSERT_TRUE(b.HasResult());
  const std::vector<unsigned char> only_meridians = b.GridMask();

  gui::AnnotationViewInput both = MakeView();
  both.elevation_deg = parallels.elevation_deg;
  both.longitude_deg = meridians.longitude_deg;
  gui::AnnotationOverlayCache c;
  c.Refresh(KeyFor(both));
  ASSERT_TRUE(c.HasResult());
  const std::vector<unsigned char>& merged = c.GridMask();

  ASSERT_EQ(only_parallels.size(), merged.size());
  ASSERT_EQ(only_meridians.size(), merged.size());
  size_t parallel_pixels = 0;
  size_t meridian_pixels = 0;
  size_t missing = 0;
  size_t stray = 0;
  for (size_t i = 0; i < merged.size(); ++i) {
    const bool p = only_parallels[i] != 0;
    const bool m = only_meridians[i] != 0;
    parallel_pixels += p ? 1 : 0;
    meridian_pixels += m ? 1 : 0;
    if ((p || m) && merged[i] == 0) {
      ++missing;
    }
    if (!p && !m && merged[i] != 0) {
      ++stray;
    }
  }
  ASSERT_GT(parallel_pixels, 0u) << "the fixture must actually draw parallels";
  ASSERT_GT(meridian_pixels, 0u) << "the fixture must actually draw meridians";
  EXPECT_EQ(missing, 0u) << "the merged mask dropped pixels one of the two families lit";
  EXPECT_EQ(stray, 0u) << "the merged mask lit pixels neither family did";
}

// --- The zenith / nadir markers: a FOURTH thing the one call serves ---
//
// They ARE a list now (marker_ids), like the three line families — but the early-out guard in
// Recompute() still has to test it separately from the other three, and that is the place where
// forgetting is silent.

TEST(AnnotationOverlayCache, MarkersAloneStillReachCore) {
  // Markers on, every angle list empty — a user who turned the grid and the circles off. The
  // early-out guard in Recompute() used to test only the three lists, and with that shape this
  // case returns before calling core at all: no result, no points, no warning.
  gui::AnnotationViewInput markers_only = MakeView();
  markers_only.marker_ids = { LUMICE_ANNOTATION_MARKER_ZENITH, LUMICE_ANNOTATION_MARKER_NADIR };
  gui::AnnotationOverlayCache cache;
  cache.Refresh(KeyFor(markers_only));
  ASSERT_TRUE(cache.HasResult()) << "a request carrying only the markers must still be computed";
  EXPECT_TRUE(cache.AngularDistMask().empty()) << "no family was asked for a mask";
  EXPECT_TRUE(cache.GridMask().empty());
  // This view is a 120 deg fisheye looking at the horizon's default (elevation 0), so it images
  // neither pole; what the case pins is that core was CALLED, which HasResult() reports.
  EXPECT_FALSE(cache.MarkerPoint(LUMICE_ANNOTATION_MARKER_ZENITH).valid);
  EXPECT_FALSE(cache.MarkerPoint(LUMICE_ANNOTATION_MARKER_NADIR).valid);
}

TEST(AnnotationOverlayCache, MarkerPointsAreReportedAndClearedWithTheKey) {
  // A view that images the zenith: straight up, over the upper hemisphere.
  gui::AnnotationViewInput up = MakeView();
  up.elevation = 90.0f;
  up.visible = gui::kVisibleUpper;
  up.marker_ids = { LUMICE_ANNOTATION_MARKER_ZENITH, LUMICE_ANNOTATION_MARKER_NADIR };

  gui::AnnotationOverlayCache cache;
  cache.Refresh(KeyFor(up));
  ASSERT_TRUE(cache.HasResult());
  ASSERT_TRUE(cache.MarkerPoint(LUMICE_ANNOTATION_MARKER_ZENITH).valid) << "the fixture must image the zenith";
  EXPECT_NEAR(cache.MarkerPoint(LUMICE_ANNOTATION_MARKER_ZENITH).px, 48.0f, 1.5f)
      << "looking straight up puts the zenith at the canvas centre";
  EXPECT_NEAR(cache.MarkerPoint(LUMICE_ANNOTATION_MARKER_ZENITH).py, 48.0f, 1.5f);
  // Its opposite is behind the camera and in the excluded hemisphere, so it must be reported as a
  // miss rather than as some default coordinate.
  EXPECT_FALSE(cache.MarkerPoint(LUMICE_ANNOTATION_MARKER_NADIR).valid);

  // Same instance, markers no longer requested: the held point must go, not linger. A consumer
  // reading a stale valid point would draw a ring for a request nobody made.
  gui::AnnotationViewInput no_markers = up;
  no_markers.marker_ids.clear();
  no_markers.elevation_deg = { 30.0f };
  cache.Refresh(KeyFor(no_markers));
  ASSERT_TRUE(cache.HasResult());
  EXPECT_FALSE(cache.MarkerPoint(LUMICE_ANNOTATION_MARKER_ZENITH).valid)
      << "a result that did not ask for the markers must hold no point";
}

TEST(AnnotationOverlayCache, TheViewKeySeesTheMarkerSwitch) {
  // The switch is part of the answer, so it has to be part of the key: two keys differing only in
  // it must compare unequal, or the debounce would hold a result computed without the markers and
  // never recompute when they are turned on.
  gui::AnnotationViewInput off = MakeView();
  off.elevation_deg = { 30.0f };
  gui::AnnotationViewInput on = off;
  on.marker_ids = { LUMICE_ANNOTATION_MARKER_ZENITH };
  EXPECT_FALSE(KeyFor(off) == KeyFor(on));
  EXPECT_TRUE(KeyFor(on) == KeyFor(on));
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

TEST(AnnotationOverlayCache, CanvasPointToShaderScreenPosFlipsYAndRecentres) {
  float out[2] = { 0.0f, 0.0f };
  // A view-matrix lens: the overlay's space is y-UP, so the conversion flips.
  const int kUp = gui::kLensTypeFisheyeEqualArea;
  // The canvas centre is the shader's origin.
  CanvasPointToShaderScreenPos(gui::AnnotationOverlayCache::Point{ 50.0f, 30.0f, true }, kUp, 100, 60, out);
  EXPECT_FLOAT_EQ(out[0], 0.0f);
  EXPECT_FLOAT_EQ(out[1], 0.0f);

  // Top-left corner of the canvas: shader-left and shader-UP, i.e. negative x and POSITIVE y. A
  // conversion that forgot the flip would answer -30 here.
  CanvasPointToShaderScreenPos(gui::AnnotationOverlayCache::Point{ 0.0f, 0.0f, true }, kUp, 100, 60, out);
  EXPECT_FLOAT_EQ(out[0], -50.0f);
  EXPECT_FLOAT_EQ(out[1], 30.0f);

  // Bottom-right corner: the opposite sign on both axes.
  CanvasPointToShaderScreenPos(gui::AnnotationOverlayCache::Point{ 100.0f, 60.0f, true }, kUp, 100, 60, out);
  EXPECT_FLOAT_EQ(out[0], 50.0f);
  EXPECT_FLOAT_EQ(out[1], -30.0f);
}

TEST(AnnotationOverlayCache, CanvasPointToShaderScreenPosLeavesYAloneForTheFullSkyFamily) {
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
  const gui::AnnotationOverlayCache::Point p{ 10.0f, 10.0f, true };
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

TEST(AnnotationOverlayCache, CanvasPointToShaderScreenPosUsesTheCanvasItIsGiven) {
  // The same canvas point is a DIFFERENT shader position on a different canvas, which is why the
  // off-screen export must convert at its own size rather than inherit the preview's answer.
  float preview[2] = { 0.0f, 0.0f };
  float export_canvas[2] = { 0.0f, 0.0f };
  const gui::AnnotationOverlayCache::Point p{ 10.0f, 10.0f, true };
  CanvasPointToShaderScreenPos(p, gui::kLensTypeFisheyeEqualArea, 100, 100, preview);
  CanvasPointToShaderScreenPos(p, gui::kLensTypeFisheyeEqualArea, 400, 200, export_canvas);
  EXPECT_FLOAT_EQ(preview[0], -40.0f);
  EXPECT_FLOAT_EQ(export_canvas[0], -190.0f);
  EXPECT_NE(preview[1], export_canvas[1]);
}

TEST(AnnotationOverlayCache, CanvasPointToShaderScreenPosSendsAMissToTheSentinel) {
  // An unimaged direction must land where the shader's distance test rejects it. Writing its
  // (0, 0) default through the conversion instead would draw a ring at the canvas corner — the
  // ordinary single-lens case, since one of the two poles is almost always off screen.
  float out[2] = { 0.0f, 0.0f };
  CanvasPointToShaderScreenPos(gui::AnnotationOverlayCache::Point{ 0.0f, 0.0f, false }, gui::kLensTypeFisheyeEqualArea,
                               100, 60, out);
  EXPECT_FLOAT_EQ(out[0], gui::kOverlaySentinel);
  EXPECT_FLOAT_EQ(out[1], gui::kOverlaySentinel);

  // A degenerate canvas is a miss too: half of zero is zero, so the arithmetic would silently
  // answer the point's own coordinates.
  CanvasPointToShaderScreenPos(gui::AnnotationOverlayCache::Point{ 5.0f, 5.0f, true }, gui::kLensTypeFisheyeEqualArea,
                               0, 0, out);
  EXPECT_FLOAT_EQ(out[0], gui::kOverlaySentinel);
}
