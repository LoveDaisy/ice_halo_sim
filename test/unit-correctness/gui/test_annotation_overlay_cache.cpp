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
