// Under the print tone the overlay's text labels are ink, like the lines they name.
//
// The lines already are: the preview shader's blendAnnotationColor() and the CLI's
// BlendAnnotation() / PaintLabels() ignore the family colour under print and only darken the paper
// by coverage (doc/print-mode-subtractive-ink.md §7, the annotation instance). The GUI's labels
// take neither path — they are built by the four Build*LabelSet functions (app_panels.cpp) and
// rasterized through an ImGui draw list — which is exactly how they were missed when that rule's
// call sites were counted, and why the proposition needs a gate of its own: export_parity excludes
// the labels from its comparison, and the print-mode functional case probes shader pixels only.
//
// What this file pins is the appearance each builder hands the drawer, per family and per tone:
//   print  -> `color` is pure black (ImGui's dst*(1-a)+src*a with a black src IS the ink formula,
//             so no second constant exists to compare against) and `has_bg` is off (on paper the
//             text is the darkest ink there is; a plate would only take the page around it down).
//   screen -> `color` is the family's own colour, `has_bg` is what the family always had. This half
//             is the byte-for-byte-unchanged claim every committed reference image rests on.
// Both colour and plate are asserted for EVERY family, because two of the four (the circles and the
// markers) never set has_bg and inherit the struct's default of true — the two where the plate rule
// actually changes something, and the two a grid/horizon-only check would leave uncovered.
//
// One computed view for all four: dual fisheye equal-area at full sky images all six marker
// directions, so BuildMarkerLabelSets emits a set for every enabled marker rather than skipping the
// unimaged ones — a fresh, never-computed cache would leave that family with nothing to assert on.
// The anchors are asserted non-empty for the same reason: an empty set is a builder that returned
// early, not one that was exercised.

#include <gtest/gtest.h>

#include <vector>

#include "gui/annotation_anchors.hpp"
#include "gui/app.hpp"
#include "gui/gui_constants.hpp"
#include "gui/gui_state.hpp"
#include "gui/overlay_labels.hpp"
#include "lumice.h"

namespace gui = lumice::gui;

namespace {

constexpr int kCanvasW = 512;
constexpr int kCanvasH = 512;

gui::AnnotationAnchors ComputeFullSkyView() {
  gui::AnnotationViewInput vin;
  vin.lens_type = gui::kLensTypeDualFisheyeEqualArea;
  vin.fov = 180.0f;
  vin.visible = gui::kVisibleFull;
  vin.overlap = gui::kDualFisheyeOverlap;
  vin.sun_altitude_deg = 20.0f;
  vin.angular_dist_deg = { 22.0f, 46.0f };
  vin.elevation_deg = gui::ComputeGridElevationAngles(30.0f);
  vin.longitude_deg = gui::ComputeGridLongitudeAngles(30.0f);
  vin.horizon = true;
  for (int i = 0; i < LUMICE_ANNOTATION_MARKER_COUNT; ++i) {
    vin.marker_ids.push_back(i);
  }
  gui::AnnotationAnchors cache;
  cache.Compute(gui::MakeAnnotationViewKey(vin, kCanvasW, kCanvasH));
  return cache;
}

gui::GuiState MakeState(int tone) {
  gui::GuiState state;
  state.renderer.tone = tone;
  for (gui::MarkerAppearance& m : state.markers) {
    m.label = true;
  }
  return state;
}

// Three separate comparisons rather than one on the array: a channel swap or a single channel left
// behind must name the channel it failed on.
void ExpectColour(const gui::CurveLabelSet& set, const float (&rgb)[3], const char* family) {
  EXPECT_FLOAT_EQ(set.color[0], rgb[0]) << family << " red";
  EXPECT_FLOAT_EQ(set.color[1], rgb[1]) << family << " green";
  EXPECT_FLOAT_EQ(set.color[2], rgb[2]) << family << " blue";
}

constexpr float kBlack[3] = { 0.0f, 0.0f, 0.0f };

}  // namespace

TEST(OverlayLabelPrintInk, ScreenKeepsEachFamilysColourAndPlate) {
  const gui::AnnotationAnchors cache = ComputeFullSkyView();
  ASSERT_TRUE(cache.HasResult());
  const gui::GuiState state = MakeState(LUMICE_TONE_SCREEN);
  const float w = static_cast<float>(kCanvasW);
  const float h = static_cast<float>(kCanvasH);

  const gui::CurveLabelSet circles = gui::BuildSunCirclesLabelSet(cache, state, w, h);
  ASSERT_FALSE(circles.anchors.empty());
  ExpectColour(circles, state.sun_circles_color, "sun circles");
  EXPECT_TRUE(circles.has_bg);

  const gui::CurveLabelSet horizon = gui::BuildHorizonLabelSet(cache, state, w, h);
  ASSERT_FALSE(horizon.anchors.empty());
  ExpectColour(horizon, state.horizon_color, "horizon");
  EXPECT_FALSE(horizon.has_bg);

  const gui::CurveLabelSet grid = gui::BuildGridLabelSet(cache, state, w, h);
  ASSERT_FALSE(grid.anchors.empty());
  ExpectColour(grid, state.grid_color, "grid");
  EXPECT_FALSE(grid.has_bg);

  const std::vector<gui::CurveLabelSet> markers = gui::BuildMarkerLabelSets(cache, state, w, h);
  ASSERT_EQ(markers.size(), static_cast<std::size_t>(LUMICE_ANNOTATION_MARKER_COUNT));
  for (int i = 0; i < LUMICE_ANNOTATION_MARKER_COUNT; ++i) {
    if (markers[i].anchors.empty()) {
      ADD_FAILURE() << "marker " << i << ": no anchor, builder returned early";
      continue;
    }
    ExpectColour(markers[i], state.markers[i].color, "marker");
    EXPECT_TRUE(markers[i].has_bg) << "marker " << i;
  }
}

TEST(OverlayLabelPrintInk, PrintMakesEveryFamilyBlackWithNoPlate) {
  const gui::AnnotationAnchors cache = ComputeFullSkyView();
  ASSERT_TRUE(cache.HasResult());
  const gui::GuiState state = MakeState(LUMICE_TONE_PRINT);
  const float w = static_cast<float>(kCanvasW);
  const float h = static_cast<float>(kCanvasH);

  const gui::CurveLabelSet circles = gui::BuildSunCirclesLabelSet(cache, state, w, h);
  ASSERT_FALSE(circles.anchors.empty());
  ExpectColour(circles, kBlack, "sun circles");
  EXPECT_FALSE(circles.has_bg);

  const gui::CurveLabelSet horizon = gui::BuildHorizonLabelSet(cache, state, w, h);
  ASSERT_FALSE(horizon.anchors.empty());
  ExpectColour(horizon, kBlack, "horizon");
  EXPECT_FALSE(horizon.has_bg);

  const gui::CurveLabelSet grid = gui::BuildGridLabelSet(cache, state, w, h);
  ASSERT_FALSE(grid.anchors.empty());
  ExpectColour(grid, kBlack, "grid");
  EXPECT_FALSE(grid.has_bg);

  const std::vector<gui::CurveLabelSet> markers = gui::BuildMarkerLabelSets(cache, state, w, h);
  ASSERT_EQ(markers.size(), static_cast<std::size_t>(LUMICE_ANNOTATION_MARKER_COUNT));
  for (int i = 0; i < LUMICE_ANNOTATION_MARKER_COUNT; ++i) {
    if (markers[i].anchors.empty()) {
      ADD_FAILURE() << "marker " << i << ": no anchor, builder returned early";
      continue;
    }
    ExpectColour(markers[i], kBlack, "marker");
    EXPECT_FALSE(markers[i].has_bg) << "marker " << i;
  }
}
