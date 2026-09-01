// The numbers written along the preview's coordinate grid: which ones appear, and where.
//
// Every case here describes a view, runs the production label pipeline and reads the labels back —
// no ImGui context, no draw list. The one sibling that stays in
// test/gui/functional/test_gui_overlay_labels.cpp is modal_does_not_leak_to_foreground, because what
// it asserts is which DRAW LIST the labels land in.
//
// WHERE THESE LABELS COME FROM NOW. They used to be walked by ComputeOverlayLabels in this
// process; they are walked by core's annotation layer instead, reach the GUI as anchors through
// AnnotationOverlayCache, and become OverlayLabels via BuildGridLabelSet / BuildHorizonLabelSet +
// AppendCurveLabels. The grid moved first; the HORIZON has now followed it, which is what let the
// GUI-side walk be deleted outright — so Compute() below drives that chain rather than one
// function, for both families. The propositions are unchanged: every one of them is about which
// numbers a user sees and where, which is a property of the whole chain and was never a property of
// the walk alone. Pointing them at the new producer is what keeps the move from silently dropping
// the coverage it was made under.
//
// The last three cases are the regression anchors for the four placement gaps audited in
// doc/overlay-label-placement.md; their assertion semantics must not be weakened.

#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <vector>

#include "gui/annotation_overlay_cache.hpp"
#include "gui/app.hpp"
#include "gui/gui_constants.hpp"
#include "gui/gui_state.hpp"
#include "gui/overlay_labels.hpp"
#include "gui/preview_renderer.hpp"

namespace gui = lumice::gui;

namespace {

// Group ids, imported now that overlay_labels.hpp publishes them (they used to live in an anonymous
// namespace in the .cpp and had to be mirrored here).
constexpr int kGridGroup = gui::kGroupGrid;
constexpr int kSunGroup = gui::kGroupSunCircles;
// UTF-8 for the degree sign, spelled as bytes so the assertions match AddLabel's "%.0f\xC2\xB0"
// output without depending on how the compiler handles the source charset.
constexpr const char* kDeg = "\xC2\xB0";

// The view a case describes, plus which families it wants. A struct of this file's own rather than
// a production type: the production input the cases used to fill (OverlayLabelInput) went away with
// the walk it fed, and what these cases were ever really parameterised over is a VIEW — every field
// below is either a projection parameter or a family switch.
struct ViewDesc {
  int lens_type = 0;
  float fov = 120.0f;
  float elevation = 0.0f;
  float azimuth = 0.0f;
  float roll = 0.0f;
  int visible = 0;
  bool front = false;
  bool show_horizon = false;
  // The grid's density, expanded into explicit angle lists exactly as the preview does. Default
  // 10 deg is what the old production default was, so no case's expected counts move.
  float grid_step = 10.0f;
};

// An input with the coordinate grid on and everything else off — the baseline every grid case
// starts from, since the three overlay families are independently gated and a case about one of
// them must not be reading another's labels.
ViewDesc MakeGridOnly(int visible, int lens_type, float elevation, float azimuth, bool front = false) {
  ViewDesc in;
  in.lens_type = lens_type;
  in.fov = 120.0f;
  in.elevation = elevation;
  in.azimuth = azimuth;
  in.roll = 0.0f;
  in.visible = visible;
  in.front = front;
  in.show_horizon = false;
  return in;
}

// Ask core for this view's anchors and turn them into OverlayLabels exactly as RenderPreviewPanel
// does — same expansion functions, same cache, same set builders — so what these cases read is what
// the preview draws.
std::vector<gui::OverlayLabel> Compute(const ViewDesc& in, float vp_w, float vp_h, float vp_x = 0.0f, float vp_y = 0.0f,
                                       bool with_grid = true) {
  gui::AnnotationViewInput vin;
  vin.lens_type = in.lens_type;
  vin.fov = in.fov;
  vin.azimuth = in.azimuth;
  vin.elevation = in.elevation;
  vin.roll = in.roll;
  vin.visible = in.visible;
  vin.front = in.front;
  vin.horizon = in.show_horizon;
  if (with_grid) {
    vin.elevation_deg = gui::ComputeGridElevationAngles(in.grid_step);
    vin.longitude_deg = gui::ComputeGridLongitudeAngles(in.grid_step);
  }
  // Refresh, not Update: one shot, so there is no run of frames to debounce over. A fresh cache
  // per call so no case can inherit another's settled result.
  gui::AnnotationOverlayCache cache;
  cache.Refresh(gui::MakeAnnotationViewKey(vin, static_cast<int>(vp_w), static_cast<int>(vp_h)));
  // Each family's appearance rides on its set, so a default-constructed GuiState is the whole of it
  // here — no case in this file asserts a colour.
  gui::GuiState state;
  std::vector<gui::OverlayLabel> labels;
  if (in.show_horizon) {
    gui::AppendCurveLabels(gui::BuildHorizonLabelSet(cache, state, vp_w, vp_h), vp_x, vp_y, labels);
  }
  if (with_grid) {
    gui::AppendCurveLabels(gui::BuildGridLabelSet(cache, state, vp_w, vp_h), vp_x, vp_y, labels);
  }
  return labels;
}

int CountInGroup(const std::vector<gui::OverlayLabel>& labels, int group) {
  int n = 0;
  for (const auto& l : labels) {
    n += (l.group == group) ? 1 : 0;
  }
  return n;
}

int CountGrid(const ViewDesc& in, float vp_w, float vp_h) {
  return CountInGroup(Compute(in, vp_w, vp_h), kGridGroup);
}

// Distinct grid label TEXTS, i.e. how many different latitude/longitude values are shown. A count of
// labels cannot tell "five latitudes each labelled once" from "one latitude labelled five times".
int CountDistinctGridTexts(const std::vector<gui::OverlayLabel>& labels) {
  std::vector<std::string> seen;
  for (const auto& l : labels) {
    if (l.group != kGridGroup) {
      continue;
    }
    bool dup = false;
    for (const auto& s : seen) {
      dup = dup || s == l.text;
    }
    if (!dup) {
      seen.push_back(l.text);
    }
  }
  return static_cast<int>(seen.size());
}

int CountText(const std::vector<gui::OverlayLabel>& labels, const std::string& text) {
  int n = 0;
  for (const auto& l : labels) {
    n += (l.text == text) ? 1 : 0;
  }
  return n;
}

bool HasGridText(const std::vector<gui::OverlayLabel>& labels, const std::string& text) {
  for (const auto& l : labels) {
    if (l.group == kGridGroup && l.text == text) {
      return true;
    }
  }
  return false;
}

}  // namespace

// ---- Every projection must reach the same label-generating machinery ----
//
// The dispatch in PixelToWorldDir is a switch over lens type, and a lens missing from it falls
// through to the rectangular inverse — which does not fail, it silently answers with a different
// projection's geometry. That is how single orthographic once shipped with upside-down latitude
// labels. Comparing its label count against equidistant fisheye at the same view catches it: the two
// share a projection topology (one disc, view matrix applied) and differ only in the radial law, so
// their counts stay within a factor of two of each other while a wholly different projection does
// not.
//
// Dual orthographic is deliberately not given a parallel case: those projections always size their
// twin discs to fit exactly, so a viewport edge is at most tangent to a disc and never a chord, and
// the edge-sampling loop yields no sample pairs at all for any configuration. Its fix in
// PixelToWorldDir uses the same dispatch pattern, so this case covers it; the two disc-centre probes
// further down cover its geometry directly.
//
// Both label families are read here, because the dispatch is one switch and a lens dropped from it
// is dropped for everything drawn through it. The sun-circle half had its own version of the same
// defect — an interior-label block gated to lens types 0-3 by number, so single orthographic drew no
// sun-circle labels at all — and it is the stricter of the two: EQUAL counts, since the two lenses
// share that code path outright, where the grid counts only have to stay within a factor of two.
TEST(OverlayLabels, SingleOrthographicReachesTheSameLabellingPathAsFisheye) {
  auto ortho = MakeGridOnly(gui::kVisibleFull, gui::kLensTypeFisheyeOrthographic, 0.0f, 0.0f);
  ortho.fov = 60.0f;
  auto fisheye = MakeGridOnly(gui::kVisibleFull, gui::kLensTypeFisheyeEquidist, 0.0f, 0.0f);
  fisheye.fov = 60.0f;

  const int n_ortho = CountGrid(ortho, 200.0f, 200.0f);
  const int n_fisheye = CountGrid(fisheye, 200.0f, 200.0f);
  EXPECT_GT(n_ortho, 0);
  EXPECT_GT(n_fisheye, 0);
  EXPECT_GE(n_ortho * 2, n_fisheye);
  EXPECT_LE(n_ortho, n_fisheye * 2);
}

// ---- Front mode hides what is behind the camera, and only in front mode ----
//
// The sun-circle half of this rule used to be asserted here, on a circle drawn around a sun placed
// behind the camera. It moved out of this file with the walk that placed those labels: the GUI no
// longer decides where a circle's label goes, it reads core's anchors (AnnotationOverlayCache), and
// core applies the front clip itself from ViewSnapshot::front. The proposition is core's now, and
// core's own front-clip cases carry it.
//
// The grid's half stays below, and with it the pairing that made the assertion mean something: a
// cull that ran in every mode would satisfy "front hides it" while deleting labels the user asked
// to see, and a cull that never ran would satisfy "full keeps it". Only the asymmetry between the
// two tells them apart.

// The grid's half of the same rule, plus what must survive it. Rectangular maps the whole sphere
// into the viewport rectangle, so it is the cleanest place to see the cull act. A wide fisheye is
// where the opposite risk lives: front mode must still label the hemisphere edge itself, or the
// visible half loses its coordinates entirely.
//
// MEASURED BY WHERE THE LABELS ARE, NOT HOW MANY. This case used to count them, on the reasoning
// that culling the back half removes the labels sitting there. That proxy held for the walk this
// file used to drive and does not hold for the one it drives now: the curve-centric placement puts
// each curve's number at its first VISIBLE sample, so a parallel that runs behind the camera keeps
// its label and MOVES it onto the visible arc. Counting therefore reads 52 either way while the
// anchors of 33 of them have moved — a proxy that agrees with a broken implementation and a
// correct one alike. What the user actually needs is that no number is printed over sky the front
// clip removed, which is what is asserted below, by un-projecting each anchor back to the direction
// it names. At elev=0/az=0 the camera looks along -x, so "in front" is wx < 0.
TEST(OverlayLabels, FrontModeLabelsOnlyTheHemisphereItDrawsAndStillLabelsItsEdge) {
  float view[9];
  gui::BuildViewMatrix(0.0f, 0.0f, 0.0f, view);
  // The same slack core's own front predicate carries at the boundary (kFrontEps there): a curve
  // tangent to the hemisphere edge is kept, and it is the edge that must stay labelled.
  constexpr float kFrontEps = 0.01f;

  // Un-project one anchor. The labels arrive in viewport pixels with a top-left origin; the
  // un-projection works in centre-origin, y-up coordinates.
  auto anchor_wx = [&](const gui::OverlayLabel& l, float vp_w, float vp_h, int lens, float fov) {
    float wx = 0.0f;
    float wy = 0.0f;
    float wz = 0.0f;
    bool valid = false;
    gui::detail::PixelToWorldDirForTesting(l.screen_x - vp_w * 0.5f, vp_h * 0.5f - l.screen_y, vp_w, vp_h, lens, fov,
                                           view, &wx, &wy, &wz, &valid);
    return valid ? wx : 0.0f;
  };

  auto rect_full = MakeGridOnly(gui::kVisibleFull, gui::kLensTypeRectangular, 0.0f, 0.0f);
  auto rect_front = MakeGridOnly(gui::kVisibleFull, gui::kLensTypeRectangular, 0.0f, 0.0f, /*front=*/true);
  const auto full_labels = Compute(rect_full, 800.0f, 400.0f);
  const auto front_labels = Compute(rect_front, 800.0f, 400.0f);
  ASSERT_GT(CountInGroup(full_labels, kGridGroup), 0);
  ASSERT_GT(CountInGroup(front_labels, kGridGroup), 0) << "front mode must not leave the visible half unlabelled";

  int behind_in_front_mode = 0;
  for (const auto& l : front_labels) {
    if (l.group == kGridGroup && anchor_wx(l, 800.0f, 400.0f, gui::kLensTypeRectangular, rect_front.fov) > kFrontEps) {
      ++behind_in_front_mode;
    }
  }
  EXPECT_EQ(behind_in_front_mode, 0) << "front mode printed " << behind_in_front_mode
                                     << " grid label(s) over sky it does not draw";

  // The positive control, and the reason the case is a pair: a cull that ran unconditionally would
  // satisfy the assertion above while deleting labels the user asked to see. Full mode must put
  // some of them behind the camera.
  int behind_in_full_mode = 0;
  for (const auto& l : full_labels) {
    if (l.group == kGridGroup && anchor_wx(l, 800.0f, 400.0f, gui::kLensTypeRectangular, rect_full.fov) > kFrontEps) {
      ++behind_in_full_mode;
    }
  }
  EXPECT_GT(behind_in_full_mode, 0) << "without the front clip a rectangular view labels the whole sphere, so some "
                                       "anchors must land behind the camera; none did, which means the clip is on "
                                       "in both modes";

  // A 280 deg fisheye reaches well past the front hemisphere, so front mode has something to remove
  // there too — and must still label what is left.
  auto wide_full = MakeGridOnly(gui::kVisibleFull, gui::kLensTypeFisheyeEquidist, 0.0f, 0.0f);
  auto wide_front = MakeGridOnly(gui::kVisibleFull, gui::kLensTypeFisheyeEquidist, 0.0f, 0.0f, /*front=*/true);
  wide_full.fov = 280.0f;
  wide_front.fov = 280.0f;
  EXPECT_GT(CountGrid(wide_full, 800.0f, 600.0f), 0);
  EXPECT_GT(CountGrid(wide_front, 800.0f, 600.0f), 0);
}

// ---- The globe lens hides what is around the back of the sphere ----
//
// A globe is drawn from outside, so a direction is visible only while it stays on the near side:
// dz > 1/kGlobeCameraD. With the camera distance at 4 that threshold is 0.25, and at elev=0/az=0 the
// eye-space dz reduces to the world x component, which makes the consequence exactly predictable.
// Both rows are needed: without the positive control, raising the threshold to 1.0 would cull
// everything and satisfy the negative one.
TEST(OverlayLabels, TheGlobeLensHidesLabelsOnTheFarSideOfTheSphere) {
  auto globe = MakeGridOnly(gui::kVisibleFull, gui::kLensTypeGlobe, 0.0f, 0.0f);
  globe.fov = 90.0f;  // the globe's own maximum, so the per-frame clamp does not interact
  const auto labels = Compute(globe, 200.0f, 200.0f);
  ASSERT_GT(CountInGroup(labels, kGridGroup), 0);

  // At 70 degrees the widest azimuth reaches cos(70) = 0.34 > 0.25, so some of the ring is on the
  // near side. At 80 it is cos(80) = 0.17 < 0.25 and the whole ring is behind — and without the
  // cull it would project to about a quarter of the way up a 200px viewport, comfortably visible.
  // (85 would work equally well; 80 is the outermost step the altitude set currently walks, so this
  // stays correct if the set is ever narrowed.)
  for (const char* sign : { "", "-" }) {
    EXPECT_TRUE(HasGridText(labels, std::string(sign) + "70" + kDeg)) << sign << "70";
    EXPECT_FALSE(HasGridText(labels, std::string(sign) + "80" + kDeg)) << sign << "80";
  }

  // A loose companion, kept as one line rather than as its own case because of what it does NOT
  // cover: at this configuration the globe frustum is a 45-degree cone around +Z, so every sampled
  // direction already satisfies the near-side test and the inequality is secured by the
  // field-of-view difference alone. The rows above are what pin the cull.
  auto fisheye = MakeGridOnly(gui::kVisibleFull, gui::kLensTypeFisheyeEquidist, 0.0f, 0.0f);
  fisheye.fov = 180.0f;
  EXPECT_LT(CountInGroup(labels, kGridGroup), CountGrid(fisheye, 200.0f, 200.0f));
}

// ---- Labels are positioned relative to the panel, not the desktop ----
//
// Every label must shift by exactly the viewport origin, with the set and its order unchanged. The
// regression: the caller passed panel-local coordinates where OS screen coordinates were expected,
// so the labels stayed pinned to the desktop origin — invisible until the window happened to sit at
// the top-left corner of the primary monitor.
TEST(OverlayLabels, EveryLabelMovesWithTheViewportOriginAndNothingElseChanges) {
  const auto in = MakeGridOnly(gui::kVisibleFull, gui::kLensTypeFisheyeEquidist, 0.0f, 0.0f);
  constexpr float kDx = 123.0f;
  constexpr float kDy = 45.0f;

  const auto at_origin = Compute(in, 800.0f, 400.0f);
  const auto shifted = Compute(in, 800.0f, 400.0f, kDx, kDy);
  ASSERT_GT(at_origin.size(), 0u);
  ASSERT_EQ(shifted.size(), at_origin.size());

  for (size_t i = 0; i < at_origin.size(); ++i) {
    EXPECT_EQ(shifted[i].text, at_origin[i].text) << "label " << i;
    EXPECT_EQ(shifted[i].group, at_origin[i].group) << "label " << i;
    EXPECT_NEAR(shifted[i].screen_x - at_origin[i].screen_x, kDx, 1e-3f) << "label " << i;
    EXPECT_NEAR(shifted[i].screen_y - at_origin[i].screen_y, kDy, 1e-3f) << "label " << i;
  }
}

// ---- Un-projecting a pixel back to a sky direction ----
//
// This is the function every label position is built on, and its failure mode is not a crash but a
// picture that is upside down or mirrored — the rectangular fallback answered every unlisted lens
// with latitudes of the wrong sign, which is how "the latitude labels are inverted" was reported.
// Each row below is a point whose answer is derivable in closed form, so a changed formula turns
// red instead of agreeing with itself.
//
// The world frame at elev=0/az=0/roll=0 has forward at -x and the zenith at -z. Viewports are
// 200x200, so the dual-disc lenses put short_res at 100, circle_radius at 50, and the upper
// hemisphere's disc centre at px=-50.
TEST(OverlayLabels, EachProjectionUnprojectsItsAnchorPixelsToTheDirectionItsGeometryDemands) {
  float view[9];
  gui::BuildViewMatrix(0.0f, 0.0f, 0.0f, view);

  struct ProbeCase {
    const char* name;
    float px;
    float py;
    int lens;
    float fov;
    float wx;
    float wy;
    float wz;
    float tol;
  };
  const ProbeCase kCases[] = {
    // Single orthographic at 180 degrees has r_norm = sin(theta), so the disc edge is theta=90.
    // The centre pixel is theta=0, i.e. straight ahead.
    { "single ortho, centre is straight ahead", 0.0f, 0.0f, gui::kLensTypeFisheyeOrthographic, 180.0f, -1.0f, 0.0f,
      0.0f, 1e-4f },
    // The top edge is r_norm=1 at phi=90, i.e. the zenith. This is the load-bearing "up is positive
    // latitude" row: the retired fallback mapped this pixel to -90 degrees, its exact inverse.
    { "single ortho, top edge is the zenith", 0.0f, 100.0f, gui::kLensTypeFisheyeOrthographic, 180.0f, 0.0f, 0.0f,
      -1.0f, 1e-3f },
    // Dual orthographic's two disc centres are the two poles; the branch that picks between them is
    // the only thing that can put them the wrong way round.
    { "dual ortho, left disc centre is the zenith", -50.0f, 0.0f, gui::kLensTypeDualFisheyeOrthographic, 170.0f, 0.0f,
      0.0f, -1.0f, 1e-3f },
    { "dual ortho, right disc centre is the nadir", 50.0f, 0.0f, gui::kLensTypeDualFisheyeOrthographic, 170.0f, 0.0f,
      0.0f, 1.0f, 1e-3f },
    // At half the disc radius each dual-fisheye type's radial law gives a different, exactly
    // derivable polar angle — which a disc-centre probe cannot see, since theta comes out 0 there
    // for every type. py=0 keeps the probe on the disc's horizontal axis, where phi=0.
    // Equidistant: theta = use_r * 90 = 45 degrees.
    { "dual equidistant at half radius", -25.0f, 0.0f, gui::kLensTypeDualFisheyeEquidist, 180.0f, 0.0f, 0.70711f,
      -0.70711f, 1e-3f },
    // Stereographic: theta = 2*atan(use_r) = 53.13 degrees.
    { "dual stereographic at half radius", -25.0f, 0.0f, gui::kLensTypeDualFisheyeStereographic, 180.0f, 0.0f, 0.8f,
      -0.6f, 1e-3f },
  };

  for (const ProbeCase& c : kCases) {
    float wx = 0.0f;
    float wy = 0.0f;
    float wz = 0.0f;
    bool valid = false;
    gui::detail::PixelToWorldDirForTesting(c.px, c.py, 200.0f, 200.0f, c.lens, c.fov, view, &wx, &wy, &wz, &valid);
    EXPECT_TRUE(valid) << c.name;
    EXPECT_NEAR(wx, c.wx, c.tol) << c.name;
    EXPECT_NEAR(wy, c.wy, c.tol) << c.name;
    EXPECT_NEAR(wz, c.wz, c.tol) << c.name;
  }

  // Two more where only the SIGN is derivable, which is exactly what the inversion bug got wrong.
  // Rectangular: a pixel above centre must be an upward direction (altitude = asin(-wz) > 0).
  float wx = 0.0f;
  float wy = 0.0f;
  float wz = 0.0f;
  bool valid = false;
  gui::detail::PixelToWorldDirForTesting(0.0f, 30.0f, 200.0f, 200.0f, gui::kLensTypeRectangular, 360.0f, view, &wx, &wy,
                                         &wz, &valid);
  EXPECT_TRUE(valid);
  EXPECT_LT(wz, -0.1f) << "a pixel above centre in rectangular must point up";

  // Dual equal-area: a point above the upper disc's centre must tilt toward +x and stay up. Without
  // the vertical flip this came out tilted toward -x — the mirrored picture that was reported.
  gui::detail::PixelToWorldDirForTesting(-50.0f, 30.0f, 200.0f, 200.0f, gui::kLensTypeDualFisheyeEqualArea, 180.0f,
                                         view, &wx, &wy, &wz, &valid);
  EXPECT_TRUE(valid);
  EXPECT_GT(wx, 0.1f) << "above the upper disc's centre must tilt toward +x";
  EXPECT_LT(wz, 0.0f) << "... and stay in the upper hemisphere";
}

// ---- A label that would hang off the panel is pulled back inside ----
//
// The inset is two pixels, and the label's own size counts against the far edges. The last row is
// the deliberate exception: a viewport too narrow to hold the text at all leaves x untouched rather
// than clamping to a nonsense position, which keeps the legacy "centred on its anchor" look.
TEST(OverlayLabels, ALabelIsPulledBackInsideTheViewportOnEveryEdge) {
  const ImVec2 kText(30.0f, 14.0f);  // a 30x14 glyph run
  constexpr float kVpX = 10.0f;
  constexpr float kVpY = 20.0f;
  constexpr float kVpW = 200.0f;
  constexpr float kVpH = 100.0f;

  struct ClampCase {
    const char* name;
    ImVec2 pos;
    float vp_w;
    ImVec2 expected;
  };
  const ClampCase kCases[] = {
    // x=5 would start left of the viewport at x=10; the inset puts it at 12.
    { "past the left edge", ImVec2(5.0f, 50.0f), kVpW, ImVec2(12.0f, 50.0f) },
    // x=195 plus 30 of text ends at 225, past the right edge at 210; 210 - 30 - 2 = 178.
    { "past the right edge", ImVec2(195.0f, 50.0f), kVpW, ImVec2(178.0f, 50.0f) },
    { "above the top edge", ImVec2(50.0f, 15.0f), kVpW, ImVec2(50.0f, 22.0f) },
    // 20 + 100 - 14 - 2 = 104.
    { "below the bottom edge", ImVec2(50.0f, 110.0f), kVpW, ImVec2(50.0f, 104.0f) },
    { "already inside", ImVec2(50.0f, 50.0f), kVpW, ImVec2(50.0f, 50.0f) },
    // 20 wide cannot hold 30 of text plus two insets, so x is left alone; y is processed normally
    // and happens to need no adjustment.
    { "a viewport too narrow for the text", ImVec2(5.0f, 50.0f), 20.0f, ImVec2(5.0f, 50.0f) },
  };

  for (const ClampCase& c : kCases) {
    const ImVec2 clamped = gui::detail::ClampLabelPosToViewport(c.pos, kText, kVpX, kVpY, c.vp_w, kVpH);
    EXPECT_EQ(clamped.x, c.expected.x) << c.name;
    EXPECT_EQ(clamped.y, c.expected.y) << c.name;
  }
}

// ---- Two boundary-nudge cases used to sit here, and they are gone rather than moved ----
//
// They asserted that the hemisphere boundary curve and the front-half boundary curve are pushed
// three degrees onto the visible side before a label is placed on them. Both built the curve from a
// `sin(3 degrees)` constant of their own and then asserted arithmetic about it — one recovered the
// altitude with asin, the other took a dot product against an orthonormal basis — so neither called
// anything that computes a label, and neither could go red for any change to the code that does.
//
// The mechanism they described belongs to the retired boundary-centric placement. Its replacement
// walks each curve and emits at the entry point of every visible arc, and the only tolerance band
// left at the front-hemisphere edge is the float-noise `kFrontEps` in overlay_labels.cpp — there is
// no three-degree offset anywhere in src/gui to be red about. What the curve walk really does at
// those edges is pinned by the cases that drive it: the front-mode pair above, and the arc-count cap
// at the bottom of this file.

// ---- The sky's interior latitudes are labelled, not only where curves meet the panel edge ----
//
// At a wide field of view the grid curves close inside the viewport and never cross its edge, so an
// edge-crossing sampler alone leaves the picture with no latitudes at all. Distinct texts rather
// than a raw count: five labels all reading the same value would not be five latitudes.
TEST(OverlayLabels, AFullSkyViewIsLabelledWithSeveralDistinctLatitudes) {
  struct SkyCase {
    const char* name;
    int visible;
    int lens;
  };
  const SkyCase kCases[] = {
    { "orthographic, whole sky", gui::kVisibleFull, gui::kLensTypeFisheyeOrthographic },
    // Upper alone: the equator boundary covers altitude 0, and the interior sampler must fill the
    // rest — otherwise the hemisphere carries a single label.
    { "orthographic, upper hemisphere", gui::kVisibleUpper, gui::kLensTypeFisheyeOrthographic },
    // A second lens, so this is a property of every view-transformed projection rather than of
    // orthographic in particular.
    { "equidistant, whole sky", gui::kVisibleFull, gui::kLensTypeFisheyeEquidist },
  };

  for (const SkyCase& c : kCases) {
    auto in = MakeGridOnly(c.visible, c.lens, 0.0f, 0.0f);
    in.fov = 180.0f;
    EXPECT_GE(CountDistinctGridTexts(Compute(in, 200.0f, 200.0f)), 5) << c.name;
  }
}

// ---- Either of the horizon's two switches puts it in the request ----
//
// Each overlay family has one toggle for its curve and one for its numbers, and both halves now
// come out of the same annotation request: the curve as a mask the preview shader samples, the
// numbers as anchors. So the request carries the horizon when EITHER switch is on, exactly as the
// grid and the circles do (the next case pins theirs), and neither switch alone can starve the
// other's half.
//
// This case used to assert `in.horizon == labels`, because the preview derived the horizon LINE
// itself in its fragment shader from fwidth(altitude) and had no use for a mask. That was the last
// annotation the two renderers each computed for themselves, and the two answers were not the same
// curve; the shader reads core's mask now. What the label switch still decides on its own is
// whether the TEXT is drawn, and that is read where the anchors are consumed (RenderPreviewPanel),
// the same place the other two families' label switches are read.
TEST(OverlayToggle, EitherHorizonSwitchPutsItInTheAnnotationRequest) {
  struct ToggleCase {
    const char* name;
    bool lines;
    bool labels;
  };
  const ToggleCase kCases[] = {
    { "lines on, labels off", true, false },
    { "lines off, labels on", false, true },
    { "both on", true, true },
    { "both off", false, false },
  };

  for (const ToggleCase& c : kCases) {
    gui::GuiState s;
    s.show_horizon_line = s.show_grid_line = s.show_sun_circles_line = c.lines;
    s.show_horizon_label = s.show_grid_label = s.show_sun_circles_label = c.labels;
    const auto in = gui::AnnotationViewInputFor(s, s.renderer);
    EXPECT_EQ(in.horizon, c.lines || c.labels) << c.name;
  }
}

// ---- Which family a request carries follows the switches, not the frame ----
//
// The two lists AnnotationViewInputFor fills are not free: each angle is a level the mask sweep
// tests per pixel and a curve the label walk walks. A view input that always carried the grid would
// make a user who turned the grid off pay for it on every settle, invisibly.
TEST(OverlayToggle, TheAnnotationRequestCarriesOnlyTheFamiliesWhoseSwitchesAreOn) {
  gui::GuiState s;
  s.renderer.fov = 90.0f;  // step 20 deg, so the lists are small but non-empty

  s.show_grid_line = s.show_grid_label = false;
  s.show_sun_circles_line = s.show_sun_circles_label = false;
  auto off = gui::AnnotationViewInputFor(s, s.renderer);
  EXPECT_TRUE(off.elevation_deg.empty());
  EXPECT_TRUE(off.longitude_deg.empty());
  EXPECT_TRUE(off.angular_dist_deg.empty());

  // The LINE switch alone is enough: a user drawing curves and no numbers still needs the mask.
  s.show_grid_line = true;
  auto lines_only = gui::AnnotationViewInputFor(s, s.renderer);
  EXPECT_FALSE(lines_only.elevation_deg.empty());
  EXPECT_FALSE(lines_only.longitude_deg.empty());
  EXPECT_TRUE(lines_only.angular_dist_deg.empty()) << "the circles' switches are still off";

  // ...and so is the LABEL switch alone, for the mirror-image reason.
  s.show_grid_line = false;
  s.show_grid_label = true;
  auto labels_only = gui::AnnotationViewInputFor(s, s.renderer);
  EXPECT_FALSE(labels_only.elevation_deg.empty());
  EXPECT_FALSE(labels_only.longitude_deg.empty());
}

// ---- The horizon owns the zero, and the grid must not print a second one ----
//
// The grid skips altitude 0 so the two families do not stack two "0" labels on the same line. The
// prime meridian legitimately emits a "0" of its own as a LONGITUDE, so the grid-only expectation is
// at most one, never two — two means the altitude skip is broken.
TEST(OverlayToggle, TheZeroDegreeLabelBelongsToTheHorizonAndTheGridDoesNotRepeatIt) {
  auto in = MakeGridOnly(gui::kVisibleFull, gui::kLensTypeFisheyeEquidist, 0.0f, 0.0f);
  in.fov = 180.0f;  // full sky, so the equator meets the viewport edges symmetrically
  const std::string zero = std::string("0") + kDeg;

  auto horizon_only = in;
  horizon_only.show_horizon = true;
  EXPECT_GE(CountText(Compute(horizon_only, 200.0f, 200.0f, 0.0f, 0.0f, /*with_grid=*/false), zero), 1);

  auto grid_only = in;
  grid_only.show_horizon = false;
  EXPECT_LE(CountText(Compute(grid_only, 200.0f, 200.0f), zero), 1);
}

// ---- The four placement gaps of doc/overlay-label-placement.md ----
//
// Each of these was a projection where the user could see grid curves with no numbers on them at
// all, or the opposite — one curve wearing a pile of them. The first three are counts because the
// defect was total absence; the fourth is a cap because the defect was a cluster.

// The globe and rectangular lenses drew latitudes but no longitudes: the globe's sampler only walked
// altitude rings, and rectangular's top and bottom edges are azimuth-singular, so the edge sampler's
// continuity guard rejected every azimuth crossing there. Walking each curve directly reaches both.
TEST(OverlayLabels, TheGlobeAndRectangularLensesLabelLongitudesAndNotOnlyLatitudes) {
  struct LensCase {
    const char* name;
    int lens;
    float fov;
    float vp_w;
    float vp_h;
    int min_labels;  // pre-fix counts were 13 (globe, latitudes only) and 40 (rectangular)
  };
  const LensCase kCases[] = {
    { "globe", gui::kLensTypeGlobe, 60.0f, 512.0f, 512.0f, 20 },
    { "rectangular", gui::kLensTypeRectangular, 120.0f, 800.0f, 400.0f, 30 },
  };

  for (const LensCase& c : kCases) {
    auto in = MakeGridOnly(gui::kVisibleFull, c.lens, /*elev=*/20.0f, /*az=*/0.0f);
    in.fov = c.fov;
    EXPECT_GE(CountGrid(in, c.vp_w, c.vp_h), c.min_labels) << c.name;
  }
}

// Dual fisheye had no labels at all: a full-sky lens skipped every interior sampler, and edge
// sampling never crossed either disc. Both discs must carry them, not just one — a per-disc
// visibility test that got one branch wrong would leave half the sky unlabelled.
TEST(OverlayLabels, DualFisheyeLabelsBothOfItsDiscs) {
  auto in = MakeGridOnly(gui::kVisibleFull, gui::kLensTypeDualFisheyeEqualArea, /*elev=*/20.0f, /*az=*/0.0f);
  in.show_horizon = true;
  const auto labels = Compute(in, 512.0f, 512.0f);
  ASSERT_GT(CountInGroup(labels, kGridGroup), 0);

  // On a 512-wide viewport the discs are centred at x=128 and x=384; the band between 192 and 320 is
  // left out so a label straddling the seam decides neither way.
  bool has_left = false;
  bool has_right = false;
  for (const auto& l : labels) {
    has_left = has_left || (l.group == kGridGroup && l.screen_x < 192.0f);
    has_right = has_right || (l.group == kGridGroup && l.screen_x > 320.0f);
  }
  EXPECT_TRUE(has_left);
  EXPECT_TRUE(has_right);
}

// A curve grazing the viewport edge used to emit a label at every adjacent pixel pair that crossed
// it — about fifteen copies of "10" stacked on one line. Walking the curve emits at most one per
// visible arc, and a closed altitude ring has one or two arcs in a single fisheye view. The cap of
// four allows for the longitude curve of the same value contributing its own.
TEST(OverlayLabels, ACurveGrazingTheViewportEdgeDoesNotStackLabelsOnTopOfEachOther) {
  auto in = MakeGridOnly(gui::kVisibleFull, gui::kLensTypeFisheyeEquidist, 0.0f, 0.0f);
  in.fov = 120.0f;
  EXPECT_LE(CountText(Compute(in, 512.0f, 512.0f), std::string("10") + kDeg), 4);
}
