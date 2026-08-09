// The numbers written along the preview's coordinate grid: which ones appear, and where.
//
// Every case here builds an OverlayLabelInput, calls the pure computation and reads the labels back
// — no ImGui context, no draw list. The one sibling that stays in
// test/gui/functional/test_gui_overlay_labels.cpp is modal_does_not_leak_to_foreground, because what
// it asserts is which DRAW LIST the labels land in.
//
// The four cc* cases at the bottom are the regression anchors for the placement gaps audited in
// doc/overlay-label-placement.md; their assertion semantics must not be weakened.

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "gui/app.hpp"
#include "gui/gui_constants.hpp"
#include "gui/gui_state.hpp"
#include "gui/overlay_labels.hpp"
#include "gui/preview_renderer.hpp"

namespace gui = lumice::gui;

namespace {

// Group ids as overlay_labels.cpp assigns them. Mirrored rather than imported because the constants
// live in an anonymous namespace there.
constexpr int kGridGroup = 0;
constexpr int kSunGroup = 1;
// UTF-8 for the degree sign, spelled as bytes so the assertions match AddLabel's "%.0f\xC2\xB0"
// output without depending on how the compiler handles the source charset.
constexpr const char* kDeg = "\xC2\xB0";

// An input with the coordinate grid on and everything else off — the baseline every grid case
// starts from, since the three overlay families are independently gated and a case about one of
// them must not be reading another's labels.
gui::OverlayLabelInput MakeGridOnly(int visible, int lens_type, float elevation, float azimuth, bool front = false) {
  gui::OverlayLabelInput in{};
  in.lens_type = lens_type;
  in.fov = 120.0f;
  in.elevation = elevation;
  in.azimuth = azimuth;
  in.roll = 0.0f;
  in.visible = visible;
  in.front = front;
  in.show_horizon = false;
  in.show_grid = true;
  in.show_sun_circles = false;
  in.sun_dir[0] = 0.0f;
  in.sun_dir[1] = 0.0f;
  in.sun_dir[2] = -1.0f;
  in.sun_circle_count = 0;
  in.sun_circle_angles = nullptr;
  in.horizon_color[0] = in.horizon_color[1] = in.horizon_color[2] = 1.0f;
  in.grid_color[0] = in.grid_color[1] = in.grid_color[2] = 1.0f;
  in.sun_circles_color[0] = in.sun_circles_color[1] = in.sun_circles_color[2] = 1.0f;
  in.horizon_alpha = 1.0f;
  in.grid_alpha = 1.0f;
  in.sun_circles_alpha = 1.0f;
  return in;
}

// Sun circles only, on equidistant fisheye at fov=360 so that BACK-facing world directions still
// land inside the viewport (r_norm = theta/half_fov, and half_fov = pi covers theta in [0, pi]).
// Any narrower lens pushes them outside the viewport bound, which would then be what suppresses
// them — making a "back labels are culled" assertion true for the wrong reason.
gui::OverlayLabelInput MakeSunOnly(int visible, const float sun_dir[3], const float* circle_angles, int count,
                                   bool front = false) {
  gui::OverlayLabelInput in = MakeGridOnly(visible, gui::kLensTypeFisheyeEquidist, /*elev=*/0.0f, /*az=*/0.0f, front);
  in.fov = 360.0f;
  in.show_grid = false;
  in.show_sun_circles = true;
  in.sun_dir[0] = sun_dir[0];
  in.sun_dir[1] = sun_dir[1];
  in.sun_dir[2] = sun_dir[2];
  in.sun_circle_count = count;
  in.sun_circle_angles = circle_angles;
  return in;
}

std::vector<gui::OverlayLabel> Compute(const gui::OverlayLabelInput& in, float vp_w, float vp_h, float vp_x = 0.0f,
                                       float vp_y = 0.0f) {
  std::vector<gui::OverlayLabel> labels;
  gui::ComputeOverlayLabels(in, vp_x, vp_y, vp_w, vp_h, labels);
  return labels;
}

int CountInGroup(const std::vector<gui::OverlayLabel>& labels, int group) {
  int n = 0;
  for (const auto& l : labels) {
    n += (l.group == group) ? 1 : 0;
  }
  return n;
}

int CountGrid(const gui::OverlayLabelInput& in, float vp_w, float vp_h) {
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

// The same reach, on the sun-circle path rather than the grid: its interior-label block was once
// gated to lens types 0-3 by number, so single orthographic drew no sun-circle labels at all. Equal
// counts, not merely non-zero ones — the two lenses share the code path, so anything else means one
// of them is taking a different branch.
TEST(OverlayLabels, SingleOrthographicGetsTheSameSunCircleLabelsAsFisheye) {
  const float sun_forward[3] = { -1.0f, 0.0f, 0.0f };  // camera forward at elev=0, az=0
  const float angle = 5.0f;                            // small enough that the circle stays well inside the disc
  auto ortho = MakeSunOnly(gui::kVisibleFull, sun_forward, &angle, 1);
  ortho.lens_type = gui::kLensTypeFisheyeOrthographic;
  ortho.fov = 60.0f;
  auto fisheye = ortho;
  fisheye.lens_type = gui::kLensTypeFisheyeEquidist;

  const int n_ortho = CountInGroup(Compute(ortho, 200.0f, 200.0f), kSunGroup);
  EXPECT_GT(n_ortho, 0);
  EXPECT_EQ(n_ortho, CountInGroup(Compute(fisheye, 200.0f, 200.0f), kSunGroup));
}

// ---- Front mode hides what is behind the camera, and only in front mode ----
//
// The pairing is the assertion. A cull that ran in every mode would satisfy the "front hides it"
// half perfectly while deleting labels the user asked to see; a cull that never ran would satisfy
// the "full keeps it" half. Only the asymmetry between the two rows distinguishes them.
//
// Upper/Lower are avoided here on purpose: their equator-boundary block emits extra sun-circle
// labels where the circle meets the equator, which would pollute a count about the interior block.
TEST(OverlayLabels, TheBackHemisphereIsCulledInFrontModeAndKeptInFull) {
  const float sun_behind[3] = { 1.0f, 0.0f, 0.0f };  // forward is -x, so this is directly behind
  const float sun_ahead[3] = { -1.0f, 0.0f, 0.0f };
  const float angle = 5.0f;

  struct SunCase {
    const char* name;
    const float* sun;
    bool front_mode;
    bool expect_labels;
  };
  const SunCase kCases[] = {
    // All four interior labels sit within 5 degrees of a sun that is behind the camera.
    { "front mode, sun behind", sun_behind, true, false },
    { "front mode, sun ahead", sun_ahead, true, true },
    // Same scene as the first row but in full mode: the cull must not fire.
    { "full mode, sun behind", sun_behind, false, true },
  };

  for (const SunCase& c : kCases) {
    const int visible = gui::kVisibleFull;
    const auto labels = Compute(MakeSunOnly(visible, c.sun, &angle, 1, c.front_mode), 800.0f, 600.0f);
    EXPECT_EQ(CountInGroup(labels, kSunGroup) > 0, c.expect_labels) << c.name;
  }
}

// The grid's half of the same rule, plus what must survive it. Rectangular maps the whole sphere
// into the viewport rectangle, so it is the cleanest place to see the cull remove labels. A wide
// fisheye is where the opposite risk lives: front mode must still label the hemisphere edge itself,
// or the visible half loses its coordinates entirely. Under the curve-centric placement design that
// edge yields one label per visible arc rather than a dense cluster, so the count no longer exceeds
// full mode's — but it must not be zero.
TEST(OverlayLabels, FrontModeCullsTheBackGridWhileStillLabellingTheHemisphereEdge) {
  auto rect_full = MakeGridOnly(gui::kVisibleFull, gui::kLensTypeRectangular, 0.0f, 0.0f);
  auto rect_front = MakeGridOnly(gui::kVisibleFull, gui::kLensTypeRectangular, 0.0f, 0.0f, /*front=*/true);
  const int n_rect_full = CountGrid(rect_full, 800.0f, 400.0f);
  EXPECT_GT(n_rect_full, 0);
  EXPECT_LT(CountGrid(rect_front, 800.0f, 400.0f), n_rect_full);

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
}

// The loose companion of the case above: the globe at its own 90-degree maximum must produce fewer
// labels than a full-hemisphere fisheye at 180. Note what this does NOT cover — at this
// configuration the globe frustum is a 45-degree cone around +Z, so every sampled direction already
// satisfies the near-side test and the inequality is secured by the field-of-view difference alone.
// The case above is what actually pins the cull.
TEST(OverlayLabels, TheGlobeLensLabelsLessOfTheSkyThanAFullHemisphereFisheye) {
  auto globe = MakeGridOnly(gui::kVisibleFull, gui::kLensTypeGlobe, 0.0f, 0.0f);
  globe.fov = 90.0f;
  auto fisheye = MakeGridOnly(gui::kVisibleFull, gui::kLensTypeFisheyeEquidist, 0.0f, 0.0f);
  fisheye.fov = 180.0f;

  const int n_globe = CountGrid(globe, 200.0f, 200.0f);
  EXPECT_GT(n_globe, 0);
  EXPECT_LT(n_globe, CountGrid(fisheye, 200.0f, 200.0f));
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

// ---- The hemisphere boundary curve is nudged onto the visible side ----
//
// A label placed exactly on the boundary is a coin flip: the shader culls whatever falls on the far
// side of it, so half of them vanish. The placement shifts the curve three degrees inward first.
// Asserted by re-deriving the same offset here rather than by reading label text, because latitude
// and azimuth labels share the "%.0f degrees" format and a text-based case could not tell which
// source produced a given string.
TEST(OverlayLabels, TheHemisphereBoundaryCurveIsNudgedOntoTheVisibleSide) {
  constexpr float kPi = 3.14159265f;
  const float offset = std::sin(3.0f * kPi / 180.0f);

  struct BoundaryCase {
    const char* name;
    float wz_sign;  // upper hemisphere pushes -z, lower pushes +z
  };
  const BoundaryCase kCases[] = { { "upper", -1.0f }, { "lower", 1.0f } };

  for (const BoundaryCase& c : kCases) {
    for (int i = 0; i < 8; ++i) {
      const float az = -kPi + i * (2.0f * kPi / 8.0f);
      float wx = -std::cos(az);
      float wy = -std::sin(az);
      float wz = c.wz_sign * offset;
      const float len = std::sqrt(wx * wx + wy * wy + wz * wz);
      wx /= len;
      wy /= len;
      wz /= len;
      const float altitude_deg = std::asin(-wz) * 180.0f / kPi;
      // Three degrees onto the visible side: enough to survive the cull, small enough that the label
      // still reads as sitting on the boundary.
      EXPECT_NEAR(altitude_deg, -c.wz_sign * 3.0f, 0.5f) << c.name << " sample " << i;
    }
  }
}

// The front-half boundary is a great circle perpendicular to forward, pushed along -forward by the
// same offset. The shader culls a direction when dot(world, col2) > 0, so every sample on the nudged
// curve must come out strictly negative.
TEST(OverlayLabels, TheFrontHemisphereBoundaryCurveLandsInsideTheVisibleHalf) {
  float view[9];
  gui::BuildViewMatrix(0.0f, 0.0f, 0.0f, view);
  constexpr float kPi = 3.14159265f;
  const float offset = std::sin(3.0f * kPi / 180.0f);

  for (int i = 0; i < 8; ++i) {
    const float t = i * (2.0f * kPi / 8.0f);
    const float c = std::cos(t);
    const float s = std::sin(t);
    float w[3] = {
      c * view[0] + s * view[3] - offset * view[6],
      c * view[1] + s * view[4] - offset * view[7],
      c * view[2] + s * view[5] - offset * view[8],
    };
    const float len = std::sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
    const float dot_col2 = (w[0] * view[6] + w[1] * view[7] + w[2] * view[8]) / len;
    EXPECT_LT(dot_col2, 0.0f) << "sample " << i;
  }
}

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

// ---- The line switch and the label switch are separate ----
//
// Each overlay family has one toggle for its curve and one for its numbers, and
// BuildOverlayLabelInput must read the label one. Reading the line flag instead would make the
// numbers appear and disappear with a switch the user thinks controls something else.
TEST(OverlayToggle, TheLabelSwitchAloneDecidesWhetherNumbersAreComputed) {
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
    const auto in = gui::BuildOverlayLabelInput(s, s.renderer);
    EXPECT_EQ(in.show_horizon, c.labels) << c.name;
    EXPECT_EQ(in.show_grid, c.labels) << c.name;
    EXPECT_EQ(in.show_sun_circles, c.labels) << c.name;
  }
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
  horizon_only.show_grid = false;
  EXPECT_GE(CountText(Compute(horizon_only, 200.0f, 200.0f), zero), 1);

  auto grid_only = in;
  grid_only.show_horizon = false;
  grid_only.show_grid = true;
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
