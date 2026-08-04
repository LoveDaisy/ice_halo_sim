// ComputeOverlayLabels / DrawOverlayLabels contract tests. Every case here instantiates an
// OverlayLabelInput, calls the pure computation directly and inspects the returned labels — no
// ImGui context, no draw list — so they run in the windowless target.
//
// The one sibling that stays in test/gui/functional/test_gui_overlay_labels.cpp is
// modal_does_not_leak_to_foreground: it needs a real ImGui frame, because what it asserts is
// which DRAW LIST the labels land in.
//
// cc1-cc4 at the bottom are the direct regression anchors for the four placement gaps audited in
// doc/overlay-label-placement.md; their assertion semantics must not be weakened.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "gui/app.hpp"
#include "gui/gui_constants.hpp"
#include "gui/gui_state.hpp"
#include "gui/overlay_labels.hpp"
#include "gui/preview_renderer.hpp"

namespace {

// Convenience: build an OverlayLabelInput with grid enabled, sun circles disabled.
lumice::gui::OverlayLabelInput MakeGridOnly(int visible, int lens_type, float elevation, float azimuth,
                                            bool front = false) {
  lumice::gui::OverlayLabelInput in{};
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

// Convenience: input with sun circles only. Uses Fisheye Equidistant (lens_type=2) with
// fov=360° so back-facing world directions land WITHIN the viewport (r_norm = theta/half_fov,
// and half_fov=π fits theta∈[0,π]). Other lenses or smaller fov push back-facing dirs outside
// viewport bounds — the viewport bound check would then mask is_visible_front, making
// "back labels suppressed" tests vacuously true. With this setup, is_visible_front is the
// sole gate for sun-circle interior labels in the back hemisphere.
lumice::gui::OverlayLabelInput MakeSunOnly(int visible, const float sun_dir[3], const float* circle_angles, int count,
                                           bool front = false) {
  lumice::gui::OverlayLabelInput in =
      MakeGridOnly(visible, /*lens=Fisheye Equidistant*/ 2, /*elev*/ 0.0f, /*az*/ 0.0f, front);
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

int CountSunCircleLabels(const std::vector<lumice::gui::OverlayLabel>& labels) {
  int n = 0;
  for (const auto& l : labels) {
    if (l.group == 1)
      ++n;  // group 1 = sun circles
  }
  return n;
}

int CountGridLabels(const std::vector<lumice::gui::OverlayLabel>& labels) {
  int n = 0;
  for (const auto& l : labels) {
    if (l.group == 0)
      ++n;
  }
  return n;
}

// Counts distinct grid-group label texts (i.e. unique latitude/longitude
// values present), used by the source-4 ortho/equidist fov=180 tests below.
int CountUniqueGridLabels(const std::vector<lumice::gui::OverlayLabel>& labels) {
  std::vector<std::string> seen;
  for (const auto& l : labels) {
    if (l.group != 0)
      continue;
    bool dup = false;
    for (const auto& s : seen) {
      if (s == l.text) {
        dup = true;
        break;
      }
    }
    if (!dup)
      seen.push_back(l.text);
  }
  return static_cast<int>(seen.size());
}

}  // namespace

// Regression test for the single-orthographic dispatch: at the same
// view config, single orthographic (lens=8) and Fisheye Equidistant (lens=2)
// must produce comparably-sized grid label sets. Pre-fix, lens=8 fell to the
// RectangularInv fallback in PixelToWorldDir, producing a label set with
// completely different geometry (and the symptomatic upside-down latitude
// labels once reported). The orthographic / equidistant
// fisheye pair shares the same projection topology (single disc, view-matrix
// applied), so their label counts on the same viewport differ only in the
// radial mapping detail — well within ±50% of each other in practice.
//
// Note (dual orthographic, lens=9): a parallel test for lens=9 is omitted
// intentionally. Dual fisheye / orthographic projections always size their
// twin discs to fit exactly in the viewport (circle_radius = min(W/2, H)/2),
// so viewport edges are at most tangent to the discs — never a chord — and
// ComputeOverlayLabels' edge-sampling loop yields no valid sample pairs for
// any lens=9 configuration. The fix for lens=9 in PixelToWorldDir uses the
// same dispatch pattern as lens=8 (FisheyeInv vs DualFisheyeInv with type=3),
// so the lens=8 regression below is sufficient coverage of the dispatch.
TEST(OverlayLabels, single_orthographic_dispatch_matches_fisheye) {
  constexpr float kVpX = 0.0f;
  constexpr float kVpY = 0.0f;
  constexpr float kVpW = 200.0f;
  constexpr float kVpH = 200.0f;

  auto in_ortho = MakeGridOnly(lumice::gui::kVisibleFull, lumice::gui::kLensTypeFisheyeOrthographic,
                               /*elev*/ 0.0f, /*az*/ 0.0f);
  in_ortho.fov = 60.0f;
  std::vector<lumice::gui::OverlayLabel> labels_ortho;
  lumice::gui::ComputeOverlayLabels(in_ortho, kVpX, kVpY, kVpW, kVpH, labels_ortho);

  auto in_fisheye = MakeGridOnly(lumice::gui::kVisibleFull, lumice::gui::kLensTypeFisheyeEquidist,
                                 /*elev*/ 0.0f, /*az*/ 0.0f);
  in_fisheye.fov = 60.0f;
  std::vector<lumice::gui::OverlayLabel> labels_fisheye;
  lumice::gui::ComputeOverlayLabels(in_fisheye, kVpX, kVpY, kVpW, kVpH, labels_fisheye);

  int n_ortho = CountGridLabels(labels_ortho);
  int n_fisheye = CountGridLabels(labels_fisheye);

  // Both must be non-zero (sanity).
  EXPECT_GT(n_ortho, 0);
  EXPECT_GT(n_fisheye, 0);

  // The two single-fisheye-class projections differ only in radial mapping —
  // edge-sampling crossings should cluster at similar counts. Ratio guard
  // catches regressions like the pre-fix Rectangular fallback (which would
  // give a count off by a large factor from the fisheye reference).
  // Pick a generous ±50% band to absorb radial-mapping nonlinearities while
  // still failing under a wholly different projection.
  EXPECT_GE(n_ortho * 2, n_fisheye);  // n_ortho >= n_fisheye / 2
  EXPECT_LE(n_ortho, n_fisheye * 2);  // n_ortho <= n_fisheye * 2
}

// Test A: Front-mode rectangular grid produces strictly fewer labels than Full-mode.
// Rectangular (lens_type=7) maps the full sphere to the viewport rectangle, so edge sampling
// exercises grid crossings across the entire sphere — cleanest baseline for hemisphere-cull comparison.
TEST(OverlayLabels, front_rectangular_grid_culls_back) {
  auto in_full = MakeGridOnly(/*visible=*/2, /*lens=Rectangular*/ 7, /*elev*/ 0.0f, /*az*/ 0.0f);
  auto in_front = MakeGridOnly(/*visible=*/2, /*lens=Rectangular*/ 7, /*elev*/ 0.0f, /*az*/ 0.0f, /*front=*/true);

  std::vector<lumice::gui::OverlayLabel> labels_full;
  std::vector<lumice::gui::OverlayLabel> labels_front;
  lumice::gui::ComputeOverlayLabels(in_full, 0.0f, 0.0f, 800.0f, 400.0f, labels_full);
  lumice::gui::ComputeOverlayLabels(in_front, 0.0f, 0.0f, 800.0f, 400.0f, labels_front);

  int n_full = CountGridLabels(labels_full);
  int n_front = CountGridLabels(labels_front);

  EXPECT_GT(n_full, 0);        // baseline sanity
  EXPECT_LT(n_front, n_full);  // Front strictly fewer (back hemisphere labels culled)
}

// Test B: Front-mode + sun behind camera → sun-circle interior labels suppressed.
TEST(OverlayLabels, front_sun_circle_back_suppressed) {
  // forward (elev=0,az=0,roll=0) = (-1,0,0); sun_dir=(1,0,0) places the sun directly behind.
  const float sun_back[3] = { 1.0f, 0.0f, 0.0f };
  const float angle = 5.0f;  // small circle stays inside the viewport (interior labels path)
  auto in = MakeSunOnly(/*visible=*/2, sun_back, &angle, 1, /*front=*/true);

  std::vector<lumice::gui::OverlayLabel> labels;
  lumice::gui::ComputeOverlayLabels(in, 0.0f, 0.0f, 800.0f, 600.0f, labels);

  // All 4 interior labels lie within ±5° of sun_dir in the back hemisphere → all culled.
  EXPECT_EQ(CountSunCircleLabels(labels), 0);
}

// Test C: Front-mode + sun in front → sun-circle interior labels retained.
TEST(OverlayLabels, front_sun_circle_front_retained) {
  // sun_dir = forward = (-1,0,0); 4 interior labels all in front hemisphere.
  const float sun_front[3] = { -1.0f, 0.0f, 0.0f };
  const float angle = 5.0f;
  auto in = MakeSunOnly(/*visible=*/2, sun_front, &angle, 1, /*front=*/true);

  std::vector<lumice::gui::OverlayLabel> labels;
  lumice::gui::ComputeOverlayLabels(in, 0.0f, 0.0f, 800.0f, 600.0f, labels);

  // Expect at least some interior labels (the projection may also drop some by viewport bounds,
  // but at least 1 should survive for a small 5° circle near the optical axis).
  EXPECT_GT(CountSunCircleLabels(labels), 0);
}

// Test E: Front mode + Fisheye Equidistant fov=280° must label the hemisphere boundary.
// Regression guard: the front-half hemisphere edge must still receive labels when
// visible=Front is set.
//
// Semantics note (task 288.6): Under boundary-centric the old assert was
// `n_front > n_full` because `sample_front_great_circle` densely sampled the
// hemisphere boundary and emitted MANY labels per crossing. Under curve-centric
// the design intent is "1 boundary label / visible arc, no clustering"
// (plan §D3 / Step 3), so n_front no longer exceeds n_full — but n_front must
// still be > 0, which proves curves crossing the front-hemisphere edge do
// produce boundary labels at the entry transition.
TEST(OverlayLabels, front_fisheye_boundary_labels) {
  auto in_front =
      MakeGridOnly(/*visible=*/2, /*lens=Fisheye Equidistant*/ 2, /*elev*/ 0.0f, /*az*/ 0.0f, /*front=*/true);
  auto in_full = MakeGridOnly(/*visible=*/2, /*lens=Fisheye Equidistant*/ 2, /*elev*/ 0.0f, /*az*/ 0.0f);
  in_front.fov = 280.0f;
  in_full.fov = 280.0f;

  std::vector<lumice::gui::OverlayLabel> labels_front;
  std::vector<lumice::gui::OverlayLabel> labels_full;
  lumice::gui::ComputeOverlayLabels(in_front, 0.0f, 0.0f, 800.0f, 600.0f, labels_front);
  lumice::gui::ComputeOverlayLabels(in_full, 0.0f, 0.0f, 800.0f, 600.0f, labels_full);

  int n_front = CountGridLabels(labels_front);
  int n_full = CountGridLabels(labels_full);

  EXPECT_GT(n_full, 0);   // baseline sanity (interior labels in Full mode)
  EXPECT_GT(n_front, 0);  // front mode still emits boundary labels at hemisphere edge
}

// Test D: Full mode + back sun must NOT cull (is_visible_front no-op outside Front).
// Critical pairing with Test B: same sun_back scenario, but Full mode keeps labels while
// Test B's Front mode drops them. The asymmetry (Full > 0, Front == 0) is the regression
// detector — if is_visible_front mistakenly evaluated the dot check in non-Front modes,
// n_full would also drop to 0. We avoid Upper/Lower here because their equator-boundary
// block (runs only for Upper/Lower, lens 0-3) generates extra sun-circle edge labels at
// the equator-circle intersection, polluting an interior-block-focused regression test.
TEST(OverlayLabels, full_sun_circle_back_kept) {
  const float sun_back[3] = { 1.0f, 0.0f, 0.0f };
  const float angle = 5.0f;

  std::vector<lumice::gui::OverlayLabel> labels;
  auto in = MakeSunOnly(/*visible=Full*/ 2, sun_back, &angle, 1);
  lumice::gui::ComputeOverlayLabels(in, 0.0f, 0.0f, 800.0f, 600.0f, labels);

  // Must be > 0: matches Test B scenario exactly except for visible=Full vs Front, so any
  // non-zero count proves is_visible_front did not gate in Full mode.
  EXPECT_GT(CountSunCircleLabels(labels), 0);
}

// overlay_labels/globe_label_visibility — Globe lens (lens=10) at its
// MaxFov=90° produces strictly fewer grid labels than Fisheye Equidistant
// at fov=180° under the same view (elev=0, az=0).
//
// What this test actually verifies (honest scope):
//   1) Globe lens label generation is wired up (n_globe > 0).
//   2) Globe at its native MaxFov never produces more grid labels than a
//      full-hemisphere fisheye at fov=180° (regression guard against
//      Globe label sampling running away or duplicating across passes).
//
// What this test does NOT directly verify:
//   - Globe's back-hemisphere `dz > 1/kGlobeCameraD` cull at
//     overlay_labels.cpp:299. At fov=90°/el=0/az=0, the Globe frustum is
//     a 45°-half-angle cone around +Z, so all sampled directions satisfy
//     dz ≥ cos(45°) ≈ 0.707 — the dz cull is geometrically unreachable
//     with this config. The strict inequality below is therefore secured
//     by the fov-asymmetry (90° vs 180°) only, not by the dz cull.
// Direct dz-cull coverage requires either a tilted view (e.g. el ≈ 60°
// bringing the cone into the wz < 0.25 region) or a same-fov comparison
// that pulls the cone past the dz threshold.
// fov=90° here equals MaxFov(Globe), so per-frame clamp does not interact.
TEST(OverlayLabels, globe_label_visibility) {
  // Globe at fov=90° — its full native field of view (MaxFov(Globe)=90°).
  auto in_globe = MakeGridOnly(lumice::gui::kVisibleFull, lumice::gui::kLensTypeGlobe,
                               /*elev*/ 0.0f, /*az*/ 0.0f);
  in_globe.fov = 90.0f;
  // Fisheye Equidistant at fov=180° — full hemisphere, deliberately
  // wider than Globe's 90° to keep n_fisheye > n_globe by a margin.
  auto in_fisheye = MakeGridOnly(lumice::gui::kVisibleFull, lumice::gui::kLensTypeFisheyeEquidist,
                                 /*elev*/ 0.0f, /*az*/ 0.0f);
  in_fisheye.fov = 180.0f;

  std::vector<lumice::gui::OverlayLabel> labels_globe;
  std::vector<lumice::gui::OverlayLabel> labels_fisheye;
  lumice::gui::ComputeOverlayLabels(in_globe, 0.0f, 0.0f, 200.0f, 200.0f, labels_globe);
  lumice::gui::ComputeOverlayLabels(in_fisheye, 0.0f, 0.0f, 200.0f, 200.0f, labels_fisheye);

  int n_globe = CountGridLabels(labels_globe);
  int n_fisheye = CountGridLabels(labels_fisheye);

  // Globe must produce at least one label.
  EXPECT_GT(n_globe, 0);
  // Globe (fov=90°) strictly fewer than fisheye (fov=180°) under the same view.
  EXPECT_LT(n_globe, n_fisheye);
}

// overlay_labels/globe_label_dz_cull_active — directly pin the back-hemisphere
// dz cull at overlay_labels.cpp:299 (`if (dz <= 1.0f / kGlobeCameraD)`).
//
// Companion to globe_label_visibility above, which only verifies the loose
// n_globe < n_fisheye(fov=180°) inequality — that one passes even if the dz
// cull is removed (it's secured by fov-asymmetry alone). This test fails
// whenever the dz cull is removed, threshold flipped, or comparison reversed.
//
// Mechanism:
//   With Globe + fov=90° + elev=0 + az=0, all grid labels in the output come
//   from sample_interior_latitudes (Source 4 in overlay_labels.cpp's
//   sample-source dispatch comment). Source 1 (viewport edges) is empty here:
//   the visible-cone half-angle is asin(1/kGlobeCameraD) = asin(0.25) ≈ 14.5°,
//   so all viewport-edge rays at fov/2 = 45° miss the unit sphere
//   (PixelToWorldDir returns invalid). Source 2/3 are gated on visible≠Full,
//   Source 5 on show_sun_circles=true; both inactive here. So only Source 4
//   contributes, and Source 4's only forward-projection gate (besides
//   viewport-bound) is the dz cull itself — no other path can write the
//   altitude text labels we assert against.
//
// Geometric pin (with kGlobeCameraD=4 ⇒ threshold dz=0.25, view_matrix at
// elev=0/az=0 reduces to dz_eye = wx):
//   - alt=±70°: max wx across azimuths = cos(70°) ≈ 0.342 > 0.25 → some az
//     passes → "70°" / "-70°" labels MUST appear (positive control; guards
//     against threshold being raised to 1.0 which would silently pass the
//     negative assertion).
//   - alt=±80°: max wx across azimuths = cos(80°) ≈ 0.174 < 0.25 → ALL az
//     fail dz cull → "80°" / "-80°" labels MUST NOT appear (the only
//     observable consequence of the dz cull at this config). Removing the
//     cull would let alt=±80° through (denom=D-dz≈3.826, focal=100, projected
//     py ≈ sin(80°)·100/3.826 ≈ 25.7 < hh=100, well inside viewport).
//
// ±85° would also satisfy the math, but kAltitudeSteps in
// overlay_labels.cpp:694 currently runs ±10°…±80° — picking ±80° keeps the
// test resilient to step-set narrowing (only widening could miss it).
TEST(OverlayLabels, globe_label_dz_cull_active) {
  auto in = MakeGridOnly(lumice::gui::kVisibleFull, lumice::gui::kLensTypeGlobe,
                         /*elev*/ 0.0f, /*az*/ 0.0f);
  in.fov = 90.0f;

  std::vector<lumice::gui::OverlayLabel> labels;
  lumice::gui::ComputeOverlayLabels(in, 0.0f, 0.0f, 200.0f, 200.0f, labels);

  // Mirrors kGroupGrid in overlay_labels.cpp:346 — kept private to the
  // test because the constant is in an anonymous namespace there. Source 4
  // emits altitude labels with this group at overlay_labels.cpp:730.
  // Byte-literal "\xC2\xB0" (UTF-8 for °) matches AddLabel's
  // "%.0f\xC2\xB0" format byte-for-byte without depending on
  // source-charset handling.
  constexpr int kGridGroup = 0;
  auto has_label = [&labels, kGridGroup](const char* text) {
    for (const auto& l : labels) {
      if (l.group == kGridGroup && l.text == text) {
        return true;
      }
    }
    return false;
  };

  // Sanity: generation path is alive.
  EXPECT_GT(CountGridLabels(labels), 0);

  // Positive control — ±70° passes dz cull (max wx = cos(70°) > 0.25).
  // Without these the negative assertion below could silently pass by
  // having the threshold raised to 1.0 (cull everything → 80° also absent).
  EXPECT_TRUE(has_label("70\xC2\xB0"));
  EXPECT_TRUE(has_label("-70\xC2\xB0"));

  // The dz cull's only observable consequence at this config: ±80° MUST
  // be culled (max wx = cos(80°) < 0.25, fails for all azimuths).
  EXPECT_TRUE(!has_label("80\xC2\xB0"));
  EXPECT_TRUE(!has_label("-80\xC2\xB0"));
}

// Test G: ComputeOverlayLabels output translates linearly with vp_screen origin.
// Contract: for any (vp_x, vp_y) offset, every emitted OverlayLabel.screen_x/y
// must shift by exactly that amount; label count and ordering stay identical.
// Regression for gui-polish-v16: caller in RenderPreviewPanel forgot to convert
// (panel_x, kTopBarHeight) through MainVpPos() into OS screen coords, which made
// overlay labels stick to the desktop origin instead of the host window when the
// window was dragged or sat on a non-primary monitor.
TEST(OverlayLabels, screen_origin_translation_invariance) {
  auto in = MakeGridOnly(/*visible=*/2, /*lens=Fisheye Equidistant*/ 2, /*elev*/ 0.0f, /*az*/ 0.0f);

  constexpr float kVpW = 800.0f;
  constexpr float kVpH = 400.0f;
  constexpr float kDx = 123.0f;
  constexpr float kDy = 45.0f;
  constexpr float kEps = 1e-3f;

  std::vector<lumice::gui::OverlayLabel> labels_origin;
  std::vector<lumice::gui::OverlayLabel> labels_shifted;
  lumice::gui::ComputeOverlayLabels(in, 0.0f, 0.0f, kVpW, kVpH, labels_origin);
  lumice::gui::ComputeOverlayLabels(in, kDx, kDy, kVpW, kVpH, labels_shifted);

  EXPECT_GT(static_cast<int>(labels_origin.size()), 0);
  EXPECT_EQ(labels_shifted.size(), labels_origin.size());

  for (size_t i = 0; i < labels_origin.size(); ++i) {
    const auto& a = labels_origin[i];
    const auto& b = labels_shifted[i];
    EXPECT_EQ(a.text, b.text);
    EXPECT_EQ(a.group, b.group);
    EXPECT_LT(std::abs((b.screen_x - a.screen_x) - kDx), kEps);
    EXPECT_LT(std::abs((b.screen_y - a.screen_y) - kDy), kEps);
  }
}

// Single orthographic at fov=180° has the analytic property that
// r_norm = sin(θ), i.e. the disc edge (r_norm=1) maps to θ=90°.
// Center pixel (px=py=0) → θ=0 → fisheyeInverse returns view dir (0, 0, -1)
// (camera-space "looking forward"). With elev=0,az=0,roll=0 the view
// matrix maps view (0, 0, -1) to world (-1, 0, 0): world forward.
TEST(OverlayLabels, pixel_to_world_dir_single_ortho_center) {
  float view[9];
  lumice::gui::BuildViewMatrix(/*elev*/ 0.0f, /*az*/ 0.0f, /*roll*/ 0.0f, view);

  float wx = 0;
  float wy = 0;
  float wz = 0;
  bool valid = false;
  lumice::gui::detail::PixelToWorldDirForTesting(0.0f, 0.0f, 200.0f, 200.0f, lumice::gui::kLensTypeFisheyeOrthographic,
                                                 /*fov*/ 180.0f, view, &wx, &wy, &wz, &valid);

  EXPECT_TRUE(valid);
  // World forward at (elev=0, az=0) is -x.
  EXPECT_LT(std::abs(wx - (-1.0f)), 1e-4f);
  EXPECT_LT(std::abs(wy), 1e-4f);
  EXPECT_LT(std::abs(wz), 1e-4f);
}

// Top-edge midpoint at fov=180°: r_norm=1 → θ=π/2. With phi=π/2 (py>0),
// view dir = (0, 1, 0) which the (elev=0,az=0,roll=0) view matrix maps to
// world dir (0, 0, -1). altitude = asin(-wz) = asin(1) = 90° — the zenith.
// This is the load-bearing assertion for "上半 = 正纬度": the highest
// possible altitude reachable in single orthographic must surface at the
// top of the viewport. Pre-fix RectangularInv mapped (0, hh) to lat=-π/2
// (negative), the precise inverse of this expectation.
TEST(OverlayLabels, pixel_to_world_dir_single_ortho_top_zenith) {
  float view[9];
  lumice::gui::BuildViewMatrix(0.0f, 0.0f, 0.0f, view);

  float wx = 0;
  float wy = 0;
  float wz = 0;
  bool valid = false;
  // py = +100 = +hh on a 200x200 viewport (shader convention: +py is image up).
  lumice::gui::detail::PixelToWorldDirForTesting(0.0f, 100.0f, 200.0f, 200.0f,
                                                 lumice::gui::kLensTypeFisheyeOrthographic,
                                                 /*fov*/ 180.0f, view, &wx, &wy, &wz, &valid);

  EXPECT_TRUE(valid);
  EXPECT_LT(std::abs(wx), 1e-3f);
  EXPECT_LT(std::abs(wy), 1e-3f);
  // -wz = sin(altitude); altitude=+90° → -wz = +1 → wz = -1.
  EXPECT_LT(std::abs(wz - (-1.0f)), 1e-3f);
}

// Dual orthographic at the upper-hemisphere disc center
// (px=-circle_radius, py=0): use_r=0, theta=0, world dir = (0, 0, -1)
// (zenith). Same load-bearing assertion as the single ortho top-zenith
// test, but exercises DualFisheyeInv(type=3) which has no other coverage
// (ComputeOverlayLabels' edge sampling can't reach lens=9 — see notes
// in OverlayLabels.single_orthographic_dispatch_matches_fisheye above).
TEST(OverlayLabels, pixel_to_world_dir_dual_ortho_left_disc_center) {
  // view_matrix is unused for lens=9 (no view transform). Use identity-ish
  // value so any accidental application would surface in the assertions.
  float view[9];
  lumice::gui::BuildViewMatrix(0.0f, 0.0f, 0.0f, view);

  // Viewport 200x200 → short_res = min(100, 200) = 100, circle_radius = 50.
  // Left circle center is at (-circle_radius, 0) = (-50, 0) in pixel coords.
  float wx = 0;
  float wy = 0;
  float wz = 0;
  bool valid = false;
  lumice::gui::detail::PixelToWorldDirForTesting(-50.0f, 0.0f, 200.0f, 200.0f,
                                                 lumice::gui::kLensTypeDualFisheyeOrthographic,
                                                 /*fov*/ 170.0f, view, &wx, &wy, &wz, &valid);

  EXPECT_TRUE(valid);
  EXPECT_LT(std::abs(wx), 1e-3f);
  EXPECT_LT(std::abs(wy), 1e-3f);
  // Left disc center → upper-hemisphere zenith → wz = -1.
  EXPECT_LT(std::abs(wz - (-1.0f)), 1e-3f);
}

// Dual orthographic right disc center → lower-hemisphere zenith (z = +1).
// Symmetric pair to the left-disc test; failing this would indicate the
// in_left/in_right branch in DualFisheyeInv is mis-attributing samples.
TEST(OverlayLabels, pixel_to_world_dir_dual_ortho_right_disc_center) {
  float view[9];
  lumice::gui::BuildViewMatrix(0.0f, 0.0f, 0.0f, view);

  float wx = 0;
  float wy = 0;
  float wz = 0;
  bool valid = false;
  lumice::gui::detail::PixelToWorldDirForTesting(50.0f, 0.0f, 200.0f, 200.0f,
                                                 lumice::gui::kLensTypeDualFisheyeOrthographic,
                                                 /*fov*/ 170.0f, view, &wx, &wy, &wz, &valid);

  EXPECT_TRUE(valid);
  EXPECT_LT(std::abs(wx), 1e-3f);
  EXPECT_LT(std::abs(wy), 1e-3f);
  // Right disc center → lower-hemisphere zenith → wz = +1.
  EXPECT_LT(std::abs(wz - 1.0f), 1e-3f);
}

// Rectangular: a top pixel (py=+30 on a 200×200 viewport, y-up) must map to an
// UP direction (positive altitude ⇒ -wz > 0 ⇒ wz < 0). Pre-fix RectangularInv
// gave lat<0 here (down), the precise inverse of this expectation.
TEST(OverlayLabels, pixel_to_world_dir_rectangular_top_is_up) {
  float view[9];
  lumice::gui::BuildViewMatrix(0.0f, 0.0f, 0.0f, view);  // unused for rectangular
  float wx = 0, wy = 0, wz = 0;
  bool valid = false;
  lumice::gui::detail::PixelToWorldDirForTesting(0.0f, 30.0f, 200.0f, 200.0f, lumice::gui::kLensTypeRectangular,
                                                 /*fov*/ 360.0f, view, &wx, &wy, &wz, &valid);
  EXPECT_TRUE(valid);
  // altitude = asin(-wz) > 0 for an up direction ⇒ wz < 0.
  EXPECT_LT(wz, -0.1f);
}

// Dual-fisheye (equal area): in the upper (left) circle, a point ABOVE the disc
// center (px=-circle_radius=-50, py=+30) must map to a direction tilted toward
// +x (wx>0) and still up (wz<0). Pre-fix (no py flip) this gave wx<0 — the
// vertical mirror owner reported. Pins the corrected in-disc vertical orientation.
TEST(OverlayLabels, pixel_to_world_dir_dual_fisheye_upper_above_center) {
  float view[9];
  lumice::gui::BuildViewMatrix(0.0f, 0.0f, 0.0f, view);  // unused for dual fisheye
  float wx = 0, wy = 0, wz = 0;
  bool valid = false;
  // 200×200 ⇒ short_res=min(100,200)=100 ⇒ circle_radius=50; left disc center px=-50.
  lumice::gui::detail::PixelToWorldDirForTesting(-50.0f, 30.0f, 200.0f, 200.0f,
                                                 lumice::gui::kLensTypeDualFisheyeEqualArea,
                                                 /*fov*/ 180.0f, view, &wx, &wy, &wz, &valid);
  EXPECT_TRUE(valid);
  EXPECT_GT(wx, 0.1f);  // tilts toward +x (was <0 pre-fix — the reported flip)
  EXPECT_LT(wz, 0.0f);  // upper hemisphere (up)
}

// Dual fisheye, equidistant and stereographic: the two types of the family with no other
// coverage anywhere — equal-area is reached by the lens_proj pixel references and
// orthographic by the two disc-center tests above, but nothing before these two cases ran
// DualFisheyeInv's type==1 / type==2 branches at all.
//
// A disc-center probe (use_r=0) cannot see them: theta comes out 0 for every type. So both
// cases probe at HALF the disc radius, where each type's radial law gives a different, exactly
// derivable polar angle — which is what turns a changed formula red rather than letting it
// agree with itself:
//   equidistant:   theta = use_r * 90°       = 45°       -> (sin, cos) = (0.70711, 0.70711)
//   stereographic: theta = 2*atan(use_r * 1) = 53.1301°  -> (sin, cos) = (0.8, 0.6)
// Both viewports are 200x200 ⇒ short_res = min(100, 200) = 100, circle_radius = 50, so the
// left (upper-hemisphere) disc is centred at px=-50 and the probe sits at px=-25, py=0.
// py=0 keeps the probe on the disc's horizontal axis, where phi=0 and the recovered direction
// is (0, sin theta, -cos theta): no x tilt, +y side, upper hemisphere.
TEST(OverlayLabels, pixel_to_world_dir_dual_equidist_half_radius) {
  float view[9];
  lumice::gui::BuildViewMatrix(0.0f, 0.0f, 0.0f, view);  // unused for dual fisheye
  float wx = 0, wy = 0, wz = 0;
  bool valid = false;
  lumice::gui::detail::PixelToWorldDirForTesting(-25.0f, 0.0f, 200.0f, 200.0f,
                                                 lumice::gui::kLensTypeDualFisheyeEquidist,
                                                 /*fov*/ 180.0f, view, &wx, &wy, &wz, &valid);
  EXPECT_TRUE(valid);
  EXPECT_NEAR(wx, 0.0f, 1e-3f);
  EXPECT_NEAR(wy, 0.70711f, 1e-3f);   // sin(45°)
  EXPECT_NEAR(wz, -0.70711f, 1e-3f);  // -cos(45°), upper hemisphere
}

TEST(OverlayLabels, pixel_to_world_dir_dual_stereographic_half_radius) {
  float view[9];
  lumice::gui::BuildViewMatrix(0.0f, 0.0f, 0.0f, view);  // unused for dual fisheye
  float wx = 0, wy = 0, wz = 0;
  bool valid = false;
  lumice::gui::detail::PixelToWorldDirForTesting(-25.0f, 0.0f, 200.0f, 200.0f,
                                                 lumice::gui::kLensTypeDualFisheyeStereographic,
                                                 /*fov*/ 180.0f, view, &wx, &wy, &wz, &valid);
  EXPECT_TRUE(valid);
  EXPECT_NEAR(wx, 0.0f, 1e-3f);
  EXPECT_NEAR(wy, 0.8f, 1e-3f);   // sin(2*atan(0.5))
  EXPECT_NEAR(wz, -0.6f, 1e-3f);  // -cos(2*atan(0.5)), upper hemisphere
}

// detail::ClampLabelPosToViewport contract — pure function tests for the
// viewport-inset behaviour.
// Pin all four edges and the "no-op when label fits centred" path.
TEST(OverlayLabels, clamp_label_pos_left_edge) {
  // Viewport: (10, 20, 200, 100). Label glyph 30×14 placed at pos.x=5
  // (i.e. would extend from x=5 to x=35 — overlaps left edge x=10).
  // Expected: clamped to vp_x + 2 = 12.
  ImVec2 clamped = lumice::gui::detail::ClampLabelPosToViewport(ImVec2(5.0f, 50.0f), ImVec2(30.0f, 14.0f), 10.0f, 20.0f,
                                                                200.0f, 100.0f);
  EXPECT_EQ(clamped.x, 12.0f);
  EXPECT_EQ(clamped.y, 50.0f);  // y stayed put (50 inside [20+2, 20+100-14-2] = [22, 104])
}

TEST(OverlayLabels, clamp_label_pos_right_edge) {
  // Label at pos.x=195 with text width 30 would extend to x=225, past
  // vp.x + vp.w = 210. Clamp to 210 - 30 - 2 = 178.
  ImVec2 clamped = lumice::gui::detail::ClampLabelPosToViewport(ImVec2(195.0f, 50.0f), ImVec2(30.0f, 14.0f), 10.0f,
                                                                20.0f, 200.0f, 100.0f);
  EXPECT_EQ(clamped.x, 178.0f);
  EXPECT_EQ(clamped.y, 50.0f);
}

TEST(OverlayLabels, clamp_label_pos_top_edge) {
  // Label at pos.y=15 (above vp_y=20). Clamp to vp_y + 2 = 22.
  ImVec2 clamped = lumice::gui::detail::ClampLabelPosToViewport(ImVec2(50.0f, 15.0f), ImVec2(30.0f, 14.0f), 10.0f,
                                                                20.0f, 200.0f, 100.0f);
  EXPECT_EQ(clamped.x, 50.0f);
  EXPECT_EQ(clamped.y, 22.0f);
}

TEST(OverlayLabels, clamp_label_pos_bottom_edge) {
  // Label at pos.y=110. Bottom edge: vp_y + vp_h - text_h - 2 = 20 + 100 - 14 - 2 = 104.
  ImVec2 clamped = lumice::gui::detail::ClampLabelPosToViewport(ImVec2(50.0f, 110.0f), ImVec2(30.0f, 14.0f), 10.0f,
                                                                20.0f, 200.0f, 100.0f);
  EXPECT_EQ(clamped.x, 50.0f);
  EXPECT_EQ(clamped.y, 104.0f);
}

TEST(OverlayLabels, clamp_label_pos_no_clamp_when_inside) {
  // Label fully inside viewport — pos returned unchanged.
  ImVec2 clamped = lumice::gui::detail::ClampLabelPosToViewport(ImVec2(50.0f, 50.0f), ImVec2(30.0f, 14.0f), 10.0f,
                                                                20.0f, 200.0f, 100.0f);
  EXPECT_EQ(clamped.x, 50.0f);
  EXPECT_EQ(clamped.y, 50.0f);
}

TEST(OverlayLabels, clamp_label_pos_viewport_too_narrow) {
  // x fallback: viewport width 20 cannot fit text width 30 + 2×2 inset
  // (`vp_w > text_size.x + 2×kViewportInsetPx` is false), so x is left
  // untouched to preserve the legacy "centered on anchor" rendering.
  // y is processed normally (vp_h=100 ≫ text_h+4=18) but happens to need
  // no clamp here (pos.y=50 already inside [22, 104]).
  ImVec2 clamped = lumice::gui::detail::ClampLabelPosToViewport(ImVec2(5.0f, 50.0f), ImVec2(30.0f, 14.0f), 10.0f, 20.0f,
                                                                20.0f, 100.0f);
  EXPECT_EQ(clamped.x, 5.0f);  // unchanged
  EXPECT_EQ(clamped.y, 50.0f);
}

// Hemisphere boundary inset — verify the inward-shifted boundary curve
// produces world directions on the visible side. Re-implement the equator
// / front-half lambdas at test level (same formulas as overlay_labels.cpp:540-)
// and assert the inset direction. This avoids the "label text source
// confusion" problem (latitude vs azimuth same %.0f° format) that ruled
// out a label-text-based regression test in the plan.
TEST(OverlayLabels, hemisphere_boundary_inset_upper_pushes_negative_z) {
  // Mirror the lambda in overlay_labels.cpp: equator inset for visible=upper
  // shifts wz by -sin(3°) before renormalisation, then renormalises.
  constexpr float kBoundaryInsetDeg = 3.0f;
  constexpr float kPi = 3.14159265f;
  constexpr float kDeg2Rad = kPi / 180.0f;
  float boundary_offset = std::sin(kBoundaryInsetDeg * kDeg2Rad);

  // For visible=upper, every sample on the offset equator must have wz < 0
  // (altitude = asin(-wz) > 0, i.e. inside upper hemisphere).
  // Sample 8 evenly-spaced t values along the boundary curve.
  for (int i = 0; i < 8; ++i) {
    float t = i * (2.0f * kPi / 8.0f);
    float az = -kPi + t;
    float wx = -std::cos(az);
    float wy = -std::sin(az);
    float wz = -1.0f * boundary_offset;
    float len = std::sqrt(wx * wx + wy * wy + wz * wz);
    wx /= len;
    wy /= len;
    wz /= len;
    EXPECT_LT(wz, 0.0f);  // upper-hemisphere visible side
    // Magnitude check: altitude should be ≈ 3° (asin(0.0523/len) ≈ 3°).
    float altitude_deg = std::asin(-wz) * 180.0f / kPi;
    EXPECT_GT(altitude_deg, 2.5f);
    EXPECT_LT(altitude_deg, 3.5f);
  }
}

TEST(OverlayLabels, hemisphere_boundary_inset_lower_pushes_positive_z) {
  // Symmetric pair of upper test — visible=lower shifts wz by +sin(3°).
  constexpr float kBoundaryInsetDeg = 3.0f;
  constexpr float kPi = 3.14159265f;
  constexpr float kDeg2Rad = kPi / 180.0f;
  float boundary_offset = std::sin(kBoundaryInsetDeg * kDeg2Rad);

  for (int i = 0; i < 8; ++i) {
    float t = i * (2.0f * kPi / 8.0f);
    float az = -kPi + t;
    float wx = -std::cos(az);
    float wy = -std::sin(az);
    float wz = +1.0f * boundary_offset;
    float len = std::sqrt(wx * wx + wy * wy + wz * wz);
    wx /= len;
    wy /= len;
    wz /= len;
    EXPECT_GT(wz, 0.0f);  // lower-hemisphere visible side
    float altitude_deg = std::asin(-wz) * 180.0f / kPi;
    EXPECT_LT(altitude_deg, -2.5f);
    EXPECT_GT(altitude_deg, -3.5f);
  }
}

// Smoke test for the visible=front boundary inset path. The front-half
// boundary in overlay_labels.cpp is a great circle perpendicular to forward
// (col2 = -forward), pushed by `-boundary_offset · col2` to land inside the
// visible front hemisphere. With identity-ish view matrix from
// BuildViewMatrix(elev=0, az=0, roll=0), col2 = (1, 0, 0) → forward = -x →
// pushing along -col2 means subtracting (boundary_offset, 0, 0). Replicate
// the lambda formula and assert dot(world, col2) < 0 (i.e. world has
// strictly more `-forward` content, i.e. lies on the visible front side).
TEST(OverlayLabels, hemisphere_boundary_inset_front_pushes_along_forward) {
  float view[9];
  lumice::gui::BuildViewMatrix(0.0f, 0.0f, 0.0f, view);

  constexpr float kBoundaryInsetDeg = 3.0f;
  constexpr float kPi = 3.14159265f;
  constexpr float kDeg2Rad = kPi / 180.0f;
  float boundary_offset = std::sin(kBoundaryInsetDeg * kDeg2Rad);

  for (int i = 0; i < 8; ++i) {
    float t_param = i * (2.0f * kPi / 8.0f);
    float c = std::cos(t_param);
    float s = std::sin(t_param);
    float wx = c * view[0] + s * view[3] - boundary_offset * view[6];
    float wy = c * view[1] + s * view[4] - boundary_offset * view[7];
    float wz = c * view[2] + s * view[5] - boundary_offset * view[8];
    float len = std::sqrt(wx * wx + wy * wy + wz * wz);
    wx /= len;
    wy /= len;
    wz /= len;
    // dot(world, col2) must be < 0 so the sample sits on the visible
    // front side (shader cull: dot(world, col2) > 0 → invisible).
    float dot_col2 = wx * view[6] + wy * view[7] + wz * view[8];
    EXPECT_LT(dot_col2, 0.0f);
  }
}

// Single orthographic must emit sun-circle interior labels for circles that
// don't intersect the viewport edge — same coverage as single fisheye.
// Pre-fix, the interior-label block in ComputeOverlayLabels was gated to
// `lens_type 0-3`, leaving lens=8 with no interior labels and a silent UX
// regression found in manual verification.
// Post-fix uses `!LensIsFullSky(...)` and lens=8 falls through to the same
// path as lens=2.
TEST(OverlayLabels, single_orthographic_sun_circle_interior_labels) {
  // Sun directly forward (camera-forward = -x at elev=0,az=0). A small
  // 5° angular-distance circle centred on the sun stays well inside the
  // orthographic disc, so all 4 interior labels at 0/90/180/270° around
  // the circle should land within viewport bounds and survive cull.
  // Note: existing MakeSunOnly callers use `const float sun_back[3]` /
  // `const float angle` and so trigger the same clang-tidy
  // "Invalid case style for constant" warnings — kept here without
  // `const` to avoid spreading the violation into new code (see
  // learnings/code-quality.md "项目 clang-tidy ... 函数内 const 局部").
  float sun_forward[3] = { -1.0f, 0.0f, 0.0f };
  float angle = 5.0f;
  auto in_ortho = MakeSunOnly(/*visible=Full*/ 2, sun_forward, &angle, 1);
  in_ortho.lens_type = lumice::gui::kLensTypeFisheyeOrthographic;
  in_ortho.fov = 60.0f;

  std::vector<lumice::gui::OverlayLabel> labels;
  lumice::gui::ComputeOverlayLabels(in_ortho, /*vp_x*/ 0.0f, /*vp_y*/ 0.0f,
                                    /*vp_w*/ 200.0f, /*vp_h*/ 200.0f, labels);

  // Expect ≥ 1 sun-circle label (the 4 placement points are evenly spaced;
  // viewport-bounds + collision-avoidance may drop some, but at least one
  // should survive for a small circle near the optical axis).
  EXPECT_GT(CountSunCircleLabels(labels), 0);

  // Cross-check: the same configuration with lens=Fisheye Equidistant
  // (lens=2) should produce the same label count, since the two single-
  // fisheye-class projections share the interior-label code path.
  auto in_fisheye = in_ortho;
  in_fisheye.lens_type = lumice::gui::kLensTypeFisheyeEquidist;
  std::vector<lumice::gui::OverlayLabel> labels_fisheye;
  lumice::gui::ComputeOverlayLabels(in_fisheye, 0.0f, 0.0f, 200.0f, 200.0f, labels_fisheye);
  EXPECT_EQ(CountSunCircleLabels(labels), CountSunCircleLabels(labels_fisheye));
}

// T1: orthographic + fov=180 + visible=Full → ≥ 5 distinct latitude labels.
TEST(OverlayLabels, ortho_fov180_full_visible_emits_latitude_labels) {
  auto in = MakeGridOnly(lumice::gui::kVisibleFull, lumice::gui::kLensTypeFisheyeOrthographic,
                         /*elev*/ 0.0f, /*az*/ 0.0f);
  in.fov = 180.0f;
  std::vector<lumice::gui::OverlayLabel> labels;
  lumice::gui::ComputeOverlayLabels(in, 0.0f, 0.0f, 200.0f, 200.0f, labels);
  EXPECT_GE(CountUniqueGridLabels(labels), 5);
}

// T2: orthographic + fov=180 + visible=Upper → ≥ 5 distinct latitude labels.
// Hemisphere-equator boundary alone covers altitude=0 only; source 4 fills
// the interior latitudes.
TEST(OverlayLabels, ortho_fov180_upper_visible_emits_latitude_labels) {
  auto in = MakeGridOnly(lumice::gui::kVisibleUpper, lumice::gui::kLensTypeFisheyeOrthographic,
                         /*elev*/ 0.0f, /*az*/ 0.0f);
  in.fov = 180.0f;
  std::vector<lumice::gui::OverlayLabel> labels;
  lumice::gui::ComputeOverlayLabels(in, 0.0f, 0.0f, 200.0f, 200.0f, labels);
  EXPECT_GE(CountUniqueGridLabels(labels), 5);
}

// T3: equidistant fov=180 + visible=Full → ≥ 5 distinct latitude labels.
// Verifies source 4 activates for all view-transformed (non-linear) lenses,
// not just orthographic.
TEST(OverlayLabels, equidist_fov180_full_visible_consistent) {
  auto in = MakeGridOnly(lumice::gui::kVisibleFull, lumice::gui::kLensTypeFisheyeEquidist,
                         /*elev*/ 0.0f, /*az*/ 0.0f);
  in.fov = 180.0f;
  std::vector<lumice::gui::OverlayLabel> labels;
  lumice::gui::ComputeOverlayLabels(in, 0.0f, 0.0f, 200.0f, 200.0f, labels);
  EXPECT_GE(CountUniqueGridLabels(labels), 5);
}

// Line/label toggle contract tests: BuildOverlayLabelInput reads
// GuiState::show_<x>_label fields (NOT show_<x>_line). The companion line
// routing (pp.overlay.show_<x> = g_state.show_<x>_line in RenderRightPanel)
// is a trivial inline assignment validated by code review + manual walk-through.
// These tests pin only the label-side wiring, which is the side that branches
// on the GuiState→OverlayLabelInput conversion function.
//
// Test A: line-only — label flag drives input, line flag is ignored by BuildOverlayLabelInput.
TEST(OverlayToggle, line_only_label_input_false) {
  lumice::gui::GuiState s;
  s.show_horizon_line = true;
  s.show_horizon_label = false;
  s.show_grid_line = true;
  s.show_grid_label = false;
  s.show_sun_circles_line = true;
  s.show_sun_circles_label = false;
  auto in = lumice::gui::BuildOverlayLabelInput(s, s.renderer);
  EXPECT_EQ(in.show_horizon, false);
  EXPECT_EQ(in.show_grid, false);
  EXPECT_EQ(in.show_sun_circles, false);
}

// Test B: label-only — only label flag drives input.
TEST(OverlayToggle, label_only_label_input_true) {
  lumice::gui::GuiState s;
  s.show_horizon_line = false;
  s.show_horizon_label = true;
  s.show_grid_line = false;
  s.show_grid_label = true;
  s.show_sun_circles_line = false;
  s.show_sun_circles_label = true;
  auto in = lumice::gui::BuildOverlayLabelInput(s, s.renderer);
  EXPECT_EQ(in.show_horizon, true);
  EXPECT_EQ(in.show_grid, true);
  EXPECT_EQ(in.show_sun_circles, true);
}

// Test C: both on — input reflects label flags (true).
TEST(OverlayToggle, both_on_label_input_true) {
  lumice::gui::GuiState s;
  s.show_horizon_line = true;
  s.show_horizon_label = true;
  s.show_grid_line = true;
  s.show_grid_label = true;
  s.show_sun_circles_line = true;
  s.show_sun_circles_label = true;
  auto in = lumice::gui::BuildOverlayLabelInput(s, s.renderer);
  EXPECT_EQ(in.show_horizon, true);
  EXPECT_EQ(in.show_grid, true);
  EXPECT_EQ(in.show_sun_circles, true);
}

// Test D: both off — three-layer assertion to distinguish "gate intercept"
// from "computed-but-empty" path:
//   (1) precondition: GuiState label flags are all false
//   (2) expected: BuildOverlayLabelInput emits all-false
//   (3) alternative path absence: the panel-side gate triple-OR
//       (label_label_label) evaluates to false, proving label sampling is
//       gated out by source flags and not by downstream filtering.
TEST(OverlayToggle, both_off_three_layer_assertion) {
  lumice::gui::GuiState s;
  // (1) Precondition.
  s.show_horizon_line = false;
  s.show_horizon_label = false;
  s.show_grid_line = false;
  s.show_grid_label = false;
  s.show_sun_circles_line = false;
  s.show_sun_circles_label = false;
  EXPECT_EQ(s.show_horizon_label, false);
  EXPECT_EQ(s.show_grid_label, false);
  EXPECT_EQ(s.show_sun_circles_label, false);
  // (2) Expected: BuildOverlayLabelInput sees all-false.
  auto in = lumice::gui::BuildOverlayLabelInput(s, s.renderer);
  EXPECT_EQ(in.show_horizon, false);
  EXPECT_EQ(in.show_grid, false);
  EXPECT_EQ(in.show_sun_circles, false);
  // (3) Alternative-path absence (documentary): the panel-side gate
  //     (show_horizon_label || show_grid_label || show_sun_circles_label)
  //     evaluates to false. This is the same expression used in
  //     app_panels.cpp:674 / app.cpp:301 to skip label sampling entirely.
  //     Note: this assertion is structurally tautological — Layer 1 already
  //     fixed all three label fields to false, so `gate_open` is mathematically
  //     forced to false. It is kept as an executable comment that pins the
  //     gate expression's structure (e.g. catches a future refactor that
  //     replaces the OR with a bug like `&& show_grid_label`).
  const bool gate_open = s.show_horizon_label || s.show_grid_label || s.show_sun_circles_label;
  EXPECT_EQ(gate_open, false);
}

// horizon_label produces a "0°" edge label
// (latitude=0 crossing). Grid skips 0° to avoid clutter, so horizon owns this
// value. Use Fisheye Equidistant with fov=180 (full sky) so the equator hits
// the viewport edges symmetrically.
TEST(OverlayToggle, horizon_label_emits_zero_degree) {
  auto in = MakeGridOnly(lumice::gui::kVisibleFull, lumice::gui::kLensTypeFisheyeEquidist,
                         /*elev*/ 0.0f, /*az*/ 0.0f);
  in.fov = 180.0f;
  in.show_horizon = true;
  in.show_grid = false;
  in.show_sun_circles = false;
  std::vector<lumice::gui::OverlayLabel> labels;
  lumice::gui::ComputeOverlayLabels(in, 0.0f, 0.0f, 200.0f, 200.0f, labels);
  // Expect at least one "0°" label (latitude=0 crossing the viewport edge).
  int zero_deg_count = 0;
  for (const auto& l : labels) {
    if (l.text == "0\xC2\xB0") {
      ++zero_deg_count;
    }
  }
  EXPECT_GE(zero_deg_count, 1);
}

// horizon_label is independent from grid.
// The grid loop skips altitude=0° (horizon owns it), so when only grid is on
// the *latitude* 0° must not appear.
//
// Semantics note (task 288.6): Under curve-centric, longitude=0° (the prime
// meridian) emits a "0°" azimuth label from `process_longitude_curve(0)`.
// This is legitimate output. The grid skip-0 rule applies only to altitude,
// so the post-rewrite expectation is "at most 1 '0°' label (the longitude
// one), never 2+ (which would imply the altitude-0 skip is broken)".
TEST(OverlayToggle, grid_only_skips_zero_degree) {
  auto in = MakeGridOnly(lumice::gui::kVisibleFull, lumice::gui::kLensTypeFisheyeEquidist,
                         /*elev*/ 0.0f, /*az*/ 0.0f);
  in.fov = 180.0f;
  in.show_horizon = false;
  in.show_grid = true;
  in.show_sun_circles = false;
  std::vector<lumice::gui::OverlayLabel> labels;
  lumice::gui::ComputeOverlayLabels(in, 0.0f, 0.0f, 200.0f, 200.0f, labels);
  int zero_deg_count = 0;
  for (const auto& l : labels) {
    if (l.text == "0\xC2\xB0") {
      ++zero_deg_count;
    }
  }
  // Allow at most 1 "0°" label (the longitude=0° prime-meridian label);
  // the altitude=0° label MUST be skipped (horizon owns that text) — a
  // bug in the grid g==0 skip would push this to 2 or more.
  EXPECT_LE(zero_deg_count, 1);
}

// CC-1: globe lens emits longitude labels (was 0 pre-rewrite — only
// sample_interior_latitudes wrote globe labels, and it only walked
// altitude rings). Curve-centric process_longitude_curve walks each az
// value as a curve, producing boundary anchors on the globe's disc edge.
TEST(OverlayLabels, cc1_globe_has_longitude_labels) {
  auto in = MakeGridOnly(lumice::gui::kVisibleFull, lumice::gui::kLensTypeGlobe,
                         /*elev*/ 20.0f, /*az*/ 0.0f);
  in.fov = 60.0f;
  std::vector<lumice::gui::OverlayLabel> labels;
  lumice::gui::ComputeOverlayLabels(in, 0.0f, 0.0f, 512.0f, 512.0f, labels);
  // Pre-rewrite count was ~13 (latitude rings only). Post-rewrite must
  // include longitude labels — threshold ≥ 20 with margin to absorb
  // visibility variation across viewport sizes / elevations.
  EXPECT_GE(CountGridLabels(labels), 20);
}

// CC-2: rectangular lens emits longitude labels (was 0 pre-rewrite — the
// top/bottom viewport edges were polar-azimuth-singular, so the old
// edge-sampling Crosses guard `abs(az1-az0)<20` rejected every potential
// azimuth crossing). Curve-centric walks each az curve directly and emits
// at the first visible sample (the curve hits the viewport top/bottom).
TEST(OverlayLabels, cc2_rectangular_has_longitude_labels) {
  auto in = MakeGridOnly(lumice::gui::kVisibleFull, lumice::gui::kLensTypeRectangular,
                         /*elev*/ 20.0f, /*az*/ 0.0f);
  // Pre-rewrite rectangular had ~40 latitude labels (left/right edge
  // crossings) and 0 longitude. Post-rewrite must include both.
  std::vector<lumice::gui::OverlayLabel> labels;
  lumice::gui::ComputeOverlayLabels(in, 0.0f, 0.0f, 800.0f, 400.0f, labels);
  EXPECT_GE(CountGridLabels(labels), 30);
}

// CC-3: dual fisheye emits labels on BOTH discs (was 0 pre-rewrite — full
// sky lens skipped all interior/hemisphere samplers, and viewport edge
// sampling never crossed the two discs). Curve-centric requires the
// forward projector for dual fisheye (added Step 1) plus per-disc
// visibility detection.
TEST(OverlayLabels, cc3_dual_fisheye_has_labels_on_both_discs) {
  auto in = MakeGridOnly(lumice::gui::kVisibleFull, lumice::gui::kLensTypeDualFisheyeEqualArea,
                         /*elev*/ 20.0f, /*az*/ 0.0f);
  in.show_horizon = true;
  // 512×512 vp: short_res_dual = min(256, 512) = 256, circle_radius = 128.
  // Left disc center at px=-128, right at px=+128 (in pixel-offset space).
  // After viewport translation, screen_x mid = vp_x + vp_w/2 = 256, so
  // labels on the left disc have screen_x roughly in [0, 256) and right
  // in (256, 512]. Use a generous threshold to avoid flakiness at the
  // edge between discs.
  std::vector<lumice::gui::OverlayLabel> labels;
  lumice::gui::ComputeOverlayLabels(in, 0.0f, 0.0f, 512.0f, 512.0f, labels);
  EXPECT_GT(CountGridLabels(labels), 0);

  bool has_left = false, has_right = false;
  for (const auto& l : labels) {
    if (l.group != 0)
      continue;
    if (l.screen_x < 192.0f)
      has_left = true;
    if (l.screen_x > 320.0f)
      has_right = true;
  }
  EXPECT_TRUE(has_left);
  EXPECT_TRUE(has_right);
}

// CC-4: a single altitude curve emits at most a small number of labels
// (no edge-grazing clustering). Pre-rewrite, when an altitude curve grazed
// a viewport edge (e.g. fisheye fov=120, equator at elev=0), every adjacent
// pixel-pair on the top edge crossed the same alt value, producing a
// cluster of ~15 identical labels. Curve-centric emits at most 1 per
// visible arc, capping at a small handful per curve. Closed altitude=10°
// curve typically has 0–2 visible arcs in any single fisheye view.
TEST(OverlayLabels, cc4_no_label_clustering_per_altitude_line) {
  auto in = MakeGridOnly(lumice::gui::kVisibleFull, lumice::gui::kLensTypeFisheyeEquidist,
                         /*elev*/ 0.0f, /*az*/ 0.0f);
  in.fov = 120.0f;
  std::vector<lumice::gui::OverlayLabel> labels;
  lumice::gui::ComputeOverlayLabels(in, 0.0f, 0.0f, 512.0f, 512.0f, labels);

  // Count labels whose text exactly equals "10°" — these all come from
  // the alt=10° altitude curve (longitude curves at az=10° have a
  // different text in this view, but we still allow up to 2 in case
  // future label-text rules unify formats).
  int alt10_count = 0;
  for (const auto& l : labels) {
    if (l.text == "10\xC2\xB0") {
      ++alt10_count;
    }
  }
  // Pre-rewrite this could reach ~15 from edge-grazing clusters; allow
  // up to 4 (latitude=10° curve + longitude=10° curve, each potentially
  // emitting 1–2 anchors).
  EXPECT_LE(alt10_count, 4);
}
