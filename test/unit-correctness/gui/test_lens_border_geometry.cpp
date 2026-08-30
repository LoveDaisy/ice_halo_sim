// Where the lens-border circle goes, and which lenses have one at all.
//
// The border traces the locus at which the active projection's inverse stops being defined — the
// exact place its domain guard flips a pixel from "maps to a sky direction" to "black". So the
// claim under test is not "the radius formula reproduces its own derivation" (a formula copied
// from a wrong coefficient satisfies that trivially); it is "the radius formula lands on the
// validity flip of a projection inverse this repo already ships and already relies on".
//
// That independent oracle is detail::PixelToWorldDirForTesting — the CPU mirror of the preview
// shader's inverse, in production use for the overlay labels, the zenith/nadir markers and mouse
// picking. Each case samples a handful of azimuths, steps a couple of pixels inside and outside
// the predicted radius, and asserts valid==true inside and valid==false outside. A wrong
// coefficient moves the predicted radius off the real flip and one of the two sides goes red.
//
// The formula below is a deliberate second copy of the shader's, not a shared helper: the shader
// is the only production consumer, and a helper both sides called would make this file assert
// against itself. Keeping the copy here is what buys the cross-check — at the price of having to
// edit it whenever overlayLensBorder() changes, which the comment on it says out loud.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <string>
#include <vector>

#include "gui/gui_constants.hpp"
#include "gui/gui_state.hpp"
#include "gui/overlay_labels.hpp"
#include "gui/preview_renderer.hpp"

namespace gui = lumice::gui;

namespace {

constexpr float kPi = 3.14159265358979323846f;

// One border circle in center-origin pixel coordinates.
struct BorderCircle {
  float cx = 0.0f;
  float cy = 0.0f;
  float r = 0.0f;
};

// Synced with overlayLensBorder() in src/gui/preview_renderer.cpp — every coefficient here has a
// twin there and the two must be edited together. Returns the circles the border draws for this
// lens (empty for a lens that has none).
std::vector<BorderCircle> BorderCirclesForTesting(int lens_type, float fov_deg, float res_x, float res_y) {
  const float half_fov = fov_deg * 0.5f * kPi / 180.0f;
  constexpr float kMinSin = 1e-4f;

  if ((lens_type >= gui::kLensTypeDualFisheyeEqualArea && lens_type <= gui::kLensTypeDualFisheyeStereographic) ||
      lens_type == gui::kLensTypeDualFisheyeOrthographic) {
    const float circle_radius = std::min(res_x * 0.5f, res_y) * 0.5f;
    return { { -circle_radius, 0.0f, circle_radius }, { circle_radius, 0.0f, circle_radius } };
  }

  float r_boundary = 0.0f;
  if (lens_type == gui::kLensTypeFisheyeEqualArea) {
    const float sn = std::sin(half_fov * 0.5f);
    if (sn < kMinSin) {
      return {};
    }
    r_boundary = 1.0f / sn;
  } else if (lens_type == gui::kLensTypeFisheyeEquidist) {
    if (half_fov < kMinSin) {
      return {};
    }
    r_boundary = kPi / half_fov;
  } else if (lens_type == gui::kLensTypeFisheyeOrthographic) {
    const float sn = std::sin(half_fov);
    if (sn < kMinSin) {
      return {};
    }
    r_boundary = 1.0f / sn;
  } else {
    return {};
  }

  const float img_radius = std::min(res_x, res_y) * 0.5f;
  return { { 0.0f, 0.0f, img_radius * r_boundary } };
}

// The identity orientation. The domain guards being probed all fire before the view matrix is
// applied, so the orientation cannot move the flip — but a matrix still has to be supplied, and an
// identity one keeps the world directions readable if a case ever needs to print them.
struct IdentityView {
  float m[9] = {};
  IdentityView() { gui::BuildViewMatrix(0.0f, 0.0f, 0.0f, m); }
};

bool PixelIsInsideDomain(float px, float py, float res_x, float res_y, int lens_type, float fov_deg,
                         const float view[9]) {
  float wx = 0.0f;
  float wy = 0.0f;
  float wz = 0.0f;
  bool valid = false;
  gui::detail::PixelToWorldDirForTesting(px, py, res_x, res_y, lens_type, fov_deg, view, &wx, &wy, &wz, &valid);
  return valid;
}

// Walk a circle and assert the domain flips across it. `probe_px` is how far inside / outside the
// predicted radius the two samples sit: big enough to clear float noise at the flip, small enough
// that no other boundary can slip between the sample and the circle.
//
// `all_circles` is the whole border set for this lens, not just the one being walked, because the
// dual-fisheye pair is TANGENT at the origin: the sample placed just outside the left circle at
// azimuth 0 lands inside the right one, where the inverse is legitimately defined. The outside
// assertion is therefore made against the UNION of the circles — outside this one and outside every
// other one — which is the actual claim ("beyond the border the projection has nothing to show"),
// and skipping those samples silently would instead have dropped the assertion where the two
// circles meet.
void ExpectDomainFlipsAcross(const BorderCircle& circle, const std::vector<BorderCircle>& all_circles, int lens_type,
                             float fov_deg, float res_x, float res_y, float probe_px, const std::string& what) {
  const IdentityView view;
  constexpr int kAzimuthSamples = 8;
  int outside_samples_asserted = 0;
  for (int i = 0; i < kAzimuthSamples; ++i) {
    const float phi = 2.0f * kPi * static_cast<float>(i) / static_cast<float>(kAzimuthSamples);
    const float ux = std::cos(phi);
    const float uy = std::sin(phi);

    const float in_x = circle.cx + ux * (circle.r - probe_px);
    const float in_y = circle.cy + uy * (circle.r - probe_px);
    const float out_x = circle.cx + ux * (circle.r + probe_px);
    const float out_y = circle.cy + uy * (circle.r + probe_px);

    // Non-fatal: a fatal assert inside this loop would hide every azimuth after the first failure,
    // and which azimuths fail is the diagnostic (a wrong radius fails all of them, an off-center
    // circle only some).
    EXPECT_TRUE(PixelIsInsideDomain(in_x, in_y, res_x, res_y, lens_type, fov_deg, view.m))
        << what << ": azimuth " << i << " — the point " << probe_px << " px INSIDE the predicted border radius ("
        << circle.r << " px) is outside the projection domain, so the border is drawn too far out";

    const bool out_covered_elsewhere = std::any_of(all_circles.begin(), all_circles.end(), [&](const BorderCircle& c) {
      const float dx = out_x - c.cx;
      const float dy = out_y - c.cy;
      return std::sqrt(dx * dx + dy * dy) <= c.r;
    });
    if (out_covered_elsewhere) {
      continue;  // tangency: this sample is inside a sibling circle, so it says nothing here
    }
    ++outside_samples_asserted;
    EXPECT_FALSE(PixelIsInsideDomain(out_x, out_y, res_x, res_y, lens_type, fov_deg, view.m))
        << what << ": azimuth " << i << " — the point " << probe_px << " px OUTSIDE the predicted border radius ("
        << circle.r << " px) is still inside the projection domain, so the border is drawn too far in";
  }
  // A tangency that swallowed every outside sample would leave the "too far in" half of the claim
  // untested while the case still reported green. At most one of eight samples can be swallowed by
  // the single tangent point, so this bound is loose on purpose.
  EXPECT_GE(outside_samples_asserted, kAzimuthSamples - 1) << what << ": too few outside samples survived";
}

// ==================================================================================================
// AC1 — the border sits on the projection's domain boundary
//
// One case per shader branch that can produce a border: the three single-lens asin/theta guards and
// the dual-fisheye circle clip. The FOVs are the ones issue-side arithmetic names, chosen so the
// circle lands well inside the viewport with room for both probes.
// ==================================================================================================

constexpr float kResX = 800.0f;
constexpr float kResY = 800.0f;
constexpr float kProbePx = 2.0f;

TEST(LensBorderGeometry, equal_area_border_is_the_asin_domain_edge) {
  // fov=120 → r_boundary = 1 / sin(30°) = 2.0 image radii = 800 px on this viewport. A square
  // viewport keeps the whole circle addressable; the probes do not need it to be on screen, only
  // for the inverse to be defined there, which is a pure function of the pixel offset.
  const auto circles = BorderCirclesForTesting(gui::kLensTypeFisheyeEqualArea, 120.0f, kResX, kResY);
  ASSERT_EQ(circles.size(), 1u);
  ExpectDomainFlipsAcross(circles[0], circles, gui::kLensTypeFisheyeEqualArea, 120.0f, kResX, kResY, kProbePx,
                          "fisheye equal-area fov=120");
}

TEST(LensBorderGeometry, equidistant_border_is_the_theta_pi_edge) {
  // fov=180 → r_boundary = π / (π/2) = 2.0 image radii. The guard is theta >= π rather than an
  // asin domain, so this case is what proves the second coefficient was not copied from the first.
  const auto circles = BorderCirclesForTesting(gui::kLensTypeFisheyeEquidist, 180.0f, kResX, kResY);
  ASSERT_EQ(circles.size(), 1u);
  ExpectDomainFlipsAcross(circles[0], circles, gui::kLensTypeFisheyeEquidist, 180.0f, kResX, kResY, kProbePx,
                          "fisheye equidistant fov=180");
}

TEST(LensBorderGeometry, orthographic_border_is_the_asin_domain_edge) {
  // fov=180 → r_boundary = 1 / sin(90°) = 1.0 image radius = 400 px: the circle inscribed in the
  // viewport. Note sin(half_fov) here vs sin(half_fov/2) for equal area — the halving is exactly
  // the kind of coefficient this case exists to hold in place.
  const auto circles = BorderCirclesForTesting(gui::kLensTypeFisheyeOrthographic, 180.0f, kResX, kResY);
  ASSERT_EQ(circles.size(), 1u);
  ExpectDomainFlipsAcross(circles[0], circles, gui::kLensTypeFisheyeOrthographic, 180.0f, kResX, kResY, kProbePx,
                          "fisheye orthographic fov=180");
}

TEST(LensBorderGeometry, dual_fisheye_border_is_the_hard_circle_clip) {
  // Two circles, and their radius does not read the FOV at all: dualFisheyeInverse clips on
  // min(w/2, h)/2 before it ever branches on the projection type. A 1600×800 viewport makes the
  // two circles exactly fill it, which is the layout the dual-fisheye export uses.
  constexpr float kDualResX = 1600.0f;
  const auto circles = BorderCirclesForTesting(gui::kLensTypeDualFisheyeEqualArea, 180.0f, kDualResX, kResY);
  ASSERT_EQ(circles.size(), 2u);
  ExpectDomainFlipsAcross(circles[0], circles, gui::kLensTypeDualFisheyeEqualArea, 180.0f, kDualResX, kResY, kProbePx,
                          "dual fisheye equal-area, left circle");
  ExpectDomainFlipsAcross(circles[1], circles, gui::kLensTypeDualFisheyeEqualArea, 180.0f, kDualResX, kResY, kProbePx,
                          "dual fisheye equal-area, right circle");
}

TEST(LensBorderGeometry, all_four_dual_variants_share_one_clip_circle) {
  // The reason all four dual variants carry a border while single-lens stereographic does not: the
  // clip is upstream of the per-type theta branch, so the type cannot move it. Asserting the four
  // agree is what makes that a mechanical fact rather than a reading of the shader.
  constexpr float kDualResX = 1600.0f;
  const int kVariants[] = { gui::kLensTypeDualFisheyeEqualArea, gui::kLensTypeDualFisheyeEquidist,
                            gui::kLensTypeDualFisheyeStereographic, gui::kLensTypeDualFisheyeOrthographic };
  const auto reference = BorderCirclesForTesting(kVariants[0], 180.0f, kDualResX, kResY);
  ASSERT_EQ(reference.size(), 2u);
  for (int lens : kVariants) {
    const auto circles = BorderCirclesForTesting(lens, 180.0f, kDualResX, kResY);
    // Non-fatal + skip: ASSERT_ here would return out of the whole case and leave the remaining
    // variants unchecked, and "which of the four" is what this case is asked.
    if (circles.size() != 2u) {
      ADD_FAILURE() << "lens " << lens << ": expected 2 clip circles, got " << circles.size();
      continue;
    }
    for (std::size_t i = 0; i < circles.size(); ++i) {
      EXPECT_FLOAT_EQ(circles[i].cx, reference[i].cx) << "lens " << lens << " circle " << i;
      EXPECT_FLOAT_EQ(circles[i].cy, reference[i].cy) << "lens " << lens << " circle " << i;
      EXPECT_FLOAT_EQ(circles[i].r, reference[i].r) << "lens " << lens << " circle " << i;
    }
    ExpectDomainFlipsAcross(circles[0], circles, lens, 180.0f, kDualResX, kResY, kProbePx,
                            "dual variant " + std::to_string(lens) + ", left circle");
  }
}

// ==================================================================================================
// AC2 — which lenses have a border
//
// LensHasBorder() is the single authority; the shader's branch list and this table are the two
// things that must agree with it. The classification is the owner's, with one derived part: the
// four dual variants are in because their black region comes from the clip circle asserted above,
// not from the projection formula that gets single-lens stereographic excluded.
// ==================================================================================================

TEST(LensBorderGeometry, the_lens_border_set_is_exactly_the_seven_fisheye_family_members) {
  struct Row {
    int lens_type;
    bool has_border;
    const char* why;
  };
  const Row kRows[] = {
    { gui::kLensTypeLinear, false, "the whole plane is in the domain" },
    { gui::kLensTypeFisheyeEqualArea, true, "asin(r*sin(fov/4)) domain guard" },
    { gui::kLensTypeFisheyeEquidist, true, "theta = r*half_fov reaches pi" },
    { gui::kLensTypeFisheyeStereographic, false, "theta never reaches 180, so it always fills the view" },
    { gui::kLensTypeDualFisheyeEqualArea, true, "hard circle clip in dualFisheyeInverse" },
    { gui::kLensTypeDualFisheyeEquidist, true, "hard circle clip in dualFisheyeInverse" },
    { gui::kLensTypeDualFisheyeStereographic, true, "hard circle clip — upstream of the type branch" },
    { gui::kLensTypeRectangular, false, "bounded by two straight lines, not a circle" },
    { gui::kLensTypeFisheyeOrthographic, true, "asin(r*sin(fov/2)) domain guard" },
    { gui::kLensTypeDualFisheyeOrthographic, true, "hard circle clip in dualFisheyeInverse" },
    { gui::kLensTypeGlobe, false, "the sphere outline is not a projection-domain edge" },
  };
  static_assert(std::size(kRows) == gui::kLensTypeCount,
                "every LensType must state whether it has a border — that is the point of this table");

  int with_border = 0;
  for (const auto& row : kRows) {
    EXPECT_EQ(gui::LensHasBorder(row.lens_type), row.has_border) << "lens " << row.lens_type << ": " << row.why;
    // The predicate and the geometry must not disagree: a lens LensHasBorder() calls out must have
    // circles to draw, and one it excludes must have none. Splitting them would let the UI offer a
    // border the shader never draws (or the reverse) with both halves passing their own test.
    const bool has_circles = !BorderCirclesForTesting(row.lens_type, 180.0f, kResX, kResY).empty();
    EXPECT_EQ(has_circles, row.has_border) << "lens " << row.lens_type << ": " << row.why;
    if (row.has_border) {
      ++with_border;
    }
  }
  EXPECT_EQ(with_border, gui::kLensBorderLensTypeCount);
}

TEST(LensBorderGeometry, a_border_radius_that_diverges_draws_nothing) {
  // Not a degenerate-input guard for its own sake: at fov=360 the orthographic boundary radius is
  // 1/sin(180°), i.e. unbounded, and equal area at fov→0 does the same. "No circle" is the right
  // answer there — the whole addressable plane is inside the domain — and the alternative is an
  // inf/NaN radius reaching smoothstep.
  EXPECT_TRUE(BorderCirclesForTesting(gui::kLensTypeFisheyeOrthographic, 360.0f, kResX, kResY).empty());
  EXPECT_TRUE(BorderCirclesForTesting(gui::kLensTypeFisheyeEqualArea, 0.0f, kResX, kResY).empty());
  EXPECT_TRUE(BorderCirclesForTesting(gui::kLensTypeFisheyeEquidist, 0.0f, kResX, kResY).empty());
}

}  // namespace
