// The on-axis per-pixel solid angle, checked against oracles that are not itself.
//
// `core/lens_proj_build.hpp::ComputeAxisSolidAngle` answers one question per lens type: how much
// sky does the pixel on the optical axis subtend? It is the factor that converts the exposure
// anchor (a radiance per steradian, `core/anchor_buffer.hpp`) into the units a rendered pixel is
// in (a radiance integrated over that pixel), so a wrong constant on any branch is a silently
// mis-exposed render for that lens and nothing else — no crash, no NaN, no failing pixel
// comparison anywhere, because every pixel moves by the same factor.
//
// Four of the eleven branches had a prior one-off derivation to check against; the rest did not,
// and three of them (the equidistant / stereographic / orthographic members of the dual-fisheye
// family) had never been derived anywhere. So this file does not compare the implementation to
// any earlier statement of the answer. It rebuilds the answer twice, from two different premises:
//
//   ORACLE A — the LIMIT. For a rotationally symmetric map rho(theta) the on-axis solid angle
//     per unit image area is the limit of (solid angle of the cone of half-angle theta) divided
//     by (area of the image disc that cone fills):
//
//         Omega_axis = lim   2*pi*(1 - cos theta) / (pi * rho(theta)^2)
//                    theta->0
//                    = lim   4 sin^2(theta/2) / rho(theta)^2
//
//     evaluated in double at a small theta, with a convergence check at theta/10 that says the
//     limit has actually been reached rather than assumed. The cone's solid angle is written in
//     closed form rather than as a Riemann sum on purpose: 2*pi*(1 - cos theta) IS the definition
//     of a cone's solid angle, so quadrature would add error without removing a shared premise —
//     what has to be kept out of the oracle is the SUBJECT's algebra (`numer / pix_per_rad^2`),
//     and it is. The half-angle form `4 sin^2(theta/2)` is the same quantity written so that no
//     digits are lost to `1 - cos` cancellation at the small theta the limit needs.
//
//     What Oracle A shares with the subject is rho(theta) — the definition of each projection,
//     restated below from the same relations the shader's *Inverse functions solve. That is the
//     deliberate shared premise, exactly as in test/unit-correctness/gui/test_preview_jacobian.cpp,
//     and it leaves one gap: a branch that used the WRONG MAPPING for its type would be confirmed
//     by an oracle that made the same substitution.
//
//   ORACLE B — the PRODUCTION FORWARD closes that gap. For each type it takes a direction at a
//     known angle off the optical axis, projects it with `lm_proj::ProjectExitToPixel` (the one
//     the renderer and both GPU kernels actually use), and checks the pixel that comes back
//     against the radius rho(theta) restated here. It says nothing about the on-axis limit; what
//     it says is that the mapping Oracle A differentiated is the mapping this lens type is
//     really rendered with, including the dual-fisheye disc layout and the overlap `r_scale`.
//
// Both are needed and neither is redundant: A alone would accept a correct derivative of the
// wrong lens, B alone would accept any on-axis constant whatsoever.

#include <algorithm>
#include <cmath>
#include <vector>

#include "config/render_config.hpp"
#include "core/geo3d.hpp"
#include "core/lens_proj_build.hpp"
#include "core/shared/projection_shared.h"
#include "gtest/gtest.h"

namespace {

using lumice::LensParam;
using lumice::RenderConfig;
using lumice::Rotation;

constexpr double kPi = 3.14159265358979323846;

// One test point: a lens type plus the calibration knobs that move its answer.
struct LensCase {
  const char* name;
  LensParam::LensType type;
  float fov_deg;
  int res_w;
  int res_h;
  float overlap;  // dual-fisheye only; 0 everywhere else
};

// Every LensType appears at least once. The dual-fisheye family appears twice each — once with
// no overlap ring and once with one — because `r_scale` is the parameter a signature taking only
// (type, fov, resolution) could not have seen, and a case list without it would not notice.
const LensCase kCases[] = {
  { "linear_60", LensParam::kLinear, 60.0f, 512, 512, 0.0f },
  { "linear_120_wide", LensParam::kLinear, 120.0f, 1024, 683, 0.0f },
  { "globe_90", LensParam::kGlobe, 90.0f, 512, 512, 0.0f },
  { "fisheye_equal_area_180", LensParam::kFisheyeEqualArea, 180.0f, 512, 512, 0.0f },
  { "fisheye_equal_area_120", LensParam::kFisheyeEqualArea, 120.0f, 512, 512, 0.0f },
  { "fisheye_equidistant_180", LensParam::kFisheyeEquidistant, 180.0f, 512, 512, 0.0f },
  { "fisheye_stereographic_180", LensParam::kFisheyeStereographic, 180.0f, 512, 512, 0.0f },
  { "fisheye_orthographic_140", LensParam::kFisheyeOrthographic, 140.0f, 512, 512, 0.0f },
  { "rectangular_360", LensParam::kRectangular, 360.0f, 1024, 512, 0.0f },
  { "dual_equal_area", LensParam::kDualFisheyeEqualArea, 180.0f, 1024, 512, 0.0f },
  { "dual_equal_area_overlap", LensParam::kDualFisheyeEqualArea, 180.0f, 1024, 512, 0.0872f },
  { "dual_equidistant", LensParam::kDualFisheyeEquidistant, 180.0f, 1024, 512, 0.0f },
  { "dual_equidistant_overlap", LensParam::kDualFisheyeEquidistant, 180.0f, 1024, 512, 0.0872f },
  { "dual_stereographic", LensParam::kDualFisheyeStereographic, 180.0f, 1024, 512, 0.0f },
  { "dual_stereographic_overlap", LensParam::kDualFisheyeStereographic, 180.0f, 1024, 512, 0.0872f },
  // No overlap twin: BuildProjParams leaves this member's r_scale at 1 whatever `overlap_` says
  // (ComputeORScale is deferred), so a second row would only re-run the first.
  { "dual_orthographic", LensParam::kDualFisheyeOrthographic, 180.0f, 1024, 512, 0.0f },
};

RenderConfig MakeConfig(const LensCase& c) {
  RenderConfig cfg;
  cfg.lens_.type_ = c.type;
  cfg.lens_.fov_ = c.fov_deg;
  cfg.resolution_[0] = c.res_w;
  cfg.resolution_[1] = c.res_h;
  cfg.overlap_ = c.overlap;
  return cfg;
}

lm_proj::ProjParams MakeParams(const LensCase& c) {
  const RenderConfig cfg = MakeConfig(c);
  const auto short_pix = static_cast<float>(std::min(cfg.resolution_[0], cfg.resolution_[1]));
  return lumice::BuildProjParams(cfg, Rotation{}, short_pix);
}

// The image radius in PIXELS for a direction at polar angle `theta` off the optical axis.
//
// Restated from each projection's defining relation, in double, and NOT from the subject: these
// are the same five laws test_preview_jacobian.cpp lists (rho = f tan t / f sqrt2 sin(t/2) /
// f t/(pi/2) / f tan(t/2) / f sin t), plus the globe's sphere perspective. `p.scale` and the
// dual family's disc radius carry the per-instance calibration, read from the same POD the
// renderer projects with, so nothing here re-derives a scale.
double RhoOfTheta(const lm_proj::ProjParams& p, double theta) {
  const auto type = static_cast<LensParam::LensType>(p.proj_type);
  const double s = p.scale;
  // Dual-fisheye discs: radius min(w/2, h)/2 with the hemisphere shrunk into it by r_scale.
  const double dual_r = static_cast<double>(std::min(p.img_w / 2, p.img_h)) / 2.0 * p.r_scale;
  switch (type) {
    case LensParam::kLinear:
      return s * std::tan(theta);
    case LensParam::kGlobe: {
      // Camera at distance D from the unit sphere: the surface point at angle theta from the
      // near pole images at focal * sin(theta) / (D - cos(theta)).
      const double d = lm_proj::kGlobeCameraD;
      return s * std::sin(theta) / (d - std::cos(theta));
    }
    case LensParam::kFisheyeEqualArea:
      return s * std::sqrt(2.0) * std::sin(theta / 2.0);
    case LensParam::kFisheyeEquidistant:
      return s * theta / (kPi / 2.0);
    case LensParam::kFisheyeStereographic:
      return s * std::tan(theta / 2.0);
    case LensParam::kFisheyeOrthographic:
      return s * std::sin(theta);
    case LensParam::kRectangular:
      // Equirectangular is not rotationally symmetric, but it is isotropic to leading order about
      // its own centre: lon and lat both carry the same pixels-per-radian, and the cos(lat) area
      // element is 1 + O(theta^2) there. The convergence check below is what says that residual is
      // not being cashed as the answer.
      return s * theta;
    case LensParam::kDualFisheyeEqualArea:
      return dual_r * std::sqrt(2.0) * std::sin(theta / 2.0);
    case LensParam::kDualFisheyeEquidistant:
      return dual_r * theta / (kPi / 2.0);
    case LensParam::kDualFisheyeStereographic:
      return dual_r * std::tan(theta / 2.0);
    case LensParam::kDualFisheyeOrthographic:
      return dual_r * std::sin(theta);
  }
  return 0.0;
}

// Oracle A at one theta: (cone solid angle) / (image disc area), the pi cancelling.
double LimitEstimate(const lm_proj::ProjParams& p, double theta) {
  const double rho = RhoOfTheta(p, theta);
  const double half = std::sin(theta / 2.0);
  return 4.0 * half * half / (rho * rho);
}

// A world direction at angle `theta` off the lens's optical axis, in the plane the projections
// map to their +/-x image axis. The optical axis differs between families because
// ProjectExitToPixel feeds them different vectors: the single-lens and rectangular branches keep
// camera-frame c = -w with c.z > 0, the dual-fisheye branch keeps s = -w and splits on s.z, and
// globe's visible hemisphere is c.z < -1/D, i.e. the OPPOSITE pole of the same c.
void AxisOffsetDir(LensParam::LensType type, double theta, float out[3]) {
  const double st = std::sin(theta);
  const double ct = std::cos(theta);
  if (type == LensParam::kGlobe) {
    // c = (sin t, 0, -cos t)  =>  w = -c
    out[0] = static_cast<float>(-st);
    out[1] = 0.0f;
    out[2] = static_cast<float>(ct);
    return;
  }
  // c (or s) = (sin t, 0, cos t)  =>  w = -c
  out[0] = static_cast<float>(-st);
  out[1] = 0.0f;
  out[2] = static_cast<float>(-ct);
}

// Where the optical axis itself lands, in pixels. Single-lens / rectangular / globe put it at the
// canvas centre; the dual-fisheye family puts it at the centre of the disc the direction fell in.
void AxisPixelCentre(const lm_proj::ProjParams& p, double* cx, double* cy) {
  const auto type = static_cast<LensParam::LensType>(p.proj_type);
  const bool dual = type == LensParam::kDualFisheyeEqualArea || type == LensParam::kDualFisheyeEquidistant ||
                    type == LensParam::kDualFisheyeStereographic || type == LensParam::kDualFisheyeOrthographic;
  *cy = p.img_h / 2.0;
  if (!dual) {
    *cx = p.img_w / 2.0;
    return;
  }
  const double disc_r = static_cast<double>(std::min(p.img_w / 2, p.img_h)) / 2.0;
  *cx = p.img_w / 2.0 - disc_r;  // the upper hemisphere's disc (DualFisheyeToPixelXY, is_upper)
}

}  // namespace

// ORACLE A. Each closed form equals the limit rebuilt from that projection's own mapping.
TEST(AxisSolidAngle, MatchesTheConeOverDiscLimit) {
  for (const auto& c : kCases) {
    const auto p = MakeParams(c);
    const double subject = lumice::ComputeAxisSolidAngle(p);
    const double oracle = LimitEstimate(p, 1e-4);
    if (!(subject > 0.0)) {
      ADD_FAILURE() << c.name << ": closed form returned " << subject << " for a valid calibration";
      continue;
    }
    EXPECT_NEAR(subject / oracle, 1.0, 1e-6)
        << c.name << ": closed form " << subject << " vs cone/disc limit " << oracle;
  }
}

// ORACLE A, convergence. The finite theta above must be inside the limit, not perched on the
// O(theta^2) tail of it — otherwise a branch could be wrong by exactly that tail and still pass.
TEST(AxisSolidAngle, TheLimitHasActuallyConverged) {
  for (const auto& c : kCases) {
    const auto p = MakeParams(c);
    const double coarse = LimitEstimate(p, 1e-3);
    const double fine = LimitEstimate(p, 1e-4);
    // Two decades of theta is four decades of the leading error term, so agreement at 1e-8 says
    // the residual at 1e-4 is far below the 1e-6 band the test above runs at.
    EXPECT_NEAR(coarse / fine, 1.0, 1e-6)
        << c.name << ": estimate still moving with theta (" << coarse << " vs " << fine << ")";
  }
}

// ORACLE B. The mapping Oracle A differentiates is the one this lens type is really rendered
// with — checked against the production forward, at an angle far enough off axis that a wrong
// family separates by many pixels.
TEST(AxisSolidAngle, TheMappingBelongsToTheType) {
  for (const auto& c : kCases) {
    const auto p = MakeParams(c);
    // 20 degrees: inside every type's domain at every fov in the table (the narrowest is the
    // 60-degree rectilinear, whose half-fov is 30), and far enough out that the four fisheye
    // laws are already tens of pixels apart.
    const double theta = 20.0 * kPi / 180.0;
    float w[3];
    AxisOffsetDir(c.type, theta, w);
    const auto hit = lm_proj::ProjectExitToPixel(p, w[0], w[1], w[2]);
    if (hit.count < 1) {
      ADD_FAILURE() << c.name << ": production forward culled a direction 20 deg off axis";
      continue;
    }
    double cx = 0.0;
    double cy = 0.0;
    AxisPixelCentre(p, &cx, &cy);
    const double dx = (hit.hits[0].px + 0.5) - cx;
    const double dy = (hit.hits[0].py + 0.5) - cy;
    const double measured = std::sqrt(dx * dx + dy * dy);
    const double predicted = RhoOfTheta(p, theta);
    // One pixel of slack for the forward's floor() binning, half a pixel of centring, and the
    // float arithmetic underneath both. A mapping from the wrong family misses by far more: at
    // this theta the four fisheye laws span 88 to 118 pixels on a 512-wide frame.
    EXPECT_NEAR(measured, predicted, 1.5) << c.name << ": production forward puts a 20 deg ray at " << measured
                                          << " px, the restated mapping predicts " << predicted;
  }
}

// The guard, stated as behaviour rather than inferred from the code: a degenerate calibration
// returns 0 (which callers already treat as "no anchor") rather than an infinity that would paint
// a frame white.
TEST(AxisSolidAngle, DegenerateCalibrationReturnsZero) {
  lm_proj::ProjParams p{};
  p.proj_type = static_cast<int>(LensParam::kLinear);
  p.scale = 0.0f;
  EXPECT_FLOAT_EQ(lumice::ComputeAxisSolidAngle(p), 0.0f);

  lm_proj::ProjParams d{};
  d.proj_type = static_cast<int>(LensParam::kDualFisheyeEqualArea);
  d.img_w = 0;
  d.img_h = 0;
  d.r_scale = 1.0f;
  EXPECT_FLOAT_EQ(lumice::ComputeAxisSolidAngle(d), 0.0f);
}

// The overlap ring is not cosmetic to this number. Shrinking the hemisphere into a smaller disc
// puts more sky behind every pixel, by exactly the square of the shrink — the property a
// (type, fov, resolution) signature could not have expressed at all.
TEST(AxisSolidAngle, OverlapWidensTheAxisPixel) {
  const LensCase* plain = nullptr;
  const LensCase* ringed = nullptr;
  for (const auto& c : kCases) {
    if (c.type == LensParam::kDualFisheyeEqualArea) {
      (c.overlap > 0.0f ? ringed : plain) = &c;
    }
  }
  ASSERT_NE(plain, nullptr);
  ASSERT_NE(ringed, nullptr);
  const auto p_plain = MakeParams(*plain);
  const auto p_ring = MakeParams(*ringed);
  const float w_plain = lumice::ComputeAxisSolidAngle(p_plain);
  const float w_ring = lumice::ComputeAxisSolidAngle(p_ring);
  EXPECT_GT(w_ring, w_plain);
  // ComputeEARScale(dz) = 1/sqrt(1+dz), so the ratio is exactly (1 + dz).
  EXPECT_NEAR(w_ring / w_plain, 1.0 + ringed->overlap, 1e-5);
}
