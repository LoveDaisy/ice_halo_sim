// The preview projections' relative illumination, checked against an oracle that is not itself.
//
// src/gui/preview_jacobian.hpp holds a closed form per projection branch, and the GLSL in
// preview_renderer.cpp holds a hand-transcribed twin of each. Comparing those two to each other
// would only catch a typo made in one of them; a mistake in the DERIVATION, or a transcription
// error made identically in both places, would pass. So this file does not compare them. It
// rebuilds the Jacobian numerically from the projection's own mapping theta(rho) — the mapping the
// shader inverts, which the visual and parity suites already exercise pixel-by-pixel — and holds
// the closed forms to that:
//
//     Omega_p(rho) = sin(theta(rho)) * |dtheta/drho| / rho        (rotational symmetry)
//     m(rho)       = Omega_p(rho) / Omega_p(on axis)
//
// with the derivative taken by central difference. That leaves only theta(rho) shared between the
// oracle and the thing under test, and theta(rho) is the definition of the projection rather than
// the algebra this file exists to check.
//
// A second oracle covers what the first one cannot. The derivative check shares theta(rho) with
// the subject, so an error in the MAPPING itself — the wrong fisheye law, a factor of two in a
// half-angle — is invisible to it: both sides would move together. The integral check does not
// share it. Integrating m over the image plane must come out as (the solid angle the lens covers)
// / (its on-axis Omega_p), and the first of those is 2*pi*(1 - cos(half_fov)) for any lens here,
// straight from the definition of a cone and touching neither the closed forms nor the mappings.
//
// Both directions were confirmed by breaking the code on purpose rather than assumed: dropping one
// cosine from the rectilinear branch fails the derivative check by construction, and the whole
// point of writing the integral separately is that it answers a question the other one does not
// ask.

#include <cmath>
#include <vector>

#include "gtest/gtest.h"
#include "gui/preview_jacobian.hpp"

namespace {

using lumice::gui::FisheyeKind;
using lumice::gui::kGlobeCameraD;

constexpr double kPi = 3.14159265358979323846;

// ---------------------------------------------------------------------------------------------
// The mappings, restated from the projections' definitions. These mirror what the shader's
// *Inverse functions solve; they are the shared premise of oracle and subject, deliberately.
// r_norm is the image radius normalized to the image circle (or to `focal` for a rectilinear
// lens, where there is no circle).

double ThetaRectilinear(double rho, double focal) {
  return std::atan(rho / focal);
}

double ThetaFisheye(FisheyeKind kind, double r_norm, double half_fov) {
  switch (kind) {
    case FisheyeKind::kEqualArea:
      return 2.0 * std::asin(r_norm * std::sin(half_fov * 0.5));
    case FisheyeKind::kEquidistant:
      return r_norm * half_fov;
    case FisheyeKind::kStereographic:
      return 2.0 * std::atan(r_norm * std::tan(half_fov * 0.5));
    case FisheyeKind::kOrthographic:
    default:
      return std::asin(r_norm * std::sin(half_fov));
  }
}

// Globe: the polar angle of the sphere point a pixel's ray hits, from the same ray-sphere solve
// the shader's globeInverse runs. Camera at (0,0,D) in eye space looking toward -z at a unit
// sphere centred on the origin; psi is measured from the camera axis.
double PsiGlobe(double rho, double focal) {
  const double d = kGlobeCameraD;
  const double len = std::sqrt(rho * rho + focal * focal);
  const double dz = -focal / len;  // ray direction z, eye space
  const double b = d * dz;
  const double disc = b * b - (d * d - 1.0);
  if (disc < 0.0) {
    return std::nan("");
  }
  const double t = -b - std::sqrt(disc);
  const double mu = d + t * dz;  // hit_eye.z, and the sphere point is unit length
  return std::acos(std::min(1.0, std::max(-1.0, mu)));
}

// Numerical Omega_p, up to the constant that cancels in m: sin(theta) * dtheta/drho / rho.
template <typename ThetaOfRho>
double NumericOmega(ThetaOfRho theta_of_rho, double rho, double h) {
  const double t0 = theta_of_rho(rho - h);
  const double t1 = theta_of_rho(rho + h);
  return std::sin(theta_of_rho(rho)) * ((t1 - t0) / (2.0 * h)) / rho;
}

// m from the numerical Jacobian: the ratio to its value at a radius small enough to stand in for
// the axis. 1e-4 of the image radius puts the residual curvature of every branch here below 1e-8.
template <typename ThetaOfRho>
double NumericRelIllum(ThetaOfRho theta_of_rho, double rho, double scale) {
  const double h = 1e-6 * scale;
  const double rho_axis = 1e-4 * scale;
  return NumericOmega(theta_of_rho, rho, h) / NumericOmega(theta_of_rho, rho_axis, h);
}

}  // namespace

// ---------------------------------------------------------------------------------------------
// Per-branch agreement with the numerical Jacobian.

TEST(PreviewJacobian, RectilinearMatchesNumericalJacobian) {
  // Three focal lengths, i.e. three FOVs on the same canvas: 60, 120 and 160 degrees on a 512px
  // short edge. The last is the FOV the GUI<->CLI divergence was originally measured at.
  const double img_radius = 256.0;
  for (double fov_deg : { 60.0, 120.0, 160.0 }) {
    const double focal = img_radius / std::tan(fov_deg * kPi / 360.0);
    // Out to 1.665 image radii — the corner of a 4:3 frame, past which no pixel exists.
    for (double frac : { 0.05, 0.2, 0.4, 0.6, 0.8, 1.0, 1.4, 1.665 }) {
      const double rho = frac * img_radius;
      const double expect = NumericRelIllum([focal](double r) { return ThetaRectilinear(r, focal); }, rho, img_radius);
      const double got = lumice::gui::RelIllumRectilinear(static_cast<float>(rho), static_cast<float>(focal));
      EXPECT_NEAR(got, expect, 1e-4 * expect) << "fov=" << fov_deg << " rho/R=" << frac;
    }
  }
}

TEST(PreviewJacobian, FisheyeBranchesMatchNumericalJacobian) {
  const double img_radius = 256.0;
  const std::vector<FisheyeKind> kinds = { FisheyeKind::kEqualArea, FisheyeKind::kEquidistant,
                                           FisheyeKind::kStereographic, FisheyeKind::kOrthographic };
  for (FisheyeKind kind : kinds) {
    for (double fov_deg : { 60.0, 120.0, 180.0 }) {
      const double half_fov = fov_deg * kPi / 360.0;
      // Stop at 0.9 of the image circle: orthographic is clamped in the last half pixel by design
      // (checked separately below), and the central difference needs room on both sides.
      for (double r_norm : { 0.05, 0.2, 0.4, 0.6, 0.8, 0.9 }) {
        const double rho = r_norm * img_radius;
        const double expect = NumericRelIllum(
            [kind, half_fov, img_radius](double r) { return ThetaFisheye(kind, r / img_radius, half_fov); }, rho,
            img_radius);
        const double got = lumice::gui::RelIllumFisheye(kind, static_cast<float>(r_norm), static_cast<float>(half_fov),
                                                        static_cast<float>(img_radius));
        EXPECT_NEAR(got, expect, 1e-4 * expect)
            << "kind=" << static_cast<int>(kind) << " fov=" << fov_deg << " r_norm=" << r_norm;
      }
    }
  }
}

TEST(PreviewJacobian, EquirectMatchesNumericalAreaElement) {
  // The equirectangular map is separable, so the rotationally symmetric form above does not apply;
  // its area element is checked directly. dOmega = cos(lat) dlon dlat against dA = scale^2 dlon
  // dlat, so m must be cos(lat) with no scale left in it.
  const double scale = 256.0 / kPi;
  for (double lat_deg : { 0.0, 15.0, 45.0, 75.0, 89.0 }) {
    const double lat = lat_deg * kPi / 180.0;
    const double h = 1e-6;
    // Numerical: the solid angle of a lat-band of height h at `lat`, per unit image area.
    const double d_omega = std::sin(lat + h) - std::sin(lat - h);  // integrated cos(lat) dlat
    const double d_area = 2.0 * h * scale * scale / scale;         // dy = scale*dlat, per unit dlon in px
    const double omega = d_omega / (2.0 * h) / (d_area / (2.0 * h));
    const double omega_axis = 1.0 / scale;  // the same expression at lat = 0
    EXPECT_NEAR(lumice::gui::RelIllumEquirect(static_cast<float>(lat)), omega / omega_axis, 1e-5) << "lat=" << lat_deg;
  }
}

TEST(PreviewJacobian, GlobeMatchesNumericalJacobian) {
  const double img_radius = 256.0;
  for (double fov_deg : { 30.0, 60.0 }) {
    const double focal = img_radius / std::tan(fov_deg * kPi / 360.0);
    const double rho_limb = focal / std::sqrt(kGlobeCameraD * kGlobeCameraD - 1.0);
    for (double frac : { 0.05, 0.25, 0.5, 0.75, 0.9 }) {
      const double rho = frac * rho_limb;
      const double expect = NumericRelIllum([focal](double r) { return PsiGlobe(r, focal); }, rho, rho_limb);
      const double got = lumice::gui::RelIllumGlobe(static_cast<float>(rho), static_cast<float>(focal));
      EXPECT_NEAR(got, expect, 1e-3 * expect) << "fov=" << fov_deg << " rho/limb=" << frac;
    }
  }
}

// ---------------------------------------------------------------------------------------------
// The normalization itself. The derivative check above compares two ratios and so cannot see a
// wrong on-axis value; this integral can. Integrating m over the image plane gives the lens's
// total solid angle divided by its on-axis Omega_p, and both of those are known independently:
// a cone of half-angle hf subtends 2*pi*(1 - cos(hf)), and Omega_p(0) comes out of the mapping's
// own small-radius limit.

TEST(PreviewJacobian, IntegralOverImageRecoversCoveredSolidAngle) {
  const double img_radius = 256.0;
  const int kSteps = 20000;
  struct Case {
    FisheyeKind kind;
    double fov_deg;
  };
  for (const Case& c : { Case{ FisheyeKind::kEqualArea, 120.0 }, Case{ FisheyeKind::kEquidistant, 120.0 },
                         Case{ FisheyeKind::kStereographic, 120.0 }, Case{ FisheyeKind::kOrthographic, 120.0 },
                         Case{ FisheyeKind::kEquidistant, 180.0 } }) {
    const double half_fov = c.fov_deg * kPi / 360.0;
    // integral of m dA over the disc, in pixel^2
    double integral = 0.0;
    for (int i = 0; i < kSteps; i++) {
      const double r_norm = (i + 0.5) / kSteps;
      const double m = lumice::gui::RelIllumFisheye(c.kind, static_cast<float>(r_norm), static_cast<float>(half_fov),
                                                    static_cast<float>(img_radius));
      integral += m * 2.0 * kPi * (r_norm * img_radius) * (img_radius / kSteps);
    }
    // Omega_p on axis, from the mapping alone: theta ~ k * r_norm near zero gives
    // Omega_p(0) = k^2 / img_radius^2.
    const double k = ThetaFisheye(c.kind, 1e-6, half_fov) / 1e-6;
    const double omega_axis = k * k / (img_radius * img_radius);
    const double covered = 2.0 * kPi * (1.0 - std::cos(half_fov));
    EXPECT_NEAR(integral * omega_axis, covered, 2e-3 * covered)
        << "kind=" << static_cast<int>(c.kind) << " fov=" << c.fov_deg;
  }
}

// ---------------------------------------------------------------------------------------------
// AC4: the equal-area branches are the identity, so an equal-area preview is bit-for-bit what it
// was before this factor existed and its committed reference images stand without a re-shoot.

TEST(PreviewJacobian, EqualAreaIsExactlyOneEverywhere) {
  for (double fov_deg : { 30.0, 96.0, 120.0, 180.0 }) {
    const double half_fov = fov_deg * kPi / 360.0;
    for (double r_norm = 0.0; r_norm <= 1.5; r_norm += 0.01) {
      // Exact equality, not a tolerance: a value that merely rounds to 1 would still perturb the
      // last bit of every equal-area pixel, and the claim being made is bit-for-bit.
      EXPECT_EQ(lumice::gui::RelIllumFisheye(FisheyeKind::kEqualArea, static_cast<float>(r_norm),
                                             static_cast<float>(half_fov), 256.0f),
                1.0f)
          << "fov=" << fov_deg << " r_norm=" << r_norm;
    }
  }
  // Dual fisheye's equal-area half is the same branch at a fixed half-FOV of pi/2.
  EXPECT_EQ(lumice::gui::RelIllumFisheye(FisheyeKind::kEqualArea, 0.7f, static_cast<float>(kPi * 0.5), 128.0f), 1.0f);
}

// ---------------------------------------------------------------------------------------------
// AC1: what this factor does, and does not do, to ABSOLUTE exposure.
//
// Absolute EV anchors on emitted energy and is not re-derived per frame, so whatever this factor
// multiplies into a pixel lands directly in that pixel's absolute brightness. Normalizing on axis
// is what decides the answer: the centre of the frame is unchanged at every FOV and for every
// lens, and what appears is the vignetting shape alone. The alternative — normalizing against the
// source texture's texel solid angle — would instead have moved absolute brightness by a factor
// set by the preview canvas's angular resolution, which is a display choice rather than physics,
// and would not have made absolute mode comparable across lenses either (the GUI's absolute
// denominator counts the source texture's pixels while the CLI's counts its own canvas's). See
// doc/ev-pipeline-architecture.md 7.6.

TEST(PreviewJacobian, OnAxisIsUnityForEveryLensAndFov) {
  for (double fov_deg : { 20.0, 60.0, 120.0, 179.0 }) {
    const double half_fov = fov_deg * kPi / 360.0;
    const double focal = 256.0 / std::tan(half_fov);
    EXPECT_NEAR(lumice::gui::RelIllumRectilinear(0.0f, static_cast<float>(focal)), 1.0, 1e-6) << "fov=" << fov_deg;
    EXPECT_NEAR(lumice::gui::RelIllumGlobe(0.0f, static_cast<float>(focal)), 1.0, 1e-6) << "fov=" << fov_deg;
    for (FisheyeKind kind : { FisheyeKind::kEqualArea, FisheyeKind::kEquidistant, FisheyeKind::kStereographic,
                              FisheyeKind::kOrthographic }) {
      EXPECT_NEAR(lumice::gui::RelIllumFisheye(kind, 0.0f, static_cast<float>(half_fov), 256.0f), 1.0, 1e-6)
          << "kind=" << static_cast<int>(kind) << " fov=" << fov_deg;
    }
  }
  EXPECT_NEAR(lumice::gui::RelIllumEquirect(0.0f), 1.0, 1e-6);
}

TEST(PreviewJacobian, VignettingDeepensWithFieldOfView) {
  // The shape that absolute mode DOES gain: the same pixel of the frame sits further off axis as
  // the FOV widens, so the corner darkens monotonically. Stated as an assertion because it is the
  // half of the absolute-mode answer that is not "nothing changes".
  const double img_radius = 256.0;
  const double corner = 1.665 * img_radius;  // 4:3 frame corner
  double previous = 2.0;
  for (double fov_deg : { 40.0, 80.0, 120.0, 160.0 }) {
    const double focal = img_radius / std::tan(fov_deg * kPi / 360.0);
    const double m = lumice::gui::RelIllumRectilinear(static_cast<float>(corner), static_cast<float>(focal));
    EXPECT_LT(m, previous) << "fov=" << fov_deg;
    EXPECT_GT(m, 0.0) << "fov=" << fov_deg;
    previous = m;
  }
  EXPECT_LT(previous, 0.01);  // at fov 160 the 4:3 corner is more than 6 stops down
}

// ---------------------------------------------------------------------------------------------
// The two branches that genuinely diverge, and the clamp that keeps them finite.

TEST(PreviewJacobian, OrthographicStaysFiniteAtAndPastTheRim) {
  const float half_fov = static_cast<float>(kPi * 0.5);  // fov 180: the rim is exactly theta = 90
  for (float img_radius : { 64.0f, 256.0f, 1024.0f }) {
    const float at_rim = lumice::gui::RelIllumFisheye(FisheyeKind::kOrthographic, 1.0f, half_fov, img_radius);
    EXPECT_TRUE(std::isfinite(at_rim)) << "img_radius=" << img_radius;
    // The clamp is the half-pixel rule, so it must move with the raster: a finer grid resolves
    // closer to the singularity and reports a larger factor. A fixed magic ceiling would not.
    const float coarser = lumice::gui::RelIllumFisheye(FisheyeKind::kOrthographic, 1.0f, half_fov, img_radius * 0.5f);
    EXPECT_GT(at_rim, coarser) << "img_radius=" << img_radius;
    // Past the rim the shader has already discarded the pixel; the value must still be finite so a
    // NaN cannot leak through a branch that is evaluated before the domain guard.
    EXPECT_EQ(lumice::gui::RelIllumFisheye(FisheyeKind::kOrthographic, 4.0f, half_fov, img_radius), at_rim);
  }
  // And it really is a divergence being held back, not a flat branch: the factor grows without
  // bound as the grid gets finer.
  EXPECT_GT(lumice::gui::RelIllumFisheye(FisheyeKind::kOrthographic, 1.0f, half_fov, 16384.0f), 100.0f);
}

TEST(PreviewJacobian, GlobeStaysFiniteAtAndPastTheSilhouette) {
  const float focal = 256.0f / std::tan(static_cast<float>(kPi / 12.0));  // fov 30, the globe default
  const float rho_limb = focal / std::sqrt(kGlobeCameraD * kGlobeCameraD - 1.0f);
  const float at_limb = lumice::gui::RelIllumGlobe(rho_limb, focal);
  EXPECT_TRUE(std::isfinite(at_limb));
  EXPECT_GT(at_limb, 1.0f);  // the silhouette compresses sky, so it brightens
  // Past the silhouette no ray hits the sphere at all; the clamp must hold rather than produce a
  // negative radicand.
  EXPECT_EQ(lumice::gui::RelIllumGlobe(rho_limb * 4.0f, focal), at_limb);
  EXPECT_TRUE(std::isfinite(lumice::gui::RelIllumGlobe(rho_limb * 4.0f, focal)));
}
