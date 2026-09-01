#ifndef LUMICE_GUI_PREVIEW_JACOBIAN_HPP
#define LUMICE_GUI_PREVIEW_JACOBIAN_HPP

// The preview projection's RELATIVE ILLUMINATION, in one place, on the CPU.
//
// WHAT THIS IS FOR. The CLI renders a lens directly: every ray is binned into the pixel it lands
// on, so a pixel accumulates the energy arriving over ITS OWN solid angle, and the projection's
// per-pixel solid angle Omega_p is baked into the image. That is camera semantics and it is the
// correct thing for core to do. The GUI preview does not image a scene; it RESAMPLES the equal-area
// all-sky texture core produced, where every texel subtends the same solid angle, so a screen pixel
// carries radiance L with no Omega_p in it at all. The two therefore differ by exactly Omega_p. For
// an equal-area target lens Omega_p is spatially constant and the difference is a global factor
// nobody could see; for anything else it varies across the frame, and the GUI was missing the
// projection's own natural vignetting. These functions are that missing factor.
//
// WHAT IS NORMALIZED, AND WHY IT IS NOT Omega_p ITSELF. What the shader multiplies by is
//
//     m(pos) = Omega_p(pos) / Omega_p(on axis)
//
// — the RELATIVE ILLUMINATION of the projection, the standard photometric quantity, dimensionless
// and equal to 1 at the frame centre by construction. It is deliberately NOT the absolute ratio
// Omega_p / Omega_texel_source, and the reason is mechanical rather than aesthetic. Both sides of
// the GUI<->CLI comparison self-anchor their exposure on their own image, and they anchor on
// DIFFERENT images: the GUI's relative-EV denominator is a P99 over the source texture
// (server_poller.cpp's LUMICE_ComputeP99Y, computed before any reprojection), the CLI's is a P99
// over its own already-Jacobian-baked render (server/render.cpp RenderConsumer::ExposureScale).
// Writing out both chains, the GUI displays L / P99_sky(L) and the CLI displays
// L * Omega_p / P99_frame(L * Omega_p): the SCALE of Omega_p cancels on the CLI side against its
// own anchor, and only the SHAPE is left over as a disagreement. Multiplying the GUI by the
// absolute ratio would therefore not restore agreement — it would add a second, uncorrelated global
// gain whose physical content is the target canvas's angular resolution, which is a display choice.
// Measured: that gain is 0.331 for an equal-area fisheye at fov 96 on 512x683 (the GUI would go
// three times too dark) and 16 for a rectilinear lens at fov 160 on the same canvas (sixteen times
// too bright, i.e. clipped). Normalizing on axis adds the shape and leaves the level alone.
//
// Two consequences worth stating because they are load-bearing further down:
//   * Every equal-area branch has m identically 1, exactly, at every pixel. Equal-area previews are
//     bit-for-bit what they were before this factor existed — which is what lets the committed
//     equal-area reference images stand without a re-shoot.
//   * Under ABSOLUTE EV the on-axis brightness does not move with FOV either. That is the same
//     statement, not a separate decision, and doc/ev-pipeline-architecture.md 7.5 records why it is
//     the right one: absolute mode's cross-lens comparability is already outside its stated
//     contract (the GUI's absolute denominator counts the SOURCE TEXTURE's pixels, app.cpp, while
//     the CLI's counts its own canvas's, render.cpp), so an absolute Omega_p would not repair it.
//
// DERIVATION, shared by every entry below. For any projection with rotational symmetry about the
// optical axis, writing rho for the image radius and theta for the polar angle it images:
//
//     dOmega = sin(theta) dtheta dphi,   dA = rho drho dphi
//     Omega_p(rho) = dOmega/dA = sin(theta) * |dtheta/drho| / rho
//
// Each function states the mapping theta(rho) it inverts and the closed form that falls out. The
// results are the textbook ones (the cos^3 law for a pinhole; the constant density that defines an
// equal-area fisheye; sin(theta)/theta for equidistant; cos^4(theta/2) for stereographic; the
// 1/cos(theta) divergence of orthographic; the cos(latitude) area element of an equirectangular
// map) — see e.g. Ray, "Applied Photographic Optics", on natural vignetting, and Pharr/Jakob/
// Humphreys, "Physically Based Rendering", on converting between area and solid-angle measure,
// which is what the globe entry needs. Nothing here is fitted to a PSNR number.
//
// THIS FILE IS A MIRROR, and mirrors drift. The live copy is the GLSL in preview_renderer.cpp
// (relIllum* in the fragment shader); this one exists so the formulas can be tested without a
// window, in the same shape as overlay_labels.cpp's CPU mirror of the inverse projections and
// mono_exposure_scale.hpp's of the exposure arithmetic. A transcription error made identically on
// both sides is invisible to any test that only compares them, so the unit test does not: it checks
// each closed form against a NUMERICAL INTEGRATION of the projection it belongs to
// (test/unit-correctness/gui/test_preview_jacobian.cpp), which is an independent oracle.
//
// Deliberately free of ImGui / GL / server dependencies so it can be unit-tested without a window.

#include <algorithm>
#include <cmath>

#include "gui/gui_constants.hpp"

namespace lumice::gui {

// Fisheye sub-type, matching the shader's `fisheyeInverse(..., int type)` switch and
// `dualFisheyeInverse`'s. Not an alias of LensType: both the single-lens and the dual-lens families
// index the same four mappings, and the orthographic member of each sits at a different LensType.
enum class FisheyeKind : int {
  kEqualArea = 0,
  kEquidistant = 1,
  kStereographic = 2,
  kOrthographic = 3,
};

// A point sample of a Jacobian cannot describe a pixel the raster cannot resolve. Both divergent
// branches (orthographic at its image circle, globe at the sphere's silhouette) are clamped to the
// value the factor takes half a pixel inside the singular locus — the outermost sample the grid can
// actually take. This is a statement about the sampling grid, not a fudge factor: it contains no
// number that was chosen to make an image look right, and it moves with the resolution.
inline constexpr float kSingularityGuardPixels = 0.5f;

// Rectilinear (pinhole) — LensType kLensTypeLinear, and the camera half of kLensTypeGlobe.
//
//   rho = focal * tan(theta)   =>   Omega_p = cos^3(theta) / focal^2
//
// so m = cos^3(theta), the classic cos^3 natural falloff, written directly in rho to keep it free
// of a trig round trip: cos(theta) = focal / hypot(focal, rho).
//
// No clamp and none needed: theta = atan(rho/focal) is strictly below 90 degrees at every finite
// radius, so this branch has no singularity and no image circle to fall off.
inline float RelIllumRectilinear(float rho, float focal) {
  const float c = focal / std::sqrt(focal * focal + rho * rho);
  return c * c * c;
}

// The four fisheye mappings, single-lens and dual alike.
//
// `r_norm` is the image radius normalized so that r_norm = 1 is the edge of the image circle
// (rho / img_radius for a single lens, rho_local / circle_radius for one half of a dual fisheye).
// `half_fov` is half the full field of view in radians; the dual family fixes it at pi/2.
// `img_radius` is the image circle's radius IN PIXELS, needed only by the orthographic clamp.
//
//   equal area    r_norm * sin(hf/2) = sin(theta/2)   => Omega_p = 4 sin^2(hf/2) / R^2   (constant)
//   equidistant   theta = r_norm * hf                 => Omega_p = hf^2/R^2 * sin(theta)/theta
//   stereographic r_norm * tan(hf/2) = tan(theta/2)   => Omega_p = 4 tan^2(hf/2) cos^4(theta/2)/R^2
//   orthographic  r_norm * sin(hf)   = sin(theta)     => Omega_p = sin^2(hf) / (R^2 cos(theta))
//
// Dividing each by its own value at r_norm = 0 leaves the m column below, in which every parameter
// except theta has cancelled — the relative illumination of a fisheye does not depend on its FOV
// except through the theta the radius maps to.
inline float RelIllumFisheye(FisheyeKind kind, float r_norm, float half_fov, float img_radius) {
  switch (kind) {
    case FisheyeKind::kEqualArea:
      // The definition of equal area is that this is constant, so its normalized form is 1
      // everywhere, exactly. Returned as a literal rather than computed: an equal-area preview must
      // come out bit-identical to one rendered before this factor existed.
      return 1.0f;

    case FisheyeKind::kEquidistant: {
      // m = sin(theta)/theta. The removable singularity at the frame centre is taken by the series
      // rather than by a division, so the centre pixel is 1 - theta^2/6 and not a NaN.
      const float theta = r_norm * half_fov;
      if (theta < 1e-3f) {
        return 1.0f - theta * theta / 6.0f;
      }
      return std::sin(theta) / theta;
    }

    case FisheyeKind::kStereographic: {
      // m = cos^4(theta/2), and theta/2 = atan(r_norm * tan(hf/2)) is exactly what the inverse
      // already solves, so this needs no second trig chain: cos(theta/2) = 1/hypot(1, r_norm*t).
      const float t = r_norm * std::tan(half_fov * 0.5f);
      const float c = 1.0f / std::sqrt(1.0f + t * t);
      const float c2 = c * c;
      return c2 * c2;
    }

    case FisheyeKind::kOrthographic:
    default: {
      // m = 1/cos(theta), with sin(theta) = r_norm * sin(hf). This one diverges at the image
      // circle, and the divergence is the projection, not a defect: an orthographic fisheye
      // compresses the horizon into its last ring, so a CLI render really does pile the energy of
      // an unbounded solid angle into those pixels. What the raster cannot do is sample past its
      // own last pixel centre, so r_norm is held half a pixel inside the rim.
      const float r_max = img_radius > kSingularityGuardPixels ? 1.0f - kSingularityGuardPixels / img_radius : 0.0f;
      const float s = std::min(r_norm, r_max) * std::sin(half_fov);
      const float cos_theta = std::sqrt(std::max(1.0f - s * s, 1e-12f));
      return 1.0f / cos_theta;
    }
  }
}

// Equirectangular — LensType kLensTypeRectangular.
//
//   lon = x/scale, lat = -y/scale, so dA = scale^2 dlon dlat against dOmega = cos(lat) dlon dlat
//   => Omega_p = cos(lat) / scale^2, and m = cos(lat).
//
// The poles go to zero and that is correct: an equirectangular map stretches a point into a whole
// row, so a CLI render leaves that row nearly empty. No clamp — this branch has no divergence, and
// the shader's own domain guard already discards |lat| > pi/2.
inline float RelIllumEquirect(float lat) {
  return std::max(std::cos(lat), 0.0f);
}

// Globe — LensType kLensTypeGlobe. A pinhole camera at distance D = kGlobeCameraD from a unit
// sphere, looking at it from OUTSIDE; the sky direction a pixel shows is the surface point its ray
// hits, because the sphere is centred on the origin.
//
// Writing mu for the cosine of the angle between that surface point and the camera axis, the
// forward map is rho = focal * sqrt(1 - mu^2) / (D - mu) (this is the same relation the shader's
// ray-sphere solve satisfies, and `mu` below is exactly its hit_eye.z). Differentiating,
//
//     rho * drho/dpsi = focal^2 sin(psi) (D mu - 1) / (D - mu)^3,  mu = cos(psi)
//     Omega_p = sin(psi) / (rho drho/dpsi) = (D - mu)^3 / (focal^2 (D mu - 1))
//
// and on axis (mu = 1) that is (D-1)^2/focal^2, giving
//
//     m = (D - mu)^3 / ((D mu - 1) (D - 1)^2).
//
// It diverges at the silhouette mu = 1/D, where the sphere turns away from the camera and an
// unbounded strip of sky is imaged into one ring of pixels — the same physics as the orthographic
// branch, and clamped the same way. The silhouette sits at rho = focal / sqrt(D^2 - 1); mu is
// therefore recovered from a radius held half a pixel inside it, by solving the forward map
//
//     k = rho/focal:  mu = (D k^2 + sqrt(1 - k^2 (D^2 - 1))) / (k^2 + 1)
//
// which is the near root, the one the shader's `-b - sqrt(disc)` picks.
inline float RelIllumGlobe(float rho, float focal) {
  const float d = kGlobeCameraD;
  const float rho_limb = focal / std::sqrt(d * d - 1.0f);
  const float rho_max = std::max(rho_limb - kSingularityGuardPixels, 0.0f);
  const float k = std::min(rho, rho_max) / focal;
  const float k2 = k * k;
  const float disc = std::max(1.0f - k2 * (d * d - 1.0f), 0.0f);
  const float mu = (d * k2 + std::sqrt(disc)) / (k2 + 1.0f);
  const float denom = std::max(d * mu - 1.0f, 1e-12f);
  const float num = d - mu;
  return num * num * num / (denom * (d - 1.0f) * (d - 1.0f));
}

}  // namespace lumice::gui

#endif  // LUMICE_GUI_PREVIEW_JACOBIAN_HPP
