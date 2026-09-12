#pragma once

#include <cmath>
#include <optional>

namespace lumice {

// =================================================================================================
// The ONE focal-length → field-of-view conversion for a `lens` that states `f` instead of `fov`,
// shared by both readers of the config document.
//
// Two readers decode that object: core's `LensParam::from_json` (src/config/render_config.cpp,
// behind the CLI and the C API) and the GUI's import path (src/gui/file_io.cpp,
// `DeserializeFromJson`). `f` is the author's second way of saying how wide the lens is, and the
// angle it maps to depends on the projection — so a document written with `f` must map to the same
// `fov` on both sides, or "what the GUI shows" and "what the CLI renders" diverge on an input the
// author DID write, which no round-trip test on either side can see. So the formulas live here,
// once, and both sides call this function rather than each carrying a copy of the switch.
//
// The parameter is a formula family rather than either side's lens-type enum, for the same reason
// util/lens_fov_default.hpp takes a bool: core's `LensParam::LensType` and the GUI's `LensType` are
// two different types, and a shared function bound to one could only be reached from the other
// side by copying the enum. Eleven lens types collapse onto six formulas (each dual-fisheye shares
// its single-lens sibling's, and `globe` shares linear's), so the family is what actually varies;
// each caller keeps the small `LensType → LensFocalFormula` switch on its own side, in the file that
// owns its enum. Those two switches are the one thing this header cannot unify — when a lens type
// is added, both must be extended, and neither writes a `default:` so the compiler names the one
// that was missed.
//
// The formulas are the inverse of the per-projection scale in src/server/render.cpp
// (ComputeLensScale) and of the GUI preview shader's `*Inverse` branches: `f` is the 35mm-equivalent
// focal length and `d = 12mm` is half the short edge of a 35mm frame (24mm / 2), the short-edge
// convention `fov` itself is measured on (doc/configuration.md, "Lens"). `d` is part of the formula,
// not a parameter — neither side has ever supplied its own, and letting them would reopen the
// divergence this file closes.
//
// `nullopt` means the formula has no solution for this `f`: equal-area needs `f >= d/2 = 6mm` and
// orthographic `f >= d = 12mm` to reach an edge of the frame at all. The function reports the
// domain error and does not decide what to do about it — throwing a config-layer exception would
// make a config-layer contract out of a header that may not know config exists; core rejects the
// document (as it always has), the GUI warns and falls back to the shared default.
//
// `kRadToDegree` below is a second spelling of core's `math::kRadToDegree` (src/core/math.hpp),
// deliberately: util/ may be included from both sides precisely because it includes neither, so it
// cannot reach core's constant. Same literal, same float; the two cannot drift apart without one
// of them being edited, and a test on each side pins the angle they produce.
// =================================================================================================

enum class LensFocalFormula {
  kLinear,                // `fov = 2 · atan(d / f)`; also `globe`, whose on-image scale is linear's
  kFisheyeEqualArea,      // `fov = 4 · asin(d / 2f)`; no solution when `d / 2f > 1`
  kFisheyeEquidistant,    // `fov = d / f` (radians), as core has always computed it — note that
                          // doc/configuration.md states `2d / f`, which is also what the forward
                          // projection's edge implies; reconciling the two changes the CLI's
                          // output for such configs and is a decision of its own, not a move.
  kFisheyeStereographic,  // `fov = 4 · atan(d / 2f)`
  kFisheyeOrthographic,   // `fov = 2 · asin(d / f)`; no solution when `d / f > 1`
  kRectangular,           // always full-sky; `f` is ignored and `fov` is 0 by convention
};

inline std::optional<float> LensFocalLengthToFovDegrees(LensFocalFormula formula, float f_mm) {
  constexpr float kHalfShortEdge = 12.0f;  // half short edge of 35mm film (24mm / 2)
  constexpr float kRadToDegree = 180.0f / 3.14159265359f;
  const float d = kHalfShortEdge;
  const float f = f_mm;
  switch (formula) {
    case LensFocalFormula::kLinear:
      return std::atan2(d, f) * 2 * kRadToDegree;
    case LensFocalFormula::kFisheyeEqualArea:
      if (d / (2 * f) > 1.0f) {
        return std::nullopt;
      }
      return std::asin(d / (2 * f)) * 4 * kRadToDegree;
    case LensFocalFormula::kFisheyeEquidistant:
      return (d / f) * kRadToDegree;
    case LensFocalFormula::kFisheyeStereographic:
      return std::atan(d / (2 * f)) * 4 * kRadToDegree;
    case LensFocalFormula::kFisheyeOrthographic:
      // r = f · sin(theta); the frame edge is r_max = d. Unlike equal-area's r = 2f · sin(theta/2)
      // (denominator 2f), orthographic divides by f directly.
      if (d / f > 1.0f) {
        return std::nullopt;
      }
      return std::asin(d / f) * 2 * kRadToDegree;
    case LensFocalFormula::kRectangular:
      return 0.0f;
  }
  return std::nullopt;  // unreachable: every enumerator returns above
}

}  // namespace lumice
