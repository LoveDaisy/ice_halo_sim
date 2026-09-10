#pragma once

namespace lumice {

// =================================================================================================
// The ONE line-width rule for the annotation curves, shared by both renderers.
//
// The celestial horizon, the parallels and meridians of the coordinate grid, and the circles of
// constant angular distance from the sun are all LEVEL SETS of a per-pixel angle field, in degrees:
// altitude, azimuth, angular distance. Both renderers decide "is this pixel on the line" the same
// way — by comparing the pixel's distance to the level, in degrees, against a half-width derived
// from how fast the field changes across a pixel:
//
//   half_width_deg = clamp(fwidth(field_deg), kAnnotationLineFwidthMinDeg, kAnnotationLineFwidthMaxDeg)
//                  * kAnnotationLineHalfWidthPx
//   on the line   <=>  |field_deg - level_deg| < half_width_deg
//
// `fwidth` here is |dF/dx| + |dF/dy| in degrees per pixel, so a fixed count of PIXELS (the
// multiplier) becomes a locally correct count of degrees, and the line reads the same width at a
// 1 degree field of view and at 180. Both evaluators compute it as a forward difference against
// neighbouring pixels rather than reading a hardware derivative: the CLI on the CPU over a
// rendered canvas (src/core/lens_proj_build.hpp, mask_detail::LevelSetMaskFromField), the GUI
// preview per fragment in its shader (src/gui/preview_renderer.cpp, overlayAuxLines, re-projecting
// the right/bottom neighbour pixels via inverseWorldDir rather than using the hardware fwidth() —
// the hardware derivative's quad granularity measured a real defect on rectilinear scenes, see
// overlayAuxLines for the details). "On the line" is a hard threshold, `|field_deg - level_deg| <
// half_width_deg`, evaluated identically by both sides (not an antialiased falloff). Two
// evaluators of ONE definition — and the three numbers below are that definition's only free
// parameters, so this is where they live, once.
//
// This file is in src/util/ for the reason label_viewport_clamp.hpp states: src/gui/ may not
// include core/ (the C API boundary, scripts/check_policies.py) and core may not depend on the
// GUI, so a rule written on either side could only be shared by copying it. Three floats with no
// simulation or configuration semantics are exactly what may travel through util/. GLSL cannot
// include this header either; the preview uploads the three values as uniforms from these
// constants rather than spelling the digits into the shader source.
// =================================================================================================

// Lower clamp on the local gradient, in degrees per pixel. Not zero: a zero half-width makes the
// hard threshold `|field_deg - level_deg| < half_width_deg` unsatisfiable everywhere, collapsing
// the line to nothing on both sides (the shader's `lineCoverage` and the CPU's forward-difference
// gradient alike). Low enough that it never binds at any field of view this renderer supports — the
// narrowest is 1 degree over hundreds of pixels, ~1e-3 deg/px — so the measured gradient is honoured
// wherever it is finite. (A higher floor pins the degrees-per-pixel conversion and makes the line
// grow with magnification; that was a measured defect, not a hypothetical.)
inline constexpr float kAnnotationLineFwidthMinDeg = 1e-4f;

// Upper clamp, in degrees per pixel. The azimuth field's gradient diverges at the poles and every
// field's does at a lens's image-circle rim, where one pixel spans many degrees; without a ceiling
// the line there would become a band tens of pixels deep.
inline constexpr float kAnnotationLineFwidthMaxDeg = 2.0f;

// The line's half-width, in pixels, before conversion to degrees by the clamped gradient. 1.5 px
// either side of the curve is a 3 px line — the same figure the marker rings and the lens border
// use for their own pixel-space edges.
inline constexpr float kAnnotationLineHalfWidthPx = 1.5f;

}  // namespace lumice
