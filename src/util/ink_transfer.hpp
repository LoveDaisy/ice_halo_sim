#pragma once

#include <algorithm>
#include <cmath>

namespace lumice {

// =================================================================================================
// The ONE subtractive (density) transfer curve, shared by both renderers.
//
// Under RenderConfig::kPrint the picture is made of INK on PAPER, not of light added to a sky:
//
//   e     = Y * ExposureScale()      // the exposed scalar, CIE Y after the exposure chain
//   D     = kInkGamma * log10(1 + e) // optical density
//   out_j = paper_j * 10^(-D)        // transmittance times the paper's own colour
//
// which is the second law of doc/print-mode-subtractive-ink.md §3.2. It is structurally NOT the
// additive operator with different parameters: 10^(-D) <= 1 for every D >= 0, so ink can only ever
// make the paper darker, while `L * c + background` is monotonically non-decreasing in radiance.
// That is why the two are a mode enum rather than two ends of one knob.
//
// This file is that curve's SINGLE OWNER on the C++ side. It lives in src/util/ rather than in
// either renderer for the reason AGENTS.md names for this exemption: src/gui/ may not include
// core/ or config/ (the C API boundary, enforced by scripts/check_policies.py), so a rule written
// on either side could only be shared by copying it. A pure, stateless function carrying no
// simulation or configuration semantics is exactly what may travel that way.
//
// It has a THIRD implementation that this header cannot reach: the preview fragment shader's
// `subtractiveInk()` (src/gui/preview_renderer.cpp), because GLSL cannot #include a C++ header.
// That one is a hand transcription and says so at its definition; keep the two in step.
// =================================================================================================

// The density slope. Calibrated from data, not chosen: the P95 of the non-zero exposed scalar `e`
// over four scenes was solved back to D = 0.5 and the geometric mean taken, and the owner accepted
// the result by eye (doc/print-mode-subtractive-ink.md §3.2).
//
// It is deliberately NOT a config field and NOT a parameter of the functions below — see that
// document's §9.2. Do not re-calibrate it here; a change to this number is a change to the
// document's decision, and test/unit-correctness/util/test_ink_transfer.cpp pins it so that such a
// change has to be made on purpose.
inline constexpr float kInkGamma = 11.0f;

// Optical density of the ink laid down by an exposed scalar `e`.
//
// `e` is CIE Y after ExposureScale() and nothing else. This curve was calibrated on that domain
// alone: do NOT hand it an annotation's alpha (that path is a plain coverage multiply, `1 - alpha`
// — see BlendAnnotation() in src/server/render.cpp), and do not hand it a per-channel value (print
// is greyscale by construction, doc/print-mode-subtractive-ink.md §5).
//
// Negative `e` is clamped to zero rather than rejected: log10 of a negative is NaN, and a caller
// that has clamped nothing upstream should get the paper back, not a poisoned pixel.
inline float InkOpticalDensity(float e) {
  return kInkGamma * std::log10(1.0f + std::max(e, 0.0f));
}

// Transmittance of that ink: in (0, 1] for every density >= 0, which is the algebraic statement of
// "the subtractive path can never come out brighter than the paper it started on".
inline float InkTransmittance(float density) {
  return std::pow(10.0f, -density);
}

}  // namespace lumice
