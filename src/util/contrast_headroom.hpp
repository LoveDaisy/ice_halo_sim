#pragma once

#include <algorithm>

namespace lumice {

// =================================================================================================
// The ONE contrast-headroom predicate, shared by both renderers.
//
// Each tone operator has a degenerate direction, and doc/print-mode-subtractive-ink.md §8 (owner
// decision D6) is that the two are ONE idea and must therefore be ONE predicate:
//
//   tone      zero-energy colour   the law's far end   headroom
//   screen    background           white (clamp at 1)  1 - background
//   print     paper                black (D -> inf)    paper itself
//
// The "zero-energy colour" is what a pixel that received no ray energy comes out as under that
// law; the headroom is how far it still is from the other end of the law. When that distance goes
// to zero the picture is gone while the configuration still looks reasonable — and the screen half
// of that is the state the user feedback behind this whole scrum came from: a white sky leaves the
// grid and overlay lines perfectly visible (those are drawn by a lerp, BlendAnnotation in
// src/server/render.cpp) and only the halo disappears, which reads as broken software rather than
// as an unfortunate colour choice.
//
// It lives in src/util/ for the reason AGENTS.md names for this exemption, the same one
// ink_transfer.hpp beside it takes: src/gui/ may not include core/ or config/ (the C API boundary,
// enforced by scripts/check_policies.py), so a rule BOTH sides must obey could otherwise only be
// shared by copying it — and two copies of a threshold are two thresholds. A pure, stateless
// function carrying no simulation or configuration semantics is exactly what may travel that way.
//
// INPUT DOMAIN — the colour handed in must ALREADY be sRGB-encoded, and that is load-bearing
// rather than a convenience:
//
//  * Both laws' outputs pass through the same LinearToSrgb before reaching 8-bit
//    (src/server/render.cpp's clamp/gamma tail). In that encoded space the far end of each law is
//    exactly representable — LinearToSrgb(1) == 1 and LinearToSrgb(0) == 0 hold precisely — so
//    `margin` below is the exact 8-bit distance the brightest possible feature could ever open up
//    against the zero-energy colour, not an approximation of it.
//  * Measured in LINEAR space the same subtraction OVERSTATES what is visible, because the sRGB
//    curve compresses the highlights (its slope near linear 1.0 is about 0.44). A predicate built
//    on `1 - background_linear` therefore stays quiet on configurations that really are degenerate.
//    The two spellings are not two conventions; one of them under-reports.
//
// This function does NOT convert, on purpose: conversion is the caller's, which keeps one job per
// function and matches how the GUI and the CLI actually hold these fields. src/gui/'s
// RenderConfig::background / ::paper are sRGB already (gui_state.hpp) and call straight in; core's
// RenderConfig::background_ / ::paper_ are linear (src/config/render_config.hpp) and are encoded
// per channel with util/color_space.hpp's LinearToSrgb on the way in.
// =================================================================================================

// Which end of its law a tone operator drives towards — i.e. where the headroom is measured TO.
// A named pair rather than a bare bool so `ToneLawLimit::kWhite` reads at the call site, and so a
// third operator would have to be added here rather than squeezed into a two-valued type.
enum class ToneLawLimit {
  kWhite,  // additive / screen: out = clamp(L * c + background), saturating at 1
  kBlack,  // subtractive / print: out = paper * 10^(-D), asymptotic to 0
};

// How close to the law's far end the headroom may get before the configuration is called out, in
// 8-bit levels of the sRGB-encoded output.
//
// WHY TEN — the chain, which is an engineering inference of this task and NOT an owner decision
// (doc/print-mode-subtractive-ink.md's decision table carries no number for it; §11.2's discipline
// applies, so treat this as questionable and say so if evidence disagrees):
//
//  1. `margin` is an UPPER BOUND on contrast, not a typical value. A real feature only ever
//     approaches its law's far end and never reaches it — screen's clamp saturates at infinite
//     radiance, print's 10^(-D) reaches 0 only as D -> inf. So whatever the simulation computes,
//     nothing in the finished image can be further than `margin` from the zero-energy colour.
//  2. If that bound is itself under 10/255, then no feature — however bright, however dense —
//     can separate from its ground by more than roughly ten 8-bit levels.
//  3. Ten levels is below what survives the ordinary handling a finished halo image gets (PNG
//     re-compression, an uncalibrated display, a social-media re-encode), and below what a
//     LARGE, LOW-SPATIAL-FREQUENCY feature needs to be seen at all — a halo or an arc is much
//     harder to pick out at a given contrast than a hard edge of the same contrast is.
//  4. Ten rather than the 2-3 a just-noticeable-difference argument would give: the point is to
//     speak while the image can still be RESCUED (switching tone, darkening the sky, lightening
//     the paper all still produce a visible improvement), not to confirm it is already unusable.
//
// Single point of definition, so adjusting it is one edit and no call site is involved.
inline constexpr int kContrastHeadroomWarnLevels = 10;
inline constexpr float kContrastHeadroomWarnMargin = static_cast<float>(kContrastHeadroomWarnLevels) / 255.0f;

// Distance from an sRGB-encoded zero-energy colour to the far end of its tone law, in [0, 1].
//
// Per channel, then the MINIMUM across channels rather than a luminance — doc §8's "逐通道取值、
// 跨通道取最小", and the reason is that the structure goes first in whichever channel runs out
// first. A coloured paper or a coloured sky is exactly where the two differ: a deep blue paper has
// plenty of luminance headroom left in blue while red and green are already on the floor, and the
// red and green halves of the picture are already gone there. A luminance measure would weight that
// away (Y is 0.07 blue) and stay silent.
inline float ContrastHeadroomMargin(const float srgb_zero_energy_color[3], ToneLawLimit limit) {
  float margin = 1.0f;
  for (int j = 0; j < 3; j++) {
    // Clamped rather than trusted: these fields come from a JSON document and from a colour
    // picker, and an out-of-range component would otherwise report a negative or >1 headroom —
    // which, being compared against a small positive threshold, fails in the silent direction.
    const float c = std::clamp(srgb_zero_energy_color[j], 0.0f, 1.0f);
    margin = std::min(margin, limit == ToneLawLimit::kWhite ? 1.0f - c : c);
  }
  return margin;
}

// Is this zero-energy colour close enough to its law's far end to be worth telling the user about?
//
// Strictly `<`: a configuration sitting exactly on the threshold is not yet called out, which makes
// the boundary a fact of this function rather than of whichever caller asks first.
inline bool ContrastHeadroomIsLow(const float srgb_zero_energy_color[3], ToneLawLimit limit) {
  return ContrastHeadroomMargin(srgb_zero_energy_color, limit) < kContrastHeadroomWarnMargin;
}

}  // namespace lumice
