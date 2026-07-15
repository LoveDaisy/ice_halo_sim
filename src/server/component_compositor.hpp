#ifndef SERVER_COMPONENT_COMPOSITOR_H_
#define SERVER_COMPONENT_COMPOSITOR_H_

#include <cstdint>
#include <string>
#include <vector>

namespace lumice {

class RenderConsumer;
struct ColorClassTable;

// task-339.4: how per-color-class Y-lanes are combined into one displayed color
// per pixel (see doc/gui-custom-spectrum-and-raypath-color.md §4.7 + §4.8).
// dominant / additive share the SINGLE mono-image exposure scale
// (RenderConsumer::ExposureScale()) — there is never a per-lane / per-composite
// renormalization (that was the spike's false-color bug). painter's exposure
// path is decoupled (§4.8): alpha uses the self-anchor A only,
// display_exposure_scale is a post-composite brightness multiplier.
enum class CompositeMode { kDominant, kAdditive, kPainter };

// Parse the RaypathColorConfig::mode_ string ("dominant" / "additive" /
// "painter") into a CompositeMode. Any other value falls back to kPainter with
// a one-shot LOG_WARNING (task-painter-alpha-over-composite: default is now
// painter; §4.8).
CompositeMode ParseCompositeMode(const std::string& mode_str);

// Composite the consumer's per-color-class Y-lanes into a W*H*3 linear-RGB
// image. Classes are traversed in ascending z_order (list-first = top layer,
// see doc/gui-custom-spectrum-and-raypath-color.md §4.7). The self-anchor
//   `A = consumer.ParticipatingExposureScale(participating_p99_y)`
// is task-347 (Fix B): server-side self-anchored off the participating-P99 of
// the currently-visible class union (computed BEFORE the pixel loop, in-scope
// of the same call). Hiding a bright class therefore shrinks the participating
// set → lowers the anchor P99 → raises A → the remaining dim classes brighten
// in the same DoSnapshot, with no GUI auto-EV feedback round-trip.
// `display_exposure_scale` is an optional display-time GUI EV multiplier
// (default 1.0); one global scalar shared by every lane. Per-lane
// renormalization is the assumed-invalid "false-color" bug from the scrum-336
// spike and is structurally excluded here (a03 / doc §4.3). Modes:
//   - kDominant: `ey_c = laneY_c[p] * (A * display_exposure_scale)`; argmax
//     over c (strict >, ascending scan → tie goes to the class earlier in
//     the list), painted with `color_c * ey_c` (byte-for-byte pre-§4.8).
//   - kAdditive: `ey_c = laneY_c[p] * (A * display_exposure_scale)`;
//     Σ_c color_c × ey_c, per-channel clamped to [0,1] (byte-for-byte pre-§4.8).
//   - kPainter : Porter-Duff "over" front-to-back with `alpha_c = min(ey_c, 1)`
//     where `ey_c = laneY_c[p] * A` (self-anchor ONLY — display_exposure_scale
//     is NOT in the alpha path). The `over` colour slot holds the class's
//     pure hue `color_c` (not `color_c * ey_c` — reserving the brightness slot
//     for alpha avoids double-counting). Result is post-multiplied by
//     `display_exposure_scale` and clamped to [0,1]^3 per pixel. This is the
//     doc §4.8 redesign: fixes the pre-§4.8 "binary occluder" black-hole where
//     a dim top-layer class 100% masked bright classes below; painter output
//     is capped by `color_c` (no HDR headroom — a designed property, not a bug).
//
// Visibility:
//   - If any class has solo_ == true, the participating set = {c : classes_[c]
//     .solo_}.
//   - Otherwise the participating set = {c : classes_[c].visible_}.
// Overlap (a ray satisfying multiple classes → non-zero energy in each of their
// lanes) is resolved naturally by the mode over the per-class lanes — no
// dedicated overlap branch.
//
// out_participating_p99_y (optional, may be nullptr): P99 of the union of
// non-zero UNEXPOSED (raw) Y values across the participating classes' lanes
// — the anchor now consumed both by the internal ParticipatingExposureScale
// (self-anchor) AND by the C API surface (composite_p99_y, still reported so
// the GUI has a diagnostic view of the current anchor). Written iff non-null
// AND the class table has any resolvable bits AND at least one participating
// class exists; set to 0 when no participating class carries any positive Y.
// task-347 (Fix B) NOTE: with participating-P99 now computed BEFORE `s`,
// the exposure-scale early-return branch (`s <= 0`) also writes this value
// (equal to what the P99 pass produced) — a semantic tightening vs the
// pre-347 "left untouched on false return" behavior. All in-tree consumers
// (server.cpp DoSnapshot loops that skip false-return renderers, the C API
// mono path which explicitly writes 0, tests initializing to 0 before the
// call) are unaffected — the write-on-false path can only observe the
// tightening when the caller passes a non-null pointer AND ignores the
// return value AND does NOT pre-zero the field, which no consumer does.
//
// Returns false (leaving out_linear_rgb untouched) when the class table has no
// resolvable bits (referenced_mask_ == 0) or the participating-P99 anchor is
// zero / snapshot intensity is zero (no snapshot data / all lanes empty /
// every class hidden with no positive lane values). Otherwise resizes
// out_linear_rgb to W*H*3 and returns true — even when every class is hidden
// yielding a legitimately all-black composite (participating-P99==0 branch is
// promoted to false because the self-anchor cannot brighten a zero signal).
bool CompositeColorClassesLinear(const RenderConsumer& consumer, const ColorClassTable& class_table, CompositeMode mode,
                                 float display_exposure_scale, std::vector<float>& out_linear_rgb,
                                 float* out_participating_p99_y);
// Overload for callers that only need the pre-345.3 shape (display_exposure_scale=1.0,
// participating-P99 not requested). Delegates to the full form so behavior stays single-sourced.
inline bool CompositeColorClassesLinear(const RenderConsumer& consumer, const ColorClassTable& class_table,
                                        CompositeMode mode, std::vector<float>& out_linear_rgb) {
  return CompositeColorClassesLinear(consumer, class_table, mode, 1.0f, out_linear_rgb, nullptr);
}

// Convert a W*H*3 linear-RGB buffer (as produced above) to sRGB uint8, clamping
// to [0,1] before the sRGB transfer (mirrors PostSnapshot's final stage).
void LinearRgbToSrgbU8(const std::vector<float>& linear_rgb, std::vector<uint8_t>& out_srgb);

}  // namespace lumice

#endif  // SERVER_COMPONENT_COMPOSITOR_H_
