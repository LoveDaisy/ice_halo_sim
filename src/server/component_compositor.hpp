#ifndef SERVER_COMPONENT_COMPOSITOR_H_
#define SERVER_COMPONENT_COMPOSITOR_H_

#include <cstdint>
#include <string>
#include <vector>

namespace lumice {

class RenderConsumer;
struct ColorClassTable;

// task-339.4: how per-color-class Y-lanes are combined into one displayed color
// per pixel (see doc/gui-custom-spectrum-and-raypath-color.md §4.7). All three
// modes share the SINGLE mono-image exposure scale
// (RenderConsumer::ExposureScale()) — there is never a per-lane / per-composite
// renormalization (that was the spike's false-color bug).
enum class CompositeMode { kDominant, kAdditive, kPainter };

// Parse the RaypathColorConfig::mode_ string ("dominant" / "additive" /
// "painter") into a CompositeMode. Any other value falls back to kDominant with
// a one-shot LOG_WARNING (matching the transitional adapter's behavior kept
// through 339.2/339.3).
CompositeMode ParseCompositeMode(const std::string& mode_str);

// Composite the consumer's per-color-class Y-lanes into a W*H*3 linear-RGB
// image. Classes are traversed in `class_table.classes_` order — the list order
// IS the z-order (see doc/gui-custom-spectrum-and-raypath-color.md §4.7). Per
// pixel, each participating class c contributes exposed value
// `ey_c = classLaneY_c[p] * s`, where
//   `s = consumer.ExposureScale() * display_exposure_scale`
// — the single mono-image exposure multiplied by an OPTIONAL display-time
// scalar (default 1.0 = pre-345.3 behavior). `display_exposure_scale` is one
// global scalar shared by every lane in every mode; per-lane renormalization
// is the assumed-invalid "false-color" bug from the scrum-336 spike and is
// structurally excluded here (a03 / doc §4.3). Modes:
//   - kDominant: argmax_c ey_c (strict >, ascending scan → tie goes to the
//     class earlier in the list), painted with that class's color.
//   - kAdditive: Σ_c color_c × ey_c, per-channel clamped to [0,1].
//   - kPainter : first class in list order with ey_c > 0 (list-first = top
//     layer), painted with that class's color.
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
// — the anchor the GUI's auto-EV feeds ComputeEvAuto with for composite
// display (mono path continues to use xyz_data-derived P99). Written iff
// non-null AND the function returns true; set to 0 when no participating
// class carries any positive Y.
//
// Returns false (leaving out_linear_rgb untouched) when the class table has no
// resolvable bits (referenced_mask_ == 0) or the exposure scale is 0 (no
// snapshot / zero intensity). Otherwise resizes out_linear_rgb to W*H*3 and
// returns true — even when every class is hidden (legitimately all-black).
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
