#ifndef SERVER_COMPONENT_COMPOSITOR_H_
#define SERVER_COMPONENT_COMPOSITOR_H_

#include <cstdint>
#include <string>
#include <vector>

namespace lumice {

class RenderConsumer;
struct ComponentColorMap;
struct ColorClassTable;

// task-336.3: how per-component Y-lanes are combined into one displayed color
// per pixel (see plan §4). All three modes share the SINGLE mono-image exposure
// scale (RenderConsumer::ExposureScale()) — there is never a per-lane /
// per-composite renormalization (that was the spike's false-color bug).
enum class CompositeMode { kDominant, kAdditive, kPainter };

// Runtime options for CompositeComponentLinear. Produced by
// ToLegacyCompositeOptions(ColorClassTable) after task-339.2's schema switch to
// color classes; the flat BuildCompositeOptions builder was retired alongside
// BuildComponentColorMap. 339.3/339.4 will consume ColorClassTable directly.
struct CompositeOptions {
  CompositeMode mode_ = CompositeMode::kDominant;
  uint64_t hidden_mask_ = 0;  // bit set → hide that component
  uint64_t solo_mask_ = 0;    // non-zero → restrict visible set to solo'd bits (overrides hide)
};

// Composite the consumer's per-component Y-lanes into a W*H*3 linear-RGB image.
//
// Each visible bit b contributes exposed lane value `ey_b = laneY_b[p] * s`
// where `s = consumer.ExposureScale()` — the single mono-image exposure. Modes:
//   - kDominant: argmax_b ey_b (strict >, ascending → min-bit tie), color × ey.
//   - kAdditive: Σ_b color_b × ey_b, per-channel clamped to [0,1].
//   - kPainter : first (lowest) visible bit with ey_b > 0, color × ey.
//
// Returns false (leaving out_linear_rgb untouched) when the consumer has no
// colored bits (colored_mask == 0) or the exposure scale is 0 (no snapshot /
// zero intensity). Otherwise resizes out_linear_rgb to W*H*3 and returns true —
// even when every bit is hidden (a legitimately all-black composite).
bool CompositeComponentLinear(const RenderConsumer& consumer, const ComponentColorMap& color_map,
                              const CompositeOptions& options, std::vector<float>& out_linear_rgb);

// Convert a W*H*3 linear-RGB buffer (as produced above) to sRGB uint8, clamping
// to [0,1] before the sRGB transfer (mirrors PostSnapshot's final stage).
void LinearRgbToSrgbU8(const std::vector<float>& linear_rgb, std::vector<uint8_t>& out_srgb);

// Transitional adapter (task-339.2): fold a ColorClassTable's per-class mode /
// visible / solo into the 336.1 per-bit CompositeOptions the current compositor
// consumes, until 339.4 rewrites the compositor per-class. Lives in the server
// layer (not config/color_class_table) because CompositeOptions/CompositeMode
// are server concepts — config must not reverse-depend on server (doc/
// architecture.md Config→Server one-way layering).
CompositeOptions ToLegacyCompositeOptions(const ColorClassTable& class_table, const std::string& mode);

}  // namespace lumice

#endif  // SERVER_COMPONENT_COMPOSITOR_H_
