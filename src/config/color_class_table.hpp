#ifndef CONFIG_COLOR_CLASS_TABLE_H_
#define CONFIG_COLOR_CLASS_TABLE_H_

#include <cstdint>
#include <vector>

namespace lumice {

struct ComponentTable;
struct ComponentColorMap;
struct RaypathColorConfig;
struct SceneConfig;

enum class ColorClassCombine { kAny, kAll };

// One runtime color class: an RGB, a boolean combine over the flat member-bits
// set, and per-class display-time visibility. `member_bits_` is the OR-union
// of each ref's resolved bits (task-339.2 plan §4.1). Predicate against a
// per-ray component mask (task-339.3 will consume this):
//   any: (ray_mask & member_bits_) != 0
//   all: (ray_mask & member_bits_) == member_bits_
struct ColorClass {
  float color_[3]{};
  ColorClassCombine combine_ = ColorClassCombine::kAny;
  bool visible_ = true;
  bool solo_ = false;
  uint64_t member_bits_ = 0;
};

// Ordered color-class list; list order = z-order for 339.4's per-class
// compositor. `referenced_mask_` is the OR of all classes' `member_bits_` —
// used by the server to decide whether to allocate per-component Y-lanes and
// as the RenderConsumer's `colored_mask` input (drop-in replacement for
// 336.1's ComponentColorMap::colored_mask_).
struct ColorClassTable {
  std::vector<ColorClass> classes_;
  uint64_t referenced_mask_ = 0;
};

// Pure function: 339.2 color-class DTO + scene (for id→ci resolution) + config
// -time component table → runtime color-class table.
//
// Errors (all std::invalid_argument, caught by server → Error::InvalidConfig):
//   - unknown `combine` string;
//   - out-of-range layer;
//   - crystal/filter id has no matching (crystal.id_, filter.id_) pair in
//     scene.ms_[layer].setting_[];
//   - degenerate duplicate: more than one setting with the same
//     (crystal.id_, filter.id_) pair on the same layer;
//   - explicit summand out of range for the referenced filter;
//   - combine:"all" class with an omit-summand ref that resolves to >1 bits
//     (decision 4 ban — keeps the flat member_bits_ representation safe for
//     cross-layer AND). Error message carries the ref quadruple + remediation.
// Warnings (via LOG_WARNING, non-fatal):
//   - a ref hits kNoBit (>64-summand overflow): skip that bit;
//   - a class ends up with member_bits_ == 0 (empty match_ or all bits kNoBit):
//     class is kept in the list to preserve z-order but contributes 0 to
//     `referenced_mask_` and is a no-op at predicate time.
ColorClassTable BuildColorClassTable(const RaypathColorConfig& color_cfg, const SceneConfig& scene,
                                     const ComponentTable& table);

// Transitional adapter to the 336.1 per-bit runtime shape (used by the current
// consumer until 339.3 rewrites it per-class). Each class paints its color into
// `colors_[bit]` for every bit in `member_bits_`. For the migrated e2e fixture
// (all single-member classes) this is bit-for-bit equivalent to 336.1's
// BuildComponentColorMap. Multi-member "any" classes are approximated per-bit;
// per-class z-order and overlap upgrades ship in 339.4 (see plan R7).
//
// The composite-options adapter (ToLegacyCompositeOptions) lives in the SERVER
// layer (server/component_compositor.hpp) because CompositeOptions/CompositeMode
// are server concepts — config must not reverse-depend on server.
ComponentColorMap ToLegacyColorMap(const ColorClassTable& class_table);

}  // namespace lumice

#endif  // CONFIG_COLOR_CLASS_TABLE_H_
