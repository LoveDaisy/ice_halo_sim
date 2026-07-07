#ifndef CONFIG_COLOR_CLASS_TABLE_H_
#define CONFIG_COLOR_CLASS_TABLE_H_

#include <cstdint>
#include <vector>

namespace lumice {

struct ComponentTable;
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

// Ordered color-class list; list order = z-order for the 339.4 per-class
// compositor. `referenced_mask_` is the OR of all classes' `member_bits_` —
// used by the server to decide whether to allocate per-color-class Y-lanes,
// and as the compositor's early-exit gate.
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

// Structural equality for consumer-rebuild eligibility (task-339.3 decision 1):
// compares z-order-preserving (combine_, member_bits_) per class — NOT just the
// OR'd referenced_mask_, since two configs can share referenced_mask_ while
// splitting/merging classes differently (same total bits, different lane count
// / shape). Returns true when the RenderConsumer's per-class lane layout must
// be rebuilt from scratch. `color_`/`visible_`/`solo_` do not affect lane
// layout (they only steer the display-time compositor) and are refreshed
// unconditionally by CommitConfig, so they intentionally do not participate.
//
// Mirrors the placement convention of `NeedsRebuild(const RenderConfig&,
// const RenderConfig&)` (`src/config/render_config.{hpp,cpp}`) — same name, same
// "compared type owns the predicate" home.
bool NeedsRebuild(const ColorClassTable& old_table, const ColorClassTable& new_table);

}  // namespace lumice

#endif  // CONFIG_COLOR_CLASS_TABLE_H_
