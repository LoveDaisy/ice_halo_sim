#ifndef CONFIG_COMPONENT_COLOR_MAP_H_
#define CONFIG_COMPONENT_COLOR_MAP_H_

#include <array>
#include <cstdint>

#include "config/component_table.hpp"

namespace lumice {

struct RaypathColorConfig;  // fwd — full definition in raypath_color_config.hpp

// Runtime lookup produced by joining a user-visible RaypathColorConfig with
// the config-time ComponentTable. bit i in `colored_mask_` is set iff
// `colors_[i]` carries a user-configured RGB for that component bit.
//
// Unset bits (colored_mask_ bit i = 0) are the responsibility of downstream
// composition (336.2/336.3) — this task does NOT decide how uncolored rays are
// displayed (background bucket / hidden / natural color), see plan §3.1(c).
struct ComponentColorMap {
  std::array<float[3], ComponentTable::kMaxBits> colors_{};
  uint64_t colored_mask_ = 0;
};

// Pure function: for each entry in `color_cfg`, linearly scan `table.entries_`
// for a triple match on (layer, crystal_id, summand_idx) and:
//
//   - No matching triple in the table → std::invalid_argument. This is a
//     config-author error (typo / out-of-range index) — fail fast, message
//     carries the offending triple.
//   - Matching entry but `entry.bit_ == ComponentTable::kNoBit` → LOG_WARNING
//     (once per offending entry) and skip. The soft-cap overflow is an
//     expected runtime-scale limit, not user input error; consistent with
//     BuildComponentTable's own overflow handling. Warning wording is
//     deliberately distinct from BuildComponentTable's overflow warning to
//     avoid the impression of duplicated logs.
//   - Matching entry with a valid bit → write `color_` into `colors_[bit]`
//     and set that bit in `colored_mask_`. Duplicate triples: last-write
//     wins (plan §3.3 — no warn / no throw; users see the wrong color in the
//     render).
//
// Empty `color_cfg` → default-constructed map (`colored_mask_ == 0`); the
// downstream lane consumer treats this as "no per-component color configured"
// and preserves pre-336 rendering behavior.
ComponentColorMap BuildComponentColorMap(const RaypathColorConfig& color_cfg, const ComponentTable& table);

// Shared (layer, crystal_id, summand_idx) → ComponentTableEntry lookup, the
// single source of truth used by both BuildComponentColorMap and the 336.3
// BuildCompositeOptions so the two agree bit-for-bit. Returns nullptr when no
// entry matches the triple. `core/def.hpp` (IdType) is transitively available
// via component_table.hpp.
const ComponentTableEntry* FindComponentTableEntry(const ComponentTable& table, IdType layer, IdType crystal_id,
                                                   IdType summand_idx);

}  // namespace lumice

#endif  // CONFIG_COMPONENT_COLOR_MAP_H_
