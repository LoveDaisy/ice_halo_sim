#ifndef CONFIG_COLOR_GATE_TABLE_H_
#define CONFIG_COLOR_GATE_TABLE_H_

// CPU-only, Design-2 (2026-07-08, doc/gui-custom-spectrum-and-raypath-color.md
// §4.0) placement-scoped color-predicate → component-bit table. Consumed by
// the CPU emit gate (simulator.cpp legacy path + cpu_trace_backend.cpp) as a
// non-destructive pass beside the physical filter check.
//
// The GPU backends (metal_trace_backend.mm, cuda_trace_backend.cu) still consume
// the Fork-C `ComponentTable` from `component_table.hpp`. Both tables coexist
// until scrum-3c retires the Fork-C GPU device gate. Do NOT conflate them:
// bit numberings are independent and semantically unrelated (one scans
// physical-filter summands, the other scans `raypath_color[].match[]` predicates).

#include <cstdint>
#include <vector>

#include "config/filter_config.hpp"
#include "core/def.hpp"

namespace lumice {

struct RaypathColorConfig;
struct SceneConfig;

// One entry per (layer, crystal_id, unique-predicate) tuple. `bit_` is the
// assigned component position in [0, 64) or the sentinel
// `ComponentTable::kNoBit` when the total unique-predicate count exceeds
// `ComponentTable::kMaxBits` (soft cap; overflow entries are recorded so
// UI/logging can surface warnings but they cannot participate in the uint64
// mask).
//
// `crystal_id_` is `CrystalConfig::id_` (NOT the setting_[] slot index) — the
// key the user writes in JSON. Bit assignment uses lexicographic
// (layer, crystal_id, predicate insertion) order — deterministic and
// idempotent (`BuildColorGateTable` is a pure function of its inputs).
struct ColorGateEntry {
  IdType layer_;
  IdType crystal_id_;
  uint8_t bit_;
  SimpleFilterParam predicate_;
};

struct ColorGateTable {
  std::vector<ColorGateEntry> entries_;
};

// Per-placement view for the CPU emit gate: predicates + parallel bit array
// for one (layer, crystal_id). Filled in insertion order (matches
// ColorGateTable.entries_).
struct ColorGatePlacement {
  std::vector<SimpleFilterParam> predicates_;
  std::vector<uint8_t> bits_;
};

// Build the CPU-only color-predicate → component-bit table for one commit.
//
// Semantics (see doc/gui-custom-spectrum-and-raypath-color.md §4.0):
//   - Walk color_cfg.classes_[].match_[]; for each ref, verify (layer,
//     crystal_id) is in scope and, per placement, resolve to EXACTLY ONE
//     scattering setting on that layer (§3.2 decision 2(b) — throw on
//     ambiguity to keep the Design-2 "physical-filter-independent" contract
//     honest; the alternative OR-across-settings silently changes
//     semantics on migrated fixtures).
//   - Within a (layer, crystal_id), dedupe predicates by structural equality
//     (SimpleFilterParam operator== from config_compare.hpp). Distinct
//     placements never share bits, even when the predicate structure is
//     equal (satisfies the placement-scoped承诺 #2 layer key).
//   - Assign bits in insertion order; overflow past `ComponentTable::kMaxBits`
//     gets `kNoBit` and a single warning log per build.
//
// Throws std::invalid_argument on:
//   - layer index out of range;
//   - crystal_id has 0 matches on the specified layer;
//   - crystal_id matches >= 2 settings on the specified layer (Design 2
//     forbids ambiguity — physically distinct placements must use distinct
//     `CrystalConfig::id_`s, even when the geometry is copied).
//
// The function is a pure fn of (color_cfg, scene) — deterministic and
// idempotent. Two independent invocations with equal inputs return equal
// outputs (asserted by test_color_gate_table's Idempotence test).
ColorGateTable BuildColorGateTable(const RaypathColorConfig& color_cfg, const SceneConfig& scene);

// Runtime helper: slice `table` by (layer, crystal_id) into a dense placement
// view (predicates + bits, both in insertion order). Missing (layer, crystal_id)
// yields an empty ColorGatePlacement (silently — the emit gate has nothing to
// do for that placement).
ColorGatePlacement ColorGatePlacementFor(const ColorGateTable& table, IdType layer, IdType crystal_id);

}  // namespace lumice

#endif  // CONFIG_COLOR_GATE_TABLE_H_
