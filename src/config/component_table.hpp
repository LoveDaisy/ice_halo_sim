#ifndef CONFIG_COMPONENT_TABLE_H_
#define CONFIG_COMPONENT_TABLE_H_

// GPU / Fork-C legacy component-bit source. As of the Design-2 redirect
// (2026-07-08, doc/gui-custom-spectrum-and-raypath-color.md §4.0) the CPU
// backends have switched to the placement-scoped
// `ColorGateTable` (see color_gate_table.hpp). This file remains untouched
// as the source of truth for the Metal/CUDA device gates until scrum-3c
// retires the Fork-C GPU path. Do NOT conflate the two: bit numberings are
// independent — this table scans physical-filter summands, `ColorGateTable`
// scans `raypath_color[].match[]` predicates.

#include <cstddef>
#include <cstdint>
#include <vector>

#include "core/def.hpp"

namespace lumice {

struct SceneConfig;  // fwd decl — full definition in config/proj_config.hpp

// Static (config-time) mapping of a raypath-color "summand" to a component bit
// index. Keyed by (layer, crystal, summand):
//
// - layer: index into `SceneConfig::ms_[]`.
// - crystal_id: index into `SceneConfig::ms_[layer].setting_[]` (the STATIC
//   config slot, NOT the runtime `RaySeg::crystal_idx_` — the latter is a
//   per-batch resampled instance index and unstable across batches when
//   crystals are randomised).
// - summand_idx: index into the OR-of-AND summand list carried by the
//   crystal's filter, following the counting rules in `BuildComponentTable`
//   below.
//
// bit_ is the assigned component position in [0, kMaxBits) or the sentinel
// `kNoBit` when the total summand count exceeds `kMaxBits`. Overflow entries
// are still recorded in `entries_` so downstream code can enumerate every
// summand (and, e.g., surface warnings in the UI) but they cannot participate
// in the uint64 mask.
struct ComponentTableEntry {
  IdType layer_;
  IdType crystal_id_;
  IdType summand_idx_;
  uint8_t bit_;
};

struct ComponentTable {
  // uint64 mask can hold at most 64 distinct bits. The plan (§2 default
  // assumptions) picks 64 as the soft cap; over-cap summands get kNoBit and
  // are logged once at construction, but no throw / no fatal.
  static constexpr uint8_t kNoBit = 0xFFu;
  static constexpr size_t kMaxBits = 64;

  std::vector<ComponentTableEntry> entries_;
};

// Pure function: walk `scene.ms_[mi].setting_[ci].filter_.param_` in
// (mi, ci, summand_idx) order and assign component bits in that lexicographic
// order.
//
// Summand counting rules (see plan §2 default assumptions — evaluated in
// this order via std::visit on FilterParam / SimpleFilterParam):
//   - NoneFilterParam       → 1 summand (whole-crystal virtual summand for
//                             the raypath-color engine's {layer, crystal}
//                             ref — task-339.1; the emitted per-ray mask is
//                             always 0b1 because NoneSpec::Match() is always
//                             true, so every ray traversing a None-filter
//                             crystal picks up this crystal's bit)
//   - Simple non-None       → 1 summand (the whole simple filter is a single
//                             summand)
//   - ComplexFilterParam    → filters_.size() summands (each OR-summand is
//                             one entry; the AND-chain inside a summand does
//                             not fan out further)
//
// The function has no I/O, no RNG, no CPU-heavy work — safe to call at every
// simulation entry point. Phase-1 leaves the returned table unconsumed on the
// hot path; T2 will start using it to look up per-summand bits when producing
// masks in `CollectData`.
ComponentTable BuildComponentTable(const SceneConfig& scene);

// Runtime lookup helper (task-331.2): collect the summand -> component-bit
// mapping for one (layer, crystal-slot) as a dense vector indexed by
// summand_idx. Entry values are the assigned bit index in [0, kMaxBits) or
// `ComponentTable::kNoBit` for over-budget summands. A summand index absent
// from the table (should not happen for a well-formed table) also reads
// kNoBit. Missing (layer, crystal_id) -> empty vector (e.g. a crystal-slot
// index that does not exist in the scene). None-filter crystals return a
// 1-element vector (their whole-crystal bit) after task-339.1.
//
// The CPU emit gate calls this once per (layer, crystal-slot) and uses it to
// map a FilterSpec's per-summand match mask (FilterSpec::CheckSummandMask)
// onto global component bits. Keyed by the STATIC setting-slot index — see the
// crystal_id_ note on ComponentTableEntry above.
std::vector<uint8_t> ComponentBitsFor(const ComponentTable& table, IdType layer, IdType crystal_id);

}  // namespace lumice

#endif  // CONFIG_COMPONENT_TABLE_H_
