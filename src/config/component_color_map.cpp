#include "config/component_color_map.hpp"

#include <stdexcept>
#include <string>

#include "config/component_table.hpp"
#include "config/raypath_color_config.hpp"
#include "util/logger.hpp"

namespace lumice {

namespace {

std::string FormatTriple(IdType layer, IdType crystal_id, IdType summand_idx) {
  return "(layer=" + std::to_string(layer) + ", crystal_slot=" + std::to_string(crystal_id) +
         ", summand=" + std::to_string(summand_idx) + ")";
}

const ComponentTableEntry* FindEntry(const ComponentTable& table, const RaypathColorEntry& e) {
  for (const auto& te : table.entries_) {
    if (te.layer_ == e.layer_ && te.crystal_id_ == e.crystal_id_ && te.summand_idx_ == e.summand_idx_) {
      return &te;
    }
  }
  return nullptr;
}

}  // namespace

ComponentColorMap BuildComponentColorMap(const RaypathColorConfig& color_cfg, const ComponentTable& table) {
  ComponentColorMap map;
  for (const auto& e : color_cfg.entries_) {
    const auto* entry = FindEntry(table, e);
    if (entry == nullptr) {
      throw std::invalid_argument("raypath_color: no ComponentTable entry for " +
                                  FormatTriple(e.layer_, e.crystal_id_, e.summand_idx_));
    }
    if (entry->bit_ == ComponentTable::kNoBit) {
      // Deliberately distinct wording from BuildComponentTable's own overflow
      // warning ("summand(s) exceeded the {}-bit component budget") — this
      // one flags a color config referencing an already-overflowed summand,
      // not the table overflow itself.
      LOG_WARNING(
          "raypath_color: color config references summand {} that overflowed the {}-bit component budget; skipping",
          FormatTriple(e.layer_, e.crystal_id_, e.summand_idx_), ComponentTable::kMaxBits);
      continue;
    }
    map.colors_[entry->bit_][0] = e.color_[0];
    map.colors_[entry->bit_][1] = e.color_[1];
    map.colors_[entry->bit_][2] = e.color_[2];
    map.colored_mask_ |= (static_cast<uint64_t>(1) << entry->bit_);
  }
  return map;
}

}  // namespace lumice
