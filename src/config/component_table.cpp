#include "config/component_table.hpp"

#include <cstddef>
#include <variant>

#include "config/filter_config.hpp"
#include "config/proj_config.hpp"
#include "util/logger.hpp"

namespace lumice {

namespace {

// Return the number of OR-summands the given filter contributes to the
// component table, following the plan §2 rules:
//   - NoneFilterParam       → 1 (whole-crystal virtual summand for the
//                             raypath-color engine's {layer, crystal} ref
//                             resolution — see task-339.1; the summand's
//                             per-ray mask is emitted by NoneSpec's default
//                             MatchSummandMask which is always 0b1)
//   - other simple filters  → 1
//   - ComplexFilterParam    → filters_.size()
size_t CountSummands(const FilterParam& param) {
  return std::visit(
      [](const auto& p) -> size_t {
        using T = std::decay_t<decltype(p)>;
        if constexpr (std::is_same_v<T, SimpleFilterParam>) {
          return 1;
        } else if constexpr (std::is_same_v<T, ComplexFilterParam>) {
          return p.filters_.size();
        } else {
          return 0;
        }
      },
      param);
}

}  // namespace

ComponentTable BuildComponentTable(const SceneConfig& scene) {
  ComponentTable table;
  size_t next_bit = 0;
  size_t overflow_count = 0;

  for (size_t mi = 0; mi < scene.ms_.size(); mi++) {
    const auto& layer = scene.ms_[mi];
    for (size_t ci = 0; ci < layer.setting_.size(); ci++) {
      const auto& setting = layer.setting_[ci];
      size_t summand_count = CountSummands(setting.filter_.param_);
      for (size_t sk = 0; sk < summand_count; sk++) {
        ComponentTableEntry entry{};
        entry.layer_ = static_cast<IdType>(mi);
        entry.crystal_id_ = static_cast<IdType>(ci);
        entry.summand_idx_ = static_cast<IdType>(sk);
        if (next_bit < ComponentTable::kMaxBits) {
          entry.bit_ = static_cast<uint8_t>(next_bit);
          next_bit++;
        } else {
          entry.bit_ = ComponentTable::kNoBit;
          overflow_count++;
        }
        table.entries_.push_back(entry);
      }
    }
  }

  if (overflow_count > 0) {
    // Global logger — the table builder is called from simulator entry points
    // (both legacy and backend paths) and neither hands us a per-scene Logger.
    // The soft-cap warning is expected to be rare (>64 total summands across
    // all layers and crystals) and mostly diagnostic.
    LOG_WARNING(
        "ComponentTable: {} summand(s) exceeded the {}-bit component budget and were "
        "assigned kNoBit (phase-1: no runtime consumer yet)",
        overflow_count, ComponentTable::kMaxBits);
  }

  return table;
}

std::vector<uint8_t> ComponentBitsFor(const ComponentTable& table, IdType layer, IdType crystal_id) {
  std::vector<uint8_t> bits;
  for (const auto& e : table.entries_) {
    if (e.layer_ != layer || e.crystal_id_ != crystal_id) {
      continue;
    }
    if (e.summand_idx_ >= bits.size()) {
      bits.resize(static_cast<size_t>(e.summand_idx_) + 1, ComponentTable::kNoBit);
    }
    bits[e.summand_idx_] = e.bit_;
  }
  return bits;
}

}  // namespace lumice
