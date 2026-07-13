#include "config/color_gate_table.hpp"

#include <stdexcept>
#include <string>

#include "config/component_table.hpp"
// config_compare.hpp provides operator== for each SimpleFilterParam
// alternative (NoneFilterParam / RaypathFilterParam / EntryExitFilterParam /
// DirectionFilterParam / CrystalFilterParam). std::variant's default operator==
// forwards to these; clangd's unused-includes check misses this transitive
// dependency, so the include is required despite the warning.
// NOLINTNEXTLINE(misc-include-cleaner)
#include "config/config_compare.hpp"
#include "config/proj_config.hpp"
#include "config/raypath_color_config.hpp"
#include "util/logger.hpp"

namespace lumice {

namespace {

std::string FormatRef(const RaypathColorRef& r) {
  return "(layer=" + std::to_string(r.layer_) + ", crystal_id=" + std::to_string(r.crystal_) + ")";
}

// Verify (ref.layer_, ref.crystal_) is unambiguous — Design 2 §3.2 decision 2(b)
// throw-on-ambiguity. Returns nothing (side-effect: throws on ill-formed refs);
// callers do NOT need the resolved ci index (predicate evaluation runs against
// the physically-parallel color_spec, keyed by CrystalConfig::id_, not by ci).
void VerifyRefPlacementIsUnambiguous(const RaypathColorRef& ref, const SceneConfig& scene) {
  if (ref.layer_ >= scene.ms_.size()) {
    throw std::invalid_argument("raypath_color: layer index out of range " + FormatRef(ref));
  }
  const auto& layer = scene.ms_[ref.layer_];
  size_t match_count = 0;
  for (const auto& setting : layer.setting_) {
    if (setting.crystal_.id_ == ref.crystal_) {
      ++match_count;
    }
  }
  if (match_count == 0) {
    throw std::invalid_argument("raypath_color: no scattering setting with crystal_id " + std::to_string(ref.crystal_) +
                                " on layer " + std::to_string(ref.layer_) + " " + FormatRef(ref));
  }
  if (match_count >= 2) {
    throw std::invalid_argument(
        "raypath_color: crystal_id " + std::to_string(ref.crystal_) + " matches " + std::to_string(match_count) +
        " scattering settings on layer " + std::to_string(ref.layer_) +
        " — Design 2 requires per-placement crystal_id to be unique; use distinct CrystalConfig::id_s (geometry may "
        "still be identical). " +
        FormatRef(ref));
  }
}

}  // namespace

ColorGateTable BuildColorGateTable(const RaypathColorConfig& color_cfg, const SceneConfig& scene) {
  ColorGateTable table;
  size_t next_bit = 0;
  size_t overflow_count = 0;

  for (const auto& cc : color_cfg.classes_) {
    for (const auto& ref : cc.match_) {
      VerifyRefPlacementIsUnambiguous(ref, scene);

      // Structural dedup: same (layer, crystal_id, predicate, symmetry) reuses
      // the same bit. Symmetry is part of the dedup key because two refs that
      // share predicate literal but differ in symmetry match different
      // equivalence classes of raypaths (semantically distinct color-matching
      // criteria) and must occupy separate bits — otherwise two color classes
      // would "contaminate" each other's hits. Deliberate O(N*M) linear scan —
      // bit budget is capped at 64, so this loop is tiny.
      bool duplicate = false;
      for (const auto& existing : table.entries_) {
        if (existing.layer_ == ref.layer_ && existing.crystal_id_ == ref.crystal_ &&
            existing.predicate_ == ref.predicate_ && existing.symmetry_ == ref.symmetry_) {
          duplicate = true;
          break;
        }
      }
      if (duplicate) {
        continue;
      }

      ColorGateEntry entry{};
      entry.layer_ = ref.layer_;
      entry.crystal_id_ = ref.crystal_;
      entry.predicate_ = ref.predicate_;
      entry.symmetry_ = ref.symmetry_;
      if (next_bit < ComponentTable::kMaxBits) {
        entry.bit_ = static_cast<uint8_t>(next_bit);
        ++next_bit;
      } else {
        entry.bit_ = ComponentTable::kNoBit;
        ++overflow_count;
      }
      table.entries_.push_back(std::move(entry));
    }
  }

  if (overflow_count > 0) {
    LOG_WARNING("ColorGateTable: {} predicate(s) exceeded the {}-bit component budget and were assigned kNoBit",
                overflow_count, ComponentTable::kMaxBits);
  }
  return table;
}

ColorGatePlacement ColorGatePlacementFor(const ColorGateTable& table, IdType layer, IdType crystal_id) {
  ColorGatePlacement out;
  for (const auto& e : table.entries_) {
    if (e.layer_ != layer || e.crystal_id_ != crystal_id) {
      continue;
    }
    out.predicates_.push_back(e.predicate_);
    out.bits_.push_back(e.bit_);
    out.symmetries_.push_back(e.symmetry_);
  }
  return out;
}

ColorPlacementGrouping GroupPlacementBySymmetry(const ColorGatePlacement& placement) {
  ColorPlacementGrouping grouping;
  size_t n = placement.predicates_.size();
  grouping.group_of_.assign(n, 0);
  grouping.group_symmetry_.reserve(n);
  for (size_t k = 0; k < n; ++k) {
    uint8_t sym = placement.symmetries_[k];
    size_t gi = grouping.group_symmetry_.size();
    for (size_t i = 0; i < grouping.group_symmetry_.size(); ++i) {
      if (grouping.group_symmetry_[i] == sym) {
        gi = i;
        break;
      }
    }
    if (gi == grouping.group_symmetry_.size()) {
      grouping.group_symmetry_.push_back(sym);
    }
    grouping.group_of_[k] = gi;
  }
  return grouping;
}

}  // namespace lumice
