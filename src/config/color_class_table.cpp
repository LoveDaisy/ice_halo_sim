#include "config/color_class_table.hpp"

#include <stdexcept>
#include <string>

#include "config/color_gate_table.hpp"
#include "config/component_table.hpp"
// config_compare.hpp provides operator== for SimpleFilterParam's alternatives;
// std::variant's operator== forwards to them. clangd's include-cleaner misses
// the transitive dependency, hence the explicit disable below.
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

// Resolve a Design-2 ref to its component bit by lookup in the ColorGateTable.
// Both callers (server.cpp CommitConfig and any future direct consumer) build
// the same gate table from the same config, so this lookup is guaranteed to
// find a match when the ref is well-formed. Returns ComponentTable::kNoBit
// when the gate table assigned kNoBit to this predicate (bit-budget overflow);
// throws only on the defensive "no matching entry" case that should be
// unreachable.
uint8_t ResolveRefToBit(const RaypathColorRef& ref, const ColorGateTable& table) {
  for (const auto& e : table.entries_) {
    // Symmetry is part of the lookup key (matches BuildColorGateTable's dedup
    // key in scrum-color-predicate-symmetry): once the gate table splits refs
    // by symmetry into distinct bits, the class-table resolver must key on the
    // same tuple or a ref with non-default symmetry_ would silently resolve to
    // the first same-predicate entry (default symmetry) and inherit its bit —
    // directly violating AC2 (bit map preserved across symmetry).
    if (e.layer_ == ref.layer_ && e.crystal_id_ == ref.crystal_ && e.predicate_ == ref.predicate_ &&
        e.symmetry_ == ref.symmetry_) {
      return e.bit_;
    }
  }
  throw std::invalid_argument("raypath_color: internal — ref " + FormatRef(ref) +
                              " has no matching entry in the ColorGateTable (build/consume out of sync)");
}

}  // namespace

ColorClassTable BuildColorClassTable(const RaypathColorConfig& color_cfg, const SceneConfig& /*scene*/,
                                     const ColorGateTable& gate_table) {
  ColorClassTable out;
  out.classes_.reserve(color_cfg.classes_.size());

  for (const auto& cc : color_cfg.classes_) {
    ColorClass cls{};
    cls.color_[0] = cc.color_[0];
    cls.color_[1] = cc.color_[1];
    cls.color_[2] = cc.color_[2];
    // Default z_order_ = the vector position at construction time. Config JSON currently
    // carries no explicit z-order; without an override the compositor's ascending sort
    // yields identical iteration order to the pre-342.2 code (which used the vector
    // position directly). LUMICE_SetRaypathColors overrides this later at display-time.
    cls.z_order_ = static_cast<int>(out.classes_.size());
    if (cc.combine_ == "any") {
      cls.combine_ = ColorClassCombine::kAny;
    } else if (cc.combine_ == "all") {
      cls.combine_ = ColorClassCombine::kAll;
    } else {
      throw std::invalid_argument("raypath_color: unknown combine \"" + cc.combine_ +
                                  "\" (expected \"any\" or \"all\")");
    }
    cls.visible_ = cc.visible_;
    cls.solo_ = cc.solo_;

    size_t skipped = 0;
    for (const auto& ref : cc.match_) {
      uint8_t ref_bit = ResolveRefToBit(ref, gate_table);
      if (ref_bit == ComponentTable::kNoBit) {
        // Predicate overflowed the 64-bit budget in BuildColorGateTable — kept
        // in the class but contributes 0 (a single WARNING was already logged
        // by the gate-table builder; no need to double-warn here).
        ++skipped;
        continue;
      }
      cls.member_bits_ |= (static_cast<uint64_t>(1) << ref_bit);
    }

    if (skipped > 0) {
      LOG_WARNING("raypath_color: color class skipped {} ref(s) that overflowed the {}-bit component budget", skipped,
                  ComponentTable::kMaxBits);
    }
    if (cls.member_bits_ == 0) {
      // Class kept to preserve z-order; contributes 0 to referenced_mask_.
      // Predicate never fires (any: 0 → false; all: all-bits-in-empty-set is
      // trivially satisfied but member_bits_==0 masks to 0 → never contributes
      // energy). One-shot warn per class is loud enough here.
      LOG_WARNING(
          "raypath_color: color class has no resolvable component bits (empty match or all predicates "
          "overflowed the {}-bit budget); class retained to preserve z-order but will contribute no color",
          ComponentTable::kMaxBits);
    }
    out.referenced_mask_ |= cls.member_bits_;
    out.classes_.push_back(cls);
  }

  return out;
}

bool NeedsRebuild(const ColorClassTable& old_table, const ColorClassTable& new_table) {
  if (old_table.classes_.size() != new_table.classes_.size()) {
    return true;
  }
  for (size_t i = 0; i < old_table.classes_.size(); ++i) {
    const auto& a = old_table.classes_[i];
    const auto& b = new_table.classes_[i];
    if (a.combine_ != b.combine_ || a.member_bits_ != b.member_bits_) {
      return true;
    }
  }
  return false;
}

}  // namespace lumice
