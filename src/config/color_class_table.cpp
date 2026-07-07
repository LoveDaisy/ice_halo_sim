#include "config/color_class_table.hpp"

#include <stdexcept>
#include <string>
#include <variant>

#include "config/component_table.hpp"
#include "config/filter_config.hpp"
#include "config/proj_config.hpp"
#include "config/raypath_color_config.hpp"
#include "util/logger.hpp"

namespace lumice {

namespace {

bool IsNoneFilter(const FilterConfig& f) {
  // NoneFilterParam is the inner variant alt of SimpleFilterParam (not of the
  // outer FilterParam). Requires two-level unwrap (plan §4.3 M2). Type-based
  // check (vs. id_ == kInvalidId) tolerates a user-declared none filter that
  // carries a non-invalid id.
  if (!std::holds_alternative<SimpleFilterParam>(f.param_)) {
    return false;
  }
  const auto& simple = std::get<SimpleFilterParam>(f.param_);
  return std::holds_alternative<NoneFilterParam>(simple);
}

std::string FormatRef(const RaypathColorRef& r) {
  std::string s = "(layer=" + std::to_string(r.layer_) + ", crystal=" + std::to_string(r.crystal_);
  if (r.has_filter_) {
    s += ", filter=" + std::to_string(r.filter_);
  } else {
    s += ", filter=<none>";
  }
  if (r.has_summand_) {
    s += ", summand=" + std::to_string(r.summand_);
  }
  s += ")";
  return s;
}

// Locate the setting_[] slot ci in scene.ms_[layer] whose (crystal.id_,
// filter.id_) matches the ref. Throws on layer OOB, 0 matches, or duplicate
// (>=2) matches. Sets `out_ci` on success.
size_t ResolveRefToCi(const RaypathColorRef& ref, const SceneConfig& scene) {
  if (ref.layer_ >= scene.ms_.size()) {
    throw std::invalid_argument("raypath_color: layer index out of range " + FormatRef(ref));
  }
  const auto& layer = scene.ms_[ref.layer_];
  size_t match_ci = layer.setting_.size();  // sentinel: not found
  size_t match_count = 0;
  for (size_t ci = 0; ci < layer.setting_.size(); ++ci) {
    const auto& setting = layer.setting_[ci];
    if (setting.crystal_.id_ != ref.crystal_) {
      continue;
    }
    if (ref.has_filter_) {
      if (setting.filter_.id_ != ref.filter_) {
        continue;
      }
    } else {
      if (!IsNoneFilter(setting.filter_)) {
        continue;
      }
    }
    match_ci = ci;
    ++match_count;
  }
  if (match_count == 0) {
    throw std::invalid_argument("raypath_color: no matching scattering setting for " + FormatRef(ref));
  }
  if (match_count >= 2) {
    throw std::invalid_argument("raypath_color: multiple scattering settings match " + FormatRef(ref) +
                                " (degenerate duplicate (crystal,filter) pair)");
  }
  return match_ci;
}

}  // namespace

ColorClassTable BuildColorClassTable(const RaypathColorConfig& color_cfg, const SceneConfig& scene,
                                     const ComponentTable& table) {
  ColorClassTable out;
  out.classes_.reserve(color_cfg.classes_.size());

  for (const auto& cc : color_cfg.classes_) {
    ColorClass cls{};
    cls.color_[0] = cc.color_[0];
    cls.color_[1] = cc.color_[1];
    cls.color_[2] = cc.color_[2];
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

    for (const auto& ref : cc.match_) {
      const size_t ci = ResolveRefToCi(ref, scene);
      const auto bits = ComponentBitsFor(table, ref.layer_, static_cast<IdType>(ci));

      uint64_t ref_bits = 0;
      if (ref.has_summand_) {
        if (ref.summand_ >= bits.size()) {
          throw std::invalid_argument("raypath_color: summand index out of range " + FormatRef(ref));
        }
        const uint8_t b = bits[ref.summand_];
        if (b == ComponentTable::kNoBit) {
          LOG_WARNING("raypath_color: ref {} maps to a summand that overflowed the {}-bit component budget; skipping",
                      FormatRef(ref), ComponentTable::kMaxBits);
        } else {
          ref_bits = (static_cast<uint64_t>(1) << b);
        }
      } else {
        // Omit-summand: OR-union of all summand bits of the referenced filter
        // (or the single none-filter whole-crystal bit — that's exactly
        // bits.size() == 1). kNoBit entries are skipped with a warning.
        size_t skipped = 0;
        for (const uint8_t b : bits) {
          if (b == ComponentTable::kNoBit) {
            ++skipped;
            continue;
          }
          ref_bits |= (static_cast<uint64_t>(1) << b);
        }
        if (skipped > 0) {
          LOG_WARNING("raypath_color: ref {} skipped {} summand(s) that overflowed the {}-bit component budget",
                      FormatRef(ref), skipped, ComponentTable::kMaxBits);
        }
        // Decision 4 ban: combine:"all" + omit-summand + >1 resolved bit → throw.
        // Keeps the flat member_bits_ representation semantically correct for
        // cross-layer AND (each ref must resolve to exactly one bit). Portable
        // popcount (MSVC-safe: no __builtin_popcountll).
        if (cls.combine_ == ColorClassCombine::kAll) {
          int popcount = 0;
          for (uint64_t m = ref_bits; m != 0; m &= (m - 1)) {
            ++popcount;
          }
          if (popcount > 1) {
            throw std::invalid_argument(
                "raypath_color: combine:\"all\" class contains ref " + FormatRef(ref) + " that resolves to " +
                std::to_string(popcount) +
                " bits (omit-summand on multi-summand filter); use an explicit summand or combine:\"any\"");
          }
        }
      }

      cls.member_bits_ |= ref_bits;
    }

    if (cls.member_bits_ == 0) {
      // Class kept to preserve z-order; contributes 0 to referenced_mask_.
      // Predicate never fires (any: 0 → false; all: all-bits-in-empty-set is
      // trivially satisfied but member_bits_==0 masks to 0 → never contributes
      // energy). One-shot warn per class is loud enough here.
      LOG_WARNING(
          "raypath_color: color class has no resolvable component bits (empty match or all summands "
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
