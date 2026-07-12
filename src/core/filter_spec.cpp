#include "core/filter_spec.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <optional>
#include <utility>
#include <variant>
#include <vector>

#include "config/color_gate_table.hpp"
#include "config/filter_config.hpp"
#include "core/crystal.hpp"
#include "core/def.hpp"
#include "core/math.hpp"
#include "core/raypath.hpp"

namespace lumice {

namespace {

constexpr int kFnPeriodHex = 6;

// In-place P-canonical shift on the recorder buffer. Mirrors Crystal::PCanonicalShift
// for hexagonal crystals (fn_period=6).
void PCanonicalShiftInPlace(uint8_t* data, size_t size) {
  int first_pri = -1;
  for (size_t i = 0; i < size; i++) {
    uint8_t x = data[i];
    if (x < 3) {
      continue;
    }
    uint8_t pyr = x / 10;
    int pri = static_cast<int>(x % 10);
    if (first_pri < 0) {
      first_pri = pri;
    }
    pri = (pri + kFnPeriodHex - first_pri) % kFnPeriodHex + 3;
    data[i] = static_cast<uint8_t>(pyr * 10 + pri);
  }
}

bool LexLessRecorder(const uint8_t* a, const uint8_t* b, size_t size) {
  for (size_t i = 0; i < size; i++) {
    if (a[i] != b[i]) {
      return a[i] < b[i];
    }
  }
  return false;
}

}  // namespace

namespace detail {

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void ReduceBuffer(uint8_t* data, size_t size, uint8_t symmetry, int sigma_a, bool d_applicable) {
  if (symmetry == FilterConfig::kSymNone) {
    return;
  }

  if (symmetry & FilterConfig::kSymP) {
    PCanonicalShiftInPlace(data, size);
  }

  if ((symmetry & FilterConfig::kSymD) && d_applicable) {
    uint8_t scratch[kMaxHits]{};
    for (size_t i = 0; i < size; i++) {
      uint8_t x = data[i];
      if (x < 3) {
        scratch[i] = x;
        continue;
      }
      uint8_t pyr = x / 10;
      int pri0 = static_cast<int>(x % 10) - 3;
      int new_pri0 = ((sigma_a - pri0) % kFnPeriodHex + kFnPeriodHex) % kFnPeriodHex;
      scratch[i] = static_cast<uint8_t>(pyr * 10 + new_pri0 + 3);
    }
    if (symmetry & FilterConfig::kSymP) {
      PCanonicalShiftInPlace(scratch, size);
    }
    if (LexLessRecorder(scratch, data, size)) {
      std::memcpy(data, scratch, size);
    }
  }

  if (symmetry & FilterConfig::kSymB) {
    uint8_t scratch[kMaxHits]{};
    bool changed = false;
    for (size_t i = 0; i < size; i++) {
      uint8_t x = data[i];
      if (x <= 2) {
        scratch[i] = static_cast<uint8_t>(3 - x);
        changed = true;
      } else if (x >= 13 && x <= 18) {
        scratch[i] = static_cast<uint8_t>(x + 10);
        changed = true;
      } else if (x >= 23 && x <= 28) {
        scratch[i] = static_cast<uint8_t>(x - 10);
        changed = true;
      } else {
        scratch[i] = x;
      }
    }
    if (changed && LexLessRecorder(scratch, data, size)) {
      std::memcpy(data, scratch, size);
    }
  }
}

}  // namespace detail

bool RaypathOrbit::Contains(const RaypathRecorder& rp_input, const uint8_t* overflow_arena) const {
  assert(!rp_input.HasOverflow() || overflow_arena != nullptr);
  if (rp_input.size_ != canonical_.size_) {
    return false;
  }
  // Resolve the raw hit buffer: arena slot for overflow recorders, otherwise
  // inline data_. Inline-only callers (EntryExitSpec ee, unit tests) pass
  // overflow_arena=nullptr; if they ever feed an overflow recorder we crash
  // loudly here rather than silently misread inline bytes.
  const uint8_t* src =
      rp_input.HasOverflow() ? overflow_arena + static_cast<size_t>(rp_input.overflow_idx_) * kMaxHits : rp_input.data_;
  size_t size = rp_input.size_;

  if (fn_period_ < 0 || symmetry_ == FilterConfig::kSymNone) {
    return std::memcmp(src, canonical_.data_, size) == 0;
  }

  uint8_t scratch[kMaxHits]{};
  std::memcpy(scratch, src, size);
  detail::ReduceBuffer(scratch, size, symmetry_, sigma_a_, d_applicable_);
  return std::memcmp(scratch, canonical_.data_, size) == 0;
}

namespace {

RaypathOrbit BuildOrbit(const Crystal& crystal, const std::vector<IdType>& rp, uint8_t symmetry, int sigma_a,
                        bool d_applicable) {
  RaypathOrbit orbit;
  auto canonical_vec = crystal.ReduceRaypath(rp, symmetry, sigma_a, d_applicable);
  orbit.canonical_.Clear();
  for (auto fn : canonical_vec) {
    orbit.canonical_ << fn;
  }
  orbit.symmetry_ = symmetry;
  orbit.sigma_a_ = sigma_a;
  orbit.d_applicable_ = d_applicable;
  orbit.fn_period_ = crystal.FnPeriod();
  return orbit;
}

// task-339.1: None passes every ray AND now contributes a single whole-crystal
// component bit (summand_idx=0) so the raypath-color engine's {layer, crystal}
// ref resolves. The base-class MatchSummandMask default (mask = Match() ? 1 : 0)
// already produces the right semantics — Match() is always true, so mask is
// always 0b1 — so no override is needed here.
class NoneSpec : public FilterSpec {
 public:
  bool Match(const RaySeg& /*ray*/, const RaypathRecorder& /*rec*/, const uint8_t* /*arena*/) const override {
    return true;
  }
};

class RaypathSpec : public FilterSpec {
 public:
  RaypathSpec(const Crystal& crystal, const std::vector<IdType>& rp, uint8_t symmetry, int sigma_a, bool d_applicable)
      : orbit_(BuildOrbit(crystal, rp, symmetry, sigma_a, d_applicable)) {}

  bool Match(const RaySeg& /*ray*/, const RaypathRecorder& rec, const uint8_t* arena) const override {
    return orbit_.Contains(rec, arena);
  }

 private:
  RaypathOrbit orbit_;
};

class EntryExitSpec : public FilterSpec {
 public:
  EntryExitSpec(const Crystal& crystal, std::optional<IdType> entry, std::optional<IdType> exit, size_t min_len,
                std::optional<size_t> max_len, uint8_t symmetry, int sigma_a, bool d_applicable)
      : min_len_(min_len), max_len_(max_len), has_entry_(entry.has_value()), has_exit_(exit.has_value()),
        has_orbit_(has_entry_ || has_exit_),
        orbit_(BuildOrbitFromEnds(crystal, entry, exit, symmetry, sigma_a, d_applicable)) {}

  bool Match(const RaySeg& /*ray*/, const RaypathRecorder& rec, const uint8_t* arena) const override {
    size_t size = rec.size_;
    if (size == 0) {
      return false;
    }
    if (size < min_len_) {
      return false;
    }
    if (max_len_.has_value() && size > *max_len_) {
      return false;
    }
    if (!has_orbit_) {
      return true;  // both wildcard: length-only match
    }
    // Resolve first/last byte; they may live in the arena for overflow recorders.
    assert(!rec.HasOverflow() || arena != nullptr);
    const uint8_t* data = rec.HasOverflow() ? arena + static_cast<size_t>(rec.overflow_idx_) * kMaxHits : rec.data_;
    RaypathRecorder ee;
    ee.Clear();
    if (has_entry_ && has_exit_) {
      ee << static_cast<IdType>(data[0]);
      ee << static_cast<IdType>(data[size - 1]);
    } else if (has_entry_) {
      ee << static_cast<IdType>(data[0]);
    } else {
      ee << static_cast<IdType>(data[size - 1]);
    }
    // ee is inline-only by construction → arena=nullptr.
    return orbit_.Contains(ee, nullptr);
  }

 private:
  static RaypathOrbit BuildOrbitFromEnds(const Crystal& crystal, std::optional<IdType> entry,
                                         std::optional<IdType> exit, uint8_t symmetry, int sigma_a, bool d_applicable) {
    std::vector<IdType> rp;
    if (entry.has_value() && exit.has_value()) {
      rp.push_back(*entry);
      rp.push_back(*exit);
    } else if (entry.has_value()) {
      rp.push_back(*entry);
    } else if (exit.has_value()) {
      rp.push_back(*exit);
    }
    // For the double-wildcard case we return a default-constructed orbit that
    // is never consulted (has_orbit_ guards Match).
    if (rp.empty()) {
      return RaypathOrbit{};
    }
    return BuildOrbit(crystal, rp, symmetry, sigma_a, d_applicable);
  }

  size_t min_len_;
  std::optional<size_t> max_len_;
  bool has_entry_;
  bool has_exit_;
  bool has_orbit_;
  RaypathOrbit orbit_;
};

class DirectionSpec : public FilterSpec {
 public:
  DirectionSpec(float lon_deg, float lat_deg, float radii_deg)
      : d_{ std::cos(lat_deg * math::kDegreeToRad) * std::cos(lon_deg * math::kDegreeToRad),
            std::cos(lat_deg * math::kDegreeToRad) * std::sin(lon_deg * math::kDegreeToRad),
            std::sin(lat_deg * math::kDegreeToRad) },
        radii_c_(std::cos(radii_deg * math::kDegreeToRad)) {}

  bool Match(const RaySeg& ray, const RaypathRecorder& /*rec*/, const uint8_t* /*arena*/) const override {
    return Dot3(d_, ray.d_) > radii_c_;
  }

 private:
  float d_[3];
  float radii_c_;
};

class CrystalSpec : public FilterSpec {
 public:
  explicit CrystalSpec(IdType crystal_id) : crystal_id_(crystal_id) {}
  bool Match(const RaySeg& ray, const RaypathRecorder& /*rec*/, const uint8_t* /*arena*/) const override {
    return ray.crystal_config_id_ == crystal_id_;
  }

 private:
  IdType crystal_id_;
};

class ComplexSpec : public FilterSpec {
 public:
  ComplexSpec(const Crystal& crystal, const std::vector<std::vector<std::pair<IdType, SimpleFilterParam>>>& all_param,
              uint8_t symmetry, int sigma_a, bool d_applicable);

  bool Match(const RaySeg& ray, const RaypathRecorder& rec, const uint8_t* arena) const override {
    for (const auto& or_clause : filters_) {
      bool and_check = true;
      for (const auto& and_f : or_clause) {
        if (!and_f->Match(ray, rec, arena)) {
          and_check = false;
          break;
        }
      }
      if (and_check) {
        return true;
      }
    }
    return false;
  }

  // Evaluate EVERY OR-summand (no cross-clause short-circuit) so bit k reflects
  // whether summand k matched. The returned boolean (mask != 0) is bit-identical
  // to the short-circuit Match() above; the only difference is that all clauses
  // are evaluated so the per-summand information survives the collapse. Summand
  // index k here matches BuildComponentTable's summand_idx_ (both walk filters_
  // in order). Bits at k >= 64 cannot be represented and are dropped (a single
  // filter with > 64 OR-summands already exceeds the uint64 component budget).
  uint64_t MatchSummandMask(const RaySeg& ray, const RaypathRecorder& rec, const uint8_t* arena,
                            bool* out_matched) const override {
    uint64_t mask = 0;
    for (size_t k = 0; k < filters_.size() && k < 64; k++) {
      bool and_check = true;
      for (const auto& and_f : filters_[k]) {
        if (!and_f->Match(ray, rec, arena)) {
          and_check = false;
          break;
        }
      }
      if (and_check) {
        mask |= (1ull << k);
      }
    }
    if (out_matched != nullptr) {
      *out_matched = (mask != 0);
    }
    return mask;
  }

 private:
  std::vector<std::vector<std::unique_ptr<FilterSpec>>> filters_;
};

struct SimpleSpecCreator {
  const Crystal& crystal_;
  uint8_t symmetry_;
  int sigma_a_;
  bool d_applicable_;

  std::unique_ptr<FilterSpec> operator()(const NoneFilterParam& /*p*/) const { return std::make_unique<NoneSpec>(); }
  std::unique_ptr<FilterSpec> operator()(const RaypathFilterParam& p) const {
    return std::make_unique<RaypathSpec>(crystal_, p.raypath_, symmetry_, sigma_a_, d_applicable_);
  }
  std::unique_ptr<FilterSpec> operator()(const EntryExitFilterParam& p) const {
    return std::make_unique<EntryExitSpec>(crystal_, p.entry_, p.exit_, p.min_len_, p.max_len_, symmetry_, sigma_a_,
                                           d_applicable_);
  }
  std::unique_ptr<FilterSpec> operator()(const DirectionFilterParam& p) const {
    return std::make_unique<DirectionSpec>(p.lon_, p.lat_, p.radii_);
  }
  std::unique_ptr<FilterSpec> operator()(const CrystalFilterParam& p) const {
    return std::make_unique<CrystalSpec>(p.crystal_id_);
  }
};

ComplexSpec::ComplexSpec(const Crystal& crystal,
                         const std::vector<std::vector<std::pair<IdType, SimpleFilterParam>>>& all_param,
                         uint8_t symmetry, int sigma_a, bool d_applicable) {
  filters_.reserve(all_param.size());
  for (const auto& or_clause : all_param) {
    std::vector<std::unique_ptr<FilterSpec>> ands;
    ands.reserve(or_clause.size());
    for (const auto& and_entry : or_clause) {
      ands.emplace_back(std::visit(SimpleSpecCreator{ crystal, symmetry, sigma_a, d_applicable }, and_entry.second));
    }
    filters_.emplace_back(std::move(ands));
  }
}

struct TopSpecCreator {
  const Crystal& crystal_;
  uint8_t symmetry_;
  int sigma_a_;
  bool d_applicable_;

  std::unique_ptr<FilterSpec> operator()(const SimpleFilterParam& p) const {
    return std::visit(SimpleSpecCreator{ crystal_, symmetry_, sigma_a_, d_applicable_ }, p);
  }
  std::unique_ptr<FilterSpec> operator()(const ComplexFilterParam& p) const {
    return std::make_unique<ComplexSpec>(crystal_, p.filters_, symmetry_, sigma_a_, d_applicable_);
  }
};

}  // namespace

std::unique_ptr<FilterSpec> FilterSpec::Create(const FilterConfig& config, const Crystal& crystal,
                                               const AxisDistribution& axis_dist) {
  bool d_applicable = detail::IsDApplicable(axis_dist);
  int sigma_a = d_applicable ? detail::ComputeSigmaA(axis_dist.roll_dist.mean) : 0;
  auto spec = std::visit(TopSpecCreator{ crystal, config.symmetry_, sigma_a, d_applicable }, config.param_);
  spec->action_ = config.action_;
  return spec;
}

std::vector<ColorSpecGroupOwned> BuildColorSpecGroups(const ColorGatePlacement& placement, const Crystal& crystal,
                                                      const AxisDistribution& axis_dist) {
  std::vector<ColorSpecGroupOwned> groups;
  if (placement.predicates_.empty()) {
    return groups;  // AC3 zero-cost fast path.
  }
  size_t n = placement.predicates_.size();
  // In-order group formation: first pass records each distinct symmetry value's
  // group index (first-occurrence order). Second pass populates each group's
  // ComplexFilterParam with its predicates + parallel bits vector.
  std::vector<size_t> group_of(n, 0);
  std::vector<uint8_t> group_symmetry;
  group_symmetry.reserve(n);
  for (size_t k = 0; k < n; ++k) {
    uint8_t sym = placement.symmetries_[k];
    size_t gi = group_symmetry.size();
    for (size_t i = 0; i < group_symmetry.size(); ++i) {
      if (group_symmetry[i] == sym) {
        gi = i;
        break;
      }
    }
    if (gi == group_symmetry.size()) {
      group_symmetry.push_back(sym);
    }
    group_of[k] = gi;
  }

  size_t group_count = group_symmetry.size();
  std::vector<ComplexFilterParam> cfps(group_count);
  groups.resize(group_count);
  for (size_t k = 0; k < n; ++k) {
    size_t gi = group_of[k];
    // One OR-summand (single-clause AND-of-1) per predicate — matches the
    // pre-refactor synthesized shape so ComplexSpec's per-summand mask reports
    // exactly which predicate matched. Preserves placement-original order
    // within the group (bits[j] recovers placement's bit for local summand j).
    cfps[gi].filters_.push_back({ { kInvalidId, placement.predicates_[k] } });
    groups[gi].bits.push_back(placement.bits_[k]);
  }

  for (size_t gi = 0; gi < group_count; ++gi) {
    FilterConfig fc{};
    fc.id_ = kInvalidId;
    fc.symmetry_ = group_symmetry[gi];
    fc.action_ = FilterConfig::kFilterIn;
    fc.param_ = FilterParam{ cfps[gi] };
    groups[gi].spec = FilterSpec::Create(fc, crystal, axis_dist);
  }
  return groups;
}

}  // namespace lumice
