#include "core/device_filter_desc.hpp"

#if defined(__APPLE__)

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <optional>
#include <variant>
#include <vector>

#include "config/filter_config.hpp"
#include "core/crystal.hpp"
#include "core/def.hpp"
#include "core/math.hpp"

namespace lumice {
namespace detail {

namespace {

// Mirror of `RaypathOrbit`/`BuildOrbit` (filter_spec.cpp:139-152): canonical
// hits are produced by `Crystal::ReduceRaypath` on the configured raypath. We
// recompute here rather than calling `BuildOrbit` to keep `filter_spec.cpp`
// platform-agnostic (plan D5).
void FillCanonicalBytes(const Crystal& crystal, const std::vector<IdType>& rp, uint8_t symmetry, int sigma_a,
                        bool d_applicable, DeviceFilterDesc& out) {
  auto canonical = crystal.ReduceRaypath(rp, symmetry, sigma_a, d_applicable);
  out.canonical_len = static_cast<uint8_t>(std::min<size_t>(canonical.size(), kMaxHits));
  for (uint8_t i = 0; i < out.canonical_len; ++i) {
    out.canonical_bytes[i] = static_cast<uint8_t>(canonical[i] & 0xFF);
  }
}

void FillRaypath(const Crystal& crystal, const RaypathFilterParam& p, uint8_t symmetry, int sigma_a, bool d_applicable,
                 DeviceFilterDesc& out) {
  out.type = kDeviceFilterTypeRaypath;
  FillCanonicalBytes(crystal, p.raypath_, symmetry, sigma_a, d_applicable, out);
}

void FillEntryExit(const Crystal& crystal, const EntryExitFilterParam& p, uint8_t symmetry, int sigma_a,
                   bool d_applicable, DeviceFilterDesc& out) {
  out.type = kDeviceFilterTypeEntryExit;
  out.has_entry = p.entry_.has_value() ? 1u : 0u;
  out.has_exit = p.exit_.has_value() ? 1u : 0u;
  out.min_len = static_cast<uint32_t>(p.min_len_);
  // max_len == 0 sentinels "no upper bound" on device (matches plan D2).
  out.max_len = p.max_len_.has_value() ? static_cast<uint32_t>(*p.max_len_) : 0u;

  // BuildOrbitFromEnds: empty rp ⇒ length-only match (has_orbit_ guards Match
  // on host; the device path mirrors via has_entry==0 && has_exit==0).
  std::vector<IdType> rp;
  if (p.entry_.has_value()) {
    rp.push_back(*p.entry_);
  }
  if (p.exit_.has_value()) {
    rp.push_back(*p.exit_);
  }
  if (!rp.empty()) {
    FillCanonicalBytes(crystal, rp, symmetry, sigma_a, d_applicable, out);
  }
}

void FillDirection(const DirectionFilterParam& p, DeviceFilterDesc& out) {
  // Mirror of DirectionSpec ctor (filter_spec.cpp:243-247).
  out.type = kDeviceFilterTypeDirection;
  float lon_rad = p.lon_ * math::kDegreeToRad;
  float lat_rad = p.lat_ * math::kDegreeToRad;
  out.dir[0] = std::cos(lat_rad) * std::cos(lon_rad);
  out.dir[1] = std::cos(lat_rad) * std::sin(lon_rad);
  out.dir[2] = std::sin(lat_rad);
  out.radii_c = std::cos(p.radii_ * math::kDegreeToRad);
}

void FillCrystal(const CrystalFilterParam& p, DeviceFilterDesc& out) {
  out.type = kDeviceFilterTypeCrystal;
  out.crystal_id = static_cast<uint32_t>(p.crystal_id_);
}

struct SimpleVisitor {
  const Crystal& crystal;
  uint8_t symmetry;
  int sigma_a;
  bool d_applicable;
  DeviceFilterDesc& out;

  void operator()(const NoneFilterParam& /*p*/) const { out.type = kDeviceFilterTypeNone; }
  void operator()(const RaypathFilterParam& p) const { FillRaypath(crystal, p, symmetry, sigma_a, d_applicable, out); }
  void operator()(const EntryExitFilterParam& p) const {
    FillEntryExit(crystal, p, symmetry, sigma_a, d_applicable, out);
  }
  void operator()(const DirectionFilterParam& p) const { FillDirection(p, out); }
  void operator()(const CrystalFilterParam& p) const { FillCrystal(p, out); }
};

// Fill the Complex filter's top-level fields (type / or_clause_count /
// and_term_counts[]). Does NOT touch sub_desc_start — that field is owned by
// the caller (`EnsureFilterBuffers`) which assigns it before appending the
// sub-descs via `BuildComplexSubDescs` (plan §3 D5).
void FillComplexDescTop(const ComplexFilterParam& p, DeviceFilterDesc& out) {
  // The OR-clause count is bounded by the struct-level `and_term_counts[]`
  // array length (MSL has no dynamic allocation). Raising the limit requires
  // updating `kDeviceFilterMaxOrClauses` + the C++/MSL array length together.
  assert(p.filters_.size() <= kDeviceFilterMaxOrClauses &&
         "Complex filter exceeds kDeviceFilterMaxOrClauses (8); raise constant + arrays together");
  out.type = kDeviceFilterTypeComplex;
  out.or_clause_count = static_cast<uint8_t>(p.filters_.size());
  // and_term_counts[] is zero-initialized by `DeviceFilterDesc desc{}` at the
  // top of BuildDeviceFilterDesc; only set the slots we use.
  for (size_t i = 0; i < p.filters_.size(); ++i) {
    // and_term_counts[i] is uint8_t (0..255); AND-term count is far smaller in
    // practice (the canonical ms3 config uses 1 per clause). The 255 ceiling
    // is sufficient; raising would only require widening this field.
    assert(p.filters_[i].size() <= 255u && "Complex AND-term count exceeds uint8_t (255)");
    out.and_term_counts[i] = static_cast<uint8_t>(p.filters_[i].size());
  }
}

struct TopVisitor {
  const Crystal& crystal;
  uint8_t symmetry;
  int sigma_a;
  bool d_applicable;
  DeviceFilterDesc& out;

  void operator()(const SimpleFilterParam& p) const {
    std::visit(SimpleVisitor{ crystal, symmetry, sigma_a, d_applicable, out }, p);
  }
  void operator()(const ComplexFilterParam& p) const {
    // Top-level desc only: or_clause_count + and_term_counts. sub_desc_start
    // is assigned in EnsureFilterBuffers immediately before BuildComplexSubDescs
    // is invoked, so it stays 0 here.
    FillComplexDescTop(p, out);
  }
};

}  // namespace

DeviceFilterDesc BuildDeviceFilterDesc(const FilterConfig& config, const Crystal& crystal,
                                       const AxisDistribution& axis_dist) {
  DeviceFilterDesc desc{};  // zero-init all fields
  desc.action = (config.action_ == FilterConfig::kFilterIn) ? 0u : 1u;
  desc.symmetry = config.symmetry_;
  bool d_applicable = detail::IsDApplicable(axis_dist);
  desc.d_applicable = d_applicable ? 1u : 0u;
  desc.sigma_a = d_applicable ? detail::ComputeSigmaA(axis_dist.roll_dist.mean) : 0;
  desc.fn_period = crystal.FnPeriod();

  std::visit(TopVisitor{ crystal, config.symmetry_, desc.sigma_a, d_applicable, desc }, config.param_);
  return desc;
}

void BuildComplexSubDescs(const ComplexFilterParam& p, const Crystal& crystal, uint8_t symmetry, int sigma_a,
                          bool d_applicable, std::vector<DeviceFilterDesc>& out_sub_descs) {
  assert(p.filters_.size() <= kDeviceFilterMaxOrClauses &&
         "Complex filter exceeds kDeviceFilterMaxOrClauses (8); raise constant + arrays together");
  for (const auto& or_clause : p.filters_) {
    // Same uint8_t bound as FillComplexDescTop; kept here so an unsynced caller
    // (e.g. tests bypassing FillComplexDescTop) still trips the check.
    assert(or_clause.size() <= 255u && "Complex AND-term count exceeds uint8_t (255)");
    for (const auto& and_entry : or_clause) {
      // Mirror ComplexSpec ctor (filter_spec.cpp:316): sub-spec inherits the
      // parent Complex's symmetry / sigma_a / d_applicable. and_entry.first
      // is the host-side filter id (unused on device — sub_specs are inlined
      // by position).
      DeviceFilterDesc sub{};
      sub.symmetry = symmetry;
      sub.d_applicable = d_applicable ? 1u : 0u;
      sub.sigma_a = sigma_a;
      sub.fn_period = crystal.FnPeriod();
      // sub-filter predicate only; the top-level Complex `action` is XORed by
      // DeviceFilterCheck. Mirrors host ComplexSpec::Match calling
      // `and_f->Match` (not `Check`) per filter_spec.cpp:274-288.
      sub.action = 0u;
      std::visit(SimpleVisitor{ crystal, symmetry, sigma_a, d_applicable, sub }, and_entry.second);
      out_sub_descs.push_back(sub);
    }
  }
}

std::vector<uint8_t> BuildDeviceGetFnBytes(const Crystal& crystal) {
  size_t n = crystal.PolygonFaceCount();
  std::vector<uint8_t> bytes(n, 0u);
  for (size_t i = 0; i < n; ++i) {
    IdType fn = crystal.GetFn(static_cast<IdType>(i));
    // GetFn returns kInvalidId for out-of-range; that only happens on a
    // mismatched crystal/poly index, which is a programming error elsewhere.
    // We clamp to 0xFF to keep the byte stream well-defined.
    bytes[i] = static_cast<uint8_t>(fn & 0xFF);
  }
  return bytes;
}

}  // namespace detail
}  // namespace lumice

#endif  // defined(__APPLE__)
