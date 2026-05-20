#include "core/filter_spec.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <utility>
#include <variant>
#include <vector>

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
void ReduceRecorder(RaypathRecorder& rp, uint8_t symmetry, int sigma_a, bool d_applicable) {
  if (symmetry == FilterConfig::kSymNone) {
    return;
  }

  if (symmetry & FilterConfig::kSymP) {
    PCanonicalShiftInPlace(rp.recorder_, rp.size_);
  }

  if ((symmetry & FilterConfig::kSymD) && d_applicable) {
    uint8_t scratch[kMaxHits]{};
    for (size_t i = 0; i < rp.size_; i++) {
      uint8_t x = rp.recorder_[i];
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
      PCanonicalShiftInPlace(scratch, rp.size_);
    }
    if (LexLessRecorder(scratch, rp.recorder_, rp.size_)) {
      std::memcpy(rp.recorder_, scratch, rp.size_);
    }
  }

  if (symmetry & FilterConfig::kSymB) {
    uint8_t scratch[kMaxHits]{};
    bool changed = false;
    for (size_t i = 0; i < rp.size_; i++) {
      uint8_t x = rp.recorder_[i];
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
    if (changed && LexLessRecorder(scratch, rp.recorder_, rp.size_)) {
      std::memcpy(rp.recorder_, scratch, rp.size_);
    }
  }
}

}  // namespace detail

bool RaypathOrbit::Contains(const RaypathRecorder& rp_input) const {
  if (fn_period_ < 0 || symmetry_ == FilterConfig::kSymNone) {
    return rp_input == canonical_;
  }
  if (rp_input.size_ != canonical_.size_) {
    return false;
  }
  RaypathRecorder rp = rp_input;
  detail::ReduceRecorder(rp, symmetry_, sigma_a_, d_applicable_);
  return rp == canonical_;
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

class NoneSpec : public FilterSpec {
 public:
  bool Match(const RaySeg& /*ray*/) const override { return true; }
};

class RaypathSpec : public FilterSpec {
 public:
  RaypathSpec(const Crystal& crystal, const std::vector<IdType>& rp, uint8_t symmetry, int sigma_a, bool d_applicable)
      : orbit_(BuildOrbit(crystal, rp, symmetry, sigma_a, d_applicable)) {}

  bool Match(const RaySeg& ray) const override { return orbit_.Contains(ray.rp_); }

 private:
  RaypathOrbit orbit_;
};

class EntryExitSpec : public FilterSpec {
 public:
  EntryExitSpec(const Crystal& crystal, IdType entry, IdType exit, uint8_t symmetry, int sigma_a, bool d_applicable)
      : orbit_(BuildOrbit(crystal, std::vector<IdType>{ entry, exit }, symmetry, sigma_a, d_applicable)) {}

  bool Match(const RaySeg& ray) const override {
    if (ray.rp_.size_ == 0) {
      return false;
    }
    RaypathRecorder ee;
    ee.Clear();
    ee << ray.rp_[0] << ray.rp_[ray.rp_.size_ - 1];
    return orbit_.Contains(ee);
  }

 private:
  RaypathOrbit orbit_;
};

class DirectionSpec : public FilterSpec {
 public:
  DirectionSpec(float lon_deg, float lat_deg, float radii_deg)
      : d_{ std::cos(lat_deg * math::kDegreeToRad) * std::cos(lon_deg * math::kDegreeToRad),
            std::cos(lat_deg * math::kDegreeToRad) * std::sin(lon_deg * math::kDegreeToRad),
            std::sin(lat_deg * math::kDegreeToRad) },
        radii_c_(std::cos(radii_deg * math::kDegreeToRad)) {}

  bool Match(const RaySeg& ray) const override { return Dot3(d_, ray.d_) > radii_c_; }

 private:
  float d_[3];
  float radii_c_;
};

class CrystalSpec : public FilterSpec {
 public:
  explicit CrystalSpec(IdType crystal_id) : crystal_id_(crystal_id) {}
  bool Match(const RaySeg& ray) const override { return ray.crystal_config_id_ == crystal_id_; }

 private:
  IdType crystal_id_;
};

class ComplexSpec : public FilterSpec {
 public:
  ComplexSpec(const Crystal& crystal, const std::vector<std::vector<std::pair<IdType, SimpleFilterParam>>>& all_param,
              uint8_t symmetry, int sigma_a, bool d_applicable);

  bool Match(const RaySeg& ray) const override {
    for (const auto& or_clause : filters_) {
      bool and_check = true;
      for (const auto& and_f : or_clause) {
        if (!and_f->Match(ray)) {
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
    return std::make_unique<EntryExitSpec>(crystal_, p.entry_, p.exit_, symmetry_, sigma_a_, d_applicable_);
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

}  // namespace lumice
