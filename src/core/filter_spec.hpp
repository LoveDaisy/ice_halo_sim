#ifndef SRC_CORE_FILTER_SPEC_H_
#define SRC_CORE_FILTER_SPEC_H_

#include <cstdint>
#include <memory>

#include "config/filter_config.hpp"
#include "core/crystal.hpp"
#include "core/raypath.hpp"

namespace lumice {

// RaypathOrbit: canonical representative + membership test under symmetry.
struct RaypathOrbit {
  RaypathRecorder canonical_;
  uint8_t symmetry_ = FilterConfig::kSymNone;
  int sigma_a_ = 0;
  bool d_applicable_ = false;
  // fn_period_ < 0: custom crystal, no reduction; current detail::ReduceRecorder
  // hardcodes kFnPeriod=6 for hexagonal prism/pyramid (the only supported family).
  int fn_period_ = -1;

  bool Contains(const RaypathRecorder& rp_input) const;
};

class FilterSpec {
 public:
  FilterSpec() = default;
  FilterSpec(const FilterSpec&) = default;
  FilterSpec(FilterSpec&&) = default;
  FilterSpec& operator=(const FilterSpec&) = default;
  FilterSpec& operator=(FilterSpec&&) = default;
  virtual ~FilterSpec() = default;

  virtual bool Match(const RaySeg& ray) const = 0;

  static std::unique_ptr<FilterSpec> Create(const FilterConfig& config, const Crystal& crystal,
                                            const AxisDistribution& axis_dist);
};

namespace detail {
// In-place reduce of RaypathRecorder under (symmetry, sigma_a, d_applicable).
// Mirrors Crystal::ReduceRaypath for hexagonal crystals (fn_period=6).
// Precondition: symmetry != kSymNone (caller already gates this).
void ReduceRecorder(RaypathRecorder& rp, uint8_t symmetry, int sigma_a, bool d_applicable);
}  // namespace detail

}  // namespace lumice

#endif  // SRC_CORE_FILTER_SPEC_H_
