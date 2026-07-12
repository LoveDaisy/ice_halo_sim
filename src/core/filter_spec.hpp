// See doc/filter-architecture.md §2/§3 for the simulation-side filter gate semantics
// and multi-scatter pruning behaviour that this class implements.
#ifndef SRC_CORE_FILTER_SPEC_H_
#define SRC_CORE_FILTER_SPEC_H_

#include <cstdint>
#include <memory>
#include <vector>

#include "config/filter_config.hpp"
#include "core/crystal.hpp"
#include "core/raypath.hpp"

namespace lumice {

struct ColorGatePlacement;

// RaypathOrbit: canonical representative + membership test under symmetry.
struct RaypathOrbit {
  RaypathRecorder canonical_;
  uint8_t symmetry_ = FilterConfig::kSymNone;
  int sigma_a_ = 0;
  bool d_applicable_ = false;
  // fn_period_ < 0: custom crystal, no reduction; current detail::ReduceBuffer
  // hardcodes kFnPeriod=6 for hexagonal prism/pyramid (the only supported family).
  int fn_period_ = -1;

  // overflow_arena resolves recorders whose hit data lives in their owning
  // RayBuffer's arena (Round 2 / #247.4). Pass nullptr for inline-only
  // recorders (e.g. EntryExitSpec's stack-built ee or unit-test fixtures).
  bool Contains(const RaypathRecorder& rp_input, const uint8_t* overflow_arena = nullptr) const;
};

class FilterSpec {
 public:
  FilterSpec() = default;
  FilterSpec(const FilterSpec&) = default;
  FilterSpec(FilterSpec&&) = default;
  FilterSpec& operator=(const FilterSpec&) = default;
  FilterSpec& operator=(FilterSpec&&) = default;
  virtual ~FilterSpec() = default;

  virtual bool Match(const RaySeg& ray, const RaypathRecorder& rec, const uint8_t* overflow_arena = nullptr) const = 0;

  bool Check(const RaySeg& ray, const RaypathRecorder& rec, const uint8_t* overflow_arena = nullptr) const {
    bool m = Match(ray, rec, overflow_arena);
    return action_ == FilterConfig::kFilterIn ? m : !m;
  }

  // Single-pass per-summand evaluation for the raypath-color foundation
  // (task-331.2). Returns a mask whose bit k is set iff OR-summand k matched,
  // and (via out_matched) the pre-action collapse boolean == Match(). All
  // simple filters (including None after task-339.1) expose exactly one
  // summand (bit 0); the base-class default `mask = Match() ? 1 : 0` handles
  // this uniformly. ComplexSpec overrides this to evaluate EVERY OR-summand
  // (no short-circuit) so the gate's collapse-to-boolean does not discard the
  // per-summand information the component mask needs.
  //
  // Callers that produce component bits at the emit gate MUST use this (via
  // CheckSummandMask) instead of Match()+Check() so the predicates are
  // evaluated once, not twice.
  virtual uint64_t MatchSummandMask(const RaySeg& ray, const RaypathRecorder& rec, const uint8_t* overflow_arena,
                                    bool* out_matched) const {
    bool m = Match(ray, rec, overflow_arena);
    if (out_matched != nullptr) {
      *out_matched = m;
    }
    return m ? 1ull : 0ull;
  }

  // Action-applied gate decision plus the per-summand mask, in a single pass.
  // Equivalent to Check() for the returned boolean, but also surfaces the
  // pre-action per-summand mask so the caller can map matched summands onto
  // component bits without a second predicate evaluation.
  bool CheckSummandMask(const RaySeg& ray, const RaypathRecorder& rec, const uint8_t* overflow_arena,
                        uint64_t* out_summand_mask) const {
    bool matched = false;
    uint64_t mask = MatchSummandMask(ray, rec, overflow_arena, &matched);
    if (out_summand_mask != nullptr) {
      *out_summand_mask = mask;
    }
    return action_ == FilterConfig::kFilterIn ? matched : !matched;
  }

  static std::unique_ptr<FilterSpec> Create(const FilterConfig& config, const Crystal& crystal,
                                            const AxisDistribution& axis_dist);

 protected:
  // action_ is set by FilterSpec::Create after the derived class is fully
  // constructed; derived constructors must not depend on its value.
  FilterConfig::Action action_ = FilterConfig::kFilterIn;
};

// Owning wrapper around one per-symmetry-group synthetic color FilterSpec, plus
// the placement-original bit-index map for its OR-summands. Produced by
// `BuildColorSpecGroups`; consumed by `CollectData` (via the non-owning
// `ColorSpecGroup` view) to OR component bits into a ray's carried mask on the
// CPU emit gate.
//
// The group split is by `ColorGatePlacement::symmetries_` value: predicates
// carrying the same P/B/D bitmask share one synthesized `FilterConfig`
// (`symmetry_ = that value`, one OR-summand per predicate); different symmetry
// values live in separate groups. Within a group, summand order == the group's
// predicates' placement-original order, so `bits[k]` recovers the placement's
// bit index for local summand k without a second lookup.
struct ColorSpecGroupOwned {
  std::unique_ptr<FilterSpec> spec;
  std::vector<uint8_t> bits;  // local summand idx (in this group's ComplexFilterParam) -> global component bit
};

// Split a ColorGatePlacement into per-symmetry-value groups. Each group becomes
// one synthetic `ComplexFilterParam` (OR-of-singleton-AND-clauses) wrapped in a
// `FilterConfig{symmetry_=<group's value>, action_=kFilterIn}` and handed to
// `FilterSpec::Create` — so the same `Crystal::ReduceRaypath`/`ExpandRaypath`
// pipeline that runs for a physical filter with `symmetry=X` runs for the color
// pass with `symmetry=X`, giving the two orbits by construction (a12: no
// re-implementation of the symmetry match).
//
// Group formation uses "first-occurrence-of-symmetry-value" order; within a
// group, predicate order == placement's original order — so a placement of
// symmetries [None, P, None, P] yields two groups (None with predicates 0 and
// 2, then P with predicates 1 and 3) and each summand's `bits` entry maps back
// to the placement's `bits_[k]`.
//
// Empty placement returns an empty vector (AC3 zero-cost path — the caller
// passes `nullptr` to CollectData when there's nothing to OR).
//
// When every ref in `placement` carries the default `kSymNone`, this produces
// exactly one group containing all predicates in placement order — a
// bit-for-bit no-op relative to the pre-refactor "single synthesized
// ComplexFilterParam with kSymNone" call site (AC3).
std::vector<ColorSpecGroupOwned> BuildColorSpecGroups(const ColorGatePlacement& placement, const Crystal& crystal,
                                                      const AxisDistribution& axis_dist);

namespace detail {
// In-place canonicalisation of a raypath byte buffer under (symmetry, sigma_a,
// d_applicable). Mirrors Crystal::ReduceRaypath for hexagonal crystals
// (fn_period=6). Operates on a raw uint8_t buffer so it works uniformly on
// inline data_ and on copies of arena slots. Precondition: symmetry != kSymNone.
void ReduceBuffer(uint8_t* data, size_t size, uint8_t symmetry, int sigma_a, bool d_applicable);
}  // namespace detail

}  // namespace lumice

#endif  // SRC_CORE_FILTER_SPEC_H_
