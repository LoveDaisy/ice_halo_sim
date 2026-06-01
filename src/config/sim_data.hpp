#ifndef CONFIG_SIM_DATA_H_
#define CONFIG_SIM_DATA_H_

#include <cstddef>
#include <memory>

#include "core/crystal.hpp"
#include "core/def.hpp"
#include "core/math.hpp"
#include "core/raypath.hpp"

namespace lumice {

struct RayBuffer {
  RayBuffer();
  explicit RayBuffer(size_t capacity);

  RaySeg& operator[](size_t idx) const;

  // Parallel-array access to the per-ray RaypathRecorder (moved out of RaySeg
  // to keep the hot float/id BufferWrapper cache density high). Named method
  // rather than a second operator[] overload to avoid `buf[i]` ambiguity
  // between RaySeg and RaypathRecorder arrays.
  RaypathRecorder& RecorderAt(size_t idx);
  const RaypathRecorder& RecorderAt(size_t idx) const;

  void Reset(size_t capacity);
  bool Empty() const;
  // Single-RaySeg entry point. Asserts RaySeg::IsValidComplete() at entry
  // to gate the N4 construction-time invariants (Debug only; noop in Release).
  // Buffer-to-buffer overload below is internal data movement and skips this
  // gate — its inputs were already validated when first emplaced.
  void EmplaceBack(RaySeg r, const RaypathRecorder& rec);
  void EmplaceBack(const RayBuffer& buffer, size_t start = 0, size_t len = kInfSize);

  RaySeg* rays() const;
  RaySeg* begin() const;
  RaySeg* end() const;

  // TODO(soa-refactor): if RayBuffer gains more owning members in the future,
  // give RayBuffer its own copy/move ctors + assignment ops so SimData can
  // delegate instead of manually synchronizing each owning field below. (See
  // scrum-cpu-soa-refactor review.md Suggestion 1.)
  size_t capacity_;
  size_t size_;
  std::unique_ptr<RaySeg[]> rays_;
  std::unique_ptr<RaypathRecorder[]> recorders_;
};


struct SimData {
  SimData();
  explicit SimData(size_t capacity);
  SimData(const SimData& other);
  SimData(SimData&& other) noexcept;
  ~SimData() = default;

  SimData& operator=(const SimData& other);
  SimData& operator=(SimData&& other) noexcept;

  // ----- Data -----
  float curr_wl_;
  uint64_t generation_ = 0;
  RayBuffer rays_;
  std::vector<Crystal> crystals_;
  std::vector<AxisDistribution> crystal_axis_dists_;  // parallel to crystals_
  std::vector<size_t> outgoing_indices_;              // Indices of kOutgoing rays in rays_ (filled by Simulator)

  // Pre-packed outgoing ray data for cache-friendly access in Consume().
  // Tightly packed by outgoing order: outgoing_d_[k*3..k*3+2] and outgoing_w_[k]
  // correspond to outgoing_indices_[k]. Filled by Simulator alongside outgoing_indices_.
  std::vector<float> outgoing_d_;  // direction (3 floats per outgoing ray)
  std::vector<float> outgoing_w_;  // weight (1 float per outgoing ray)

  size_t root_ray_count_ = 0;  // Count of root rays (prev_ray_idx_ == kInfSize)
};

using SimDataPtrS = std::shared_ptr<SimData>;
using SimDataPtrU = std::unique_ptr<SimData>;

}  // namespace lumice

#endif  // CONFIG_SIM_DATA_H_
