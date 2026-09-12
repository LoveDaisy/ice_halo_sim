// Single-source definition of the online ray-allocation tally and the q snapshot
// that scene.ray_allocation = adaptive runs on. Host-only C++ (no MSL/CUDA
// address-space shims are needed: the device side accumulates plain floats and
// the host writes them into these types), but it lives in core/shared/ because
// the three trace backends (CpuTraceBackend, MetalTraceBackend, CudaTraceBackend)
// and the legacy Simulator all WRITE the tally and must agree, to the letter, on
// what each field means. They include this header and nothing else of the
// simulator's; the formulas that CONSUME the tally
// (ComputeAdaptiveRayAllocationWeights, ComputeRayAllocationCorrection) stay in
// core/simulator.hpp, where the one host-side owner of the loop
// (RayAllocationOnline) reads it.
#ifndef CORE_SHARED_RAY_ALLOCATION_SHARED_H_
#define CORE_SHARED_RAY_ALLOCATION_SHARED_H_

#include <cstddef>
#include <vector>

namespace lumice {

// What one (layer, entry) contributed to the Neyman statistic over some span of
// tracing — a batch on the producing side, the whole run once accumulated. In the
// layout of SceneConfig::ms_[mi].setting_[ci].
//
// The contract every writer obeys:
//
//   rays    — what PartitionCrystalRayNum DEALT the entry on this layer. On a
//             continuation layer (mi > 0) that is the re-dealt survivors of the
//             layer above, so a deep layer's count is smaller by construction.
//             Known to the host at dispatch time; no backend has to count it.
//   sum_w   — Σ over the entry's TRUE exits of w / c.
//   sum_w2  — Σ over the same rays of (w / c)².
//
// "True exit" is a ray that leaves the crystal on this layer AND is handed to
// the image — the same rays SimData::outgoing_w_ / the device XYZ accumulator
// receive. A ray that continues into the next layer (the ms prob draw said so)
// is NOT counted here: its energy lands later, under the entry it is re-dealt to.
// A filter-fail ray is not counted either (it lands nothing). A true exit the
// user's lens does not image still counts: the statistic is a property of the
// scene, not of the frame.
//
// `w` is the weight the exit was accumulated at and `c` is THIS (layer, entry)'s
// own correction — ComputeRayAllocationCorrection's (p_i/ΣP)/(q_i/ΣQ) for the
// deal the ray was born into on this layer. Dividing that one factor out, and
// only that one, is what Neyman wants: for layer L entry i the variance
// contribution is n_i · c_i² · E[(w_in · e_i)²] with n_i = q_i · N_L, w_in the
// weight the ray ENTERED the layer with (upstream corrections included) and e_i
// the layer's own uncorrected contribution; minimizing over q gives
// q_i ∝ p_i · √E[(w_in · e_i)²], i.e. the second moment of w / c_i. Dividing
// nothing out would fold c_i into its own estimate; dividing the whole chain out
// would optimize a different estimator than the one the image is built from. On
// the first layer w_in is the nominal weight, so there the tally is exactly what
// a proportional trace of the scene would measure.
//
// Σ semantics throughout: a consumer ADDS a tally into its own, never overwrites.
struct RayAllocationEntryTally {
  double sum_w = 0.0;
  double sum_w2 = 0.0;
  size_t rays = 0;

  RayAllocationEntryTally& operator+=(const RayAllocationEntryTally& o) {
    sum_w += o.sum_w;
    sum_w2 += o.sum_w2;
    rays += o.rays;
    return *this;
  }
};

// [mi][ci], mirrors SceneConfig::ms_. Empty on every proportional path (zero-cost:
// nothing is sized, nothing is written).
using RayAllocationTally = std::vector<std::vector<RayAllocationEntryTally>>;

// The q every layer is dealt by, [mi][ci], mirroring SceneConfig::ms_. Immutable
// once published: a batch reads one snapshot at its start and keeps it for the
// batch, so the correction it charges emitted_energy_ with is the one its rays
// were born under. Carries q only — the correction is derived from (p, q) by the
// single owner ComputeRayAllocationCorrection at the point of use, never stored.
struct RayAllocationSnapshot {
  std::vector<std::vector<float>> q;
};

}  // namespace lumice

#endif  // CORE_SHARED_RAY_ALLOCATION_SHARED_H_
