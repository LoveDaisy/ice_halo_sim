#ifndef CORE_TRACE_OPS_H_
#define CORE_TRACE_OPS_H_

#include <cstddef>
#include <vector>

#include "config/light_config.hpp"
#include "config/proj_config.hpp"
#include "config/sim_data.hpp"
#include "core/crystal.hpp"
#include "core/math.hpp"

namespace lumice {

class FilterSpec;

// Internal trace-pipeline primitives shared between Simulator and the
// alternative seam implementations (e.g. CpuTraceBackend).
//
// IMPORTANT: this header is NOT part of the public C API. It is intentionally
// not included from simulator.hpp — backends that need these primitives
// include it directly to keep simulator.hpp's TU set unpolluted.
//
// All free-function bodies live in simulator.cpp. The one type declared here
// (NewSampleTracker, at the bottom) is a deliberate exception to the otherwise
// stateless character of this header: it carries the minimum state a shared
// PREDICATE needs to stay a single source. It is a judgement helper, not a
// pipeline stage — anything with real per-session machinery belongs on a
// backend, not here.

// Sample crystal origin (p, from_face_, to_face_). Direction d MUST already
// be set on the buffer.
void InitRay_p_fid(const Crystal& curr_crystal, RayBuffer* ray_buf_ptr);

// Set initial direction d (sampled from light source), weight w, and
// prev_ray_idx for `ray_num` rays.
void InitRay_d_w_previdx(const SunParam& light_param, const WlParam& wl_param, size_t ray_num, RayBuffer* ray_buf_ptr);

// Sample per-ray crystal-orientation rotation matrices, writing into
// buffer_data[0]. buffer_data[1] is unused.
void InitRay_rot(RandomNumberGenerator& rng, const AxisDistribution& crystal_axis, RayBuffer buffer_data[2]);

// Fill crystal_idx_/crystal_config_id_/root_ray_idx_/recorder state for the
// rays in buffer_data[0]. `all_data_idx` is the index where buffer_data[0]
// will be EmplaceBack'd into all_data; root_ray_idx_ counts from there.
void InitRay_other_info(const Crystal& curr_crystal, size_t curr_crystal_id, size_t all_data_idx,
                        RayBuffer buffer_data[2]);

// First-MS-layer init: sample direction from sun, sample p on crystal,
// orient the crystal, fill bookkeeping, and EmplaceBack into all_data.
void InitRayFirstMs(RandomNumberGenerator& rng, const SunParam& light_param, const WlParam& wl_param,
                    size_t curr_ray_num, const Crystal& curr_crystal, size_t curr_crystal_id,
                    const AxisDistribution& crystal_axis, RayBuffer buffer_data[2], RayBuffer& all_data);

// Non-first MS layer init: copy continuation rays from init_data, sample a
// fresh crystal orientation per ray, rotate d into crystal-local, sample p,
// fill bookkeeping, EmplaceBack into all_data. `init_ray_offset` is advanced
// by curr_ray_num.
void InitRayOtherMs(RandomNumberGenerator& rng, const RayBuffer init_data[2], size_t curr_ray_num,
                    const Crystal& curr_crystal, size_t curr_crystal_id, const AxisDistribution& crystal_axis,
                    RayBuffer buffer_data[2], RayBuffer& all_data, size_t& init_ray_offset);

// One hit: refract+reflect (HitSurface) -> propagate, fan out into
// buffer_data[1] (2x input size). buffer_data[0] is consumed (size_=0).
void TraceRayBasicInfo(const Crystal& curr_crystal, float refractive_index, size_t curr_ray_num,
                       RayBuffer* buffer_data);

// Append the new to_face_ id onto every recorder slot in buffer_data[1].
void FillRayOtherInfo(const Crystal& curr_crystal, RayBuffer buffer_data[2]);

// Allocate the all_data buffer with expected total-ray capacity for a session.
RayBuffer AllocateAllData(const SceneConfig& config, size_t ray_num);

// Sample the raw prism shape scalars, honoring PrismCrystalParam::sync_group_
// (draw order: height, then face_distance[0..5] — the same order MakeCrystal's
// prism branch has always used). Exposed beyond MakeCrystal so tests can assert
// on the raw scalars and on the RNG draw count directly: MakeCrystal folds them
// into closed-form Crystal geometry, which may reject or degenerate the very
// inputs a sampling test needs to see.
// Returns h (already abs()'d); dist_out receives the 6 signed face distances.
float SamplePrismShapeScalars(RandomNumberGenerator& rng, const PrismCrystalParam& p, float dist_out[6]);

// Same for pyramid. Draw order is h_pyr_u_ -> h_prs_ -> h_pyr_l_ ->
// face_distance[0..5] — NOT the struct declaration order; see ShapeScalar.
// h1/h2/h3 are already abs()'d.
void SamplePyramidShapeScalars(RandomNumberGenerator& rng, const PyramidCrystalParam& p, float& h1, float& h2,
                               float& h3, float dist_out[6]);

// Build a Crystal from a CrystalParam variant using the given RNG. Matches
// the file-local CrystalMaker visitor used by Simulator (for non-deterministic
// params each call samples a fresh shape using the RNG).
Crystal MakeCrystal(RandomNumberGenerator& rng, const CrystalParam& param);

// Predicate: is `param` deterministic (no stochastic shape distribution on any
// of prism `h_`/`d_[6]` or pyramid `h_prs_`/`h_pyr_u_`/`h_pyr_l_`/`d_[6]`)?
// Deterministic params never consume the RNG in MakeCrystal, so callers use
// this to decide whether a cached Crystal/geometry upload may be reused across
// draws. Definition lives in simulator.cpp (single source, already exported at
// namespace scope — this header only adds the declaration so backends can see
// it without duplicating the visitor logic).
bool IsDeterministic(const CrystalParam& param);

// Single source for one question: "is this (layer, ci) about to produce a NEW
// crystal geometry, or is it re-deriving one this scene already sampled?"
//
// It exists so the backends that need that distinction share one judgement
// instead of hand-writing it each: two hand-written copies of a predicate that
// must agree are exactly how a semantic drifts (one excludes some path, the
// other forgets to). Callers own the counter; this owns only the verdict.
//
// Rules:
//   - a stochastic param is ALWAYS a new sample (each MakeCrystal draws a
//     different shape), and is never remembered;
//   - a deterministic param is a new sample the first time this scene asks for
//     it and a reuse for the rest of the scene — MakeCrystal is bit-identical
//     across repeated calls, so re-deriving it samples nothing new even when a
//     backend does rebuild it every batch;
//   - identity is the CrystalParam's ADDRESS, i.e. its (layer, ci) slot in the
//     scene, not its value: two slots that happen to carry equal params are two
//     populations, and the legacy path's own crystal cache keys the same way.
//
// Lifetime: one instance per backend instance = one Simulator::Run(), NOT one
// per session. A per-batch reset would degrade it to "did this ci repeat inside
// one batch", which is the very cross-batch double-count it exists to stop.
// Scene-pointer identity is the invalidation signal (the pointers in `sampled_`
// point into that scene's config), mirroring how the CUDA backend guards its
// geometry pool.
class NewSampleTracker {
 public:
  // Call at each BeginSession: clears the remembered set only when the scene
  // actually changed, so repeated batches of one scene keep accumulating.
  void ResetIfSceneChanged(const void* scene) {
    if (scene == scene_) {
      return;
    }
    scene_ = scene;
    sampled_.clear();
  }

  // True = count this as a fresh geometry sample. See the rules above.
  bool MarkIfNew(const CrystalParam& param) {
    if (!IsDeterministic(param)) {
      return true;
    }
    const CrystalParam* key = &param;
    for (const auto* seen : sampled_) {
      if (seen == key) {
        return false;
      }
    }
    sampled_.push_back(key);
    return true;
  }

 private:
  const void* scene_ = nullptr;
  // Deterministic (layer, ci) params already sampled in `scene_`. Linear scan:
  // the size is the scene's (layer, ci) count — single digits in practice.
  std::vector<const CrystalParam*> sampled_;
};

}  // namespace lumice

#endif  // CORE_TRACE_OPS_H_
