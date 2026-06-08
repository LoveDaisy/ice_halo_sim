#ifndef CORE_TRACE_OPS_H_
#define CORE_TRACE_OPS_H_

#include <cstddef>

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
// All function bodies live in simulator.cpp.

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

// Build a Crystal from a CrystalParam variant using the given RNG. Matches
// the file-local CrystalMaker visitor used by Simulator (for non-deterministic
// params each call samples a fresh shape using the RNG).
Crystal MakeCrystal(RandomNumberGenerator& rng, const CrystalParam& param);

}  // namespace lumice

#endif  // CORE_TRACE_OPS_H_
