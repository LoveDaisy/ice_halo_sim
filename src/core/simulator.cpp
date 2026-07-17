#include "core/simulator.hpp"

#include <algorithm>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <numeric>
#include <optional>
#include <string>
#include <thread>
#include <variant>

#include "config/color_gate_table.hpp"
#include "config/component_table.hpp"
#include "config/crystal_config.hpp"
#include "config/light_config.hpp"
#include "config/proj_config.hpp"
#include "config/raypath_color_config.hpp"
#include "config/render_config.hpp"
#include "config/sim_data.hpp"
#include "core/backend/cpu_trace_backend.hpp"
#include "core/backend/trace_backend.hpp"
#include "core/buffer.hpp"
#include "core/crystal.hpp"
#include "core/filter_spec.hpp"
#include "core/lat_lut.hpp"
#include "core/math.hpp"
#include "core/optics.hpp"
#include "core/shared/lat_path_selection.hpp"
#include "util/env_knobs.hpp"
#include "util/illuminant.hpp"
#include "util/logger.hpp"
#include "util/queue.hpp"

#if defined(__APPLE__)
#include "core/backend/metal_trace_backend.hpp"
#endif

#if defined(LUMICE_CUDA_ENABLED)
#include "core/backend/cuda_trace_backend.hpp"
#endif

namespace lumice {

namespace detail {

// Maps a triangle id to its polygon-face index via argmax of the dot product
// against polygon-face normals (kFaceCoplanarFloor=1e-2). Mirrors the
// BuildPolygonFaceData side (crystal.cpp); a first-match `dot > 1-1e-3` cannot
// distinguish adjacent upper-pyramid faces on extreme-wedge (~≥87.4°) crystals
// because their normals' dot ≈ 0.9994. Returns kInvalidId only if the best
// match is below the sanity floor (~8.1° slack), which should not happen for
// valid crystals. Exposed in detail:: for unit-test access; not part of the
// public API.
// NOTE: kFaceCoplanarFloor must match the value in crystal.cpp::BuildPolygonFaceData.
IdType PolygonFaceOfTri(const Crystal& crystal, int tri_id) {
  const float* tn = crystal.GetTriangleNormal() + tri_id * 3;
  const float* pn = crystal.GetPolygonFaceNormal();
  constexpr float kFaceCoplanarFloor = 1e-2f;
  int best_p = -1;
  float best_dot = -1.0f;
  for (size_t p = 0; p < crystal.PolygonFaceCount(); p++) {
    float dot = Dot3(tn, pn + p * 3);
    if (dot > best_dot) {
      best_dot = dot;
      best_p = static_cast<int>(p);
    }
  }
  if (best_p < 0 || best_dot < 1.0f - kFaceCoplanarFloor) {
    return kInvalidId;
  }
  return static_cast<IdType>(best_p);
}

}  // namespace detail

/**
 * @brief Sample on crystal & init origin (p, from_face_, to_face_) of rays.
 *        NOTE: direction of rays should be given.
 */
void InitRay_p_fid(const Crystal& curr_crystal, RayBuffer* ray_buf_ptr) {
  if (!ray_buf_ptr) {
    return;
  }

  RayBuffer& ray_buf = *ray_buf_ptr;
  // p & to_face_: sample on crystal triangles (area-weighted), then map to polygon face.
  auto total_faces = curr_crystal.TotalTriangles();
  const auto* face_area = curr_crystal.GetTirangleArea();
  const auto* face_norm = curr_crystal.GetTriangleNormal();
  const auto* face_vtx = curr_crystal.GetTriangleVtx();

  constexpr size_t kMaxTriangles = 64;
  float proj_prob_buf[kMaxTriangles];
  std::unique_ptr<float[]> proj_prob_heap;
  float* proj_prob = proj_prob_buf;
  if (total_faces > kMaxTriangles) {
    proj_prob_heap = std::make_unique<float[]>(total_faces);
    proj_prob = proj_prob_heap.get();
  }
  for (auto& r : ray_buf) {
    const auto* d = r.d_;
    for (size_t j = 0; j < total_faces; j++) {
      proj_prob[j] = std::max(-Dot3(d, face_norm + j * 3) * face_area[j], 0.0f);
    }
    int tri_id = 0;
    RandomSample(total_faces, proj_prob, &tri_id);
    SampleTrianglePoint(face_vtx + tri_id * 9, r.p_);
    // Initial entry segment: no source face, hit face = the sampled one's polygon.
    r.from_face_ = kInvalidId;
    r.to_face_ = detail::PolygonFaceOfTri(curr_crystal, tri_id);
    if (r.to_face_ == kInvalidId) {
      // Triangle has no matching polygon face — should not happen for a valid crystal.
      // Zero weight to suppress downstream contribution; HitSurface also guards
      // kInvalidId at loop entry, preventing any OOB access.
      LOG_WARNING("PolygonFaceOfTri: tri {} has no matching polygon face; zeroing ray weight", tri_id);
      r.w_ = 0.0f;
    }
  }
}

static void SampleRayDir(const SunParam& p, float* d, size_t num, size_t step) {
  SampleSphCapPoint(p.azimuth_ + 180.0f, -p.altitude_, p.diameter_ / 2.0f, d, num, step);
}

/**
 * @brief Set initial value of d & w (direction and intensity) for rays.
 */
void InitRay_d_w_previdx(const SunParam& light_param, const WlParam& wl_param, size_t ray_num,  // input
                         RayBuffer* ray_buf_ptr) {                                              // output
  if (!ray_buf_ptr) {
    return;
  }
  const auto& ray_buf = *ray_buf_ptr;

  // w, prev_ray_idx: set init weight & previous ray index
  for (auto& r : ray_buf) {
    r.w_ = wl_param.weight_;
    r.prev_ray_idx_ = kInfSize;
  }

  // d: sample direction
  SampleRayDir(light_param, ray_buf[0].d_, ray_num, sizeof(RaySeg));

  // Then rotate
  for (auto& r : ray_buf) {
    r.crystal_rot_.ApplyInverse(r.d_);
  }
}


Rotation BuildCrystalRotation(float azimuth_rad, float latitude_rad, float roll_rad) {
  static constexpr float kEy[3] = { 0, 1, 0 };
  static constexpr float kEz[3] = { 0, 0, 1 };
  // Chain reads inner-to-outer (Rotation::Chain left-multiplies):
  //   inner Rz(roll) -> middle Ry(latitude - pi/2) = Ry(-zenith) -> outer Rz(azimuth - pi).
  // Resulting matrix: R = Rz(azimuth - pi) * Ry(-zenith) * Rz(roll).
  return Rotation(kEz, roll_rad).Chain(kEy, latitude_rad - math::kPi_2).Chain(kEz, azimuth_rad - math::kPi);
}


void InitRay_rot(RandomNumberGenerator& rng, const AxisDistribution& crystal_axis,  // input
                 RayBuffer buffer_data[2]) {                                        // output
  float lon_lat_roll[3]{};
  // Resolve the shared latitude LUT ONCE per crystal-batch: crystal_axis is
  // constant across this buffer, so a per-ray cache lookup (the old CachedLatLut)
  // is wasteful, and — worse — pulling the shared build-once cache on every ray
  // would serialize all worker threads on its mutex. Resolve here, pass the
  // pointer into the per-ray sampler (task-335). Only the LUT-routed families
  // need it; full-sphere / legacy-gauss / no-random take their own sampler
  // branch and ignore a nullptr.
  const bool full_sphere = crystal_axis.IsFullSphereUniform();
  const LatLut* lat_lut = nullptr;
  if (!full_sphere && lat_path::SelectLatPath(crystal_axis).kind == lat_path::LatPathKind::kLutInverseCdf) {
    lat_lut = GetSharedLatLut(crystal_axis.latitude_dist);
  }
  for (auto& r : buffer_data[0]) {
    if (!full_sphere) {
      RandomSampler::SampleSphericalPointsSph(crystal_axis, lon_lat_roll, 1, lat_lut);
    } else {
      // Randomly sample on sphere (with asin(u) Jacobian correction); roll sampled separately below.
      RandomSampler::SampleSphericalPointsSph(lon_lat_roll);
      lon_lat_roll[2] = rng.Get(crystal_axis.roll_dist) * math::kDegreeToRad;
    }
    r.crystal_rot_ = BuildCrystalRotation(lon_lat_roll[0], lon_lat_roll[1], lon_lat_roll[2]);
  }
}


void InitRay_other_info(const Crystal& curr_crystal, size_t curr_crystal_id, size_t all_data_idx,  // input
                        RayBuffer buffer_data[2]) {                                                // output
  for (size_t i = 0; i < buffer_data[0].size_; i++) {
    auto& r = buffer_data[0][i];
    r.crystal_idx_ = curr_crystal_id;
    r.crystal_config_id_ = curr_crystal.config_id_;
    r.root_ray_idx_ = all_data_idx++;
    buffer_data[0].RecorderClear(i);
    buffer_data[0].RecorderAppend(i, curr_crystal.GetFn(r.to_face_));
  }
}


// NOLINTNEXTLINE(readability-function-size)
void InitRayFirstMs(RandomNumberGenerator& rng, const SunParam& light_param, const WlParam& wl_param,
                    size_t curr_ray_num,                                                                        // input
                    const Crystal& curr_crystal, size_t curr_crystal_id, const AxisDistribution& crystal_axis,  // input
                    RayBuffer buffer_data[2], RayBuffer& all_data) {  // output
  buffer_data[0].size_ = curr_ray_num;

  // 1.0 init crystal_rot
  InitRay_rot(rng, crystal_axis, buffer_data);

  // 1.1 init d & w & (prev_ray_idx)
  InitRay_d_w_previdx(light_param, wl_param, curr_ray_num, buffer_data + 0);

  // 1.2 init p & fid
  InitRay_p_fid(curr_crystal, buffer_data + 0);

  // 1.3 init crystal_id, root_ray_idx, rp, state
  InitRay_other_info(curr_crystal, curr_crystal_id, all_data.size_, buffer_data);

  // 1.4 First-MS reset of the per-ray component mask (task-331.1). The mask is
  // the only cross-MS-layer per-ray state added since scrum-237 removed
  // is_prior_filter_failed_/anchor lane. Reset MUST happen here and NOT inside
  // InitRay_other_info: the latter is shared with InitRayOtherMs, and zeroing
  // the mask on subsequent layers would break the "first-layer reset, then OR
  // forward" semantics that T2/T3 rely on.
  for (size_t i = 0; i < buffer_data[0].size_; i++) {
    buffer_data[0].SetComponent(i, 0);
  }

  all_data.EmplaceBack(buffer_data[0]);
}


// NOLINTNEXTLINE(readability-function-size)
void InitRayOtherMs(RandomNumberGenerator& rng, const RayBuffer init_data[2], size_t curr_ray_num,              // input
                    const Crystal& curr_crystal, size_t curr_crystal_id, const AxisDistribution& crystal_axis,  // input
                    RayBuffer buffer_data[2], RayBuffer& all_data, size_t& init_ray_offset) {  // output
  buffer_data[0].size_ = 0;

  // 1.0 copy previous rays.
  buffer_data[0].EmplaceBack(init_data[0], init_ray_offset, curr_ray_num);
  init_ray_offset += curr_ray_num;

  // 1.1 init crystal_rot
  InitRay_rot(rng, crystal_axis, buffer_data);
  // Then rotate d
  for (auto& r : buffer_data[0]) {
    r.crystal_rot_.ApplyInverse(r.d_);
  }

  // 1.2 init p & fid
  InitRay_p_fid(curr_crystal, buffer_data + 0);

  // 1.3 init crystal_id, crystal_config_id, root_ray_idx, rp, state
  InitRay_other_info(curr_crystal, curr_crystal_id, all_data.size_, buffer_data);

  // 1.4 reset is_continue_ (pool-reuse guard: init_data carries is_continue_=true
  // from CollectData; must reset before entering the new MS layer's hit loop)
  // See doc/raypath-rayseg-architecture.md §3 "Reset Points Summary".
  for (auto& r : buffer_data[0]) {
    r.is_continue_ = false;
  }

  all_data.EmplaceBack(buffer_data[0]);
}


struct CrystalMaker {
  RandomNumberGenerator& rng_;

  Crystal operator()(const PrismCrystalParam& p) {
    // Heights are folded with std::abs — a negative height has no physical
    // interpretation independent of crystal orientation. face_distance is signed:
    // negative d shifts the plane past the origin, still yielding a valid convex
    // body as long as opposite-pair sums d_i + d_{i+3} > 0 (verified downstream
    // by Crystal::CreatePrism's Euler-manifold gate; degenerate combos are
    // rejected as zero-triangle Crystals rather than silently absolute-valued at
    // the sampler boundary, which was hiding an entire input path from the
    // geometry pipeline).
    float h = std::abs(rng_.Get(p.h_));
    float dist[6]{};
    for (int i = 0; i < 6; i++) {
      dist[i] = rng_.Get(p.d_[i]);
    }
    return Crystal::CreatePrism(h, dist);
  }

  Crystal operator()(const PyramidCrystalParam& p) {
    // Same rationale as the prism branch: heights fold, face_distance is signed.
    float h1 = std::abs(rng_.Get(p.h_pyr_u_));
    float h2 = std::abs(rng_.Get(p.h_prs_));
    float h3 = std::abs(rng_.Get(p.h_pyr_l_));
    float dist[6]{};
    for (int i = 0; i < 6; i++) {
      dist[i] = rng_.Get(p.d_[i]);
    }
    return Crystal::CreatePyramid(p.wedge_angle_u_, p.wedge_angle_l_, h1, h2, h3, dist);
  }
};


Crystal MakeCrystal(RandomNumberGenerator& rng, const CrystalParam& param) {
  return std::visit(CrystalMaker{ rng }, param);
}


bool IsDeterministic(const CrystalParam& param) {
  auto all_no_random = [](const Distribution* d, size_t n) {
    return !std::any_of(d, d + n, [](const auto& x) { return x.type != DistributionType::kNoRandom; });
  };
  return std::visit(
      [&](const auto& p) {
        if constexpr (std::is_same_v<std::decay_t<decltype(p)>, PrismCrystalParam>) {
          return p.h_.type == DistributionType::kNoRandom && all_no_random(p.d_, 6);
        } else {
          return p.h_prs_.type == DistributionType::kNoRandom &&  //
                 p.h_pyr_u_.type == DistributionType::kNoRandom && p.h_pyr_l_.type == DistributionType::kNoRandom &&
                 all_no_random(p.d_, 6);
        }
      },
      param);
}


RayBuffer AllocateAllData(const SceneConfig& config, size_t ray_num) {
  // Calculate total rays (expected value) used in the whole simulation.
  // total = n * (2 * k + 1) * (1 + k * p1 + k^2 * p1 * p2 + k^3 * p1 * p2 * p3 + ...)
  // where k = max_hits
  size_t total_ray_num = ray_num * (2 * config.max_hits_ + 1);
  float x = 1;
  float y = 1;
  float x_max = 1;
  float y_max = 1;
  for (size_t i = 0; i + 1 < config.ms_.size(); i++) {
    y *= config.max_hits_ * config.ms_[i].prob_;
    x += y;
    y_max *= config.max_hits_;
    x_max += y_max;
  }
  total_ray_num = static_cast<size_t>(total_ray_num * std::min(x * 1.5f, x_max));

  return RayBuffer(total_ray_num);
}


std::unique_ptr<size_t[]> PartitionCrystalRayNum(const std::vector<float>& proportions, size_t ray_num,
                                                 std::vector<double>& carry) {
  auto crystal_cnt = proportions.size();
  auto c_num = std::make_unique<size_t[]>(crystal_cnt);
  assert(carry.size() == crystal_cnt);
  if (crystal_cnt == 0 || ray_num == 0) {
    return c_num;
  }

  // Normalize proportions
  float total_prop = 0.0f;
  for (size_t ci = 0; ci < crystal_cnt; ci++) {
    total_prop += std::max(0.0f, proportions[ci]);
  }
  if (total_prop <= 0.0f) {
    return c_num;
  }

  // Per-crystal carry with largest-remainder correction.
  // Each crystal independently accumulates its fractional remainder in carry[ci].
  // All crystals use floor(ideal) first, then the difference (ray_num - assigned)
  // is distributed by largest-remainder to guarantee exact total.
  // Note: carry[ci] can be negative after deficit correction (a "debt" that is repaid
  // in subsequent batches). We clamp ideal to 0 before casting to size_t to avoid UB.
  size_t assigned = 0;
  for (size_t ci = 0; ci < crystal_cnt; ci++) {
    double ideal = carry[ci] + (static_cast<double>(std::max(0.0f, proportions[ci])) / total_prop) * ray_num;
    auto alloc = static_cast<size_t>(std::max(0.0, ideal));  // Clamp: negative ideal → 0 alloc
    carry[ci] = ideal - static_cast<double>(alloc);
    c_num[ci] = alloc;
    assigned += alloc;
  }

  // Largest-remainder correction: distribute deficit or surplus.
  if (assigned != ray_num) {
    std::vector<size_t> idx(crystal_cnt);
    std::iota(idx.begin(), idx.end(), 0);

    if (assigned < ray_num) {
      // Deficit: give extra rays to crystals with largest carry (closest to rounding up).
      size_t deficit = ray_num - assigned;
      std::partial_sort(idx.begin(), idx.begin() + deficit, idx.end(),
                        [&carry](size_t a, size_t b) { return carry[a] > carry[b]; });
      for (size_t i = 0; i < deficit; i++) {
        c_num[idx[i]]++;
        carry[idx[i]] -= 1.0;
      }
    } else {
      // Surplus: reclaim rays from crystals with smallest carry that have alloc > 0.
      size_t surplus = assigned - ray_num;
      // Partition: crystals with alloc > 0 first, then sort by carry ascending.
      auto reclaimable_end = std::partition(idx.begin(), idx.end(), [&c_num](size_t i) { return c_num[i] > 0; });
      std::partial_sort(idx.begin(),
                        idx.begin() + std::min(surplus, static_cast<size_t>(reclaimable_end - idx.begin())),
                        reclaimable_end, [&carry](size_t a, size_t b) { return carry[a] < carry[b]; });
      for (size_t i = 0; i < surplus && idx.begin() + i < reclaimable_end; i++) {
        c_num[idx[i]]--;
        carry[idx[i]] += 1.0;
      }
    }
  }

  return c_num;
}


void TraceRayBasicInfo(const Crystal& curr_crystal, float refractive_index, size_t curr_ray_num,
                       RayBuffer* buffer_data) {
  // 1 HitSurface.
  {
    float_bf_t d_in{ buffer_data[0][0].d_, sizeof(RaySeg) };
    float_bf_t w_in{ &buffer_data[0][0].w_, sizeof(RaySeg) };
    id_bf_t to_face_in{ &buffer_data[0][0].to_face_, sizeof(RaySeg) };
    float_bf_t d_out{ buffer_data[1][0].d_, sizeof(RaySeg) };
    float_bf_t w_out{ &buffer_data[1][0].w_, sizeof(RaySeg) };
    HitSurface(curr_crystal, refractive_index, curr_ray_num,  // Input
               d_in, w_in, to_face_in,                        // Input, d, w, to_face
               d_out, w_out);                                 // Output, d, w
  }

  // 2 Propagate.
  // Source-face guard reuses the input segment's to_face_ as the next segment's
  // from_face_ (the face the ray just hit). step=2 is shared by reflect+refract
  // (both children have the same source face).
  {
    float_bf_t d_in{ buffer_data[1][0].d_, sizeof(RaySeg) };
    float_bf_t w_in{ &buffer_data[1][0].w_, sizeof(RaySeg) };
    float_bf_t p_in{ buffer_data[0][0].p_, sizeof(RaySeg) };
    id_bf_t from_face_in{ &buffer_data[0][0].to_face_, sizeof(RaySeg) };
    float_bf_t p_out{ buffer_data[1][0].p_, sizeof(RaySeg) };
    id_bf_t to_face_out{ &buffer_data[1][0].to_face_, sizeof(RaySeg) };
    Propagate(curr_crystal, curr_ray_num * 2, 2,  // Input
              d_in, p_in, w_in,                   // Input, d, w, p(1/2)
              from_face_in,                       // Source face ids (step=2, shared by reflect+refract pair)
              p_out, to_face_out);                // Output, p, to_face
  }

  for (size_t i = 0; i < curr_ray_num; i++) {
    // Fan-out hot path (#247.4 Round 2): RecorderFanOut routes the trivial-POD
    // copy in-place (recorders_ memcpy ×2) and only takes the arena dup branch
    // for overflow rays — cold under bench(max_hits=8).
    buffer_data[1].RecorderFanOut(buffer_data[0], i, i * 2 + 0, i * 2 + 1);
    // task-331.1: mirror the recorder fan-out for the per-ray component mask.
    // Both child segments inherit the parent's mask verbatim (T2/T3 OR-produce
    // bits later; this fan-out keeps the mask propagating through the hit
    // loop). Currently a no-op semantically (all masks are 0 in T1), but wiring
    // it now means T2 doesn't have to touch simulator.cpp fan-out code.
    buffer_data[1].ComponentFanOut(buffer_data[0], i, i * 2 + 0, i * 2 + 1);
    // Each child segment's from_face_ = parent's to_face_ (the face just hit).
    buffer_data[1][i * 2 + 0].from_face_ = buffer_data[0][i].to_face_;
    buffer_data[1][i * 2 + 1].from_face_ = buffer_data[0][i].to_face_;
    buffer_data[1][i * 2 + 0].crystal_idx_ = buffer_data[0][i].crystal_idx_;
    buffer_data[1][i * 2 + 1].crystal_idx_ = buffer_data[0][i].crystal_idx_;
    buffer_data[1][i * 2 + 0].crystal_config_id_ = buffer_data[0][i].crystal_config_id_;
    buffer_data[1][i * 2 + 1].crystal_config_id_ = buffer_data[0][i].crystal_config_id_;
    buffer_data[1][i * 2 + 0].crystal_rot_ = buffer_data[0][i].crystal_rot_;
    buffer_data[1][i * 2 + 1].crystal_rot_ = buffer_data[0][i].crystal_rot_;
    buffer_data[1][i * 2 + 0].root_ray_idx_ = buffer_data[0][i].root_ray_idx_;
    buffer_data[1][i * 2 + 1].root_ray_idx_ = buffer_data[0][i].root_ray_idx_;
  }

  buffer_data[0].size_ = 0;
  buffer_data[1].size_ = curr_ray_num * 2;
}


void FillRayOtherInfo(const Crystal& curr_crystal, RayBuffer buffer_data[2]) {
  for (size_t j = 0; j < buffer_data[1].size_; j++) {
    auto& r = buffer_data[1][j];
    if (r.to_face_ != kInvalidId) {
      buffer_data[1].RecorderAppend(j, curr_crystal.GetFn(r.to_face_));
    }
  }
}


// Design A filter semantics: filter-fail = ray terminates (not outgoing, not continue).
// See doc/raypath-rayseg-architecture.md §3 for the segment state-machine transition rules.
//
// Multi-group implementation (scrum-color-predicate-symmetry): `color_groups`
// carries one entry per distinct color-predicate symmetry value at this
// placement. Physical filter check + prob roll still happen exactly once per
// ray (outside the group loop) so RNG order is invariant to color-config
// changes. Each group contributes its matched summand bits into the ray's
// mask via OR; distinct groups are additive.
// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void CollectData(RandomNumberGenerator& rng, const MsInfo& ms_info, const FilterSpec* spec,  // input
                 RayBuffer* buffer_data, RayBuffer* init_data,                               // output
                 const std::vector<ColorSpecGroup>* color_groups) {                          // input
  for (size_t idx = 0; idx < buffer_data[1].size_; idx++) {
    auto& r = buffer_data[1][idx];
    const auto& rec = buffer_data[1].RecorderAt(idx);
    // Explicit reset: guard against stale bool from pool-reused buffer slots.
    r.is_continue_ = false;

    if (r.w_ < 0) {
      // 0. Total reflection — rotate to world space.
      r.crystal_rot_.Apply(r.d_);
      r.crystal_rot_.Apply(r.p_);
    } else if (r.to_face_ == kInvalidId) {
      // 1. Outgoing candidates. Rotate to world space first so filter operates in world coordinates.
      r.crystal_rot_.Apply(r.d_);
      r.crystal_rot_.Apply(r.p_);

      // Design 2: gate decision is pure `spec->Check()` (physical filter);
      // color predicates are a SEPARATE `color_spec->MatchSummandMask()` pass
      // evaluated only AFTER the physical filter admits the ray. The two
      // predicates are decoupled — physical filter decides survival, color
      // spec decides tagging (never touches `w_` / `is_continue_`).
      const uint8_t* arena = buffer_data[1].OverflowArena();
      bool filter_pass = (spec == nullptr) ? true : spec->Check(r, rec, arena);
      if (filter_pass) {
        // Non-destructive color pass: for each symmetry-group color spec,
        // evaluate its per-summand mask and OR the mapped bits into the ray's
        // carried mask. Zero-cost when color_groups is null / empty (AC3).
        if (color_groups != nullptr && !color_groups->empty()) {
          uint64_t produced = 0;
          for (const auto& g : *color_groups) {
            if (g.spec == nullptr || g.bits == nullptr) {
              continue;
            }
            uint64_t summand_mask = 0;
            (void)g.spec->CheckSummandMask(r, rec, arena, &summand_mask);
            if (summand_mask == 0) {
              continue;
            }
            const std::vector<uint8_t>& cb = *g.bits;
            for (size_t k = 0; k < cb.size() && k < 64; k++) {
              if ((summand_mask & (1ull << k)) != 0) {
                uint8_t bit = cb[k];
                if (bit < ComponentTable::kMaxBits) {
                  produced |= (1ull << bit);
                }
              }
            }
          }
          if (produced != 0) {
            buffer_data[1].SetComponent(idx, buffer_data[1].ComponentAt(idx) | produced);
          }
        }
        if (rng.GetUniform() < ms_info.prob_) {
          // 1.1 filter-pass + prob-pass → continue to next MS level.
          r.is_continue_ = true;
        }
        // 1.2 filter-pass + prob-fail → emit outgoing (IsOutgoing() = true).
      } else {
        // 1.3 filter-fail → ray terminates. Reuse TIR sentinel (w_<0) so the ray
        // is excluded from IsOutgoing() and IsContinue(). Direction/position are
        // already in world space from the Apply() above, matching real-TIR layout.
        r.w_ = -1.0f;
      }
    }
    // 2. Normal rays (to_face_ != kInvalidId && w_ >= 0) stay in crystal-local coordinates.

    // task-331.1: capture the source mask once. Single-ray EmplaceBack
    // deliberately does not touch components_ (its signature stays untouched
    // to avoid ripple-changing every caller); the mask hand-off is explicit
    // here so T2 has an obvious insertion point for the "OR in this layer's
    // hit bits" step and so pool-reused destination slots don't leak stale
    // values from earlier iterations.
    uint64_t src_component = buffer_data[1].ComponentAt(idx);
    if (r.IsNormal()) {
      // rec comes from buffer_data[1]; pass it as arena_src so overflow slots
      // get duped into buffer_data[0]'s own arena (fix for max_hits>kInlineCap
      // null-deref: the 2-arg overload would copy overflow_idx_ pointing at the
      // wrong arena and crash the next RecorderFanOut).
      size_t dst_idx = buffer_data[0].size_;  // read before EmplaceBack mutates size_.
      buffer_data[0].EmplaceBack(r, rec, buffer_data[1]);
      // EmplaceBack loses one slot when size_+1==capacity_ — replay that guard
      // so we don't SetComponent past size_. size_ moved iff the ray was
      // actually appended.
      if (buffer_data[0].size_ > dst_idx) {
        buffer_data[0].SetComponent(dst_idx, src_component);
      }
    }
    if (r.IsContinue()) {
      size_t dst_idx = init_data[1].size_;
      init_data[1].EmplaceBack(r, rec, buffer_data[1]);
      if (init_data[1].size_ > dst_idx) {
        init_data[1].SetComponent(dst_idx, src_component);
      }
    }
  }
}

// Two-parameter overload preserved so existing single-spec / single-bits-vector
// call sites (unit tests + any consumer that has not migrated to the multi-group
// path) keep compiling. Forwards to the multi-group implementation by wrapping
// the pair into a single-element ColorSpecGroup, or by passing null when the
// caller signalled "no color pass".
void CollectData(RandomNumberGenerator& rng, const MsInfo& ms_info, const FilterSpec* spec,  // input
                 RayBuffer* buffer_data, RayBuffer* init_data,                               // output
                 const FilterSpec* color_spec, const std::vector<uint8_t>* color_bits) {     // input
  if (color_spec == nullptr || color_bits == nullptr) {
    CollectData(rng, ms_info, spec, buffer_data, init_data, static_cast<const std::vector<ColorSpecGroup>*>(nullptr));
    return;
  }
  std::vector<ColorSpecGroup> groups{ { color_spec, color_bits } };
  CollectData(rng, ms_info, spec, buffer_data, init_data, &groups);
}


namespace {
// Global atomic counter feeding `Simulator::effective_seed_` when the user-facing
// `seed_` is 0 (task 260.6). `fetch_add(1) + 1u` maps the sequence 0,1,2,...
// to 1,2,3,..., guaranteeing the derived seed is non-zero (activates the Metal
// backend's `gen_seed_ != 0` device-gen gate) and pairwise distinct across
// Simulator instances regardless of which thread constructs them. After 2^32
// constructions the counter wraps and the next `+1u` returns 0; in practice
// that point is unreachable (>4e9 Simulator constructions per process).
// NOTE: do NOT replace with `| 1u` — combined with an init of 0 it produces
// pairwise collisions (0→1, 1→1, 2→3, 3→3, ...).
std::atomic<uint32_t> g_simulator_seed_counter{ 0 };

uint32_t DeriveEffectiveSeed(uint32_t seed) {
  if (seed != 0) {
    return seed;
  }
  return g_simulator_seed_counter.fetch_add(1, std::memory_order_relaxed) + 1u;
}
}  // namespace

Simulator::Simulator(QueuePtrS<SimBatch> config_queue, QueuePtrS<SimData> data_queue, uint32_t seed)
    : config_queue_(std::move(config_queue)), data_queue_(std::move(data_queue)), stop_(false), idle_(true),
      seed_(seed), effective_seed_(DeriveEffectiveSeed(seed)),
      rng_(seed != 0 ? seed :
                       static_cast<uint32_t>(std::chrono::system_clock::now().time_since_epoch().count() ^
                                             (std::hash<std::thread::id>{}(std::this_thread::get_id())))) {}

Simulator::Simulator(Simulator&& other) noexcept
    : config_queue_(std::move(other.config_queue_)), data_queue_(std::move(other.data_queue_)),
      stop_(other.stop_.load()), idle_(other.idle_.load()), seed_(other.seed_), effective_seed_(other.effective_seed_),
      rng_(other.rng_), logger_(std::move(other.logger_)),
      preferred_backend_(other.preferred_backend_.load(std::memory_order_acquire)) {}

Simulator& Simulator::operator=(Simulator&& other) noexcept {
  if (this == &other) {
    return *this;
  }

  config_queue_ = std::move(other.config_queue_);
  data_queue_ = std::move(other.data_queue_);
  stop_ = other.stop_.load();
  idle_ = other.idle_.load();
  seed_ = other.seed_;
  effective_seed_ = other.effective_seed_;
  rng_ = other.rng_;
  logger_ = std::move(other.logger_);
  preferred_backend_.store(other.preferred_backend_.load(std::memory_order_acquire), std::memory_order_release);
  return *this;
}

void Simulator::SetPreferredBackend(BackendKind backend) {
  preferred_backend_.store(backend, std::memory_order_release);
}

namespace {

// Create a TraceBackend instance for this Run() entry. Precedence:
//   1) env-var LUMICE_TRACE_BACKEND (debug/CI override; legacy semantics):
//      - unset / empty / "legacy"   -> falls through to step 2
//      - "cpu_backend"               -> CpuTraceBackend
//      - "metal" on Apple            -> MetalTraceBackend
//      - "cuda" (LUMICE_CUDA_ENABLED) -> CudaTraceBackend if an eligible device
//                                       (>= sm_61) is present, else legacy CPU
//      - unknown / unavailable       -> nullptr (logged WARN)
//   2) preferred_backend (set by Server::SetPreferredBackend, default kCpu):
//      - kMetal on Apple             -> MetalTraceBackend
//      - kMetal elsewhere            -> nullptr (silent no-op; GUI checkbox
//                                       on non-Apple builds equates to CPU)
//      - kCuda (LUMICE_CUDA_ENABLED) -> CudaTraceBackend if an eligible device
//                                       (>= sm_61) is present, else legacy CPU;
//                                       runtime probe logged one-shot on both paths
//      - kCuda (no LUMICE_CUDA_ENABLED) -> nullptr (legacy CPU)
// Returns nullptr to keep the legacy CPU path (Simulator::SimulateOneWavelength).
std::unique_ptr<TraceBackend> CreateBackend(BackendKind preferred_backend, Logger& logger) {
  if (std::optional<std::string> override = env::TraceBackendOverride(logger)) {
    const std::string& name = *override;
    if (name.empty() || name == "legacy") {
      // fall through to preferred_backend
    } else if (name == "cpu_backend") {
      ILOG_INFO(logger, "LUMICE_TRACE_BACKEND=cpu_backend → routing via CpuTraceBackend");
      return std::make_unique<CpuTraceBackend>();
    } else if (name == "metal") {
#if defined(__APPLE__)
      ILOG_INFO(logger, "LUMICE_TRACE_BACKEND=metal → routing via MetalTraceBackend");
      return std::make_unique<MetalTraceBackend>(&logger);
#else
      ILOG_WARN(logger, "LUMICE_TRACE_BACKEND=metal requested on non-Apple platform; falling back to legacy CPU");
      return nullptr;
#endif
    } else if (name == "cuda") {
#if defined(LUMICE_CUDA_ENABLED)
      ILOG_INFO(logger, "CUDA availability probe: {}", CudaDeviceDiagnostics());
      if (CudaDeviceAvailable()) {
        ILOG_INFO(logger, "LUMICE_TRACE_BACKEND=cuda → routing via CudaTraceBackend");
        return std::make_unique<CudaTraceBackend>(&logger);
      }
      ILOG_WARN(logger, "LUMICE_TRACE_BACKEND=cuda requested but no eligible CUDA device; falling back to legacy CPU");
      return nullptr;
#else
      ILOG_WARN(logger,
                "LUMICE_TRACE_BACKEND=cuda requested but LUMICE_CUDA_ENABLED not set; falling back to legacy CPU");
      return nullptr;
#endif
    } else {
      ILOG_WARN(logger, "Unknown LUMICE_TRACE_BACKEND={}; falling back to preferred backend", name);
    }
  }
  // -Wswitch: exhaustive over BackendKind, no `default:` so adding a new enum
  // value forces the compiler to surface this site (and ResolveGpuRoute /
  // LUMICE_BackendAvailable below).
  switch (preferred_backend) {
    case BackendKind::kCpu:
      return nullptr;
    case BackendKind::kMetal:
#if defined(__APPLE__)
      ILOG_INFO(logger, "preferred_backend=metal → routing via MetalTraceBackend");
      return std::make_unique<MetalTraceBackend>(&logger);
#else
      return nullptr;  // non-Apple: silent no-op (CPU)
#endif
    case BackendKind::kCuda:
#if defined(LUMICE_CUDA_ENABLED)
      ILOG_INFO(logger, "CUDA availability probe: {}", CudaDeviceDiagnostics());
      if (CudaDeviceAvailable()) {
        ILOG_INFO(logger, "preferred_backend=cuda → routing via CudaTraceBackend");
        return std::make_unique<CudaTraceBackend>(&logger);
      }
      ILOG_WARN(logger, "preferred_backend=cuda but no eligible CUDA device; falling back to legacy CPU");
      return nullptr;
#else
      // CUDA backend gated behind LUMICE_CUDA_ENABLED=ON (dev49 docker /
      // CUDAToolkit hosts only). Mac / Windows builds without the gate take
      // the legacy CPU fallback.
      ILOG_WARN(logger, "preferred_backend=cuda but LUMICE_CUDA_ENABLED not set; falling back to legacy CPU");
      return nullptr;
#endif
  }
  return nullptr;
}

// Compatibility gate: even when a backend is selected, a particular batch may
// not be backend-eligible (multi-renderer config, lens/view limits, etc.).
// Falls back to legacy CPU on mismatch — logs WARN at most once per Run() via
// the per-Run() latches on `warned_*`.
bool CanUseBackend(const TraceBackend* backend, const SimBatch& batch, Logger& logger, bool& warned_no_renders,
                   bool& warned_multi_renderer, bool& warned_compat) {
  if (backend == nullptr) {
    return false;
  }
  if (!batch.renders_ || batch.renders_->empty()) {
    if (!warned_no_renders) {
      ILOG_WARN(logger, "TraceBackend selected but SimBatch carries no renders_; falling back to legacy CPU");
      warned_no_renders = true;
    }
    return false;
  }
  if (batch.renders_->size() != 1) {
    if (!warned_multi_renderer) {
      ILOG_WARN(logger, "TraceBackend path supports a single renderer only (got {}); falling back to legacy CPU",
                batch.renders_->size());
      warned_multi_renderer = true;
    }
    return false;
  }
  const auto& r = (*batch.renders_)[0];
  if (!backend->IsCompatible(r)) {
    if (!warned_compat) {
      ILOG_WARN(logger, "backend incompatible with render config (lens_type={}, el={:.2f}); falling back to legacy CPU",
                static_cast<int>(r.lens_.type_), r.view_.el_);
      warned_compat = true;
    }
    return false;
  }
  return true;
}

}  // namespace

void Simulator::Run() {
  stop_ = false;
  ILOG_DEBUG(logger_, "Simulator::Run: entry");

  // When a fixed seed is provided, also seed the thread-local global RNG singleton
  // used by sampling functions (RandomSample, SampleTrianglePoint, SampleSphCapPoint, etc.)
  // to ensure fully deterministic behavior.
  if (seed_ != 0) {
    RandomNumberGenerator::GetInstance().SetSeed(seed_);
  }

  // Pick a TraceBackend once per Run() entry. nullptr keeps the legacy CPU
  // path (zero behavior change when LUMICE_TRACE_BACKEND is unset).
  //
  // ARCHITECTURAL INVARIANT (task-260.5, scrum-258.10): backend instances are
  // created PER Run() and never pooled or reused across Run()s. MetalTraceBackend
  // relies on this to implement the !seeded gate that resets both the RNG state
  // and root_ray_count exactly once per Run() — if a future refactor introduces
  // a backend pool, that gate's semantics must be revisited (cross-Run() PCG
  // determinism + counter rollover both depend on the per-Run() lifecycle here).
  auto backend = CreateBackend(preferred_backend_.load(std::memory_order_acquire), logger_);
  // scrum-312 third-clock drain cadence cap + fresh window per Run().
  xyz_drain_batches_ = env::XyzDrainBatches(logger_, kDefaultXyzDrainBatches);
  geom_clock_ = env::GeomClock(logger_, kSmallBatchRayNum);
  xyz_win_ = XyzDrainWindow{};
  bool warned_no_renders = false;
  bool warned_multi_renderer = false;
  bool warned_compat = false;

  CrystalCache crystal_cache;
  SimWorkspace workspace;
  std::vector<std::vector<double>> ray_alloc_carry;
  uint64_t prev_generation = 0;

  while (true) {
    // scrum-312 third-clock: producer-pause flush. When no batch is queued we are
    // about to block; drain any pending device XYZ window first so the consumer/
    // GUI sees the latest image at the natural display cadence (the producer
    // feeds a burst, pauses, we drain) instead of waiting for the next burst.
    if (xyz_win_.pending && config_queue_->Empty()) {
      DrainDeviceXyz(backend.get());
    }
    auto batch = config_queue_->Get();  // Will block until get one
    if (batch.ray_num_ == 0 || stop_) {
      ILOG_DEBUG(logger_, "Simulator::Run: exit (ray_num={}, stop={})", batch.ray_num_, stop_.load());
      break;
    }

    if (!batch.scene_) {
      ILOG_DEBUG(logger_, "Simulator::Run: exit (null scene)");
      break;
    }
    const auto& config = *batch.scene_;
    auto generation = batch.generation_;
    idle_ = false;

    // Reset carry when config changes (new generation = new proportions).
    if (generation != prev_generation) {
      ray_alloc_carry.clear();
      // scrum-312 third-clock: flush the old generation's device XYZ window
      // BEFORE any new-generation batch traces into the (shared, persistent)
      // device buffer — otherwise the new generation's atomicAdds would mix into
      // the old generation's un-drained accumulation.
      DrainDeviceXyz(backend.get());
      prev_generation = generation;
    }

    bool use_backend =
        CanUseBackend(backend.get(), batch, logger_, warned_no_renders, warned_multi_renderer, warned_compat);

    // task-282: BackendUnavailableError signals the backend cannot run this
    // session (Metal PSO build failure on macOS 26.5, etc.). On first miss
    // we drop the backend instance for the remainder of the Run() (resetting
    // `backend` flips `use_backend` to false on the next CanUseBackend call)
    // and execute the current wavelength on the legacy CPU path here. Crystal
    // cache + workspace + ray_alloc_carry are in Run() scope, so the fallback
    // is a direct re-dispatch — no signature plumbing into the helper.
    auto run_with_backend = [&](const WlParam& wl_param) {
      // Self-guard: every current call site gates on a non-null backend, but this
      // keeps the lambda safe if a future caller forgets — same effect as the
      // catch-block fallback (run the legacy CPU path, report "backend not used").
      if (!backend) {
        SimulateOneWavelength(config, batch.raypath_color_.get(), wl_param, batch.ray_num_, crystal_cache, workspace,
                              generation, ray_alloc_carry);
        return false;
      }
      try {
        SimulateOneWavelengthWithBackend(*backend, config, (*batch.renders_)[0], batch.raypath_color_, wl_param,
                                         batch.ray_num_, generation);
        return true;
      } catch (const BackendUnavailableError& e) {
        ILOG_WARN(
            logger_,
            "TraceBackend unavailable ({}); dropping backend and falling back to legacy CPU for the rest of this Run()",
            e.what());
        backend.reset();
        SimulateOneWavelength(config, batch.raypath_color_.get(), wl_param, batch.ray_num_, crystal_cache, workspace,
                              generation, ray_alloc_carry);
        return false;
      }
    };

    const auto& spectrum = config.light_source_.spectrum_;
    if (auto* illuminant = std::get_if<IlluminantType>(&spectrum)) {
      // Standard illuminant.
      // scrum-268.8 (DR-3 / Step 9): only backends with a per-ray wavelength
      // pool (Metal, WlPoolSize() > 0) sample wavelength per-ray on device. For
      // those, the per-batch host sampling is deleted — it would only consume
      // rng_ and set a misleading curr_wl_. Pass a zero-wl WlParam so curr_wl_
      // stays 0: IF the per-ray wavelength is ever dropped upstream, the
      // consumer renders black (loud) instead of silently collapsing onto a
      // flat spectrum (the bug fixed in a101c53e). Pool-less backends (CPU /
      // cpu_backend / legacy) keep per-batch uniform wl + SPD weight.
      bool backend_per_ray_wl = use_backend && backend->WlPoolSize() > 0u;
      if (backend_per_ray_wl) {
        // task-282 fallback samples a host wl (matches the no-backend branch)
        // if the backend drops mid-call; otherwise the per-ray pool drives wl.
        // The fallback already ran the CPU path here using a zero-wl WlParam
        // (consistent with pool-driven semantics: no per-batch host wl when the
        // backend owns the pool); subsequent batches take the use_backend == false
        // branch since `backend` is now reset. Nothing more to do on either result.
        (void)run_with_backend(WlParam{});
      } else {
        float wl = 380.0f + rng_.GetUniform() * 400.0f;  // [380, 780] nm
        float weight = GetIlluminantSpd(*illuminant, wl);
        WlParam wl_param{ wl, weight };
        if (use_backend) {
          run_with_backend(wl_param);
        } else {
          SimulateOneWavelength(config, batch.raypath_color_.get(), wl_param, batch.ray_num_, crystal_cache, workspace,
                                generation, ray_alloc_carry);
        }
      }
    } else {
      // Discrete wavelength list
      const auto& wl_params = std::get<std::vector<WlParam>>(spectrum);
      for (const auto& wl_param : wl_params) {
        if (stop_) {
          break;
        }
        if (use_backend && backend) {
          run_with_backend(wl_param);
        } else {
          SimulateOneWavelength(config, batch.raypath_color_.get(), wl_param, batch.ray_num_, crystal_cache, workspace,
                                generation, ray_alloc_carry);
        }
      }
    }

    idle_ = true;
  }
  // scrum-312 third-clock: final flush on loop exit (ray_num==0 sentinel / stop /
  // null scene) so the last accumulation window is not lost on shutdown.
  DrainDeviceXyz(backend.get());
}


void Simulator::SimulateOneWavelength(const SceneConfig& config, const RaypathColorConfig* raypath_color,
                                      const WlParam& wl_param, size_t ray_num, CrystalCache& crystal_cache,
                                      SimWorkspace& workspace, uint64_t generation,
                                      std::vector<std::vector<double>>& ray_alloc_carry) {
  ILOG_TRACE(logger_, "Run: get config: ray({}), wl({:.1f},{:.2f})",  //
             ray_num, wl_param.wl_, wl_param.weight_);

  float wl = wl_param.wl_;
  size_t original_ray_num = ray_num;  // ray_num is overwritten in the ms loop; keep original for normalization.

  // Design 2 (task-engine-redirect-design2 Step 4): the CPU emit gate now
  // scans `raypath_color[].match[]` predicates on demand (not physical-filter
  // summands eagerly). Missing raypath_color → empty ColorGateTable → the
  // color pass in `CollectData` becomes a no-op (AC3).
  RaypathColorConfig empty_color;
  const RaypathColorConfig& color_cfg = (raypath_color != nullptr) ? *raypath_color : empty_color;
  ColorGateTable color_gate_table = BuildColorGateTable(color_cfg, config);
  ILOG_DEBUG(logger_, "ColorGateTable built: {} entries (legacy path)", color_gate_table.entries_.size());

  RayBuffer all_data = AllocateAllData(config, ray_num);
  std::vector<float> outgoing_d;
  std::vector<float> outgoing_w;
  // task-331.1: per-outgoing-ray component mask parallel to outgoing_d_/w_.
  // Phase-1 emits 0s (no producer wired), but the plumbing must exist so T2/T3
  // don't need to touch this legacy path.
  std::vector<uint64_t> outgoing_component;
  auto& init_data = workspace.init_data;
  auto& buffer_data = workspace.buffer_data;
  init_data[0].size_ = 0;
  init_data[1].Reset(ray_num * config.max_hits_);

  std::vector<Crystal> all_crystals;
  all_crystals.reserve(16);
  std::vector<AxisDistribution> all_axis_dists;
  all_axis_dists.reserve(16);

  bool first_ms = true;
  for (size_t mi = 0; mi < config.ms_.size() && !stop_; mi++) {
    const auto& m = config.ms_[mi];
    auto ms_crystal_cnt = m.setting_.size();
    std::vector<float> proportions;
    proportions.reserve(ms_crystal_cnt);
    for (size_t ci = 0; ci < ms_crystal_cnt; ci++) {
      proportions.push_back(m.setting_[ci].crystal_proportion_);
    }

    // Lazy-initialize carry for this scattering layer.
    if (ray_alloc_carry.size() <= mi) {
      ray_alloc_carry.resize(config.ms_.size());
    }
    if (ray_alloc_carry[mi].size() != ms_crystal_cnt) {
      ray_alloc_carry[mi].assign(ms_crystal_cnt, 0.0);
    }

    auto crystal_ray_num = PartitionCrystalRayNum(proportions, ray_num, ray_alloc_carry[mi]);

    // NOTE: ray_num will change between scatterings.
    buffer_data[0].Reset(ray_num * 2);
    buffer_data[1].Reset(ray_num * 2);

    size_t init_ray_offset = 0;
    for (size_t ci = 0; ci < ms_crystal_cnt && !stop_; ci++) {
      const auto& s = m.setting_[ci];
      std::unique_ptr<FilterSpec> spec;
      std::vector<ColorSpecGroupOwned> color_groups_owned;
      std::vector<ColorSpecGroup> color_groups;

      // Design 2: color pass is keyed by CrystalConfig::id_ (user-visible id),
      // not ci (setting-slot index). The predicates for this (layer, crystal_id)
      // become a synthetic ComplexFilterParam (OR-of-singleton-AND-clauses) that
      // we feed to the same `FilterSpec::Create` machinery — its
      // `MatchSummandMask` then reports per-predicate matches without a second
      // predicate implementation. Depends only on config, so compute once per ci.
      ColorGatePlacement color_placement =
          ColorGatePlacementFor(color_gate_table, static_cast<IdType>(mi), s.crystal_.id_);

      bool deterministic = IsDeterministic(s.crystal_.param_);

      // Look up cross-call cache for deterministic crystal params.
      const CrystalParam* param_ptr = &s.crystal_.param_;
      const Crystal* cached_crystal = nullptr;
      if (deterministic) {
        for (const auto& [key, val] : crystal_cache) {
          if (key == param_ptr) {
            cached_crystal = &val;
            break;
          }
        }
      }

      // For deterministic params, create/copy crystal once per ci iteration.
      size_t ci_crystal_id = kInfSize;

      // geom_clock_ (LUMICE_GEOM_CLOCK, default kSmallBatchRayNum) sets how many
      // rays share one sampled shape. It doubles as the ray-batch size here,
      // exactly as the shipped constant did, so sweeping it moves both clocks at
      // once. Isolating the geometry effect from the batching effect therefore
      // needs a deterministic-params control run, where resampling is a no-op and
      // only the batching effect remains.
      for (size_t cn = 0; cn < crystal_ray_num[ci] && !stop_; cn += geom_clock_) {  // for a same crystal
        size_t curr_ray_num = std::min(geom_clock_, crystal_ray_num[ci] - cn);

        size_t curr_crystal_id = 0;
        if (deterministic && ci_crystal_id != kInfSize) {
          // Reuse crystal from earlier batch in this ci loop
          curr_crystal_id = ci_crystal_id;
        } else if (cached_crystal != nullptr) {
          // Copy from cross-call cache (deterministic, first batch of this ci)
          curr_crystal_id = all_crystals.size();
          all_crystals.emplace_back(*cached_crystal);
          all_axis_dists.emplace_back(s.crystal_.axis_);
          ci_crystal_id = curr_crystal_id;
        } else {
          // Create new crystal (random params, or first-ever deterministic creation)
          curr_crystal_id = all_crystals.size();
          all_crystals.emplace_back(std::visit(CrystalMaker{ rng_ }, s.crystal_.param_));
          all_axis_dists.emplace_back(s.crystal_.axis_);
          if (deterministic) {
            crystal_cache.emplace_back(param_ptr, all_crystals.back());
            cached_crystal = &crystal_cache.back().second;
            ci_crystal_id = curr_crystal_id;
          }
        }
        const auto& curr_crystal = all_crystals[curr_crystal_id];
        // Reuse spec across cn batches when crystal is invariant (deterministic).
        // Random crystals: rebuild per cn batch since each batch may produce a new crystal.
        if (!spec || !deterministic) {
          spec = FilterSpec::Create(s.filter_, curr_crystal, s.crystal_.axis_);
          // Design 2 + scrum-color-predicate-symmetry: build per-symmetry
          // color spec groups. Each group synthesises one ComplexFilterParam
          // (its predicates as OR-summands) wrapped in a FilterConfig carrying
          // the group's symmetry — so `FilterSpec::Create` runs
          // `ReduceRaypath`/`ExpandRaypath` on the color predicates the same
          // way it does on the physical filter (a12: no re-implementation).
          color_groups_owned = BuildColorSpecGroups(color_placement, curr_crystal, s.crystal_.axis_);
          color_groups.clear();
          color_groups.reserve(color_groups_owned.size());
          for (auto& g : color_groups_owned) {
            color_groups.push_back({ g.spec.get(), &g.bits });
          }
        }

        // 1. Initialize data
        if (first_ms) {
          InitRayFirstMs(rng_, config.light_source_.param_, wl_param, curr_ray_num,  // input
                         curr_crystal, curr_crystal_id, s.crystal_.axis_,            // input
                         buffer_data, all_data);                                     // output
        } else {
          InitRayOtherMs(rng_, init_data, curr_ray_num,                    // input
                         curr_crystal, curr_crystal_id, s.crystal_.axis_,  // input
                         buffer_data, all_data, init_ray_offset);          // output
        }

        // 2. Start tracing
        float refractive_index = curr_crystal.GetRefractiveIndex(wl);
        for (size_t i = 0; i < config.max_hits_ && !stop_; i++) {
          // 2.1 Trace rays. Deal with d, p, w, fid
          size_t curr_batch_size = buffer_data[0].size_;
          TraceRayBasicInfo(curr_crystal, refractive_index, curr_batch_size,  // input
                            buffer_data);                                     // output

          // 2.2 Fill other information: append face-number to raypath
          FillRayOtherInfo(curr_crystal, buffer_data);

          // 2.3 Collect data. And set ray properties: state.
          CollectData(rng_, m, spec.get(),                              // input
                      buffer_data, init_data,                           // output
                      color_groups.empty() ? nullptr : &color_groups);  // input (Design-2 color producer)

          // 2.4 Copy to all_data + collect outgoing rays (d/w pre-pack).
          all_data.EmplaceBack(buffer_data[1]);
          for (size_t j = 0; j < buffer_data[1].size_; j++) {
            const auto& r = buffer_data[1][j];
            if (r.IsOutgoing()) {
              outgoing_d.push_back(r.d_[0]);
              outgoing_d.push_back(r.d_[1]);
              outgoing_d.push_back(r.d_[2]);
              outgoing_w.push_back(r.w_);
              // task-331.2: CollectData has now OR'd this layer's component
              // bits into the mask (0 only when no summand matched / no map).
              outgoing_component.push_back(buffer_data[1].ComponentAt(j));
            }
          }
        }  // hit loop
      }  // small batch loop
    }  // crystal loop

    ray_num = init_data[1].size_;
    init_data[0].Reset(ray_num * (config.max_hits_ + 1));
    std::swap(init_data[0], init_data[1]);

    // Shuffle init_data. SwapRay (not std::swap(buf[i],buf[j])) so each ray's
    // OR-accumulated component mask stays paired with it across the layer
    // boundary — operator[] only exposes RaySeg, so a plain swap would leave the
    // parallel components_ array behind and decorrelate the cross-layer mask
    // (task-331.3). RNG draw order is unchanged (SwapRay draws nothing).
    for (size_t i = 0; i < init_data[0].size_; i++) {
      size_t j = static_cast<size_t>(rng_.GetUniform() * (init_data[0].size_ - i)) + i;
      init_data[0].SwapRay(i, j);
    }

    first_ms = false;
  }  // ms loop
  if (stop_) {
    return;
  }

  SimData sim_data;
  sim_data.curr_wl_ = wl;
  sim_data.generation_ = generation;
  assert(all_crystals.size() == all_axis_dists.size());
  sim_data.crystals_ = std::move(all_crystals);
  sim_data.crystal_axis_dists_ = std::move(all_axis_dists);
  sim_data.rays_ = std::move(all_data);
  sim_data.outgoing_d_ = std::move(outgoing_d);
  sim_data.outgoing_w_ = std::move(outgoing_w);
  sim_data.outgoing_component_ = std::move(outgoing_component);  // task-331.1
  sim_data.root_ray_count_ = original_ray_num;
  sim_data.crystal_count_ = sim_data.crystals_.size();
  data_queue_->Emplace(std::move(sim_data));
}

// Backend-routed wavelength step (TraceBackend seam integration, scrum-258.1).
// Mirrors the wavelength-loop semantics of SimulateOneWavelength but delegates
// trace -> recorder -> projection to the supplied backend (CPU or Metal) and
// then routes the backend's world-space EXIT rays back into the legacy
// consumer projection path via SimData.outgoing_d_/w_. This is the canonical
// exit-seam buffer-egress path: the backend reports {dir, weight} per exit
// ray; the consumer projects + XYZ-accumulates them exactly as it does for
// Metal-OFF, so backend / legacy paths share the same downstream pipeline.
//
// Behaviour change vs the original 252.3 image-seam path:
//   - No ReadbackImage / no Y-channel reverse-compute. The backend's
//     per-batch O(W*H) image readback is replaced by an O(exit rays)
//     buffer copy (see TraceBackend::ReadbackExitRays).
//   - SimData carries outgoing_d_/w_ as the single payload form; the consumer
//     derives the outgoing-ray count from outgoing_w_.size().
//   - root_ray_count_ = ray_num distinguishes a valid backend batch from
//     the queue shutdown sentinel (default-constructed SimData has
//     root_ray_count_ == 0); the sentinel discriminator lives in
//     server.cpp::ConsumeData (固化 with this change, scrum-258.1 Step 3).
//
// `stop_` is checked between TraceLayer/Recombine calls. On stop the session
// is closed via EndSession (RAII-equivalent) but no SimData is emplaced.
void Simulator::DrainDeviceXyz(TraceBackend* backend) {
  // Drain-OR-settle the pending third-clock window: with a live backend this
  // reads the device XYZ accumulator back into a SimData and enqueues it; with a
  // null backend (task-282 fallback killed it mid-window) it enqueues a degenerate
  // credit-only SimData instead — so the sim_scene_cnt_ books stay balanced even
  // though the device image is unrecoverable. Callers should NOT assume a real
  // image always lands.
  // Takes a pointer (not a reference like SimulateOneWavelengthWithBackend)
  // because the Run() flush sites pass `backend.get()`, and `backend` CAN be null
  // there: the task-282 fallback resets it to null mid-Run() on
  // BackendUnavailableError. See the backend==nullptr branch below for how the
  // pending window's counter debt is still settled in that degraded case.
  // (SimulateOneWavelengthWithBackend stays a reference: run_with_backend gates on
  // non-null before calling it.)
  // scrum-312 third-clock drain: read the persistent device XYZ accumulator into
  // a SimData carrying the WINDOW-aggregated root/crystal counts, enqueue it, and
  // reset the window. ReadbackXyzAccum zeroes the device buffer after the copy so
  // the next window starts clean. No-op when nothing has accumulated.
  if (!xyz_win_.pending) {
    return;
  }
  SimData sim_data;
  sim_data.curr_wl_ = xyz_win_.wl;
  sim_data.generation_ = xyz_win_.generation;
  sim_data.root_ray_count_ = xyz_win_.root_rays;
  sim_data.crystal_count_ = xyz_win_.crystals;
  // task-color-degrade-gui-surfacing: carry the window's GPU color-degrade tally
  // into the drained SimData (config constant, direct copy — not accumulated).
  sim_data.color_degrade_counts_ = xyz_win_.color_degrade_counts_;
  // This one SimData stands in for xyz_win_.calls per-wavelength calls; ConsumeData
  // decrements sim_scene_cnt_ by this so the counter invariant stays balanced.
  sim_data.sim_scene_credit_ = xyz_win_.calls;
  if (backend != nullptr) {
    // Normal drain: pull the accumulated image off the device.
    size_t pix = static_cast<size_t>(xyz_win_.w) * static_cast<size_t>(xyz_win_.h);
    sim_data.xyz_pixel_data_.resize(pix * 3u);
    XyzImageData xyz_out;
    xyz_out.data = sim_data.xyz_pixel_data_.data();
    xyz_out.width = xyz_win_.w;
    xyz_out.height = xyz_win_.h;
    backend->ReadbackXyzAccum(xyz_out, sim_data.xyz_landed_weight_);
    // task-358.1 Step 4: drain the device per-color-class Y-lane accumulator.
    // No-op (empty vector, class_count=0) when the backend / session carries no
    // raypath_color config — consumer's ConsumeDeviceFused path stays byte-
    // identical to pre-358.1 (AC4).
    backend->ReadbackClassLanes(sim_data.lane_pixel_data_, sim_data.lane_class_count_);
  } else {
    // Degraded path (code-review round-2 Major): backend went null mid-window via
    // the task-282 fallback. The device image is unrecoverable, but we MUST still
    // emplace so ConsumeData decrements sim_scene_cnt_ by this window's credit —
    // else the increments GenerateScene already counted for these batches are
    // never balanced and the server never idles (the "producer++ / consumer--
    // must stay bound" invariant; dropping the window entirely reintroduces the
    // historical IDLE-hang class). Empty xyz_pixel_data_ → the consumer's 0-exit
    // path decrements without accumulating. root_ray_count_ stays > 0 so this is
    // not mistaken for the shutdown sentinel (rays_ empty && root_ray_count_ == 0).
    ILOG_WARN(logger_,
              "DrainDeviceXyz: backend null mid-window; settling {} credit with empty image "
              "(device data lost to fallback)",
              xyz_win_.calls);
  }
  data_queue_->Emplace(std::move(sim_data));
  xyz_win_ = XyzDrainWindow{};
}

void Simulator::SimulateOneWavelengthWithBackend(TraceBackend& backend, const SceneConfig& scene,
                                                 const RenderConfig& render,
                                                 std::shared_ptr<const RaypathColorConfig> raypath_color,
                                                 const WlParam& wl_param, size_t ray_num, uint64_t generation) {
  ILOG_TRACE(logger_, "Run(backend): ray({}), wl({:.1f},{:.2f})", ray_num, wl_param.wl_, wl_param.weight_);

  if (ray_num == 0 || scene.ms_.empty()) {
    return;
  }

  // task-331.2: the component table is now built and consumed INSIDE the
  // backend that produces per-ray masks (CpuTraceBackend::BeginSession builds
  // it; its emit gate slices it per (layer, crystal)). Metal/CUDA leave
  // component_mask at 0 until T5/T6 wire device-side production, so this
  // host-side entry no longer builds a throwaway table.

  // Task 260.6: hand the backend `effective_seed_` (non-zero) so device-gen
  // activates even when the user-facing `seed_` is 0 (default random mode).
  // When `seed_ != 0` this equals `seed_` → determinism contract unchanged.
  SessionSpec spec{ &scene, &render, wl_param, effective_seed_, std::move(raypath_color) };
  backend.BeginSession(spec);
  // RAII guard: EndSession() is called on all exit paths, including exceptions
  // thrown by TraceLayer/Recombine (which would otherwise skip EndSession).
  struct EndOnExit {
    TraceBackend& b;
    ~EndOnExit() noexcept { b.EndSession(); }
    EndOnExit(const EndOnExit&) = delete;
    EndOnExit& operator=(const EndOnExit&) = delete;
  } session_guard{ backend };

  // Drive (TraceLayer -> Recombine)* n -> TraceLayer (tail) -> ReadbackImage.
  // The first call must be a host RootRaySource; per trace_backend.hpp lines
  // 118-124, backends self-generate initial rays from scene config, so the
  // d/p/w/tf pointers stay null and only `count` is consumed.
  HostRayBatch host{};
  host.count = ray_num;
  RootRaySource roots = RootRaySource::FromHost(host);

  // task-268.4 per-layer drain (produce -> drain -> recycle): DrainExits
  // after each TraceLayer copies that layer's exit records into a host-side
  // accumulator AND resets the device exit slot so the backend can recycle
  // a single-layer-sized exit buffer across all MS layers. Grow-on-overflow
  // inside the backend's TraceLayer covers extreme single-layer fan-out
  // (e.g. ms3_mixed_pyramid_heavy: ~169-211 exits/root vs. ComputeOutCap's
  // max_hits*2+4 estimate). Single-MS sessions still drain exactly once.
  std::vector<ExitRayRecord> exit_records;
  bool aborted = false;
  for (size_t mi = 0; mi < scene.ms_.size(); mi++) {
    if (stop_) {
      aborted = true;
      break;
    }
    LayerHandlePtr handle = backend.TraceLayer(roots);

    std::vector<ExitRayRecord> layer_exits;
    backend.DrainExits(layer_exits);
    if (!layer_exits.empty()) {
      if (exit_records.empty()) {
        exit_records = std::move(layer_exits);
      } else {
        exit_records.insert(exit_records.end(), std::make_move_iterator(layer_exits.begin()),
                            std::make_move_iterator(layer_exits.end()));
      }
    }

    bool last_layer = (mi + 1 == scene.ms_.size());
    if (last_layer) {
      // No Recombine on the final layer — handle goes out of scope.
      break;
    }
    if (stop_) {
      aborted = true;
      break;
    }
    // Continuation-pool decorrelation shuffle (default shuffle = true). All
    // backends now honor it: CpuTraceBackend via host Fisher-Yates, Metal/CUDA
    // via a device-side Feistel gather (task-gpu-backend-recombine-shuffle).
    // It removes the per-parent-CI grouping that biased multi-CI continuation
    // layers' ray→crystal pairing (explore-300).
    RecombineSpec rspec{};
    roots = backend.Recombine(std::move(handle), rspec);
  }

  if (aborted) {
    return;
  }

  int w = render.resolution_[0];
  int h = render.resolution_[1];
  if (w <= 0 || h <= 0) {
    return;
  }

  // S1 device-fused path: backend accumulated XYZ on-device; read it back and
  // enqueue a SimData carrying xyz_pixel_data_ (W*H*3). DrainExits already
  // returned empty vectors so exit_records is empty. The consumer's Neumaier
  // path in RenderConsumer::Consume picks up xyz_pixel_data_ instead of the
  // ScatterOutgoingToXyz path.
  if (backend.SupportsDeviceXyzAccum()) {
    if (backend.SupportsThirdClockDrain()) {
      // scrum-312 third-clock: the device XYZ accumulator persists across
      // batches (BeginSession no longer zeroes it). Accumulate this batch's
      // normalization/stats into the drain window and return WITHOUT reading
      // back — the drain (readback+reset+enqueue) fires on display cadence in
      // Run() (producer-pause / generation-change / run-exit) or when the window
      // hits the batch cap here. This sheds the per-batch synchronous D2H tax.
      xyz_win_.pending = true;
      xyz_win_.root_rays += ray_num;
      xyz_win_.crystals += backend.GetLastBatchCrystalCount();
      xyz_win_.generation = generation;
      xyz_win_.w = w;
      xyz_win_.h = h;
      xyz_win_.wl = wl_param.wl_;
      xyz_win_.calls += 1;
      // task-color-degrade-gui-surfacing: OVERWRITE (not +=) — the tally is a
      // config constant, identical on every batch of this committed config.
      xyz_win_.color_degrade_counts_ = backend.GetLastColorDegradeCounts();
      if (xyz_win_.calls >= xyz_drain_batches_) {
        DrainDeviceXyz(&backend);
      }
      return;
    }
    // Legacy per-batch drain (unified-memory backends, e.g. Metal): readback +
    // enqueue every batch. Cheap here because the readback is a shared-memory
    // memcpy, not a synchronous PCIe D2H. Metal moves to the third clock in 312.4.
    SimData sim_data;
    sim_data.curr_wl_ = wl_param.wl_;
    sim_data.generation_ = generation;
    sim_data.root_ray_count_ = ray_num;
    sim_data.crystal_count_ = backend.GetLastBatchCrystalCount();
    // task-color-degrade-gui-surfacing: carry the GPU color-degrade tally on the
    // legacy per-batch drain too. Dead for today's backends (Metal + CUDA both take
    // the third-clock window branch above), but keeps this path from silently
    // dropping the warning if a device-XYZ / non-third-clock backend is ever added.
    sim_data.color_degrade_counts_ = backend.GetLastColorDegradeCounts();
    size_t pix = static_cast<size_t>(w) * static_cast<size_t>(h);
    sim_data.xyz_pixel_data_.resize(pix * 3u);
    XyzImageData xyz_out;
    xyz_out.data = sim_data.xyz_pixel_data_.data();
    xyz_out.width = w;
    xyz_out.height = h;
    backend.ReadbackXyzAccum(xyz_out, sim_data.xyz_landed_weight_);
    // task-358.1 Step 4: drain the device per-color-class Y-lane accumulator.
    // No-op when the backend / session has no raypath_color config (AC4).
    backend.ReadbackClassLanes(sim_data.lane_pixel_data_, sim_data.lane_class_count_);
    data_queue_->Emplace(std::move(sim_data));
    return;
  }

  // Exit seam (scrum-258.1/258.2): hand the dir/weight projection inputs to
  // the legacy consumer via outgoing_d_/w_. The rich metadata
  // (path / crystal_id / ms_layer_idx) is forwarded into
  // sim_data.exit_records_ for 258.3 filter + symmetry fold to consume —
  // 258.2 only produces it. The consumer derives the outgoing-ray count from
  // outgoing_w_.size() (and a non-empty assertion in render.cpp).
  size_t exit_count = exit_records.size();
  // 0-exit batch: still Emplace a SimData so server.cpp::ConsumeData decrements
  // sim_scene_cnt_. root_ray_count_=ray_num>0 distinguishes from the shutdown
  // sentinel (rays_ empty && root_ray_count_==0). Consumer-side guard skips the
  // projection call when outgoing_d_ is empty — do not collapse these two paths.
  std::vector<float> exit_d(exit_count * 3);
  std::vector<float> exit_w(exit_count);
  // scrum-268.8 (DR-3): per-outgoing-ray wavelength is the Metal + illuminant
  // seam. For other backends or the discrete-wl path, leave outgoing_wl_ empty
  // and let the consumer's per-batch curr_wl_ branch take over — that path is
  // bit-identical to the pre-DR-3 behaviour.
  uint32_t pool_size = backend.WlPoolSize();
  bool illuminant_mode = std::holds_alternative<IlluminantType>(scene.light_source_.spectrum_);
  bool fill_per_ray_wl = (pool_size > 0u) && illuminant_mode;
  std::vector<float> exit_wl;
  if (fill_per_ray_wl) {
    exit_wl.resize(exit_count);
  }
  // task-331.1: per-outgoing-ray component mask parallel to exit_d_/exit_w_.
  // Filled unconditionally (the field on ExitRayRecord always exists, unlike
  // the pool-gated wl seam above). Metal/CUDA leave the field at 0 today
  // because their kernels don't produce per-ray masks yet (see plan §0.2 —
  // T5/T6 deliver via device-side per-component atomicAdd rather than mirroring
  // the ExitRayRecord carrier). CpuTraceBackend populates the field for real.
  std::vector<uint64_t> exit_component(exit_count);
  for (size_t i = 0; i < exit_count; i++) {
    exit_d[i * 3 + 0] = exit_records[i].dir[0];
    exit_d[i * 3 + 1] = exit_records[i].dir[1];
    exit_d[i * 3 + 2] = exit_records[i].dir[2];
    exit_w[i] = exit_records[i].weight;
    if (fill_per_ray_wl) {
      float idx = static_cast<float>(exit_records[i].wl_idx) + 0.5f;
      exit_wl[i] = 380.0f + idx * 400.0f / static_cast<float>(pool_size);
    }
    exit_component[i] = exit_records[i].component_mask;
  }

  SimData sim_data;
  sim_data.curr_wl_ = wl_param.wl_;
  sim_data.generation_ = generation;
  sim_data.root_ray_count_ = ray_num;  // distinguishes a valid backend batch
                                       // from the shutdown sentinel (see
                                       // server.cpp::ConsumeData).
  sim_data.crystal_count_ = backend.GetLastBatchCrystalCount();
  sim_data.outgoing_d_ = std::move(exit_d);
  sim_data.outgoing_w_ = std::move(exit_w);
  sim_data.outgoing_wl_ = std::move(exit_wl);
  sim_data.outgoing_component_ = std::move(exit_component);  // task-331.1
  sim_data.exit_records_ = std::move(exit_records);
  data_queue_->Emplace(std::move(sim_data));
}

void Simulator::Stop() {
  stop_ = true;
  config_queue_->Shutdown();
  data_queue_->Shutdown();
}

bool Simulator::IsIdle() const {
  return !stop_ && idle_;
}

void Simulator::SetLogLevel(LogLevel level) {
  logger_.SetLevel(level);
}

}  // namespace lumice
