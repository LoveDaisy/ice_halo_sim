#include "server/render.hpp"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <memory>
#include <vector>

#include "config/component_table.hpp"
#include "config/render_config.hpp"
#include "config/sim_data.hpp"
#include "core/color_util.hpp"
#include "core/def.hpp"
#include "core/lens_proj_build.hpp"
#include "core/math.hpp"
#include "core/raypath.hpp"
#include "core/shared/accum_shared.h"
#include "core/shared/projection_shared.h"
#include "util/color_data.hpp"
#include "util/color_space.hpp"


namespace lumice {

// Color transforms (SpectrumToXyz, kNormScale) live in core/color_util.hpp.
// Projection is single-sourced by lm_proj::ProjectExitToPixel (see
// core/shared/projection_shared.h); the host builds its ProjParams via
// BuildProjParams (see core/lens_proj_build.hpp). The old per-type
// GetProjFunc + inline overlap_fwd lambda + Pass 2 hand-rewrite are gone —
// one per-ray call now returns 0/1/2 pixel hits with bump_landed to split
// main from overlap.


// =============== Renderer ===============
RenderConsumer::RenderConsumer(RenderConfig config, ColorClassTable class_table)
    : config_(std::move(config)),
      short_pix_(static_cast<float>(std::min(config_.resolution_[0], config_.resolution_[1]))),
      internal_xyz_(std::make_unique<float[]>(config_.resolution_[0] * config_.resolution_[1] * 3)),
      comp_xyz_(std::make_unique<float[]>(config_.resolution_[0] * config_.resolution_[1] * 3)),
      snapshot_xyz_(std::make_unique<float[]>(config_.resolution_[0] * config_.resolution_[1] * 3)),
      snapshot_work_(std::make_unique<float[]>(config_.resolution_[0] * config_.resolution_[1] * 3)),
      snapshot_image_buffer_(std::make_unique<uint8_t[]>(config_.resolution_[0] * config_.resolution_[1] * 3)),
      class_table_(std::move(class_table)),
      lane_pixel_count_(static_cast<size_t>(config_.resolution_[0]) * static_cast<size_t>(config_.resolution_[1])) {
  float ax_z[3]{ 0, 0, 1 };
  float ax_y[3]{ 0, 1, 0 };
  rot_.Chain({ ax_z, (-90.0f + config_.view_.ro_) * math::kDegreeToRad })
      .Chain({ ax_y, (90.0f - config_.view_.el_) * math::kDegreeToRad })
      .Chain({ ax_z, config_.view_.az_ * math::kDegreeToRad });

  // task-339.3: allocate one W*H Y-lane per color class (compact by z-order).
  // Empty class table → no lane state, pre-336 zero heap allocations.
  if (HasColorClasses() && lane_pixel_count_ > 0) {
    size_t class_count = class_table_.classes_.size();
    lane_y_.resize(class_count);
    snapshot_lane_y_.resize(class_count);
    for (size_t i = 0; i < class_count; ++i) {
      lane_y_[i] = std::make_unique<float[]>(lane_pixel_count_);
      snapshot_lane_y_[i] = std::make_unique<float[]>(lane_pixel_count_);
    }
  }
}

const float* RenderConsumer::GetColorClassLaneY(size_t class_idx) const {
  if (class_idx >= snapshot_lane_y_.size()) {
    return nullptr;
  }
  return snapshot_lane_y_[class_idx].get();
}

// task-336.3: single source of truth for the mono-image exposure scale (see
// plan §1.1). PostSnapshot() below calls this so the inline scale and the
// compositor's scale can never drift apart.
float RenderConsumer::ExposureScale() const {
  int total_pix = config_.resolution_[0] * config_.resolution_[1];
  if (total_pix <= 0 || snapshot_intensity_ <= 0.0f) {
    return 0.0f;
  }
  return config_.intensity_factor_ * kNormScale * total_pix / snapshot_intensity_;
}


void RenderConsumer::ConsumeDeviceFused(const SimData& data) {
  // S1 device-fused: backend already accumulated XYZ on-device; skip
  // projection and fold the pixel buffer into internal_xyz_ via Neumaier.
  auto t0 = std::chrono::steady_clock::now();
  size_t total = static_cast<size_t>(config_.resolution_[0]) * static_cast<size_t>(config_.resolution_[1]) * 3u;
  assert(data.xyz_pixel_data_.size() == total);
  for (size_t i = 0u; i < total; ++i) {
    NeumaierAdd(internal_xyz_[i], comp_xyz_[i], data.xyz_pixel_data_[i]);
  }
  total_intensity_ += data.xyz_landed_weight_;
  // task-336.2 out-of-scope: GPU device-fused path does not populate
  // outgoing_component_. Warn once so a user mixing GPU + raypath_color
  // sees a signal in the log instead of silently empty lanes. Handoff for
  // device-side component lanes is tracked as scrum-3c (see plan §7 risk 3).
  if (HasColorClasses() && !logged_missing_component_) {
    ILOG_WARN(logger_,
              "RenderConsumer: raypath_color configured but the device-fused (GPU) path "
              "does not populate outgoing_component_ — component lanes will not accumulate. "
              "This scrum's lane display targets CPU only.");
    logged_missing_component_ = true;
  }
  // Count toward the consume profile (proj=0: device did the projection).
  // The batch-invariance positive control reads "Consume profile: N batches"
  // to confirm the commit grain took effect — the device-fused branch must
  // bump the counter or that witness reads 0 / the line never logs.
  consume_count_++;
  consume_accum_us_ += std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
}


// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void RenderConsumer::Consume(const SimData& data) {
  if (!data.xyz_pixel_data_.empty()) {
    ConsumeDeviceFused(data);
    return;
  }

  // scrum-258.1: SimData carries a single payload form — outgoing_d_/w_
  // (plus the optional per-ray outgoing_wl_ added in scrum-268.8 (DR-3)) —
  // regardless of whether the simulator ran via the legacy CPU path or a
  // TraceBackend (exit seam). Both converge here and run through the
  // projection pipeline below.
  auto t0 = std::chrono::steady_clock::now();
  // Resize pre-allocated buffers if needed (grow-only).
  // Use outgoing count for capacity — it's the upper bound for filtered rays.
  size_t outgoing_count = data.outgoing_w_.size();
  size_t needed = std::max(data.rays_.size_, outgoing_count);
  if (needed > buf_capacity_) {
    buf_capacity_ = needed;
    d_buf_ = std::make_unique<float[]>(buf_capacity_ * 3);
    w_buf_ = std::make_unique<float[]>(buf_capacity_);
    xy_buf_ = std::make_unique<int[]>(buf_capacity_ * 2);
    overlap_w_buf_ = std::make_unique<float[]>(buf_capacity_);
    // scrum-268.8 (DR-3): per-ray wavelength side-car, lock-step grow.
    wl_buf_ = std::make_unique<float[]>(buf_capacity_);
    overlap_wl_buf_ = std::make_unique<float[]>(buf_capacity_);
    // task-339.3: per-ray component mask side-cars. Gated on HasColorClasses()
    // so the pre-336 zero-config path stays at zero extra allocation (review
    // Minor #1). Once allocated, they follow the same grow-only capacity as
    // w_buf_ / overlap_w_buf_ / wl_buf_.
    if (HasColorClasses()) {
      comp_buf_ = std::make_unique<uint64_t[]>(buf_capacity_);
      overlap_comp_buf_ = std::make_unique<uint64_t[]>(buf_capacity_);
    }
  }

  // Design A: filter runs simulator-side (see doc/filter-architecture.md §2).
  // All rays in data.outgoing_* have already been filter-gated; the consumer
  // simply projects and accumulates. No per-Consume FilterSpec table needed.
  const auto proj_params = BuildProjParams(config_, rot_, short_pix_);
  // Verbose diagnostic for the dual-fisheye-orthographic no-overlap policy —
  // matches the legacy log message so downstream tooling (docs/tests grep-ing
  // the string) keeps functioning.
  if (config_.overlap_ > 0 && config_.lens_.type_ == LensParam::kDualFisheyeOrthographic) {
    ILOG_VERBOSE(logger_, "Dual Fisheye Orthographic: overlap parameter is ignored in this release");
  }

  // Copy pre-filtered outgoing rays into contiguous buffers (Design A:
  // simulator-side filter has already dropped non-matching rays).
  assert(!data.outgoing_w_.empty() || data.rays_.Empty());
  size_t filtered_ray_num = outgoing_count;
  if (!data.outgoing_d_.empty()) {
    std::memcpy(d_buf_.get(), data.outgoing_d_.data(), filtered_ray_num * 3 * sizeof(float));
    std::memcpy(w_buf_.get(), data.outgoing_w_.data(), filtered_ray_num * sizeof(float));
  }
  // scrum-268.8 (DR-3): per-ray wavelength is supplied by Metal (and the CPU
  // path when it migrates). Empty vector → legacy per-batch curr_wl_ branch.
  bool per_ray_wl = !data.outgoing_wl_.empty();
  // scrum-268.8 (DR-3) anti-silent-fallback gate: an exit-seam batch carrying
  // outgoing rays but neither per-ray wavelength nor a valid per-batch curr_wl_
  // means the per-ray wl was dropped upstream (e.g. a commit-chunk / transport
  // path that forgot to carry outgoing_wl_, as in a101c53e). The fallback below
  // would silently render a flat, illuminant-independent spectrum — exactly the
  // bug that hid for an entire scrum. Fail loud instead. CPU/legacy and
  // discrete-wl paths set a real curr_wl_ (≥380) and never trip this.
  if (!per_ray_wl && !data.outgoing_d_.empty() && data.curr_wl_ < 1.0f) {
    ILOG_ERROR(logger_,
               "RenderConsumer: {} outgoing rays with no per-ray wavelength and curr_wl_={:.1f} (<1) — "
               "per-ray wavelength was dropped upstream (scrum-268.8)",
               data.outgoing_d_.size() / 3, data.curr_wl_);
    assert(false && "scrum-268.8: per-ray wavelength dropped before consumer");
  }
  if (per_ray_wl) {
    assert(data.outgoing_wl_.size() == filtered_ray_num && "outgoing_wl_ size must match outgoing_w_ when present");
    std::memcpy(wl_buf_.get(), data.outgoing_wl_.data(), filtered_ray_num * sizeof(float));
  }
  // task-339.3: gather per-ray component masks alongside wavelengths.
  bool has_lanes = HasColorClasses();
  bool has_component = has_lanes && !data.outgoing_component_.empty();
  if (has_component) {
    assert(data.outgoing_component_.size() == filtered_ray_num &&
           "outgoing_component_ size must match outgoing_w_ when present");
    std::memcpy(comp_buf_.get(), data.outgoing_component_.data(), filtered_ray_num * sizeof(uint64_t));
  } else if (has_lanes && filtered_ray_num > 0 && !logged_missing_component_) {
    // Non-fatal (issue.md non-goal: GPU device-side lane accumulation is out of
    // scope for 336.2). Log once so silent zero-lane data during a colored
    // config still leaves an actionable trace.
    ILOG_WARN(logger_,
              "RenderConsumer: raypath_color configured but this batch carries no "
              "outgoing_component_ (empty vector) — component lanes will not accumulate for "
              "this batch. Most likely a GPU backend that has not been extended for lane "
              "delivery yet (see scrum-3c).");
    logged_missing_component_ = true;
  }

  // Single-pass per-ray projection: dispatch to ProjectExitToPixel, split
  // hits into "main" (bump_landed=true → drives landed_weight) vs "overlap"
  // (bump_landed=false → dual-write for dual-fisheye seam ring). Each batch
  // then flows through the existing SpectrumToXyz batch interface — Pass 2
  // stays as a second SpectrumToXyz call to preserve the legacy
  // normalization contract (overlap contributions do NOT enter total_intensity_).
  const int w_res = config_.resolution_[0];
  const int h_res = config_.resolution_[1];
  size_t main_n = 0;
  size_t overlap_n = 0;
  float landed_weight = 0.0f;
  for (size_t i = 0; i < filtered_ray_num; ++i) {
    auto hit = lm_proj::ProjectExitToPixel(proj_params, d_buf_[i * 3 + 0], d_buf_[i * 3 + 1], d_buf_[i * 3 + 2]);
    for (int k = 0; k < hit.count; ++k) {
      int px = hit.hits[k].px;
      int py = hit.hits[k].py;
      if (px < 0 || px >= w_res || py < 0 || py >= h_res) {
        continue;
      }
      if (hit.hits[k].bump_landed) {
        xy_buf_[main_n] = py * w_res + px;
        w_buf_[main_n] = w_buf_[i];
        if (per_ray_wl) {
          wl_buf_[main_n] = wl_buf_[i];
        }
        // task-336.2: parallel compaction of the component mask. main_n <= i
        // (same in-place-compaction invariant that lets w_buf_[main_n] =
        // w_buf_[i] be safe here).
        if (has_component) {
          comp_buf_[main_n] = comp_buf_[i];
        }
        landed_weight += w_buf_[i];
        ++main_n;
      } else {
        // Overlap ring uses dedicated side-arrays so main-batch data is not
        // clobbered before it hits SpectrumToXyz.
        overlap_w_buf_[overlap_n] = w_buf_[i];
        if (per_ray_wl) {
          overlap_wl_buf_[overlap_n] = wl_buf_[i];
        }
        if (has_component) {
          overlap_comp_buf_[overlap_n] = comp_buf_[i];
        }
        // Reuse the tail of xy_buf_ for overlap pixels (main uses the head).
        // Safe because filtered_ray_num is the shared upper bound and both
        // main_n and overlap_n are bounded by hit.count * filtered_ray_num.
        xy_buf_[filtered_ray_num + overlap_n] = py * w_res + px;
        ++overlap_n;
      }
    }
  }
  auto t2 = std::chrono::steady_clock::now();

  if (per_ray_wl) {
    SpectrumToXyzPerRay(wl_buf_.get(), w_buf_.get(), xy_buf_.get(), internal_xyz_.get(), main_n);
  } else {
    SpectrumToXyz(data.curr_wl_, w_buf_.get(), xy_buf_.get(), internal_xyz_.get(), main_n);
  }
  total_intensity_ += landed_weight;
  // task-339.3: fan the same batch into per-color-class Y-lanes. Shares the
  // wl / w / xy inputs of the SpectrumToXyz(PerRay) call above → same
  // exposure, same projection, same rounding (issue.md shared-exposure hard
  // invariant).
  if (has_component) {
    AccumulateColorClassLanes(per_ray_wl, wl_buf_.get(), data.curr_wl_, w_buf_.get(), comp_buf_.get(), xy_buf_.get(),
                              main_n);
  }

  if (overlap_n > 0) {
    // Pass 2 does NOT update total_intensity_ — preserves normalization.
    if (per_ray_wl) {
      SpectrumToXyzPerRay(overlap_wl_buf_.get(), overlap_w_buf_.get(), xy_buf_.get() + filtered_ray_num,
                          internal_xyz_.get(), overlap_n);
    } else {
      SpectrumToXyz(data.curr_wl_, overlap_w_buf_.get(), xy_buf_.get() + filtered_ray_num, internal_xyz_.get(),
                    overlap_n);
    }
    // task-339.3: overlap ring lane accumulation, symmetric with main pass.
    if (has_component) {
      AccumulateColorClassLanes(per_ray_wl, overlap_wl_buf_.get(), data.curr_wl_, overlap_w_buf_.get(),
                                overlap_comp_buf_.get(), xy_buf_.get() + filtered_ray_num, overlap_n);
    }
  }

  auto t3 = std::chrono::steady_clock::now();

  consume_count_++;
  consume_proj_us_ += std::chrono::duration<double, std::micro>(t2 - t0).count();
  consume_accum_us_ += std::chrono::duration<double, std::micro>(t3 - t2).count();
}

// task-339.3: per-ray → per-color-class Y scatter. For each ray we evaluate
// every class's predicate (any / all) against the ray's raw component mask and,
// on match, add the ray's Y into that class's lane. Overlap (a ray satisfies
// multiple classes) is intentional: each class independently accumulates,
// which is exactly what makes rule-lanes the source of AND (cross-layer via
// `all`) and overlap (compositor at 339.4).
//
// referenced_mask_ short-circuit: a ray whose masked bits intersect no class
// cannot satisfy any predicate. Safe for `any` (mask & bits == 0 → false) and
// safe for `all` because the empty-member class guard (bits == 0 → skip)
// prevents the vacuous-truth case (mask & 0 == 0 == 0 would otherwise fire).
void RenderConsumer::AccumulateColorClassLanes(bool per_ray_wl, const float* wl_buf, float curr_wl, const float* w_buf,
                                               const uint64_t* comp_buf, const int* xy_buf, size_t num) {
  if (num == 0 || !HasColorClasses()) {
    return;
  }
  uint64_t ref_mask = class_table_.referenced_mask_;
  // Bound by lane_y_.size() (the allocated-lanes source of truth) — NOT
  // class_table_.classes_.size() — to match PrepareSnapshot()/Reset() and stay
  // safe if the two ever decouple (e.g. lane_pixel_count_ == 0 leaves lane_y_
  // empty while HasColorClasses() is still true). lane_y_ is either empty or
  // sized to classes_.size(), so classes_[c] stays in range for every c here.
  size_t class_count = lane_y_.size();
  for (size_t i = 0; i < num; ++i) {
    uint64_t mask = comp_buf[i];
    if ((mask & ref_mask) == 0) {
      continue;
    }
    float wl_i = per_ray_wl ? wl_buf[i] : curr_wl;
    float y = SpectrumToYSingle(wl_i, w_buf[i]);
    if (y == 0.0f) {
      continue;
    }
    size_t pidx = static_cast<size_t>(xy_buf[i]);
    for (size_t c = 0; c < class_count; ++c) {
      const auto& cls = class_table_.classes_[c];
      uint64_t bits = cls.member_bits_;
      if (bits == 0) {
        // Defensive: an empty-member class never contributes energy. Guards
        // the `all` vacuous-truth trap (`mask & 0 == 0 == bits` would be
        // trivially satisfied by every ray). `any` is already safe here
        // (mask & 0 == 0), but treating both branches uniformly removes a
        // silent asymmetry a future reader could easily miss.
        continue;
      }
      bool satisfied = (cls.combine_ == ColorClassCombine::kAny) ? ((mask & bits) != 0) : ((mask & bits) == bits);
      if (satisfied) {
        lane_y_[c][pidx] += y;
      }
    }
  }
}

void RenderConsumer::LogConsumeProfile() const {
  if (consume_count_ == 0) {
    return;
  }
  double avg_proj = consume_proj_us_ / static_cast<double>(consume_count_);
  double avg_accum = consume_accum_us_ / static_cast<double>(consume_count_);
  double avg_total = avg_proj + avg_accum;
  ILOG_INFO(logger_, "Consume profile: {} batches, avg {:.1f}us (proj {:.1f}us {:.0f}% + accum {:.1f}us {:.0f}%)",
            consume_count_, avg_total, avg_proj, avg_proj / avg_total * 100, avg_accum, avg_accum / avg_total * 100);
}

// See doc/ev-pipeline-architecture.md §2.2
// See doc/accumulator-consumer-architecture.md §4.2 (two-phase snapshot protocol, Phase 1).
void RenderConsumer::PrepareSnapshot() {
  size_t total = static_cast<size_t>(config_.resolution_[0]) * config_.resolution_[1] * 3u;
  // True running sum = internal_xyz_ + comp_xyz_ (Neumaier residual). The
  // device-fused path (ConsumeDeviceFused) accumulates the compensation in
  // comp_xyz_; folding it here is what realizes the precision gain — without
  // this add the compensation would be tracked but never applied (plain +=).
  // comp_xyz_ stays all-zero on the legacy projection path, so this is a no-op
  // there.
  for (size_t i = 0u; i < total; ++i) {
    snapshot_xyz_[i] = internal_xyz_[i] + comp_xyz_[i];
  }
  snapshot_intensity_ = total_intensity_;
  // task-339.3: shadow per-class lanes into snapshot_lane_y_ under the same
  // two-phase snapshot protocol. lane_y_ has no Neumaier compensation
  // counterpart (single-precision scatter Y is enough for display), so a plain
  // memcpy matches internal_xyz_'s treatment when comp_xyz_ is all-zero on the
  // legacy path. PostSnapshot is not touched: lanes carry raw Y, no EV/sRGB
  // conversion (compositor multiplies by ExposureScale()).
  for (size_t c = 0; c < lane_y_.size(); ++c) {
    std::memcpy(snapshot_lane_y_[c].get(), lane_y_[c].get(), lane_pixel_count_ * sizeof(float));
  }
}

// See doc/accumulator-consumer-architecture.md §4.2 (Phase 1.5 — runs outside consumer_mutex_).
void RenderConsumer::CountEffectivePixels() {
  int total_pix = config_.resolution_[0] * config_.resolution_[1];
  int count = 0;
  const float* xyz = snapshot_xyz_.get();
  for (int i = 0; i < total_pix; i++) {
    if (xyz[i * 3] != 0 || xyz[i * 3 + 1] != 0 || xyz[i * 3 + 2] != 0) {
      count++;
    }
  }
  effective_pix_ = std::max(count, 1);
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void RenderConsumer::PostSnapshot() {
  int total_pix = config_.resolution_[0] * config_.resolution_[1];
  if (total_pix <= 0 || snapshot_intensity_ <= 0) {
    std::memset(snapshot_image_buffer_.get(), 0, total_pix * 3);
    return;
  }

  // Copy to work buffer — preserve snapshot_xyz_ for GetRawXyzResult().
  auto buf_size = static_cast<size_t>(total_pix) * 3;
  std::memcpy(snapshot_work_.get(), snapshot_xyz_.get(), buf_size * sizeof(float));
  float* float_data = snapshot_work_.get();

  // Intensity scaling uses config_.intensity_factor_ (from CLI JSON / CommitConfig snapshot).
  // GUI rendering uses a separate path: exposure_offset → shader uniform (see app_panels.cpp).
  // task-336.3: the scale expression now lives in ExposureScale() (single source
  // shared with the compositor). We are past the total_pix/snapshot_intensity_
  // guard above, so ExposureScale() returns the same non-zero value the inline
  // expression used to compute — bit-identical.
  float scale = ExposureScale();
  for (size_t i = 0; i < buf_size; i++) {
    float_data[i] *= scale;
  }

  bool use_real_color = config_.ray_color_[0] < 0;
  for (int i = 0; i < total_pix; i++) {
    float* xyz = float_data + i * 3;
    float rgb[3];

    if (use_real_color) {
      // Gamut clip → matrix multiply
      float clipped[3];
      GamutClipXyz(xyz, clipped);
      XyzToLinearRgb(clipped, rgb);
    } else {
      // Skip gamut clip; use D65 gray (luminance-only) → matrix multiply → ray_color tint.
      // Inline matrix multiply (no clamp before ray_color — clamp after bg blending below).
      float gray[3];
      for (int j = 0; j < 3; j++) {
        gray[j] = kWhitePointD65[j] * xyz[1];
      }
      for (int j = 0; j < 3; j++) {
        float v = 0;
        for (int k = 0; k < 3; k++) {
          v += gray[k] * kXyzToRgb[j * 3 + k];
        }
        rgb[j] = v * config_.ray_color_[j];
      }
    }

    // Background blending + clamp
    for (int j = 0; j < 3; j++) {
      rgb[j] += config_.background_[j];
      rgb[j] = std::clamp(rgb[j], 0.0f, 1.0f);
    }
    std::memcpy(float_data + i * 3, rgb, 3 * sizeof(float));
  }

  LinearToSrgbBatch(float_data, 3 * total_pix);

  for (int i = 0; i < total_pix * 3; i++) {
    snapshot_image_buffer_[i] = static_cast<uint8_t>(float_data[i] * 255);
  }
}

Result RenderConsumer::GetResult() const {
  return RenderResult{ config_.id_, config_.resolution_[0], config_.resolution_[1], snapshot_image_buffer_.get() };
}

// See doc/ev-pipeline-architecture.md §2.3
RawXyzResult RenderConsumer::GetRawXyzResult() const {
  int total_pix = config_.resolution_[0] * config_.resolution_[1];
  float per_pixel_intensity = total_pix > 0 ? snapshot_intensity_ / (kNormScale * total_pix) : 0.0f;
  return { config_.id_,
           config_.resolution_[0],
           config_.resolution_[1],
           snapshot_xyz_.get(),
           per_pixel_intensity,
           config_.intensity_factor_,
           {},
           {},
           effective_pix_ };
}

// See doc/ev-pipeline-architecture.md §3.2
// See doc/accumulator-consumer-architecture.md §3.1 (reset path).
void RenderConsumer::Reset() {
  total_intensity_ = 0;
  snapshot_intensity_ = 0;
  effective_pix_ = 0;
  auto buf_size = static_cast<size_t>(config_.resolution_[0]) * config_.resolution_[1] * 3;
  std::memset(internal_xyz_.get(), 0, buf_size * sizeof(float));
  std::memset(comp_xyz_.get(), 0, buf_size * sizeof(float));
  // task-339.3: zero per-class lanes; snapshot_lane_y_ is not zeroed here
  // (PrepareSnapshot will memcpy over it, mirroring snapshot_xyz_).
  for (auto& lane : lane_y_) {
    std::memset(lane.get(), 0, lane_pixel_count_ * sizeof(float));
  }
  // snapshot_xyz_ not zeroed: PrepareSnapshot will memcpy over it.
  // has_ever_consumed_ = false (set in Stop) ensures GetRawXyzResults returns has_valid_data_=false
  // until new data arrives, preventing stale snapshot reads.
}

// See doc/accumulator-consumer-architecture.md §5.3 (ResetWith path — layout fields guaranteed identical).
void RenderConsumer::ResetWith(const RenderConfig& new_config) {
  // NeedsRebuild guarantees layout fields (resolution, lens, view, visible, filter) are identical,
  // so assigning the full config is safe — layout-derived state (rot_, buffers) stays valid.
  config_ = new_config;
  Reset();
}

}  // namespace lumice
