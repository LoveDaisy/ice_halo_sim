#include "server/render.hpp"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <memory>
#include <vector>

#include "config/render_config.hpp"
#include "config/sim_data.hpp"
#include "core/color_util.hpp"
#include "core/def.hpp"
#include "core/lens_proj.hpp"
#include "core/math.hpp"
#include "core/projection.hpp"
#include "core/raypath.hpp"
#include "util/color_data.hpp"
#include "util/color_space.hpp"


namespace lumice {

// Color transforms (SpectrumToXyz, kNormScale) live in core/color_util.hpp.
// Lens projections (LensProjParam, *Project, GetProjFunc, ComputeEDRScale,
// ComputeSTRScale) live in core/lens_proj.hpp. This file uses them directly.


// =============== Renderer ===============
RenderConsumer::RenderConsumer(RenderConfig config)
    : config_(std::move(config)),
      short_pix_(static_cast<float>(std::min(config_.resolution_[0], config_.resolution_[1]))),
      internal_xyz_(std::make_unique<float[]>(config_.resolution_[0] * config_.resolution_[1] * 3)),
      snapshot_xyz_(std::make_unique<float[]>(config_.resolution_[0] * config_.resolution_[1] * 3)),
      snapshot_work_(std::make_unique<float[]>(config_.resolution_[0] * config_.resolution_[1] * 3)),
      snapshot_image_buffer_(std::make_unique<uint8_t[]>(config_.resolution_[0] * config_.resolution_[1] * 3)) {
  float ax_z[3]{ 0, 0, 1 };
  float ax_y[3]{ 0, 1, 0 };
  rot_.Chain({ ax_z, (-90.0f + config_.view_.ro_) * math::kDegreeToRad })
      .Chain({ ax_y, (90.0f - config_.view_.el_) * math::kDegreeToRad })
      .Chain({ ax_z, config_.view_.az_ * math::kDegreeToRad });
}


void RenderConsumer::Consume(const SimData& data) {
  // scrum-258.1: SimData carries a single payload form — outgoing_d_/w_ +
  // outgoing_indices_(.size() only) — regardless of whether the simulator
  // ran via the legacy CPU path or a TraceBackend (exit seam). Both
  // converge here and run through the projection pipeline below.
  auto t0 = std::chrono::steady_clock::now();
  // Resize pre-allocated buffers if needed (grow-only).
  // Use outgoing count for capacity — it's the upper bound for filtered rays.
  size_t outgoing_count = data.outgoing_indices_.size();
  size_t needed = std::max(data.rays_.size_, outgoing_count);
  if (needed > buf_capacity_) {
    buf_capacity_ = needed;
    d_buf_ = std::make_unique<float[]>(buf_capacity_ * 3);
    w_buf_ = std::make_unique<float[]>(buf_capacity_);
    xy_buf_ = std::make_unique<int[]>(buf_capacity_ * 2);
    overlap_w_buf_ = std::make_unique<float[]>(buf_capacity_);
  }

  // Design A: filter runs simulator-side (see doc/filter-architecture.md §2).
  // All rays in data.outgoing_* have already been filter-gated; the consumer
  // simply projects and accumulates. No per-Consume FilterSpec table needed.
  auto lens_proj = GetProjFunc(config_.lens_.type_);
  LensProjParam proj_param{ config_.lens_.fov_,
                            short_pix_,
                            rot_,
                            config_.visible_,
                            { config_.resolution_[0], config_.resolution_[1] },
                            { config_.lens_shift_[0], config_.lens_shift_[1] },
                            0.0f,
                            1.0f };

  // Compute overlap r_scale for dual fisheye lens types.
  // overlap_ comes from RenderConfig (GUI sets sin(5°), CLI defaults to 0).
  if (config_.overlap_ > 0) {
    switch (config_.lens_.type_) {
      case LensParam::kDualFisheyeEqualArea:
        proj_param.max_abs_dz_ = config_.overlap_;
        proj_param.r_scale_ = projection::ComputeEARScale(config_.overlap_);
        break;
      case LensParam::kDualFisheyeEquidistant:
        proj_param.max_abs_dz_ = config_.overlap_;
        proj_param.r_scale_ = ComputeEDRScale(config_.overlap_);
        break;
      case LensParam::kDualFisheyeStereographic:
        proj_param.max_abs_dz_ = config_.overlap_;
        proj_param.r_scale_ = ComputeSTRScale(config_.overlap_);
        break;
      default:
        // Non-dual-fisheye: overlap is ignored.
        // Dual Orthographic intentionally falls here — overlap support requires
        // a non-trivial ComputeORScale derivation and is deferred (task D3).
        if (config_.lens_.type_ == LensParam::kDualFisheyeOrthographic) {
          ILOG_VERBOSE(logger_, "Dual Fisheye Orthographic: overlap parameter is ignored in this release");
        }
        break;
    }
  }

  // Shared overlap helper: projects a sky direction to the opposite fisheye hemisphere.
  // Used by both the unfiltered overlap pass 2 (path B) and the filtered overlap pass 2.
  auto overlap_fwd = [&](float sky_x, float sky_y, float z_hemi, float r_s) {
    switch (config_.lens_.type_) {
      case LensParam::kDualFisheyeEqualArea:
        return projection::FisheyeEqualAreaForward(sky_x, sky_y, z_hemi, r_s);
      case LensParam::kDualFisheyeEquidistant:
        return projection::FisheyeEquidistantForward(sky_x, sky_y, z_hemi, r_s);
      case LensParam::kDualFisheyeStereographic:
        return projection::FisheyeStereographicForward(sky_x, sky_y, z_hemi, r_s);
      case LensParam::kDualFisheyeOrthographic:
        // TODO(lens-ortho-overlap): when overlap support is added, implement
        // ComputeORScale and route via the outer switch. Reaching this lambda
        // with r_s != 1.0 silently yields wrong geometry without the scale fn.
        // Today: unreachable because outer switch lands in `default: break;`.
        return projection::FisheyeOrthographicForward(sky_x, sky_y, z_hemi, r_s);
      default:
        return projection::ProjXY{ 0, 0, false };
    }
  };

  // Copy pre-filtered outgoing rays into contiguous buffers (Design A:
  // simulator-side filter has already dropped non-matching rays).
  assert(!data.outgoing_indices_.empty() || data.rays_.Empty());
  size_t filtered_ray_num = outgoing_count;
  if (!data.outgoing_d_.empty()) {
    std::memcpy(d_buf_.get(), data.outgoing_d_.data(), filtered_ray_num * 3 * sizeof(float));
    std::memcpy(w_buf_.get(), data.outgoing_w_.data(), filtered_ray_num * sizeof(float));
  }

  lens_proj(proj_param, d_buf_.get(), xy_buf_.get(), filtered_ray_num);
  auto t2 = std::chrono::steady_clock::now();

  // Save w_buf_ before compaction (compaction overwrites w_buf_ in-place).
  // overlap_w_buf_ is needed by filtered pass 2 to read original weights by ray index.
  // For path B, this overwrites the unfiltered outgoing weights saved above — that is safe
  // because the unfiltered overlap pass 2 has already completed.
  if (proj_param.max_abs_dz_ > 0) {
    std::memcpy(overlap_w_buf_.get(), w_buf_.get(), filtered_ray_num * sizeof(float));
  }

  size_t final_ray_num = 0;
  float landed_weight = 0;
  for (size_t i = 0; i < filtered_ray_num; i++) {
    if (xy_buf_[i * 2 + 0] < 0 || xy_buf_[i * 2 + 0] >= config_.resolution_[0] ||  //
        xy_buf_[i * 2 + 1] < 0 || xy_buf_[i * 2 + 1] >= config_.resolution_[1]) {
      continue;
    }
    xy_buf_[final_ray_num] = xy_buf_[i * 2 + 1] * config_.resolution_[0] + xy_buf_[i * 2 + 0];
    w_buf_[final_ray_num] = w_buf_[i];
    landed_weight += w_buf_[i];
    final_ray_num++;
  }
  SpectrumToXyz(data.curr_wl_, w_buf_.get(), xy_buf_.get(), internal_xyz_.get(), final_ray_num);
  total_intensity_ += landed_weight;

  // === Pass 2: Overlap dual-write (only for dual fisheye with max_abs_dz > 0) ===
  // Project overlap-zone rays to the opposite hemisphere, filling the ring r ∈ (r_scale, 1].
  // d_buf_ is immutable (projection receives const float*). w_buf_ was compacted in pass 1,
  // so we use overlap_w_buf_ (copied before compaction... see below).
  // xy_buf_ and w_buf_ are safe to reuse — pass 1 data already consumed by SpectrumToXyz.
  if (proj_param.max_abs_dz_ > 0) {
    // overlap_w_buf_ was copied from w_buf_ before pass 1 compaction (see copy above).
    // For each overlap ray, project to the opposite hemisphere with z_hemi < 0.
    size_t overlap_count = 0;
    int w = config_.resolution_[0];
    int h = config_.resolution_[1];
    for (size_t i = 0; i < filtered_ray_num; i++) {
      float sky_x = -d_buf_[i * 3 + 0];
      float sky_y = -d_buf_[i * 3 + 1];
      float sky_z = -d_buf_[i * 3 + 2];
      if (std::abs(sky_z) >= proj_param.max_abs_dz_) {
        continue;  // not in overlap zone
      }

      // Opposite hemisphere: z_hemi is negative (past equator)
      bool primary_upper = (sky_z >= 0);
      float z_hemi_opp = primary_upper ? -sky_z : sky_z;  // = -|sky_z| < 0
      auto proj = overlap_fwd(sky_x, sky_y, z_hemi_opp, proj_param.r_scale_);
      float fx = 0;
      float fy = 0;
      projection::DualFisheyeToPixel(proj.x, proj.y, !primary_upper, w, h, &fx, &fy);
      int px = static_cast<int>(std::floor(fx + 0.5f));
      int py = static_cast<int>(std::floor(fy + 0.5f));
      if (px < 0 || px >= w || py < 0 || py >= h) {
        continue;
      }
      xy_buf_[overlap_count] = py * w + px;
      w_buf_[overlap_count] = overlap_w_buf_[i];
      overlap_count++;
    }
    if (overlap_count > 0) {
      // Pass 2 does NOT update total_intensity_ — preserves normalization.
      SpectrumToXyz(data.curr_wl_, w_buf_.get(), xy_buf_.get(), internal_xyz_.get(), overlap_count);
    }
  }

  auto t3 = std::chrono::steady_clock::now();

  consume_count_++;
  consume_proj_us_ += std::chrono::duration<double, std::micro>(t2 - t0).count();
  consume_accum_us_ += std::chrono::duration<double, std::micro>(t3 - t2).count();
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
  int total_pix = config_.resolution_[0] * config_.resolution_[1];
  std::memcpy(snapshot_xyz_.get(), internal_xyz_.get(), total_pix * 3 * sizeof(float));
  snapshot_intensity_ = total_intensity_;
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
  float scale = config_.intensity_factor_ * kNormScale * total_pix / snapshot_intensity_;
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
