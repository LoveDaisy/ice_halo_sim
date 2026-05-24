#include "server/render.hpp"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <functional>
#include <map>
#include <memory>
#include <vector>

#include "config/render_config.hpp"
#include "config/sim_data.hpp"
#include "core/def.hpp"
#include "core/math.hpp"
#include "core/projection.hpp"
#include "core/raypath.hpp"
#include "util/color_data.hpp"
#include "util/color_space.hpp"


namespace lumice {

// Display brightness baseline: maps the physically-derived per-pixel radiance to a visually
// reasonable [0,1] range at EV=0. The average illuminated pixel appears at ~5% brightness,
// so bright halo features (~20× average) approach full white. This constant is independent
// of resolution and FOV — use EV (intensity_factor) for user-controlled brightness adjustment.
constexpr float kNormScale = 0.08f;

// =============== Color transforms ===============

void SpectrumToXyz(float wl, const float* v, const int* xy, float* xyz, size_t num = 1) {
  int wl_key = static_cast<int>(wl + 0.5f);
  if (wl_key < kMinWavelength || wl_key > kMaxWavelength) {
    return;
  }

  for (size_t i = 0; i < num; i++) {
    size_t idx = xy == nullptr ? i * 3 : xy[i] * 3;
    xyz[idx + 0] += kCmfX[wl_key - kMinWavelength] * v[i];
    xyz[idx + 1] += kCmfY[wl_key - kMinWavelength] * v[i];
    xyz[idx + 2] += kCmfZ[wl_key - kMinWavelength] * v[i];
  }
}


// =============== Lens projections ===============
// All projection functions assume `d` is a unit-length direction vector (|d| = 1).
// This is guaranteed by the ray tracing pipeline (normalized outgoing ray directions).
struct LensProjParam {
  float fov_;
  float short_pix_;
  Rotation rot_;
  RenderConfig::VisibleRange visible_range_;
  int resolution_[2];        // x, y
  int lens_shift_[2];        // dx, dy
  float max_abs_dz_ = 0.0f;  // overlap zone |sky.z| threshold (0 = no overlap)
  float r_scale_ = 1.0f;     // projection r_scale for overlap normalization
};


using ProjFunc = std::function<void(const LensProjParam&, const float*, int*, size_t)>;

void LinearProject(const LensProjParam& p, const float* d, int* xy, size_t num = 1) {
  float scale = p.short_pix_ / 2.0f / std::tan(p.fov_ / 2.0f * math::kDegreeToRad);
  for (size_t i = 0; i < num; i++, d += 3, xy += 2) {
    if ((p.visible_range_ == RenderConfig::kUpper && d[2] > 0) ||  //
        (p.visible_range_ == RenderConfig::kLower && d[2] < 0)) {
      xy[0] = -1;
      xy[1] = -1;
      continue;
    }

    float d_cam[3]{ -d[0], -d[1], -d[2] };
    p.rot_.ApplyInverse(d_cam);
    auto proj = projection::LinearForward(d_cam[0], d_cam[1], d_cam[2]);
    if (!proj.valid) {
      xy[0] = -1;
      xy[1] = -1;
      continue;
    }

    xy[0] = static_cast<int>(std::floor(proj.x * scale + p.resolution_[0] / 2.0f + 0.5f + p.lens_shift_[0]));
    xy[1] = static_cast<int>(std::floor(proj.y * scale + p.resolution_[1] / 2.0f + 0.5f + p.lens_shift_[1]));
  }
}

// Equal area fisheye: unified projection with r=1 at equator normalization.
// Scale formula absorbs sqrt(2) factor: Type B normalization r = sqrt(2) * sin(theta/2).
// NOTE: scale formula must match f→fov conversion in render_config.cpp (equal area model).
void FisheyeEqualAreaProject(const LensProjParam& p, const float* d, int* xy, size_t num = 1) {
  float scale = p.short_pix_ / 2.0f / std::sqrt(2.0f) / std::sin(p.fov_ / 4.0f * math::kDegreeToRad);

  for (size_t i = 0; i < num; i++, d += 3, xy += 2) {
    if ((p.visible_range_ == RenderConfig::kUpper && d[2] > 0) ||  //
        (p.visible_range_ == RenderConfig::kLower && d[2] < 0)) {
      xy[0] = -1;
      xy[1] = -1;
      continue;
    }

    float d_cam[3]{ -d[0], -d[1], -d[2] };
    p.rot_.ApplyInverse(d_cam);
    if (d_cam[2] <= 0) {
      xy[0] = -1;
      xy[1] = -1;
      continue;
    }
    auto proj = projection::FisheyeEqualAreaForward(d_cam[0], d_cam[1], d_cam[2]);

    xy[0] = static_cast<int>(std::floor(proj.x * scale + p.resolution_[0] / 2.0f + 0.5f + p.lens_shift_[0]));
    xy[1] = static_cast<int>(std::floor(proj.y * scale + p.resolution_[1] / 2.0f + 0.5f + p.lens_shift_[1]));
  }
}

// Equidistant fisheye: unified projection with r=1 at equator normalization.
// Scale formula absorbs pi/2 factor: Type B normalization r = theta / (pi/2).
// NOTE: scale formula must match f→fov conversion in render_config.cpp (equidistant model).
void FisheyeEquidistantProject(const LensProjParam& p, const float* d, int* xy, size_t num = 1) {
  float scale = p.short_pix_ * math::kPi_2 / (p.fov_ * math::kDegreeToRad);

  for (size_t i = 0; i < num; i++, d += 3, xy += 2) {
    if ((p.visible_range_ == RenderConfig::kUpper && d[2] > 0) ||  //
        (p.visible_range_ == RenderConfig::kLower && d[2] < 0)) {
      xy[0] = -1;
      xy[1] = -1;
      continue;
    }

    float d_cam[3]{ -d[0], -d[1], -d[2] };
    p.rot_.ApplyInverse(d_cam);
    if (d_cam[2] <= 0) {
      xy[0] = -1;
      xy[1] = -1;
      continue;
    }
    auto proj = projection::FisheyeEquidistantForward(d_cam[0], d_cam[1], d_cam[2]);

    xy[0] = static_cast<int>(std::floor(proj.x * scale + p.resolution_[0] / 2.0f + 0.5f + p.lens_shift_[0]));
    xy[1] = static_cast<int>(std::floor(proj.y * scale + p.resolution_[1] / 2.0f + 0.5f + p.lens_shift_[1]));
  }
}

// Stereographic fisheye: unified projection with r=1 at equator normalization.
// Scale formula unchanged: Type B normalization r = tan(theta/2) is already r=1 at equator.
// NOTE: scale formula must match f→fov conversion in render_config.cpp (stereographic model).
void FisheyeStereographicProject(const LensProjParam& p, const float* d, int* xy, size_t num = 1) {
  float scale = p.short_pix_ / 2.0f / std::tan(p.fov_ / 4.0f * math::kDegreeToRad);

  for (size_t i = 0; i < num; i++, d += 3, xy += 2) {
    if ((p.visible_range_ == RenderConfig::kUpper && d[2] > 0) ||  //
        (p.visible_range_ == RenderConfig::kLower && d[2] < 0)) {
      xy[0] = -1;
      xy[1] = -1;
      continue;
    }

    float d_cam[3]{ -d[0], -d[1], -d[2] };
    p.rot_.ApplyInverse(d_cam);
    if (d_cam[2] <= 0) {
      xy[0] = -1;
      xy[1] = -1;
      continue;
    }
    auto proj = projection::FisheyeStereographicForward(d_cam[0], d_cam[1], d_cam[2]);

    xy[0] = static_cast<int>(std::floor(proj.x * scale + p.resolution_[0] / 2.0f + 0.5f + p.lens_shift_[0]));
    xy[1] = static_cast<int>(std::floor(proj.y * scale + p.resolution_[1] / 2.0f + 0.5f + p.lens_shift_[1]));
  }
}

// Overlap r_scale computation for dual fisheye projections.
// r_scale shrinks the primary projection so r=1 at the overlap boundary instead of the equator.
// Equal-area variant lives in core/projection (used by GUI export path as well).
float ComputeEDRScale(float max_abs_dz) {
  return (max_abs_dz <= 0) ? 1.0f : math::kPi_2 / (math::kPi_2 + std::asin(max_abs_dz));
}
float ComputeSTRScale(float max_abs_dz) {
  return (max_abs_dz <= 0) ? 1.0f : 1.0f / std::tan((math::kPi_2 + std::asin(max_abs_dz)) / 2.0f);
}

// Orthographic fisheye: r = sin(theta). Scale derived from (short_pix/2) / sin(fov/2)
// so that a direction at theta = fov/2 maps to the short-edge boundary.
// NOTE: scale formula must match f→fov conversion in render_config.cpp (orthographic model).
void FisheyeOrthographicProject(const LensProjParam& p, const float* d, int* xy, size_t num = 1) {
  float scale = p.short_pix_ / 2.0f / std::sin(p.fov_ / 2.0f * math::kDegreeToRad);

  for (size_t i = 0; i < num; i++, d += 3, xy += 2) {
    if ((p.visible_range_ == RenderConfig::kUpper && d[2] > 0) ||  //
        (p.visible_range_ == RenderConfig::kLower && d[2] < 0)) {
      xy[0] = -1;
      xy[1] = -1;
      continue;
    }

    float d_cam[3]{ -d[0], -d[1], -d[2] };
    p.rot_.ApplyInverse(d_cam);
    if (d_cam[2] <= 0) {
      xy[0] = -1;
      xy[1] = -1;
      continue;
    }
    // Forward's dz<0 guard cannot trip here: d_cam[2]<=0 was filtered above.
    auto proj = projection::FisheyeOrthographicForward(d_cam[0], d_cam[1], d_cam[2]);

    xy[0] = static_cast<int>(std::floor(proj.x * scale + p.resolution_[0] / 2.0f + 0.5f + p.lens_shift_[0]));
    xy[1] = static_cast<int>(std::floor(proj.y * scale + p.resolution_[1] / 2.0f + 0.5f + p.lens_shift_[1]));
  }
}

// Dual equal area fisheye: full hemisphere per circle, fov ignored.
// No visible_range or behind-camera early exit — by design, all directions are projected.
// Out-of-bounds pixel coordinates are handled by the caller (SpectrumToXyz bounds check).
// Caller handles hemisphere selection: z flip + is_upper flag for DualFisheyeToPixel layout.
void DualFisheyeEqualAreaProject(const LensProjParam& p, const float* d, int* xy, size_t num = 1) {
  for (size_t i = 0; i < num; i++, d += 3, xy += 2) {
    float sky_x = -d[0], sky_y = -d[1], sky_z = -d[2];
    bool is_upper = (sky_z >= 0);
    float z_hemi = is_upper ? sky_z : -sky_z;
    auto proj = projection::FisheyeEqualAreaForward(sky_x, sky_y, z_hemi, p.r_scale_);
    float fx, fy;
    projection::DualFisheyeToPixel(proj.x, proj.y, is_upper, p.resolution_[0], p.resolution_[1], &fx, &fy);
    xy[0] = static_cast<int>(std::floor(fx + 0.5f));
    xy[1] = static_cast<int>(std::floor(fy + 0.5f));
  }
}

// Dual equidistant fisheye: full hemisphere per circle, fov ignored.
void DualFisheyeEquidistantProject(const LensProjParam& p, const float* d, int* xy, size_t num = 1) {
  for (size_t i = 0; i < num; i++, d += 3, xy += 2) {
    float sky_x = -d[0], sky_y = -d[1], sky_z = -d[2];
    bool is_upper = (sky_z >= 0);
    float z_hemi = is_upper ? sky_z : -sky_z;
    auto proj = projection::FisheyeEquidistantForward(sky_x, sky_y, z_hemi, p.r_scale_);
    float fx, fy;
    projection::DualFisheyeToPixel(proj.x, proj.y, is_upper, p.resolution_[0], p.resolution_[1], &fx, &fy);
    xy[0] = static_cast<int>(std::floor(fx + 0.5f));
    xy[1] = static_cast<int>(std::floor(fy + 0.5f));
  }
}

// Dual stereographic fisheye: full hemisphere per circle, fov ignored.
void DualFisheyeStereographicProject(const LensProjParam& p, const float* d, int* xy, size_t num = 1) {
  for (size_t i = 0; i < num; i++, d += 3, xy += 2) {
    float sky_x = -d[0], sky_y = -d[1], sky_z = -d[2];
    bool is_upper = (sky_z >= 0);
    float z_hemi = is_upper ? sky_z : -sky_z;
    auto proj = projection::FisheyeStereographicForward(sky_x, sky_y, z_hemi, p.r_scale_);
    float fx, fy;
    projection::DualFisheyeToPixel(proj.x, proj.y, is_upper, p.resolution_[0], p.resolution_[1], &fx, &fy);
    xy[0] = static_cast<int>(std::floor(fx + 0.5f));
    xy[1] = static_cast<int>(std::floor(fy + 0.5f));
  }
}

// Dual orthographic fisheye: full hemisphere per circle, fov ignored.
// NOT SUPPORTED: overlap; r_scale always 1.0. See task-lens-orthographic D3 —
// ComputeORScale and dual-pass integration are deferred to a separate backlog item.
// The caller-derived z_hemi is always >= 0, so FisheyeOrthographicForward's dz<0
// guard never triggers on this path; proj.valid is trivially true.
void DualFisheyeOrthographicProject(const LensProjParam& p, const float* d, int* xy, size_t num = 1) {
  for (size_t i = 0; i < num; i++, d += 3, xy += 2) {
    float sky_x = -d[0], sky_y = -d[1], sky_z = -d[2];
    bool is_upper = (sky_z >= 0);
    float z_hemi = is_upper ? sky_z : -sky_z;
    auto proj = projection::FisheyeOrthographicForward(sky_x, sky_y, z_hemi, p.r_scale_);
    float fx, fy;
    projection::DualFisheyeToPixel(proj.x, proj.y, is_upper, p.resolution_[0], p.resolution_[1], &fx, &fy);
    xy[0] = static_cast<int>(std::floor(fx + 0.5f));
    xy[1] = static_cast<int>(std::floor(fy + 0.5f));
  }
}

// Rectangular (equirectangular) projection: always full-sky, fov is ignored.
// No visible_range or behind-camera early exit — by design, all directions are projected.
// lens_shift_ is intentionally not applied — full-sky equirectangular has no meaningful shift.
void RectangularProject(const LensProjParam& p, const float* d, int* xy, size_t num = 1) {
  auto short_res = std::min(p.resolution_[0] / 2, p.resolution_[1]);
  float scale = short_res / math::kPi;

  float ax_z[3]{ 0, 0, 1 };
  p.rot_.Apply(ax_z);
  float az0 = std::atan2(ax_z[1], ax_z[0]);
  for (size_t i = 0; i < num; i++, d += 3, xy += 2) {
    auto proj = projection::RectangularForward(-d[0], -d[1], -d[2]);
    float lon = proj.x - az0;  // subtract camera azimuth offset
    while (lon < -math::kPi) {
      lon += 2 * math::kPi;
    }
    while (lon > math::kPi) {
      lon -= 2 * math::kPi;
    }

    int raw_x = static_cast<int>(std::floor(lon * scale + p.resolution_[0] / 2.0f + 0.5f));
    xy[0] = ((raw_x % p.resolution_[0]) + p.resolution_[0]) % p.resolution_[0];
    xy[1] = static_cast<int>(std::floor(-proj.y * scale + p.resolution_[1] / 2.0f + 0.5f));
  }
}

ProjFunc GetProjFunc(LensParam::LensType type) {
  static std::map<LensParam::LensType, ProjFunc> lens_proj_map{
    { LensParam::kLinear, LinearProject },
    { LensParam::kFisheyeEqualArea, FisheyeEqualAreaProject },
    { LensParam::kFisheyeEquidistant, FisheyeEquidistantProject },
    { LensParam::kFisheyeStereographic, FisheyeStereographicProject },
    { LensParam::kDualFisheyeEqualArea, DualFisheyeEqualAreaProject },
    { LensParam::kDualFisheyeEquidistant, DualFisheyeEquidistantProject },
    { LensParam::kDualFisheyeStereographic, DualFisheyeStereographicProject },
    { LensParam::kRectangular, RectangularProject },
    { LensParam::kFisheyeOrthographic, FisheyeOrthographicProject },
    { LensParam::kDualFisheyeOrthographic, DualFisheyeOrthographicProject },
  };

  return lens_proj_map.at(type);
}


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

  // Anchor buffers (~25 MB each at 2048×1024) are lazily allocated on the first Consume
  // call that produces filter-fail emission. ON mode never allocates; OFF mode without
  // any filter also never allocates because the simulator short-circuits to the Design A
  // collect path (SimData::anchor_d_ stays empty). This keeps OFF-without-filter at parity
  // with ON-mode memory use. See plan §Step 7 / doc/filter-architecture.md §7.
}


void RenderConsumer::Consume(const SimData& data) {
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

  // === F1 anchor lane ===
  // Anchor accumulates filter-pass + filter-fail emission for a filter-independent EV anchor.
  // Filter-pass contribution is identical to internal_xyz_ above, so we re-project nothing —
  // instead PrepareSnapshot will sum snapshot_xyz_ + anchor_snapshot_xyz_ pixel-by-pixel.
  // Here we only project the filter-fail outgoing lane (sim_data.anchor_d_/anchor_w_).
  // Overlap dual-write is intentionally NOT applied to the anchor lane: the anchor lane is
  // used solely for EV P99 statistics, and the overlap ring is a small geometric artifact
  // that contributes negligibly to the P99 percentile. Skipping it keeps Consume hot-path
  // cost minimal.
  // The lane is naturally inert when no filter spec is configured: simulator.cpp keeps
  // anchor_d_/anchor_w_ empty in that case, the guard below short-circuits without any
  // allocation, and the GUI's degenerate fallback (anchor_p99_y <= 0) is then taken.
  if (!data.anchor_d_.empty()) {
    if (!anchor_internal_xyz_) {
      // Lazy allocation: first time we see filter-fail emission in OFF mode.
      auto pix_count = static_cast<size_t>(config_.resolution_[0]) * config_.resolution_[1] * 3;
      anchor_internal_xyz_ = std::make_unique<float[]>(pix_count);
      anchor_snapshot_xyz_ = std::make_unique<float[]>(pix_count);
      std::memset(anchor_internal_xyz_.get(), 0, pix_count * sizeof(float));
      std::memset(anchor_snapshot_xyz_.get(), 0, pix_count * sizeof(float));
    }
    size_t anchor_count = data.anchor_w_.size();
    if (anchor_count > buf_capacity_) {
      buf_capacity_ = anchor_count;
      d_buf_ = std::make_unique<float[]>(buf_capacity_ * 3);
      w_buf_ = std::make_unique<float[]>(buf_capacity_);
      xy_buf_ = std::make_unique<int[]>(buf_capacity_ * 2);
      overlap_w_buf_ = std::make_unique<float[]>(buf_capacity_);
    }
    std::memcpy(d_buf_.get(), data.anchor_d_.data(), anchor_count * 3 * sizeof(float));
    std::memcpy(w_buf_.get(), data.anchor_w_.data(), anchor_count * sizeof(float));
    lens_proj(proj_param, d_buf_.get(), xy_buf_.get(), anchor_count);

    size_t anchor_final = 0;
    float anchor_landed = 0;
    for (size_t i = 0; i < anchor_count; i++) {
      if (xy_buf_[i * 2 + 0] < 0 || xy_buf_[i * 2 + 0] >= config_.resolution_[0] ||  //
          xy_buf_[i * 2 + 1] < 0 || xy_buf_[i * 2 + 1] >= config_.resolution_[1]) {
        continue;
      }
      xy_buf_[anchor_final] = xy_buf_[i * 2 + 1] * config_.resolution_[0] + xy_buf_[i * 2 + 0];
      w_buf_[anchor_final] = w_buf_[i];
      anchor_landed += w_buf_[i];
      anchor_final++;
    }
    SpectrumToXyz(data.curr_wl_, w_buf_.get(), xy_buf_.get(), anchor_internal_xyz_.get(), anchor_final);
    anchor_total_intensity_ += anchor_landed;
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

void RenderConsumer::PrepareSnapshot() {
  int total_pix = config_.resolution_[0] * config_.resolution_[1];
  std::memcpy(snapshot_xyz_.get(), internal_xyz_.get(), total_pix * 3 * sizeof(float));
  snapshot_intensity_ = total_intensity_;

  // OFF-mode anchor P99 / intensity. Combined emission = filter-pass (snapshot_xyz_) +
  // filter-fail (anchor_internal_xyz_), giving a filter-independent EV anchor. Computed
  // server-side to avoid shipping a second full-XYZ buffer across the C API.
  if (anchor_internal_xyz_) {
    std::memcpy(anchor_snapshot_xyz_.get(), anchor_internal_xyz_.get(), total_pix * 3 * sizeof(float));
    anchor_snapshot_intensity_ = snapshot_intensity_ + anchor_total_intensity_;

    // P99 of combined Y. Allocate per call (called rarely; snapshot intervals dominate
    // rendering cost). Skip zero pixels to match GUI ComputeP99Y semantics.
    std::vector<float> y_vals;
    y_vals.reserve(static_cast<size_t>(total_pix));
    for (int i = 0; i < total_pix; i++) {
      float y = snapshot_xyz_[i * 3 + 1] + anchor_snapshot_xyz_[i * 3 + 1];
      if (y > 0.0f) {
        y_vals.push_back(y);
      }
    }
    if (y_vals.empty()) {
      anchor_p99_y_ = 0.0f;
    } else {
      auto idx = static_cast<size_t>(static_cast<float>(y_vals.size()) * 0.99f);
      if (idx >= y_vals.size()) {
        idx = y_vals.size() - 1;
      }
      std::nth_element(y_vals.begin(), y_vals.begin() + static_cast<ptrdiff_t>(idx), y_vals.end());
      anchor_p99_y_ = y_vals[idx];
    }
  }
}

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
  int pix = config_.norm_mode_ == 1 ? effective_pix_ : total_pix;
  float scale = config_.intensity_factor_ * kNormScale * pix / snapshot_intensity_;
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

RawXyzResult RenderConsumer::GetRawXyzResult() const {
  int total_pix = config_.resolution_[0] * config_.resolution_[1];
  float per_pixel_intensity = total_pix > 0 ? snapshot_intensity_ / (kNormScale * total_pix) : 0.0f;
  // OFF mode: scalar per-pixel anchor intensity; ON mode (anchor_internal_xyz_ == nullptr) returns 0.
  float anchor_per_pixel = total_pix > 0 ? anchor_snapshot_intensity_ / (kNormScale * total_pix) : 0.0f;
  return { config_.id_,
           config_.resolution_[0],
           config_.resolution_[1],
           snapshot_xyz_.get(),
           per_pixel_intensity,
           config_.intensity_factor_,
           {},
           {},
           effective_pix_,
           anchor_p99_y_,
           anchor_per_pixel };
}

void RenderConsumer::Reset() {
  total_intensity_ = 0;
  snapshot_intensity_ = 0;
  effective_pix_ = 0;
  auto buf_size = static_cast<size_t>(config_.resolution_[0]) * config_.resolution_[1] * 3;
  std::memset(internal_xyz_.get(), 0, buf_size * sizeof(float));
  // snapshot_xyz_ not zeroed: PrepareSnapshot will memcpy over it.
  // has_ever_consumed_ = false (set in Stop) ensures GetRawXyzResults returns has_valid_data_=false
  // until new data arrives, preventing stale snapshot reads.

  // anchor_internal_xyz_ may be null in OFF-without-filter (no filter-fail emission ever produced);
  // we deliberately do not allocate-on-Reset to preserve the OFF-without-filter zero-cost path.
  // anchor_snapshot_xyz_ is not zeroed: PrepareSnapshot will overwrite it before any read.
  if (anchor_internal_xyz_) {
    std::memset(anchor_internal_xyz_.get(), 0, buf_size * sizeof(float));
  }
  anchor_total_intensity_ = 0;
  anchor_snapshot_intensity_ = 0;
  anchor_p99_y_ = 0;
}

void RenderConsumer::ResetWith(const RenderConfig& new_config) {
  // NeedsRebuild guarantees layout fields (resolution, lens, view, visible, filter) are identical,
  // so assigning the full config is safe — layout-derived state (rot_, buffers) stays valid.
  config_ = new_config;
  Reset();
}

}  // namespace lumice
