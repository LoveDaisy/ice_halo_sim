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
#include "core/filter.hpp"
#include "core/math.hpp"
#include "core/raypath.hpp"
#include "util/color_data.hpp"


namespace lumice {

// Display brightness baseline: maps the physically-derived per-pixel radiance to a visually
// reasonable [0,1] range at EV=0. The average illuminated pixel appears at ~5% brightness,
// so bright halo features (~20× average) approach full white. This constant is independent
// of resolution and FOV — use EV (intensity_factor) for user-controlled brightness adjustment.
constexpr float kNormScale = 0.08f;

// =============== Color transforms ===============
// Convert linear rgb to sRGB
void SrgbGamma(float* rgb, size_t num) {
  for (size_t i = 0; i < num; i++) {
    if (rgb[i] < 0.0031308) {
      rgb[i] *= 12.92f;
    } else {
      rgb[i] = 1.055f * std::pow(rgb[i], 1.0f / 2.4f) - 0.055f;
    }
  }
}

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
  float diag_pix_;
  Rotation rot_;
  RenderConfig::VisibleRange visible_range_;
  int resolution_[2];  // x, y
  int lens_shift_[2];  // dx, dy
};


using ProjFunc = std::function<void(const LensProjParam&, const float*, int*, size_t)>;

void LinearProject(const LensProjParam& p, const float* d, int* xy, size_t num = 1) {
  float scale = p.diag_pix_ / 2.0f / std::tan(p.fov_ / 2.0f * math::kDegreeToRad);
  for (size_t i = 0; i < num; i++, d += 3, xy += 2) {
    if ((p.visible_range_ == RenderConfig::kUpper && d[2] > 0) ||  //
        (p.visible_range_ == RenderConfig::kLower && d[2] < 0)) {
      xy[0] = -1;
      xy[1] = -1;
      continue;
    }

    float d_cam[3]{ -d[0], -d[1], -d[2] };
    p.rot_.ApplyInverse(d_cam);
    if (d_cam[2] < 0) {
      xy[0] = -1;
      xy[1] = -1;
      continue;
    }

    d_cam[0] /= d_cam[2];
    d_cam[1] /= d_cam[2];
    xy[0] = static_cast<int>(std::floor(d_cam[0] * scale + p.resolution_[0] / 2.0f + 0.5f + p.lens_shift_[0]));
    xy[1] = static_cast<int>(std::floor(d_cam[1] * scale + p.resolution_[1] / 2.0f + 0.5f + p.lens_shift_[1]));
  }
}

// Equal area fisheye: r = 2f·sin(θ/2)
// NOTE: scale formula must match f→fov conversion in render_config.cpp (equal area model).
void FisheyeEqualAreaProject(const LensProjParam& p, const float* d, int* xy, size_t num = 1) {
  float scale = p.diag_pix_ / 2.0f / std::sin(p.fov_ / 4.0f * math::kDegreeToRad);

  for (size_t i = 0; i < num; i++, d += 3, xy += 2) {
    if ((p.visible_range_ == RenderConfig::kUpper && d[2] > 0) ||  //
        (p.visible_range_ == RenderConfig::kLower && d[2] < 0)) {
      xy[0] = -1;
      xy[1] = -1;
      continue;
    }

    float d_cam[3]{ -d[0], -d[1], -d[2] };
    p.rot_.ApplyInverse(d_cam);
    if (d_cam[2] < 0) {
      xy[0] = -1;
      xy[1] = -1;
      continue;
    }

    float az = std::atan2(d_cam[1], d_cam[0]);
    float theta = math::kPi_2 - std::asin(std::clamp(d_cam[2], -1.0f, 1.0f));
    float r = scale * std::sin(theta / 2.0f);
    xy[0] = static_cast<int>(std::floor(r * std::cos(az) + p.resolution_[0] / 2.0f + 0.5f + p.lens_shift_[0]));
    xy[1] = static_cast<int>(std::floor(r * std::sin(az) + p.resolution_[1] / 2.0f + 0.5f + p.lens_shift_[1]));
  }
}

// Equidistant fisheye: r = f·θ
// NOTE: scale formula must match f→fov conversion in render_config.cpp (equidistant model).
void FisheyeEquidistantProject(const LensProjParam& p, const float* d, int* xy, size_t num = 1) {
  float scale = p.diag_pix_ / (p.fov_ * math::kDegreeToRad);

  for (size_t i = 0; i < num; i++, d += 3, xy += 2) {
    if ((p.visible_range_ == RenderConfig::kUpper && d[2] > 0) ||  //
        (p.visible_range_ == RenderConfig::kLower && d[2] < 0)) {
      xy[0] = -1;
      xy[1] = -1;
      continue;
    }

    float d_cam[3]{ -d[0], -d[1], -d[2] };
    p.rot_.ApplyInverse(d_cam);
    if (d_cam[2] < 0) {
      xy[0] = -1;
      xy[1] = -1;
      continue;
    }

    float az = std::atan2(d_cam[1], d_cam[0]);
    float theta = math::kPi_2 - std::asin(std::clamp(d_cam[2], -1.0f, 1.0f));
    float r = scale * theta;
    xy[0] = static_cast<int>(std::floor(r * std::cos(az) + p.resolution_[0] / 2.0f + 0.5f + p.lens_shift_[0]));
    xy[1] = static_cast<int>(std::floor(r * std::sin(az) + p.resolution_[1] / 2.0f + 0.5f + p.lens_shift_[1]));
  }
}

// Stereographic fisheye: r = 2f·tan(θ/2)
// NOTE: scale formula must match f→fov conversion in render_config.cpp (stereographic model).
void FisheyeStereographicProject(const LensProjParam& p, const float* d, int* xy, size_t num = 1) {
  float scale = p.diag_pix_ / 2.0f / std::tan(p.fov_ / 4.0f * math::kDegreeToRad);

  for (size_t i = 0; i < num; i++, d += 3, xy += 2) {
    if ((p.visible_range_ == RenderConfig::kUpper && d[2] > 0) ||  //
        (p.visible_range_ == RenderConfig::kLower && d[2] < 0)) {
      xy[0] = -1;
      xy[1] = -1;
      continue;
    }

    float d_cam[3]{ -d[0], -d[1], -d[2] };
    p.rot_.ApplyInverse(d_cam);
    if (d_cam[2] < 0) {
      xy[0] = -1;
      xy[1] = -1;
      continue;
    }

    float az = std::atan2(d_cam[1], d_cam[0]);
    float theta = math::kPi_2 - std::asin(std::clamp(d_cam[2], -1.0f, 1.0f));
    float r = scale * std::tan(theta / 2.0f);
    xy[0] = static_cast<int>(std::floor(r * std::cos(az) + p.resolution_[0] / 2.0f + 0.5f + p.lens_shift_[0]));
    xy[1] = static_cast<int>(std::floor(r * std::sin(az) + p.resolution_[1] / 2.0f + 0.5f + p.lens_shift_[1]));
  }
}

// Dual equal area fisheye: full hemisphere per circle, fov ignored.
// No visible_range or behind-camera early exit — by design, all directions are projected.
// Out-of-bounds pixel coordinates are handled by the caller (SpectrumToXyz bounds check).
void DualFisheyeEqualAreaProject(const LensProjParam& p, const float* d, int* xy, size_t num = 1) {
  auto short_res = std::min(p.resolution_[0] / 2, p.resolution_[1]);
  float scale = short_res / 2.0f / std::sin(math::kPi_4);

  for (size_t i = 0; i < num; i++, d += 3, xy += 2) {
    float az = std::atan2(-d[1], -d[0]);
    float theta = math::kPi_2 - std::abs(std::asin(std::clamp(-d[2], -1.0f, 1.0f)));

    // fov is ignored here
    float r = scale * std::abs(std::sin(theta / 2.0f));
    if (d[2] > 0) {
      // Lower semisphere
      xy[0] = static_cast<int>(
          std::floor(r * std::cos(math::kPi_2 - az) + p.resolution_[0] / 2.0f + 0.5f + short_res / 2.0f));
      xy[1] = static_cast<int>(std::floor(r * std::sin(math::kPi_2 - az) + p.resolution_[1] / 2.0f + 0.5f));
    } else {
      // Upper semisphere
      xy[0] = static_cast<int>(
          std::floor(r * std::cos(math::kPi_2 + az) + p.resolution_[0] / 2.0f + 0.5f - short_res / 2.0f));
      xy[1] = static_cast<int>(std::floor(r * std::sin(math::kPi_2 + az) + p.resolution_[1] / 2.0f + 0.5f));
    }
  }
}

// Dual equidistant fisheye: full hemisphere per circle, fov ignored.
// No visible_range or behind-camera early exit — by design, all directions are projected.
void DualFisheyeEquidistantProject(const LensProjParam& p, const float* d, int* xy, size_t num = 1) {
  auto short_res = std::min(p.resolution_[0] / 2, p.resolution_[1]);
  float scale = short_res / 2.0f / math::kPi_4;  // at θ=π/2: r = scale·π/2 = short_res/2

  for (size_t i = 0; i < num; i++, d += 3, xy += 2) {
    float az = std::atan2(-d[1], -d[0]);
    float theta = math::kPi_2 - std::abs(std::asin(std::clamp(-d[2], -1.0f, 1.0f)));

    // fov is ignored here — dual fisheye is always full-hemisphere projection
    float r = scale * std::abs(theta);
    if (d[2] > 0) {
      // Lower semisphere
      xy[0] = static_cast<int>(
          std::floor(r * std::cos(math::kPi_2 - az) + p.resolution_[0] / 2.0f + 0.5f + short_res / 2.0f));
      xy[1] = static_cast<int>(std::floor(r * std::sin(math::kPi_2 - az) + p.resolution_[1] / 2.0f + 0.5f));
    } else {
      // Upper semisphere
      xy[0] = static_cast<int>(
          std::floor(r * std::cos(math::kPi_2 + az) + p.resolution_[0] / 2.0f + 0.5f - short_res / 2.0f));
      xy[1] = static_cast<int>(std::floor(r * std::sin(math::kPi_2 + az) + p.resolution_[1] / 2.0f + 0.5f));
    }
  }
}

// Dual stereographic fisheye: full hemisphere per circle, fov ignored.
// No visible_range or behind-camera early exit — by design, all directions are projected.
void DualFisheyeStereographicProject(const LensProjParam& p, const float* d, int* xy, size_t num = 1) {
  auto short_res = std::min(p.resolution_[0] / 2, p.resolution_[1]);
  float scale = short_res / 2.0f;  // tan(π/4) = 1, so scale = short_res/2

  for (size_t i = 0; i < num; i++, d += 3, xy += 2) {
    float az = std::atan2(-d[1], -d[0]);
    float theta = math::kPi_2 - std::abs(std::asin(std::clamp(-d[2], -1.0f, 1.0f)));

    // fov is ignored here — dual fisheye is always full-hemisphere projection
    float r = scale * std::abs(std::tan(theta / 2.0f));
    if (d[2] > 0) {
      // Lower semisphere
      xy[0] = static_cast<int>(
          std::floor(r * std::cos(math::kPi_2 - az) + p.resolution_[0] / 2.0f + 0.5f + short_res / 2.0f));
      xy[1] = static_cast<int>(std::floor(r * std::sin(math::kPi_2 - az) + p.resolution_[1] / 2.0f + 0.5f));
    } else {
      // Upper semisphere
      xy[0] = static_cast<int>(
          std::floor(r * std::cos(math::kPi_2 + az) + p.resolution_[0] / 2.0f + 0.5f - short_res / 2.0f));
      xy[1] = static_cast<int>(std::floor(r * std::sin(math::kPi_2 + az) + p.resolution_[1] / 2.0f + 0.5f));
    }
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
    float lon = std::atan2(-d[1], -d[0]) - az0;
    while (lon < -math::kPi) {
      lon += 2 * math::kPi;
    }
    while (lon > math::kPi) {
      lon -= 2 * math::kPi;
    }
    float lat = std::asin(std::clamp(-d[2], -1.0f, 1.0f));
    if (lat > math::kPi_2) {
      lat = math::kPi - lat;
    }
    if (lat < -math::kPi_2) {
      lat = -math::kPi - lat;
    }

    int raw_x = static_cast<int>(std::floor(lon * scale + p.resolution_[0] / 2.0f + 0.5f));
    xy[0] = ((raw_x % p.resolution_[0]) + p.resolution_[0]) % p.resolution_[0];
    xy[1] = static_cast<int>(std::floor(-lat * scale + p.resolution_[1] / 2.0f + 0.5f));
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
  };

  return lens_proj_map.at(type);
}


// =============== Renderer ===============
RenderConsumer::RenderConsumer(RenderConfig config)
    : config_(std::move(config)), diag_pix_(std::sqrt(config_.resolution_[0] * config_.resolution_[0] +
                                                      config_.resolution_[1] * config_.resolution_[1])),
      internal_xyz_(std::make_unique<float[]>(config_.resolution_[0] * config_.resolution_[1] * 3)),
      snapshot_xyz_(std::make_unique<float[]>(config_.resolution_[0] * config_.resolution_[1] * 3)),
      snapshot_image_buffer_(std::make_unique<uint8_t[]>(config_.resolution_[0] * config_.resolution_[1] * 3)) {
  float ax_z[3]{ 0, 0, 1 };
  float ax_y[3]{ 0, 1, 0 };
  rot_.Chain({ ax_z, (-90.0f + config_.view_.ro_) * math::kDegreeToRad })
      .Chain({ ax_y, (90.0f - config_.view_.el_) * math::kDegreeToRad })
      .Chain({ ax_z, config_.view_.az_ * math::kDegreeToRad });

  for (const auto& f : config_.ms_filter_) {
    filters_.emplace_back(Filter::Create(f));
  }
}


bool FilterRay(const RayBuffer& rays, size_t i, const std::vector<FilterPtrU>& filters,
               const std::vector<Crystal>& crystals) {
  if (filters.empty()) {
    return true;
  }

  const auto& r = rays[i];
  bool filter_checked = true;
  size_t curr_idx = i;
  size_t root_idx = r.root_ray_idx_;
  for (auto fit = filters.rbegin(); fit != filters.rend(); /* increament see below */) {
    if (curr_idx != kInvalidId) {
      root_idx = rays[curr_idx].root_ray_idx_;
    } else {
      filter_checked = false;
      break;
    }

    (*fit)->InitCrystalSymmetry(crystals.at(r.crystal_idx_));
    if (!(*fit)->Check(rays[curr_idx])) {
      filter_checked = false;
      break;
    }

    // increament
    curr_idx = rays[root_idx].prev_ray_idx_;
    fit++;
  }
  return filter_checked && curr_idx == kInfSize;
}


void RenderConsumer::Consume(const SimData& data) {
  auto t0 = std::chrono::steady_clock::now();
  const auto& crystals = data.crystals_;

  // Resize pre-allocated buffers if needed (grow-only).
  // Use outgoing count for capacity — it's the upper bound for filtered rays.
  size_t outgoing_count = data.outgoing_indices_.size();
  size_t needed = std::max(data.rays_.size_, outgoing_count);
  if (needed > buf_capacity_) {
    buf_capacity_ = needed;
    d_buf_ = std::make_unique<float[]>(buf_capacity_ * 3);
    w_buf_ = std::make_unique<float[]>(buf_capacity_);
    xy_buf_ = std::make_unique<int[]>(buf_capacity_ * 2);
  }

  // Filter + copy outgoing rays into contiguous buffers.
  assert(!data.outgoing_indices_.empty() || data.rays_.Empty());
  size_t filtered_ray_num = 0;

  // Beam tracing data has empty rays_ — the slow path (chain walk) cannot work.
  // Force fast path for BT data even if render-level filters are configured.
  bool force_fast = data.rays_.Empty() && !data.outgoing_d_.empty();
  if ((filters_.empty() || force_fast) && !data.outgoing_d_.empty()) {
    // Fast path: bulk memcpy from pre-packed contiguous arrays.
    filtered_ray_num = outgoing_count;
    std::memcpy(d_buf_.get(), data.outgoing_d_.data(), filtered_ray_num * 3 * sizeof(float));
    std::memcpy(w_buf_.get(), data.outgoing_w_.data(), filtered_ray_num * sizeof(float));
  } else {
    // Slow path: filter present — must call FilterRay per ray (chain walk through rays buffer).
    for (size_t i : data.outgoing_indices_) {
      const auto& r = data.rays_[i];
      if (!FilterRay(data.rays_, i, filters_, crystals)) {
        continue;
      }
      std::memcpy(d_buf_.get() + filtered_ray_num * 3, r.d_, 3 * sizeof(float));
      w_buf_[filtered_ray_num] = r.w_;
      filtered_ray_num++;
    }
  }
  auto t1 = std::chrono::steady_clock::now();

  auto lens_proj = GetProjFunc(config_.lens_.type_);
  LensProjParam proj_param{ config_.lens_.fov_,
                            diag_pix_,
                            rot_,
                            config_.visible_,
                            { config_.resolution_[0], config_.resolution_[1] },
                            { config_.lens_shift_[0], config_.lens_shift_[1] } };
  lens_proj(proj_param, d_buf_.get(), xy_buf_.get(), filtered_ray_num);
  auto t2 = std::chrono::steady_clock::now();

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
  auto t3 = std::chrono::steady_clock::now();

  consume_count_++;
  consume_filter_us_ += std::chrono::duration<double, std::micro>(t1 - t0).count();
  consume_proj_us_ += std::chrono::duration<double, std::micro>(t2 - t1).count();
  consume_accum_us_ += std::chrono::duration<double, std::micro>(t3 - t2).count();
}

void RenderConsumer::LogConsumeProfile() const {
  if (consume_count_ == 0) {
    return;
  }
  double avg_filter = consume_filter_us_ / static_cast<double>(consume_count_);
  double avg_proj = consume_proj_us_ / static_cast<double>(consume_count_);
  double avg_accum = consume_accum_us_ / static_cast<double>(consume_count_);
  double avg_total = avg_filter + avg_proj + avg_accum;
  ILOG_INFO(logger_,
            "Consume profile: {} batches, avg {:.1f}us (filter {:.1f}us {:.0f}% + proj {:.1f}us {:.0f}% + "
            "accum {:.1f}us {:.0f}%)",
            consume_count_, avg_total, avg_filter, avg_filter / avg_total * 100, avg_proj, avg_proj / avg_total * 100,
            avg_accum, avg_accum / avg_total * 100);
}

void RenderConsumer::PrepareSnapshot() {
  int total_pix = config_.resolution_[0] * config_.resolution_[1];
  std::memcpy(snapshot_xyz_.get(), internal_xyz_.get(), total_pix * 3 * sizeof(float));
  snapshot_intensity_ = total_intensity_;
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

  // Work on snapshot_xyz_ in-place (destructive to snapshot_xyz_, but that's fine —
  // PrepareSnapshot will overwrite it next time).
  float* float_data = snapshot_xyz_.get();
  int pix = config_.norm_mode_ == 1 ? effective_pix_ : total_pix;
  for (int i = 0; i < total_pix * 3; i++) {
    float_data[i] *= config_.intensity_factor_ * kNormScale * pix / snapshot_intensity_;
  }

  bool use_real_color = config_.ray_color_[0] < 0;
  float gray[3];
  for (int i = 0; i < total_pix; i++) {
    float* xyz = float_data + i * 3;
    for (int j = 0; j < 3; j++) {
      gray[j] = kWhitePointD65[j] * xyz[1];
    }

    if (use_real_color) {
      float r = 1.0f;
      for (int j = 0; j < 3; j++) {
        float a = 0;
        float b = 0;
        for (int k = 0; k < 3; k++) {
          a += -gray[k] * kXyzToRgb[j * 3 + k];
          b += (xyz[k] - gray[k]) * kXyzToRgb[j * 3 + k];
        }
        if (a * b > 0 && a / b < r) {
          r = a / b;
        }
      }

      for (int j = 0; j < 3; j++) {
        xyz[j] = (xyz[j] - gray[j]) * r + gray[j];
      }
    } else {
      std::memcpy(xyz, gray, 3 * sizeof(float));
    }

    float rgb[3]{};
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        rgb[j] += xyz[k] * kXyzToRgb[j * 3 + k];
      }
      if (!use_real_color) {
        rgb[j] *= config_.ray_color_[j];
      }
      rgb[j] += config_.background_[j];
      rgb[j] = std::clamp(rgb[j], 0.0f, 1.0f);
    }
    std::memcpy(float_data + i * 3, rgb, 3 * sizeof(float));
  }

  SrgbGamma(float_data, 3 * total_pix);

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

void RenderConsumer::ResetWith(const RenderConfig& new_config) {
  // NeedsRebuild guarantees layout fields (resolution, lens, view, visible, filter) are identical,
  // so assigning the full config is safe — layout-derived state (rot_, filters_, buffers) stays valid.
  config_ = new_config;
  Reset();
}

}  // namespace lumice
