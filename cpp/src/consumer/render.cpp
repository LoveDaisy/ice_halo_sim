#include "consumer/render.hpp"

#include <cmath>
#include <cstddef>
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
#include "process/color_data.hpp"
#include "util/log.hpp"

namespace icehalo {
namespace v3 {

// =============== Lens projections ===============
struct LensProjParam {
  float fov_;
  float diag_pix_;
  Rotation rot_;
  RenderConfig::VisibleRange visible_range_;
  int resolution_[2];  // x, y
  int lens_shift_[2];  // dx, dy
};


using ProjFunc = std::function<void(const LensProjParam&, const float*, int*)>;

void LinearProject(const LensProjParam& p, const float* d, int* xy) {
  if ((p.visible_range_ == RenderConfig::kUpper && d[2] > 0) ||  //
      (p.visible_range_ == RenderConfig::kLower && d[2] < 0)) {
    xy[0] = -1;
    xy[1] = -1;
    return;
  }

  float d_cam[3]{ d[1], -d[0], -d[2] };  // original (x, y, z) --> (-y, x, z) camera convention
  p.rot_.Apply(d_cam);
  if (d_cam[2] < 0) {
    xy[0] = -1;
    xy[1] = -1;
    return;
  }

  d_cam[0] /= d_cam[2];
  d_cam[1] /= d_cam[2];
  float scale = p.diag_pix_ / std::tan(p.fov_ / 2.0f * math::kDegreeToRad);
  xy[0] = static_cast<int>(d_cam[0] * scale + p.resolution_[0] / 2.0f + 0.5f + p.lens_shift_[0]);
  xy[1] = static_cast<int>(d_cam[1] * scale + p.resolution_[1] / 2.0f + 0.5f + p.lens_shift_[1]);
}

void DualFisheyeEqualAreaProject(const LensProjParam& p, const float* d, int* xy) {
  // visible_range is ignored here
  auto short_res = std::min(p.resolution_[0] / 2, p.resolution_[1]);

  float az = std::atan2(-d[1], -d[0]);
  float theta = math::kPi_2 - std::abs(std::asin(-d[2]));

  // fov is ignored here
  float scale = short_res / 2.0f / std::sin(math::kPi_4);
  float r = scale * std::abs(std::sin(theta / 2));
  if (d[2] > 0) {
    // Lower semisphere
    xy[0] = static_cast<int>(r * std::cos(math::kPi_2 - az) + p.resolution_[0] / 2.0f + 0.5f + short_res / 2.0f);
    xy[1] = static_cast<int>(r * std::sin(math::kPi_2 - az) + p.resolution_[1] / 2.0f + 0.5f);
  } else {
    // Upper semisphere
    xy[0] = static_cast<int>(r * std::cos(math::kPi_2 + az) + p.resolution_[0] / 2.0f + 0.5f - short_res / 2.0f);
    xy[1] = static_cast<int>(r * std::sin(math::kPi_2 + az) + p.resolution_[1] / 2.0f + 0.5f);
  }
}

ProjFunc GetProjFunc(LensParam::LensType type) {
  static std::map<LensParam::LensType, ProjFunc> lens_proj_map{
    { LensParam::kLinear, LinearProject },
    { LensParam::kDualFisheyeEqualArea, DualFisheyeEqualAreaProject },
  };

  return lens_proj_map.at(type);
}


void SrgbGamma(float* linear_rgb, size_t num) {
  for (size_t i = 0; i < num; i++) {
    if (linear_rgb[i] < 0.0031308) {
      linear_rgb[i] *= 12.92f;
    } else {
      linear_rgb[i] = static_cast<float>(1.055 * std::pow(linear_rgb[i], 1.0 / 2.4) - 0.055);
    }
  }
}

// =============== Renderer ===============
Renderer::Renderer(RenderConfig config)
    : config_(config), diag_pix_(std::sqrt(config.resolution_[0] * config.resolution_[0] +
                                           config.resolution_[1] * config.resolution_[1])),
      image_buffer_(new uint8_t[config.resolution_[0] * config.resolution_[1] * 3]{}) {
  float ax_z[3]{ 0, 0, 1 };
  float ax_y[3]{ 0, 1, 0 };
  rot_.Chain({ ax_z, -config.view_.az_ * math::kDegreeToRad })
      .Chain({ ax_y, -(90.0f - config.view_.el_) * math::kDegreeToRad })
      .Chain({ ax_z, -config.view_.ro_ * math::kDegreeToRad });
}

void Renderer::Consume(const SimData& data) {
  std::vector<FilterPtrU> filters;
  for (const auto& f : config_.ms_filter_) {
    filters.emplace_back(Filter::Create(f));
  }

  int wl_key = static_cast<int>(data.curr_wl_);
  if (!internal_data_.count(wl_key)) {
    int total_pix = config_.resolution_[0] * config_.resolution_[1];
    internal_data_.emplace(wl_key, std::unique_ptr<float[]>{ new float[total_pix]{} });
  }
  float* curr_data = internal_data_.at(wl_key).get();

  int xy[2];
  auto lens_proj = GetProjFunc(config_.lens_.type_);
  LensProjParam proj_param{ config_.lens_.fov_,
                            diag_pix_,
                            rot_,
                            config_.visible_,
                            { config_.resolution_[0], config_.resolution_[1] },
                            { config_.lens_shift_[0], config_.lens_shift_[1] } };
  const auto& crystals = data.crystals_;

  for (size_t i = 0; i < data.rays_.size_; i++) {
    const auto& r = data.rays_[i];
    // Filter current ray
    // 1. ray state must be kOutgoing
    if (r.state_ != RaySeg::kOutgoing) {
      continue;
    }

    // 2. then check every filter for every scattering
    bool filter_checked = true;
    size_t curr_idx = i;
    size_t root_idx = r.root_ray_idx_;
    for (auto fit = filters.rbegin(); fit != filters.rend(); /* increament see below */) {
      if (curr_idx != kInvalidId) {
        root_idx = data.rays_[curr_idx].root_ray_idx_;
      } else {
        filter_checked = false;
        break;
      }

      (*fit)->InitCrystalSymmetry(crystals.at(r.crystal_id_));
      if (!(*fit)->Check(data.rays_[curr_idx])) {
        filter_checked = false;
        break;
      }

      // increament
      curr_idx = data.rays_[root_idx].prev_ray_idx_;
      fit++;
    }
    if (!filters.empty() && (!filter_checked || curr_idx != kInvalidId)) {
      continue;
    }

    // Do rendering
    LOG_DEBUG("render ray: %.4f,%.4f,%.4f,%.4f", r.d_[0], r.d_[1], r.d_[2], r.w_);
    lens_proj(proj_param, r.d_, xy);
    if (xy[0] < 0 || xy[0] >= config_.resolution_[0] || xy[1] < 0 || xy[1] >= config_.resolution_[1]) {
      continue;
    }
    curr_data[xy[1] * config_.resolution_[0] + xy[0]] += r.w_;
  }
  total_intensity_ += data.total_intensity_;
  LOG_DEBUG("renderer wl: %zu", internal_data_.size());
}

Result Renderer::GetResult() const {
  int total_pix = config_.resolution_[0] * config_.resolution_[1];
  std::unique_ptr<float[]> float_data{ new float[total_pix * 3]{} };
  for (const auto& [wl, data] : internal_data_) {
    if (wl < kMinWavelength || wl > kMaxWavelength) {
      continue;
    }
    for (int i = 0; i < total_pix; i++) {
      // Step 1. Spectrum to XYZ
      float* xyz = float_data.get() + i * 3;
      float v = data[i] * config_.intensity_factor_ / total_intensity_ * 1e5;  // TODO: determine the scale factor
      xyz[0] += kCmfX[wl - kMinWavelength] * v;
      xyz[1] += kCmfY[wl - kMinWavelength] * v;
      xyz[2] += kCmfZ[wl - kMinWavelength] * v;
    }
  }

  bool use_real_color = config_.ray_color_[0] < 0;
  float gray[3];
  for (int i = 0; i < total_pix; i++) {
    // Step 2. XYZ to linear RGB
    float* xyz = float_data.get() + i * 3;
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
    std::memcpy(float_data.get() + i * 3, rgb, 3 * sizeof(float));
  }

  // Step 3. Convert linear sRGB to sRGB
  SrgbGamma(float_data.get(), 3 * total_pix);

  for (int i = 0; i < total_pix * 3; i++) {
    image_buffer_[i] = static_cast<uint8_t>(float_data[i] * 255);
  }
  return RenderResult{ config_.resolution_[0], config_.resolution_[1], image_buffer_.get() };
}

}  // namespace v3
}  // namespace icehalo
