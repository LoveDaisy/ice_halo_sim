#include "render.hpp"

#include <cmath>
#include <cstring>
#include <limits>
#include <utility>

#include "context/context.hpp"
#include "core/math.hpp"
#include "include/log.hpp"
#include "process/color_data.hpp"
#include "process/simulation.hpp"
#include "util/threading_pool.hpp"

namespace icehalo {

void EqualAreaFishEye(Pose3f cam_pose,               // Camera rotation. [lon, lat, roll]
                      float hov,                     // Half field of view.
                      size_t data_number,            // Data number
                      const float* dir,              // Ray directions, [x, y, z]
                      int img_wid, int img_hei,      // Image size
                      float* img_xy,                 // Image coordinates
                      VisibleRange visible_range) {  // Visible range
  float img_r = std::max(img_wid, img_hei) / 2.0f;
  std::unique_ptr<float[]> dir_copy{ new float[data_number * 3]{} };

  cam_pose.ToRad();

  RotateZ(cam_pose.val(), dir, dir_copy.get(), 4, 3, data_number);
  for (size_t i = 0; i < data_number; i++) {
    if (std::abs(Norm3(dir_copy.get() + i * 3) - 1.0) > 1e-4) {
      img_xy[i * 2 + 0] = std::numeric_limits<float>::quiet_NaN();
      img_xy[i * 2 + 1] = std::numeric_limits<float>::quiet_NaN();
    } else if (visible_range == VisibleRange::kFront && dir_copy[i * 3 + 2] > 0) {
      img_xy[i * 2 + 0] = std::numeric_limits<float>::quiet_NaN();
      img_xy[i * 2 + 1] = std::numeric_limits<float>::quiet_NaN();
    } else if (visible_range == VisibleRange::kUpper && dir[i * 4 + 2] > 0) {
      img_xy[i * 2 + 0] = std::numeric_limits<float>::quiet_NaN();
      img_xy[i * 2 + 1] = std::numeric_limits<float>::quiet_NaN();
    } else if (visible_range == VisibleRange::kLower && dir[i * 4 + 2] < 0) {
      img_xy[i * 2 + 0] = std::numeric_limits<float>::quiet_NaN();
      img_xy[i * 2 + 1] = std::numeric_limits<float>::quiet_NaN();
    } else {
      float lon = std::atan2(-dir_copy[i * 3 + 1], -dir_copy[i * 3 + 0]);
      float lat = std::asin(-dir_copy[i * 3 + 2] / Norm3(dir_copy.get() + i * 3));
      float proj_r = img_r / 2.0f / std::sin(hov / 2.0f * math::kDegreeToRad);
      float r = 2.0f * proj_r * std::sin((math::kPi_2 - lat) / 2.0f);

      img_xy[i * 2 + 0] = r * std::cos(lon) + img_wid / 2.0f - 0.5f;
      img_xy[i * 2 + 1] = -r * std::sin(lon) + img_hei / 2.0f - 0.5f;  // y increase downside on image
    }
  }
}


void EquidistantFishEye(Pose3f cam_pose,               // Camera rotation. [lon, lat, roll]
                        float hov,                     // Half field of view.
                        size_t data_number,            // Data number
                        const float* dir,              // Ray directions, [x, y, z]
                        int img_wid, int img_hei,      // Image size
                        float* img_xy,                 // Image coordinates
                        VisibleRange visible_range) {  // Visible range
  float img_r = std::max(img_wid, img_hei) / 2.0f;
  std::unique_ptr<float[]> dir_copy{ new float[data_number * 3]{} };

  cam_pose.ToRad();

  RotateZ(cam_pose.val(), dir, dir_copy.get(), 4, 3, data_number);
  for (size_t i = 0; i < data_number; i++) {
    if (std::abs(Norm3(dir_copy.get() + i * 3) - 1.0) > 1e-4) {
      img_xy[i * 2 + 0] = std::numeric_limits<float>::quiet_NaN();
      img_xy[i * 2 + 1] = std::numeric_limits<float>::quiet_NaN();
    } else if (visible_range == VisibleRange::kFront && dir_copy[i * 3 + 2] > 0) {
      img_xy[i * 2 + 0] = std::numeric_limits<float>::quiet_NaN();
      img_xy[i * 2 + 1] = std::numeric_limits<float>::quiet_NaN();
    } else if (visible_range == VisibleRange::kUpper && dir[i * 4 + 2] > 0) {
      img_xy[i * 2 + 0] = std::numeric_limits<float>::quiet_NaN();
      img_xy[i * 2 + 1] = std::numeric_limits<float>::quiet_NaN();
    } else if (visible_range == VisibleRange::kLower && dir[i * 4 + 2] < 0) {
      img_xy[i * 2 + 0] = std::numeric_limits<float>::quiet_NaN();
      img_xy[i * 2 + 1] = std::numeric_limits<float>::quiet_NaN();
    } else {
      float lon = std::atan2(-dir_copy[i * 3 + 1], -dir_copy[i * 3 + 0]);
      float lat = std::asin(-dir_copy[i * 3 + 2] / Norm3(dir_copy.get() + i * 3));
      float r = (math::kPi_2 - lat) / (hov * math::kDegreeToRad) * img_r;

      img_xy[i * 2 + 0] = r * std::cos(lon) + img_wid / 2.0f - 0.5f;
      img_xy[i * 2 + 1] = -r * std::sin(lon) + img_hei / 2.0f - 0.5f;  // y increase downside on image
    }
  }
}


void DualEqualAreaFishEye(Pose3f /* cam_rot */,                // Not used
                          float /* hov */,                     // Not used
                          size_t data_number,                  // Data number
                          const float* dir,                    // Ray directions, [x, y, z]
                          int img_wid, int img_hei,            // Image size
                          float* img_xy,                       // Image coordinates
                          VisibleRange /* visible_range */) {  // Not used
  float img_r = std::min(img_wid / 2, img_hei) / 2.0f;
  float proj_r = img_r / 2.0f / std::sin(45.0f * math::kDegreeToRad);

  std::unique_ptr<float[]> dir_copy{ new float[data_number * 3]{} };

  Pose3f cam_pose(90.0f, 90.0f, 0.0f);
  cam_pose.ToRad();

  RotateZ(cam_pose.val(), dir, dir_copy.get(), 4, 3, data_number);
  for (size_t i = 0; i < data_number; i++) {
    if (std::abs(Norm3(dir_copy.get() + i * 3) - 1.0) > 1e-4) {
      img_xy[i * 2 + 0] = std::numeric_limits<float>::quiet_NaN();
      img_xy[i * 2 + 1] = std::numeric_limits<float>::quiet_NaN();
    } else {
      float lon = std::atan2(-dir_copy[i * 3 + 1], -dir_copy[i * 3 + 0]);
      float lat = std::asin(-dir_copy[i * 3 + 2] / Norm3(dir_copy.get() + i * 3));
      if (lat < 0) {
        lon = math::kPi - lon;
      }
      float r = 2.0f * proj_r * std::sin((math::kPi_2 - std::abs(lat)) / 2.0f);

      img_xy[i * 2 + 0] = r * std::cos(lon) + img_r + (lat > 0 ? -0.5f : 2 * img_r - 0.5f);
      img_xy[i * 2 + 1] = -r * std::sin(lon) + img_r - 0.5f;  // y increase downside on image
    }
  }
}


void DualEquidistantFishEye(Pose3f /* cam_rot */,                // Not used
                            float /* hov */,                     // Not used
                            size_t data_number,                  // Data number
                            const float* dir,                    // Ray directions, [x, y, z]
                            int img_wid, int img_hei,            // Image size
                            float* img_xy,                       // Image coordinates
                            VisibleRange /* visible_range */) {  // Not used
  float img_r = std::min(img_wid / 2, img_hei) / 2.0f;

  std::unique_ptr<float[]> dir_copy{ new float[data_number * 3]{} };

  Pose3f cam_pose(90.0f, 90.0f, 0.0f);
  cam_pose.ToRad();

  RotateZ(cam_pose.val(), dir, dir_copy.get(), 4, 3, data_number);
  for (size_t i = 0; i < data_number; i++) {
    if (std::abs(Norm3(dir_copy.get() + i * 3) - 1.0) > 1e-4) {
      img_xy[i * 2 + 0] = std::numeric_limits<float>::quiet_NaN();
      img_xy[i * 2 + 1] = std::numeric_limits<float>::quiet_NaN();
    } else {
      float lon = std::atan2(-dir_copy[i * 3 + 1], -dir_copy[i * 3 + 0]);
      float lat = std::asin(-dir_copy[i * 3 + 2] / Norm3(dir_copy.get() + i * 3));
      if (lat < 0) {
        lon = math::kPi - lon;
      }
      float r = (1.0f - std::abs(lat) * 2.0f / math::kPi) * img_r;

      img_xy[i * 2 + 0] = r * std::cos(lon) + img_r + (lat > 0 ? -0.5f : 2 * img_r - 0.5f);
      img_xy[i * 2 + 1] = -r * std::sin(lon) + img_r - 0.5f;  // y increase downside on image
    }
  }
}


void Linear(Pose3f cam_pose,               // Camera rotation. [lon, lat, roll]
            float hov,                     // Half field of view.
            size_t data_number,            // Data number
            const float* dir,              // Ray directions, [x, y, z]
            int img_wid, int img_hei,      // Image size
            float* img_xy,                 // Image coordinates
            VisibleRange visible_range) {  // Visible range
  std::unique_ptr<float[]> dir_copy{ new float[data_number * 3]{} };

  cam_pose.ToRad();

  RotateZ(cam_pose.val(), dir, dir_copy.get(), 4, 3, data_number);
  for (size_t i = 0; i < data_number; i++) {
    if (dir_copy[i * 3 + 2] > 0 || std::abs(Norm3(dir_copy.get() + i * 3) - 1.0) > 1e-4) {
      img_xy[i * 2 + 0] = std::numeric_limits<float>::quiet_NaN();
      img_xy[i * 2 + 1] = std::numeric_limits<float>::quiet_NaN();
    } else if (visible_range == VisibleRange::kFront && dir_copy[i * 3 + 2] > 0) {
      img_xy[i * 2 + 0] = std::numeric_limits<float>::quiet_NaN();
      img_xy[i * 2 + 1] = std::numeric_limits<float>::quiet_NaN();
    } else if (visible_range == VisibleRange::kUpper && dir[i * 4 + 2] > 0) {
      img_xy[i * 2 + 0] = std::numeric_limits<float>::quiet_NaN();
      img_xy[i * 2 + 1] = std::numeric_limits<float>::quiet_NaN();
    } else if (visible_range == VisibleRange::kLower && dir[i * 4 + 2] < 0) {
      img_xy[i * 2 + 0] = std::numeric_limits<float>::quiet_NaN();
      img_xy[i * 2 + 1] = std::numeric_limits<float>::quiet_NaN();
    } else {
      float x = dir_copy[i * 3 + 0] / dir_copy[i * 3 + 2];
      float y = dir_copy[i * 3 + 1] / dir_copy[i * 3 + 2];

      img_xy[i * 2 + 0] = img_wid / 2.0f + img_wid / 2.0f * x / std::tan(hov * math::kDegreeToRad);
      img_xy[i * 2 + 1] = img_hei / 2.0f - img_wid / 2.0f * y / std::tan(hov * math::kDegreeToRad);
    }
  }
}


void Equirectangular(Pose3f cam_pose,                     // Not used
                     float /* hov */,                     // Not used
                     size_t data_number,                  // Data number
                     const float* dir,                    // Ray directions [x, y, z]
                     int img_wid, int img_hei,            // Image size
                     float* img_xy,                       // Image coordinates
                     VisibleRange /* visible_range */) {  // Not used
  float img_r = std::min(img_wid / 2, img_hei) / 2.0f;

  std::unique_ptr<float[]> dir_copy{ new float[data_number * 3]{} };

  cam_pose.val(90.0f - cam_pose.lon(), 90.0f, 0.0f);
  cam_pose.ToRad();

  RotateZ(cam_pose.val(), dir, dir_copy.get(), 4, 3, data_number);
  for (size_t i = 0; i < data_number; i++) {
    if (std::abs(Norm3(dir_copy.get() + i * 3) - 1.0) > 1e-4) {
      img_xy[i * 2 + 0] = std::numeric_limits<float>::quiet_NaN();
      img_xy[i * 2 + 1] = std::numeric_limits<float>::quiet_NaN();
    } else {
      float lon = std::atan2(-dir_copy[i * 3 + 1], -dir_copy[i * 3 + 0]);
      float lat = std::asin(-dir_copy[i * 3 + 2] / Norm3(dir_copy.get() + i * 3));
      img_xy[i * 2 + 0] = static_cast<int>(img_wid / 2.0f + lon / math::kPi * 2 * img_r - 0.5f);
      img_xy[i * 2 + 1] = static_cast<int>(img_hei / 2.0f - lat / math::kPi * 2 * img_r - 0.5f);
    }
  }
}


ProjectionFunction GetProjectionFunction(LensType lens_type) {
  static EnumMap<LensType, ProjectionFunction> projection_functions = {
    { LensType::kLinear, &Linear },
    { LensType::kEqualArea, &EqualAreaFishEye },
    { LensType::kEquidistant, &EquidistantFishEye },
    { LensType::kDualEquidistant, &DualEquidistantFishEye },
    { LensType::kDualEqualArea, &DualEqualAreaFishEye },
    { LensType::kEquirectangular, &Equirectangular },
  };

  if (projection_functions.count(lens_type)) {
    return projection_functions[lens_type];
  } else {
    return {};
  }
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

void SpecToRgbJob(int i, const std::vector<ImageSpectrumData>& spec_data, float factor, const float* background_color,
                  const float* ray_color, uint8_t* rgb_data) {
  /* Step 1. Spectrum to XYZ */
  float xyz[3]{};
  for (const auto& [wl, val] : spec_data) {
    if (wl < kMinWavelength || wl > kMaxWaveLength) {
      continue;
    }
    float v = val[i] * factor;
    xyz[0] += kCmfX[wl - kMinWavelength] * v;
    xyz[1] += kCmfY[wl - kMinWavelength] * v;
    xyz[2] += kCmfZ[wl - kMinWavelength] * v;
  }

  /* Step 2. XYZ to linear RGB */
  float gray[3];
  for (int j = 0; j < 3; j++) {
    gray[j] = kWhitePointD65[j] * xyz[1];
  }

  bool use_real_color = ray_color[0] < 0;
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
    std::memcpy(xyz, gray, sizeof(xyz));
  }
  float rgb[3]{};
  for (int j = 0; j < 3; j++) {
    for (int k = 0; k < 3; k++) {
      rgb[j] += xyz[k] * kXyzToRgb[j * 3 + k];
    }
    rgb[j] = std::min(std::max(rgb[j], 0.0f), 1.0f);
    if (!use_real_color) {
      rgb[j] *= ray_color[j];
    }
    rgb[j] += background_color[j];
  }

  /* Step 3. Convert linear sRGB to sRGB */
  SrgbGamma(rgb, 3);
  for (int j = 0; j < 3; j++) {
    rgb_data[i * 3 + j] = static_cast<uint8_t>(rgb[j] * std::numeric_limits<uint8_t>::max());
  }
}


void RenderSpecToRgb(const ThreadingPoolPtr& threading_pool,
                     const std::vector<ImageSpectrumData>& spec_data,        // spectrum data
                     size_t data_number, float factor,                       //
                     const float* background_color, const float* ray_color,  // background and ray color
                     uint8_t* rgb_data) {                                    // rgb data, data_number * 3
  threading_pool->CommitRangeStepJobsAndWait(0, data_number, [=, &spec_data](int /* thread_id */, int i) {
    SpecToRgbJob(i, spec_data, factor, background_color, ray_color, rgb_data);
  });
}


void RenderSpecToGray(const ThreadingPoolPtr& threading_pool,
                      const std::vector<ImageSpectrumData>& spec_data,  // spec_data: wavelength_number * data_number
                      size_t data_number, float factor,                 //
                      ColorCompactLevel level, int index,               // color compact level and channel index
                      uint8_t* rgb_data) {                              // rgb data, data_number * 3
  if (index < 0 || static_cast<size_t>(index) >= spec_data.size()) {
    return;
  }
  auto* curr_spec_data = spec_data[index].second.get();
  threading_pool->CommitRangeStepJobsAndWait(0, data_number, [=](int /* thread_id */, int i) {
    /* Step 1. Spectrum to XYZ */
    float y = curr_spec_data[i] * factor;

    /* Step 2. XYZ to linear RGB */
    float gray[3];
    for (int j = 0; j < 3; j++) {
      gray[j] = kWhitePointD65[j] * y;
    }

    float val = 0;
    for (int k = 0; k < 3; k++) {
      val += gray[k] * kXyzToRgb[3 + k];
    }
    val = std::min(std::max(val, 0.0f), 1.0f);

    /* Step 3. Convert linear sRGB to sRGB */
    SrgbGamma(&val, 1);
    if (level == ColorCompactLevel::kMonochrome && 0 <= index && index < 3) {
      rgb_data[i * 3 + index] = static_cast<uint8_t>(val * std::numeric_limits<uint8_t>::max());
    } else if (level == ColorCompactLevel::kLowQuality && 0 <= index && index < 6) {
      constexpr uint8_t kMaxVal = 1 << 4;
      auto tmp_val = static_cast<uint8_t>(val * kMaxVal);
      rgb_data[i * 3 + index / 2] |= (tmp_val << (index % 2 ? 0 : 4));
    }
  });
}


Renderer::Renderer()
    : cam_ctx_{}, render_ctx_{}, sun_ctx_{}, output_image_buffer_{}, total_w_(0),
      threading_pool_(ThreadingPool::CreatePool()) {}


Renderer::Renderer(Renderer&& other) noexcept
    : cam_ctx_(std::move(other.cam_ctx_)), render_ctx_(std::move(other.render_ctx_)),
      sun_ctx_(std::move(other.sun_ctx_)), output_image_buffer_(std::move(other.output_image_buffer_)),
      spectrum_data_(std::move(other.spectrum_data_)),
      spectrum_data_compensation_(std::move(other.spectrum_data_compensation_)), total_w_(other.total_w_),
      threading_pool_(std::move(other.threading_pool_)) {}


Renderer& Renderer::operator=(Renderer&& other) noexcept {
  cam_ctx_ = std::move(other.cam_ctx_);
  render_ctx_ = std::move(other.render_ctx_);
  sun_ctx_ = std::move(other.sun_ctx_);
  output_image_buffer_ = std::move(other.output_image_buffer_);
  spectrum_data_ = std::move(other.spectrum_data_);
  spectrum_data_compensation_ = std::move(other.spectrum_data_compensation_);
  total_w_ = other.total_w_;
  threading_pool_ = std::move(other.threading_pool_);
  return *this;
}


void Renderer::SetCameraContext(CameraContextPtr cam_ctx) {
  cam_ctx_ = std::move(cam_ctx);
}


void Renderer::SetRenderContext(RenderContextPtr render_ctx) {
  render_ctx_ = std::move(render_ctx);
  output_image_buffer_.reset(new uint8_t[render_ctx_->GetImageWidth() * render_ctx_->GetImageHeight() * 3]);
}


void Renderer::SetSunContext(SunContextPtr sun_ctx) {
  sun_ctx_ = std::move(sun_ctx);
}


void Renderer::SetThreadingPool(ThreadingPoolPtr threading_pool) {
  threading_pool_ = std::move(threading_pool);
}


void Renderer::LoadRayData(int identifier, const RayCollectionInfo& collection_info,
                           const SimpleRayData& final_ray_data) {
  if (!cam_ctx_) {
    throw std::invalid_argument("Camera context is not set!");
  }
  if (!render_ctx_) {
    throw std::invalid_argument("Render context is not set!");
  }

  auto weight = final_ray_data.wavelength_weight;
  auto color_compact_level = render_ctx_->GetColorCompactLevel();
  auto wavelength = final_ray_data.wavelength;
  if (color_compact_level == ColorCompactLevel::kTrueColor &&
      (wavelength < kMinWavelength || wavelength > kMaxWaveLength || weight <= 0)) {
    LOG_ERROR("Wavelength out of range!");
    return;
  }

  auto img_hei = render_ctx_->GetImageHeight();
  auto img_wid = render_ctx_->GetImageWidth();

  auto data_iter = std::find_if(spectrum_data_.begin(), spectrum_data_.end(),
                                [=](const ImageSpectrumData& d) { return d.first == identifier; });
  if (data_iter == spectrum_data_.end()) {
    spectrum_data_.emplace_back(identifier, new float[img_hei * img_wid]{});
    data_iter = spectrum_data_.end() - 1;
  }
  auto* current_data = data_iter->second.get();

  auto data_cmp_iter = std::find_if(spectrum_data_compensation_.begin(), spectrum_data_compensation_.end(),
                                    [=](const ImageSpectrumData& d) { return d.first == identifier; });
  if (data_cmp_iter == spectrum_data_compensation_.end()) {
    spectrum_data_compensation_.emplace_back(identifier, new float[img_hei * img_wid]{});
    data_cmp_iter = spectrum_data_compensation_.end() - 1;
  }
  auto* current_data_compensation = data_cmp_iter->second.get();

  if (collection_info.is_partial_data) {
    LoadPartialRayData(collection_info.idx, final_ray_data, current_data, current_data_compensation);
  } else {
    LoadFullRayData(final_ray_data, current_data, current_data_compensation);
  }

  total_w_ += final_ray_data.init_ray_num * weight;
}


void Renderer::LoadPartialRayData(const std::vector<size_t>& idx, const SimpleRayData& final_ray_data,
                                  float* current_data, float* current_data_compensation) {
  auto img_hei = render_ctx_->GetImageHeight();
  auto img_wid = render_ctx_->GetImageWidth();

  auto projection_type = cam_ctx_->GetLensType();
  auto pf = GetProjectionFunction(projection_type);
  if (!pf) {
    LOG_ERROR("Unknown projection type!");
    return;
  }

  const auto* final_ray_buf = final_ray_data.buf.get();
  auto weight = final_ray_data.wavelength_weight;
  auto num = idx.size();
  threading_pool_->CommitRangeSliceJobsAndWait(0, num, [=](int /* thread_id */, int start_idx, int end_idx) {
    size_t current_num = end_idx - start_idx;
    std::unique_ptr<float[]> tmp_xy{ new float[current_num * 2] };
    for (size_t j = 0; j < current_num; j++) {
      pf(cam_ctx_->GetCameraTargetDirection(), cam_ctx_->GetFov(), 1, final_ray_buf + idx[start_idx + j] * 4, img_wid,
         img_hei, tmp_xy.get() + j * 2, render_ctx_->GetVisibleRange());

      if (std::isnan(tmp_xy[j * 2 + 0]) || std::isnan(tmp_xy[j * 2 + 1])) {
        continue;
      }
      int x = static_cast<int>(tmp_xy[j * 2 + 0]);
      int y = static_cast<int>(tmp_xy[j * 2 + 1]);
      if (projection_type != LensType::kDualEqualArea && projection_type != LensType::kDualEquidistant &&
          projection_type != LensType::kEquirectangular) {
        x += render_ctx_->GetImageOffsetX();
        y += render_ctx_->GetImageOffsetY();
      }
      if (x < 0 || x >= static_cast<int>(img_wid) || y < 0 || y >= static_cast<int>(img_hei)) {
        continue;
      }
      auto tmp_val = final_ray_buf[(start_idx + j) * 4 + 3] * weight - current_data_compensation[y * img_wid + x];
      auto tmp_sum = current_data[y * img_wid + x] + tmp_val;
      current_data_compensation[y * img_wid + x] = tmp_sum - current_data[y * img_wid + x] - tmp_val;
      current_data[y * img_wid + x] = tmp_sum;
    }
  });
}


void Renderer::LoadFullRayData(const SimpleRayData& final_ray_data, float* current_data,
                               float* current_data_compensation) {
  auto img_hei = render_ctx_->GetImageHeight();
  auto img_wid = render_ctx_->GetImageWidth();

  auto projection_type = cam_ctx_->GetLensType();
  auto pf = GetProjectionFunction(projection_type);
  if (!pf) {
    LOG_ERROR("Unknown projection type!");
    return;
  }

  auto num = final_ray_data.buf_ray_num;
  const auto* final_ray_buf = final_ray_data.buf.get();
  auto weight = final_ray_data.wavelength_weight;
  threading_pool_->CommitRangeSliceJobsAndWait(0, num, [=](int /* thread_id */, int start_idx, int end_idx) {
    size_t current_num = end_idx - start_idx;
    std::unique_ptr<float[]> tmp_xy{ new float[current_num * 2] };
    pf(cam_ctx_->GetCameraTargetDirection(), cam_ctx_->GetFov(), current_num, final_ray_buf + start_idx * 4, img_wid,
       img_hei, tmp_xy.get(), render_ctx_->GetVisibleRange());

    for (size_t j = 0; j < current_num; j++) {
      if (std::isnan(tmp_xy[j * 2 + 0]) || std::isnan(tmp_xy[j * 2 + 1])) {
        continue;
      }
      int x = static_cast<int>(tmp_xy[j * 2 + 0]);
      int y = static_cast<int>(tmp_xy[j * 2 + 1]);
      if (projection_type != LensType::kDualEqualArea && projection_type != LensType::kDualEquidistant &&
          projection_type != LensType::kEquirectangular) {
        x += render_ctx_->GetImageOffsetX();
        y += render_ctx_->GetImageOffsetY();
      }
      if (x < 0 || x >= static_cast<int>(img_wid) || y < 0 || y >= static_cast<int>(img_hei)) {
        continue;
      }
      auto tmp_val = final_ray_buf[(start_idx + j) * 4 + 3] * weight - current_data_compensation[y * img_wid + x];
      auto tmp_sum = current_data[y * img_wid + x] + tmp_val;
      current_data_compensation[y * img_wid + x] = tmp_sum - current_data[y * img_wid + x] - tmp_val;
      current_data[y * img_wid + x] = tmp_sum;
    }
  });
}


void Renderer::Render() {
  if (!render_ctx_) {
    throw std::invalid_argument("Render context is not set!");
  }

  RenderHaloImage();
  DrawGrids();
}


void Renderer::RenderHaloImage() {
  auto img_hei = render_ctx_->GetImageHeight();
  auto img_wid = render_ctx_->GetImageWidth();
  const auto* ray_color = render_ctx_->GetRayColor();
  const auto* background_color = render_ctx_->GetBackgroundColor();
  auto factor = static_cast<float>(img_hei * img_wid / 160.0 / total_w_ * render_ctx_->GetIntensity());
  auto color_compact_level = render_ctx_->GetColorCompactLevel();

  if (color_compact_level == ColorCompactLevel::kTrueColor) {
    RenderSpecToRgb(threading_pool_, spectrum_data_, img_wid * img_hei, factor, background_color, ray_color,
                    output_image_buffer_.get());
  } else {
    auto ch_num = std::min(spectrum_data_.size(), kImageBits / static_cast<size_t>(color_compact_level));
    for (size_t i = 0; i < ch_num; i++) {
      RenderSpecToGray(threading_pool_, spectrum_data_, img_wid * img_hei, factor, color_compact_level, i,
                       output_image_buffer_.get());
    }
  }
}


void SetPixelValue(int img_wid, int img_hei, uint8_t* image_data, int x, int y, const float color[3], float alpha) {
  constexpr uint8_t kMaxPixelValue = std::numeric_limits<uint8_t>::max();
  if (x >= 0 && x < img_wid && y >= 0 && y < img_hei) {
    auto curr_ind = y * img_wid + x;
    image_data[curr_ind * 3 + 0] =
        static_cast<uint8_t>(color[0] * alpha * kMaxPixelValue + image_data[curr_ind * 3 + 0] * (1 - alpha));
    image_data[curr_ind * 3 + 1] =
        static_cast<uint8_t>(color[1] * alpha * kMaxPixelValue + image_data[curr_ind * 3 + 1] * (1 - alpha));
    image_data[curr_ind * 3 + 2] =
        static_cast<uint8_t>(color[2] * alpha * kMaxPixelValue + image_data[curr_ind * 3 + 2] * (1 - alpha));
  }
}


void DrawLine(size_t pt_num, const float* xy, int img_wid, int img_hei, uint8_t* image_data, LineSpecifier line_spec) {
  auto width = line_spec.width;
  auto alpha = line_spec.alpha;
  const auto* color = line_spec.color;

  float last_arc_len = 0.0f;
  bool draw_line = true;
  for (size_t i = 0; i + 1 < pt_num; i++) {
    auto curr_x = xy[i * 2 + 0];
    auto curr_y = xy[i * 2 + 1];
    auto next_x = xy[(i + 1) * 2 + 0];
    auto next_y = xy[(i + 1) * 2 + 1];

    if ((curr_x < 0 || curr_x >= img_wid || curr_y < 0 || curr_y >= img_hei) &&
        (next_x < 0 || next_x >= img_wid || next_y < 0 || next_y >= img_hei)) {
      continue;
    }
    auto d2 = (curr_x - next_x) * (curr_x - next_x) + (curr_y - next_y) * (curr_y - next_y);
    auto exclusion_limit = std::max(img_wid, img_hei) * Renderer::kLineD2ExclusionLimitRatio;
    if (d2 > exclusion_limit * exclusion_limit) {
      continue;
    }

    bool steep = std::abs(next_y - curr_y) > std::abs(next_x - curr_x);
    if (steep) {
      std::swap(curr_x, curr_y);
      std::swap(next_x, next_y);
    }
    if (curr_x > next_x) {
      std::swap(curr_x, next_x);
      std::swap(curr_y, next_y);
    }
    auto dx = next_x - curr_x;
    auto dy = next_y - curr_y;
    auto gradient = dy / dx;

    float curr_seg_len = 0.0f;
    float last_s = std::numeric_limits<float>::quiet_NaN();
    float last_t = std::numeric_limits<float>::quiet_NaN();

    auto start_x = i == 0 ? curr_x - width / 2.0f : curr_x;
    auto end_x = i + 2 == pt_num ? next_x + width / 2.0f : next_x;
    for (auto s = std::floor(start_x); s < std::ceil(end_x); s += 1.0f) {
      auto tmp_dx = s - curr_x;
      auto t = curr_y + tmp_dx * gradient;
      if (std::isnan(last_s)) {
        last_s = s;
        last_t = t;
      }

      if (line_spec.type == LineType::kDashed) {
        curr_seg_len = std::sqrt((s - last_s) * (s - last_s) + (t - last_t) * (t - last_t));
        if (last_arc_len + curr_seg_len >= LineSpecifier::kDefaultDashSize * width) {
          last_arc_len -= LineSpecifier::kDefaultDashSize * width;
          last_s = s;
          last_t = t;
          draw_line = !draw_line;
        }
      }
      if (!draw_line) {
        continue;
      }

      auto start_y = curr_y + tmp_dx * gradient - width / 2.0f;
      auto end_y = curr_y + tmp_dx * gradient + width / 2.0f;
      for (t = std::floor(start_y); t < std::ceil(end_y); t += 1.0f) {
        auto tmp_dy = t - curr_y;
        auto p = (tmp_dx * dx + tmp_dy * dy) / (dx * dx + dy * dy);
        p = std::min(std::max(p, 0.0f), 1.0f);
        auto d = std::sqrt((tmp_dx - p * dx) * (tmp_dx - p * dx) + (tmp_dy - p * dy) * (tmp_dy - p * dy));
        auto weight = 0.5f - d + width / 2.0f;
        weight = std::min(std::max(weight, 0.0f), 1.0f);

        auto tmp_x = static_cast<int>(s);
        auto tmp_y = static_cast<int>(t);
        if (steep) {
          std::swap(tmp_x, tmp_y);
        }
        SetPixelValue(img_wid, img_hei, image_data, tmp_x, tmp_y, color, weight * alpha);
      }
    }
    last_arc_len += curr_seg_len;
  }
}


void Renderer::DrawGrids() {
  DrawElevationGrids();
  DrawRadiusGrids();
}


void Renderer::DrawElevationGrids() {
  auto projection_type = cam_ctx_->GetLensType();
  auto pf = GetProjectionFunction(projection_type);
  if (!pf) {
    LOG_ERROR("Unknown projection type!");
    return;
  }

  auto cam_pose = cam_ctx_->GetCameraTargetDirection();
  auto hov = cam_ctx_->GetFov();
  auto img_wid = render_ctx_->GetImageWidth();
  auto img_hei = render_ctx_->GetImageHeight();
  auto visible_range = render_ctx_->GetVisibleRange();
  auto need_offset =
      (cam_ctx_->GetLensType() != LensType::kDualEqualArea && cam_ctx_->GetLensType() != LensType::kDualEquidistant &&
       cam_ctx_->GetLensType() != LensType::kEquirectangular);

  std::vector<GridLine> elevation_grids{};
  for (const auto& g : render_ctx_->GetElevationGrids()) {
    if (!need_offset && std::abs(g.value) < 1e-4) {
      elevation_grids.emplace_back(g);
      elevation_grids.back().value = 0.01f;
      elevation_grids.emplace_back(g);
      elevation_grids.back().value = -0.01f;
    } else {
      elevation_grids.emplace_back(g);
    }
  }

  std::vector<float> line_pts;
  for (const auto& g : elevation_grids) {
    line_pts.clear();
    float curr_step = kDefaultLineStep;
    float curr_dir[3]{};
    float curr_xy[2]{};
    float last_xy[2]{};
    for (float azi = 0.0f; azi < 360.0f + curr_step / 2; /* update azi inside loop */) {
      curr_dir[0] = std::cos(azi * math::kDegreeToRad) * std::cos(g.value * math::kDegreeToRad);
      curr_dir[1] = std::sin(azi * math::kDegreeToRad) * std::cos(g.value * math::kDegreeToRad);
      curr_dir[2] = std::sin(g.value * math::kDegreeToRad);
      pf(cam_pose, hov, 1, curr_dir, img_wid, img_hei, curr_xy, visible_range);
      if (need_offset) {
        curr_xy[0] += render_ctx_->GetImageOffsetX();
        curr_xy[1] += render_ctx_->GetImageOffsetY();
      }
      if (line_pts.size() >= 2 && (curr_xy[0] > 0 && curr_xy[0] < img_wid && curr_xy[1] > 0 && curr_xy[1] < img_hei)) {
        float d2 = (last_xy[0] - curr_xy[0]) * (last_xy[0] - curr_xy[0]) +
                   (last_xy[1] - curr_xy[1]) * (last_xy[1] - curr_xy[1]);
        if (d2 > kLineD2Upper && curr_step > kMinLineStep) {
          azi = azi - curr_step / 2.0f;
          curr_step /= 2.0f;
          continue;
        } else if (d2 < kLineD2Lower && curr_step < kMaxLineStep) {
          azi = azi + curr_step;
          curr_step *= 2.0f;
          continue;
        }
      }
      azi += curr_step;
      line_pts.emplace_back(curr_xy[0]);
      line_pts.emplace_back(curr_xy[1]);
      last_xy[0] = curr_xy[0];
      last_xy[1] = curr_xy[1];
    }
    DrawLine(line_pts.size() / 2, line_pts.data(), img_wid, img_hei, output_image_buffer_.get(), g.line_specifier);
  }
}


void Renderer::DrawRadiusGrids() {
  auto projection_type = cam_ctx_->GetLensType();
  auto pf = GetProjectionFunction(projection_type);
  if (!pf) {
    LOG_ERROR("Unknown projection type!");
    return;
  }

  auto cam_pose = cam_ctx_->GetCameraTargetDirection();
  auto hov = cam_ctx_->GetFov();
  auto img_wid = render_ctx_->GetImageWidth();
  auto img_hei = render_ctx_->GetImageHeight();
  auto visible_range = render_ctx_->GetVisibleRange();
  auto need_offset =
      (cam_ctx_->GetLensType() != LensType::kDualEqualArea && cam_ctx_->GetLensType() != LensType::kDualEquidistant &&
       cam_ctx_->GetLensType() != LensType::kEquirectangular);

  std::vector<float> line_pts;
  Pose3f sun_pose{ 90.0f, sun_ctx_->GetSunAltitude(), 0 };
  sun_pose.ToRad();
  for (const auto& g : render_ctx_->GetRadiusGrids()) {
    line_pts.clear();
    // constexpr float kAngStep = 1.0f;
    float curr_step = kDefaultLineStep;
    float last_z = 0;
    float curr_dir[3]{};
    float central_dir[3]{};
    float curr_xy[2]{};
    float last_xy[2]{};
    for (float ang = 0.0f; ang < 360.0f + curr_step / 2; /* update ang inside loop */) {
      central_dir[0] = -std::cos(ang * math::kDegreeToRad) * std::sin(g.value * math::kDegreeToRad);
      central_dir[1] = -std::sin(ang * math::kDegreeToRad) * std::sin(g.value * math::kDegreeToRad);
      central_dir[2] = -std::cos(g.value * math::kDegreeToRad);
      RotateZBack(sun_pose.val(), central_dir, curr_dir);
      pf(cam_pose, hov, 1, curr_dir, img_wid, img_hei, curr_xy, visible_range);
      if (need_offset) {
        curr_xy[0] += render_ctx_->GetImageOffsetX();
        curr_xy[1] += render_ctx_->GetImageOffsetY();
      }
      if (line_pts.size() >= 2 && (curr_xy[0] > 0 && curr_xy[0] < img_wid && curr_xy[1] > 0 && curr_xy[1] < img_hei)) {
        float d2 = (last_xy[0] - curr_xy[0]) * (last_xy[0] - curr_xy[0]) +
                   (last_xy[1] - curr_xy[1]) * (last_xy[1] - curr_xy[1]);
        if (d2 > kLineD2Upper && curr_step > kMinLineStep) {
          ang = ang - curr_step / 2.0f;
          curr_step /= 2.0f;
          continue;
        } else if (d2 < kLineD2Lower && curr_step < kMaxLineStep) {
          ang = ang + curr_step;
          curr_step *= 2.0f;
          continue;
        }
      }
      ang += curr_step;

      if (!need_offset && !line_pts.empty() && last_z * curr_dir[2] < 0) {
        DrawLine(line_pts.size() / 2, line_pts.data(), img_wid, img_hei, output_image_buffer_.get(), g.line_specifier);
        line_pts.clear();
      }
      line_pts.emplace_back(curr_xy[0]);
      line_pts.emplace_back(curr_xy[1]);
      last_z = curr_dir[2];
      last_xy[0] = curr_xy[0];
      last_xy[1] = curr_xy[1];
    }
    DrawLine(line_pts.size() / 2, line_pts.data(), img_wid, img_hei, output_image_buffer_.get(), g.line_specifier);
  }
}


uint8_t* Renderer::GetImageBuffer() const {
  return output_image_buffer_.get();
}


}  // namespace icehalo
