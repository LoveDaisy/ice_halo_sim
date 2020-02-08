#include "render.h"

#include <cmath>
#include <cstring>
#include <limits>
#include <utility>

#include "context.h"
#include "mymath.h"
#include "threadingpool.h"

namespace icehalo {

void EqualAreaFishEye(const float* cam_rot,          // Camera rotation. [lon, lat, roll]
                      float hov,                     // Half field of view.
                      size_t data_number,            // Data number
                      const float* dir,              // Ray directions, [x, y, z]
                      int img_wid, int img_hei,      // Image size
                      int* img_xy,                   // Image coordinates
                      VisibleRange visible_range) {  // Visible range
  float img_r = std::max(img_wid, img_hei) / 2.0f;
  std::unique_ptr<float[]> dir_copy{ new float[data_number * 3]{} };
  float cam_rot_copy[3];
  std::memcpy(cam_rot_copy, cam_rot, sizeof(float) * 3);
  cam_rot_copy[0] *= -1;
  cam_rot_copy[1] *= -1;
  for (auto& i : cam_rot_copy) {
    i *= math::kDegreeToRad;
  }

  math::RotateZWithDataStep(cam_rot_copy, dir, dir_copy.get(), 4, 3, data_number);
  for (size_t i = 0; i < data_number; i++) {
    if (std::abs(math::Norm3(dir_copy.get() + i * 3) - 1.0) > 1e-4) {
      img_xy[i * 2 + 0] = std::numeric_limits<int>::min();
      img_xy[i * 2 + 1] = std::numeric_limits<int>::min();
    } else if (visible_range == VisibleRange::kFront && dir_copy[i * 3 + 2] < 0) {
      img_xy[i * 2 + 0] = std::numeric_limits<int>::min();
      img_xy[i * 2 + 1] = std::numeric_limits<int>::min();
    } else if (visible_range == VisibleRange::kUpper && dir[i * 4 + 2] > 0) {
      img_xy[i * 2 + 0] = std::numeric_limits<int>::min();
      img_xy[i * 2 + 1] = std::numeric_limits<int>::min();
    } else if (visible_range == VisibleRange::kLower && dir[i * 4 + 2] < 0) {
      img_xy[i * 2 + 0] = std::numeric_limits<int>::min();
      img_xy[i * 2 + 1] = std::numeric_limits<int>::min();
    } else {
      float lon = std::atan2(dir_copy[i * 3 + 1], dir_copy[i * 3 + 0]);
      float lat = std::asin(dir_copy[i * 3 + 2] / math::Norm3(dir_copy.get() + i * 3));
      float proj_r = img_r / 2.0f / std::sin(hov / 2.0f * math::kDegreeToRad);
      float r = 2.0f * proj_r * std::sin((math::kPi / 2.0f - lat) / 2.0f);

      img_xy[i * 2 + 0] = static_cast<int>(std::round(r * std::cos(lon) + img_wid / 2.0));
      img_xy[i * 2 + 1] = static_cast<int>(std::round(r * std::sin(lon) + img_hei / 2.0));
    }
  }
}


void EquidistantFishEye(const float* cam_rot,          // Camera rotation. [lon, lat, roll]
                        float hov,                     // Half field of view.
                        size_t data_number,            // Data number
                        const float* dir,              // Ray directions, [x, y, z]
                        int img_wid, int img_hei,      // Image size
                        int* img_xy,                   // Image coordinates
                        VisibleRange visible_range) {  // Visible range
  float img_r = std::max(img_wid, img_hei) / 2.0f;
  std::unique_ptr<float[]> dir_copy{ new float[data_number * 3]{} };
  float cam_rot_copy[3];
  std::memcpy(cam_rot_copy, cam_rot, sizeof(float) * 3);
  cam_rot_copy[0] *= -1;
  cam_rot_copy[1] *= -1;
  for (auto& i : cam_rot_copy) {
    i *= math::kDegreeToRad;
  }

  math::RotateZWithDataStep(cam_rot_copy, dir, dir_copy.get(), 4, 3, data_number);
  for (decltype(data_number) i = 0; i < data_number; i++) {
    if (std::abs(math::Norm3(dir_copy.get() + i * 3) - 1.0) > 1e-4) {
      img_xy[i * 2 + 0] = std::numeric_limits<int>::min();
      img_xy[i * 2 + 1] = std::numeric_limits<int>::min();
    } else if (visible_range == VisibleRange::kFront && dir_copy[i * 3 + 2] < 0) {
      img_xy[i * 2 + 0] = std::numeric_limits<int>::min();
      img_xy[i * 2 + 1] = std::numeric_limits<int>::min();
    } else if (visible_range == VisibleRange::kUpper && dir[i * 4 + 2] > 0) {
      img_xy[i * 2 + 0] = std::numeric_limits<int>::min();
      img_xy[i * 2 + 1] = std::numeric_limits<int>::min();
    } else if (visible_range == VisibleRange::kLower && dir[i * 4 + 2] < 0) {
      img_xy[i * 2 + 0] = std::numeric_limits<int>::min();
      img_xy[i * 2 + 1] = std::numeric_limits<int>::min();
    } else {
      float lon = std::atan2(dir_copy[i * 3 + 1], dir_copy[i * 3 + 0]);
      float lat = std::asin(dir_copy[i * 3 + 2] / math::Norm3(dir_copy.get() + i * 3));
      float r = (math::kPi / 2.0f - lat) / (hov * math::kDegreeToRad) * img_r;

      img_xy[i * 2 + 0] = static_cast<int>(std::round(r * std::cos(lon) + img_wid / 2.0));
      img_xy[i * 2 + 1] = static_cast<int>(std::round(r * std::sin(lon) + img_hei / 2.0));
    }
  }
}


void DualEqualAreaFishEye(const float* /* cam_rot */,          // Not used
                          float /* hov */,                     // Not used
                          size_t data_number,                  // Data number
                          const float* dir,                    // Ray directions, [x, y, z]
                          int img_wid, int img_hei,            // Image size
                          int* img_xy,                         // Image coordinates
                          VisibleRange /* visible_range */) {  // Not used
  float img_r = std::min(img_wid / 2, img_hei) / 2.0f;
  float proj_r = img_r / 2.0f / std::sin(45.0f * math::kDegreeToRad);

  std::unique_ptr<float[]> dir_copy{ new float[data_number * 3]{} };
  float cam_rot_copy[3] = { 90.0f, 89.999f, 0.0f };
  cam_rot_copy[0] *= -1;
  cam_rot_copy[1] *= -1;
  for (auto& i : cam_rot_copy) {
    i *= math::kDegreeToRad;
  }

  math::RotateZWithDataStep(cam_rot_copy, dir, dir_copy.get(), 4, 3, data_number);
  for (decltype(data_number) i = 0; i < data_number; i++) {
    if (std::abs(math::Norm3(dir_copy.get() + i * 3) - 1.0) > 1e-4) {
      img_xy[i * 2 + 0] = std::numeric_limits<int>::min();
      img_xy[i * 2 + 1] = std::numeric_limits<int>::min();
    } else {
      float lon = std::atan2(dir_copy[i * 3 + 1], dir_copy[i * 3 + 0]);
      float lat = std::asin(dir_copy[i * 3 + 2] / math::Norm3(dir_copy.get() + i * 3));
      if (lat < 0) {
        lon = math::kPi - lon;
      }
      float r = 2.0f * proj_r * std::sin((math::kPi / 2.0f - std::abs(lat)) / 2.0f);

      img_xy[i * 2 + 0] = static_cast<int>(std::round(r * std::cos(lon) + img_r + (lat > 0 ? -0.5 : 2 * img_r - 0.5)));
      img_xy[i * 2 + 1] = static_cast<int>(std::round(r * std::sin(lon) + img_r - 0.5));
    }
  }
}


void DualEquidistantFishEye(const float* /* cam_rot */,          // Not used
                            float /* hov */,                     // Not used
                            size_t data_number,                  // Data number
                            const float* dir,                    // Ray directions, [x, y, z]
                            int img_wid, int img_hei,            // Image size
                            int* img_xy,                         // Image coordinates
                            VisibleRange /* visible_range */) {  // Not used
  float img_r = std::min(img_wid / 2, img_hei) / 2.0f;

  std::unique_ptr<float[]> dir_copy{ new float[data_number * 3]{} };
  float cam_rot_copy[3] = { 90.0f, 89.999f, 0.0f };
  cam_rot_copy[0] *= -1;
  cam_rot_copy[1] *= -1;
  for (auto& i : cam_rot_copy) {
    i *= math::kDegreeToRad;
  }

  math::RotateZWithDataStep(cam_rot_copy, dir, dir_copy.get(), 4, 3, data_number);
  for (decltype(data_number) i = 0; i < data_number; i++) {
    if (std::abs(math::Norm3(dir_copy.get() + i * 3) - 1.0) > 1e-4) {
      img_xy[i * 2 + 0] = std::numeric_limits<int>::min();
      img_xy[i * 2 + 1] = std::numeric_limits<int>::min();
    } else {
      float lon = std::atan2(dir_copy[i * 3 + 1], dir_copy[i * 3 + 0]);
      float lat = std::asin(dir_copy[i * 3 + 2] / math::Norm3(dir_copy.get() + i * 3));
      if (lat < 0) {
        lon = math::kPi - lon;
      }
      float r = (1.0f - std::abs(lat) * 2.0f / math::kPi) * img_r;

      img_xy[i * 2 + 0] = static_cast<int>(std::round(r * std::cos(lon) + img_r + (lat > 0 ? -0.5 : 2 * img_r - 0.5)));
      img_xy[i * 2 + 1] = static_cast<int>(std::round(r * std::sin(lon) + img_r - 0.5));
    }
  }
}


void RectLinear(const float* cam_rot,          // Camera rotation. [lon, lat, roll]
                float hov,                     // Half field of view.
                size_t data_number,            // Data number
                const float* dir,              // Ray directions, [x, y, z]
                int img_wid, int img_hei,      // Image size
                int* img_xy,                   // Image coordinates
                VisibleRange visible_range) {  // Visible range
  std::unique_ptr<float[]> dir_copy{ new float[data_number * 3]{} };
  float cam_rot_copy[3];
  std::memcpy(cam_rot_copy, cam_rot, sizeof(float) * 3);
  cam_rot_copy[0] *= -1;
  cam_rot_copy[1] *= -1;
  for (auto& i : cam_rot_copy) {
    i *= math::kDegreeToRad;
  }

  math::RotateZWithDataStep(cam_rot_copy, dir, dir_copy.get(), 4, 3, data_number);
  for (size_t i = 0; i < data_number; i++) {
    if (dir_copy[i * 3 + 2] < 0 || std::abs(math::Norm3(dir_copy.get() + i * 3) - 1.0) > 1e-4) {
      img_xy[i * 2 + 0] = std::numeric_limits<int>::min();
      img_xy[i * 2 + 1] = std::numeric_limits<int>::min();
    } else if (visible_range == VisibleRange::kFront && dir_copy[i * 3 + 2] < 0) {
      img_xy[i * 2 + 0] = std::numeric_limits<int>::min();
      img_xy[i * 2 + 1] = std::numeric_limits<int>::min();
    } else if (visible_range == VisibleRange::kUpper && dir[i * 4 + 2] > 0) {
      img_xy[i * 2 + 0] = std::numeric_limits<int>::min();
      img_xy[i * 2 + 1] = std::numeric_limits<int>::min();
    } else if (visible_range == VisibleRange::kLower && dir[i * 4 + 2] < 0) {
      img_xy[i * 2 + 0] = std::numeric_limits<int>::min();
      img_xy[i * 2 + 1] = std::numeric_limits<int>::min();
    } else {
      float x = dir_copy[i * 3 + 0] / dir_copy[i * 3 + 2];
      float y = dir_copy[i * 3 + 1] / dir_copy[i * 3 + 2];
      x = static_cast<float>(img_wid / 2.0 * x / std::tan(hov * math::kDegreeToRad) + img_wid / 2.0);
      y = static_cast<float>(img_wid / 2.0 * y / std::tan(hov * math::kDegreeToRad) + img_hei / 2.0);

      img_xy[i * 2 + 0] = static_cast<int>(std::round(x));
      img_xy[i * 2 + 1] = static_cast<int>(std::round(y));
    }
  }
}


MyUnorderedMap<LensType, ProjectionFunction>& GetProjectionFunctions() {
  static MyUnorderedMap<LensType, ProjectionFunction> projection_functions = {
    { LensType::kLinear, &RectLinear },
    { LensType::kEqualArea, &EqualAreaFishEye },
    { LensType::kEquidistant, &EquidistantFishEye },
    { LensType::kDualEquidistant, &DualEquidistantFishEye },
    { LensType::kDualEqualArea, &DualEqualAreaFishEye },
  };

  return projection_functions;
}


void SrgbGamma(float* linear_rgb) {
  for (int i = 0; i < 3; i++) {
    if (linear_rgb[i] < 0.0031308) {
      linear_rgb[i] *= 12.92f;
    } else {
      linear_rgb[i] = static_cast<float>(1.055 * std::pow(linear_rgb[i], 1.0 / 2.4) - 0.055);
    }
  }
}


constexpr float SpectrumRenderer::kWhitePointD65[];
constexpr float SpectrumRenderer::kXyzToRgb[];
constexpr float SpectrumRenderer::kCmfX[];
constexpr float SpectrumRenderer::kCmfY[];
constexpr float SpectrumRenderer::kCmfZ[];


SpectrumRenderer::SpectrumRenderer(ProjectContextPtr context)
    : context_(std::move(context)),
      output_image_buffer_{
        new uint8_t[3 * context_->render_ctx_->GetImageWidth() * context_->render_ctx_->GetImageHeight()]
      },
      total_w_(0) {}


SpectrumRenderer::~SpectrumRenderer() {
  ResetData();
}


void SpectrumRenderer::LoadDataFiles() {
  auto projection_type = context_->cam_ctx_->GetLensType();
  const auto& projection_functions = GetProjectionFunctions();
  if (projection_functions.find(projection_type) == projection_functions.end()) {
    std::fprintf(stderr, "Unknown projection type!\n");
    return;
  }

  std::vector<File> files = ListDataFiles(context_->GetDataDirectory().c_str());
  int i = 0;
  for (auto& f : files) {
    auto t0 = std::chrono::system_clock::now();
    auto num = LoadDataFromFile(f);
    auto t1 = std::chrono::system_clock::now();
    std::chrono::duration<float, std::ratio<1, 1000>> diff = t1 - t0;
    std::printf(" Loading data (%d/%zu): %.2fms; total %d pts\n", i + 1, files.size(), diff.count(), num);
    i++;
  }
}


void SpectrumRenderer::LoadData(float wl, float weight, const SimulationRayData& simulation_data) {
  const auto& ray_seg_set = simulation_data.GetFinalRaySegments();
  auto num = ray_seg_set.size();

  auto projection_type = context_->cam_ctx_->GetLensType();
  auto& projection_functions = GetProjectionFunctions();
  if (projection_functions.find(projection_type) == projection_functions.end()) {
    std::fprintf(stderr, "Unknown projection type!\n");
    return;
  }
  auto& pf = projection_functions[projection_type];

  auto wavelength = static_cast<int>(wl);
  if (wavelength < SpectrumRenderer::kMinWavelength || wavelength > SpectrumRenderer::kMaxWaveLength || weight < 0) {
    std::fprintf(stderr, "Wavelength out of range!\n");
    return;
  }

  auto img_hei = context_->render_ctx_->GetImageHeight();
  auto img_wid = context_->render_ctx_->GetImageWidth();

  if (!spectrum_data_.count(wavelength) || !spectrum_data_[wavelength]) {
    spectrum_data_[wavelength].reset(new float[img_hei * img_wid]{});
    spectrum_data_compensation_[wavelength].reset(new float[img_hei * img_wid]{});
  }
  auto* current_data = spectrum_data_[wavelength].get();
  auto* current_data_compensation = spectrum_data_compensation_[wavelength].get();

  auto threading_pool = ThreadingPool::GetInstance();
  threading_pool->AddRangeBasedJobs(num, [=, &ray_seg_set](size_t start_idx, size_t end_idx) {
    size_t current_num = end_idx - start_idx;
    std::unique_ptr<float[]> curr_data{ new float[current_num * 4] };
    auto* p = curr_data.get();
    for (size_t j = start_idx; j < end_idx; j++) {
      const auto& r = ray_seg_set[j];
      auto axis_rot = r->root_ctx->main_axis.val();
      icehalo::math::RotateZBack(axis_rot, r->dir.val(), p);
      p[3] = r->w;
      p += 4;
    }

    std::unique_ptr<int[]> tmp_xy{ new int[current_num * 2] };
    p = curr_data.get();
    pf(context_->cam_ctx_->GetCameraTargetDirection(), context_->cam_ctx_->GetFov(), current_num, p, img_wid, img_hei,
       tmp_xy.get(), context_->render_ctx_->GetVisibleRange());

    for (size_t j = 0; j < current_num; j++) {
      int x = tmp_xy[j * 2 + 0];
      int y = tmp_xy[j * 2 + 1];
      if (x == std::numeric_limits<int>::min() || y == std::numeric_limits<int>::min()) {
        continue;
      }
      if (projection_type != LensType::kDualEqualArea && projection_type != LensType::kDualEquidistant) {
        x += context_->render_ctx_->GetImageOffsetX();
        y += context_->render_ctx_->GetImageOffsetY();
      }
      if (x < 0 || x >= static_cast<int>(img_wid) || y < 0 || y >= static_cast<int>(img_hei)) {
        continue;
      }
      auto tmp_val = curr_data[j * 4 + 3] * weight - current_data_compensation[y * img_wid + x];
      auto tmp_sum = current_data[y * img_wid + x] + tmp_val;
      current_data_compensation[y * img_wid + x] = tmp_sum - current_data[y * img_wid + x] - tmp_val;
      current_data[y * img_wid + x] = tmp_sum;
    }
  });
  threading_pool->WaitFinish();

  total_w_ += context_->GetInitRayNum() * weight;
}


void SpectrumRenderer::LoadData(float wl, float weight, const float* curr_data, size_t num) {
  auto projection_type = context_->cam_ctx_->GetLensType();
  auto& projection_functions = GetProjectionFunctions();
  if (projection_functions.find(projection_type) == projection_functions.end()) {
    std::fprintf(stderr, "Unknown projection type!\n");
    return;
  }
  auto& pf = projection_functions[projection_type];

  auto wavelength = static_cast<int>(wl);
  if (wavelength < SpectrumRenderer::kMinWavelength || wavelength > SpectrumRenderer::kMaxWaveLength || weight < 0) {
    std::fprintf(stderr, "Wavelength out of range!\n");
    return;
  }

  auto img_hei = context_->render_ctx_->GetImageHeight();
  auto img_wid = context_->render_ctx_->GetImageWidth();

  if (!spectrum_data_.count(wavelength) || !spectrum_data_[wavelength]) {
    spectrum_data_[wavelength].reset(new float[img_hei * img_wid]{});
    spectrum_data_compensation_[wavelength].reset(new float[img_hei * img_wid]{});
  }
  auto* current_data = spectrum_data_[wavelength].get();
  auto* current_data_compensation = spectrum_data_compensation_[wavelength].get();

  auto threading_pool = ThreadingPool::GetInstance();
  threading_pool->AddRangeBasedJobs(num, [=](size_t start_idx, size_t end_idx) {
    size_t current_num = end_idx - start_idx;
    std::unique_ptr<int[]> tmp_xy{ new int[current_num * 2] };
    pf(context_->cam_ctx_->GetCameraTargetDirection(), context_->cam_ctx_->GetFov(), current_num,
       curr_data + start_idx * 4, img_wid, img_hei, tmp_xy.get(), context_->render_ctx_->GetVisibleRange());

    for (size_t j = 0; j < current_num; j++) {
      int x = tmp_xy[j * 2 + 0];
      int y = tmp_xy[j * 2 + 1];
      if (x == std::numeric_limits<int>::min() || y == std::numeric_limits<int>::min()) {
        continue;
      }
      if (projection_type != LensType::kDualEqualArea && projection_type != LensType::kDualEquidistant) {
        x += context_->render_ctx_->GetImageOffsetX();
        y += context_->render_ctx_->GetImageOffsetY();
      }
      if (x < 0 || x >= static_cast<int>(img_wid) || y < 0 || y >= static_cast<int>(img_hei)) {
        continue;
      }
      auto tmp_val = curr_data[(start_idx + j) * 4 + 3] * weight - current_data_compensation[y * img_wid + x];
      auto tmp_sum = current_data[y * img_wid + x] + tmp_val;
      current_data_compensation[y * img_wid + x] = tmp_sum - current_data[y * img_wid + x] - tmp_val;
      current_data[y * img_wid + x] = tmp_sum;
    }
  });
  threading_pool->WaitFinish();

  total_w_ += context_->GetInitRayNum() * weight;
}


void SpectrumRenderer::ResetData() {
  total_w_ = 0;
  spectrum_data_.clear();
  spectrum_data_compensation_.clear();
}


void SpectrumRenderer::RenderToImage() {
  auto img_hei = context_->render_ctx_->GetImageHeight();
  auto img_wid = context_->render_ctx_->GetImageWidth();
  auto wl_num = spectrum_data_.size();
  std::unique_ptr<float[]> wl_data{ new float[wl_num] };
  std::unique_ptr<float[]> flat_spec_data{ new float[wl_num * img_wid * img_hei] };

  GatherSpectrumData(wl_data.get(), flat_spec_data.get());
  auto ray_color = context_->render_ctx_->GetRayColor();
  auto background_color = context_->render_ctx_->GetBackgroundColor();
  bool use_rgb = ray_color[0] < 0;

  if (use_rgb) {
    Rgb(wl_num, img_wid * img_hei, wl_data.get(), flat_spec_data.get(), output_image_buffer_.get());
  } else {
    Gray(wl_num, img_wid * img_hei, wl_data.get(), flat_spec_data.get(), output_image_buffer_.get());
  }
  for (decltype(img_wid) i = 0; i < img_wid * img_hei; i++) {
    for (int c = 0; c < 3; c++) {
      auto v = static_cast<int>(background_color[c] * kColorMaxVal);
      if (use_rgb) {
        v += output_image_buffer_[i * 3 + c];
      } else {
        v += static_cast<int>(output_image_buffer_[i * 3 + c] * 1.0 * ray_color[c]);
      }
      v = std::max(std::min(v, static_cast<int>(kColorMaxVal)), 0);
      output_image_buffer_[i * 3 + c] = static_cast<uint8_t>(v);
    }
  }

  /* Draw horizontal */
  // float imgR = std::min(img_wid_ / 2, img_hei_) / 2.0f;
  // TODO
}


uint8_t* SpectrumRenderer::GetImageBuffer() const {
  return output_image_buffer_.get();
}


int SpectrumRenderer::LoadDataFromFile(File& file) {
  auto file_size = file.GetBytes();
  std::unique_ptr<float[]> read_buffer{ new float[file_size / sizeof(float)]{} };

  file.Open(openmode::kRead | openmode::kBinary);
  auto read_count = file.Read(read_buffer.get(), 2);
  if (read_count == 0) {
    std::fprintf(stderr, "Failed to read wavelength data!\n");
    file.Close();
    return -1;
  }

  auto wavelength = static_cast<int>(read_buffer[0]);
  auto wavelength_weight = read_buffer[1];
  if (wavelength < SpectrumRenderer::kMinWavelength || wavelength > SpectrumRenderer::kMaxWaveLength ||
      wavelength_weight < 0) {
    std::fprintf(stderr, "Wavelength out of range!\n");
    file.Close();
    return -1;
  }

  read_count = file.Read(read_buffer.get(), file_size / sizeof(float));
  auto total_ray_count = read_count / 4;
  file.Close();

  if (total_ray_count == 0) {
    return 0;
  }

  LoadData(wavelength, wavelength_weight, read_buffer.get(), total_ray_count);

  return static_cast<int>(total_ray_count);
}


void SpectrumRenderer::GatherSpectrumData(float* wl_data_out, float* sp_data_out) {
  auto img_hei = context_->render_ctx_->GetImageHeight();
  auto img_wid = context_->render_ctx_->GetImageWidth();
  auto intensity_factor = static_cast<float>(context_->render_ctx_->GetIntensity());

  int k = 0;
  for (const auto& kv : spectrum_data_) {
    wl_data_out[k] = static_cast<float>(kv.first);
    std::memcpy(sp_data_out + k * img_wid * img_hei, kv.second.get(), img_wid * img_hei * sizeof(float));
    k++;
  }

  auto factor = 1e5f / total_w_ * intensity_factor;
  for (size_t i = 0; i < img_wid * img_hei * spectrum_data_.size(); i++) {
    sp_data_out[i] *= factor;
  }
}


void SpectrumRenderer::Rgb(size_t wavelength_number, size_t data_number,  //
                           const float* wavelengths,
                           const float* spec_data,  // spec_data: wavelength_number x data_number
                           uint8_t* rgb_data) {     // rgb data, data_number x 3
  for (decltype(data_number) i = 0; i < data_number; i++) {
    /* Step 1. Spectrum to XYZ */
    float xyz[3] = { 0 };
    for (decltype(wavelength_number) j = 0; j < wavelength_number; j++) {
      auto wl = static_cast<int>(wavelengths[j]);
      if (wl < kMinWavelength || wl > kMaxWaveLength) {
        continue;
      }
      float v = spec_data[j * data_number + i];
      xyz[0] += kCmfX[wl - kMinWavelength] * v;
      xyz[1] += kCmfY[wl - kMinWavelength] * v;
      xyz[2] += kCmfZ[wl - kMinWavelength] * v;
    }

    /* Step 2. XYZ to linear RGB */
    float gray[3];
    for (int j = 0; j < 3; j++) {
      gray[j] = kWhitePointD65[j] * xyz[1];
    }

    float r = 1.0f;
    for (int j = 0; j < 3; j++) {
      float a = 0, b = 0;
      for (int k = 0; k < 3; k++) {
        a += -gray[k] * kXyzToRgb[j * 3 + k];
        b += (xyz[k] - gray[k]) * kXyzToRgb[j * 3 + k];
      }
      if (a * b > 0 && a / b < r) {
        r = a / b;
      }
    }

    float rgb[3] = { 0 };
    for (int j = 0; j < 3; j++) {
      xyz[j] = (xyz[j] - gray[j]) * r + gray[j];
    }
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        rgb[j] += xyz[k] * kXyzToRgb[j * 3 + k];
      }
      rgb[j] = std::min(std::max(rgb[j], 0.0f), 1.0f);
    }

    /* Step 3. Convert linear sRGB to sRGB */
    SrgbGamma(rgb);
    for (int j = 0; j < 3; j++) {
      rgb_data[i * 3 + j] = static_cast<uint8_t>(rgb[j] * 255);
    }
  }
}


void SpectrumRenderer::Gray(size_t wavelength_number, size_t data_number,  //
                            const float* wavelengths,
                            const float* spec_data,  // spec_data: wavelength_number x data_number
                            uint8_t* rgb_data) {     // rgb data, data_number x 3
  for (decltype(data_number) i = 0; i < data_number; i++) {
    /* Step 1. Spectrum to XYZ */
    float xyz[3] = { 0 };
    for (decltype(wavelength_number) j = 0; j < wavelength_number; j++) {
      auto wl = static_cast<int>(wavelengths[j]);
      if (wl < kMinWavelength || wl > kMaxWaveLength) {
        continue;
      }
      float v = spec_data[j * data_number + i];
      xyz[0] += kCmfX[wl - kMinWavelength] * v;
      xyz[1] += kCmfY[wl - kMinWavelength] * v;
      xyz[2] += kCmfZ[wl - kMinWavelength] * v;
    }

    /* Step 2. XYZ to linear RGB */
    float gray[3];
    for (int j = 0; j < 3; j++) {
      gray[j] = kWhitePointD65[j] * xyz[1];
    }

    float rgb[3] = { 0 };
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        rgb[j] += gray[k] * kXyzToRgb[j * 3 + k];
      }
      rgb[j] = std::min(std::max(rgb[j], 0.0f), 1.0f);
    }

    /* Step 3. Convert linear sRGB to sRGB */
    SrgbGamma(rgb);
    for (int j = 0; j < 3; j++) {
      rgb_data[i * 3 + j] = static_cast<uint8_t>(rgb[j] * 255);
    }
  }
}

}  // namespace icehalo
