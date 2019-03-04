#include "optics.h"
#include "render.h"
#include "mymath.h"
#include "context.h"

#include <limits>
#include <cstring>
#include <cmath>


namespace IceHalo {

void EqualAreaFishEye(const float* cam_rot,      // Camera rotation. [lon, lat, roll]
                      float hov,                 // Half field of view.
                      uint64_t data_number,      // Data number
                      const float* dir,          // Ray directions, [x, y, z]
                      int img_wid, int img_hei,  // Image size
                      int* img_xy,               // Image coordinates
                      VisibleSemiSphere visible_semi_sphere) {
  float img_r = std::max(img_wid, img_hei) / 2.0f;
  auto* dir_copy = new float[data_number * 3];
  float cam_rot_copy[3];
  std::memcpy(cam_rot_copy, cam_rot, sizeof(float) * 3);
  cam_rot_copy[0] *= -1;
  cam_rot_copy[1] *= -1;
  for (float &i : cam_rot_copy) {
    i *= Math::kPi / 180.0f;
  }

  Math::RotateZ(cam_rot_copy, dir, dir_copy, data_number);
  for (decltype(data_number) i = 0; i < data_number; i++) {
    if (std::abs(Math::Norm3(dir_copy + i * 3) - 1.0) > 1e-4) {
      img_xy[i * 2 + 0] = std::numeric_limits<int>::min();
      img_xy[i * 2 + 1] = std::numeric_limits<int>::min();
    } else if (visible_semi_sphere == VisibleSemiSphere::kCamera && dir_copy[i * 3 + 2] < 0) {
      img_xy[i * 2 + 0] = std::numeric_limits<int>::min();
      img_xy[i * 2 + 1] = std::numeric_limits<int>::min();
    } else if (visible_semi_sphere == VisibleSemiSphere::kUpper && dir[i * 3 + 2] > 0) {
      img_xy[i * 2 + 0] = std::numeric_limits<int>::min();
      img_xy[i * 2 + 1] = std::numeric_limits<int>::min();
    } else if (visible_semi_sphere == VisibleSemiSphere::kLower && dir[i * 3 + 2] < 0) {
      img_xy[i * 2 + 0] = std::numeric_limits<int>::min();
      img_xy[i * 2 + 1] = std::numeric_limits<int>::min();
    } else {
      float lon = std::atan2(dir_copy[i * 3 + 1], dir_copy[i * 3 + 0]);
      float lat = std::asin(dir_copy[i * 3 + 2] / Math::Norm3(dir_copy + i * 3));
      float proj_r = img_r / 2.0f / std::sin(hov / 2.0f / 180.0f * Math::kPi);
      float r = 2.0f * proj_r * std::sin((Math::kPi / 2.0f - lat) / 2.0f);

      img_xy[i * 2 + 0] = static_cast<int>(std::round(r * std::cos(lon) + img_wid / 2.0f));
      img_xy[i * 2 + 1] = static_cast<int>(std::round(r * std::sin(lon) + img_hei / 2.0f));
    }
  }

  delete[] dir_copy;
}


void DualEqualAreaFishEye(const float* /* cam_rot */,     // Not used
                          float  /* hov */,               // Not used
                          uint64_t data_number,           // Data number
                          const float* dir,               // Ray directions, [x, y, z]
                          int img_wid, int img_hei,       // Image size
                          int* img_xy,                    // Image coordinates
                          VisibleSemiSphere /* visible_semi_sphere */) {
  float img_r = std::min(img_wid / 2, img_hei) / 2.0f;
  float proj_r = img_r / 2.0f / std::sin(45.0f / 180.0f * Math::kPi);

  auto* dir_copy = new float[data_number * 3];
  float cam_rot_copy[3] = {90.0f, 89.999f, 0.0f};
  cam_rot_copy[0] *= -1;
  cam_rot_copy[1] *= -1;
  for (float &i : cam_rot_copy) {
    i *= Math::kDegreeToRad;
  }

  Math::RotateZ(cam_rot_copy, dir, dir_copy, data_number);
  for (decltype(data_number) i = 0; i < data_number; i++) {
    if (std::abs(Math::Norm3(dir_copy + i * 3) - 1.0) > 1e-4) {
      img_xy[i * 2 + 0] = std::numeric_limits<int>::min();
      img_xy[i * 2 + 1] = std::numeric_limits<int>::min();
    } else {
      float lon = std::atan2(dir_copy[i * 3 + 1], dir_copy[i * 3 + 0]);
      float lat = std::asin(dir_copy[i * 3 + 2] / Math::Norm3(dir_copy + i * 3));
      if (lat < 0) {
        lon = Math::kPi - lon;
      }
      float r = 2.0f * proj_r * std::sin((Math::kPi / 2.0f - std::abs(lat)) / 2.0f);

      img_xy[i * 2 + 0] = static_cast<int>(std::round(r * std::cos(lon) + img_r + (lat > 0 ? -0.5 : 2 * img_r - 0.5)));
      img_xy[i * 2 + 1] = static_cast<int>(std::round(r * std::sin(lon) + img_r - 0.5));
    }
  }

  delete[] dir_copy;
}


void DualEquidistantFishEye(const float* /* cam_rot */,     // Not used
                            float  /* hov */,               // Not used
                            uint64_t data_number,           // Data number
                            const float* dir,               // Ray directions, [x, y, z]
                            int img_wid, int img_hei,       // Image size
                            int* img_xy,                    // Image coordinates
                            VisibleSemiSphere /* visible_semi_sphere */) {
  float img_r = std::min(img_wid / 2, img_hei) / 2.0f;

  auto* dir_copy = new float[data_number * 3];
  float cam_rot_copy[3] = {90.0f, 89.999f, 0.0f};
  cam_rot_copy[0] *= -1;
  cam_rot_copy[1] *= -1;
  for (float &i : cam_rot_copy) {
    i *= Math::kDegreeToRad;
  }

  Math::RotateZ(cam_rot_copy, dir, dir_copy, data_number);
  for (decltype(data_number) i = 0; i < data_number; i++) {
    if (std::abs(Math::Norm3(dir_copy + i * 3) - 1.0) > 1e-4) {
      img_xy[i * 2 + 0] = std::numeric_limits<int>::min();
      img_xy[i * 2 + 1] = std::numeric_limits<int>::min();
    } else {
      float lon = std::atan2(dir_copy[i * 3 + 1], dir_copy[i * 3 + 0]);
      float lat = std::asin(dir_copy[i * 3 + 2] / Math::Norm3(dir_copy + i * 3));
      if (lat < 0) {
        lon = Math::kPi - lon;
      }
      float r = (1.0f - std::abs(lat) * 2.0f / Math::kPi) * img_r;

      img_xy[i * 2 + 0] = static_cast<int>(std::round(r * std::cos(lon) + img_r + (lat > 0 ? -0.5 : 2 * img_r - 0.5)));
      img_xy[i * 2 + 1] = static_cast<int>(std::round(r * std::sin(lon) + img_r - 0.5));
    }
  }

  delete[] dir_copy;
}


void RectLinear(const float* cam_rot,      // Camera rotation. [lon, lat, roll]
                float hov,                 // Half field of view.
                uint64_t data_number,      // Data number
                const float* dir,          // Ray directions, [x, y, z]
                int img_wid, int img_hei,  // Image size
                int* img_xy,               // Image coordinates
                VisibleSemiSphere visible_semi_sphere) {
  auto* dir_copy = new float[data_number * 3];
  float cam_rot_copy[3];
  std::memcpy(cam_rot_copy, cam_rot, sizeof(float) * 3);
  cam_rot_copy[0] *= -1;
  cam_rot_copy[1] *= -1;
  for (float &i : cam_rot_copy) {
    i *= Math::kDegreeToRad;
  }

  Math::RotateZ(cam_rot_copy, dir, dir_copy, data_number);
  for (decltype(data_number) i = 0; i < data_number; i++) {
    if (dir_copy[i * 3 + 2] < 0 || std::abs(Math::Norm3(dir_copy + i * 3) - 1.0) > 1e-4) {
      img_xy[i * 2 + 0] = std::numeric_limits<int>::min();
      img_xy[i * 2 + 1] = std::numeric_limits<int>::min();
    } else if (visible_semi_sphere == VisibleSemiSphere::kCamera && dir_copy[i * 3 + 2] < 0) {
      img_xy[i * 2 + 0] = std::numeric_limits<int>::min();
      img_xy[i * 2 + 1] = std::numeric_limits<int>::min();
    } else if (visible_semi_sphere == VisibleSemiSphere::kUpper && dir[i * 3 + 2] > 0) {
      img_xy[i * 2 + 0] = std::numeric_limits<int>::min();
      img_xy[i * 2 + 1] = std::numeric_limits<int>::min();
    } else if (visible_semi_sphere == VisibleSemiSphere::kLower && dir[i * 3 + 2] < 0) {
      img_xy[i * 2 + 0] = std::numeric_limits<int>::min();
      img_xy[i * 2 + 1] = std::numeric_limits<int>::min();
    } else {
      double x = dir_copy[i * 3 + 0] / dir_copy[i * 3 + 2];
      double y = dir_copy[i * 3 + 1] / dir_copy[i * 3 + 2];
      x = x * img_wid / 2 / std::tan(hov * Math::kPi / 180.0f) + img_wid / 2.0f;
      y = y * img_wid / 2 / std::tan(hov * Math::kPi / 180.0f) + img_hei / 2.0f;

      img_xy[i * 2 + 0] = static_cast<int>(std::round(x));
      img_xy[i * 2 + 1] = static_cast<int>(std::round(y));
    }
  }

  delete[] dir_copy;
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


constexpr int SpectrumRenderer::kMinWavelength;
constexpr int SpectrumRenderer::kMaxWaveLength;
constexpr uint8_t SpectrumRenderer::kColorMaxVal;
constexpr float SpectrumRenderer::kWhitePointD65[];
constexpr float SpectrumRenderer::kXyzToRgb[];
constexpr float SpectrumRenderer::kCmfX[];
constexpr float SpectrumRenderer::kCmfY[];
constexpr float SpectrumRenderer::kCmfZ[];


SpectrumRenderer::SpectrumRenderer(const IceHalo::RenderContextPtr& context)
  : context_(context), total_w_(0) {}


SpectrumRenderer::~SpectrumRenderer() {
  ResetData();
}


void SpectrumRenderer::LoadData() {
  std::vector<File> files = ListDataFiles(context_->GetDataDirectory().c_str());
  int i = 0;
  for (auto& f : files) {
    auto t0 = std::chrono::system_clock::now();
    auto num = LoadDataFromFile(f);
    auto t1 = std::chrono::system_clock::now();
    std::chrono::duration<float, std::ratio<1, 1000> > diff = t1 - t0;
    std::printf(" Loading data (%d/%zu): %.2fms; total %d pts\n", i + 1, files.size(), diff.count(), num);
    i++;
  }
}


void SpectrumRenderer::ResetData() {
  total_w_ = 0;
  for (const auto& kv : spectrum_data_) {
    delete[] kv.second;
  }
  for (const auto& kv : spectrum_data_compensation_) {
    delete[] kv.second;
  }
  spectrum_data_.clear();
  spectrum_data_compensation_.clear();
}


void SpectrumRenderer::RenderToRgb(uint8_t* rgb_data) {
  auto img_hei = context_->GetImageHeight();
  auto img_wid = context_->GetImageWidth();
  auto wl_num = spectrum_data_.size();
  auto* wl_data = new float[wl_num];
  auto* flat_spec_data = new float[wl_num * img_wid * img_hei];

  GatherSpectrumData(wl_data, flat_spec_data);
  auto ray_color = context_->GetRayColor();
  auto background_color = context_->GetBackgroundColor();
  bool use_rgb = ray_color[0] < 0;

  if (use_rgb) {
    Rgb(wl_num, img_wid * img_hei, wl_data, flat_spec_data, rgb_data);
  } else {
    Gray(wl_num, img_wid * img_hei, wl_data, flat_spec_data, rgb_data);
  }
  for (decltype(img_wid) i = 0; i < img_wid * img_hei; i++) {
    for (int c = 0; c < 3; c++) {
      auto v = static_cast<int>(background_color[c] * kColorMaxVal);
      if (use_rgb) {
        v += rgb_data[i * 3 + c];
      } else {
        v += static_cast<int>(rgb_data[i * 3 + c] * ray_color[c]);
      }
      v = std::max(std::min(v, static_cast<int>(kColorMaxVal)), 0);
      rgb_data[i * 3 + c] = static_cast<uint8_t>(v);
    }
  }

  /* Draw horizontal */
  // float imgR = std::min(img_wid_ / 2, img_hei_) / 2.0f;
  // TODO

  delete[] wl_data;
  delete[] flat_spec_data;
}


int SpectrumRenderer::LoadDataFromFile(IceHalo::File& file) {
  auto projection_type = context_->GetProjectionType();
  if (projection_functions.find(projection_type) == projection_functions.end()) {
    std::fprintf(stderr, "Unknown projection type!\n");
    return -1;
  }

  auto file_size = file.GetSize();
  auto* read_buffer = new float[file_size / sizeof(float)];

  file.Open(OpenMode::kRead | OpenMode::kBinary);
  auto read_count = file.Read(read_buffer, 2);
  if (read_count <= 0) {
    std::fprintf(stderr, "Failed to read wavelength data!\n");
    file.Close();
    delete[] read_buffer;
    return -1;
  }

  auto wavelength = static_cast<int>(read_buffer[0]);
  auto wavelength_weight = read_buffer[1];
  if (wavelength < SpectrumRenderer::kMinWavelength ||
      wavelength > SpectrumRenderer::kMaxWaveLength ||
      wavelength_weight < 0) {
    std::fprintf(stderr, "Wavelength out of range!\n");
    file.Close();
    delete[] read_buffer;
    return -1;
  }

  read_count = file.Read(read_buffer, file_size / sizeof(float));
  auto total_ray_count = read_count / 4;
  file.Close();

  if (total_ray_count == 0) {
    delete[] read_buffer;
    return 0;
  }

  auto* tmp_dir = new float[total_ray_count * 3];
  auto* tmp_w = new float[total_ray_count];
  auto* tmp_xy = new int[total_ray_count * 2];
  for (decltype(read_count) i = 0; i < total_ray_count; i++) {
    std::memcpy(tmp_dir + i * 3, read_buffer + i * 4, 3 * sizeof(float));
    tmp_w[i] = read_buffer[i * 4 + 3] * wavelength_weight;
  }
  delete[] read_buffer;

  auto img_hei = context_->GetImageHeight();
  auto img_wid = context_->GetImageWidth();
  projection_functions[projection_type](
    context_->GetCamRot(), context_->GetFov(), total_ray_count, tmp_dir,
    img_wid, img_hei, tmp_xy, context_->GetVisibleSemiSphere());
  delete[] tmp_dir;

  float* current_data = nullptr;
  float* current_data_compensation = nullptr;
  auto it = spectrum_data_.find(wavelength);
  if (it != spectrum_data_.end()) {
    current_data = it->second;
    current_data_compensation = spectrum_data_compensation_[wavelength];
  } else {
    current_data = new float[img_hei * img_wid];
    current_data_compensation = new float[img_hei * img_wid];
    for (decltype(img_hei) i = 0; i < img_hei * img_wid; i++) {
      current_data[i] = 0;
      current_data_compensation[i] = 0;
    }
    spectrum_data_[wavelength] = current_data;
    spectrum_data_compensation_[wavelength] = current_data_compensation;
  }

  for (decltype(total_ray_count) i = 0; i < total_ray_count; i++) {
    int x = tmp_xy[i * 2 + 0];
    int y = tmp_xy[i * 2 + 1];
    if (x == std::numeric_limits<int>::min() || y == std::numeric_limits<int>::min()) {
      continue;
    }
    if (projection_type != ProjectionType::kDualEqualArea && projection_type != ProjectionType::kDualEquidistant) {
      x += context_->GetOffsetX();
      y += context_->GetOffsetY();
    }
    if (x < 0 || x >= static_cast<int>(img_wid) || y < 0 || y >= static_cast<int>(img_hei)) {
      continue;
    }
    auto tmp_val = tmp_w[i] - current_data_compensation[y * img_wid + x];
    auto tmp_sum = current_data[y * img_wid + x] + tmp_val;
    current_data_compensation[y * img_wid + x] = tmp_sum - current_data[y * img_wid + x] - tmp_val;
    current_data[y * img_wid + x] = tmp_sum;
  }
  delete[] tmp_xy;
  delete[] tmp_w;

  total_w_ += context_->GetTotalRayNum() * wavelength_weight;
  return static_cast<int>(total_ray_count);
}


void SpectrumRenderer::GatherSpectrumData(float* wl_data_out, float* sp_data_out) {
  auto img_hei = context_->GetImageHeight();
  auto img_wid = context_->GetImageWidth();
  auto intensity_factor = context_->GetIntensityFactor();

  int k = 0;
  for (const auto& kv : spectrum_data_) {
    wl_data_out[k] = kv.first;
    std::memcpy(sp_data_out + k * img_wid * img_hei, kv.second, img_wid * img_hei * sizeof(float));
    k++;
  }
  for (decltype(spectrum_data_.size()) i = 0; i < img_wid * img_hei * spectrum_data_.size(); i++) {
    sp_data_out[i] *= 1e5 / total_w_ * intensity_factor;
  }
}


void SpectrumRenderer::Rgb(size_t wavelength_number, size_t data_number,
                           const float* wavelengths, const float* spec_data,
                           uint8_t* rgb_data) {
  for (decltype(data_number) i = 0; i < data_number; i++) {
    /* Step 1. Spectrum to XYZ */
    float xyz[3] = { 0 };
    for (decltype(wavelength_number) j = 0; j < wavelength_number; j++) {
      auto wl = static_cast<int>(wavelengths[j]);
      float v = wl >= kMinWavelength && wl <= kMaxWaveLength ? spec_data[j*data_number + i] : 0.0f;
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
        a += -gray[k] * kXyzToRgb[j*3 + k];
        b += (xyz[k] - gray[k]) * kXyzToRgb[j*3 + k];
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
        rgb[j] += xyz[k] * kXyzToRgb[j*3 + k];
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


void SpectrumRenderer::Gray(size_t wavelength_number, size_t data_number,
                            const float* wavelengths, const float* spec_data,
                            uint8_t* rgb_data) {
  for (decltype(data_number) i = 0; i < data_number; i++) {
    /* Step 1. Spectrum to XYZ */
    float xyz[3] = { 0 };
    for (decltype(wavelength_number) j = 0; j < wavelength_number; j++) {
      auto wl = static_cast<int>(wavelengths[j]);
      float v = wl >= kMinWavelength && wl <= kMaxWaveLength ? spec_data[j*data_number + i] : 0.0f;
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
        rgb[j] += gray[k] * kXyzToRgb[j*3 + k];
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

}   // namespace IceHalo
