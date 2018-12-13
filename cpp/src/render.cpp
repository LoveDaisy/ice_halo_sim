#include "optics.h"
#include "render.h"
#include "mymath.h"

#include <limits>
#include <cstring>
#include <cmath>


namespace IceHalo {

void equiAreaFishEye(float* camRot,      // Camera rotation. [lon, lat, roll]
                     float hov,        // Half field of view.
                     uint64_t dataNumber,  // Data number
                     float* dir,       // Ray directions, [x, y, z]
                     int imgWid, int imgHei, // Image size
                     int* imgXY,       // Image coordinates
                     VisibleSemiSphere visibleSemiSphere) {
  float imgR = std::max(imgWid, imgHei) / 2.0f;
  auto* dirCopy = new float[dataNumber * 3];
  float camRotCopy[3];
  std::memcpy(dirCopy, dir, sizeof(float) * 3 * dataNumber);
  std::memcpy(camRotCopy, camRot, sizeof(float) * 3);
  camRotCopy[0] *= -1;
  camRotCopy[1] *= -1;
  for (float &i : camRotCopy) {
    i *= Math::kPi / 180.0f;
  }

  Math::rotateZ(camRotCopy, dirCopy, dataNumber);
  for (decltype(dataNumber) i = 0; i < dataNumber; i++) {
    if (std::abs(Math::norm3(dirCopy + i * 3) - 1.0) > 1e-4) {
      imgXY[i * 2 + 0] = std::numeric_limits<int>::min();
      imgXY[i * 2 + 1] = std::numeric_limits<int>::min();
    } else if (visibleSemiSphere == VisibleSemiSphere::CAMERA && dirCopy[i * 3 + 2] < 0) {
      imgXY[i * 2 + 0] = std::numeric_limits<int>::min();
      imgXY[i * 2 + 1] = std::numeric_limits<int>::min();
    } else {
      float lon = std::atan2(dirCopy[i * 3 + 1], dirCopy[i * 3 + 0]);
      float lat = std::asin(dirCopy[i * 3 + 2] / Math::norm3(dirCopy + i * 3));
      float projR = imgR / 2.0f / std::sin(hov / 2.0f / 180.0f * Math::kPi);
      float r = 2.0f * projR * std::sin((Math::kPi / 2.0f - lat) / 2.0f);

      imgXY[i * 2 + 0] = static_cast<int>(std::round(r * std::cos(lon) + imgWid / 2.0f));
      imgXY[i * 2 + 1] = static_cast<int>(std::round(r * std::sin(lon) + imgHei / 2.0f));
    }
  }

  delete[] dirCopy;
}


void dualEquiAreaFishEye(
  [[gnu::unused]] float* camRot,      // Not used
  [[gnu::unused]] float hov,        // Not used
                  uint64_t dataNumber,  // Data number
                  float* dir,       // Ray directions, [x, y, z]
                  int imgWid, int imgHei, // Image size
                  int* imgXY,       // Image coordinates
  [[gnu::unused]] VisibleSemiSphere visibleSemiSphere) {
  float imgR = std::min(imgWid / 2, imgHei) / 2.0f;
  float projR = imgR / 2.0f / std::sin(45.0f / 180.0f * Math::kPi);

  auto* dirCopy = new float[dataNumber * 3];
  float camRotCopy[3] = {90.0f, 89.999f, 0.0f};
  std::memcpy(dirCopy, dir, sizeof(float) * 3 * dataNumber);
  camRotCopy[0] *= -1;
  camRotCopy[1] *= -1;
  for (float &i : camRotCopy) {
    i *= Math::kPi / 180.0f;
  }

  Math::rotateZ(camRotCopy, dirCopy, dataNumber);
  for (decltype(dataNumber) i = 0; i < dataNumber; i++) {
    if (std::abs(Math::norm3(dirCopy + i * 3) - 1.0) > 1e-4) {
      imgXY[i * 2 + 0] = std::numeric_limits<int>::min();
      imgXY[i * 2 + 1] = std::numeric_limits<int>::min();
    } else {
      float lon = std::atan2(dirCopy[i * 3 + 1], dirCopy[i * 3 + 0]);
      float lat = std::asin(dirCopy[i * 3 + 2] / Math::norm3(dirCopy + i * 3));
      if (lat < 0) {
        lon = Math::kPi - lon;
      }
      float r = 2.0f * projR * std::sin((Math::kPi / 2.0f - std::abs(lat)) / 2.0f);

      imgXY[i * 2 + 0] = static_cast<int>(std::round(r * std::cos(lon) + imgR + (lat > 0 ? -0.5 : 2 * imgR - 0.5)));
      imgXY[i * 2 + 1] = static_cast<int>(std::round(r * std::sin(lon) + imgR - 0.5));
    }
  }

  delete[] dirCopy;
}


void dualEquiDistantFishEye(
  [[gnu::unused]] float* camRot,      // Not used
  [[gnu::unused]] float hov,        // Not used
                  uint64_t dataNumber,  // Data number
                  float* dir,       // Ray directions, [x, y, z]
                  int imgWid, int imgHei, // Image size
                  int* imgXY,       // Image coordinates
  [[gnu::unused]] VisibleSemiSphere visibleSemiSphere) {
  float imgR = std::min(imgWid / 2, imgHei) / 2.0f;

  auto* dirCopy = new float[dataNumber * 3];
  float camRotCopy[3] = {90.0f, 89.999f, 0.0f};
  std::memcpy(dirCopy, dir, sizeof(float) * 3 * dataNumber);
  camRotCopy[0] *= -1;
  camRotCopy[1] *= -1;
  for (float &i : camRotCopy) {
    i *= Math::kPi / 180.0f;
  }

  Math::rotateZ(camRotCopy, dirCopy, dataNumber);
  for (decltype(dataNumber) i = 0; i < dataNumber; i++) {
    if (std::abs(Math::norm3(dirCopy + i * 3) - 1.0) > 1e-4) {
      imgXY[i * 2 + 0] = std::numeric_limits<int>::min();
      imgXY[i * 2 + 1] = std::numeric_limits<int>::min();
    } else {
      float lon = std::atan2(dirCopy[i * 3 + 1], dirCopy[i * 3 + 0]);
      float lat = std::asin(dirCopy[i * 3 + 2] / Math::norm3(dirCopy + i * 3));
      if (lat < 0) {
        lon = Math::kPi - lon;
      }
      float r = (1.0f - std::abs(lat) * 2.0f / Math::kPi) * imgR;

      imgXY[i * 2 + 0] = static_cast<int>(std::round(r * std::cos(lon) + imgR + (lat > 0 ? -0.5 : 2 * imgR - 0.5)));
      imgXY[i * 2 + 1] = static_cast<int>(std::round(r * std::sin(lon) + imgR - 0.5));
    }
  }

  delete[] dirCopy;
}


void rectLinear(float* camRot,      // Camera rotation. [lon, lat, roll]
                float hov,        // Half field of view.
                uint64_t dataNumber,  // Data number
                float* dir,       // Ray directions, [x, y, z]
                int imgWid, int imgHei, // Image size
                int* imgXY,       // Image coordinates
[[gnu::unused]] VisibleSemiSphere visibleSemiSphere) {
  auto* dirCopy = new float[dataNumber * 3];
  float camRotCopy[3];
  std::memcpy(dirCopy, dir, sizeof(float) * 3 * dataNumber);
  std::memcpy(camRotCopy, camRot, sizeof(float) * 3);
  camRotCopy[0] *= -1;
  camRotCopy[1] *= -1;
  for (float &i : camRotCopy) {
    i *= Math::kPi / 180.0f;
  }

  Math::rotateZ(camRotCopy, dirCopy, dataNumber);
  for (decltype(dataNumber) i = 0; i < dataNumber; i++) {
    if (dirCopy[i * 3 + 2] < 0 || std::abs(Math::norm3(dirCopy + i * 3) - 1.0) > 1e-4) {
      imgXY[i * 2 + 0] = std::numeric_limits<int>::min();
      imgXY[i * 2 + 1] = std::numeric_limits<int>::min();
    } else {
      double x = dirCopy[i * 3 + 0] / dirCopy[i * 3 + 2];
      double y = dirCopy[i * 3 + 1] / dirCopy[i * 3 + 2];
      x = x * imgWid / 2 / std::tan(hov * Math::kPi / 180.0f) + imgWid / 2.0f;
      y = y * imgWid / 2 / std::tan(hov * Math::kPi / 180.0f) + imgHei / 2.0f;

      imgXY[i * 2 + 0] = static_cast<int>(std::round(x));
      imgXY[i * 2 + 1] = static_cast<int>(std::round(y));
    }
  }

  delete[] dirCopy;
}


constexpr int SpectrumRenderer::MIN_WL;
constexpr int SpectrumRenderer::MAX_WL;
constexpr float SpectrumRenderer::_W[];
constexpr float SpectrumRenderer::_mt[];
constexpr float SpectrumRenderer::_cmf_x[];
constexpr float SpectrumRenderer::_cmf_y[];
constexpr float SpectrumRenderer::_cmf_z[];

void SpectrumRenderer::rgb(int waveLengthNumber, float* waveLengths,
                           int dataNumber, float* specData, uint8_t* rgbData) {
  for (int i = 0; i < dataNumber; i++) {
    /* Step 1. Spectrum to XYZ */
    float xyz[3] = { 0 };
    for (int j = 0; j < waveLengthNumber; j++) {
      auto wl = static_cast<int>(waveLengths[j]);
      float v = wl >= MIN_WL && wl <= MAX_WL ? specData[j*dataNumber + i] : 0.0f;
      xyz[0] += _cmf_x[wl - MIN_WL] * v;
      xyz[1] += _cmf_y[wl - MIN_WL] * v;
      xyz[2] += _cmf_z[wl - MIN_WL] * v;
    }

    /* Step 2. XYZ to linear RGB */
    float gray[3];
    for (int j = 0; j < 3; j++) {
      gray[j] = _W[j] * xyz[1];
    }

    float r = 1.0f;
    for (int j = 0; j < 3; j++) {
      float a = 0, b = 0;
      for (int k = 0; k < 3; k++) {
        a += -gray[k] * _mt[j*3 + k];
        b += (xyz[k] - gray[k]) * _mt[j*3 + k];
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
        rgb[j] += xyz[k] * _mt[j*3 + k];
      }
      rgb[j] = fmin(fmax(rgb[j], 0.0f), 1.0f);
    }

    /* Step 3. Convert linear sRGB to sRGB */
    for (int j = 0; j < 3; j++) {
      if (rgb[j] < 0.0031308) {
        rgb[j] *= 12.92f;
      } else {
        rgb[j] = static_cast<float>(1.055 * pow(rgb[j], 1.0/2.4) - 0.055);
      }
      rgbData[i * 3 + j] = static_cast<uint8_t>(rgb[j] * 255);
    }
  }
}


void SpectrumRenderer::gray(int waveLengthNumber, float* waveLengths,
                            int dataNumber, float* specData, uint8_t* rgbData) {
  for (int i = 0; i < dataNumber; i++) {
    /* Step 1. Spectrum to XYZ */
    float xyz[3] = { 0 };
    for (int j = 0; j < waveLengthNumber; j++) {
      auto wl = static_cast<int>(waveLengths[j]);
      float v = wl >= MIN_WL && wl <= MAX_WL ? specData[j*dataNumber + i] : 0.0f;
      xyz[0] += _cmf_x[wl - MIN_WL] * v;
      xyz[1] += _cmf_y[wl - MIN_WL] * v;
      xyz[2] += _cmf_z[wl - MIN_WL] * v;
    }

    /* Step 2. XYZ to linear RGB */
    float gray[3];
    for (int j = 0; j < 3; j++) {
      gray[j] = _W[j] * xyz[1];
    }

    float rgb[3] = { 0 };
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        rgb[j] += gray[k] * _mt[j*3 + k];
      }
      rgb[j] = fmin(fmax(rgb[j], 0.0f), 1.0f);
    }

    /* Step 3. Convert linear sRGB to sRGB */
    for (int j = 0; j < 3; j++) {
      if (rgb[j] < 0.0031308) {
        rgb[j] *= 12.92f;
      } else {
        rgb[j] = static_cast<float>(1.055 * pow(rgb[j], 1.0/2.4) - 0.055);
      }
      rgbData[i * 3 + j] = static_cast<uint8_t>(rgb[j] * 255);
    }
  }
}

}   // namespace IceHalo
