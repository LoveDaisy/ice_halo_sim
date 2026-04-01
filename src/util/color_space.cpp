#include "util/color_space.hpp"

#include <algorithm>
#include <cmath>

#include "util/color_data.hpp"

namespace lumice {

void GamutClipXyz(const float xyz[3], float clipped[3]) {
  float gray[3];
  for (int j = 0; j < 3; j++) {
    gray[j] = kWhitePointD65[j] * xyz[1];
  }

  float s = 1.0f;
  float diff[3];
  for (int j = 0; j < 3; j++) {
    diff[j] = xyz[j] - gray[j];
  }
  for (int j = 0; j < 3; j++) {
    float a = 0;
    float b = 0;
    for (int k = 0; k < 3; k++) {
      a += -gray[k] * kXyzToRgb[j * 3 + k];
      b += diff[k] * kXyzToRgb[j * 3 + k];
    }
    if (a * b > 0 && a / b < s) {
      s = a / b;
    }
  }
  for (int j = 0; j < 3; j++) {
    clipped[j] = diff[j] * s + gray[j];
  }
}

void XyzToLinearRgb(const float xyz[3], float rgb[3]) {
  for (int j = 0; j < 3; j++) {
    float v = 0;
    for (int k = 0; k < 3; k++) {
      v += xyz[k] * kXyzToRgb[j * 3 + k];
    }
    rgb[j] = std::clamp(v, 0.0f, 1.0f);
  }
}

float LinearToSrgb(float linear) {
  if (linear < 0.0031308f) {
    return linear * 12.92f;
  }
  return 1.055f * std::pow(linear, 1.0f / 2.4f) - 0.055f;
}

void LinearToSrgbBatch(float* rgb, int channel_count) {
  for (int i = 0; i < channel_count; i++) {
    rgb[i] = LinearToSrgb(rgb[i]);
  }
}

void XyzToSrgb(const float xyz[3], float rgb[3]) {
  float clipped[3];
  GamutClipXyz(xyz, clipped);
  XyzToLinearRgb(clipped, rgb);
  for (int j = 0; j < 3; j++) {
    rgb[j] = LinearToSrgb(rgb[j]);
  }
}

void XyzToSrgbUint8(const float* xyz_in, unsigned char* out, int pixel_count) {
  for (int i = 0; i < pixel_count; i++) {
    float rgb[3];
    XyzToSrgb(xyz_in + i * 3, rgb);
    for (int j = 0; j < 3; j++) {
      out[i * 3 + j] = static_cast<unsigned char>(std::clamp(rgb[j], 0.0f, 1.0f) * 255.0f);
    }
  }
}

void XyzToSrgbUint8(const float* xyz_in, unsigned char* out, int pixel_count, float intensity_scale) {
  for (int i = 0; i < pixel_count; i++) {
    float xyz[3];
    for (int j = 0; j < 3; j++) {
      xyz[j] = xyz_in[i * 3 + j] * intensity_scale;
    }
    float rgb[3];
    XyzToSrgb(xyz, rgb);
    for (int j = 0; j < 3; j++) {
      out[i * 3 + j] = static_cast<unsigned char>(std::clamp(rgb[j], 0.0f, 1.0f) * 255.0f);
    }
  }
}

}  // namespace lumice
