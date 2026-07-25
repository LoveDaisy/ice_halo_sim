#include "test_screenshot.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

#include "gui/gl_common.h"

// Only include stb headers for declarations — implementations are provided by lumice_gui_obj (stb_impl.cpp).
#include <stb_image.h>
#include <stb_image_write.h>

namespace lumice::test {

std::vector<unsigned char> ReadTexturePixels(GLuint tex_id, int w, int h) {
  if (w <= 0 || h <= 0 || tex_id == 0) {
    return {};
  }

  // glGetTexImage is not available in OpenGL 3.2 Core Profile on macOS.
  // Use a temporary FBO + glReadPixels instead.
  GLint prev_fbo = 0;
  glGetIntegerv(GL_FRAMEBUFFER_BINDING, &prev_fbo);

  GLuint tmp_fbo = 0;
  glGenFramebuffers(1, &tmp_fbo);
  glBindFramebuffer(GL_FRAMEBUFFER, tmp_fbo);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tex_id, 0);

  std::vector<unsigned char> pixels(static_cast<size_t>(w) * h * 4);
  glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());

  // Cleanup temp FBO and restore previous binding
  glBindFramebuffer(GL_FRAMEBUFFER, static_cast<GLuint>(prev_fbo));
  glDeleteFramebuffers(1, &tmp_fbo);

  // Flip Y axis: OpenGL textures have origin at bottom-left, images at top-left
  const size_t row_bytes = static_cast<size_t>(w) * 4;
  std::vector<unsigned char> row_buf(row_bytes);
  for (int y = 0; y < h / 2; ++y) {
    unsigned char* top = pixels.data() + static_cast<size_t>(y) * row_bytes;
    unsigned char* bot = pixels.data() + static_cast<size_t>(h - 1 - y) * row_bytes;
    std::copy(top, top + row_bytes, row_buf.data());
    std::copy(bot, bot + row_bytes, top);
    std::copy(row_buf.data(), row_buf.data() + row_bytes, bot);
  }

  return pixels;
}

bool SavePng(const char* path, const unsigned char* data, int w, int h, int channels) {
  if (!path || !data || w <= 0 || h <= 0 || channels <= 0) {
    return false;
  }
  return stbi_write_png(path, w, h, channels, data, w * channels) != 0;
}

bool LoadPng(const char* path, std::vector<unsigned char>& data, int& w, int& h, int& channels) {
  if (!path) {
    return false;
  }
  unsigned char* raw = stbi_load(path, &w, &h, &channels, 0);
  if (!raw) {
    return false;
  }
  data.assign(raw, raw + static_cast<size_t>(w) * h * channels);
  stbi_image_free(raw);
  return true;
}

double ComputePsnr(const unsigned char* img1, const unsigned char* img2, int w, int h, int channels) {
  if (!img1 || !img2 || w <= 0 || h <= 0 || channels <= 0) {
    return -1.0;
  }

  const size_t total = static_cast<size_t>(w) * h * channels;
  double mse = 0.0;
  for (size_t i = 0; i < total; ++i) {
    double diff = static_cast<double>(img1[i]) - static_cast<double>(img2[i]);
    mse += diff * diff;
  }
  mse /= static_cast<double>(total);

  if (mse == 0.0) {
    return std::numeric_limits<double>::infinity();
  }

  return 10.0 * std::log10(255.0 * 255.0 / mse);
}

std::vector<unsigned char> StripAlpha(const unsigned char* rgba, int width, int height) {
  if (!rgba || width <= 0 || height <= 0) {
    return {};
  }
  const size_t kPixelCount = static_cast<size_t>(width) * height;
  std::vector<unsigned char> rgb(kPixelCount * 3);
  for (size_t i = 0; i < kPixelCount; ++i) {
    rgb[i * 3 + 0] = rgba[i * 4 + 0];
    rgb[i * 3 + 1] = rgba[i * 4 + 1];
    rgb[i * 3 + 2] = rgba[i * 4 + 2];
  }
  return rgb;
}

bool CheckAgainstReference(const char* group, const char* tag, const std::string& tmp_path, const std::string& ref_path,
                           double threshold, bool keep_capture_png) {
  std::vector<unsigned char> ref_data;
  int ref_w = 0;
  int ref_h = 0;
  int ref_ch = 0;
  bool loaded = LoadPng(ref_path.c_str(), ref_data, ref_w, ref_h, ref_ch);
  if (!loaded) {
    fprintf(stderr, "[%s] %s: reference not found at %s\n", group, tag, ref_path.c_str());
    fprintf(stderr, "[%s] %s: Copy %s to %s\n", group, tag, tmp_path.c_str(), ref_path.c_str());
    return false;
  }

  std::vector<unsigned char> cap_data;
  int cap_w = 0;
  int cap_h = 0;
  int cap_ch = 0;
  if (!LoadPng(tmp_path.c_str(), cap_data, cap_w, cap_h, cap_ch)) {
    fprintf(stderr, "[%s] %s: failed to load capture from %s\n", group, tag, tmp_path.c_str());
    return false;
  }

  if (cap_w != ref_w || cap_h != ref_h) {
    fprintf(stderr, "[%s] %s: size mismatch cap=%dx%dx%d ref=%dx%dx%d\n", group, tag, cap_w, cap_h, cap_ch, ref_w,
            ref_h, ref_ch);
    return false;
  }

  // When ref is RGB (e.g. JPEG) and capture is RGBA, strip alpha before comparison.
  const unsigned char* cmp_ptr = cap_data.data();
  std::vector<unsigned char> converted;
  int cmp_channels = cap_ch;
  if (ref_ch == 3 && cap_ch == 4) {
    converted = StripAlpha(cap_data.data(), cap_w, cap_h);
    cmp_ptr = converted.data();
    cmp_channels = 3;
  } else if (ref_ch != cap_ch) {
    fprintf(stderr, "[%s] %s: channel mismatch cap=%d ref=%d\n", group, tag, cap_ch, ref_ch);
    return false;
  }

  double psnr = ComputePsnr(cmp_ptr, ref_data.data(), ref_w, ref_h, cmp_channels);
  fprintf(stderr, "[%s] %s: PSNR=%.2f dB (threshold=%.1f dB)\n", group, tag, psnr, threshold);
  if (psnr < threshold) {
    fprintf(stderr, "[%s] %s: PSNR below threshold — possible regression\n", group, tag);
    return false;
  }
  // Callers pass the binary's --keep-export-png flag here; scripts/regen_gui_test_refs.py
  // relies on it to collect the per-run PNGs it pixel-averages into a mean reference.
  if (!keep_capture_png) {
    std::remove(tmp_path.c_str());
  }
  return true;
}

}  // namespace lumice::test
