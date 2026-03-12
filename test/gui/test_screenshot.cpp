#include "test_screenshot.hpp"

#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION
#endif
#include <OpenGL/gl3.h>

#include <algorithm>
#include <cmath>
#include <limits>

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

}  // namespace lumice::test
