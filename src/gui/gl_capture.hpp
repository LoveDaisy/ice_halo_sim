#ifndef LUMICE_GUI_GL_CAPTURE_HPP
#define LUMICE_GUI_GL_CAPTURE_HPP

#include <cstring>
#include <vector>

#include "gui/gl_common.h"

namespace lumice::gui {

// Read back an (x,y,w,h) region from the *currently bound* framebuffer as
// RGBA8, flipping Y from OpenGL bottom-up to top-down (PNG-friendly).
//
// Contracts (ALL required):
//   - Must be called from the thread that owns the GL context
//     (i.e., the main render thread).
//   - Caller is responsible for binding the target FBO before this call.
//     When capturing the default framebuffer (ImGui content), must be
//     called AFTER ImGui::Render() and BEFORE glfwSwapBuffers(), otherwise
//     readback will silently return a previous frame or partial content.
//   - out_rgba_top_down is resized to w*h*4 on success.
//
// Returns false if (w,h) is non-positive.
inline bool ReadbackGlRegionToRgba(int x, int y, int w, int h, std::vector<unsigned char>& out_rgba_top_down) {
  if (w <= 0 || h <= 0) {
    return false;
  }
  const size_t kRowBytes = static_cast<size_t>(w) * 4;
  out_rgba_top_down.assign(static_cast<size_t>(h) * kRowBytes, 0);
  glReadPixels(x, y, w, h, GL_RGBA, GL_UNSIGNED_BYTE, out_rgba_top_down.data());

  std::vector<unsigned char> row(kRowBytes);
  for (int i = 0; i < h / 2; ++i) {
    unsigned char* top = out_rgba_top_down.data() + static_cast<size_t>(i) * kRowBytes;
    unsigned char* bottom = out_rgba_top_down.data() + static_cast<size_t>(h - 1 - i) * kRowBytes;
    std::memcpy(row.data(), top, kRowBytes);
    std::memcpy(top, bottom, kRowBytes);
    std::memcpy(bottom, row.data(), kRowBytes);
  }
  return true;
}

}  // namespace lumice::gui

#endif  // LUMICE_GUI_GL_CAPTURE_HPP
