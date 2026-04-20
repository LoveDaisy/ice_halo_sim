#include "gui/export_fbo_renderer.hpp"

#include "gui/gl_capture.hpp"
#include "gui/gl_common.h"
#include "imgui.h"
#include "imgui_impl_opengl3.h"

namespace lumice::gui {
namespace {

// Single choke-point for ImGui internal API use. If ImGui is upgraded, grep this
// symbol and re-verify against the new ImDrawList API. Verified for v1.91.8-docking
// (cache commit f6a6076), spike contract test: export/spike_fbo_imdrawlist_end_to_end.
inline void ResetDrawListForNewFrame(ImDrawList& dl) {
  dl._ResetForNewFrame();  // ImGui internal (underscore-prefixed) API.
}

// RAII-ish wrapper is overkill for a single call site; inline the create/cleanup pair
// and return {fbo, rbo}. Caller must restore prev_fbo + delete resources.
struct ScopedFbo {
  GLuint fbo = 0;
  GLuint rbo = 0;
  GLint prev_fbo = 0;
  bool complete = false;
};

ScopedFbo CreateRgba8Fbo(int w, int h) {
  ScopedFbo s;
  glGetIntegerv(GL_FRAMEBUFFER_BINDING, &s.prev_fbo);

  glGenFramebuffers(1, &s.fbo);
  glBindFramebuffer(GL_FRAMEBUFFER, s.fbo);
  glGenRenderbuffers(1, &s.rbo);
  glBindRenderbuffer(GL_RENDERBUFFER, s.rbo);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, w, h);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, s.rbo);

  s.complete = (glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE);
  return s;
}

void DestroyFbo(ScopedFbo& s) {
  glBindFramebuffer(GL_FRAMEBUFFER, s.prev_fbo);
  if (s.rbo != 0) {
    glDeleteRenderbuffers(1, &s.rbo);
  }
  if (s.fbo != 0) {
    glDeleteFramebuffers(1, &s.fbo);
  }
}

// Compose and render the overlay labels onto the currently bound FBO using
// a self-owned ImDrawList + ImDrawData::AddDrawList. Spike-verified path.
void RenderOverlayToFbo(const OverlayLabelInput& overlay_input, int dst_w, int dst_h) {
  std::vector<OverlayLabel> labels;
  ComputeOverlayLabels(overlay_input, 0.0f, 0.0f, static_cast<float>(dst_w), static_cast<float>(dst_h), labels);
  if (labels.empty()) {
    return;
  }

  ImDrawList dl(ImGui::GetDrawListSharedData());
  ResetDrawListForNewFrame(dl);
  dl.PushTextureID(ImGui::GetIO().Fonts->TexID);
  dl.PushClipRect(ImVec2(0.0f, 0.0f), ImVec2(static_cast<float>(dst_w), static_cast<float>(dst_h)));
  AppendOverlayToDrawList(&dl, labels);
  dl.PopClipRect();
  dl.PopTextureID();

  ImDrawData draw_data;
  draw_data.Clear();
  draw_data.DisplayPos = ImVec2(0.0f, 0.0f);
  draw_data.DisplaySize = ImVec2(static_cast<float>(dst_w), static_cast<float>(dst_h));
  draw_data.FramebufferScale = ImVec2(1.0f, 1.0f);
  draw_data.AddDrawList(&dl);

  ImGui_ImplOpenGL3_RenderDrawData(&draw_data);
}

}  // namespace

std::vector<unsigned char> RenderExportToRgba(PreviewRenderer& renderer, const PreviewParams& params, int dst_w,
                                              int dst_h, const std::optional<OverlayLabelInput>& overlay_input) {
  if (dst_w <= 0 || dst_h <= 0) {
    return {};
  }
  // GL_MAX_RENDERBUFFER_SIZE is available in GL 3.0+ (including macOS 3.3 Core);
  // GL_MAX_FRAMEBUFFER_{WIDTH,HEIGHT} requires GL 4.3 and isn't available on macOS.
  // Renderbuffer size is the binding constraint for this FBO attachment anyway.
  GLint max_rb = 0;
  glGetIntegerv(GL_MAX_RENDERBUFFER_SIZE, &max_rb);
  if (max_rb > 0 && (dst_w > max_rb || dst_h > max_rb)) {
    return {};
  }

  ScopedFbo s = CreateRgba8Fbo(dst_w, dst_h);
  if (!s.complete) {
    DestroyFbo(s);
    return {};
  }

  glViewport(0, 0, dst_w, dst_h);
  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);

  renderer.Render(0, 0, dst_w, dst_h, params);

  if (overlay_input.has_value()) {
    RenderOverlayToFbo(*overlay_input, dst_w, dst_h);
  }

  std::vector<unsigned char> rgba;
  bool ok = ReadbackGlRegionToRgba(0, 0, dst_w, dst_h, rgba);

  DestroyFbo(s);

  if (!ok) {
    return {};
  }
  return rgba;
}

}  // namespace lumice::gui
