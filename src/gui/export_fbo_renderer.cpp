#include "gui/export_fbo_renderer.hpp"

#include "gui/app.hpp"
#include "gui/gl_capture.hpp"
#include "gui/gl_common.h"
#include "gui/gui_logger.hpp"
#include "imgui.h"
#include "imgui_impl_opengl3.h"

namespace lumice::gui {
namespace {

// Single choke-point for ImGui internal API use. If ImGui is upgraded, grep this
// symbol and re-verify against the new ImDrawList API. Verified for v1.91.8-docking
// (cache commit f6a6076), contract test: gui_test's
// export/a_self_owned_drawlist_still_reaches_a_bound_fbo, whose body is the original
// spike probe (RunSpikeFboImDrawListEndToEnd in test/gui/functional/test_export.cpp).
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
//
// COORDINATE SYSTEMS, the thing this function is easy to get wrong in a way nothing reports:
// the FBO is `dst_w x dst_h` DEVICE pixels, but everything a draw list holds — the anchors, the
// clip rect, the glyph quads ImGui emits from the font atlas — is in LOGICAL POINTS. The OpenGL3
// backend bridges the two by multiplying by FramebufferScale, and it requires
// `DisplaySize * FramebufferScale == the target surface's device pixel size`. So the size handed
// to DisplaySize and to the label pass must be the LOGICAL one, and FramebufferScale carries the
// DPI alone. Setting FramebufferScale to 1 while feeding device-pixel sizes also satisfies that
// product — which is why the positions looked right — but the glyph height is
// `baked font size * FramebufferScale` and depends on nothing else, so the text came out at 1/dpi
// of its on-screen size. Change one of the three and you must change all three.
void RenderOverlayToFbo(const std::vector<CurveLabelSet>& curve_labels, int dst_w, int dst_h, float dpi_scale_x,
                        float dpi_scale_y) {
  std::vector<OverlayLabel> labels;
  for (const CurveLabelSet& set : curve_labels) {
    AppendCurveLabels(set, 0.0f, 0.0f, labels);
  }
  if (labels.empty()) {
    return;
  }

  // A non-positive scale would put an infinity into DisplaySize and take the whole frame's draw
  // data with it; it cannot arise from a live GLFW window but this function is also reachable from
  // tests that name their own numbers.
  const float sx = dpi_scale_x > 0.0f ? dpi_scale_x : 1.0f;
  const float sy = dpi_scale_y > 0.0f ? dpi_scale_y : 1.0f;
  const float logical_w = static_cast<float>(dst_w) / sx;
  const float logical_h = static_cast<float>(dst_h) / sy;

  ImDrawList dl(ImGui::GetDrawListSharedData());
  ResetDrawListForNewFrame(dl);
  dl.PushTextureID(ImGui::GetIO().Fonts->TexID);
  dl.PushClipRect(ImVec2(0.0f, 0.0f), ImVec2(logical_w, logical_h));
  AppendOverlayToDrawList(&dl, labels, 0.0f, 0.0f, logical_w, logical_h);
  dl.PopClipRect();
  dl.PopTextureID();

  ImDrawData draw_data;
  draw_data.Clear();
  draw_data.DisplayPos = ImVec2(0.0f, 0.0f);
  draw_data.DisplaySize = ImVec2(logical_w, logical_h);
  draw_data.FramebufferScale = ImVec2(sx, sy);
  draw_data.AddDrawList(&dl);

  ImGui_ImplOpenGL3_RenderDrawData(&draw_data);
}

// The single render core, shared by the on-screen preview and the PNG export. Assumes the target
// FBO is already bound and that the caller has saved whatever GL state it cares about; leaves the
// viewport and the clear colour set to what it used, per the two callers' own restore code.
//
// `clear_rgba` is a parameter rather than a constant because the two consumers want different
// answers for the pixels the preview shader does not paint: the export wants black (nothing behind
// it), the on-screen path wants the colour the frame's own glClear already put in that rectangle.
void RenderFrameContentToBoundFbo(PreviewRenderer& renderer, const PreviewParams& params, int dst_w, int dst_h,
                                  const std::vector<CurveLabelSet>& curve_labels, float dpi_scale_x, float dpi_scale_y,
                                  const GLfloat clear_rgba[4]) {
  glViewport(0, 0, dst_w, dst_h);
  glClearColor(clear_rgba[0], clear_rgba[1], clear_rgba[2], clear_rgba[3]);
  glClear(GL_COLOR_BUFFER_BIT);

  renderer.Render(0, 0, dst_w, dst_h, params);

  RenderOverlayToFbo(curve_labels, dst_w, dst_h, dpi_scale_x, dpi_scale_y);
}

// The on-screen path's FBO, held across frames. Deliberately owned here rather than by
// PreviewRenderer: that class owns the shader and the textures, and a framebuffer is a third kind
// of resource with a different lifetime question (it is reallocated on every viewport resize).
// Keeping it beside CreateRgba8Fbo puts all of this file's FBO handling under one owner; the cost
// is that teardown is two calls at the call site instead of one.
struct PersistentFbo {
  GLuint fbo = 0;
  GLuint rbo = 0;
  int w = 0;
  int h = 0;
  bool complete = false;
  bool logged_failure = false;
};

PersistentFbo g_preview_frame_fbo;

}  // namespace

std::vector<unsigned char> RenderExportToRgba(PreviewRenderer& renderer, const PreviewParams& params, int dst_w,
                                              int dst_h, const std::vector<CurveLabelSet>& curve_labels,
                                              float dpi_scale_x, float dpi_scale_y) {
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

  // Save caller GL state that we mutate, per header contract.
  GLint prev_viewport[4] = { 0, 0, 0, 0 };
  glGetIntegerv(GL_VIEWPORT, prev_viewport);
  GLfloat prev_clear_color[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
  glGetFloatv(GL_COLOR_CLEAR_VALUE, prev_clear_color);

  ScopedFbo s = CreateRgba8Fbo(dst_w, dst_h);
  if (!s.complete) {
    DestroyFbo(s);
    glViewport(prev_viewport[0], prev_viewport[1], prev_viewport[2], prev_viewport[3]);
    glClearColor(prev_clear_color[0], prev_clear_color[1], prev_clear_color[2], prev_clear_color[3]);
    return {};
  }

  const GLfloat export_clear[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
  RenderFrameContentToBoundFbo(renderer, params, dst_w, dst_h, curve_labels, dpi_scale_x, dpi_scale_y, export_clear);

  std::vector<unsigned char> rgba;
  bool ok = ReadbackGlRegionToRgba(0, 0, dst_w, dst_h, rgba);

  DestroyFbo(s);
  glViewport(prev_viewport[0], prev_viewport[1], prev_viewport[2], prev_viewport[3]);
  glClearColor(prev_clear_color[0], prev_clear_color[1], prev_clear_color[2], prev_clear_color[3]);

  if (!ok) {
    return {};
  }
  return rgba;
}

void RenderPreviewFrameAndBlit(PreviewRenderer& renderer, const PreviewParams& params, int dst_x, int dst_y, int dst_w,
                               int dst_h, const std::vector<CurveLabelSet>& curve_labels, float dpi_scale_x,
                               float dpi_scale_y) {
  // Hard guard, not an optimization: a collapsed preview panel or a minimized window produces a
  // zero extent every frame, and glRenderbufferStorage is undefined for one.
  if (dst_w <= 0 || dst_h <= 0) {
    return;
  }

  PersistentFbo& s = g_preview_frame_fbo;

  GLint prev_fbo = 0;
  glGetIntegerv(GL_FRAMEBUFFER_BINDING, &prev_fbo);
  GLint prev_viewport[4] = { 0, 0, 0, 0 };
  glGetIntegerv(GL_VIEWPORT, prev_viewport);
  // Also the colour the region being overwritten was cleared with — see the header contract.
  GLfloat prev_clear_color[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
  glGetFloatv(GL_COLOR_CLEAR_VALUE, prev_clear_color);

  if (s.fbo == 0) {
    glGenFramebuffers(1, &s.fbo);
    glGenRenderbuffers(1, &s.rbo);
  }
  if (s.w != dst_w || s.h != dst_h) {
    glBindFramebuffer(GL_FRAMEBUFFER, s.fbo);
    glBindRenderbuffer(GL_RENDERBUFFER, s.rbo);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, dst_w, dst_h);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, s.rbo);
    s.complete = (glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE);
    s.w = dst_w;
    s.h = dst_h;
  }

  if (!s.complete) {
    // Once per process, not once per frame: this runs in the draw loop.
    if (!s.logged_failure) {
      s.logged_failure = true;
      GUI_LOG_ERROR("[GUI] Preview FBO incomplete ({}x{}); the preview area will not be drawn", dst_w, dst_h);
    }
    glBindFramebuffer(GL_FRAMEBUFFER, prev_fbo);
    glViewport(prev_viewport[0], prev_viewport[1], prev_viewport[2], prev_viewport[3]);
    return;
  }

  glBindFramebuffer(GL_FRAMEBUFFER, s.fbo);
  RenderFrameContentToBoundFbo(renderer, params, dst_w, dst_h, curve_labels, dpi_scale_x, dpi_scale_y,
                               prev_clear_color);

  // GL_NEAREST with equal source and destination extents is a straight copy: the FBO is allocated
  // at the viewport's DEVICE pixel size precisely so this stays 1:1 and no resampling softens the
  // preview. GL_LINEAR would be legal here and silently wrong the day the two sizes diverge.
  glBindFramebuffer(GL_READ_FRAMEBUFFER, s.fbo);
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, prev_fbo);
  glBlitFramebuffer(0, 0, dst_w, dst_h, dst_x, dst_y, dst_x + dst_w, dst_y + dst_h, GL_COLOR_BUFFER_BIT, GL_NEAREST);

  glBindFramebuffer(GL_FRAMEBUFFER, prev_fbo);
  glViewport(prev_viewport[0], prev_viewport[1], prev_viewport[2], prev_viewport[3]);
  glClearColor(prev_clear_color[0], prev_clear_color[1], prev_clear_color[2], prev_clear_color[3]);
}

void RunSharedFrameRenderPass(int display_w, int display_h) {
  glViewport(0, 0, display_w, display_h);
  glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);

  // Render the preview before the ImGui overlay: image, overlay lines and overlay labels into an
  // off-screen FBO, then blitted back onto this viewport rect. The FBO is the same render core
  // the Screenshot export reads back, which is what makes the two agree by construction rather
  // than by two paths being kept in step. Position in the sequence is unchanged, so ImGui's draw
  // data still lands on top and modals still occlude the preview — labels included, now for the
  // same reason the lines already were.
  if (g_preview_vp.active) {
    RenderPreviewFrameAndBlit(g_preview, g_preview_vp.params, g_preview_vp.vp_x, g_preview_vp.vp_y, g_preview_vp.vp_w,
                              g_preview_vp.vp_h, g_preview_vp.curve_labels, g_preview_vp.dpi_scale_x,
                              g_preview_vp.dpi_scale_y);
  }

  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void DestroyPreviewFrameFbo() {
  PersistentFbo& s = g_preview_frame_fbo;
  if (s.rbo != 0) {
    glDeleteRenderbuffers(1, &s.rbo);
  }
  if (s.fbo != 0) {
    glDeleteFramebuffers(1, &s.fbo);
  }
  s = PersistentFbo{};
}

// Export Configure functions only override view_proj / overlay / bg. The
// source sub-struct (max_abs_dz / r_scale) is inherited from the live preview
// — live preview already hard-codes kDualFisheyeOverlap in app_panels.cpp, and
// both equirect and dual-fisheye shaders sample the same dual-fisheye source
// texture, so inheriting source yields byte-identical preview/export when
// lens_type matches.
void ConfigureDualFisheyeExportParams(PreviewParams& params) {
  params.view_proj = kDualFisheyeExportViewProj;
  params.overlay = OverlayDecoration::Disabled();
  params.bg = Background::Disabled();
}

void ConfigureEquirectExportParams(PreviewParams& params) {
  params.view_proj = kEquirectExportViewProj;
  params.overlay = OverlayDecoration::Disabled();
  params.bg = Background::Disabled();
}

}  // namespace lumice::gui
