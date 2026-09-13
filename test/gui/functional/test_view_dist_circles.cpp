// The view circles — constant angular distance from the camera's OPTICAL AXIS — in rendered GUI
// pixels: that the family's whole chain, from the angle list through the preview shader's reading
// of the view matrix, puts a ring on screen where the lens says it goes, that the ring belongs to
// the CAMERA and not to the sun, and that the panel's switches reach it.
//
// What this suite is for. The sun circles' suite (test_angular_dist_circles.cpp) already proves the
// shared machinery: the packed level list, the angular-distance field, the y sense of the world
// frame. What it cannot say anything about is the one thing this family adds — the CENTRE. The
// shader takes it from -u_view_matrix[2] rather than from a uniform the CPU fills, so there is no
// value a unit test could read back; only a rendered pixel can say which direction the ring is
// about. Two of the cases below are built around that: a ring whose centre followed the sun, or
// stayed pinned to the world frame when the camera turned, would fail them while every other test
// of the feature stayed green.
//
// Capture path for the four geometry cases: RenderExportToRgba's own off-screen FBO, the same one
// the sun circles' suite uses, so nothing depends on window size or panel layout. The last case
// drives the PANEL — the section fold, the angle editor, the two switches — and reads the on-screen
// preview back off the default framebuffer through g_fullframe_capture, because what it claims is
// that a user's clicks reach the pixels, and that is a claim about the live path.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <vector>

#include "gui/annotation_anchors.hpp"  // GuiSunWorldDir
#include "gui/app.hpp"                 // PreviewAnnotationAnchors, g_preview, g_preview_vp
#include "gui/export_fbo_renderer.hpp"
#include "gui/gui_constants.hpp"
#include "gui/gui_state.hpp"
#include "test_gui_shared.hpp"

namespace {

constexpr float kPi = 3.14159265358979323846f;

// Saturated primary, for the same reason the sun circles' suite uses one: a single channel test
// separates it from any blend against black, and the GuiState default can change without touching
// this file.
constexpr float kCircleR = 0.0f;
constexpr float kCircleG = 1.0f;
constexpr float kCircleB = 0.0f;

constexpr float kFov = 120.0f;
constexpr float kCircleDeg = 22.0f;
// Off-axis enough that a sun-centred ring of this radius would sit entirely in the upper half of
// the frame — the whole ring, not most of it — which is what makes "still on the centre" a
// discriminating claim rather than a tolerance.
constexpr float kOffAxisDeg = 40.0f;

struct RenderRequest {
  bool requested = false;
  bool done = false;
  bool show_line = true;
  int probe_w = 256;
  int probe_h = 256;
  float level_deg = kCircleDeg;
  float camera_elevation = 0.0f;
  float sun_altitude = 90.0f;
  std::vector<unsigned char> rgba;
};

RenderRequest g_req;

void RunRenderRequest() {
  gui::g_preview.ClearTexture();

  gui::PreviewParams params = gui::g_preview_vp.params;
  params.view_proj.lens_type = gui::kLensTypeLinear;
  params.view_proj.fov = kFov;
  params.view_proj.visible = gui::kVisibleFull;
  params.view_proj.elevation = g_req.camera_elevation;
  params.view_proj.azimuth = 0.0f;
  params.view_proj.roll = 0.0f;
  params.view_proj.front = false;
  params.source.max_abs_dz = gui::kDualFisheyeOverlap;
  params.source.r_scale = 1.0f / std::sqrt(1.0f + gui::kDualFisheyeOverlap);
  params.exposure.intensity_factor = 1.0f;
  params.exposure.intensity_scale = 0.0f;  // 8-bit RGB mode; no simulation has been run

  params.overlay.show_view_dist = g_req.show_line;
  params.overlay.view_dist_color[0] = kCircleR;
  params.overlay.view_dist_color[1] = kCircleG;
  params.overlay.view_dist_color[2] = kCircleB;
  params.overlay.view_dist_alpha = 1.0f;
  params.overlay.view_dist_deg = { g_req.level_deg };
  // The sun circles stay OFF, but their reference direction is still uploaded and still varies
  // between cases: a view-circle shader that read u_reference_dir by mistake would move with it.
  params.overlay.show_sun_circles = false;
  gui::GuiSunWorldDir(g_req.sun_altitude, params.overlay.reference_dir);

  g_req.rgba = gui::RenderExportToRgba(gui::g_preview, params, g_req.probe_w, g_req.probe_h);
  g_req.done = true;
  g_req.requested = false;
}

bool g_upload_requested = false;
bool g_upload_done = false;

// Both marshalled onto the render thread: the off-screen render for the geometry cases, and the
// black texture the panel case needs (RenderPreviewPanel publishes no viewport without one, and
// black is what makes "a coloured pixel is an annotation" unambiguous).
void ViewDistGuiFunc(ImGuiTestContext* /*ctx*/) {
  if (g_req.requested && !g_req.done) {
    RunRenderRequest();
  }
  if (g_upload_requested && !g_upload_done) {
    static const std::vector<unsigned char> kBlack(static_cast<size_t>(4) * 2 * 3, 0);
    gui::g_preview.UploadTexture(kBlack.data(), 4, 2);
    g_upload_done = true;
  }
}

std::vector<unsigned char> RenderFrame(ImGuiTestContext* ctx, const RenderRequest& req) {
  g_req = req;
  g_req.requested = true;
  g_req.done = false;
  g_req.rgba.clear();
  for (int i = 0; i < 120 && !g_req.done; ++i) {
    ctx->Yield();
  }
  return g_req.rgba;
}

// Center-origin, y-up, in a buffer that is row-major top-down.
bool ReadPixel(const std::vector<unsigned char>& rgba, int w, int h, float pos_x, float pos_y, unsigned char* out_rgb) {
  const int col = static_cast<int>(std::lround(pos_x + static_cast<float>(w) * 0.5f));
  const int row = static_cast<int>(std::lround(static_cast<float>(h) * 0.5f - pos_y));
  if (col < 0 || col >= w || row < 0 || row >= h) {
    return false;
  }
  const std::size_t off =
      (static_cast<std::size_t>(row) * static_cast<std::size_t>(w) + static_cast<std::size_t>(col)) * 4;
  out_rgb[0] = rgba[off + 0];
  out_rgb[1] = rgba[off + 1];
  out_rgb[2] = rgba[off + 2];
  return true;
}

bool LooksLikeTheCircle(const unsigned char rgb[3]) {
  return rgb[1] > 100 && rgb[0] < 80 && rgb[2] < 80;
}

// +-4 px along the radius, for the reason the sun circles' suite gives: the line is a few pixels
// wide and the rounding can land one off, and four pixels is far too narrow to find a ring that is
// somewhere else. A probe that falls off the canvas is skipped rather than counted either way.
bool CircleFoundNear(const std::vector<unsigned char>& rgba, int w, int h, float radius, float phi) {
  for (int d = -4; d <= 4; ++d) {
    unsigned char rgb[3] = {};
    const float r = radius + static_cast<float>(d);
    if (!ReadPixel(rgba, w, h, std::cos(phi) * r, std::sin(phi) * r, rgb)) {
      continue;
    }
    if (LooksLikeTheCircle(rgb)) {
      return true;
    }
  }
  return false;
}

std::size_t CountCirclePixels(const std::vector<unsigned char>& rgba) {
  std::size_t n = 0;
  for (std::size_t i = 0; i + 3 < rgba.size(); i += 4) {
    const unsigned char rgb[3] = { rgba[i], rgba[i + 1], rgba[i + 2] };
    if (LooksLikeTheCircle(rgb)) {
      ++n;
    }
  }
  return n;
}

// A linear (rectilinear) lens maps an angle t off-axis to r = f * tan(t), with f fixed by the
// half-FOV filling half the SHORT edge. Written out rather than called from the shader's helper on
// purpose: an independent prediction is the point.
float PredictedRadiusPx(int w, int h, float level_deg) {
  const float half_fov = kFov * 0.5f * kPi / 180.0f;
  const float f = static_cast<float>(std::min(w, h)) * 0.5f / std::tan(half_fov);
  return f * std::tan(level_deg * kPi / 180.0f);
}

// The ring is a full circle about the frame centre: found at four azimuths (an arc or a smear in
// one direction would satisfy one), and NOT at the centre (a level set read as "everything
// inside" would light the whole disc).
void ExpectConcentricRing(ImGuiTestContext* ctx, const std::vector<unsigned char>& rgba, int w, int h, float radius) {
  int found = 0;
  for (int k = 0; k < 4; ++k) {
    if (CircleFoundNear(rgba, w, h, radius, static_cast<float>(k) * kPi * 0.5f)) {
      ++found;
    }
  }
  IM_CHECK_EQ(found, 4);
  unsigned char centre[3] = {};
  IM_CHECK(ReadPixel(rgba, w, h, 0.0f, 0.0f, centre));
  IM_CHECK(!LooksLikeTheCircle(centre));
}

// ---- the live-path case's helpers ----

struct Frame {
  std::vector<unsigned char> rgba;
  int w = 0;
  int h = 0;
};

// The preview viewport, read back off the default framebuffer after this frame's render pass. Same
// protocol and same timing note as test_annotation_line_tracking.cpp's CaptureViewportThenWrite: a
// TestFunc resumes at EndFrame, so the viewport rect published for the frame about to be drawn is
// the one captured.
bool CaptureViewport(ImGuiTestContext* ctx, Frame* out) {
  if (!gui::g_preview_vp.active || gui::g_preview_vp.vp_w <= 0 || gui::g_preview_vp.vp_h <= 0) {
    return false;
  }
  g_fullframe_capture.Reset();
  g_fullframe_capture.rect_x = gui::g_preview_vp.vp_x;
  g_fullframe_capture.rect_y = gui::g_preview_vp.vp_y;
  g_fullframe_capture.rect_w = gui::g_preview_vp.vp_w;
  g_fullframe_capture.rect_h = gui::g_preview_vp.vp_h;
  g_fullframe_capture.requested.store(true);
  for (int i = 0; i < 4 && !g_fullframe_capture.done.load(); ++i) {
    ctx->Yield(1);
  }
  if (!g_fullframe_capture.done.load()) {
    return false;
  }
  out->rgba = g_fullframe_capture.pixels;
  out->w = g_fullframe_capture.width;
  out->h = g_fullframe_capture.height;
  return out->rgba.size() == static_cast<size_t>(out->w) * static_cast<size_t>(out->h) * 4;
}

// Pixels whose max-channel |delta| exceeds a small tolerance. A tolerance rather than byte
// identity because the two captures are two frames of a live window, and what is asked is whether
// an annotation appeared, not whether the compositor is deterministic to the bit.
std::size_t DifferingPixels(const Frame& a, const Frame& b) {
  if (a.w != b.w || a.h != b.h || a.rgba.size() != b.rgba.size()) {
    return static_cast<std::size_t>(-1);
  }
  std::size_t n = 0;
  for (std::size_t i = 0; i + 3 < a.rgba.size(); i += 4) {
    int dmax = 0;
    for (std::size_t c = 0; c < 3; ++c) {
      dmax = std::max(dmax, std::abs(static_cast<int>(a.rgba[i + c]) - static_cast<int>(b.rgba[i + c])));
    }
    if (dmax > 8) {
      ++n;
    }
  }
  return n;
}

}  // namespace

void RegisterViewDistCircleTests(ImGuiTestEngine* engine) {
  // The whole chain, on the frame's own centre: with a linear lens looking at the horizon and the
  // sun 90 deg away from the axis, the 22 deg view circle is a ring of a radius the lens fixes,
  // about the frame centre. A shader that centred it on u_reference_dir — the natural copy-paste
  // error from the family this one was written from — puts the ring 90 deg away and fails here.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "view_dist", "a_circle_is_drawn_about_the_frame_centre_at_the_lens_radius");
    t->GuiFunc = ViewDistGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      RenderRequest req;
      req.camera_elevation = 0.0f;
      req.sun_altitude = 90.0f;
      const std::vector<unsigned char> rgba = RenderFrame(ctx, req);
      IM_CHECK(!rgba.empty());
      IM_CHECK_EQ(rgba.size(), static_cast<std::size_t>(req.probe_w) * req.probe_h * 4);

      const float expect_r = PredictedRadiusPx(req.probe_w, req.probe_h, kCircleDeg);
      IM_CHECK(expect_r > 8.0f);
      IM_CHECK(expect_r < req.probe_h * 0.5f - 8.0f);
      ExpectConcentricRing(ctx, rgba, req.probe_w, req.probe_h, expect_r);

      // And the sun's position is not an input: the same frame with the sun on the axis is the
      // same frame. Compared whole rather than at the ring, so a second, sun-centred ring showing
      // up anywhere would count.
      req.sun_altitude = 0.0f;
      const std::vector<unsigned char> same = RenderFrame(ctx, req);
      IM_CHECK(!same.empty());
      IM_CHECK(same == rgba);
    };
  }

  // The family's self-evident radius: a circle at fov/2 is INSCRIBED in the short edge — it
  // touches the top and bottom of a landscape frame and clears the sides. Asserted on a
  // non-square canvas so "short edge" is a claim and not a coincidence of w == h, and it is the
  // reason a user reaches for this overlay at all (where does my field of view end?).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "view_dist", "the_half_fov_circle_is_inscribed_in_the_short_edge");
    t->GuiFunc = ViewDistGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      RenderRequest req;
      req.probe_w = 256;
      req.probe_h = 192;
      req.level_deg = kFov * 0.5f;
      const std::vector<unsigned char> rgba = RenderFrame(ctx, req);
      IM_CHECK(!rgba.empty());

      // f * tan(fov/2) = short_edge / 2 by construction; stated through the predictor so the two
      // readings of the same identity are both on record.
      const float expect_r = PredictedRadiusPx(req.probe_w, req.probe_h, req.level_deg);
      IM_CHECK_LT(std::fabs(expect_r - static_cast<float>(req.probe_h) * 0.5f), 0.01f);

      // Top and bottom: the ring's crown sits in the outermost rows. The probe window reaches
      // outside the canvas for half its width there, which CircleFoundNear skips, so a hit is
      // necessarily in the last few rows.
      IM_CHECK(CircleFoundNear(rgba, req.probe_w, req.probe_h, expect_r, kPi * 0.5f));
      IM_CHECK(CircleFoundNear(rgba, req.probe_w, req.probe_h, expect_r, -kPi * 0.5f));
      // Left and right: the same radius lands 32 px inside the long edge, so it is found there
      // too, and the columns beyond it are clear — the ring does not reach the sides.
      IM_CHECK(CircleFoundNear(rgba, req.probe_w, req.probe_h, expect_r, 0.0f));
      IM_CHECK(CircleFoundNear(rgba, req.probe_w, req.probe_h, expect_r, kPi));
      unsigned char edge[3] = {};
      IM_CHECK(ReadPixel(rgba, req.probe_w, req.probe_h, static_cast<float>(req.probe_w) * 0.5f - 2.0f, 0.0f, edge));
      IM_CHECK(!LooksLikeTheCircle(edge));
    };
  }

  // The centre follows the CAMERA. With the camera tilted 40 deg up, the ring is still about the
  // frame centre — where the sun circles' suite, under the same tilt of the sun instead, requires
  // the whole ring in the upper half. A view circle whose centre stayed at the world direction
  // the camera had at elevation 0 would land entirely in the LOWER half here and fail the
  // concentric check.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "view_dist", "an_off_axis_camera_keeps_the_ring_on_the_frame_centre");
    t->GuiFunc = ViewDistGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      RenderRequest req;
      req.camera_elevation = kOffAxisDeg;
      req.sun_altitude = 90.0f;
      const std::vector<unsigned char> rgba = RenderFrame(ctx, req);
      IM_CHECK(!rgba.empty());
      ExpectConcentricRing(ctx, rgba, req.probe_w, req.probe_h,
                           PredictedRadiusPx(req.probe_w, req.probe_h, kCircleDeg));
    };
  }

  // The switch, asserted as the ABSENCE of any circle-coloured pixel in the whole frame.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "view_dist", "the_line_switch_removes_every_circle_pixel");
    t->GuiFunc = ViewDistGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      RenderRequest req;
      const std::vector<unsigned char> on = RenderFrame(ctx, req);
      IM_CHECK(!on.empty());
      IM_CHECK(CountCirclePixels(on) > 0);

      req.show_line = false;
      const std::vector<unsigned char> off = RenderFrame(ctx, req);
      IM_CHECK(!off.empty());
      IM_CHECK_EQ(CountCirclePixels(off), static_cast<std::size_t>(0));
    };
  }

  // The live path, from the panel a user sees. Starting from a new document — section folded, no
  // angles, both switches off — the user unfolds the section, adds an angle through the fold's
  // editor, and switches the line on: the preview changes, and changes AT THE RING. The label
  // switch is then shown to be its own switch (its anchors appear and disappear with it, with the
  // line switch left alone), and switching everything off again restores the frame.
  //
  // The preview is a black texture over a linear lens, so the only coloured pixels are the ones
  // this family paints. The family colour is set by assignment — it is appearance, and the
  // proposition is about the switches, not about the colour swatch.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "view_dist", "from_the_folded_section_the_two_switches_reach_the_preview");
    t->GuiFunc = ViewDistGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      const ScopedPopups popup_guard(ctx);
      ctx->Yield(2);
      g_upload_requested = true;
      g_upload_done = false;
      for (int i = 0; i < 60 && !g_upload_done; ++i) {
        ctx->Yield();
      }
      IM_CHECK(g_upload_done);

      gui::g_state.renderer.lens_type = gui::kLensTypeLinear;
      gui::g_state.renderer.fov = kFov;
      gui::g_state.renderer.visible = gui::kVisibleFull;
      gui::g_state.renderer.front = false;
      gui::g_state.renderer.elevation = 0.0f;
      gui::g_state.renderer.azimuth = 0.0f;
      gui::g_state.renderer.roll = 0.0f;
      gui::g_state.view_dist_color[0] = kCircleR;
      gui::g_state.view_dist_color[1] = kCircleG;
      gui::g_state.view_dist_color[2] = kCircleB;
      gui::g_state.view_dist_alpha = 1.0f;
      ctx->MouseMoveToPos(ImVec2(-100.0f, -100.0f));
      ctx->Yield(6);

      // The premise: a new document opens with the section folded, the list empty and both
      // switches off. Stated so the arithmetic below measures the task it names.
      IM_CHECK(!gui::g_state.angular_dist_section_open);
      IM_CHECK(gui::g_state.view_dist_angles.empty());
      IM_CHECK(!gui::g_state.show_view_dist_line);
      IM_CHECK(!gui::g_state.show_view_dist_label);

      Frame baseline;
      IM_CHECK(CaptureViewport(ctx, &baseline));

      // Unfold the Angular Distance section by a click on its header, add an angle through the
      // Lens Center row's own fold, then switch the line on. Named ref for the panel clicks (a
      // control scrolled out of view reads as missing to a wildcard lookup); released for the
      // popup, which is a different window.
      ctx->SetRef("//##RightPanel");
      ctx->ItemClick("**/Angular Distance from...##angular_dist");
      ctx->Yield(2);
      IM_CHECK(gui::g_state.angular_dist_section_open);
      ctx->ItemClick("**/###view_dist_fold");
      ctx->SetRef("");
      ctx->Yield(3);
      ctx->ItemClick("**/22\xc2\xb0");
      ctx->Yield(2);
      ctx->KeyPress(ImGuiKey_Escape);
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.view_dist_angles.size(), static_cast<size_t>(1));
      IM_CHECK_EQ(gui::g_state.view_dist_angles[0], kCircleDeg);

      ctx->SetRef("//##RightPanel");
      ctx->ItemClick("**/##view_dist_line");
      ctx->SetRef("");
      ctx->Yield(2);
      IM_CHECK(gui::g_state.show_view_dist_line);
      ctx->MouseMoveToPos(ImVec2(-100.0f, -100.0f));
      ctx->Yield(4);

      Frame line_on;
      IM_CHECK(CaptureViewport(ctx, &line_on));
      IM_CHECK_GT(DifferingPixels(baseline, line_on), static_cast<std::size_t>(0));
      // ...and the change is AT THE RING: the predicted radius, about the viewport's centre, in
      // device pixels — the space the capture is in.
      const float expect_r = PredictedRadiusPx(line_on.w, line_on.h, kCircleDeg);
      int found = 0;
      for (int k = 0; k < 4; ++k) {
        if (CircleFoundNear(line_on.rgba, line_on.w, line_on.h, expect_r, static_cast<float>(k) * kPi * 0.5f)) {
          ++found;
        }
      }
      IM_CHECK_EQ(found, 4);
      // The line switch alone asks core for no anchors: the label half is off.
      IM_CHECK(gui::PreviewAnnotationAnchors().ViewDistLabels().empty());

      // The label switch, with the line left ON: anchors appear, and the frame changes again
      // (text is drawn). Then the line switch OFF with the label left ON: the ring goes, the
      // anchors stay — the two switches are independent in both directions.
      ctx->SetRef("//##RightPanel");
      ctx->ItemClick("**/##view_dist_label");
      ctx->SetRef("");
      ctx->Yield(4);
      IM_CHECK(gui::g_state.show_view_dist_label);
      IM_CHECK(!gui::PreviewAnnotationAnchors().ViewDistLabels().empty());
      Frame both_on;
      IM_CHECK(CaptureViewport(ctx, &both_on));
      IM_CHECK_GT(DifferingPixels(line_on, both_on), static_cast<std::size_t>(0));

      ctx->SetRef("//##RightPanel");
      ctx->ItemClick("**/##view_dist_line");
      ctx->SetRef("");
      ctx->Yield(4);
      IM_CHECK(!gui::g_state.show_view_dist_line);
      IM_CHECK(!gui::PreviewAnnotationAnchors().ViewDistLabels().empty());
      Frame label_only;
      IM_CHECK(CaptureViewport(ctx, &label_only));
      IM_CHECK_GT(DifferingPixels(baseline, label_only), static_cast<std::size_t>(0));
      IM_CHECK_GT(DifferingPixels(line_on, label_only), static_cast<std::size_t>(0));

      // Everything off: back to the frame we started from.
      ctx->SetRef("//##RightPanel");
      ctx->ItemClick("**/##view_dist_label");
      ctx->SetRef("");
      ctx->Yield(4);
      IM_CHECK(!gui::g_state.show_view_dist_label);
      IM_CHECK(gui::PreviewAnnotationAnchors().ViewDistLabels().empty());
      Frame restored;
      IM_CHECK(CaptureViewport(ctx, &restored));
      IM_CHECK_EQ(DifferingPixels(baseline, restored), static_cast<std::size_t>(0));

      gui::g_state.angular_dist_section_open = false;
      ctx->Yield(2);
    };
  }
}
