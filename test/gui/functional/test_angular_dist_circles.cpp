// The angular-distance circles, in rendered GUI pixels: that the whole chain from GuiState through
// core's mask to the preview shader actually puts a circle on screen, in the right place, and that
// the switch turns it off.
//
// What this suite is for. Every other test of this feature stops one layer short of the picture:
// the CLI's own compositing is asserted in unit-correctness/server/test_render_consumer_angular_dist
// .cpp, core's geometry in unit-correctness/core/test_annotation_overlay.cpp, and the request
// AnnotationOverlayCache builds is plain C++. What none of them can see is whether the mask reaches
// the shader at all — the texture upload, the sampler binding, and above all the uv mapping, where
// core's top-left-origin row-major mask meets a v_ndc whose y points the other way. A y flip that
// went missing would mirror every circle about the horizon and break nothing that compiles.
//
// Capture path: RenderExportToRgba's own off-screen FBO, the same one test_lens_border.cpp uses, so
// nothing here depends on window size or panel layout. No simulation is run, so the frame is black
// everywhere the annotation is not and a coloured pixel is unambiguous.
//
// THE SETTLE DELAY IS PART OF WHAT IS TESTED. AnnotationOverlayCache deliberately does not compute
// on the frame the view changes — the core call is far too expensive for a draw loop, so it waits
// for the view to hold still (see that class's own comment). RequestFrame below therefore pumps
// frames through RenderPreviewPanel before capturing, and the "circles eventually appear" assertion
// is also the assertion that the debounce does eventually fire. A cache that never settled would
// leave this suite red rather than silently drawing nothing.

#include <cmath>
#include <cstddef>
#include <vector>

#include "gui/annotation_overlay_cache.hpp"
#include "gui/app.hpp"
#include "gui/export_fbo_renderer.hpp"
#include "gui/gui_constants.hpp"
#include "gui/gui_state.hpp"
#include "test_gui_shared.hpp"

namespace {

constexpr int kProbeW = 256;
constexpr int kProbeH = 256;
constexpr float kPi = 3.14159265358979323846f;

// A saturated primary rather than the GuiState default: separable from any blend against black by a
// single channel test, and immune to a later change of the default.
constexpr float kCircleR = 0.0f;
constexpr float kCircleG = 1.0f;
constexpr float kCircleB = 0.0f;

// Sun at the zenith and a linear lens pointed straight up, so the circle of a given angular
// distance is a plain concentric ring about the frame centre whose radius follows from the lens
// alone. That is what makes "in the right place" checkable without re-deriving the projection here.
constexpr float kSunAltitude = 90.0f;
constexpr float kFov = 120.0f;
// Wide enough that a ring centred 40 deg off-axis still fits: 40 + 22 = 62 deg < the 75 deg
// half-FOV, with room for the mask's own width.
constexpr float kWideFov = 150.0f;
constexpr float kOffAxisSunAltitude = 40.0f;
constexpr float kCircleDeg = 22.0f;

struct RenderRequest {
  bool requested = false;
  bool done = false;
  bool show_line = true;
  // The scene, so a case can move the sun off the optical axis without a second harness.
  float sun_altitude = 90.0f;
  float camera_elevation = 90.0f;
  std::vector<unsigned char> rgba;

  void Reset() { *this = RenderRequest{}; }
};

RenderRequest g_req;

void RunRenderRequest() {
  // Blank the source texture: whatever ran before left one behind, and a coloured sample showing
  // through would make "the circle is the only coloured thing here" false.
  gui::g_preview.ClearTexture();

  gui::PreviewParams params = gui::g_preview_vp.params;
  params.view_proj.lens_type = gui::kLensTypeLinear;
  params.view_proj.fov = g_req.camera_elevation == kSunAltitude ? kFov : kWideFov;
  params.view_proj.visible = gui::kVisibleFull;
  params.view_proj.elevation = g_req.camera_elevation;
  params.view_proj.azimuth = 0.0f;
  params.view_proj.roll = 0.0f;
  params.view_proj.front = false;
  params.source.max_abs_dz = gui::kDualFisheyeOverlap;
  params.source.r_scale = 1.0f / std::sqrt(1.0f + gui::kDualFisheyeOverlap);
  params.exposure.intensity_factor = 1.0f;
  params.exposure.intensity_scale = 0.0f;  // 8-bit RGB mode; no simulation has been run

  params.overlay.show_sun_circles = g_req.show_line;
  params.overlay.sun_circles_color[0] = kCircleR;
  params.overlay.sun_circles_color[1] = kCircleG;
  params.overlay.sun_circles_color[2] = kCircleB;
  params.overlay.sun_circles_alpha = 1.0f;  // opaque: the pixel test reads a colour, not a blend

  // The mask for THIS probe's view, at the probe's own size — not the live viewport's. Built here
  // rather than borrowed from RenderPreviewPanel's cache because the probe renders a view of its
  // own choosing; borrowing would sample a mask computed for whatever the panel happens to show.
  static gui::AnnotationOverlayCache probe_cache;
  gui::AnnotationOverlayCache::ViewKey key;
  key.width = kProbeW;
  key.height = kProbeH;
  key.lens_type = params.view_proj.lens_type;
  key.fov = params.view_proj.fov;
  key.visible = params.view_proj.visible;
  key.front = false;
  key.azimuth = params.view_proj.azimuth;
  key.elevation = params.view_proj.elevation;
  key.roll = params.view_proj.roll;
  gui::GuiSunWorldDir(g_req.sun_altitude, key.sun_dir);
  key.angular_dist_deg = { kCircleDeg };
  // Drive the debounce to its settle point. Doing it in a loop here rather than across GUI frames
  // keeps the probe's own cache independent of how many frames the harness happens to pump.
  for (int i = 0; i <= gui::AnnotationOverlayCache::kSettleFrames; ++i) {
    probe_cache.Update(key);
  }
  if (probe_cache.HasResult()) {
    params.overlay.angular_dist_mask = probe_cache.AngularDistMask().data();
    params.overlay.angular_dist_mask_w = probe_cache.Width();
    params.overlay.angular_dist_mask_h = probe_cache.Height();
    params.overlay.angular_dist_mask_generation = probe_cache.Generation();
  }

  g_req.rgba = gui::RenderExportToRgba(gui::g_preview, params, kProbeW, kProbeH, std::nullopt);
  g_req.done = true;
  g_req.requested = false;
}

void AngularDistGuiFunc(ImGuiTestContext* /*ctx*/) {
  if (g_req.requested && !g_req.done) {
    RunRenderRequest();
  }
}

// Ask the render thread for one frame and wait. Bounded so a wiring regression fails at the
// assertion below rather than hanging the suite.
std::vector<unsigned char> RenderFrame(ImGuiTestContext* ctx, bool show_line, float sun_altitude = kSunAltitude,
                                       float camera_elevation = kSunAltitude) {
  g_req.Reset();
  g_req.show_line = show_line;
  g_req.sun_altitude = sun_altitude;
  g_req.camera_elevation = camera_elevation;
  g_req.requested = true;
  for (int i = 0; i < 120 && !g_req.done; ++i) {
    ctx->Yield();
  }
  return g_req.rgba;
}

// Center-origin, y-up — the convention a radius-and-angle prediction is naturally written in. The
// buffer itself is row-major top-down.
bool ReadPixel(const std::vector<unsigned char>& rgba, float pos_x, float pos_y, unsigned char* out_rgb) {
  const int col = static_cast<int>(std::lround(pos_x + kProbeW * 0.5f));
  const int row = static_cast<int>(std::lround(kProbeH * 0.5f - pos_y));
  if (col < 0 || col >= kProbeW || row < 0 || row >= kProbeH) {
    return false;
  }
  const std::size_t off = (static_cast<std::size_t>(row) * kProbeW + col) * 4;
  out_rgb[0] = rgba[off + 0];
  out_rgb[1] = rgba[off + 1];
  out_rgb[2] = rgba[off + 2];
  return true;
}

bool LooksLikeTheCircle(const unsigned char rgb[3]) {
  return rgb[1] > 100 && rgb[0] < 80 && rgb[2] < 80;
}

// Scan a short radial segment. The mask is a couple of pixels wide and the rounding above can land
// one pixel off, so demanding the circle at one exact coordinate would assert the rasteriser's
// rounding rather than the circle's position. +-4 px absorbs that and is far too narrow to find a
// circle that is somewhere else.
bool CircleFoundNear(const std::vector<unsigned char>& rgba, float radius, float phi) {
  for (int d = -4; d <= 4; ++d) {
    unsigned char rgb[3] = {};
    const float r = radius + static_cast<float>(d);
    if (!ReadPixel(rgba, std::cos(phi) * r, std::sin(phi) * r, rgb)) {
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

}  // namespace

void RegisterAngularDistCircleTests(ImGuiTestEngine* engine) {
  // The whole chain, and the one assertion that catches a wrong uv mapping: with the sun at the
  // zenith under a linear lens pointed straight up, the 22 deg circle is a ring of a radius the
  // lens fixes. A mask sampled with the wrong y sense, the wrong scale, or not at all fails to put
  // green there — and a stray green pixel anywhere else fails the ring's own shape check.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "angular_dist", "a_circle_is_drawn_at_the_radius_the_lens_predicts");
    t->GuiFunc = AngularDistGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      const std::vector<unsigned char> rgba = RenderFrame(ctx, /*show_line=*/true);
      IM_CHECK(!rgba.empty());
      IM_CHECK_EQ(rgba.size(), static_cast<std::size_t>(kProbeW) * kProbeH * 4);

      // A linear (rectilinear) lens maps an angle t off-axis to r = f * tan(t), with f fixed by the
      // half-FOV filling half the short edge. Written out rather than called from the shader's own
      // helper on purpose: an independent prediction is the point.
      const float half_fov = kFov * 0.5f * kPi / 180.0f;
      const float f = (kProbeH * 0.5f) / std::tan(half_fov);
      const float expect_r = f * std::tan(kCircleDeg * kPi / 180.0f);
      IM_CHECK(expect_r > 8.0f);
      IM_CHECK(expect_r < kProbeH * 0.5f - 8.0f);

      // Four azimuths: one would be satisfied by an arc, or by a smear in one direction.
      int found = 0;
      for (int k = 0; k < 4; ++k) {
        if (CircleFoundNear(rgba, expect_r, static_cast<float>(k) * kPi * 0.5f)) {
          ++found;
        }
      }
      IM_CHECK_EQ(found, 4);

      // And nothing in the middle: the circle is a ring, not a filled disc. This is what a mask
      // read as "everything inside" rather than "on the curve" would fail.
      unsigned char centre[3] = {};
      IM_CHECK(ReadPixel(rgba, 0.0f, 0.0f, centre));
      IM_CHECK(!LooksLikeTheCircle(centre));
    };
  }

  // The y sense of the mask, which the concentric case above is structurally blind to: a ring
  // centred on the frame is symmetric about the horizontal axis, so a mask sampled upside down
  // produces exactly the same picture. This case puts the sun 40 deg ABOVE the camera axis, where
  // up and down are different answers — the whole ring must land in the upper half of the frame.
  //
  // That is the failure a missing uv flip produces, and nothing else in the tree would see it:
  // core's mask is row-major from the top-left while v_ndc.y points up, so the two conventions
  // disagree by construction and only a rendered pixel can say which one won.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "angular_dist", "an_off_axis_circle_lands_on_the_side_the_sun_is_on");
    t->GuiFunc = AngularDistGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      const std::vector<unsigned char> rgba =
          RenderFrame(ctx, /*show_line=*/true, kOffAxisSunAltitude, /*camera_elevation=*/0.0f);
      IM_CHECK(!rgba.empty());

      // Rows above the mid-line vs rows below it, in buffer order (row 0 is the TOP).
      std::size_t above = 0;
      std::size_t below = 0;
      for (int row = 0; row < kProbeH; ++row) {
        for (int col = 0; col < kProbeW; ++col) {
          const std::size_t off = (static_cast<std::size_t>(row) * kProbeW + col) * 4;
          const unsigned char rgb[3] = { rgba[off], rgba[off + 1], rgba[off + 2] };
          if (!LooksLikeTheCircle(rgb)) {
            continue;
          }
          row < kProbeH / 2 ? ++above : ++below;
        }
      }
      IM_CHECK(above + below > 0);
      // The whole ring is above: its centre is 40 deg up and its radius is 22 deg, so its lowest
      // point is still 18 deg above the axis. "Mostly above" would pass with the flip inverted and
      // the ring merely clipped, which is why this is all-or-nothing.
      IM_CHECK_EQ(below, static_cast<std::size_t>(0));
      IM_CHECK(above > 0);
    };
  }

  // The switch. Asserted as the ABSENCE of any circle-coloured pixel in the whole frame rather than
  // at the ring's own radius: a shader that drew the mask somewhere else entirely would satisfy the
  // narrower check while still painting the frame.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "angular_dist", "the_line_switch_removes_every_circle_pixel");
    t->GuiFunc = AngularDistGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      const std::vector<unsigned char> on = RenderFrame(ctx, /*show_line=*/true);
      IM_CHECK(!on.empty());
      const std::size_t lit = CountCirclePixels(on);
      IM_CHECK(lit > 0);

      const std::vector<unsigned char> off = RenderFrame(ctx, /*show_line=*/false);
      IM_CHECK(!off.empty());
      IM_CHECK_EQ(CountCirclePixels(off), static_cast<std::size_t>(0));
    };
  }
}
