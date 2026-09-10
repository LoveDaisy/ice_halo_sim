// The auxiliary lines FOLLOW the view, frame by frame, through the live preview path.
//
// The proposition. While the camera is being dragged, `RenderPreviewPanel` republishes the view
// every frame and the picture re-projects every frame — and the horizon, the grid, the circles and
// the markers must move WITH it, on the same frame, not catch up once the drag stops. This suite
// drives the production per-frame path (`gui::g_state.renderer` is what the mouse drag writes, and
// `RenderPreviewPanel` is what reads it) and reads the DEFAULT framebuffer back after each frame,
// so what is asserted is the pixels the user sees, one frame at a time.
//
// Why a functional case and not a reference image. A committed image is one frame; the defect this
// guards is a relationship BETWEEN frames — an annotation drawn from a stale answer while the frame
// around it is fresh. The only instrument that sees it is a per-frame readback with a per-frame
// analytic prediction beside it, which is what `HorizonRowAt` / `SunMarkerCentreAt` supply.
//
// The prediction is the GUI's own forward projection (`ProjectWorldDirToScreen`), which is the
// mirror the label placement and mouse interaction already run on. It is independent of the thing
// under test in the way that matters: it is a projection of a KNOWN direction, not a reading of
// whatever the annotation machinery happened to compute.
//
// Capture path: the g_fullframe_capture sub-region hook over the preview viewport rectangle
// `g_preview_vp` publishes (device pixels, bottom-left origin, exactly what glReadPixels wants), the
// same protocol test/gui/parity/test_gui_preview_export_parity.cpp reads the screen arm through.
// The pixels come back TOP-DOWN (ReadbackGlRegionToRgba flips them), so row 0 is the top of the
// viewport; the two predictors below convert the shader's y-up answer into that space. The frame
// timing — which view a captured frame was rendered with — is spelled out at
// CaptureViewportThenWrite, because it is the one thing about this harness that is easy to get
// wrong by one frame.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <vector>

#include "gui/app.hpp"
#include "gui/gui_constants.hpp"
#include "gui/gui_state.hpp"
#include "gui/preview_renderer.hpp"
#include "test_gui_shared.hpp"

namespace {

// The sweep: one elevation per frame, every frame different. Twelve frames — more than the three
// the old debounce waited for, so a "recomputes after N still frames" scheme cannot pass by
// accident, and far more than one so a single lucky frame cannot either.
constexpr int kSweepFrames = 12;
constexpr float kSweepStartDeg = -22.0f;
constexpr float kSweepStepDeg = 4.0f;

// A wide single lens looking at the horizon, so the horizon crosses the frame's central column
// at a row the elevation alone decides, and every elevation in the sweep keeps it on screen.
constexpr int kLens = gui::kLensTypeFisheyeEqualArea;
constexpr float kFov = 120.0f;
constexpr float kSunAltitude = 30.0f;

// Every family gets a colour of its own so a channel test names the family rather than "some
// annotation". The horizon is the one asserted on; the others are ON because the proposition is
// about the whole overlay, and a family left off is a family whose per-frame cost this case would
// not be paying. Magenta for the marker keeps its ring out of the red test (its blue channel is as
// high as its red), so the two families cannot be confused near the horizon.
constexpr std::array<float, 3> kHorizonColor = { 1.0f, 0.0f, 0.0f };
constexpr std::array<float, 3> kGridColor = { 0.0f, 0.0f, 1.0f };
constexpr std::array<float, 3> kCirclesColor = { 0.0f, 1.0f, 0.0f };
constexpr std::array<float, 3> kMarkerColor = { 1.0f, 0.0f, 1.0f };

// Tolerance between the predicted row and the nearest horizon pixel. The line is antialiased over
// about a pixel and a half either side of the curve, and the prediction is evaluated at the
// column's centre while the curve crosses the column at a slight angle; 2 px absorbs both and is
// far too narrow to find a line that is a frame or more behind (one sweep step is ~30 px here).
constexpr int kTolerancePx = 2;

// The marker ring: its centre is predicted, and the ring is looked for on an annulus of that
// radius around it. Radius is the GuiState default the panel writes into the shader.
constexpr float kRingSearchHalfWidthPx = 2.5f;

bool g_upload_requested = false;
bool g_upload_done = false;

// A texture is the precondition for the preview to be drawn at all (RenderPreviewPanel gates its
// whole viewport publication on `HasTexture() || HasBackground()`), and BLACK is the content that
// makes "a coloured pixel is an annotation" unambiguous. A GL call belongs on the render thread,
// hence the GuiFunc marshalling.
void TrackingGuiFunc(ImGuiTestContext* /*ctx*/) {
  if (g_upload_requested && !g_upload_done) {
    static const std::vector<unsigned char> kBlack(static_cast<size_t>(4) * 2 * 3, 0);
    gui::g_preview.UploadTexture(kBlack.data(), 4, 2);
    g_upload_done = true;
  }
}

void EnsureBlackTexture(ImGuiTestContext* ctx) {
  g_upload_requested = true;
  g_upload_done = false;
  for (int i = 0; i < 60 && !g_upload_done; ++i) {
    ctx->Yield();
  }
}

void InstallScene(gui::GuiState& s) {
  s.renderer.lens_type = kLens;
  s.renderer.fov = kFov;
  s.renderer.visible = gui::kVisibleFull;
  s.renderer.front = false;
  s.renderer.elevation = kSweepStartDeg;
  s.renderer.azimuth = 0.0f;
  s.renderer.roll = 0.0f;
  s.sun.altitude = kSunAltitude;

  s.show_horizon_line = true;
  s.show_horizon_label = false;
  std::copy(kHorizonColor.begin(), kHorizonColor.end(), std::begin(s.horizon_color));
  s.horizon_alpha = 1.0f;

  s.show_grid_line = true;
  s.show_grid_label = false;
  std::copy(kGridColor.begin(), kGridColor.end(), std::begin(s.grid_color));
  s.grid_alpha = 1.0f;

  s.show_sun_circles_line = true;
  s.show_sun_circles_label = false;
  s.sun_circle_angles = { 22.0f, 46.0f };
  std::copy(kCirclesColor.begin(), kCirclesColor.end(), std::begin(s.sun_circles_color));
  s.sun_circles_alpha = 1.0f;

  for (gui::MarkerAppearance& m : s.markers) {
    m.show = false;
    m.label = false;
  }
  s.markers[LUMICE_ANNOTATION_MARKER_SUN].show = true;
  std::copy(kMarkerColor.begin(), kMarkerColor.end(), std::begin(s.markers[LUMICE_ANNOTATION_MARKER_SUN].color));
  s.markers_alpha = 1.0f;

  s.show_lens_border_line = false;
}

// One frame's worth of the preview viewport, read back off the default framebuffer AFTER that
// frame's render pass, rows top-down, together with the view that frame was rendered with.
struct Frame {
  std::vector<unsigned char> rgba;
  int w = 0;
  int h = 0;
  gui::ViewProjection view;
};

// WHEN THIS RUNS, relative to the frame. A TestFunc is resumed by the test engine at
// ImGui::EndFrame — after every panel of the current frame has run, and before that frame's render
// pass. So at the moment this is called, RenderPreviewPanel has ALREADY published `g_preview_vp`
// for the frame about to be drawn and captured, and a write to `g_state.renderer` made here lands
// in the NEXT frame, not this one. The snapshot of `view` is therefore taken here, before the
// yield, and it is the view the captured pixels were rendered with; `next_elevation` is written
// after the snapshot so the following frame sees a new view, which is what keeps every frame of
// the sweep different from the one before it.
//
// Returns false if the hook did not answer within a few frames.
bool CaptureViewportThenWrite(ImGuiTestContext* ctx, float next_elevation, Frame* out) {
  const int vp_x = gui::g_preview_vp.vp_x;
  const int vp_y = gui::g_preview_vp.vp_y;
  const int vp_w = gui::g_preview_vp.vp_w;
  const int vp_h = gui::g_preview_vp.vp_h;
  if (!gui::g_preview_vp.active || vp_w <= 0 || vp_h <= 0) {
    return false;
  }
  out->view = gui::g_preview_vp.params.view_proj;
  g_fullframe_capture.Reset();
  g_fullframe_capture.rect_x = vp_x;
  g_fullframe_capture.rect_y = vp_y;
  g_fullframe_capture.rect_w = vp_w;
  g_fullframe_capture.rect_h = vp_h;
  g_fullframe_capture.requested.store(true);
  gui::g_state.renderer.elevation = next_elevation;
  // ONE yield is the expectation: the request is consumed after this frame's render pass. The loop
  // bound is a safety net, not the mechanism.
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

// Top-down rows, like the capture.
const unsigned char* PixelAt(const Frame& f, int col, int row) {
  return &f.rgba[(static_cast<size_t>(row) * static_cast<size_t>(f.w) + static_cast<size_t>(col)) * 4];
}

bool LooksLikeTheHorizon(const unsigned char* rgb) {
  return rgb[0] > 150 && rgb[1] < 80 && rgb[2] < 80;
}

bool LooksLikeTheMarker(const unsigned char* rgb) {
  return rgb[0] > 150 && rgb[1] < 80 && rgb[2] > 150;
}

// Where the horizon crosses the central column, as a top-down capture row, for the view a frame
// was rendered with. The direction straight ahead at altitude 0 and the view's
// azimuth is on the horizon by definition; with roll 0 it lands on the centre column.
float HorizonRowAt(const gui::ViewProjection& view, int vp_w, int vp_h) {
  constexpr float kDeg2Rad = 3.14159265358979323846f / 180.0f;
  const float az = view.azimuth * kDeg2Rad;
  // Direction light travels, the convention every annotation direction uses: altitude = asin(-z),
  // azimuth = atan2(-y, -x). Altitude 0 at the view's azimuth.
  const float dir[3] = { -std::cos(az), -std::sin(az), 0.0f };
  const std::array<float, 2> p = gui::ProjectWorldDirToScreen(view, dir, vp_w, vp_h);
  // Centre-origin, y-up pixels -> top-down row.
  return static_cast<float>(vp_h) * 0.5f - p[1];
}

// The sun marker's centre, in the same top-down capture space.
std::array<float, 2> SunMarkerCentreAt(const gui::ViewProjection& view, int vp_w, int vp_h) {
  constexpr float kDeg2Rad = 3.14159265358979323846f / 180.0f;
  const float sa = kSunAltitude * kDeg2Rad;
  const float dir[3] = { -std::cos(sa), 0.0f, -std::sin(sa) };  // GuiSunWorldDir at azimuth 0
  const std::array<float, 2> p = gui::ProjectWorldDirToScreen(view, dir, vp_w, vp_h);
  return { static_cast<float>(vp_w) * 0.5f + p[0], static_cast<float>(vp_h) * 0.5f - p[1] };
}

// The horizon-coloured row nearest to `expect_row` on the central column, or -1 if the column
// carries no horizon pixel at all.
int NearestHorizonRow(const Frame& f, float expect_row) {
  const int col = f.w / 2;
  int best = -1;
  float best_d = 1e9f;
  for (int row = 0; row < f.h; ++row) {
    if (!LooksLikeTheHorizon(PixelAt(f, col, row))) {
      continue;
    }
    const float d = std::fabs(static_cast<float>(row) + 0.5f - expect_row);
    if (d < best_d) {
      best_d = d;
      best = row;
    }
  }
  return best;
}

// Marker-coloured pixels on the annulus of the ring's radius around `centre`.
int CountRingPixelsAround(const Frame& f, const std::array<float, 2>& centre, float radius_px) {
  int n = 0;
  const int r_lo = std::max(0, static_cast<int>(std::floor(centre[1] - radius_px - kRingSearchHalfWidthPx)));
  const int r_hi = std::min(f.h - 1, static_cast<int>(std::ceil(centre[1] + radius_px + kRingSearchHalfWidthPx)));
  const int c_lo = std::max(0, static_cast<int>(std::floor(centre[0] - radius_px - kRingSearchHalfWidthPx)));
  const int c_hi = std::min(f.w - 1, static_cast<int>(std::ceil(centre[0] + radius_px + kRingSearchHalfWidthPx)));
  for (int row = r_lo; row <= r_hi; ++row) {
    for (int col = c_lo; col <= c_hi; ++col) {
      const float dx = static_cast<float>(col) + 0.5f - centre[0];
      const float dy = static_cast<float>(row) + 0.5f - centre[1];
      const float d = std::sqrt(dx * dx + dy * dy);
      if (std::fabs(d - radius_px) > kRingSearchHalfWidthPx) {
        continue;
      }
      if (LooksLikeTheMarker(PixelAt(f, col, row))) {
        ++n;
      }
    }
  }
  return n;
}

float SweepElevation(int k) {
  return kSweepStartDeg + kSweepStepDeg * static_cast<float>(k);
}

// The scene, installed and drawn once, with the cursor parked off the preview so no hover chrome
// lands in the captured rectangle.
void SetUpScene(ImGuiTestContext* ctx) {
  ResetTestState();
  ctx->Yield(2);
  EnsureBlackTexture(ctx);
  InstallScene(gui::g_state);
  ctx->MouseMoveToPos(ImVec2(-100.0f, -100.0f));
  // Let the viewport be published and the starting view be drawn — including by any consumer that
  // needs a run of still frames before it answers. What the sweep then asserts is what happens on
  // the frames AFTER this, when the view changes on every one of them.
  ctx->Yield(6);
}

// One sweep step: capture the frame that carries view k, and hand the next view to the frame after
// it. Every check reports and returns rather than asserting, so a failure on one frame leaves the
// rest of the sweep to run and be reported — the per-frame log is the evidence, and a sweep that
// stopped at its first bad frame would hide how many there were.
bool CaptureSweepFrame(ImGuiTestContext* ctx, int k, Frame* f) {
  if (!CaptureViewportThenWrite(ctx, SweepElevation(k + 1), f)) {
    fprintf(stderr, "[annotation_tracking] frame %2d: the capture hook did not answer\n", k);
    return false;
  }
  // The frame was rendered with view k — the premise of comparing its pixels against a prediction
  // from that view. View 0 is what SetUpScene installed; every later one is what the previous
  // step wrote after its own snapshot.
  if (std::fabs(f->view.elevation - SweepElevation(k)) > 1e-4f) {
    fprintf(stderr, "[annotation_tracking] frame %2d: rendered with elevation %.3f, not the expected %.3f\n", k,
            f->view.elevation, SweepElevation(k));
    return false;
  }
  return true;
}

// True when frame k shows the horizon where its own view puts it.
bool HorizonFrameOk(ImGuiTestContext* ctx, int k) {
  Frame f;
  if (!CaptureSweepFrame(ctx, k, &f)) {
    return false;
  }
  const float expect_row = HorizonRowAt(f.view, f.w, f.h);
  if (!(expect_row > 4.0f && expect_row < static_cast<float>(f.h) - 4.0f)) {
    fprintf(stderr, "[annotation_tracking] frame %2d: predicted row %.2f is off the viewport; the sweep is mis-sized\n",
            k, expect_row);
    return false;
  }
  const int found = NearestHorizonRow(f, expect_row);
  const float err = found < 0 ? -1.0f : std::fabs(static_cast<float>(found) + 0.5f - expect_row);
  fprintf(stderr, "[annotation_tracking] frame %2d elevation %6.1f expect_row %7.2f found_row %4d err %5.1f\n", k,
          SweepElevation(k), expect_row, found, err);
  return found >= 0 && err <= static_cast<float>(kTolerancePx);
}

// True when frame k shows the sun marker's ring around the centre its own view puts it at.
bool SunMarkerFrameOk(ImGuiTestContext* ctx, int k, float radius) {
  Frame f;
  if (!CaptureSweepFrame(ctx, k, &f)) {
    return false;
  }
  const std::array<float, 2> centre = SunMarkerCentreAt(f.view, f.w, f.h);
  if (!(centre[0] > radius + 4.0f && centre[0] < static_cast<float>(f.w) - radius - 4.0f && centre[1] > radius + 4.0f &&
        centre[1] < static_cast<float>(f.h) - radius - 4.0f)) {
    fprintf(stderr, "[annotation_tracking] frame %2d: predicted ring (%.1f, %.1f) is off the viewport\n", k, centre[0],
            centre[1]);
    return false;
  }
  const int on_ring = CountRingPixelsAround(f, centre, radius);
  // A ring of this radius is ~50 px around; a handful of hits would be a stray, and a stale ring
  // one step behind lands entirely outside the annulus (the step is ~30 px, the search band
  // 2.5 px).
  fprintf(stderr, "[annotation_tracking] frame %2d elevation %6.1f ring centre (%.1f, %.1f) hits %d\n", k,
          SweepElevation(k), centre[0], centre[1], on_ring);
  return on_ring >= 20;
}

}  // namespace

void RegisterAnnotationLineTrackingTests(ImGuiTestEngine* engine) {
  // The horizon line, on twelve consecutive frames with a different elevation on each: every frame
  // must show it where THAT frame's view puts it. This is the case that reads red on a preview whose
  // curves are computed for a settled view and frozen during a drag, and green on one that
  // evaluates them per frame.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "annotation_tracking", "the_horizon_follows_the_view_on_every_frame");
    t->GuiFunc = TrackingGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      SetUpScene(ctx);
      IM_CHECK(gui::g_preview_vp.active);

      int frames_off = 0;
      for (int k = 0; k < kSweepFrames; ++k) {
        if (!HorizonFrameOk(ctx, k)) {
          ++frames_off;
        }
      }
      IM_CHECK_EQ(frames_off, 0);
    };
  }

  // The reference-point markers come from core's anchors rather than from the fragment shader's
  // own arithmetic, so they are the other half of the same proposition: the anchors, too, must be
  // this frame's. Same sweep, read at the sun marker's predicted ring.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "annotation_tracking", "the_sun_marker_follows_the_view_on_every_frame");
    t->GuiFunc = TrackingGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      SetUpScene(ctx);
      IM_CHECK(gui::g_preview_vp.active);
      const float radius = gui::g_state.markers_radius_px;

      int frames_off = 0;
      for (int k = 0; k < kSweepFrames; ++k) {
        if (!SunMarkerFrameOk(ctx, k, radius)) {
          ++frames_off;
        }
      }
      IM_CHECK_EQ(frames_off, 0);
    };
  }
}
