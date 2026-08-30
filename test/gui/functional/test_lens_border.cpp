// The lens border, in rendered pixels: which lenses draw one, and where it survives.
//
// What this suite is for. The border is drawn by the fragment shader, from a radius the shader
// derives itself — so the CPU-side formula asserted in unit-correctness/gui/test_lens_border_geometry
// .cpp proves the arithmetic and proves nothing about whether the GLSL beside it says the same
// thing. These cases close that gap by reading pixels: they predict a screen coordinate from the
// CPU formula and demand the border actually be there, and demand its absence where the classifier
// says there is no border at all.
//
// Two claims, and they fail for different reasons:
//   * A lens outside LensHasBorder() must render byte-identical with the switch on and off. A shader
//     branch that let, say, `linear` through would draw a circle over a projection that has no
//     boundary — and nothing else in the suite would notice, because the border is off by default.
//   * The border must survive the visibility gate. It is the lens's own image circle, an optical
//     property, so it stays whole no matter which half-sky `visible` keeps. The shader draws it
//     outside the `result.w >= 0.5 && pixel_visible` test for exactly that reason, and this is the
//     case that holds that placement in place: move the call back inside the gate and the discarded
//     half of the circle disappears.
//
// Capture path: RenderExportToRgba's own off-screen FBO, the same one the lens_proj references use,
// so nothing here depends on window size or panel layout. No simulation is run — the source texture
// stays empty, which makes the frame black everywhere the border is not, and a coloured pixel
// therefore unambiguous.

#include <cmath>
#include <cstddef>
#include <iterator>
#include <string>
#include <vector>

#include "gui/export_fbo_renderer.hpp"
#include "gui/gui_constants.hpp"
#include "test_gui_shared.hpp"

namespace {

constexpr int kProbeW = 256;
constexpr int kProbeH = 256;
constexpr float kPi = 3.14159265358979323846f;

// The border colour these cases render with. Deliberately not the GuiState default: a saturated
// primary is separable from any anti-aliasing blend against black by a single channel test, and it
// cannot be confused with a default that later changes.
constexpr float kBorderR = 1.0f;
constexpr float kBorderG = 0.0f;
constexpr float kBorderB = 0.0f;

// A render request marshalled to the frame loop. RenderExportToRgba must run on the thread that
// owns the GL context, which is the render thread, not the test coroutine — the same crossing
// test_file_ops.cpp makes for its pixel probes.
struct RenderRequest {
  bool requested = false;
  bool done = false;

  int lens_type = 0;
  float fov = 180.0f;
  int visible = 0;
  bool show_border = false;

  std::vector<unsigned char> rgba;

  void Reset() { *this = RenderRequest{}; }
};

RenderRequest g_req;

void RunRenderRequest() {
  // Blank the source texture: whatever ran before this case left one behind, and a coloured sample
  // showing through would make "the border is the only coloured thing here" false. Deferred to the
  // next Render() by design (preview_renderer.hpp), which is the RenderExportToRgba below.
  gui::g_preview.ClearTexture();

  gui::PreviewParams params{};
  params.view_proj.lens_type = g_req.lens_type;
  params.view_proj.fov = g_req.fov;
  params.view_proj.visible = g_req.visible;
  params.view_proj.elevation = 0.0f;
  params.view_proj.azimuth = 0.0f;
  params.view_proj.roll = 0.0f;
  params.source.max_abs_dz = gui::kDualFisheyeOverlap;
  params.source.r_scale = 1.0f / std::sqrt(1.0f + gui::kDualFisheyeOverlap);
  params.exposure.intensity_factor = 1.0f;
  params.exposure.intensity_scale = 0.0f;  // 8-bit RGB mode; no simulation has been run

  params.overlay.show_lens_border = g_req.show_border;
  params.overlay.lens_border_color[0] = kBorderR;
  params.overlay.lens_border_color[1] = kBorderG;
  params.overlay.lens_border_color[2] = kBorderB;
  params.overlay.lens_border_alpha = 1.0f;  // fully opaque: the pixel test reads a colour, not a blend

  g_req.rgba = gui::RenderExportToRgba(gui::g_preview, params, kProbeW, kProbeH, std::nullopt);
  g_req.done = true;
  g_req.requested = false;
}

void LensBorderGuiFunc(ImGuiTestContext* /*ctx*/) {
  if (g_req.requested && !g_req.done) {
    RunRenderRequest();
  }
}

// Ask the render thread for one frame and wait for it. Bounded so a wiring regression fails at the
// assertion below rather than hanging the suite.
std::vector<unsigned char> RenderFrame(ImGuiTestContext* ctx, int lens_type, float fov, int visible, bool show_border) {
  g_req.Reset();
  g_req.lens_type = lens_type;
  g_req.fov = fov;
  g_req.visible = visible;
  g_req.show_border = show_border;
  g_req.requested = true;
  for (int i = 0; i < 60 && !g_req.done; ++i) {
    ctx->Yield();
  }
  return g_req.rgba;
}

// Pixel at center-origin, y-up coordinates — the convention the shader's `pos` uses, so a
// coordinate predicted from the border formula can be handed straight in. The buffer itself is
// row-major top-down.
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

// Scan a short radial segment for the border. The circle is anti-aliased over ~3 px and the row/col
// rounding above can land a whole pixel off, so demanding the border at one exact coordinate would
// be asserting the rasteriser's rounding rather than the border's position. A window of ±3 px is
// wide enough to absorb that and far too narrow to find a border that is elsewhere.
bool BorderFoundNear(const std::vector<unsigned char>& rgba, float cx, float cy, float radius, float phi) {
  const float ux = std::cos(phi);
  const float uy = std::sin(phi);
  for (int d = -3; d <= 3; ++d) {
    unsigned char rgb[3] = {};
    const float r = radius + static_cast<float>(d);
    if (!ReadPixel(rgba, cx + ux * r, cy + uy * r, rgb)) {
      continue;
    }
    // Red channel well above black, and the other two still near it: the border colour, not a
    // texture sample. Nothing else in this frame is coloured — the source texture is empty.
    if (rgb[0] > 100 && rgb[1] < 80 && rgb[2] < 80) {
      return true;
    }
  }
  return false;
}

}  // namespace

void RegisterLensBorderTests(ImGuiTestEngine* engine) {
  // AC2. Byte-for-byte, not "looks the same": the claim is that the shader takes no branch at all
  // for these lenses, and any difference — one anti-aliased pixel at one azimuth — falsifies it.
  // Two representatives rather than one, because they reach the border function through different
  // shader paths: `linear` is view-transformed and `rectangular` is full-sky, and a branch written
  // in terms of the wrong one of those two groupings would leak through exactly one of them.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "lens_border", "a_lens_without_a_border_renders_identically_either_way");
    t->GuiFunc = LensBorderGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      struct Row {
        int lens_type;
        float fov;
        const char* name;
      };
      const Row kRows[] = {
        { gui::kLensTypeLinear, 90.0f, "linear" },
        { gui::kLensTypeFisheyeStereographic, 180.0f, "fisheye stereographic" },
        { gui::kLensTypeRectangular, 90.0f, "rectangular" },
        { gui::kLensTypeGlobe, 30.0f, "globe" },
      };
      for (const Row& row : kRows) {
        // The classifier and the pixels must agree; asserting only the pixels would pass on a
        // classifier that had quietly gained a member the shader ignores. Reported non-fatally:
        // a fatal assert here would return out of the whole case and leave the remaining lenses
        // unexamined, and which lenses leaked is the diagnostic.
        if (gui::LensHasBorder(row.lens_type)) {
          IM_ERRORF("%s: LensHasBorder() claims this lens has a border, but this case says it has none", row.name);
          continue;
        }

        const std::vector<unsigned char> off = RenderFrame(ctx, row.lens_type, row.fov, gui::kVisibleFull, false);
        const std::vector<unsigned char> on = RenderFrame(ctx, row.lens_type, row.fov, gui::kVisibleFull, true);
        if (off.empty() || on.empty()) {
          // Non-fatal, and then stop driving: a failed readback makes every comparison below it
          // meaningless, so continuing would cascade reds off one root cause.
          IM_ERRORF("%s: the off-screen render produced no pixels", row.name);
          break;
        }
        if (off != on) {
          IM_ERRORF("%s: turning the lens border on changed the frame, but this lens has no border", row.name);
          break;
        }
      }
    };
  }

  // AC1's pixel half + AC3, over all three single-lens families that carry a border. One case,
  // because they are one render per family: with visible=upper the shader discards the lower
  // half-sky, so the SAME frame answers "is the border where the arithmetic says" (upper azimuths)
  // and "does it survive the visibility gate" (lower azimuths).
  //
  // This is what stops a wrong coefficient in the GLSL from passing. The unit-correctness sibling
  // proves the ARITHMETIC against an independent inverse; it reads no pixels, so on its own it is
  // satisfied by a shader that computes something else entirely. Measured: changing the equal-area
  // branch from sin(half_fov/2) to sin(half_fov) in the shader left every other assertion in this
  // task green — these rows are what turns that red.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "lens_border", "the_border_is_drawn_whole_through_the_visibility_gate");
    t->GuiFunc = LensBorderGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      struct Row {
        int lens_type;
        float fov;
        float radius_px;
        const char* name;
      };
      // radius_px is spelled as a NUMBER, not recomputed from the shader's formula, so a reader can
      // check it by hand and so a shared helper cannot make this case agree with the shader by
      // construction. img_radius is min(w,h)/2 = 128 px at this probe size.
      //
      //   equal area   fov=200: 1 / sin(50°)  = 1.30541 img radii -> 167.09 px
      //   equidistant  fov=270: pi / (135°)   = 1.33333 img radii -> 170.67 px
      //   orthographic fov=180: 1 / sin(90°)  = 1.00000 img radii -> 128.00 px
      //
      // The FOVs are chosen so the three radii separate: each one is more than the search window
      // away from what the OTHER two formulas would give for the same lens, so a coefficient copied
      // between branches lands outside the window and is not found.
      const Row kRows[] = {
        { gui::kLensTypeFisheyeEqualArea, 200.0f, 167.09f, "fisheye equal-area fov=200" },
        { gui::kLensTypeFisheyeEquidist, 270.0f, 170.67f, "fisheye equidistant fov=270" },
        { gui::kLensTypeFisheyeOrthographic, 180.0f, 128.00f, "fisheye orthographic fov=180" },
      };

      // Diagonal azimuths, not the axes: a circle of radius up to ~181 px is fully addressable at
      // 45° in a 256x256 frame, whereas on the axes the widest of these three circles would run off
      // the edge and the probe would silently find nothing to read.
      const float kUpperPhis[] = { 0.25f * kPi, 0.75f * kPi };
      const float kLowerPhis[] = { -0.25f * kPi, -0.75f * kPi };

      // Every frame is captured FIRST, then every frame is examined. Not a stylistic split: an
      // ImGuiTestContext action opens with `if (IsError()) return;`, so a report raised while
      // examining row 1 would silently turn row 2's render into a no-op and its failures into
      // echoes of row 1's. Rendering up front keeps all three diagnoses independent, which is the
      // point of having three rows.
      std::vector<std::vector<unsigned char>> frames;
      for (const Row& row : kRows) {
        frames.push_back(RenderFrame(ctx, row.lens_type, row.fov, gui::kVisibleUpper, true));
      }

      // Nothing below drives the GUI — these are buffer reads — so every failure is reported
      // non-fatally and examination continues. WHICH azimuths fail is the whole diagnostic: all
      // four means the radius is wrong, only the lower two means the border went back behind the
      // visibility gate.
      for (std::size_t i = 0; i < std::size(kRows); ++i) {
        const Row& row = kRows[i];
        const std::vector<unsigned char>& frame = frames[i];
        if (frame.empty()) {
          IM_ERRORF("%s: the off-screen render produced no pixels", row.name);
          continue;
        }

        for (float phi : kUpperPhis) {
          if (!BorderFoundNear(frame, 0.0f, 0.0f, row.radius_px, phi)) {
            IM_ERRORF("%s: no border at azimuth %.2f rad, radius %.1f px (kept half-sky)", row.name,
                      static_cast<double>(phi), static_cast<double>(row.radius_px));
          }
        }
        for (float phi : kLowerPhis) {
          // Not a repetition of the loop above: this half of the sky is discarded, so every pixel
          // here is black unless the border was drawn outside the visibility gate.
          if (!BorderFoundNear(frame, 0.0f, 0.0f, row.radius_px, phi)) {
            IM_ERRORF(
                "%s: no border at azimuth %.2f rad, radius %.1f px — the DISCARDED half-sky, so "
                "the border is being gated by pixel_visible",
                row.name, static_cast<double>(phi), static_cast<double>(row.radius_px));
          }
        }

        // The border is a CIRCLE, not a fill: a point at the frame centre must stay black. Without
        // this, a shader that painted the whole disc would satisfy every check above.
        unsigned char inner[3] = {};
        if (!ReadPixel(frame, 0.0f, 0.0f, inner)) {
          IM_ERRORF("%s: the frame centre is not readable", row.name);
        } else if (inner[0] >= 40) {
          IM_ERRORF("%s: the frame centre reads r=%d — the border is filling the disc, not tracing it", row.name,
                    (int)inner[0]);
        }
      }
    };
  }

  // The dual-fisheye pair, whose radius comes from the hard circle clip rather than a FOV-dependent
  // formula — the one lens family where a FOV-derived radius would still look plausible on screen.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "lens_border", "the_dual_fisheye_border_traces_both_clip_circles");
    t->GuiFunc = LensBorderGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // circle_radius = min(w/2, h)/2, and this frame is square, so it is w/4 = h/4 — the two
      // circles sit side by side across the middle of the frame. fov is passed as 180 and must not
      // matter: dualFisheyeInverse clips before it reads a type or a FOV.
      constexpr float kCircleR = kProbeW * 0.25f;
      const std::vector<unsigned char> frame =
          RenderFrame(ctx, gui::kLensTypeDualFisheyeEqualArea, 180.0f, gui::kVisibleFull, true);
      IM_CHECK(!frame.empty());

      // Straight up and straight down from each centre: on the circle, and far from the point where
      // the two circles touch, so neither sample can be satisfied by the other circle's outline.
      const float kPhis[] = { 0.5f * kPi, -0.5f * kPi };
      // Non-fatal for the reason the case above gives: the frame is captured, nothing here drives
      // the GUI, and "the left circle is there but the right one is not" is a different defect from
      // "neither is".
      for (float phi : kPhis) {
        if (!BorderFoundNear(frame, -kCircleR, 0.0f, kCircleR, phi)) {
          IM_ERRORF("left clip circle: no border at azimuth %.2f rad", static_cast<double>(phi));
        }
        if (!BorderFoundNear(frame, kCircleR, 0.0f, kCircleR, phi)) {
          IM_ERRORF("right clip circle: no border at azimuth %.2f rad", static_cast<double>(phi));
        }
      }
    };
  }
}
