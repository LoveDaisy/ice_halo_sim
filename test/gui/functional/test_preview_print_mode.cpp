// The subtractive (print) operator, in rendered pixels.
//
// Why this suite has to exist at all, and why the CPU unit tests next door are not enough. The
// preview's fragment shader is a THIRD implementation of the ink curve, beside
// src/util/ink_transfer.hpp and its use in src/server/render.cpp — not the same code running
// somewhere else, but a hand transcription into GLSL, which cannot #include a C++ header. Nothing
// in the build checks that the two agree. This repo has already paid for that once: a change was
// held only by "CPU formula versus CPU oracle" cross-checks, and a red-state probe that corrupted a
// shader coefficient left every test green. So the comparison here is deliberately made the
// expensive way — render through the real shader, read the pixels back, and predict them by calling
// the C++ function that is the authority.
//
// The four claims that are only checkable this way:
//
//   * the curve itself. A pixel of known exposure must come back as paper * 10^(-D) with D from
//     lumice::InkOpticalDensity. This is what pins kInkGamma and the log base across the seam.
//   * the paper's hue survives on the GPU, not merely in the C++ arithmetic.
//   * a pixel outside the visibility gate is bare paper, not black. The shader's zero-energy colour
//     is a different line of code from the CLI's, and the GPU side owns cases the CLI has no
//     equivalent for — outside the image circle of a fisheye, in particular.
//   * annotations do not read their colour field under print. Same claim as the CLI's, asserted the
//     same way: change the colour, the bytes must not move.
//
// And one negative claim, which is the reason a green run here means anything: with tone left at
// screen the frame must be byte-for-byte what it was before print existed. That is asserted
// positively, against a rendered screen-mode reference computed in this file, rather than inferred
// from "no other test went red".
//
// Capture path, fixture shape and the reasons for both are inherited from
// test_preview_background.cpp beside it: RenderExportToRgba's own off-screen FBO, no simulation, a
// uniform source texture uploaded directly so every expected value is closed-form.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include "gui/export_fbo_renderer.hpp"
#include "gui/gui_constants.hpp"
#include "test_gui_shared.hpp"
#include "util/color_space.hpp"
#include "util/ink_transfer.hpp"

namespace {

// A warm off-white sheet, three DISTINCT channels and none of them 0 or 1: a dropped or swapped
// channel shows up, and none sits on a clamp boundary where an error would be absorbed.
constexpr float kPaperR = 0.94f;
constexpr float kPaperG = 0.88f;
constexpr float kPaperB = 0.76f;

// The GPU and the C++ oracle both finish with a truncation of `v * 255` after two transcendental
// calls (a pow in the ink curve, another in the sRGB transfer curve), evaluated by two different
// compilers for two different targets. Wherever they land on opposite sides of an integer a whole
// byte of difference appears from a float difference far below it — the same narrowing effect
// test_preview_background.cpp measured at exactly 1 LSB for its own comparison.
//
// 2 rather than that file's 1 because this chain has one more transcendental in it. It is a
// tolerance on the NARROWING only: the failures this suite exists to catch are far away. A wrong
// kInkGamma is 40+ LSB (measured with the red-state probe: 11.0 -> 2.2 moves the mid-exposure probe
// from 96 to 216), and a flipped transmittance exponent saturates the channel outright.
constexpr int kInkToleranceLsb = 2;

// A render request marshalled to the frame loop: both the upload and RenderExportToRgba must run on
// the thread owning the GL context, which is the render thread and not the test coroutine.
struct RenderRequest {
  bool requested = false;
  bool done = false;

  int canvas_w = 256;
  int canvas_h = 256;

  int lens_type = gui::kLensTypeLinear;
  float fov = 90.0f;
  float elevation = 0.0f;
  int visible = gui::kVisibleFull;

  int tone = 1;  // 1 = print; the suite's default, with the screen arm asking for 0 explicitly
  float paper_srgb[3] = { kPaperR, kPaperG, kPaperB };
  // Non-zero and NOT the paper colour: print must ignore the sky entirely, so leaving this at zero
  // would make "ignored" and "added" produce the same pixels.
  float sky_srgb[3] = { 0.2f, 0.35f, 0.6f };

  int tex_size = 64;
  float uniform_xyz[3] = { 0.0f, 0.0f, 0.0f };
  float intensity_scale = 1.0f;

  // Annotation overlays whose colour the print arm varies. Two families rather than one, because
  // they exercise the two DIFFERENT shader functions the tone branch had to be threaded through:
  // the horizon goes through overlayAuxLines() (mask-driven), the lens border through
  // overlayLensBorder() (geometry derived in the shader). A change that fixed only one of them
  // would pass a single-family check.
  bool show_horizon = false;
  bool show_lens_border = false;
  float annotation_color[3] = { 0.8f, 0.2f, 0.2f };

  std::vector<unsigned char> rgba;

  void Reset() { *this = RenderRequest{}; }
};

RenderRequest g_req;

void RunRenderRequest() {
  const int n = g_req.tex_size * g_req.tex_size;
  std::vector<float> xyz(static_cast<std::size_t>(n) * 3);
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < 3; ++j) {
      xyz[static_cast<std::size_t>(i) * 3 + j] = g_req.uniform_xyz[j];
    }
  }
  gui::g_preview.UploadXyzTexture(xyz.data(), g_req.tex_size, g_req.tex_size);

  gui::PreviewParams params{};
  params.view_proj.lens_type = g_req.lens_type;
  params.view_proj.fov = g_req.fov;
  params.view_proj.elevation = g_req.elevation;
  params.view_proj.visible = g_req.visible;
  params.source.max_abs_dz = gui::kDualFisheyeOverlap;
  params.source.r_scale = 1.0f / std::sqrt(1.0f + gui::kDualFisheyeOverlap);
  params.exposure.intensity_factor = 1.0f;
  params.exposure.intensity_scale = g_req.intensity_scale;
  lumice::SrgbToLinearRgb(g_req.sky_srgb, params.background_color_linear);
  lumice::SrgbToLinearRgb(g_req.paper_srgb, params.paper_color_linear);
  params.tone = g_req.tone;

  // The horizon's geometry comes from a mask core would normally supply; a synthetic band is
  // enough and keeps this file free of the annotation cache. Static so it outlives the borrow the
  // params make of it, and the generation is bumped so Render() re-uploads rather than reusing a
  // previous frame's texture.
  static std::vector<unsigned char> horizon_mask;
  static unsigned long long mask_generation = 0;
  if (g_req.show_horizon) {
    horizon_mask.assign(static_cast<std::size_t>(g_req.canvas_w) * g_req.canvas_h, 0);
    const int band_row = g_req.canvas_h / 2;
    for (int row = band_row - 1; row <= band_row + 1; ++row) {
      for (int col = 0; col < g_req.canvas_w; ++col) {
        horizon_mask[static_cast<std::size_t>(row) * g_req.canvas_w + col] = 1;
      }
    }
    params.overlay.show_horizon = true;
    params.overlay.horizon_alpha = 0.6f;
    params.overlay.horizon_mask = horizon_mask.data();
    params.overlay.horizon_mask_w = g_req.canvas_w;
    params.overlay.horizon_mask_h = g_req.canvas_h;
    params.overlay.horizon_mask_generation = ++mask_generation;
    std::copy(std::begin(g_req.annotation_color), std::end(g_req.annotation_color),
              std::begin(params.overlay.horizon_color));
  }
  if (g_req.show_lens_border) {
    params.overlay.show_lens_border = true;
    params.overlay.lens_border_alpha = 0.6f;
    std::copy(std::begin(g_req.annotation_color), std::end(g_req.annotation_color),
              std::begin(params.overlay.lens_border_color));
  }

  g_req.rgba = gui::RenderExportToRgba(gui::g_preview, params, g_req.canvas_w, g_req.canvas_h);
  g_req.done = true;
  g_req.requested = false;
}

// Runs on the render thread every frame; the TestFunc below is what drives ctx (RenderFrame yields
// until this has serviced the request). Same split as PreviewBackgroundGuiFunc next door.
void PreviewPrintModeGuiFunc(ImGuiTestContext* /*ctx*/) {
  if (g_req.requested && !g_req.done) {
    RunRenderRequest();
  }
}

// Ask the render thread for one frame and wait for it. Bounded so a wiring regression fails at an
// assertion rather than hanging the suite.
void RenderFrame(ImGuiTestContext* ctx) {
  g_req.requested = true;
  g_req.done = false;
  for (int i = 0; i < 60 && !g_req.done; ++i) {
    ctx->Yield();
  }
}

// Pixel at center-origin, y-up — the convention the shader's `pos` uses, so a coordinate predicted
// from a projection formula can be handed straight in. The buffer is row-major top-down, RGBA.
bool ReadPixel(const std::vector<unsigned char>& rgba, int w, int h, float pos_x, float pos_y, unsigned char* out_rgb) {
  const int col = static_cast<int>(std::lround(pos_x + w * 0.5f));
  const int row = static_cast<int>(std::lround(h * 0.5f - pos_y));
  if (col < 0 || col >= w || row < 0 || row >= h) {
    return false;
  }
  const std::size_t off = (static_cast<std::size_t>(row) * w + col) * 4;
  out_rgb[0] = rgba[off + 0];
  out_rgb[1] = rgba[off + 1];
  out_rgb[2] = rgba[off + 2];
  return true;
}

// The bytes the C++ authority says a pixel of exposure `e` must come out as. This is the whole
// point of the file: `lumice::InkOpticalDensity` / `InkTransmittance` are the same functions the
// CLI calls, and the shader is a transcription of them that nothing else compares against.
//
// `e` is CIE Y after the intensity scale. The lens's relative illumination is 1 at the axis, so the
// probes below all sit at or near the frame centre where this prediction needs no Jacobian.
void ExpectedInkBytes(float e, int out_rgb[3]) {
  const float paper_srgb[3] = { kPaperR, kPaperG, kPaperB };
  const float transmittance = lumice::InkTransmittance(lumice::InkOpticalDensity(e));
  for (int j = 0; j < 3; ++j) {
    const float paper_linear = lumice::SrgbToLinear(paper_srgb[j]);
    const float v = lumice::LinearToSrgb(std::clamp(paper_linear * transmittance, 0.0f, 1.0f));
    out_rgb[j] = static_cast<int>(v * 255.0f);
  }
}

// Report, non-fatally, if the pixel is not what the C++ curve predicts. Non-fatal throughout this
// file for the reason the sibling suite gives: nothing below a report drives the GUI, and WHICH
// probe failed is the diagnostic — a fatal assert inside a probe list would hide every probe after
// the first.
void ExpectInkAt(const char* tag, float e, float pos_x, float pos_y) {
  unsigned char rgb[3] = {};
  if (!ReadPixel(g_req.rgba, g_req.canvas_w, g_req.canvas_h, pos_x, pos_y, rgb)) {
    IM_ERRORF("%s: pixel (%.1f, %.1f) is outside the %dx%d frame", tag, static_cast<double>(pos_x),
              static_cast<double>(pos_y), g_req.canvas_w, g_req.canvas_h);
    return;
  }
  int expected[3] = {};
  ExpectedInkBytes(e, expected);
  for (int j = 0; j < 3; ++j) {
    if (std::abs(static_cast<int>(rgb[j]) - expected[j]) > kInkToleranceLsb) {
      IM_ERRORF(
          "%s: pixel (%.1f, %.1f) channel %d reads %d, the C++ ink curve says %d for e=%.4f. The "
          "shader's subtractiveInk() is a hand transcription of src/util/ink_transfer.hpp and "
          "nothing but this comparison holds the two in step — check kInkGamma and the log base "
          "before relaxing anything. Full pixel (%d, %d, %d), expected (%d, %d, %d).",
          tag, static_cast<double>(pos_x), static_cast<double>(pos_y), j, (int)rgb[j], expected[j],
          static_cast<double>(e), (int)rgb[0], (int)rgb[1], (int)rgb[2], expected[0], expected[1], expected[2]);
    }
  }
}

// Report, non-fatally, if the pixel is not bare paper (the zero-energy colour under print).
void ExpectPaperAt(const char* tag, float pos_x, float pos_y) {
  unsigned char rgb[3] = {};
  if (!ReadPixel(g_req.rgba, g_req.canvas_w, g_req.canvas_h, pos_x, pos_y, rgb)) {
    IM_ERRORF("%s: pixel (%.1f, %.1f) is outside the %dx%d frame", tag, static_cast<double>(pos_x),
              static_cast<double>(pos_y), g_req.canvas_w, g_req.canvas_h);
    return;
  }
  int expected[3] = {};
  ExpectedInkBytes(0.0f, expected);
  for (int j = 0; j < 3; ++j) {
    if (std::abs(static_cast<int>(rgb[j]) - expected[j]) > kInkToleranceLsb) {
      IM_ERRORF(
          "%s: pixel (%.1f, %.1f) reads (%d, %d, %d), expected bare paper (%d, %d, %d). A reading of "
          "(0, 0, 0) means the shader's zero-energy colour is still black — under print an unexposed "
          "region is the sheet, not a hole.",
          tag, static_cast<double>(pos_x), static_cast<double>(pos_y), (int)rgb[0], (int)rgb[1], (int)rgb[2],
          expected[0], expected[1], expected[2]);
    }
  }
}

std::size_t CountDifferingBytes(const std::vector<unsigned char>& a, const std::vector<unsigned char>& b) {
  std::size_t n = 0;
  for (std::size_t i = 0; i < a.size() && i < b.size(); ++i) {
    if (a[i] != b[i]) {
      ++n;
    }
  }
  return n;
}

}  // namespace

void RegisterPreviewPrintModeTests(ImGuiTestEngine* engine) {
  // The curve across the seam. Three exposures spanning the range a real scene lives in (the
  // non-zero `e` in this project's scenes is 1e-3..1e-1) plus one far above it, each rendered on
  // its own frame and each predicted by the C++ function.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "preview_print_mode", "the_shader_matches_the_cpp_ink_curve");
    t->GuiFunc = PreviewPrintModeGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // A uniform XYZ field whose Y is the exposure under test. X and Z are set to the same value
      // so that a shader reading the wrong component would still be reading SOMETHING — leaving
      // them zero would let `tex_color.x` pass by accident on a black-X field.
      const float kExposures[] = { 0.003f, 0.03f, 0.2f, 2.0f };
      for (float e : kExposures) {
        g_req.Reset();
        g_req.uniform_xyz[0] = e;
        g_req.uniform_xyz[1] = e;
        g_req.uniform_xyz[2] = e;
        RenderFrame(ctx);
        if (g_req.rgba.empty()) {
          IM_ERRORF("e=%.4f: the render produced no pixels", static_cast<double>(e));
          continue;
        }
        // Centre only: relative illumination is 1 on the axis, so the prediction needs no Jacobian
        // there. The off-axis behaviour is the projection's, and it has its own suite.
        ExpectInkAt("ink-curve", e, 0.0f, 0.0f);
      }
    };
  }

  // The paper's hue survives on the GPU. Read as a ratio in linear light rather than on bytes: the
  // transfer curve is not linear, so a byte-space ratio would fail for reasons unrelated to the
  // operator.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "preview_print_mode", "the_paper_keeps_its_hue_under_ink");
    t->GuiFunc = PreviewPrintModeGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      g_req.Reset();
      g_req.uniform_xyz[0] = 0.05f;
      g_req.uniform_xyz[1] = 0.05f;
      g_req.uniform_xyz[2] = 0.05f;
      RenderFrame(ctx);
      IM_CHECK(!g_req.rgba.empty());

      unsigned char rgb[3] = {};
      if (!ReadPixel(g_req.rgba, g_req.canvas_w, g_req.canvas_h, 0.0f, 0.0f, rgb)) {
        IM_ERRORF("hue: the centre pixel is outside the %dx%d frame", g_req.canvas_w, g_req.canvas_h);
        return;
      }
      const float paper_srgb[3] = { kPaperR, kPaperG, kPaperB };
      float linear[3];
      for (int j = 0; j < 3; ++j) {
        linear[j] = lumice::SrgbToLinear(static_cast<float>(rgb[j]) * (1.0f / 255.0f));
      }
      const float t_implied = linear[0] / lumice::SrgbToLinear(paper_srgb[0]);
      for (int j = 1; j < 3; ++j) {
        const float expected = t_implied * lumice::SrgbToLinear(paper_srgb[j]);
        if (std::fabs(linear[j] - expected) > 6e-3f) {
          IM_ERRORF(
              "hue: channel %d is %.4f in linear, but one shared transmittance would put it at "
              "%.4f. Every channel must be the SAME transmittance times its own paper component; "
              "a per-channel density would tint the sheet.",
              j, static_cast<double>(linear[j]), static_cast<double>(expected));
        }
      }
      if (t_implied > 0.99f) {
        IM_ERRORF("hue: the implied transmittance is %.4f — no ink was laid down, so this case checked nothing",
                  static_cast<double>(t_implied));
      }
    };
  }

  // Outside the visibility gate: bare paper, not black. Two regions the GPU owns and the CLI has no
  // equivalent for — beyond a fisheye's image circle, and in the half-sky `visible` discards.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "preview_print_mode", "unimaged_regions_are_bare_paper");
    t->GuiFunc = PreviewPrintModeGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // A 180 deg equal-area fisheye, horizon-centred, visible=upper — so the lower half of the
      // frame is imaged sky the document excludes, a different code path from "no direction at all".
      //
      // The canvas is deliberately WIDER than it is tall. The image radius is min(w, h)/2, but the
      // equal-area inverse's domain reaches 1/sin(fov/4) = sqrt(2) image radii at fov=180 — which
      // on a square canvas lands within a pixel of the corner, so a "corner is outside the circle"
      // probe there measures nothing. (This is the same wide-FOV annulus test_preview_background.cpp
      // documents as a deliberate divergence from the CLI.) Widening the canvas puts real,
      // unambiguous out-of-domain pixels at the left and right edges: 128 * sqrt(2) = 181 px, and
      // the frame now extends to 200.
      g_req.Reset();
      g_req.canvas_w = 400;
      g_req.canvas_h = 256;
      g_req.lens_type = gui::kLensTypeFisheyeEqualArea;
      g_req.fov = 180.0f;
      g_req.elevation = 0.0f;
      g_req.visible = gui::kVisibleUpper;
      g_req.uniform_xyz[0] = 0.05f;
      g_req.uniform_xyz[1] = 0.05f;
      g_req.uniform_xyz[2] = 0.05f;
      RenderFrame(ctx);
      IM_CHECK(!g_req.rgba.empty());

      // 195 px from the axis: past the 181 px domain edge, so no direction was ever projected here.
      ExpectPaperAt("outside-image-circle", -195.0f, 0.0f);
      // Low in the frame and near the axis: inside the circle, in the excluded lower hemisphere.
      ExpectPaperAt("excluded-hemisphere", 0.0f, -100.0f);
      // The control: the upper half is imaged and must NOT be bare paper, or the two probes above
      // are just describing a frame that is paper everywhere.
      unsigned char lit[3] = {};
      int paper_bytes[3] = {};
      ExpectedInkBytes(0.0f, paper_bytes);
      if (ReadPixel(g_req.rgba, g_req.canvas_w, g_req.canvas_h, 0.0f, 60.0f, lit)) {
        if (std::abs(static_cast<int>(lit[0]) - paper_bytes[0]) <= kInkToleranceLsb) {
          IM_ERRORF(
              "control: the imaged upper hemisphere is bare paper too (%d, %d, %d) — no ink landed "
              "anywhere, so the two probes above hold vacuously",
              (int)lit[0], (int)lit[1], (int)lit[2]);
        }
      }
    };
  }

  // AC4 on the GPU: an annotation's colour must not reach the page. Same claim and same shape as
  // the CLI's, with the screen arm as the control.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "preview_print_mode", "annotation_colour_does_not_reach_the_page");
    t->GuiFunc = PreviewPrintModeGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      const auto render_with = [ctx](int tone, float r, float g, float b) {
        g_req.Reset();
        g_req.tone = tone;
        // A fisheye so overlayLensBorder() has a circle to draw at all — the linear family has no
        // image boundary and that half of the check would silently do nothing.
        g_req.lens_type = gui::kLensTypeFisheyeEqualArea;
        g_req.fov = 180.0f;
        g_req.elevation = 0.0f;
        g_req.show_horizon = true;
        g_req.show_lens_border = true;
        g_req.annotation_color[0] = r;
        g_req.annotation_color[1] = g;
        g_req.annotation_color[2] = b;
        g_req.uniform_xyz[0] = 0.05f;
        g_req.uniform_xyz[1] = 0.05f;
        g_req.uniform_xyz[2] = 0.05f;
        RenderFrame(ctx);
        return g_req.rgba;
      };

      const std::vector<unsigned char> print_red = render_with(1, 1.0f, 0.0f, 0.0f);
      const std::vector<unsigned char> print_green = render_with(1, 0.0f, 1.0f, 0.0f);
      IM_CHECK(!print_red.empty());
      const std::size_t print_diff = CountDifferingBytes(print_red, print_green);
      if (print_diff != 0) {
        IM_ERRORF(
            "%zu bytes moved when only the horizon's COLOUR changed. Under print an annotation is "
            "ink and carries a coverage only — see blendAnnotationColor() in preview_renderer.cpp. "
            "Do not fix this by giving print its own line palette.",
            print_diff);
      }

      const std::vector<unsigned char> screen_red = render_with(0, 1.0f, 0.0f, 0.0f);
      const std::vector<unsigned char> screen_green = render_with(0, 0.0f, 1.0f, 0.0f);
      if (CountDifferingBytes(screen_red, screen_green) == 0) {
        IM_ERRORF(
            "the screen control did not move either: no annotation was actually drawn at "
            "fov=%.0f, so the print assertion above proves nothing",
            static_cast<double>(g_req.fov));
      }
    };
  }

  // The negative claim, asserted positively. With tone at screen the frame must be exactly the
  // additive result — the picked sky on a zero-energy pixel, black outside the gate — which is what
  // it was before print existed. This is AC1's second half on the GPU: positive evidence, not "no
  // other test went red".
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "preview_print_mode", "screen_tone_is_unchanged_by_the_print_branch");
    t->GuiFunc = PreviewPrintModeGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      g_req.Reset();
      g_req.tone = 0;
      g_req.canvas_w = 400;  // out-of-domain pixels at the edges — see the case above for why
      g_req.canvas_h = 256;
      g_req.lens_type = gui::kLensTypeFisheyeEqualArea;
      g_req.fov = 180.0f;
      // Zero energy: every pixel inside the gate is then exactly the picked sky colour, which is
      // the identity the additive chain owes and the strongest thing to compare against.
      g_req.uniform_xyz[0] = 0.0f;
      g_req.uniform_xyz[1] = 0.0f;
      g_req.uniform_xyz[2] = 0.0f;
      RenderFrame(ctx);
      IM_CHECK(!g_req.rgba.empty());

      unsigned char centre[3] = {};
      if (ReadPixel(g_req.rgba, g_req.canvas_w, g_req.canvas_h, 0.0f, 0.0f, centre)) {
        for (int j = 0; j < 3; ++j) {
          const int expected = static_cast<int>(std::lround(g_req.sky_srgb[j] * 255.0f));
          if (std::abs(static_cast<int>(centre[j]) - expected) > 0) {
            IM_ERRORF(
                "screen tone, channel %d: a zero-energy pixel reads %d, the picked sRGB byte is %d. "
                "The print branch has changed the additive path — tone=screen must be untouched.",
                j, (int)centre[j], expected);
          }
        }
      }
      // And outside the image circle it must still be pure BLACK, not paper: the zero-energy colour
      // is per-mode, and this is the half of that rule the print arm cannot check.
      unsigned char corner[3] = {};
      if (ReadPixel(g_req.rgba, g_req.canvas_w, g_req.canvas_h, -195.0f, 0.0f, corner)) {
        if (corner[0] != 0 || corner[1] != 0 || corner[2] != 0) {
          IM_ERRORF(
              "screen tone: the region outside the image circle reads (%d, %d, %d), expected pure "
              "black — the print branch's zero-energy colour has leaked into the screen path",
              (int)corner[0], (int)corner[1], (int)corner[2]);
        }
      }
    };
  }
}
