// The sky colour behind the halo, in rendered pixels.
//
// What this suite is for. The preview's fragment shader adds a background colour to the halo's
// radiance, and three separate claims about that addition are only checkable by reading pixels:
//
//   * WHERE in the chain it happens. The addition is a radiance sum, so it belongs in linear RGB,
//     before the clamp and before the sRGB transfer curve. Get that wrong and a pixel carrying no
//     halo energy comes back gamma-encoded a second time — a picker value of 0.2 renders as byte
//     123 instead of 51. The unit sibling (unit-correctness/util/test_color_space.cpp) pins the
//     same identity for the CPU implementation and says nothing about the GLSL beside it.
//   * WHERE ON SCREEN it is allowed to happen. Only inside the shader's own visibility gate
//     (`result.w >= 0.5 && pixel_visible`): outside the lens's image circle, in the half-sky the
//     `visible` setting discards, and behind the camera when `front` is on, nothing was ever
//     projected, so painting those regions would turn a fisheye render into a solid rectangle with
//     an invisible circle inside it.
//   * That the exports inherit both. The three PNG export entry points reuse this shader through
//     an off-screen FBO at their own sizes, which is an argument, not a measurement — the gate is
//     expressed in fractions of the image radius, and a hard-coded pixel radius would look right
//     at the preview size and wrong at an export size.
//
// Capture path: RenderExportToRgba's own off-screen FBO, the same one the lens_proj references and
// test_lens_border.cpp use, so nothing here depends on window size or panel layout. No simulation
// is run; the source texture is uploaded directly, which is what makes every pixel's expected
// value computable in closed form.
//
// What a user sees when these break: they pick a sky colour and the preview does not change; or it
// changes to a colour that is not the one they picked; or the black surround outside a fisheye's
// image circle fills in with sky and the picture stops looking like a photograph of the sky.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include "gui/export_fbo_renderer.hpp"
#include "gui/gui_constants.hpp"
#include "gui/preview_jacobian.hpp"
#include "test_gui_shared.hpp"
#include "util/color_space.hpp"

namespace {

constexpr float kPi = 3.14159265358979323846f;

// The sky colour every case renders with unless it says otherwise. Deliberately three DIFFERENT
// channel values, none of them 0 or 1: a swapped or dropped channel is then visible, and none of
// the three sits on a clamp boundary where an arithmetic error would be absorbed.
constexpr float kSkyR = 0.2f;
constexpr float kSkyG = 0.35f;
constexpr float kSkyB = 0.6f;

// The sky identity is EXACT: a zero-energy pixel renders the picked sRGB bytes with no tolerance
// at all. Measured, not assumed — the whole suite was first run demanding exact equality, and the
// four cases that read the sky colour passed at 0 LSB (0.2/0.35/0.6 render as 51/89/153 on the
// nose). The one comparison that needs slack is the GPU-against-CPU one below, and it gets its own
// constant so that the slack cannot silently spread to the identity this suite exists to pin.
constexpr int kSkyToleranceLsb = 0;

// The shader and the CPU bake are two implementations of one chain, and their last step is a
// TRUNCATION of `v * 255`, not a rounding — so wherever the two land on opposite sides of an
// integer, a difference of one whole byte appears from a float difference far below it. Measured
// across the five fields swept below: every disagreement is exactly 1 LSB and always in the same
// direction (the shader one byte brighter), never 2. The same narrowing is what makes the CLI's
// own background contract allow 1 LSB (test/e2e-correctness/test_background_color_contract.py),
// and it predates this work — the background-free overload truncates identically.
//
// This is a tolerance on the NARROWING only. The failures this case exists to catch — a bake that
// forgot the background, or added it in the wrong space — are 50 to 120 LSB away.
constexpr int kBakeToleranceLsb = 1;

// A render request marshalled to the frame loop. Both the texture upload and RenderExportToRgba
// must run on the thread that owns the GL context, which is the render thread and not the test
// coroutine — the same crossing test_lens_border.cpp and test_preview_texture.cpp make.
struct RenderRequest {
  bool requested = false;
  bool done = false;

  int canvas_w = 256;
  int canvas_h = 256;

  int lens_type = gui::kLensTypeLinear;
  float fov = 90.0f;
  float elevation = 0.0f;
  float azimuth = 0.0f;
  float roll = 0.0f;
  int visible = gui::kVisibleFull;
  bool front = false;

  // The picker colour, in sRGB. Converted to linear on the way into PreviewParams, which is the
  // conversion app_panels.cpp performs in production.
  float sky_srgb[3] = { kSkyR, kSkyG, kSkyB };

  // Source texture: a uniform XYZ field of this size. Uniform on purpose — see the note above
  // the bake-vs-shader case for why a spatially varying field would measure texture filtering
  // rather than the colour chain.
  int tex_size = 64;
  float uniform_xyz[3] = { 0.0f, 0.0f, 0.0f };
  float intensity_scale = 1.0f;

  // When true the source is baked to sRGB bytes on the CPU (background folded in) and uploaded as
  // an 8-bit texture, so the shader's XYZ branch — and its background addition — never runs.
  bool bake_on_cpu = false;

  // A panorama export preset to apply on top, or -1 for none. Mirrors what app.cpp's export entry
  // points do: BuildExportParams copies the live preview params, then one Configure*ExportParams
  // call overwrites the view and the decorations.
  int export_preset = -1;

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

  float sky_linear[3];
  lumice::SrgbToLinearRgb(g_req.sky_srgb, sky_linear);

  if (g_req.bake_on_cpu) {
    std::vector<unsigned char> baked(static_cast<std::size_t>(n) * 3);
    LUMICE_XyzToSrgbUint8WithBackground(xyz.data(), baked.data(), n, g_req.intensity_scale, sky_linear);
    gui::g_preview.UploadTexture(baked.data(), g_req.tex_size, g_req.tex_size);
  } else {
    gui::g_preview.UploadXyzTexture(xyz.data(), g_req.tex_size, g_req.tex_size);
  }

  gui::PreviewParams params{};
  params.view_proj.lens_type = g_req.lens_type;
  params.view_proj.fov = g_req.fov;
  params.view_proj.elevation = g_req.elevation;
  params.view_proj.azimuth = g_req.azimuth;
  params.view_proj.roll = g_req.roll;
  params.view_proj.visible = g_req.visible;
  params.view_proj.front = g_req.front;
  params.source.max_abs_dz = gui::kDualFisheyeOverlap;
  params.source.r_scale = 1.0f / std::sqrt(1.0f + gui::kDualFisheyeOverlap);
  params.exposure.intensity_factor = 1.0f;
  params.exposure.intensity_scale = g_req.bake_on_cpu ? 0.0f : g_req.intensity_scale;
  // Set on BOTH paths on purpose. The baked path renders an 8-bit texture that already contains
  // the sky, so the shader must ignore this field there; leaving it filled in is what makes the
  // bake-vs-shader case also assert that it does, rather than adding the sky a second time.
  std::copy(std::begin(sky_linear), std::end(sky_linear), std::begin(params.background_color_linear));

  if (g_req.export_preset == gui::kLensTypeDualFisheyeEqualArea) {
    gui::ConfigureDualFisheyeExportParams(params);
  } else if (g_req.export_preset == gui::kLensTypeRectangular) {
    gui::ConfigureEquirectExportParams(params);
  }

  g_req.rgba = gui::RenderExportToRgba(gui::g_preview, params, g_req.canvas_w, g_req.canvas_h, std::nullopt);
  g_req.done = true;
  g_req.requested = false;
}

void PreviewBackgroundGuiFunc(ImGuiTestContext* /*ctx*/) {
  if (g_req.requested && !g_req.done) {
    RunRenderRequest();
  }
}

// The live preview panel only fills PreviewParams when it has something to show, so the case that
// reads those params back has to put a texture there first — and an upload is a GL call, so it
// happens here rather than in the test coroutine.
bool g_live_upload_done = false;

void LivePanelGuiFunc(ImGuiTestContext* /*ctx*/) {
  if (!g_live_upload_done) {
    InitSynthTexture();
    gui::g_preview.UploadTexture(g_synth_tex.data(), kSynthTexW, kSynthTexH);
    g_live_upload_done = true;
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

// Pixel at center-origin, y-up coordinates — the convention the shader's `pos` uses, so a
// coordinate predicted from a projection formula can be handed straight in. The buffer itself is
// row-major top-down and RGBA.
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

// The byte a channel of the picker colour is expected to render as.
int ExpectedSkyByte(float srgb_channel) {
  return static_cast<int>(std::lround(srgb_channel * 255.0f));
}

// The byte the same channel would render as if the background had been added AFTER the sRGB
// transfer curve instead of before it — i.e. gamma-encoded a second time. Spelled out here rather
// than called from the library so the diagnostic names the failure a reader is most likely to be
// looking at, without this file reaching past the C API for a four-line formula.
int DoubleEncodedByte(float srgb_channel) {
  const float v =
      srgb_channel < 0.0031308f ? srgb_channel * 12.92f : 1.055f * std::pow(srgb_channel, 1.0f / 2.4f) - 0.055f;
  return static_cast<int>(std::lround(v * 255.0f));
}

// Report, non-fatally, if the pixel at (pos_x, pos_y) is not the sky colour. Non-fatal throughout
// this file: nothing below a report drives the GUI (these are buffer reads), and WHICH probe fails
// is the diagnostic — a fatal assert inside a probe list would hide every probe after the first.
void ExpectSkyAt(const char* tag, const std::vector<unsigned char>& rgba, int w, int h, float pos_x, float pos_y) {
  unsigned char rgb[3] = {};
  if (!ReadPixel(rgba, w, h, pos_x, pos_y, rgb)) {
    IM_ERRORF("%s: pixel (%.1f, %.1f) is outside the %dx%d frame", tag, static_cast<double>(pos_x),
              static_cast<double>(pos_y), w, h);
    return;
  }
  const float srgb[3] = { kSkyR, kSkyG, kSkyB };
  for (int j = 0; j < 3; ++j) {
    const int expected = ExpectedSkyByte(srgb[j]);
    if (std::abs(static_cast<int>(rgb[j]) - expected) > kSkyToleranceLsb) {
      IM_ERRORF(
          "%s: pixel (%.1f, %.1f) channel %d reads %d, the picked sRGB byte is %d. A value near %d "
          "would mean the colour was added after the gamma curve instead of before it; 0 would mean "
          "it was not added at all. Full pixel (%d, %d, %d).",
          tag, static_cast<double>(pos_x), static_cast<double>(pos_y), j, (int)rgb[j], expected,
          DoubleEncodedByte(srgb[j]), (int)rgb[0], (int)rgb[1], (int)rgb[2]);
    }
  }
}

// Report, non-fatally, if the pixel at (pos_x, pos_y) is not pure black.
void ExpectBlackAt(const char* tag, const std::vector<unsigned char>& rgba, int w, int h, float pos_x, float pos_y) {
  unsigned char rgb[3] = {};
  if (!ReadPixel(rgba, w, h, pos_x, pos_y, rgb)) {
    IM_ERRORF("%s: pixel (%.1f, %.1f) is outside the %dx%d frame", tag, static_cast<double>(pos_x),
              static_cast<double>(pos_y), w, h);
    return;
  }
  if (rgb[0] != 0 || rgb[1] != 0 || rgb[2] != 0) {
    IM_ERRORF(
        "%s: pixel (%.1f, %.1f) reads (%d, %d, %d), expected pure black — the sky colour is being "
        "painted outside the shader's `result.w >= 0.5 && pixel_visible` gate",
        tag, static_cast<double>(pos_x), static_cast<double>(pos_y), (int)rgb[0], (int)rgb[1], (int)rgb[2]);
  }
}

}  // namespace

void RegisterPreviewBackgroundTests(ImGuiTestEngine* engine) {
  // The identity the whole colour-space contract exists for, read off the screen: a pixel with no
  // halo energy renders as the sRGB triple the user picked, because LinearToSrgb(SrgbToLinear(x))
  // == x. This is the case that separates "the background is added in linear RGB" from "the
  // background is added after the gamma curve" — the two readings are ~70 LSB apart per channel.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "preview_background", "a_zero_energy_pixel_renders_the_picked_colour");
    t->GuiFunc = PreviewBackgroundGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      g_req.Reset();
      g_req.canvas_w = 256;
      g_req.canvas_h = 256;
      g_req.lens_type = gui::kLensTypeLinear;
      g_req.fov = 90.0f;
      // A zero XYZ field: every pixel inside the gate carries no halo energy at all, so the whole
      // frame is the identity above rather than one hand-picked pixel.
      g_req.uniform_xyz[0] = 0.0f;
      g_req.uniform_xyz[1] = 0.0f;
      g_req.uniform_xyz[2] = 0.0f;
      RenderFrame(ctx);
      IM_CHECK(!g_req.rgba.empty());

      // Centre plus four points spread over the frame. A linear lens at fov=90 images the whole
      // 256x256 canvas, so all five are inside the gate and must all be the picked colour.
      const float kProbes[][2] = {
        { 0.0f, 0.0f }, { -100.0f, -100.0f }, { 100.0f, -100.0f }, { -100.0f, 100.0f }, { 100.0f, 100.0f }
      };
      for (const auto& p : kProbes) {
        ExpectSkyAt("zero-energy", g_req.rgba, g_req.canvas_w, g_req.canvas_h, p[0], p[1]);
      }
    };
  }

  // The gate, in its three independent forms. Each is rendered twice — once with the region that
  // should be black, once with the same geometry under a setting that admits it — so a black pixel
  // is attributable to the gate rather than to the frame being empty for some unrelated reason.
  // Without the paired positive control every one of these would also pass on a shader that had
  // stopped drawing a background at all.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "preview_background", "the_background_stops_at_the_visibility_gate");
    t->GuiFunc = PreviewBackgroundGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // ---- (1) Outside the lens's image circle ----
      // Equal-area fisheye at fov=180 on a 400x200 canvas: img_radius = min(w,h)/2 = 100 px, and
      // the projection's own domain guard (r * sin(fov/4) <= 1) puts its outer edge at
      // 100 / sin(45 deg) = 141.4 px. The canvas is deliberately NOT square: on a square one the
      // domain circle reaches exactly to the corners (img_radius * sqrt(2)) and there is no
      // addressable pixel outside it to probe.
      g_req.Reset();
      g_req.canvas_w = 400;
      g_req.canvas_h = 200;
      g_req.lens_type = gui::kLensTypeFisheyeEqualArea;
      g_req.fov = 180.0f;
      RenderFrame(ctx);
      IM_CHECK(!g_req.rgba.empty());
      ExpectSkyAt("inside the image circle", g_req.rgba, 400, 200, 0.0f, 0.0f);
      ExpectBlackAt("outside the image circle", g_req.rgba, 400, 200, 180.0f, 0.0f);

      // ---- (2) The half-sky `visible` discards ----
      // Same lens, square canvas, camera on the horizon: the frame's upper half is sky above the
      // horizon and its lower half is below, so ONE frame carries both the kept and the discarded
      // region. img_radius = 128 px; (0, +-60) is well inside the image circle either way, which
      // is what makes the difference between the two attributable to `visible` alone.
      g_req.Reset();
      g_req.canvas_w = 256;
      g_req.canvas_h = 256;
      g_req.lens_type = gui::kLensTypeFisheyeEqualArea;
      g_req.fov = 180.0f;
      g_req.visible = gui::kVisibleUpper;
      RenderFrame(ctx);
      IM_CHECK(!g_req.rgba.empty());
      ExpectSkyAt("upper half-sky, kept", g_req.rgba, 256, 256, 0.0f, 60.0f);
      ExpectBlackAt("lower half-sky, discarded", g_req.rgba, 256, 256, 0.0f, -60.0f);

      // ---- (3) Behind the camera, with `front` on ----
      // `front` discards directions behind the camera, so it can only bite on a lens whose frame
      // reaches past 90 deg from the optical axis: at fov=270 the equal-area image circle runs out
      // to theta = 135 deg. The probe sits at r = 130 px on the 45 deg diagonal of a 256x256
      // frame, where theta = 2*asin((130/128) * sin(67.5 deg)) = 139.5 deg — behind the camera,
      // and inside the projection's domain edge at 128 / sin(67.5 deg) = 138.5 px.
      const float kBehindR = 130.0f;
      const float kBehindX = kBehindR * std::cos(0.25f * kPi);
      const float kBehindY = kBehindR * std::sin(0.25f * kPi);

      g_req.Reset();
      g_req.canvas_w = 256;
      g_req.canvas_h = 256;
      g_req.lens_type = gui::kLensTypeFisheyeEqualArea;
      g_req.fov = 270.0f;
      g_req.front = false;
      RenderFrame(ctx);
      IM_CHECK(!g_req.rgba.empty());
      ExpectSkyAt("behind the camera, front off", g_req.rgba, 256, 256, kBehindX, kBehindY);

      g_req.Reset();
      g_req.canvas_w = 256;
      g_req.canvas_h = 256;
      g_req.lens_type = gui::kLensTypeFisheyeEqualArea;
      g_req.fov = 270.0f;
      g_req.front = true;
      RenderFrame(ctx);
      IM_CHECK(!g_req.rgba.empty());
      ExpectBlackAt("behind the camera, front on", g_req.rgba, 256, 256, kBehindX, kBehindY);
    };
  }

  // The exports, measured rather than argued. Two separate claims here, and they fail differently:
  //   * the gate is expressed in fractions of the image radius, so it lands in the same PLACE
  //     relative to the picture at any output size — a hard-coded pixel radius passes at one size
  //     and fails at the other;
  //   * a panorama export preset does not lose the background on its way through. The preset
  //     overwrites the view and clears the background IMAGE overlay, and the sky COLOUR sits in
  //     the same struct one field away. (The struct-level half of that claim, which does not need
  //     a frame, is unit-correctness/gui/test_export_params.cpp.)
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "preview_background", "the_exports_paint_the_same_background");
    t->GuiFunc = PreviewBackgroundGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Two output sizes, one aspect-shaped like a preview panel and one like a real export. The
      // probes are in units of the image radius, not pixels, which is the whole point.
      struct Size {
        int w;
        int h;
        const char* name;
      };
      const Size kSizes[] = { { 400, 200, "400x200" }, { 800, 450, "800x450" } };
      for (const Size& size : kSizes) {
        g_req.Reset();
        g_req.canvas_w = size.w;
        g_req.canvas_h = size.h;
        g_req.lens_type = gui::kLensTypeFisheyeEqualArea;
        g_req.fov = 180.0f;
        RenderFrame(ctx);
        if (g_req.rgba.empty()) {
          IM_ERRORF("%s: the off-screen render produced no pixels", size.name);
          continue;
        }
        // img_radius = min(w,h)/2; the domain edge is at 1.414 of that. 0.5 is comfortably inside,
        // 1.7 comfortably outside, and both are addressable on the x axis at both sizes.
        const float img_radius = std::min(size.w, size.h) * 0.5f;
        ExpectSkyAt((std::string(size.name) + " inside").c_str(), g_req.rgba, size.w, size.h, 0.5f * img_radius, 0.0f);
        ExpectBlackAt((std::string(size.name) + " outside").c_str(), g_req.rgba, size.w, size.h, 1.7f * img_radius,
                      0.0f);
      }

      // The dual-fisheye panorama preset at its own output shape. circle_radius =
      // min(w/2, h)/2 = 128 px on a 512x256 frame, with the two clip circles centred at
      // (-128, 0) and (+128, 0); (0, 100) is outside both.
      g_req.Reset();
      g_req.canvas_w = 512;
      g_req.canvas_h = 256;
      g_req.export_preset = gui::kLensTypeDualFisheyeEqualArea;
      RenderFrame(ctx);
      if (g_req.rgba.empty()) {
        IM_ERRORF("%s", "dual fisheye export preset: the off-screen render produced no pixels");
      } else {
        ExpectSkyAt("dual fisheye export, left disc", g_req.rgba, 512, 256, -128.0f, 0.0f);
        ExpectSkyAt("dual fisheye export, right disc", g_req.rgba, 512, 256, 128.0f, 0.0f);
        ExpectBlackAt("dual fisheye export, between the discs", g_req.rgba, 512, 256, 0.0f, 100.0f);
      }

      // The equirectangular preset. Full-sky: every pixel maps to a direction, so the whole frame
      // is inside the gate and there is no black region to probe — which is itself the claim, and
      // the reason the corners are probed rather than only the centre.
      g_req.Reset();
      g_req.canvas_w = 512;
      g_req.canvas_h = 256;
      g_req.export_preset = gui::kLensTypeRectangular;
      RenderFrame(ctx);
      if (g_req.rgba.empty()) {
        IM_ERRORF("%s", "equirect export preset: the off-screen render produced no pixels");
      } else {
        const float kProbes[][2] = { { 0.0f, 0.0f }, { -250.0f, -120.0f }, { 250.0f, 120.0f } };
        for (const auto& p : kProbes) {
          ExpectSkyAt("equirect export", g_req.rgba, 512, 256, p[0], p[1]);
        }
      }
    };
  }

  // The .lmc bake against the screen. Saving a document converts the XYZ frame to sRGB bytes on
  // the CPU (RefreshCpuTextureForSave) instead of in the shader, and the saved picture is supposed
  // to be the one the user was looking at — so the two implementations of the same chain, one in
  // GLSL and one in C++, have to agree byte for byte with the background folded in.
  //
  // The source field is UNIFORM, and several uniform fields are swept rather than one varying one.
  // A spatially varying field would compare a shader that interpolates XYZ and then converts
  // against a bake that converts and then interpolates — a real difference in texture filtering,
  // not in the colour chain, which is what this case is about. The mapping from screen to texel is
  // covered by the gate cases above and by test_gui_lens_projection.cpp.
  //
  // The lens is EQUAL-AREA, and that is now load-bearing rather than incidental. The XYZ branch
  // multiplies each sample by the target lens's relative illumination — its per-pixel solid angle
  // normalized on axis, so that the preview carries the projection's natural vignetting the way a
  // CLI render does (src/gui/preview_jacobian.hpp, doc/ev-pipeline-architecture.md §7.5). The 8-bit
  // branch cannot: its texels were already clipped to [0,1] and already have the sky composited
  // into them, so neither the halo term nor the sky term can be recovered to scale one and not the
  // other. On an equal-area projection that factor is exactly 1 on both sides and this case
  // measures the colour chain alone, which is what it owns. On any other it would measure the
  // projection instead — and the case below pins that divergence directly, so moving this one here
  // hides nothing.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "preview_background", "the_baked_document_matches_the_shader");
    t->GuiFunc = PreviewBackgroundGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      struct Field {
        float xyz[3];
        float intensity_scale;
        const char* name;
      };
      const Field kFields[] = {
        { { 0.0f, 0.0f, 0.0f }, 1.0f, "no energy" },
        { { 0.09f, 0.08f, 0.11f }, 1.0f, "dim, in gamut" },
        { { 0.35f, 0.30f, 0.20f }, 1.0f, "mid, warm" },
        // Out of gamut before the clip, so the case also covers the gamut-clip half of the chain.
        { { 0.60f, 0.12f, 0.02f }, 1.0f, "saturated, out of gamut" },
        // Scaled past white, so the post-background clamp is exercised on every channel.
        { { 0.40f, 0.42f, 0.45f }, 4.0f, "clipped white" },
      };

      // Same geometry for both halves of every pair: an equal-area fisheye at fov=180 on a 256px
      // frame has an image radius of 128 px and a domain that reaches sqrt(2) of it, so all three
      // probes (r = 0, 89 and 114 px) are inside the gate and the only thing differing between the
      // two renders is which implementation produced the colour.
      const float kProbes[][2] = { { 0.0f, 0.0f }, { -80.0f, 40.0f }, { 90.0f, -70.0f } };

      for (const Field& field : kFields) {
        auto render = [&](bool bake) {
          g_req.Reset();
          g_req.canvas_w = 256;
          g_req.canvas_h = 256;
          g_req.lens_type = gui::kLensTypeFisheyeEqualArea;
          g_req.fov = 180.0f;
          std::copy(std::begin(field.xyz), std::end(field.xyz), std::begin(g_req.uniform_xyz));
          g_req.intensity_scale = field.intensity_scale;
          g_req.bake_on_cpu = bake;
          RenderFrame(ctx);
          return g_req.rgba;
        };
        const std::vector<unsigned char> shader_frame = render(false);
        const std::vector<unsigned char> baked_frame = render(true);
        if (shader_frame.empty() || baked_frame.empty()) {
          IM_ERRORF("%s: an off-screen render produced no pixels", field.name);
          continue;
        }

        for (const auto& p : kProbes) {
          unsigned char a[3] = {};
          unsigned char b[3] = {};
          if (!ReadPixel(shader_frame, 256, 256, p[0], p[1], a) || !ReadPixel(baked_frame, 256, 256, p[0], p[1], b)) {
            IM_ERRORF("%s: probe (%.1f, %.1f) is outside the frame", field.name, static_cast<double>(p[0]),
                      static_cast<double>(p[1]));
            continue;
          }
          for (int j = 0; j < 3; ++j) {
            if (std::abs(static_cast<int>(a[j]) - static_cast<int>(b[j])) > kBakeToleranceLsb) {
              IM_ERRORF(
                  "%s: at (%.1f, %.1f) channel %d the shader renders %d and the CPU bake renders %d. "
                  "The saved document would not look like the screen. Shader (%d, %d, %d) vs bake "
                  "(%d, %d, %d).",
                  field.name, static_cast<double>(p[0]), static_cast<double>(p[1]), j, (int)a[j], (int)b[j], (int)a[0],
                  (int)a[1], (int)a[2], (int)b[0], (int)b[1], (int)b[2]);
            }
          }
        }
      }
    };
  }

  // The second declared, deliberate disagreement in this file, and the one that only exists on a
  // NON-equal-area projection: a document reloaded from a .lmc does not carry the projection's
  // natural vignetting, and the live view of the same document does.
  //
  // The mechanism. The XYZ branch multiplies each sample by the target lens's relative illumination
  // before the exposure scale reaches it, so the preview collects energy per pixel the way a CLI
  // render does. The 8-bit branch — a document loaded from disk, whose texels came out of
  // LUMICE_XyzToSrgbUint8WithBackground — cannot do the same, and the reason is not effort. Those
  // texels are the sum of the halo and the sky, clamped to [0, 1] and quantized. Scaling them
  // scales the sky too, which neither the CLI nor the live path does; and adding the sky back at
  // (1 - m) would be exact only where the bake did not clip, which is precisely where a halo image
  // is brightest. A frozen 8-bit snapshot cannot be re-exposed after the fact.
  //
  // So the divergence is a property of the .lmc format, not a defect to be fixed here, and NEITHER
  // SIDE may be quietly changed to match the other. What this case does is state the relationship
  // exactly rather than approximately, so that any change to either branch lands as a red:
  //
  //     shader_linear = (bake_linear - sky) * m + sky
  //
  // which is an identity as long as the gamut clip is inactive — hence the single dim, in-gamut
  // field. m comes from src/gui/preview_jacobian.hpp, the CPU mirror of the GLSL the shader runs,
  // so this is also the one place the two mirrors are compared against each other. (What says the
  // formula itself is right is test/unit-correctness/gui/test_preview_jacobian.cpp, which holds it
  // to a numerically rebuilt Jacobian instead.)
  //
  // Three probes on a linear lens at fov=90, 256x256, focal = 128 px: r = 0 (m = 1, a positive
  // control that says the two branches still agree where the factor is the identity), r = 89 px
  // (m = 0.61) and r = 114 px (m = 0.42). Without the first, "the two disagree" would also pass on
  // a shader that had simply gone dark.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "preview_background", "the_baked_document_lacks_the_projection_vignetting");
    t->GuiFunc = PreviewBackgroundGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Dim and in gamut on purpose: the identity above holds only where the bake did not clip.
      const float kField[3] = { 0.09f, 0.08f, 0.11f };
      auto render = [&](bool bake) {
        g_req.Reset();
        g_req.canvas_w = 256;
        g_req.canvas_h = 256;
        g_req.lens_type = gui::kLensTypeLinear;
        g_req.fov = 90.0f;
        std::copy(std::begin(kField), std::end(kField), std::begin(g_req.uniform_xyz));
        g_req.bake_on_cpu = bake;
        RenderFrame(ctx);
        return g_req.rgba;
      };
      const std::vector<unsigned char> shader_frame = render(false);
      const std::vector<unsigned char> baked_frame = render(true);
      IM_CHECK(!shader_frame.empty());
      IM_CHECK(!baked_frame.empty());

      const float sky_srgb[3] = { kSkyR, kSkyG, kSkyB };
      float sky_linear[3];
      lumice::SrgbToLinearRgb(sky_srgb, sky_linear);

      // focal for a linear lens on a square 256px frame at fov=90: short_edge/2 / tan(45 deg).
      const float kFocal = 128.0f;
      const float kProbes[][2] = { { 0.0f, 0.0f }, { -80.0f, 40.0f }, { 90.0f, -70.0f } };
      // Two 8-bit round trips (the bake's quantization, then the render's) and a float multiply
      // between them. Measured across these probes and channels the residual is at most 1 LSB;
      // 2 is the headroom, and it is far below the 15-to-60 LSB the vignetting itself moves.
      constexpr int kVignetteToleranceLsb = 2;

      for (const auto& p : kProbes) {
        unsigned char shader_px[3] = {};
        unsigned char baked_px[3] = {};
        if (!ReadPixel(shader_frame, 256, 256, p[0], p[1], shader_px) ||
            !ReadPixel(baked_frame, 256, 256, p[0], p[1], baked_px)) {
          IM_ERRORF("probe (%.1f, %.1f) is outside the frame", static_cast<double>(p[0]), static_cast<double>(p[1]));
          continue;
        }
        const float rho = std::sqrt(p[0] * p[0] + p[1] * p[1]);
        const float m = lumice::gui::RelIllumRectilinear(rho, kFocal);
        for (int j = 0; j < 3; ++j) {
          const float baked_linear = lumice::SrgbToLinear(static_cast<float>(baked_px[j]) / 255.0f);
          const float predicted = lumice::LinearToSrgb((baked_linear - sky_linear[j]) * m + sky_linear[j]) * 255.0f;
          if (std::abs(static_cast<int>(shader_px[j]) - static_cast<int>(predicted)) > kVignetteToleranceLsb) {
            IM_ERRORF(
                "at (%.1f, %.1f) channel %d: the live XYZ branch renders %d, and rescaling the bake by the "
                "relative illumination m=%.4f predicts %.1f. The two branches have stopped standing in the "
                "declared relationship. Shader (%d, %d, %d) vs bake (%d, %d, %d).",
                static_cast<double>(p[0]), static_cast<double>(p[1]), j, (int)shader_px[j], (double)m,
                (double)predicted, (int)shader_px[0], (int)shader_px[1], (int)shader_px[2], (int)baked_px[0],
                (int)baked_px[1], (int)baked_px[2]);
          }
        }
      }
      // And the divergence really is visible, not a rounding difference: off axis the live branch
      // must be materially darker than the reloaded one. Without this, the identity above would
      // still pass if m collapsed to 1 everywhere and the vignetting silently disappeared.
      unsigned char shader_far[3] = {};
      unsigned char baked_far[3] = {};
      IM_CHECK(ReadPixel(shader_frame, 256, 256, 90.0f, -70.0f, shader_far));
      IM_CHECK(ReadPixel(baked_frame, 256, 256, 90.0f, -70.0f, baked_far));
      IM_CHECK_LT((int)shader_far[1] + 10, (int)baked_far[1]);
    };
  }

  // How far past the equator this renderer's single-lens domain reaches, pinned at three radii.
  //
  // The mechanism. This renderer works backwards from a pixel through the projection law, and its
  // only stopping condition is that law's own domain — for equal area, `r * sin(fov/4) <= 1`,
  // which at fov=180 reaches theta = 180 deg, a radius of sqrt(2) image radii. So a wide-FOV
  // single-lens preview shows sky well past the equator, out to the antipode of the lens axis.
  //
  // This case used to be a pinned CLI/GUI DIVERGENCE. The CLI's projection is a forward one, and
  // it culled every single-lens fisheye at the equator whatever the configured FOV, so the
  // annulus between r = img_radius and r = sqrt(2) * img_radius was sky here and black there. That
  // was a product-semantics question rather than a defect on either side, and it was settled in
  // 474.1 the way this renderer already had it: core's cull is now taken per lens type and runs to
  // theta = 180 deg for equal-area. The annulus is sky on both sides now. The pixel-for-pixel
  // comparison of the two domains lives in test/unit-correctness/gui/test_visible_mask_gui_parity.cpp,
  // which asserts equality; what stays here is this renderer's own half of it, in real frames.
  //
  // Three probes, not one, at fov=180 equal-area on a 512x256 frame (img_radius = 128 px):
  //   r = 100 px  theta = 67.1 deg   inside the equator            -> sky (positive control)
  //   r = 150 px  theta = 111.9 deg  past it, inside the domain    -> sky
  //   r = 220 px  beyond 181.0 px    outside the domain            -> black (negative control)
  // Without the outer two, "past the equator is sky" would also pass on a shader that painted the
  // entire frame, and without the inner one it would pass on a shader that painted nothing.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "preview_background", "the_single_lens_domain_reaches_past_the_equator");
    t->GuiFunc = PreviewBackgroundGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      g_req.Reset();
      g_req.canvas_w = 512;
      g_req.canvas_h = 256;
      g_req.lens_type = gui::kLensTypeFisheyeEqualArea;
      g_req.fov = 180.0f;
      g_req.visible = gui::kVisibleFull;
      RenderFrame(ctx);
      IM_CHECK(!g_req.rgba.empty());

      ExpectSkyAt("inside the CLI's equator (r = 100 px)", g_req.rgba, 512, 256, 100.0f, 0.0f);
      ExpectSkyAt("past the equator at 128 px (r = 150 px)", g_req.rgba, 512, 256, 150.0f, 0.0f);
      ExpectBlackAt("beyond this renderer's own domain (r = 220 px, past 181 px)", g_req.rgba, 512, 256, 220.0f, 0.0f);
    };
  }

  // The one link every case above takes for granted: that the live panel actually hands the user's
  // colour to the renderer, every frame. Everything else here builds PreviewParams by hand, which
  // proves the shader and says nothing about whether anything fills that field in the running app —
  // and "the picker moves and the preview does not" is the exact complaint this whole task is
  // fixing. So this case drives real frames of RenderPreviewPanel and reads the params back out.
  //
  // Reading the struct rather than the pixels, deliberately: the pixels are already covered above,
  // and what is unproven here is the assignment, which the struct shows directly. It also pins the
  // CONVERSION — the field must hold linear RGB, not the sRGB the picker produced, and the two are
  // far enough apart (0.6 -> 0.318) that a missing conversion cannot pass.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "preview_background", "the_live_panel_hands_the_picked_colour_to_the_renderer");
    t->GuiFunc = LivePanelGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      g_live_upload_done = false;
      ctx->Yield(3);

      // Two colours in a row, not one: a single value could be matched by a field that happens to
      // be initialised to it, and re-reading after a change is what "every frame" means.
      const float kColours[][3] = { { kSkyR, kSkyG, kSkyB }, { 0.9f, 0.05f, 0.42f } };
      for (const auto& picked : kColours) {
        std::copy(std::begin(picked), std::end(picked), std::begin(gui::g_state.renderer.background));
        ctx->Yield(3);

        float expected[3];
        lumice::SrgbToLinearRgb(picked, expected);
        for (int j = 0; j < 3; ++j) {
          const float actual = gui::g_preview_vp.params.background_color_linear[j];
          if (std::fabs(actual - expected[j]) > 1e-6f) {
            IM_ERRORF(
                "channel %d: the panel handed the renderer %.6f, expected %.6f (linear) for a picked "
                "sRGB %.3f. Reading %.3f back would mean the sRGB value was passed through with no "
                "conversion; 0 would mean the panel never wrote the field at all.",
                j, static_cast<double>(actual), static_cast<double>(expected[j]), static_cast<double>(picked[j]),
                static_cast<double>(picked[j]));
          }
        }
      }
    };
  }
}
