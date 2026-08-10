// The background image the preview composites the render on top of.
//
// What this suite is for. A background is the one thing in the preview that is not computed: the
// user loads a photograph of a real sky and checks whether the simulated halo lands where the real
// one did. Every question below therefore ends in pixels or in the uniforms that produce them, and
// none of them can be answered without a GL context — the image lives in a texture, the blend is
// two lines of fragment shader, and the "contain" fit that letterboxes a mismatched aspect is
// computed on the CPU but only observable in the exported frame.
//
// Deliberately NOT here. Whether the aspect and background fields survive a document round trip is
// composition-correctness/gui/test_document_roundtrip_chain.cpp; whether a legacy document without
// them falls back to the factory values, and which ratio each preset reports, are
// unit-correctness/gui/test_render_bg_logic.cpp; which aspect options are disabled without a
// background image is whole-domain in unit-correctness/gui/test_gui_widget_rules.cpp. Nothing below
// restates any of them.
//
// The one control that cannot be driven from the panel is `Load Bg##display`: it calls
// DoLoadBackground, which opens a native file dialog. The image is therefore pushed into the
// renderer directly, on the main thread, the same way the dialog's callback would. Every OTHER
// control in the Background row — Clear, Show — is clicked for real below.
//
// What a user sees when these break: a loaded sky is stretched instead of fitted, or is silently
// ignored; the Alpha slider stops fading between the photograph and the render; clearing the image
// leaves the window locked to an aspect ratio derived from an image that is no longer there.

#include <stb_image.h>

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "gui/gui_state.hpp"
#include "test_gui_shared.hpp"

namespace {

// Both background images shipped with the suite, with the aspect each one's pixel dimensions
// demand. The numbers are the files' own, restated here so a swapped or re-encoded reference has
// to come past this table rather than silently redefine what "landscape" means.
struct BgImage {
  const char* file;
  int width;
  int height;
};
constexpr BgImage kLandscape{ "bg_test_landscape.jpg", 2650, 1580 };
constexpr BgImage kPortrait{ "bg_test_portrait.jpg", 1608, 2488 };

std::string RefPath(const BgImage& img) {
  return std::string(LUMICE_TEST_REF_DIR "/") + img.file;
}

// Main-thread work this suite needs: the two GL uploads. Both are requests rather than direct
// calls because a TestFunc runs on the test engine's coroutine, and a GL call from there is a call
// on the wrong thread.
void BackgroundGuiFunc(ImGuiTestContext* /*ctx*/) {
  // The render layer. Without a texture in the renderer the export path refuses to run at all
  // (ExportPreviewPng's first guard), so even the "background only" cases need one.
  if (g_export_test.upload_requested && !g_export_test.upload_done) {
    InitSynthTexture();
    gui::g_preview.UploadTexture(g_synth_tex.data(), kSynthTexW, kSynthTexH);
    g_export_test.upload_done = true;
  }
  if (g_bg_test.bg_upload_requested && !g_bg_test.bg_upload_done) {
    int w = 0;
    int h = 0;
    int channels = 0;
    unsigned char* raw = stbi_load(g_bg_test.bg_image_path.c_str(), &w, &h, &channels, 3);
    if (raw) {
      gui::g_preview.UploadBgTexture(raw, w, h);
      stbi_image_free(raw);
    }
    g_bg_test.bg_upload_done = true;
  }
}

// Put a render texture and a background image in place, and return once both have landed.
void LoadRenderAndBackground(ImGuiTestContext* ctx, const BgImage& img) {
  g_export_test.Reset();
  g_export_test.upload_requested = true;
  g_bg_test.Reset();
  g_bg_test.bg_image_path = RefPath(img);
  g_bg_test.bg_upload_requested = true;
  ctx->Yield(3);
  IM_CHECK(g_bg_test.bg_upload_done);
  IM_CHECK(gui::g_preview.HasBackground());
}

// Mean channel value over one column of an RGB image. A column rather than a single pixel because
// what the letterbox test is asking about is a whole strip of the frame, and one pixel of a
// photograph can be any brightness at all.
double ColumnMean(const std::vector<unsigned char>& rgb, int w, int h, int channels, int x) {
  double sum = 0.0;
  for (int y = 0; y < h; ++y) {
    const size_t idx = (static_cast<size_t>(y) * w + x) * channels;
    sum += rgb[idx] + rgb[idx + 1] + rgb[idx + 2];
  }
  return sum / (h * 3.0);
}

bool IsDisabled(const ImGuiTestItemInfo& info) {
  return (info.ItemFlags & ImGuiItemFlags_Disabled) != 0;
}

// The aspect cases, indexed by ImGuiTest::ArgVariant. Landscape and portrait are the two sides of
// the `vp_aspect > bg_aspect` branch in the contain fit, and a transposed width/height would
// satisfy either one alone.
struct AspectCase {
  const char* name;
  BgImage img;
};
const AspectCase kAspectCases[] = {
  { "a_landscape_image_reports_its_own_wide_aspect", kLandscape },
  { "a_portrait_image_reports_its_own_tall_aspect", kPortrait },
};
constexpr int kAspectCaseCount = sizeof(kAspectCases) / sizeof(kAspectCases[0]);

}  // namespace

void RegisterBackgroundOverlayTests(ImGuiTestEngine* engine) {
  // The aspect a loaded image reports is the file's own, not the viewport's and not the previous
  // image's. Two independent registrations off one table (see kAspectCases), so a red run names
  // which orientation broke rather than stopping at the first.
  for (int idx = 0; idx < kAspectCaseCount; ++idx) {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "background_overlay", kAspectCases[idx].name);
    t->GuiFunc = BackgroundGuiFunc;
    t->ArgVariant = idx;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      const BgImage& img = kAspectCases[ctx->Test->ArgVariant].img;
      ResetTestState();
      IM_CHECK_EQ(gui::g_preview.HasBackground(), false);

      LoadRenderAndBackground(ctx, img);

      const float expected = static_cast<float>(img.width) / static_cast<float>(img.height);
      IM_CHECK_LT(std::abs(gui::g_preview.GetBgAspect() - expected), 1e-3f);
    };
  }

  // Clearing from the panel is a four-part undo, and the fourth part is the one with no other
  // guard: `Match Background` is an aspect preset whose ratio is derived from an image, so leaving
  // it selected after the image is gone points the window sizer at a ratio that no longer has a
  // source. DoClearBackground drops back to Free for exactly that reason.
  //
  // The old case this replaces called PreviewRenderer::ClearBackground() directly — which clears
  // the texture and nothing else — so it could not have seen the preset fallback at all.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "background_overlay", "clearing_the_image_takes_the_match_background_preset_with_it");
    t->GuiFunc = BackgroundGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();

      // With no image loaded there is nothing to clear, and the button says so.
      ctx->Yield(2);
      IM_CHECK(IsDisabled(ctx->ItemInfo("**/Clear##display_bg")));

      LoadRenderAndBackground(ctx, kLandscape);
      // DoLoadBackground writes bg_path after the file dialog returns; pushing the image straight
      // into the renderer skips that, so the field is filled in here. Without it the Clear
      // assertion below would be asserting that an already-empty string is still empty.
      gui::g_state.bg_path = RefPath(kLandscape);
      gui::g_state.aspect_preset = gui::AspectPreset::kMatchBg;
      ctx->ItemClick("**/Show##display_bg");
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.bg_show, true);

      ctx->ItemClick("**/Clear##display_bg");
      ctx->Yield(2);

      IM_CHECK_EQ(gui::g_preview.HasBackground(), false);
      IM_CHECK(gui::g_state.bg_path.empty());
      IM_CHECK_EQ(gui::g_state.bg_show, false);
      IM_CHECK_EQ(gui::g_state.aspect_preset, gui::AspectPreset::kFree);
    };
  }

  // The Show checkbox is the user's on/off switch, and what it has to reach is a shader uniform
  // baked once per frame by RenderPreviewPanel. The two are joined by an `&&` — bg_show AND an
  // image is actually loaded — so a checkbox wired to the wrong half looks right on screen until
  // the image is cleared underneath it.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "background_overlay", "the_show_checkbox_reaches_the_baked_parameters");
    t->GuiFunc = BackgroundGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      LoadRenderAndBackground(ctx, kLandscape);
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_preview_vp.params.bg.enabled, false);  // loaded, not shown

      ctx->ItemClick("**/Show##display_bg");
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_preview_vp.params.bg.enabled, true);
      IM_CHECK_LT(std::abs(gui::g_preview_vp.params.bg.aspect - gui::g_preview.GetBgAspect()), 1e-4f);

      ctx->ItemClick("**/Show##display_bg");
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_preview_vp.params.bg.enabled, false);
    };
  }

  // Alpha is a crossfade between two layers, and the reason to assert it in exported pixels rather
  // than in the uniform is that the uniform reaching the shader and the shader USING it are
  // different claims: `final_color = bg*(1-a) + render*a` is one line, and a build that dropped it
  // would still pass every parameter assertion above.
  //
  // The two endpoints are compared rather than a midpoint because they are the two the user can
  // name: 0 is "show me the photograph", 1 is "show me the simulation". Their PSNR is an upper
  // bound on how similar the two layers are allowed to be, not a measurement of either.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "background_overlay", "alpha_crossfades_between_the_photo_and_the_render");
    t->GuiFunc = BackgroundGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      LoadRenderAndBackground(ctx, kLandscape);
      ctx->ItemClick("**/Show##display_bg");
      ctx->Yield(2);
      IM_CHECK_GT(gui::g_preview_vp.vp_w, 0);
      IM_CHECK_GT(gui::g_preview_vp.vp_h, 0);

      // The slider's own value, set on the field rather than dragged: what is under test is the
      // blend, and a drag would additionally pin this case to the slider's pixel geometry.
      const std::string path_bg = GuiTestTempPath("lumice_bg_alpha0.png").string();
      gui::g_state.bg_alpha = 0.0f;
      ctx->Yield(2);  // let RenderPreviewPanel bake the new params
      IM_CHECK(RequestAndWaitPreviewExport(ctx, gui::g_preview_vp, path_bg));

      const std::string path_render = GuiTestTempPath("lumice_bg_alpha1.png").string();
      gui::g_state.bg_alpha = 1.0f;
      ctx->Yield(2);
      IM_CHECK(RequestAndWaitPreviewExport(ctx, gui::g_preview_vp, path_render));

      std::vector<unsigned char> img_bg;
      std::vector<unsigned char> img_render;
      int w0 = 0, h0 = 0, ch0 = 0;
      int w1 = 0, h1 = 0, ch1 = 0;
      IM_CHECK(lumice::test::LoadPng(path_bg.c_str(), img_bg, w0, h0, ch0));
      IM_CHECK(lumice::test::LoadPng(path_render.c_str(), img_render, w1, h1, ch1));
      IM_CHECK_EQ(w0, w1);
      IM_CHECK_EQ(h0, h1);
      IM_CHECK_EQ(ch0, ch1);

      const double psnr = lumice::test::ComputePsnr(img_bg.data(), img_render.data(), w0, h0, ch0);
      fprintf(stderr, "[background_overlay] alpha endpoints: PSNR = %.2f dB (must stay under 30)\n", psnr);
      IM_CHECK_LT(psnr, 30.0);

      std::remove(path_bg.c_str());
      std::remove(path_render.c_str());
    };
  }

  // The contain fit. A photograph whose aspect does not match the viewport is fitted INSIDE it and
  // the remainder is left black — not stretched, which would move every feature in the sky away
  // from where the render puts it and defeat the whole point of loading a reference photo.
  //
  // A portrait image in the harness's landscape viewport puts those bars on the left and right,
  // which is what makes this assertable without re-deriving the shader's UV arithmetic here: the
  // outermost column has to be black and the middle one has to carry image. The precondition is
  // asserted rather than assumed, so a harness whose viewport ever becomes taller than 0.65:1
  // fails saying so instead of quietly testing nothing.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "background_overlay", "a_mismatched_aspect_is_letterboxed_not_stretched");
    t->GuiFunc = BackgroundGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      LoadRenderAndBackground(ctx, kPortrait);
      ctx->ItemClick("**/Show##display_bg");
      gui::g_state.bg_alpha = 0.0f;  // photograph only, so the render cannot fill the bars
      ctx->Yield(2);
      IM_CHECK_GT(gui::g_preview_vp.vp_w, 0);
      IM_CHECK_GT(gui::g_preview_vp.vp_h, 0);

      const std::string path = GuiTestTempPath("lumice_bg_contain.png").string();
      IM_CHECK(RequestAndWaitPreviewExport(ctx, gui::g_preview_vp, path));

      std::vector<unsigned char> img;
      int w = 0, h = 0, ch = 0;
      IM_CHECK(lumice::test::LoadPng(path.c_str(), img, w, h, ch));
      IM_CHECK_GT(w, 0);
      IM_CHECK_GT(h, 0);

      const float vp_aspect = static_cast<float>(w) / static_cast<float>(h);
      const float bg_aspect = gui::g_preview.GetBgAspect();
      fprintf(stderr, "[background_overlay] contain: vp_aspect=%.3f bg_aspect=%.3f img=%dx%d\n", vp_aspect, bg_aspect,
              w, h);
      IM_CHECK_GT(vp_aspect, bg_aspect);  // the precondition for side bars, not an assumption

      const double edge = ColumnMean(img, w, h, ch, 0);
      const double middle = ColumnMean(img, w, h, ch, w / 2);
      fprintf(stderr, "[background_overlay] contain: edge column mean=%.2f, middle column mean=%.2f\n", edge, middle);
      IM_CHECK_LT(edge, 1.0);     // letterbox: the shader writes exact black outside the fit
      IM_CHECK_GT(middle, 10.0);  // and the photograph inside it (this one is a dark night sky)

      std::remove(path.c_str());
    };
  }
}
