// The sky-colour swatch in the right panel's Display group.
//
// What this suite is for. `renderer.background` was reachable only from the Settings table until
// this control existed, and everything downstream of the field was already pinned: the shader
// identity and the three gates by functional/test_preview_background.cpp, the field's exclusion
// from re-sim by unit-correctness/gui/test_state_reconcile.cpp's inert row, the composite path by
// unit-correctness/server/test_composite_background.cpp, the CLI by
// e2e-correctness/test_background_*.py. What none of them touch is the CONTROL: whether the widget
// the user actually operates is bound to that field, and whether operating it keeps the promise the
// tier registration makes on its behalf.
//
// So both cases here drive the real picker with the mouse — click the swatch, drag inside the
// popup's saturation/value square, release — rather than assigning to the field. Assigning to the
// field is what the sibling suites already do, and it is precisely the step that cannot answer
// either question.
//
//   * the_sky_on_screen_is_the_sky_that_was_picked — the whole chain, twice: mouse ->
//     g_state.renderer.background -> PreviewParams -> shader -> pixel, compared against the closed
//     form the colour-space contract promises. Once off the DEFAULT FRAMEBUFFER (literally what is
//     on screen) and once off the export FBO. Two readings because every other pixel test of this
//     colour uses the export FBO, so "the preview shows it" has until now been an inference from
//     "the exporter produces it".
//   * picking_a_sky_colour_never_disturbs_the_simulation — the governance promise. Editing the sky
//     must not restart a run or mark the document as needing one, INCLUDING on the intermediate
//     frames of a drag, which is when a live preview writes the field most often.
//
// What a user sees when these break: the swatch does nothing; or the preview shows a colour other
// than the one in the picker; or dragging the swatch during a long run silently throws that run
// away and starts it again.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "gui/gui_state.hpp"
#include "imgui_internal.h"
#include "test_gui_shared.hpp"
#include "util/color_space.hpp"

namespace {

// RAII pairing for ImGuiTestContext::SetRef. A second copy of the six lines in
// test_overlay_controls.cpp, kept file-local for the same mechanical reason stated there: a fatal
// IM_CHECK expands to `return` in the enclosing lambda, so a tail-of-function SetRef("") runs only
// on the passing path — and a real regression here would otherwise leave the ref pinned to this
// panel for every case that runs afterwards, turning one red into a cascade.
struct ScopedRef {
  ScopedRef(ImGuiTestContext* ctx, const char* ref) : ctx_(ctx) { ctx_->SetRef(ref); }
  ~ScopedRef() { ctx_->SetRef(""); }

  ScopedRef(const ScopedRef&) = delete;
  ScopedRef& operator=(const ScopedRef&) = delete;

 private:
  ImGuiTestContext* ctx_;
};

// The panel window, and the ONE item inside the Display group whose id the engine can find by
// label. Everything else here is derived from it.
constexpr const char* kPanelRef = "//##RightPanel";
constexpr const char* kEvInput = "**/##EV##display_input";

// The swatch's id is REPRODUCED, not searched for: ColorButton never calls
// IMGUI_TEST_ENGINE_ITEM_INFO, so it registers no debug label and a "**/" lookup — which matches on
// labels — cannot reach it however it is spelled. The same derivation, and the same reason, as
// test_overlay_controls.cpp's five rows; this one is a seed shorter because the control is not in a
// table. Two seeds, outermost first: the window the row was submitted into, taken from a sibling
// item rather than assumed, and ColorEdit3's PushID of its own label, under which ImGui submits
// "##ColorButton".
ImGuiID SkySwatchID(ImGuiTestContext* ctx) {
  const ImGuiTestItemInfo ev = ctx->ItemInfo(kEvInput);
  if (ev.Window == nullptr) {
    return 0;
  }
  const ImGuiID edit_id = ImGui::GetIDWithSeed("Sky Color##display_sky_color", nullptr, ev.Window->ID);
  return ImGui::GetIDWithSeed("##ColorButton", nullptr, edit_id);
}

// Open the picker and drag across its saturation/value square, corner to corner. Two corners rather
// than "wherever the mouse already is", so the drag reaches a genuinely different colour whatever
// the document started at. `on_hold` runs on the frames where the button is still down — the ones a
// live control writes the field on, and the ones a re-sim guard has to survive.
//
// Leaves the popup open; the caller closes it, because a fatal report between here and there
// expands to `return` and a popup left open would be inherited by every case that follows.
void DragTheSkyPicker(ImGuiTestContext* ctx, void (*on_hold)()) {
  const ImGuiID swatch = SkySwatchID(ctx);
  IM_CHECK(swatch != 0);
  IM_CHECK(ctx->ItemExists(swatch));
  ctx->ItemClick(swatch);
  ctx->Yield(2);

  // Inside the picker popup now. "sv" is the saturation/value square — an ordinary InvisibleButton,
  // and unlike the swatch it IS registered with the test engine.
  IM_CHECK(ctx->ItemExists("**/sv"));
  const ImGuiTestItemInfo sv = ctx->ItemInfo("**/sv");
  const ImVec2 lo = sv.RectFull.Min;
  const ImVec2 hi = sv.RectFull.Max;
  const ImVec2 start(lo.x + (hi.x - lo.x) * 0.15f, lo.y + (hi.y - lo.y) * 0.15f);
  const ImVec2 end(lo.x + (hi.x - lo.x) * 0.85f, lo.y + (hi.y - lo.y) * 0.85f);

  ctx->MouseMoveToPos(start);
  ctx->MouseDown(0);
  ctx->Yield(2);
  if (on_hold != nullptr) {
    on_hold();
  }
  ctx->MouseMoveToPos(end);
  ctx->Yield(2);
  if (on_hold != nullptr) {
    on_hold();
  }
  ctx->MouseUp(0);
  ctx->Yield(2);
}

// The byte a channel of the picked sRGB colour must render as. The round trip through linear is
// what the whole colour-space contract buys, and spelling it out here rather than expecting
// `picked * 255` is what makes the expectation the CONTRACT rather than a restatement of the float
// that happens to be in the struct.
//
// Rounded, not truncated: this is read off an 8-bit FRAMEBUFFER, which rounds, whereas the CLI's
// `static_cast<uint8_t>(v * 255)` truncates. The same split is already recorded next door — the
// shader-side probes in test_preview_background.cpp round and match exactly, and that suite's
// bake-vs-shader comparison carries a 1 LSB allowance for precisely this narrowing.
int ExpectedSkyByte(float srgb_channel) {
  return static_cast<int>(std::lround(lumice::LinearToSrgb(lumice::SrgbToLinear(srgb_channel)) * 255.0f));
}

// One LSB, where the suite next door pins the same identity at zero. The difference is the probe:
// its colours are CHOSEN constants, picked away from any rounding boundary, while this case's
// colour is whatever a mouse drag across the picker landed on and can therefore sit arbitrarily
// close to one. The failures this case exists to catch — a control wired to nothing (byte 0), or a
// colour gamma-encoded twice — are 30 to 120 LSB away, so the allowance costs the claim nothing.
constexpr int kPickedToleranceLsb = 1;

// A zero radiance field, uploaded on request. The live preview panel only fills PreviewParams when
// it has something to show, and an upload is a GL call — so it happens in the GuiFunc rather than in
// the test coroutine. All-zero, so every pixel inside the shader's gate carries the background and
// nothing else and its expected value is closed form.
//
// UploadXyzTexture, NOT the uint8 UploadTexture beside it: the shader adds the sky only under
// `u_xyz_mode == 1` (preview_renderer.cpp), because mode 0 is a document loaded from disk whose
// pixels were baked WITH a background already and adding one again would apply it twice. The live
// simulation preview — the surface a user is looking at while dragging this control — is the XYZ
// one, so that is the mode this case has to measure in. Uploading the uint8 texture instead gives
// a fully black frame with no error anywhere, which is exactly what it did before this comment.
bool g_zero_upload_done = false;
std::vector<float> g_zero_tex;

void SkyControlGuiFunc(ImGuiTestContext* /*ctx*/) {
  if (!g_zero_upload_done) {
    g_zero_tex.assign(static_cast<size_t>(kSynthTexW) * kSynthTexH * 3, 0.0f);
    gui::g_preview.UploadXyzTexture(g_zero_tex.data(), kSynthTexW, kSynthTexH);
    g_zero_upload_done = true;
  }
}

// The four fields the tier registration promises a sky edit leaves alone, captured together so the
// "unchanged" claim is one comparison rather than four scattered ones.
struct SimWitness {
  bool dirty;
  gui::GuiState::SimState sim_state;
  uint64_t committed_epoch;
  uint64_t display_epoch_floor;

  static SimWitness Take() {
    return { gui::g_state.dirty, gui::g_state.sim_state, gui::g_state.committed_epoch,
             gui::g_state.display_epoch_floor };
  }
};

SimWitness g_before;

void ExpectSimUndisturbed(const char* when) {
  const SimWitness now = SimWitness::Take();
  if (now.dirty != g_before.dirty) {
    IM_ERRORF(
        "%s: dirty went %d -> %d. A sky edit changes what shows behind the rays, not the rays,"
        " so it must not ask for a re-run (gui_state_tiers.hpp carves renderer.background out"
        " of the resim projection).",
        when, static_cast<int>(g_before.dirty), static_cast<int>(now.dirty));
  }
  if (now.sim_state != g_before.sim_state) {
    IM_ERRORF("%s: sim_state went %d -> %d", when, static_cast<int>(g_before.sim_state),
              static_cast<int>(now.sim_state));
  }
  if (now.committed_epoch != g_before.committed_epoch) {
    IM_ERRORF("%s: committed_epoch went %llu -> %llu", when, (unsigned long long)g_before.committed_epoch,
              (unsigned long long)now.committed_epoch);
  }
  if (now.display_epoch_floor != g_before.display_epoch_floor) {
    IM_ERRORF(
        "%s: display_epoch_floor went %llu -> %llu — the display fence was raised, which"
        " discards payloads from the generation currently on screen.",
        when, (unsigned long long)g_before.display_epoch_floor, (unsigned long long)now.display_epoch_floor);
  }
}

void OnHoldExpectUndisturbed() {
  ExpectSimUndisturbed("mid-drag");
}

}  // namespace

void RegisterBackgroundMainUiControlTests(ImGuiTestEngine* engine) {
  // The whole chain, mouse to pixel. Which colour the drag lands on is not predicted — it cannot
  // be, and it does not need to be: the document records what was picked, and the claim is that
  // the screen agrees with the document. The two independent readings are what make that a real
  // comparison, and the "it moved off black" check below is what stops it being vacuous.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "background_main_ui_control", "the_sky_on_screen_is_the_sky_that_was_picked");
    t->GuiFunc = SkyControlGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      // ResetTestState delegates to DoNew, which drops the preview texture — so the upload flag has
      // to be cleared here for the GuiFunc to put one back. Without this the export below refuses
      // (ExportPreviewPng returns false on !HasTexture) and the failure names the export rather
      // than the missing texture.
      g_zero_upload_done = false;
      ctx->Yield(2);
      // A linear lens at fov 90 images the whole canvas and `full` removes the horizon cut, so
      // every exported pixel is inside the shader's gate — the centre sample below is then the
      // background and nothing else, with no dependence on where the image circle falls.
      gui::g_state.renderer.lens_type = gui::kLensTypeLinear;
      gui::g_state.renderer.fov = 90.0f;
      gui::g_state.renderer.visible = gui::kVisibleFull;
      ctx->Yield(3);

      const ScopedRef panel_ref(ctx, kPanelRef);
      const float factory[3] = { gui::g_state.renderer.background[0], gui::g_state.renderer.background[1],
                                 gui::g_state.renderer.background[2] };
      DragTheSkyPicker(ctx, nullptr);
      ctx->PopupCloseAll();
      ctx->Yield(3);

      const float picked[3] = { gui::g_state.renderer.background[0], gui::g_state.renderer.background[1],
                                gui::g_state.renderer.background[2] };
      // The swatch is bound to the document's own field — and the drag really moved, so the
      // pixel comparison below is not comparing black against black.
      IM_CHECK(picked[0] != factory[0] || picked[1] != factory[1] || picked[2] != factory[2]);

      IM_CHECK_GT(gui::g_preview_vp.vp_w, 0);
      IM_CHECK_GT(gui::g_preview_vp.vp_h, 0);

      // Reading 1 — the DEFAULT FRAMEBUFFER, i.e. the pixels actually on the user's screen.
      // Worth its own reading rather than inferring it from the export below: every existing pixel
      // test of this colour goes through RenderExportToRgba's off-screen FBO, so "what the preview
      // shows" has so far only ever been argued from "what the exporter produces". Same shader
      // program, different framebuffer and different viewport rectangle — the part that is argued
      // is exactly the part a wrong viewport or a stale bind would break. The capture rect is the
      // live preview viewport, in the framebuffer coordinates PreviewRenderer::Render was handed.
      // No overlay lines can land on the sample: every show_*_line defaults off (gui_state.hpp) and
      // nothing here turns one on.
      g_fullframe_capture.Reset();
      g_fullframe_capture.rect_x = gui::g_preview_vp.vp_x;
      g_fullframe_capture.rect_y = gui::g_preview_vp.vp_y;
      g_fullframe_capture.rect_w = gui::g_preview_vp.vp_w;
      g_fullframe_capture.rect_h = gui::g_preview_vp.vp_h;
      g_fullframe_capture.requested.store(true);
      for (int i = 0; i < 10 && !g_fullframe_capture.done.load(); ++i) {
        ctx->Yield(1);
      }
      IM_CHECK(g_fullframe_capture.done.load());
      IM_CHECK_EQ(g_fullframe_capture.width, gui::g_preview_vp.vp_w);
      IM_CHECK_EQ(g_fullframe_capture.height, gui::g_preview_vp.vp_h);
      {
        const int cw = g_fullframe_capture.width;
        const int chh = g_fullframe_capture.height;
        const size_t screen_off = ((static_cast<size_t>(chh / 2) * cw) + static_cast<size_t>(cw / 2)) * 4;
        IM_CHECK_LT(screen_off + 2, g_fullframe_capture.pixels.size());
        for (int j = 0; j < 3; ++j) {
          const int got = static_cast<int>(g_fullframe_capture.pixels[screen_off + j]);
          const int want = ExpectedSkyByte(picked[j]);
          if (std::abs(got - want) > kPickedToleranceLsb) {
            IM_ERRORF(
                "on screen, channel %d: the centre of the preview viewport reads %d, the picked sRGB %.4f"
                " renders as %d. This is the default framebuffer, not the export FBO.",
                j, got, static_cast<double>(picked[j]), want);
          }
        }
      }

      // Reading 2 — the export path, from the same live viewport params.
      const std::string path = GuiTestTempPath("lumice_sky_control.png").string();
      IM_CHECK(RequestAndWaitPreviewExport(ctx, gui::g_preview_vp, path));

      std::vector<unsigned char> img;
      int w = 0;
      int h = 0;
      int ch = 0;
      IM_CHECK(lumice::test::LoadPng(path.c_str(), img, w, h, ch));
      IM_CHECK_GT(w, 0);
      IM_CHECK_GT(h, 0);
      IM_CHECK_GE(ch, 3);

      const size_t off = ((static_cast<size_t>(h / 2) * w) + static_cast<size_t>(w / 2)) * ch;
      // Same claim, second entry point.
      for (int j = 0; j < 3; ++j) {
        const int got = static_cast<int>(img[off + j]);
        const int want = ExpectedSkyByte(picked[j]);
        if (std::abs(got - want) > kPickedToleranceLsb) {
          IM_ERRORF(
              "channel %d: the centre pixel reads %d, the picked sRGB %.4f renders as %d. 0 would mean the"
              " control never reached the renderer; a value far above %d would mean the colour was added"
              " after the gamma curve instead of before it.",
              j, got, static_cast<double>(picked[j]), want, want);
        }
      }
      std::remove(path.c_str());
    };
  }

  // The governance promise, on the control rather than on the field. test_state_reconcile.cpp
  // already pins that a renderer.background DIFF drives nothing; what it cannot say is whether the
  // widget reaches the field by a route that also touches something else. The sky colour is a
  // display-time setting today — the shader adds it when it draws — so the panel has no push path
  // of its own to the server; the point of this case is that it stays that way, because "nudge the
  // preview" and "restart the simulation" are one careless line apart.
  //
  // The mid-drag checks are the sharp half. This control writes the live field on every frame the
  // mouse is held, so a guard that only ran on release would leave the intermediate frames free to
  // mark the document dirty — and the user would see a "configuration changed" banner appear while
  // they were still choosing a colour.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "background_main_ui_control", "picking_a_sky_colour_never_disturbs_the_simulation");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(3);

      const ScopedRef panel_ref(ctx, kPanelRef);
      const float before[3] = { gui::g_state.renderer.background[0], gui::g_state.renderer.background[1],
                                gui::g_state.renderer.background[2] };
      g_before = SimWitness::Take();
      // The baseline this case is worth anything against: a document that already needed a re-run
      // would make "dirty is unchanged" true for the wrong reason.
      IM_CHECK(!g_before.dirty);

      DragTheSkyPicker(ctx, OnHoldExpectUndisturbed);
      ctx->PopupCloseAll();
      ctx->Yield(3);

      ExpectSimUndisturbed("after release");
      // Non-vacuity: the drag has to have actually edited something, or every check above is a
      // statement about a control that did nothing.
      IM_CHECK(gui::g_state.renderer.background[0] != before[0] || gui::g_state.renderer.background[1] != before[1] ||
               gui::g_state.renderer.background[2] != before[2]);
    };
  }
}
