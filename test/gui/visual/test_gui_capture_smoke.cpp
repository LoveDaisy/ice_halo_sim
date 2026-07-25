// Capture-harness smoke test — the end-to-end proof that the visual-regression harness
// (FullFrameCaptureState + lumice::test::CheckAgainstReference + the reference-group
// registry in scripts/regen_gui_test_refs.py) accepts a reference group other than auto_ev.
//
// Scene: the default GUI frame right after ResetTestState() — no simulation, no random
// source — so it doubles as the coverage for FullFrameCaptureState itself.
//
// Category "capture_harness" is deliberately not "smoke": gui_test's --filter is a
// case-insensitive SUBSTRING match on name OR category (imgui_te_engine::PassFilter), so a
// "smoke" category would also be selected by the existing gui_smoke tests and pollute the
// PSNR sampling that the regen driver does per group.

#include <cstdio>
#include <string>
#include <vector>

#include "test_gui_shared.hpp"

// Calibrated by scripts/regen_gui_test_refs.py Phase B (--group capture_harness) as
// floor((mean − 3σ) · 2) / 2; see test/gui/references/_thresholds.json.
static constexpr double kPsnrThreshold = 40.0;

void RegisterCaptureHarnessTests(ImGuiTestEngine* engine) {
  ImGuiTest* t = IM_REGISTER_TEST(engine, "capture_harness", "fullframe");
  // No GuiFunc: capture happens in the main loop's post-RenderDrawData hook.
  t->TestFunc = [](ImGuiTestContext* ctx) {
    ResetTestState();
    g_fullframe_capture.Reset();

    // Eliminate hover state: a highlighted card baked into the reference would make
    // every later no-hover run fail (same rationale as screenshot/left_panel_psnr).
    ctx->MouseMoveToPos(ImVec2(-100.0f, -100.0f));
    ctx->Yield(3);

    // rect_w/rect_h left at 0 → whole default framebuffer.
    g_fullframe_capture.requested.store(true);

    // Poll up to 10 frames for the main-loop hook to complete the readback.
    for (int i = 0; i < 10 && !g_fullframe_capture.done.load(); ++i) {
      ctx->Yield(1);
    }
    IM_CHECK(g_fullframe_capture.done.load());

    // Size gate before pixel comparison — GLFW hidden-window Retina scaling has been
    // observed to differ between cold and warm starts, and a silently resized capture
    // must fail loudly rather than produce a garbage PSNR.
    IM_CHECK_GT(g_fullframe_capture.width, 0);
    IM_CHECK_GT(g_fullframe_capture.height, 0);
    fprintf(stderr, "[capture_harness] fullframe: captured size = %dx%d\n", g_fullframe_capture.width,
            g_fullframe_capture.height);

    // "Non-zero" gate: catches a readback that silently returned zeros or missed the
    // ImGui draw, which would otherwise pass as a black-image comparison.
    bool has_nonzero = false;
    for (size_t i = 0; i < g_fullframe_capture.pixels.size() && !has_nonzero; ++i) {
      if (g_fullframe_capture.pixels[i] != 0) {
        has_nonzero = true;
      }
    }
    IM_CHECK(has_nonzero);

    // Tmp filename must match ReferenceGroup.tmp_prefix in scripts/regen_gui_test_refs.py.
    const std::string tmp_path = "/tmp/lumice_capture_harness_fullframe.png";
    const std::string ref_path = std::string(LUMICE_TEST_REF_DIR) + "/smoke_fullframe.jpg";
    auto rgb = lumice::test::StripAlpha(g_fullframe_capture.pixels.data(), g_fullframe_capture.width,
                                        g_fullframe_capture.height);
    IM_CHECK(
        lumice::test::SavePng(tmp_path.c_str(), rgb.data(), g_fullframe_capture.width, g_fullframe_capture.height, 3));

    IM_CHECK(lumice::test::CheckAgainstReference("capture_harness", "fullframe", tmp_path, ref_path, kPsnrThreshold,
                                                 g_keep_export_png));
  };
}
