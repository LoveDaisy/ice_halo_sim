// Display-strip layout pixel regression — a disk-reference baseline for the internal layout of the
// viewport's bottom strip (src/gui/app_panels.cpp: RenderDisplayStrip / RenderOverlaysTab): the
// Overlays table's column widths (swatch / Overlay / Line / Label / Alpha / fold) and row order.
//
// Why this exists: functional/test_overlay_controls.cpp asserts what a checkbox or drag DOES —
// which field flips, what value lands. Nothing there reads a pixel, so a column that collapsed to
// zero width or a row that silently stopped being built would stay green; the acceptance path for
// that was a developer eyeballing the strip locally, same gap modal_layout and defaults_panel_layout
// closed for their own windows. This group is one half of shell-test-strategy-and-reshoot's mandate
// to give the strip (new in the shell reorg, doc/gui-layout-architecture.md §4) the same coverage —
// the Grade tab's histogram/exposure controls are covered by their own functional and pixel suites
// already (test_preview_pixels.cpp, test_gui_lens_projection.cpp) and are not re-shot here.
//
// Capture path: the DEFAULT framebuffer through g_fullframe_capture's sub-region protocol, using the
// live ImGui window rectangle of "##DisplayStrip". Unlike modal_layout / defaults_panel_layout /
// crystal_inspector_layout, this window needs no WindowMove or dock-layout lookup at all: it is
// fixed-geometry chrome glued to the viewport's bottom edge (ImGuiWindowFlags_NoMove, positioned by
// SetNextPanelGeometry every frame, doc/gui-layout-architecture.md §4), so its rectangle is the same
// every run regardless of test order. The consequence carried over from the other three groups is
// unchanged: this reference is tied to the harness window size, the font atlas and the ImGui style,
// and any of those moving is a legitimate reason to re-run
// scripts/regen_gui_test_refs.py --group display_strip_layout.

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "gui/panels.hpp"
#include "test_gui_shared.hpp"

namespace {

// One scene: the Overlays table at its default state (every row unchecked, no fold expanded). The
// AC only asks for "at least one" scene — the table has no state-dependent column that a second
// scene would newly exercise; the fold popups are a separate ImGui window layered on top, not part
// of this window's own rectangle, so expanding one would not change what this capture covers.
constexpr double kDeterministicThresholdDb = 40.0;

}  // namespace

void RegisterDisplayStripLayoutTests(ImGuiTestEngine* engine) {
  ImGuiTest* t = IM_REGISTER_TEST(engine, "display_strip_layout", "overlays");
  t->TestFunc = [](ImGuiTestContext* ctx) {
    ResetTestState();
    g_fullframe_capture.Reset();

    OpenDisplayStripTab(ctx, "Overlays");
    ctx->Yield(4);

    ImGuiWindow* win = ctx->GetWindowByRef("##DisplayStrip");
    IM_CHECK(win != nullptr);
    IM_CHECK(win->WasActive);

    // Park the mouse off-window: a hovered widget bakes a highlight into the reference and every
    // later no-hover run then fails (same rationale as the other three layout groups).
    ctx->MouseMoveToPos(ImVec2(-100.0f, -100.0f));
    ctx->Yield(4);

    // Window and framebuffer geometry come from ImGui's IO rather than from GLFW directly:
    // glfwGetCurrentContext() is thread-local and returns null on the test coroutine's thread.
    const ImGuiIO& io = ImGui::GetIO();
    const float win_w = io.DisplaySize.x;
    const float win_h = io.DisplaySize.y;
    const float sx = io.DisplayFramebufferScale.x;
    const float sy = io.DisplayFramebufferScale.y;
    IM_CHECK_GT(win_w, 0.0f);
    IM_CHECK_GT(win_h, 0.0f);
    const int fb_w = static_cast<int>(std::lround(win_w * sx));
    const int fb_h = static_cast<int>(std::lround(win_h * sy));

    const ImVec2 vp_pos = ImGui::GetMainViewport()->Pos;
    const float lx = win->Pos.x - vp_pos.x;
    const float ly = win->Pos.y - vp_pos.y;
    fprintf(stderr, "[display_strip_layout] overlays: fb=%dx%d win pos=(%.1f,%.1f) size=(%.1f,%.1f)\n", fb_w, fb_h, lx,
            ly, win->Size.x, win->Size.y);

    // ImGui (origin top-left, window coords) -> glReadPixels (origin bottom-left, framebuffer).
    const int rx = static_cast<int>(std::lround(lx * sx));
    const int ry = static_cast<int>(std::lround((win_h - (ly + win->Size.y)) * sy));
    const int rw = static_cast<int>(std::lround(win->Size.x * sx));
    const int rh = static_cast<int>(std::lround(win->Size.y * sy));

    // "Not clipped" as a machine check rather than a look at the picture: a strip whose edge ran
    // off the framebuffer would otherwise be captured as a shorter, internally-consistent image and
    // pass forever against an equally truncated reference.
    IM_CHECK_GE(rx, 0);
    IM_CHECK_GE(ry, 0);
    IM_CHECK_LE(rx + rw, fb_w);
    IM_CHECK_LE(ry + rh, fb_h);

    g_fullframe_capture.rect_x = rx;
    g_fullframe_capture.rect_y = ry;
    g_fullframe_capture.rect_w = rw;
    g_fullframe_capture.rect_h = rh;
    g_fullframe_capture.requested.store(true);
    for (int i = 0; i < 10 && !g_fullframe_capture.done.load(); ++i) {
      ctx->Yield(1);
    }
    IM_CHECK(g_fullframe_capture.done.load());
    IM_CHECK_EQ(g_fullframe_capture.width, rw);
    IM_CHECK_EQ(g_fullframe_capture.height, rh);

    // Catches a readback that silently returned zeros or missed the ImGui draw, which would
    // otherwise pass as a black-image comparison.
    bool has_nonzero = false;
    for (size_t i = 0; i < g_fullframe_capture.pixels.size() && !has_nonzero; ++i) {
      if (g_fullframe_capture.pixels[i] != 0) {
        has_nonzero = true;
      }
    }
    IM_CHECK(has_nonzero);

    // Tmp filename must match ReferenceGroup.tmp_prefix in scripts/regen_gui_test_refs.py.
    const std::string tmp_path = GuiTestTempPath("lumice_display_strip_layout_overlays.png").string();
    const std::string ref_path = std::string(LUMICE_TEST_REF_DIR) + "/display_strip_layout_overlays.png";
    auto rgb = lumice::test::StripAlpha(g_fullframe_capture.pixels.data(), g_fullframe_capture.width,
                                        g_fullframe_capture.height);
    IM_CHECK(
        lumice::test::SavePng(tmp_path.c_str(), rgb.data(), g_fullframe_capture.width, g_fullframe_capture.height, 3));

    IM_CHECK(lumice::test::CheckAgainstReference("display_strip_layout", "overlays", tmp_path, ref_path,
                                                 kDeterministicThresholdDb, g_keep_export_png));
  };
}
