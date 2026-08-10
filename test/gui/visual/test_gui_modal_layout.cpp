// Edit-modal layout pixel regression — a disk-reference baseline for the internal control
// layout of the Crystal / Filter edit popup (src/gui/edit_modals.cpp): slider/input widths,
// property-table column widths, control ordering, and the modal's own auto-resized size.
//
// Why this exists: every other gui_test that opens the modal asserts *state* (a click changed
// a field) or *geometry invariants* (a width is above some floor). None of them reads a
// committed image, so a change to a column width, a slider length or the order of two rows
// could not turn a test red — the acceptance path for those was a developer eyeballing the
// modal locally. Three rounds of modal polish landed that way.
//
// Capture path: unlike lens_proj (which renders through its own off-screen FBO and is
// therefore independent of the window), this suite reads the DEFAULT framebuffer through
// g_fullframe_capture's sub-region protocol, using the live ImGui window rectangle of
// "Edit Entry". That is the point — the thing under test is how ImGui laid the modal out on
// screen, so the on-screen pixels are the only faithful source. The consequence is that these
// references are tied to the harness window size, the font atlas and the ImGui style; any of
// those moving is a legitimate reason to re-run scripts/regen_gui_test_refs.py.
//
// Category "modal_layout" doubles as the "[modal_layout]" tag CheckAgainstReference prints,
// which is how scripts/regen_gui_test_refs.py attributes PSNR samples to this group in a
// shared full-suite stderr. It must therefore stay unique across groups.

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "IconsFontAwesome6.h"  // ICON_FA_* selectors: the modal buttons carry icon-prefixed labels
#include "gui/panels.hpp"
#include "test_gui_shared.hpp"

namespace {

enum class FilterKind { kNone, kRaypath, kEntryExit };

struct ModalLayoutScene {
  const char* name;
  gui::EditTarget tab;
  gui::CrystalType crystal_type;
  FilterKind filter;
  // Target layout. The test always drives H->V (or V->H) as a toggle while the modal is
  // already open, never by pre-setting the flag; see the snap note in the test body.
  bool vertical;
  // Crystal tab only: expand the default-collapsed "Face Distance" section, which swaps in a
  // second 6-row property table. Does not change the modal's size (the content pane is a
  // fixed-height child), only what is drawn inside it.
  bool expand_face_distance;
  // Asserted before the capture. This is the direct guard for the H<->V snap: without the
  // toggle the vertical modal keeps the ~820 px width the stretch column last converged to
  // instead of snapping to kEditModalMinWidthVertical, and the reference would silently be
  // shot in the wrong layout.
  float expect_width;
  double psnr_threshold;
};

// Every scene here is deterministic — no simulation, no RNG, and the modal's crystal preview
// pins its sample seed to kPreviewFixedSampleSeed whenever the shape carries no randomization
// (edit_modals.cpp: AdvancePreviewAnimSeed), which is the case for all four. So they compare
// pixel-identical (PSNR=inf) and there is no finite mean − 4σ to calibrate: Phase A measured
// zero pixel variance over 10 full-suite runs, and Phase B found 60/60 runs bit-identical for
// every scene, recording the driver's deterministic floor. 40 dB is therefore not a sampled
// statistic but the repo-wide floor for deterministic GL comparisons (visual/left_panel,
// visual/crystal_preview_prism, capture_harness/fullframe) — bit-exactness cannot be demanded of a
// committed reference compared on another machine's GL stack.
// See groups.modal_layout in test/gui/references/_thresholds.json.
static constexpr double kDeterministicThresholdDb = 40.0;

// clang-format off
static const ModalLayoutScene kScenes[] = {
  {"crystal_prism",   gui::EditTarget::kCrystal, gui::CrystalType::kPrism,   FilterKind::kNone,      false, false, 820.0f, kDeterministicThresholdDb},
  {"crystal_pyramid", gui::EditTarget::kCrystal, gui::CrystalType::kPyramid, FilterKind::kNone,      true,  true,  420.0f, kDeterministicThresholdDb},
  {"filter_raypath",  gui::EditTarget::kFilter,  gui::CrystalType::kPrism,   FilterKind::kRaypath,   false, false, 820.0f, kDeterministicThresholdDb},
  {"filter_ee",       gui::EditTarget::kFilter,  gui::CrystalType::kPrism,   FilterKind::kEntryExit, false, false, 820.0f, kDeterministicThresholdDb},
};
// clang-format on
static constexpr int kSceneCount = sizeof(kScenes) / sizeof(kScenes[0]);

// Where the modal is parked before the capture. ImGui remembers a window's position in memory
// for the whole process and does NOT re-center a modal that merely changed size, so the
// position "Edit Entry" happens to be at when a scene runs is a function of which earlier test
// last opened it: measured, the same scene sat at y=48 running under --filter modal_layout and
// at y=93 under the full suite, which for the 884 px vertical modal leaves 3 px of headroom
// above the framebuffer bottom. Pinning the position makes the "modal fits on screen" gate a
// property of this suite rather than of the test order. The value is arbitrary except that both
// the 820x517 and 420x884 layouts must fit inside 1600x980 from here.
constexpr float kModalParkX = 20.0f;
constexpr float kModalParkY = 20.0f;

void BuildSceneState(const ModalLayoutScene& scene) {
  auto& entry = gui::g_state.layers[0].entries[0];
  gui::g_state.crystals[entry.crystal_id].type = scene.crystal_type;

  switch (scene.filter) {
    case FilterKind::kNone:
      break;
    case FilterKind::kRaypath: {
      gui::FilterConfig f;
      f.name = "rp";
      f.SetRaypath(gui::RaypathParams{ "3-5-1" });
      gui::SetFilter(gui::g_state, entry, f);
      break;
    }
    case FilterKind::kEntryExit: {
      gui::FilterConfig f;
      f.name = "ee";
      gui::EntryExitParams ep;
      ep.entry_text = "3";
      ep.exit_text = "5";
      gui::SetFilter(gui::g_state, entry, f);
      gui::g_state.filters[*entry.filter_id].SetEntryExit(ep);
      gui::g_state.filters[*entry.filter_id].name = "ee";
      break;
    }
  }
}

}  // namespace

void RegisterModalLayoutTests(ImGuiTestEngine* engine) {
  for (int idx = 0; idx < kSceneCount; idx++) {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "modal_layout", kScenes[idx].name);
    t->ArgVariant = idx;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      const auto& scene = kScenes[ctx->Test->ArgVariant];
      ResetTestState();
      g_fullframe_capture.Reset();

      BuildSceneState(scene);

      // The vertical (420 px) and horizontal (820 px) widths are only reached via
      // SetNextWindowSize on the frame RenderEditModals observes modal_layout_vertical
      // FLIP (edit_modals.cpp). Without a flip the window keeps whatever width
      // AlwaysAutoResize last converged to, which the stretch column then holds — the
      // "~800 wide vertical modal" trap. So: pre-set the OPPOSITE of the target while the
      // modal is closed (that flip is consumed harmlessly — a popup that is not open
      // clears NextWindowData), open, then flip to the target while it is open.
      gui::g_state.modal_layout_vertical = !scene.vertical;
      ctx->Yield(2);

      gui::EditRequest req{ scene.tab, /*layer_idx=*/0, /*entry_idx=*/0 };
      gui::OpenEditModal(req, gui::g_state);
      ctx->Yield(4);

      gui::g_state.modal_layout_vertical = scene.vertical;
      ctx->Yield(6);

      ImGuiWindow* win = ctx->GetWindowByRef("Edit Entry");
      IM_CHECK(win != nullptr);
      IM_CHECK(win->WasActive);

      // Select the tab explicitly rather than relying on OpenEditModal's
      // g_active_tab + g_pending_tab_select. That intent does NOT survive the layout toggle
      // above: the two layouts host the TabBar in different child windows
      // (##modal_right_pane vs ##modal_bottom_pane), so toggling builds a fresh ImGuiTabBar
      // that falls back to its first tab (Crystal) — long after the one-shot
      // ImGuiTabItemFlags_SetSelected was consumed. Measured: without this click the two
      // filter scenes captured the Crystal tab and came out byte-identical to crystal_prism,
      // i.e. three scenes' worth of green covering one scene's worth of pixels.
      ctx->ItemClick(scene.tab == gui::EditTarget::kFilter ? "**/###filter_tab" : "**/###crystal_tab");
      ctx->Yield(3);

      // Face Distance is a collapsible section whose open state lives in ImGui storage and
      // survives ResetTestState. ItemOpen/ItemClose are idempotent (they read the item's
      // Opened status first), so normalizing it here makes each scene independent of
      // whatever the previous test left behind, in either direction.
      if (scene.tab == gui::EditTarget::kCrystal) {
        if (scene.expand_face_distance) {
          ctx->ItemOpen("**/Face Distance##modal");
        } else {
          ctx->ItemClose("**/Face Distance##modal");
        }
        ctx->Yield(3);
      }

      // Pin the modal's position (see kModalParkX/Y). Done after the layout toggle so the drag
      // target is computed against the final size, and before the geometry is read below.
      ctx->WindowMove("Edit Entry", ImVec2(kModalParkX, kModalParkY));
      ctx->Yield(2);

      // Park the mouse off-window: a hovered widget bakes a highlight into the reference and
      // every later no-hover run then fails (same rationale as visual/left_panel).
      // Must come after WindowMove, which drives the mouse to the title bar to drag it.
      ctx->MouseMoveToPos(ImVec2(-100.0f, -100.0f));
      ctx->Yield(4);

      // Window and framebuffer geometry come from ImGui's IO rather than from GLFW directly:
      // glfwGetCurrentContext() is thread-local and returns null on the test coroutine's
      // thread. imgui_impl_glfw's NewFrame fills DisplaySize from glfwGetWindowSize and
      // DisplayFramebufferScale from framebuffer/window, so these are the same two numbers
      // the capture hook's glfwGetFramebufferSize will report.
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
      fprintf(stderr, "[modal_layout] %s: fb=%dx%d win=%.0fx%.0f modal pos=(%.1f,%.1f) size=(%.1f,%.1f)\n", scene.name,
              fb_w, fb_h, win_w, win_h, lx, ly, win->Size.x, win->Size.y);

      IM_CHECK_EQ(win->Size.x, scene.expect_width);
      IM_CHECK_EQ(lx, kModalParkX);
      IM_CHECK_EQ(ly, kModalParkY);

      // ImGui (origin top-left, window coords) -> glReadPixels (origin bottom-left, framebuffer).
      int rx = static_cast<int>(std::lround(lx * sx));
      int ry = static_cast<int>(std::lround((win_h - (ly + win->Size.y)) * sy));
      int rw = static_cast<int>(std::lround(win->Size.x * sx));
      int rh = static_cast<int>(std::lround(win->Size.y * sy));
      fprintf(stderr, "[modal_layout] %s: capture rect = (%d,%d,%d,%d)\n", scene.name, rx, ry, rw, rh);

      // "Not clipped" (AC2) as a machine check rather than a look at the picture: a modal
      // whose bottom ran off the framebuffer would otherwise be captured as a shorter,
      // internally-consistent image and pass forever against an equally truncated reference.
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

      // Catches a readback that silently returned zeros or missed the ImGui draw, which
      // would otherwise pass as a black-image comparison.
      bool has_nonzero = false;
      for (size_t i = 0; i < g_fullframe_capture.pixels.size() && !has_nonzero; ++i) {
        if (g_fullframe_capture.pixels[i] != 0) {
          has_nonzero = true;
        }
      }
      IM_CHECK(has_nonzero);

      // Tmp filename must match ReferenceGroup.tmp_prefix in scripts/regen_gui_test_refs.py.
      const std::string tmp_path = GuiTestTempPath(std::string("lumice_modal_layout_") + scene.name + ".png").string();
      const std::string ref_path = std::string(LUMICE_TEST_REF_DIR) + "/modal_layout_" + scene.name + ".png";
      auto rgb = lumice::test::StripAlpha(g_fullframe_capture.pixels.data(), g_fullframe_capture.width,
                                          g_fullframe_capture.height);
      IM_CHECK(lumice::test::SavePng(tmp_path.c_str(), rgb.data(), g_fullframe_capture.width,
                                     g_fullframe_capture.height, 3));

      // Leave the collapsible section as this suite found it. On an IM_CHECK failure above
      // this line is not reached (the macro expands to a bare `return`) and ImGuiTestContext
      // methods no-op once the test is in error, so a *failing* modal_layout scene can leave
      // Face Distance expanded for later tests. That is accepted: the suite is already red at
      // that point, and every scene here normalizes the section on entry anyway.
      if (scene.tab == gui::EditTarget::kCrystal && scene.expand_face_distance) {
        ctx->ItemClose("**/Face Distance##modal");
        ctx->Yield(2);
      }

      // Close the modal. Not strictly required — ResetTestState -> ResetModalState clears
      // g_active_modal, and BeginPopupModal closes a popup whose p_open reads false on the very
      // next frame — but leaving one open would hand the following test a frame of stale modal
      // for no reason. Placed after the capture so it cannot influence the compared pixels.
      ctx->ItemClick("**/" ICON_FA_XMARK " Cancel##edit_modal");
      ctx->Yield(2);

      IM_CHECK(lumice::test::CheckAgainstReference("modal_layout", scene.name, tmp_path, ref_path, scene.psnr_threshold,
                                                   g_keep_export_png));
    };
  }
}
