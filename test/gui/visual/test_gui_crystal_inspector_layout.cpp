// Document inspector crystal-page layout pixel regression — a disk-reference baseline for the
// internal control layout of the Crystal / Filter tabs on the document inspector's crystal page
// (src/gui/edit_modals.cpp: RenderCrystalInspector): slider/input widths, property-table column
// widths and control ordering.
//
// This group replaces visual/test_gui_modal_layout.cpp, retired when the Crystal / Axis / Filter
// edit popup was replaced by a persistent inspector page (doc/gui-layout-architecture.md §2). The
// proposition it protects is unchanged — nothing else in this suite reads a committed image of the
// page, so a column width, a slider length or a row order could drift with only a developer's eye
// to catch it — only the host changed: a fixed-size docked window that scrolls its content instead
// of a popup that auto-resized to fit it. That is why there is no width-per-scene table here the
// way the old suite had one (crystal_prism 820 vs crystal_pyramid 420): the inspector's width is the
// document column's, constant across every scene, and functional/test_edit_modal.cpp already pins
// it to kLeftPanelWidth.
//
// Capture path: the DEFAULT framebuffer through g_fullframe_capture's sub-region protocol, using
// the live ImGui window rectangle of "##DocumentInspector" — same technique as modal_layout and
// defaults_panel_layout, for the same reason (the thing under test is how ImGui laid the page out
// on screen). Unlike those two, this window is DOCKED rather than floating, so there is no
// WindowMove/park step: its position and size come from the default dock layout, which is
// deterministic under gui_test (io.IniFilename is always nullptr — no persisted layout to drift
// with). The consequence carried over unchanged: these references are tied to the harness window
// size, the font atlas and the ImGui style, and any of those moving is a legitimate reason to
// re-run scripts/regen_gui_test_refs.py --group crystal_inspector_layout.
//
// Scene names are unchanged from modal_layout on purpose — crystal_prism and crystal_pyramid still
// name the same two crystal-tab depths (the pyramid scene is the deepest: Face Distance expanded,
// swapping in a second 6-row property table), and filter_raypath / filter_ee still name the same two
// filter kinds. test/gui/visual/test_preview_pixels.cpp already documents why the collision with its
// own crystal_preview_-prefixed names is harmless (distinct [group] tags).

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "gui/dock_layout.hpp"  // kDocumentInspectorWindowName — the page's host window
#include "gui/panels.hpp"
#include "test_gui_shared.hpp"

namespace {

enum class FilterKind { kNone, kRaypath, kEntryExit };

struct CrystalInspectorScene {
  const char* name;
  gui::CrystalType crystal_type;
  FilterKind filter;
  bool open_filter_tab;
  // Crystal tab only: expand the default-collapsed "Face Distance" section, which swaps in a
  // second 6-row property table. This is the "deepest" scene the group's job description names —
  // the widest sweep of rows any scene here submits.
  bool expand_face_distance;
  double psnr_threshold;
};

// Every scene here is deterministic — no simulation, no RNG, and the page's crystal preview pins
// its sample seed to kPreviewFixedSampleSeed whenever the shape carries no randomization
// (edit_modals.cpp: AdvancePreviewAnimSeed), which is the case for all four. So they compare
// pixel-identical (PSNR=inf) and there is no finite mean − 4σ to calibrate — the same deterministic
// floor modal_layout used to record. See groups.crystal_inspector_layout in
// test/gui/references/_thresholds.json.
constexpr double kDeterministicThresholdDb = 40.0;

// clang-format off
const CrystalInspectorScene kScenes[] = {
  { "crystal_prism",   gui::CrystalType::kPrism,   FilterKind::kNone,      false, false, kDeterministicThresholdDb },
  { "crystal_pyramid", gui::CrystalType::kPyramid, FilterKind::kNone,      false, true,  kDeterministicThresholdDb },
  { "filter_raypath",  gui::CrystalType::kPrism,   FilterKind::kRaypath,   true,  false, kDeterministicThresholdDb },
  { "filter_ee",       gui::CrystalType::kPrism,   FilterKind::kEntryExit, true,  false, kDeterministicThresholdDb },
};
// clang-format on
constexpr int kSceneCount = sizeof(kScenes) / sizeof(kScenes[0]);

void BuildSceneState(const CrystalInspectorScene& scene) {
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

void RegisterCrystalInspectorLayoutTests(ImGuiTestEngine* engine) {
  for (int idx = 0; idx < kSceneCount; idx++) {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "crystal_inspector_layout", kScenes[idx].name);
    t->ArgVariant = idx;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      const auto& scene = kScenes[ctx->Test->ArgVariant];
      ResetTestState();
      g_fullframe_capture.Reset();

      // The document column's split ratio outlives ResetTestState (dock geometry is not g_state,
      // see ResetTestState's own comment in test_gui_main.cpp), and at least one other suite
      // deliberately drags it — restoring afterwards, but not to bit-identical quantization
      // (test_document_column.cpp: "Not an exact height: the separator lands where the pointer
      // left it"). Measured: the inspector's captured height differs (488 px run in isolation vs
      // 700 px after the full suite's drag-and-restore case ran first) unless this is pinned. View
      // -> Reset Layout is the same round trip test_shell_chrome.cpp's
      // reset_layout_clears_a_collapsed_left_panel exercises, and it fully rebuilds the dock tree
      // from the same literal ratios every time, so it is what makes this capture rectangle a
      // function of the harness alone rather than of whichever tests happened to run earlier.
      ctx->ItemClick("##TopBar/View");
      ctx->Yield(2);
      ctx->ItemClick("**/Reset Layout");
      ctx->Yield(3);

      BuildSceneState(scene);

      if (scene.open_filter_tab) {
        OpenFilterTab(ctx);
      } else {
        OpenCrystalTab(ctx);
      }
      ctx->Yield(4);

      if (scene.expand_face_distance) {
        ctx->ItemOpen("**/Face Distance##modal");
        ctx->Yield(3);
      }

      ImGuiWindow* win = ctx->GetWindowByRef(gui::kDocumentInspectorWindowName);
      IM_CHECK(win != nullptr);
      IM_CHECK(win->WasActive);

      // Park the mouse off-window: a hovered widget bakes a highlight into the reference and every
      // later no-hover run then fails (same rationale as visual/left_panel and modal_layout).
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
      fprintf(stderr, "[crystal_inspector_layout] %s: fb=%dx%d win pos=(%.1f,%.1f) size=(%.1f,%.1f)\n", scene.name,
              fb_w, fb_h, lx, ly, win->Size.x, win->Size.y);

      // ImGui (origin top-left, window coords) -> glReadPixels (origin bottom-left, framebuffer).
      const int rx = static_cast<int>(std::lround(lx * sx));
      const int ry = static_cast<int>(std::lround((win_h - (ly + win->Size.y)) * sy));
      const int rw = static_cast<int>(std::lround(win->Size.x * sx));
      const int rh = static_cast<int>(std::lround(win->Size.y * sy));

      // "Not clipped" as a machine check rather than a look at the picture: a page whose bottom
      // ran off the framebuffer would otherwise be captured as a shorter, internally-consistent
      // image and pass forever against an equally truncated reference.
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
      const std::string tmp_path =
          GuiTestTempPath(std::string("lumice_crystal_inspector_layout_") + scene.name + ".png").string();
      const std::string ref_path =
          std::string(LUMICE_TEST_REF_DIR) + "/crystal_inspector_layout_" + scene.name + ".png";
      auto rgb = lumice::test::StripAlpha(g_fullframe_capture.pixels.data(), g_fullframe_capture.width,
                                          g_fullframe_capture.height);
      IM_CHECK(lumice::test::SavePng(tmp_path.c_str(), rgb.data(), g_fullframe_capture.width,
                                     g_fullframe_capture.height, 3));

      // Leave the collapsible section as this suite found it, same rationale modal_layout recorded:
      // on an IM_CHECK failure above this line is not reached, but the suite is already red at that
      // point and every scene here normalizes the section on entry anyway.
      if (scene.expand_face_distance) {
        ctx->ItemClose("**/Face Distance##modal");
        ctx->Yield(2);
      }

      IM_CHECK(lumice::test::CheckAgainstReference("crystal_inspector_layout", scene.name, tmp_path, ref_path,
                                                   scene.psnr_threshold, g_keep_export_png));
    };
  }
}
