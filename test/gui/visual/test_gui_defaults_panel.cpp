// Defaults-panel layout pixel regression — a disk-reference baseline for the internal layout of
// the "Save Current as Defaults" modal (src/gui/defaults_panel.cpp): the section headers, the two
// tables' column widths, the warning column that 405.5 will fill, the source column and its Revert
// button, and the pinned action row.
//
// Why this exists: the functional suite (test/gui/functional/test_gui_defaults_panel.cpp) asserts
// what the panel DOES — which key lands in which section, what a click writes to disk. Nothing
// there reads a pixel, so a column that collapsed to zero width, a warning column that silently
// stopped being built, or an action row that drifted back inside the scrolling body would stay
// green. That last one is not hypothetical: it is the defect this task's own implementation hit.
//
// Capture path: the DEFAULT framebuffer through g_fullframe_capture's sub-region protocol, using
// the live ImGui window rectangle of the modal — the same technique as modal_layout, and for the
// same reason (the thing under test is how ImGui laid the panel out on screen). The consequence is
// the same too: these references are tied to the harness window size, the font atlas and the ImGui
// style, and any of those moving is a legitimate reason to re-run
// scripts/regen_gui_test_refs.py --group defaults_panel_layout.
//
// Isolation: every scene installs an explicit, freshly emptied user-config directory. Without it
// the panel would render whatever personal defaults the developer running the suite happens to
// have saved, and the committed reference would carry one machine's state — green there, a
// slightly-low PSNR everywhere else. gui_test's own baseline is already kDisabled; these scenes
// need a WRITABLE directory (one of them shows saved defaults), so they opt into an explicit one.

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "functional/user_defaults_test_env.hpp"
#include "gui/defaults_panel.hpp"
#include "gui/user_defaults.hpp"
#include "test_gui_shared.hpp"

namespace {

using lumice::test_user_defaults::FreshOverlayDir;
using lumice::test_user_defaults::ResetUserDefaultsChannels;
using nlohmann::json;

enum class SceneKind {
  kPendingChanges,  // §2 populated, §3 collapsed — what the File-menu entry opens
  kOtherExpanded,   // §3 expanded over a document whose defaults are already saved (Mine + Revert)
  kFiltered,        // the search box narrowing §2
  kNoChanges,       // nothing differs — §2's empty state
};

struct DefaultsPanelScene {
  const char* name;
  SceneKind kind;
  double psnr_threshold;
};

// Every scene is deterministic: no simulation, no RNG, and no value in the panel comes from
// anywhere but an explicitly set GuiState field or the scene's own override file. They therefore
// compare pixel-identical (PSNR=inf), which leaves mean − 4σ no finite sample; 40 dB is the
// repo-wide floor for deterministic GL comparisons rather than a calibrated statistic, because
// bit-exactness cannot be demanded of a reference compared on another machine's GL stack.
// See groups.defaults_panel_layout in test/gui/references/_thresholds.json.
constexpr double kDeterministicThresholdDb = 40.0;

// clang-format off
const DefaultsPanelScene kScenes[] = {
  { "pending_changes", SceneKind::kPendingChanges, kDeterministicThresholdDb },
  { "other_expanded",  SceneKind::kOtherExpanded,  kDeterministicThresholdDb },
  { "filtered",        SceneKind::kFiltered,       kDeterministicThresholdDb },
  { "no_changes",      SceneKind::kNoChanges,      kDeterministicThresholdDb },
};
// clang-format on
constexpr int kSceneCount = sizeof(kScenes) / sizeof(kScenes[0]);

// Where the modal is parked before the capture. ImGui remembers a window's position for the whole
// process, so without pinning it the captured rectangle would depend on which earlier test last
// moved a window — the same trap modal_layout documents. Arbitrary except that the 760x560 panel
// must fit inside the 1600x980 harness window from here.
constexpr float kPanelParkX = 40.0f;
constexpr float kPanelParkY = 40.0f;

// The values every "edited" scene shows. Spread across shapes (float / enum / array / nested
// object) so the reference covers more than one formatting branch of FormatDiffValue.
void ApplyEdits() {
  gui::g_state.bg_alpha = 0.42f;
  gui::g_state.renderer.fov = 95.0f;
  gui::g_state.renderer.lens_type = gui::kLensTypeFisheyeEqualArea;
  gui::g_state.renderer.background[1] = 0.5f;
  gui::g_state.sun.altitude = 33.0f;
}

// The regen driver picks .png or .jpg per scene (whichever is smaller without adding noise), so
// the extension is its decision, not this file's. Resolving it here keeps a future re-shoot that
// flips the format from silently failing with "reference not found".
std::string ResolveReferencePath(const char* scene_name) {
  const std::string base = std::string(LUMICE_TEST_REF_DIR) + "/defaults_panel_" + scene_name;
  std::error_code ec;
  if (std::filesystem::exists(base + ".png", ec)) {
    return base + ".png";
  }
  return base + ".jpg";
}

}  // namespace

void RegisterDefaultsPanelLayoutTests(ImGuiTestEngine* engine) {
  for (int idx = 0; idx < kSceneCount; idx++) {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel_layout", kScenes[idx].name);
    t->ArgVariant = idx;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      const auto& scene = kScenes[ctx->Test->ArgVariant];

      // Isolation BEFORE ResetTestState, not after: ResetTestState -> DoNew -> MakeNewDocumentState
      // reads the process-wide user-config source, so a guard installed afterwards would leave the
      // captured document built from whatever defaults the running machine has saved. Measured:
      // with the guard installed after, `gui_test --user-config <a real directory>` shot the
      // pending_changes scene at 20.7 dB against its own reference — a scene that looked isolated
      // and was not.
      const auto dir = FreshOverlayDir((std::string("visual_") + scene.name).c_str());
      lumice::test_user_defaults::ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);

      ResetTestState();
      ResetUserDefaultsChannels();
      g_fullframe_capture.Reset();

      if (scene.kind == SceneKind::kOtherExpanded) {
        // A document that already carries its own defaults, so §3 has both "Mine" rows (with a
        // Revert button) and "Factory" rows in the same table.
        json doc;
        doc["bg_alpha"] = 0.42f;
        doc["renderer"]["fov"] = 95.0f;
        IM_CHECK(gui::WriteUserDefaultsFile(dir, doc));
        gui::g_state = gui::MakeNewDocumentState();
      } else if (scene.kind != SceneKind::kNoChanges) {
        ApplyEdits();
      }

      gui::OpenDefaultsPanel(gui::g_state, gui::DefaultsPanelSection::kPendingChanges);
      ctx->Yield(4);

      if (scene.kind == SceneKind::kOtherExpanded) {
        ctx->ItemOpen("**/###defaults_other");
        ctx->Yield(3);
      } else if (scene.kind == SceneKind::kFiltered) {
        // KeyCharsReplaceEnter (inside ItemInputValue) presses Enter, which deactivates the input:
        // an ACTIVE InputText draws a blinking caret, and a caret whose phase depends on the frame
        // count is exactly the kind of pixel that makes a reference flake.
        ctx->ItemInputValue("**/###defaults_search", "renderer");
        ctx->Yield(3);
      }

      ImGuiWindow* win = ctx->GetWindowByRef(gui::kDefaultsPanelTitle);
      IM_CHECK(win != nullptr);
      IM_CHECK(win->WasActive);

      ctx->WindowMove(gui::kDefaultsPanelTitle, ImVec2(kPanelParkX, kPanelParkY));
      ctx->Yield(2);

      // Park the mouse off-window: a hovered row or button bakes a highlight into the reference,
      // and every later no-hover run then fails. Must come after WindowMove, which drives the
      // mouse to the title bar to drag it.
      ctx->MouseMoveToPos(ImVec2(-100.0f, -100.0f));
      ctx->Yield(4);

      // Geometry from ImGui's IO rather than GLFW directly: glfwGetCurrentContext() is
      // thread-local and returns null on the test coroutine's thread.
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
      fprintf(stderr, "[defaults_panel_layout] %s: fb=%dx%d panel pos=(%.1f,%.1f) size=(%.1f,%.1f)\n", scene.name, fb_w,
              fb_h, lx, ly, win->Size.x, win->Size.y);
      IM_CHECK_EQ(lx, kPanelParkX);
      IM_CHECK_EQ(ly, kPanelParkY);

      // ImGui (origin top-left, window coords) -> glReadPixels (origin bottom-left, framebuffer).
      const int rx = static_cast<int>(std::lround(lx * sx));
      const int ry = static_cast<int>(std::lround((win_h - (ly + win->Size.y)) * sy));
      const int rw = static_cast<int>(std::lround(win->Size.x * sx));
      const int rh = static_cast<int>(std::lround(win->Size.y * sy));

      // "Not clipped" as a machine check rather than a look at the picture: a panel running off
      // the framebuffer would be captured as a shorter, internally consistent image and would pass
      // forever against an equally truncated reference.
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

      // Catches a readback that silently returned zeros, which would otherwise pass as a
      // black-image comparison.
      bool has_nonzero = false;
      for (size_t i = 0; i < g_fullframe_capture.pixels.size() && !has_nonzero; ++i) {
        if (g_fullframe_capture.pixels[i] != 0) {
          has_nonzero = true;
        }
      }
      IM_CHECK(has_nonzero);

      // Tmp filename must match ReferenceGroup.tmp_prefix in scripts/regen_gui_test_refs.py.
      const std::string tmp_path = std::string("/tmp/lumice_defaults_panel_") + scene.name + ".png";
      const std::string ref_path = ResolveReferencePath(scene.name);
      auto rgb = lumice::test::StripAlpha(g_fullframe_capture.pixels.data(), g_fullframe_capture.width,
                                          g_fullframe_capture.height);
      IM_CHECK(lumice::test::SavePng(tmp_path.c_str(), rgb.data(), g_fullframe_capture.width,
                                     g_fullframe_capture.height, 3));

      // Close through the button, so the popup stack unwinds the way a user would leave it and the
      // next scene in this single-process suite starts from a closed panel.
      ctx->ItemClick("**/###defaults_close");
      ctx->Yield(2);

      IM_CHECK(lumice::test::CheckAgainstReference("defaults_panel_layout", scene.name, tmp_path, ref_path,
                                                   scene.psnr_threshold, g_keep_export_png));
    };
  }
}
