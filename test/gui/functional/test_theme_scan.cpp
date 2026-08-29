// The oracle for "does this colour follow the palette?" — the class-B half of theme closure.
//
// The question. theme.cpp owns a Palette, but a colour only follows it if the call site READS it.
// A literal ImVec4/IM_COL32 written at a call site produces exactly the same pixel whatever palette
// is installed, and nothing in the build says so: it compiles, it renders, it looks deliberate. The
// mechanical statement of the defect is therefore a differential one — swap the palette for a
// deliberately distant one, and any pixel that DID NOT MOVE was painted by something that ignored
// the palette.
//
// What these cases are, and are not. They are the scene battery that differential needs: each one
// drives the app into a state where one known family of literals is on screen, then exports the
// rectangle that contains it. They compare nothing. The comparison is done outside, by
// scripts/scan_theme_leaks.py, which runs this same category twice — once with the production
// palette, once with --theme-palette contrast — and reports the pixels that came out equal.
//
// And the output of THAT is a candidate list, never a verdict: a colour can legitimately be
// palette-independent (a user-chosen data colour, a mark drawn on the rendered image). Every
// candidate is dispositioned by hand against the criteria in doc/gui-visual-language.md section 7.
//
// Why they are still assertions rather than a bare export. Each case checks that the state it meant
// to reach was actually reached (the tinted branch is live, the modal really opened) and that the
// capture came back non-empty. Without that, a scene that silently stopped reaching its literal
// would export a clean-looking PNG and the scan would report "no candidates here" — a false green
// produced by the scene, not by the code under test.
//
// These are B1 (a rendered frame is the observable) and they live in gui_test for that reason.

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "IconsFontAwesome6.h"
#include "gui/app.hpp"
#include "gui/color_window.hpp"
#include "gui/edit_modals.hpp"
#include "gui/gui_logger.hpp"
#include "gui/gui_state.hpp"
#include "gui/log_sink.hpp"
#include "test_gui_shared.hpp"

namespace {

using lumice::gui::g_state;

// Filename the two runs pair by. scripts/scan_theme_leaks.py matches PNGs across its two export
// directories by name alone, so the name has to be a function of the scene and nothing else.
std::string ScenePath(const char* scene) {
  return GuiTestTempPath(std::string("theme_scan_") + scene + ".png").string();
}

// Read a window's live rectangle out of the default framebuffer and write it as a PNG.
//
// The rect comes from the window rather than from constants so the scene keeps framing its subject
// after a layout change. ImGui is origin-top-left in window coordinates and glReadPixels is
// origin-bottom-left in framebuffer pixels, hence the flip; the Retina scale is applied here for
// the same reason functional/test_status_bar.cpp applies it — the hook takes framebuffer pixels.
//
// window_ref == nullptr captures the whole framebuffer (rect_w left at 0).
bool ExportRegion(ImGuiTestContext* ctx, const char* window_ref, const char* scene) {
  g_fullframe_capture.Reset();
  if (window_ref != nullptr) {
    ImGuiWindow* win = ctx->GetWindowByRef(window_ref);
    if (win == nullptr) {
      IM_ERRORF("scene %s: window %s is not up", scene, window_ref);
      return false;
    }
    const ImGuiIO& io = ImGui::GetIO();
    const float sx = io.DisplayFramebufferScale.x;
    const float sy = io.DisplayFramebufferScale.y;
    const float fb_w = io.DisplaySize.x * sx;
    const float fb_h = io.DisplaySize.y * sy;
    const ImVec2 vp_pos = ImGui::GetMainViewport()->Pos;
    // Clipped to the framebuffer, and that is load-bearing rather than defensive. ImGui clamps
    // every window to style.WindowMinSize (32 px), so the 28 px status bar is 32 px tall and its
    // last four rows sit BELOW the bottom of the window. Reading them back yields four rows that
    // are black under either palette — 12.5% of that capture, reported as unchanged, i.e. a
    // candidate the scan invented out of its own framing. Any rectangle read here is intersected
    // with the framebuffer first, so what the scan sees is only pixels the app actually drew.
    const float x0 = std::max(0.0f, (win->Pos.x - vp_pos.x) * sx);
    const float y0 = std::max(0.0f, (win->Pos.y - vp_pos.y) * sy);
    const float x1 = std::min(fb_w, (win->Pos.x - vp_pos.x + win->Size.x) * sx);
    const float y1 = std::min(fb_h, (win->Pos.y - vp_pos.y + win->Size.y) * sy);
    g_fullframe_capture.rect_x = static_cast<int>(std::lround(x0));
    g_fullframe_capture.rect_y = static_cast<int>(std::lround(fb_h - y1));
    g_fullframe_capture.rect_w = static_cast<int>(std::lround(x1 - x0));
    g_fullframe_capture.rect_h = static_cast<int>(std::lround(y1 - y0));
  }
  g_fullframe_capture.requested.store(true);
  for (int i = 0; i < 10 && !g_fullframe_capture.done.load(); ++i) {
    ctx->Yield(1);
  }
  if (!g_fullframe_capture.done.load()) {
    IM_ERRORF("scene %s: the framebuffer readback did not complete", scene);
    return false;
  }
  if (g_fullframe_capture.width <= 0 || g_fullframe_capture.height <= 0) {
    IM_ERRORF("scene %s: captured %dx%d", scene, g_fullframe_capture.width, g_fullframe_capture.height);
    return false;
  }
  // A readback that quietly returned zeros compares equal to itself under both palettes, i.e. it
  // reports the whole rectangle as a candidate. Catching it here keeps that out of the scan.
  bool has_nonzero = false;
  for (size_t i = 0; i < g_fullframe_capture.pixels.size() && !has_nonzero; ++i) {
    has_nonzero = g_fullframe_capture.pixels[i] != 0;
  }
  if (!has_nonzero) {
    IM_ERRORF("scene %s: the capture is entirely black", scene);
    return false;
  }
  const std::vector<unsigned char> rgb = lumice::test::StripAlpha(
      g_fullframe_capture.pixels.data(), g_fullframe_capture.width, g_fullframe_capture.height);
  const std::string path = ScenePath(scene);
  if (!lumice::test::SavePng(path.c_str(), rgb.data(), g_fullframe_capture.width, g_fullframe_capture.height, 3)) {
    IM_ERRORF("scene %s: could not write %s", scene, path.c_str());
    return false;
  }
  ctx->LogInfo("[theme_scan] %s -> %s (%dx%d)", scene, path.c_str(), g_fullframe_capture.width,
               g_fullframe_capture.height);
  return true;
}

// Park the cursor off-window and let the frame settle before capturing. A hovered widget paints a
// different colour token than the one the scene is aiming at, and a tooltip would cover it.
void Settle(ImGuiTestContext* ctx) {
  ctx->MouseMoveToPos(ImVec2(-100.0f, -100.0f));
  ctx->Yield(3);
}

// One color class over the default document's only placement — enough to put the top bar into its
// "colors are configured" branch. Mirrors functional/test_color_window.cpp's MakeMatchAllClass;
// copied rather than shared because that one is file-local there and this needs only the shape.
gui::ColorClassConfig MakeMatchAllClass() {
  gui::ColorClassConfig cls;
  cls.color[0] = 1.0f;
  cls.color[1] = 0.0f;
  cls.color[2] = 0.0f;
  cls.visible = true;
  cls.z_order = 0;
  gui::ColorClassRefConfig ref;
  ref.layer_idx = 0;
  ref.crystal_pool_id = g_state.layers[0].entries[0].crystal_id;
  ref.match_all = true;
  cls.match.push_back(ref);
  return cls;
}

// A second entry bound to the SAME pool slot and filter as the first: that is what CountEntriesSharing
// counts, so it is what puts the LINK badge on both cards.
void AddEntrySharingSlotZero(ImGuiTestContext* ctx) {
  gui::EntryCard shared;
  shared.crystal_id = g_state.layers[0].entries[0].crystal_id;
  shared.filter_id = g_state.layers[0].entries[0].filter_id;
  g_state.layers[0].entries.push_back(shared);
  gui::g_thumbnail_cache.OnLayerStructureChanged();
  ctx->Yield(3);
}

// The GUI logger's sinks and level are process-wide and gui_test is one process. Same shape as
// functional/test_log_panel.cpp's guard, and for the same reason: a case that leaves a sink
// attached changes what every later case sees.
struct ScopedLogPanel {
  bool prev_enable;
  std::shared_ptr<gui::ImGuiLogSink> prev_sink;
  std::vector<spdlog::sink_ptr> prev_sinks;
  spdlog::level::level_enum prev_level;
  int prev_gui_log_level;

  ScopedLogPanel()
      : prev_enable(g_enable_log_panel), prev_sink(gui::g_imgui_log_sink), prev_sinks(gui::GetGuiLogger().sinks()),
        prev_level(gui::GetGuiLogger().level()), prev_gui_log_level(g_state.gui_log_level) {
    g_enable_log_panel = true;
    gui::g_imgui_log_sink = std::make_shared<gui::ImGuiLogSink>();
    gui::GetGuiLogger().sinks().push_back(gui::g_imgui_log_sink);
    gui::GetGuiLogger().set_level(spdlog::level::trace);
  }
  ~ScopedLogPanel() {
    gui::GetGuiLogger().sinks() = prev_sinks;
    gui::GetGuiLogger().set_level(prev_level);
    g_state.gui_log_level = prev_gui_log_level;
    gui::g_imgui_log_sink = prev_sink;
    g_enable_log_panel = prev_enable;
    g_state.log_panel_open = false;
  }
};

}  // namespace

void RegisterThemeScanTests(ImGuiTestEngine* engine) {
  // The two top-bar controls that paint themselves: the Colors button, tinted while the document
  // has color classes, and the Colored checkbox, tinted while the preview is showing a composite.
  // Both are "content state projected onto a control", and both currently choose their own blue.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "theme_scan", "topbar_badges");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(3);
      g_state.raypath_color.push_back(MakeMatchAllClass());
      // Ground truth for the checkbox's label AND its tint branch. SyncFromPoller is the only
      // production writer, and it does nothing without a server, so this survives to the frame
      // that draws it.
      g_state.last_uploaded_as_composite = true;
      ctx->Yield(4);

      // The scene really is in the branch it exists to photograph. Without this the case would
      // still export a plausible top bar, and the scan would read "no candidates" from a top bar
      // that never entered either branch.
      IM_CHECK(gui::ShouldTintColorsButton(g_state.raypath_color.empty()));
      IM_CHECK(ctx->ItemExists("##TopBar/Colored##CompositePreviewToggle"));

      Settle(ctx);
      ExportRegion(ctx, "##TopBar", "topbar_badges");

      g_state.last_uploaded_as_composite = false;
      g_state.raypath_color.clear();
      ctx->Yield(2);
    };
  }

  // The status bar's run-state word. Each state is its own export because they are three separate
  // literals in one switch, and a single capture would only ever contain one of them.
  //
  // sim_state is not written directly: SyncFromPoller derives it from run_intent every tick, so a
  // direct write does not survive to the drawing frame (the mechanism functional/test_status_bar.cpp
  // documents). "Stopping..." is unreachable that way — with no live server a kStopping intent is
  // promoted immediately — so it has no scene here and is dispositioned by reading the switch it
  // shares with the two that do.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "theme_scan", "status_bar_states");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(3);

      struct State {
        const char* scene;
        gui::RunIntent intent;
        bool dirty;
      };
      static const State kStates[] = {
        { "status_ready", gui::RunIntent::kNone, false },  // GoodTextColor()
        { "status_simulating", gui::RunIntent::kRunning, false },
        { "status_done", gui::RunIntent::kLoaded, false },
        { "status_modified", gui::RunIntent::kLoaded, true },  // WarningTextColor()
      };
      for (const State& s : kStates) {
        g_state.run_intent = s.intent;
        g_state.dirty = s.dirty;
        Settle(ctx);
        ExportRegion(ctx, "##StatusBar", s.scene);
        // One row per loop level: every ImGuiTestContext action opens with `if (IsError()) return`,
        // so a reported failure here turns every later row into a shadow of the first.
        if (ctx->IsError()) {
          break;
        }
      }
      g_state.run_intent = gui::RunIntent::kNone;
      g_state.dirty = false;
      ctx->Yield(2);
    };
  }

  // The log panel paints one line per severity. Two of the five branches are literals (trace/debug
  // grey, the default white) and two go through semantic_colors.hpp — so this one rectangle holds
  // both a suspected leak and a suspected exemption, which is the comparison the disposition needs.
  //
  // Logged through the logger's runtime log() rather than the GUI_LOG_* macros: those compile out
  // below SPDLOG_ACTIVE_LEVEL, and this target does not set it.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "theme_scan", "log_panel_levels");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ScopedLogPanel scoped;
      g_state.log_panel_open = true;
      g_state.gui_log_level = 0;
      ctx->Yield(3);

      gui::GetGuiLogger().log(spdlog::level::trace, "theme_scan trace line");
      gui::GetGuiLogger().log(spdlog::level::debug, "theme_scan debug line");
      gui::GetGuiLogger().log(spdlog::level::info, "theme_scan default line");
      gui::GetGuiLogger().log(spdlog::level::warn, "theme_scan warning line");
      gui::GetGuiLogger().log(spdlog::level::err, "theme_scan error line");
      ctx->Yield(4);

      // All five reached the ring buffer the panel reads. A level filtered out at the logger would
      // leave its colour branch undrawn and silently out of the scan.
      IM_CHECK(gui::g_imgui_log_sink->Size() >= (size_t)5);

      Settle(ctx);
      ExportRegion(ctx, "##LogPanel", "log_panel_levels");
    };
  }

  // The entry card's chrome: the thumbnail's placeholder fill and its border, the dimming an
  // excluded card gets, and the LINK badge two cards sharing a pool slot both carry.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "theme_scan", "entry_card_chrome");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(3);
      AddEntrySharingSlotZero(ctx);
      g_state.layers[0].entries[1].enabled = false;
      ctx->Yield(4);

      IM_CHECK_EQ((int)g_state.layers[0].entries.size(), 2);
      IM_CHECK_EQ(gui::CountEntriesSharing(g_state, g_state.layers[0].entries[0].crystal_id,
                                           g_state.layers[0].entries[0].filter_id),
                  2);

      Settle(ctx);
      ExportRegion(ctx, "##LeftPanel", "entry_card_chrome");
    };
  }

  // The thumbnail PLACEHOLDER, which the scene above cannot photograph however long it waits: the
  // cache renders a texture within a frame or two of the cards appearing, and from then on the card
  // composites an image. The placeholder is what stands in until then, and it had its own pair of
  // greys.
  //
  // Reaching it deterministically is a matter of arithmetic rather than timing luck. The cache
  // rebuilds at most kMaxThumbnailUpdatesPerFrame (2) slots per frame, so seven cards on seven
  // distinct pool slots, invalidated all at once, cannot all be back before the readback lands —
  // and the case asserts afterwards that at least one really was still waiting, so a future change
  // to that budget turns this into a red rather than into a scene that quietly photographs nothing.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "theme_scan", "thumb_placeholder");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(3);
      std::vector<int> ids;
      ids.push_back(g_state.layers[0].entries[0].crystal_id);
      for (int i = 0; i < 6; ++i) {
        gui::CrystalConfig extra;
        gui::EntryCard card;
        card.crystal_id = static_cast<int>(g_state.crystals.size());
        g_state.crystals.push_back(extra);
        g_state.layers[0].entries.push_back(card);
        ids.push_back(card.crystal_id);
      }
      gui::g_thumbnail_cache.OnLayerStructureChanged();
      // Long enough for every card to have got its texture, so what follows is a transition from
      // "all present" to "most missing" rather than a state that was never established. The cursor
      // is parked BEFORE the invalidation, not after: Settle()'s three frames are three frames of
      // rebuilding, which is most of the budget this scene is spending.
      Settle(ctx);
      ctx->Yield(40);
      gui::g_thumbnail_cache.InvalidateAll();
      // One frame, and exactly one. The test engine's coroutine runs at the END of an ImGui frame,
      // so a cache invalidated here first reaches the cards on the NEXT frame — while the capture
      // hook fires at the end of the CURRENT one. Requesting the readback without this yield
      // photographs the frame that was drawn before the invalidation, i.e. the textures. Each
      // further frame rebuilds two more slots, so anything beyond one gives the queue time to
      // finish and the scene stops reaching the branch at all.
      ctx->Yield(1);
      ExportRegion(ctx, "##LeftPanel", "thumb_placeholder");
      if (ctx->IsError()) {
        return;
      }

      int still_pending = 0;
      for (int id : ids) {
        if (gui::g_thumbnail_cache.GetTexture(id) == 0) {
          ++still_pending;
        }
      }
      ctx->LogInfo("[theme_scan] thumb_placeholder: %d/%d slots still rendering at capture time", still_pending,
                   static_cast<int>(ids.size()));
      IM_CHECK_GT(still_pending, 0);
    };
  }

  // The co-shared border: with the edit modal open on one card, every OTHER card bound to the same
  // (crystal, filter) pair is outlined to say the in-flight edit will reach it too. That outline is
  // the one card-chrome colour not reachable from the scene above, because it needs a modal open.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "theme_scan", "card_co_shared");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      const ScopedPopups popup_guard(ctx);
      ctx->Yield(3);
      AddEntrySharingSlotZero(ctx);

      gui::EditRequest req{ gui::EditTarget::kCrystal, /*layer_idx=*/0, /*entry_idx=*/0 };
      gui::OpenEditModal(req, g_state);
      ctx->Yield(4);
      IM_CHECK(gui::IsEditModalOpen());
      IM_CHECK_EQ(gui::GetEditModalTarget().entry_idx, 0);

      Settle(ctx);
      ExportRegion(ctx, "##LeftPanel", "card_co_shared");
    };
  }

  // The sync-group swatch, which is where the palette-independent colours the criterion is supposed
  // to EXEMPT actually live: the six-colour group table is indexed by group number, and the digit
  // drawn on a swatch picks black or white from that fill's luminance. Both are in this rectangle,
  // next to ordinary themed chrome, so the scan reports them together and the disposition has to
  // separate them by reason rather than by which scene they came from.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "theme_scan", "sync_swatch_probe");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      const ScopedPopups popup_guard(ctx);
      ctx->Yield(2);

      // Two groups, chosen for their luminance: group 2's amber crosses the 0.55 luma threshold that
      // selects a black digit, group 1's blue does not and gets a white one. One capture therefore
      // contains both branches of the derived-contrast rule.
      g_state.crystals[g_state.layers[0].entries[0].crystal_id].height.sync_group = 2;
      g_state.crystals[g_state.layers[0].entries[0].crystal_id].face_distance[2].sync_group = 1;
      ctx->Yield(2);

      gui::EditRequest req{ gui::EditTarget::kCrystal, /*layer_idx=*/0, /*entry_idx=*/0 };
      gui::OpenEditModal(req, g_state);
      ctx->Yield(4);
      IM_CHECK(gui::IsEditModalOpen());
      ctx->ItemOpen("**/Face Distance##modal");
      ctx->Yield(4);
      IM_CHECK_NE(ctx->ItemInfo("**/##sync_Face 3##modal_fd", ImGuiTestOpFlags_NoError).ID, (ImGuiID)0);

      Settle(ctx);
      ExportRegion(ctx, "Edit Entry", "sync_swatch_probe");
    };
  }

  // The Overlay group's table and its drag fields, in all three states a drag field has. They are
  // expected to be clean — they were written against the theme's tokens — and that is exactly why
  // they are here: an oracle that only visits the places already known to be dirty cannot report
  // that anywhere is clean.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "theme_scan", "overlay_table_controls");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(3);
      g_state.show_grid_line = true;
      ctx->Yield(3);

      const ImGuiTestItemInfo drag = ctx->ItemInfo("//##RightPanel/**/##grid_alpha");
      IM_CHECK_NE(drag.ID, (ImGuiID)0);

      Settle(ctx);
      ExportRegion(ctx, "##RightPanel", "overlay_table_default");
      if (ctx->IsError()) {
        return;
      }

      // Hover, then drag. The two states push different frame-background tokens, and the point of
      // photographing them separately is that a literal in either one is invisible in the other.
      ctx->MouseMove("//##RightPanel/**/##grid_alpha");
      ctx->Yield(3);
      ExportRegion(ctx, "##RightPanel", "overlay_table_hover");
      if (ctx->IsError()) {
        return;
      }

      ctx->MouseDown(ImGuiMouseButton_Left);
      ctx->Yield(3);
      const bool active = ImGui::GetActiveID() == drag.ID;
      ExportRegion(ctx, "##RightPanel", "overlay_table_active");
      ctx->MouseUp(ImGuiMouseButton_Left);
      ctx->Yield(2);
      IM_CHECK(active);

      g_state.show_grid_line = false;
      ctx->Yield(2);
    };
  }
}
