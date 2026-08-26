#include <GLFW/glfw3.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>

#include "IconsFontAwesome6.h"
#include "gui/app.hpp"
#include "gui/aspect_ratio_rules.hpp"
#include "gui/color_window.hpp"
#include "gui/composite_exposure_push.hpp"
#include "gui/crystal_preview.hpp"
#include "gui/defaults_panel.hpp"
#include "gui/destructive_style.hpp"
#include "gui/edit_modals.hpp"
#include "gui/field_editor_registry.hpp"
#include "gui/gui_constants.hpp"
#include "gui/gui_ev_auto.hpp"
#include "gui/gui_logger.hpp"
#include "gui/overlay_labels.hpp"
#include "gui/panels.hpp"
#include "gui/preview_renderer.hpp"  // ComputeBgUvTransform / kBgModifierName
#include "gui/semantic_colors.hpp"
#include "gui/sim_state_rules.hpp"
#include "gui/sun_circle_rules.hpp"
#include "gui/theme.hpp"
#include "imgui.h"
#include "util/path_utils.hpp"  // PathToU8 — the pending export path is shown in the overwrite prompt

// =============================================================================
// GUI window z-order convention (task-gui-window-zorder, scrum-gui-polish-v12)
// -----------------------------------------------------------------------------
// ImGui z-order is decided by two ASYMMETRIC mechanisms (imgui.cpp 6550-6553
// for creation; 12776 for click splice; 12839-12842 for focus flag check):
//   1. First-time Begin (creation): if the window has
//      ImGuiWindowFlags_NoBringToFrontOnFocus, ImGui calls
//      g.Windows.push_front -> the window goes to the FRONT of g.Windows,
//      which is the BOTTOM of the visual z-order (rendered first).
//      Without the flag: g.Windows.push_back -> goes to the BACK of
//      g.Windows = TOP visually. This is the design intent: the flag
//      means "this window is background, never raise it".
//   2. Click / FocusWindow: BringWindowToDisplayFront splices the window to
//      the BACK of g.Windows (visually TOP) — UNLESS the window has
//      NoBringToFrontOnFocus, in which case the splice is skipped.
//
// Practical implication: NoBringToFrontOnFocus simultaneously
//   (a) places the window at the BOTTOM of g.Windows on creation, and
//   (b) freezes it there against click side-effects.
// Windows without the flag float ABOVE the NoBringToFrontOnFocus cluster.
//
// Layered model (from bottom to top):
//
//   Layer 4 (Top, ImGui-managed):
//     - Tooltip / DragDrop overlay (ImGui internal, automatic top)
//     - GetForegroundDrawList (debug overlay; not used in this project)
//
//   Layer 3 (Floating; default raise behavior — NO NoBringToFrontOnFocus,
//            so push_back on creation -> floats above the background cluster):
//     - Staged "Edit Entry" (BeginPopupModal, on ImGui popup stack -> always top)
//     - Immediate "Edit Entry" (ImGui::Begin regular window)
//     - "Unsaved Changes" (BeginPopupModal)
//     - "##LogPanel" — user-toggleable; raisable on click; sits naturally
//       above the LeftPanel / RightPanel cluster.
//     - ICON_FA_PALETTE " Colors" (color_window.cpp:508) — user-toggleable
//       floating window. Manual click detection in the background cluster
//       (e.g. RenderEntryCard's IsMouseHoveringRect path) MUST gate on
//       IsWindowHovered() or !io.WantCaptureMouse to avoid click-through
//       when Colors covers the panels beneath it
//       (task-color-window-mouse-capture / 346.2).
//
//   Background cluster (NoBringToFrontOnFocus, push_front on creation
//                       -> bottom of g.Windows):
//     - "##LeftPanel" / "##RightPanel" — fixed left/right strips.
//     - "##TopBar" / "##StatusBar" — fixed top/bottom bars.
//     - "##PreviewPanel" — transparent (NoBackground); the OpenGL preview
//       shader is rendered into this region between ImGui::Render and
//       SwapBuffers in main.cpp.
//     Within this cluster, push_front means the LATEST Begin'd window ends
//     up at index 0 (bottom). Visual order within the cluster is therefore
//     the REVERSE of main.cpp Render* call order. Cluster members do not
//     overlap each other, so this internal ordering has no visual effect.
//
// -----------------------------------------------------------------------------
// CHECKLIST when adding a new ImGui::Begin window (in this file or elsewhere):
//   1. Register its layer (Background cluster / Layer 3 / Layer 4) in the
//      model above.
//   2. If background cluster: flags MUST include
//      ImGuiWindowFlags_NoBringToFrontOnFocus. The window will be pushed to
//      the front of g.Windows on creation (bottom visually) and frozen
//      there against click splices. Within the cluster, visual stacking is
//      the REVERSE of main.cpp Render* call order — call later to render
//      LOWER. Usually irrelevant because cluster members do not overlap.
//   3. If Layer 3 (floating): do NOT add NoBringToFrontOnFocus. The window
//      will be push_back'd on creation and naturally float above the
//      background cluster, and clicks will splice it to the very top.
//   4. Code-review must reject any new Begin not registered here, or any
//      main.cpp Render* call order that contradicts this model.
//
// SCOPE of this convention:
// File-level soft constraint. No global automation gate; enforcement relies on
// code-review human inspection. (Whether to promote to CLAUDE.md or a
// clang-tidy check is left to task-closeout decision.)
// =============================================================================

namespace lumice::gui {

using SimState = GuiState::SimState;

namespace {
// With ImGuiConfigFlags_ViewportsEnable (gui-polish-v15), window positions
// and ForegroundDrawList coordinates are in absolute OS screen space, not
// relative to the main GLFW window. All fixed-layout panels must anchor to
// the main viewport's origin to stay inside the host window.
inline ImVec2 MainVpPos(float x, float y) {
  const ImGuiViewport* vp = ImGui::GetMainViewport();
  return ImVec2(vp->Pos.x + x, vp->Pos.y + y);
}

// Pin a chrome panel to the main viewport so ImGui never promotes it to an
// independent OS viewport. Without SetNextWindowViewport, panels that sit at
// the viewport edge (e.g. status bar at the bottom row) may be promoted,
// which makes them appear covered by the host window or float outside it.
inline void SetNextPanelGeometry(float x, float y, float w, float h) {
  const ImGuiViewport* vp = ImGui::GetMainViewport();
  ImGui::SetNextWindowPos(ImVec2(vp->Pos.x + x, vp->Pos.y + y));
  ImGui::SetNextWindowSize(ImVec2(w, h));
  ImGui::SetNextWindowViewport(vp->ID);
}
}  // namespace

void RenderTopBar(float window_width) {
  SetNextPanelGeometry(0, 0, window_width, kTopBarHeight);
  ImGui::Begin("##TopBar", nullptr,
               ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                   ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoBringToFrontOnFocus);

  // Left-panel collapse toggle (placed before Run/Stop; owns the leftmost slot of the top bar
  // so it can never overlap with panel-internal headers).
  {
    const char* left_toggle_label = g_state.left_panel_collapsed ? ICON_FA_CHEVRON_RIGHT "##left_panel_toggle" :
                                                                   ICON_FA_CHEVRON_LEFT "##left_panel_toggle";
    if (ImGui::Button(left_toggle_label)) {
      g_state.left_panel_collapsed = !g_state.left_panel_collapsed;
    }
    ImGui::SameLine();
    ImGui::TextDisabled("|");
    ImGui::SameLine();
  }

  // Run/Stop — fixed width (max of ALL three labels, incl. "Stopping…") to prevent layout shift on
  // toggle. `busy` widens the file-op gates below: New/Open/Save/backend-toggle stay disabled while
  // the backend is still draining an async Stop (kStopping), not just while simulating.
  bool simulating = IsSimulating(g_state.sim_state);
  bool stopping = IsStopping(g_state.sim_state);
  bool busy = IsBusy(g_state.sim_state);
  const auto& style = ImGui::GetStyle();
  const char* kRunLabel = ICON_FA_PLAY " Run";
  const char* kStopLabel = ICON_FA_STOP " Stop";
  const char* kStoppingLabel = ICON_FA_STOP " Stopping...";
  float run_stop_width = std::max({ ImGui::CalcTextSize(kRunLabel).x, ImGui::CalcTextSize(kStopLabel).x,
                                    ImGui::CalcTextSize(kStoppingLabel).x }) +
                         style.FramePadding.x * 2;
  if (simulating) {
    // Stop is a destructive action in the same sense as delete/remove — it shares their palette
    // rather than keeping a second, slightly different red of its own.
    PushDestructiveStyle();
    if (ImGui::Button(kStopLabel, ImVec2(run_stop_width, 0))) {
      DoStop();
    }
    PopDestructiveStyle();
  } else if (stopping) {
    // Async Stop in flight: greyed, disabled "Stopping…" (DoStop is idempotent, but the disabled
    // button makes the in-flight state unambiguous and blocks re-entry at the UI layer).
    ImGui::BeginDisabled();
    ImGui::Button(kStoppingLabel, ImVec2(run_stop_width, 0));
    ImGui::EndDisabled();
  } else {
    PushGoodButtonStyle();
    if (ImGui::Button(kRunLabel, ImVec2(run_stop_width, 0))) {
      DoRun(/*user_initiated=*/true);
    }
    PopGoodButtonStyle();
  }

  // Revert area — always rendered for stable layout, hidden when not modified.
  // Alpha=0 + BeginDisabled: invisible and non-interactive, but still occupies layout space.
  // The hidden area intercepts clicks, which is harmless in this horizontal toolbar context.
  bool modified = IsModified(g_state.sim_state);
  if (!modified) {
    ImGui::PushStyleVar(ImGuiStyleVar_Alpha, 0.0f);
  }
  ImGui::BeginDisabled(!modified);
  ImGui::SameLine();
  ImGui::TextColored(WarningTextColor(), ICON_FA_CIRCLE_EXCLAMATION);
  // task-349.2 Step 2 (AC1/AC3): tooltip explains what the ⚠ + Revert row
  // means. Source-agnostic wording (config changed, not "you added a color
  // class") — main-scene edits and color-class edits reach kModified through
  // the same ReconcileSimState pipeline, so a single tooltip covers both.
  // Attached to the icon rather than the button so the button's own hover
  // action (click to revert) is not shadowed. Only shown when modified,
  // since the row is BeginDisabled(alpha=0) otherwise.
  if (modified && ImGui::IsItemHovered()) {
    ImGui::SetTooltip(
        "Configuration changed since the last run.\n"
        "Click Run to re-simulate, or Revert to discard the changes.");
  }
  ImGui::SameLine();
  if (ImGui::SmallButton("Revert") && modified) {  // `&& modified`: redundant safety guard over BeginDisabled
    DoRevert();
  }
  ImGui::EndDisabled();
  if (!modified) {
    ImGui::PopStyleVar();
  }

  ImGui::SameLine();
  ImGui::TextDisabled("|");
  ImGui::SameLine();

  // File operations — New/Open disabled while busy (simulating OR async Stop draining); Save menu
  // itself stays enabled so read-only exports (Screenshot / Dual Fisheye Equal Area /
  // Equirectangular / Config JSON) remain reachable.
  ImGui::BeginDisabled(busy);
  if (ImGui::Button("New")) {
    CheckUnsavedAndDo(PendingAction::kNew);
  }
  ImGui::SameLine();
  if (ImGui::Button("Open")) {
    CheckUnsavedAndDo(PendingAction::kOpen);
  }
  ImGui::EndDisabled();
  ImGui::SameLine();
  {
    if (ImGui::Button("Save")) {
      ImGui::OpenPopup("SaveMenu");
    }
    if (ImGui::BeginPopup("SaveMenu")) {
      bool no_texture = !g_preview.HasTexture();
      bool has_server = g_server != nullptr && g_state.sim_state != GuiState::SimState::kIdle;
      ImGui::BeginDisabled(busy);
      if (ImGui::MenuItem("Save")) {
        DoSave();
      }
      if (ImGui::MenuItem("Save Copy")) {
        DoSaveAs();
      }
      ImGui::EndDisabled();
      ImGui::Separator();
      if (ImGui::MenuItem("Screenshot...", nullptr, false, !no_texture)) {
        DoExportPreviewPng();
      }
      if (ImGui::MenuItem("Dual Fisheye Equal Area...", nullptr, false, has_server)) {
        DoExportDualFisheyeEqualAreaPng();
      }
      if (ImGui::MenuItem("Equirectangular...", nullptr, false, has_server)) {
        DoExportEquirectangularPng();
      }
      if (ImGui::MenuItem("Config JSON...")) {
        DoExportConfigJson();
      }
      ImGui::Separator();
      // Personal defaults are NOT in this menu. They used to be, on the reasoning that "what a new
      // document starts from" is a file-scope decision and so belongs with the other export
      // commands. The reasoning was sound and the placement still failed the only test that
      // matters: a user looking for their settings does not open a Save menu to find them, and in
      // practice did not. The entry is now a Settings button in the top bar — see RenderTopBar.
      ImGui::MenuItem("Include Texture in .lmc", nullptr, &g_state.save_texture);
      ImGui::MenuItem("Include Overlay in Screenshot", nullptr, &g_state.screenshot_include_overlay);
      ImGui::EndPopup();
    }
  }

  ImGui::SameLine();
  ImGui::TextDisabled("|");
  ImGui::SameLine();

  // task-345.5 (⑥): dedicated "feature button" group, immediately right of
  // New/Open/Save. Colors is the first occupant; future cross-cutting toggles
  // unrelated to file I/O or panel layout should land here rather than
  // competing for status-bar space.
  //
  // task-gui-feedback-affordances Step 1 (AC2): visually distinguish "has color
  // classes" vs "no color classes" so the topbar signals whether coloring is
  // configured before the window is opened. Derived state (empty check on
  // raypath_color) — no new state source. Blue/purple tint avoids collision
  // with Run (green) / Stop (red).
  const bool tint_colors_button = ShouldTintColorsButton(g_state.raypath_color.empty());
  if (tint_colors_button) {
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.30f, 0.35f, 0.65f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.40f, 0.45f, 0.75f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.25f, 0.28f, 0.55f, 1.0f));
  }
  if (ImGui::Button(ICON_FA_PALETTE " Colors")) {
    // task-348.3 AC3 (⑦): apply the "default enable on open with no classes" rule
    // ONLY on the false→true transition of color_window_open. Doing it here (inside
    // the click branch, guarded by `opening`) — not per-frame inside RenderColorWindow
    // — is what makes the behavior "memory-preserving": the user can still manually
    // toggle Colored off after opening, and subsequent focus changes / clicks that do
    // not close-then-reopen the window will not overwrite that choice.
    const bool opening = !g_state.color_window_open;
    g_state.color_window_open = !g_state.color_window_open;
    if (opening && ShouldDefaultEnableColorsOnOpen(g_state.raypath_color.empty())) {
      g_state.show_composite_preview = true;
    }
  }
  if (tint_colors_button) {
    ImGui::PopStyleColor(3);
  }

  // task-colored-toggle-to-topbar (346.3): colored/full-spectrum display-time
  // toggle, relocated from the status bar (task-345.4) to sit next to Colors.
  // Gated identically (raypath_color non-empty) so it renders only when at
  // least one color class exists (AC4: no color classes ⇒ no checkbox,
  // matching pre-346.3 status-bar behavior). Label/checked-state reads
  // GROUND TRUTH (last_uploaded_as_composite), click writes the user
  // preference (show_composite_preview) — same read/write split as
  // gui_state.hpp:794-803. The checkbox living in ##TopBar (a window
  // independent of the Colors window's own render call) is itself the
  // persistent "currently in colored mode" marker required by AC3: closing
  // Colors does not touch this window.
  if (!g_state.raypath_color.empty()) {
    ImGui::SameLine();
    // task-349.3 (#4): revert 348.3 icon-only Button back to a plain-text Checkbox
    // (no ICON_FA_PALETTE prefix) so this display-time toggle reads visually
    // distinct from the icon-bearing "Colors" open-window button one slot to the
    // left (owner-rejected: two adjacent palette-icon controls confused which was
    // the toggle vs. the window opener). Semantic split unchanged from 345.4/346.3:
    // label + `checked` read GROUND TRUTH (last_uploaded_as_composite), click
    // writes via shared ToggleCompositePreview(g_state) — the shared writer was
    // introduced in 348.3 and stays after the widget-shape revert (a12: two
    // control sites, one write path).
    //
    // task-349.2 Step 3 (#6): read the shared signal cache BEFORE rendering the
    // Colored toggle so we can wrap it in BeginDisabled() when every configured
    // color class matches zero rays (composite would be empty; control would
    // appear "unclickable / non-responding" without visual explanation). The
    // 500 ms throttled poll is unaffected by call-site order — same source as
    // the Colors window and the aggregate pip below, so all three cannot drift.
    //
    // Style tokens: Checkbox renders as frame background + check mark, not a
    // button surface — accent must go on FrameBg/FrameBgHovered/CheckMark. Using
    // ImGuiCol_Button here would silently no-op (this was the 346.3→348.3 pitfall
    // recorded in learnings/code-quality.md; reverting the widget type must
    // re-swap the token set).
    const bool composite_now = g_state.last_uploaded_as_composite;
    const std::vector<int>& signal_flags = RefreshColorClassSignals(g_state, g_server);
    // task-fix-color-window-visibility-consistency: merged "composite would be
    // empty" predicate covers both prior triggers — no rays match, OR every
    // matching class is currently hidden (visible=false, or solo'd out by
    // another class). Single owner shared with the Colors-window Enable
    // checkbox so the two indicators cannot disagree.
    const bool composite_empty = NoVisibleMatchedColorClass(g_state, signal_flags);
    const char* mode_label = composite_now ? "Colored" : "Full Spectrum";
    const std::string checkbox_id = std::string(mode_label) + "##CompositePreviewToggle";
    bool checked = composite_now;
    if (composite_now) {
      ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.35f, 0.55f, 0.85f, 1.0f));
      ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, ImVec4(0.45f, 0.65f, 0.95f, 1.0f));
      ImGui::PushStyleColor(ImGuiCol_CheckMark, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
    }
    ImGui::BeginDisabled(composite_empty);
    if (Checkbox(checkbox_id.c_str(), &checked)) {
      ToggleCompositePreview(g_state);
    }
    ImGui::EndDisabled();
    if (composite_now) {
      ImGui::PopStyleColor(3);
    }
    // Tooltip logic (both enabled and disabled cases): AllowWhenDisabled so the
    // BeginDisabled() wrapper does not eat the hover. When disabled, show the
    // "no matches" reason (a12: shared string with the Colors-window mirror);
    // when enabled, show the existing "Currently: Colored / Full Spectrum" hint.
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
      if (composite_empty) {
        ImGui::SetTooltip("%s", kColorsDisabledNoMatchTooltip);
      } else {
        ImGui::SetTooltip(
            "Toggle colored composite / full-spectrum preview.\n"
            "Display-time only -- does not re-simulate or discard color classes.\n"
            "%s",
            composite_now ? "Currently: Colored" : "Currently: Full Spectrum");
      }
    }

    // task-348.1 Step 3 (① 反馈缺失): when every configured color class has no
    // matching rays, the Colored composite is empty and the button above appears
    // "unclickable / non-responding" (last_uploaded_as_composite never flips true).
    // Surface an aggregate warning pip here so the user sees WHY nothing changes —
    // reads the same shared signal cache as the Colors window's per-row warnings
    // (single source, a12), so both indicators agree by construction. Silent when
    // no class has non-empty match[] (matches per-row semantics).
    //
    // task-349.2 Step 3: the pip stays alongside the disabled button (both driven
    // by composite_empty) — the two indicators are complementary, not redundant:
    // the button greying is the immediate visual cue "cannot toggle now", the
    // pip is the persistent per-row / aggregate warning surface.
    if (composite_empty) {
      ImGui::SameLine();
      ImGui::PushStyleColor(ImGuiCol_Text, WarningTextColor());
      ImGui::TextUnformatted(ICON_FA_TRIANGLE_EXCLAMATION);
      ImGui::PopStyleColor();
      if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip(
            "No visible color class currently matches any rays -- the composite would be empty.\n"
            "Either no rays match (they may be blocked by a physical filter), or every matching\n"
            "class is currently hidden. Open the Colors window to inspect per-class details.");
      }
    }
  }

  // Personal defaults, promoted out of the Save menu. The panel it opens edits what a NEW document
  // starts from, so by file-scope logic it belonged with the export commands — but it was reached
  // by nobody who had not been told where it was, which is the only measure a settings entry has.
  //
  // Two deliberate choices, both worth knowing before this button gets tidied away again:
  //   - It is separated from the Colors group by the same "|" the file group uses, because the two
  //     are different kinds of control. Colors and the Colored checkbox are toggles that change
  //     what the viewport shows; this opens a modal that edits persistent preferences. Sharing a
  //     run of buttons with no break would let a modal launcher read as one more view toggle.
  //   - It is NOT gated on `busy`. Reading and editing personal defaults is independent of whether
  //     a simulation is running, same as Colors — a settings entry that disappears while the thing
  //     the user is watching runs is a settings entry they cannot find when they think to look.
  // The panel's preset library is one section inside it, so retuning a preset is still reachable;
  // it no longer has a menu item of its own pointing straight at that section.
  ImGui::SameLine();
  ImGui::TextDisabled("|");
  ImGui::SameLine();
  if (ImGui::Button(ICON_FA_GEAR " Settings")) {
    OpenDefaultsPanel(g_state, DefaultsPanelSection::kSettings);
  }

  // Right-panel collapse toggle — right-aligned so it sits flush with the right panel's outer edge.
  // Also note: when the right panel is already collapsed, RenderCollapsedStrip's internal button
  // still expands it; this top-bar toggle simply offers a symmetric alternate entry point.
  {
    const char* right_toggle_label = g_state.right_panel_collapsed ? ICON_FA_CHEVRON_LEFT "##right_panel_toggle" :
                                                                     ICON_FA_CHEVRON_RIGHT "##right_panel_toggle";
    // Use the max width of both label states so the button's left edge doesn't jitter when toggled.
    float w_expanded = ImGui::CalcTextSize(ICON_FA_CHEVRON_RIGHT "##right_panel_toggle", nullptr, true).x;
    float w_collapsed = ImGui::CalcTextSize(ICON_FA_CHEVRON_LEFT "##right_panel_toggle", nullptr, true).x;
    float btn_w = std::max(w_expanded, w_collapsed) + style.FramePadding.x * 2.0f;
    float right_edge = ImGui::GetWindowContentRegionMax().x;
    ImGui::SameLine();
    ImGui::SetCursorPosX(right_edge - btn_w);
    if (ImGui::Button(right_toggle_label)) {
      g_state.right_panel_collapsed = !g_state.right_panel_collapsed;
    }
  }

  ImGui::End();
}

namespace {
constexpr float kCollapseBtnSize = 20.0f;

// Draw a collapse/expand button as a foreground overlay using ImGui theme colors.
// Returns true if clicked. Coordinates are viewport-local; under multi-viewport
// the main-viewport origin is applied to reach absolute screen space used by
// ForegroundDrawList and io.MousePos.
bool OverlayButton(const char* label, float local_x, float local_y) {
  ImVec2 pos = MainVpPos(local_x, local_y);
  ImVec2 max(pos.x + kCollapseBtnSize, pos.y + kCollapseBtnSize);

  ImDrawList* fg = ImGui::GetForegroundDrawList();
  ImGuiIO& io = ImGui::GetIO();
  // The collapse strip is drawn directly to ForegroundDrawList without a Begin(), so no
  // ImGui window exists to gate against. Fall back to WantCaptureMouse, which is set by
  // NewFrame() when any real ImGui window (e.g. Colors) sits under the cursor. This
  // prevents click-through when a floating window covers the strip
  // (task-color-window-mouse-capture).
  bool hovered = !io.WantCaptureMouse &&
                 (io.MousePos.x >= pos.x && io.MousePos.x <= max.x && io.MousePos.y >= pos.y && io.MousePos.y <= max.y);
  bool clicked = hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left);

  ImU32 bg_col = ImGui::GetColorU32(clicked ? ImGuiCol_ButtonActive :
                                    hovered ? ImGuiCol_ButtonHovered :
                                              ImGuiCol_Button);
  fg->AddRectFilled(pos, max, bg_col, 3.0f);

  ImVec2 text_size = ImGui::CalcTextSize(label);
  float tx = pos.x + (kCollapseBtnSize - text_size.x) * 0.5f;
  float ty = pos.y + (kCollapseBtnSize - text_size.y) * 0.5f;
  fg->AddText(ImVec2(tx, ty), ImGui::GetColorU32(ImGuiCol_Text), label);

  return clicked;
}

// Draw the collapsed strip background + expand button via foreground draw list.
// No ImGui window needed — avoids WindowMinSize issues. Coordinates are
// viewport-local (see OverlayButton comment).
void RenderCollapsedStrip(const char* btn_label, float strip_x, float strip_y, float strip_h, bool* collapsed) {
  ImDrawList* fg = ImGui::GetForegroundDrawList();
  ImVec2 strip_min = MainVpPos(strip_x, strip_y);
  ImVec2 strip_max = MainVpPos(strip_x + kCollapseBtnSize, strip_y + strip_h);
  fg->AddRectFilled(strip_min, strip_max, ImGui::GetColorU32(ImGuiCol_WindowBg));
  float btn_y = strip_y + (strip_h - kCollapseBtnSize) * 0.5f;
  if (OverlayButton(btn_label, strip_x, btn_y)) {
    *collapsed = false;
  }
}
}  // namespace

void RenderLeftPanel(float window_height) {
  float panel_height = window_height - kTopBarHeight - kStatusBarHeight;

  if (g_state.left_panel_collapsed) {
    RenderCollapsedStrip(ICON_FA_CHEVRON_RIGHT, 0, kTopBarHeight, panel_height, &g_state.left_panel_collapsed);
    return;
  }

  // Pick-mode: Esc cancels (read here before any ImGui::Begin so the key event
  // isn't consumed by inner widgets first).
  bool pick_active_at_entry = g_state.pick_link_source.has_value();
  if (pick_active_at_entry && ImGui::IsKeyPressed(ImGuiKey_Escape, false)) {
    g_state.pick_link_source.reset();
  }
  // Remember whether pick was active at the start of this frame so we can
  // detect "pick just completed" at the bottom and re-open the modal.
  std::optional<GuiState::EntryRef> pick_source_at_entry =
      pick_active_at_entry ? g_state.pick_link_source : std::nullopt;

  SetNextPanelGeometry(0, kTopBarHeight, kLeftPanelWidth, panel_height);
  ImGui::Begin("##LeftPanel", nullptr,
               ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                   ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse |
                   ImGuiWindowFlags_NoBringToFrontOnFocus);

  // Pick-mode hint bar — render above the scroll area so the user always sees
  // the active-pick state and the Esc instruction. The actual click target is
  // each entry card (handled inside RenderEntryCard via InvisibleButton).
  if (pick_active_at_entry) {
    const auto& src = *g_state.pick_link_source;
    ImGui::PushStyleColor(ImGuiCol_Text, WarningTextColor());
    ImGui::TextWrapped("Pick mode: click an entry to share crystal/filter from Layer %d / Entry %d (Esc to cancel)",
                       src.layer_idx, src.entry_idx);
    ImGui::PopStyleColor();
    ImGui::Separator();
  }

  // ---- Layout: cards (scroll) + toolbar ----
  float avail_h = ImGui::GetContentRegionAvail().y;
  auto& style = ImGui::GetStyle();
  float toolbar_h = ImGui::GetFrameHeight() + style.ItemSpacing.y;
  float cards_h = std::max(0.0f, avail_h - toolbar_h);

  // Process thumbnail update queue before rendering cards
  g_thumbnail_cache.ProcessUpdateQueue(g_state, kMaxThumbnailUpdatesPerFrame);

  // ---- Card scroll area (fills panel above the toolbar) ----
  ImGui::BeginChild("##CardScroll", ImVec2(0, cards_h), ImGuiChildFlags_None);
  RenderScatteringSection(g_state);
  ImGui::EndChild();

  // ---- Bottom toolbar: add layer only (per-layer delete lives on the header row) ----
  ImGui::Spacing();
  if (ImGui::SmallButton("+ Layer")) {
    // Bind the new layer's seed entry to a fresh pool slot — see panels.cpp's
    // "+ Crystal" handler for the same rationale (avoid implicit link to slot 0).
    EntryCard new_entry;
    new_entry.crystal_id = static_cast<int>(g_state.crystals.size());
    g_state.crystals.emplace_back();
    Layer new_layer;
    new_layer.entries.push_back(new_entry);
    // Footgun #2 guard: the layer that WAS the last layer is about to become
    // an intermediate layer. If its prob is (near-)zero, no rays will reach the
    // new layer — the newly added layer would be silently dead. Promote to a
    // sensible continuation default so the user sees rays in the new layer by
    // default. This is a user-initiated state transition (not a load), so it
    // is not covered by the "don't silently rewrite loaded values" rule.
    // Zero-detection MUST use IsProbZero (same helper as panels.cpp) — a raw
    // == 0.0f would let a slider-dragged 1e-7 sneak past this promotion.
    if (!g_state.layers.empty() && IsProbZero(g_state.layers.back().probability)) {
      g_state.layers.back().probability = kDefaultContinuationProb;
    }
    g_state.layers.push_back(std::move(new_layer));
    g_thumbnail_cache.OnLayerStructureChanged();
  }

  // Process edit request: open modal if an edit button or card area was clicked
  if (GetEditRequest().target != EditTarget::kNone) {
    const auto& req = GetEditRequest();
    if (req.target == EditTarget::kCard) {
      const auto modal_tgt = GetEditModalTarget();
      if (!IsEditModalOpen() || modal_tgt.layer_idx != req.layer_idx || modal_tgt.entry_idx != req.entry_idx) {
        EditRequest resolved = req;
        resolved.target = IsEditModalOpen() ? GetActiveTabAsEditTarget() : EditTarget::kCrystal;
        OpenEditModal(resolved, g_state);
      }
    } else {
      OpenEditModal(req, g_state);
    }
    ResetEditRequest();
  }

  // Pick-mode cancel: blank area / panel-switch click.
  // If pick is still active after cards are rendered (no card's InvisibleButton consumed
  // the click), a left mouse click anywhere cancels pick. Covers clicking blank space in
  // the LeftPanel, the right panel, or any non-card widget. Esc was handled at frame start.
  if (g_state.pick_link_source.has_value() && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
    g_state.pick_link_source.reset();
    pick_source_at_entry.reset();  // suppress spurious modal re-open
  }

  // Pick-mode completion: if pick was active at frame entry but is now reset
  // (cleared by RenderEntryCard's pick-click handler), re-open the modal on
  // the SOURCE entry so the user resumes editing where they started. The
  // editing entry's crystal_id was just re-bound to the clicked card's
  // crystal, so also reset the singleton trackball view to that crystal's
  // default orientation — otherwise the modal preview keeps the old
  // crystal's rotation while the thumbnail (which always renders from the
  // entry's axis distribution) shows the new one. Cancel paths
  // (Esc / blank-area click) clear pick_source_at_entry and skip this
  // branch, so view reset only fires when a link was actually applied.
  if (pick_source_at_entry.has_value() && !g_state.pick_link_source.has_value()) {
    const auto& src = *pick_source_at_entry;
    const auto& editing_entry = g_state.layers[src.layer_idx].entries[src.entry_idx];
    ResetCrystalViewToCrystal(g_state.crystals[editing_entry.crystal_id]);
    EditRequest reopen;
    reopen.target = EditTarget::kCrystal;
    reopen.layer_idx = src.layer_idx;
    reopen.entry_idx = src.entry_idx;
    OpenEditModal(reopen, g_state);
  }

  ImGui::End();
}

void RenderRightPanel(GLFWwindow* window, float window_width, float window_height) {
  float panel_height = window_height - kTopBarHeight - kStatusBarHeight;

  if (g_state.right_panel_collapsed) {
    RenderCollapsedStrip(ICON_FA_CHEVRON_LEFT, window_width - kCollapseBtnSize, kTopBarHeight, panel_height,
                         &g_state.right_panel_collapsed);
    return;
  }

  float panel_x = window_width - kRightPanelWidth;
  SetNextPanelGeometry(panel_x, kTopBarHeight, kRightPanelWidth, panel_height);
  ImGui::Begin("##RightPanel", nullptr,
               ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                   ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoBringToFrontOnFocus);

  // ---- Scene Group ----
  if (ImGui::CollapsingHeader("Scene", ImGuiTreeNodeFlags_DefaultOpen)) {
    RenderSceneControls(g_state);
  }

  // Copy-model renderer: GuiState always owns a valid renderer by default construction.
  auto& r = g_state.renderer;

  // ---- View Group ----
  if (ImGui::CollapsingHeader("View", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::PushItemWidth(-(kLabelColWidth + ImGui::GetStyle().ItemSpacing.x));
    ImGui::SeparatorText("Lens");
    // Use BeginCombo + Selectable to honour kLensTypePresentationOrder (gui_state.hpp).
    // The enum value (r.lens_type) is preserved unchanged; only the display order differs.
    if (ImGui::BeginCombo("Lens Type##view", kLensTypeNames[r.lens_type])) {
      for (int idx : kLensTypePresentationOrder) {
        bool selected = (r.lens_type == idx);
        if (ImGui::Selectable(kLensTypeNames[idx], selected)) {
          // The lens switch and its pose fix-ups live in gui_state.hpp, shared with the defaults
          // panel's per-row lens editor. .lmc loading and tests bypass both controls by writing
          // lens_type directly, so they keep their fov.
          ApplyLensTypeSelection(r, idx);
        }
        if (selected) {
          ImGui::SetItemDefaultFocus();
        }
      }
      ImGui::EndCombo();
    }
    // Domain, format and disabled-when all come from the field editor registry rather than being
    // written here — same for every slider below whose field is a registered document leaf. The
    // bound is the lens' own MaxFov (the registry calls LUMICE_MaxFov), and `enabled` is the
    // full-sky gate that used to be spelled `BeginDisabled(full_sky)` at this line.
    const FieldEditorConstraint fov_c = ConstraintFor("renderer.fov", g_state);
    ImGui::BeginDisabled(!fov_c.enabled);
    SliderWithInput("FOV##view", &r.fov, static_cast<float>(fov_c.min_value), static_cast<float>(fov_c.max_value),
                    fov_c.fmt, fov_c.scale);
    ImGui::EndDisabled();
    bool is_globe = (r.lens_type == kLensTypeGlobe);
    ImGui::SeparatorText("Visibility");
    // Same registry query as the FOV slider above, for the same reason. What used to stand here was
    // a hand-paired nest — `BeginDisabled()` under `full_sky` on the outside, `BeginDisabled(
    // is_globe)` under `!full_sky` on the inside — whose NET effect each widget saw had to be read
    // off the interleaving of four `if`s. The two gates are already registered (renderer.visible →
    // NotUnderFullSky, renderer.front → NotUnderFullSkyOrGlobe), and each already folds the
    // full-sky case in, so the call site needs no nesting: one flat pair per field.
    const FieldEditorConstraint visible_c = ConstraintFor("renderer.visible", g_state);
    ImGui::BeginDisabled(!visible_c.enabled);
    ImGui::RadioButton("Upper##visible", &r.visible, kVisibleUpper);
    ImGui::SameLine();
    ImGui::RadioButton("Full##visible", &r.visible, kVisibleFull);
    ImGui::SameLine();
    ImGui::RadioButton("Lower##visible", &r.visible, kVisibleLower);
    ImGui::EndDisabled();
    // Between the two pairs rather than inside either: SameLine only moves the draw cursor for the
    // next widget, so it is unaffected by — and does not affect — the disabled stack.
    ImGui::SameLine(0, 20);
    const FieldEditorConstraint front_c = ConstraintFor("renderer.front", g_state);
    ImGui::BeginDisabled(!front_c.enabled);
    Checkbox("Front##visible", &r.front);
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
      ImGui::SetTooltip("Show front hemisphere only\n(combine with Upper/Full/Lower)");
    }
    ImGui::EndDisabled();
    ImGui::SeparatorText("Pose");
    if (is_globe) {
      ImGui::TextDisabled("(?)");
      if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip(
            "In Globe lens, Az/El/Roll control the observer's orbit\n"
            "around the sphere, not the camera's own attitude.\n"
            "Roll is locked to 0 in this mode (slider is greyed out).");
      }
    }
    // The elevation limit backs off one degree from the pole under Globe; that, like the full-sky
    // gate the two share, is the registry's to state. Both entries carry the SAME gate
    // (NotUnderFullSky), so which of the two `enabled` values wraps the pair cannot matter.
    const FieldEditorConstraint el_c = ConstraintFor("renderer.elevation", g_state);
    const FieldEditorConstraint az_c = ConstraintFor("renderer.azimuth", g_state);
    ImGui::BeginDisabled(!el_c.enabled);
    SliderWithInput("Elevation##view", &r.elevation, static_cast<float>(el_c.min_value),
                    static_cast<float>(el_c.max_value), el_c.fmt, el_c.scale);
    SliderWithInput("Azimuth##view", &r.azimuth, static_cast<float>(az_c.min_value), static_cast<float>(az_c.max_value),
                    az_c.fmt, az_c.scale);
    ImGui::EndDisabled();
    // roll's gate is the wider one (full-sky OR globe) — again read, not restated.
    const FieldEditorConstraint roll_c = ConstraintFor("renderer.roll", g_state);
    ImGui::BeginDisabled(!roll_c.enabled);
    SliderWithInput("Roll##view", &r.roll, static_cast<float>(roll_c.min_value), static_cast<float>(roll_c.max_value),
                    roll_c.fmt, roll_c.scale);
    ImGui::EndDisabled();

    ImGui::Separator();
    float btn_w = ImGui::CalcTextSize("Reset").x + ImGui::GetStyle().FramePadding.x * 2.0f;
    float avail = ImGui::GetContentRegionAvail().x;
    if (avail > btn_w) {
      ImGui::SameLine(avail - btn_w);
    }
    if (ImGui::SmallButton("Reset##view")) {
      ViewDefaults d = DefaultViewParamsFor(r.lens_type);
      r.fov = d.fov;
      r.elevation = d.elevation;
      r.azimuth = d.azimuth;
      r.roll = d.roll;
    }

    ImGui::PopItemWidth();
  }

  // ---- Display Group ----
  if (ImGui::CollapsingHeader("Display", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::PushItemWidth(-(kLabelColWidth + ImGui::GetStyle().ItemSpacing.x));
    ImGui::SeparatorText("Rendering");
    // Rust-tinted input: changing Resolution re-runs the simulation and discards accumulated rays.
    // The warning is the point — see doc/gui-visual-language.md §7.
    ImGui::PushStyleColor(ImGuiCol_FrameBg, WarningFillColor(0.6f));
    ImGui::Combo("Resolution##display", &r.sim_resolution_index, kSimResolutionLabels, kSimResolutionCount);
    ImGui::PopStyleColor();
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("Re-runs simulation; accumulated rays reset");
    }
    ImGui::BeginGroup();
    const FieldEditorConstraint ev_c = ConstraintFor("renderer.exposure_offset", g_state);
    SliderWithInput("EV##display", &r.exposure_offset, static_cast<float>(ev_c.min_value),
                    static_cast<float>(ev_c.max_value), ev_c.fmt, ev_c.scale);
    ImGui::EndGroup();
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("Exposure value offset for display brightness");
    }

    ImGui::SeparatorText("Aspect Ratio");
    int preset_idx = static_cast<int>(g_state.aspect_preset);
    const char* preview_label = kAspectPresetNames[preset_idx];
    if (ImGui::BeginCombo("Preset##display_aspect", preview_label)) {
      for (int i = 0; i < kAspectPresetCount; i++) {
        bool disabled = AspectPresetOptionDisabled(static_cast<AspectPreset>(i), g_preview.HasBackground());
        ImGui::BeginDisabled(disabled);
        bool selected = (i == preset_idx);
        if (ImGui::Selectable(kAspectPresetNames[i], selected)) {
          g_state.aspect_preset = static_cast<AspectPreset>(i);
          ApplyAspectRatio(window, g_state.aspect_preset, g_state.aspect_portrait);
        }
        if (selected) {
          ImGui::SetItemDefaultFocus();
        }
        ImGui::EndDisabled();
      }
      ImGui::EndCombo();
    }
    ImGui::BeginDisabled(AspectFlipDisabled(g_state.aspect_preset));
    const char* flip_label = g_state.aspect_portrait ? "Portrait" : "Landscape";
    if (ImGui::Button(flip_label)) {
      g_state.aspect_portrait = !g_state.aspect_portrait;
      ApplyAspectRatio(window, g_state.aspect_preset, g_state.aspect_portrait);
    }
    ImGui::EndDisabled();

    // Screen-too-small warning: rendered only when the requested aspect could
    // not be honored AND the user is still on a non-Free preset (the
    // ApplyAspectRatio path already clears aspect_clamp on Free / kMatchBg-no-bg,
    // but we re-check here so a stale signal from a missed callback path
    // cannot leak through).
    if (g_state.aspect_clamp.was_clamped && g_state.aspect_preset != AspectPreset::kFree) {
      // Disabled Selectable for the static header (ImGui::Text* widgets are
      // emitted with id=0 so they cannot be located by the GUI test engine;
      // disabled Selectable still calls ItemAdd with a real ID derived from
      // the label, so it is addressable while remaining non-interactive).
      // Dynamic ratio detail follows as a plain Text below.
      ImGui::PushStyleColor(ImGuiCol_Text, WarningTextColor());
      ImGui::Selectable("Screen too small for this aspect", false, ImGuiSelectableFlags_Disabled);
      ImGui::Text("preview ~%.2f:1, export %.2f:1", g_state.aspect_clamp.achieved_preview_ratio,
                  g_state.aspect_clamp.requested_preview_ratio);
      ImGui::PopStyleColor();
    }

    ImGui::SeparatorText("Background");
    if (ImGui::Button("Load Bg##display")) {
      DoLoadBackground(window);
    }
    ImGui::SameLine();
    bool no_bg = !g_preview.HasBackground();
    ImGui::BeginDisabled(no_bg);
    if (ImGui::Button("Clear##display_bg")) {
      DoClearBackground();
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(no_bg);
    Checkbox("Show##display_bg", &g_state.bg_show);
    // bg_alpha's own gate (WhenBackgroundShown) already covers BOTH "no image loaded" and "image
    // hidden", so it subsumes the outer BeginDisabled(no_bg) this sits inside. The outer one stays
    // because it also wraps the Show checkbox, which is not this field's control; a doubly-pushed
    // disabled state is idempotent.
    const FieldEditorConstraint bg_alpha_c = ConstraintFor("bg_alpha", g_state);
    ImGui::BeginDisabled(!bg_alpha_c.enabled);
    SliderWithInput("Alpha##display", &g_state.bg_alpha, static_cast<float>(bg_alpha_c.min_value),
                    static_cast<float>(bg_alpha_c.max_value), bg_alpha_c.fmt, bg_alpha_c.scale);
    ImGui::EndDisabled();

    // Fine adjustment for the same three fields the canvas gesture writes — not a second owner of
    // the transform, the same `g_state` floats from the other end. A photograph is aligned by
    // dragging until it is nearly right and then nudging, and a drag cannot nudge.
    const FieldEditorConstraint bg_ox_c = ConstraintFor("bg_offset_x", g_state);
    ImGui::BeginDisabled(!bg_ox_c.enabled);
    SliderWithInput("Offset X##display_bg", &g_state.bg_offset_x, static_cast<float>(bg_ox_c.min_value),
                    static_cast<float>(bg_ox_c.max_value), bg_ox_c.fmt, bg_ox_c.scale);
    ImGui::EndDisabled();
    const FieldEditorConstraint bg_oy_c = ConstraintFor("bg_offset_y", g_state);
    ImGui::BeginDisabled(!bg_oy_c.enabled);
    SliderWithInput("Offset Y##display_bg", &g_state.bg_offset_y, static_cast<float>(bg_oy_c.min_value),
                    static_cast<float>(bg_oy_c.max_value), bg_oy_c.fmt, bg_oy_c.scale);
    ImGui::EndDisabled();
    const FieldEditorConstraint bg_scale_c = ConstraintFor("bg_scale", g_state);
    ImGui::BeginDisabled(!bg_scale_c.enabled);
    SliderWithInput("Zoom##display_bg", &g_state.bg_scale, static_cast<float>(bg_scale_c.min_value),
                    static_cast<float>(bg_scale_c.max_value), bg_scale_c.fmt, bg_scale_c.scale);
    ImGui::EndDisabled();

    // Always on screen rather than a first-run tooltip: this is a gesture reached a few times per
    // alignment session and then not for weeks, so the moment it needs to be discoverable is the
    // SECOND time, which is exactly when a one-shot hint is already gone. One dim line is cheap
    // enough that it need not compete with a tooltip for that job.
    // Short enough to survive the right panel's width: the sentence this replaced ("...pans the
    // image, ...wheel zooms it.") ran past the panel edge and lost its last two words, which is a
    // worse hint than no hint — a truncated instruction reads as a rendering fault.
    ImGui::TextDisabled("%s+drag pans, %s+wheel zooms", kBgModifierName, kBgModifierName);
    ImGui::EndDisabled();

    ImGui::PopItemWidth();
  }

  // ---- Overlay Group ----
  if (ImGui::CollapsingHeader("Overlay", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::PushItemWidth(-(kLabelColWidth + ImGui::GetStyle().ItemSpacing.x));
    ImGui::SeparatorText("Auxiliary Lines");
    // Per-overlay row layout: color picker + name (variable width) + Line / Label
    // checkboxes anchored at fixed X so the two checkbox columns align across rows
    // even though the name column has different widths (Horizon / Grid / Angular Distance).
    // Second row: Alpha slider.
    const ImGuiStyle& style = ImGui::GetStyle();
    // Anchor checkbox columns at fixed X derived from the longest overlay name
    // plus widget metrics. The trailing pad (ItemSpacing.x × 2) protects against
    // ColorEdit3 / CalcTextSize sub-pixel rounding under HiDPI so the long name
    // ("Angular Distance") never overlaps the Line checkbox.
    float color_w = ImGui::GetFrameHeight();                       // ColorEdit3 NoInputs is a frame_h square
    float name_col_w = ImGui::CalcTextSize("Angular Distance").x;  // widest overlay name
    float check_box_w = ImGui::GetFrameHeight();                   // checkbox tick area
    float line_text_w = ImGui::CalcTextSize("Line").x;
    float line_col_x = color_w + style.ItemSpacing.x + name_col_w + style.ItemSpacing.x * 2.0f;
    float label_col_x = line_col_x + check_box_w + style.ItemInnerSpacing.x + line_text_w + style.ItemSpacing.x * 2.0f;

    auto overlay_row = [&](const char* name, const char* color_id, float* color, const char* line_id, bool* line_v,
                           const char* label_id, bool* label_v) {
      ImGui::ColorEdit3(color_id, color, ImGuiColorEditFlags_NoInputs);
      ImGui::SameLine();
      ImGui::TextUnformatted(name);
      ImGui::SameLine(line_col_x);
      Checkbox(line_id, line_v);
      ImGui::SameLine(label_col_x);
      Checkbox(label_id, label_v);
    };

    overlay_row("Horizon", "##horizon_color", g_state.horizon_color, "Line##horizon", &g_state.show_horizon_line,
                "Label##horizon", &g_state.show_horizon_label);
    const FieldEditorConstraint horizon_a_c = ConstraintFor("overlay_horizon_alpha", g_state);
    SliderWithInput("Alpha##horizon", &g_state.horizon_alpha, static_cast<float>(horizon_a_c.min_value),
                    static_cast<float>(horizon_a_c.max_value), horizon_a_c.fmt, horizon_a_c.scale);

    overlay_row("Grid", "##grid_color", g_state.grid_color, "Line##grid", &g_state.show_grid_line, "Label##grid",
                &g_state.show_grid_label);
    const FieldEditorConstraint grid_a_c = ConstraintFor("overlay_grid_alpha", g_state);
    SliderWithInput("Alpha##grid", &g_state.grid_alpha, static_cast<float>(grid_a_c.min_value),
                    static_cast<float>(grid_a_c.max_value), grid_a_c.fmt, grid_a_c.scale);

    overlay_row("Angular Distance", "##sun_circles_color", g_state.sun_circles_color, "Line##sun_circles",
                &g_state.show_sun_circles_line, "Label##sun_circles", &g_state.show_sun_circles_label);
    const FieldEditorConstraint sun_a_c = ConstraintFor("overlay_sun_circles_alpha", g_state);
    SliderWithInput("Alpha##sun_circles", &g_state.sun_circles_alpha, static_cast<float>(sun_a_c.min_value),
                    static_cast<float>(sun_a_c.max_value), sun_a_c.fmt, sun_a_c.scale);

    if (g_state.show_sun_circles_line || g_state.show_sun_circles_label) {
      if (ImGui::Button("Edit Angles...##overlay")) {
        ImGui::OpenPopup("SunCirclesEdit");
      }
      if (ImGui::BeginPopup("SunCirclesEdit")) {
        bool at_limit = SunCirclesAtLimit(g_state.sun_circle_angles.size());

        // Preset buttons
        const float presets[] = { 9.0f, 22.0f, 28.0f, 46.0f };
        for (float p : presets) {
          const bool already = SunCircleAlreadyPresent(g_state.sun_circle_angles, p);
          char label[16];
          std::snprintf(label, sizeof(label), "%.0f\xc2\xb0", p);
          ImGui::BeginDisabled(already || at_limit);
          if (ImGui::Button(label)) {
            g_state.sun_circle_angles.push_back(p);
            std::sort(g_state.sun_circle_angles.begin(), g_state.sun_circle_angles.end());
          }
          ImGui::EndDisabled();
          ImGui::SameLine();
        }
        ImGui::NewLine();

        // Custom angle input
        static float custom_angle = 22.0f;
        ImGui::PushItemWidth(60.0f);
        ImGui::InputFloat("##custom_angle", &custom_angle, 0.0f, 0.0f, "%.1f");
        ImGui::PopItemWidth();
        ImGui::SameLine();
        ImGui::BeginDisabled(at_limit);
        if (ImGui::Button("+##add_circle")) {
          custom_angle = ClampSunCircleAngle(custom_angle);
          g_state.sun_circle_angles.push_back(custom_angle);
          std::sort(g_state.sun_circle_angles.begin(), g_state.sun_circle_angles.end());
        }
        ImGui::EndDisabled();

        // Current list with delete buttons
        ImGui::Separator();
        int remove_idx = -1;
        for (int i = 0; i < static_cast<int>(g_state.sun_circle_angles.size()); i++) {
          ImGui::Text("%.1f\xc2\xb0", g_state.sun_circle_angles[i]);
          ImGui::SameLine();
          char del_label[32];
          std::snprintf(del_label, sizeof(del_label), "x##del_%d", i);
          if (ImGui::SmallButton(del_label)) {
            remove_idx = i;
          }
        }
        if (remove_idx >= 0) {
          g_state.sun_circle_angles.erase(g_state.sun_circle_angles.begin() + remove_idx);
        }

        ImGui::EndPopup();
      }
    }

    // Zenith / Nadir pixel-space marker. Single line toggle (no label column —
    // markers don't carry text); radius slider mirrors the per-overlay alpha row.
    ImGui::ColorEdit3("##zenith_nadir_color", g_state.zenith_nadir_color, ImGuiColorEditFlags_NoInputs);
    ImGui::SameLine();
    ImGui::TextUnformatted("Zenith/Nadir");
    ImGui::SameLine(line_col_x);
    Checkbox("##zenith_nadir_line", &g_state.show_zenith_nadir_line);
    const FieldEditorConstraint zn_a_c = ConstraintFor("overlay_zenith_nadir_alpha", g_state);
    SliderWithInput("Alpha##zenith_nadir", &g_state.zenith_nadir_alpha, static_cast<float>(zn_a_c.min_value),
                    static_cast<float>(zn_a_c.max_value), zn_a_c.fmt, zn_a_c.scale);
    const FieldEditorConstraint zn_r_c = ConstraintFor("overlay_zenith_nadir_radius_px", g_state);
    SliderWithInput("Radius##zenith_nadir", &g_state.zenith_nadir_radius_px, static_cast<float>(zn_r_c.min_value),
                    static_cast<float>(zn_r_c.max_value), zn_r_c.fmt, zn_r_c.scale);

    ImGui::PopItemWidth();
  }

  ImGui::End();
}

void RenderPreviewPanel(GLFWwindow* window, float window_width, float window_height) {
  float left_w = g_state.left_panel_collapsed ? kCollapseBtnSize : kLeftPanelWidth;
  float right_w = g_state.right_panel_collapsed ? kCollapseBtnSize : kRightPanelWidth;
  float panel_x = left_w;
  float panel_width = window_width - left_w - right_w;
  float panel_height = window_height - kTopBarHeight - kStatusBarHeight;
  SetNextPanelGeometry(panel_x, kTopBarHeight, panel_width, panel_height);
  ImGui::Begin("##PreviewPanel", nullptr,
               ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                   ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoBackground |
                   ImGuiWindowFlags_NoBringToFrontOnFocus);

  // Renderer invariants (previously in RenderViewBar, runs every frame).
  // Copy-model: GuiState always owns a single renderer, no vector/index bookkeeping needed.
  {
    auto& r = g_state.renderer;
    float max_fov = LUMICE_MaxFov(static_cast<LUMICE_LensType>(r.lens_type));
    r.fov = std::min(r.fov, max_fov);
    if (LensIsFullSky(r.lens_type)) {  // Full-sky lenses: force view angles to zero
      r.elevation = 0.0f;
      r.azimuth = 0.0f;
      r.roll = 0.0f;
    }
  }

  float preview_height = panel_height;

  g_preview_vp.active = false;

  if (g_preview.HasTexture() || g_preview.HasBackground()) {
    // Compute viewport in framebuffer pixels (for HiDPI)
    int fb_w = 0;
    int fb_h = 0;
    glfwGetFramebufferSize(window, &fb_w, &fb_h);
    float scale_x = static_cast<float>(fb_w) / window_width;
    float scale_y = static_cast<float>(fb_h) / window_height;

    auto& rc = g_state.renderer;

    // Store viewport for deferred rendering
    g_preview_vp.active = true;
    g_preview_vp.vp_x = static_cast<int>(panel_x * scale_x);
    g_preview_vp.vp_y = static_cast<int>(kStatusBarHeight * scale_y);  // OpenGL Y is bottom-up
    g_preview_vp.vp_w = static_cast<int>(panel_width * scale_x);
    g_preview_vp.vp_h = static_cast<int>(preview_height * scale_y);
    auto& pp = g_preview_vp.params;
    pp.view_proj = BuildPreviewViewProjFromRenderer(rc);
    float ev_total = rc.exposure_offset + g_state.ev_auto;
    pp.exposure.intensity_factor = std::pow(2.0f, ev_total);

    // task-347 (Fix B) DECOUPLE: the composite path is now server-side self-
    // anchored on participating-P99 (see doc/ev-pipeline-architecture.md §6.6).
    // GUI must push ONLY the manual `exposure_offset` here — folding `ev_auto`
    // back in would multiply the auto-anchor by the auto-EV a second time and
    // reintroduce the double-count that Fix B is meant to eliminate. The mono
    // shader-uniform path above still uses `ev_total = exposure_offset +
    // ev_auto` because the mono pipeline anchors off the mono ExposureScale
    // (integral over the whole image), not off a self-anchored P99.
    //
    // Push guard logic (value-changed OR off->on edge) lives in
    // ShouldPushCompositeExposure (gui/composite_exposure_push.hpp,
    // code-review round 1 Major #1 + Minor #1) so the four branches are
    // independently unit-testable rather than only reachable through a full
    // ImGui frame. See that header for the off->on rationale (plan-review
    // Minor #2).
    {
      static float s_last_pushed_ev = std::numeric_limits<float>::quiet_NaN();
      static bool s_last_composite_active = false;
      constexpr float kCompositeEvPushEpsilon = 1e-4f;
      // Fixed +1.0 EV (2x) brightness boost baked into the composite-mode
      // exposure baseline (task-color-ev-boost / A1). owner-observed perceptual
      // gap: single-color composite reads dimmer than full-spectrum at the same
      // manual exposure offset. Stacked on top of rc.exposure_offset (baseline,
      // not override). Independent of Fix B's "no ev_auto" constraint — this is
      // a static perceptual compensation, not an auto-anchored term.
      constexpr float kColorModeEvBoost = 1.0f;
      const bool composite_active = g_server != nullptr && !g_state.raypath_color.empty();
      // Fix B: push value == manual EV offset only (no ev_auto).
      // + kColorModeEvBoost: static composite-mode brightness baseline (see above).
      const float composite_ev_push = rc.exposure_offset + kColorModeEvBoost;
      if (lumice::gui::ShouldPushCompositeExposure(composite_active, s_last_composite_active, composite_ev_push,
                                                   s_last_pushed_ev, kCompositeEvPushEpsilon)) {
        LUMICE_SetCompositeExposure(g_server, composite_ev_push);
        // Wake a paused poller so the next frame can pick up the freshly re-baked
        // composite even after a finite sim has completed (same rationale as the
        // color-window PushDisplayState path — see task-345.2 (③) in color_window.cpp).
        // No-op when the poller is already running. WakeForRefresh (not WakeForRestart)
        // preserves valid=true across the wake edge — same display-time-inert contract
        // as color/visible/solo/z_order edits (task-color-migration §3 D3).
        g_server_poller.WakeForRefresh(g_server);
        s_last_pushed_ev = composite_ev_push;
      }
      s_last_composite_active = composite_active;
    }
    float norm_intensity = g_state.snapshot_intensity;
    pp.exposure.intensity_scale = norm_intensity > 0 ? pp.exposure.intensity_factor / norm_intensity : 0.0f;
    // Overlap parameters for dual fisheye texture sampling.
    pp.source.max_abs_dz = kDualFisheyeOverlap;
    pp.source.r_scale = 1.0f / std::sqrt(1.0f + kDualFisheyeOverlap);
    pp.bg.enabled = g_state.bg_show && g_preview.HasBackground();
    pp.bg.alpha = g_state.bg_alpha;
    pp.bg.aspect = g_preview.GetBgAspect();
    pp.bg.pan_x = g_state.bg_offset_x;
    pp.bg.pan_y = g_state.bg_offset_y;
    pp.bg.zoom = g_state.bg_scale;

    // Auxiliary line overlay parameters (line flags only — labels are handled
    // separately via BuildOverlayLabelInput below).
    pp.overlay.show_horizon = g_state.show_horizon_line;
    pp.overlay.show_grid = g_state.show_grid_line;
    pp.overlay.show_sun_circles = g_state.show_sun_circles_line;
    std::copy(std::begin(g_state.horizon_color), std::end(g_state.horizon_color), std::begin(pp.overlay.horizon_color));
    std::copy(std::begin(g_state.grid_color), std::end(g_state.grid_color), std::begin(pp.overlay.grid_color));
    std::copy(std::begin(g_state.sun_circles_color), std::end(g_state.sun_circles_color),
              std::begin(pp.overlay.sun_circles_color));
    pp.overlay.horizon_alpha = g_state.horizon_alpha;
    pp.overlay.grid_alpha = g_state.grid_alpha;
    pp.overlay.sun_circles_alpha = g_state.sun_circles_alpha;
    pp.overlay.grid_step = ComputeGridStep(rc.fov);
    // Precompute sun direction in world space (azimuth fixed at 0, only altitude matters)
    constexpr float kDeg2Rad = 3.14159265358979323846f / 180.0f;
    float sa = g_state.sun.altitude * kDeg2Rad;
    pp.overlay.sun_dir[0] = -std::cos(sa);
    pp.overlay.sun_dir[1] = 0.0f;
    pp.overlay.sun_dir[2] = -std::sin(sa);
    pp.overlay.sun_circle_count = std::min(static_cast<int>(g_state.sun_circle_angles.size()), kMaxSunCircles);
    for (int i = 0; i < pp.overlay.sun_circle_count; i++) {
      pp.overlay.sun_circle_angles[i] = g_state.sun_circle_angles[i];
    }

    // Zenith / Nadir pixel-space marker. zenith world dir = (0,0,-1), nadir = (0,0,+1)
    // (see preview_renderer.cpp:overlayAuxLines altitude convention).
    pp.overlay.show_zenith_nadir = g_state.show_zenith_nadir_line;
    std::copy(std::begin(g_state.zenith_nadir_color), std::end(g_state.zenith_nadir_color),
              std::begin(pp.overlay.zenith_nadir_color));
    pp.overlay.zenith_nadir_alpha = g_state.zenith_nadir_alpha;
    pp.overlay.zenith_nadir_radius_px = g_state.zenith_nadir_radius_px;
    constexpr float kZenithWorldDir[3] = { 0.f, 0.f, -1.f };
    constexpr float kNadirWorldDir[3] = { 0.f, 0.f, 1.f };
    auto zpos = ProjectWorldDirToScreen(pp.view_proj, kZenithWorldDir, g_preview_vp.vp_w, g_preview_vp.vp_h);
    auto npos = ProjectWorldDirToScreen(pp.view_proj, kNadirWorldDir, g_preview_vp.vp_w, g_preview_vp.vp_h);
    pp.overlay.zenith_screen_pos[0] = zpos[0];
    pp.overlay.zenith_screen_pos[1] = zpos[1];
    pp.overlay.nadir_screen_pos[0] = npos[0];
    pp.overlay.nadir_screen_pos[1] = npos[1];

    // Overlay labels at viewport edges (drawn on the preview window's draw list so
    // modals correctly occlude them). BuildOverlayLabelInput is shared with
    // DoExportPreviewPng (off-screen FBO path) so both call sites produce
    // byte-identical OverlayLabelInput for a given state.
    if (g_state.show_horizon_label || g_state.show_grid_label || g_state.show_sun_circles_label) {
      OverlayLabelInput label_input = BuildOverlayLabelInput(g_state, rc);

      // Viewport rect in absolute OS screen coordinates. DrawOverlayLabels emits to
      // ImGui::GetWindowDrawList(), and with ImGuiConfigFlags_ViewportsEnable (gui-polish-v15)
      // draw list coordinates are absolute screen space, not relative to the host GLFW window.
      // Anchor (panel_x, kTopBarHeight) through MainVpPos() so labels stay glued to the
      // preview viewport when the host window is dragged or sits on a non-primary monitor.
      // Note: the export_fbo_renderer.cpp path passes (0, 0, w, h) intentionally — it owns a
      // self-allocated ImDrawList targeting an off-screen FBO and must NOT add this offset.
      ImVec2 vp_origin = MainVpPos(panel_x, kTopBarHeight);
      float vp_sx = vp_origin.x;
      float vp_sy = vp_origin.y;
      float vp_sw = panel_width;
      float vp_sh = preview_height;

      static std::vector<OverlayLabel> labels;
      ComputeOverlayLabels(label_input, vp_sx, vp_sy, vp_sw, vp_sh, labels);
      DrawOverlayLabels(labels, vp_sx, vp_sy, vp_sw, vp_sh);
    }

    // Mouse interaction: orbit with drag, FOV with scroll — or, with the pan/zoom modifier held,
    // the background image instead of the camera.
    //
    // The camera half is disabled for lenses in kFullSkyLensTypes (dual fisheye 4-6, rectangular
    // 7, dual orthographic 9): their shader path skips the view matrix so view angles + FOV have
    // no visual effect. The background half is NOT gated on that, because the background is pinned
    // to the window rather than seen through the lens — it is composited in NDC by
    // ComputeBgUvTransform, which never consults the view matrix, so panning it means the same
    // thing under a full-sky lens as under any other.
    bool full_sky = LensIsFullSky(rc.lens_type);
    ImVec2 avail = ImGui::GetContentRegionAvail();
    ImGui::InvisibleButton("##preview_interact", avail);

    {
      bool is_hovered = ImGui::IsItemHovered();
      bool is_active = ImGui::IsItemActive();
      ImGuiIO& io = ImGui::GetIO();

      // Four cases, closed: modifier held + background operable -> move the background; modifier
      // held + background NOT operable -> nothing at all; modifier not held -> the camera
      // behaviour below, unchanged. The `!bg_modifier` on the camera branches is what makes the
      // second case a no-op instead of silently orbiting: a user pressing the modifier over a
      // hidden background has stated what they meant to move, and swallowing the modifier to move
      // something else is worse than doing nothing.
      const bool bg_active = g_preview.HasBackground() && g_state.bg_show;
      // Alt/Option on every platform. Cmd on macOS is not an option: ImGui aliases Super+Left
      // into a right click before this handler runs — see kBgModifierName in preview_renderer.hpp.
      const bool bg_modifier = io.KeyAlt;

      if (bg_active && bg_modifier && is_active && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
        // Solve for "the texel under the cursor stays under the cursor". With
        // bg_uv = ndc * scale + offset, moving the cursor by dndc requires
        // offset_new = offset_old - dndc * scale; since `scale` already carries the zoom, this is
        // correct at every zoom level with no separate drag-gain curve.
        const BgUvTransform t = ComputeBgUvTransform(g_preview_vp.vp_w, g_preview_vp.vp_h, g_preview.GetBgAspect(),
                                                     g_state.bg_offset_x, g_state.bg_offset_y, g_state.bg_scale);
        // io.MouseDelta is in ImGui points while vp_w/vp_h are framebuffer pixels; scale_x/scale_y
        // (the DPI factors captured above, not the transform's) reconcile the two, which is what
        // keeps the image glued to the cursor on a HiDPI display rather than moving at half speed.
        const float dndc_x = io.MouseDelta.x * scale_x * 2.0f / static_cast<float>(g_preview_vp.vp_w);
        // Screen Y grows downward, NDC Y upward.
        const float dndc_y = -io.MouseDelta.y * scale_y * 2.0f / static_cast<float>(g_preview_vp.vp_h);

        // Clamp on this path too, not only in the sliders: a slider clamps what IT produces, it
        // does not retroactively pull an out-of-range value back, so an unclamped drag could park
        // the offset far past the slider's travel with no way back except touching the slider.
        const FieldEditorConstraint ox_c = ConstraintFor("bg_offset_x", g_state);
        const FieldEditorConstraint oy_c = ConstraintFor("bg_offset_y", g_state);
        g_state.bg_offset_x =
            std::max(static_cast<float>(ox_c.min_value),
                     std::min(static_cast<float>(ox_c.max_value), g_state.bg_offset_x - dndc_x * t.scale_x));
        g_state.bg_offset_y =
            std::max(static_cast<float>(oy_c.min_value),
                     std::min(static_cast<float>(oy_c.max_value), g_state.bg_offset_y - dndc_y * t.scale_y));
      }

      if (bg_active && bg_modifier && is_hovered && io.MouseWheel != 0.0f) {
        // Multiplicative, so one notch is the same proportional change everywhere on the range —
        // matching the kLog slider the same field is edited by.
        const FieldEditorConstraint scale_c = ConstraintFor("bg_scale", g_state);
        const float zoomed = g_state.bg_scale * std::pow(1.1f, io.MouseWheel);
        g_state.bg_scale =
            std::max(static_cast<float>(scale_c.min_value), std::min(static_cast<float>(scale_c.max_value), zoomed));
      }

      if (!bg_modifier && !full_sky && is_active && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
        ImVec2 delta = io.MouseDelta;
        // Sensitivity is the lens's angular resolution at the frame center, so one pixel of
        // drag moves the content one pixel whatever the FOV and viewport are. A fixed deg/px
        // was ~180x too fast at fov=1° and ~70x too slow at fov=179°. vp_w/vp_h are
        // framebuffer pixels (set above from the DPI scale), the same units
        // ProjectWorldDirToScreen works in, which is what makes this DPI-correct for free.
        float gain = ComputeDragGainDegPerPixel(rc.lens_type, rc.fov, g_preview_vp.vp_w, g_preview_vp.vp_h);
        // Globe is outside-in: u_view_matrix * (0,0,D) yields camera world
        // position, so +az/+el move the camera and the sphere drifts in the
        // opposite direction. Flip both signs so a right/down drag moves the
        // sphere right/down, matching the inside-out lenses' feel.
        if (rc.lens_type == kLensTypeGlobe) {
          rc.azimuth += delta.x * gain;
          rc.elevation -= delta.y * gain;
        } else {
          rc.azimuth -= delta.x * gain;
          rc.elevation += delta.y * gain;
        }
        // Wrap azimuth into [-180, 180] so dragging past the slider's clamp
        // boundary continues seamlessly instead of getting pinned at ±180.
        if (rc.azimuth > 180.0f) {
          rc.azimuth -= 360.0f;
        } else if (rc.azimuth < -180.0f) {
          rc.azimuth += 360.0f;
        }
        // Globe lens needs a tighter clamp to avoid view-matrix degeneracy at
        // ±90°; inside-out lenses keep the existing ±90° range.
        float el_lim = (rc.lens_type == kLensTypeGlobe) ? 89.0f : 90.0f;
        rc.elevation = std::max(-el_lim, std::min(el_lim, rc.elevation));
      }

      if (!bg_modifier && !full_sky && is_hovered && io.MouseWheel != 0.0f) {
        float fov_max = LUMICE_MaxFov(static_cast<LUMICE_LensType>(rc.lens_type));
        rc.fov -= io.MouseWheel * 5.0f;
        rc.fov = std::max(1.0f, std::min(fov_max, rc.fov));
      }
    }
  } else {
    ImVec2 avail = ImGui::GetContentRegionAvail();
    ImVec2 text_size = ImGui::CalcTextSize("Render Preview");
    ImGui::SetCursorPos(ImVec2((avail.x - text_size.x) * 0.5f, (avail.y - text_size.y) * 0.5f));
    ImGui::TextDisabled("Render Preview");
  }

  ImGui::End();
}

void RenderStatusBar(float window_width, float window_height) {
  SetNextPanelGeometry(0, window_height - kStatusBarHeight, window_width, kStatusBarHeight);
  ImGui::Begin("##StatusBar", nullptr,
               ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                   ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoBringToFrontOnFocus);

  // Status indicator
  switch (g_state.sim_state) {
    case SimState::kIdle:
      ImGui::TextColored(GoodTextColor(), "Ready");
      break;
    case SimState::kSimulating:
      ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.0f, 1.0f), "Simulating...");
      break;
    case SimState::kStopping:
      ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.0f, 1.0f), "Stopping...");
      break;
    case SimState::kDone:
      ImGui::TextColored(ImVec4(0.3f, 0.7f, 1.0f, 1.0f), "Done");
      break;
    case SimState::kModified:
      ImGui::TextColored(WarningTextColor(), "Modified");
      break;
  }

  // Stats
  if (g_state.stats_sim_ray_num > 0) {
    ImGui::SameLine();
    LUMICE_RayCount n = g_state.stats_sim_ray_num;
    char buf[64];
    // "Total" is the summed count over all discrete wavelengths. Since task-323
    // `ray_num` is itself the requested total; the actual traced count reported
    // here is ceil(ray_num / N_wavelengths) × N_wavelengths (may overshoot by <N).
    if (n >= 1'000'000'000ULL) {
      snprintf(buf, sizeof(buf), "| Total rays: %.1f x10^9", n / 1e9);
    } else if (n >= 1'000'000ULL) {
      snprintf(buf, sizeof(buf), "| Total rays: %.1f x10^6", n / 1e6);
    } else {
      snprintf(buf, sizeof(buf), "| Total rays: %.1f x10^3", n / 1e3);
    }
    ImGui::Text("%s", buf);

    // Sampling density. Deliberately inside the SAME `stats_sim_ray_num > 0` gate as "Total rays"
    // above and under no additional condition of its own: whether a dimension is randomized is
    // exactly what this readout is for, so hiding it when a dimension is fixed would suppress the
    // answer in the case the user most needs it ("1 per 5.4 x10^6 rays" IS the explanation for an
    // over-sharp render). The shared gate only asks "has a run happened", which is orthogonal.
    //
    // Plain text on purpose: no progress bar, no color grading, no check/cross. Neither counter has
    // a "good" value -- a low shape count is correct for a fixed shape and expected on the GPU
    // route -- so any better/worse styling here would manufacture false alarms.
    // NOTE: this segment builds its text in a pure function (app.cpp) while "Total rays" above
    // formats inline. The inconsistency is deliberate, not an oversight — do NOT "unify" it by
    // inlining this one. `ImGui::Text`/`TextUnformatted` submit an item ID of 0, so a test cannot
    // address the rendered string through the item API; extracting the text lets the string itself
    // be asserted, with a separate pixel test proving it reaches the framebuffer. A future status
    // bar segment that wants test coverage should follow THIS pattern rather than the inline one.
    ImGui::SameLine();
    const std::string sampling =
        FormatSamplingSegment(g_state.stats_crystal_num, g_state.stats_orientation_num, g_state.stats_sim_ray_num);
    ImGui::TextUnformatted(sampling.c_str());
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("%s", FormatSamplingTooltip(g_state.stats_crystal_num, g_state.stats_orientation_num,
                                                    g_state.stats_sim_ray_num)
                                  .c_str());
    }
  }

  // Sim resolution + lens info (renderer is always embedded in GuiState).
  {
    auto& rc = g_state.renderer;
    int res = kSimResolutions[rc.sim_resolution_index];
    ImGui::SameLine();
    ImGui::Text("| %dx%d  %s  FOV:%.0f", res, res / 2, kLensTypeNames[rc.lens_type], rc.fov);
  }

  ImGui::SameLine();
  ImGui::Text("|");
  ImGui::SameLine();

  if (g_state.current_file_path.empty()) {
    ImGui::Text("No file");
  } else {
    auto filename = g_state.current_file_path.filename().u8string();
    if (g_state.dirty) {
      ImGui::Text("%s *", filename.c_str());
    } else {
      ImGui::Text("%s", filename.c_str());
    }
  }

  // task-colored-toggle-to-topbar (346.3): the colored/full-spectrum mode toggle
  // that used to sit here (task-345.4) moved to the top bar next to the Colors
  // button. The status bar right cluster now contains only the Log button; the
  // width formula below dropped `mode_w` and its trailing `mode_gap` term.
  {
    const char* log_label = g_state.log_panel_open ? ICON_FA_CHEVRON_DOWN " Log" : ICON_FA_CHEVRON_RIGHT " Log";
    const float pad_x = ImGui::GetStyle().FramePadding.x * 2;
    const float log_w = ImGui::CalcTextSize(log_label).x + pad_x;
    ImGui::SameLine(ImGui::GetWindowWidth() - log_w - ImGui::GetStyle().WindowPadding.x);
    if (ImGui::SmallButton(log_label)) {
      g_state.log_panel_open = !g_state.log_panel_open;
    }
  }

  ImGui::End();
}

// Pending message text for the Import Warning modal. Filled by
// SetImportComplexFilterWarning from the JSON import path; consumed (and
// cleared) by RenderImportWarningPopup when the user dismisses the modal.
namespace {
std::string g_pending_import_warning;
}  // namespace

void SetImportComplexFilterWarning(const std::string& msg) {
  if (!g_pending_import_warning.empty()) {
    g_pending_import_warning += "\n";
  }
  g_pending_import_warning += msg;
}

std::string PeekImportComplexFilterWarning() {
  return g_pending_import_warning;
}

void ClearImportComplexFilterWarning() {
  g_pending_import_warning.clear();
}

void RenderImportWarningPopup() {
  static std::string active_msg;
  if (!g_pending_import_warning.empty()) {
    active_msg = std::move(g_pending_import_warning);
    // A moved-from std::string is valid-but-unspecified, not guaranteed empty;
    // clear() makes the trigger false next frame so the popup opens once.
    g_pending_import_warning.clear();
    ImGui::OpenPopup("Import Warning");
  }

  if (ImGui::BeginPopupModal("Import Warning", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::TextUnformatted("Some settings in the imported config could not be fully represented in the GUI:");
    ImGui::Separator();
    ImGui::TextUnformatted(active_msg.c_str());
    ImGui::Separator();
    ImGui::TextUnformatted("They were simplified on load. Edit the config file / CLI directly to keep the originals.");
    if (ImGui::Button("OK", ImVec2(80, 0))) {
      active_msg.clear();
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }
}

// See app.hpp for why this sentence is a named constant and what is asserted about it.
const char* const kExportOverwriteWarningText =
    "Exporting replaces it with what the GUI can express, so anything in it the GUI\n"
    "cannot represent will be lost. Save as .lmc instead to keep this project\n"
    "without touching that file.";

// Export config JSON, onto a path that already holds a file. Same modal idiom as
// RenderSaveModifiedPopup above (BeginPopupModal + explicit buttons, so Escape and click-outside
// cannot dismiss it — see the note there for why that holds for every modal in this app), because
// the same thing is being protected: a write the user cannot undo.
void RenderExportOverwriteConfirmPopup() {
  if (g_show_export_overwrite_confirm_popup) {
    ImGui::OpenPopup("Overwrite Config File");
    g_show_export_overwrite_confirm_popup = false;
  }

  if (ImGui::BeginPopupModal("Overwrite Config File", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::TextUnformatted("A file already exists at:");
    ImGui::TextUnformatted(PathToU8(g_pending_export_json_path).c_str());
    ImGui::Separator();
    ImGui::TextUnformatted(kExportOverwriteWarningText);
    ImGui::Separator();

    if (ImGui::Button("Overwrite", ImVec2(120, 0))) {
      ConfirmPendingConfigJsonExport();
      ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel", ImVec2(80, 0))) {
      CancelPendingConfigJsonExport();
      ImGui::CloseCurrentPopup();
    }

    ImGui::EndPopup();
  }
}

// Generic GUI warning modal (not import-specific). Fires ONCE per distinct warning episode:
// SetGuiWarning is idempotent while the same message is in-flight, so a persistent condition
// re-detected on every debounced commit (e.g. an over-bounds filter re-checked on each 70ms
// DoRun while the user drags an unrelated slider) does NOT re-open the modal and freeze
// interaction. ClearGuiWarning (called on a successful commit) re-arms it so a later
// re-occurrence of the same condition warns again.
//
// Identity contract (the message text IS the episode key): callers MUST guarantee that one
// logical warning event always produces byte-identical text, and that distinct events use
// distinct text. Today the only caller (DoRun over-bounds) uses a constant string, so this
// holds. Before a second caller reuses this modal, promote the key from message content to an
// explicit warning-id/source enum (text becomes display-only) to avoid text collisions
// (two events, same wording -> new warning swallowed) or wording drift (one event, changed
// wording -> spurious re-open).
std::string g_gui_warning_current;   // "" = none; else the message currently in-flight
bool g_gui_warning_trigger = false;  // request OpenPopup on the next render

void SetGuiWarning(const std::string& msg) {
  if (msg == g_gui_warning_current) {
    return;  // this exact warning is already showing / dismissed — do not re-open (anti-spam)
  }
  g_gui_warning_current = msg;
  g_gui_warning_trigger = true;
}

void ClearGuiWarning() {
  g_gui_warning_current.clear();
  g_gui_warning_trigger = false;
}

// Test accessor: the message currently in-flight ("" if none).
std::string PeekGuiWarning() {
  return g_gui_warning_current;
}

// Test accessor: is a modal re-open pending on the next render? See app.hpp.
bool IsGuiWarningPending() {
  return g_gui_warning_trigger;
}

namespace internal_test {
void ConsumeGuiWarningPending() {
  g_gui_warning_trigger = false;
}
}  // namespace internal_test

void RenderGuiWarningPopup() {
  if (g_gui_warning_trigger) {
    g_gui_warning_trigger = false;
    ImGui::OpenPopup("Warning");
  }
  if (ImGui::BeginPopupModal("Warning", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::TextUnformatted(g_gui_warning_current.c_str());
    ImGui::Separator();
    if (ImGui::Button("OK", ImVec2(80, 0))) {
      // Keep g_gui_warning_current set so the same persistent condition, re-detected on the
      // next debounced commit, is deduped (does not re-open). ClearGuiWarning (on a successful
      // commit) re-arms it.
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }
}

// Executes the New/Open/Quit action queued in g_pending_action (if any) and
// clears it. Shared by the three button branches below and in
// RenderSaveModifiedPopup that resume a deferred action once it is safe to do
// so (code-review-02 M2: was three independent copies of the same 4-branch
// switch; consolidated to a single call site).
void ResolvePendingAction(GLFWwindow* window) {
  switch (g_pending_action) {
    case PendingAction::kNew:
      DoNew();
      break;
    case PendingAction::kOpen:
      DoOpen();
      break;
    case PendingAction::kQuit:
      glfwSetWindowShouldClose(window, GLFW_TRUE);
      break;
    default:
      break;
  }
  g_pending_action = PendingAction::kNone;
}

void RenderUnsavedPopup(GLFWwindow* window) {
  if (g_show_unsaved_popup) {
    ImGui::OpenPopup("Unsaved Changes");
    g_show_unsaved_popup = false;
  }

  if (ImGui::BeginPopupModal("Unsaved Changes", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::Text("You have unsaved changes. Save before continuing?");
    ImGui::Separator();

    if (ImGui::Button("Save", ImVec2(80, 0))) {
      // task-cleanup-hardening AC4 code-review-01 M1: route through the
      // kModified gate (DoSave) instead of bypassing it via PerformSave. If
      // sim_state == kModified, DoSave() defers to RenderSaveModifiedPopup and
      // leaves g_pending_action queued — its three branches below decide
      // whether the deferred New/Open/Quit actually runs. If not modified,
      // DoSave() falls through to PerformSave() synchronously, same as before.
      DoSave();
      if (!g_show_save_modified_popup) {
        ResolvePendingAction(window);
      }
      ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Don't Save", ImVec2(100, 0))) {
      ResolvePendingAction(window);
      ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel", ImVec2(80, 0))) {
      g_pending_action = PendingAction::kNone;
      ImGui::CloseCurrentPopup();
    }

    ImGui::EndPopup();
  }
}

// task-cleanup-hardening AC4: RenderSaveModifiedPopup — the "Save while
// sim_state == kModified" prompt. The owner ruling (issue.md §偏离-E) is that
// silently serializing "stale preview + fresh config + dirty=false" and
// clearing Modified is a bug: the .lmc records a preview that does NOT match
// its own config, and the user loses the visual warning that a re-run is
// needed. This popup gates that flow — the user picks one of three:
//   - "Run first"  : DoRun() to produce a fresh preview matching the current
//                    config, then close the popup; the user re-invokes Save
//                    when ready. Disabled if no live server (kIdle).
//   - "Save anyway": PerformSave / PerformSaveAs (bypass the check). Freezes
//                    the last committed run's preview into the .lmc — legit
//                    when the user knowingly wants to snapshot pre-edit state.
//   - "Cancel"     : Clears the pending save kind and closes; no side effect.
//
// code-review-01 M1: this popup can also be reached via RenderUnsavedPopup's
// "Save" button (DoSave() defers here when kModified), which leaves a
// g_pending_action (New/Open/Quit) queued. Only "Save anyway" — the branch
// that actually performs a serialization — resumes that deferred action;
// "Run first" and "Cancel" both clear it, since neither persists anything and
// silently proceeding with New/Open/Quit would discard the edit the original
// Unsaved-changes prompt was protecting. When this popup is opened directly
// from the top-bar Save button, g_pending_action is always kNone already, so
// clearing/switching on it here is a no-op in that path.
void RenderSaveModifiedPopup(GLFWwindow* window) {
  // window is used by the "Save anyway" branch's chained kQuit case (see
  // code-review-01 M1 doc comment above).
  //
  // code-review-02 M1 investigated whether Escape can dismiss this modal
  // without going through any of the three buttons below, which would leave
  // g_pending_save_kind / g_pending_action stale for a later, unrelated Save
  // to misfire on. Verified NOT reachable in this codebase: Dear ImGui's
  // NavUpdateCancelRequest() (imgui.cpp) only routes Escape to
  // ClosePopupToLevel() when the topmost open popup does NOT have
  // ImGuiWindowFlags_Modal set — BeginPopupModal always sets that flag, so
  // Escape never reaches the close path for ANY modal in this app,
  // regardless of p_open or window flags (io.ConfigFlags has
  // ImGuiConfigFlags_NavEnableKeyboard on in both main.cpp:126 and
  // test_gui_main.cpp:264, so the nav-active precondition is satisfied — the
  // modal exclusion is what actually blocks it). Modals also block
  // click-outside-to-close by design. The only exits are the three buttons.
  // gui_test file_ops/escape_does_not_dismiss_the_modified_prompt
  // (test/gui/functional/test_file_ops.cpp) pins this via a real
  // ctx->KeyPress(ImGuiKey_Escape) against the live popup.
  if (g_show_save_modified_popup) {
    ImGui::OpenPopup("Save Modified Config");
    g_show_save_modified_popup = false;
  }

  if (ImGui::BeginPopupModal("Save Modified Config", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::Text("The config has been modified since the last simulation run.");
    ImGui::Text("The on-screen preview reflects the previous config, not the current one.");
    ImGui::Separator();

    // "Run first" is only meaningful when there is a live server to run on and no run is already
    // inflight. The second half is the SAME "busy" notion the top bar gates New/Open/Save on —
    // shared through sim_state_rules.hpp rather than restated here, which is what the note that
    // used to sit at this line ("single-source would be nicer but the top bar's enable predicate
    // is inlined and not exported") asked for. The two gates are still distinct predicates: this
    // one additionally requires a live server.
    const bool can_run = CanRunFromModal(g_server != nullptr, g_state.sim_state);
    ImGui::BeginDisabled(!can_run);
    if (ImGui::Button("Run first", ImVec2(100, 0))) {
      DoRun(/*user_initiated=*/true);
      g_pending_save_kind = PendingSaveKind::kNone;
      // Abort any chained New/Open/Quit (see doc comment above) — Run doesn't
      // persist anything, so proceeding now would still discard the edit.
      g_pending_action = PendingAction::kNone;
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("Save anyway", ImVec2(120, 0))) {
      switch (g_pending_save_kind) {
        case PendingSaveKind::kSave:
          PerformSave();
          break;
        case PendingSaveKind::kSaveAs:
          PerformSaveAs();
          break;
        case PendingSaveKind::kNone:
          break;
      }
      g_pending_save_kind = PendingSaveKind::kNone;
      // Resume the New/Open/Quit deferred by RenderUnsavedPopup's Save button
      // (see doc comment above) — a real save just happened, so it's safe to
      // proceed. No-op when this popup was opened directly (g_pending_action
      // is kNone in that path).
      ResolvePendingAction(window);
      ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel", ImVec2(80, 0))) {
      g_pending_save_kind = PendingSaveKind::kNone;
      // Abort any chained New/Open/Quit — see doc comment above.
      g_pending_action = PendingAction::kNone;
      ImGui::CloseCurrentPopup();
    }

    ImGui::EndPopup();
  }
}

void RenderLogPanel(float window_width, float window_height) {
  if (!g_imgui_log_sink) {
    return;
  }

  constexpr float kLogPanelHeight = 250.0f;

  if (!g_state.log_panel_open) {
    return;
  }

  SetNextPanelGeometry(0, window_height - kLogPanelHeight - kStatusBarHeight, window_width, kLogPanelHeight);
  // ##LogPanel intentionally does NOT carry NoBringToFrontOnFocus: it
  // belongs to Layer 3 (floating, raisable) per the z-order convention block
  // at the top of this file. ImGui creates NoBringToFrontOnFocus windows via
  // push_front (= bottom of g.Windows) and others via push_back (= top), so
  // adding the flag here would push LogPanel into the background cluster
  // BELOW LeftPanel/RightPanel — the opposite of the desired stacking.
  ImGui::Begin("##LogPanel", &g_state.log_panel_open,
               ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

  // Config controls row
  static const char* const kLevelNames[] = { "Trace", "Debug", "Verbose", "Info", "Warning", "Error", "Off" };
  static const LUMICE_LogLevel kLevelMap[] = { LUMICE_LOG_TRACE, LUMICE_LOG_DEBUG,   LUMICE_LOG_VERBOSE,
                                               LUMICE_LOG_INFO,  LUMICE_LOG_WARNING, LUMICE_LOG_ERROR,
                                               LUMICE_LOG_OFF };

  ImGui::Text("GUI");
  ImGui::SameLine();
  ImGui::PushItemWidth(80);
  if (ImGui::Combo("##GuiLevel", &g_state.gui_log_level, kLevelNames, 7)) {
    SetGuiLogLevel(static_cast<spdlog::level::level_enum>(g_state.gui_log_level));
  }
  ImGui::PopItemWidth();

  ImGui::SameLine();
  ImGui::Text("Core");
  ImGui::SameLine();
  ImGui::PushItemWidth(80);
  if (ImGui::Combo("##CoreLevel", &g_state.core_log_level, kLevelNames, 7)) {
    if (g_server) {
      LUMICE_SetLogLevel(g_server, kLevelMap[g_state.core_log_level]);
    }
  }
  ImGui::PopItemWidth();

  ImGui::SameLine();
  if (Checkbox("File", &g_state.log_to_file)) {
    if (g_file_log_sink) {
      g_file_log_sink->set_level(g_state.log_to_file ? spdlog::level::trace : spdlog::level::off);
    }
  }
  if (g_state.log_to_file) {
    ImGui::SameLine();
    ImGui::TextDisabled("%s", g_log_file_path.c_str());
  }

  ImGui::SameLine(ImGui::GetWindowWidth() - 60);
  if (ImGui::Button("Clear")) {
    g_imgui_log_sink->Clear();
  }

  ImGui::Separator();

  // Log content area with auto-scroll
  ImGui::BeginChild("LogContent", ImVec2(0, 0), ImGuiChildFlags_None, ImGuiWindowFlags_HorizontalScrollbar);

  auto entry_count = g_imgui_log_sink->Size();
  ImGuiListClipper clipper;
  clipper.Begin(static_cast<int>(entry_count));

  // We need random access — collect visible entries via ForEachEntry with index filtering.
  // For simplicity and correctness with clipper, read all entries once per frame into a local cache.
  // This is acceptable because the deque is bounded to 4096 entries.
  struct CachedEntry {
    spdlog::level::level_enum level;
    const char* text;
  };
  static std::vector<LogEntry> frame_cache;
  frame_cache.clear();
  frame_cache.reserve(entry_count);
  g_imgui_log_sink->ForEachEntry([](size_t /*idx*/, const LogEntry& e) { frame_cache.push_back(e); });

  while (clipper.Step()) {
    for (int i = clipper.DisplayStart; i < clipper.DisplayEnd && i < static_cast<int>(frame_cache.size()); i++) {
      const auto& entry = frame_cache[i];
      ImVec4 color;
      switch (entry.level) {
        case spdlog::level::trace:
        case spdlog::level::debug:
          color = ImVec4(0.6f, 0.6f, 0.6f, 1.0f);
          break;
        case spdlog::level::warn:
          color = WarningTextColor();
          break;
        case spdlog::level::err:
        case spdlog::level::critical:
          color = DestructiveTextColor();
          break;
        default:
          color = ImVec4(1.0f, 1.0f, 1.0f, 1.0f);
          break;
      }
      ImGui::PushStyleColor(ImGuiCol_Text, color);
      ImGui::TextUnformatted(entry.message.c_str());
      ImGui::PopStyleColor();
    }
  }

  // Auto-scroll to bottom when new entries arrive
  if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 20.0f) {
    ImGui::SetScrollHereY(1.0f);
  }

  ImGui::EndChild();
  ImGui::End();
}

}  // namespace lumice::gui
