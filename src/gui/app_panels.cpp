#include <GLFW/glfw3.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>

#include "IconsFontAwesome6.h"
#include "gui/app.hpp"
#include "gui/composite_exposure_push.hpp"
#include "gui/crystal_preview.hpp"
#include "gui/edit_modals.hpp"
#include "gui/gui_constants.hpp"
#include "gui/gui_ev_auto.hpp"
#include "gui/gui_logger.hpp"
#include "gui/overlay_labels.hpp"
#include "gui/panels.hpp"
#include "imgui.h"

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
  bool simulating = (g_state.sim_state == SimState::kSimulating);
  bool stopping = (g_state.sim_state == SimState::kStopping);
  bool busy = simulating || stopping;
  const auto& style = ImGui::GetStyle();
  const char* kRunLabel = ICON_FA_PLAY " Run";
  const char* kStopLabel = ICON_FA_STOP " Stop";
  const char* kStoppingLabel = ICON_FA_STOP " Stopping...";
  float run_stop_width = std::max({ ImGui::CalcTextSize(kRunLabel).x, ImGui::CalcTextSize(kStopLabel).x,
                                    ImGui::CalcTextSize(kStoppingLabel).x }) +
                         style.FramePadding.x * 2;
  if (simulating) {
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.6f, 0.1f, 0.1f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.7f, 0.2f, 0.2f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.5f, 0.05f, 0.05f, 1.0f));
    if (ImGui::Button(kStopLabel, ImVec2(run_stop_width, 0))) {
      DoStop();
    }
    ImGui::PopStyleColor(3);
  } else if (stopping) {
    // Async Stop in flight: greyed, disabled "Stopping…" (DoStop is idempotent, but the disabled
    // button makes the in-flight state unambiguous and blocks re-entry at the UI layer).
    ImGui::BeginDisabled();
    ImGui::Button(kStoppingLabel, ImVec2(run_stop_width, 0));
    ImGui::EndDisabled();
  } else {
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.45f, 0.15f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.2f, 0.55f, 0.2f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.1f, 0.35f, 0.1f, 1.0f));
    if (ImGui::Button(kRunLabel, ImVec2(run_stop_width, 0))) {
      DoRun();
    }
    ImGui::PopStyleColor(3);
  }

  // Revert area — always rendered for stable layout, hidden when not modified.
  // Alpha=0 + BeginDisabled: invisible and non-interactive, but still occupies layout space.
  // The hidden area intercepts clicks, which is harmless in this horizontal toolbar context.
  bool modified = (g_state.sim_state == SimState::kModified);
  if (!modified) {
    ImGui::PushStyleVar(ImGuiStyleVar_Alpha, 0.0f);
    ImGui::BeginDisabled();
  }
  ImGui::SameLine();
  ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.0f, 1.0f), ICON_FA_CIRCLE_EXCLAMATION);
  ImGui::SameLine();
  if (ImGui::SmallButton("Revert") && modified) {  // `&& modified`: redundant safety guard over BeginDisabled
    DoRevert();
  }
  if (!modified) {
    ImGui::EndDisabled();
    ImGui::PopStyleVar();
  }

  ImGui::SameLine();
  ImGui::TextDisabled("|");
  ImGui::SameLine();

  // File operations — New/Open disabled while busy (simulating OR async Stop draining); Save menu
  // itself stays enabled so read-only exports (Screenshot / Dual Fisheye Equal Area /
  // Equirectangular / Config JSON) remain reachable.
  if (busy) {
    ImGui::BeginDisabled();
  }
  if (ImGui::Button("New")) {
    CheckUnsavedAndDo(PendingAction::kNew);
  }
  ImGui::SameLine();
  if (ImGui::Button("Open")) {
    CheckUnsavedAndDo(PendingAction::kOpen);
  }
  if (busy) {
    ImGui::EndDisabled();
  }
  ImGui::SameLine();
  {
    if (ImGui::Button("Save")) {
      ImGui::OpenPopup("SaveMenu");
    }
    if (ImGui::BeginPopup("SaveMenu")) {
      bool no_texture = !g_preview.HasTexture();
      bool has_server = g_server != nullptr && g_state.sim_state != GuiState::SimState::kIdle;
      if (busy) {
        ImGui::BeginDisabled();
      }
      if (ImGui::MenuItem("Save")) {
        DoSave();
      }
      if (ImGui::MenuItem("Save Copy")) {
        DoSaveAs();
      }
      if (busy) {
        ImGui::EndDisabled();
      }
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
  if (ImGui::Button(ICON_FA_PALETTE " Colors")) {
    g_state.color_window_open = !g_state.color_window_open;
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
    const bool composite_now = g_state.last_uploaded_as_composite;
    const char* mode_label = composite_now ? ICON_FA_PALETTE " Colored" : ICON_FA_PALETTE " Full Spectrum";
    std::string checkbox_id = std::string(mode_label) + "##CompositePreviewToggle";
    bool checked = composite_now;
    if (composite_now) {
      // Checkbox renders as frame background + check mark, not a button — the
      // 345.4 accent used ImGuiCol_Button which does not apply here.
      ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.35f, 0.55f, 0.85f, 1.0f));
      ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, ImVec4(0.45f, 0.65f, 0.95f, 1.0f));
      ImGui::PushStyleColor(ImGuiCol_CheckMark, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
    }
    if (ImGui::Checkbox(checkbox_id.c_str(), &checked)) {
      g_state.show_composite_preview = !g_state.show_composite_preview;
    }
    if (composite_now) {
      ImGui::PopStyleColor(3);
    }
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip(
          "Toggle colored composite / full-spectrum preview.\n"
          "Display-time only -- does not re-simulate or discard color classes.");
    }
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
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.85f, 0.2f, 1.0f));
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
    g_state.MarkDirty();
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
  bool full_sky = LensIsFullSky(r.lens_type);

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
          if (r.lens_type != idx) {
            bool was_globe = (r.lens_type == kLensTypeGlobe);
            bool now_globe = (idx == kLensTypeGlobe);
            r.lens_type = idx;
            ViewDefaults d = DefaultViewParamsFor(idx);
            // Reset fov to the new lens' default so e.g. first-time entry to
            // Globe uses 30° instead of inheriting Linear's 90°. .lmc loading
            // and tests bypass this combo by writing lens_type directly, so
            // they keep their fov.
            r.fov = d.fov;
            if (was_globe != now_globe) {
              // Globe is outside-in: crossing the boundary inverts both az and el.
              // az: add 180 (mod 360) — self-inverse, same formula both directions.
              r.azimuth += 180.0f;
              if (r.azimuth > 180.0f) {
                r.azimuth -= 360.0f;
              }
              // el: negate — self-inverse, same formula both directions.
              r.elevation = -r.elevation;
              // Globe el is limited to ±89° to avoid view-matrix degeneracy.
              if (now_globe) {
                r.elevation = std::max(-89.0f, std::min(89.0f, r.elevation));
              }
            }
          }
        }
        if (selected) {
          ImGui::SetItemDefaultFocus();
        }
      }
      ImGui::EndCombo();
    }
    float max_fov = LUMICE_MaxFov(static_cast<LUMICE_LensType>(r.lens_type));
    ImGui::BeginDisabled(full_sky);
    SliderWithInput("FOV##view", &r.fov, 1.0f, max_fov, "%.0f");
    ImGui::EndDisabled();
    bool is_globe = (r.lens_type == kLensTypeGlobe);
    ImGui::SeparatorText("Visibility");
    if (full_sky) {
      ImGui::BeginDisabled();
    }
    ImGui::RadioButton("Upper##visible", &r.visible, kVisibleUpper);
    ImGui::SameLine();
    ImGui::RadioButton("Full##visible", &r.visible, kVisibleFull);
    ImGui::SameLine();
    ImGui::RadioButton("Lower##visible", &r.visible, kVisibleLower);
    ImGui::SameLine(0, 20);
    if (!full_sky) {
      ImGui::BeginDisabled(is_globe);
    }
    ImGui::Checkbox("Front##visible", &r.front);
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
      ImGui::SetTooltip("Show front hemisphere only\n(combine with Upper/Full/Lower)");
    }
    if (!full_sky) {
      ImGui::EndDisabled();
    }
    if (full_sky) {
      ImGui::EndDisabled();
    }
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
    float el_lim = is_globe ? 89.0f : 90.0f;
    ImGui::BeginDisabled(full_sky);
    SliderWithInput("Elevation##view", &r.elevation, -el_lim, el_lim, "%.2f");
    SliderWithInput("Azimuth##view", &r.azimuth, -180.0f, 180.0f, "%.2f");
    ImGui::EndDisabled();
    ImGui::BeginDisabled(full_sky || is_globe);
    SliderWithInput("Roll##view", &r.roll, -180.0f, 180.0f, "%.2f");
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
      g_state.MarkDirty();
    }

    ImGui::PopItemWidth();
  }

  // ---- Display Group ----
  if (ImGui::CollapsingHeader("Display", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::PushItemWidth(-(kLabelColWidth + ImGui::GetStyle().ItemSpacing.x));
    ImGui::SeparatorText("Rendering");
    const char* res_labels[] = { "512", "1024", "2048", "4096" };
    ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.45f, 0.28f, 0.12f, 0.6f));
    if (ImGui::Combo("Resolution##display", &r.sim_resolution_index, res_labels, kSimResolutionCount)) {
      g_state.MarkDirty();
    }
    ImGui::PopStyleColor();
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("Re-runs simulation; accumulated rays reset");
    }
    ImGui::BeginGroup();
    SliderWithInput("EV##display", &r.exposure_offset, -6.0f, 6.0f, "%.1f");
    ImGui::EndGroup();
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("Exposure value offset for display brightness");
    }

    ImGui::SeparatorText("Aspect Ratio");
    int preset_idx = static_cast<int>(g_state.aspect_preset);
    const char* preview_label = kAspectPresetNames[preset_idx];
    if (ImGui::BeginCombo("Preset##display_aspect", preview_label)) {
      for (int i = 0; i < kAspectPresetCount; i++) {
        bool is_match_bg = (static_cast<AspectPreset>(i) == AspectPreset::kMatchBg);
        bool disabled = is_match_bg && !g_preview.HasBackground();
        if (disabled) {
          ImGui::BeginDisabled();
        }
        bool selected = (i == preset_idx);
        if (ImGui::Selectable(kAspectPresetNames[i], selected)) {
          g_state.aspect_preset = static_cast<AspectPreset>(i);
          ApplyAspectRatio(window, g_state.aspect_preset, g_state.aspect_portrait);
        }
        if (selected) {
          ImGui::SetItemDefaultFocus();
        }
        if (disabled) {
          ImGui::EndDisabled();
        }
      }
      ImGui::EndCombo();
    }
    bool disable_flip = (g_state.aspect_preset == AspectPreset::kFree || g_state.aspect_preset == AspectPreset::k1x1 ||
                         g_state.aspect_preset == AspectPreset::kMatchBg);
    ImGui::BeginDisabled(disable_flip);
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
      const ImVec4 kClampWarnColor = ImVec4(1.0f, 0.7f, 0.2f, 1.0f);
      ImGui::PushStyleColor(ImGuiCol_Text, kClampWarnColor);
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
    ImGui::Checkbox("Show##display_bg", &g_state.bg_show);
    ImGui::BeginDisabled(!g_state.bg_show);
    SliderWithInput("Alpha##display", &g_state.bg_alpha, 0.0f, 1.0f, "%.2f");
    ImGui::EndDisabled();
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
      ImGui::Checkbox(line_id, line_v);
      ImGui::SameLine(label_col_x);
      ImGui::Checkbox(label_id, label_v);
    };

    overlay_row("Horizon", "##horizon_color", g_state.horizon_color, "Line##horizon", &g_state.show_horizon_line,
                "Label##horizon", &g_state.show_horizon_label);
    SliderWithInput("Alpha##horizon", &g_state.horizon_alpha, 0.0f, 1.0f, "%.2f");

    overlay_row("Grid", "##grid_color", g_state.grid_color, "Line##grid", &g_state.show_grid_line, "Label##grid",
                &g_state.show_grid_label);
    SliderWithInput("Alpha##grid", &g_state.grid_alpha, 0.0f, 1.0f, "%.2f");

    overlay_row("Angular Distance", "##sun_circles_color", g_state.sun_circles_color, "Line##sun_circles",
                &g_state.show_sun_circles_line, "Label##sun_circles", &g_state.show_sun_circles_label);
    SliderWithInput("Alpha##sun_circles", &g_state.sun_circles_alpha, 0.0f, 1.0f, "%.2f");

    if (g_state.show_sun_circles_line || g_state.show_sun_circles_label) {
      if (ImGui::Button("Edit Angles...##overlay")) {
        ImGui::OpenPopup("SunCirclesEdit");
      }
      if (ImGui::BeginPopup("SunCirclesEdit")) {
        bool at_limit = static_cast<int>(g_state.sun_circle_angles.size()) >= kMaxSunCircles;

        // Preset buttons
        const float presets[] = { 9.0f, 22.0f, 28.0f, 46.0f };
        for (float p : presets) {
          bool already = false;
          for (float a : g_state.sun_circle_angles) {
            if (std::abs(a - p) < 0.01f) {
              already = true;
              break;
            }
          }
          char label[16];
          std::snprintf(label, sizeof(label), "%.0f\xc2\xb0", p);
          if (already || at_limit) {
            ImGui::BeginDisabled();
          }
          if (ImGui::Button(label)) {
            g_state.sun_circle_angles.push_back(p);
            std::sort(g_state.sun_circle_angles.begin(), g_state.sun_circle_angles.end());
          }
          if (already || at_limit) {
            ImGui::EndDisabled();
          }
          ImGui::SameLine();
        }
        ImGui::NewLine();

        // Custom angle input
        static float custom_angle = 22.0f;
        ImGui::PushItemWidth(60.0f);
        ImGui::InputFloat("##custom_angle", &custom_angle, 0.0f, 0.0f, "%.1f");
        ImGui::PopItemWidth();
        ImGui::SameLine();
        if (at_limit) {
          ImGui::BeginDisabled();
        }
        if (ImGui::Button("+##add_circle")) {
          custom_angle = std::max(0.1f, std::min(180.0f, custom_angle));
          g_state.sun_circle_angles.push_back(custom_angle);
          std::sort(g_state.sun_circle_angles.begin(), g_state.sun_circle_angles.end());
        }
        if (at_limit) {
          ImGui::EndDisabled();
        }

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
    ImGui::Checkbox("##zenith_nadir_line", &g_state.show_zenith_nadir_line);
    SliderWithInput("Alpha##zenith_nadir", &g_state.zenith_nadir_alpha, 0.0f, 1.0f, "%.2f");
    SliderWithInput("Radius##zenith_nadir", &g_state.zenith_nadir_radius_px, 2.0f, 20.0f, "%.1f px");

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
      const bool composite_active = g_server != nullptr && !g_state.raypath_color.empty();
      // Fix B: push value == manual EV offset only (no ev_auto).
      const float composite_ev_push = rc.exposure_offset;
      if (lumice::gui::ShouldPushCompositeExposure(composite_active, s_last_composite_active, composite_ev_push,
                                                   s_last_pushed_ev, kCompositeEvPushEpsilon)) {
        LUMICE_SetCompositeExposure(g_server, composite_ev_push);
        // Wake a paused poller so the next frame can pick up the freshly re-baked
        // composite even after a finite sim has completed (same rationale as the
        // color-window PushDisplayState path — see task-345.2 (③) in color_window.cpp).
        // No-op when the poller is already running.
        g_server_poller.EnsureRunning(g_server);
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

    // Mouse interaction: orbit with drag, FOV with scroll.
    // Disabled for lenses in kFullSkyLensTypes (dual fisheye 4-6, rectangular 7,
    // dual orthographic 9): their shader path skips the view matrix so view
    // angles + FOV have no visual effect.
    bool full_sky = LensIsFullSky(rc.lens_type);
    ImVec2 avail = ImGui::GetContentRegionAvail();
    ImGui::InvisibleButton("##preview_interact", avail);

    if (!full_sky) {
      bool is_hovered = ImGui::IsItemHovered();
      bool is_active = ImGui::IsItemActive();

      ImGuiIO& io = ImGui::GetIO();
      if (is_active && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
        ImVec2 delta = io.MouseDelta;
        // Globe is outside-in: u_view_matrix * (0,0,D) yields camera world
        // position, so +az/+el move the camera and the sphere drifts in the
        // opposite direction. Flip both signs so a right/down drag moves the
        // sphere right/down, matching the inside-out lenses' feel.
        if (rc.lens_type == kLensTypeGlobe) {
          rc.azimuth += delta.x * 0.3f;
          rc.elevation -= delta.y * 0.3f;
        } else {
          rc.azimuth -= delta.x * 0.3f;
          rc.elevation += delta.y * 0.3f;
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

      if (is_hovered && io.MouseWheel != 0.0f) {
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
      ImGui::TextColored(ImVec4(0.4f, 0.8f, 0.4f, 1.0f), "Ready");
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
      ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.0f, 1.0f), "Modified");
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
    ImGui::TextUnformatted("Some filters in the imported config cannot be represented in the GUI:");
    ImGui::Separator();
    ImGui::TextUnformatted(active_msg.c_str());
    ImGui::Separator();
    ImGui::TextUnformatted("These filters were dropped. The rendered image will differ from the original.");
    if (ImGui::Button("OK", ImVec2(80, 0))) {
      active_msg.clear();
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

void RenderUnsavedPopup(GLFWwindow* window) {
  if (g_show_unsaved_popup) {
    ImGui::OpenPopup("Unsaved Changes");
    g_show_unsaved_popup = false;
  }

  if (ImGui::BeginPopupModal("Unsaved Changes", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::Text("You have unsaved changes. Save before continuing?");
    ImGui::Separator();

    if (ImGui::Button("Save", ImVec2(80, 0))) {
      DoSave();
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
      ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Don't Save", ImVec2(100, 0))) {
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
  if (ImGui::Checkbox("File", &g_state.log_to_file)) {
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
          color = ImVec4(1.0f, 0.85f, 0.3f, 1.0f);
          break;
        case spdlog::level::err:
        case spdlog::level::critical:
          color = ImVec4(1.0f, 0.3f, 0.3f, 1.0f);
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
