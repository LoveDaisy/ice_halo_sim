#include <GLFW/glfw3.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include "config/render_config.hpp"
#include "gui/app.hpp"
#include "gui/edit_modals.hpp"
#include "gui/gui_constants.hpp"
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
//     - panels.cpp::SliderWithPreset BeginPopup
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
    const char* left_toggle_label = g_state.left_panel_collapsed ? ">##left_panel_toggle" : "<##left_panel_toggle";
    if (ImGui::Button(left_toggle_label)) {
      g_state.left_panel_collapsed = !g_state.left_panel_collapsed;
    }
    ImGui::SameLine();
    ImGui::TextDisabled("|");
    ImGui::SameLine();
  }

  // Run/Stop — fixed width to prevent layout shift
  bool simulating = (g_state.sim_state == SimState::kSimulating);
  const auto& style = ImGui::GetStyle();
  float run_stop_width =
      std::max(ImGui::CalcTextSize("Run").x, ImGui::CalcTextSize("Stop").x) + style.FramePadding.x * 2;
  if (simulating) {
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.6f, 0.1f, 0.1f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.7f, 0.2f, 0.2f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.5f, 0.05f, 0.05f, 1.0f));
    if (ImGui::Button("Stop", ImVec2(run_stop_width, 0))) {
      DoStop();
    }
    ImGui::PopStyleColor(3);
  } else {
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.45f, 0.15f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.2f, 0.55f, 0.2f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.1f, 0.35f, 0.1f, 1.0f));
    if (ImGui::Button("Run", ImVec2(run_stop_width, 0))) {
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
  ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.0f, 1.0f), "!");
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

  // File operations — New/Open disabled while simulating; Save menu itself stays
  // enabled so read-only exports (Screenshot / Dual Fisheye Equal Area / Equirectangular /
  // Config JSON) remain reachable.
  if (simulating) {
    ImGui::BeginDisabled();
  }
  if (ImGui::Button("New")) {
    CheckUnsavedAndDo(PendingAction::kNew);
  }
  ImGui::SameLine();
  if (ImGui::Button("Open")) {
    CheckUnsavedAndDo(PendingAction::kOpen);
  }
  if (simulating) {
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
      if (simulating) {
        ImGui::BeginDisabled();
      }
      if (ImGui::MenuItem("Save")) {
        DoSave();
      }
      if (ImGui::MenuItem("Save Copy")) {
        DoSaveAs();
      }
      if (simulating) {
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

  // Right-panel collapse toggle — right-aligned so it sits flush with the right panel's outer edge.
  // Also note: when the right panel is already collapsed, RenderCollapsedStrip's internal button
  // still expands it; this top-bar toggle simply offers a symmetric alternate entry point.
  {
    const char* right_toggle_label = g_state.right_panel_collapsed ? "<##right_panel_toggle" : ">##right_panel_toggle";
    // Use the max width of both label states so the button's left edge doesn't jitter when toggled.
    float w_expanded = ImGui::CalcTextSize(">##right_panel_toggle", nullptr, true).x;
    float w_collapsed = ImGui::CalcTextSize("<##right_panel_toggle", nullptr, true).x;
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
  bool hovered = (io.MousePos.x >= pos.x && io.MousePos.x <= max.x && io.MousePos.y >= pos.y && io.MousePos.y <= max.y);
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
    RenderCollapsedStrip(">", 0, kTopBarHeight, panel_height, &g_state.left_panel_collapsed);
    return;
  }

  SetNextPanelGeometry(0, kTopBarHeight, kLeftPanelWidth, panel_height);
  ImGui::Begin("##LeftPanel", nullptr,
               ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                   ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse |
                   ImGuiWindowFlags_NoBringToFrontOnFocus);

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
    Layer new_layer;
    new_layer.entries.emplace_back();
    g_state.layers.push_back(std::move(new_layer));
    g_thumbnail_cache.OnLayerStructureChanged();
    g_state.MarkDirty();
  }

  // Process edit request: open modal if an edit button was clicked
  if (GetEditRequest().target != EditTarget::kNone) {
    OpenEditModal(GetEditRequest(), g_state);
    ResetEditRequest();
  }

  ImGui::End();
}

void RenderRightPanel(GLFWwindow* window, float window_width, float window_height) {
  float panel_height = window_height - kTopBarHeight - kStatusBarHeight;

  if (g_state.right_panel_collapsed) {
    RenderCollapsedStrip("<", window_width - kCollapseBtnSize, kTopBarHeight, panel_height,
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
    ImGui::SeparatorText("Projection");
    // Use BeginCombo + Selectable to honour kLensTypePresentationOrder (gui_state.hpp).
    // The enum value (r.lens_type) is preserved unchanged; only the display order differs.
    if (ImGui::BeginCombo("Lens Type##view", kLensTypeNames[r.lens_type])) {
      for (int idx : kLensTypePresentationOrder) {
        bool selected = (r.lens_type == idx);
        if (ImGui::Selectable(kLensTypeNames[idx], selected)) {
          r.lens_type = idx;
        }
        if (selected) {
          ImGui::SetItemDefaultFocus();
        }
      }
      ImGui::EndCombo();
    }
    float max_fov = MaxFov(static_cast<LensParam::LensType>(r.lens_type));
    ImGui::BeginDisabled(full_sky);
    SliderWithInput("FOV##view", &r.fov, 1.0f, max_fov, "%.0f");
    ImGui::EndDisabled();
    ImGui::Combo("Visible##view", &r.visible, kVisibleNames, kVisibleCount);

    ImGui::SeparatorText("Camera");
    ImGui::BeginDisabled(full_sky);
    SliderWithInput("Elevation##view", &r.elevation, -90.0f, 90.0f, "%.2f");
    SliderWithInput("Azimuth##view", &r.azimuth, -180.0f, 180.0f, "%.2f");
    SliderWithInput("Roll##view", &r.roll, -180.0f, 180.0f, "%.2f");
    ImGui::EndDisabled();

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
    SliderWithInput("EV##display", &r.exposure_offset, -6.0f, 6.0f, "%.1f");

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
      ImGui::PushID("aspect_clamp_warning");
      ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f), "Screen too small — preview ~%.2f:1 (export %.2f:1)",
                         g_state.aspect_clamp.achieved_preview_ratio, g_state.aspect_clamp.requested_preview_ratio);
      ImGui::PopID();
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
    ImGui::Checkbox("Horizon##overlay", &g_state.show_horizon);
    ImGui::SameLine();
    ImGui::ColorEdit3("##horizon_color", g_state.horizon_color, ImGuiColorEditFlags_NoInputs);
    SliderWithInput("Alpha##horizon", &g_state.horizon_alpha, 0.0f, 1.0f, "%.2f");
    ImGui::Checkbox("Grid##overlay", &g_state.show_grid);
    ImGui::SameLine();
    ImGui::ColorEdit3("##grid_color", g_state.grid_color, ImGuiColorEditFlags_NoInputs);
    SliderWithInput("Alpha##grid", &g_state.grid_alpha, 0.0f, 1.0f, "%.2f");
    ImGui::Checkbox("Sun Circles##overlay", &g_state.show_sun_circles);
    ImGui::SameLine();
    ImGui::ColorEdit3("##sun_circles_color", g_state.sun_circles_color, ImGuiColorEditFlags_NoInputs);
    SliderWithInput("Alpha##sun_circles", &g_state.sun_circles_alpha, 0.0f, 1.0f, "%.2f");

    if (g_state.show_sun_circles) {
      if (ImGui::Button("Edit Circles...##overlay")) {
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
    float max_fov = MaxFov(static_cast<LensParam::LensType>(r.lens_type));
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
    pp.view_proj.lens_type = rc.lens_type;
    pp.view_proj.fov = rc.fov;
    pp.view_proj.elevation = rc.elevation;
    pp.view_proj.azimuth = rc.azimuth;
    pp.view_proj.roll = rc.roll;
    pp.view_proj.visible = rc.visible;
    pp.exposure.intensity_factor = std::pow(2.0f, rc.exposure_offset);
    pp.exposure.intensity_scale =
        g_state.snapshot_intensity > 0 ? pp.exposure.intensity_factor / g_state.snapshot_intensity : 0.0f;
    // Overlap parameters for dual fisheye texture sampling.
    pp.source.max_abs_dz = kDualFisheyeOverlap;
    pp.source.r_scale = 1.0f / std::sqrt(1.0f + kDualFisheyeOverlap);
    pp.bg.enabled = g_state.bg_show && g_preview.HasBackground();
    pp.bg.alpha = g_state.bg_alpha;
    pp.bg.aspect = g_preview.GetBgAspect();

    // Auxiliary line overlay parameters
    pp.overlay.show_horizon = g_state.show_horizon;
    pp.overlay.show_grid = g_state.show_grid;
    pp.overlay.show_sun_circles = g_state.show_sun_circles;
    std::copy(std::begin(g_state.horizon_color), std::end(g_state.horizon_color), std::begin(pp.overlay.horizon_color));
    std::copy(std::begin(g_state.grid_color), std::end(g_state.grid_color), std::begin(pp.overlay.grid_color));
    std::copy(std::begin(g_state.sun_circles_color), std::end(g_state.sun_circles_color),
              std::begin(pp.overlay.sun_circles_color));
    pp.overlay.horizon_alpha = g_state.horizon_alpha;
    pp.overlay.grid_alpha = g_state.grid_alpha;
    pp.overlay.sun_circles_alpha = g_state.sun_circles_alpha;
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

    // Overlay labels at viewport edges (drawn on the preview window's draw list so
    // modals correctly occlude them). BuildOverlayLabelInput is shared with
    // DoExportPreviewPng (off-screen FBO path) so both call sites produce
    // byte-identical OverlayLabelInput for a given state.
    if (g_state.show_horizon || g_state.show_grid || g_state.show_sun_circles) {
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
        rc.azimuth -= delta.x * 0.3f;
        rc.elevation += delta.y * 0.3f;
        rc.elevation = std::max(-90.0f, std::min(90.0f, rc.elevation));
      }

      if (is_hovered && io.MouseWheel != 0.0f) {
        float fov_max = MaxFov(static_cast<LensParam::LensType>(rc.lens_type));
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
    unsigned long n = g_state.stats_sim_ray_num;
    char buf[64];
    if (n >= 1'000'000'000UL) {
      snprintf(buf, sizeof(buf), "| Rays: %.1f x10^9", n / 1e9);
    } else if (n >= 1'000'000UL) {
      snprintf(buf, sizeof(buf), "| Rays: %.1f x10^6", n / 1e6);
    } else {
      snprintf(buf, sizeof(buf), "| Rays: %.1f x10^3", n / 1e3);
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

  // Log panel toggle button (right-aligned)
  {
    const char* log_label = g_state.log_panel_open ? "Log [v]" : "Log [>]";
    float button_w = ImGui::CalcTextSize(log_label).x + ImGui::GetStyle().FramePadding.x * 2;
    ImGui::SameLine(ImGui::GetWindowWidth() - button_w - ImGui::GetStyle().WindowPadding.x);
    if (ImGui::SmallButton(log_label)) {
      g_state.log_panel_open = !g_state.log_panel_open;
    }
  }

  ImGui::End();
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
