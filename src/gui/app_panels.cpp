#include <GLFW/glfw3.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include "config/render_config.hpp"
#include "gui/app.hpp"
#include "gui/gui_constants.hpp"
#include "gui/gui_logger.hpp"
#include "gui/overlay_labels.hpp"
#include "gui/panels.hpp"
#include "imgui.h"

namespace lumice::gui {

using SimState = GuiState::SimState;

void RenderTopBar(float window_width) {
  ImGui::SetNextWindowPos(ImVec2(0, 0));
  ImGui::SetNextWindowSize(ImVec2(window_width, kTopBarHeight));
  ImGui::Begin("##TopBar", nullptr,
               ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                   ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoCollapse);

  // Run/Stop + Revert
  bool simulating = (g_state.sim_state == SimState::kSimulating);
  if (simulating) {
    if (ImGui::Button("Stop")) {
      DoStop();
    }
  } else {
    if (ImGui::Button("Run")) {
      DoRun();
    }
  }
  if (g_state.sim_state == SimState::kModified) {
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.0f, 1.0f), "!");
    ImGui::SameLine();
    if (ImGui::SmallButton("Revert")) {
      DoRevert();
    }
  }

  ImGui::SameLine();
  ImGui::TextDisabled("|");
  ImGui::SameLine();

  // File operations
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
  ImGui::SameLine();
  if (ImGui::Button("Save")) {
    DoSave();
  }
  ImGui::SameLine();
  if (ImGui::Button("Save As")) {
    DoSaveAs();
  }
  if (simulating) {
    ImGui::EndDisabled();
  }
  ImGui::SameLine();
  ImGui::TextDisabled("|");
  ImGui::SameLine();
  {
    if (ImGui::Button("Export")) {
      ImGui::OpenPopup("ExportMenu");
    }
    if (ImGui::BeginPopup("ExportMenu")) {
      bool no_texture = !g_preview.HasTexture();
      bool has_server = g_server != nullptr && g_state.sim_state != GuiState::SimState::kIdle;
      if (ImGui::MenuItem("Screenshot...", nullptr, false, !no_texture)) {
        DoExportPreviewPng();
      }
      if (ImGui::MenuItem("Panorama...", nullptr, false, has_server)) {
        DoExportEquirectPng();
      }
      if (ImGui::MenuItem("Config JSON...")) {
        DoExportConfigJson();
      }
      ImGui::Separator();
      ImGui::MenuItem("Include Texture in .lmc", nullptr, &g_state.save_texture);
      ImGui::EndPopup();
    }
  }

  // Ray count shortcut
  ImGui::SameLine();
  ImGui::TextDisabled("|");
  ImGui::SameLine();

  if (simulating) {
    ImGui::BeginDisabled();
  }
  if (ImGui::Checkbox("Infinite##topbar", &g_state.sim.infinite)) {
    g_state.MarkDirty();
  }
  ImGui::SameLine();
  ImGui::SetNextItemWidth(120);
  if (!g_state.sim.infinite) {
    if (SliderWithInput("Rays(M)##topbar", &g_state.sim.ray_num_millions, 0.1f, 100.0f)) {
      g_state.MarkDirty();
    }
  } else {
    ImGui::BeginDisabled();
    SliderWithInput("Rays(M)##topbar", &g_state.sim.ray_num_millions, 0.1f, 100.0f);
    ImGui::EndDisabled();
  }
  if (simulating) {
    ImGui::EndDisabled();
  }

  ImGui::End();
}

namespace {
constexpr float kCollapseBtnSize = 20.0f;

// Draw a collapse/expand button as a foreground overlay using ImGui theme colors.
// Returns true if clicked.
bool OverlayButton(const char* label, float screen_x, float screen_y) {
  ImVec2 pos(screen_x, screen_y);
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
// No ImGui window needed — avoids WindowMinSize issues.
void RenderCollapsedStrip(const char* btn_label, float strip_x, float strip_y, float strip_h, bool* collapsed) {
  ImDrawList* fg = ImGui::GetForegroundDrawList();
  fg->AddRectFilled(ImVec2(strip_x, strip_y), ImVec2(strip_x + kCollapseBtnSize, strip_y + strip_h),
                    ImGui::GetColorU32(ImGuiCol_WindowBg));
  float btn_y = strip_y + (strip_h - kCollapseBtnSize) * 0.5f;
  if (OverlayButton(btn_label, strip_x, btn_y)) {
    *collapsed = false;
  }
}
}  // namespace

void RenderLeftPanel(float window_height) {
  float panel_height = window_height - kTopBarHeight - kStatusBarHeight;

  if (g_panel_collapsed) {
    RenderCollapsedStrip(">", 0, kTopBarHeight, panel_height, &g_panel_collapsed);
    return;
  }

  ImGui::SetNextWindowPos(ImVec2(0, kTopBarHeight));
  ImGui::SetNextWindowSize(ImVec2(kLeftPanelWidth, panel_height));
  ImGui::Begin("##LeftPanel", nullptr,
               ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                   ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);

  if (ImGui::BeginTabBar("ConfigTabs")) {
    if (ImGui::BeginTabItem("Crystal")) {
      bool has_crystal =
          g_state.selected_crystal >= 0 && g_state.selected_crystal < static_cast<int>(g_state.crystals.size());

      if (has_crystal) {
        // Split: scrollable params (upper) + fixed preview (lower).
        // Preview is square (side = panel content width). Params get the remainder.
        float content_w = ImGui::GetContentRegionAvail().x;
        float avail_h = ImGui::GetContentRegionAvail().y;
        auto& style = ImGui::GetStyle();
        float separator_h = ImGui::GetTextLineHeight() + style.SeparatorTextPadding.y * 2 + style.ItemSpacing.y;
        float controls_h = ImGui::GetFrameHeight() + style.ItemSpacing.y;
        float preview_size = content_w;
        float params_h = avail_h - preview_size - separator_h - controls_h;

        // Parameters scroll region (upper part)
        ImGui::BeginChild("##CrystalParams", ImVec2(0, params_h), ImGuiChildFlags_None);
        RenderCrystalTab(g_state);
        ImGui::EndChild();

        // Fixed crystal 3D preview (bottom)
        ImGui::SeparatorText("3D Preview");
        auto& cr = g_state.crystals[g_state.selected_crystal];

        // Update mesh if crystal changed
        int hash = CrystalParamHash(cr);
        if (cr.id != g_crystal_mesh_id || hash != g_crystal_mesh_hash) {
          char json_buf[512];
          auto* fd = cr.face_distance;
          if (cr.type == CrystalType::kPrism) {
            snprintf(json_buf, sizeof(json_buf),
                     R"({"type":"prism","shape":{"height":%.4f,)"
                     R"("face_distance":[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f]}})",
                     cr.height, fd[0], fd[1], fd[2], fd[3], fd[4], fd[5]);
          } else {
            snprintf(json_buf, sizeof(json_buf),
                     R"({"type":"pyramid","shape":{"prism_h":%.4f,"upper_h":%.4f,"lower_h":%.4f,)"
                     R"("upper_wedge_angle":%.4f,"lower_wedge_angle":%.4f,)"
                     R"("face_distance":[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f]}})",
                     cr.prism_h, cr.upper_h, cr.lower_h, cr.upper_alpha, cr.lower_alpha, fd[0], fd[1], fd[2], fd[3],
                     fd[4], fd[5]);
          }

          LUMICE_CrystalMesh mesh{};
          if (LUMICE_GetCrystalMesh(nullptr, json_buf, &mesh) == LUMICE_OK) {
            for (int vi = 0; vi < mesh.vertex_count; vi++) {
              float y = mesh.vertices[vi * 3 + 1];
              float z = mesh.vertices[vi * 3 + 2];
              mesh.vertices[vi * 3 + 1] = z;
              mesh.vertices[vi * 3 + 2] = -y;
            }
            for (int ei = 0; ei < mesh.edge_count; ei++) {
              for (int side = 0; side < 2; side++) {
                float* n = &mesh.edge_face_normals[ei * 6 + side * 3];
                float ny = n[1];
                float nz = n[2];
                n[1] = nz;
                n[2] = -ny;
              }
            }

            if (mesh.vertex_count > 0) {
              float min_x = mesh.vertices[0], max_x = mesh.vertices[0];
              float min_y = mesh.vertices[1], max_y = mesh.vertices[1];
              float min_z = mesh.vertices[2], max_z = mesh.vertices[2];
              for (int vi = 1; vi < mesh.vertex_count; vi++) {
                float x = mesh.vertices[vi * 3];
                float y = mesh.vertices[vi * 3 + 1];
                float z = mesh.vertices[vi * 3 + 2];
                min_x = std::min(min_x, x);
                max_x = std::max(max_x, x);
                min_y = std::min(min_y, y);
                max_y = std::max(max_y, y);
                min_z = std::min(min_z, z);
                max_z = std::max(max_z, z);
              }
              float extent = std::max({ max_x - min_x, max_y - min_y, max_z - min_z });
              if (extent > 1e-6f) {
                float scale = 1.0f / extent;
                for (int vi = 0; vi < mesh.vertex_count; vi++) {
                  mesh.vertices[vi * 3] *= scale;
                  mesh.vertices[vi * 3 + 1] *= scale;
                  mesh.vertices[vi * 3 + 2] *= scale;
                }
              }
            }

            g_crystal_renderer.UpdateMesh(mesh.vertices, mesh.vertex_count, mesh.edges, mesh.edge_count, mesh.triangles,
                                          mesh.triangle_count, mesh.edge_face_normals);
            g_crystal_mesh_id = cr.id;
            g_crystal_mesh_hash = hash;
          }
        }

        // Render to FBO
        auto crystal_style = static_cast<CrystalStyle>(g_crystal_style);
        g_crystal_renderer.Render(g_crystal_rotation, g_crystal_zoom, crystal_style);

        // Display FBO texture
        ImVec2 area_start = ImGui::GetCursorScreenPos();
        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        draw_list->AddRectFilled(area_start, ImVec2(area_start.x + preview_size, area_start.y + preview_size),
                                 IM_COL32(38, 38, 38, 255));

        auto tex_id = static_cast<ImTextureID>(g_crystal_renderer.GetTextureId());
        ImVec2 uv0(0, 1);
        ImVec2 uv1(1, 0);
        ImGui::Image(tex_id, ImVec2(preview_size, preview_size), uv0, uv1);

        ImGui::SetCursorScreenPos(area_start);
        ImGui::InvisibleButton("##CrystalPreviewBtn", ImVec2(preview_size, preview_size));
        if (ImGui::IsItemHovered()) {
          ImGui::SetItemKeyOwner(ImGuiKey_MouseWheelY);
          ImGuiIO& io = ImGui::GetIO();
          if (ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
            ApplyTrackballRotation(io.MouseDelta.x, io.MouseDelta.y);
          }
          if (io.MouseWheel != 0.0f) {
            g_crystal_zoom *= (1.0f - io.MouseWheel * 0.1f);
            g_crystal_zoom = std::max(0.5f, std::min(10.0f, g_crystal_zoom));
          }
        }
        ImGui::PushItemWidth(120.0f);
        ImGui::Combo("##CrystalStyle", &g_crystal_style, kCrystalStyleNames, kCrystalStyleCount);
        ImGui::PopItemWidth();
        ImGui::SameLine();
        if (ImGui::SmallButton("Reset View")) {
          ResetCrystalView();
        }
      } else {
        // No crystal selected — full space for parameters
        ImGui::BeginChild("##CrystalParamsOnly", ImVec2(0, 0), ImGuiChildFlags_None);
        RenderCrystalTab(g_state);
        ImGui::EndChild();
      }

      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Scene")) {
      ImGui::BeginChild("##SceneParams", ImVec2(0, 0), ImGuiChildFlags_None);
      RenderSceneTab(g_state);
      ImGui::EndChild();
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Filter")) {
      ImGui::BeginChild("##FilterParams", ImVec2(0, 0), ImGuiChildFlags_None);
      RenderFilterTab(g_state);
      ImGui::EndChild();
      ImGui::EndTabItem();
    }
    ImGui::EndTabBar();
  }

  ImGui::End();

  // Collapse button overlay at top-right of left panel (offset inward to avoid scrollbar)
  if (OverlayButton("<", kLeftPanelWidth - kCollapseBtnSize - 20, kTopBarHeight + 4)) {
    g_panel_collapsed = true;
  }
}

void RenderRightPanel(GLFWwindow* window, float window_width, float window_height) {
  float panel_height = window_height - kTopBarHeight - kStatusBarHeight;

  if (g_state.right_panel_collapsed) {
    RenderCollapsedStrip("<", window_width - kCollapseBtnSize, kTopBarHeight, panel_height,
                         &g_state.right_panel_collapsed);
    return;
  }

  float panel_x = window_width - kRightPanelWidth;
  ImGui::SetNextWindowPos(ImVec2(panel_x, kTopBarHeight));
  ImGui::SetNextWindowSize(ImVec2(kRightPanelWidth, panel_height));
  ImGui::Begin(
      "##RightPanel", nullptr,
      ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);

  if (g_state.selected_renderer < 0 || g_state.selected_renderer >= static_cast<int>(g_state.renderers.size())) {
    ImGui::End();
    return;
  }

  auto& r = g_state.renderers[g_state.selected_renderer];
  bool full_sky = (r.lens_type >= 4);

  if (ImGui::BeginTabBar("ViewTabs")) {
    // ---- View Tab ----
    if (ImGui::BeginTabItem("View")) {
      ImGui::PushItemWidth(-(kLabelColWidth + ImGui::GetStyle().ItemSpacing.x));
      ImGui::SeparatorText("Projection");
      ImGui::Combo("Lens Type##view", &r.lens_type, kLensTypeNames, kLensTypeCount);
      float max_fov = MaxFov(static_cast<LensParam::LensType>(r.lens_type));
      ImGui::BeginDisabled(full_sky);
      SliderWithInput("FOV##view", &r.fov, 1.0f, max_fov, "%.0f");
      ImGui::EndDisabled();
      ImGui::Combo("Visible##view", &r.visible, kVisibleNames, kVisibleCount);

      ImGui::SeparatorText("Camera");
      ImGui::BeginDisabled(full_sky);
      SliderWithInput("Elevation##view", &r.elevation, -90.0f, 90.0f, "%.1f");
      SliderWithInput("Azimuth##view", &r.azimuth, -180.0f, 180.0f, "%.1f");
      SliderWithInput("Roll##view", &r.roll, -180.0f, 180.0f, "%.1f");
      ImGui::EndDisabled();

      ImGui::PopItemWidth();
      ImGui::EndTabItem();
    }

    // ---- Display Tab ----
    if (ImGui::BeginTabItem("Display")) {
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
      SliderWithInput("EV##display", &r.exposure_offset, -3.0f, 7.0f, "%.1f");

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
      bool disable_flip =
          (g_state.aspect_preset == AspectPreset::kFree || g_state.aspect_preset == AspectPreset::k1x1 ||
           g_state.aspect_preset == AspectPreset::kMatchBg);
      ImGui::BeginDisabled(disable_flip);
      const char* flip_label = g_state.aspect_portrait ? "Portrait" : "Landscape";
      if (ImGui::Button(flip_label)) {
        g_state.aspect_portrait = !g_state.aspect_portrait;
        ApplyAspectRatio(window, g_state.aspect_preset, g_state.aspect_portrait);
      }
      ImGui::EndDisabled();

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
      ImGui::EndTabItem();
    }

    // ---- Overlay Tab ----
    if (ImGui::BeginTabItem("Overlay")) {
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
      ImGui::EndTabItem();
    }

    ImGui::EndTabBar();
  }

  ImGui::End();

  // Collapse button overlay at top-right of right panel (symmetric with left panel)
  if (OverlayButton(">", panel_x + kRightPanelWidth - kCollapseBtnSize - 4, kTopBarHeight + 4)) {
    g_state.right_panel_collapsed = true;
  }
}

void RenderPreviewPanel(GLFWwindow* window, float window_width, float window_height) {
  float left_w = g_panel_collapsed ? kCollapseBtnSize : kLeftPanelWidth;
  float right_w = g_state.right_panel_collapsed ? kCollapseBtnSize : kRightPanelWidth;
  float panel_x = left_w;
  float panel_width = window_width - left_w - right_w;
  float panel_height = window_height - kTopBarHeight - kStatusBarHeight;
  ImGui::SetNextWindowPos(ImVec2(panel_x, kTopBarHeight));
  ImGui::SetNextWindowSize(ImVec2(panel_width, panel_height));
  ImGui::Begin("##PreviewPanel", nullptr,
               ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                   ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoBackground);

  // Renderer invariants (previously in RenderViewBar, runs every frame)
  if (g_state.renderers.empty()) {
    RenderConfig r;
    r.id = g_state.next_renderer_id++;
    g_state.renderers.push_back(r);
    g_state.selected_renderer = 0;
  }
  g_state.selected_renderer = 0;
  if (g_state.selected_renderer >= 0 && g_state.selected_renderer < static_cast<int>(g_state.renderers.size())) {
    auto& r = g_state.renderers[g_state.selected_renderer];
    float max_fov = MaxFov(static_cast<LensParam::LensType>(r.lens_type));
    r.fov = std::min(r.fov, max_fov);
    if (r.lens_type >= 4) {  // Full-sky lenses: force view angles to zero
      r.elevation = 0.0f;
      r.azimuth = 0.0f;
      r.roll = 0.0f;
    }
  }

  float preview_height = panel_height;

  g_preview_vp.active = false;

  if ((g_preview.HasTexture() || g_preview.HasBackground()) && g_state.selected_renderer >= 0 &&
      g_state.selected_renderer < static_cast<int>(g_state.renderers.size())) {
    // Compute viewport in framebuffer pixels (for HiDPI)
    int fb_w = 0;
    int fb_h = 0;
    glfwGetFramebufferSize(window, &fb_w, &fb_h);
    float scale_x = static_cast<float>(fb_w) / window_width;
    float scale_y = static_cast<float>(fb_h) / window_height;

    auto& rc = g_state.renderers[g_state.selected_renderer];

    // Store viewport for deferred rendering
    g_preview_vp.active = true;
    g_preview_vp.vp_x = static_cast<int>(panel_x * scale_x);
    g_preview_vp.vp_y = static_cast<int>(kStatusBarHeight * scale_y);  // OpenGL Y is bottom-up
    g_preview_vp.vp_w = static_cast<int>(panel_width * scale_x);
    g_preview_vp.vp_h = static_cast<int>(preview_height * scale_y);
    g_preview_vp.params.lens_type = rc.lens_type;
    g_preview_vp.params.fov = rc.fov;
    g_preview_vp.params.elevation = rc.elevation;
    g_preview_vp.params.azimuth = rc.azimuth;
    g_preview_vp.params.roll = rc.roll;
    g_preview_vp.params.visible = rc.visible;
    g_preview_vp.params.intensity_factor = std::pow(2.0f, rc.exposure_offset);
    g_preview_vp.params.intensity_scale =
        g_state.snapshot_intensity > 0 ? g_preview_vp.params.intensity_factor / g_state.snapshot_intensity : 0.0f;
    // Overlap parameters for dual fisheye texture sampling.
    g_preview_vp.params.max_abs_dz = kDualFisheyeOverlap;
    g_preview_vp.params.r_scale = 1.0f / std::sqrt(1.0f + kDualFisheyeOverlap);
    g_preview_vp.params.bg_enabled = g_state.bg_show && g_preview.HasBackground();
    g_preview_vp.params.bg_alpha = g_state.bg_alpha;
    g_preview_vp.params.bg_aspect = g_preview.GetBgAspect();

    // Auxiliary line overlay parameters
    g_preview_vp.params.show_horizon = g_state.show_horizon;
    g_preview_vp.params.show_grid = g_state.show_grid;
    g_preview_vp.params.show_sun_circles = g_state.show_sun_circles;
    std::copy(std::begin(g_state.horizon_color), std::end(g_state.horizon_color),
              std::begin(g_preview_vp.params.horizon_color));
    std::copy(std::begin(g_state.grid_color), std::end(g_state.grid_color), std::begin(g_preview_vp.params.grid_color));
    std::copy(std::begin(g_state.sun_circles_color), std::end(g_state.sun_circles_color),
              std::begin(g_preview_vp.params.sun_circles_color));
    g_preview_vp.params.horizon_alpha = g_state.horizon_alpha;
    g_preview_vp.params.grid_alpha = g_state.grid_alpha;
    g_preview_vp.params.sun_circles_alpha = g_state.sun_circles_alpha;
    // Precompute sun direction in world space (azimuth fixed at 0, only altitude matters)
    constexpr float kDeg2Rad = 3.14159265358979323846f / 180.0f;
    float sa = g_state.sun.altitude * kDeg2Rad;
    g_preview_vp.params.sun_dir[0] = -std::cos(sa);
    g_preview_vp.params.sun_dir[1] = 0.0f;
    g_preview_vp.params.sun_dir[2] = -std::sin(sa);
    g_preview_vp.params.sun_circle_count = std::min(static_cast<int>(g_state.sun_circle_angles.size()), kMaxSunCircles);
    for (int i = 0; i < g_preview_vp.params.sun_circle_count; i++) {
      g_preview_vp.params.sun_circle_angles[i] = g_state.sun_circle_angles[i];
    }

    // Overlay labels at viewport edges (drawn via ImGui foreground draw list)
    if (g_state.show_horizon || g_state.show_grid || g_state.show_sun_circles) {
      OverlayLabelInput label_input{};
      label_input.lens_type = rc.lens_type;
      label_input.fov = rc.fov;
      label_input.elevation = rc.elevation;
      label_input.azimuth = rc.azimuth;
      label_input.roll = rc.roll;
      label_input.visible = rc.visible;
      label_input.show_horizon = g_state.show_horizon;
      label_input.show_grid = g_state.show_grid;
      label_input.show_sun_circles = g_state.show_sun_circles;
      std::copy(std::begin(g_preview_vp.params.sun_dir), std::end(g_preview_vp.params.sun_dir),
                std::begin(label_input.sun_dir));
      label_input.sun_circle_count = g_preview_vp.params.sun_circle_count;
      label_input.sun_circle_angles = g_preview_vp.params.sun_circle_angles;
      std::copy(std::begin(g_state.horizon_color), std::end(g_state.horizon_color),
                std::begin(label_input.horizon_color));
      std::copy(std::begin(g_state.grid_color), std::end(g_state.grid_color), std::begin(label_input.grid_color));
      std::copy(std::begin(g_state.sun_circles_color), std::end(g_state.sun_circles_color),
                std::begin(label_input.sun_circles_color));
      label_input.grid_alpha = g_state.grid_alpha;
      label_input.sun_circles_alpha = g_state.sun_circles_alpha;

      // Convert viewport from framebuffer pixels to ImGui logical screen coordinates
      float vp_sx = panel_x;
      float vp_sy = kTopBarHeight;
      float vp_sw = panel_width;
      float vp_sh = preview_height;

      static std::vector<OverlayLabel> labels;
      ComputeOverlayLabels(label_input, vp_sx, vp_sy, vp_sw, vp_sh, labels);
      DrawOverlayLabels(labels);
    }

    // Mouse interaction: orbit with drag, FOV with scroll
    // Disabled for full-sky lens types (dual fisheye, rectangular)
    bool full_sky = (rc.lens_type >= 4);
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
  ImGui::SetNextWindowPos(ImVec2(0, window_height - kStatusBarHeight));
  ImGui::SetNextWindowSize(ImVec2(window_width, kStatusBarHeight));
  ImGui::Begin("##StatusBar", nullptr,
               ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                   ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoCollapse);

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

  // Sim resolution + lens info
  if (g_state.selected_renderer >= 0 && g_state.selected_renderer < static_cast<int>(g_state.renderers.size())) {
    auto& rc = g_state.renderers[g_state.selected_renderer];
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

  ImGui::SetNextWindowPos(ImVec2(0, window_height - kLogPanelHeight - kStatusBarHeight));
  ImGui::SetNextWindowSize(ImVec2(window_width, kLogPanelHeight));
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
