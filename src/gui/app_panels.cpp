#include <GLFW/glfw3.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include "config/render_config.hpp"
#include "gui/app.hpp"
#include "gui/gui_logger.hpp"
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

  ImGui::Text("Lumice");
  ImGui::SameLine();

  // Push buttons to the right side
  float button_start_x = window_width - 440.0f;
  if (button_start_x > ImGui::GetCursorPosX()) {
    ImGui::SetCursorPosX(button_start_x);
  }

  bool simulating = (g_state.sim_state == SimState::kSimulating);
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
  ImGui::SameLine();
  {
    bool no_texture = !g_preview.HasTexture();
    if (no_texture) {
      ImGui::BeginDisabled();
    }
    if (ImGui::Button("Export")) {
      ImGui::OpenPopup("ExportMenu");
    }
    if (no_texture) {
      ImGui::EndDisabled();
    }
    if (ImGui::BeginPopup("ExportMenu")) {
      if (ImGui::MenuItem("Screenshot...")) {
        DoExportPreviewPng();
      }
      bool has_server = g_server != nullptr && g_state.sim_state != GuiState::SimState::kIdle;
      if (ImGui::MenuItem("Panorama...", nullptr, false, has_server)) {
        DoExportEquirectPng();
      }
      if (ImGui::MenuItem("Config JSON...")) {
        DoExportConfigJson();
      }
      ImGui::EndPopup();
    }
  }
  if (simulating) {
    ImGui::EndDisabled();
  }
  ImGui::SameLine();
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

  ImGui::End();
}

void RenderLeftPanel(float window_height) {
  if (g_panel_collapsed) {
    return;
  }

  float panel_height = window_height - kTopBarHeight - kStatusBarHeight;
  ImGui::SetNextWindowPos(ImVec2(0, kTopBarHeight));
  ImGui::SetNextWindowSize(ImVec2(kLeftPanelWidth, panel_height));
  ImGui::Begin(
      "##LeftPanel", nullptr,
      ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);

  if (ImGui::BeginTabBar("ConfigTabs")) {
    if (ImGui::BeginTabItem("Crystal")) {
      RenderCrystalTab(g_state);

      // Crystal 3D preview
      if (g_state.selected_crystal >= 0 && g_state.selected_crystal < static_cast<int>(g_state.crystals.size())) {
        ImGui::Separator();
        if (ImGui::CollapsingHeader("3D Preview", ImGuiTreeNodeFlags_DefaultOpen)) {
          auto& cr = g_state.crystals[g_state.selected_crystal];

          // Update mesh if crystal changed
          int hash = CrystalParamHash(cr);
          if (cr.id != g_crystal_mesh_id || hash != g_crystal_mesh_hash) {
            // Build JSON for LUMICE_GetCrystalMesh
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
                       R"("upper_indices":[%d,%d,%d],"lower_indices":[%d,%d,%d],)"
                       R"("face_distance":[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f]}})",
                       cr.prism_h, cr.upper_h, cr.lower_h, cr.upper_indices[0], cr.upper_indices[1],
                       cr.upper_indices[2], cr.lower_indices[0], cr.lower_indices[1], cr.lower_indices[2], fd[0], fd[1],
                       fd[2], fd[3], fd[4], fd[5]);
            }

            LUMICE_CrystalMesh mesh{};
            if (LUMICE_GetCrystalMesh(nullptr, json_buf, &mesh) == LUMICE_OK) {
              // Transform from Core coords (Z-up) to screen coords (Y-up):
              // (x, y, z)_screen = (x, z, -y)_core
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

              // Normalize by AABB longest axis so all crystals display at similar size
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

              g_crystal_renderer.UpdateMesh(mesh.vertices, mesh.vertex_count, mesh.edges, mesh.edge_count,
                                            mesh.triangles, mesh.triangle_count, mesh.edge_face_normals);
              g_crystal_mesh_id = cr.id;
              g_crystal_mesh_hash = hash;
            }
          }

          // Render to FBO
          auto crystal_style = static_cast<CrystalStyle>(g_crystal_style);
          g_crystal_renderer.Render(g_crystal_rotation, g_crystal_zoom, crystal_style);

          // Display FBO texture — square, centered, with matching background fill
          ImVec2 avail = ImGui::GetContentRegionAvail();
          float button_h = ImGui::GetFrameHeightWithSpacing();
          float preview_size = avail.x;
          float area_h = preview_size;

          // Fill the entire available area with the same background as the FBO
          ImVec2 area_start = ImGui::GetCursorScreenPos();
          ImDrawList* draw_list = ImGui::GetWindowDrawList();
          draw_list->AddRectFilled(area_start, ImVec2(area_start.x + avail.x, area_start.y + area_h),
                                   IM_COL32(38, 38, 38, 255));  // Match FBO clear color (0.15)

          // Center the image horizontally
          float offset_x = (avail.x - preview_size) * 0.5f;
          if (offset_x > 0.0f) {
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + offset_x);
          }

          auto tex_id = static_cast<ImTextureID>(g_crystal_renderer.GetTextureId());
          ImVec2 uv0(0, 1);  // Flip Y for OpenGL
          ImVec2 uv1(1, 0);
          ImGui::Image(tex_id, ImVec2(preview_size, preview_size), uv0, uv1);

          // Mouse interaction on the image
          if (ImGui::IsItemHovered()) {
            ImGuiIO& io = ImGui::GetIO();
            if (ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
              ApplyTrackballRotation(io.MouseDelta.x, io.MouseDelta.y);
            }
            if (io.MouseWheel != 0.0f) {
              g_crystal_zoom *= (1.0f - io.MouseWheel * 0.1f);
              g_crystal_zoom = std::max(0.5f, std::min(10.0f, g_crystal_zoom));
            }
          }

          // Advance cursor past the filled area, then draw controls
          ImGui::SetCursorScreenPos(ImVec2(area_start.x, area_start.y + area_h));
          ImGui::PushItemWidth(120.0f);
          ImGui::Combo("##CrystalStyle", &g_crystal_style, kCrystalStyleNames, kCrystalStyleCount);
          ImGui::PopItemWidth();
          ImGui::SameLine();
          if (ImGui::SmallButton("Reset View")) {
            ResetCrystalView();
          }
        }
      }

      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Scene")) {
      RenderSceneTab(g_state);
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Render")) {
      RenderRenderTab(g_state);
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Filter")) {
      RenderFilterTab(g_state);
      ImGui::EndTabItem();
    }
    ImGui::EndTabBar();
  }

  ImGui::End();
}

void RenderFloatingLensBar(float window_width) {
  if (!g_panel_collapsed) {
    return;
  }
  if (g_state.selected_renderer < 0 || g_state.selected_renderer >= static_cast<int>(g_state.renderers.size())) {
    return;
  }

  auto& rc = g_state.renderers[g_state.selected_renderer];

  constexpr float kBarHeight = 36.0f;
  constexpr float kBarPadding = 10.0f;
  float bar_width = std::min(600.0f, window_width - 2 * kBarPadding);
  float bar_x = (window_width - bar_width) * 0.5f;
  float bar_y = kTopBarHeight + kBarPadding;

  ImGui::SetNextWindowPos(ImVec2(bar_x, bar_y));
  ImGui::SetNextWindowSize(ImVec2(bar_width, kBarHeight));
  ImGui::SetNextWindowBgAlpha(0.6f);
  ImGui::Begin("##FloatingLens", nullptr,
               ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                   ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoScrollbar);

  ImGui::PushItemWidth(120.0f);
  ImGui::Combo("##LensType", &rc.lens_type, kLensTypeNames, kLensTypeCount);
  ImGui::SameLine();
  ImGui::PushItemWidth(80.0f);
  float max_fov = MaxFov(static_cast<LensParam::LensType>(rc.lens_type));
  ImGui::SliderFloat("FOV", &rc.fov, 1.0f, max_fov, "%.0f");
  ImGui::PopItemWidth();
  ImGui::SameLine();
  ImGui::Text("El:%.0f Az:%.0f", rc.elevation, rc.azimuth);
  ImGui::PopItemWidth();

  ImGui::End();
}

void RenderPreviewPanel(GLFWwindow* window, float window_width, float window_height) {
  float panel_x = g_panel_collapsed ? 0.0f : kLeftPanelWidth;
  float panel_width = window_width - panel_x;
  float panel_height = window_height - kTopBarHeight - kStatusBarHeight;
  ImGui::SetNextWindowPos(ImVec2(panel_x, kTopBarHeight));
  ImGui::SetNextWindowSize(ImVec2(panel_width, panel_height));
  ImGui::Begin("##PreviewPanel", nullptr,
               ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                   ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoBackground);

  // Aspect ratio bar
  {
    ImGui::Text("Aspect:");
    ImGui::SameLine();
    ImGui::PushItemWidth(150.0f);
    int preset_idx = static_cast<int>(g_state.aspect_preset);
    const char* preview_label = kAspectPresetNames[preset_idx];
    if (ImGui::BeginCombo("##AspectPreset", preview_label)) {
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
    ImGui::PopItemWidth();

    // Portrait/landscape toggle button
    ImGui::SameLine();
    bool disable_flip = (g_state.aspect_preset == AspectPreset::kFree || g_state.aspect_preset == AspectPreset::k1x1);
    ImGui::BeginDisabled(disable_flip);
    const char* flip_label = g_state.aspect_portrait ? "Portrait" : "Landscape";
    if (ImGui::Button(flip_label)) {
      g_state.aspect_portrait = !g_state.aspect_portrait;
      ApplyAspectRatio(window, g_state.aspect_preset, g_state.aspect_portrait);
    }
    ImGui::EndDisabled();
  }

  // Background image controls
  {
    if (ImGui::Button("Load Bg")) {
      DoLoadBackground(window);
    }
    ImGui::SameLine();
    bool no_bg = !g_preview.HasBackground();
    ImGui::BeginDisabled(no_bg);
    ImGui::Checkbox("Show", &g_state.bg_show);
    ImGui::SameLine();
    ImGui::BeginDisabled(!g_state.bg_show);
    ImGui::PushItemWidth(120.0f);
    ImGui::SliderFloat("Alpha", &g_state.bg_alpha, 0.0f, 1.0f, "%.2f");
    ImGui::PopItemWidth();
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("Clear")) {
      DoClearBackground();
    }
    ImGui::EndDisabled();
  }

  float aspect_bar_h = ImGui::GetCursorPosY();
  g_aspect_bar_height = aspect_bar_h;
  float preview_height = panel_height - aspect_bar_h;

  g_preview_vp.active = false;

  if (g_preview.HasTexture() && g_state.selected_renderer >= 0 &&
      g_state.selected_renderer < static_cast<int>(g_state.renderers.size())) {
    // Compute viewport in framebuffer pixels (for HiDPI)
    int fb_w = 0;
    int fb_h = 0;
    glfwGetFramebufferSize(window, &fb_w, &fb_h);
    float scale_x = static_cast<float>(fb_w) / window_width;
    float scale_y = static_cast<float>(fb_h) / window_height;

    auto& rc = g_state.renderers[g_state.selected_renderer];

    // Store viewport for deferred rendering (subtract aspect bar height)
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
    // The texture is ALWAYS dual fisheye EA (file_io.cpp hardcodes dual_fisheye_equal_area),
    // so r_scale is always set regardless of the viewing lens type (Linear, Fisheye, etc.).
    // Keep in sync with render.cpp overlap r_scale computation (same formula, same constant).
    g_preview_vp.params.max_abs_dz = kDualFisheyeOverlap;
    g_preview_vp.params.r_scale = 1.0f / std::sqrt(1.0f + kDualFisheyeOverlap);
    g_preview_vp.params.bg_enabled = g_state.bg_show && g_preview.HasBackground();
    g_preview_vp.params.bg_alpha = g_state.bg_alpha;
    g_preview_vp.params.bg_aspect = g_preview.GetBgAspect();

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
