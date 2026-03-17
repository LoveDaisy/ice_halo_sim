#include "gui/app.hpp"

#include <GLFW/glfw3.h>
#include <spdlog/spdlog.h>
#include <stb_image.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "gui/file_io.hpp"
#include "gui/panels.hpp"
#include "imgui.h"

namespace lumice::gui {

using SimState = GuiState::SimState;

// Global state definitions
GuiState g_state;
PreviewRenderer g_preview;
CrystalRenderer g_crystal_renderer;
LUMICE_Server* g_server = nullptr;
ServerPoller g_server_poller;
bool g_panel_collapsed = false;
PreviewViewport g_preview_vp;

// Crystal preview trackball state
float g_crystal_rotation[16] = {
  1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1,
};
float g_crystal_zoom = 2.5f;
int g_crystal_style = 1;      // Default: Hidden Line (index into kCrystalStyleNames)
int g_crystal_mesh_id = -1;   // Crystal ID of cached mesh
int g_crystal_mesh_hash = 0;  // Hash of crystal params for change detection

int g_programmatic_resize = 0;
float g_aspect_bar_height = 30.0f;  // Updated each frame by RenderPreviewPanel

bool g_show_unsaved_popup = false;
PendingAction g_pending_action = PendingAction::kNone;

void GlfwErrorCallback(int error, const char* description) {
  fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

float GetAspectRatio(AspectPreset preset) {
  switch (preset) {
    case AspectPreset::k16x9:
      return 16.0f / 9.0f;
    case AspectPreset::k3x2:
      return 3.0f / 2.0f;
    case AspectPreset::k4x3:
      return 4.0f / 3.0f;
    case AspectPreset::k1x1:
      return 1.0f;
    case AspectPreset::kFree:
    case AspectPreset::kMatchBg:
    default:
      return 0.0f;
  }
}

void WindowSizeCallback(GLFWwindow* /*window*/, int /*width*/, int /*height*/) {
  if (g_programmatic_resize > 0) {
    g_programmatic_resize--;
    return;
  }
  if (g_state.aspect_preset != AspectPreset::kFree) {
    g_state.aspect_preset = AspectPreset::kFree;
  }
}

void ApplyAspectRatio(GLFWwindow* window, AspectPreset preset, bool portrait, float override_ratio) {
  float ratio = 0.0f;
  if (preset == AspectPreset::kMatchBg) {
    ratio = override_ratio > 0.0f ? override_ratio : g_preview.GetBgAspect();
    if (!g_preview.HasBackground()) {
      return;
    }
  } else {
    ratio = GetAspectRatio(preset);
  }
  if (portrait && ratio > 0.0f) {
    ratio = 1.0f / ratio;
  }
  if (ratio <= 0.0f) {
    return;
  }

  int win_w = 0;
  int win_h = 0;
  glfwGetWindowSize(window, &win_w, &win_h);

  float bar_h = g_aspect_bar_height;
  float preview_w = std::max(1.0f, static_cast<float>(win_w) - kLeftPanelWidth);
  float preview_h = preview_w / ratio;
  auto target_h = static_cast<int>(preview_h + kTopBarHeight + kStatusBarHeight + bar_h);
  int target_w = win_w;

  int work_x = 0;
  int work_y = 0;
  int work_w = 0;
  int work_h = 0;
  glfwGetMonitorWorkarea(glfwGetPrimaryMonitor(), &work_x, &work_y, &work_w, &work_h);

  target_w = std::clamp(target_w, kMinWindowWidth, work_w);
  target_h = std::clamp(target_h, kMinWindowHeight, work_h);

  // If height was clamped, recalculate width to maintain ratio
  float actual_preview_h = static_cast<float>(target_h) - kTopBarHeight - kStatusBarHeight - bar_h;
  if (actual_preview_h > 0.0f) {
    float actual_preview_w = actual_preview_h * ratio;
    int recalc_w = static_cast<int>(actual_preview_w + kLeftPanelWidth);
    if (recalc_w >= kMinWindowWidth && recalc_w <= work_w) {
      target_w = recalc_w;
    }
  }

  g_programmatic_resize = 2;  // Expect up to 2 callbacks (some platforms fire intermediate + final)
  glfwSetWindowSize(window, target_w, target_h);

  // Clamp window position to stay within screen
  int pos_x = 0;
  int pos_y = 0;
  glfwGetWindowPos(window, &pos_x, &pos_y);
  bool moved = false;
  if (pos_x + target_w > work_x + work_w) {
    pos_x = work_x + work_w - target_w;
    moved = true;
  }
  if (pos_y + target_h > work_y + work_h) {
    pos_y = work_y + work_h - target_h;
    moved = true;
  }
  if (pos_x < work_x) {
    pos_x = work_x;
    moved = true;
  }
  if (pos_y < work_y) {
    pos_y = work_y;
    moved = true;
  }
  if (moved) {
    glfwSetWindowPos(window, pos_x, pos_y);
  }
}

int CrystalParamHash(const CrystalConfig& c) {
  // Simple hash to detect parameter changes
  int h = static_cast<int>(c.type);
  auto hash_float = [](float f) {
    int i;
    std::memcpy(&i, &f, sizeof(i));
    return i;
  };
  h ^= hash_float(c.height) * 31;
  h ^= hash_float(c.prism_h) * 37;
  h ^= hash_float(c.upper_h) * 41;
  h ^= hash_float(c.lower_h) * 43;
  for (int i = 0; i < 3; i++) {
    h ^= c.upper_indices[i] * (47 + i);
    h ^= c.lower_indices[i] * (53 + i);
  }
  return h;
}

void ResetCrystalView() {
  // Default slightly elevated view (rotate +20 deg around X = tilt top away)
  constexpr float kAngle = 0.35f;  // ~20 degrees
  float c = std::cos(kAngle);
  float s = std::sin(kAngle);
  // Rotation around X axis
  g_crystal_rotation[0] = 1;
  g_crystal_rotation[1] = 0;
  g_crystal_rotation[2] = 0;
  g_crystal_rotation[3] = 0;
  g_crystal_rotation[4] = 0;
  g_crystal_rotation[5] = c;
  g_crystal_rotation[6] = s;
  g_crystal_rotation[7] = 0;
  g_crystal_rotation[8] = 0;
  g_crystal_rotation[9] = -s;
  g_crystal_rotation[10] = c;
  g_crystal_rotation[11] = 0;
  g_crystal_rotation[12] = 0;
  g_crystal_rotation[13] = 0;
  g_crystal_rotation[14] = 0;
  g_crystal_rotation[15] = 1;
  g_crystal_zoom = 2.5f;
}

// Apply incremental rotation around axis (dx, dy) from mouse drag
void ApplyTrackballRotation(float dx, float dy) {
  float angle = std::sqrt(dx * dx + dy * dy) * 0.01f;
  if (angle < 1e-6f)
    return;

  // Rotation axis perpendicular to drag: "grab and move" model.
  // Drag right (dx>0) → axis +Y (up) → crystal turns right.
  // Drag down (dy>0) → axis +X (right) → crystal top comes toward viewer.
  float ax = dy / (angle / 0.01f);
  float ay = dx / (angle / 0.01f);
  float az = 0.0f;
  float len = std::sqrt(ax * ax + ay * ay + az * az);
  if (len < 1e-6f)
    return;
  ax /= len;
  ay /= len;
  az /= len;

  float ca = std::cos(angle);
  float sa = std::sin(angle);

  // Rodrigues rotation matrix (column-major)
  float r[16] = {};
  r[0] = ca + ax * ax * (1 - ca);
  r[1] = ay * ax * (1 - ca) + az * sa;
  r[2] = az * ax * (1 - ca) - ay * sa;
  r[4] = ax * ay * (1 - ca) - az * sa;
  r[5] = ca + ay * ay * (1 - ca);
  r[6] = az * ay * (1 - ca) + ax * sa;
  r[8] = ax * az * (1 - ca) + ay * sa;
  r[9] = ay * az * (1 - ca) - ax * sa;
  r[10] = ca + az * az * (1 - ca);
  r[15] = 1.0f;

  // new_rotation = r * g_crystal_rotation
  float tmp[16];
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      float sum = 0.0f;
      for (int k = 0; k < 4; k++) {
        sum += r[i + k * 4] * g_crystal_rotation[k + j * 4];
      }
      tmp[i + j * 4] = sum;
    }
  }
  std::memcpy(g_crystal_rotation, tmp, sizeof(g_crystal_rotation));
}

void DoSave() {
  if (g_state.current_file_path.empty()) {
    g_state.current_file_path = ShowSaveDialog();
    if (g_state.current_file_path.empty()) {
      return;
    }
  }
  if (SaveLmcFile(g_state.current_file_path, g_state, g_preview, g_state.save_texture)) {
    g_state.dirty = false;
  }
}

void DoSaveAs() {
  auto path = ShowSaveDialog();
  if (!path.empty()) {
    g_state.current_file_path = path;
    if (SaveLmcFile(path, g_state, g_preview, g_state.save_texture)) {
      g_state.dirty = false;
    }
  }
}

void DoExportPreviewPng() {
  auto path = ShowExportPngDialog();
  if (!path.empty()) {
    ExportPreviewPng(path.c_str(), g_preview, g_preview_vp);
  }
}

// Helper: load image from path, downsample if needed, upload to bg texture.
// Returns true on success.
static bool LoadAndUploadBgImage(const std::string& path) {
  int w = 0;
  int h = 0;
  int channels = 0;
  unsigned char* raw = stbi_load(path.c_str(), &w, &h, &channels, 3);  // Force 3 channels (RGB)
  if (!raw) {
    spdlog::warn("Failed to load background image: {}", path);
    return false;
  }

  // Copy to vector, then free stbi allocation
  size_t byte_count = static_cast<size_t>(w) * h * 3;
  std::vector<unsigned char> data(raw, raw + byte_count);
  stbi_image_free(raw);

  // Box downsample while max dimension > 4096
  while (std::max(w, h) > 4096) {
    int new_w = w / 2;
    int new_h = h / 2;
    for (int y = 0; y < new_h; y++) {
      for (int x = 0; x < new_w; x++) {
        for (int c = 0; c < 3; c++) {
          int sum = data[(y * 2 * w + x * 2) * 3 + c] + data[(y * 2 * w + x * 2 + 1) * 3 + c] +
                    data[((y * 2 + 1) * w + x * 2) * 3 + c] + data[((y * 2 + 1) * w + x * 2 + 1) * 3 + c];
          data[(y * new_w + x) * 3 + c] = static_cast<unsigned char>(sum / 4);
        }
      }
    }
    w = new_w;
    h = new_h;
    data.resize(static_cast<size_t>(w) * h * 3);
  }

  g_preview.UploadBgTexture(data.data(), w, h);
  return true;
}

void DoOpen() {
  auto path = ShowOpenDialog();
  if (!path.empty()) {
    std::vector<unsigned char> tex_data;
    int tex_w = 0;
    int tex_h = 0;
    if (LoadLmcFile(path, g_state, tex_data, tex_w, tex_h)) {
      g_state.current_file_path = path;
      g_state.dirty = false;
      if (!tex_data.empty()) {
        g_preview.UploadTexture(tex_data.data(), tex_w, tex_h);
        g_state.sim_state = SimState::kDone;
      } else {
        g_state.sim_state = SimState::kIdle;
      }

      // Restore background image from saved path (uses deserialized alpha, not reset to 0.5)
      g_preview.ClearBackground();
      if (!g_state.bg_path.empty()) {
        if (LoadAndUploadBgImage(g_state.bg_path)) {
          // bg_show and bg_alpha already restored from deserialization
        } else {
          // Degradation: bg image not found — clear show, reset kMatchBg→kFree
          g_state.bg_show = false;
          if (g_state.aspect_preset == AspectPreset::kMatchBg) {
            g_state.aspect_preset = AspectPreset::kFree;
          }
        }
      }
    }
  }
}

void DoNew() {
  g_state = InitDefaultState();
  g_preview.ClearTexture();
  g_preview.ClearBackground();
  g_crystal_mesh_id = -1;
  g_crystal_mesh_hash = 0;
}

void DoLoadBackground(GLFWwindow* window) {
  auto path = ShowOpenImageDialog();
  if (path.empty()) {
    return;
  }

  if (!LoadAndUploadBgImage(path)) {
    return;
  }

  g_state.bg_path = path;
  g_state.bg_show = true;
  g_state.bg_alpha = 0.5f;  // Start at 50% to immediately show overlay effect
  g_state.aspect_preset = AspectPreset::kMatchBg;
  ApplyAspectRatio(window, AspectPreset::kMatchBg, false, g_preview.GetBgAspect());
}

void DoClearBackground() {
  g_preview.ClearBackground();
  g_state.bg_path.clear();
  g_state.bg_show = false;
  if (g_state.aspect_preset == AspectPreset::kMatchBg) {
    g_state.aspect_preset = AspectPreset::kFree;
  }
}

void DoRun() {
  if (!g_server)
    return;
  auto json_str = SerializeCoreConfig(g_state);
  fprintf(stderr, "[GUI] CommitConfig JSON:\n%s\n", json_str.c_str());
  g_state.last_committed_json = json_str;
  auto err = LUMICE_CommitConfig(g_server, json_str.c_str());
  if (err == LUMICE_OK) {
    g_state.sim_state = SimState::kSimulating;
    g_state.stats_ray_seg_num = 0;
    g_state.stats_sim_ray_num = 0;
    g_server_poller.Start(g_server);
    fprintf(stderr, "[GUI] Simulation started\n");
  } else {
    fprintf(stderr, "[GUI] CommitConfig FAILED with error code %d\n", err);
  }
}

void DoStop() {
  if (!g_server)
    return;
  g_server_poller.Stop();  // Must stop poller before server to avoid dangling access
  LUMICE_StopServer(g_server);
  g_state.sim_state = SimState::kDone;
}

void DoRevert() {
  if (!g_state.last_committed_json.empty()) {
    DeserializeFromJson(g_state.last_committed_json, g_state);
    g_state.sim_state = SimState::kDone;
  }
}

void SyncFromPoller() {
  if (g_state.sim_state != SimState::kSimulating) {
    return;
  }

  PollerData data;
  if (!g_server_poller.TrySyncData(data)) {
    return;  // Poller busy writing — skip this frame
  }
  if (!data.valid) {
    return;  // Worker hasn't produced data yet
  }

  // Update simulation state
  if (data.server_state == LUMICE_SERVER_IDLE) {
    g_state.sim_state = SimState::kDone;
    fprintf(stderr, "[GUI] Simulation done\n");
  }

  // Update stats
  if (data.stats_sim_ray_num > 0) {
    g_state.stats_ray_seg_num = data.stats_ray_seg_num;
    g_state.stats_sim_ray_num = data.stats_sim_ray_num;
  }

  // Upload texture (GL call — must be on main thread)
  if (data.has_new_texture && g_state.selected_renderer >= 0) {
    g_preview.UploadTexture(data.texture_data.data(), data.texture_width, data.texture_height);
    fprintf(stderr, "[GUI] Texture uploaded: %dx%d\n", data.texture_width, data.texture_height);
  }
}

void CheckUnsavedAndDo(PendingAction action) {
  if (g_state.dirty) {
    g_pending_action = action;
    g_show_unsaved_popup = true;
  } else {
    switch (action) {
      case PendingAction::kNew:
        DoNew();
        break;
      case PendingAction::kOpen:
        DoOpen();
        break;
      default:
        break;
    }
  }
}

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
  if (simulating) {
    ImGui::EndDisabled();
  }
  ImGui::SameLine();
  {
    bool no_texture = !g_preview.HasTexture();
    if (no_texture) {
      ImGui::BeginDisabled();
    }
    if (ImGui::Button("Export")) {
      DoExportPreviewPng();
    }
    if (no_texture) {
      ImGui::EndDisabled();
    }
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
            char json_buf[256];
            if (cr.type == CrystalType::kPrism) {
              snprintf(json_buf, sizeof(json_buf), R"({"type":"prism","shape":{"height":%.4f}})", cr.height);
            } else {
              snprintf(json_buf, sizeof(json_buf),
                       R"({"type":"pyramid","shape":{"prism_h":%.4f,"upper_h":%.4f,"lower_h":%.4f}})", cr.prism_h,
                       cr.upper_h, cr.lower_h);
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
          float area_h = std::max(avail.y - button_h, 40.0f);
          float preview_size = std::min(avail.x, area_h);

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
  ImGui::SliderFloat("FOV", &rc.fov, 1.0f, 360.0f, "%.0f");
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
    std::copy(std::begin(rc.ray_color), std::end(rc.ray_color), std::begin(g_preview_vp.params.ray_color));
    std::copy(std::begin(rc.background), std::end(rc.background), std::begin(g_preview_vp.params.background));
    g_preview_vp.params.intensity_factor = std::pow(2.0f, rc.exposure_offset);
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
        rc.fov -= io.MouseWheel * 5.0f;
        rc.fov = std::max(1.0f, std::min(360.0f, rc.fov));
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
    auto pos = g_state.current_file_path.find_last_of("/\\");
    auto filename = (pos != std::string::npos) ? g_state.current_file_path.substr(pos + 1) : g_state.current_file_path;
    if (g_state.dirty) {
      ImGui::Text("%s *", filename.c_str());
    } else {
      ImGui::Text("%s", filename.c_str());
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

}  // namespace lumice::gui
