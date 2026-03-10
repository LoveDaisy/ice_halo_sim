#include <GLFW/glfw3.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

#include "gui/crystal_renderer.hpp"
#include "gui/file_io.hpp"
#include "gui/gui_state.hpp"
#include "gui/panels.hpp"
#include "gui/preview_renderer.hpp"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "include/lumice.h"

namespace {

using SimState = lumice::gui::GuiState::SimState;

lumice::gui::GuiState g_state;
lumice::gui::PreviewRenderer g_preview;
lumice::gui::CrystalRenderer g_crystal_renderer;
LUMICE_Server* g_server = nullptr;
bool g_panel_collapsed = false;

// Crystal preview trackball state
float g_crystal_rotation[16] = {
  1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1,
};
float g_crystal_zoom = 2.5f;
int g_crystal_mesh_id = -1;   // Crystal ID of cached mesh
int g_crystal_mesh_hash = 0;  // Hash of crystal params for change detection

int CrystalParamHash(const lumice::gui::CrystalConfig& c) {
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
  // Default 45-degree top-down view (rotate -30 deg around X)
  constexpr float kAngle = -0.52f;  // ~30 degrees
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

  // Rotation axis is perpendicular to drag direction (in screen space)
  float ax = dy / (angle / 0.01f);
  float ay = -dx / (angle / 0.01f);
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
bool g_show_unsaved_popup = false;
enum class PendingAction { kNone, kNew, kOpen, kQuit };
PendingAction g_pending_action = PendingAction::kNone;
auto g_last_poll_time = std::chrono::steady_clock::now();

constexpr int kInitWindowWidth = 1280;
constexpr int kInitWindowHeight = 720;
constexpr int kMinWindowWidth = 800;
constexpr int kMinWindowHeight = 600;
constexpr float kLeftPanelWidth = 380.0f;
constexpr float kTopBarHeight = 40.0f;
constexpr float kStatusBarHeight = 28.0f;

void GlfwErrorCallback(int error, const char* description) {
  fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

void DoSave() {
  if (g_state.current_file_path.empty()) {
    g_state.current_file_path = lumice::gui::ShowSaveDialog();
    if (g_state.current_file_path.empty()) {
      return;
    }
  }
  auto json_str = lumice::gui::SerializeToJson(g_state);
  if (lumice::gui::WriteStringToFile(g_state.current_file_path, json_str)) {
    g_state.dirty = false;
  }
}

void DoSaveAs() {
  auto path = lumice::gui::ShowSaveDialog();
  if (!path.empty()) {
    g_state.current_file_path = path;
    auto json_str = lumice::gui::SerializeToJson(g_state);
    if (lumice::gui::WriteStringToFile(path, json_str)) {
      g_state.dirty = false;
    }
  }
}

void DoOpen() {
  auto path = lumice::gui::ShowOpenDialog();
  if (!path.empty()) {
    std::string content;
    if (lumice::gui::ReadFileToString(path, content)) {
      if (lumice::gui::DeserializeFromJson(content, g_state)) {
        g_state.current_file_path = path;
        g_state.dirty = false;
      }
    }
  }
}

void DoNew() {
  g_state = lumice::gui::InitDefaultState();
}

void DoRun() {
  if (!g_server)
    return;
  auto json_str = lumice::gui::SerializeToJson(g_state);
  fprintf(stderr, "[GUI] CommitConfig JSON:\n%s\n", json_str.c_str());
  g_state.last_committed_json = json_str;
  auto err = LUMICE_CommitConfig(g_server, json_str.c_str());
  if (err == LUMICE_OK) {
    g_state.sim_state = SimState::kSimulating;
    g_state.stats_ray_seg_num = 0;
    g_state.stats_sim_ray_num = 0;
    fprintf(stderr, "[GUI] Simulation started\n");
  } else {
    fprintf(stderr, "[GUI] CommitConfig FAILED with error code %d\n", err);
  }
}

void DoStop() {
  if (!g_server)
    return;
  LUMICE_StopServer(g_server);
  g_state.sim_state = SimState::kDone;
}

void DoRevert() {
  if (!g_state.last_committed_json.empty()) {
    lumice::gui::DeserializeFromJson(g_state.last_committed_json, g_state);
    g_state.sim_state = SimState::kDone;
  }
}

void FetchRenderResults() {
  if (!g_server || g_state.selected_renderer < 0)
    return;
  LUMICE_RenderResult renders[2]{};
  LUMICE_GetRenderResults(g_server, renders, 1);
  if (renders[0].img_buffer != nullptr) {
    g_preview.UploadTexture(renders[0].img_buffer, renders[0].img_width, renders[0].img_height);
    fprintf(stderr, "[GUI] Texture uploaded: %dx%d\n", renders[0].img_width, renders[0].img_height);
  }
}

void PollServerState() {
  if (!g_server)
    return;
  if (g_state.sim_state != SimState::kSimulating)
    return;

  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - g_last_poll_time).count();
  if (elapsed < 1000)
    return;
  g_last_poll_time = now;

  // Check server state
  LUMICE_ServerState server_state{};
  LUMICE_QueryServerState(g_server, &server_state);
  if (server_state == LUMICE_SERVER_IDLE) {
    g_state.sim_state = SimState::kDone;
    fprintf(stderr, "[GUI] Simulation done\n");
  }

  // Get stats
  LUMICE_StatsResult stats[2]{};
  LUMICE_GetStatsResults(g_server, stats, 1);
  if (stats[0].sim_ray_num > 0) {
    g_state.stats_ray_seg_num = stats[0].ray_seg_num;
    g_state.stats_sim_ray_num = stats[0].sim_ray_num;
  }

  // Get render results and upload texture
  FetchRenderResults();
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
  float button_start_x = window_width - 380.0f;
  if (button_start_x > ImGui::GetCursorPosX()) {
    ImGui::SetCursorPosX(button_start_x);
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
  if (g_state.sim_state == SimState::kSimulating) {
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
      lumice::gui::RenderCrystalTab(g_state);

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
            if (cr.type == lumice::gui::CrystalType::kPrism) {
              snprintf(json_buf, sizeof(json_buf), R"({"type":"prism","shape":{"height":%.4f}})", cr.height);
            } else {
              snprintf(json_buf, sizeof(json_buf),
                       R"({"type":"pyramid","shape":{"prism_h":%.4f,"upper_h":%.4f,"lower_h":%.4f}})", cr.prism_h,
                       cr.upper_h, cr.lower_h);
            }

            LUMICE_CrystalMesh mesh{};
            if (LUMICE_GetCrystalMesh(nullptr, json_buf, &mesh) == LUMICE_OK) {
              g_crystal_renderer.UpdateMesh(mesh.vertices, mesh.vertex_count, mesh.edges, mesh.edge_count);
              g_crystal_mesh_id = cr.id;
              g_crystal_mesh_hash = hash;
            }
          }

          // Render to FBO
          g_crystal_renderer.Render(g_crystal_rotation, g_crystal_zoom);

          // Display FBO texture
          float avail_w = ImGui::GetContentRegionAvail().x;
          float preview_size = std::min(avail_w, 200.0f);
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

          if (ImGui::SmallButton("Reset View")) {
            ResetCrystalView();
          }
        }
      }

      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Scene")) {
      lumice::gui::RenderSceneTab(g_state);
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Render")) {
      lumice::gui::RenderRenderTab(g_state);
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Filter")) {
      lumice::gui::RenderFilterTab(g_state);
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
  ImGui::Combo("##LensType", &rc.lens_type, lumice::gui::kLensTypeNames, lumice::gui::kLensTypeCount);
  ImGui::SameLine();
  ImGui::PushItemWidth(80.0f);
  ImGui::SliderFloat("FOV", &rc.fov, 1.0f, 360.0f, "%.0f");
  ImGui::PopItemWidth();
  ImGui::SameLine();
  ImGui::Text("El:%.0f Az:%.0f", rc.elevation, rc.azimuth);
  ImGui::PopItemWidth();

  ImGui::End();
}

// Stored preview viewport for deferred rendering (after ImGui::Render)
struct PreviewViewport {
  bool active = false;
  int vp_x = 0;
  int vp_y = 0;
  int vp_w = 0;
  int vp_h = 0;
  lumice::gui::PreviewParams params;
};
PreviewViewport g_preview_vp;

void RenderPreviewPanel(GLFWwindow* window, float window_width, float window_height) {
  float panel_x = g_panel_collapsed ? 0.0f : kLeftPanelWidth;
  float panel_width = window_width - panel_x;
  float panel_height = window_height - kTopBarHeight - kStatusBarHeight;
  ImGui::SetNextWindowPos(ImVec2(panel_x, kTopBarHeight));
  ImGui::SetNextWindowSize(ImVec2(panel_width, panel_height));
  ImGui::Begin("##PreviewPanel", nullptr,
               ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                   ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoBackground);

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

    // Store viewport for deferred rendering
    g_preview_vp.active = true;
    g_preview_vp.vp_x = static_cast<int>(panel_x * scale_x);
    g_preview_vp.vp_y = static_cast<int>(kStatusBarHeight * scale_y);  // OpenGL Y is bottom-up
    g_preview_vp.vp_w = static_cast<int>(panel_width * scale_x);
    g_preview_vp.vp_h = static_cast<int>(panel_height * scale_y);
    g_preview_vp.params.lens_type = rc.lens_type;
    g_preview_vp.params.fov = rc.fov;
    g_preview_vp.params.elevation = rc.elevation;
    g_preview_vp.params.azimuth = rc.azimuth;
    g_preview_vp.params.roll = rc.roll;
    g_preview_vp.params.visible = rc.visible;
    std::copy(std::begin(rc.ray_color), std::end(rc.ray_color), std::begin(g_preview_vp.params.ray_color));
    std::copy(std::begin(rc.background), std::end(rc.background), std::begin(g_preview_vp.params.background));
    g_preview_vp.params.intensity_factor = rc.intensity_factor;

    // Mouse interaction: orbit with left drag, FOV with scroll
    ImGuiIO& io = ImGui::GetIO();
    ImVec2 mouse_pos = io.MousePos;
    bool in_preview = mouse_pos.x >= panel_x && mouse_pos.x < panel_x + panel_width && mouse_pos.y >= kTopBarHeight &&
                      mouse_pos.y < kTopBarHeight + panel_height;

    if (in_preview && !ImGui::IsAnyItemActive()) {
      // Left drag: orbit
      if (ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
        ImVec2 delta = io.MouseDelta;
        rc.azimuth -= delta.x * 0.3f;
        rc.elevation += delta.y * 0.3f;
        rc.elevation = std::max(-90.0f, std::min(90.0f, rc.elevation));
      }

      // Scroll: FOV
      if (io.MouseWheel != 0.0f) {
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
    ImGui::Text("| Rays: %lu", g_state.stats_sim_ray_num);
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

}  // namespace


int main(int /*argc*/, char** /*argv*/) {
  glfwSetErrorCallback(GlfwErrorCallback);
  if (!glfwInit()) {
    fprintf(stderr, "Failed to initialize GLFW\n");
    return 1;
  }

  // OpenGL 3.3 Core Profile
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

  GLFWwindow* window = glfwCreateWindow(kInitWindowWidth, kInitWindowHeight, "Lumice", nullptr, nullptr);
  if (!window) {
    fprintf(stderr, "Failed to create GLFW window\n");
    glfwTerminate();
    return 1;
  }

  glfwSetWindowSizeLimits(window, kMinWindowWidth, kMinWindowHeight, GLFW_DONT_CARE, GLFW_DONT_CARE);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);  // VSync

  // imgui setup
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  io.IniFilename = nullptr;  // Disable imgui.ini persistence

  ImGui::StyleColorsDark();

  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init("#version 330");

  g_state = lumice::gui::InitDefaultState();

  // Initialize preview renderer
  if (!g_preview.Init()) {
    fprintf(stderr, "Failed to initialize preview renderer\n");
    return 1;
  }

  // Initialize crystal renderer (256x256 FBO)
  if (!g_crystal_renderer.Init(256, 256)) {
    fprintf(stderr, "Failed to initialize crystal renderer\n");
    return 1;
  }
  ResetCrystalView();

  // Create Lumice server
  g_server = LUMICE_CreateServer();
  LUMICE_InitLogger(g_server);

  // Window close callback: intercept to check for unsaved changes
  glfwSetWindowCloseCallback(window, [](GLFWwindow* w) {
    if (g_state.dirty) {
      glfwSetWindowShouldClose(w, GLFW_FALSE);
      g_pending_action = PendingAction::kQuit;
      g_show_unsaved_popup = true;
    }
  });

  // Main loop
  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();

    // Poll server state periodically
    PollServerState();

    // Keyboard shortcuts
    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_S)) {
      if (io.KeyShift) {
        DoSaveAs();
      } else {
        DoSave();
      }
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Tab) && !io.WantCaptureKeyboard) {
      g_panel_collapsed = !g_panel_collapsed;
    }

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // Framebuffer size for glViewport (may differ from window size on HiDPI)
    int display_w, display_h;
    glfwGetFramebufferSize(window, &display_w, &display_h);

    // Window size for imgui layout (logical pixels)
    int win_w, win_h;
    glfwGetWindowSize(window, &win_w, &win_h);
    float layout_width = static_cast<float>(win_w);
    float layout_height = static_cast<float>(win_h);

    RenderTopBar(layout_width);
    RenderLeftPanel(layout_height);
    RenderPreviewPanel(window, layout_width, layout_height);
    RenderFloatingLensBar(layout_width);
    RenderStatusBar(layout_width, layout_height);
    RenderUnsavedPopup(window);

    // Rendering
    ImGui::Render();
    glViewport(0, 0, display_w, display_h);
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    // Render preview shader before ImGui overlay
    if (g_preview_vp.active) {
      g_preview.Render(g_preview_vp.vp_x, g_preview_vp.vp_y, g_preview_vp.vp_w, g_preview_vp.vp_h, g_preview_vp.params);
    }

    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(window);
  }

  // Cleanup
  g_crystal_renderer.Destroy();
  g_preview.Destroy();
  LUMICE_DestroyServer(g_server);
  g_server = nullptr;

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();

  glfwDestroyWindow(window);
  glfwTerminate();
  return 0;
}
