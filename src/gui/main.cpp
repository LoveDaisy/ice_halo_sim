#include <GLFW/glfw3.h>

#include <chrono>
#include <cstdio>
#include <string>

#include "gui/file_io.hpp"
#include "gui/gui_state.hpp"
#include "gui/panels.hpp"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "include/lumice.h"

namespace {

using SimState = lumice::gui::GuiState::SimState;

lumice::gui::GuiState g_state;
LUMICE_Server* g_server = nullptr;
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
  g_state.last_committed_json = json_str;
  auto err = LUMICE_CommitConfig(g_server, json_str.c_str());
  if (err == LUMICE_OK) {
    g_state.sim_state = SimState::kSimulating;
    g_state.stats_ray_seg_num = 0;
    g_state.stats_sim_ray_num = 0;
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
  }

  // Get stats
  LUMICE_StatsResult stats[2]{};
  LUMICE_GetStatsResults(g_server, stats, 1);
  if (stats[0].sim_ray_num > 0) {
    g_state.stats_ray_seg_num = stats[0].ray_seg_num;
    g_state.stats_sim_ray_num = stats[0].sim_ray_num;
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
  float panel_height = window_height - kTopBarHeight - kStatusBarHeight;
  ImGui::SetNextWindowPos(ImVec2(0, kTopBarHeight));
  ImGui::SetNextWindowSize(ImVec2(kLeftPanelWidth, panel_height));
  ImGui::Begin(
      "##LeftPanel", nullptr,
      ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);

  if (ImGui::BeginTabBar("ConfigTabs")) {
    if (ImGui::BeginTabItem("Crystal")) {
      lumice::gui::RenderCrystalTab(g_state);
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

void RenderPreviewPanel(float window_width, float window_height) {
  float panel_x = kLeftPanelWidth;
  float panel_width = window_width - kLeftPanelWidth;
  float panel_height = window_height - kTopBarHeight - kStatusBarHeight;
  ImGui::SetNextWindowPos(ImVec2(panel_x, kTopBarHeight));
  ImGui::SetNextWindowSize(ImVec2(panel_width, panel_height));
  ImGui::Begin(
      "##PreviewPanel", nullptr,
      ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);

  ImVec2 avail = ImGui::GetContentRegionAvail();
  ImVec2 text_size = ImGui::CalcTextSize("Render Preview");
  ImGui::SetCursorPos(ImVec2((avail.x - text_size.x) * 0.5f, (avail.y - text_size.y) * 0.5f));
  ImGui::TextDisabled("Render Preview");

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
    RenderPreviewPanel(layout_width, layout_height);
    RenderStatusBar(layout_width, layout_height);
    RenderUnsavedPopup(window);

    // Rendering
    ImGui::Render();
    glViewport(0, 0, display_w, display_h);
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(window);
  }

  // Cleanup
  LUMICE_DestroyServer(g_server);
  g_server = nullptr;

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();

  glfwDestroyWindow(window);
  glfwTerminate();
  return 0;
}
