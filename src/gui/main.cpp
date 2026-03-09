#include <GLFW/glfw3.h>

#include <cstdio>

#include "gui/gui_state.hpp"
#include "gui/panels.hpp"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

namespace {

lumice::gui::GuiState g_state;

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

void RenderTopBar(float window_width) {
  ImGui::SetNextWindowPos(ImVec2(0, 0));
  ImGui::SetNextWindowSize(ImVec2(window_width, kTopBarHeight));
  ImGui::Begin("##TopBar", nullptr,
               ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                   ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoCollapse);

  ImGui::Text("Lumice");
  ImGui::SameLine();

  // Push buttons to the right side
  float button_start_x = window_width - 280.0f;
  if (button_start_x > ImGui::GetCursorPosX()) {
    ImGui::SetCursorPosX(button_start_x);
  }

  ImGui::Button("Open");
  ImGui::SameLine();
  ImGui::Button("Save");
  ImGui::SameLine();
  ImGui::Button("Run");

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

  ImGui::TextColored(ImVec4(0.4f, 0.8f, 0.4f, 1.0f), "●");
  ImGui::SameLine();
  ImGui::Text("Ready");
  ImGui::SameLine();
  ImGui::Text("|");
  ImGui::SameLine();
  ImGui::Text("No file");

  ImGui::End();
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

  // Main loop
  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();

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

    // Rendering
    ImGui::Render();
    glViewport(0, 0, display_w, display_h);
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(window);
  }

  // Cleanup
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();

  glfwDestroyWindow(window);
  glfwTerminate();
  return 0;
}
