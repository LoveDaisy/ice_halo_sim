#include <GLFW/glfw3.h>

#include <chrono>
#include <cstdio>

#include "gui/app.hpp"
#include "gui/gl_common.h"
#include "gui/gl_init.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

namespace gui = lumice::gui;

int main(int /*argc*/, char** /*argv*/) {
  glfwSetErrorCallback(gui::GlfwErrorCallback);
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

  GLFWwindow* window = glfwCreateWindow(gui::kInitWindowWidth, gui::kInitWindowHeight, "Lumice", nullptr, nullptr);
  if (!window) {
    fprintf(stderr, "Failed to create GLFW window\n");
    glfwTerminate();
    return 1;
  }

  glfwSetWindowSizeLimits(window, gui::kMinWindowWidth, gui::kMinWindowHeight, GLFW_DONT_CARE, GLFW_DONT_CARE);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);  // VSync

  if (!gui::InitGLLoader()) {
    glfwDestroyWindow(window);
    glfwTerminate();
    return 1;
  }

  // imgui setup
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  io.IniFilename = nullptr;  // Disable imgui.ini persistence

  ImGui::StyleColorsDark();

  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init("#version 330");

  gui::g_state = gui::InitDefaultState();

  // Initialize preview renderer
  if (!gui::g_preview.Init()) {
    fprintf(stderr, "Failed to initialize preview renderer\n");
    return 1;
  }

  // Initialize crystal renderer (256x256 FBO)
  if (!gui::g_crystal_renderer.Init(256, 256)) {
    fprintf(stderr, "Failed to initialize crystal renderer\n");
    return 1;
  }
  gui::ResetCrystalView();

  // Create Lumice server
  gui::g_server = LUMICE_CreateServer();
  LUMICE_InitLogger(gui::g_server);

  // Window size callback: detect user manual resize vs programmatic resize
  glfwSetWindowSizeCallback(window, gui::WindowSizeCallback);

  // Window close callback: intercept to check for unsaved changes
  glfwSetWindowCloseCallback(window, [](GLFWwindow* w) {
    if (gui::g_state.dirty) {
      glfwSetWindowShouldClose(w, GLFW_FALSE);
      gui::g_pending_action = gui::PendingAction::kQuit;
      gui::g_show_unsaved_popup = true;
    }
  });

  // Main loop
  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();

    // Sync data from background server poller (non-blocking)
    gui::SyncFromPoller();

    // Live-edit: auto-commit config when parameters change during simulation.
    // CommitConfig internally routes to hot-update (no Stop/Start) for lightweight
    // changes (sun params, scatter prob), or full restart for structural changes.
    // Throttled to at most once per kCommitIntervalMs (100ms).
    {
      static auto last_commit = std::chrono::steady_clock::now();
      if (gui::g_state.dirty) {
        auto ss = gui::g_state.sim_state;
        if (ss == gui::GuiState::SimState::kSimulating || ss == gui::GuiState::SimState::kDone) {
          auto now = std::chrono::steady_clock::now();
          auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_commit).count();
          if (elapsed >= gui::kCommitIntervalMs) {
            gui::g_state.dirty = false;
            gui::DoRun();  // CommitConfig decides hot-update vs restart internally
            last_commit = now;
          }
        }
      }
    }

    // Keyboard shortcuts
    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_S)) {
      if (io.KeyShift) {
        gui::DoSaveAs();
      } else {
        gui::DoSave();
      }
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Tab) && !io.WantCaptureKeyboard) {
      gui::g_panel_collapsed = !gui::g_panel_collapsed;
    }

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // Framebuffer size for glViewport (may differ from window size on HiDPI)
    int display_w = 0;
    int display_h = 0;
    glfwGetFramebufferSize(window, &display_w, &display_h);

    // Window size for imgui layout (logical pixels)
    int win_w = 0;
    int win_h = 0;
    glfwGetWindowSize(window, &win_w, &win_h);
    auto layout_width = static_cast<float>(win_w);
    auto layout_height = static_cast<float>(win_h);

    gui::RenderTopBar(layout_width);
    gui::RenderLeftPanel(layout_height);
    gui::RenderPreviewPanel(window, layout_width, layout_height);
    gui::RenderFloatingLensBar(layout_width);
    gui::RenderStatusBar(layout_width, layout_height);
    gui::RenderUnsavedPopup(window);

    // Rendering
    ImGui::Render();
    glViewport(0, 0, display_w, display_h);
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    // Render preview shader before ImGui overlay
    if (gui::g_preview_vp.active) {
      gui::g_preview.Render(gui::g_preview_vp.vp_x, gui::g_preview_vp.vp_y, gui::g_preview_vp.vp_w,
                            gui::g_preview_vp.vp_h, gui::g_preview_vp.params);
    }

    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(window);
  }

  // Cleanup
  gui::g_server_poller.Stop();  // Stop poller before destroying server
  gui::g_crystal_renderer.Destroy();
  gui::g_preview.Destroy();
  LUMICE_DestroyServer(gui::g_server);
  gui::g_server = nullptr;

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();

  glfwDestroyWindow(window);
  glfwTerminate();
  return 0;
}
