#include <GLFW/glfw3.h>
#include <spdlog/sinks/dist_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <string_view>

#include "gui/app.hpp"
#include "gui/gl_common.h"
#include "gui/gl_init.h"
#include "gui/log_sink.hpp"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "util/logger.hpp"

namespace gui = lumice::gui;

int main(int argc, char** argv) {
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

  // Create Lumice server and initialize logger BEFORE renderer init,
  // so shader compile/link/FBO errors can use LOG_ERROR instead of fprintf.
  gui::g_server = LUMICE_CreateServer();
  LUMICE_InitLogger(gui::g_server);

  // Set up GUI log sinks: ImGui ring buffer + file sink (default off).
  // Uses dist_sink_mt as the sole sink on each logger for thread-safe fan-out.
  {
    auto dist_sink = std::make_shared<spdlog::sinks::dist_sink_mt>();

    // Keep existing stdout sink
    auto stdout_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    stdout_sink->set_pattern(lumice::kLogPattern);
    dist_sink->add_sink(stdout_sink);

    // ImGui ring buffer sink
    gui::g_imgui_log_sink = std::make_shared<gui::ImGuiLogSink>();
    gui::g_imgui_log_sink->set_pattern(lumice::kLogPattern);
    dist_sink->add_sink(gui::g_imgui_log_sink);

    // File sink (default level=off, enabled via GUI checkbox)
    std::string log_path;
    if (const char* home = std::getenv("HOME")) {
      log_path = std::string(home) + "/lumice.log";
    } else {
      log_path = "lumice.log";
    }
    gui::g_file_log_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(log_path, true);
    gui::g_file_log_sink->set_pattern(lumice::kLogPattern);
    gui::g_file_log_sink->set_level(spdlog::level::off);
    dist_sink->add_sink(gui::g_file_log_sink);

    // Replace sinks on all loggers (single-threaded at this point, safe)
    spdlog::default_logger()->sinks() = { dist_sink };
    lumice::GetGlobalLogger().GetSpdLogger()->sinks() = { dist_sink };
  }

  // Parse CLI arguments for log level.
  // --log-level / -v / -d control GUI log level (global logger, LOG_* macros).
  // --core-log-level controls Core log level (server logger, ILOG_* macros).
  // Default: both warn. -v sets GUI to info, -d sets GUI to debug.
  {
    auto parse_level = [](std::string_view s) -> LUMICE_LogLevel {
      if (s == "trace")
        return LUMICE_LOG_TRACE;
      if (s == "debug")
        return LUMICE_LOG_DEBUG;
      if (s == "info")
        return LUMICE_LOG_INFO;
      if (s == "warn")
        return LUMICE_LOG_WARNING;
      if (s == "error")
        return LUMICE_LOG_ERROR;
      if (s == "off")
        return LUMICE_LOG_OFF;
      return LUMICE_LOG_WARNING;
    };

    LUMICE_LogLevel gui_level = LUMICE_LOG_WARNING;
    LUMICE_LogLevel core_level = LUMICE_LOG_WARNING;
    for (int i = 1; i < argc; ++i) {
      std::string_view arg(argv[i]);
      if (arg == "-v") {
        gui_level = LUMICE_LOG_INFO;
      } else if (arg == "-d") {
        gui_level = LUMICE_LOG_DEBUG;
      } else if (arg == "--log-level" && i + 1 < argc) {
        gui_level = parse_level(argv[++i]);
      } else if (arg == "--core-log-level" && i + 1 < argc) {
        core_level = parse_level(argv[++i]);
      }
    }
    // Set core level first (LUMICE_SetLogLevel sets both server + global logger),
    // then override global logger to GUI level.
    LUMICE_SetLogLevel(gui::g_server, core_level);
    lumice::GetGlobalLogger().SetLevel(static_cast<lumice::LogLevel>(gui_level));
  }

  // Initialize preview renderer
  if (!gui::g_preview.Init()) {
    LOG_ERROR("Failed to initialize preview renderer");
    return 1;
  }

  // Initialize crystal renderer (256x256 FBO)
  if (!gui::g_crystal_renderer.Init(256, 256)) {
    LOG_ERROR("Failed to initialize crystal renderer");
    return 1;
  }
  gui::ResetCrystalView();

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
    // DoRun fills LUMICE_Config struct and calls LUMICE_CommitConfigStruct (no JSON string roundtrip).
    // Throttled to at most once per kCommitIntervalMs.
    {
      static auto last_commit = std::chrono::steady_clock::now();
      if (gui::g_state.dirty) {
        auto ss = gui::g_state.sim_state;
        if (ss == gui::GuiState::SimState::kSimulating || ss == gui::GuiState::SimState::kDone) {
          auto now = std::chrono::steady_clock::now();
          auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_commit).count();
          if (elapsed >= gui::kCommitIntervalMs) {
            gui::g_state.dirty = false;
            gui::DoRun();
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
    gui::RenderLogPanel(layout_width, layout_height);
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
