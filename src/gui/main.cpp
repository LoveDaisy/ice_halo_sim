#include <GLFW/glfw3.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#ifdef _WIN32
#include <windows.h>
#endif

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <string_view>
#include <thread>

#include "gui/app.hpp"
#include "gui/gl_common.h"
#include "gui/gl_init.h"
#include "gui/gui_logger.hpp"
#include "gui/log_sink.hpp"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

namespace gui = lumice::gui;

int main(int argc, char** argv) {
#ifdef _WIN32
  // Reattach stdout/stderr when launched from a console (cmd/PowerShell).
  // WIN32 subsystem detaches the console; this reconnects it for log output.
  if (AttachConsole(ATTACH_PARENT_PROCESS)) {
    freopen("CONOUT$", "w", stdout);
    freopen("CONOUT$", "w", stderr);
  }
#endif

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

  // Create Lumice server and initialize Core logger.
  gui::g_server = LUMICE_CreateServer();
  LUMICE_InitLogger(gui::g_server);

  // Set up GUI-side log sinks (independent from Core's spdlog).
  // GUI logs go through GUI's own spdlog logger; Core logs arrive via C API callback.
  {
    // ImGui ring buffer sink (shared between GUI logger and Core callback)
    gui::g_imgui_log_sink = std::make_shared<gui::ImGuiLogSink>();
    gui::g_imgui_log_sink->set_pattern(gui::kGuiLogPattern);

    // File sink (default level=off, enabled via GUI checkbox)
    std::filesystem::path log_path;
    if (const char* home = std::getenv("HOME")) {
      log_path = std::filesystem::path(home) / "lumice.log";
    } else {
      log_path = "lumice.log";
    }
    log_path = std::filesystem::absolute(log_path);
    gui::g_log_file_path = log_path.u8string();
    gui::g_file_log_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(log_path.string(), true);
    gui::g_file_log_sink->set_pattern(gui::kGuiLogPattern);
    gui::g_file_log_sink->set_level(spdlog::level::off);

    // Set GUI logger sinks: stdout + ImGui + file
    auto stdout_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    stdout_sink->set_pattern(gui::kGuiLogPattern);
    gui::SetGuiLoggerSinks({ stdout_sink, gui::g_imgui_log_sink, gui::g_file_log_sink });

    // Flush strategy: warn+ immediately, all levels every 1s
    gui::GetGuiLogger().flush_on(spdlog::level::warn);
    spdlog::flush_every(std::chrono::seconds(1));

    // Register C API callback to receive Core logs → pipe into ImGui ring buffer
    LUMICE_SetLogCallback([](LUMICE_LogLevel level, const char* /*name*/, const char* message) {
      if (gui::g_imgui_log_sink) {
        auto spd_level = static_cast<spdlog::level::level_enum>(level);
        gui::g_imgui_log_sink->ReceiveExternal(spd_level, message);
      }
    });
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
    // Set core level via C API, GUI level via GUI logger
    LUMICE_SetLogLevel(gui::g_server, core_level);
    gui::SetGuiLogLevel(static_cast<spdlog::level::level_enum>(gui_level));
    // Sync panel dropdowns with CLI-set levels
    gui::g_state.gui_log_level = static_cast<int>(gui_level);
    gui::g_state.core_log_level = static_cast<int>(core_level);
  }

  // Initialize preview renderer
  if (!gui::g_preview.Init()) {
    GUI_LOG_ERROR("Failed to initialize preview renderer");
    return 1;
  }

  // Initialize crystal renderer (256x256 FBO)
  if (!gui::g_crystal_renderer.Init(256, 256)) {
    GUI_LOG_ERROR("Failed to initialize crystal renderer");
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
    auto frame_start = std::chrono::steady_clock::now();
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

    // Fallback frame rate limit: prevents busy-wait when VSync fails
    // (known issue on Windows+NVIDIA, GLFW #1559/#2049).
    // When VSync works, SwapBuffers already blocks ~16ms so this sleep is skipped.
    auto frame_end = std::chrono::steady_clock::now();
    auto frame_ms = std::chrono::duration_cast<std::chrono::milliseconds>(frame_end - frame_start).count();
    if (frame_ms < gui::kTargetFrameTimeMs) {
      std::this_thread::sleep_for(std::chrono::milliseconds(gui::kTargetFrameTimeMs - frame_ms));
    }
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
