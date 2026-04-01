#include <GLFW/glfw3.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#ifdef _WIN32
#include <timeapi.h>
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
  // Console subsystem (IMAGE_SUBSYSTEM_WINDOWS_CUI) gives longer thread time slices
  // than GUI subsystem, critical for the 18+ Simulator compute threads (~3.4x throughput
  // difference). For normal GUI launch, release the console so no window is visible.
  // Keep it for diagnostic modes that need stdout/stderr output.
  {
    bool keep_console = false;
    for (int i = 1; i < argc; ++i) {
      std::string_view arg(argv[i]);
      if (arg == "--perf-bench" || arg == "-v" || arg == "-d" || arg == "--log-level" || arg == "--core-log-level") {
        keep_console = true;
        break;
      }
    }
    if (!keep_console) {
      FreeConsole();
    }
  }

  // Raise timer resolution from 15.6ms to ~1ms so that cv_.wait_for() and Sleep()
  // are precise enough for our 20ms poll interval. Without this, SleepConditionVariableSRW
  // rounds up to 3 timer ticks (~47ms), causing a timing race with the 50ms commit interval.
  timeBeginPeriod(1);
#endif

  // Parse perf-bench flag early — needed before glfwSwapInterval.
  bool perf_bench = false;
  bool skip_calibration = false;
  for (int i = 1; i < argc; ++i) {
    std::string_view arg(argv[i]);
    if (arg == "--perf-bench") {
      perf_bench = true;
      skip_calibration = true;
    } else if (arg == "--skip-calibration") {
      skip_calibration = true;
    }
  }

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
  glfwSwapInterval(perf_bench ? 0 : 1);  // VSync off for perf-bench to match test binary

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

  // perf_bench and skip_calibration already parsed above (before glfwSwapInterval).

  // Create Lumice server and initialize Core logger.
  gui::g_server = LUMICE_CreateServer();
  LUMICE_InitLogger(gui::g_server);

  // Set up GUI-side log sinks (independent from Core's spdlog).
  // GUI logs go through GUI's own spdlog logger; Core logs arrive via C API callback.
  {
    // ImGui ring buffer sink (shared between GUI logger and Core callback)
    gui::g_imgui_log_sink = std::make_shared<gui::ImGuiLogSink>();

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
    gui::g_file_log_sink->set_level(spdlog::level::off);

    // Set GUI logger sinks: stdout + ImGui + file, then apply our custom formatter to all.
    auto stdout_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    gui::SetGuiLoggerSinks({ stdout_sink, gui::g_imgui_log_sink, gui::g_file_log_sink });
    gui::GetGuiLogger().set_formatter(lumice::CreateLumiceFormatter(gui::kGuiLogPattern));

    // Flush strategy: warning+ immediately, all levels every 1s
    // spdlog::err = our warning level (see spdlog_levels.hpp)
    gui::GetGuiLogger().flush_on(spdlog::level::err);
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
      if (s == "verbose")
        return LUMICE_LOG_VERBOSE;
      if (s == "info")
        return LUMICE_LOG_INFO;
      if (s == "warn" || s == "warning")
        return LUMICE_LOG_WARNING;
      if (s == "error")
        return LUMICE_LOG_ERROR;
      if (s == "off")
        return LUMICE_LOG_OFF;
      return LUMICE_LOG_WARNING;
    };

    LUMICE_LogLevel gui_level = LUMICE_LOG_INFO;
    LUMICE_LogLevel core_level = LUMICE_LOG_WARNING;
    for (int i = 1; i < argc; ++i) {
      std::string_view arg(argv[i]);
      if (arg == "-v") {
        gui_level = LUMICE_LOG_VERBOSE;
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

  // Calibrate quality gate threshold by running a short simulation with default config.
  // Must happen after server creation but before the main loop.
  if (!skip_calibration) {
    gui::CalibrateQualityThreshold();
  }

  // --perf-bench: use EXACTLY the same startup path as test_gui_main.cpp's StartPerfSimulation,
  // measure steady-state throughput for 2 seconds, print result, and exit.
  // This allows apples-to-apples comparison with LumiceGUITests --filter perf_test.
  if (perf_bench) {
    // Same config as StartPerfSimulation in test_gui_main.cpp
    gui::g_state.sun.altitude = 20.0f;
    gui::g_state.sun.azimuth = 0.0f;
    gui::g_state.sun.diameter = 0.5f;
    gui::g_state.sun.spectrum_index = 2;
    gui::g_state.sim.infinite = true;
    gui::g_state.sim.max_hits = 8;
    if (!gui::g_state.renderers.empty()) {
      auto& r = gui::g_state.renderers[0];
      r.lens_type = 1;
      r.fov = 360.0f;
      r.sim_resolution_index = 0;  // 512
      r.visible = 2;
      r.background[0] = r.background[1] = r.background[2] = 0.0f;
      r.exposure_offset = 0.0f;
    }
    gui::DoRun();

    // Wait for first data
    auto timeout = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (gui::g_state.stats_sim_ray_num == 0 && std::chrono::steady_clock::now() < timeout) {
      glfwPollEvents();
      gui::SyncFromPoller();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Measure for 2 seconds
    auto start_rays = gui::g_state.stats_sim_ray_num;
    auto start_time = std::chrono::steady_clock::now();
    auto end_time = start_time + std::chrono::seconds(2);
    int frames = 0;
    while (std::chrono::steady_clock::now() < end_time) {
      glfwPollEvents();
      gui::SyncFromPoller();

      ImGui_ImplOpenGL3_NewFrame();
      ImGui_ImplGlfw_NewFrame();
      ImGui::NewFrame();

      int win_w = 0, win_h = 0;
      glfwGetWindowSize(window, &win_w, &win_h);
      auto lw = static_cast<float>(win_w);
      auto lh = static_cast<float>(win_h);
      gui::RenderTopBar(lw);
      gui::RenderLeftPanel(lh);
      gui::RenderPreviewPanel(window, lw, lh);
      gui::RenderFloatingLensBar(lw);
      gui::RenderStatusBar(lw, lh);

      ImGui::Render();
      int dw = 0, dh = 0;
      glfwGetFramebufferSize(window, &dw, &dh);
      glViewport(0, 0, dw, dh);
      glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
      glClear(GL_COLOR_BUFFER_BIT);
      if (gui::g_preview_vp.active) {
        gui::g_preview.Render(gui::g_preview_vp.vp_x, gui::g_preview_vp.vp_y, gui::g_preview_vp.vp_w,
                              gui::g_preview_vp.vp_h, gui::g_preview_vp.params);
      }
      ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
      glfwSwapBuffers(window);
      frames++;
    }

    auto end_rays = gui::g_state.stats_sim_ray_num;
    double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
    double rps = elapsed > 0 ? static_cast<double>(end_rays - start_rays) / elapsed : 0;
    fprintf(stderr, "[PERF-BENCH] steady_state: %.1f rays/sec (%lu rays in %.1fs, %d frames, %.1f FPS)\n", rps,
            end_rays - start_rays, elapsed, frames, frames / elapsed);

    // Cleanup and exit
    gui::g_server_poller.Stop();
    gui::g_crystal_renderer.Destroy();
    gui::g_preview.Destroy();
    LUMICE_DestroyServer(gui::g_server);
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
#ifdef _WIN32
    timeEndPeriod(1);
#endif
    return 0;
  }

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

#ifdef _WIN32
  timeEndPeriod(1);
#endif
  return 0;
}
