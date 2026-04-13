#include <GLFW/glfw3.h>

// clang-format off
#ifdef _WIN32
#include <windows.h>  // Must precede timeapi.h (provides UINT, DWORD, etc.)
#include <timeapi.h>
#endif
// clang-format on

#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "gui/gl_common.h"

#define IMGUI_DEFINE_MATH_OPERATORS
#include "gui/app.hpp"
#include "gui/edit_modals.hpp"
#include "gui/gl_init.h"
#include "gui/gui_logger.hpp"
#include "gui/log_sink.hpp"
#include "gui/panels.hpp"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "imgui_internal.h"
#include "imgui_te_engine.h"
#include "imgui_te_exporters.h"
#include "test_gui_shared.hpp"

// ========== Global variable definitions ==========

ScreenshotCapture g_capture;
ExportTestState g_export_test;
BgOverlayTestState g_bg_test;
int g_core_log_level = LUMICE_LOG_INFO;
int g_gui_log_level = LUMICE_LOG_INFO;
bool g_enable_visible = false;
bool g_enable_vsync = false;
bool g_enable_frame_limit = true;
bool g_enable_main_loop_commit = false;
bool g_enable_log_panel = false;
int g_dorun_delay_ms = 0;
int g_main_loop_restart_count = 0;
unsigned long g_main_loop_cumulative_rays = 0;

// Synthetic texture for export tests
std::vector<unsigned char> g_synth_tex;

// ========== Shared function definitions ==========

void ResetTestState() {
  // Document state (delegates to DoNew: g_state, g_preview, g_crystal_mesh_id/hash)
  gui::DoNew();

  // UI view state
  gui::ResetCrystalView();
  gui::g_crystal_style = 1;
  gui::g_panel_collapsed = false;
  gui::g_state.right_panel_collapsed = false;
  gui::g_preview_vp.active = false;
  gui::g_programmatic_resize = 0;

  // Runtime state
  gui::g_show_unsaved_popup = false;
  gui::g_pending_action = gui::PendingAction::kNone;
  gui::g_server_poller.Stop();  // Stop poller before nulling server
  gui::g_server = nullptr;
  gui::ResetPendingDeleteState();

  // Modal state (edit_modals.cpp file-scope statics)
  gui::ResetModalState();

  // Test state
  g_capture.Reset();
  g_export_test.Reset();
  g_bg_test.Reset();
}

void InitSynthTexture() {
  if (g_synth_tex.empty()) {
    g_synth_tex.resize(kSynthTexW * kSynthTexH * 3);
    for (int y = 0; y < kSynthTexH; ++y) {
      for (int x = 0; x < kSynthTexW; ++x) {
        int idx = (y * kSynthTexW + x) * 3;
        g_synth_tex[idx + 0] = static_cast<unsigned char>(x * 4);        // R
        g_synth_tex[idx + 1] = static_cast<unsigned char>(y * 4);        // G
        g_synth_tex[idx + 2] = static_cast<unsigned char>((x ^ y) * 4);  // B
      }
    }
  }
}

// ========== Main ==========

int main(int argc, char** argv) {
  // Parse CLI arguments
  const char* test_filter = nullptr;
  int core_log_level = LUMICE_LOG_INFO;  // Default: INFO
  int gui_log_level = LUMICE_LOG_INFO;
  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "--filter") == 0 && i + 1 < argc) {
      test_filter = argv[++i];
    } else if (strcmp(argv[i], "--visible") == 0) {
      g_enable_visible = true;
    } else if (strcmp(argv[i], "--vsync") == 0) {
      g_enable_vsync = true;
      g_enable_visible = true;  // VSync requires visible window
    } else if (strcmp(argv[i], "--frame-limit") == 0) {
      g_enable_frame_limit = true;
    } else if (strcmp(argv[i], "--no-frame-limit") == 0) {
      g_enable_frame_limit = false;
    } else if (strcmp(argv[i], "--main-loop-commit") == 0) {
      g_enable_main_loop_commit = true;
    } else if (strcmp(argv[i], "--log-panel") == 0) {
      g_enable_log_panel = true;
    } else if (strcmp(argv[i], "--dorun-delay") == 0 && i + 1 < argc) {
      g_dorun_delay_ms = atoi(argv[++i]);
    } else if (strcmp(argv[i], "--log-level") == 0 && i + 1 < argc) {
      // Set both core and GUI log level: trace/debug/info/warning/error/off
      const char* level = argv[++i];
      int lvl = LUMICE_LOG_INFO;
      if (strcmp(level, "trace") == 0)
        lvl = LUMICE_LOG_TRACE;
      else if (strcmp(level, "debug") == 0)
        lvl = LUMICE_LOG_DEBUG;
      else if (strcmp(level, "verbose") == 0)
        lvl = LUMICE_LOG_VERBOSE;
      else if (strcmp(level, "info") == 0)
        lvl = LUMICE_LOG_INFO;
      else if (strcmp(level, "warning") == 0)
        lvl = LUMICE_LOG_WARNING;
      else if (strcmp(level, "error") == 0)
        lvl = LUMICE_LOG_ERROR;
      else if (strcmp(level, "off") == 0)
        lvl = LUMICE_LOG_OFF;
      core_log_level = lvl;
      gui_log_level = lvl;
    }
  }
  g_core_log_level = core_log_level;
  g_gui_log_level = gui_log_level;

#ifdef _WIN32
  // Match real app's timer resolution (main.cpp:40).
  // Without this, cv_.wait_for() rounds up to 15.6ms ticks.
  timeBeginPeriod(1);
#endif

  // GLFW init
  glfwSetErrorCallback(gui::GlfwErrorCallback);
  if (!glfwInit()) {
    fprintf(stderr, "Failed to initialize GLFW\n");
    return 1;
  }

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif
  if (!g_enable_visible) {
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);  // Hidden window mode
  }

  GLFWwindow* window =
      glfwCreateWindow(gui::kInitWindowWidth, gui::kInitWindowHeight, "LumiceGUITests", nullptr, nullptr);
  if (!window) {
    fprintf(stderr, "Failed to create GLFW window\n");
    glfwTerminate();
    return 1;
  }

  glfwSetWindowSizeCallback(window, gui::WindowSizeCallback);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(g_enable_vsync ? 1 : 0);
  if (g_enable_visible) {
    fprintf(stderr, "[DIAG] Visible window mode enabled\n");
  }
  if (g_enable_vsync) {
    fprintf(stderr, "[DIAG] VSync mode enabled: swapInterval(1)\n");
  }
  if (g_enable_frame_limit) {
    fprintf(stderr, "[DIAG] Frame limit mode enabled: %dms target frame time (simulates VSync)\n",
            gui::kTargetFrameTimeMs);
  }

  if (!gui::InitGLLoader()) {
    glfwDestroyWindow(window);
    glfwTerminate();
    return 1;
  }

  // ImGui setup
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  io.IniFilename = nullptr;

  ImGui::StyleColorsDark();
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init("#version 330");

  // Initialize GUI state
  gui::g_state = gui::InitDefaultState();

  if (!gui::g_preview.Init()) {
    fprintf(stderr, "Failed to initialize preview renderer\n");
    return 1;
  }
  if (!gui::g_crystal_renderer.Init(256, 256)) {
    fprintf(stderr, "Failed to initialize crystal renderer\n");
    return 1;
  }
  if (!gui::g_thumbnail_cache.Init()) {
    fprintf(stderr, "Failed to initialize thumbnail cache\n");
    return 1;
  }
  gui::ResetCrystalView();

  // Initialize ImGui log sink for --log-panel and set up core log callback.
  // Matches real app's main.cpp log initialization so RenderLogPanel has data to render.
  if (g_enable_log_panel) {
    gui::g_imgui_log_sink = std::make_shared<gui::ImGuiLogSink>();
    LUMICE_SetLogCallback([](LUMICE_LogLevel level, const char* /*name*/, const char* message) {
      if (gui::g_imgui_log_sink) {
        auto spd_level = static_cast<spdlog::level::level_enum>(level);
        gui::g_imgui_log_sink->ReceiveExternal(spd_level, message);
      }
    });
    fprintf(stderr, "[DIAG] Log panel enabled with ImGui sink\n");
  }

  if (g_dorun_delay_ms > 0) {
    fprintf(stderr, "[DIAG] DoRun delay: %dms injected after each DoRun (simulates environment lag)\n",
            g_dorun_delay_ms);
  }

  if (g_enable_main_loop_commit) {
    fprintf(stderr, "[DIAG] Main-loop-commit enabled: DoRun on main thread (matches real app)\n");
  }

  // Setup test engine
  ImGuiTestEngine* engine = ImGuiTestEngine_CreateContext();
  ImGuiTestEngineIO& test_io = ImGuiTestEngine_GetIO(engine);
  test_io.ConfigVerboseLevel = ImGuiTestVerboseLevel_Info;
  test_io.ConfigVerboseLevelOnError = ImGuiTestVerboseLevel_Debug;
  test_io.ConfigRunSpeed = ImGuiTestRunSpeed_Fast;
  test_io.ConfigNoThrottle = true;

  ImGuiTestEngine_Start(engine, ImGui::GetCurrentContext());
  ImGuiTestEngine_InstallDefaultCrashHandler();

  // Register and queue all tests
  RegisterSmokeTests(engine);
  RegisterP0Tests(engine);
  RegisterP1Tests(engine);
  RegisterP2Tests(engine);
  RegisterAspectRatioTests(engine);
  RegisterExportPreviewTests(engine);
  RegisterScreenshotTests(engine);
  RegisterVisualTests(engine);
  RegisterBgOverlayTests(engine);
  RegisterImportExportTests(engine);
  RegisterCalibrationTests(engine);
  RegisterPerfTests(engine);
  // task-test-gui-interaction: user action → state assertion coverage
  RegisterP1InteractionTests(engine);
  RegisterP1SliderBoundaryTests(engine);
  RegisterP2InteractionRenderTests(engine);
  RegisterP1RunningTests(engine);
  RegisterP2InteractionModalTests(engine);
  ImGuiTestEngine_QueueTests(engine, ImGuiTestGroup_Tests, test_filter);

  // Main loop — runs until all tests complete
  while (true) {
    glfwPollEvents();
    gui::SyncFromPoller();  // Sync server data for perf tests (no-op when g_server is null)

    // Auto-commit on main thread (matches real app's main.cpp:214-228).
    // When enabled, DoRun() blocks the main loop just like in the real app,
    // so VSync frame budget effects are faithfully reproduced.
    if (g_enable_main_loop_commit && gui::g_server) {
      static auto last_commit = std::chrono::steady_clock::now();
      if (gui::g_state.dirty) {
        auto ss = gui::g_state.sim_state;
        if (ss == gui::GuiState::SimState::kSimulating || ss == gui::GuiState::SimState::kDone) {
          auto now = std::chrono::steady_clock::now();
          auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_commit).count();
          if (elapsed >= gui::kCommitIntervalMs) {
            // Record rays from current cycle before restart
            LUMICE_StatsResult stats[2]{};
            LUMICE_GetStatsResults(gui::g_server, stats, 1);
            g_main_loop_cumulative_rays += stats[0].sim_ray_num;
            g_main_loop_restart_count++;

            gui::g_state.dirty = false;
            gui::DoRun();
            if (g_dorun_delay_ms > 0) {
              std::this_thread::sleep_for(std::chrono::milliseconds(g_dorun_delay_ms));
            }
            last_commit = now;
          }
        }
      }
    }

    if (glfwWindowShouldClose(window)) {
      if (ImGuiTestEngine_TryAbortEngine(engine)) {
        break;
      }
    }

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // Render GUI panels (needed for test engine to find widgets)
    // Must match src/gui/main.cpp main loop render calls
    int win_w = 0;
    int win_h = 0;
    glfwGetWindowSize(window, &win_w, &win_h);
    auto layout_width = static_cast<float>(win_w);
    auto layout_height = static_cast<float>(win_h);

    gui::RenderTopBar(layout_width);
    gui::RenderLeftPanel(layout_height);
    gui::RenderRightPanel(window, layout_width, layout_height);
    gui::RenderPreviewPanel(window, layout_width, layout_height);
    if (g_enable_log_panel) {
      gui::RenderLogPanel(layout_width, layout_height);
    }
    gui::RenderStatusBar(layout_width, layout_height);
    gui::RenderEditModals(gui::g_state);
    gui::RenderUnsavedPopup(window);

    ImGui::Render();

    int display_w = 0;
    int display_h = 0;
    glfwGetFramebufferSize(window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    // Render preview shader before ImGui overlay (matches real app's main.cpp)
    if (gui::g_preview_vp.active) {
      gui::g_preview.Render(gui::g_preview_vp.vp_x, gui::g_preview_vp.vp_y, gui::g_preview_vp.vp_w,
                            gui::g_preview_vp.vp_h, gui::g_preview_vp.params);
    }

    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(window);

    ImGuiTestEngine_PostSwap(engine);

    // Fallback frame rate limit (same as main.cpp) — simulates VSync timing without needing a real display
    if (g_enable_frame_limit) {
      static auto frame_start = std::chrono::steady_clock::now();
      auto frame_end = std::chrono::steady_clock::now();
      auto frame_ms = std::chrono::duration_cast<std::chrono::milliseconds>(frame_end - frame_start).count();
      if (frame_ms < gui::kTargetFrameTimeMs) {
        std::this_thread::sleep_for(std::chrono::milliseconds(gui::kTargetFrameTimeMs - frame_ms));
      }
      frame_start = std::chrono::steady_clock::now();
    }

    // Exit when all tests are done
    if (!ImGuiTestEngine_IsTestQueueEmpty(engine)) {
      continue;
    }
    if (!test_io.IsRunningTests) {
      break;
    }
  }

  // Get results
  ImGuiTestEngine_PrintResultSummary(engine);
  int count_tested = 0;
  int count_success = 0;
  ImGuiTestEngine_GetResult(engine, count_tested, count_success);
  fprintf(stderr, "[GUI Tests] %d/%d tests passed\n", count_success, count_tested);

  // Cleanup
  ImGuiTestEngine_Stop(engine);

  gui::g_thumbnail_cache.Destroy();
  gui::g_crystal_renderer.Destroy();
  gui::g_preview.Destroy();

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGuiTestEngine_DestroyContext(engine);
  ImGui::DestroyContext();

  glfwDestroyWindow(window);
  glfwTerminate();

#ifdef _WIN32
  timeEndPeriod(1);
#endif
  return (count_tested == count_success) ? 0 : 1;
}
