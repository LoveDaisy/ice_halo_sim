#include <GLFW/glfw3.h>
#include <spdlog/spdlog.h>

#ifdef _WIN32
#include <timeapi.h>
#include <windows.h>
#endif

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <string>
#include <string_view>
#include <thread>

#include "gui/app.hpp"
#include "gui/color_window.hpp"
#include "gui/defaults_panel.hpp"
#include "gui/dock_layout.hpp"
#include "gui/edit_modals.hpp"
#include "gui/file_io.hpp"
#include "gui/gl_common.h"
#include "gui/gl_init.h"
#include "gui/gui_logger.hpp"
#include "gui/gui_state_reconcile.hpp"
#include "gui/log_sink.hpp"
#include "gui/theme.hpp"
#include "gui/user_defaults.hpp"
#include "gui/window_sizing.hpp"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "util/path_utils.hpp"

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
      if (arg == "-v" || arg == "-d" || arg == "--log-level" || arg == "--core-log-level") {
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

  // Stage 1 of the GUI's log-sink assembly (independent from Core's spdlog; Core logs arrive via
  // the C API callback registered further down next to LUMICE_CreateServer). It runs HERE, before
  // GLFW / GL / ImGui init and after the FreeConsole block above — see gui_logger.hpp, which owns
  // both stages and the reason the ordering is what it is.
  gui::InstallEarlyGuiSinks();

  // Parse --user-config / --no-user-config before the first MakeNewDocumentState() call further
  // down — that call is the only place personal defaults enter a session. Passing neither flag
  // keeps today's behavior (auto-detect the OS per-user config directory).
  //
  // The resulting state is logged unconditionally: "which defaults did this machine actually
  // read" is the first question when a document opens with settings the user did not expect,
  // and it is not answerable from the UI.
  {
    const auto parsed = gui::ParseUserConfigArg(argc, argv);
    if (parsed.missing_value) {
      GUI_LOG_WARNING("[GUI] User config: '--user-config' was given without a directory after it; ignoring that flag");
    }
    const gui::UserConfigSource source =
        gui::ResolveUserConfigSource(parsed.presence, gui::kInteractiveAppUserConfigDefault);
    gui::SetUserConfigSourceForProcess(source, parsed.explicit_dir);
    switch (source) {
      case gui::UserConfigSource::kDisabled:
        GUI_LOG_INFO("[GUI] User config: disabled (--no-user-config); new documents use factory defaults only");
        break;
      case gui::UserConfigSource::kExplicitDir:
        GUI_LOG_INFO("[GUI] User config: explicit directory '{}' (--user-config)",
                     lumice::PathToU8(parsed.explicit_dir));
        break;
      case gui::UserConfigSource::kAutoDetect:
        GUI_LOG_INFO("[GUI] User config: auto-detect (OS per-user config directory)");
        break;
    }
  }

  // Parse --skip-calibration flag early.
  bool skip_calibration = false;
  for (int i = 1; i < argc; ++i) {
    std::string_view arg(argv[i]);
    if (arg == "--skip-calibration") {
      skip_calibration = true;
    }
  }

  glfwSetErrorCallback(gui::GlfwErrorCallback);
  if (!glfwInit()) {
    GUI_LOG_ERROR("Failed to initialize GLFW");
    return 1;
  }

  // OpenGL 3.3 Core Profile
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

  // Clamp the requested initial size to what the current monitor can actually
  // display — workarea already excludes OS bars (menubar/Dock/taskbar); we
  // deduct kWindowDecorationMargin for title bar + borders. Without this,
  // kInitWindowHeight=980 can be silently shrunk by the OS on 1080p displays
  // with a large Dock or 125% DPI scaling, spawning a scrollbar on the right
  // panel. Falls through to defaults on headless / nullptr-monitor environments.
  int init_w = gui::kInitWindowWidth;
  int init_h = gui::kInitWindowHeight;
  if (GLFWmonitor* primary = glfwGetPrimaryMonitor()) {
    int wx = 0;
    int wy = 0;
    int ww = 0;
    int wh = 0;
    glfwGetMonitorWorkarea(primary, &wx, &wy, &ww, &wh);
    if (ww > 0 && wh > 0) {
      auto [w, h] = gui::ClampWindowSizeToWorkarea(init_w, init_h, ww, wh);
      init_w = w;
      init_h = h;
    }
  }
  GLFWwindow* window = glfwCreateWindow(init_w, init_h, "Lumice", nullptr, nullptr);
  if (!window) {
    GUI_LOG_ERROR("Failed to create GLFW window");
    glfwTerminate();
    return 1;
  }

  glfwSetWindowSizeLimits(window, gui::kMinWindowWidth, gui::kMinWindowHeight, GLFW_DONT_CARE, GLFW_DONT_CARE);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);  // VSync on

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
  // Multi-viewport: lets Immediate-mode Edit Entry be dragged outside main window.
  // Staged BeginPopupModal keeps main-viewport constraint by ImGui semantics.
  io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;
  gui::ApplyDockingConfig(io);

  // Layout persistence. ImGui's own default ("imgui.ini") resolves against the process working
  // directory, so the same installed app would remember a different layout depending on where it
  // was launched from; the per-user config directory is the one place this project already treats
  // as "this user's settings". io.IniFilename is not copied by ImGui, so the string has to outlive
  // the context — hence the static.
  //
  // No writable directory (--no-user-config, or an OS that gave us nothing to anchor to) degrades
  // to nullptr, which is exactly the behavior this app had before docking existed: no persistence,
  // no new failure mode.
  static std::string s_layout_ini_path;
  if (const auto config_dir = gui::GetActiveUserConfigDir()) {
    s_layout_ini_path = lumice::PathToU8(*config_dir / "lumice_layout.ini");
    io.IniFilename = s_layout_ini_path.c_str();
    GUI_LOG_INFO("[GUI] Layout persistence: '{}'", s_layout_ini_path);
  } else {
    io.IniFilename = nullptr;
    GUI_LOG_INFO("[GUI] Layout persistence: disabled (no writable user config directory)");
  }

  gui::ApplyVisualLanguage(io);

  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init("#version 330");

  // Personal defaults apply to a NEW document only (invariant I1) — startup shows one.
  gui::g_state = gui::MakeNewDocumentState();
  gui::SurfaceUserDefaultsDowngrades();

  // skip_calibration already parsed above (before glfwSwapInterval).

  // Create Lumice server.
  gui::g_server = LUMICE_CreateServer();

  // Stage 2 of the log-sink assembly: the file sink, which cannot be hoisted into stage 1
  // because constructing it truncates the log file. See gui_logger.hpp.
  gui::AttachGuiFileSink();

  // Bridge Core logs into the GUI ring buffer. Kept here rather than in the sink
  // block at the top of main(): the callback only carries meaning once a server
  // exists to emit Core logs, so it pairs with LUMICE_CreateServer above.
  LUMICE_SetLogCallback([](LUMICE_LogLevel level, const char* /*name*/, const char* message) {
    if (gui::g_imgui_log_sink) {
      auto spd_level = static_cast<spdlog::level::level_enum>(level);
      gui::g_imgui_log_sink->ReceiveExternal(spd_level, message);
    }
  });

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

  // Initialize crystal renderer (512x512 FBO — supersamples the 320px modal preview)
  if (!gui::g_crystal_renderer.Init(512, 512)) {
    GUI_LOG_ERROR("Failed to initialize crystal renderer");
    return 1;
  }
  // Initialize the modal preview's trackball to the default entry's preset
  // default view (matches what Reset View / the entry-card thumbnail show).
  // Without this the user would have to click Reset View on first modal open
  // to reach the same view the outer thumbnail already shows.
  if (!gui::g_state.layers.empty() && !gui::g_state.layers[0].entries.empty()) {
    gui::ResetCrystalViewToCrystal(gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id]);
  } else {
    gui::ResetCrystalView();  // legacy fallback for unexpected empty state
  }

  // Initialize thumbnail cache (must be after GL context is ready)
  if (!gui::g_thumbnail_cache.Init()) {
    GUI_LOG_ERROR("Failed to initialize thumbnail cache");
    return 1;
  }

  // Calibrate quality gate threshold by running a short simulation with default config.
  // Must happen after server creation but before the main loop.
  if (!skip_calibration) {
    gui::CalibrateQualityThreshold();
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
    // DoRun builds a LUMICE_Scene and calls LUMICE_CommitScene (no JSON string roundtrip).
    // Throttled to at most once per kCommitIntervalMs.
    {
      static auto last_commit = std::chrono::steady_clock::now();
      if (gui::g_state.dirty) {
        auto ss = gui::g_state.sim_state;
        // Only auto-commit while actively simulating. The old `|| kDone` clause was vestigial:
        // any dirty edit on a kDone result reconciles to kModified the same frame, so kDone&&dirty
        // is never observed here. Under the single-owner reconcile, keeping it would risk auto-
        // rerunning a completed-then-edited result instead of the intended kModified + Revert UX.
        if (ss == gui::GuiState::SimState::kSimulating) {
          auto now = std::chrono::steady_clock::now();
          auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_commit).count();
          if (elapsed >= gui::kCommitIntervalMs) {
            // task-metal-gui-commit-backpressure: dirty is cleared iff DoRun actually
            // pushed the commit. When the gate defers (Metal first batch not yet landed),
            // DoRun returns false; keep dirty=true so the next 70ms tick retries with
            // g_state's latest edits. `last_commit` always advances so the check cadence
            // is unchanged (still 70ms retry window, plan §4 Step 4).
            //
            // ⚠️ MIRROR: test/gui/test_gui_main.cpp (g_enable_main_loop_commit path) and
            // test/gui/responsiveness/test_gui_perf.cpp (slider_drag scenario) copy this
            // same throttle+accounting block. Any change here MUST be mirrored there — the
            // gated-vs-committed distinction affects restart counting and rays accounting.
            bool committed = gui::DoRun(/*user_initiated=*/false);
            if (committed) {
              gui::g_state.dirty = false;
            }
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
    // Save panel collapse state before any mutations (keyboard shortcuts + button clicks during rendering).
    bool prev_left_collapsed = gui::g_state.left_panel_collapsed;
    bool prev_right_collapsed = gui::g_state.right_panel_collapsed;

    // Panel collapse shortcuts: [ for left panel, ] for right panel
    if (!io.WantCaptureKeyboard) {
      if (ImGui::IsKeyPressed(ImGuiKey_LeftBracket)) {
        gui::g_state.left_panel_collapsed = !gui::g_state.left_panel_collapsed;
      }
      if (ImGui::IsKeyPressed(ImGuiKey_RightBracket)) {
        gui::g_state.right_panel_collapsed = !gui::g_state.right_panel_collapsed;
      }
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
    // The DockSpace host must be submitted before any window that docks into it, so that the nodes
    // exist when those windows call Begin. It spans the band between the top bar and the status bar,
    // which stay fixed-geometry chrome outside the dockspace.
    const float dock_host_height = layout_height - gui::kTopBarHeight - gui::kStatusBarHeight;
    const ImGuiID dockspace_id = gui::RenderDockSpaceHost(0.0f, gui::kTopBarHeight, layout_width, dock_host_height);
    gui::BuildDefaultDockLayout(dockspace_id, layout_width, dock_host_height);
    gui::RenderDocumentTree();
    gui::RenderDocumentInspector();
    gui::RenderRightPanel(window);
    gui::RenderPreviewPanel(window, layout_width, layout_height);
    gui::RenderLogPanel(layout_width, layout_height);
    gui::RenderColorWindow(gui::g_state, gui::g_server);
    gui::RenderStatusBar(layout_width, layout_height);
    gui::RenderSpectrumModal(gui::g_state);
    gui::RenderUnsavedPopup(window);
    gui::RenderSaveModifiedPopup(window);
    gui::RenderDefaultsPanel(gui::g_state);
    gui::RenderImportWarningPopup();
    gui::RenderExportOverwriteConfirmPopup();
    gui::RenderGuiWarningPopup();

    // Reset aspect ratio to Free when panel collapse state changes (window size doesn't adjust automatically).
    if (gui::g_state.left_panel_collapsed != prev_left_collapsed ||
        gui::g_state.right_panel_collapsed != prev_right_collapsed) {
      gui::g_state.aspect_preset = gui::AspectPreset::kFree;
    }

    // Field-tier effect reconcile (scrum-gui-state-reconcile T0, M6). Runs at frame TAIL — after all
    // widget-writing Render*() calls have executed and before ImGui::Render() — so a widget edit in
    // this frame lands in state.dirty this same frame, matching the legacy DIRTY_IF wrappers'
    // synchronous semantics. Placing it at frame top (inside SyncFromPoller) would leave a
    // one-frame delay behind widget writes. Coexists idempotently with legacy DIRTY_IF sites during
    // T1-T4 migration; see gui_state_reconcile.hpp for the field participation set.
    gui::ApplyGuiEffects(gui::g_state, gui::g_server, gui::ReconcileGuiEffects(gui::g_state));

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

    // Update and render additional platform windows (multi-viewport).
    // Must save/restore current GL context because RenderPlatformWindowsDefault
    // makes each viewport's context current in turn; without restore the main
    // window would draw into the wrong context on the next frame.
    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
      GLFWwindow* backup_current_context = glfwGetCurrentContext();
      ImGui::UpdatePlatformWindows();
      ImGui::RenderPlatformWindowsDefault();
      glfwMakeContextCurrent(backup_current_context);
    }

    // gui-polish-v10: Screenshot exports (with or without overlay) now go through the
    // off-screen FBO path in RenderExportToRgba — no deferred default-framebuffer capture
    // is needed here. The old pending_screenshot hook that read back from the default FB
    // was retired because it unavoidably captured ImGui chrome (e.g. an open Save menu)
    // together with the preview (Bug 2).

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
  gui::JoinPendingStop();       // R1: drain any in-flight async Stop before tearing down the server
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
