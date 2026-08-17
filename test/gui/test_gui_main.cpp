#include <GLFW/glfw3.h>

// clang-format off
#ifdef _WIN32
#include <windows.h>  // Must precede timeapi.h (provides UINT, DWORD, etc.)
#include <timeapi.h>
#else
#include <unistd.h>  // getpid, for the per-process scratch directory
#endif
// clang-format on

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

#include "gui/gl_common.h"

#define IMGUI_DEFINE_MATH_OPERATORS
#include "gui/app.hpp"
#include "gui/color_window.hpp"
#include "gui/defaults_panel.hpp"
#include "gui/dock_layout.hpp"
#include "gui/edit_modals.hpp"
#include "gui/gl_capture.hpp"
#include "gui/gl_init.h"
#include "gui/gui_constants.hpp"
#include "gui/gui_logger.hpp"
#include "gui/gui_state_reconcile.hpp"
#include "gui/log_sink.hpp"
#include "gui/panels.hpp"
#include "gui/theme.hpp"
#include "gui/user_defaults.hpp"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "imgui_internal.h"
#include "imgui_te_engine.h"
#include "imgui_te_exporters.h"
#include "support/scoped_result_frame.hpp"
#include "test_gui_shared.hpp"

// ========== Global variable definitions ==========

ScreenshotCapture g_capture;
ExportTestState g_export_test;
BgOverlayTestState g_bg_test;
LeftPanelCaptureState g_left_panel_capture;
FullFrameCaptureState g_fullframe_capture;
AutoEvExportState g_auto_ev_export;
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
// Set by --keep-export-png; used by scripts/regen_gui_test_refs.py to collect per-run PNGs.
bool g_keep_export_png = false;
// Set by --export-dir <path>: where GuiTestTempPath() puts scratch files. See that function.
const char* g_export_dir = nullptr;
// Set once main() has finished reading argv. GuiTestTempPath() latches its directory on first
// call and must not do that before --export-dir has been seen; this makes that ordering a
// checked precondition instead of a comment (see the assert there).
bool g_args_parsed = false;
// Set by --fixed-dt: inject a deterministic per-frame dt (1/60s) and skip the
// frame-limit sleep. Decouples VSync frame-budget semantics from wall-clock cost
// so correctness tests run at full speed. See scratchpad/task-gui-test-fixed-dt.
bool g_enable_fixed_dt = false;
// Set by --export-junit <path>: emit per-test results as JUnit XML (general
// CI/regression-diff capability, not throwaway scaffolding).
const char* g_export_junit_path = nullptr;

// Synthetic texture for export tests
std::vector<unsigned char> g_synth_tex;

// ========== Shared function definitions ==========

namespace {

// Walk `window_ref` from the top in half-window steps, stopping at the first offset where
// `item_ref` resolves, and LEAVE the scroll there. Returns the info, whose ID is 0 if it never
// resolved.
//
// Half-window steps rather than a jump to the bottom because the target may be anywhere and only
// the band on screen is findable; each band has to be visited.
ImGuiTestItemInfo ScrollUntilFound(ImGuiTestContext* ctx, const char* window_ref, const char* item_ref) {
  ImGuiTestItemInfo info = ctx->ItemInfo(item_ref, ImGuiTestOpFlags_NoError);
  if (info.ID != 0) {
    return info;
  }
  // WindowInfo rather than GetWindowByRef: a CHILD window's ImGuiID is the hash of the mangled name
  // ImGui builds for it ("Parent/id_XXXXXXXX"), not the hash of the path a caller writes, so the
  // straightforward lookup silently returns null for exactly the windows that scroll here.
  ImGuiWindow* scroller = ctx->WindowInfo(window_ref, ImGuiTestOpFlags_NoError).Window;
  if (scroller == nullptr || scroller->ScrollMax.y <= 0.0f) {
    return info;
  }
  const float step = ImMax(scroller->Size.y * 0.5f, 1.0f);
  for (float y = 0.0f; y <= scroller->ScrollMax.y + step; y += step) {
    scroller->Scroll.y = ImMin(y, scroller->ScrollMax.y);
    ctx->Yield(2);
    info = ctx->ItemInfo(item_ref, ImGuiTestOpFlags_NoError);
    if (info.ID != 0) {
      break;
    }
    scroller = ctx->WindowInfo(window_ref, ImGuiTestOpFlags_NoError).Window;
    if (scroller == nullptr) {
      break;
    }
  }
  return info;
}

}  // namespace

// See the declaration in test_gui_shared.hpp for why these exist.
ImGuiTestItemInfo InspectorItemInfo(ImGuiTestContext* ctx, const char* ref) {
  ImGuiWindow* page = ctx->GetWindowByRef(gui::kDocumentInspectorWindowName);
  const float restore = page != nullptr ? page->Scroll.y : 0.0f;
  const ImGuiTestItemInfo info = ScrollUntilFound(ctx, gui::kDocumentInspectorWindowName, ref);
  // Restored so a read-only lookup cannot decide what the next assertion in the same case sees.
  page = ctx->GetWindowByRef(gui::kDocumentInspectorWindowName);
  if (page != nullptr && page->Scroll.y != restore) {
    page->Scroll.y = restore;
    ctx->Yield(2);
  }
  return info;
}

bool InspectorItemExists(ImGuiTestContext* ctx, const char* ref) {
  return InspectorItemInfo(ctx, ref).ID != 0;
}

bool ScrollTreeTo(ImGuiTestContext* ctx, const char* ref) {
  return ScrollUntilFound(ctx, kTreeScrollRef, ref).ID != 0;
}

// See the declaration in test_gui_shared.hpp for why this exists and who consumes it.
std::filesystem::path GuiTestTempPath(const std::string& filename) {
  // The directory is latched on the FIRST call and reused for the rest of the process, so a call
  // that precedes argv parsing would pin the fallback path for everything afterwards — silently,
  // writing to the wrong place with nothing to indicate it. Every call today comes from a test
  // body, which runs long after main() parses; this check is what keeps that true rather than
  // merely currently-so.
  //
  // Deliberately NOT assert(): this binary is built Release (CMAKE_CXX_FLAGS_RELEASE carries
  // -DNDEBUG), where assert compiles to nothing — the check would be absent from the only
  // configuration anyone builds gui_test in, which is worse than no check because it reads like
  // one. Failing loudly costs a branch on a path that runs a handful of times per test.
  if (!g_args_parsed) {
    fprintf(stderr,
            "[FATAL] GuiTestTempPath() called before argv was parsed — the scratch directory\n"
            "        would latch to the fallback path and --export-dir would be ignored.\n");
    std::abort();
  }

  static const std::filesystem::path dir = [] {
    if (g_export_dir != nullptr) {
      return std::filesystem::path(g_export_dir);
    }
    // Per-process, so two concurrent gui_test processes cannot collide on a fixed filename.
    // Computed once: a second call must land in the same place the first one wrote to.
    const auto pid = static_cast<unsigned long long>(
#ifdef _WIN32
        GetCurrentProcessId()
#else
        getpid()
#endif
    );
    return std::filesystem::temp_directory_path() / ("lumice_gui_test_" + std::to_string(pid));
  }();

  // Created here rather than at startup so a run that writes nothing leaves nothing behind.
  // Errors are swallowed deliberately: the caller's own open/write is the real check, and it
  // reports a far more useful failure than a directory-creation error would.
  std::error_code ec;
  std::filesystem::create_directories(dir, ec);
  return dir / filename;
}

void ResetTestState() {
  // Document state (delegates to DoNew: g_state, g_preview, g_crystal_mesh_id/hash)
  gui::DoNew();

  // UI view state
  gui::ResetCrystalView();
  gui::g_crystal_style = 1;
  gui::g_state.left_panel_collapsed = false;
  gui::g_state.right_panel_collapsed = false;
  // NOTE: panel geometry is a dock layout now, and the dock layout outlives g_state — a case that
  // moves a splitter hands the next case a different panel width unless it puts it back. This reset
  // deliberately does NOT rebuild the layout to enforce that: DockBuilderRemoveNode undocks and
  // re-docks the panels, and for the frame after that their child windows are not submitted, so
  // every case paying two frames here would turn "the card is there" into a race. The one case that
  // moves a splitter restores it by dragging back — see shell_chrome's splitter case.
  //
  // A window's SCROLL outlives g_state the same way, and unlike the splitter it is left behind by
  // ordinary cases rather than by one deliberate one: ItemInputValue scrolls whatever window the
  // field is in, so any case that types into the inspector hands the next case a page scrolled to
  // the bottom. That is not cosmetic — ImGui culls table rows outside the clip rect BEFORE
  // submitting them, so a scrolled-away row does not merely look absent, it IS absent, and the next
  // case's ItemInfo cannot find it. The inspector's own product rule (scroll to top when the page
  // swaps) does not cover this: DoNew leaves the selection where the next case puts it, so from the
  // window's point of view the page never changed.
  if (ImGuiWindow* inspector = ImGui::FindWindowByName(gui::kDocumentInspectorWindowName)) {
    inspector->Scroll.y = 0.0f;
    inspector->ScrollTarget.y = 0.0f;
  }
  gui::g_preview_vp.active = false;
  gui::g_programmatic_resize = 0;

  // Runtime state
  gui::g_show_unsaved_popup = false;
  gui::g_pending_action = gui::PendingAction::kNone;
  gui::g_show_save_modified_popup = false;
  gui::g_pending_save_kind = gui::PendingSaveKind::kNone;
  // The warning modal's in-flight message is a file-scope string in app_panels.cpp with its own
  // de-duplication key, so a message left in flight by one case decides whether the next case's
  // identical warning opens at all. Now that the harness renders the modal, that would also leave
  // a modal open across cases.
  gui::ClearGuiWarning();
  // Same shape, the import-warning sibling: a queued message left behind by one case would open
  // its modal on top of whatever the next case is driving. Several suites import configurations
  // that queue one as a side effect, which is why this is a reset rather than each case's job.
  gui::ClearImportComplexFilterWarning();
  // Same shape again, for the export-overwrite prompt: a pending export left unanswered by one
  // case would raise its modal over the next one, and — worse than a stray warning — the next
  // case's stray click on "Overwrite" would write a file.
  gui::g_show_export_overwrite_confirm_popup = false;
  gui::CancelPendingConfigJsonExport();
  gui::g_server_poller.Stop();  // Stop poller before nulling server
  // task-349.4: Stop() only kPaused the worker — the last published PreviewSnapshot
  // survives (production keeps it on purpose for slider-scrub carry-forward, see
  // server_poller.hpp:113). But `g_state = InitDefaultState()` above just zeroed
  // last_uploaded_texture_serial / display_epoch_floor / committed_epoch, so if a
  // prior test's snapshot is still valid with a non-zero texture_serial, the very
  // next SyncFromPoller() would re-upload it and any test that expects
  // g_preview.HasTexture() to reflect this test's own CPU-side ClearTexture()
  // observes the leaked prior texture instead. Use the dedicated test seam to drop
  // the payload (valid stays true → ReconcileSimState still runs correctly).
  gui::g_server_poller.InvalidateStagedTexture();
  gui::g_server = nullptr;
  gui::ResetPendingDeleteState();
  // task-cleanup-hardening S5: the RefreshColorClassSignals cache is a static
  // WindowLocalState inside color_window.cpp. Its (server, epoch) invalidation
  // keys survive across tests via a leaked stale server pointer or committed_epoch
  // if not reset here, which can mask AC2 behavior (either false-positive
  // "invalidated" because the prior test's server is now dangling, or false-
  // negative "hit throttle" because the prior test's keys happen to match).
  // Deterministic per-test reset restores the same-invariant as g_state.
  gui::ResetColorClassSignalCacheForTest();

  // Modal state (edit_modals.cpp / defaults_panel.cpp file-scope statics)
  gui::ResetModalState();
  gui::ResetDefaultsPanelTestState();

  // Test state
  g_capture.Reset();
  g_export_test.Reset();
  g_bg_test.Reset();
}

// See the contract note in test_gui_shared.hpp. Lives here because this is the TU that already
// includes imgui_internal.h, where ImGuiItemFlags_Disabled is declared.
bool IsDisabled(const ImGuiTestItemInfo& info) {
  return (info.ItemFlags & ImGuiItemFlags_Disabled) != 0;
}

// See the contract note in test_gui_shared.hpp.
void OpenEntryTab(ImGuiTestContext* ctx, int layer_idx, int entry_idx, const char* tab_ref) {
  gui::g_state.SelectCrystal(layer_idx, entry_idx);
  ctx->Yield(3);
  ctx->ItemClick(tab_ref);
  ctx->Yield(2);
}

void OpenCrystalTab(ImGuiTestContext* ctx, int layer_idx, int entry_idx) {
  OpenEntryTab(ctx, layer_idx, entry_idx, "**/###crystal_tab");
}

void OpenAxisTab(ImGuiTestContext* ctx, int layer_idx, int entry_idx) {
  OpenEntryTab(ctx, layer_idx, entry_idx, "**/###axis_tab");
}

void OpenFilterTab(ImGuiTestContext* ctx, int layer_idx, int entry_idx) {
  OpenEntryTab(ctx, layer_idx, entry_idx, "**/###filter_tab");
}

// See the contract note in test_gui_shared.hpp for why this is a destructor rather than three
// statements at the end of a case body.
ScopedPopups::~ScopedPopups() {
  // The gui side first, so the popup stops being submitted:
  // RenderSpectrumModal only call BeginPopupModal while these statics say one is open. One call
  // covers both — ResetModalState delegates to ResetSpectrumModalStateGlobals. The sun-circles
  // editor has no statics of its own: it is opened straight from its button and needs only the
  // stack pop below.
  gui::ResetModalState();
  // The two view preferences a case may have flipped to reach a layout, pinned back to the values
  // ResetTestState() hands every case. Listed so this object is a complete statement of what it
  // borrowed, not a list of what ResetTestState() happens to miss today.
  gui::g_state.modal_layout_vertical = false;
  gui::g_state.modal_immediate_mode = false;
  // ImGui's own stack is the part nothing else reaches. Level 0 rather than a search for our entry:
  // by the time a case is leaving, every popup on the stack was opened by it (a sync-group popup
  // nested inside the modal is the common second one), and restore_focus=false keeps this to a
  // resize of the stack — the focus path is the only thing in ClosePopupToLevel that would care
  // that we are running between frames rather than inside one.
  ImGuiContext& g = *ImGui::GetCurrentContext();
  if (g.OpenPopupStack.Size > 0) {
    ImGui::ClosePopupToLevel(0, /*restore_focus_to_window_under_popup=*/false);
  }
  // One frame for the window to go away with it. Yield is the only ImGuiTestContext call that still
  // does anything once the case has failed.
  ctx_->Yield(2);
}

// See the contract note in test_gui_shared.hpp. Unlike its neighbour below, this one needs nothing
// this TU owns — it lives here only because a shared helper needs one definition and this is where
// the suite's shared definitions are. Moving it costs nothing but a build-file line.
bool WaitForSimRestartAtLeast(ImGuiTestContext* ctx, unsigned long long baseline_upload_count, int timeout_ms) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    ctx->Yield();  // let the main thread run SyncFromPoller()
    if (gui::g_state.texture_upload_count >= baseline_upload_count + 1) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

// See the contract note in test_gui_shared.hpp. Lives here because this TU owns
// g_auto_ev_export and the main-loop hook that services it.
bool RequestAndWaitPreviewExport(ImGuiTestContext* ctx, const gui::PreviewViewport& vp, const std::string& path) {
  g_auto_ev_export.export_path = path;
  g_auto_ev_export.custom_vp = vp;
  g_auto_ev_export.done.store(false);
  g_auto_ev_export.requested.store(true);
  for (int i = 0; i < 10 && !g_auto_ev_export.done.load(); ++i) {
    ctx->Yield(1);
  }
  return g_auto_ev_export.done.load() && g_auto_ev_export.result;
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
    } else if (strcmp(argv[i], "--keep-export-png") == 0) {
      // Suppress std::remove in CheckAgainstReference so regen_gui_test_refs.py can collect PNGs.
      g_keep_export_png = true;
    } else if (strcmp(argv[i], "--export-dir") == 0 && i + 1 < argc) {
      // Pin where GuiTestTempPath() writes, so a collector does not have to guess it.
      g_export_dir = argv[++i];
    } else if (strcmp(argv[i], "--fixed-dt") == 0) {
      // Inject deterministic 16.67ms per-frame dt via the test engine and skip
      // the frame-limit sleep (decouples VSync frame-budget semantics from
      // wall-clock cost). Used by the build.sh correctness pool.
      g_enable_fixed_dt = true;
      g_enable_frame_limit = false;
    } else if (strcmp(argv[i], "--export-junit") == 0 && i + 1 < argc) {
      // Emit per-test results as JUnit XML for per-case pass/fail diffing.
      g_export_junit_path = argv[++i];
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
  // argv is fully read; GuiTestTempPath() may now latch its directory.
  g_args_parsed = true;

  // Personal defaults: this binary's no-flag default is DISABLED (kTestHarnessUserConfigDefault),
  // unlike LumiceGUI's auto-detect. Every scenario reaches MakeNewDocumentState() through
  // ResetTestState() -> gui::DoNew(), so without this a reference-image comparison would depend on
  // whichever user_defaults.json the developer running it happens to have saved — green here, red
  // on the next machine, and only ever as a slightly-low PSNR. --user-config stays available for a
  // test that deliberately wants a controlled override directory.
  {
    const auto parsed = gui::ParseUserConfigArg(argc, argv);
    if (parsed.missing_value) {
      fprintf(stderr, "[DIAG] User config: '--user-config' given without a directory after it; ignoring that flag\n");
    }
    const gui::UserConfigSource source =
        gui::ResolveUserConfigSource(parsed.presence, gui::kTestHarnessUserConfigDefault);
    gui::SetUserConfigSourceForProcess(source, parsed.explicit_dir);
    // Report what was actually installed, never what the default is assumed to be. A red-state
    // probe that flipped kTestHarnessUserConfigDefault to kAutoDetect caught an earlier version of
    // this line still printing "disabled": the isolation was genuinely broken and the one
    // diagnostic a reader would consult was asserting the opposite.
    switch (source) {
      case gui::UserConfigSource::kExplicitDir:
        fprintf(stderr, "[DIAG] User config: explicit directory '%s' (--user-config)\n",
                parsed.explicit_dir.string().c_str());
        break;
      case gui::UserConfigSource::kDisabled:
        fprintf(stderr, "[DIAG] User config: disabled (isolated from this machine's saved defaults)\n");
        break;
      case gui::UserConfigSource::kAutoDetect:
        fprintf(stderr, "[DIAG] User config: AUTO-DETECT — reference comparisons are NOT isolated\n");
        break;
    }
  }

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
  // Lock to non-Retina framebuffer on macOS so framebuffer size == window size
  // (1600x980) regardless of cold/warm start or visible/hidden state. Visual
  // regression tests (e.g. visual/left_panel) need deterministic
  // capture dimensions; without this hint, hidden windows may yield 400x912
  // on one run and 800x1824 on the next.
  glfwWindowHint(GLFW_COCOA_RETINA_FRAMEBUFFER, GLFW_FALSE);
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
  if (g_enable_fixed_dt) {
    fprintf(stderr, "[DIAG] Fixed-dt mode enabled: dt=16.67ms injected per frame, frame-limit sleep skipped\n");
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
  gui::ApplyDockingConfig(io);
  // Deliberately NOT mirroring src/gui/main.cpp's layout persistence: this is the same isolation
  // rule as --user-config defaulting to kDisabled here. A reference image must not depend on a
  // layout file that happens to exist on the machine that captured it.
  io.IniFilename = nullptr;

  gui::ApplyVisualLanguage(io);

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
  // Stream test engine diagnostics to stderr so failing tests surface their
  // IM_CHECK errors on the build log (default discards them).
  test_io.ConfigLogToTTY = true;
  test_io.ConfigRunSpeed = ImGuiTestRunSpeed_Fast;
  test_io.ConfigNoThrottle = true;
  // Fixed-dt mode: inject a deterministic 1/60s dt every frame (applied via
  // PostSwap's ConfigFixedDeltaTime path) instead of deriving dt from the real
  // frame clock. Paired with the skipped frame-limit sleep, correctness tests get
  // production-faithful 60fps frame-budget semantics at full wall-clock speed.
  if (g_enable_fixed_dt) {
    test_io.ConfigFixedDeltaTime = 1.0f / 60.0f;
  }
  // JUnit XML export (opt-in via --export-junit). ImGuiTestEngine_Export() runs
  // automatically at batch end / engine stop, so setting these fields suffices.
  if (g_export_junit_path != nullptr) {
    test_io.ExportResultsFilename = g_export_junit_path;
    test_io.ExportResultsFormat = ImGuiTestEngineExportFormat_JUnitXml;
  }

  ImGuiTestEngine_Start(engine, ImGui::GetCurrentContext());
  ImGuiTestEngine_InstallDefaultCrashHandler();

  // Register and queue all tests
  RegisterViewDisplayControlTests(engine);
  RegisterExportPreviewTests(engine);
  RegisterPreviewPixelTests(engine);
  RegisterPreviewTextureTests(engine);
  RegisterEntryManagementTests(engine);
  RegisterBackgroundOverlayTests(engine);
  RegisterFileOpsTests(engine);
  RegisterColorWindowTests(engine);
  RegisterFilterEditorTests(engine);
  RegisterEditModalTests(engine);
  RegisterSceneControlTests(engine);
  RegisterExecutionClusterTests(engine);
  RegisterShellChromeTests(engine);
  RegisterDocumentColumnTests(engine);
  RegisterLogPanelTests(engine);
  RegisterOverlayControlTests(engine);
  RegisterPreviewViewportTests(engine);
  RegisterPerfTests(engine);
  RegisterOverlayLabelTests(engine);
  RegisterFaceNumberOverlayTests(engine);
  RegisterRunLifecycleTests(engine);
  RegisterStatusBarTests(engine);
  RegisterPreviewAnimationTests(engine);
  RegisterCaptureHarnessTests(engine);
  RegisterSimE2eSmokeTests(engine);
  RegisterDefaultsPanelTests(engine);
  RegisterDefaultsPanelLayoutTests(engine);
  RegisterLensProjectionTests(engine);
  ImGuiTestEngine_QueueTests(engine, ImGuiTestGroup_Tests, test_filter);

  // Main loop — runs until all tests complete
  while (true) {
    glfwPollEvents();
    gui::SyncFromPoller();  // Sync server data for perf tests (no-op when g_server is null)

    // Auto-commit on main thread (matches real app's main.cpp:284-301).
    // When enabled, DoRun() blocks the main loop just like in the real app,
    // so VSync frame budget effects are faithfully reproduced.
    // ⚠️ MIRROR of src/gui/main.cpp:284-301 — if that throttle/accounting block
    // changes (dirty-clear timing, restart counting, DoRun return-value handling),
    // MIRROR the change here. task-metal-gui-commit-backpressure §4 Step 4.
    if (g_enable_main_loop_commit && gui::g_server) {
      static auto last_commit = std::chrono::steady_clock::now();
      if (gui::g_state.dirty) {
        auto ss = gui::g_state.sim_state;
        // Mirrors main.cpp: only auto-commit while simulating (kDone&&dirty reconciles to kModified).
        if (ss == gui::GuiState::SimState::kSimulating) {
          auto now = std::chrono::steady_clock::now();
          auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_commit).count();
          if (elapsed >= gui::kCommitIntervalMs) {
            // task-metal-gui-commit-backpressure: only account/clear-dirty when DoRun
            // actually pushed the commit. When the backpressure gate defers (Metal
            // first batch not landed), DoRun returns false; a false-return path must
            // NOT count as a restart nor accumulate stats[0].sim_ray_num (the sim
            // never restarted, so its counter is still growing under the previous
            // epoch — double-counting would appear as inflated cumulative rays).
            // `last_commit` always advances so the 70ms cadence check is unchanged.
            LUMICE_StatsResult stats{};
            {
              lumice::test::ScopedResultFrame frame(gui::g_server);
              LUMICE_FrameGetStats(frame.get(), &stats);
            }
            unsigned long snapshot_rays = stats.sim_ray_num;

            bool committed = gui::DoRun(/*user_initiated=*/false);
            if (committed) {
              g_main_loop_cumulative_rays += snapshot_rays;
              g_main_loop_restart_count++;
              gui::g_state.dirty = false;
            }
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
    // Mirrors src/gui/main.cpp: same call, same arguments, so the dock layout a screenshot is taken
    // from is the product's layout rather than a harness-local copy of it.
    const float dock_host_height = layout_height - gui::kTopBarHeight - gui::kStatusBarHeight;
    const ImGuiID dockspace_id = gui::RenderDockSpaceHost(0.0f, gui::kTopBarHeight, layout_width, dock_host_height);
    gui::BuildDefaultDockLayout(dockspace_id, layout_width, dock_host_height);
    gui::RenderDocumentTree();
    gui::RenderDocumentInspector();
    gui::RenderRightPanel(window);
    gui::RenderPreviewPanel(window, layout_width, layout_height);
    if (g_enable_log_panel) {
      gui::RenderLogPanel(layout_width, layout_height);
    }
    // task-345.5: Colors window was previously never rendered by the harness
    // main loop, leaving its ImGui-level widgets (including new tooltips) zero-
    // covered. Adding it here mirrors src/gui/main.cpp:346. RenderColorWindow
    // early-returns when color_window_open is false, so cost for existing
    // tests is negligible.
    gui::RenderColorWindow(gui::g_state, gui::g_server);
    gui::RenderStatusBar(layout_width, layout_height);
    gui::RenderSpectrumModal(gui::g_state);
    gui::RenderUnsavedPopup(window);
    // task-cleanup-hardening code-review-01 M1: RenderSaveModifiedPopup was
    // added to src/gui/main.cpp's frame loop (AC4) but never mirrored here,
    // so its widgets (Run first / Save anyway / Cancel) were unreachable by
    // any UI-driven gui_test — the existing p2_modal AC4 tests could only
    // exercise DoSave()/PerformSave() directly. Mirrors main.cpp:352.
    gui::RenderSaveModifiedPopup(window);
    // Same omission, found again from the other end: the generic warning modal was in the
    // product's frame loop and not in this one, so the only proposition about it that gui_test
    // could state was about a flag.
    gui::RenderGuiWarningPopup();
    // Its import-specific sibling, absent here for the same reason and added with the rewrite of
    // the suites that queue one. Two things make that safe now. The message it opens on is a
    // file-scope string in app_panels.cpp, and ResetTestState() clears it, so an import performed
    // by one case cannot open a modal in the middle of the next one. What a reset cannot close is
    // a modal already on screen, so the case that opens this one dismisses it through its own OK
    // button — an open modal swallows every subsequent ItemClick in the process.
    gui::RenderImportWarningPopup();
    // The export-overwrite prompt, mirrored here from the same block of src/gui/main.cpp. A
    // confirmation modal that only the product's loop renders is a confirmation no test can drive
    // — and "the prompt never appeared, so the export silently did not happen" is precisely the
    // failure mode it exists to prevent, so it has to be reachable from here.
    gui::RenderExportOverwriteConfirmPopup();
    // Mirrors src/gui/main.cpp: a Render*Panel that only the production loop calls is
    // unreachable for every gui_test ("Unable to locate item"), a failure this repo has
    // already paid for twice.
    gui::RenderDefaultsPanel(gui::g_state);

    // Field-tier effect reconcile at frame TAIL — mirrors src/gui/main.cpp M6 placement so widget
    // edits driven by ImGuiTestEngine land in state.dirty within the same frame (rather than one
    // frame late as they would if this were folded into SyncFromPoller). Required for AC2
    // same-frame regression coverage.
    gui::ApplyGuiEffects(gui::g_state, gui::g_server, gui::ReconcileGuiEffects(gui::g_state));

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

    // Left-panel default-framebuffer capture hook (task-left-panel-visual-regression).
    // Must run AFTER ImGui_ImplOpenGL3_RenderDrawData and BEFORE glfwSwapBuffers,
    // per ReadbackGlRegionToRgba contract for default-framebuffer capture.
    //
    // Rect formula stays in sync with RenderLeftPanel's SetNextWindowPos/Size
    // (src/gui/app_panels.cpp). Modifying left-panel geometry requires updating
    // this block too. Retina scale is resolved at runtime via GLFW and validated
    // by IM_CHECK_EQ(ref_w, capture.width) in the test.
    if (g_left_panel_capture.requested.exchange(false)) {
      int fb_w = 0;
      int fb_h = 0;
      glfwGetFramebufferSize(window, &fb_w, &fb_h);
      int win_w2 = 0;
      int win_h2 = 0;
      glfwGetWindowSize(window, &win_w2, &win_h2);
      // Locals intentionally non-const: project's clang-tidy ConstantCase=CamelCase+k
      // would force kSx/kRx etc. which is worse for short runtime-computed values.
      float sx = win_w2 > 0 ? static_cast<float>(fb_w) / static_cast<float>(win_w2) : 1.0f;
      float sy = win_h2 > 0 ? static_cast<float>(fb_h) / static_cast<float>(win_h2) : 1.0f;
      int rx = 0;
      int ry = static_cast<int>(gui::kStatusBarHeight * sy);
      int rw = static_cast<int>(gui::kLeftPanelWidth * sx);
      int rh = fb_h - static_cast<int>((gui::kTopBarHeight + gui::kStatusBarHeight) * sy);
      if (lumice::gui::ReadbackGlRegionToRgba(rx, ry, rw, rh, g_left_panel_capture.pixels)) {
        g_left_panel_capture.width = rw;
        g_left_panel_capture.height = rh;
        g_left_panel_capture.done.store(true);
      } else {
        // fprintf matches diagnostic style used elsewhere in this file
        // (glfwInit failure, DIAG messages); not a log-framework path.
        fprintf(stderr, "[LeftPanelCapture] ReadbackGlRegionToRgba failed (rx=%d ry=%d rw=%d rh=%d fb=%dx%d)\n", rx, ry,
                rw, rh, fb_w, fb_h);
      }
    }

    // Generic default-framebuffer capture hook (whole frame or caller-selected rect).
    // Same timing contract as the left-panel hook above: AFTER RenderDrawData, BEFORE
    // glfwSwapBuffers. Unlike that hook, the rectangle comes from the requesting test
    // (see FullFrameCaptureState), so no panel geometry is hard-coded here.
    if (g_fullframe_capture.requested.exchange(false)) {
      int fb_w = 0;
      int fb_h = 0;
      glfwGetFramebufferSize(window, &fb_w, &fb_h);
      // Locals intentionally non-const: matches the left-panel hook's rationale
      // (project clang-tidy would force kRx/kRy naming on const locals).
      int rx = g_fullframe_capture.rect_x;
      int ry = g_fullframe_capture.rect_y;
      int rw = g_fullframe_capture.rect_w;
      int rh = g_fullframe_capture.rect_h;
      if (rw <= 0 || rh <= 0) {
        rx = 0;
        ry = 0;
        rw = fb_w;
        rh = fb_h;
      }
      if (lumice::gui::ReadbackGlRegionToRgba(rx, ry, rw, rh, g_fullframe_capture.pixels)) {
        g_fullframe_capture.width = rw;
        g_fullframe_capture.height = rh;
        g_fullframe_capture.done.store(true);
      } else {
        fprintf(stderr, "[FullFrameCapture] ReadbackGlRegionToRgba failed (rx=%d ry=%d rw=%d rh=%d fb=%dx%d)\n", rx, ry,
                rw, rh, fb_w, fb_h);
      }
    }

    // Auto-EV preview export hook (task-visual-regression).
    // ExportPreviewPng renders to a separate off-screen FBO, so placement here
    // (after RenderDrawData, before SwapBuffers) is safe — no default-framebuffer
    // dependency unlike the left-panel capture above.
    if (g_auto_ev_export.requested.exchange(false)) {
      g_auto_ev_export.result =
          gui::ExportPreviewPng(g_auto_ev_export.export_path, gui::g_preview, g_auto_ev_export.custom_vp);
      g_auto_ev_export.done.store(true);
    }

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
  // Stop the background poller before global static destructors run, mirroring the real app's exit
  // teardown (src/gui/main.cpp:413-414). The M6 frame-tail ApplyGuiEffects can leave the poller
  // RUNNING (display-push WakeForRefresh) after the final test; without an explicit Stop() here the
  // static g_server_poller dtor's worker-join hangs at process exit — gui_test runs every test,
  // prints the summary, then never returns.
  gui::JoinPendingStop();
  gui::g_server_poller.Stop();

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
