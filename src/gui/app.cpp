#include "gui/app.hpp"

#include <GLFW/glfw3.h>
#include <spdlog/spdlog.h>  // NOLINT(misc-include-cleaner) — used by logger.hpp macros
#include <stb_image.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include "gui/export_fbo_renderer.hpp"
#include "gui/file_io.hpp"
#include "gui/gui_logger.hpp"
#include "util/color_space.hpp"
#include "util/path_utils.hpp"

namespace lumice::gui {

using SimState = GuiState::SimState;

// Global state definitions
GuiState g_state;
PreviewRenderer g_preview;
CrystalRenderer g_crystal_renderer;
ThumbnailCache g_thumbnail_cache;
LUMICE_Server* g_server = nullptr;
ServerPoller g_server_poller;
PreviewViewport g_preview_vp;

int g_programmatic_resize = 0;


bool g_show_unsaved_popup = false;
PendingAction g_pending_action = PendingAction::kNone;

OverlayLabelInput BuildOverlayLabelInput(const GuiState& state, const RenderConfig& rc) {
  OverlayLabelInput input{};
  input.lens_type = rc.lens_type;
  input.fov = rc.fov;
  input.elevation = rc.elevation;
  input.azimuth = rc.azimuth;
  input.roll = rc.roll;
  input.visible = rc.visible;
  input.show_horizon = state.show_horizon;
  input.show_grid = state.show_grid;
  input.show_sun_circles = state.show_sun_circles;

  // Sun direction in world space (azimuth fixed at 0, only altitude matters).
  // Formula is byte-identical to RenderPreviewPanel's assignment into
  // g_preview_vp.params.overlay.sun_dir (see src/gui/app_panels.cpp ~L534-538):
  //   sa = altitude * deg2rad
  //   sun_dir = (-cos(sa), 0, -sin(sa))
  // Both callers share this helper so a future formula change only has to land here.
  constexpr float kDeg2Rad = 3.14159265358979323846f / 180.0f;
  float sa = state.sun.altitude * kDeg2Rad;
  input.sun_dir[0] = -std::cos(sa);
  input.sun_dir[1] = 0.0f;
  input.sun_dir[2] = -std::sin(sa);

  input.sun_circle_count = std::min(static_cast<int>(state.sun_circle_angles.size()), kMaxSunCircles);
  input.sun_circle_angles = state.sun_circle_angles.data();

  std::copy(std::begin(state.horizon_color), std::end(state.horizon_color), std::begin(input.horizon_color));
  std::copy(std::begin(state.grid_color), std::end(state.grid_color), std::begin(input.grid_color));
  std::copy(std::begin(state.sun_circles_color), std::end(state.sun_circles_color),
            std::begin(input.sun_circles_color));
  input.grid_alpha = state.grid_alpha;
  input.sun_circles_alpha = state.sun_circles_alpha;
  return input;
}

std::shared_ptr<ImGuiLogSink> g_imgui_log_sink;
std::shared_ptr<spdlog::sinks::basic_file_sink_mt> g_file_log_sink;
std::string g_log_file_path;

void GlfwErrorCallback(int error, const char* description) {
  fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

float GetAspectRatio(AspectPreset preset) {
  switch (preset) {
    case AspectPreset::k16x9:
      return 16.0f / 9.0f;
    case AspectPreset::k3x2:
      return 3.0f / 2.0f;
    case AspectPreset::k4x3:
      return 4.0f / 3.0f;
    case AspectPreset::k1x1:
      return 1.0f;
    case AspectPreset::k2x1:
      return 2.0f;
    case AspectPreset::kFree:
    case AspectPreset::kMatchBg:
    default:
      return 0.0f;
  }
}

void WindowSizeCallback(GLFWwindow* /*window*/, int /*width*/, int /*height*/) {
  if (g_programmatic_resize > 0) {
    g_programmatic_resize--;
    return;
  }
  if (g_state.aspect_preset != AspectPreset::kFree) {
    g_state.aspect_preset = AspectPreset::kFree;
  }
}

void ApplyAspectRatio(GLFWwindow* window, AspectPreset preset, bool portrait, float override_ratio) {
  float ratio = 0.0f;
  if (preset == AspectPreset::kMatchBg) {
    ratio = override_ratio > 0.0f ? override_ratio : g_preview.GetBgAspect();
    if (!g_preview.HasBackground()) {
      return;
    }
  } else {
    ratio = GetAspectRatio(preset);
  }
  if (portrait && ratio > 0.0f) {
    ratio = 1.0f / ratio;
  }
  if (ratio <= 0.0f) {
    return;
  }

  int win_w = 0;
  int win_h = 0;
  glfwGetWindowSize(window, &win_w, &win_h);

  constexpr float kCollapsedStripWidth = 20.0f;  // Must match kCollapseBtnSize in app_panels.cpp
  float left_w = g_state.left_panel_collapsed ? kCollapsedStripWidth : kLeftPanelWidth;
  float right_w = g_state.right_panel_collapsed ? kCollapsedStripWidth : kRightPanelWidth;
  float preview_w = std::max(1.0f, static_cast<float>(win_w) - left_w - right_w);
  float preview_h = preview_w / ratio;
  auto target_h = static_cast<int>(preview_h + kTopBarHeight + kStatusBarHeight);
  int target_w = win_w;

  int work_x = 0;
  int work_y = 0;
  int work_w = 0;
  int work_h = 0;
  glfwGetMonitorWorkarea(glfwGetPrimaryMonitor(), &work_x, &work_y, &work_w, &work_h);

  target_w = std::clamp(target_w, kMinWindowWidth, work_w);
  target_h = std::clamp(target_h, kMinWindowHeight, work_h);

  // If height was clamped, recalculate width to maintain ratio
  float actual_preview_h = static_cast<float>(target_h) - kTopBarHeight - kStatusBarHeight;
  if (actual_preview_h > 0.0f) {
    float actual_preview_w = actual_preview_h * ratio;
    int recalc_w = static_cast<int>(actual_preview_w + left_w + right_w);
    if (recalc_w >= kMinWindowWidth && recalc_w <= work_w) {
      target_w = recalc_w;
    }
  }

  g_programmatic_resize = 2;  // Expect up to 2 callbacks (some platforms fire intermediate + final)
  glfwSetWindowSize(window, target_w, target_h);

  // Clamp window position to stay within screen
  int pos_x = 0;
  int pos_y = 0;
  glfwGetWindowPos(window, &pos_x, &pos_y);
  bool moved = false;
  if (pos_x + target_w > work_x + work_w) {
    pos_x = work_x + work_w - target_w;
    moved = true;
  }
  if (pos_y + target_h > work_y + work_h) {
    pos_y = work_y + work_h - target_h;
    moved = true;
  }
  if (pos_x < work_x) {
    pos_x = work_x;
    moved = true;
  }
  if (pos_y < work_y) {
    pos_y = work_y;
    moved = true;
  }
  if (moved) {
    glfwSetWindowPos(window, pos_x, pos_y);
  }
}

// Ensure CPU-side sRGB uint8 texture is available for .lmc save.
// Uses raw XYZ data + current GUI intensity_scale to match the shader's rendering,
// rather than the old PostSnapshot path which used stale CommitConfig-time EV.
static void RefreshCpuTextureForSave() {
  if (!g_server || g_state.sim_state == SimState::kIdle) {
    return;  // No simulation data to refresh
  }

  LUMICE_RawXyzResult xyz_results[2]{};
  LUMICE_GetRawXyzResults(g_server, xyz_results, 1);
  if (xyz_results[0].xyz_buffer == nullptr || xyz_results[0].img_width <= 0 || xyz_results[0].img_height <= 0) {
    return;
  }

  int w = xyz_results[0].img_width;
  int h = xyz_results[0].img_height;

  // Compute intensity_scale using the CURRENT GUI exposure_offset, matching the shader.
  float intensity_factor = std::pow(2.0f, g_state.renderer.exposure_offset);
  float per_pixel_intensity = xyz_results[0].snapshot_intensity;
  float intensity_scale = per_pixel_intensity > 0 ? intensity_factor / per_pixel_intensity : 0.0f;

  // Convert XYZ→sRGB on CPU using the same algorithm as the shader
  std::vector<unsigned char> srgb(static_cast<size_t>(w) * h * 3);
  lumice::XyzToSrgbUint8(xyz_results[0].xyz_buffer, srgb.data(), w * h, intensity_scale);

  g_preview.UpdateCpuTextureData(srgb.data(), w, h);
}

void DoSave() {
  if (g_state.current_file_path.empty()) {
    g_state.current_file_path = ShowSaveDialog();
    if (g_state.current_file_path.empty()) {
      return;
    }
  }
  RefreshCpuTextureForSave();
  if (SaveLmcFile(g_state.current_file_path, g_state, g_preview, g_state.save_texture)) {
    g_state.dirty = false;
    GUI_LOG_INFO("[GUI] DoSave: {}", PathToU8(g_state.current_file_path));
  }
}

void DoSaveAs() {
  auto path = ShowSaveDialog();
  if (!path.empty()) {
    g_state.current_file_path = path;
    RefreshCpuTextureForSave();
    if (SaveLmcFile(path, g_state, g_preview, g_state.save_texture)) {
      g_state.dirty = false;
      GUI_LOG_INFO("[GUI] DoSaveAs: {}", PathToU8(path));
    }
  }
}

// Build PreviewParams for export: copy current preview viewport params, but override
// intensity_* fields to sample the GUI EV slider live (not the CommitConfig-time value
// that once leaked into Core snapshot exports — gui-polish-v10 Bug 1 root cause).
//
// Formula mirrors app_panels.cpp's live preview path, keeping shader u_intensity_scale
// byte-identical between preview and export:
//   intensity_factor = 2^exposure_offset
//   intensity_scale  = intensity_factor / snapshot_intensity  (0 if snapshot_intensity <= 0)
static PreviewParams BuildExportParams() {
  PreviewParams params = g_preview_vp.params;
  params.exposure.intensity_factor = std::pow(2.0f, g_state.renderer.exposure_offset);
  params.exposure.intensity_scale =
      g_state.snapshot_intensity > 0 ? params.exposure.intensity_factor / g_state.snapshot_intensity : 0.0f;
  return params;
}

void DoExportPreviewPng() {
  auto path = ShowExportPngDialog();
  if (path.empty()) {
    return;
  }

  PreviewParams params = BuildExportParams();
  std::optional<OverlayLabelInput> overlay;
  if (g_state.screenshot_include_overlay && (g_state.show_horizon || g_state.show_grid || g_state.show_sun_circles)) {
    overlay = BuildOverlayLabelInput(g_state, g_state.renderer);
  }

  int w = g_preview_vp.vp_w;
  int h = g_preview_vp.vp_h;
  auto rgba = RenderExportToRgba(g_preview, params, w, h, overlay);
  if (rgba.empty()) {
    GUI_LOG_ERROR("[GUI] Export screenshot failed: RenderExportToRgba returned empty (vp={}x{})", w, h);
    return;
  }

  if (!WriteRgbaBufferToPng(path, w, h, rgba)) {
    GUI_LOG_ERROR("[GUI] Export screenshot failed: PNG write error path={}", PathToU8(path));
    return;
  }
  GUI_LOG_INFO("[GUI] Export screenshot{}: {}", overlay.has_value() ? " (overlay)" : "", PathToU8(path));
}

void DoExportDualFisheyeEqualAreaPng() {
  auto path = ShowExportDualFisheyeEqualAreaDialog();
  if (path.empty()) {
    return;
  }

  int w = g_preview.GetTextureWidth();
  int h = g_preview.GetTextureHeight();
  if (w <= 0 || h <= 0) {
    GUI_LOG_ERROR("[GUI] Export dual fisheye: no texture loaded");
    return;
  }

  PreviewParams params = BuildExportParams();
  ConfigureDualFisheyeExportParams(params);

  auto rgba = RenderExportToRgba(g_preview, params, w, h, std::nullopt);
  if (rgba.empty()) {
    GUI_LOG_ERROR("[GUI] Export dual fisheye: RenderExportToRgba empty (size={}x{})", w, h);
    return;
  }
  if (!WriteRgbaBufferToPng(path, w, h, rgba)) {
    GUI_LOG_ERROR("[GUI] Export dual fisheye: PNG write failed path={}", PathToU8(path));
    return;
  }
  GUI_LOG_INFO("[GUI] Export dual fisheye equal-area: {}", PathToU8(path));
}

void DoExportEquirectangularPng() {
  auto path = ShowExportEquirectangularDialog();
  if (path.empty()) {
    return;
  }

  int tex_w = g_preview.GetTextureWidth();
  int tex_h = g_preview.GetTextureHeight();
  if (tex_w <= 0 || tex_h <= 0) {
    GUI_LOG_ERROR("[GUI] Export equirect: no texture loaded");
    return;
  }
  // Preserve 155.4 约定：short_res = min(tex_w/2, tex_h) for strict 2:1 output.
  int short_res = std::min(tex_w / 2, tex_h);
  if (short_res <= 0) {
    GUI_LOG_ERROR("[GUI] Export equirect: texture too small (tex={}x{})", tex_w, tex_h);
    return;
  }
  int dst_w = 2 * short_res;
  int dst_h = short_res;

  PreviewParams params = BuildExportParams();
  ConfigureEquirectExportParams(params);

  auto rgba = RenderExportToRgba(g_preview, params, dst_w, dst_h, std::nullopt);
  if (rgba.empty()) {
    GUI_LOG_ERROR("[GUI] Export equirect: RenderExportToRgba empty (size={}x{})", dst_w, dst_h);
    return;
  }
  if (!WriteRgbaBufferToPng(path, dst_w, dst_h, rgba)) {
    GUI_LOG_ERROR("[GUI] Export equirect: PNG write failed path={}", PathToU8(path));
    return;
  }
  GUI_LOG_INFO("[GUI] Export equirectangular: {}", PathToU8(path));
}

void DoExportConfigJson() {
  auto path = ShowExportJsonDialog();
  if (!path.empty()) {
    auto json_str = SerializeCoreConfig(g_state);
    ExportConfigJson(path, json_str);
    GUI_LOG_INFO("[GUI] Export config JSON: {}", PathToU8(path));
  }
}

// Helper: load image from path, downsample if needed, upload to bg texture.
// Returns true on success.
static bool LoadAndUploadBgImage(const std::filesystem::path& path) {
  int w = 0;
  int h = 0;
  int channels = 0;
  auto u8path = PathToU8(path);
  unsigned char* raw = stbi_load(u8path.c_str(), &w, &h, &channels, 3);  // Force 3 channels (RGB)
  if (!raw) {
    GUI_LOG_WARNING("Failed to load background image: {}", u8path);
    return false;
  }

  // Copy to vector, then free stbi allocation
  size_t byte_count = static_cast<size_t>(w) * h * 3;
  std::vector<unsigned char> data(raw, raw + byte_count);
  stbi_image_free(raw);

  // Box downsample while max dimension > 4096
  while (std::max(w, h) > 4096) {
    int new_w = w / 2;
    int new_h = h / 2;
    for (int y = 0; y < new_h; y++) {
      for (int x = 0; x < new_w; x++) {
        for (int c = 0; c < 3; c++) {
          int sum = data[(y * 2 * w + x * 2) * 3 + c] + data[(y * 2 * w + x * 2 + 1) * 3 + c] +
                    data[((y * 2 + 1) * w + x * 2) * 3 + c] + data[((y * 2 + 1) * w + x * 2 + 1) * 3 + c];
          data[(y * new_w + x) * 3 + c] = static_cast<unsigned char>(sum / 4);
        }
      }
    }
    w = new_w;
    h = new_h;
    data.resize(static_cast<size_t>(w) * h * 3);
  }

  g_preview.UploadBgTexture(data.data(), w, h);
  return true;
}

void DoOpen() {
  auto path = ShowOpenDialog();
  if (path.empty()) {
    return;
  }

  // Check extension to determine file format
  bool is_json = path.extension() == ".json";

  if (is_json) {
    // Import CLI JSON config
    std::ifstream in(path);
    if (!in.is_open()) {
      return;
    }
    std::string json_str((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    GuiState new_state = InitDefaultState();
    if (DeserializeFromJson(json_str, new_state)) {
      g_state = new_state;
      g_state.current_file_path.clear();  // Don't set .json path as save target
      g_state.dirty = true;               // Unsaved new project
      g_state.sim_state = SimState::kIdle;
      g_thumbnail_cache.OnLayerStructureChanged();
      g_preview.ClearTexture();
      g_preview.ClearBackground();
      GUI_LOG_INFO("[GUI] DoOpen (JSON import): {}", PathToU8(path));
    }
    return;
  }

  // Load .lmc binary file
  std::vector<unsigned char> tex_data;
  int tex_w = 0;
  int tex_h = 0;
  if (LoadLmcFile(path, g_state, tex_data, tex_w, tex_h)) {
    g_state.current_file_path = path;
    g_state.dirty = false;
    g_thumbnail_cache.OnLayerStructureChanged();
    GUI_LOG_INFO("[GUI] DoOpen: {}", PathToU8(path));
    if (!tex_data.empty()) {
      g_preview.UploadTexture(tex_data.data(), tex_w, tex_h);
      g_state.sim_state = SimState::kDone;
    } else {
      g_state.sim_state = SimState::kIdle;
    }

    // Restore background image from saved path (uses deserialized alpha, not reset to 0.5)
    g_preview.ClearBackground();
    if (!g_state.bg_path.empty()) {
      if (LoadAndUploadBgImage(g_state.bg_path)) {
        // bg_show and bg_alpha already restored from deserialization
      } else {
        // Degradation: bg image not found — clear show, reset kMatchBg→kFree
        g_state.bg_show = false;
        if (g_state.aspect_preset == AspectPreset::kMatchBg) {
          g_state.aspect_preset = AspectPreset::kFree;
        }
      }
    }
  }
}

void DoNew() {
  g_state = InitDefaultState();
  g_thumbnail_cache.OnLayerStructureChanged();
  g_preview.ClearTexture();
  g_preview.ClearBackground();
  g_crystal_mesh_hash = 0;
  GUI_LOG_INFO("[GUI] DoNew");
}

void DoLoadBackground(GLFWwindow* window) {
  auto path = ShowOpenImageDialog();
  if (path.empty()) {
    return;
  }

  if (!LoadAndUploadBgImage(path)) {
    return;
  }

  g_state.bg_path = path;
  g_state.bg_show = true;
  g_state.bg_alpha = 0.5f;  // Start at 50% to immediately show overlay effect
  g_state.aspect_preset = AspectPreset::kMatchBg;
  ApplyAspectRatio(window, AspectPreset::kMatchBg, false, g_preview.GetBgAspect());
}

void DoClearBackground() {
  g_preview.ClearBackground();
  g_state.bg_path.clear();
  g_state.bg_show = false;
  if (g_state.aspect_preset == AspectPreset::kMatchBg) {
    g_state.aspect_preset = AspectPreset::kFree;
  }
}

// Run a short simulation with the current default config to measure platform throughput,
// then set the quality gate threshold to max(kMinRaysFloor, measured_rate * 10%).
// Called once at startup before the main loop. Typically completes in < 200ms.
void CalibrateQualityThreshold() {
  if (!g_server) {
    return;
  }
  constexpr int kCalibrationRays = 100000;
  constexpr double kCalibrationFraction = 0.4;  // Accept frames >= 40% of typical ray count per window

  // Use current default state to build a calibration config
  LUMICE_Config config{};
  FillLumiceConfig(g_state, &config);
  config.infinite = 0;
  config.ray_num = kCalibrationRays;

  auto t0 = std::chrono::steady_clock::now();
  auto err = LUMICE_CommitConfigStruct(g_server, &config, nullptr);
  if (err != LUMICE_OK) {
    GUI_LOG_WARNING("[Calibration] CommitConfig failed ({}), using default threshold", static_cast<int>(err));
    return;
  }
  auto t_commit = std::chrono::steady_clock::now();
  GUI_LOG_DEBUG("[Calibration] CommitConfig took {:.1f}ms",
                std::chrono::duration<double, std::milli>(t_commit - t0).count());

  // Wait for simulation to complete (server returns to IDLE)
  constexpr int kMaxWaitMs = 2000;
  int waited_ms = 0;
  LUMICE_ServerState final_state = LUMICE_SERVER_RUNNING;
  while (waited_ms < kMaxWaitMs) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    waited_ms += 10;
    LUMICE_QueryServerState(g_server, &final_state);
    if (final_state == LUMICE_SERVER_IDLE) {
      break;
    }
  }

  auto t1 = std::chrono::steady_clock::now();
  double elapsed_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

  // Read stats to get actual rays simulated
  LUMICE_StatsResult stats[2]{};
  LUMICE_GetStatsResults(g_server, stats, 1);
  GUI_LOG_DEBUG("[Calibration] waited {}ms, state={}, sim_ray_num={}", waited_ms,
                final_state == LUMICE_SERVER_IDLE ? "IDLE" : "RUNNING", stats[0].sim_ray_num);

  if (stats[0].sim_ray_num == 0 || elapsed_ms <= 0) {
    GUI_LOG_WARNING("[Calibration] no data produced in {:.0f}ms, using default threshold", elapsed_ms);
    return;
  }

  // Compute: how many rays would be produced in the calibration window?
  // Uses kCalibrationWindowMs (fixed) instead of kCommitIntervalMs so that changing
  // the commit interval doesn't accidentally tighten/loosen the quality gate.
  double rays_per_window = static_cast<double>(stats[0].sim_ray_num) * kCalibrationWindowMs / elapsed_ms;
  auto threshold = std::max(kMinRaysFloor, static_cast<unsigned long>(rays_per_window * kCalibrationFraction));

  g_server_poller.SetCalibratedThreshold(threshold);
  GUI_LOG_INFO("[Calibration] done in {:.0f}ms: {} rays, {:.0f} rays/window({}ms), threshold={}", elapsed_ms,
               stats[0].sim_ray_num, rays_per_window, kCalibrationWindowMs, threshold);

  // Stop the server — ready for user's first actual Run
  LUMICE_StopServer(g_server);
}


void DoRun() {
  if (!g_server) {
    return;
  }
  auto run_start = std::chrono::steady_clock::now();

  // Pre-check: will CommitConfig rebuild consumers (destroying old buffers)?
  // Only renderer layout changes (resolution/lens/view/visible/filter) trigger rebuild.
  // High-frequency slider changes (crystal/sun/scattering) always reuse consumers.
  // If reuse is expected, skip Poller Stop — the poller's raw pointers remain valid.
  // NeedsRebuild comparison: RenderConfig::id was removed during the renderer copy-model
  // migration; since `id` was always 1 pre-migration, operator== behavior is strictly looser
  // and cannot produce new false-positive reuse paths.
  bool expect_rebuild =
      !g_state.last_committed_state.has_value() || g_state.renderer != g_state.last_committed_state->renderer;
  if (expect_rebuild) {
    g_server_poller.Stop();  // Must pause before consumer destruction
  }

  LUMICE_Config config{};
  FillLumiceConfig(g_state, &config);
  int reused = 0;
  auto err = LUMICE_CommitConfigStruct(g_server, &config, &reused);
  if (err == LUMICE_OK) {
    g_state.last_committed_state = GuiState::ConfigSnapshot::From(g_state);
    g_state.sim_state = SimState::kSimulating;
    g_state.stats_ray_seg_num = 0;
    g_state.stats_sim_ray_num = 0;
    // Safety check: if GUI predicted reuse but server rebuilt consumers, the poller
    // was not stopped and may hold dangling pointers. This should never happen because
    // the GUI comparison is a superset of the server's NeedsRebuild check.
    if (!expect_rebuild && !reused) {
      GUI_LOG_WARNING(
          "[GUI] DoRun: predict/actual mismatch! GUI predicted reuse but server rebuilt. "
          "Stopping poller to prevent dangling pointer.");
      g_server_poller.Stop();
    }
    if (expect_rebuild || !reused) {
      g_server_poller.Start(g_server);  // Rebuild: new consumers, reset server pointer
    }
    // Unconditionally ensure poller is running — covers all edge cases:
    // DoStop→DoRun (poller was paused), PollOnce self-pause (finite rays done), etc.
    // If already kRunning, this is a no-op (zero overhead).
    g_server_poller.EnsureRunning(g_server);
    // Discard old staged texture before unlocking to prevent stale data from being
    // uploaded in the next SyncFromPoller (filter change may not trigger rebuild,
    // so Start() may not have been called to reset staged data).
    if (g_state.intensity_locked) {
      g_server_poller.InvalidateStagedTexture();
    }
    g_state.intensity_locked = false;
    auto run_end = std::chrono::steady_clock::now();
    GUI_LOG_INFO("[GUI] DoRun: config committed ({:.1f}ms)",
                 std::chrono::duration<double, std::milli>(run_end - run_start).count());
  } else {
    GUI_LOG_WARNING("[GUI] CommitConfig FAILED with error code {}", static_cast<int>(err));
    // Restore poller if it was stopped for rebuild but CommitConfig failed
    if (expect_rebuild) {
      g_server_poller.EnsureRunning(g_server);
    }
  }
}

void DoStop() {
  if (!g_server)
    return;
  g_server_poller.Stop();  // Must stop poller before server to avoid dangling access
  LUMICE_StopServer(g_server);
  g_state.sim_state = SimState::kDone;
  GUI_LOG_INFO("[GUI] DoStop");
}

void DoRevert() {
  if (g_state.last_committed_state) {
    const auto& snapshot = *g_state.last_committed_state;
    // Restore configuration fields atomically, then fire GUI side effects.
    // Order rationale: ApplyTo() is pure field assignment. OnLayerStructureChanged()
    // currently only touches the thumbnail cache (see thumbnail_cache.cpp) and does
    // not read other g_state fields, so invoking it after full assignment is
    // equivalent to the previous order (layers assigned -> callback -> other fields).
    // If OnLayerStructureChanged ever starts reading g_state.sun/sim/renderers, this
    // order remains correct (callback sees fully restored state). Runtime state (dirty,
    // sim_state, poller counters, etc.) is intentionally preserved by ApplyTo.
    snapshot.ApplyTo(g_state);
    g_thumbnail_cache.OnLayerStructureChanged();
    g_state.sim_state = SimState::kDone;
  }
}

void SyncFromPoller() {
  if (g_state.sim_state != SimState::kSimulating) {
    return;
  }

  PollerData data;
  if (!g_server_poller.TrySyncData(data)) {
    GUI_LOG_VERBOSE("[GUI] SyncFromPoller: skipped (poller busy)");
    return;
  }
  if (!data.valid) {
    return;  // Worker hasn't produced data yet
  }

  // Only transition to kDone when the simulation has actually processed rays and then stopped.
  // A transient IDLE during restart (before any rays are consumed) has stats_sim_ray_num == 0,
  // so it's safely ignored here.
  if (data.server_state == LUMICE_SERVER_IDLE && data.stats_sim_ray_num > 0) {
    g_state.sim_state = SimState::kDone;
    GUI_LOG_INFO("[GUI] Simulation done");
  }

  // Update stats
  if (data.stats_sim_ray_num > 0) {
    g_state.stats_ray_seg_num = data.stats_ray_seg_num;
    g_state.stats_sim_ray_num = data.stats_sim_ray_num;
  }

  // Upload XYZ float texture (GL call — must be on main thread).
  // Quality gate is in ServerPoller::PollOnce — staged data is always worth displaying.
  // The intensity > 0 guard prevents black-frame flicker during slider scrubbing (restart
  // transiently produces 0-intensity snapshots). Filter changes set intensity_locked via
  // MarkFilterDirty() to block old data from overwriting the cleared display.
  // NOTE: original guard `g_state.selected_renderer >= 0` was an artifact of the ID/vector
  // model; with the copy-model renderer embedded directly, GuiState always owns a valid
  // renderer by default construction, so the guard is replaced by value-semantics.
  bool upload_ok = data.has_new_texture && data.snapshot_intensity > 0 && !g_state.intensity_locked;
  if (upload_ok) {
    GUI_LOG_VERBOSE("[GUI] SyncFromPoller: upload tex_rays={}, intensity={:.6f}, eff_pixels={}, factor={:.6f}",
                    data.texture_ray_count, data.snapshot_intensity, data.effective_pixels, data.intensity_factor);
    g_preview.UploadXyzTexture(data.xyz_data.data(), data.texture_width, data.texture_height);
    g_state.snapshot_intensity = data.snapshot_intensity;
    g_state.effective_pixels = data.effective_pixels;
    g_state.texture_upload_count++;
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

}  // namespace lumice::gui
