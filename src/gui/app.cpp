#include "gui/app.hpp"

#include <GLFW/glfw3.h>
#include <spdlog/spdlog.h>  // NOLINT(misc-include-cleaner) — used by logger.hpp macros
#include <stb_image.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <future>
#include <string>
#include <thread>
#include <vector>

#include "gui/export_fbo_renderer.hpp"
#include "gui/file_io.hpp"
#include "gui/gui_ev_auto.hpp"
#include "gui/gui_logger.hpp"
#include "gui/window_sizing.hpp"
#include "include/lumice_config_scope.hpp"
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
// Tracks whether the live g_server was *constructed* for a GPU backend (not the
// env-resolved actual). Startup uses LUMICE_CreateServer() → preferred_backend=CPU,
// so this is false until the first GPU toggle. See MaybeReconstructServerForBackend.
bool g_server_is_gpu = false;
PreviewViewport g_preview_vp;

// Async Stop plumbing (blueprint §5/§8, 1.6). DoStop offloads the blocking teardown sequence
// (`poller.Stop() + LUMICE_StopServer`) onto this future so the UI thread returns immediately and
// paints the optimistic "Stopping…" state. g_stop_inflight is the coarse completion latch read by
// SyncFromPoller to advance the intent to kStopped. Both live at file scope (single GUI thread
// owns the future; the atomic crosses the one background thread).
std::atomic<bool> g_stop_inflight{ false };
namespace {
std::future<void> g_stop_future;
}  // namespace

// Block until any in-flight async Stop has finished (poller + server drained), then release the
// future. Idempotent / cheap when no stop is pending. MUST be called before every path that
// destroys or reconstructs g_server / g_server_poller, so the background thread can never touch a
// freed server (R1 use-after-free guard): shutdown (main.cpp), DoRun top, and
// MaybeReconstructServerForBackend.
void JoinPendingStop() {
  if (g_stop_future.valid()) {
    g_stop_future.wait();
    g_stop_future = {};
  }
}

int g_programmatic_resize = 0;


bool g_show_unsaved_popup = false;
PendingAction g_pending_action = PendingAction::kNone;

float ComputeGridStep(float fov) {
  // FOV here is full-angle in degrees (matches RenderConfig::fov / ViewProjection::fov).
  // Aim: ~4–6 grid lines across the visible FOV; round to a "nice" step.
  if (fov >= 120.0f)
    return 30.0f;
  if (fov >= 60.0f)
    return 20.0f;
  if (fov >= 30.0f)
    return 10.0f;
  if (fov >= 15.0f)
    return 5.0f;
  if (fov >= 6.0f)
    return 2.0f;
  if (fov >= 2.0f)
    return 1.0f;
  return 0.5f;
}

OverlayLabelInput BuildOverlayLabelInput(const GuiState& state, const RenderConfig& rc) {
  OverlayLabelInput input{};
  input.lens_type = rc.lens_type;
  input.fov = rc.fov;
  input.elevation = rc.elevation;
  input.azimuth = rc.azimuth;
  input.roll = EffectiveRollForLens(rc.lens_type, rc.roll);
  input.visible = rc.visible;
  input.front = rc.front;
  input.show_horizon = state.show_horizon_label;
  input.show_grid = state.show_grid_label;
  input.show_sun_circles = state.show_sun_circles_label;
  input.horizon_alpha = state.horizon_alpha;

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
  input.grid_step = ComputeGridStep(rc.fov);
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
  // Manual resize takes the user out of any preset, so the "screen too small"
  // warning is no longer meaningful — clear it so the GUI does not display a
  // stale clamp banner against a Free preset.
  g_state.aspect_clamp = {};
}

void ApplyAspectRatio(GLFWwindow* window, AspectPreset preset, bool portrait, float override_ratio) {
  float ratio = 0.0f;
  if (preset == AspectPreset::kMatchBg) {
    ratio = override_ratio > 0.0f ? override_ratio : g_preview.GetBgAspect();
    if (!g_preview.HasBackground()) {
      g_state.aspect_clamp = {};
      return;
    }
  } else {
    ratio = GetAspectRatio(preset);
  }
  if (portrait && ratio > 0.0f) {
    ratio = 1.0f / ratio;
  }
  if (ratio <= 0.0f) {
    // kFree (or any other ratio-less preset) reaches here — no preview-region
    // ratio to honor, so any prior clamp warning is no longer relevant.
    g_state.aspect_clamp = {};
    return;
  }

  int win_w = 0;
  int win_h = 0;
  glfwGetWindowSize(window, &win_w, &win_h);
  int pos_x = 0;
  int pos_y = 0;
  glfwGetWindowPos(window, &pos_x, &pos_y);

  constexpr float kCollapsedStripWidth = 20.0f;  // Must match kCollapseBtnSize in app_panels.cpp
  float left_w = g_state.left_panel_collapsed ? kCollapsedStripWidth : kLeftPanelWidth;
  float right_w = g_state.right_panel_collapsed ? kCollapsedStripWidth : kRightPanelWidth;

  // Select the monitor containing the window center so multi-monitor users do
  // not get yanked back to primary when aspect ratio changes (v11 bug #4).
  int cx = pos_x + win_w / 2;
  int cy = pos_y + win_h / 2;
  int mon_count = 0;
  GLFWmonitor** mons = glfwGetMonitors(&mon_count);
  std::vector<MonitorRect> rects;
  if (mon_count > 0) {
    rects.reserve(static_cast<size_t>(mon_count));
  }
  for (int i = 0; i < mon_count; i++) {
    MonitorRect r{};
    glfwGetMonitorWorkarea(mons[i], &r.x, &r.y, &r.w, &r.h);
    rects.push_back(r);
  }
  int work_x = 0;
  int work_y = 0;
  int work_w = 0;
  int work_h = 0;
  int mon_idx = SelectMonitorIndexByCenter(cx, cy, rects.data(), static_cast<int>(rects.size()));
  if (mon_idx >= 0) {
    const auto& r = rects[mon_idx];
    work_x = r.x;
    work_y = r.y;
    work_w = r.w;
    work_h = r.h;
  } else {
    glfwGetMonitorWorkarea(glfwGetPrimaryMonitor(), &work_x, &work_y, &work_w, &work_h);
  }

  AspectFitResult fit =
      ResolveAspectFit(win_w, ratio, work_w, work_h, left_w, right_w, kTopBarHeight, kStatusBarHeight);
  int target_w = fit.target_w;
  int target_h = fit.target_h;

  g_programmatic_resize = 2;  // Expect up to 2 callbacks (some platforms fire intermediate + final)
  glfwSetWindowSize(window, target_w, target_h);

  // Clamp window position to stay within the selected monitor's workarea.
  // pos_x/pos_y were read at function entry (pre-resize); GLFW's SetWindowSize
  // anchors on top-left, so the position remains valid post-resize.
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

  // Surface the clamp signal to the GUI. Done after glfwSetWindowSize so the
  // achieved/requested ratios written to state correspond to the size we
  // actually asked the OS for (the resize callback may further fudge the size
  // by ±1 px, but ResolveAspectFit already accounts for chrome rounding).
  g_state.aspect_clamp.was_clamped = fit.was_clamped;
  g_state.aspect_clamp.requested_preview_ratio = fit.requested_preview_ratio;
  g_state.aspect_clamp.achieved_preview_ratio = fit.achieved_preview_ratio;
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

  // Compute intensity_scale matching the shader via ev_total = exposure_offset + ev_auto.
  float intensity_factor = std::pow(2.0f, g_state.renderer.exposure_offset + g_state.ev_auto);
  float norm_intensity = xyz_results[0].snapshot_intensity;
  float intensity_scale = norm_intensity > 0 ? intensity_factor / norm_intensity : 0.0f;

  // Convert XYZ→sRGB on CPU using the same algorithm as the shader
  std::vector<unsigned char> srgb(static_cast<size_t>(w) * h * 3);
  LUMICE_XyzToSrgbUint8(xyz_results[0].xyz_buffer, srgb.data(), w * h, intensity_scale);

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
// intensity_* fields to sample the GUI EV slider live.
//   ev_total         = exposure_offset + ev_auto
//   intensity_factor = 2^ev_total
//   intensity_scale  = intensity_factor / snapshot_intensity   (0 if snapshot_intensity <= 0)
static PreviewParams BuildExportParams() {
  PreviewParams params = g_preview_vp.params;
  float ev_total = g_state.renderer.exposure_offset + g_state.ev_auto;
  params.exposure.intensity_factor = std::pow(2.0f, ev_total);
  float norm_intensity = g_state.snapshot_intensity;
  params.exposure.intensity_scale = norm_intensity > 0 ? params.exposure.intensity_factor / norm_intensity : 0.0f;
  return params;
}

void DoExportPreviewPng() {
  auto path = ShowExportPngDialog();
  if (path.empty()) {
    return;
  }

  PreviewParams params = BuildExportParams();
  std::optional<OverlayLabelInput> overlay;
  if (g_state.screenshot_include_overlay &&
      (g_state.show_horizon_label || g_state.show_grid_label || g_state.show_sun_circles_label)) {
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
  if (path.empty()) {
    return;
  }
  // Build-or-reject is a pure, unit-tested function (BuildExportJsonOrWarn): it refuses to
  // produce a config when a filter overflows the ABI bounds — otherwise the JSON twin would
  // emit a semantically-opposite match-all stand-in and we would silently write a wrong
  // config (code-review-02/03 Major). This wrapper only owns the dialog path + user surface.
  std::string json_str;
  std::string warning;
  if (!BuildExportJsonOrWarn(g_state, &json_str, &warning)) {
    GUI_LOG_WARNING("[GUI] Export config JSON aborted: {}", warning);
    SetGuiWarning(warning);
    return;
  }
  ExportConfigJson(path, json_str);
  GUI_LOG_INFO("[GUI] Export config JSON: {}", PathToU8(path));
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
      // Intent: no server run behind this import (→ kIdle via ReconcileSimState).
      g_state.run_intent = RunIntent::kNone;
      g_thumbnail_cache.OnLayerStructureChanged();
      g_preview.ClearTexture();
      g_preview.ClearBackground();
      // Reset modal preview trackball to the imported config's first entry.
      if (!g_state.layers.empty() && !g_state.layers[0].entries.empty()) {
        ResetCrystalViewToCrystal(g_state.crystals[g_state.layers[0].entries[0].crystal_id]);
      }
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
    // Reset modal preview trackball to the loaded file's first entry.
    if (!g_state.layers.empty() && !g_state.layers[0].entries.empty()) {
      ResetCrystalViewToCrystal(g_state.crystals[g_state.layers[0].entries[0].crystal_id]);
    }
    GUI_LOG_INFO("[GUI] DoOpen: {}", PathToU8(path));
    if (!tex_data.empty()) {
      g_preview.UploadTexture(tex_data.data(), tex_w, tex_h);
      // Intent: a baked static result (→ kDone via ReconcileSimState, no server run).
      g_state.run_intent = RunIntent::kLoaded;
    } else {
      // Intent: no result to show (→ kIdle).
      g_state.run_intent = RunIntent::kNone;
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
  // Reset modal preview trackball to the new default entry's preset view —
  // otherwise a stale drag pose from before New persists into the new doc.
  if (!g_state.layers.empty() && !g_state.layers[0].entries.empty()) {
    ResetCrystalViewToCrystal(g_state.crystals[g_state.layers[0].entries[0].crystal_id]);
  }
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
  lumice::ConfigColorGuard config_color_guard(config);  // v4.8: release raypath_color on any early return
  if (!FillLumiceConfig(g_state, &config)) {
    // Unlike DoRun (which pops SetGuiWarning), calibration degrades silently to the default
    // threshold: it does not touch the user's edited/committed config, so a log line is
    // sufficient and a modal would be noise. Intentional asymmetry, not an oversight.
    GUI_LOG_WARNING("[Calibration] filter exceeds ABI limits, using default threshold");
    return;
  }
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
  auto threshold = std::max(kMinRaysFloor, static_cast<unsigned long long>(rays_per_window * kCalibrationFraction));

  g_server_poller.SetCalibratedThreshold(threshold);
  GUI_LOG_INFO("[Calibration] done in {:.0f}ms: {} rays, {:.0f} rays/window({}ms), threshold={}", elapsed_ms,
               stats[0].sim_ray_num, rays_per_window, kCalibrationWindowMs, threshold);

  // Stop the server — ready for user's first actual Run
  LUMICE_StopServer(g_server);
}


// Resolve which GPU backend to request on this host: Metal on Apple, CUDA on
// NVIDIA. Both are single-engine GPU routes (vs the CPU N-worker topology). Probes
// the C API at runtime, so a host with neither resolves to CPU (the toggle that
// drives this is hidden when no GPU is available — see panels.cpp).
static int ResolveGpuBackend() {
  if (LUMICE_IsBackendAvailable(LUMICE_BACKEND_METAL)) {
    return LUMICE_BACKEND_METAL;
  }
  if (LUMICE_IsBackendAvailable(LUMICE_BACKEND_CUDA)) {
    return LUMICE_BACKEND_CUDA;
  }
  return LUMICE_BACKEND_CPU;
}

// Reconstruct the server so its orchestration *topology* matches the requested
// backend. Backend is a construction-time property — CPU is N-worker
// queue-per-Simulator; the GPU route (Metal/CUDA) is a single engine with large
// dispatch (task-268.7) — so it cannot be flipped on a live server
// (SetPreferredBackend on an N-worker server would run GPU kernels inside the
// 12-worker structure, the 0.58× anti-pattern this scrum removed). Toggling the
// "Use GPU" checkbox therefore tears the server down and rebuilds it with the new
// preferred_backend.
//
// The accumulated image is intentionally discarded: CPU and GPU are
// statistically-equivalent-but-not-identical sample streams, so mixing them in one
// accumulation buffer is physically unsound — a clean reset is the correct
// semantics, and a fresh re-converge is what a backend A/B comparison wants.
//
// Returns true iff the server was reconstructed; the caller must then force a full
// consumer rebuild + fresh poller Start (the new server has no consumers).
bool MaybeReconstructServerForBackend() {
  bool want_gpu = g_state.use_gpu_backend;
  if (want_gpu == g_server_is_gpu) {
    return false;  // backend unchanged — keep the live server
  }
  GUI_LOG_INFO("[GUI] Backend toggle: reconstructing server ({} -> {})", g_server_is_gpu ? "GPU" : "CPU",
               want_gpu ? "GPU" : "CPU");
  JoinPendingStop();       // R1: a background stop may still hold this server — drain it before destroy
  g_server_poller.Stop();  // synchronous: worker confirmed no longer touching the old server
  LUMICE_DestroyServer(g_server);

  LUMICE_ServerConfig cfg{};
  cfg.num_workers = 0;  // 0 = PhysicalCoreCount (CPU); ignored on the GPU single engine
  cfg.sim_seed = 0;     // 0 = random — matches LUMICE_CreateServer() startup default
  cfg.preferred_backend = want_gpu ? ResolveGpuBackend() : LUMICE_BACKEND_CPU;
  g_server = LUMICE_CreateServerEx(&cfg);
  g_server_is_gpu = want_gpu;

  // Re-apply per-server settings that died with the old instance (cf. main.cpp startup).
  LUMICE_SetLogLevel(g_server, static_cast<LUMICE_LogLevel>(g_state.core_log_level));

  // The fresh server restarts its epoch authority at 0. Reset the GUI's display generation so the
  // new backend's low-epoch frames are not fenced out by the old server's carried-over
  // display_epoch_floor (else the preview freezes on the previous backend's texture). See
  // GuiState::ResetDisplayGenerationForBackendSwap. The epoch is re-read from the new server by
  // DoRun's post-commit LUMICE_GetSimLifecycle.
  g_state.ResetDisplayGenerationForBackendSwap();
  return true;
}

void DoRun() {
  if (!g_server) {
    return;
  }
  // R1: a previous Stop may still be draining on the background thread. It calls poller.Stop() +
  // LUMICE_StopServer on this same server; joining first makes the DoRun commit/rebuild below
  // race-free (and the reused-consumer / poller-pointer reasoning valid).
  JoinPendingStop();
  auto run_start = std::chrono::steady_clock::now();

  // Backend toggle reconstructs the server (per-backend orchestration topology).
  // A reconstructed server has no consumers, so force the full-rebuild path below.
  // No-op (returns false) when the GPU toggle is off / unchanged or unavailable.
  bool backend_reconstructed = MaybeReconstructServerForBackend();

  // Pre-check: will CommitConfig rebuild consumers (destroying old buffers)?
  // Only renderer layout changes (resolution/lens/view/visible/filter) trigger rebuild.
  // High-frequency slider changes (crystal/sun/scattering) always reuse consumers.
  // If reuse is expected, skip Poller Stop — the poller's raw pointers remain valid.
  // NeedsRebuild comparison: RenderConfig::id was removed during the renderer copy-model
  // migration; since `id` was always 1 pre-migration, operator== behavior is strictly looser
  // and cannot produce new false-positive reuse paths.
  bool expect_rebuild = backend_reconstructed || !g_state.last_committed_state.has_value() ||
                        g_state.renderer != g_state.last_committed_state->renderer;

  // Single typed-struct commit path (327.4): all filter types — including multi-segment
  // raypath / multi-value EE (expanded to N simple + 1 complex) — go through the C struct, so
  // the out_reused signal is always available (no more JSON path that forced full rebuild and
  // tore live-preview buffers on every EE/multi-segment slider drag).
  //
  // Build the struct BEFORE stopping the poller: if a filter exceeds the ABI bounds
  // (composition pool / clause / filter capacity), keep the previously committed state
  // (graceful degradation) — poller untouched, buffer not torn — rather than commit a
  // truncated config.
  LUMICE_Config config{};
  lumice::ConfigColorGuard config_color_guard(config);  // v4.8: release raypath_color on any early return
  FilterOverflowInfo overflow;
  ColorClassOverflowInfo color_overflow;
  if (!FillLumiceConfig(g_state, &config, &overflow, &color_overflow)) {
    // color_overflow.class_index >= 0 iff FillLumiceConfig failed on the color-class walk
    // (which runs strictly after the filter walk), rather than a physical-filter overflow.
    // Reusing the filter-overflow wording/limit for a color overflow would misattribute the
    // resource type and print the wrong cap number to the user (code-review-01 Major 1).
    std::string warning_msg;
    std::string log_locator;
    if (color_overflow.class_index >= 0) {
      log_locator = FormatColorOverflowLocator(color_overflow);
      // User-visible prompt (the Log panel is collapsed by default): the edit did NOT apply
      // and the previous configuration was kept, so the user knows why nothing changed.
      warning_msg = "This raypath color configuration exceeds its limits (" + log_locator +
                    ").\nThe previous configuration was kept. Simplify the color configuration and try again.";
    } else {
      // Locator ("filter \"NAME\", Layer L / Entry E", or "Layer L / Entry E" when unnamed)
      // identifying which filter reference tripped the ABI bounds, captured inside
      // FillLumiceConfig. Built by FormatOverflowLocator so the format is unit-testable
      // (test_gui_import_export.cpp) rather than only exercised through on-screen GUI.
      log_locator = FormatOverflowLocator(overflow);
      warning_msg = "This filter has too many OR segments / values to apply (limit " +
                    std::to_string(LUMICE_MAX_CONFIG_CLAUSES) + "; " + log_locator +
                    ").\nThe previous configuration was kept. Simplify the filter and try again.";
    }
    // Log-panel dedup: gate the Log write on the SAME state SetGuiWarning uses for modal dedup
    // (in-flight message equals the new one). Sharing the key intentionally couples the two
    // channels — a wording change (e.g. new Layer/Entry after the user switches which filter
    // triggers the overflow) counts as a fresh event and re-logs. See plan.md §7 risk 1 for
    // why we did NOT introduce a separate `g_last_overlimit_log_msg` static variable.
    if (PeekGuiWarning() != warning_msg) {
      GUI_LOG_WARNING("[GUI] DoRun: {} exceeds ABI limits ({}); keeping the previous configuration.",
                      color_overflow.class_index >= 0 ? "raypath color configuration" : "filter", log_locator);
    }
    SetGuiWarning(warning_msg);
    return;
  }

  if (expect_rebuild) {
    g_server_poller.Stop();  // Must pause before consumer destruction
  }

  // Backend preference is now a construction-time property of the server
  // (see MaybeReconstructServerForBackend) — no per-DoRun SetPreferredBackend push.

  int reused = 0;
  LUMICE_ErrorCode err = LUMICE_CommitConfigStruct(g_server, &config, &reused);
  if (err == LUMICE_OK) {
    // A good commit clears any prior over-bounds warning so a later re-occurrence warns again.
    ClearGuiWarning();
    g_state.last_committed_state = GuiState::ConfigSnapshot::From(g_state);
    // Intent + epoch readback (I1): the synchronous commit above has already minted (or reused)
    // the lifecycle epoch; read it back so ReconcileSimState keys the display on THIS generation.
    // A filter change always rebuilds (epoch++), so the new epoch strictly exceeds any
    // display_epoch_floor MarkFilterDirty raised — the floor lifts itself with no explicit unlock.
    LUMICE_SimLifecycleResult lc{};
    LUMICE_GetSimLifecycle(g_server, &lc);
    g_state.committed_epoch = lc.epoch;
    g_state.run_intent = RunIntent::kRunning;
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
    // No staged-texture invalidate here: the epoch floor (raised by MarkFilterDirty) already
    // fences the old generation's payloads, and the freshly committed epoch (read back above)
    // lifts that fence. Crystal-scrub reuse (no filter change) MUST keep carry-forward, so we
    // deliberately do not drop the staged texture.
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
  // Idempotent: a stop is already draining on the background thread (kStopping intent-latched).
  // Re-entry (double-click, or a UI path that slipped past the disabled button) is a no-op — the
  // future is single-owner and must not be overwritten while pending.
  if (g_state.run_intent == RunIntent::kStopping) {
    return;
  }
  // Optimistic async Stop (blueprint §5/§8): set the intent synchronously so the UI paints
  // "Stopping…" THIS frame, then offload the EXISTING blocking teardown sequence onto a background
  // thread. The backend cannot interrupt the in-flight batch, so LUMICE_StopServer blocks until it
  // drains (server.cpp active_workers_==0); doing it off the UI thread keeps the toolbar live.
  // This writes the INTENT only — sim_state stays single-owner (ReconcileSimState in SyncFromPoller).
  g_state.run_intent = RunIntent::kStopping;
  g_stop_inflight.store(true);
  g_stop_future = std::async(std::launch::async, [srv = g_server] {
    g_server_poller.Stop();  // fast: wait out one readback poll (reuses the validated stop order)
    LUMICE_StopServer(srv);  // slow: blocks on the in-flight batch drain — now off the UI thread
    g_stop_inflight.store(false);
  });
  GUI_LOG_INFO("[GUI] DoStop: stopping (async)");
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
    // order remains correct (callback sees fully restored state). Runtime state (run_intent,
    // committed_epoch, poller counters, etc.) is intentionally preserved by ApplyTo.
    snapshot.ApplyTo(g_state);
    g_thumbnail_cache.OnLayerStructureChanged();
    // Revert restores config == committed, so it is no longer dirty. This is load-bearing under
    // the single-owner reconcile: with the old direct sim_state=kDone write gone, a leftover
    // dirty=true would make ReconcileSimState re-derive kDone+dirty → kModified every frame
    // (R5 — the revert would appear not to take). Clearing dirty settles it back to kDone.
    g_state.dirty = false;
  }
}

// Terminal display state a drained Stop resolves to (single flip point). Blueprint §8 originally
// abstracted this as "Idle", but that was reconsidered to kDone (2026-07-03, doc §8): the GUI's Save guards are
// keyed on kIdle (RefreshCpuTextureForSave early-return + Save-menu `has_server`), so Stop→Idle
// would regress save-after-stop, and kIdle would be overloaded with "never-run" vs
// "stopped-with-result". kDone == current terminal behavior, so the Save guards need no migration.
// A future flip back to kIdle is this one line + the Save-guard migration.
constexpr GuiState::SimState kStopEndState = GuiState::SimState::kDone;

// Single-owner sim_state reconcile (I2). Pure — see app.hpp for the contract. The §1.3 truth
// table: intent picks a base state; a fresh COMPLETED observation is the only thing that pulls a
// kRunning intent to kDone; a dirty edit on a kDone result surfaces as kModified.
GuiState::SimState ReconcileSimState(RunIntent intent, uint64_t committed_epoch, const PreviewSnapshot* snap,
                                     bool dirty) {
  using S = GuiState::SimState;
  // "Fresh" = the observation belongs to THIS committed generation (I1). A stale epoch (< committed,
  // e.g. an old snapshot or a transient poll after a re-commit bumped the epoch) is discarded so a
  // prior generation's COMPLETED never masquerades as this generation's completion. epoch > committed
  // cannot happen (the GUI reads the epoch back synchronously on commit).
  const bool fresh = snap != nullptr && snap->valid && snap->epoch == committed_epoch;
  S base;
  switch (intent) {
    case RunIntent::kNone:
      base = S::kIdle;  // fresh session / JSON import / .lmc without a baked texture
      break;
    case RunIntent::kLoaded:
      base = S::kDone;  // static loaded result (no server run backing it)
      break;
    case RunIntent::kStopping:
      base = S::kStopping;  // optimistic: async Stop is draining the in-flight batch (pure intent —
                            // does NOT depend on a fresh observation, and is not demoted by dirty)
      break;
    case RunIntent::kStopped:
      base = kStopEndState;  // drained terminal state; single flip point (see kStopEndState above)
      break;
    case RunIntent::kRunning:
      base = (fresh && snap->lifecycle == LUMICE_LIFECYCLE_COMPLETED) ? S::kDone : S::kSimulating;
      break;
  }
  // A dirty edit layered on a completed result is kModified (old MarkDirty kDone→kModified rule).
  // kIdle/kSimulating are never demoted by dirty.
  return (base == S::kDone && dirty) ? S::kModified : base;
}

// Display upload gate (I1/I6) — pure predicate, see app.hpp. Uploads a payload only when it is a
// fresh (unseen serial), non-empty frame (intensity>0 or a valid zero-ray terminal frame) whose
// epoch clears display_epoch_floor. The floor (raised by MarkFilterDirty) fences the old
// generation; a null / cold-start payload is skipped so GL keeps the last frame (no black flicker).
bool ShouldUploadPayload(const PreviewSnapshot& snap, unsigned long long last_uploaded_texture_serial,
                         uint64_t display_epoch_floor) {
  const auto& payload = snap.payload;
  return payload != nullptr && snap.texture_serial != last_uploaded_texture_serial &&
         (payload->snapshot_intensity > 0 || payload->texture_ray_count > 0) &&
         payload->payload_epoch > display_epoch_floor;
}

void SyncFromPoller() {
  // Atomically load the whole immutable snapshot (invariant I5). One lock-free atomic_load; the
  // consumer never observes a half-updated field combination.
  auto snap = g_server_poller.LoadSnapshot();

  // Level-triggered single-owner reconcile (I2/I3): unconditionally derive sim_state every frame
  // from (intent, committed_epoch, observation, dirty). No early-return on !=kSimulating — the
  // durable COMPLETED snapshot self-heals to kDone on any future frame even after the poller
  // self-pauses, and loaded/idle states also need per-frame derivation. This is the ONLY sim_state
  // writer in production.
  const auto prev_state = g_state.sim_state;
  g_state.sim_state = ReconcileSimState(g_state.run_intent, g_state.committed_epoch, snap.get(), g_state.dirty);
  if (prev_state != SimState::kDone && g_state.sim_state == SimState::kDone) {
    GUI_LOG_INFO("[GUI] Simulation done");
  }

  // Mealy next-intent advance for the async Stop (blueprint §5/§8): once the background stop thread
  // has drained (g_stop_inflight cleared, i.e. LUMICE_StopServer returned), promote the optimistic
  // kStopping intent to the terminal kStopped. This writes the INTENT (a reconcile INPUT) only —
  // sim_state is still written in exactly one place above (I2 preserved). Placed AFTER the reconcile
  // so the terminal kStopEndState paints on the NEXT frame's reconcile.
  if (g_state.run_intent == RunIntent::kStopping && !g_stop_inflight.load()) {
    g_state.run_intent = RunIntent::kStopped;
  }

  if (!snap || !snap->valid) {
    return;  // No observation yet (or a valid=false restart reset). State already reconciled.
  }

  // Stats: apply only when the observation matches the committed generation (I1), so a stale
  // bundle can't backfill the status bar with a prior run's counts.
  if (snap->epoch == g_state.committed_epoch && snap->stats_sim_ray_num > 0) {
    g_state.stats_ray_seg_num = snap->stats_ray_seg_num;
    g_state.stats_sim_ray_num = snap->stats_sim_ray_num;
  }

  // Upload XYZ float texture (GL call — must be on main thread). Gate is the pure ShouldUploadPayload
  // predicate (epoch-keyed, see §3). Exact-once is enforced by serial dedup; the cursor now lives in
  // GuiState (migrated from a file-scope static in 1.4).
  if (ShouldUploadPayload(*snap, g_state.last_uploaded_texture_serial, g_state.display_epoch_floor)) {
    const auto& payload = snap->payload;
    GUI_LOG_VERBOSE("[GUI] SyncFromPoller: upload tex_rays={}, intensity={:.6f}, eff_pixels={}, factor={:.6f}",
                    payload->texture_ray_count, payload->snapshot_intensity, payload->effective_pixels,
                    payload->intensity_factor);
    // task-342.4 Step 3: choose sRGB uint8 vs XYZ float upload path based on
    // raypath_color activation. When active, poller populates rgb_data with the
    // core-composited sRGB (host-side composite path, no shader change); when
    // not, we stay on the original XYZ float path bit-for-bit. Auto-EV / stats
    // / serial-dedup all keep reading xyz_data-derived fields (payload->p99_y,
    // snapshot_intensity, effective_pixels) below — those are computed on the
    // XYZ buffer, populated in both branches, so this pipe is not disturbed
    // (plan §3 keypoint 3).
    if (payload->is_composite) {
      g_preview.UploadTexture(payload->rgb_data.data(), payload->width, payload->height);
    } else {
      g_preview.UploadXyzTexture(payload->xyz_data.data(), payload->width, payload->height);
    }
    g_state.snapshot_intensity = payload->snapshot_intensity;
    g_state.effective_pixels = payload->effective_pixels;
    g_state.texture_upload_count++;
    g_state.last_uploaded_texture_serial = snap->texture_serial;  // record exact-once cursor
    // Auto-EV: visible framebuffer self-P99 normalization (see doc/adaptive-brightness.md).
    g_state.p99_raw_y = payload->p99_y;
    g_state.ev_auto = ComputeEvAuto(g_state.p99_raw_y, g_state.snapshot_intensity, g_state.target_white);
    GUI_LOG_VERBOSE("[GUI] SyncFromPoller: p99_raw_y={:.6f}, ev_auto={:.3f}", g_state.p99_raw_y, g_state.ev_auto);
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
