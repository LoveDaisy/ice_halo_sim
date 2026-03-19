#include "gui/app.hpp"

#include <GLFW/glfw3.h>
#include <spdlog/spdlog.h>
#include <stb_image.h>

#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>

#include "gui/file_io.hpp"

namespace lumice::gui {

using SimState = GuiState::SimState;

// Global state definitions
GuiState g_state;
PreviewRenderer g_preview;
CrystalRenderer g_crystal_renderer;
LUMICE_Server* g_server = nullptr;
ServerPoller g_server_poller;
bool g_panel_collapsed = false;
PreviewViewport g_preview_vp;

int g_programmatic_resize = 0;
float g_aspect_bar_height = 30.0f;  // Updated each frame by RenderPreviewPanel

bool g_show_unsaved_popup = false;
PendingAction g_pending_action = PendingAction::kNone;

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

  float bar_h = g_aspect_bar_height;
  float preview_w = std::max(1.0f, static_cast<float>(win_w) - kLeftPanelWidth);
  float preview_h = preview_w / ratio;
  auto target_h = static_cast<int>(preview_h + kTopBarHeight + kStatusBarHeight + bar_h);
  int target_w = win_w;

  int work_x = 0;
  int work_y = 0;
  int work_w = 0;
  int work_h = 0;
  glfwGetMonitorWorkarea(glfwGetPrimaryMonitor(), &work_x, &work_y, &work_w, &work_h);

  target_w = std::clamp(target_w, kMinWindowWidth, work_w);
  target_h = std::clamp(target_h, kMinWindowHeight, work_h);

  // If height was clamped, recalculate width to maintain ratio
  float actual_preview_h = static_cast<float>(target_h) - kTopBarHeight - kStatusBarHeight - bar_h;
  if (actual_preview_h > 0.0f) {
    float actual_preview_w = actual_preview_h * ratio;
    int recalc_w = static_cast<int>(actual_preview_w + kLeftPanelWidth);
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
// When in XYZ mode, tex_data_ is stale — refresh via old CPU-conversion API.
static void RefreshCpuTextureForSave() {
  if (!g_server || g_state.sim_state == SimState::kIdle) {
    return;  // No simulation data to refresh
  }
  LUMICE_RenderResult renders[2]{};
  LUMICE_GetRenderResults(g_server, renders, 1);
  if (renders[0].img_buffer != nullptr) {
    g_preview.UploadTexture(renders[0].img_buffer, renders[0].img_width, renders[0].img_height);
  }
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
  }
}

void DoSaveAs() {
  auto path = ShowSaveDialog();
  if (!path.empty()) {
    g_state.current_file_path = path;
    RefreshCpuTextureForSave();
    if (SaveLmcFile(path, g_state, g_preview, g_state.save_texture)) {
      g_state.dirty = false;
    }
  }
}

void DoExportPreviewPng() {
  auto path = ShowExportPngDialog();
  if (!path.empty()) {
    ExportPreviewPng(path.c_str(), g_preview, g_preview_vp);
  }
}

void DoExportEquirectPng() {
  if (!g_server) {
    return;
  }
  auto path = ShowExportEquirectDialog();
  if (path.empty()) {
    return;
  }
  // Get CPU-side sRGB render result (triggers DoSnapshot + PostSnapshot)
  LUMICE_RenderResult renders[2]{};
  LUMICE_GetRenderResults(g_server, renders, 1);
  if (renders[0].img_buffer == nullptr || renders[0].img_width <= 0 || renders[0].img_height <= 0) {
    return;
  }
  // Copy buffer (pointer valid only until next GetRenderResults/CommitConfig)
  size_t size = static_cast<size_t>(renders[0].img_width) * renders[0].img_height * 3;
  std::vector<unsigned char> buffer(renders[0].img_buffer, renders[0].img_buffer + size);
  ExportEquirectPng(path.c_str(), buffer.data(), renders[0].img_width, renders[0].img_height);
}

void DoExportConfigJson() {
  auto path = ShowExportJsonDialog();
  if (!path.empty()) {
    auto json_str = SerializeCoreConfig(g_state);
    ExportConfigJson(path.c_str(), json_str);
  }
}

// Helper: load image from path, downsample if needed, upload to bg texture.
// Returns true on success.
static bool LoadAndUploadBgImage(const std::string& path) {
  int w = 0;
  int h = 0;
  int channels = 0;
  unsigned char* raw = stbi_load(path.c_str(), &w, &h, &channels, 3);  // Force 3 channels (RGB)
  if (!raw) {
    spdlog::warn("Failed to load background image: {}", path);
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
  if (!path.empty()) {
    std::vector<unsigned char> tex_data;
    int tex_w = 0;
    int tex_h = 0;
    if (LoadLmcFile(path, g_state, tex_data, tex_w, tex_h)) {
      g_state.current_file_path = path;
      g_state.dirty = false;
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
}

void DoNew() {
  g_state = InitDefaultState();
  g_preview.ClearTexture();
  g_preview.ClearBackground();
  g_crystal_mesh_id = -1;
  g_crystal_mesh_hash = 0;
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

void DoRun() {
  if (!g_server) {
    return;
  }
  LUMICE_Config config{};
  FillLumiceConfig(g_state, &config);
  auto err = LUMICE_CommitConfigStruct(g_server, &config);
  if (err == LUMICE_OK) {
    g_state.last_committed_state = GuiState::ConfigSnapshot{
      g_state.crystals,
      g_state.selected_crystal,
      g_state.sun,
      g_state.sim,
      g_state.scattering,
      g_state.renderers,
      g_state.selected_renderer,
      g_state.filters,
      g_state.selected_filter,
      g_state.next_crystal_id,
      g_state.next_renderer_id,
      g_state.next_filter_id,
    };
    g_state.sim_state = SimState::kSimulating;
    g_state.stats_ray_seg_num = 0;
    g_state.stats_sim_ray_num = 0;
    g_server_poller.Start(g_server);  // Always restart: restart path stops server briefly
  } else {
    fprintf(stderr, "[GUI] CommitConfig FAILED with error code %d\n", err);
  }
}

void DoStop() {
  if (!g_server)
    return;
  g_server_poller.Stop();  // Must stop poller before server to avoid dangling access
  LUMICE_StopServer(g_server);
  g_state.sim_state = SimState::kDone;
}

void DoRevert() {
  if (g_state.last_committed_state) {
    const auto& snapshot = *g_state.last_committed_state;
    // Restore configuration fields only, preserve runtime state
    g_state.crystals = snapshot.crystals;
    g_state.selected_crystal = snapshot.selected_crystal;
    g_state.sun = snapshot.sun;
    g_state.sim = snapshot.sim;
    g_state.scattering = snapshot.scattering;
    g_state.renderers = snapshot.renderers;
    g_state.selected_renderer = snapshot.selected_renderer;
    g_state.filters = snapshot.filters;
    g_state.selected_filter = snapshot.selected_filter;
    g_state.next_crystal_id = snapshot.next_crystal_id;
    g_state.next_renderer_id = snapshot.next_renderer_id;
    g_state.next_filter_id = snapshot.next_filter_id;
    g_state.sim_state = SimState::kDone;
  }
}

void SyncFromPoller() {
  if (g_state.sim_state != SimState::kSimulating) {
    return;
  }

  PollerData data;
  if (!g_server_poller.TrySyncData(data)) {
    return;  // Poller busy writing — skip this frame
  }
  if (!data.valid) {
    return;  // Worker hasn't produced data yet
  }

  // Update simulation state
  if (data.server_state == LUMICE_SERVER_IDLE) {
    g_state.sim_state = SimState::kDone;
    fprintf(stderr, "[GUI] Simulation done\n");
  }

  // Update stats
  if (data.stats_sim_ray_num > 0) {
    g_state.stats_ray_seg_num = data.stats_ray_seg_num;
    g_state.stats_sim_ray_num = data.stats_sim_ray_num;
  }

  // Upload XYZ float texture (GL call — must be on main thread).
  // Skip when snapshot_intensity <= 0 (no rays accumulated yet after restart) to avoid
  // feeding zero-intensity data to the shader, which would cause a black frame flash.
  if (data.has_new_texture && g_state.selected_renderer >= 0 && data.snapshot_intensity > 0) {
    g_preview.UploadXyzTexture(data.xyz_data.data(), data.texture_width, data.texture_height);
    g_state.snapshot_intensity = data.snapshot_intensity;
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
