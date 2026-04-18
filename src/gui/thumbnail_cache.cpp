#include "gui/thumbnail_cache.hpp"

#include <cmath>
#include <cstring>

#include "gui/crystal_preview.hpp"
#include "gui/gl_common.h"
#include "gui/gui_constants.hpp"
#include "gui/gui_logger.hpp"
#include "gui/gui_state.hpp"
#include "gui/panels.hpp"

namespace lumice::gui {

namespace {
constexpr float kPi = 3.14159265358979323846f;

// Compute a column-major 4x4 rotation matrix for the given axis preset name.
// Convention matches CrystalRenderer::Render() rotation parameter:
// column-major, same layout as g_crystal_rotation (crystal_preview.cpp:15-17).
void GetThumbnailRotation(const std::string& preset_name, float rotation[16]) {
  // Start with identity
  std::memset(rotation, 0, 16 * sizeof(float));
  rotation[0] = 1.0f;
  rotation[5] = 1.0f;
  rotation[10] = 1.0f;
  rotation[15] = 1.0f;

  if (preset_name == "Column" || preset_name == "Parry") {
    // Side view: rotate +80° around X axis (nearly horizontal placement)
    constexpr float kAngle = 80.0f * kPi / 180.0f;
    float c = std::cos(kAngle);
    float s = std::sin(kAngle);
    // Rx(angle) column-major:
    // [1  0  0  0]
    // [0  c -s  0]
    // [0  s  c  0]
    // [0  0  0  1]
    rotation[5] = c;
    rotation[6] = s;
    rotation[9] = -s;
    rotation[10] = c;
  } else if (preset_name == "Plate") {
    // Top-down view: rotate -10° around X axis (slight tilt to see hexagonal outline)
    constexpr float kAngle = -10.0f * kPi / 180.0f;
    float c = std::cos(kAngle);
    float s = std::sin(kAngle);
    rotation[5] = c;
    rotation[6] = s;
    rotation[9] = -s;
    rotation[10] = c;
  } else if (preset_name == "Lowitz") {
    // Slightly tilted side view: rotate +60° around X axis
    constexpr float kAngle = 60.0f * kPi / 180.0f;
    float c = std::cos(kAngle);
    float s = std::sin(kAngle);
    rotation[5] = c;
    rotation[6] = s;
    rotation[9] = -s;
    rotation[10] = c;
  } else {
    // Random / Custom / unknown: isometric view (rotate +35° around X, then +25° around Y)
    if (preset_name != "Random" && preset_name != "Custom") {
      GUI_LOG_WARNING("Unknown axis preset for thumbnail: {}", preset_name);
    }
    constexpr float kAngleX = 35.0f * kPi / 180.0f;
    constexpr float kAngleY = 25.0f * kPi / 180.0f;
    float cx = std::cos(kAngleX);
    float sx = std::sin(kAngleX);
    float cy = std::cos(kAngleY);
    float sy = std::sin(kAngleY);
    // Ry * Rx (column-major):
    // [cy    sy*sx   sy*cx  0]
    // [0     cx      -sx    0]
    // [-sy   cy*sx   cy*cx  0]
    // [0     0       0      1]
    rotation[0] = cy;
    rotation[1] = 0.0f;
    rotation[2] = -sy;
    rotation[4] = sy * sx;
    rotation[5] = cx;
    rotation[6] = cy * sx;
    rotation[8] = sy * cx;
    rotation[9] = -sx;
    rotation[10] = cy * cx;
  }
}

}  // namespace

bool ThumbnailCache::Init() {
  if (!renderer_.Init(kThumbnailSize, kThumbnailSize)) {
    GUI_LOG_ERROR("Failed to initialize thumbnail renderer");
    valid_ = false;
    return false;
  }

  // Pre-allocate persistent FBOs for blit operations
  glGenFramebuffers(1, &blit_write_fbo_);
  glGenFramebuffers(1, &blit_read_fbo_);

  valid_ = true;
  return true;
}

void ThumbnailCache::Destroy() {
  // Drain any textures parked by OnLayerStructureChanged()
  if (!pending_deletes_.empty()) {
    glDeleteTextures(static_cast<int>(pending_deletes_.size()), pending_deletes_.data());
    pending_deletes_.clear();
  }

  // Release all per-entry textures
  for (auto& [key, entry] : cache_) {
    if (entry.texture) {
      glDeleteTextures(1, &entry.texture);
    }
  }
  cache_.clear();
  update_queue_.clear();

  // Release persistent blit FBOs
  if (blit_read_fbo_) {
    glDeleteFramebuffers(1, &blit_read_fbo_);
    blit_read_fbo_ = 0;
  }
  if (blit_write_fbo_) {
    glDeleteFramebuffers(1, &blit_write_fbo_);
    blit_write_fbo_ = 0;
  }

  renderer_.Destroy();
  valid_ = false;
}

uintptr_t ThumbnailCache::GetTexture(int layer_idx, int entry_idx) {
  if (!valid_) {
    return 0;
  }

  uint64_t key = MakeKey(layer_idx, entry_idx);
  auto it = cache_.find(key);
  if (it == cache_.end()) {
    // Cache miss: create dirty entry and enqueue
    cache_[key] = ThumbnailEntry{ 0, true };
    update_queue_.push_back(key);
    return 0;
  }

  if (it->second.dirty) {
    return 0;
  }

  return static_cast<uintptr_t>(it->second.texture);
}

void ThumbnailCache::ProcessUpdateQueue(const GuiState& state, int max_updates) {
  if (!valid_) {
    return;
  }

  // Drain textures parked by OnLayerStructureChanged() on non-main threads
  if (!pending_deletes_.empty()) {
    glDeleteTextures(static_cast<int>(pending_deletes_.size()), pending_deletes_.data());
    pending_deletes_.clear();
  }

  if (update_queue_.empty()) {
    return;
  }

  int processed = 0;
  while (!update_queue_.empty() && processed < max_updates) {
    uint64_t key = update_queue_.front();
    update_queue_.pop_front();

    int layer_idx = static_cast<int>(key >> 32);
    int entry_idx = static_cast<int>(key & 0xFFFFFFFF);

    // Bounds check: structure may have changed since enqueue
    if (layer_idx < 0 || layer_idx >= static_cast<int>(state.layers.size())) {
      continue;
    }
    if (entry_idx < 0 || entry_idx >= static_cast<int>(state.layers[layer_idx].entries.size())) {
      continue;
    }

    RenderThumbnail(layer_idx, entry_idx, state);
    processed++;
  }
}

void ThumbnailCache::Invalidate(int layer_idx, int entry_idx) {
  if (!valid_) {
    return;
  }

  uint64_t key = MakeKey(layer_idx, entry_idx);
  auto it = cache_.find(key);
  if (it != cache_.end()) {
    if (!it->second.dirty) {
      it->second.dirty = true;
      update_queue_.push_back(key);
    }
    // If already dirty, it's already in the queue — skip to avoid duplicates
  } else {
    // Entry not in cache yet; it will be created on next GetTexture() call
  }
}

void ThumbnailCache::InvalidateAll() {
  for (auto& [key, entry] : cache_) {
    if (!entry.dirty) {
      entry.dirty = true;
      update_queue_.push_back(key);
    }
  }
}

void ThumbnailCache::OnLayerStructureChanged() {
  // NOTE: may run on any thread (e.g. ImGui Test Engine coroutine via DoNew()),
  // so GL calls are forbidden here. Park textures for the main thread to delete
  // on the next ProcessUpdateQueue()/Destroy() tick.
  for (auto& [key, entry] : cache_) {
    if (entry.texture) {
      pending_deletes_.push_back(entry.texture);
    }
  }
  cache_.clear();
  update_queue_.clear();
}

void ThumbnailCache::RenderThumbnail(int layer_idx, int entry_idx, const GuiState& state) {
  const auto& crystal = state.layers[layer_idx].entries[entry_idx].crystal;

  // Build mesh data
  LUMICE_CrystalMesh mesh{};
  if (!BuildCrystalMeshData(crystal, &mesh)) {
    GUI_LOG_WARNING("Failed to build mesh for thumbnail ({}, {})", layer_idx, entry_idx);
    return;
  }

  // Defend against degenerate geometry
  if (mesh.vertex_count <= 0 || mesh.edge_count <= 0) {
    GUI_LOG_WARNING("Degenerate mesh for thumbnail ({}, {}): {} vertices, {} edges", layer_idx, entry_idx,
                    mesh.vertex_count, mesh.edge_count);
    return;
  }

  // Upload mesh to the shared thumbnail renderer
  renderer_.UpdateMesh(mesh.vertices, mesh.vertex_count, mesh.edges, mesh.edge_count, mesh.triangles,
                       mesh.triangle_count, mesh.edge_face_normals);

  // Determine rotation based on axis preset
  std::string preset = AxisPresetName(crystal);
  float rotation[16];
  GetThumbnailRotation(preset, rotation);

  // Render to the shared renderer's FBO
  constexpr float kThumbnailZoom = 1.4f;
  renderer_.Render(rotation, kThumbnailZoom, CrystalStyle::kHiddenLine);

  // Ensure per-entry texture exists
  uint64_t key = MakeKey(layer_idx, entry_idx);
  auto& entry = cache_[key];
  if (entry.texture == 0) {
    glGenTextures(1, &entry.texture);
    glBindTexture(GL_TEXTURE_2D, entry.texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, kThumbnailSize, kThumbnailSize, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glBindTexture(GL_TEXTURE_2D, 0);
  }

  // Blit from shared renderer's resolve texture to per-entry texture.
  // Save current framebuffer state.
  GLint prev_fbo = 0;
  glGetIntegerv(GL_FRAMEBUFFER_BINDING, &prev_fbo);

  // Attach entry texture to persistent write FBO
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, blit_write_fbo_);
  glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, entry.texture, 0);

  // Attach renderer's resolve texture to persistent read FBO
  auto renderer_tex = static_cast<unsigned int>(renderer_.GetTextureId());
  glBindFramebuffer(GL_READ_FRAMEBUFFER, blit_read_fbo_);
  glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, renderer_tex, 0);

  // GL_NEAREST: source and target are same size, no filtering needed.
  // GL_LINEAR would also trigger GL_INVALID_OPERATION if MSAA is enabled in future.
  glBlitFramebuffer(0, 0, kThumbnailSize, kThumbnailSize, 0, 0, kThumbnailSize, kThumbnailSize, GL_COLOR_BUFFER_BIT,
                    GL_NEAREST);

  // Detach textures to avoid dangling references
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, blit_write_fbo_);
  glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, 0, 0);
  glBindFramebuffer(GL_READ_FRAMEBUFFER, blit_read_fbo_);
  glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, 0, 0);

  // Restore previous framebuffer
  glBindFramebuffer(GL_FRAMEBUFFER, static_cast<GLuint>(prev_fbo));

  entry.dirty = false;
}

}  // namespace lumice::gui
