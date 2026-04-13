#ifndef LUMICE_GUI_THUMBNAIL_CACHE_HPP
#define LUMICE_GUI_THUMBNAIL_CACHE_HPP

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "gui/crystal_renderer.hpp"

namespace lumice::gui {

struct GuiState;

// Manages per-entry crystal thumbnail textures rendered offscreen.
// Uses a shared CrystalRenderer instance (compiled once) and per-entry GL textures.
// Dirty detection is event-driven: callers must call Invalidate() or OnLayerStructureChanged()
// when crystal params / axis presets / layer structure change.
class ThumbnailCache {
 public:
  // Initialize the shared thumbnail renderer. Must be called after GL context is ready.
  bool Init();

  // Release all GL resources. Must be called before GL context is destroyed.
  void Destroy();

  // Look up the thumbnail texture for the given entry.
  // - Cache miss: creates a dirty entry, enqueues for rendering, returns 0.
  // - Cache hit + dirty: returns 0 (pending update).
  // - Cache hit + clean: returns the GL texture ID.
  // Idempotent: repeated calls for the same entry in the same frame do not duplicate enqueue.
  uintptr_t GetTexture(int layer_idx, int entry_idx);

  // Process up to max_updates entries from the update queue.
  // For each entry: builds mesh, determines pose rotation, renders, blits to entry texture.
  // Skips entries whose indices are out of bounds (structure may have changed).
  // Must be called once per frame, BEFORE card rendering.
  void ProcessUpdateQueue(const GuiState& state, int max_updates);

  // Mark a specific entry as dirty and enqueue for re-rendering.
  void Invalidate(int layer_idx, int entry_idx);

  // Mark all entries as dirty and enqueue for re-rendering.
  void InvalidateAll();

  // Clear the entire cache and release all GL textures.
  // Called when layer/entry structure changes (add/delete/copy/reorder/file load).
  // Entries are lazily rebuilt on next GetTexture() call.
  void OnLayerStructureChanged();

 private:
  struct ThumbnailEntry {
    unsigned int texture = 0;
    bool dirty = true;
  };

  static uint64_t MakeKey(int layer_idx, int entry_idx) {
    return (static_cast<uint64_t>(layer_idx) << 32) | static_cast<uint32_t>(entry_idx);
  }

  void RenderThumbnail(int layer_idx, int entry_idx, const GuiState& state);

  CrystalRenderer renderer_;
  bool valid_ = false;
  std::unordered_map<uint64_t, ThumbnailEntry> cache_;
  std::vector<uint64_t> update_queue_;
};

}  // namespace lumice::gui

#endif  // LUMICE_GUI_THUMBNAIL_CACHE_HPP
