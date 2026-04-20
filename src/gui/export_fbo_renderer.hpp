#ifndef LUMICE_GUI_EXPORT_FBO_RENDERER_HPP
#define LUMICE_GUI_EXPORT_FBO_RENDERER_HPP

#include <optional>
#include <vector>

#include "gui/overlay_labels.hpp"
#include "gui/preview_renderer.hpp"

namespace lumice::gui {

// Render a preview image to an off-screen FBO and return the raw RGBA8 pixels
// (row-major, top-down; same orientation stbi_write_png expects).
//
// When `overlay_input` has a value, overlay labels (horizon / grid / sun circles) are
// composited on top using ImGui's OpenGL3 backend via a self-owned ImDrawList. The
// FBO target guarantees no UI chrome from the default framebuffer contaminates the
// output (fixes gui-polish-v10 Bug 2), and intensity uniforms on `params` are
// sampled live so exposure tracks the GUI EV slider (fixes gui-polish-v10 Bug 1).
//
// Returns an empty vector on failure (invalid size, FBO incomplete, readback error).
// Caller owns the returned buffer.
//
// Contract:
//   - Must be called from the thread that owns the GL context (main render thread).
//   - Preserves the caller's FBO binding and viewport.
//   - dst_w/dst_h must satisfy 0 < dst_{w,h} <= GL_MAX_FRAMEBUFFER_{WIDTH,HEIGHT}.
std::vector<unsigned char> RenderExportToRgba(PreviewRenderer& renderer, const PreviewParams& params, int dst_w,
                                              int dst_h, const std::optional<OverlayLabelInput>& overlay_input);

}  // namespace lumice::gui

#endif  // LUMICE_GUI_EXPORT_FBO_RENDERER_HPP
