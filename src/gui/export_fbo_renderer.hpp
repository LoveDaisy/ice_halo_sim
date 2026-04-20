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
//   - Preserves the caller's FBO binding, viewport, and glClearColor.
//   - dst_w/dst_h must satisfy 0 < dst_{w,h} <= GL_MAX_RENDERBUFFER_SIZE.
//     (GL_MAX_FRAMEBUFFER_{WIDTH,HEIGHT} would be tighter but requires GL 4.3;
//      macOS OpenGL 3.3 Core exposes only GL_MAX_RENDERBUFFER_SIZE.)
std::vector<unsigned char> RenderExportToRgba(PreviewRenderer& renderer, const PreviewParams& params, int dst_w,
                                              int dst_h, const std::optional<OverlayLabelInput>& overlay_input);

// Partial-override helpers for export paths. Both assume `params` already carries the
// live EV-synced `intensity_factor` / `intensity_scale` from `BuildExportParams()` and
// overwrite the 11 export-specific fields that must not inherit viewport dirty state.
//
// Shared between production (`app.cpp::DoExport*Png`) and test helpers
// (`test_gui_export.cpp::Apply*Override`) so new `PreviewParams` fields only need to be
// audited in one place — replaces the earlier "Must stay in sync" comment contract.
//
// Required by `sampleDualFisheye`: both variants keep `max_abs_dz` / `r_scale` aligned
// with `kDualFisheyeOverlap` because the equirect shader samples the dual-fisheye
// source texture format.
void ConfigureDualFisheyeExportParams(PreviewParams& params);
void ConfigureEquirectExportParams(PreviewParams& params);

}  // namespace lumice::gui

#endif  // LUMICE_GUI_EXPORT_FBO_RENDERER_HPP
