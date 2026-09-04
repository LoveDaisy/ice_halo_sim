#ifndef LUMICE_GUI_EXPORT_FBO_RENDERER_HPP
#define LUMICE_GUI_EXPORT_FBO_RENDERER_HPP

#include <vector>

#include "gui/overlay_labels.hpp"
#include "gui/preview_renderer.hpp"

namespace lumice::gui {

// Device pixels -> logical points, the conversion that has to agree with `dpi_scale_*` above.
// One implementation because it is exactly the shape this pairing exists to remove: the same two
// lines living in two files, one of them carrying the `dpi > 0` guard and the other not, is how
// the halved export text got in. A non-positive DPI is a surface that never reported one, and for
// it points and pixels are the same thing.
inline float DeviceToLogical(int device_px, float dpi) {
  return dpi > 0.0f ? static_cast<float>(device_px) / dpi : static_cast<float>(device_px);
}

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
//   - `curve_labels` carries every label the export draws: one set per annotation family — the
//     horizon, the angular-distance circles and the coordinate grid — each with its own colour and
//     collision group, all of them core's anchors. A LIST rather than a set per family because a
//     fourth family joining is an entry, not another parameter, and because the collision pass
//     already reads the group off each set. An EMPTY list is how a caller says "no text": there is
//     no second, GUI-walked source to gate separately any more, so the list is the whole decision.
//     Defaulted, so a caller wanting none states nothing.
//   - `dpi_scale_*` is the point-to-device-pixel ratio of the target surface: `dst_w`/`dst_h` are
//     DEVICE pixels, while `curve_labels`' anchors and the text drawn from them live in LOGICAL
//     POINTS. The glyph's device-pixel height is the baked font size times this scale and nothing
//     else, so a caller that hands device-pixel anchors and leaves this at 1.0 gets text at half
//     size on a Retina display while the positions still look right — that asymmetry was the
//     defect this parameter exists to make impossible to reintroduce. Defaulted to 1.0 for the
//     callers whose target has no DPI of its own (the reference-image and unit-test paths, which
//     name their own canvas size in pixels and pass no labels at all).
std::vector<unsigned char> RenderExportToRgba(PreviewRenderer& renderer, const PreviewParams& params, int dst_w,
                                              int dst_h, const std::vector<CurveLabelSet>& curve_labels = {},
                                              float dpi_scale_x = 1.0f, float dpi_scale_y = 1.0f);

// Render one on-screen preview frame through the SAME core the export above uses, then blit it
// back onto the caller's framebuffer at the viewport rect `(dst_x, dst_y, dst_w, dst_h)`.
//
// This is the whole point of the pairing: image, overlay lines and overlay labels reach the screen
// and the exported PNG through one function, so "the screenshot shows what the screen shows" is a
// property of the structure rather than of two paths kept in step by hand. There is no second
// label drawing path to gate, to scale, or to forget.
//
// Contract (differences from RenderExportToRgba only):
//   - The FBO is PERSISTENT across calls — this runs every frame — and is reallocated only when
//     `(dst_w, dst_h)` changes. Call DestroyPreviewFrameFbo() before the GL context goes away.
//   - dst_w/dst_h <= 0 is a no-op (a collapsed panel or a minimized window produces it every
//     frame, and glRenderbufferStorage with a zero extent is undefined).
//   - The FBO is cleared with the CALLER'S current clear colour, not with black: the region this
//     blit overwrites used to be painted by the caller's own glClear, and any pixel the preview
//     shader does not reach must keep the colour it had before this function existed.
void RenderPreviewFrameAndBlit(PreviewRenderer& renderer, const PreviewParams& params, int dst_x, int dst_y, int dst_w,
                               int dst_h, const std::vector<CurveLabelSet>& curve_labels, float dpi_scale_x,
                               float dpi_scale_y);

// Run the shared tail of the frame loop — the stretch both the production app (src/gui/main.cpp)
// and the gui_test harness (test/gui/test_gui_main.cpp) execute identically, from clearing the
// default framebuffer through submitting ImGui's draw data: set the GL viewport, clear, render and
// blit the preview through RenderPreviewFrameAndBlit if one is active, then submit ImGui's draw
// data via ImGui_ImplOpenGL3_RenderDrawData. Every step but the preview blit is UNCONDITIONAL —
// the preview is one interior branch on `g_preview_vp.active`, not a precondition for calling
// this: a caller with no preview still needs the clear and the draw-data submission.
//
// This is the ONLY place that sequence exists. It used to live twice, character for character,
// kept in step by a comment in the harness copy saying it "matches real app's main.cpp". Adding
// RenderPreviewFrameAndBlit (PR #304) had to reach both copies, and the second one was found by
// hand, mid-change, because nothing would have reported missing it: had it been left out, gui_test
// would have gone on passing while rendering its preview through a frame loop the app no longer
// had, and every on-screen reference image would have been captured from that stale loop. A
// comment cannot go red; a single implementation cannot be half-updated.
//
// Deliberately NOT folded in, both for the same reason — the two callers do these at DIFFERENT
// points of their own frame, so folding them would silently reorder one of them:
//   - ImGui::Render() — main.cpp queries the framebuffer size before its panels run and calls
//     Render() after; test_gui_main.cpp calls Render() first and queries after. Each caller keeps
//     its own call, immediately next to its own query, so neither ordering moves.
//   - The display_w/display_h query itself — passed in as parameters rather than queried here. In
//     main.cpp the query happens before the panels run, which matters: a panel can call
//     glfwSetWindowSize() (aspect-ratio preset) mid-frame, and what glfwGetFramebufferSize()
//     reports between that call and the next glfwPollEvents() is not guaranteed across backends.
//
// Reads global state: `g_preview` / `g_preview_vp`. Both callers already read that same pair here
// today, so this adds no coupling that was not already on this code path.
void RunSharedFrameRenderPass(int display_w, int display_h);

// Release the persistent FBO held by RenderPreviewFrameAndBlit. Must be called on the GL thread
// while the context is still current, next to the other renderer teardown calls.
void DestroyPreviewFrameFbo();

// Partial-override helpers for export paths. Both assume `params` already carries the
// live EV-synced `intensity_factor` / `intensity_scale` from `BuildExportParams()` and
// overwrite the 11 export-specific fields that must not inherit viewport dirty state.
//
// Shared between production (`app.cpp::DoExport*Png`) and the tests that exercise them
// (`test/unit-correctness/gui/test_export_params.cpp` asserts field-by-field what each
// preset may and may not overwrite; `test/gui/functional/test_export.cpp` calls the same
// two functions on the pixel path) so new `PreviewParams` fields only need to be audited
// in one place — replaces the earlier "Must stay in sync" comment contract.
//
// Required by `sampleDualFisheye`: both variants keep `max_abs_dz` / `r_scale` aligned
// with `kDualFisheyeOverlap` because the equirect shader samples the dual-fisheye
// source texture format.
void ConfigureDualFisheyeExportParams(PreviewParams& params);
void ConfigureEquirectExportParams(PreviewParams& params);

}  // namespace lumice::gui

#endif  // LUMICE_GUI_EXPORT_FBO_RENDERER_HPP
