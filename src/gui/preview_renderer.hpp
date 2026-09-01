#ifndef LUMICE_GUI_PREVIEW_RENDERER_HPP
#define LUMICE_GUI_PREVIEW_RENDERER_HPP

#include <array>
#include <vector>

#include "gui/gui_constants.hpp"
#include "gui/gui_state.hpp"

namespace lumice::gui {

inline constexpr float kOverlaySentinel = -9999.f;

// Source texture format. Dual-fisheye overlap parameters for sampling a
// packed front/back hemisphere texture. Owned by the producer (GUI hard-codes
// kDualFisheyeOverlap today; future: server/RawXyzResult may supply).
struct SourceFormat {
  float max_abs_dz = 0.0f;  // overlap zone threshold (0 = no blend)
  float r_scale = 1.0f;     // projection r_scale for overlap normalization
};

// View / projection parameters. Field order is part of the ABI for
// kDualFisheyeExportViewProj / kEquirectExportViewProj aggregate initializers
// defined just below — DO NOT REORDER fields; insertions must be mirrored in
// the aggregate initializers in the same edit.
struct ViewProjection {
  int lens_type = kLensTypeLinear;  // Index into kLensTypeNames (int for Core interop)
  float fov = 90.0f;                // Degrees
  float elevation = 0.0f;           // Degrees
  float azimuth = 0.0f;             // Degrees
  float roll = 0.0f;                // Degrees
  int visible = kVisibleFull;       // Index into kVisibleNames (int for shader uniform)
  bool front = false;               // Independent front-hemisphere clip flag
};

// Canonical ViewProjection values for export code paths. Kept adjacent to
// ViewProjection so aggregate-initializer field order can be verified at a
// glance. Field order, top-to-bottom: lens_type, fov, elevation, azimuth, roll, visible, front.
inline constexpr ViewProjection kDualFisheyeExportViewProj = {
  kLensTypeDualFisheyeEqualArea, 180.0f, 0.0f, 0.0f, 0.0f, kVisibleFull, false,
};
inline constexpr ViewProjection kEquirectExportViewProj = {
  kLensTypeRectangular, 180.0f, 0.0f, 0.0f, 0.0f, kVisibleFull, false,
};

struct Exposure {
  float intensity_factor = 1.0f;
  float intensity_scale = 0.0f;  // = intensity_factor / per_pixel_intensity (0 = not in XYZ mode)
};

// Auxiliary line overlay (horizon, altitude grid, sun circles) drawn on top
// of the preview.
//
// The show_* fields here control **line** rendering only (shader uniforms
// u_show_horizon / u_show_grid / u_show_sun_circles). They are sourced from
// GuiState::show_<x>_line. The companion fields GuiState::show_<x>_label are read where the
// label anchors are consumed (app_panels.cpp / app.cpp, through AnnotationOverlayCache), not here.
struct OverlayDecoration {
  bool show_horizon = false;
  bool show_grid = false;
  // The switch only. Where the circles are is no longer described here: it comes from the mask
  // PreviewRenderer::UploadAngularDistMask was last given, which the caller fills from its
  // AnnotationOverlayCache. sun_dir / sun_circle_angles / sun_circle_count used to live here and
  // were the shader's inputs for deriving the curve itself; core derives it now.
  bool show_sun_circles = false;
  // BORROWED for the duration of the Render call: core's angular-distance mask for this view,
  // row-major mask_w*mask_h with a top-left origin, or null for "none computed". Owned by the
  // caller's AnnotationOverlayCache. Passed rather than uploaded directly because the upload needs
  // a GL context and this struct is filled during the ImGui build phase; `generation` is what lets
  // Render() re-upload only when the mask is actually new instead of once per frame.
  const unsigned char* angular_dist_mask = nullptr;
  int angular_dist_mask_w = 0;
  int angular_dist_mask_h = 0;
  unsigned long long angular_dist_mask_generation = 0;
  // The coordinate grid's mask, on the same borrowed-for-the-call terms as the circles' above.
  // ONE mask for parallels and meridians together: they share this struct's single grid_color /
  // grid_alpha, so nothing downstream could tell the two apart even if they arrived separately.
  const unsigned char* grid_mask = nullptr;
  int grid_mask_w = 0;
  int grid_mask_h = 0;
  unsigned long long grid_mask_generation = 0;
  float horizon_color[3] = { 0.8f, 0.2f, 0.2f };
  float grid_color[3] = { 1.0f, 1.0f, 1.0f };
  float sun_circles_color[3] = { 1.0f, 0.9f, 0.3f };
  float horizon_alpha = 0.6f;
  float grid_alpha = 0.3f;
  float sun_circles_alpha = 0.5f;

  // Zenith / Nadir pixel-space ring marker.
  // *_screen_pos: CPU-precomputed; center-origin pixel coords, y-up
  // (matches shader's `pos = v_ndc * u_resolution * 0.5`). Sentinel
  // (-9999, -9999) when the direction is offscreen / behind the camera
  // / unsupported lens — shader compares distance and naturally skips it.
  bool show_zenith_nadir = false;
  float zenith_screen_pos[2] = { kOverlaySentinel, kOverlaySentinel };
  float nadir_screen_pos[2] = { kOverlaySentinel, kOverlaySentinel };
  float zenith_nadir_color[3] = { 0.8f, 0.2f, 0.2f };
  float zenith_nadir_alpha = 0.6f;
  float zenith_nadir_radius_px = 8.0f;

  // Lens border: outline of the projection's valid image circle. The shader
  // derives the geometry itself from lens type / FOV / resolution (see
  // overlayLensBorder), so unlike the zenith/nadir marker there is no
  // CPU-precomputed screen position or radius here.
  bool show_lens_border = false;
  float lens_border_color[3] = { 0.3f, 0.7f, 1.0f };
  float lens_border_alpha = 0.6f;

  static OverlayDecoration Disabled() { return {}; }
};

struct Background {
  bool enabled = false;
  float alpha = 1.0f;
  float aspect = 1.0f;
  // 2D pan + zoom of the background image within the viewport rectangle, sourced from
  // GuiState::bg_offset_x / bg_offset_y / bg_scale. `zoom` here is NOT ViewProjection::fov: it
  // only scales the background's UV transform (the photo the user is comparing against), and
  // never touches the simulated frame's camera. Both happen to be reachable from a scroll on the
  // canvas, told apart by whether the pan/zoom modifier key is held — see kBgModifierName below.
  // Identity is (0, 0, 1): centered contain fit.
  float pan_x = 0.0f;
  float pan_y = 0.0f;
  float zoom = 1.0f;

  static Background Disabled() { return {}; }
};

struct PreviewParams {
  SourceFormat source;
  ViewProjection view_proj;
  Exposure exposure;
  OverlayDecoration overlay;
  Background bg;

  // The sky colour painted behind the halo. Named apart from `bg` above on purpose: that one is
  // the background IMAGE overlay (a photo the user compares against, blended as a lerp that dims
  // the halo), this one is a background COLOUR added to the halo's radiance. Different blend laws
  // for different jobs — see the shader's u_background use and Background's own comment.
  //
  // LINEAR RGB, converted from GuiState::RenderConfig::background (which is sRGB, the numbers a
  // colour picker shows) by whoever fills this struct — app_panels.cpp for the live preview,
  // inherited unchanged by the three export entry points through BuildExportParams. Linear because
  // the addition has to happen before the sRGB transfer curve; the default of all zeroes makes the
  // addition a no-op, so a caller that never touches this field renders exactly as before.
  float background_color_linear[3] = { 0.0f, 0.0f, 0.0f };
};

class PreviewRenderer {
 public:
  bool Init();
  void Destroy();

  // Upload equirectangular image (RGB, uint8, row-major) — for .lmc load
  void UploadTexture(const unsigned char* data, int width, int height);

  // Upload equirectangular XYZ float data — for live simulation preview
  void UploadXyzTexture(const float* data, int width, int height);

  // Render preview into the given viewport region (in framebuffer pixels)
  void Render(int vp_x, int vp_y, int vp_w, int vp_h, const PreviewParams& params);

  // Hand the renderer core's angular-distance mask for the current view: row-major
  // width*height bytes, 1 where a circle passes, top-left origin. NOT per frame — the caller
  // uploads when its AnnotationOverlayCache produces a new result, and the texture persists
  // until the next one. A null / degenerate argument clears it.
  void UploadAngularDistMask(const unsigned char* data, int width, int height);
  void ClearAngularDistMask();

  // The same for the coordinate grid's mask, on texture unit 3. Same contract in every respect;
  // see UploadAngularDistMask above.
  void UploadGridMask(const unsigned char* data, int width, int height);
  void ClearGridMask();

  bool HasTexture() const { return tex_width_ > 0 && tex_height_ > 0; }
  void ClearTexture();

  // Update CPU-side texture data only (no GL upload, no xyz_mode_ change).
  // Used by Save to refresh tex_data_ without disturbing the GPU texture.
  void UpdateCpuTextureData(const unsigned char* data, int width, int height);

  // CPU-side texture data access (for .lmc file save)
  const unsigned char* GetTextureData() const { return tex_data_.empty() ? nullptr : tex_data_.data(); }
  int GetTextureWidth() const { return tex_width_; }
  int GetTextureHeight() const { return tex_height_; }

  // Background image texture management (GL upload only, no file I/O)
  void UploadBgTexture(const unsigned char* data, int width, int height);
  void ClearBackground();
  bool HasBackground() const { return bg_width_ > 0 && bg_height_ > 0; }
  float GetBgAspect() const { return bg_aspect_; }

 private:
  unsigned int shader_program_ = 0;
  unsigned int vao_ = 0;
  unsigned int vbo_ = 0;
  unsigned int texture_ = 0;
  int tex_width_ = 0;
  int tex_height_ = 0;
  std::vector<unsigned char> tex_data_;  // CPU-side copy of texture (RGB uint8, for .lmc save)
  bool xyz_mode_ = false;                // true when texture contains XYZ float data

  // Deferred GL blank request. ClearTexture() sets this from any thread
  // (callable from coroutine workers without a GL context); Render() (main
  // thread, GL context) consumes it by re-uploading a 1x1 black pixel into
  // texture_ so the sim layer stops sampling stale pixels. Any method that
  // writes fresh real pixel data into texture_ MUST clear this flag first,
  // so the newest real write always wins over a pending blank (invariant not
  // enforced by the compiler — grep this comment before adding a new upload).
  bool needs_gl_blank_ = false;

  void UploadBlankSimTexture();

  // PBO double-buffer for async XYZ texture upload (GLsync stored as void* to
  // avoid including GL headers in this header; cast to GLsync in the .cpp).
  std::array<unsigned int, 2> pbo_ = { 0, 0 };
  std::array<void*, 2> pbo_fence_ = { nullptr, nullptr };
  std::array<size_t, 2> pbo_byte_count_ = { 0, 0 };
  int pbo_index_ = 0;

  // Background image texture (no CPU-side copy — loaded from file path)
  unsigned int bg_texture_ = 0;
  // R8, one texel per viewport pixel: core's angular-distance mask for the last SETTLED view.
  // Zero when nothing has been uploaded, which the shader reads as "draw no circles" rather than
  // as an empty mask — the two are the same picture but only one of them is honest about why.
  unsigned int angular_dist_tex_ = 0;
  // Which AnnotationOverlayCache generation angular_dist_tex_ holds. 0 means "nothing uploaded",
  // which is why the cache's counter starts at 1.
  unsigned long long angular_dist_tex_generation_ = 0;
  // The coordinate grid's mask, same lifetime and same sentinel rules as the two above.
  unsigned int grid_tex_ = 0;
  unsigned long long grid_tex_generation_ = 0;
  int bg_width_ = 0;
  int bg_height_ = 0;
  float bg_aspect_ = 1.0f;
};

// Build view-to-world 3x3 rotation matrix from camera orientation (degrees).
// OpenGL column-major layout: out[col*3 + row].
// Synced with shader u_view_matrix usage (preview_renderer.cpp).
void BuildViewMatrix(float elevation_deg, float azimuth_deg, float roll_deg, float out[9]);

// Project a unit world-space direction to screen pixel coordinates matching the
// fragment shader's coordinate system: origin at viewport center, x right, y up,
// units = pixels (i.e. `pos = v_ndc * resolution * 0.5`). Returns sentinel
// {-9999.f, -9999.f} if the direction is behind the camera, outside the viewport,
// or the lens is unsupported (e.g. kLensTypeGlobe). Synced with the shader's
// inverse projection helpers (linearInverse / fisheyeInverse / dualFisheyeInverse
// / rectangularInverse) in preview_renderer.cpp.
std::array<float, 2> ProjectWorldDirToScreen(const ViewProjection& vp, const float world_dir[3], int vp_w, int vp_h);

// Preview drag sensitivity: how many degrees of azimuth/elevation one pixel of
// mouse motion should produce, so that content at the center of the frame moves
// a CONSTANT number of screen pixels per dragged pixel, for any lens/FOV/viewport.
// That constant is kDragSensitivity, calibrated with the implementation; the
// invariant this function exists for is that the ratio does not depend on FOV or
// viewport size, not the particular value it holds.
//
// This is the analytic inverse of the forward projections' angular resolution at
// theta=0, times kDragSensitivity — see the derivation table in the definition
// (preview_renderer.cpp).
// Returns 0 for a degenerate viewport (vp_w <= 0 || vp_h <= 0), and the historical
// constant 0.3 deg/px for a lens type that has no drag interaction (full-sky).
// Isotropic by construction: azimuth and elevation share one scalar, because the
// radial projection laws are rotationally symmetric about the optical axis.
float ComputeDragGainDegPerPixel(int lens_type, float fov_deg, int vp_w, int vp_h);

// The CPU half of the background overlay's contain fit, with the user's pan/zoom folded in.
// The fragment shader's `bg_uv = v_ndc * u_bg_uv_scale + u_bg_uv_offset` line is unchanged and
// does not know pan/zoom exists; everything the user dials in arrives through these four numbers.
//
// bg_aspect is width/height of the loaded image; vp_w/vp_h are framebuffer pixels. `zoom` divides
// the scale (a larger zoom samples a smaller UV span, i.e. the photo grows on screen) and `pan`
// is added straight onto the UV offset, so one unit of pan is one full texture width/height at
// any zoom. scale_y is negative: stbi loads top-down while the GL texture origin is bottom-left.
//
// zoom == 1 && pan == 0 reproduces the historical hard-coded centered fit bit for bit, which is
// what makes an .lmc written before these fields existed render identically with no compat branch.
struct BgUvTransform {
  float scale_x;
  float scale_y;
  float offset_x;
  float offset_y;
};
BgUvTransform ComputeBgUvTransform(int vp_w, int vp_h, float bg_aspect, float pan_x, float pan_y, float zoom);

// How the on-screen hint spells the key that arms the background pan/zoom gestures. The key
// itself is io.KeyAlt on every platform; only its printed name differs, because that is what is
// engraved on the keyboard the reader is looking at.
//
// Cmd on macOS was the obvious-looking alternative and does not work: ImGui, with
// ConfigMacOSXBehaviors (on by default under __APPLE__), rewrites Super+LeftClick into a RIGHT
// click at the event-queue level — "Super+Left Click aliased into Right Click", ImGui::
// AddMouseButtonEvent in imgui.cpp. The left-button drag the canvas handler waits for therefore
// never arrives, and no amount of correct code downstream can see it. Option/Alt has no such
// aliasing and is the pan modifier most image tools already use, so there is no platform split in
// behaviour left to arbitrate — only in wording.
#if defined(__APPLE__)
inline constexpr const char* kBgModifierName = "Option";
#else
inline constexpr const char* kBgModifierName = "Alt";
#endif

// Build a ViewProjection from the renderer sub-state of GuiState.
// roll is wrapped through EffectiveRollForLens so that lens types that ignore
// roll (e.g. dual-fisheye) always see 0° — mirrors app_panels.cpp:742-747.
inline ViewProjection BuildPreviewViewProjFromRenderer(const RenderConfig& rc) {
  ViewProjection vp;
  vp.lens_type = rc.lens_type;
  vp.fov = rc.fov;
  vp.elevation = rc.elevation;
  vp.azimuth = rc.azimuth;
  vp.roll = EffectiveRollForLens(rc.lens_type, rc.roll);
  vp.visible = rc.visible;
  vp.front = rc.front;
  return vp;
}

}  // namespace lumice::gui

#endif  // LUMICE_GUI_PREVIEW_RENDERER_HPP
