#ifndef LUMICE_GUI_PREVIEW_RENDERER_HPP
#define LUMICE_GUI_PREVIEW_RENDERER_HPP

#include <array>
#include <optional>
#include <vector>

#include "gui/gui_constants.hpp"
#include "gui/gui_state.hpp"

namespace lumice::gui {

inline constexpr float kOverlaySentinel = -9999.f;

// How many parallels, and how many meridians, the preview shader can draw at once: the size of the
// two level-list uniform arrays in kFragmentShader (preview_renderer.cpp, u_elevation_deg /
// u_longitude_deg). Equal to LUMICE_MAX_ANNOTATION_LINES because the widest list the GUI builds —
// 720 meridians at its narrowest field of view (ComputeGridLongitudeAngles at a 0.5 deg step) —
// is what that ceiling was widened for, so the two are one number for one reason. A list longer
// than this is TRUNCATED at upload (Render()), which the GUI cannot produce and core would reject.
// The circles' capacity is kMaxAnnotationCircles (gui_constants.hpp), the list's own ceiling.
inline constexpr int kMaxOverlayLevels = LUMICE_MAX_ANNOTATION_LINES;

// The "no marker anywhere" position array. A function for the same reason GuiState's
// MakeDefaultMarkers() is one: the value is six identical pairs, and a member initializer spelling
// them out is six lines that say nothing the name does not.
inline std::array<std::array<float, 2>, LUMICE_ANNOTATION_MARKER_COUNT> MakeAllSentinelMarkerPositions() {
  std::array<std::array<float, 2>, LUMICE_ANNOTATION_MARKER_COUNT> out;
  out.fill({ kOverlaySentinel, kOverlaySentinel });
  return out;
}

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

// Auxiliary line overlay (horizon, altitude grid, sun circles, view circles) drawn on top
// of the preview.
//
// The show_* fields here control **line** rendering only (shader uniforms
// u_show_horizon / u_show_grid / u_show_sun_circles / u_show_view_dist). They are sourced from
// GuiState::show_<x>_line. The companion fields GuiState::show_<x>_label are read where the
// label anchors are consumed (app_panels.cpp / app.cpp, through AnnotationAnchors), not here.
struct OverlayDecoration {
  bool show_horizon = false;
  bool show_grid = false;
  bool show_sun_circles = false;
  bool show_view_dist = false;
  // WHERE the curves are, stated as the definition the shader evaluates per fragment rather than as
  // pixels: the parallels and meridians of the grid and the circles' radii, in degrees, and the
  // direction the circles are centred on. The same three lists and the same direction go into the
  // core anchor request (AnnotationViewInputFor, app_panels.cpp), which is what makes the drawn
  // curve and the label placed on it two readings of one input rather than two inputs.
  //
  // Parallels/meridians past kMaxOverlayLevels and either ring family's circles past
  // kMaxAnnotationCircles are not uploaded (Render() clamps the counts); the GUI's own lists never
  // reach either bound.
  std::vector<float> elevation_deg;
  std::vector<float> longitude_deg;
  std::vector<float> angular_dist_deg;
  // Circles about the camera's OPTICAL AXIS. No centre field beside this list, unlike
  // reference_dir for the sun circles below: the axis is -u_view_matrix[2] in the shader's world
  // frame, and that matrix is uploaded from view_proj every frame already — a second statement of
  // the same direction here could only disagree with it (core derives its axis from the same
  // view angles, so the two sides agree by construction rather than by a value being copied).
  std::vector<float> view_dist_deg;
  // Unit vector, world frame, the direction light TRAVELS (altitude = asin(-z)) — GuiSunWorldDir's
  // output, the same value the anchor request's reference_dir carries.
  float reference_dir[3] = { 0.0f, 0.0f, -1.0f };
  float horizon_color[3] = { 0.8f, 0.2f, 0.2f };
  float grid_color[3] = { 1.0f, 1.0f, 1.0f };
  float sun_circles_color[3] = { 1.0f, 0.9f, 0.3f };
  float view_dist_color[3] = { 0.4f, 0.9f, 1.0f };
  float horizon_alpha = 0.6f;
  float grid_alpha = 0.3f;
  float sun_circles_alpha = 0.5f;
  float view_dist_alpha = 0.5f;

  // The sky reference points: pixel-space ring markers, one slot per core marker id.
  //
  // marker_screen_pos: CPU-precomputed; center-origin pixel coords, y-up (matches the shader's
  // `pos = v_ndc * u_resolution * 0.5`). Sentinel (-9999, -9999) when the direction is offscreen /
  // behind the camera / on an unsupported lens — the shader compares distance and naturally skips
  // it.
  //
  // There is deliberately NO per-marker enable flag and no count beside these: the sentinel
  // ALREADY expresses "do not draw this one", so a marker the user has switched off is written as
  // one and needs no second switch. A count would additionally impose an ordering the id indexing
  // exists to avoid.
  std::array<std::array<float, 2>, LUMICE_ANNOTATION_MARKER_COUNT> marker_screen_pos = MakeAllSentinelMarkerPositions();
  std::array<std::array<float, 3>, LUMICE_ANNOTATION_MARKER_COUNT> marker_color{};
  // Family-wide, matching GuiState and LUMICE_RenderParam: colour tells the points apart, size and
  // transparency say what KIND of annotation they are.
  float markers_alpha = 0.6f;
  float markers_radius_px = 8.0f;

  // Lens border: outline of the projection's valid image circle. The shader
  // derives the geometry itself from lens type / FOV / resolution (see
  // overlayLensBorder), so unlike the reference-point markers there is no
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

  // The PAPER colour under `tone == 1`, and a third distinct thing from the two above: `bg` is a
  // photo behind the halo, `background_color_linear` is the SKY the halo's light is added to, and
  // this is the SHEET the ink is laid on. They are separate fields so that "print onto the default
  // black background" is not a state the document can reach at all — see
  // doc/print-mode-subtractive-ink.md §6.
  //
  // LINEAR RGB, on the same terms as background_color_linear: converted from
  // GuiState::RenderConfig::paper (sRGB, what the picker shows) by whoever fills this struct, and
  // inherited unchanged by the export entry points through BuildExportParams.
  float paper_color_linear[3] = { 1.0f, 1.0f, 1.0f };

  // Which operator turns radiance into pixels: 0 = screen (additive, the historical one), 1 =
  // print (subtractive ink on paper). Same int spelling as config::RenderConfig::Tone and
  // GuiState::RenderConfig::tone, so the value travels the whole chain without a mapping table
  // that could invert.
  //
  // The default of 0 is what makes every caller that never touches this field render exactly as
  // before, the same guarantee background_color_linear's all-zero default gives.
  int tone = 0;
};

class PreviewRenderer {
 public:
  // What the source texture's samples MEAN, and therefore how much of the display chain the
  // shader still owes them. Mirrored by the kTexMode* constants in the GLSL (preview_renderer.cpp)
  // — the two enumerations are one contract and their values must stay in step.
  enum class TextureMode : int {
    // 8-bit sRGB texels that already carry the sky colour, drawn as they are. Two producers, and
    // neither can be corrected at display time: ClearTexture()'s 1x1 black pixel, and a .lmc
    // written before the format carried radiance-only textures (v <= 3), whose sky is summed into
    // every texel and cannot be un-summed where the bake clipped.
    kSrgbComposited = 0,
    // Float XYZ radiance from the live simulation.
    kXyz = 1,
    // 8-bit sRGB texels carrying the halo's radiance ALONE — exposure already applied, no sky.
    // The shader applies the target lens's relative illumination and then the sky, by the same
    // lines and in the same order as the XYZ branch, so a picture reopened from disk renders the
    // way the live view rendered it.
    kSrgbRadiance = 2,
  };

  bool Init();
  void Destroy();

  // Upload equirectangular image (RGB, uint8, row-major) whose texels ALREADY carry the sky
  // colour — see TextureMode::kSrgbComposited. The only remaining producer is a legacy .lmc.
  void UploadTexture(const unsigned char* data, int width, int height);

  // The same bytes, but carrying the halo's radiance alone — see TextureMode::kSrgbRadiance.
  // This is the entry point every current producer uses: the .lmc bake and the composite
  // (raypath-colour) preview both hand over radiance-only pixels.
  void UploadRadianceTexture(const unsigned char* data, int width, int height);

  // Upload equirectangular XYZ float data — for live simulation preview
  void UploadXyzTexture(const float* data, int width, int height);

  // Render preview into the given viewport region (in framebuffer pixels)
  void Render(int vp_x, int vp_y, int vp_w, int vp_h, const PreviewParams& params);

  bool HasTexture() const { return tex_width_ > 0 && tex_height_ > 0; }
  void ClearTexture();

  // Update CPU-side texture data only (no GL upload, no tex_mode_ change).
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
  std::vector<unsigned char> tex_data_;                  // CPU-side copy of texture (RGB uint8, for .lmc save)
  TextureMode tex_mode_ = TextureMode::kSrgbComposited;  // what texture_ currently holds

  // Deferred GL blank request. ClearTexture() sets this from any thread
  // (callable from coroutine workers without a GL context); Render() (main
  // thread, GL context) consumes it by re-uploading a 1x1 black pixel into
  // texture_ so the sim layer stops sampling stale pixels. Any method that
  // writes fresh real pixel data into texture_ MUST clear this flag first,
  // so the newest real write always wins over a pending blank (invariant not
  // enforced by the compiler — grep this comment before adding a new upload).
  bool needs_gl_blank_ = false;

  void UploadBlankSimTexture();

  // GL body shared by UploadTexture / UploadRadianceTexture; see the definition.
  void UploadUint8Texture(const unsigned char* data, int width, int height, TextureMode mode);

  // PBO double-buffer for async XYZ texture upload (GLsync stored as void* to
  // avoid including GL headers in this header; cast to GLsync in the .cpp).
  std::array<unsigned int, 2> pbo_ = { 0, 0 };
  std::array<void*, 2> pbo_fence_ = { nullptr, nullptr };
  std::array<size_t, 2> pbo_byte_count_ = { 0, 0 };
  int pbo_index_ = 0;

  // Background image texture (no CPU-side copy — loaded from file path)
  unsigned int bg_texture_ = 0;
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

// --- Background eyedropper: screen point -> photo pixel -> colour ---------------------------
// Declared here, beside ComputeBgUvTransform and for the same reason: these are the CPU half of
// the same mapping, and a unit test must be able to hold them without a GL context.

struct NdcPoint {
  float x;
  float y;
};

// A screen-space displacement in ImGui LOGICAL POINTS, expressed in NDC units. dpi_scale_* is what
// reconciles the two (vp_w/vp_h are framebuffer pixels), and the Y sign flip is here because screen
// Y grows downward while NDC Y grows upward. Returns {0,0} on a degenerate viewport.
NdcPoint ScreenDeltaToNdcDelta(float dx_pt, float dy_pt, float dpi_scale_x, float dpi_scale_y, int vp_w, int vp_h);

// A screen point given RELATIVE TO THE VIEWPORT'S TOP-LEFT corner, in logical points, as NDC.
// Same relation as above with the corner's own NDC (-1, +1) added: one owner, two call shapes.
NdcPoint ScreenPosToNdc(float x_pt, float y_pt, float dpi_scale_x, float dpi_scale_y, int vp_w, int vp_h);

// Column/row into the background image's CPU copy — row 0 is the TOP row, matching stbi's native
// order and therefore the order the bytes were uploaded in.
struct BgPixelIndex {
  int col;
  int row;
};

// uv in [0,1]^2 -> {col, row}; anything outside that square is the letterbox and yields nullopt.
// No Y flip of its own: ComputeBgUvTransform's negative scale_y is the single owner of the flip,
// and a second one here would silently mirror the sample against what the user sees.
std::optional<BgPixelIndex> BgUvToPixelIndex(float u, float v, int img_w, int img_h);

// Everything the mapping needs about the current frame, in one named carrier rather than ten
// positional scalars — the same set the shader's background stage is fed from.
struct BgSampleGeometry {
  float dpi_scale_x = 1.0f;
  float dpi_scale_y = 1.0f;
  int vp_w = 0;
  int vp_h = 0;
  float bg_aspect = 1.0f;
  float pan_x = 0.0f;
  float pan_y = 0.0f;
  float zoom = 1.0f;
  int img_w = 0;
  int img_h = 0;
};

// The eyedropper sample, end to end: a cursor position over the preview -> the sRGB triple of the
// photo pixel under it, in the [0,1] convention RenderConfig::background stores. nullopt when the
// point falls on the letterbox, or when there is no CPU copy to read.
//
// No colour-space conversion happens or should happen here: the shader composites the photo AFTER
// clampAndGamma, i.e. in the same sRGB encoding these bytes carry, and `background` is stored in
// that encoding too (app_panels.cpp converts to linear only when filling PreviewParams). Byte/255
// is therefore the exact answer, not an approximation of one.
//
// `pixels` is the RGB, row-major, top-down buffer GuiState::bg_pixels holds; img_w/img_h in `geom`
// must be the size it was filled at.
std::optional<std::array<float, 3>> SampleBgColorAtScreenPos(float x_pt, float y_pt, const BgSampleGeometry& geom,
                                                             const std::vector<unsigned char>& pixels);

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
