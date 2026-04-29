#ifndef LUMICE_GUI_PREVIEW_RENDERER_HPP
#define LUMICE_GUI_PREVIEW_RENDERER_HPP

#include <vector>

#include "gui/gui_constants.hpp"

namespace lumice::gui {

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
};

// Canonical ViewProjection values for export code paths. Kept adjacent to
// ViewProjection so aggregate-initializer field order can be verified at a
// glance. Field order, top-to-bottom: lens_type, fov, elevation, azimuth, roll, visible.
inline constexpr ViewProjection kDualFisheyeExportViewProj = {
  kLensTypeDualFisheyeEqualArea, 180.0f, 0.0f, 0.0f, 0.0f, kVisibleFull,
};
inline constexpr ViewProjection kEquirectExportViewProj = {
  kLensTypeRectangular, 180.0f, 0.0f, 0.0f, 0.0f, kVisibleFull,
};

struct Exposure {
  float intensity_factor = 1.0f;
  float intensity_scale = 0.0f;  // = intensity_factor / per_pixel_intensity (0 = not in XYZ mode)
};

// Auxiliary line overlay (horizon, altitude grid, sun circles) drawn on top
// of the preview. sun_dir is precomputed on CPU from GuiState::sun.altitude.
//
// The show_* fields here control **line** rendering only (shader uniforms
// u_show_horizon / u_show_grid / u_show_sun_circles). They are sourced from
// GuiState::show_<x>_line. The companion fields GuiState::show_<x>_label are
// consumed by OverlayLabelInput, not this struct.
struct OverlayDecoration {
  bool show_horizon = false;
  bool show_grid = false;
  bool show_sun_circles = false;
  float sun_dir[3] = {};  // precomputed world-space unit vector
  int sun_circle_count = 0;
  float sun_circle_angles[kMaxSunCircles] = {};  // degrees
  float horizon_color[3] = { 0.8f, 0.2f, 0.2f };
  float grid_color[3] = { 1.0f, 1.0f, 1.0f };
  float sun_circles_color[3] = { 1.0f, 0.9f, 0.3f };
  float horizon_alpha = 0.6f;
  float grid_alpha = 0.3f;
  float sun_circles_alpha = 0.5f;

  static OverlayDecoration Disabled() { return {}; }
};

struct Background {
  bool enabled = false;
  float alpha = 1.0f;
  float aspect = 1.0f;

  static Background Disabled() { return {}; }
};

struct PreviewParams {
  SourceFormat source;
  ViewProjection view_proj;
  Exposure exposure;
  OverlayDecoration overlay;
  Background bg;
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

}  // namespace lumice::gui

#endif  // LUMICE_GUI_PREVIEW_RENDERER_HPP
