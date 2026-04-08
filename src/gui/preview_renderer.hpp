#ifndef LUMICE_GUI_PREVIEW_RENDERER_HPP
#define LUMICE_GUI_PREVIEW_RENDERER_HPP

#include <vector>

#include "gui/gui_constants.hpp"

namespace lumice::gui {

struct PreviewParams {
  int lens_type = 0;       // Index into kLensTypeNames
  float fov = 90.0f;       // Degrees
  float elevation = 0.0f;  // Degrees
  float azimuth = 0.0f;    // Degrees
  float roll = 0.0f;       // Degrees
  int visible = 2;         // 0=upper, 1=lower, 2=full
  float intensity_factor = 1.0f;
  float intensity_scale = 0.0f;  // = intensity_factor / per_pixel_intensity (0 = not in XYZ mode)
  float max_abs_dz = 0.0f;       // overlap zone threshold (0 = no blend)
  float r_scale = 1.0f;          // projection r_scale for overlap normalization
  bool bg_enabled = false;
  float bg_alpha = 1.0f;
  float bg_aspect = 1.0f;

  // Auxiliary lines (sun_dir is precomputed on CPU from GuiState::sun.altitude/azimuth)
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
