#ifndef LUMICE_GUI_PREVIEW_RENDERER_HPP
#define LUMICE_GUI_PREVIEW_RENDERER_HPP

#include <vector>

namespace lumice::gui {

struct PreviewParams {
  int lens_type = 0;       // Index into kLensTypeNames
  float fov = 90.0f;       // Degrees
  float elevation = 0.0f;  // Degrees
  float azimuth = 0.0f;    // Degrees
  float roll = 0.0f;       // Degrees
  int visible = 2;         // 0=upper, 1=lower, 2=full
  float ray_color[3] = { 1.0f, 1.0f, 1.0f };
  float background[3] = { 0.0f, 0.0f, 0.0f };
  float intensity_factor = 1.0f;
};

class PreviewRenderer {
 public:
  bool Init();
  void Destroy();

  // Upload equirectangular image (RGB, uint8, row-major)
  void UploadTexture(const unsigned char* data, int width, int height);

  // Render preview into the given viewport region (in framebuffer pixels)
  void Render(int vp_x, int vp_y, int vp_w, int vp_h, const PreviewParams& params);

  bool HasTexture() const { return tex_width_ > 0 && tex_height_ > 0; }
  void ClearTexture();

  // CPU-side texture data access (for .lmc file save)
  const unsigned char* GetTextureData() const { return tex_data_.empty() ? nullptr : tex_data_.data(); }
  int GetTextureWidth() const { return tex_width_; }
  int GetTextureHeight() const { return tex_height_; }

 private:
  unsigned int shader_program_ = 0;
  unsigned int vao_ = 0;
  unsigned int vbo_ = 0;
  unsigned int texture_ = 0;
  int tex_width_ = 0;
  int tex_height_ = 0;
  std::vector<unsigned char> tex_data_;  // CPU-side copy of texture (RGB uint8)
};

}  // namespace lumice::gui

#endif  // LUMICE_GUI_PREVIEW_RENDERER_HPP
