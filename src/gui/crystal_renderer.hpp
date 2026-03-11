#ifndef LUMICE_GUI_CRYSTAL_RENDERER_HPP
#define LUMICE_GUI_CRYSTAL_RENDERER_HPP

#include <cstdint>

namespace lumice::gui {

class CrystalRenderer {
 public:
  bool Init(int width, int height);
  void Destroy();

  // Update mesh data (flat arrays: vertices=[x,y,z,...], edges=[v0,v1,...])
  void UpdateMesh(const float* vertices, int vertex_count, const int* edges, int edge_count);

  // Render the wireframe with given rotation quaternion and zoom factor
  // rotation is a 4x4 rotation matrix (column-major)
  void Render(const float rotation[16], float zoom);

  // Get the rendered texture ID for ImGui::Image()
  uintptr_t GetTextureId() const;

  int Width() const { return width_; }
  int Height() const { return height_; }

 private:
  int width_ = 0;
  int height_ = 0;
  unsigned int fbo_ = 0;          // Multisample render target
  unsigned int ms_color_rb_ = 0;  // Multisample color renderbuffer
  unsigned int ms_depth_rb_ = 0;  // Multisample depth renderbuffer
  unsigned int resolve_fbo_ = 0;  // Resolve target (non-multisample)
  unsigned int color_tex_ = 0;    // Resolved texture for ImGui display
  unsigned int shader_ = 0;
  unsigned int vao_ = 0;
  unsigned int vbo_ = 0;
  unsigned int ebo_ = 0;
  int edge_count_ = 0;
};

}  // namespace lumice::gui

#endif  // LUMICE_GUI_CRYSTAL_RENDERER_HPP
