#ifndef LUMICE_GUI_CRYSTAL_RENDERER_HPP
#define LUMICE_GUI_CRYSTAL_RENDERER_HPP

#include <cstdint>
#include <vector>

namespace lumice::gui {

enum class CrystalStyle { kWireframe, kHiddenLine, kXRay, kShaded };

inline const char* const kCrystalStyleNames[] = { "Wireframe", "Hidden Line", "X-Ray", "Shaded" };
constexpr int kCrystalStyleCount = 4;

class CrystalRenderer {
 public:
  bool Init(int width, int height);
  void Destroy();

  // Update mesh data from LUMICE_CrystalMesh fields
  void UpdateMesh(const float* vertices, int vertex_count, const int* edges, int edge_count, const int* triangles,
                  int triangle_count, const float* edge_face_normals);

  // Render with given rotation, zoom, and style
  void Render(const float rotation[16], float zoom, CrystalStyle style);

  // Get the rendered texture ID for ImGui::Image()
  uintptr_t GetTextureId() const;

  int Width() const { return width_; }
  int Height() const { return height_; }

 private:
  int width_ = 0;
  int height_ = 0;

  // MSAA FBO
  unsigned int fbo_ = 0;
  unsigned int ms_color_rb_ = 0;
  unsigned int ms_depth_rb_ = 0;
  unsigned int resolve_fbo_ = 0;
  unsigned int color_tex_ = 0;

  // Edge shader (VS + GS + FS, supports dashing)
  unsigned int edge_shader_ = 0;

  // Face shader (VS + FS, flat shading with dFdx/dFdy)
  unsigned int face_shader_ = 0;

  // Geometry buffers
  unsigned int vao_ = 0;
  unsigned int vbo_ = 0;
  unsigned int front_ebo_ = 0;
  unsigned int back_ebo_ = 0;
  unsigned int tri_ebo_ = 0;

  // CPU mesh data for per-frame edge classification
  std::vector<int> all_edges_;             // edge vertex pairs
  std::vector<float> edge_face_normals_;   // 6 floats per edge
  int total_edge_count_ = 0;
  int tri_count_ = 0;
};

}  // namespace lumice::gui

#endif  // LUMICE_GUI_CRYSTAL_RENDERER_HPP
