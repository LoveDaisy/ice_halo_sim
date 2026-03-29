#ifndef CORE_GEO3D_H_
#define CORE_GEO3D_H_

#include <cstddef>
#include <memory>

#include "core/math.hpp"

namespace lumice {

class Crystal;

class Rotation {
 public:
  Rotation();
  Rotation(const float* ax, float theta);
  Rotation(const float* from, const float* to);

  Rotation& Chain(const Rotation& rotate);
  Rotation& Chain(const float* ax, float theta);
  Rotation& Chain(const float* from, const float* to);
  Rotation& Inverse();
  void Apply(float* pt, size_t num = 1) const;
  void ApplyInverse(float* pt, size_t num = 1) const;

 private:
  void FillMat(const float* ax, float theta);

  float mat_[9];
};


void RandomSample(int pop_size, const float* weight, int* out, size_t sample_num = 1);

void SampleTrianglePoint(const float* vertices, float* out_pt, size_t sample_num = 1);

void SampleSphCapPoint(float lon, float lat, float cap_radii, float* out_pt,    //
                       size_t sample_num = 1, size_t step = 3 * sizeof(float),  //
                       AngleUnit unit = AngleUnit::kDegree);                    //

void SampleSph(float radii, float* out_pt, size_t sample_num = 1);

void SampleBall(float radii, float* out_pt, size_t sample_num = 1);


class Mesh {
 public:
  Mesh();
  Mesh(size_t vtx_cnt, size_t triangle_cnt);
  Mesh(size_t vtx_cnt, std::unique_ptr<float[]> vtx, size_t triangle_cnt, std::unique_ptr<int[]> triangle_idx);
  Mesh(const Mesh& other);
  Mesh(Mesh&& other) noexcept;
  ~Mesh() = default;

  Mesh& operator=(const Mesh& other);
  Mesh& operator=(Mesh&& other) noexcept;

  size_t GetVtxCnt() const;
  size_t GetTriangleCnt() const;

  float* GetVtxPtr(size_t idx);
  int* GetTrianglePtr(size_t idx);

 private:
  size_t vtx_cnt_;
  size_t triangle_cnt_;
  std::unique_ptr<float[]> vertices_;  // vertex coordinates, 3 * vtx_cnt_
  std::unique_ptr<int[]> triangle_;    // vertex index, 3 * triangle_cnt_
};


constexpr int kHexPrismVtxCnt = 12;
constexpr int kHexPrismTriCnt = 20;
constexpr int kHexPyramidVtxCnt = 24;
constexpr int kHexPyramidTriCnt = 44;
constexpr float kIceCrystalC = 1.629f;
constexpr int kMaxHexCrystalPlanes = 20;  // 2 basal + 6 prism + 6 upper pyr + 6 lower pyr

// ====== Unified hex crystal plane equations ======
// Generates plane equations for hexagonal crystal (convex: prism and/or pyramid).
// Prism = h1≈0, h3≈0 degenerate case (8 planes). Full pyramid = up to 20 planes.
// Returns actual plane count. out_coef must hold at least kMaxHexCrystalPlanes * 4 floats.
size_t FillHexCrystalCoef(float upper_alpha, float lower_alpha, float h1, float h2, float h3, const float* dist,
                          float* out_coef);

// ====== Unified convex polyhedron mesh creation ======
Mesh CreateConvexPolyhedronMesh(int plane_cnt, const float* coef);

// ====== Prism ======
Mesh CreatePrismMesh(float h);

Mesh CreatePrismMesh(float h, const float* dist);

// ====== Pyramid ======
Mesh CreatePyramidMesh(float h1, float h2, float h3);

Mesh CreatePyramidMesh(int upper_idx1, int upper_idx4, int lower_idx1, int lower_idx4,  // Miller index
                       float h1, float h2, float h3,                                    // height
                       const float* dist);                                              // face distance

Mesh CreatePyramidMesh(float upper_alpha, float lower_alpha,  // wedge angle, angle between pyramidal face and c-axis
                       float h1, float h2, float h3,          // height
                       const float* dist);                    // face distance

// ====== Concave pyramid ======
Mesh CreateConcavePyramidMesh(float h1, float h2, float h3);

Mesh CreateConcavePyramidMesh(int upper_idx1, int upper_idx4, int lower_idx1, int lower_idx4,  // Miller index
                              float h1, float h2, float h3,                                    // height
                              const float* dist);                                              // face distance

Mesh CreateConcavePyramidMesh(float upper_alpha, float lower_alpha,  // wedge angle
                              float h1, float h2, float h3,          // height
                              const float* dist);                    // face distance

// ====== 2D Convex Polygon Geometry ======

// Maximum vertices for a 2D convex polygon (beam cross-sections, face projections).
constexpr size_t kMaxPolyVertices = 24;

// Minimum polygon area threshold (below this, polygons are considered degenerate).
constexpr float kMinBeamArea = 1e-8f;

// 2D convex polygon with fixed-capacity vertex storage.
// Vertices are stored as [x0,y0, x1,y1, ...] in counter-clockwise order.
struct ConvexPolygon2D {
  float vertices[2 * kMaxPolyVertices]{};
  size_t count = 0;

  size_t VertexCount() const { return count; }
  float Area() const;  // Shoelace formula (returns absolute value)
};

// Build an orthonormal basis (u, v) perpendicular to direction d.
// d must be a unit vector. u and v are output as 3-float arrays.
void BuildOrthonormalBasis(const float* d, float* u, float* v);

// Project 3D points onto 2D plane defined by orthonormal basis (u, v).
// pts_3d: n points as [x0,y0,z0, x1,y1,z1, ...], pts_2d: output [u0,v0, u1,v1, ...]
void ProjectTo2D(const float* basis_u, const float* basis_v, const float* pts_3d, size_t n, float* pts_2d);

// Clip a 2D convex polygon by a half-plane: nx*x + ny*y + d >= 0.
// Returns the clipped polygon (may have 0 vertices if fully clipped).
ConvexPolygon2D ClipByHalfPlane(const ConvexPolygon2D& poly, float nx, float ny, float d);

// Compute the intersection of two convex polygons using Sutherland-Hodgman clipping.
// Both polygons must have CCW vertex order.
ConvexPolygon2D IntersectConvexPolygons(const ConvexPolygon2D& a, const ConvexPolygon2D& b);

// Extract ordered polygon face vertices from Crystal's triangle mesh.
// Returns vertex count. out_vertices_3d must have space for 3*kMaxPolyVertices floats.
size_t ExtractPolygonFaceVertices(const Crystal& crystal, int poly_face_id, float* out_vertices_3d);

}  // namespace lumice

#endif  // CORE_GEO3D_H_
