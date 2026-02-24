#ifndef CORE_GEO3D_H_
#define CORE_GEO3D_H_

#include <cstddef>
#include <memory>

#include "core/math.hpp"

namespace lumice {

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

}  // namespace lumice

#endif  // CORE_GEO3D_H_
