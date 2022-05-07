#ifndef CORE_GEO3D_H_
#define CORE_GEO3D_H_

#include <cstddef>
#include <memory>

#include "core/math.hpp"

namespace icehalo {
namespace v3 {

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
  Mesh(const Mesh& other);
  Mesh(Mesh&& other);
  ~Mesh() = default;

  Mesh& operator=(const Mesh& other);
  Mesh& operator=(Mesh&& other);

  size_t GetVtxCnt() const;
  size_t GetTriangleCnt() const;

  void SetVtx(const float* data);
  void SetTriangle(const int* idx);

  float* GetVtxPtr(size_t idx);
  int* GetTrianglePtr(size_t idx);

 private:
  size_t vtx_cnt_;
  size_t triangle_cnt_;
  std::unique_ptr<float[]> vertices_;  // vertex coordinates, 3 * vtx_cnt_
  std::unique_ptr<int[]> triangle_;    // vertex index, 3 * triangle_cnt_
};

}  // namespace v3


}  // namespace icehalo

#endif  // CORE_GEO3D_H_