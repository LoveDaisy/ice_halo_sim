#include "core/geo3d.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <limits>
#include <memory>
#include <set>
#include <tuple>
#include <utility>

#include "core/math.hpp"

namespace icehalo {
namespace v3 {

Rotation::Rotation() : mat_{ 1, 0, 0, 0, 1, 0, 0, 0, 1 } {}

Rotation::Rotation(const float* ax, float theta) : mat_{} {
  FillMat(ax, theta);
}

Rotation::Rotation(const float* from, const float* to) : mat_{} {
  float ax[3];
  Cross3(from, to, ax);
  float theta = Norm3(ax);
  for (auto& x : ax) {
    x /= theta;
  }
  FillMat(ax, theta);
}

Rotation& Rotation::Chain(const Rotation& rotate) {
  // Left-multiply next rotate matrix
  float m0[9];
  std::memcpy(m0, mat_, 9 * sizeof(float));

  // Naive matrix multiply
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      mat_[i * 3 + j] = 0;
      for (int k = 0; k < 3; k++) {
        mat_[i * 3 + j] += rotate.mat_[i * 3 + k] * m0[k * 3 + j];
      }
    }
  }
  return *this;
}

Rotation& Rotation::Chain(const float* ax, float theta) {
  return Chain(Rotation(ax, theta));
}

Rotation& Rotation::Chain(const float* from, const float* to) {
  return Chain(Rotation(from, to));
}

Rotation& Rotation::Inverse() {
  for (int r = 0; r < 3; r++) {
    for (int c = r + 1; c < 3; c++) {
      std::swap(mat_[r * 3 + c], mat_[c * 3 + r]);
    }
  }
  return *this;
}

void Rotation::Apply(float* pt, size_t num) const {
  float new_pt[3];
  for (size_t id = 0; id < num; id++) {
    for (int k = 0; k < 3; k++) {
      new_pt[k] = Dot3(mat_ + k * 3, pt + id * 3);
    }
    std::memcpy(pt + id * 3, new_pt, 3 * sizeof(float));
  }
}

void Rotation::ApplyInverse(float* pt, size_t num) const {
  float inv_mat[9];
  for (int r = 0; r < 3; r++) {
    for (int c = r + 1; c < 3; c++) {
      inv_mat[c * 3 + r] = mat_[r * 3 + c];
      inv_mat[r * 3 + c] = mat_[c * 3 + r];
    }
    inv_mat[r * 3 + r] = mat_[r * 3 + r];
  }

  float new_pt[3];
  for (size_t id = 0; id < num; id++) {
    for (int k = 0; k < 3; k++) {
      new_pt[k] = Dot3(inv_mat + k * 3, pt + id * 3);
    }
    std::memcpy(pt + id * 3, new_pt, 3 * sizeof(float));
  }
}

void Rotation::FillMat(const float* ax, float theta) {
  float c = std::cos(theta);
  float s = std::sin(theta);
  float cc = 1 - c;
  mat_[0] = ax[0] * ax[0] * cc + c;
  mat_[1] = ax[0] * ax[1] * cc - ax[2] * s;
  mat_[2] = ax[0] * ax[2] * cc + ax[1] * s;
  mat_[3] = ax[0] * ax[1] * cc + ax[2] * s;
  mat_[4] = ax[1] * ax[1] * cc + c;
  mat_[5] = ax[1] * ax[2] * cc - ax[0] * s;
  mat_[6] = ax[0] * ax[2] * cc - ax[1] * s;
  mat_[7] = ax[1] * ax[2] * cc + ax[0] * s;
  mat_[8] = ax[2] * ax[2] * cc + c;
}


void RandomSample(int pop_size, const float* weight, int* out, size_t sample_num) {
  if (pop_size <= 0) {
    return;
  }
  if (pop_size == 1) {
    for (size_t i = 0; i < sample_num; i++) {
      out[i] = 0;
    }
    return;
  }

  std::unique_ptr<float[]> p{ new float[pop_size + 1]{} };
  std::memcpy(p.get() + 1, weight, pop_size * sizeof(float));
  for (int i = 1; i < pop_size; i++) {
    p[i + 1] = std::max(p[i + 1], 0.0f) + p[i];
  }
  for (int i = 0; i < pop_size; i++) {
    p[i + 1] /= p[pop_size];
  }

  auto* rng = RandomNumberGenerator::GetInstance();
  for (size_t i = 0; i < sample_num; i++) {
    auto curr_p = rng->GetUniform();
    for (int j = 0; j < pop_size; j++) {
      if (p[j] < curr_p && curr_p <= p[j + 1]) {
        out[i] = j;
        break;
      }
    }
  }
}


void SampleTrianglePoint(const float* vertices, float* out_pt, size_t sample_num) {
  auto* rng = RandomNumberGenerator::GetInstance();
  float e1[3]{ vertices[3] - vertices[0], vertices[4] - vertices[1], vertices[5] - vertices[2] };
  float e2[3]{ vertices[6] - vertices[0], vertices[7] - vertices[1], vertices[8] - vertices[2] };
  for (size_t i = 0; i < sample_num; i++) {
    auto u = rng->GetUniform();
    auto v = rng->GetUniform();
    if (u + v > 1.0f) {
      u = 1.0f - u;
      v = 1.0f - v;
    }
    out_pt[i * 3 + 0] = u * e1[0] + v * e2[0] + vertices[0];
    out_pt[i * 3 + 1] = u * e1[1] + v * e2[1] + vertices[1];
    out_pt[i * 3 + 2] = u * e1[2] + v * e2[2] + vertices[2];
  }
}


void SampleSphCapPoint(float lon, float lat, float cap_radii, float* out_pt,  //
                       size_t sample_num, size_t step,                        //
                       AngleUnit unit) {
  if (unit == AngleUnit::kDegree) {
    lon *= math::kDegreeToRad;
    lat *= math::kDegreeToRad;
    cap_radii *= math::kDegreeToRad;
  }

  auto* rng = RandomNumberGenerator::GetInstance();
  float c_cap = std::cos(cap_radii);
  float c_lon = std::cos(lon);
  float s_lon = std::sin(lon);
  float c_lat = std::cos(lat);
  float s_lat = std::sin(lat);
  for (size_t i = 0; i < sample_num; i++) {
    // 1. Sample arount x-axis
    float x = rng->GetUniform();
    x += (1 - x) * c_cap;
    float r = std::sqrt(1.0f - x * x);

    float u = rng->GetUniform() * 2 * math::kPi;
    float y = std::cos(u) * r;
    float z = std::sin(u) * r;

    // 2. Then rotate
    // R = Rz(lon).Ry(-lat)
    //     | cos(lon)cos(lat), -sin(lon), -cos(lon)sin(lat) |
    //   = | sin(lon)cos(lat),  cos(lon), -sin(lon)sin(lat) |
    //     | sin(lat),          0,         cos(lat)         |
    auto* p = reinterpret_cast<float*>(reinterpret_cast<uint8_t*>(out_pt) + i * step);
    p[0] = c_lon * c_lat * x - s_lon * y - c_lon * s_lat * z;
    p[1] = s_lon * c_lat * x - c_lon * y - s_lon * s_lat * z;
    p[2] = s_lat * x + c_lat * z;
  }
}

void SampleSph(float radii, float* out_pt, size_t sample_num) {
  auto* rng = RandomNumberGenerator::GetInstance();
  for (size_t i = 0; i < sample_num; i++) {
    auto z = rng->GetUniform() * 2 - 1;
    float r = std::sqrt(1.0f - z * z);
    float q = rng->GetUniform() * 2 * math::kPi;
    out_pt[i * 3 + 0] = std::cos(q) * r;
    out_pt[i * 3 + 1] = std::sin(q) * r;
    out_pt[i * 3 + 2] = z;
  }
  for (size_t i = 0; i < sample_num * 3; i++) {
    out_pt[i] *= radii;
  }
}

void SampleBall(float radii, float* out_pt, size_t sample_num) {
  auto* rng = RandomNumberGenerator::GetInstance();
  for (size_t i = 0; i < sample_num; i++) {
    float u = rng->GetUniform();
    float r = std::cbrt(u) * radii;
    float z = (rng->GetUniform() * 2 - 1.0f) * r;
    float rr = std::sqrt(r * r - z * z);
    float q = rng->GetUniform() * 2 * math::kPi;
    float x = std::cos(q) * rr;
    float y = std::sin(q) * rr;
    out_pt[i * 3 + 0] = x;
    out_pt[i * 3 + 1] = y;
    out_pt[i * 3 + 2] = z;
  }
}


Mesh::Mesh() : vtx_cnt_(0), triangle_cnt_(0) {}

Mesh::Mesh(size_t vtx_cnt, size_t triangle_cnt)
    : vtx_cnt_(vtx_cnt), triangle_cnt_(triangle_cnt), vertices_(new float[vtx_cnt * 3]{}),
      triangle_(new int[triangle_cnt * 3]{}) {}

Mesh::Mesh(size_t vtx_cnt, std::unique_ptr<float[]> vtx, size_t triangle_cnt, std::unique_ptr<int[]> triangle_idx)
    : vtx_cnt_(vtx_cnt), triangle_cnt_(triangle_cnt), vertices_(std::move(vtx)), triangle_(std::move(triangle_idx)) {}

Mesh::Mesh(const Mesh& other)
    : vtx_cnt_(other.vtx_cnt_), triangle_cnt_(other.triangle_cnt_), vertices_(new float[other.vtx_cnt_ * 3]),
      triangle_(new int[other.triangle_cnt_ * 3]) {
  std::memcpy(vertices_.get(), other.vertices_.get(), vtx_cnt_ * 3 * sizeof(float));
  std::memcpy(triangle_.get(), other.triangle_.get(), triangle_cnt_ * 3 * sizeof(int));
}

Mesh::Mesh(Mesh&& other)
    : vtx_cnt_(other.vtx_cnt_), triangle_cnt_(other.triangle_cnt_), vertices_(std::move(other.vertices_)),
      triangle_(std::move(other.triangle_)) {
  other.vtx_cnt_ = 0;
  other.triangle_cnt_ = 0;
  other.vertices_ = nullptr;
  other.triangle_ = nullptr;
}

Mesh& Mesh::operator=(const Mesh& other) {
  if (&other == this) {
    return *this;
  }

  vtx_cnt_ = other.vtx_cnt_;
  triangle_cnt_ = other.triangle_cnt_;
  vertices_.reset(new float[vtx_cnt_ * 3]);
  triangle_.reset(new int[triangle_cnt_ * 3]);
  std::memcpy(vertices_.get(), other.vertices_.get(), vtx_cnt_ * 3 * sizeof(float));
  std::memcpy(triangle_.get(), other.triangle_.get(), triangle_cnt_ * 3 * sizeof(int));
  return *this;
}

Mesh& Mesh::operator=(Mesh&& other) {
  if (&other == this) {
    return *this;
  }

  vtx_cnt_ = other.vtx_cnt_;
  triangle_cnt_ = other.triangle_cnt_;
  vertices_ = std::move(other.vertices_);
  triangle_ = std::move(other.triangle_);

  other.vtx_cnt_ = 0;
  other.triangle_cnt_ = 0;
  other.vertices_ = nullptr;
  other.triangle_ = nullptr;
  return *this;
}

size_t Mesh::GetVtxCnt() const {
  return vtx_cnt_;
}

size_t Mesh::GetTriangleCnt() const {
  return triangle_cnt_;
}

float* Mesh::GetVtxPtr(size_t idx) {
  return vertices_.get() + idx * 3;
}

int* Mesh::GetTrianglePtr(size_t idx) {
  return triangle_.get() + idx * 3;
}


Mesh CreatePrismMesh(float h) {
  using math::kSqrt3_4;

  std::unique_ptr<float[]> vtx{ new float[kHexPrismVtxCnt * 3]{
      kSqrt3_4,  -1.0f / 4.0f, h / 2.0f,   // upper: vtx1
      kSqrt3_4,  1.0f / 4.0f,  h / 2.0f,   // upper: vtx2
      0.0f,      1.0f / 2.0f,  h / 2.0f,   // upper: vtx3
      -kSqrt3_4, 1.0f / 4.0f,  h / 2.0f,   // upper: vtx4
      -kSqrt3_4, -1.0f / 4.0f, h / 2.0f,   // upper: vtx5
      0.0f,      -1.0f / 2.0f, h / 2.0f,   // upper: vtx6
      kSqrt3_4,  -1.0f / 4.0f, -h / 2.0f,  // lower: vtx1
      kSqrt3_4,  1.0f / 4.0f,  -h / 2.0f,  // lower: vtx2
      0.0f,      1.0f / 2.0f,  -h / 2.0f,  // lower: vtx3
      -kSqrt3_4, 1.0f / 4.0f,  -h / 2.0f,  // lower: vtx4
      -kSqrt3_4, -1.0f / 4.0f, -h / 2.0f,  // lower: vtx5
      0.0f,      -1.0f / 2.0f, -h / 2.0f,  // lower: vtx6
  } };
  std::unique_ptr<int[]> triangle_idx{ new int[kHexPrismTriCnt * 3]{
      0,  1,  2,   // upper: fn1
      0,  2,  3,   // upper: fn1
      3,  4,  5,   // upper: fn1
      3,  5,  0,   // upper: fn1
      0,  6,  1,   // side: fn3
      6,  7,  1,   // side: fn3
      1,  7,  2,   // side: fn4
      7,  8,  2,   // side: fn4
      2,  8,  3,   // side: fn5
      8,  9,  3,   // side: fn5
      3,  9,  4,   // side: fn6
      9,  10, 4,   // side: fn6
      4,  10, 5,   // side: fn7
      10, 11, 5,   // side: fn7
      5,  11, 0,   // side: fn8
      11, 6,  0,   // side: fn8
      6,  8,  7,   // lower: fn2
      6,  9,  8,   // lower: fn2
      9,  11, 10,  // lower: fn2
      9,  6,  11,  // lower: fn2
  } };
  return Mesh(kHexPrismVtxCnt, std::move(vtx), kHexPrismTriCnt, std::move(triangle_idx));
}


Mesh CreatePrismMesh(float h, const float* dist) {
  using math::kSqrt3_2;
  using math::kSqrt3_4;

  // a*x + b*y + c <= 0, (a, b, c)
  const float kCoef[6 * 3]{
    1.0f,  0.0f,      -dist[0] * kSqrt3_4,  //
    0.5f,  kSqrt3_2,  -dist[1] * kSqrt3_4,  //
    -0.5f, kSqrt3_2,  -dist[2] * kSqrt3_4,  //
    -1.0f, 0.0f,      -dist[3] * kSqrt3_4,  //
    -0.5f, -kSqrt3_2, -dist[4] * kSqrt3_4,  //
    0.5f,  -kSqrt3_2, -dist[5] * kSqrt3_4,  //
  };

  // 1. Find out all candidate vertices
  std::unique_ptr<float[]> vtx{ new float[kHexPrismVtxCnt * 3]{} };
  for (int i = 0; i < 6; i++) {
    // A vertex is the intersection of current and previous plane
    int i1 = i;
    int i2 = (i + 5) % 6;
    SolveLines(kCoef + i1 * 3, kCoef + i2 * 3, vtx.get() + i * 3);
    vtx[i * 3 + 2] = h / 2.0f;
  }

  // 2. Filter out invalid vertices
  size_t vtx_cnt = 0;
  for (size_t i = 0; i < 6; i++) {
    // Check every plane
    if (IsInPolygon2(6, kCoef, vtx.get() + i * 3)) {
      if (vtx_cnt < i) {
        std::memcpy(vtx.get() + vtx_cnt * 3, vtx.get() + i * 3, 3 * sizeof(float));
      }
    } else {
      // If invalid, then new vertex must be the intersection of previous and next plane
      int i1 = (i + 1) % 6;
      int i2 = (i + 5) % 6;
      SolveLines(kCoef + i1 * 3, kCoef + i2 * 3, vtx.get() + vtx_cnt * 3);
      vtx[vtx_cnt * 3 + 2] = h / 2.0f;
      i++;
    }
    vtx_cnt++;
  }

  // 3. Copy data to make another basal face
  std::memcpy(vtx.get() + vtx_cnt * 3, vtx.get(), vtx_cnt * 3 * sizeof(float));
  for (int i = 0; i < 6; i++) {
    vtx[vtx_cnt * 3 + i * 3 + 2] = -h / 2.0f;
  }

  std::unique_ptr<int[]> triangle_idx{ new int[kHexPrismTriCnt * 3]{} };
  size_t triangle_cnt = 0;
  for (size_t i = 1; i + 1 < vtx_cnt; i++) {
    // Basal face. Face number 1
    triangle_idx[triangle_cnt * 3 + 0] = 0;
    triangle_idx[triangle_cnt * 3 + 1] = i;
    triangle_idx[triangle_cnt * 3 + 2] = (i + 1) % vtx_cnt;
    triangle_cnt++;
  }
  for (size_t i = 0; i < vtx_cnt; i++) {
    // Prism face first half
    triangle_idx[triangle_cnt * 3 + 0] = i;
    triangle_idx[triangle_cnt * 3 + 1] = i + vtx_cnt;
    triangle_idx[triangle_cnt * 3 + 2] = (i + 1) % vtx_cnt;
    triangle_cnt++;
    // Prism face second half
    triangle_idx[triangle_cnt * 3 + 0] = i + vtx_cnt;
    triangle_idx[triangle_cnt * 3 + 1] = (i + 1) % vtx_cnt + vtx_cnt;
    triangle_idx[triangle_cnt * 3 + 2] = (i + 1) % vtx_cnt;
    triangle_cnt++;
  }
  for (size_t i = 1; i + 1 < vtx_cnt; i++) {
    // Basal face. Face number 2
    triangle_idx[triangle_cnt * 3 + 0] = vtx_cnt;
    triangle_idx[triangle_cnt * 3 + 1] = (i + 1) % vtx_cnt + vtx_cnt;
    triangle_idx[triangle_cnt * 3 + 2] = i + vtx_cnt;
    triangle_cnt++;
  }

  return Mesh(vtx_cnt * 2, std::move(vtx), triangle_cnt, std::move(triangle_idx));
}


Mesh CreatePyramidMesh(float h1, float h2, float h3) {
  using math::kSqrt3_4;

  float h2_2 = h2 / 2.0f;
  float z1 = h2_2 + kIceCrystalC / 2.0f * h1;
  float z3 = -h2_2 - kIceCrystalC / 2.0f * h3;
  float r1 = 1 - h1;
  float r3 = 1 - h3;
  std::unique_ptr<float[]> vtx{ new float[kHexPyramidVtxCnt * 3]{
      r1 * kSqrt3_4,  r1 * -1.0f / 4.0f, z1,     // upper 0: vtx1
      r1 * kSqrt3_4,  r1 * 1.0f / 4.0f,  z1,     // upper 0: vtx2
      r1 * 0.0f,      r1 * 1.0f / 2.0f,  z1,     // upper 0: vtx3
      r1 * -kSqrt3_4, r1 * 1.0f / 4.0f,  z1,     // upper 0: vtx4
      r1 * -kSqrt3_4, r1 * -1.0f / 4.0f, z1,     // upper 0: vtx5
      r1 * 0.0f,      r1 * -1.0f / 2.0f, z1,     // upper 0: vtx6
      kSqrt3_4,       -1.0f / 4.0f,      h2_2,   // upper 1: vtx1
      kSqrt3_4,       1.0f / 4.0f,       h2_2,   // upper 1: vtx2
      0.0f,           1.0f / 2.0f,       h2_2,   // upper 1: vtx3
      -kSqrt3_4,      1.0f / 4.0f,       h2_2,   // upper 1: vtx4
      -kSqrt3_4,      -1.0f / 4.0f,      h2_2,   // upper 1: vtx5
      0.0f,           -1.0f / 2.0f,      h2_2,   // upper 1: vtx6
      kSqrt3_4,       -1.0f / 4.0f,      -h2_2,  // lower 2: vtx1
      kSqrt3_4,       1.0f / 4.0f,       -h2_2,  // lower 2: vtx2
      0.0f,           1.0f / 2.0f,       -h2_2,  // lower 2: vtx3
      -kSqrt3_4,      1.0f / 4.0f,       -h2_2,  // lower 2: vtx4
      -kSqrt3_4,      -1.0f / 4.0f,      -h2_2,  // lower 2: vtx5
      0.0f,           -1.0f / 2.0f,      -h2_2,  // lower 2: vtx6
      r3 * kSqrt3_4,  r3 * -1.0f / 4.0f, z3,     // lower 3: vtx1
      r3 * kSqrt3_4,  r3 * 1.0f / 4.0f,  z3,     // lower 3: vtx2
      r3 * 0.0f,      r3 * 1.0f / 2.0f,  z3,     // lower 3: vtx3
      r3 * -kSqrt3_4, r3 * 1.0f / 4.0f,  z3,     // lower 3: vtx4
      r3 * -kSqrt3_4, r3 * -1.0f / 4.0f, z3,     // lower 3: vtx5
      r3 * 0.0f,      r3 * -1.0f / 2.0f, z3,     // lower 3: vtx6
  } };

  std::unique_ptr<int[]> tri{ new int[kHexPyramidTriCnt * 3]{
      0,  1,  2,   // upper basal: fn1
      0,  2,  3,   // upper basal: fn1
      3,  4,  5,   // upper basal: fn1
      3,  5,  0,   // upper basal: fn1
      0,  6,  1,   // pyramid: fn13
      6,  7,  1,   // pyramid: fn13
      1,  7,  2,   // pyramid: fn14
      7,  8,  2,   // pyramid: fn14
      2,  8,  3,   // pyramid: fn15
      8,  9,  3,   // pyramid: fn15
      3,  9,  4,   // pyramid: fn16
      9,  10, 4,   // pyramid: fn16
      4,  10, 5,   // pyramid: fn17
      10, 11, 5,   // pyramid: fn17
      5,  11, 0,   // pyramid: fn18
      11, 6,  0,   // pyramid: fn18
      6,  12, 7,   // prism: fn3
      12, 13, 7,   // prism: fn3
      7,  13, 8,   // prism: fn4
      13, 14, 8,   // prism: fn4
      8,  14, 9,   // prism: fn5
      14, 15, 9,   // prism: fn5
      9,  15, 10,  // prism: fn6
      15, 16, 10,  // prism: fn6
      10, 16, 11,  // prism: fn7
      16, 17, 11,  // prism: fn7
      11, 17, 6,   // prism: fn8
      17, 12, 6,   // prism: fn8
      12, 18, 13,  // pyramid: fn23
      18, 19, 13,  // pyramid: fn23
      13, 19, 14,  // pyramid: fn24
      19, 20, 14,  // pyramid: fn24
      14, 20, 15,  // pyramid: fn25
      20, 21, 15,  // pyramid: fn25
      15, 21, 16,  // pyramid: fn26
      21, 22, 16,  // pyramid: fn26
      16, 22, 17,  // pyramid: fn27
      22, 23, 17,  // pyramid: fn27
      17, 23, 12,  // pyramid: fn28
      23, 18, 12,  // pyramid: fn28
      24, 26, 25,  // lower basal: fn2
      24, 27, 26,  // lower basal: fn2
      27, 29, 28,  // lower basal: fn2
      27, 24, 29,  // lower basal: fn2
  } };

  return Mesh(kHexPyramidVtxCnt, std::move(vtx), kHexPyramidTriCnt, std::move(tri));
}


constexpr int kHexPyramidPlaneCnt = 20;


std::array<float, kHexPyramidPlaneCnt * 4> FillGeneralPyramidCoef(float upper_alpha, float lower_alpha,  //
                                                                  float h1, float h2, float h3,          //
                                                                  const float* dist) {                   //
  using math::kPi_3;
  using math::kPi_6;

  constexpr int kUpperPyrOffset = 2;
  constexpr int kLowerPyrOffset = 14;
  constexpr int kPriOffset = 8;

  float h2_2 = h2 / 2.0f;
  float a1 = math::kSqrt3_4 / std::tan(upper_alpha * math::kDegreeToRad);
  float a2 = math::kSqrt3_4 / std::tan(lower_alpha * math::kDegreeToRad);

  std::array<float, kHexPyramidPlaneCnt * 4> coef{};
  auto* coef_ptr = coef.data();

  coef[2] = 1.0f;
  coef[6] = -1.0f;

  // Step 1. Fill coefficients except basal surfaces.
  for (int i = 0; i < 6; i++) {
    // upper pyramidal
    float x1 = 0.5f * std::cos(-kPi_6 + i * kPi_3);
    float x2 = 0.5f * std::cos(kPi_6 + i * kPi_3);
    float y1 = 0.5f * std::sin(-kPi_6 + i * kPi_3);
    float y2 = 0.5f * std::sin(kPi_6 + i * kPi_3);
    float det = x1 * y2 - x2 * y1;
    coef[(i + kUpperPyrOffset) * 4 + 0] = a1 * (y2 - y1);
    coef[(i + kUpperPyrOffset) * 4 + 1] = a1 * (x1 - x2);
    coef[(i + kUpperPyrOffset) * 4 + 2] = det;
    coef[(i + kUpperPyrOffset) * 4 + 3] = -(h2_2 + a1 * dist[i]) * det;
    // prismatic
    coef[(i + kPriOffset) * 4 + 0] = y2 - y1;
    coef[(i + kPriOffset) * 4 + 1] = x1 - x2;
    coef[(i + kPriOffset) * 4 + 2] = 0;
    coef[(i + kPriOffset) * 4 + 3] = -dist[i] * det;
    // lower pyramidal
    coef[(i + kLowerPyrOffset) * 4 + 0] = a2 * (y2 - y1);
    coef[(i + kLowerPyrOffset) * 4 + 1] = a2 * (x1 - x2);
    coef[(i + kLowerPyrOffset) * 4 + 2] = -det;
    coef[(i + kLowerPyrOffset) * 4 + 3] = -(h2_2 + a2 * dist[i]) * det;
  }

  // Step 2. Find out min and max z value, and complete coefficient array.
  float z_max = std::numeric_limits<float>::lowest();
  float z_min = std::numeric_limits<float>::max();
  float xyz[3];
  for (int i = 2; i < kHexPyramidPlaneCnt; i++) {
    for (int j = i + 1; j < kHexPyramidPlaneCnt; j++) {
      for (int k = j + 1; k < kHexPyramidPlaneCnt; k++) {
        // Find an intersection point
        if (!SolvePlanes(coef_ptr + i * 4, coef_ptr + j * 4, coef_ptr + k * 4, xyz)) {
          continue;
        }
        // Check if it is inner point.
        if (IsInPolyhedron3(kHexPyramidPlaneCnt - 2, coef_ptr + 8, xyz)) {
          if (xyz[2] > z_max) {
            z_max = xyz[2];
          }
          if (xyz[2] < z_min) {
            z_min = xyz[2];
          }
        }
      }
    }
  }
  coef[3] = (-z_max + h2_2) * h1 - h2_2;
  coef[7] = (z_min + h2_2) * h3 - h2_2;

  return coef;
}


Mesh CreatePyramidMesh(int upper_idx1, int upper_idx4, int lower_idx1, int lower_idx4,  // Miller index
                       float h1, float h2, float h3,                                    // height
                       const float* dist) {                                             // face distance
  float upper_alpha = std::atan(math::kSqrt3_2 * upper_idx4 / upper_idx1 / kIceCrystalC) * math::kRadToDegree;
  float lower_alpha = std::atan(math::kSqrt3_2 * lower_idx4 / lower_idx1 / kIceCrystalC) * math::kRadToDegree;

  return CreatePyramidMesh(upper_alpha, lower_alpha, h1, h2, h3, dist);
}


Mesh CreatePyramidMesh(float upper_alpha, float lower_alpha,  // wedge angle
                       float h1, float h2, float h3,          // height
                       const float* dist) {                   // face distance
  // Step 1. Construct coefficients.
  auto coef = FillGeneralPyramidCoef(upper_alpha, lower_alpha, h1, h2, h3, dist);

  // Step 2. Find out all inner points.
  auto [vtx, vtx_cnt] = SolveConvexPolyhedronVtx(kHexPyramidPlaneCnt, coef.data());

  // Step 3. Find all plannar faces
  auto plannar_faces = CollectSurfaceVtx(vtx_cnt, vtx.get(), kHexPyramidPlaneCnt, coef.data());

  // Step 4. Triangulation
  auto [tri, tri_cnt] = Triangulate(vtx_cnt, vtx.get(), plannar_faces);

  return Mesh(vtx_cnt, std::move(vtx), tri_cnt, std::move(tri));
}

}  // namespace v3
}  // namespace icehalo
