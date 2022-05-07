#include "core/geo3d.hpp"

#include <cstddef>
#include <memory>
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

void Mesh::SetVtx(const float* data) {
  std::memcpy(vertices_.get(), data, vtx_cnt_ * 3 * sizeof(float));
}

void Mesh::SetTriangle(const int* idx) {
  std::memcpy(triangle_.get(), idx, triangle_cnt_ * 3 * sizeof(int));
}

float* Mesh::GetVtxPtr(size_t idx) {
  return vertices_.get() + idx * 3;
}

int* Mesh::GetTrianglePtr(size_t idx) {
  return triangle_.get() + idx * 3;
}


Mesh Mesh::CreateIrregularPrism(float h, const float* dist) {
  using math::kSqrt3_2;

  // a*x + b*y + c <= 0, (a, b, c)
  const float kCoef[6 * 3]{
    1.0f,  0.0f,      -dist[0],  //
    0.5f,  kSqrt3_2,  -dist[1],  //
    -0.5f, kSqrt3_2,  -dist[2],  //
    -1.0f, 0.0f,      -dist[3],  //
    -0.5f, -kSqrt3_2, -dist[4],  //
    0.5f,  -kSqrt3_2, -dist[5],  //
  };

  std::unique_ptr<float[]> vtx{ new float[kHexPrismVtxCnt * 3]{} };
  for (int i = 0; i < 6; i++) {
    int i1 = i;
    int i2 = (i + 5) % 6;
    SolveLinear2(kCoef + i1 * 3, kCoef + i2 * 3, vtx.get() + i * 3);
    vtx[i * 3 + 2] = h / 2.0f;
  }
  // Filter out invalid vertices
  size_t vtx_cnt = 0;
  for (size_t i = 0; i < 6; i++) {
    // Check every plane
    if (IsInPolygon2(6, kCoef, vtx.get() + i * 3)) {
      if (vtx_cnt < i) {
        std::memcpy(vtx.get() + vtx_cnt * 3, vtx.get() + i * 3, 3 * sizeof(float));
      }
    } else {
      int i1 = (i + 1) % 6;
      int i2 = (i + 5) % 6;
      SolveLinear2(kCoef + i1 * 3, kCoef + i2 * 3, vtx.get() + vtx_cnt * 3);
      vtx[vtx_cnt * 3 + 2] = h / 2.0f;
      i++;
    }
    vtx_cnt++;
  }

  std::memcpy(vtx.get() + vtx_cnt * 3, vtx.get(), vtx_cnt * 3 * sizeof(float));
  for (int i = 0; i < 6; i++) {
    vtx[vtx_cnt * 3 + i * 3 + 2] = -h / 2.0f;
  }

  std::unique_ptr<int[]> triangle_idx{ new int[kHexPrismTriCnt * 3]{} };
  size_t triangle_cnt = 0;
  for (size_t i = 1; i + 1 < vtx_cnt; i++) {
    // fn1
    triangle_idx[triangle_cnt * 3 + 0] = 0;
    triangle_idx[triangle_cnt * 3 + 1] = i;
    triangle_idx[triangle_cnt * 3 + 2] = (i + 1) % vtx_cnt;
    triangle_cnt++;
  }
  for (size_t i = 0; i < vtx_cnt; i++) {
    // prism face 0
    triangle_idx[triangle_cnt * 3 + 0] = i;
    triangle_idx[triangle_cnt * 3 + 1] = i + vtx_cnt;
    triangle_idx[triangle_cnt * 3 + 2] = (i + 1) % vtx_cnt;
    triangle_cnt++;
    // prism face 1
    triangle_idx[triangle_cnt * 3 + 0] = i + vtx_cnt;
    triangle_idx[triangle_cnt * 3 + 1] = (i + 1) % vtx_cnt + vtx_cnt;
    triangle_idx[triangle_cnt * 3 + 2] = (i + 1) % vtx_cnt;
    triangle_cnt++;
  }
  for (size_t i = 1; i + 1 < vtx_cnt; i++) {
    // fn2
    triangle_idx[triangle_cnt * 3 + 0] = vtx_cnt;
    triangle_idx[triangle_cnt * 3 + 1] = (i + 1) % vtx_cnt + vtx_cnt;
    triangle_idx[triangle_cnt * 3 + 2] = i + vtx_cnt;
    triangle_cnt++;
  }

  return Mesh(vtx_cnt * 2, std::move(vtx), triangle_cnt, std::move(triangle_idx));
}

}  // namespace v3
}  // namespace icehalo
