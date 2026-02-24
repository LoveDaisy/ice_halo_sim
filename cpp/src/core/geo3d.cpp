#include "core/geo3d.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <limits>
#include <memory>
#include <tuple>
#include <utility>

#include "core/math.hpp"

namespace lumice {

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

  constexpr int kMaxStackPopSize = 64;
  float p_stack[kMaxStackPopSize + 1];
  std::unique_ptr<float[]> p_heap;
  float* p = p_stack;
  if (pop_size > kMaxStackPopSize) {
    p_heap = std::make_unique<float[]>(pop_size + 1);
    p = p_heap.get();
  }
  p[0] = 0;
  std::memcpy(p + 1, weight, pop_size * sizeof(float));
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
    : vtx_cnt_(vtx_cnt), triangle_cnt_(triangle_cnt), vertices_(std::make_unique<float[]>(vtx_cnt * 3)),
      triangle_(std::make_unique<int[]>(triangle_cnt * 3)) {}

Mesh::Mesh(size_t vtx_cnt, std::unique_ptr<float[]> vtx, size_t triangle_cnt, std::unique_ptr<int[]> triangle_idx)
    : vtx_cnt_(vtx_cnt), triangle_cnt_(triangle_cnt), vertices_(std::move(vtx)), triangle_(std::move(triangle_idx)) {}

Mesh::Mesh(const Mesh& other)
    : vtx_cnt_(other.vtx_cnt_), triangle_cnt_(other.triangle_cnt_),
      vertices_(std::make_unique<float[]>(other.vtx_cnt_ * 3)),
      triangle_(std::make_unique<int[]>(other.triangle_cnt_ * 3)) {
  std::memcpy(vertices_.get(), other.vertices_.get(), vtx_cnt_ * 3 * sizeof(float));
  std::memcpy(triangle_.get(), other.triangle_.get(), triangle_cnt_ * 3 * sizeof(int));
}

Mesh::Mesh(Mesh&& other) noexcept
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
  vertices_ = std::make_unique<float[]>(vtx_cnt_ * 3);
  triangle_ = std::make_unique<int[]>(triangle_cnt_ * 3);
  std::memcpy(vertices_.get(), other.vertices_.get(), vtx_cnt_ * 3 * sizeof(float));
  std::memcpy(triangle_.get(), other.triangle_.get(), triangle_cnt_ * 3 * sizeof(int));
  return *this;
}

Mesh& Mesh::operator=(Mesh&& other) noexcept {
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


// ====== Unified hex crystal plane equations ======

size_t FillHexCrystalCoef(float upper_alpha, float lower_alpha, float h1, float h2, float h3, const float* dist,
                          float* out_coef) {
  using math::kPi_3;
  using math::kPi_6;

  float h2_2 = h2 / 2.0f;
  size_t cnt = 0;

  // Upper basal: (0, 0, 1, d) — d filled later
  out_coef[0] = 0;
  out_coef[1] = 0;
  out_coef[2] = 1.0f;
  out_coef[3] = 0;
  cnt++;

  // Lower basal: (0, 0, -1, d) — d filled later
  out_coef[4] = 0;
  out_coef[5] = 0;
  out_coef[6] = -1.0f;
  out_coef[7] = 0;
  cnt++;

  // Prism faces (always 6)
  for (int i = 0; i < 6; i++) {
    float x1 = 0.5f * std::cos(-kPi_6 + i * kPi_3);
    float x2 = 0.5f * std::cos(kPi_6 + i * kPi_3);
    float y1 = 0.5f * std::sin(-kPi_6 + i * kPi_3);
    float y2 = 0.5f * std::sin(kPi_6 + i * kPi_3);
    float det = x1 * y2 - x2 * y1;
    out_coef[cnt * 4 + 0] = y2 - y1;
    out_coef[cnt * 4 + 1] = x1 - x2;
    out_coef[cnt * 4 + 2] = 0;
    out_coef[cnt * 4 + 3] = -dist[i] * det;
    cnt++;
  }

  bool has_upper = h1 > math::kFloatEps;
  bool has_lower = h3 > math::kFloatEps;

  // Upper pyramidal faces (6, if h1 > 0)
  if (has_upper) {
    float a1 = math::kSqrt3_4 / std::tan(upper_alpha * math::kDegreeToRad);
    for (int i = 0; i < 6; i++) {
      float x1 = 0.5f * std::cos(-kPi_6 + i * kPi_3);
      float x2 = 0.5f * std::cos(kPi_6 + i * kPi_3);
      float y1 = 0.5f * std::sin(-kPi_6 + i * kPi_3);
      float y2 = 0.5f * std::sin(kPi_6 + i * kPi_3);
      float det = x1 * y2 - x2 * y1;
      out_coef[cnt * 4 + 0] = a1 * (y2 - y1) * dist[i];
      out_coef[cnt * 4 + 1] = a1 * (x1 - x2) * dist[i];
      out_coef[cnt * 4 + 2] = det;
      out_coef[cnt * 4 + 3] = -(h2_2 + a1 * dist[i]) * det;
      cnt++;
    }
  }

  // Lower pyramidal faces (6, if h3 > 0)
  if (has_lower) {
    float a2 = math::kSqrt3_4 / std::tan(lower_alpha * math::kDegreeToRad);
    for (int i = 0; i < 6; i++) {
      float x1 = 0.5f * std::cos(-kPi_6 + i * kPi_3);
      float x2 = 0.5f * std::cos(kPi_6 + i * kPi_3);
      float y1 = 0.5f * std::sin(-kPi_6 + i * kPi_3);
      float y2 = 0.5f * std::sin(kPi_6 + i * kPi_3);
      float det = x1 * y2 - x2 * y1;
      out_coef[cnt * 4 + 0] = a2 * (y2 - y1) * dist[i];
      out_coef[cnt * 4 + 1] = a2 * (x1 - x2) * dist[i];
      out_coef[cnt * 4 + 2] = -det;
      out_coef[cnt * 4 + 3] = -(h2_2 + a2 * dist[i]) * det;
      cnt++;
    }
  }

  // Set basal face d values
  if (!has_upper && !has_lower) {
    // Pure prism: d = -h2/2
    out_coef[3] = -h2_2;
    out_coef[7] = -h2_2;
  } else {
    // Pyramid: solve for z_max/z_min from non-basal planes
    float z_max = std::numeric_limits<float>::lowest();
    float z_min = std::numeric_limits<float>::max();
    float xyz[3];
    for (size_t i = 2; i < cnt; i++) {
      for (size_t j = i + 1; j < cnt; j++) {
        for (size_t k = j + 1; k < cnt; k++) {
          if (!SolvePlanes(out_coef + i * 4, out_coef + j * 4, out_coef + k * 4, xyz)) {
            continue;
          }
          if (IsInPolyhedron3(static_cast<int>(cnt - 2), out_coef + 8, xyz)) {
            z_max = std::max(z_max, xyz[2]);
            z_min = std::min(z_min, xyz[2]);
          }
        }
      }
    }
    out_coef[3] = (-z_max + h2_2) * h1 - h2_2;
    out_coef[7] = (z_min + h2_2) * h3 - h2_2;
  }

  return cnt;
}


// ====== Unified convex polyhedron mesh creation ======

Mesh CreateConvexPolyhedronMesh(int plane_cnt, const float* coef) {
  auto [vtx, vtx_cnt] = SolveConvexPolyhedronVtx(plane_cnt, coef);
  auto faces = CollectSurfaceVtx(vtx_cnt, vtx.get(), plane_cnt, coef);
  auto [tri, tri_cnt] = Triangulate(vtx_cnt, vtx.get(), faces);
  return Mesh(vtx_cnt, std::move(vtx), tri_cnt, std::move(tri));
}


// ====== Prism ======

Mesh CreatePrismMesh(float h) {
  float dist[6]{ 1, 1, 1, 1, 1, 1 };
  float coef[kMaxHexCrystalPlanes * 4];
  auto cnt = FillHexCrystalCoef(0, 0, 0, h, 0, dist, coef);
  return CreateConvexPolyhedronMesh(static_cast<int>(cnt), coef);
}


Mesh CreatePrismMesh(float h, const float* dist) {
  float coef[kMaxHexCrystalPlanes * 4];
  auto cnt = FillHexCrystalCoef(0, 0, 0, h, 0, dist, coef);
  return CreateConvexPolyhedronMesh(static_cast<int>(cnt), coef);
}


// ====== Pyramid ======

Mesh CreatePyramidMesh(float h1, float h2, float h3) {
  float dist[6]{ 1, 1, 1, 1, 1, 1 };
  float alpha = std::atan(math::kSqrt3_2 / kIceCrystalC) * math::kRadToDegree;
  float coef[kMaxHexCrystalPlanes * 4];
  auto cnt = FillHexCrystalCoef(alpha, alpha, h1, h2, h3, dist, coef);
  return CreateConvexPolyhedronMesh(static_cast<int>(cnt), coef);
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
  float coef[kMaxHexCrystalPlanes * 4];
  auto cnt = FillHexCrystalCoef(upper_alpha, lower_alpha, h1, h2, h3, dist, coef);
  return CreateConvexPolyhedronMesh(static_cast<int>(cnt), coef);
}


// ====== Constants for concave pyramid (kept for FillConcavePyramidCoef) ======
constexpr int kHexPyramidPlaneCnt = 20;
constexpr int kUpperPyramidPlaneCnt = 6;
constexpr int kLowerPyramidPlaneCnt = 6;
constexpr int kMiddlePrismPlaneCnt = 6;


// NOLINTNEXTLINE(readability-function-cognitive-complexity)
std::array<float, kHexPyramidPlaneCnt * 4> FillConcavePyramidCoef(float upper_alpha, float lower_alpha,  //
                                                                  float h1, float h2, float h3,          //
                                                                  const float* dist) {                   //
  using math::kPi_3;
  using math::kPi_6;

  constexpr int kUpperPyrOffset = 2;
  constexpr int kPriOffset = 8;
  constexpr int kLowerPyrOffset = 14;

  float h2_2 = h2 / 2.0f;
  float a1 = -math::kSqrt3_4 / std::tan(upper_alpha * math::kDegreeToRad);
  float a2 = -math::kSqrt3_4 / std::tan(lower_alpha * math::kDegreeToRad);

  // Faces: [upper basal, lower basal, upper pyramidal, lower pyramidal, prismatic]
  std::array<float, kHexPyramidPlaneCnt * 4> coef{};

  coef[2] = -1.0f;  // negative polyhedron
  coef[6] = 1.0f;   // negative polyhedron

  // Step 1. Fill coefficients except basal surfaces.
  for (int i = 0; i < 6; i++) {
    // upper pyramidal
    float x1 = 0.5f * std::cos(-kPi_6 + i * kPi_3);
    float x2 = 0.5f * std::cos(kPi_6 + i * kPi_3);
    float y1 = 0.5f * std::sin(-kPi_6 + i * kPi_3);
    float y2 = 0.5f * std::sin(kPi_6 + i * kPi_3);
    float det = x1 * y2 - x2 * y1;
    coef[(i + kUpperPyrOffset) * 4 + 0] = -a1 * (y2 - y1) * dist[i];    // negative polyhedron
    coef[(i + kUpperPyrOffset) * 4 + 1] = -a1 * (x1 - x2) * dist[i];    // negative polyhedron
    coef[(i + kUpperPyrOffset) * 4 + 2] = -det;                         // negative polyhedron
    coef[(i + kUpperPyrOffset) * 4 + 3] = (h2_2 + a1 * dist[i]) * det;  // negative polyhedron
    // prismatic
    coef[(i + kPriOffset) * 4 + 0] = y2 - y1;
    coef[(i + kPriOffset) * 4 + 1] = x1 - x2;
    coef[(i + kPriOffset) * 4 + 2] = 0;
    coef[(i + kPriOffset) * 4 + 3] = -dist[i] * det;
    // lower pyramidal
    coef[(i + kLowerPyrOffset) * 4 + 0] = -a2 * (y2 - y1) * dist[i];    // negative polyhedron
    coef[(i + kLowerPyrOffset) * 4 + 1] = -a2 * (x1 - x2) * dist[i];    // negative polyhedron
    coef[(i + kLowerPyrOffset) * 4 + 2] = det;                          // negative polyhedron
    coef[(i + kLowerPyrOffset) * 4 + 3] = (h2_2 + a2 * dist[i]) * det;  // negative polyhedron
  }

  // Step 2. Find out min z value for upper pyramid, and max z value for lower pyramid, then complete coefficient array.
  float xyz[3];
  const auto* coef_ptr = coef.data();
  // Upper
  float z_min = std::numeric_limits<float>::max();  // For upper pyramid
  for (int i = kUpperPyrOffset; i < kPriOffset; i++) {
    for (int j = i + 1; j < kPriOffset; j++) {
      for (int k = j + 1; k < kPriOffset; k++) {
        // Find an intersection point
        if (!SolvePlanes(coef_ptr + i * 4, coef_ptr + j * 4, coef_ptr + k * 4, xyz)) {
          continue;
        }
        // Check if it is inner point.
        if (IsInPolyhedron3(kUpperPyramidPlaneCnt, coef_ptr + kUpperPyrOffset, xyz)) {
          if (xyz[2] < z_min) {
            z_min = xyz[2];
          }
        }
      }
    }
  }
  coef[3] = (z_min - h2_2) * h1 + h2_2;  // negative polyhedron

  // Lower
  float z_max = std::numeric_limits<float>::lowest();  // For lower pyramid
  for (int i = kLowerPyrOffset; i < kHexPyramidPlaneCnt; i++) {
    for (int j = i + 1; j < kHexPyramidPlaneCnt; j++) {
      for (int k = j + 1; k < kHexPyramidPlaneCnt; k++) {
        // Find an intersection point
        if (!SolvePlanes(coef_ptr + i * 4, coef_ptr + j * 4, coef_ptr + k * 4, xyz)) {
          continue;
        }
        // Check if it is inner point.
        if (IsInPolyhedron3(kLowerPyramidPlaneCnt, coef_ptr + kLowerPyrOffset, xyz)) {
          if (xyz[2] > z_max) {
            z_max = xyz[2];
          }
        }
      }
    }
  }
  coef[7] = -(z_max + h2_2) * h3 + h2_2;  // negative polyhedron

  return coef;
}


Mesh CreateConcavePyramidMesh(float h1, float h2, float h3) {
  auto dist = std::make_unique<float[]>(6);
  std::fill(dist.get(), dist.get() + 6, 1.0f);
  return CreateConcavePyramidMesh(1, 1, 1, 1, h1, h2, h3, dist.get());
}


// NOLINTNEXTLINE(readability-function-size)
Mesh CreateConcavePyramidMesh(int upper_idx1, int upper_idx4, int lower_idx1, int lower_idx4,  // Miller index
                              float h1, float h2, float h3,                                    // height
                              const float* dist) {                                             // face distance
  float upper_alpha = std::atan(math::kSqrt3_2 * upper_idx4 / upper_idx1 / kIceCrystalC) * math::kRadToDegree;
  float lower_alpha = std::atan(math::kSqrt3_2 * lower_idx4 / lower_idx1 / kIceCrystalC) * math::kRadToDegree;
  return CreateConcavePyramidMesh(upper_alpha, lower_alpha, h1, h2, h3, dist);
}


Mesh CreateConcavePyramidMesh(float upper_alpha, float lower_alpha,  // wedge angle
                              float h1, float h2, float h3,          // height
                              const float* dist) {                   // face distance
  // Step 1. Construct coefficients
  // surface order:
  // upper basal, lower basal (negative)
  // upper pyramidal (negative)
  // prismatic
  // lower pyramidal (negative)
  auto coef = FillConcavePyramidCoef(upper_alpha, lower_alpha, h1, h2, h3, dist);
  const auto* coef_ptr = coef.data();

  // Step 2. Find all vertices
  // Negative upper pyramid
  auto upper_negative_pyramid_coef = std::make_unique<float[]>((kUpperPyramidPlaneCnt + kMiddlePrismPlaneCnt + 1) * 4);
  std::memcpy(upper_negative_pyramid_coef.get(), coef_ptr, 4 * sizeof(float));
  std::memcpy(upper_negative_pyramid_coef.get() + 4, coef_ptr + 4 * 2, 4 * 6 * sizeof(float));
  std::memcpy(upper_negative_pyramid_coef.get() + 28, coef_ptr + 4 * 8, 4 * 6 * sizeof(float));
  // Negative lower pyramid
  auto lower_negative_pyramid_coef = std::make_unique<float[]>((kLowerPyramidPlaneCnt + kMiddlePrismPlaneCnt + 1) * 4);
  std::memcpy(lower_negative_pyramid_coef.get(), coef_ptr, 4 * sizeof(float));
  std::memcpy(lower_negative_pyramid_coef.get() + 4, coef_ptr + 4 * 2, 4 * 6 * sizeof(float));
  std::memcpy(lower_negative_pyramid_coef.get() + 28, coef_ptr + 4 * 8, 4 * 6 * sizeof(float));

  auto [upper_vtx, upper_vtx_cnt] = ConvexPolyhedronDifferenceVtx(
      kUpperPyramidPlaneCnt + kMiddlePrismPlaneCnt + 1, upper_negative_pyramid_coef.get(),
      kLowerPyramidPlaneCnt + kMiddlePrismPlaneCnt + 1, lower_negative_pyramid_coef.get());
  auto [lower_vtx, lower_vtx_cnt] = ConvexPolyhedronDifferenceVtx(
      kLowerPyramidPlaneCnt + kMiddlePrismPlaneCnt + 1, lower_negative_pyramid_coef.get(),
      kUpperPyramidPlaneCnt + kMiddlePrismPlaneCnt + 1, upper_negative_pyramid_coef.get());

  auto vtx = std::make_unique<float[]>((upper_vtx_cnt + lower_vtx_cnt) * 3);
  std::memcpy(vtx.get(), upper_vtx.get(), upper_vtx_cnt * 3 * sizeof(float));
  std::memcpy(vtx.get() + upper_vtx_cnt * 3, lower_vtx.get(), lower_vtx_cnt * 3 * sizeof(float));

  // Step 3. Collect vertices by surface.
  auto plannar_faces = CollectSurfaceVtx(upper_vtx_cnt + lower_vtx_cnt, vtx.get(), kHexPyramidPlaneCnt, coef_ptr);

  // Step 4. Triangulation
  auto [tri, tri_cnt] = Triangulate(upper_vtx_cnt + lower_vtx_cnt, vtx.get(), plannar_faces);

  return Mesh(upper_vtx_cnt + lower_vtx_cnt, std::move(vtx), tri_cnt, std::move(tri));
}

}  // namespace lumice
