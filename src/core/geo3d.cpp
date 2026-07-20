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
#include <vector>

#include "core/math.hpp"
#include "util/logger.hpp"

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

  auto& rng = RandomNumberGenerator::GetInstance();
  for (size_t i = 0; i < sample_num; i++) {
    auto curr_p = rng.GetUniform();
    out[i] = detail::RandomSampleSelectBin(curr_p, p, pop_size);
  }
}

namespace detail {
int RandomSampleSelectBin(float curr_p, const float* p, int pop_size) {
  for (int j = 0; j < pop_size; j++) {
    if (p[j] < curr_p && curr_p <= p[j + 1]) {
      return j;
    }
  }
  // No-match fallback: curr_p == 0.0 is the only path here (intervals cover (0, 1]).
  // MSVC STL uniform_real_distribution<float> can return exactly 0.0; libc++/libstdc++
  // do not, so this branch is inert on Mac/gcc (parity zero risk). The first j with
  // p[j+1] > p[j] is the first positive-weight bin, which equals the inverse-CDF limit
  // at x=0. Without this, callers would silently keep their default index (e.g.
  // InitRay_p_fid leaving tri_id=0), which can pick a zero-weight bin (the 77H leak).
  for (int j = 0; j < pop_size; j++) {
    if (p[j + 1] > p[j]) {
      return j;
    }
  }
  // Unreachable under a valid cumulative array (p[pop_size]==1 guarantees at least one
  // positive-weight bin). Only an all-zero-weight input (p[pop_size]==0) reaches here;
  // that violates the caller contract, so we fall back to bin 0 defensively.
  return 0;
}
}  // namespace detail


void SampleTrianglePoint(const float* vertices, float* out_pt, size_t sample_num) {
  auto& rng = RandomNumberGenerator::GetInstance();
  float e1[3]{ vertices[3] - vertices[0], vertices[4] - vertices[1], vertices[5] - vertices[2] };
  float e2[3]{ vertices[6] - vertices[0], vertices[7] - vertices[1], vertices[8] - vertices[2] };
  for (size_t i = 0; i < sample_num; i++) {
    auto u = rng.GetUniform();
    auto v = rng.GetUniform();
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

  auto& rng = RandomNumberGenerator::GetInstance();
  float c_cap = std::cos(cap_radii);
  float c_lon = std::cos(lon);
  float s_lon = std::sin(lon);
  float c_lat = std::cos(lat);
  float s_lat = std::sin(lat);
  for (size_t i = 0; i < sample_num; i++) {
    // 1. Sample arount x-axis
    float x = rng.GetUniform();
    x += (1 - x) * c_cap;
    float r = std::sqrt(1.0f - x * x);

    float u = rng.GetUniform() * 2 * math::kPi;
    float y = std::cos(u) * r;
    float z = std::sin(u) * r;

    // 2. Then rotate
    // R = Rz(lon).Ry(-lat)
    //     | cos(lon)cos(lat), -sin(lon), -cos(lon)sin(lat) |
    //   = | sin(lon)cos(lat),  cos(lon), -sin(lon)sin(lat) |
    //     | sin(lat),          0,         cos(lat)         |
    auto* p = reinterpret_cast<float*>(reinterpret_cast<uint8_t*>(out_pt) + i * step);
    p[0] = c_lon * c_lat * x - s_lon * y - c_lon * s_lat * z;
    p[1] = s_lon * c_lat * x + c_lon * y - s_lon * s_lat * z;
    p[2] = s_lat * x + c_lat * z;
  }
}

void SampleSph(float radii, float* out_pt, size_t sample_num) {
  auto& rng = RandomNumberGenerator::GetInstance();
  for (size_t i = 0; i < sample_num; i++) {
    auto z = rng.GetUniform() * 2 - 1;
    float r = std::sqrt(1.0f - z * z);
    float q = rng.GetUniform() * 2 * math::kPi;
    out_pt[i * 3 + 0] = std::cos(q) * r;
    out_pt[i * 3 + 1] = std::sin(q) * r;
    out_pt[i * 3 + 2] = z;
  }
  for (size_t i = 0; i < sample_num * 3; i++) {
    out_pt[i] *= radii;
  }
}

void SampleBall(float radii, float* out_pt, size_t sample_num) {
  auto& rng = RandomNumberGenerator::GetInstance();
  for (size_t i = 0; i < sample_num; i++) {
    float u = rng.GetUniform();
    float r = std::cbrt(u) * radii;
    float z = (rng.GetUniform() * 2 - 1.0f) * r;
    float rr = std::sqrt(r * r - z * z);
    float q = rng.GetUniform() * 2 * math::kPi;
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

  // Alpha range protection: valid pyramid wedge angles are in (0°, 90°) exclusive.
  // At alpha=0: tan→0, a1→inf (face becomes vertical, degenerate to prism face).
  // At alpha=90: tan→inf, a1→0 (face becomes horizontal, degenerate to basal face).
  // At alpha>90: tan<0, producing invalid (inverted) normals.
  // Miller index atan(sqrt3/2 * i4/i1 / c) always returns (0°, 90°) for positive i1, i4.
  constexpr float kMinPyramidAlpha = 0.1f;   // degrees — below this, a1 overflows
  constexpr float kMaxPyramidAlpha = 89.9f;  // degrees — above this, face degenerates to basal
  bool has_upper = h1 > math::kFloatEps && upper_alpha >= kMinPyramidAlpha && upper_alpha <= kMaxPyramidAlpha;
  bool has_lower = h3 > math::kFloatEps && lower_alpha >= kMinPyramidAlpha && lower_alpha <= kMaxPyramidAlpha;

  // Zero-volume degenerate guard (task-280.6): when both pyramidal caps are dropped
  // (wedge > kMaxPyramidAlpha or h1/h3 == 0) AND the prism segment h2 ≈ 0, the upper
  // and lower basal faces coincide → zero-thickness hexagon. Downstream Triangulate
  // computes Vec3FromTo(body_center, face_center) → (0,0,0), and Normalize3 produces
  // NaN normals that silently poison the renderer. Emit a warning and short-circuit
  // to an empty plane set, mirroring the kMaxPyramidAlpha degradation pattern.
  // (doc/numerical-robustness.md §5: use <eps, not ==0.)
  // TODO(tech-debt): the basal+prism plane equations above are written into out_coef
  // before this early return — wasted writes but no UB since callers honor the
  // returned plane_cnt as the effective length. A future refactor could hoist
  // has_upper/has_lower computation to the top of the function for true early exit.
  if (!has_upper && !has_lower && h2 < math::kFloatEps) {
    LOG_WARNING("FillHexCrystalCoef: zero-volume crystal (prism_h={:.4e}, no valid pyramidal faces); skipping",
                static_cast<double>(h2));
    return 0;
  }

  // Upper pyramidal faces (6, if h1 > 0)
  if (has_upper) {
    float a1 = math::kSqrt3_4 / std::tan(upper_alpha * math::kDegreeToRad);
    for (int i = 0; i < 6; i++) {
      float x1 = 0.5f * std::cos(-kPi_6 + i * kPi_3);
      float x2 = 0.5f * std::cos(kPi_6 + i * kPi_3);
      float y1 = 0.5f * std::sin(-kPi_6 + i * kPi_3);
      float y2 = 0.5f * std::sin(kPi_6 + i * kPi_3);
      float det = x1 * y2 - x2 * y1;
      // Normal direction is fixed by Miller index (alpha), independent of face distance.
      // dist[i] only shifts the plane position via the constant term d.
      out_coef[cnt * 4 + 0] = a1 * (y2 - y1);
      out_coef[cnt * 4 + 1] = a1 * (x1 - x2);
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
      out_coef[cnt * 4 + 0] = a2 * (y2 - y1);
      out_coef[cnt * 4 + 1] = a2 * (x1 - x2);
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
    // Pyramid: solve for z_max/z_min from non-basal planes.
    // Computed in double internally to absorb float32 cancellation at extreme
    // wedges (≥ 88.5°), then narrowed back when writing the basal d values.
    // task-geometry-gen-numerical-robustness Step 4.
    std::vector<double> coef_d(cnt * 4);
    for (size_t i = 0; i < cnt * 4; i++) {
      coef_d[i] = static_cast<double>(out_coef[i]);
    }
    // Plane-triple intersection + containment both delegate to single-source
    // math.hpp helpers (SolvePlanesD scale-invariant det; IsInPolyhedron3D with
    // kIncidenceEpsD=1e-5 absorbing float-input precision residuals).
    double z_max = std::numeric_limits<double>::lowest();
    double z_min = std::numeric_limits<double>::max();
    double xyz[3];
    // Track whether any triple-plane intersection landed inside the polyhedron.
    // A dedicated boolean is used instead of comparing z_max/z_min against their
    // sentinel initial values: those sentinels flow through the arithmetic below
    // (yielding +inf in both basal d values when the loop never fired), so
    // any equality/threshold check against them would be brittle. Independent
    // state answers the semantic question directly — "did the loop find any
    // feasible vertex?" — without depending on the sentinel's numeric value.
    bool found_feasible_vertex = false;
    for (size_t i = 2; i < cnt; i++) {
      for (size_t j = i + 1; j < cnt; j++) {
        for (size_t k = j + 1; k < cnt; k++) {
          if (!SolvePlanesD(coef_d.data() + i * 4, coef_d.data() + j * 4, coef_d.data() + k * 4, xyz)) {
            continue;
          }
          if (IsInPolyhedron3D(static_cast<int>(cnt - 2), coef_d.data() + 8, xyz)) {
            z_max = std::max(z_max, xyz[2]);
            z_min = std::min(z_min, xyz[2]);
            found_feasible_vertex = true;
          }
        }
      }
    }
    // Empty-feasible-region guard: when random face_distance makes the
    // half-space intersection empty, the triple-loop never enters the
    // IsInPolyhedron3D branch and z_max/z_min retain their sentinel initial
    // values (lowest()/max()). Without this guard the arithmetic below emits
    // two +inf basal-d coefficients; downstream a currently unrelated Euler
    // check happens to reject the resulting F=0 mesh, but that containment is
    // not this function's responsibility. Degrade to the same "return 0"
    // pattern the zero-volume branch above uses; callers already handle
    // plane_cnt==0 as an empty crystal.
    if (!found_feasible_vertex) {
      LOG_WARNING(
          "FillHexCrystalCoef: empty pyramid feasible region (upper_alpha={:.3f}, "
          "lower_alpha={:.3f}, h1={:.4e}, h2={:.4e}, h3={:.4e}); skipping",
          static_cast<double>(upper_alpha), static_cast<double>(lower_alpha), static_cast<double>(h1),
          static_cast<double>(h2), static_cast<double>(h3));
      return 0;
    }
    auto h1_d = static_cast<double>(h1);
    auto h3_d = static_cast<double>(h3);
    auto h2_2_d = static_cast<double>(h2_2);
    out_coef[3] = static_cast<float>((-z_max + h2_2_d) * h1_d - h2_2_d);
    out_coef[7] = static_cast<float>((z_min + h2_2_d) * h3_d - h2_2_d);
  }

  return cnt;
}


// ====== Unified convex polyhedron mesh creation ======

Mesh CreateConvexPolyhedronMesh(int plane_cnt, const float* coef) {
  // Double-precision geometry-gen pipeline (task-geometry-gen-numerical-robustness
  // Step 4): vertex solve + co-planar grouping run in double internally to keep
  // mesh assembly stable on extreme-wedge geometry (≥ 88.5°), where float32
  // precision-loss collapses apex/anti-apex vertices into the basal ring.
  // Public coef input and Mesh output stay float — no API surface change.
  auto [vtx, vtx_cnt] = SolveConvexPolyhedronVtxD(plane_cnt, coef);
  auto faces = CollectSurfaceVtxD(vtx_cnt, vtx.get(), plane_cnt, coef);
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
  // Guard against i1=0: float division yields inf, atan(inf)=90° which is outside valid range.
  // Explicit guard avoids relying on IEEE 754 edge behavior.
  float upper_alpha =
      upper_idx1 != 0 ? std::atan(math::kSqrt3_2 * upper_idx4 / upper_idx1 / kIceCrystalC) * math::kRadToDegree : 0.0f;
  float lower_alpha =
      lower_idx1 != 0 ? std::atan(math::kSqrt3_2 * lower_idx4 / lower_idx1 / kIceCrystalC) * math::kRadToDegree : 0.0f;
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
    coef[(i + kUpperPyrOffset) * 4 + 0] = -a1 * (y2 - y1);              // negative polyhedron
    coef[(i + kUpperPyrOffset) * 4 + 1] = -a1 * (x1 - x2);              // negative polyhedron
    coef[(i + kUpperPyrOffset) * 4 + 2] = -det;                         // negative polyhedron
    coef[(i + kUpperPyrOffset) * 4 + 3] = (h2_2 + a1 * dist[i]) * det;  // negative polyhedron
    // prismatic
    coef[(i + kPriOffset) * 4 + 0] = y2 - y1;
    coef[(i + kPriOffset) * 4 + 1] = x1 - x2;
    coef[(i + kPriOffset) * 4 + 2] = 0;
    coef[(i + kPriOffset) * 4 + 3] = -dist[i] * det;
    // lower pyramidal
    coef[(i + kLowerPyrOffset) * 4 + 0] = -a2 * (y2 - y1);              // negative polyhedron
    coef[(i + kLowerPyrOffset) * 4 + 1] = -a2 * (x1 - x2);              // negative polyhedron
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
  // Guard against i1=0: float division yields inf, atan(inf)=90° which is outside valid range.
  // Explicit guard avoids relying on IEEE 754 edge behavior.
  float upper_alpha =
      upper_idx1 != 0 ? std::atan(math::kSqrt3_2 * upper_idx4 / upper_idx1 / kIceCrystalC) * math::kRadToDegree : 0.0f;
  float lower_alpha =
      lower_idx1 != 0 ? std::atan(math::kSqrt3_2 * lower_idx4 / lower_idx1 / kIceCrystalC) * math::kRadToDegree : 0.0f;
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
