#include "core/math.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <limits>
#include <memory>

#include "util/logger.hpp"


namespace lumice {

bool FloatEqual(float a, float b, float threshold) {
  return std::abs(a - b) < threshold;
}


bool FloatEqualZero(float a, float threshold) {
  return a > -threshold && a < threshold;
}


float Dot3(const float* vec1, const float* vec2) {
  return vec1[0] * vec2[0] + vec1[1] * vec2[1] + vec1[2] * vec2[2];
}


void Cross3(const float* vec1, const float* vec2, float* vec) {
  vec[0] = -vec2[1] * vec1[2] + vec1[1] * vec2[2];
  vec[1] = vec2[0] * vec1[2] - vec1[0] * vec2[2];
  vec[2] = -vec2[0] * vec1[1] + vec1[0] * vec2[1];
}


float Norm3(const float* vec) {
  return std::sqrt(Dot3(vec, vec));
}


float DiffNorm3(const float* vec1, const float* vec2) {
  float v[3];
  Vec3FromTo(vec1, vec2, v);
  return Norm3(v);
}


void Normalize3(float* vec) {
  float len = Norm3(vec);
  vec[0] /= len;
  vec[1] /= len;
  vec[2] /= len;
}


void Normalized3(const float* vec, float* vec_n) {
  float len = Norm3(vec);
  vec_n[0] = vec[0] / len;
  vec_n[1] = vec[1] / len;
  vec_n[2] = vec[2] / len;
}


void Vec3FromTo(const float* vec1, const float* vec2, float* vec) {
  vec[0] = vec2[0] - vec1[0];
  vec[1] = vec2[1] - vec1[1];
  vec[2] = vec2[2] - vec1[2];
}


void TriangleNormal(const float* p1, const float* p2, const float* p3, float* normal) {
  float v1[3]{};
  float v2[3]{};
  Vec3FromTo(p1, p2, v1);
  Vec3FromTo(p1, p3, v2);
  Cross3(v1, v2, normal);
  Normalize3(normal);
}


template <typename T>
Vec3<T>::Vec3(T x, T y, T z) : val_{ x, y, z } {}


template <typename T>
Vec3<T>::Vec3(const T* data) : val_{ data[0], data[1], data[2] } {}


template <typename T>
const T* Vec3<T>::val() const {
  return val_;
}


template <typename T>
void Vec3<T>::val(T x, T y, T z) {
  val_[0] = x;
  val_[1] = y;
  val_[2] = z;
}


template <typename T>
void Vec3<T>::val(const T* data) {
  val_[0] = data[0];
  val_[1] = data[1];
  val_[2] = data[2];
}


template <typename T>
T Vec3<T>::x() const {
  return val_[0];
}


template <typename T>
T Vec3<T>::y() const {
  return val_[1];
}


template <typename T>
T Vec3<T>::z() const {
  return val_[2];
}


template <typename T>
void Vec3<T>::x(T x) {
  val_[0] = x;
}


template <typename T>
void Vec3<T>::y(T y) {
  val_[1] = y;
}


template <typename T>
void Vec3<T>::z(T z) {
  val_[2] = z;
}


template <typename T>
Vec3<T> Vec3<T>::Normalized() {
  return Vec3<T>::Normalized(*this);
}


template <typename T>
void Vec3<T>::Normalize() {
  Normalize3(val_);
}


bool operator==(const Vec3f& lhs, const Vec3f& rhs) {
  for (int i = 0; i < 3; i++) {
    if (!FloatEqual(lhs.val()[i], rhs.val()[i])) {
      return false;
    }
  }
  return true;
}


template <typename T>
Vec3<T> Vec3<T>::Normalized(const Vec3<T>& v) {
  T data[3];
  Normalized3(v.val_, data);
  return Vec3<T>(data);
}


template <typename T>
Vec3<T>& Vec3<T>::operator+=(const Vec3<T>& v) {
  for (int i = 0; i < 3; i++) {
    val_[i] += v.val_[i];
  }
  return *this;
}


template <typename T>
Vec3<T>& Vec3<T>::operator+=(T a) {
  for (auto& i : val_) {
    i += a;
  }
  return *this;
}


template <typename T>
Vec3<T>& Vec3<T>::operator-=(const Vec3<T>& v) {
  for (int i = 0; i < 3; i++) {
    val_[i] -= v.val_[i];
  }
  return *this;
}


template <typename T>
Vec3<T>& Vec3<T>::operator-=(T a) {
  for (auto& i : val_) {
    i -= a;
  }
  return *this;
}


template <typename T>
Vec3<T>& Vec3<T>::operator/=(T a) {
  for (auto& i : val_) {
    i /= a;
  }
  return *this;
}


template <typename T>
Vec3<T>& Vec3<T>::operator*=(T a) {
  for (auto& i : val_) {
    i *= a;
  }
  return *this;
}


template <typename T>
T Vec3<T>::Dot(const Vec3<T>& v1, const Vec3<T>& v2) {
  return Dot3(v1.val_, v2.val_);
}


template <typename T>
T Vec3<T>::Norm(const Vec3<T>& v) {
  return Norm3(v.val_);
}


template <typename T>
Vec3<T> Vec3<T>::Cross(const Vec3<T>& v1, const Vec3<T>& v2) {
  T data[3];
  Cross3(v1.val_, v2.val_, data);
  return Vec3<T>(data);
}


template <typename T>
Vec3<T> Vec3<T>::FromTo(const Vec3<T>& v1, const Vec3<T>& v2) {
  T data[3];
  Vec3FromTo(v1.val_, v2.val_, data);
  return Vec3<T>(data);
}

template class Vec3<float>;


template <class T>
Pose3<T>::Pose3(const T* data, AngleUnit unit) : val_(data), unit_(unit) {}

template <class T>
Pose3<T>::Pose3(T lon, T lat, T roll, AngleUnit unit) : val_(lon, lat, roll), unit_(unit) {}

template <class T>
T Pose3<T>::lon() const {
  return val_.x();
}

template <class T>
T Pose3<T>::lat() const {
  return val_.y();
}

template <class T>
T Pose3<T>::roll() const {
  return val_.z();
}

template <class T>
void Pose3<T>::lon(T lon) {
  val_.x(lon);
}

template <class T>
void Pose3<T>::lat(T lat) {
  val_.y(lat);
}

template <class T>
void Pose3<T>::roll(T roll) {
  val_.z(roll);
}

template <class T>
const T* Pose3<T>::val() const {
  return val_.val();
}

template <class T>
void Pose3<T>::val(const T* data) {
  val_.val(data);
}

template <class T>
void Pose3<T>::val(T lat, T lon, T roll) {
  val_.val(lat, lon, roll);
}

template <class T>
AngleUnit Pose3<T>::unit() const {
  return unit_;
}

template <class T>
void Pose3<T>::ToDegree() {
  if (unit_ == AngleUnit::kRad) {
    val_ *= math::kRadToDegree;
  }
}

template <class T>
void Pose3<T>::ToRad() {
  if (unit_ == AngleUnit::kDegree) {
    val_ *= math::kDegreeToRad;
  }
}

template class Pose3<float>;


TriangleIdx::TriangleIdx(ShortIdType id1, ShortIdType id2, ShortIdType id3) : idx_{ id1, id2, id3 } {}


const ShortIdType* TriangleIdx::idx() const {
  return idx_;
}


RandomNumberGenerator::RandomNumberGenerator(uint32_t seed)
    : seed_(seed), generator_{ static_cast<std::mt19937::result_type>(seed) } {}


RandomNumberGenerator& RandomNumberGenerator::GetInstance() {
  static thread_local RandomNumberGenerator instance{ static_cast<uint32_t>(
      std::chrono::system_clock::now().time_since_epoch().count()) };
  return instance;
}


float RandomNumberGenerator::GetGaussian() {
  return gauss_dist_(generator_);
}


float RandomNumberGenerator::GetUniform() {
  return uniform_dist_(generator_);
}


float RandomNumberGenerator::Get(Distribution dist) {
  switch (dist.type) {
    case DistributionType::kUniform:
      return (GetUniform() - 0.5f) * dist.std + dist.mean;
    case DistributionType::kGaussian:
      return GetGaussian() * dist.std + dist.mean;
    case DistributionType::kZigzag:
      // Rectified arcsine: |A·sin(2πU) + B| where A=std (amplitude), B=mean (tilt offset).
      return std::abs(dist.std * std::sin(GetUniform() * 2.0f * math::kPi) + dist.mean);
    case DistributionType::kLaplacian: {
      // Laplace inverse CDF: μ - b·sign(U-0.5)·ln(1-2|U-0.5|), returns degrees.
      float u = GetUniform();
      float sign = (u < 0.5f) ? -1.0f : 1.0f;
      float arg = 1.0f - 2.0f * std::abs(u - 0.5f);
      arg = std::max(arg, std::numeric_limits<float>::min());  // Clamp to avoid ln(0).
      return dist.mean - dist.std * sign * std::log(arg);
    }
    case DistributionType::kNoRandom:
      return dist.mean;
    default:
      return 0.0f;
  }
}


void RandomNumberGenerator::Reset() {
  generator_.seed(seed_);
}


void RandomNumberGenerator::SetSeed(uint32_t seed) {
  seed_ = seed;
  generator_.seed(seed_);
}


void RandomSampler::SampleSphericalPointsSph(float* data, size_t num, size_t step) {
  auto& rng = RandomNumberGenerator::GetInstance();
  for (size_t i = 0; i < num; i++) {
    float u = rng.GetUniform() * 2 - 1;
    float lambda = rng.GetUniform() * 2 * math::kPi;

    data[i * step + 0] = lambda;
    data[i * step + 1] = std::asin(u);
  }
}


namespace {
// Compute the Jacobian rejection envelope constant M for a given latitude distribution.
// M = max(cos(phi)) over the proposal's support, used in acceptance probability cos(phi)/M.
// Tighter M → higher acceptance rate. All inputs in degrees.
// NOTE: For extreme configs where |mean| + std > 90, NormalizeLatitude folding may produce
// latitudes near ±90° where cos(phi) → 0, causing low acceptance rates but not incorrectness.
float ComputeJacobianEnvelope(const Distribution& dist) {
  switch (dist.type) {
    case DistributionType::kGaussian:
      // Proposal range ~ [mean - 3σ, mean + 3σ]; cos is maximized at latitude closest to equator.
      return std::cos(std::max(std::abs(dist.mean) - 3.0f * dist.std, 0.0f) * math::kDegreeToRad);
    case DistributionType::kZigzag:
      // Proposal range: |std·sin(2πU) + mean|.
      // When |mean| >= std: [|mean|-std, |mean|+std], M = cos((|mean|-std)°).
      // When |mean| < std: [0, |mean|+std], M = cos(0) = 1.
      return std::cos(std::max(std::abs(dist.mean) - dist.std, 0.0f) * math::kDegreeToRad);
    case DistributionType::kLaplacian:
      // Laplace tails heavier than Gaussian; use 5b instead of 3σ (covers 99.3% of mass).
      // Samples beyond 5b are rare and handled correctly by NormalizeLatitude + rejection.
      return std::cos(std::max(std::abs(dist.mean) - 5.0f * dist.std, 0.0f) * math::kDegreeToRad);
    case DistributionType::kUniform:
    default:
      return 1.0f;
  }
}
}  // namespace


void RandomSampler::SampleSphericalPointsSph(const AxisDistribution& axis_dist, float* data, size_t num) {
  auto& rng = RandomNumberGenerator::GetInstance();

  // Jacobian correction for latitude sampling.
  // Target distribution: p(phi) ∝ proposal(phi) × cos(phi)
  // where cos(phi) = sin(colatitude) is the spherical area element Jacobian.
  //
  // Three paths:
  //   1. Rayleigh (Gaussian near-pole only): 2D Gaussian → Rayleigh, exact for sin(θ)≈θ.
  //   2. Generic rejection: propose from distribution, accept with prob cos(phi)/M.
  //      Covers kGaussian (non-Rayleigh), kUniform, kZigzag, and future types.
  //   3. kNoRandom: single deterministic orientation, no Jacobian needed.
  constexpr float kPolarThresholdRad = 0.5f * math::kDegreeToRad;  // Rayleigh only for colatitude < 0.5°
  constexpr int kMaxRejectionAttempts = 1000;

  auto lat_type = axis_dist.latitude_dist.type;

  // Rayleigh path: Gaussian-only optimization for near-pole distributions.
  bool use_rayleigh = false;
  float latitude_mean_rad = 0;
  float sigma_rad = 0;
  if (lat_type == DistributionType::kGaussian) {
    latitude_mean_rad = axis_dist.latitude_dist.mean * math::kDegreeToRad;
    sigma_rad = axis_dist.latitude_dist.std * math::kDegreeToRad;
    float colatitude_center = math::kPi_2 - std::abs(latitude_mean_rad);
    use_rayleigh = (colatitude_center + 3.0f * sigma_rad) < kPolarThresholdRad;
  }

  // Precompute rejection envelope for all non-Rayleigh, non-kNoRandom types.
  float rejection_m = (lat_type != DistributionType::kNoRandom && !use_rayleigh) ?
                          ComputeJacobianEnvelope(axis_dist.latitude_dist) :
                          1.0f;

  for (size_t i = 0; i < num; i++) {
    float phi = 0;
    bool flip = false;

    if (lat_type == DistributionType::kGaussian && use_rayleigh) {
      // Rayleigh path: 2D Gaussian in tangent plane at pole → Rayleigh angular distance.
      // Guard ensures colatitude_center + 3σ < 0.5°, so colatitude rarely exceeds valid range.
      float dx = rng.GetGaussian() * sigma_rad;
      float dy = rng.GetGaussian() * sigma_rad;
      float colatitude = std::sqrt(dx * dx + dy * dy);
      phi = std::copysign(math::kPi_2 - colatitude, latitude_mean_rad);
      phi = std::max(-math::kPi_2, std::min(math::kPi_2, phi));
      // flip stays false — Rayleigh path only fires for tiny σ near poles.
    } else if (lat_type != DistributionType::kNoRandom) {
      // Generic Jacobian rejection path (kGaussian non-Rayleigh, kUniform, kZigzag, etc.).
      int attempts = 0;
      do {
        phi = rng.Get(axis_dist.latitude_dist) * math::kDegreeToRad;
        auto [norm_phi, norm_flip] = detail::NormalizeLatitude(phi);
        phi = norm_phi;
        flip = norm_flip;
        ++attempts;
        if (attempts >= kMaxRejectionAttempts) {
          LOG_WARNING("SampleSphericalPointsSph: rejection safety valve triggered (mean={}, std={})",
                      axis_dist.latitude_dist.mean, axis_dist.latitude_dist.std);
          break;
        }
      } while (rng.GetUniform() >= std::cos(phi) / rejection_m);
    } else {
      // kNoRandom: no Jacobian needed (single deterministic orientation).
      phi = rng.Get(axis_dist.latitude_dist) * math::kDegreeToRad;
    }

    float lambda = rng.Get(axis_dist.azimuth_dist) * math::kDegreeToRad;
    if (flip) {
      lambda += math::kPi;
    }

    data[i * 2 + 0] = lambda;
    data[i * 2 + 1] = phi;
  }
}


AxisDistribution::AxisDistribution()
    : azimuth_dist{ DistributionType::kNoRandom, 0, 0 }, latitude_dist{ DistributionType::kNoRandom, 90.0f, 0 },
      roll_dist{ DistributionType::kNoRandom, 0, 0 } {}


std::pair<float, bool> detail::NormalizeLatitude(float latitude_rad) {
  float theta = math::kPi_2 - latitude_rad;    // latitude → colatitude
  theta = std::fmod(theta, 2.0f * math::kPi);  // periodic normalization
  if (theta < 0) {
    theta += 2.0f * math::kPi;  // ensure [0, 2π)
  }
  bool flip = theta > math::kPi;
  if (flip) {
    theta = 2.0f * math::kPi - theta;  // reflect to [0, π]
  }
  return { math::kPi_2 - theta, flip };  // colatitude → latitude
}


bool AxisDistribution::IsFullSphereUniform() const {
  return azimuth_dist.type == DistributionType::kUniform && FloatEqual(azimuth_dist.mean, 0.0f) &&
         FloatEqual(azimuth_dist.std, 360.0f) && latitude_dist.type == DistributionType::kUniform &&
         FloatEqual(latitude_dist.mean, 90.0f) && FloatEqual(latitude_dist.std, 360.0f);
}


void to_json(nlohmann::json& obj, const Distribution& dist) {
  if (dist.type == DistributionType::kNoRandom) {
    obj = dist.mean;
  } else {
    obj["type"] = dist.type;
    obj["mean"] = dist.mean;
    obj["std"] = dist.std;
  }
}

void from_json(const nlohmann::json& obj, Distribution& dist) {
  if (obj.is_number()) {
    dist.type = DistributionType::kNoRandom;
    obj.get_to(dist.mean);
  } else if (obj.is_object()) {
    if (obj.contains("type")) {
      obj.at("type").get_to(dist.type);
    }
    if (obj.contains("mean")) {
      obj.at("mean").get_to(dist.mean);
    }
    if (obj.contains("std")) {
      obj.at("std").get_to(dist.std);
    }
  } else {
    LOG_ERROR("Cannot recognize distribution!");
  }
}

void to_json(nlohmann::json& obj, const AxisDistribution& axis) {
  // Zenith: internal latitude → external zenith (mean = 90 - lat).
  // Must handle kNoRandom (serialized as number) vs others (serialized as object).
  nlohmann::json zenith;
  to_json(zenith, axis.latitude_dist);
  if (zenith.is_number()) {
    zenith = 90.0f - axis.latitude_dist.mean;
  } else {
    zenith["mean"] = 90.0f - axis.latitude_dist.mean;
  }
  obj["zenith"] = zenith;
  obj["azimuth"] = axis.azimuth_dist;
  obj["roll"] = axis.roll_dist;
}

void from_json(const nlohmann::json& obj, AxisDistribution& axis) {
  obj.at("zenith").get_to(axis.latitude_dist);
  axis.latitude_dist.mean = 90.0f - axis.latitude_dist.mean;

  axis.azimuth_dist.type = DistributionType::kUniform;
  axis.azimuth_dist.mean = 0.0f;
  axis.azimuth_dist.std = 360.0f;
  axis.roll_dist.type = DistributionType::kUniform;
  axis.roll_dist.mean = 0.0f;
  axis.roll_dist.std = 360.0f;

  if (obj.contains("azimuth")) {
    obj.at("azimuth").get_to(axis.azimuth_dist);
  }
  if (obj.contains("roll")) {
    obj.at("roll").get_to(axis.roll_dist);
  }
}


bool SolveLines(const float* coef1, const float* coef2, float* res) {
  float det = coef1[0] * coef2[1] - coef2[0] * coef1[1];
  if (FloatEqualZero(det)) {
    res[0] = std::numeric_limits<float>::quiet_NaN();
    res[1] = std::numeric_limits<float>::quiet_NaN();
    return false;
  }
  float x = (coef1[1] * coef2[2] - coef2[1] * coef1[2]) / det;
  float y = (coef1[2] * coef2[0] - coef2[2] * coef1[0]) / det;
  res[0] = x;
  res[1] = y;
  return true;
}


bool SolvePlanes(const float* coef1, const float* coef2, const float* coef3, float* res) {
  float det = coef1[0] * coef2[1] * coef3[2] + coef1[1] * coef2[2] * coef3[0] + coef1[2] * coef2[0] * coef3[1] -
              coef1[2] * coef2[1] * coef3[0] - coef1[0] * coef2[2] * coef3[1] - coef1[1] * coef2[0] * coef3[2];
  if (FloatEqualZero(det)) {
    res[0] = std::numeric_limits<float>::quiet_NaN();
    res[1] = std::numeric_limits<float>::quiet_NaN();
    res[2] = std::numeric_limits<float>::quiet_NaN();
    return false;
  }
  float x = coef1[3] * coef2[2] * coef3[1] + coef1[1] * coef2[3] * coef3[2] + coef1[2] * coef2[1] * coef3[3] -
            coef1[1] * coef2[2] * coef3[3] - coef1[2] * coef2[3] * coef3[1] - coef1[3] * coef2[1] * coef3[2];
  float y = coef1[0] * coef2[2] * coef3[3] + coef1[2] * coef2[3] * coef3[0] + coef1[3] * coef2[0] * coef3[2] -
            coef1[3] * coef2[2] * coef3[0] - coef1[0] * coef2[3] * coef3[2] - coef1[2] * coef2[0] * coef3[3];
  float z = coef1[3] * coef2[1] * coef3[0] + coef1[0] * coef2[3] * coef3[1] + coef1[1] * coef2[0] * coef3[3] -
            coef1[0] * coef2[1] * coef3[3] - coef1[1] * coef2[3] * coef3[0] - coef1[3] * coef2[0] * coef3[1];
  res[0] = x / det;
  res[1] = y / det;
  res[2] = z / det;
  return true;
}


bool IsInPolygon2(int n, const float* coef, const float xy[2], bool boundary) {
  auto th = boundary ? math::kFloatEps : -math::kFloatEps;
  bool in = true;
  float tmp[3]{ xy[0], xy[1], 1.0f };
  for (int j = 0; j < n; j++) {
    if (Dot3(coef + j * 3, tmp) > th) {
      in = false;
      break;
    }
  }
  return in;
}


bool IsInPolyhedron3(int n, const float* coef, const float xyz[3], bool boundary) {
  auto th = boundary ? math::kFloatEps : -math::kFloatEps;
  bool in = true;
  for (int j = 0; j < n; j++) {
    if (Dot3(coef + j * 4, xyz) + coef[j * 4 + 3] > th) {
      in = false;
      break;
    }
  }
  return in;
}


// NOLINTNEXTLINE(readability-function-cognitive-complexity)
std::tuple<std::unique_ptr<float[]>, int> SolveConvexPolyhedronVtx(int plane_cnt, const float* coef_ptr) {
  using math::kFloatEps;

  int vtx_cap = plane_cnt * (plane_cnt - 1) * (plane_cnt - 2) / 6;
  auto vtx = std::make_unique<float[]>(vtx_cap * 3);
  float* vtx_ptr = vtx.get();

  float xyz[3]{};
  int vtx_cnt = 0;
  for (int i = 0; i < plane_cnt; i++) {
    for (int j = i + 1; j < plane_cnt; j++) {
      for (int k = j + 1; k < plane_cnt; k++) {
        // Find an intersection point
        if (!SolvePlanes(coef_ptr + i * 4, coef_ptr + j * 4, coef_ptr + k * 4, xyz)) {
          continue;
        }
        // Check if it is inner point.
        if (!IsInPolyhedron3(plane_cnt, coef_ptr, xyz)) {
          continue;
        }
        // Check if it is already listed.
        bool listed = false;
        for (int m = 0; m < vtx_cnt; m++) {
          if (FloatEqualZero(DiffNorm3(vtx_ptr + m * 3, xyz), 2 * kFloatEps)) {
            listed = true;
            break;
          }
        }
        if (listed) {
          continue;
        }

        std::memcpy(vtx_ptr + vtx_cnt * 3, xyz, 3 * sizeof(float));
        vtx_cnt++;
      }
    }
  }

  auto final_vtx = std::make_unique<float[]>(vtx_cnt * 3);
  std::memcpy(final_vtx.get(), vtx.get(), vtx_cnt * 3 * sizeof(float));

  return std::make_tuple(std::move(final_vtx), vtx_cnt);
}


// NOLINTNEXTLINE(readability-function-cognitive-complexity)
std::tuple<std::unique_ptr<float[]>, int> ConvexPolyhedronDifferenceVtx(int plane_cnt1, const float* coef_ptr1,
                                                                        int plane_cnt2, const float* coef_ptr2) {
  // All polyhedron faces are defined by plane coefficients:
  //    ax + by + cz + d <= 0
  // Difference of two convex polyhedrons, A - B, is defined as points that are in A but not in B.

  using math::kFloatEps;

  int plane_cnt = plane_cnt1 + plane_cnt2;
  int vtx_cap = plane_cnt * (plane_cnt - 1) * (plane_cnt - 2) / 6;

  auto vtx = std::make_unique<float[]>(vtx_cap * 3);
  float* vtx_ptr = vtx.get();

  float xyz[3]{};
  int vtx_cnt = 0;
  for (int i = 0; i < plane_cnt; i++) {
    for (int j = i + 1; j < plane_cnt; j++) {
      for (int k = j + 1; k < plane_cnt; k++) {
        // Find an intersection point
        const auto* p1 = i < plane_cnt1 ? coef_ptr1 + i * 4 : coef_ptr2 + (i - plane_cnt1) * 4;
        const auto* p2 = j < plane_cnt1 ? coef_ptr1 + j * 4 : coef_ptr2 + (j - plane_cnt1) * 4;
        const auto* p3 = k < plane_cnt1 ? coef_ptr1 + k * 4 : coef_ptr2 + (k - plane_cnt1) * 4;
        if (!SolvePlanes(p1, p2, p3, xyz)) {
          continue;
        }
        // Check if it is in polyhedron 1
        if (!IsInPolyhedron3(plane_cnt1, coef_ptr1, xyz)) {
          continue;
        }
        // Check if it is out polyhedron 2
        if (IsInPolyhedron3(plane_cnt2, coef_ptr2, xyz, false)) {
          continue;
        }
        // Check if it is already listed.
        bool listed = false;
        for (int m = 0; m < vtx_cnt; m++) {
          if (FloatEqualZero(DiffNorm3(vtx_ptr + m * 3, xyz), 2 * kFloatEps)) {
            listed = true;
            break;
          }
        }
        if (listed) {
          continue;
        }

        std::memcpy(vtx_ptr + vtx_cnt * 3, xyz, 3 * sizeof(float));
        vtx_cnt++;
      }
    }
  }

  auto vtx_final = std::make_unique<float[]>(vtx_cnt * 3);
  std::memcpy(vtx_final.get(), vtx.get(), vtx_cnt * 3 * sizeof(float));
  return std::make_tuple(std::move(vtx_final), vtx_cnt);
}


// Helper function for CollectSurfaceVtx
std::vector<int> CollectSurfaceVtx(int vtx_cnt, const float* vtx_ptr,          //
                                   int checking_plane, const float* coef_ptr,  //
                                   const std::vector<std::set<int>>& checked_faces) {
  std::vector<int> curr_face;
  bool listed = false;
  for (int k = 0; k < vtx_cnt; k++) {
    if (!FloatEqualZero(Dot3(coef_ptr + checking_plane * 4, vtx_ptr + k * 3) + coef_ptr[checking_plane * 4 + 3])) {
      continue;
    }
    curr_face.emplace_back(k);
    // Check if it is already listed
    if (curr_face.size() == 3) {
      for (const auto& s : checked_faces) {
        if (s.count(curr_face[0]) && s.count(curr_face[1]) && s.count(curr_face[2])) {
          listed = true;
          curr_face.clear();
          break;
        }
      }
    }
    if (listed) {
      break;
    }
  }
  return curr_face;
}


// NOLINTNEXTLINE(readability-function-cognitive-complexity)
std::vector<std::set<int>> CollectSurfaceVtx(int vtx_cnt, const float* vtx_ptr, int plane_cnt, const float* coef_ptr) {
  std::vector<std::set<int>> plannar_faces;
  for (int i = 0; i < vtx_cnt; i++) {
    for (int j = i + 1; j < vtx_cnt; j++) {
      // Current edge is (i,j)
      // Find planes that meet at edge (i,j)
      int plane1 = -1;
      int plane2 = -1;
      for (int k = 0; k < plane_cnt; k++) {
        if (!FloatEqualZero(Dot3(coef_ptr + k * 4, vtx_ptr + i * 3) + coef_ptr[k * 4 + 3]) ||
            !FloatEqualZero(Dot3(coef_ptr + k * 4, vtx_ptr + j * 3) + coef_ptr[k * 4 + 3])) {
          continue;
        }
        if (plane1 < 0) {
          plane1 = k;
        } else {
          plane2 = k;
          break;
        }
      }
      if (plane1 < 0 || plane2 < 0) {
        continue;
      }

      // Count in all vertices on two planes
      // Plane 1
      auto face1 = CollectSurfaceVtx(vtx_cnt, vtx_ptr, plane1, coef_ptr, plannar_faces);
      if (!face1.empty()) {
        plannar_faces.emplace_back(face1.begin(), face1.end());
      }

      // Plane 2
      auto face2 = CollectSurfaceVtx(vtx_cnt, vtx_ptr, plane2, coef_ptr, plannar_faces);
      if (!face2.empty()) {
        plannar_faces.emplace_back(face2.begin(), face2.end());
      }
    }
  }
  return plannar_faces;
}


std::tuple<std::unique_ptr<int[]>, int> Triangulate(int vtx_cnt, const float* vtx_ptr,
                                                    const std::vector<std::set<int>>& surface_vtx_idx) {
  float vtx_center[3]{};
  for (int i = 0; i < vtx_cnt; i++) {
    for (int j = 0; j < 3; j++) {
      vtx_center[j] += vtx_ptr[i * 3 + j];
    }
  }
  for (float& x : vtx_center) {
    x /= vtx_cnt;
  }

  int tri_cap = 0;
  for (const auto& curr_face : surface_vtx_idx) {
    if (curr_face.size() < 3) {
      continue;
    }
    tri_cap += curr_face.size() - 2;
  }

  auto tri = std::make_unique<int[]>(tri_cap * 3);
  int tri_cnt = 0;
  for (const auto& curr_face : surface_vtx_idx) {
    if (curr_face.size() < 3) {
      continue;
    }

    float face_center[3]{};
    std::vector<int> curr_idx;
    for (auto idx : curr_face) {
      for (int i = 0; i < 3; i++) {
        face_center[i] += vtx_ptr[idx * 3 + i];
      }
      curr_idx.emplace_back(idx);
    }
    for (auto& x : face_center) {
      x /= curr_face.size();
    }

    // Sort by signed angle around face outward normal
    float ref_n[3]{};
    Vec3FromTo(vtx_center, face_center, ref_n);
    Normalize3(ref_n);

    float v0[3]{};
    Vec3FromTo(face_center, vtx_ptr + curr_idx[0] * 3, v0);
    Normalize3(v0);
    std::sort(curr_idx.begin(), curr_idx.end(), [=](int i1, int i2) {
      float v1[3]{};
      float v2[3]{};
      Vec3FromTo(face_center, vtx_ptr + i1 * 3, v1);
      Normalize3(v1);
      Vec3FromTo(face_center, vtx_ptr + i2 * 3, v2);
      Normalize3(v2);

      float n1[3]{};
      float n2[3]{};
      Cross3(v0, v1, n1);
      Cross3(v0, v2, n2);

      float s1 = Dot3(n1, ref_n);
      float s2 = Dot3(n2, ref_n);
      float c1 = Dot3(v0, v1);
      float c2 = Dot3(v0, v2);

      return atan2(s1, c1) < atan2(s2, c2);
    });

    // Check normal direction.
    float n0[3]{};
    TriangleNormal(vtx_ptr + curr_idx[0] * 3, vtx_ptr + curr_idx[1] * 3, vtx_ptr + curr_idx[2] * 3, n0);
    Vec3FromTo(vtx_center, vtx_ptr + curr_idx[0] * 3, v0);
    if (Dot3(v0, n0) < 0) {
      std::reverse(curr_idx.begin(), curr_idx.end());
    }

    // Fill triangle index
    tri[tri_cnt * 3 + 0] = curr_idx[0];
    tri[tri_cnt * 3 + 1] = curr_idx[1];
    tri[tri_cnt * 3 + 2] = curr_idx[2];
    tri_cnt++;
    for (size_t i = 3; i < curr_idx.size(); i++) {
      tri[tri_cnt * 3 + 0] = curr_idx[0];
      tri[tri_cnt * 3 + 1] = curr_idx[i - 1];
      tri[tri_cnt * 3 + 2] = curr_idx[i];
      tri_cnt++;
    }
  }

  return std::make_tuple(std::move(tri), tri_cnt);
}

}  // namespace lumice
