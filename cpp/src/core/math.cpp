#include "core/math.hpp"

#include <smmintrin.h>
#include <xmmintrin.h>

#include <algorithm>
#include <chrono>


namespace icehalo {

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


void RotateZ(const float* lon_lat_roll, const float* input_vec, float* output_vec, size_t data_num) {
  return RotateZ(lon_lat_roll, input_vec, output_vec, 3, 3, data_num);
}


void RotateZ(const float* lon_lat_roll,  // longitude, latitude, roll
             const float* input_vec,     // input data
             float* output_vec,          // output data
             size_t input_step, size_t output_step, size_t data_num) {
  using std::cos;
  using std::sin;

  const float ax[] = {
    -cos(lon_lat_roll[2]) * sin(lon_lat_roll[0]) - cos(lon_lat_roll[0]) * sin(lon_lat_roll[1]) * sin(lon_lat_roll[2]),
    cos(lon_lat_roll[0]) * cos(lon_lat_roll[2]) - sin(lon_lat_roll[0]) * sin(lon_lat_roll[1]) * sin(lon_lat_roll[2]),
    cos(lon_lat_roll[1]) * sin(lon_lat_roll[2]),
    -cos(lon_lat_roll[0]) * cos(lon_lat_roll[2]) * sin(lon_lat_roll[1]) + sin(lon_lat_roll[0]) * sin(lon_lat_roll[2]),
    -cos(lon_lat_roll[2]) * sin(lon_lat_roll[0]) * sin(lon_lat_roll[1]) - cos(lon_lat_roll[0]) * sin(lon_lat_roll[2]),
    cos(lon_lat_roll[1]) * cos(lon_lat_roll[2]),
    cos(lon_lat_roll[0]) * cos(lon_lat_roll[1]),
    cos(lon_lat_roll[1]) * sin(lon_lat_roll[0]),
    sin(lon_lat_roll[1]),
    0
  };

#if defined(__AVX__) && defined(__SSE4_1__)
  __m128 AX0 = _mm_loadu_ps(ax + 0);
  __m128 AX1 = _mm_loadu_ps(ax + 3);
  __m128 AX2 = _mm_loadu_ps(ax + 6);

  for (size_t i = 0; i < data_num; i++) {
    float* tmp_out = output_vec + i * output_step;

    __m128 INPUT_V = _mm_loadu_ps(input_vec + i * input_step);
    __m128 DP = _mm_dp_ps(INPUT_V, AX0, 0x71);
    tmp_out[0] = DP[0];
    DP = _mm_dp_ps(INPUT_V, AX1, 0x71);
    tmp_out[1] = DP[0];
    DP = _mm_dp_ps(INPUT_V, AX2, 0x71);
    tmp_out[2] = DP[0];
  }
#else
  // Then do the matrix multiplication (using Dot3 actually)
  for (decltype(data_num) i = 0; i < data_num; i++) {
    const float* tmp_v = input_vec + i * input_step;
    float* tmp_out = output_vec + i * output_step;
    for (int j = 0; j < 3; j++) {
      tmp_out[j] = Dot3(tmp_v, ax + j * 3);
    }
  }
#endif
}


void RotateZBack(const float* lon_lat_roll, const float* input_vec, float* output_vec, size_t data_num) {
  using std::cos;
  using std::sin;

  const float ax[] = {
    -cos(lon_lat_roll[2]) * sin(lon_lat_roll[0]) - cos(lon_lat_roll[0]) * sin(lon_lat_roll[1]) * sin(lon_lat_roll[2]),
    -cos(lon_lat_roll[0]) * cos(lon_lat_roll[2]) * sin(lon_lat_roll[1]) + sin(lon_lat_roll[0]) * sin(lon_lat_roll[2]),
    cos(lon_lat_roll[0]) * cos(lon_lat_roll[1]),
    cos(lon_lat_roll[0]) * cos(lon_lat_roll[2]) - sin(lon_lat_roll[0]) * sin(lon_lat_roll[1]) * sin(lon_lat_roll[2]),
    -cos(lon_lat_roll[2]) * sin(lon_lat_roll[0]) * sin(lon_lat_roll[1]) - cos(lon_lat_roll[0]) * sin(lon_lat_roll[2]),
    cos(lon_lat_roll[1]) * sin(lon_lat_roll[0]),
    cos(lon_lat_roll[1]) * sin(lon_lat_roll[2]),
    cos(lon_lat_roll[1]) * cos(lon_lat_roll[2]),
    sin(lon_lat_roll[1]),
    0
  };

#if defined(__AVX__) && defined(__SSE4_1__)
  __m128 AX0 = _mm_loadu_ps(ax + 0);
  __m128 AX1 = _mm_loadu_ps(ax + 3);
  __m128 AX2 = _mm_loadu_ps(ax + 6);

  for (size_t i = 0; i < data_num; i++) {
    float* tmp_out = output_vec + i * 3;

    __m128 INPUT_V = _mm_loadu_ps(input_vec + i * 3);
    __m128 DP = _mm_dp_ps(INPUT_V, AX0, 0x71);
    tmp_out[0] = DP[0];
    DP = _mm_dp_ps(INPUT_V, AX1, 0x71);
    tmp_out[1] = DP[0];
    DP = _mm_dp_ps(INPUT_V, AX2, 0x71);
    tmp_out[2] = DP[0];
  }
#else
  // Then do the matrix multiplication (using Dot3 actually)
  for (decltype(data_num) i = 0; i < data_num; i++) {
    const float* tmp_v = input_vec + i * 3;
    float* tmp_out = output_vec + i * 3;
    for (int j = 0; j < 3; j++) {
      tmp_out[j] = Dot3(tmp_v, ax + j * 3);
    }
  }
#endif
}


std::vector<Vec3f> FindInnerPoints(const HalfSpaceSet& hss) {
  float *a = hss.a, *b = hss.b, *c = hss.c, *d = hss.d;
  int n = hss.n;

  std::vector<Vec3f> pts;
  for (int i = 0; i < n; i++) {
    for (int j = i + 1; j < n; j++) {
      for (int k = j + 1; k < n; k++) {
        float det = a[k] * b[j] * c[i] - a[j] * b[k] * c[i] - a[k] * b[i] * c[j] + a[i] * b[k] * c[j] +
                    a[j] * b[i] * c[k] - a[i] * b[j] * c[k];
        if (std::abs(det) <= math::kFloatEps) {
          continue;
        }
        float x = -(b[k] * c[j] * d[i] - b[j] * c[k] * d[i] - b[k] * c[i] * d[j] + b[i] * c[k] * d[j] +
                    b[j] * c[i] * d[k] - b[i] * c[j] * d[k]) /
                  det;
        float y = -(-(a[k] * c[j] * d[i]) + a[j] * c[k] * d[i] + a[k] * c[i] * d[j] - a[i] * c[k] * d[j] -
                    a[j] * c[i] * d[k] + a[i] * c[j] * d[k]) /
                  det;
        float z = -(a[k] * b[j] * d[i] - a[j] * b[k] * d[i] - a[k] * b[i] * d[j] + a[i] * b[k] * d[j] +
                    a[j] * b[i] * d[k] - a[i] * b[j] * d[k]) /
                  det;

        bool in = true;
        for (int ii = 0; ii < n; ii++) {
          in = in && (a[ii] * x + b[ii] * y + c[ii] * z + d[ii] <= math::kFloatEps);
          if (!in) {
            break;
          }
        }
        if (in) {
          pts.emplace_back(x, y, z);
        }
      }
    }
  }

  return pts;
}


void SortAndRemoveDuplicate(std::vector<Vec3f>* pts) {
  /* Sort by coordinates */
  std::sort(pts->begin(), pts->end(), [=](const Vec3f& p1, const Vec3f& p2) {
    if (p1 == p2) {
      return false;
    }
    if (p1.x() < p2.x() - math::kFloatEps) {
      return true;
    }
    if (FloatEqual(p1.x(), p2.x()) && p1.y() < p2.y() - math::kFloatEps) {
      return true;
    }
    if (FloatEqual(p1.x(), p2.x()) && FloatEqual(p1.y(), p2.y()) && p1.z() < p2.z() - math::kFloatEps) {
      return true;
    }
    return false;
  });

  /* Remove duplicated points */
  for (auto iter = pts->begin(), last_iter = pts->begin(); iter != pts->end();) {
    if (iter != last_iter && (*iter) == (*last_iter)) {
      iter = pts->erase(iter);
    } else {
      last_iter = iter;
      ++iter;
    }
  }
}


std::vector<int> FindCoplanarPoints(const std::vector<Vec3f>& pts, const Vec3f& n0, float d0) {
  std::vector<int> pts_idx;
  for (decltype(pts.size()) j = 0; j < pts.size(); j++) {
    const auto& p = pts[j];
    if (FloatEqual(Vec3f::Dot(n0, p) + d0, 0)) {
      pts_idx.push_back(static_cast<int>(j));
    }
  }
  return pts_idx;
}


void BuildPolyhedronFaces(const HalfSpaceSet& hss, const std::vector<Vec3f>& pts,  // input
                          std::vector<TriangleIdx>& faces) {                       // output
  int num = hss.n;
  float *a = hss.a, *b = hss.b, *c = hss.c, *d = hss.d;

  for (int i = 0; i < num; i++) {
    /* Find co-planer points */
    Vec3f n0(a[i], b[i], c[i]);
    std::vector<int> face_pts_idx = FindCoplanarPoints(pts, n0, d[i]);
    if (face_pts_idx.empty()) {
      continue;
    }

    /* Build triangular division */
    BuildTriangularDivision(pts, n0, face_pts_idx, faces);
  }
}


void BuildTriangularDivision(const std::vector<Vec3f>& vertex, const Vec3f& n,              // input
                             std::vector<int>& pts_idx, std::vector<TriangleIdx>& faces) {  // output
  /* Find the center of co-planer points */
  Vec3f center(0.0f, 0.0f, 0.0f);
  for (auto p : pts_idx) {
    center += vertex[p];
  }
  center /= pts_idx.size();

  /* Sort by angle */
  int idx0 = pts_idx[0];
  std::sort(pts_idx.begin() + 1, pts_idx.end(), [&n, &vertex, &center, idx0](const int idx1, const int idx2) {
    Vec3f p0 = Vec3f::FromTo(center, vertex[idx0]).Normalized();
    Vec3f p1 = Vec3f::FromTo(center, vertex[idx1]).Normalized();
    Vec3f p2 = Vec3f::FromTo(center, vertex[idx2]).Normalized();

    Vec3f n1 = Vec3f::Cross(p0, p1);
    float dir = Vec3f::Dot(n1, n);
    float c1 = Vec3f::Dot(p0, p1);
    float s1 = std::abs(dir) > math::kFloatEps ? Vec3f::Norm(n1) * (dir / std::abs(dir)) : 0;
    float angle1 = atan2(s1, c1);
    angle1 += (angle1 < 0 ? 2 * math::kPi : 0);

    Vec3f n2 = Vec3f::Cross(p0, p2);
    dir = Vec3f::Dot(n2, n);
    float c2 = Vec3f::Dot(p0, p2);
    float s2 = std::abs(dir) > math::kFloatEps ? Vec3f::Norm(n2) * (dir / std::abs(dir)) : 0;
    float angle2 = atan2(s2, c2);
    angle2 += (angle2 < 0 ? 2 * math::kPi : 0);

    if (FloatEqual(angle1, angle2)) {
      return false;
    } else {
      return angle1 < angle2;
    }
  });

  /* Construct a triangular division */
  for (decltype(pts_idx.size()) j = 1; j < pts_idx.size() - 1; j++) {
    faces.emplace_back(pts_idx[0], pts_idx[j], pts_idx[j + 1]);
  }
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
Pose3<T>::Pose3(const T* data) : val_(data) {}

template <class T>
Pose3<T>::Pose3(T lon, T lat, T roll) : val_(lon, lat, roll) {}

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

template class Pose3<float>;


TriangleIdx::TriangleIdx(ShortIdType id1, ShortIdType id2, ShortIdType id3) : idx_{ id1, id2, id3 } {}


const ShortIdType* TriangleIdx::idx() const {
  return idx_;
}


PolygonIdx::PolygonIdx() : idx_() {}


PolygonIdx::PolygonIdx(std::initializer_list<ShortIdType> idx) : idx_(idx) {}


PolygonIdx::PolygonIdx(std::vector<ShortIdType> idx) : idx_(std::move(idx)) {}


const std::vector<ShortIdType>& PolygonIdx::idx() const {
  return idx_;
}


HalfSpaceSet::HalfSpaceSet(int n, float* a, float* b, float* c, float* d) : n(n), a(a), b(b), c(c), d(d) {}


RandomNumberGenerator::RandomNumberGenerator(uint32_t seed)
    : seed_(seed), generator_{ static_cast<std::mt19937::result_type>(seed) } {}


RandomNumberGenerator* RandomNumberGenerator::GetInstance() {
#ifdef RANDOM_SEED
  static std::unique_ptr<RandomNumberGenerator> instance_{ new RandomNumberGenerator(
      static_cast<uint32_t>(std::chrono::system_clock::now().time_since_epoch().count())) };
#else
  static std::unique_ptr<RandomNumberGenerator> instance_{ new RandomNumberGenerator(kDefaultRandomSeed) };
#endif
  return instance_.get();
}


float RandomNumberGenerator::GetGaussian() {
  return gauss_dist_(generator_);
}


float RandomNumberGenerator::GetUniform() {
  return uniform_dist_(generator_);
}


float RandomNumberGenerator::Get(Distribution dist, float mean, float std) {
  switch (dist) {
    case Distribution::kUniform:
      return (GetUniform() - 0.5f) * 2 * std + mean;
    case Distribution::kGaussian:
      return GetGaussian() * std + mean;
  }
}


void RandomNumberGenerator::Reset() {
  generator_.seed(seed_);
}


void RandomSampler::SampleSphericalPointsCart(const float* dir, float std, float* data, size_t num) {
  auto* rng = RandomNumberGenerator::GetInstance();

  float lon = std::atan2(dir[1], dir[0]);
  float lat = std::asin(dir[2] / Norm3(dir));
  float rot[3] = { lon, lat, 0 };

  std::unique_ptr<float[]> tmp_dir{ new float[num * 3] };

  double dz = 2 * std::sin(std / 2.0 * math::kDegreeToRad) * std::sin(std / 2.0 * math::kDegreeToRad);
  for (decltype(num) i = 0; i < num; i++) {
    double udz = rng->GetUniform() * dz;
    double q = rng->GetUniform() * 2 * math::kPi;

    double r = std::sqrt((2.0f - udz) * udz);
    tmp_dir[i * 3 + 0] = static_cast<float>(std::cos(q) * r);
    tmp_dir[i * 3 + 1] = static_cast<float>(std::sin(q) * r);
    tmp_dir[i * 3 + 2] = static_cast<float>(1.0 - udz);
  }
  RotateZBack(rot, tmp_dir.get(), data, num);
}


void RandomSampler::SampleSphericalPointsSph(float* data, size_t num, size_t step) {
  auto* rng = RandomNumberGenerator::GetInstance();
  for (decltype(num) i = 0; i < num; i++) {
    float u = rng->GetUniform() * 2 - 1;
    float lambda = rng->GetUniform() * 2 * math::kPi;

    data[i * step + 0] = lambda;
    data[i * step + 1] = std::asin(u);
  }
}


void RandomSampler::SampleSphericalPointsSph(const AxisDistribution& axis_dist, float* data, size_t num) {
  auto* rng = RandomNumberGenerator::GetInstance();
  for (decltype(num) i = 0; i < num; i++) {
    float phi = rng->Get(axis_dist.latitude_dist,                       // distribute
                         axis_dist.latitude_mean * math::kDegreeToRad,  // mean
                         axis_dist.latitude_std * math::kDegreeToRad);  // standard deviation
    if (phi > math::kPi / 2) {
      phi = math::kPi - phi;
    }
    if (phi < -math::kPi / 2) {
      phi = -math::kPi - phi;
    }
    float lambda = 0;
    if (axis_dist.azimuth_dist == Distribution::kUniform) {
      lambda = rng->GetUniform() * 2 * math::kPi;
    } else {
      lambda = rng->Get(axis_dist.azimuth_dist,                       // distribution
                        axis_dist.azimuth_mean * math::kDegreeToRad,  // mean
                        axis_dist.azimuth_std * math::kDegreeToRad);  // standard deviation
    }

    data[i * 2 + 0] = lambda;
    data[i * 2 + 1] = phi;
  }
}


void RandomSampler::SampleTriangularPoints(const float* vertexes, float* data, size_t num) {
  auto* rng = RandomNumberGenerator::GetInstance();
  for (decltype(num) i = 0; i < num; i++) {
    float a = rng->GetUniform();
    float b = rng->GetUniform();

    if (a + b > 1.0f) {
      a = 1.0f - a;
      b = 1.0f - b;
    }

    for (int j = 0; j < 3; j++) {
      data[i * 3 + j] = (vertexes[j + 3] - vertexes[j]) * a + (vertexes[j + 6] - vertexes[j]) * b + vertexes[j];
    }
  }
}


int RandomSampler::SampleInt(const float* p, int max) {
  auto* rng = RandomNumberGenerator::GetInstance();

  float current_cum_p = 0;
  float current_p = rng->GetUniform();

  for (decltype(max) i = 0; i < max; i++) {
    current_cum_p += p[i];
    if (current_p < current_cum_p) {
      return i;
    }
  }

  return max - 1;
}


int RandomSampler::SampleInt(int max) {
  auto* rng = RandomNumberGenerator::GetInstance();
  return std::min(static_cast<int>(rng->GetUniform() * max), max - 1);
}


AxisDistribution::AxisDistribution()
    : latitude_dist(Distribution::kUniform), azimuth_dist(Distribution::kUniform), roll_dist(Distribution::kUniform),
      latitude_mean(0), azimuth_mean(0), roll_mean(0), latitude_std(0), azimuth_std(0), roll_std(0) {}

}  // namespace icehalo
