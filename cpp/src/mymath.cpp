#include "mymath.h"

#include <cstring>
#include <algorithm>
#include <chrono>


namespace IceHalo {

namespace Math {

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


void RotateZ(const float* lon_lat_roll, const float* input_vec, float* output_vec, uint64_t dataNum) {
  using std::cos;
  using std::sin;
  float ax[9] = {-cos(lon_lat_roll[2]) * sin(lon_lat_roll[0]) - cos(lon_lat_roll[0]) * sin(lon_lat_roll[1]) * sin(lon_lat_roll[2]),
                 -cos(lon_lat_roll[0]) * cos(lon_lat_roll[2]) * sin(lon_lat_roll[1]) + sin(lon_lat_roll[0]) * sin(lon_lat_roll[2]),
                 cos(lon_lat_roll[0]) * cos(lon_lat_roll[1]),
                 cos(lon_lat_roll[0]) * cos(lon_lat_roll[2]) - sin(lon_lat_roll[0]) * sin(lon_lat_roll[1]) * sin(lon_lat_roll[2]),
                 -cos(lon_lat_roll[2]) * sin(lon_lat_roll[0]) * sin(lon_lat_roll[1]) - cos(lon_lat_roll[0]) * sin(lon_lat_roll[2]),
                 cos(lon_lat_roll[1]) * sin(lon_lat_roll[0]),
                 cos(lon_lat_roll[1]) * sin(lon_lat_roll[2]),
                 cos(lon_lat_roll[1]) * cos(lon_lat_roll[2]),
                 sin(lon_lat_roll[1])};

  ConstDummyMatrix matRt(ax, 3, 3);
  ConstDummyMatrix inputVec(input_vec, dataNum, 3);
  DummyMatrix resVec(output_vec, dataNum, 3);
  MatrixMultiply(inputVec, matRt, &resVec);
}


void RotateZBack(const float* lon_lat_roll, const float* input_vec, float* output_vec,
                 uint64_t dataNum) {
  using std::cos;
  using std::sin;
  float ax[9] = {-cos(lon_lat_roll[2]) * sin(lon_lat_roll[0]) - cos(lon_lat_roll[0]) * sin(lon_lat_roll[1]) * sin(lon_lat_roll[2]),
                  cos(lon_lat_roll[0]) * cos(lon_lat_roll[2]) - sin(lon_lat_roll[0]) * sin(lon_lat_roll[1]) * sin(lon_lat_roll[2]),
                  cos(lon_lat_roll[1]) * sin(lon_lat_roll[2]),
                  -cos(lon_lat_roll[0]) * cos(lon_lat_roll[2]) * sin(lon_lat_roll[1]) + sin(lon_lat_roll[0]) * sin(lon_lat_roll[2]),
                  -cos(lon_lat_roll[2]) * sin(lon_lat_roll[0]) * sin(lon_lat_roll[1]) - cos(lon_lat_roll[0]) * sin(lon_lat_roll[2]),
                  cos(lon_lat_roll[1]) * cos(lon_lat_roll[2]),
                  cos(lon_lat_roll[0]) * cos(lon_lat_roll[1]),
                  cos(lon_lat_roll[1]) * sin(lon_lat_roll[0]),
                  sin(lon_lat_roll[1])};

  ConstDummyMatrix matR(ax, 3, 3);
  ConstDummyMatrix inputVec(input_vec, dataNum, 3);
  DummyMatrix resVec(output_vec, dataNum, 3);
  MatrixMultiply(inputVec, matR, &resVec);
}


std::vector<Vec3f> FindInnerPoints(const HalfSpaceSet& hss) {
  float* a = hss.a, *b = hss.b, *c = hss.c, *d = hss.d;
  int n = hss.n;

  std::vector<Vec3f> pts;
  for (int i = 0; i < n; i++) {
    for (int j = i+1; j < n; j++) {
      for (int k = j+1; k < n; k++) {
        float det = a[k] * b[j] * c[i] - a[j] * b[k] * c[i] - a[k] * b[i] * c[j] +
                    a[i] * b[k] * c[j] + a[j] * b[i] * c[k] - a[i] * b[j] * c[k];
        if (std::abs(det) <= kFloatEps) {
          continue;
        }
        float x = -(b[k] * c[j] * d[i] - b[j] * c[k] * d[i] - b[k] * c[i] * d[j] +
                    b[i] * c[k] * d[j] + b[j] * c[i] * d[k] - b[i] * c[j] * d[k]) / det;
        float y = -(-(a[k] * c[j] * d[i]) + a[j] * c[k] * d[i] + a[k] * c[i] * d[j] -
                    a[i] * c[k] * d[j] - a[j] * c[i] * d[k] + a[i] * c[j] * d[k]) / det;
        float z = -(a[k] * b[j] * d[i] - a[j] * b[k] * d[i] - a[k] * b[i] * d[j] +
                    a[i] * b[k] * d[j] + a[j] * b[i] * d[k] - a[i] * b[j] * d[k]) / det;

        bool in = true;
        for (int ii = 0; ii < n; ii++) {
          in = in && (a[ii] * x + b[ii] * y + c[ii] * z + d[ii] <= kFloatEps);
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
  std::sort(pts->begin(), pts->end(),
    [](const Vec3f& p1, const Vec3f& p2){
      if (p1 == p2) {
        return false;
      }
      if (p1.x() < p2.x() - kFloatEps) {
        return true;
      }
      if (FloatEqual(p1.x(), p2.x()) && p1.y() < p2.y() - kFloatEps) {
        return true;
      }
      if (FloatEqual(p1.x(), p2.x()) && FloatEqual(p1.y(), p2.y()) && p1.z() < p2.z() - kFloatEps) {
        return true;
      }
      return false;
    });

  /* Remove duplicated points */
  for (auto iter = pts->begin(), lastIter = pts->begin(); iter != pts->end(); ) {
    if (iter != lastIter && (*iter) == (*lastIter)) {
      iter = pts->erase(iter);
    } else {
      lastIter = iter;
      iter++;
    }
  }
}


std::vector<int> FindCoplanarPoints(const std::vector<Vec3f>& pts, const Vec3f& n0, float d0) {
  std::vector<int> ptsIdx;
  for (decltype(pts.size()) j = 0; j < pts.size(); j++) {
    const auto& p = pts[j];
    if (FloatEqual(Vec3f::Dot(n0, p) + d0, 0)) {
      ptsIdx.push_back(static_cast<int>(j));
    }
  }
  return ptsIdx;
}


void BuildPolyhedronFaces(const HalfSpaceSet& hss, const std::vector<Math::Vec3f>& pts,
                          std::vector<Math::TriangleIdx>& faces) {
  int num = hss.n;
  float* a = hss.a, *b = hss.b, *c = hss.c, *d = hss.d;

  for (int i = 0; i < num; i++) {
    /* Find co-planer points */
    Vec3f n0(a[i], b[i], c[i]);
    std::vector<int> facePtsIdx = FindCoplanarPoints(pts, n0, d[i]);
    if (facePtsIdx.empty()) {
      continue;
    }

    /* Build triangular division */
    BuildTriangularDivision(pts, n0, facePtsIdx, faces);
  }
}


void BuildTriangularDivision(const std::vector<Vec3f>& vertex, const Vec3f& n,
                             std::vector<int>& ptsIdx, std::vector<TriangleIdx>& faces) {
  /* Find the center of co-planer points */
  Vec3f center(0.0f, 0.0f, 0.0f);
  for (auto p : ptsIdx) {
    center += vertex[p];
  }
  center /= ptsIdx.size();

  /* Sort by angle */
  int idx0 = ptsIdx[0];
  std::sort(ptsIdx.begin() + 1, ptsIdx.end(),
    [&n, &vertex, &center, idx0](const int idx1, const int idx2){
      Vec3f p0 = Vec3f::FromTo(center, vertex[idx0]).Normalized();
      Vec3f p1 = Vec3f::FromTo(center, vertex[idx1]).Normalized();
      Vec3f p2 = Vec3f::FromTo(center, vertex[idx2]).Normalized();

      Vec3f n1 = Vec3f::Cross(p0, p1);
      float dir = Vec3f::Dot(n1, n);
      float c1 = Vec3f::Dot(p0, p1);
      float s1 = std::abs(dir) > kFloatEps ? Vec3f::Norm(n1) * (dir / std::abs(dir)) : 0;
      float angle1 = atan2(s1, c1);
      angle1 += (angle1 < 0 ? 2 * kPi : 0);

      Vec3f n2 = Vec3f::Cross(p0, p2);
      dir = Vec3f::Dot(n2, n);
      float c2 = Vec3f::Dot(p0, p2);
      float s2 = std::abs(dir) > kFloatEps ? Vec3f::Norm(n2) * (dir / std::abs(dir)) : 0;
      float angle2 = atan2(s2, c2);
      angle2 += (angle2 < 0 ? 2 * kPi : 0);

      if (FloatEqual(angle1, angle2)) {
        return false;
      } else {
        return angle1 < angle2;
      }
    });

  /* Construct a triangular division */
  for (decltype(ptsIdx.size()) j = 1; j < ptsIdx.size() - 1; j++) {
    faces.emplace_back(ptsIdx[0], ptsIdx[j], ptsIdx[j+1]);
  }
}


DummyMatrix::DummyMatrix(float* data, uint64_t row, uint64_t col)
    : rows_(row), cols_(col), data_(data) {}


ConstDummyMatrix::ConstDummyMatrix(const float *data, uint64_t row, uint64_t col)
    : DummyMatrix(nullptr, row, col), data(data) {}


int MatrixMultiply(ConstDummyMatrix& a, ConstDummyMatrix& b, DummyMatrix* res) {
  if (a.cols_ != b.rows_) {
    return -1;
  }

  for (uint64_t r = 0; r < a.rows_; r++) {
    for (uint64_t c = 0; c < b.cols_; c++) {
      float sum = 0.0f;
      for (uint64_t k = 0; k < a.cols_; k++) {
        sum += a.data[r * a.cols_ + k] * b.data[k * b.cols_ + c];
      }
      res->data_[r * res->cols_ + c] = sum;
    }
  }
  return 0;
}


template <typename T>
Vec3<T>::Vec3(T x, T y, T z) {
  val_[0] = x;
  val_[1] = y;
  val_[2] = z;
}


template <typename T>
Vec3<T>::Vec3(const T* data) {
  val_[0] = data[0];
  val_[1] = data[1];
  val_[2] = data[2];
}


template <typename T>
Vec3<T>::Vec3(const Vec3<T>& v) {
  val_[0] = v.val_[0];
  val_[1] = v.val_[1];
  val_[2] = v.val_[2];
}


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
  Math::Normalize3(val_);
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
  Math::Normalized3(v.val_, data);
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
Vec3<T>& Vec3<T>::operator+= (T a) {
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
Vec3<T>& Vec3<T>::operator-= (T a) {
  for (auto& i : val_) {
    i -= a;
  }
  return *this;
}


template <typename T>
Vec3<T>& Vec3<T>::operator/= (T a) {
  for (auto& i : val_) {
    i /= a;
  }
  return *this;
}


template <typename T>
Vec3<T>& Vec3<T>::operator*= (T a) {
  for (auto& i : val_) {
    i *= a;
  }
  return *this;
}


template <typename T>
T Vec3<T>::Dot(const Vec3<T>& v1, const Vec3<T>& v2) {
  return Math::Dot3(v1.val_, v2.val_);
}


template <typename T>
T Vec3<T>::Norm(const Vec3<T>& v) {
  return Math::Norm3(v.val_);
}


template <typename T>
Vec3<T> Vec3<T>::Cross(const Vec3<T>& v1, const Vec3<T>& v2) {
  T data[3];
  Math::Cross3(v1.val_, v2.val_, data);
  return Vec3<T>(data);
}


template <typename T>
Vec3<T> Vec3<T>::FromTo(const Vec3<T>& v1, const Vec3<T>& v2) {
  T data[3];
  Math::Vec3FromTo(v1.val_, v2.val_, data);
  return Vec3<T>(data);
}

template class Vec3<float>;


TriangleIdx::TriangleIdx(int id1, int id2, int id3) {
  idx_[0] = id1;
  idx_[1] = id2;
  idx_[2] = id3;
}


const int* TriangleIdx::idx() const {
  return &idx_[0];
}


HalfSpaceSet::HalfSpaceSet(int n, float* a, float* b, float* c, float* d)
    : n(n), a(a), b(b), c(c), d(d) {}


RandomNumberGenerator::RandomNumberGenerator(uint32_t seed)
    : generator_{static_cast<std::mt19937::result_type>(seed)} {}


RandomNumberGeneratorPtr RandomNumberGenerator::instance_ = nullptr;
std::mutex RandomNumberGenerator::instance_mutex_{};


RandomNumberGeneratorPtr RandomNumberGenerator::GetInstance() {
  if (!instance_) {
    std::unique_lock<std::mutex> lock(instance_mutex_);
    if (!instance_) {
#ifdef RANDOM_SEED
      auto seed = std::chrono::system_clock::now().time_since_epoch().count();
      instance_ = std::shared_ptr<RandomNumberGenerator>(new RandomNumberGenerator(static_cast<uint32_t>(seed)));
#else
      instance_ = std::shared_ptr<RandomNumberGenerator>(new RandomNumberGenerator(kDefaultRandomSeed));
#endif
    }
  }
  return instance_;
}


float RandomNumberGenerator::GetGaussian() {
  return gauss_dist_(generator_);
}


float RandomNumberGenerator::GetUniform() {
  return uniform_dist_(generator_);
}


float RandomNumberGenerator::Get(Distribution dist, float mean, float std) {
  switch (dist) {
    case Distribution::UNIFORM :
      return (GetUniform() - 0.5f) * 2 * std + mean;
    case Distribution::GAUSS :
      return GetGaussian() * std + mean;
  }
}


RandomSamplerPtr RandomSampler::GetInstance() {
  if (!instance_) {
    std::unique_lock<std::mutex> lock(instance_mutex_);
    if (!instance_) {
      instance_ = std::shared_ptr<RandomSampler>(new RandomSampler());
    }
  }
  return instance_;
}

RandomSamplerPtr RandomSampler::instance_ = nullptr;
std::mutex RandomSampler::instance_mutex_{};


void RandomSampler::SampleSphericalPointsCart(float* data, size_t num) {
  auto rng = RandomNumberGenerator::GetInstance();
  for (decltype(num) i = 0; i < num; i++) {
    float u = rng->GetUniform() * 2 - 1;
    float q = rng->GetUniform() * 2 * Math::kPi;

    float r = std::sqrt(1.0f - u * u);

    data[i * 3 + 0] = r * std::cos(q);
    data[i * 3 + 1] = r * std::sin(q);
    data[i * 3 + 2] = u;
  }
}


void RandomSampler::SampleSphericalPointsCart(Distribution dist, float lat, float std,
                                              float* data, size_t num) {
  auto rng = RandomNumberGenerator::GetInstance();
  for (decltype(num) i = 0; i < num; i++) {
    float phi = rng->Get(dist, lat * kDegreeToRad, std * kDegreeToRad);
    if (phi > kPi / 2) {
      phi = kPi - phi;
    }
    if (phi < -kPi / 2) {
      phi = -kPi - phi;
    }
    float lambda = rng->GetUniform() * 2 * Math::kPi;

    float r = std::cos(phi);
    data[i * 3 + 0] = r * std::cos(lambda);
    data[i * 3 + 1] = r * std::sin(lambda);
    data[i * 3 + 2] = std::sin(phi);
  }
}


void RandomSampler::SampleSphericalPointsCart(const float* dir, float std, float* data, size_t num) {
  auto rng = RandomNumberGenerator::GetInstance();

  float lon = std::atan2(dir[1], dir[0]);
  float lat = std::asin(dir[2] / Math::Norm3(dir));
  float rot[3] = { lon, lat, 0 };

  auto* tmp_dir = new float[num * 3];

  double dz = 2 * std::sin(std / 2.0 * kDegreeToRad) * std::sin(std / 2.0 * kDegreeToRad);
  for (decltype(num) i = 0; i < num; i++) {
    double udz = rng->GetUniform() * dz;
    double q = rng->GetUniform() * 2 * Math::kPi;

    double r = std::sqrt((2.0f - udz) * udz);
    tmp_dir[i * 3 + 0] = static_cast<float>(std::cos(q) * r);
    tmp_dir[i * 3 + 1] = static_cast<float>(std::sin(q) * r);
    tmp_dir[i * 3 + 2] = static_cast<float>(1.0f - udz);
  }
  Math::RotateZBack(rot, tmp_dir, data, num);

  delete[] tmp_dir;
}


void RandomSampler::SampleSphericalPointsSph(float* data, size_t num) {
  auto rng = RandomNumberGenerator::GetInstance();
  for (decltype(num) i = 0; i < num; i++) {
    float u = rng->GetUniform() * 2 - 1;
    float lambda = rng->GetUniform() * 2 * Math::kPi;

    data[i * 3 + 0] = lambda;
    data[i * 3 + 1] = std::asin(u);
  }
}


void RandomSampler::SampleSphericalPointsSph(Distribution dist, float lat, float std,
                                             float* data, size_t num) {
  auto rng = RandomNumberGenerator::GetInstance();
  for (decltype(num) i = 0; i < num; i++) {
    float phi = rng->Get(dist, lat * kDegreeToRad, std * kDegreeToRad);
    if (phi > kPi / 2) {
      phi = kPi - phi;
    }
    if (phi < -kPi / 2) {
      phi = -kPi - phi;
    }
    float lambda = rng->GetUniform() * 2 * Math::kPi;

    data[i * 2 + 0] = lambda;
    data[i * 2 + 1] = phi;
  }
}


void RandomSampler::SampleTriangularPoints(const float* vertexes, float* data, size_t num) {
  auto rng = RandomNumberGenerator::GetInstance();
  for (decltype(num) i = 0; i < num; i++) {
    float a = rng->GetUniform();
    float b = rng->GetUniform();

    if (a + b > 1.0f) {
      a = 1.0f - a;
      b = 1.0f - b;
    }

    for (int j = 0; j < 3; j++) {
      data[i * 3 + j] = (vertexes[j + 3] - vertexes[j]) * a +
                        (vertexes[j + 6] - vertexes[j]) * b + vertexes[j];
    }
  }
}


int RandomSampler::SampleInt(const float* p, int max) {
  auto rng = RandomNumberGenerator::GetInstance();

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
  auto rng = RandomNumberGenerator::GetInstance();
  return std::min(static_cast<int>(rng->GetUniform() * max), max - 1);
}

}  // namespace Math

}   // namespace IceHalo
