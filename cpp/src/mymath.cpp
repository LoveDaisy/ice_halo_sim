#include "mymath.h"

#include <cstring>
#include <algorithm>


namespace IceHalo {

namespace Math {

bool floatEqual(float a, float b, float threshold) {
  return std::abs(a - b) < threshold;
}


float dot3(const float* vec1, const float* vec2) {
  return vec1[0] * vec2[0] + vec1[1] * vec2[1] + vec1[2] * vec2[2];
}


void cross3(const float* vec1, const float* vec2, float* vec) {
  vec[0] = -vec2[1] * vec1[2] + vec1[1] * vec2[2];
  vec[1] = vec2[0] * vec1[2] - vec1[0] * vec2[2];
  vec[2] = -vec2[0] * vec1[1] + vec1[0] * vec2[1];
}


float norm3(const float* vec) {
  return std::sqrt(dot3(vec, vec));
}


float diffNorm3(const float* vec1, const float* vec2) {
  float v[3];
  vec3FromTo(vec1, vec2, v);
  return norm3(v);
}


void normalize3(float* vec) {
  float len = norm3(vec);
  vec[0] /= len;
  vec[1] /= len;
  vec[2] /= len;
}


void normalized3(const float* vec, float* vec_n) {
  float len = norm3(vec);
  vec_n[0] = vec[0] / len;
  vec_n[1] = vec[1] / len;
  vec_n[2] = vec[2] / len;
}


void vec3FromTo(const float* vec1, const float* vec2, float* vec) {
  vec[0] = vec2[0] - vec1[0];
  vec[1] = vec2[1] - vec1[1];
  vec[2] = vec2[2] - vec1[2];
}


void rotateZ(const float* lon_lat_roll, const float* input_vec, float* output_vec, uint64_t dataNum) {
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
  matMultiply(inputVec, matRt, &resVec);
}


void rotateZBack(const float* lon_lat_roll, const float* input_vec, float* output_vec, uint64_t dataNum) {
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
  matMultiply(inputVec, matR, &resVec);
}


std::vector<Vec3f> findInnerPoints(const HalfSpaceSet& hss) {
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


void sortAndRemoveDuplicate(std::vector<Vec3f>* pts) {
  /* Sort by coordinates */
  std::sort(pts->begin(), pts->end(),
    [](const Vec3f& p1, const Vec3f& p2){
      if (p1 == p2) {
        return false;
      }
      if (p1.x() < p2.x() - kFloatEps) {
        return true;
      }
      if (floatEqual(p1.x(), p2.x()) && p1.y() < p2.y() - kFloatEps) {
        return true;
      }
      if (floatEqual(p1.x(), p2.x()) && floatEqual(p1.y(), p2.y()) && p1.z() < p2.z() - kFloatEps) {
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


std::vector<int> findCoplanarPoints(const std::vector<Vec3f>& pts, const Vec3f& n0, float d0) {
  std::vector<int> ptsIdx;
  for (decltype(pts.size()) j = 0; j < pts.size(); j++) {
    const auto& p = pts[j];
    if (floatEqual(Vec3f::dot(n0, p) + d0, 0)) {
      ptsIdx.push_back(static_cast<int>(j));
    }
  }
  return ptsIdx;
}


void buildPolyhedronFaces(const HalfSpaceSet& hss, const std::vector<Math::Vec3f>& pts,
                          std::vector<Math::TriangleIdx>& faces) {
  int num = hss.n;
  float* a = hss.a, *b = hss.b, *c = hss.c, *d = hss.d;

  for (int i = 0; i < num; i++) {
    /* Find co-planer points */
    Vec3f n0(a[i], b[i], c[i]);
    std::vector<int> facePtsIdx = findCoplanarPoints(pts, n0, d[i]);
    if (facePtsIdx.empty()) {
      continue;
    }

    /* Build triangular division */
    buildTriangularDivision(pts, n0, facePtsIdx, faces);
  }
}


void buildTriangularDivision(const std::vector<Vec3f>& vertex, const Vec3f& n,
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
      Vec3f p0 = Vec3f::fromVec(center, vertex[idx0]).normalized();
      Vec3f p1 = Vec3f::fromVec(center, vertex[idx1]).normalized();
      Vec3f p2 = Vec3f::fromVec(center, vertex[idx2]).normalized();

      Vec3f n1 = Vec3f::cross(p0, p1);
      float dir = Vec3f::dot(n1, n);
      float c1 = Vec3f::dot(p0, p1);
      float s1 = std::abs(dir) > kFloatEps ? Vec3f::norm(n1) * (dir / std::abs(dir)) : 0;
      float angle1 = atan2(s1, c1);
      angle1 += (angle1 < 0 ? 2 * kPi : 0);

      Vec3f n2 = Vec3f::cross(p0, p2);
      dir = Vec3f::dot(n2, n);
      float c2 = Vec3f::dot(p0, p2);
      float s2 = std::abs(dir) > kFloatEps ? Vec3f::norm(n2) * (dir / std::abs(dir)) : 0;
      float angle2 = atan2(s2, c2);
      angle2 += (angle2 < 0 ? 2 * kPi : 0);

      if (floatEqual(angle1, angle2)) {
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
    : rowNum(row), colNum(col), data(data) {}

void DummyMatrix::transpose() {
  for (uint64_t r = 0; r < rowNum; r++) {
    for (uint64_t c = r+1; c < colNum; c++) {
      float tmp;
      tmp = data[r * colNum + c];
      data[r * colNum + c] = data[c * colNum +r];
      data[c * colNum +r] = tmp;
    }
  }
}


ConstDummyMatrix::ConstDummyMatrix(const float *data, uint64_t row, uint64_t col)
    : DummyMatrix(nullptr, row, col), data(data) {}


int matMultiply(ConstDummyMatrix& a, ConstDummyMatrix& b, DummyMatrix* res) {
  if (a.colNum != b.rowNum) {
    return -1;
  }

  for (uint64_t r = 0; r < a.rowNum; r++) {
    for (uint64_t c = 0; c < b.colNum; c++) {
      float sum = 0.0f;
      for (uint64_t k = 0; k < a.colNum; k++) {
        sum += a.data[r * a.colNum + k] * b.data[k * b.colNum + c];
      }
      res->data[r * res->colNum + c] = sum;
    }
  }
  return 0;
}


template <typename T>
Vec3<T>::Vec3(T x, T y, T z) {
  _val[0] = x;
  _val[1] = y;
  _val[2] = z;
}


template <typename T>
Vec3<T>::Vec3(const T* data) {
  _val[0] = data[0];
  _val[1] = data[1];
  _val[2] = data[2];
}


template <typename T>
Vec3<T>::Vec3(const Vec3<T>& v) {
  _val[0] = v._val[0];
  _val[1] = v._val[1];
  _val[2] = v._val[2];
}


template <typename T>
const T* Vec3<T>::val() const {
  return _val;
}


template <typename T>
void Vec3<T>::val(T x, T y, T z) {
  _val[0] = x;
  _val[1] = y;
  _val[2] = z;
}


template <typename T>
void Vec3<T>::val(const T* data) {
  _val[0] = data[0];
  _val[1] = data[1];
  _val[2] = data[2];
}


template <typename T>
T Vec3<T>::x() const {
  return _val[0];
}


template <typename T>
T Vec3<T>::y() const {
  return _val[1];
}


template <typename T>
T Vec3<T>::z() const {
  return _val[2];
}


template <typename T>
void Vec3<T>::x(T x) {
  _val[0] = x;
}


template <typename T>
void Vec3<T>::y(T y) {
  _val[1] = y;
}


template <typename T>
void Vec3<T>::z(T z) {
  _val[2] = z;
}


template <typename T>
Vec3<T> Vec3<T>::normalized() {
  return Vec3<T>::normalized(*this);
}


template <typename T>
void Vec3<T>::normalize() {
  Math::normalize3(_val);
}


bool operator==(const Vec3f& lhs, const Vec3f& rhs) {
  for (int i = 0; i < 3; i++) {
    if (!floatEqual(lhs.val()[i], rhs.val()[i])) {
      return false;
    }
  }
  return true;
}


template <typename T>
Vec3<T> Vec3<T>::normalized(const Vec3<T>& v) {
  T data[3];
  Math::normalized3(v._val, data);
  return Vec3<T>(data);
}


template <typename T>
Vec3<T>& Vec3<T>::operator+=(const Vec3<T>& v) {
  for (int i = 0; i < 3; i++) {
    _val[i] += v._val[i];
  }
  return *this;
}


template <typename T>
Vec3<T>& Vec3<T>::operator+= (T a) {
  for (auto& i : _val) {
    i += a;
  }
  return *this;
}


template <typename T>
Vec3<T>& Vec3<T>::operator-=(const Vec3<T>& v) {
  for (int i = 0; i < 3; i++) {
    _val[i] -= v._val[i];
  }
  return *this;
}


template <typename T>
Vec3<T>& Vec3<T>::operator-= (T a) {
  for (auto& i : _val) {
    i -= a;
  }
  return *this;
}


template <typename T>
Vec3<T>& Vec3<T>::operator/= (T a) {
  for (auto& i : _val) {
    i /= a;
  }
  return *this;
}


template <typename T>
Vec3<T>& Vec3<T>::operator*= (T a) {
  for (auto& i : _val) {
    i *= a;
  }
  return *this;
}


template <typename T>
T Vec3<T>::dot(const Vec3<T>& v1, const Vec3<T>& v2) {
  return Math::dot3(v1._val, v2._val);
}


template <typename T>
T Vec3<T>::norm(const Vec3<T>& v) {
  return Math::norm3(v._val);
}


template <typename T>
Vec3<T> Vec3<T>::cross(const Vec3<T>& v1, const Vec3<T>& v2) {
  T data[3];
  Math::cross3(v1._val, v2._val, data);
  return Vec3<T>(data);
}


template <typename T>
Vec3<T> Vec3<T>::fromVec(const Vec3<T>& v1, const Vec3<T>& v2) {
  T data[3];
  Math::vec3FromTo(v1._val, v2._val, data);
  return Vec3<T>(data);
}

template class Vec3<float>;


TriangleIdx::TriangleIdx(int id1, int id2, int id3) {
  _idx[0] = id1;
  _idx[1] = id2;
  _idx[2] = id3;
}


const int* TriangleIdx::idx() const {
  return &_idx[0];
}


HalfSpaceSet::HalfSpaceSet(int n, float* a, float* b, float* c, float* d)
    : n(n), a(a), b(b), c(c), d(d) {}


RandomNumberGenerator::RandomNumberGenerator() : generator{random_number_seed_} {}


RandomNumberGenerator& RandomNumberGenerator::GetInstance() {
  static RandomNumberGenerator rng;
  return rng;
}


float RandomNumberGenerator::getGaussian() {
  return gauss_dist_(generator);
}


float RandomNumberGenerator::getUniform() {
  return uniform_dist_(generator);
}


float RandomNumberGenerator::get(Distribution dist, float mean, float std) {
  switch (dist) {
    case Distribution::UNIFORM :
      return (getUniform() - 0.5f) * 2 * std + mean;
    case Distribution::GAUSS :
      return getGaussian() * std + mean;
  }
}


RandomSampler& RandomSampler::GetInstance() {
  static RandomSampler r;
  return r;
}


void RandomSampler::SampleSphericalPointsCart(float* data, size_t num) {
  auto& rng = RandomNumberGenerator::GetInstance();
  for (decltype(num) i = 0; i < num; i++) {
    float u = rng.getUniform();
    float q = rng.getUniform() * 2 * Math::kPi;

    float r = std::sqrt(1.0f - u * u);

    data[i * 3 + 0] = r * std::cos(q);
    data[i * 3 + 1] = r * std::sin(q);
    data[i * 3 + 2] = u;
  }
}


void RandomSampler::SampleSphericalPointsCart(Distribution dist, float lat, float std,
                                              float* data, size_t num) {
  auto& rng = RandomNumberGenerator::GetInstance();
  for (decltype(num) i = 0; i < num; i++) {
    float u = rng.get(dist, lat, std);
    float q = rng.getUniform() * 2 * Math::kPi;

    float r = std::cos(u);
    data[i * 3 + 0] = r * std::cos(q);
    data[i * 3 + 1] = r * std::sin(q);
    data[i * 3 + 2] = std::sin(u);
  }
}


void RandomSampler::SampleSphericalPointsCart(const float* dir, float std, float* data, size_t num) {
  auto& rng = RandomNumberGenerator::GetInstance();

  float lon = std::atan2(dir[1], dir[0]);
  float lat = std::asin(dir[2] / Math::norm3(dir));
  float rot[3] = { lon, lat, 0 };

  auto* tmp_dir = new float[num * 3];

  float dz = 1.0f - std::cos(std / 180.0f * Math::kPi);
  for (decltype(num) i = 0; i < num; i++) {
    float z = 1.0f - rng.getUniform() * dz;
    float r = std::sqrt(1.0f - z * z);
    float q = rng.getUniform() * 2 * Math::kPi;
    float x = std::cos(q) * r;
    float y = std::sin(q) * r;

    tmp_dir[i * 3 + 0] = x;
    tmp_dir[i * 3 + 1] = y;
    tmp_dir[i * 3 + 2] = z;
  }
  Math::rotateZBack(rot, tmp_dir, data, num);

  delete[] tmp_dir;
}


void RandomSampler::SampleSphericalPointsSph(Distribution dist, float lat, float std,
                                             float* data, size_t num) {
  auto& rng = RandomNumberGenerator::GetInstance();
  for (decltype(num) i = 0; i < num; i++) {
    float u = rng.get(dist, lat * kDegreeToRad, std * kDegreeToRad);
    float q = rng.getUniform() * 2 * Math::kPi;

    data[i * 2 + 0] = q;
    data[i * 2 + 1] = u;
  }
}


void RandomSampler::SampleTriangularPoints(const float* vertexes, float* data, size_t num) {
  auto& rng = RandomNumberGenerator::GetInstance();
  for (decltype(num) i = 0; i < num; i++) {
    float a = rng.getUniform();
    float b = rng.getUniform();

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
  auto& rng = RandomNumberGenerator::GetInstance();

  float current_cum_p = 0;
  float current_p = rng.getUniform();

  for (decltype(max) i = 0; i < max; i++) {
    current_cum_p += p[i];
    if (current_p < current_cum_p) {
      return i;
    }
  }

  return max - 1;
}


int RandomSampler::SampleInt(int max) {
  auto& rng = RandomNumberGenerator::GetInstance();
  return std::min(static_cast<int>(rng.getUniform() * max), max - 1);
}

}  // namespace Math

}   // namespace IceHalo
