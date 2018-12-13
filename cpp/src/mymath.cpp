#include "mymath.h"

#include <cstring>
#include <algorithm>
#include <chrono>


namespace IceHalo {

namespace Math {

bool floatEqual(float a, float b, float threshold) {
  return std::abs(a - b) < threshold;
}


float dot3(const float* vec1, const float* vec2) {
  return vec1[0]*vec2[0] + vec1[1]*vec2[1] + vec1[2]*vec2[2];
}


void cross3(const float* vec1, const float* vec2, float* vec) {
  vec[0] = -vec2[1]*vec1[2] + vec1[1]*vec2[2];
  vec[1] = vec2[0]*vec1[2] - vec1[0]*vec2[2];
  vec[2] = -vec2[0]*vec1[1] + vec1[0]*vec2[1];
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


void rotateBase(const float* ax, float angle, float* vec) {
  float c = std::cos(angle);
  float s = std::sin(angle);

  float matR[9] = {c + ax[0] * ax[0] * (1-c), ax[1] * ax[0] * (1-c) + ax[2] * s, ax[2] * ax[0] * (1-c) - ax[1] * s,
    ax[0] * ax[1] * (1-c) - ax[2] * s, c + ax[1] * ax[1] * (1-c), ax[2] * ax[1] * (1-c) + ax[0] * s,
    ax[0] * ax[2] * (1-c) + ax[1] * s, ax[1] * ax[2] * (1-c) - ax[0] * s, c + ax[2] * ax[2] * (1-c)};
  float res[9];

  DummyMatrix v(vec, 3, 3);
  DummyMatrix R(matR, 3, 3);
  DummyMatrix vn(res, 3, 3);
  DummyMatrix::multiply(v, R, vn);

  std::memcpy(vec, res, 9*sizeof(float));
}


void rotateZ(const float* lon_lat_roll, float* vec, uint64_t dataNum) {
  using namespace std;
  float ax[9] = {-sin(lon_lat_roll[0]), cos(lon_lat_roll[0]), 0.0f,
           -cos(lon_lat_roll[0]) * sin(lon_lat_roll[1]), -sin(lon_lat_roll[0]) * sin(lon_lat_roll[1]), cos(lon_lat_roll[1]),
           cos(lon_lat_roll[1]) * cos(lon_lat_roll[0]), cos(lon_lat_roll[1]) * sin(lon_lat_roll[0]), sin(lon_lat_roll[1])};
  float d[3] = {cos(lon_lat_roll[1]) * cos(lon_lat_roll[0]), cos(lon_lat_roll[1]) * sin(lon_lat_roll[0]), sin(lon_lat_roll[1])};
  rotateBase(d, lon_lat_roll[2], ax);

  DummyMatrix matR(ax, 3, 3);
  matR.transpose();

  auto* res = new float[dataNum * 3];

  DummyMatrix resVec(res, dataNum, 3);
  DummyMatrix inputVec(vec, dataNum, 3);
  DummyMatrix::multiply(inputVec, matR, resVec);
  std::memcpy(vec, res, 3 * dataNum * sizeof(float));

  delete[] res;
}


void rotateZBack(const float* lon_lat_roll, float* vec, uint64_t dataNum) {
  using namespace std;
  float ax[9] = {-sin(lon_lat_roll[0]), cos(lon_lat_roll[0]), 0.0f,
           -cos(lon_lat_roll[0]) * sin(lon_lat_roll[1]), -sin(lon_lat_roll[0]) * sin(lon_lat_roll[1]), cos(lon_lat_roll[1]),
           cos(lon_lat_roll[1]) * cos(lon_lat_roll[0]), cos(lon_lat_roll[1]) * sin(lon_lat_roll[0]), sin(lon_lat_roll[1])};
  float d[3] = {cos(lon_lat_roll[1]) * cos(lon_lat_roll[0]), cos(lon_lat_roll[1]) * sin(lon_lat_roll[0]), sin(lon_lat_roll[1])};
  rotateBase(d, lon_lat_roll[2], ax);

  DummyMatrix matR(ax, 3, 3);

  // float res[3] = { 0.0f };
  auto* res = new float[dataNum * 3];

  DummyMatrix resVec(res, dataNum, 3);
  DummyMatrix inputVec(vec, dataNum, 3);
  DummyMatrix::multiply(inputVec, matR, resVec);
  std::memcpy(vec, res, 3 * dataNum * sizeof(float));

  delete[] res;
}


void findInnerPoints(HalfSpaceSet& hss, std::vector<Vec3f>& pts) {
  float* a = hss.a,* b = hss.b,* c = hss.c,* d = hss.d;
  int n = hss.n;

  for (int i = 0; i < n; i++) {
    for (int j = i+1; j < n; j++) {
      for (int k = j+1; k < n; k++) {
        float det = a[k]*b[j]*c[i] - a[j]*b[k]*c[i] - a[k]*b[i]*c[j] + a[i]*b[k]*c[j] + a[j]*b[i]*c[k] - a[i]*b[j]*c[k];
        if (std::abs(det) <= kFloatEps) {
          continue;
        }
        float x = -(b[k]*c[j]*d[i] - b[j]*c[k]*d[i] - b[k]*c[i]*d[j] + b[i]*c[k]*d[j] + b[j]*c[i]*d[k] - b[i]*c[j]*d[k]) / det;
        float y = -(-(a[k]*c[j]*d[i]) + a[j]*c[k]*d[i] + a[k]*c[i]*d[j] - a[i]*c[k]*d[j] - a[j]*c[i]*d[k] + a[i]*c[j]*d[k]) / det;
        float z = -(a[k]*b[j]*d[i] - a[j]*b[k]*d[i] - a[k]*b[i]*d[j] + a[i]*b[k]*d[j] + a[j]*b[i]*d[k] - a[i]*b[j]*d[k]) / det;

        bool in = true;
        for (int ii = 0; ii < n; ii++) {
          in = in && (a[ii]*x + b[ii]*y + c[ii]*z + d[ii] <= kFloatEps);
          if (!in) {
            break;
          }
        }
        if (in) {
          pts.emplace_back(Vec3f(x, y, z));
        }
      }
    }
  }
}


void sortAndRemoveDuplicate(std::vector<Vec3f>& pts) {
  /* Sort by coordinates */
  std::sort(pts.begin(), pts.end(),
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
    }
  );

  /* Remove duplicated points */
  for (auto iter = pts.begin(), lastIter = pts.begin(); iter != pts.end(); ) {
    if (iter != lastIter && (*iter) == (*lastIter)) {
      iter = pts.erase(iter);
    } else {
      lastIter = iter;
      iter++;
    }
  }
}


void findCoplanarPoints(const std::vector<Vec3f>& pts, const Vec3f n0, float d0, std::vector<int>& ptsIdx) {
  for (decltype(pts.size()) j = 0; j < pts.size(); j++) {
    const auto& p = pts[j];
    if (floatEqual(Vec3f::dot(n0, p) + d0, 0)) {
      ptsIdx.push_back(static_cast<int>(j));
    }
  }
}


void buildPolyhedronFaces(HalfSpaceSet& hss, const std::vector<Math::Vec3f>& pts,
                          std::vector<Math::TriangleIdx>& faces) {
  int num = hss.n;
  float* a = hss.a, *b = hss.b, *c = hss.c, *d = hss.d;

  for (int i = 0; i < num; i++) {
    /* Find co-planer points */
    std::vector<int> facePtsIdx;
    Vec3f n0(a[i], b[i], c[i]);
    findCoplanarPoints(pts, n0, d[i], facePtsIdx);
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
    }
  );

  /* Construct a triangular division */
  for (decltype(ptsIdx.size()) j = 1; j < ptsIdx.size() - 1; j++) {
    faces.emplace_back(TriangleIdx(ptsIdx[0], ptsIdx[j], ptsIdx[j+1]));
  }
}


DummyMatrix::DummyMatrix(float* data, uint64_t row, uint64_t col)
    : rowNum(row), colNum(col), data(data) {}

int DummyMatrix::multiply(const DummyMatrix& a, const DummyMatrix& b, DummyMatrix& res) {
  if (a.colNum != b.rowNum) {
    return -1;
  }

  for (uint64_t r = 0; r < a.rowNum; r++) {
    for (uint64_t c = 0; c < b.colNum; c++) {
      float sum = 0.0f;
      for (uint64_t k = 0; k < a.colNum; k++) {
        sum += a.data[r*a.colNum + k] * b.data[k*b.colNum + c];
      }
      res.data[r*res.colNum + c] = sum;
    }
  }
  return 0;
}

void DummyMatrix::transpose() {
  for (uint64_t r = 0; r < rowNum; r++) {
    for (uint64_t c = r+1; c < colNum; c++) {
      float tmp;
      tmp = data[r*colNum + c];
      data[r*colNum + c] = data[c*colNum +r];
      data[c*colNum +r] = tmp;
    }
  }
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


int TriangleIdx::id1() const {
  return _idx[0];
}


int TriangleIdx::id2() const {
  return _idx[1];
}


int TriangleIdx::id3() const {
  return _idx[2];
}


const int * TriangleIdx::idx() const {
  return &_idx[0];
}


HalfSpaceSet::HalfSpaceSet(int n, float* a, float* b, float* c, float* d)
    : n(n), a(a), b(b), c(c), d(d) {}

}  // namespace Math


/*
  top vertex:
    2   1
  3       0
    4   5

  upper vertex:
    8   7
  9       6
    10  11

  lower vertex:
    14  13
  15      12
    16  17

  bottom vertex:
    20  19
  21      18
    22  23
*/


/*
  top vertex:
    2   1
  3       0
    4   5

  upper vertex:
    8   7
  9       6
    10  11

  lower vertex:
    14  13
  15      12
    16  17

  bottom vertex:
    20  19
  21      18
    22  23
*/


namespace Math {

OrientationGenerator::OrientationGenerator()
    : axDist(Distribution::UNIFORM), axMean(0), axStd(0),
      rollDist(Distribution::UNIFORM), rollMean(0), rollStd(0) {}

OrientationGenerator::OrientationGenerator(Distribution axDist, float axMean, float axStd,
                                           Distribution rollDist, float rollMean, float rollStd)
    : axDist(axDist), axMean(axMean), axStd(axStd),
      rollDist(rollDist), rollMean(rollMean), rollStd(rollStd) {
  unsigned int seed = static_cast<unsigned int>(std::chrono::system_clock::now().time_since_epoch().count());
  // unsigned int seed = 2345;
  generator.seed(seed);
}

void OrientationGenerator::fillData(const float* sunDir, int num, float* rayDir, float* mainAxRot) {
  for (int i = 0; i < num; i++) {
    float lon = 0, lat = 0, roll = 0;

    switch (axDist) {
      case Distribution::UNIFORM : {
        float v[3] = {gaussDistribution(generator),
                gaussDistribution(generator),
                gaussDistribution(generator)};
        Math::normalize3(v);
        lon = atan2(v[1], v[0]);
        lat = asin(v[2] / Math::norm3(v));
      }
        break;
      case Distribution::GAUSS :
        lon = uniformDistribution(generator) * 2 * Math::kPi;
        lat = gaussDistribution(generator) * axStd;
        lat += axMean;
        if (lat > Math::kPi / 2) {
          lat = Math::kPi - lat;
        }
        if (lat < -Math::kPi / 2) {
          lat = -Math::kPi - lat;
        }
        break;
    }

    switch (rollDist) {
      case Distribution::GAUSS :
        roll = gaussDistribution(generator) * rollStd + rollMean;
        break;
      case Distribution::UNIFORM :
        roll = (uniformDistribution(generator) - 0.5f) * rollStd + rollMean;
        break;
    }

    mainAxRot[i*3+0] = lon;
    mainAxRot[i*3+1] = lat;
    mainAxRot[i*3+2] = roll;

    std::memcpy(rayDir+i*3, sunDir, 3*sizeof(float));
    Math::rotateZ(mainAxRot + i * 3, rayDir + i * 3);
  }
}

}   // namespace Math

}   // namespace IceHalo
