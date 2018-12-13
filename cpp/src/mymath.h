#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <vector>
#include <cmath>
#include <random>


namespace IceHalo {

namespace Math {

constexpr float kPi = 3.14159265359f;
constexpr float kSqrt3 = 1.73205080757f;
constexpr float kFloatEps = 1e-6;


class DummyMatrix {
public:
  DummyMatrix(float* data, uint64_t row, uint64_t col);
  ~DummyMatrix() = default;

  const uint64_t rowNum;
  const uint64_t colNum;

  void transpose();

  static int multiply(const DummyMatrix& a, const DummyMatrix& b, DummyMatrix& c);

private:
  float* data;
};


template <typename T>
class Vec3 {
public:
  explicit Vec3(const T* data);
  Vec3(T x, T y, T z);
  Vec3(const Vec3<T>& v);

  T x() const;
  T y() const;
  T z() const;
  void x(T x);
  void y(T y);
  void z(T z);

  const T* val() const;
  void val(T x, T y, T z);
  void val(const T* data);

  Vec3<T> normalized();
  void normalize();

  Vec3<T>& operator+=(const Vec3<T>& v);
  Vec3<T>& operator+= (T a);
  Vec3<T>& operator-=(const Vec3<T>& v);
  Vec3<T>& operator-= (T a);
  Vec3<T>& operator/= (T a);
  Vec3<T>& operator*= (T a);


  static Vec3<T> normalized(const Vec3<T>& v);

  static T dot(const Vec3<T>& v1, const Vec3<T>& v2);
  static T norm(const Vec3<T>& v);
  static Vec3<T> cross(const Vec3<T>& v1, const Vec3<T>& v2);

  static Vec3<T> fromVec(const Vec3<T>& v1, const Vec3<T>& v2);

private:
  T _val[3];
};

bool operator==(const Vec3<float>& lhs, const Vec3<float>& rhs);

using Vec3f = Vec3<float>;


class TriangleIdx {
public:
  TriangleIdx(int id1, int id2, int id3);

  int id1() const;
  int id2() const;
  int id3() const;

  const int* idx() const;

private:
  int _idx[3];
};


/* A set of half spaces.
 * Half space is defined as:
 *    a * x + b * y + c * z + d <= 0
 */
class HalfSpaceSet {
public:
  HalfSpaceSet(int n, float* a, float* b, float* c, float* d);
  ~HalfSpaceSet() = default;

  int n;
  float* a;
  float* b;
  float* c;
  float* d;
};


enum class Distribution {
  UNIFORM,
  GAUSS
};


class OrientationGenerator {
public:
  OrientationGenerator();
  OrientationGenerator(Distribution axDist, float axMean, float axStd,
                       Distribution rollDist, float rollMean, float rollStd);
  ~OrientationGenerator() = default;

  void fillData(const float* sunDir, int num, float* rayDir, float* mainAxRot);

private:
  std::mt19937 generator;
  std::normal_distribution<float> gaussDistribution;
  std::uniform_real_distribution<float> uniformDistribution;

  Distribution axDist;
  float axMean;
  float axStd;

  Distribution rollDist;
  float rollMean;
  float rollStd;
};


bool floatEqual(float a, float b, float threshold = kFloatEps);

float dot3(const float* vec1, const float* vec2);
void cross3(const float* vec1, const float* vec2, float* vec);
float norm3(const float* vec);
float diffNorm3(const float* vec1, const float* vec2);
void normalize3(float* vec);
void normalized3(const float* vec, float* vec_n);
void vec3FromTo(const float* vec1, const float* vec2, float* vec);

void rotateBase(const float* ax, float angle, float* vec);
void rotateZ(const float* lon_lat_roll, float* vec, uint64_t dataNum = 1);
void rotateZBack(const float* lon_lat_roll, float* vec, uint64_t dataNum = 1);

void findInnerPoints(HalfSpaceSet& hss, std::vector<Vec3f>& pts);
void sortAndRemoveDuplicate(std::vector<Vec3f>& pts);
void findCoplanarPoints(const std::vector<Vec3f>& pts, const Vec3f n0, float d0, std::vector<int>& ptsIdx);
void buildPolyhedronFaces(HalfSpaceSet& hss, const std::vector<Math::Vec3f>& pts,
                          std::vector<Math::TriangleIdx>& faces);
void buildTriangularDivision(const std::vector<Vec3f>& vertex, const Vec3f& n,
                             std::vector<int>& ptsIdx, std::vector<TriangleIdx>& faces);

}   // namespace Math

}   // namespace IceHalo

#endif // GEOMETRY_H
