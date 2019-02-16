#ifndef SRC_MYMATH_H_
#define SRC_MYMATH_H_

#include <vector>
#include <cmath>
#include <random>


namespace IceHalo {

namespace Math {

class DummyMatrix;
class ConstDummyMatrix;

constexpr float kPi = 3.14159265359f;
constexpr float kSqrt3 = 1.73205080757f;
constexpr float kFloatEps = 1e-6;


int matMultiply(ConstDummyMatrix& a, ConstDummyMatrix& b, DummyMatrix* c);

class DummyMatrix {
public:
friend int matMultiply(ConstDummyMatrix& a, ConstDummyMatrix& b, DummyMatrix* c);
public:
  DummyMatrix(float* data, uint64_t row, uint64_t col);
  ~DummyMatrix() = default;

  const uint64_t rowNum;
  const uint64_t colNum;

  void transpose();

private:
  float* data;
};


class ConstDummyMatrix : public DummyMatrix {
public:
friend int matMultiply(ConstDummyMatrix& a, ConstDummyMatrix& b, DummyMatrix* c);
public:
  ConstDummyMatrix(const float* data, uint64_t row, uint64_t col);
  ~ConstDummyMatrix() = default;

private:
  const float* data;
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


class RandomNumberGenerator {
public:
  float getGaussian();
  float getUniform();
  float get(Distribution dist, float mean, float std);

  static RandomNumberGenerator& GetInstance();

private:
  RandomNumberGenerator();

  std::mt19937 generator;
  std::normal_distribution<float> gauss_dist_;
  std::uniform_real_distribution<float> uniform_dist_;

  static constexpr int random_number_seed_ = 1;
};


class RandomSampler {
public:
  /*! @brief Generate points uniformly distributed on sphere surface, in Cartesian form.
   *
   * @param data xyz data.
   * @param num number of points.
   */
  void SampleSphericalPointsCart(float* data, size_t num = 1);

  /*! @brief Generate points distributed on sphere surface up to latitude, in Cartesian form.
   *
   * @param dist distribution type, uniform or Gaussian.
   * @param lat latitude.
   * @param std standard deviation (for Gaussian) or half range (for uniform), in degree
   * @param data output data, xyz.
   * @param num number of points.
   */
  void SampleSphericalPointsCart(Distribution dist, float lat, float std, float* data, size_t num = 1);

  /*! @brief Generate points distributed uniformly on sphere around a give point, in Cartesian form.
   *
   * @param dir the given point, xyz.
   * @param std half range (like radii), in degree.
   * @param data output data, xyz.
   * @param num number of points.
   */
  void SampleSphericalPointsCart(const float* dir, float std, float* data, size_t num = 1);

  /*! @brief Generate points distributed on sphere surface up to latitude, in spherical form, (lon, lat).
   *
   * @param dist distribution type, uniform or Gaussian.
   * @param lat latitude.
   * @param std standard deviation (for Gaussian) or half range (for uniform), in degree.
   * @param data output data, (lon, lat)
   * @param num number of points.
   */
  void SampleSphericalPointsSph(Distribution dist, float lat, float std, float* data, size_t num = 1);

  /*! @brief Generate points evenly distributed on a triangle, in Cartesian form, xyz.
   *
   * @param vertexes vertexes of the triangle.
   * @param data output data, xyz.
   * @param num number of points.
   */
  void SampleTriangularPoints(const float* vertexes, float* data, size_t num = 1);

  /*! @brief Random choose an integer index from [0, max), proportional to probabilities in p.
   *
   * @param p probabilities, must have max values, sum of all p should be 1.0f.
   * @param max range bound.
   * @return chosen index.
   */
  int SampleInt(const float* p, int max);

  /*! @brief Random choose an integer from [0, max)
   *
   * @param max range bound.
   * @return chosen integer.
   */
  int SampleInt(int max);

  static RandomSampler& GetInstance();

private:
  RandomSampler() = default;
};


// class OrientationGenerator {
// public:
//   OrientationGenerator();
//   OrientationGenerator(Distribution axDist, float axMean, float axStd,
//                        Distribution rollDist, float rollMean, float rollStd);
//   ~OrientationGenerator() = default;
//
//   void fillData(const float* sunDir, int num, float* rayDir, float* mainAxRot);
//
// private:
//   std::mt19937 generator;
//   std::normal_distribution<float> gaussDistribution;
//   std::uniform_real_distribution<float> uniformDistribution;
//
//   Distribution axDist;
//   float axMean;
//   float axStd;
//
//   Distribution rollDist;
//   float rollMean;
//   float rollStd;
// };


bool floatEqual(float a, float b, float threshold = kFloatEps);

float dot3(const float* vec1, const float* vec2);
void cross3(const float* vec1, const float* vec2, float* vec);
float norm3(const float* vec);
float diffNorm3(const float* vec1, const float* vec2);
void normalize3(float* vec);
void normalized3(const float* vec, float* vec_n);
void vec3FromTo(const float* vec1, const float* vec2, float* vec);

void rotateZ(const float* lon_lat_roll, const float* input_vec, float* output_vec, uint64_t dataNum = 1);
void rotateZBack(const float* lon_lat_roll, const float* input_vec, float* output_vec, uint64_t dataNum = 1);

std::vector<Vec3f> findInnerPoints(const HalfSpaceSet& hss);
void sortAndRemoveDuplicate(std::vector<Vec3f>* pts);
std::vector<int> findCoplanarPoints(const std::vector<Vec3f>& pts, const Vec3f& n0, float d0);
void buildPolyhedronFaces(const HalfSpaceSet& hss, const std::vector<Math::Vec3f>& pts,
                          std::vector<Math::TriangleIdx>& faces);
void buildTriangularDivision(const std::vector<Vec3f>& vertex, const Vec3f& n,
                             std::vector<int>& ptsIdx, std::vector<TriangleIdx>& faces);

}   // namespace Math

}   // namespace IceHalo

#endif  // SRC_MYMATH_H_
