#ifndef SRC_MYMATH_H_
#define SRC_MYMATH_H_

#include <vector>
#include <cmath>
#include <random>
#include <memory>
#include <mutex>

namespace IceHalo {

struct AxisDistribution;

namespace Math {

class DummyMatrix;
class ConstDummyMatrix;

constexpr float kPi = 3.14159265359f;
constexpr float kSqrt3 = 1.73205080757f;
constexpr float kFloatEps = 1e-6;
constexpr float kDegreeToRad = kPi / 180.0f;
constexpr float kRadToDegree = 180.0f / kPi;


int MatrixMultiply(ConstDummyMatrix& a, ConstDummyMatrix& b, DummyMatrix* c);

class DummyMatrix {
public:
friend int MatrixMultiply(ConstDummyMatrix& a, ConstDummyMatrix& b, DummyMatrix* c);
public:
  DummyMatrix(float* data, uint64_t row, uint64_t col);
  ~DummyMatrix() = default;

  const uint64_t rows_;
  const uint64_t cols_;

private:
  float* data_;
};


class ConstDummyMatrix : public DummyMatrix {
public:
friend int MatrixMultiply(ConstDummyMatrix& a, ConstDummyMatrix& b, DummyMatrix* c);
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

  Vec3<T> Normalized();
  void Normalize();

  Vec3<T>& operator+=(const Vec3<T>& v);
  Vec3<T>& operator+= (T a);
  Vec3<T>& operator-=(const Vec3<T>& v);
  Vec3<T>& operator-= (T a);
  Vec3<T>& operator/= (T a);
  Vec3<T>& operator*= (T a);


  static Vec3<T> Normalized(const Vec3<T>& v);

  static T Dot(const Vec3<T>& v1, const Vec3<T>& v2);
  static T Norm(const Vec3<T>& v);
  static Vec3<T> Cross(const Vec3<T>& v1, const Vec3<T>& v2);

  static Vec3<T> FromTo(const Vec3<T>& v1, const Vec3<T>& v2);

private:
  T val_[3];
};

bool operator==(const Vec3<float>& lhs, const Vec3<float>& rhs);

using Vec3f = Vec3<float>;


class TriangleIdx {
public:
  TriangleIdx(int id1, int id2, int id3);

  const int* idx() const;

private:
  int idx_[3];
};


/* A set of half spaces.
 * Half space is defined as:
 *    a * x + b * y + c * z + d <= 0
 */
class HalfSpaceSet {
public:
  HalfSpaceSet(int n, float* a, float* b, float* c, float* d);

  int n;
  float* a;
  float* b;
  float* c;
  float* d;
};


enum class Distribution {
  kUniform,
  kGaussian,
};


class RandomNumberGenerator {
public:
  float GetGaussian();
  float GetUniform();
  float Get(Distribution dist, float mean, float std);

  static std::shared_ptr<RandomNumberGenerator> GetInstance();

private:
  explicit RandomNumberGenerator(uint32_t seed);

  std::mt19937 generator_;
  std::normal_distribution<float> gauss_dist_;
  std::uniform_real_distribution<float> uniform_dist_;

  static constexpr uint32_t kDefaultRandomSeed = 1;
  static std::shared_ptr<RandomNumberGenerator> instance_;
  static std::mutex instance_mutex_;
};

using RandomNumberGeneratorPtr = std::shared_ptr<RandomNumberGenerator>;


class RandomSampler {
public:
  /*! @brief Generate points distributed uniformly on sphere around a give point, in Cartesian form.
   *
   * @param dir the given point, xyz.
   * @param std half range (like radii), in degree.
   * @param data output data, xyz.
   * @param num number of points.
   */
  void SampleSphericalPointsCart(const float* dir, float std, float* data, size_t num = 1);

  /*! @brief Generate points distributed uniformly on sphere, in spherical form, (lon, lat).
   *
   * @param data output data, (lon, lat), in rad
   * @param num
   */
  void SampleSphericalPointsSph(float* data, size_t num = 1);

  /*! @brief Generate points distributed on sphere surface up to latitude, in spherical form, (lon, lat).
   *
   * @param axis_dist axis distribution, including information of zenith / azimuth / roll.
   * @param data output data, (lon, lat), in rad
   * @param num number of points.
   */
  void SampleSphericalPointsSph(const AxisDistribution& axis_dist, float* data, size_t num = 1);

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

  static std::shared_ptr<RandomSampler> GetInstance();

private:
  RandomSampler() = default;

  static std::shared_ptr<RandomSampler> instance_;
  static std::mutex instance_mutex_;
};

using RandomSamplerPtr = std::shared_ptr<RandomSampler>;


bool FloatEqual(float a, float b, float threshold = kFloatEps);
bool FloatEqualZero(float a, float threshold = kFloatEps);

float Dot3(const float* vec1, const float* vec2);
void Cross3(const float* vec1, const float* vec2, float* vec);
float Norm3(const float* vec);
float DiffNorm3(const float* vec1, const float* vec2);
void Normalize3(float* vec);
void Normalized3(const float* vec, float* vec_out);
void Vec3FromTo(const float* vec1, const float* vec2, float* vec);

void RotateZ(const float* lon_lat_roll, const float* input_vec, float* output_vec, uint64_t data_num = 1);
void RotateZBack(const float* lon_lat_roll, const float* input_vec, float* output_vec, uint64_t data_num = 1);

std::vector<Vec3f> FindInnerPoints(const HalfSpaceSet& hss);
void SortAndRemoveDuplicate(std::vector<Vec3f>* pts);
std::vector<int> FindCoplanarPoints(const std::vector<Vec3f>& pts, const Vec3f& n0, float d0);
void BuildPolyhedronFaces(const HalfSpaceSet& hss, const std::vector<Math::Vec3f>& pts,
                          std::vector<Math::TriangleIdx>& faces);
void BuildTriangularDivision(const std::vector<Vec3f>& vertex, const Vec3f& n,
                             std::vector<int>& pts_idx, std::vector<TriangleIdx>& faces);

}   // namespace Math


struct AxisDistribution {
  AxisDistribution();

  Math::Distribution latitude_dist;
  Math::Distribution azimuth_dist;
  Math::Distribution roll_dist;
  float latitude_mean;
  float azimuth_mean;
  float roll_mean;
  float latitude_std;
  float azimuth_std;
  float roll_std;
};

}   // namespace IceHalo

#endif  // SRC_MYMATH_H_
