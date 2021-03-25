#ifndef SRC_CORE_MYMATH_H_
#define SRC_CORE_MYMATH_H_

#include <cmath>
#include <memory>
#include <mutex>
#include <random>
#include <vector>

#include "core_def.hpp"

namespace icehalo {

struct AxisDistribution;

namespace math {

constexpr float kPi = 3.14159265359f;
constexpr float kSqrt3 = 1.73205080757f;
constexpr float kFloatEps = 1e-5;
constexpr float kDegreeToRad = kPi / 180.0f;
constexpr float kRadToDegree = 180.0f / kPi;

}  // namespace math


template <typename T>
class Vec3 {
 public:
  explicit Vec3(const T* data);
  Vec3(T x, T y, T z);

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
  Vec3<T>& operator+=(T a);
  Vec3<T>& operator-=(const Vec3<T>& v);
  Vec3<T>& operator-=(T a);
  Vec3<T>& operator/=(T a);
  Vec3<T>& operator*=(T a);


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


template <class T>
class Pose3 {
 public:
  explicit Pose3(const T* data);
  Pose3(T lon, T lat, T roll);

  T lon() const;
  T lat() const;
  T roll() const;
  void lon(T lon);
  void lat(T lat);
  void roll(T roll);

  const T* val() const;
  void val(T lat, T lon, T roll);
  void val(const T* data);

 private:
  Vec3<T> val_;
};

using Pose3f = Pose3<float>;


class TriangleIdx {
 public:
  TriangleIdx(ShortIdType id1, ShortIdType id2, ShortIdType id3);

  const ShortIdType* idx() const;

 private:
  ShortIdType idx_[3];
};


class PolygonIdx {
 public:
  PolygonIdx();
  PolygonIdx(std::initializer_list<ShortIdType> idx);
  explicit PolygonIdx(std::vector<ShortIdType> idx);

  const std::vector<ShortIdType>& idx() const;

 private:
  std::vector<ShortIdType> idx_;
};


/* A set of half spaces.
 * Half space is defined as:
 *    a * x + b * y + c * z + d <= 0
 */
struct HalfSpaceSet {
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
  void Reset();

  static RandomNumberGenerator* GetInstance();

 private:
  explicit RandomNumberGenerator(uint32_t seed);

  uint32_t seed_;
  std::mt19937 generator_;
  std::normal_distribution<float> gauss_dist_;
  std::uniform_real_distribution<float> uniform_dist_;

  static constexpr uint32_t kDefaultRandomSeed = 1;
};


class RandomSampler {
 public:
  /*! @brief Generate points distributed uniformly on sphere around a give point, in Cartesian form.
   *
   * @param dir the given point, xyz.
   * @param std half range (like radii), in degree.
   * @param data output data, xyz.
   * @param num number of points.
   */
  static void SampleSphericalPointsCart(const float* dir, float std, float* data, size_t num = 1);

  /*! @brief Generate points distributed uniformly on sphere, in spherical form, (lon, lat).
   *
   * @param data output data, (lon, lat), in rad
   * @param num
   */
  static void SampleSphericalPointsSph(float* data, size_t num = 1, size_t step = 3);

  /*! @brief Generate points distributed on sphere surface up to latitude, in spherical form, (lon, lat).
   *
   * @param axis_dist axis distribution, including information of zenith / azimuth / roll.
   * @param data output data, (lon, lat), in rad
   * @param num number of points.
   */
  static void SampleSphericalPointsSph(const AxisDistribution& axis_dist, float* data, size_t num = 1);

  /*! @brief Generate points evenly distributed on a triangle, in Cartesian form, xyz.
   *
   * @param vertexes vertexes of the triangle.
   * @param data output data, xyz.
   * @param num number of points.
   */
  static void SampleTriangularPoints(const float* vertexes, float* data, size_t num = 1);

  /*! @brief Random choose an integer index from [0, max), proportional to probabilities in p.
   *
   * @param p probabilities, must have max values, sum of all p should be 1.0f.
   * @param max range bound.
   * @return chosen index.
   */
  static int SampleInt(const float* p, int max);

  /*! @brief Random choose an integer from [0, max)
   *
   * @param max range bound.
   * @return chosen integer.
   */
  static int SampleInt(int max);

  RandomSampler() = delete;
};


bool FloatEqual(float a, float b, float threshold = math::kFloatEps);
bool FloatEqualZero(float a, float threshold = math::kFloatEps);

float Dot3(const float* vec1, const float* vec2);
void Cross3(const float* vec1, const float* vec2, float* vec);
float Norm3(const float* vec);
float DiffNorm3(const float* vec1, const float* vec2);
void Normalize3(float* vec);
void Normalized3(const float* vec, float* vec_out);
void Vec3FromTo(const float* vec1, const float* vec2, float* vec);

void RotateZ(const float* lon_lat_roll, const float* input_vec, float* output_vec, size_t data_num = 1);
void RotateZ(const float* lon_lat_roll, const float* input_vec, float* output_vec, size_t input_step,
             size_t output_step, size_t data_num = 1);
void RotateZBack(const float* lon_lat_roll, const float* input_vec, float* output_vec, size_t data_num = 1);

std::vector<Vec3f> FindInnerPoints(const HalfSpaceSet& hss);
void SortAndRemoveDuplicate(std::vector<Vec3f>* pts);
std::vector<int> FindCoplanarPoints(const std::vector<Vec3f>& pts, const Vec3f& n0, float d0);
void BuildPolyhedronFaces(const HalfSpaceSet& hss, const std::vector<Vec3f>& pts, std::vector<TriangleIdx>& faces);
void BuildTriangularDivision(const std::vector<Vec3f>& vertex, const Vec3f& n, std::vector<int>& pts_idx,
                             std::vector<TriangleIdx>& faces);


struct AxisDistribution {
  AxisDistribution();

  Distribution latitude_dist;
  Distribution azimuth_dist;
  Distribution roll_dist;
  float latitude_mean;
  float azimuth_mean;
  float roll_mean;
  float latitude_std;
  float azimuth_std;
  float roll_std;
};

}  // namespace icehalo

#endif  // SRC_CORE_MYMATH_H_
