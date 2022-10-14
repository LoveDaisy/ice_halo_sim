#ifndef SRC_CORE_MYMATH_H_
#define SRC_CORE_MYMATH_H_

#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <memory>
#include <random>
#include <set>
#include <vector>

#include "core/def.hpp"
#include "json.hpp"

namespace icehalo {

struct AxisDistribution;

namespace math {

constexpr float kPi = 3.14159265359f;
constexpr float kPi_2 = kPi / 2.0f;
constexpr float kPi_3 = kPi / 3.0f;
constexpr float kPi_4 = kPi / 4.0f;
constexpr float kPi_6 = kPi / 6.0f;
constexpr float kSqrt3 = 1.73205080757f;
constexpr float kSqrt3_2 = kSqrt3 / 2.0f;
constexpr float kSqrt3_4 = kSqrt3 / 4.0f;
constexpr float kFloatEps = 1e-5;
constexpr float kDegreeToRad = kPi / 180.0f;
constexpr float kRadToDegree = 180.0f / kPi;

}  // namespace math


enum class AngleUnit {
  kDegree,
  kRad,
};


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
  explicit Pose3(const T* data, AngleUnit unit = AngleUnit::kDegree);
  Pose3(T lon, T lat, T roll, AngleUnit unit = AngleUnit::kDegree);

  T lon() const;
  T lat() const;
  T roll() const;
  void lon(T lon);
  void lat(T lat);
  void roll(T roll);

  const T* val() const;
  void val(T lat, T lon, T roll);
  void val(const T* data);

  AngleUnit unit() const;
  void ToDegree();
  void ToRad();

 private:
  Vec3<T> val_;
  AngleUnit unit_;
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


enum class DistributionType {
  kNoRandom,
  kUniform,
  kGaussian,
};


struct Distribution {
  DistributionType type;
  float mean;
  float std;
};


class RandomNumberGenerator {
 public:
  explicit RandomNumberGenerator(uint32_t seed);

  float GetGaussian();
  float GetUniform();
  float Get(Distribution dist);
  void Reset();
  void SetSeed(uint32_t seed);

  static RandomNumberGenerator* GetInstance();

 private:
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
void TriangleNormal(const float* p1, const float* p2, const float* p3, float* normal);

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

  Distribution azimuth_dist;
  Distribution latitude_dist;
  Distribution roll_dist;
};


namespace v3 {

/**
 * @brief Find the intersection point of two 2D lines. Lines have forms of a*x + b*y + c = 0
 *
 * @param coef1 Coefficients for first line. [a, b, c]
 * @param coef2 Coefficients for second line. [a, b, c]
 * @param res Intersection point. [x, y]
 * @return true Find the intersection.
 * @return false No intersection.
 */
bool SolveLines(const float* coef1, const float* coef2, float* res);

/**
 * @brief Find the intersection point of three 3D planes. Planes have forms of a*x + b*y + c*z + d = 0
 *
 * @param coef1 Coefficients for 1st plane. [a, b, c, d]
 * @param coef2 Coefficients for 2nd plane. [a, b, c, d]
 * @param coef3 Coefficients for 3rd plane. [a, b, c, d]
 * @param res Intersection point. [x, y, z]
 * @return true Find the intersection.
 * @return false No intersection.
 */
bool SolvePlanes(const float* coef1, const float* coef2, const float* coef3, float* res);

/**
 * @brief Check if a 2D point locates **IN** a polygon. The polygon is defined by intersection of
 *        multiple half planes: a*x + b*y + c <= 0
 *
 * @param n Polygon edge number. The number of half planes.
 * @param coef Coefficients of half planes.
 * @param xy Point to be checked.
 * @param boundary Whether include boundary.
 * @return true
 * @return false
 */
bool IsInPolygon2(int n, const float* coef, const float xy[2], bool boundary = true);

/**
 * @brief Check if a 3D point locates **IN** a polyhedron. The polyhedron is defined by intersection of
 *        multiple half spaces: a*x + b*y + c*z + d <= 0
 *
 * @param n Number of half spaces.
 * @param coef Coefficients of half spaces. [a, b, c, d]
 * @param xyz Point to be checked.
 * @param boundary Whether include boundary.
 * @return true
 * @return false
 */
bool IsInPolyhedron3(int n, const float* coef, const float xyz[3], bool boundary = true);

/**
 * @brief Find all vertices of a convex polyhedron, which is defined by intersection of multiple half spaces:
 *        a*x + b*y + c*z + d <= 0
 *
 * @param plane_cnt Number of half spaces.
 * @param coef_ptr Coefficients of half spaces. [a, b, c, c]
 * @return std::tuple<std::unique_ptr<float[]>, int> Vertices coordinates and actual vertices number.
 */
std::tuple<std::unique_ptr<float[]>, int> SolveConvexPolyhedronVtx(int plane_cnt, const float* coef_ptr);

/**
 * @brief Find vertices of the difference of two convex polyhedrons.
 *
 * @param plane_cnt1 Half space number of first polyhedron.
 * @param coef_ptr1 Half space coefficients of first polyhedron.
 * @param plane_cnt2 Half space number of second polyhedron.
 * @param coef_ptr2 Half space coefficients of second polyhedron.
 * @param vtx_cap
 * @return std::tuple<std::unique_ptr<float[]>, int> Vertices coordinates & vertices number.
 */
std::tuple<std::unique_ptr<float[]>, int> ConvexPolyhedronDifferenceVtx(int plane_cnt1, const float* coef_ptr1,
                                                                        int plane_cnt2, const float* coef_ptr2);

/**
 * @brief Collect those vertices on a same plane together by given surface plane.
 *
 * @param vtx_cnt Vertices number.
 * @param vtx_ptr Vertices coordinate. [x, y, z]
 * @param plane_cnt Plane number.
 * @param coef_ptr Plane coefficients. [a, b, c, d]
 * @return std::vector<std::set<int>> Vertices collections. Each set<int> is a set of co-plannar vertices.
 */
std::vector<std::set<int>> CollectSurfaceVtx(int vtx_cnt, const float* vtx_ptr, int plane_cnt, const float* coef_ptr);

/**
 * @brief Triangulate mesh.
 *
 * @param vtx_cnt Vertices number.
 * @param vtx_ptr Vertices coordinates. [x, y, z]
 * @param surface_vtx_idx Mesh surface indices. Each set is a surface.
 * @return std::tuple<std::unique_ptr<int[]>, int> Triangle indices, [i1, i2, i3], and triangle number.
 */
std::tuple<std::unique_ptr<int[]>, int> Triangulate(int vtx_cnt, const float* vtx_ptr,
                                                    const std::vector<std::set<int>>& surface_vtx_idx);

}  // namespace v3


// convertion to & from json object
NLOHMANN_JSON_SERIALIZE_ENUM(  // declear macro
    DistributionType,          // enum type
    {
        { DistributionType::kUniform, "uniform" },
        { DistributionType::kGaussian, "gauss" },
    })

void to_json(nlohmann::json& obj, const Distribution& dist);
void from_json(const nlohmann::json& obj, Distribution& dist);

void to_json(nlohmann::json& obj, const AxisDistribution& dist);
void from_json(const nlohmann::json& obj, AxisDistribution& dist);

}  // namespace icehalo

#endif  // SRC_CORE_MYMATH_H_
