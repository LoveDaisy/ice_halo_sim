#ifndef SRC_CORE_OPTICS_H_
#define SRC_CORE_OPTICS_H_

#include <atomic>
#include <cstddef>
#include <memory>
#include <mutex>
#include <vector>

#include "core/core_def.hpp"
#include "core/crystal.hpp"
#include "core/math.hpp"
#include "io/serialize.hpp"


namespace icehalo {

enum class RaySegmentState : uint8_t {
  kOnGoing = 0,
  kFinished = 1,
  kCrystalAbsorbed = 2,
  kAirAbsorbed = 3,
  kContinued = 4,
};


/**
 * This struct keeps a recorder, and calculates hash of a ray path.
 * A ray path is a sequence of crystal id and face numbers,
 * and ended up with a kInvalidId value for each crystal. A typical ray path is like:
 * ~~~
 * ( 1,             3, 5,         kInvalidId,           2,            1, 3, 2,   kInvalidId, ... )
 *   |              |               |                   |             |            |
 * crystal id   face numbers  end of 1st crystal   2nd crystal   face numbers   end of 2nd crystal
 * ~~~
 *
 * With operator overloading, we can use this struct simply. If we want to calculate the hash of
 * ray path above:
 * ~~~c++
 * RayPathRecorder recorder{};
 * recorder << 1, 3, 5, kInvalidId, 2, 1, 3, 2, kInvalidId;
 * recorder.Hash();   // Get the hash of given ray path
 * ~~~
 * or alternatively:
 * ~~~c++
 * RayPath path{ 1, 3, 5, ... };
 * RayPathRecorder{}(path);   // Get the hash of given ray path
 * ~~~
 */
class RayPathRecorder : public ISerializable {
 public:
  static constexpr size_t kStep = 7;
  static constexpr size_t kTotalBits = sizeof(size_t) * CHAR_BIT;

  RayPathRecorder();

  size_t Hash() const noexcept;
  void Clear() noexcept;

  static size_t Hash(const RayPath& ray_path) noexcept;

  void Serialize(File& file, bool with_boi) const override;
  void Deserialize(File& file, endian::Endianness endianness) override;

  RayPathRecorder& operator<<(ShortIdType id);
  RayPathRecorder& operator<<(const RayPathRecorder& second);
  RayPathRecorder& operator>>(RayPathRecorder& second);
  RayPathRecorder& operator,(ShortIdType id);

 private:
  static void ReverseBits(uint8_t* n);
  static size_t RightCircularShift(size_t val, unsigned int n);
  static size_t LeftCircularShift(size_t val, unsigned int n);

  size_t hash_;
  int16_t offset_;
};


struct RaySegment : public ISerializable {
  RaySegment();
  RaySegment(const float* pt, const float* dir, float w, int face_id);

  /**
   * @brief Serialize self to a file.
   *
   * There are 4 pointers in this struct, of 2 types, RaySegment and RayInfo. Both of them are pooled
   * object types. Thus we store two uint32_t data (chunk ID, object ID) to hold the pointer.
   *
   * The file layout will be:
   * uint32 * 2,        // next_reflect
   * uint32 * 2,        // next_refract
   * uint32 * 2,        // prev
   * uint32 * 2,        // root_ctx
   * float * 3,         // pt
   * float * 3,         // dir
   * float,             // w
   * int16,             // face_id
   * uint8,             // state
   *
   * @warning We do **NOT** store raw pointer, which is impossible and is improper for deserialization.
   *
   * @param file
   * @param with_boi
   */
  void Serialize(File& file, bool with_boi) const override;

  /**
   * @brief Deserialize (load data) from a file.
   *
   * Since there are 4 pointer members in this struct, and they cannot be serialized plainly, we store
   * 2 uint32 data instead (see RaySegment::Serialize(File&, bool) ). The caller should further call
   * ObjectPool<T>::GetPointerFromSerializeData(T*) to get real pointer.
   *
   * @warning ObjectPool<T>::GetPointerFromSerializeData(T*) must be called **AFTER** the entire
   * object pool finishing its deserialization.
   *
   * @param file
   * @param endianness
   */
  void Deserialize(File& file, endian::Endianness endianness) override;

  RaySegment* next_reflect;
  RaySegment* next_refract;
  RaySegment* prev;
  RayInfo* root_ctx;

  Vec3f pt;
  Vec3f dir;
  float w;
  int face_id;
  RaySegmentState state;
  RayPathRecorder recorder;
};


struct RayInfo : public ISerializable {
  RayInfo();
  RayInfo(RaySegment* seg, int crystal_id, const float* main_axis);

  /**
   * @brief Serialize self to a file.
   *
   * There are 2 pointers in this struct, of type RaySegment. RaySegment is a pooled object type. Thus
   * we store two uint32_t data (chunk ID, object ID) to hold the pointer.
   *
   * The file layout will be:
   * uint32 * 2,            // first_ray_segment
   * uint32 * 2,            // prev_ray_segment
   * int32,                 // crystal ID
   * float * 3,             // main_axis
   *
   * @param file
   * @param with_boi
   */
  void Serialize(File& file, bool with_boi) const override;

  /**
   * @brief Deserialize (load data) from a file.
   *
   * Since there are 2 pointer members in this struct, and they cannot be serialized plainly, we store
   * 2 uint32 data (see RaySegment::Serialize(File&, bool) ) instead. The caller should further call
   * ObjectPool<T>::GetPointerFromSerializeData(T*) to get real ray segment pointer.
   *
   * @warning ObjectPool<T>::GetPointerFromSerializeData(T*) must be called **AFTER** the entire
   * object pool finishing its deserialization.
   *
   * @param file
   * @param endianness
   */
  void Deserialize(File& file, endian::Endianness endianness) override;

  RaySegment* first_ray_segment;
  RaySegment* prev_ray_segment;
  int32_t crystal_id;
  Vec3f main_axis;
};

namespace v3 {

struct RaySeg;
struct CrystalStub;

struct RayRoot {
  RayRoot* prev_;
  RaySeg* r0_;
  CrystalStub* crystal_;
};

struct RaySeg {
  float d_[3];
  float p_[3];
  float w_;
  int fid_;

  RaySeg* prev_;
  RayRoot* root_;
};

/**
 * @brief Compute the reflective & refractive result when a ray hits a surface.
 *
 * @param crystal    [input]
 * @param n          [input] refractive index
 * @param num        [input] how many rays
 * @param dir_in     [input] direction. sizeof(float) * num * 3, [dx, dy, dz], ...
 * @param face_id_in [input] index of crystal face. NOT face number (which used in raypaths).
 *                           sizeof(int) * num
 * @param w_in       [input] intensity. sizeof(float) * num
 *
 * @param dir_out    [output] direction. sizeof(float) * num * 6,
 *                            [dx, dy, dz]_{reflective, refractive}, ...
 * @param w_out      [output] intensity. sizeof(float) * num, w_{reflective, refractive}, ...
 */
void HitSurface(const Crystal* crystal, float n, size_t num,                    // input
                const float* dir_in, const int* face_id_in, const float* w_in,  // input
                float* dir_out, float* w_out);                                  // output

/**
 * @brief Find out which face (triangle) will a ray hit, then calculate its intersection point.
 *
 * @param crystal     [input]
 * @param num         [input] how many rays
 * @param pt_in       [input] start point of rays. sizeof(float) * num * 3, [px, py, pz], ...
 * @param dir_in      [input] direction of rays. sizeof(float) * num * 3, [dx, dy, dz], ...
 * @param w_in        [input] intensity. sizeof(float) * num
 * @param face_id_in  [input] index of crystal face. NOT face number (which used in raypaths).
 *                            sizeof(int) * num
 *
 * @param pt_out      [output] intersection point. sizeof(float) * num * 3, [px, py, pz], ...
 * @param face_id_out [output] index of crystal face. sizeof(int) * num.
 */
void Propagate(const Crystal* crystal, size_t num, size_t step,             // input
               const float* pt_in, const float* dir_in, const float* w_in,  // input
               float* pt_out, int* face_id_out);                            // output

}  // namespace v3


class Optics {
 public:
  static void HitSurface(const Crystal* crystal, float n, size_t num,                    // input
                         const float* dir_in, const int* face_id_in, const float* w_in,  // input
                         float* dir_out, float* w_out);                                  // output

  static void Propagate(const Crystal* crystal, size_t num,                                                 // input
                        const float* pt_in, const float* dir_in, const float* w_in, const int* face_id_in,  // input
                        float* pt_out, int* face_id_out);                                                   // output

  /*! \brief Intersect a line with many faces and find the nearest intersection point.
   *
   * \param pt a point on the line, 3 floats
   * \param dir the direction of the line, 3 floats
   * \param face_bases the face data, 6 floats for one face, represents for 2 base vector
   * \param face_points the face data, 9 floats for one face, represents for 3 vertexes
   * \param face_num the face number
   * \param p output argument, the intersection point
   * \param idx output argument, the face index of the intersection point
   */
  static void IntersectLineWithTriangles(const float* pt, const float* dir,  // input
                                         int face_id, int face_num,          // input
                                         const float* face_bases,            // input, (pt1 - pt2, pt1 - pt3)
                                         const float* face_points,           // input (pt1, pt2, pt3)
                                         const float* face_norm,             // input
                                         float* p, int* idx);                // output

  static void IntersectLineWithTrianglesSimd(const float* pt, const float* dir,  // input
                                             int face_id, int face_num,          // input
                                             const float* face_bases,            //
                                             const float* face_points,           //
                                             const float* face_norm,             //
                                             float* p, int* idx);                // output

 private:
  static float GetReflectRatio(float delta, float rr);
};


class IceRefractiveIndex {
 public:
  static constexpr float kMinWaveLength = 350;
  static constexpr float kMaxWaveLength = 900;

  static double Get(double wave_length);

 private:
  /* Shellmeier's equation:
   *
   *   n^2 = 1 + B1 * lambda^2 / (lambda^2 - C1^2)
   *           + B2 * lambda^2 / (lambda^2 - C2^2)
   *   lambda in micrometer, C1 * 1e-2, C2 * 1e2
   */
  static constexpr float kCoefAvr[] = { 0.701777f, 1.091144f, 0.884400f, 0.796950f };  // B1, B2, C1, C2
  static constexpr float kCoefO[] = { 0.696364f, 0.719271f, 0.957220f, 1.096889f };
  static constexpr float kCoefE[] = { 0.699934f, 0.640071f, 0.960906f, 0.964654f };
};

}  // namespace icehalo


#endif  // SRC_CORE_OPTICS_H_
