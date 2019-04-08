#ifndef SRC_OPTICS_H_
#define SRC_OPTICS_H_

#include <atomic>
#include <memory>
#include <mutex>
#include <vector>

#include "crystal.h"
#include "mymath.h"


namespace IceHalo {

class RaySegmentPool;
struct RayInfo;

struct RaySegment {
  friend class RaySegmentPool;

 public:
  void ResetWith(const float* pt, const float* dir, float w, int face_id);

  RaySegment* next_reflect;
  RaySegment* next_refract;
  RaySegment* prev;
  RayInfo* root_ctx;

  Math::Vec3f pt;
  Math::Vec3f dir;
  float w;
  int face_id;

  bool is_finished;

 private:
  RaySegment();
};


class RaySegmentPool {
 public:
  ~RaySegmentPool();
  RaySegmentPool(RaySegmentPool const&) = delete;
  void operator=(RaySegmentPool const&) = delete;

  RaySegment* GetRaySegment(const float* pt, const float* dir, float w, int faceId);
  void Clear();

  static RaySegmentPool* GetInstance();

 private:
  RaySegmentPool();

  static constexpr uint32_t kChunkSize = 1024 * 512;
  static RaySegmentPool* instance_;

  std::vector<RaySegment*> segments_;
  size_t current_chunk_id_;
  std::atomic<uint32_t> next_unused_id_;
  std::mutex id_mutex_;
};

using RaySegmentPoolPtr = std::shared_ptr<RaySegmentPool>;


class Optics {
 public:
  static void HitSurface(const CrystalPtr& crystal, float n, size_t num,                 // input
                         const float* dir_in, const int* face_id_in, const float* w_in,  // input
                         float* dir_out, float* w_out);                                  // output

  static void Propagate(const CrystalPtr& crystal, size_t num,                                              // input
                        const float* pt_in, const float* dir_in, const float* w_in, const int* face_id_in,  // input
                        float* pt_out, int* face_id_out);                                                   // output

  static float GetReflectRatio(float cos_angle, float rr);

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
};


class IceRefractiveIndex {
 public:
  static constexpr float kMinWaveLength = 350;
  static constexpr float kMaxWaveLength = 900;

  static float Get(float wave_length);

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

}  // namespace IceHalo


#endif  // SRC_OPTICS_H_
