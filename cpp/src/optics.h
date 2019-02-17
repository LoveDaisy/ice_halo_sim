#ifndef SRC_OPTICS_H_
#define SRC_OPTICS_H_

#include "mymath.h"
#include "context.h"

#include <atomic>
#include <mutex>
#include <vector>
#include <memory>


namespace IceHalo {

class RaySegmentPool;
class Ray;

class RaySegment {
friend class RaySegmentPool;
public:
  bool isValidEnd();
  void reset();

  RaySegment* nextReflect;
  RaySegment* nextRefract;
  RaySegment* prev;
  Ray* root;

  Math::Vec3f pt;
  Math::Vec3f dir;
  float w;
  int faceId;

  bool isFinished;

private:
  RaySegment();
};


class Ray {
public:
  Ray(RaySegment* seg, const float main_axis_rot[3]);

  size_t totalNum();

  RaySegment* firstRaySeg;
  Math::Vec3f main_axis_rot;
};

using RayPtr = std::shared_ptr<Ray>;


class RaySegmentPool {
public:
  ~RaySegmentPool();
  RaySegmentPool(RaySegmentPool const&) = delete;
  void operator=(RaySegmentPool const&) = delete;

  static RaySegmentPool& getInstance();

  RaySegment* getRaySegment(const float* pt, const float* dir, float w, int faceId);
  void clear();

private:
  RaySegmentPool();

  static constexpr uint32_t kChunkSize = 1024 * 128;

  std::vector<RaySegment*> segments;
  std::atomic<uint64_t> nextUnusedId;
  std::atomic<uint64_t> currentChunkId;
};


class Optics {
public:
  static void HitSurface(const CrystalPtr& crystal, float n, size_t num,
                         const float* dir_in, const int* face_id_in, const float* w_in,
                         float* dir_out, float* w_out);
  static void Propagate(const CrystalPtr& crystal, size_t num,
                        const float* pt_in, const float* dir_in, const float* w_in,
                        float* pt_out, int* face_id_out);

private:
  static float getReflectRatio(float cos_angle, float rr);

  /*! \brief Intersect a line with many faces and find the nearest intersection point.
   *
   * \param pt a point on the line, 3 floats
   * \param dir the direction of the line, 3 floats
   * \param faceBases the face data, 6 floats for one face, represents for 2 base vector
   * \param facePoints the face data, 9 floats for one face, represents for 3 vertexes
   * \param faceNum the face number
   * \param p output argument, the intersection point
   * \param idx output argument, the face index of the intersection point
   */
  static void intersectLineWithTriangles(const float* pt, const float* dir,
                                         const float* faceBases, const float* facePoints, int faceNum,
                                         float* p, int* idx);
};


class IceRefractiveIndex {
public:
  static float n(float waveLength);

private:
  /*
   * data from https://refractiveindex.info/?shelf=3d&book=crystals&page=ice
   */
  static constexpr float _wl[] = {350.0f, 400.0f, 410.0f, 420.0f, 430.0f, 440.0f, 450.0f, 460.0f, 470.0f,
    480.0f, 490.0f, 500.0f, 510.0f, 520.0f, 530.0f, 540.0f, 550.0f, 560.0f, 570.0f, 580.0f, 590.0f,
    600.0f, 610.0f, 620.0f, 630.0f, 640.0f, 650.0f, 660.0f, 670.0f, 680.0f, 690.0f, 700.0f, 710.0f,
    720.0f, 730.0f, 740.0f, 750.0f, 760.0f, 770.0f, 780.0f, 790.0f, 800.0f, 810.0f, 820.0f, 830.0f,
    840.0f, 850.0f, 860.0f, 870.0f, 880.0f, 890.0f, 900.0f};
  static constexpr float _n[] = {1.3249f, 1.3194f, 1.3185f, 1.3177f, 1.3170f, 1.3163f, 1.3157f, 1.3151f,
    1.3145f, 1.3140f, 1.3135f, 1.3130f, 1.3126f, 1.3122f, 1.3118f, 1.3114f, 1.3110f, 1.3106f, 1.3103f,
    1.3100f, 1.3097f, 1.3094f, 1.3091f, 1.3088f, 1.3085f, 1.3083f, 1.3080f, 1.3078f, 1.3076f, 1.3073f,
    1.3071f, 1.3069f, 1.3067f, 1.3065f, 1.3062f, 1.3060f, 1.3058f, 1.3057f, 1.3055f, 1.3053f, 1.3051f,
    1.3049f, 1.3047f, 1.3045f, 1.3044f, 1.3042f, 1.3040f, 1.3038f, 1.3037f, 1.3035f, 1.3033f, 1.3032f};
};

}   // namespace IceHalo


#endif  // SRC_OPTICS_H_
