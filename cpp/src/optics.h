#ifndef OPTICS_H
#define OPTICS_H

#include "mymath.h"

#include <atomic>
#include <mutex>


namespace IceHalo {

class SimulationContext;
class RayTracingContext;
class CrystalContext;


class RaySegment {
public:
  RaySegment();
  RaySegment(const float* pt, const float* dir, float w, int faceId = -1);
  ~RaySegment() = default;

  bool isValidEnd();
  void reset();

  RaySegment* nextReflect;
  RaySegment* nextRefract;
  RaySegment* prev;

  Math::Vec3f pt;
  Math::Vec3f dir;
  float w;
  int faceId;

  bool isFinished;
};


class Ray {
public:
  Ray(const float* pt, const float* dir, float w, int faceId = -1);
  explicit Ray(RaySegment* seg);
  ~Ray();

  size_t totalNum();

  RaySegment* firstRaySeg;

};


class Optics {
public:
  static void traceRays(SimulationContext &context);

private:
  static void hitSurface(float n, RayTracingContext* rayCtx);
  static void propagate(RayTracingContext* rayCtx, CrystalContext* cryCtx);

  static void hitSurfaceRange(float n, RayTracingContext* rayCtx, int startIdx, int endIdx);
  static void propagateRange(RayTracingContext* rayCtx, int faceNum, float* faces, int startIdx, int endIdx);
  static float getReflectRatio(float cos_angle, float rr);
  static void intersectLineTriangle(const float* pt, const float* dir, const float* face,
                                    float* p, float* t, float* alpha, float* beta);
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

  static const uint32_t chunkSize = 1024 * 128;

  // std::mutex idMutex;
  std::vector<RaySegment*> segments;
  std::atomic<uint64_t> nextUnusedId;
  std::atomic<uint64_t> currentChunkId;

};

}   // namespace IceHalo


#endif // OPTICS_H
