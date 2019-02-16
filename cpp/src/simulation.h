#ifndef SRC_SIMULATION_H_
#define SRC_SIMULATION_H_

#include "context.h"
#include "crystal.h"
#include "optics.h"

#include <vector>

namespace IceHalo {

class SimulationBufferData {
public:
  SimulationBufferData();
  ~SimulationBufferData();

  void clean();
  void allocate(size_t rayNum);

  float* pt[2];
  float* dir[2];
  float* w[2];
  int* faceId[2];
  RaySegment** raySeg[2];

  size_t rayNum;

private:
  void deleteBuffer(int idx);
};


class Simulator {
public:
  explicit Simulator(const SimulationContextPtr& context);
  ~Simulator() = default;

  void start();
  void saveFinalDirections(const char* filename);
  void saveAllRays(const char* filename);

  static constexpr float kPropMinW = 1e-6;
  static constexpr float kScatMinW = 1e-3;

private:
  void initSunRays();
  void initEntryRays(const CrystalContextPtr& ctx, int multiScatteringIdx);
  void traceRays(const CrystalPtr& crystal);
  void restoreResultRays(int multiScatteringIdx);
  void saveRaySegments();
  void refreshBuffer();

  static constexpr int kBufferSizeFactor = 8;

  SimulationContextPtr context;
  std::vector<CrystalContextPtr> activeCrystalCtxs;

  std::vector<std::vector<RayPtr> > rays;
  std::vector<std::vector<Math::Vec3f> > mainAxisRotation;

  size_t totalRayNum;
  size_t activeRayNum;
  size_t bufferSize;

  SimulationBufferData buffer;
};

}  // namespace IceHalo

#endif  // SRC_SIMULATION_H_
