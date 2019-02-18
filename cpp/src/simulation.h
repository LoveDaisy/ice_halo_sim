#ifndef SRC_SIMULATION_H_
#define SRC_SIMULATION_H_

#include "context.h"
#include "crystal.h"
#include "optics.h"

#include <vector>

namespace IceHalo {

struct SimulationBufferData {
public:
  SimulationBufferData();
  ~SimulationBufferData();

  void Clean();
  void Allocate(size_t rayNum);
  void Print();

  float* pt[2];
  float* dir[2];
  float* w[2];
  int* face_id[2];
  RaySegment** ray_seg[2];

  size_t ray_num;

private:
  void DeleteBuffer(int idx);
};


class Simulator {
public:
  explicit Simulator(const SimulationContextPtr& context);
  ~Simulator() = default;

  void Start();
  void SaveFinalDirections(const char* filename);
  void SaveAllRays(const char* filename);
  void PrintRayInfo();    // For debug

private:
  void InitSunRays();
  void InitEntryRays(const CrystalContextPtr& ctx, int multiScatterIdx);
  void TraceRays(const CrystalPtr& crystal);
  void RestoreResultRays(int multiScatterIdx);
  void StoreRaySegments();
  void RefreshBuffer();

  static constexpr int kBufferSizeFactor = 4;

  SimulationContextPtr context_;
  std::vector<CrystalContextPtr> active_crystal_ctxs_;

  std::vector<std::vector<RayPtr> > rays_;
  std::vector<RaySegment*> final_ray_segments_;

  size_t total_ray_num_;
  size_t active_ray_num_;
  size_t buffer_size_;

  SimulationBufferData buffer_;
};

}  // namespace IceHalo

#endif  // SRC_SIMULATION_H_
