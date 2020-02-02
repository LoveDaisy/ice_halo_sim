#ifndef SRC_SIMULATION_H_
#define SRC_SIMULATION_H_

#include <vector>

#include "context.h"
#include "crystal.h"
#include "optics.h"

namespace icehalo {

class SimulationData {
 public:
  void Clear();
  void PrepareNewScatter(size_t ray_num);
  void EmplaceRay(RayInfoPtrU ray);
  size_t GetLastExitRayNumber() const;

  std::vector<std::vector<RayInfoPtrU>> rays_;
  std::vector<std::vector<RaySegment*>> exit_ray_segments_;
  std::vector<RaySegment*> final_ray_segments_;
};

class Simulator {
 public:
  explicit Simulator(ProjectContextPtr context);
  Simulator(const Simulator& other) = delete;
  ~Simulator() = default;

  void SetCurrentWavelengthIndex(int index);
  void Run();
  const std::vector<RaySegment*>& GetFinalRaySegments() const;
  void SaveFinalDirections(const char* filename);
  void SaveAllRays(const char* filename);
  void PrintRayInfo();  // For debug

 private:
  struct EntryRayData {
    EntryRayData();
    ~EntryRayData();

    void Clean();
    void Allocate(size_t ray_number);

    float* ray_dir;
    RaySegment** ray_seg;
    size_t ray_num;
  };


  struct BufferData {
   public:
    BufferData();
    ~BufferData();

    void Clean();
    void Allocate(size_t ray_number);
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


  static void InitMainAxis(const CrystalContext* ctx, float* axis);

  void InitSunRays();
  void InitEntryRays(const CrystalContext* ctx);
  void TraceRays(const Crystal* crystal, AbstractRayPathFilter* filter);
  void PrepareMultiScatterRays(float prob);
  void StoreRaySegments(const Crystal* crystal, AbstractRayPathFilter* filter);
  void RefreshBuffer();

  static constexpr int kBufferSizeFactor = 4;

  ProjectContextPtr context_;

  SimulationData simulation_data_;

  int current_wavelength_index_;

  size_t total_ray_num_;
  size_t active_ray_num_;
  size_t buffer_size_;

  BufferData buffer_;
  EntryRayData entry_ray_data_;
  size_t entry_ray_offset_;
};

}  // namespace icehalo

#endif  // SRC_SIMULATION_H_
