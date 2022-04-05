#ifndef SRC_CORE_SIMULATION_H_
#define SRC_CORE_SIMULATION_H_

#include <cstddef>
#include <cstring>
#include <memory>
#include <vector>

#include "context/context.hpp"
#include "core/crystal.hpp"
#include "core/optics.hpp"
#include "io/serialize.hpp"
#include "util/threading_pool.hpp"

namespace icehalo {

struct SimpleRayData : public ISerializable {
  explicit SimpleRayData(size_t num = 0);

  int wavelength;
  float wavelength_weight;
  std::unique_ptr<float[]> buf;
  size_t buf_ray_num;
  size_t init_ray_num;

  /**
   * @brief Serialize self to a file.
   *
   * This class is a simple wrapper for a piece of contiguous memory. So this method just simply write
   * all float numbers to a file.
   *
   * The file layout is:
   * float,                 // wavelength
   * float,                 // weight
   * uint64,                // number N
   * (float * 4) * N,       // x, y, z, w
   *
   * @param file
   * @param with_boi
   */
  void Serialize(File& file, bool with_boi) const override;

  /**
   * @brief Deserialize (load data) from a file.
   *
   * This class is a simple wrapper for a piece of contiguous memory. So this method just simply read
   * all float numbers from a file.
   *
   * @param file
   * @param endianness
   */
  void Deserialize(File& file, endian::Endianness endianness) override;
};


struct RayCollectionInfo {
  size_t identifier;
  float total_energy;
  bool is_partial_data;
  std::vector<size_t> idx;
};

using RayCollectionInfoList = std::vector<RayCollectionInfo>;


class SimulationData : public ISerializable {
 public:
  SimulationData();

  WavelengthInfo wavelength_info_;
  RayPathMap ray_path_map_;

  void SetThreadingPool(ThreadingPoolPtr threading_pool);

  void Clear();
  void PrepareNewScatter(size_t ray_num);
  void AddRay(RayInfo* ray);

  std::tuple<RayCollectionInfo, SimpleRayData> CollectFinalRayData();
  std::tuple<RayCollectionInfoList, SimpleRayData> CollectSplitRayData(const ProjectContextPtr& ctx,
                                                                       const RenderSplitter& splitter);

  void AddExitRaySegment(RaySegment* r);
  const std::vector<RaySegment*>& GetLastExitRaySegments() const;
#ifdef FOR_TEST
  const std::vector<std::vector<RaySegment*>>& GetExitRaySegments() const;
#endif

  /**
   * @brief Serialize self to a file.
   *
   * This class only holds pointers to ray segments and ray infos, rather than objects themselves.
   * To serialize data completely, this method will serialize ray segment pool and ray info
   * pool first.
   *
   * The layout of file is:
   * ray info pool,         // ray info pool
   * ray seg pool,          // ray seg pool
   * uint32,                // multi-scatters, K
   * {
   *   uint32,              // ray numbers, N
   *   (uint32 * 2) * N,    // ray info pointers
   * } * K
   * {
   *   uint32,              // ray seg numbers, N
   *   (uint32 * 2) * N,    // ray seg pointers
   * } * K
   *
   * @param file
   * @param with_boi
   */
  void Serialize(File& file, bool with_boi) const override;

  /**
   * @brief Deserialize (load data) from a file.
   *
   * This class only holds pointers to ray segments and ray infos, rather than objects themselves.
   * To load data correctly, this method will deserialize ray segment pool and ray info
   * pool first, i.e. it will clear all existing data in ray segment pool and ray info pool.
   *
   * @warning It will clear all existing data in ray segment pool and ray info pool.
   *
   * @param file
   * @param endianness
   */
  void Deserialize(File& file, endian::Endianness endianness) override;

 private:
  void MakeRayPathMap(const ProjectContextPtr& proj_ctx);
  std::tuple<RayCollectionInfoList, SimpleRayData> CollectSplitHaloRayData(const ProjectContextPtr& ctx);
  std::tuple<RayCollectionInfoList, SimpleRayData> CollectSplitFilterRayData(const ProjectContextPtr& ctx,
                                                                             const RenderSplitter& splitter);
  std::vector<std::vector<size_t>> GenerateIdxList() const;

  std::vector<std::vector<RayInfo*>> rays_;
  std::vector<std::vector<RaySegment*>> exit_ray_segments_;
  std::vector<size_t> exit_ray_seg_num_;
  ThreadingPoolPtr threading_pool_;
};

namespace v3 {

constexpr size_t kMaxMultiScatterings = 4;

struct SimulationConfig {
  float sun_altitude_;
  float sun_diameter_;

  int ray_num_;
  int max_hits_;

  int ms_num_;
  float ms_prob_;
  Crystal* ms_crystal_[kMaxMultiScatterings];
};

struct SimulationData {
  size_t num_;
  std::unique_ptr<float[]> d_;
  std::unique_ptr<float[]> p_;
  std::unique_ptr<float[]> w_;
  std::unique_ptr<int[]> fid_;

  SimulationData(size_t num)
      : num_(num), d_(new float[num * 3]{}), p_(new float[num * 3]{}), w_(new float[num]{}), fid_(new int[num]{}) {}

  float* d() const { return d_.get(); }
  float* p() const { return p_.get(); }
  float* w() const { return w_.get(); }
  int* fid() const { return fid_.get(); }

  void Reset(size_t num) {
    d_.reset(new float[num * 3]{});
    p_.reset(new float[num * 3]{});
    w_.reset(new float[num]{});
    fid_.reset(new int[num]{});
    num_ = num;
  }

  static void CopyData(SimulationData& dst, size_t dst_idx, SimulationData& src, size_t src_idx, size_t num) {
    std::memcpy(dst.d_.get() + dst_idx * 3, src.d_.get() + src_idx * 3, num * 3 * sizeof(float));
    std::memcpy(dst.p_.get() + dst_idx * 3, src.p_.get() + src_idx * 3, num * 3 * sizeof(float));
    std::memcpy(dst.w_.get() + dst_idx * 1, src.w_.get() + src_idx * 1, num * 1 * sizeof(float));
    std::memcpy(dst.fid_.get() + dst_idx * 1, src.fid_.get() + src_idx * 1, num * 1 * sizeof(int));
  }
};

using SimConfigPtrS = std::shared_ptr<SimulationConfig>;
using SimConfigPtrU = std::unique_ptr<SimulationConfig>;
using SimDataPtrS = std::shared_ptr<SimulationData>;
using SimDataPtrU = std::unique_ptr<SimulationData>;

template <class T>
class Queue;

template <class T>
using QueuePtrU = std::unique_ptr<Queue<T>>;
template <class T>
using QueuePtrS = std::shared_ptr<Queue<T>>;

class Simulator {
 public:
  enum State {
    kIdle,
    kRunning,
  };

  Simulator(QueuePtrS<SimConfigPtrS> config_queue, QueuePtrS<SimDataPtrS> data_queue);

  void operator()();

  void Stop();  // Stop running works, and set idle

 private:
  QueuePtrS<SimConfigPtrS> config_queue_;
  QueuePtrS<SimDataPtrS> data_queue_;
};

}  // namespace v3

class Simulator {
 public:
  explicit Simulator(ProjectContextPtr context);
  Simulator(const Simulator& other) = delete;
  ~Simulator() = default;

  Simulator& operator=(const Simulator& other) = delete;

  void SetCurrentWavelengthIndex(int index);
  void SetThreadingPool(ThreadingPoolPtr threading_pool);
  void Run();
  const SimulationData& GetSimulationRayData();

#ifdef FOR_TEST
  void PrintRayInfo();  // For debug
#endif

 private:
  struct EntryRayData {
    EntryRayData();
    EntryRayData(const EntryRayData& other) = delete;
    ~EntryRayData();

    EntryRayData& operator=(const EntryRayData& other) = delete;

    void Clear();
    void Allocate(size_t ray_number);

    float* ray_dir;
    RaySegment** ray_seg;
    size_t ray_num;
    size_t buf_size;
  };


  struct BufferData {
   public:
    BufferData();
    BufferData(const BufferData& other) = delete;
    ~BufferData();

    BufferData& operator=(const BufferData& other) = delete;

    void Clear();
    void Allocate(size_t ray_number);
#ifdef FOR_TEST
    void Print();
#endif

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
  void TraceRays(const CrystalContext* crystal_ctx, AbstractRayPathFilter* filter);
  void PrepareMultiScatterRays(float prob);
  void StoreRaySegments(const CrystalContext* crystal_ctx, AbstractRayPathFilter* filter);
  void RefreshBuffer();

  static constexpr int kBufferSizeFactor = 4;

  ProjectContextPtr context_;
  ThreadingPoolPtr threading_pool_;

  SimulationData simulation_ray_data_;

  int current_wavelength_index_;

  size_t total_ray_num_;
  size_t active_ray_num_;
  size_t buffer_size_;

  BufferData buffer_;
  EntryRayData entry_ray_data_;
  size_t entry_ray_offset_;
};

}  // namespace icehalo

#endif  // SRC_CORE_SIMULATION_H_
