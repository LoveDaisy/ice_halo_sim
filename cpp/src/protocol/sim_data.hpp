#ifndef SRC_PROCESS_PROTOCOL_H_
#define SRC_PROCESS_PROTOCOL_H_

#include <cstddef>
#include <memory>

#include "core/def.hpp"
#include "core/optics.hpp"

namespace icehalo {
namespace v3 {

struct RaypathRecorder {
  RaypathRecorder& operator<<(IdType fn);
  IdType operator[](size_t idx);

  size_t size_ = 0;
  std::byte recorder_[kRpRcdBytes]{};

  /*
   * Use 6 bits (kRpIdBits) for a face number ID in a raypath.
   *
   * byte  |<-- byte 0  -->|<-- byte 1  -->|<-- byte 2  -->|
   *        <-- low bits                      high bits -->
   * bit    0 1 2 3 4 5 6 7 8 9 a b c d e f 0 1 2 3 4 5 6 7
   * fn    |<- fn 0  ->|<- fn 1  ->|<- fn 2  ->|<- fn 3  ->|
   *
   * For fn0, we can simply use fn0 = recorder_[0] & 0x3f
   * and for fn1, it will be fn1 = ((recorder_[0] & 0xc0) >> 6) | ((recorder_[1] & 0x0f) << 2)
   */
};

struct RaySeg {
  float d_[3];
  float p_[3];  // Generally it is for end point, **NOT** for start point.
  float w_;
  int fid_;
  IdType prev_ray_id_;
  IdType crystal_id_;
  RaypathRecorder rp_;
};

struct RayBuffer {
  RayBuffer();
  RayBuffer(size_t capacity);

  RaySeg& operator[](size_t idx) const;

  void Reset(size_t capacity);
  bool Empty() const;
  void EmplaceBack(RaySeg r);
  void EmplaceBack(const RayBuffer& buffer, size_t start = 0, size_t len = kInfSize);

  RaySeg* rays() const;
  RaySeg* begin() const;
  RaySeg* end() const;

  size_t capacity_;
  size_t size_;
  std::unique_ptr<RaySeg[]> rays_;
};


struct SimData {
  SimData();
  SimData(size_t capacity);
  SimData(const SimData& other);
  SimData(SimData&& other);
  ~SimData() = default;

  SimData& operator=(const SimData& other);
  SimData& operator=(SimData&& other);

  // ----- Data -----
  float curr_wl_;
  RayBuffer ray_buffer_;
  std::vector<Crystal> crystals_;
};

using SimDataPtrS = std::shared_ptr<SimData>;
using SimDataPtrU = std::unique_ptr<SimData>;

struct SimBackTracingData {};

}  // namespace v3
}  // namespace icehalo

#endif  // SRC_PROCESS_PROTOCOL_H_
