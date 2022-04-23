#ifndef SRC_PROCESS_PROTOCOL_H_
#define SRC_PROCESS_PROTOCOL_H_

#include <cstddef>
#include <memory>

#include "core/core_def.hpp"
#include "core/optics.hpp"

namespace icehalo {
namespace v3 {

class RaypathHashHelper {
  // Use SDBM algorithm. See http://www.cse.yorku.ca/~oz/hash.html for detail.
 public:
  RaypathHashHelper& operator<<(IdType c);
  size_t GetHash() const;

 private:
  static constexpr unsigned kMagic = 65599u;
  size_t hash_ = 0;
};


struct RaypathHash {
  size_t operator()(const std::vector<IdType>& rp);
};

struct RayBuffer {
  RayBuffer();
  RayBuffer(size_t capacity);

  RaySeg& operator[](size_t idx) const;

  void Reset(size_t capacity);
  bool Empty() const;
  RaySeg* rays() const;

  size_t capacity_;
  size_t size_;
  std::unique_ptr<RaySeg[]> rays_;
};


struct SimData {
  SimData();
  SimData(size_t capacity);
  SimData(const SimData& other);
  SimData(SimData&& other);
  ~SimData();

  SimData& operator=(const SimData& other);
  SimData& operator=(SimData&& other);

  // ----- Data -----
  float curr_wl_;
  RayBuffer ray_buffer_;
};

using SimBasicDataPtrS = std::shared_ptr<SimData>;
using SimBasicDataPtrU = std::unique_ptr<SimData>;

struct SimBackTracingData {};

}  // namespace v3
}  // namespace icehalo

#endif  // SRC_PROCESS_PROTOCOL_H_
