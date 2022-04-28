#ifndef SRC_PROCESS_PROTOCOL_H_
#define SRC_PROCESS_PROTOCOL_H_

#include <cstddef>
#include <memory>

#include "core/crystal.hpp"
#include "core/def.hpp"
#include "core/raypath.hpp"

namespace icehalo {
namespace v3 {

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
  RayBuffer rays_;
  std::vector<Crystal> crystals_;
};

using SimDataPtrS = std::shared_ptr<SimData>;
using SimDataPtrU = std::unique_ptr<SimData>;

struct SimBackTracingData {};

}  // namespace v3
}  // namespace icehalo

#endif  // SRC_PROCESS_PROTOCOL_H_
