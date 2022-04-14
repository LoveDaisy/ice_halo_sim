#ifndef SRC_PROCESS_PROTOCOL_H_
#define SRC_PROCESS_PROTOCOL_H_

#include <memory>

#include "context/crystal_config.hpp"
#include "context/light_config.hpp"
#include "core/core_def.hpp"

namespace icehalo {
namespace v3 {

// ========== Producer & consumer protocol ==========
// ---------- 1. For producer ----------

struct SimConfig {
  // ----- Some inner struct -----
  struct SimCrystalInfo {
    CrystalConfig crystal_;
    float proportion_;
  };

  struct SimMsParam {
    float ms_prob_;
    std::vector<SimCrystalInfo> crystal_info_;
  };

  // ----- Data member -----
  int ray_num_;
  int max_hits_;
  LightSourceConfig light_source_;
  std::vector<SimMsParam> ms_param_;
};

using SimConfigPtrS = std::shared_ptr<SimConfig>;
using SimConfigPtrU = std::unique_ptr<SimConfig>;


// ---------- 2. For consumer ----------

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

struct RaySeg {
  float d_[3];
  float p_[3];
  float w_;
  int fid_;
  RaypathHashHelper rp_;
};


struct SimBasicData {
  SimBasicData();
  SimBasicData(size_t capacity);

  void Reset(size_t capacity);
  bool Empty() const;

  // ----- Data -----
  float curr_wl_;
  size_t size_;
  size_t capacity_;
  std::unique_ptr<RaySeg[]> rays_;
};

using SimBasicDataPtrS = std::shared_ptr<SimBasicData>;
using SimBasicDataPtrU = std::unique_ptr<SimBasicData>;

struct SimBackTracingData {};

}  // namespace v3
}  // namespace icehalo

#endif  // SRC_PROCESS_PROTOCOL_H_
