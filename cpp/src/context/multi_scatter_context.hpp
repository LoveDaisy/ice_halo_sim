#ifndef SRC_CONTEXT_MULTI_SCATTER_CONTEXT_H_
#define SRC_CONTEXT_MULTI_SCATTER_CONTEXT_H_

#include <memory>

#include "core/core_def.hpp"
#include "io/serialize.hpp"
#include "json.hpp"

namespace icehalo {

class MultiScatterContext {
 public:
  struct CrystalInfo {
    ShortIdType crystal_id;
    ShortIdType filter_id;
    float population;

    CrystalInfo(ShortIdType crystal_id, ShortIdType filter_id, float pop)
        : crystal_id(crystal_id), filter_id(filter_id), population(pop){};
  };

  float GetProbability() const;
  bool SetProbability(float p);

  const std::vector<CrystalInfo>& GetCrystalInfo() const;

  static MultiScatterContextPtrU CreateDefault();

  friend void to_json(nlohmann::json& obj, const MultiScatterContext& ctx);
  friend void from_json(const nlohmann::json& obj, MultiScatterContext& ctx);

 private:
  explicit MultiScatterContext(float prob = 1.0f);

  void ClearCrystalInfo();
  void NormalizeCrystalPopulation();

  std::vector<CrystalInfo> crystal_infos_;  // crystal, population, filter
  float prob_;
};

}  // namespace icehalo


#endif  // SRC_CONTEXT_MULTI_SCATTER_CONTEXT_H_
