#ifndef SRC_CONTEXT_MULTI_SCATTER_CONTEXT_H_
#define SRC_CONTEXT_MULTI_SCATTER_CONTEXT_H_

#include <memory>

#include "io/serialize.h"
#include "rapidjson/document.h"


namespace icehalo {

class CrystalContext;
class AbstractRayPathFilter;

class MultiScatterContext {
 public:
  struct CrystalInfo {
    const CrystalContext* crystal_ctx;
    AbstractRayPathFilter* filter;
    float population;

    CrystalInfo(const CrystalContext* crystal_ctx, AbstractRayPathFilter* filter, float pop)
        : crystal_ctx(crystal_ctx), filter(filter), population(pop){};
  };

  explicit MultiScatterContext(float prob = 1.0f);

  float GetProbability() const;
  bool SetProbability(float p);

  const std::vector<CrystalInfo>& GetCrystalInfo() const;
  void ClearCrystalInfo();
  void AddCrystalInfo(const CrystalContext* crystal_ctx, AbstractRayPathFilter* filter, float population);
  void NormalizeCrystalPopulation();

 private:
  std::vector<CrystalInfo> crystal_infos_;  // crystal, population, filter
  float prob_;
};

}  // namespace icehalo


#endif  // SRC_CONTEXT_MULTI_SCATTER_CONTEXT_H_
