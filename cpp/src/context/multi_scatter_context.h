#ifndef SRC_CONTEXT_MULTI_SCATTER_CONTEXT_H_
#define SRC_CONTEXT_MULTI_SCATTER_CONTEXT_H_

#include <memory>

#include "io/serialize.h"
#include "rapidjson/document.h"


namespace icehalo {

class MultiScatterContext : public IJsonizable {
 public:
  struct CrystalInfo {
    int crystal_id;
    int filter_id;
    float population;

    CrystalInfo(int crystal_id, int filter_id, float pop)
        : crystal_id(crystal_id), filter_id(filter_id), population(pop){};
  };

  explicit MultiScatterContext(float prob = 1.0f);

  float GetProbability() const;
  bool SetProbability(float p);

  const std::vector<CrystalInfo>& GetCrystalInfo() const;

  void SaveToJson(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator) override;
  void LoadFromJson(const rapidjson::Value& root) override;

 private:
  void ClearCrystalInfo();
  void NormalizeCrystalPopulation();

  std::vector<CrystalInfo> crystal_infos_;  // crystal, population, filter
  float prob_;
};

}  // namespace icehalo


#endif  // SRC_CONTEXT_MULTI_SCATTER_CONTEXT_H_
