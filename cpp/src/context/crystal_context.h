#ifndef SRC_CONTEXT_CRYSTAL_CONTEXT_H_
#define SRC_CONTEXT_CRYSTAL_CONTEXT_H_

#include <memory>

#include "core/crystal.h"
#include "io/serialize.h"


namespace icehalo {

class CrystalContext;
using CrystalContextPtrU = std::unique_ptr<CrystalContext>;

class CrystalContext : public IJsonizable {
 public:
  CrystalContext();
  CrystalContext(int id, AxisDistribution axis, CrystalPtrU g);
  CrystalContext(const CrystalContext& other) = delete;

  int GetId() const;
  const Crystal* GetCrystal() const;
  AxisDistribution GetAxisDistribution() const;

  int RandomSampleFace(const float* ray_dir) const;

  void SaveToJson(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator) override;
  void LoadFromJson(rapidjson::Value& root) override;

 private:
  int id_;
  CrystalPtrU crystal_;
  AxisDistribution axis_;
};


}  // namespace icehalo


#endif  // SRC_CONTEXT_CRYSTAL_CONTEXT_H_
