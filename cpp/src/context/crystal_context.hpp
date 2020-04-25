#ifndef SRC_CONTEXT_CRYSTAL_CONTEXT_H_
#define SRC_CONTEXT_CRYSTAL_CONTEXT_H_

#include <memory>
#include <unordered_map>

#include "core/core_def.hpp"
#include "core/crystal.hpp"
#include "io/serialize.hpp"


namespace icehalo {

class CrystalContext : public IJsonizable {
 public:
  CrystalContext(const CrystalContext& other) = delete;

  ShortIdType GetId() const;
  const Crystal* GetCrystal() const;
  AxisDistribution GetAxisDistribution() const;

  int RandomSampleFace(const float* ray_dir, float* prob_buf = nullptr) const;

  void SaveToJson(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator) override;
  void LoadFromJson(const rapidjson::Value& root) override;

  static CrystalContextPtrU CreateDefault();

 private:
  CrystalContext();

  void ParseHexPrism(const rapidjson::Value& c);
  void ParseHexPyramid(const rapidjson::Value& c);
  void ParseHexPyramidStackHalf(const rapidjson::Value& c);
  void ParseCubicPyramid(const rapidjson::Value& c);
  void ParseIrregularHexPrism(const rapidjson::Value& c);
  void ParseIrregularHexPyramid(const rapidjson::Value& c);
  void ParseCustomCrystal(const rapidjson::Value& c);

  void SaveHexPrismParam(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator);
  void SaveHexPyramidH3Param(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator);
  void SaveHexPyramidI2H3Param(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator);
  void SaveHexPyramidI4H3Param(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator);
  void SaveHexPyramidA2H3Param(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator);
  void SaveHexPyramidStackHalfParam(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator);
  void SaveIrregularHexPrismParam(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator);
  void SaveIrregularHexPyramidParam(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator);
  void SaveCubicPyramidParam(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator);
  void SaveCustomCrystalParam(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator);

  ShortIdType id_;
  CrystalPtrU crystal_;
  AxisDistribution axis_;

  int idx_param_[4];
  float a_param_[2];
  float h_param_[3];
  float d_param_[6];
  std::string file_param_;
};


}  // namespace icehalo


#endif  // SRC_CONTEXT_CRYSTAL_CONTEXT_H_
