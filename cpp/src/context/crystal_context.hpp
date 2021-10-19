#ifndef SRC_CONTEXT_CRYSTAL_CONTEXT_H_
#define SRC_CONTEXT_CRYSTAL_CONTEXT_H_

#include <memory>
#include <unordered_map>

#include "core/core_def.hpp"
#include "core/crystal.hpp"
#include "io/json_util.hpp"
#include "io/serialize.hpp"

namespace icehalo {

class CrystalContext {
 public:
  CrystalContext(const CrystalContext& other) = delete;

  ShortIdType GetId() const;
  const Crystal* GetCrystal() const;
  AxisDistribution GetAxisDistribution() const;

  int RandomSampleFace(const float* ray_dir, float* prob_buf = nullptr) const;

  void PrintCrystal() const;

  static CrystalContextPtrU CreateDefault();

  friend void to_json(nlohmann::json& obj, const CrystalContext& ctx);

  friend void from_json(const nlohmann::json& obj, CrystalContext& ctx);

 private:
  CrystalContext();

  void ParseHexPrism(const nlohmann::json& obj);
  void ParseHexPyramid(const nlohmann::json& obj);
  void ParseHexPyramidStackHalf(const nlohmann::json& obj);
  void ParseCubicPyramid(const nlohmann::json& obj);
  void ParseIrregularHexPrism(const nlohmann::json& obj);
  void ParseIrregularHexPyramid(const nlohmann::json& obj);
  void ParseCustomCrystal(const nlohmann::json& obj);

  void SaveHexPrismParam(nlohmann::json& obj) const;
  void SaveHexPyramidH3Param(nlohmann::json& obj) const;
  void SaveHexPyramidI2H3Param(nlohmann::json& obj) const;
  void SaveHexPyramidI4H3Param(nlohmann::json& obj) const;
  void SaveHexPyramidA2H3Param(nlohmann::json& obj) const;
  void SaveHexPyramidStackHalfParam(nlohmann::json& obj) const;
  void SaveIrregularHexPrismParam(nlohmann::json& obj) const;
  void SaveIrregularHexPyramidParam(nlohmann::json& obj) const;
  void SaveCubicPyramidParam(nlohmann::json& obj) const;
  void SaveCustomCrystalParam(nlohmann::json& obj) const;

  ShortIdType id_;
  CrystalPtrU crystal_;
  AxisDistribution axis_;

  int idx_param_[4];
  float a_param_[2];
  float h_param_[3];
  float d_param_[6];
  std::string file_param_;
};


void to_json(nlohmann::json& obj, const AxisDistribution& dist);

void from_json(const nlohmann::json& obj, AxisDistribution& dist);

NLOHMANN_JSON_SERIALIZE_ENUM(DistributionType, {
                                                   { DistributionType::kUniform, "uniform" },
                                                   { DistributionType::kGaussian, "gauss" },
                                               })

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Distribution, type, mean, std)

}  // namespace icehalo


#endif  // SRC_CONTEXT_CRYSTAL_CONTEXT_H_
