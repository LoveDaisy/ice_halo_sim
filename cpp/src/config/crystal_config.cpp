#include "config/crystal_config.hpp"

#include <string>
#include <variant>

#include "core/math.hpp"
#include "io/json_util.hpp"
#include "json.hpp"
#include "util/log.hpp"

namespace icehalo {
namespace v3 {

// convert to & from json object
// ========== PrismCrystalParam ==========
void to_json(nlohmann::json& j, const PrismCrystalParam& p) {
  j["height"] = p.h_;
  j["distance"] = p.d_;
}

void from_json(const nlohmann::json& j, PrismCrystalParam& p) {
  // ----- height -----
  p.h_.type = DistributionType::kNoRandom;
  p.h_.mean = 1.0f;
  if (j.contains("height")) {
    j.at("height").get_to(p.h_);
  }

  // ----- face distance ------
  for (auto& x : p.d_) {
    x.type = DistributionType::kNoRandom;
    x.mean = 1.0f;
    x.std = 0.0f;
  }
  JSON_CHECK_AND_UPDATE_ARRAY_VALUE(j, "face_distance", p.d_, 6)
  for (auto& x : p.d_) {
    x.mean *= math::kSqrt3_4;
    x.std *= math::kSqrt3_4;
  }
}


// ========== PyramidCrystalParam ==========
void to_json(nlohmann::json& j, const PyramidCrystalParam& p) {
  j["prism_h"] = p.h_prs_;
  j["upper_h"] = p.h_pyr_u_;
  j["lower_h"] = p.h_pyr_l_;
  j["upper_indices"] = p.miller_indices_u_;
  j["lower_indices"] = p.miller_indices_l_;
  j["distance"] = p.d_;
}

void from_json(const nlohmann::json& j, PyramidCrystalParam& p) {
  // Heights
  j.at("prism_h").get_to(p.h_prs_);

  p.h_pyr_u_.type = DistributionType::kNoRandom;
  p.h_pyr_u_.mean = 0.0f;
  p.h_pyr_l_.type = DistributionType::kNoRandom;
  p.h_pyr_l_.mean = 0.0f;
  JSON_CHECK_AND_UPDATE_SIMPLE_VALUE(j, "upper_h", p.h_pyr_u_)
  JSON_CHECK_AND_UPDATE_SIMPLE_VALUE(j, "lower_h", p.h_pyr_l_)

  // Miller indices
  p.miller_indices_u_[0] = 1;
  p.miller_indices_u_[1] = 0;
  p.miller_indices_u_[2] = 1;
  JSON_CHECK_AND_UPDATE_ARRAY_VALUE(j, "upper_indices", p.miller_indices_u_, 3)

  p.miller_indices_l_[0] = 1;
  p.miller_indices_l_[1] = 0;
  p.miller_indices_l_[2] = 1;
  JSON_CHECK_AND_UPDATE_ARRAY_VALUE(j, "lower_indices", p.miller_indices_l_, 3)

  // Face distance
  for (auto& x : p.d_) {
    x.type = DistributionType::kNoRandom;
    x.mean = 1.0f;
    x.std = 0.0f;
  }
  JSON_CHECK_AND_UPDATE_ARRAY_VALUE(j, "face_distance", p.d_, 6)
  for (auto& x : p.d_) {
    x.mean *= math::kSqrt3_4;
    x.std *= math::kSqrt3_4;
  }
}


// ========== CrystalConfig ==========
void to_json(nlohmann::json& j, const CrystalConfig& c) {
  j["id"] = c.id_;
  j["axis"] = c.axis_;
  std::visit([&j](auto&& p) { j["shape"] = p; }, c.param_);
}

void from_json(const nlohmann::json& j, CrystalConfig& c) {
  j.at("id").get_to(c.id_);

  const auto& j_type = j.at("type");
  if (j_type == "prism") {
    c.param_ = j.at("shape").get<PrismCrystalParam>();
  } else if (j_type == "pyramid") {
    c.param_ = j.at("shape").get<PyramidCrystalParam>();
  } else {
    LOG_ERROR("Unknown crystal type!");
  }

  if (j.contains("axis")) {
    j.at("axis").get_to(c.axis_);
  } else {
    c.axis_.latitude_dist.type = DistributionType::kNoRandom;
    c.axis_.latitude_dist.mean = 90.0f;
  }
}

}  // namespace v3
}  // namespace icehalo
