#include "protocol/crystal_config.hpp"

#include <string>

#include "core/math.hpp"
#include "json.hpp"
#include "util/log.hpp"

namespace icehalo {
namespace v3 {

// convert to & from json object
// ========== PrismCrystalParam ==========
void to_json(nlohmann::json& j, const PrismCrystalParam& p) {
  j = nlohmann::json{ { "shape", { "height", p.h_ } } };
  j.at("shape")["distance"] = p.d_;
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
  }
  if (j.contains("face_distance")) {
    if (!j.at("face_distance").is_array()) {
      LOG_ERROR("Cannot recognize face_distance of crystal!");
    }

    int i = 0;
    for (const auto& j_d : j.at("face_distance")) {
      j_d.get_to(p.d_[i]);
      i++;
      if (i >= 6) {
        break;
      }
    }
  }
}


// ========== PyramidCrystalParam ==========
void to_json(nlohmann::json& j, const PyramidCrystalParam& p) {
  j = nlohmann::json{ { "shape", { "height", { p.h_pyr_u_, p.h_prs_, p.h_pyr_l_ } } } };
  j.at("shape")["distance"] = p.d_;
}

void from_json(const nlohmann::json& j, PyramidCrystalParam& p) {
  ;
}


// ========== CrystalConfig ==========
void to_json(nlohmann::json& j, const CrystalConfig& c) {
  ;
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

	j.at("axis").get_to(c.axis_);
}

}  // namespace v3
}  // namespace icehalo
