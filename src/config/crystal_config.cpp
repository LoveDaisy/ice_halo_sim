#include "config/crystal_config.hpp"

#include <nlohmann/json.hpp>
#include <numeric>
#include <variant>

#include "core/math.hpp"
#include "util/logger.hpp"

namespace lumice {

// convert to & from json object
// ========== PrismCrystalParam ==========
void to_json(nlohmann::json& j, const PrismCrystalParam& p) {
  j["height"] = p.h_;
  j["face_distance"] = p.d_;
}

void from_json(const nlohmann::json& j, PrismCrystalParam& p) {
  if (j.contains("height")) {
    j.at("height").get_to(p.h_);
  }

  // Face distance: default mean=1.0 (1.0 = regular hexagon in FillHexCrystalCoef)
  for (auto& x : p.d_) {
    x.type = DistributionType::kNoRandom;
    x.mean = 1.0f;
  }
  if (j.contains("face_distance")) {
    size_t i = 0;
    for (const auto& elem : j.at("face_distance")) {
      if (i >= 6) {
        break;
      }
      elem.get_to(p.d_[i]);
      i++;
    }
  }
}


// ========== PyramidCrystalParam ==========
void to_json(nlohmann::json& j, const PyramidCrystalParam& p) {
  j["prism_h"] = p.h_prs_;
  j["upper_h"] = p.h_pyr_u_;
  j["lower_h"] = p.h_pyr_l_;
  j["upper_indices"] = p.miller_indices_u_;
  j["lower_indices"] = p.miller_indices_l_;
  j["face_distance"] = p.d_;
}

void from_json(const nlohmann::json& j, PyramidCrystalParam& p) {
  // Heights
  j.at("prism_h").get_to(p.h_prs_);
  if (j.contains("upper_h")) {
    j.at("upper_h").get_to(p.h_pyr_u_);
  }
  if (j.contains("lower_h")) {
    j.at("lower_h").get_to(p.h_pyr_l_);
  }

  // Miller indices
  if (j.contains("upper_indices")) {
    j.at("upper_indices").get_to(p.miller_indices_u_);
  }
  if (j.contains("lower_indices")) {
    j.at("lower_indices").get_to(p.miller_indices_l_);
  }

  // Face distance: default mean=1.0 (1.0 = regular hexagon in FillHexCrystalCoef)
  for (auto& x : p.d_) {
    x.type = DistributionType::kNoRandom;
    x.mean = 1.0f;
  }
  if (j.contains("face_distance")) {
    size_t i = 0;
    for (const auto& elem : j.at("face_distance")) {
      if (i >= 6) {
        break;
      }
      elem.get_to(p.d_[i]);
      i++;
    }
  }

  // Normalize Miller indices by GCD so that e.g. [2,0,2] becomes [1,0,1]
  auto normalize_miller = [](int(&idx)[3]) {
    int g = std::gcd(std::abs(idx[0]), std::gcd(std::abs(idx[1]), std::abs(idx[2])));
    if (g > 1) {
      idx[0] /= g;
      idx[1] /= g;
      idx[2] /= g;
    }
  };
  normalize_miller(p.miller_indices_u_);
  normalize_miller(p.miller_indices_l_);
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
  }
}

}  // namespace lumice
