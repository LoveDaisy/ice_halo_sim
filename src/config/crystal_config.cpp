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


// Convert Miller index (i1, i4) to wedge angle in degrees. Returns 28.0 (default) if i1 == 0.
static float MillerToAlpha(int i1, int i4) {
  constexpr float kSqrt3_2 = 0.866025403784f;
  constexpr float kIceCrystalC = 1.629f;
  constexpr float kRadToDeg = 57.2957795131f;
  if (i1 == 0) {
    return 28.0f;
  }
  return std::atan(kSqrt3_2 * i4 / i1 / kIceCrystalC) * kRadToDeg;
}

// ========== PyramidCrystalParam ==========
void to_json(nlohmann::json& j, const PyramidCrystalParam& p) {
  j["prism_h"] = p.h_prs_;
  j["upper_h"] = p.h_pyr_u_;
  j["lower_h"] = p.h_pyr_l_;
  j["upper_wedge_angle"] = p.wedge_angle_u_;
  j["lower_wedge_angle"] = p.wedge_angle_l_;
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

  // Wedge angle: prefer "upper_wedge_angle", fallback to "upper_indices" conversion
  if (j.contains("upper_wedge_angle")) {
    p.wedge_angle_u_ = j.at("upper_wedge_angle").get<float>();
  } else if (j.contains("upper_indices") && j.at("upper_indices").is_array() && j.at("upper_indices").size() == 3) {
    auto& ui = j.at("upper_indices");
    p.wedge_angle_u_ = MillerToAlpha(ui[0].get<int>(), ui[2].get<int>());
  }
  if (j.contains("lower_wedge_angle")) {
    p.wedge_angle_l_ = j.at("lower_wedge_angle").get<float>();
  } else if (j.contains("lower_indices") && j.at("lower_indices").is_array() && j.at("lower_indices").size() == 3) {
    auto& li = j.at("lower_indices");
    p.wedge_angle_l_ = MillerToAlpha(li[0].get<int>(), li[2].get<int>());
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
