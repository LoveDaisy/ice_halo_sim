#include "config/light_config.hpp"

#include <nlohmann/json.hpp>
#include <variant>
#include <vector>

#include "util/logger.hpp"

namespace lumice {

static void LightParamToJson(nlohmann::json& j, const SunParam& p) {
  j["type"] = "sun";
  j["altitude"] = p.altitude_;
  j["azimuth"] = p.azimuth_;
  j["diameter"] = p.diameter_;
}

struct SpectrumToJson {
  nlohmann::json& j_;

  void operator()(const std::vector<WlParam>& wl_params) {
    nlohmann::json arr = nlohmann::json::array();
    for (const auto& w : wl_params) {
      arr.push_back({ { "wavelength", w.wl_ }, { "weight", w.weight_ } });
    }
    j_["spectrum"] = std::move(arr);
  }

  void operator()(IlluminantType type) {
    nlohmann::json type_json = type;  // uses NLOHMANN_JSON_SERIALIZE_ENUM
    j_["spectrum"] = type_json;
  }
};

void to_json(nlohmann::json& j, const LightSourceConfig& l) {
  j["id"] = l.id_;
  std::visit(SpectrumToJson{ j }, l.spectrum_);
  LightParamToJson(j, l.param_);
}

void from_json(const nlohmann::json& j, LightSourceConfig& l) {
  j.at("id").get_to(l.id_);

  const auto& j_spectrum = j.at("spectrum");
  if (j_spectrum.is_string()) {
    l.spectrum_ = j_spectrum.get<IlluminantType>();
  } else if (j_spectrum.is_array()) {
    std::vector<WlParam> wl_params;
    for (const auto& item : j_spectrum) {
      WlParam w{};
      item.at("wavelength").get_to(w.wl_);
      item.at("weight").get_to(w.weight_);
      wl_params.emplace_back(w);
    }
    l.spectrum_ = std::move(wl_params);
  } else {
    LOG_ERROR("Invalid spectrum format: expected string or array");
  }

  const auto& j_type = j.at("type");
  if (j_type == "sun") {
    SunParam p{};
    j.at("altitude").get_to(p.altitude_);
    if (j.contains("azimuth")) {
      j.at("azimuth").get_to(p.azimuth_);
    }
    if (j.contains("diameter")) {
      j.at("diameter").get_to(p.diameter_);
    }
    l.param_ = p;
  } else {
    LOG_ERROR("Unknown light source type: {}", j_type.get<std::string>());
  }
}

}  // namespace lumice
