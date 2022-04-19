#include "protocol/light_config.hpp"

#include <cstddef>
#include <variant>
#include <vector>

#include "io/json_util.hpp"
#include "util/log.hpp"

namespace icehalo {
namespace v3 {

void to_json(nlohmann::json& j, const LightSourceConfig& l) {
  j["id"] = l.id;
  std::vector<float> wl;
  std::vector<float> weight;
  for (const auto& w : l.wl_param_) {
    wl.emplace_back(w.wl_);
    weight.emplace_back(w.weight_);
  }
  j["wavelength"] = wl;
  j["wl_weight"] = weight;

  if (std::holds_alternative<SunParam>(l.param_)) {
    const auto& p = std::get<SunParam>(l.param_);
    j["type"] = "sun";
    j["altitude"] = p.altitude_;
    j["azimuth"] = p.azimuth_;
    j["diameter"] = p.diameter_;
  } else if (std::holds_alternative<StreetLightParam>(l.param_)) {
    const auto& p = std::get<StreetLightParam>(l.param_);
    j["type"] = "streetlight";
    j["height"] = p.height_;
    j["azimuth"] = p.azimuth_;
    j["distance"] = p.distace_;
    j["diametre"] = p.diameter_;
  }
}

void from_json(const nlohmann::json& j, LightSourceConfig& l) {
  j.at("id").get_to(l.id);

  size_t wi = 0;
  const auto& j_weight = j.at("wl_weight");
  for (const auto& j_wl : j.at("wavelength")) {
    if (wi >= j_weight.size()) {
      LOG_WARNING("wl_weight has more items than wavelength. Ignore extra items.");
      break;
    }
    WlParam w{};
    j_wl.get_to(w.wl_);
    j_weight[wi].get_to(w.weight_);
    l.wl_param_.emplace_back(w);
  }

  const auto& j_type = j.at("type");
  if (j_type == "sun") {
    SunParam p{};
    j.at("altitude").get_to(p.altitude_);
    JSON_CHECK_AND_UPDATE_SIMPLE_VALUE(j, "azimuth", p.azimuth_)
    JSON_CHECK_AND_UPDATE_SIMPLE_VALUE(j, "diameter", p.diameter_)
  } else if (j_type == "streetlight") {
    StreetLightParam p{};
    j.at("azimuth").get_to(p.azimuth_);
    j.at("distance").get_to(p.distace_);
    j.at("height").get_to(p.height_);
    JSON_CHECK_AND_UPDATE_SIMPLE_VALUE(j, "diameter", p.diameter_)
  } else {
    LOG_ERROR("Unknown light source type!");
  }
}

}  // namespace v3
}  // namespace icehalo
