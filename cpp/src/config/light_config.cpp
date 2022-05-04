#include "config/light_config.hpp"

#include <cstddef>
#include <variant>
#include <vector>

#include "include/log.hpp"
#include "io/json_util.hpp"
#include "json.hpp"

namespace icehalo {
namespace v3 {

struct LightParamToJson {
  nlohmann::json& j_;

  void operator()(const SunParam& p) {
    j_["type"] = "sun";
    j_["altitude"] = p.altitude_;
    j_["azimuth"] = p.azimuth_;
    j_["diameter"] = p.diameter_;
  }

  void operator()(const StreetLightParam& p) {
    j_["type"] = "streetlight";
    j_["height"] = p.height_;
    j_["azimuth"] = p.azimuth_;
    j_["distance"] = p.distace_;
    j_["diametre"] = p.diameter_;
  }
};

void to_json(nlohmann::json& j, const LightSourceConfig& l) {
  j["id"] = l.id_;
  std::vector<float> wl;
  std::vector<float> weight;
  for (const auto& w : l.wl_param_) {
    wl.emplace_back(w.wl_);
    weight.emplace_back(w.weight_);
  }
  j["wavelength"] = wl;
  j["wl_weight"] = weight;

  std::visit(LightParamToJson{ j }, l.param_);
}

void from_json(const nlohmann::json& j, LightSourceConfig& l) {
  j.at("id").get_to(l.id_);

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
    l.param_ = p;
  } else if (j_type == "streetlight") {
    StreetLightParam p{};
    j.at("azimuth").get_to(p.azimuth_);
    j.at("distance").get_to(p.distace_);
    j.at("height").get_to(p.height_);
    JSON_CHECK_AND_UPDATE_SIMPLE_VALUE(j, "diameter", p.diameter_)
    l.param_ = p;
  } else {
    LOG_ERROR("Unknown light source type!");
  }
}

}  // namespace v3
}  // namespace icehalo
