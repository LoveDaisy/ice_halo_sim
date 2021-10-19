#include "context/multi_scatter_context.hpp"

#include <algorithm>


namespace icehalo {

MultiScatterContext::MultiScatterContext(float prob) : prob_(std::max(std::min(prob, 1.0f), 0.0f)) {}


MultiScatterContextPtrU MultiScatterContext::CreateDefault() {
  MultiScatterContextPtrU ctx{ new MultiScatterContext };
  return ctx;
}


float MultiScatterContext::GetProbability() const {
  return prob_;
}


bool MultiScatterContext::SetProbability(float p) {
  if (p < 0 || p > 1) {
    return false;
  } else {
    prob_ = p;
    return true;
  }
}


const std::vector<MultiScatterContext::CrystalInfo>& MultiScatterContext::GetCrystalInfo() const {
  return crystal_infos_;
}


void MultiScatterContext::ClearCrystalInfo() {
  crystal_infos_.clear();
}


void MultiScatterContext::NormalizeCrystalPopulation() {
  float sum = 0;
  for (const auto& c : crystal_infos_) {
    sum += c.population;
  }
  for (auto& c : crystal_infos_) {
    c.population /= sum;
  }
}


void to_json(nlohmann::json& obj, const MultiScatterContext& ctx) {
  obj["probability"] = ctx.GetProbability();
  for (const auto& c : ctx.GetCrystalInfo()) {
    obj["crystal"].emplace_back(c.crystal_id);
    obj["ray_path_filter"].emplace_back(c.filter_id);
    obj["population"].emplace_back(c.population);
  }
}


void from_json(const nlohmann::json& obj, MultiScatterContext& ctx) {
  auto prob = obj.at("probability").get<float>();
  ctx.SetProbability(prob);

  if (!obj.at("crystal").is_array() || !obj.at("ray_path_filter").is_array() || !obj.at("population").is_array() ||
      obj.at("crystal").size() != obj.at("ray_path_filter").size() ||
      obj.at("crystal").size() != obj.at("population").size()) {
    throw nlohmann::detail::other_error::create(
        -1, "crystal ray_path_filter and population should be arrays with same length!", obj);
  }
  ctx.ClearCrystalInfo();
  for (size_t i = 0; i < obj.at("crystal").size(); i++) {
    auto crystal_id = obj.at("crystal")[i].get<ShortIdType>();
    auto filter_id = obj.at("ray_path_filter")[i].get<ShortIdType>();
    auto pop = obj.at("population")[i].get<float>();
    ctx.crystal_infos_.emplace_back(MultiScatterContext::CrystalInfo(crystal_id, filter_id, pop));
  }
  ctx.NormalizeCrystalPopulation();
}

}  // namespace icehalo
