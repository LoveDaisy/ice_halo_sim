#include "context/multi_scatter_context.h"

#include <algorithm>

#include "rapidjson/pointer.h"
#include "util/threadingpool.h"


namespace icehalo {

using rapidjson::Pointer;

MultiScatterContext::MultiScatterContext(float prob) : prob_(std::max(std::min(prob, 1.0f), 0.0f)) {}


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


void MultiScatterContext::SaveToJson(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator) {
  Pointer("/probability").Set(root, prob_, allocator);

  Pointer("/crystal/0").Create(root, allocator);
  Pointer("/ray_path_filter/0").Create(root, allocator);
  Pointer("/population/0").Create(root, allocator);
  for (const auto& c : crystal_infos_) {
    Pointer("/crystal/-").Set(root, c.crystal_id, allocator);
    Pointer("/ray_path_filter/-").Set(root, c.filter_id, allocator);
    Pointer("/population/-").Set(root, c.population, allocator);
  }
}


void MultiScatterContext::LoadFromJson(const rapidjson::Value& root) {
  auto p = Pointer("/probability").Get(root);
  if (p == nullptr || !p->IsNumber()) {
    throw std::invalid_argument("<multi_scatter.probability> cannot recognize!");
  }
  auto prob = static_cast<float>(p->GetDouble());
  if (prob < 0) {
    throw std::invalid_argument("<multi_scatter.probability> is invalid!");
  }
  prob_ = prob;

  ClearCrystalInfo();
  std::vector<int> tmp_crystal_id;
  p = Pointer("/crystal").Get(root);
  if (p == nullptr || !p->IsArray()) {
    throw std::invalid_argument("<multi_scatter.crystal> cannot recognize!");
  }
  for (auto& pc : p->GetArray()) {
    if (!pc.IsUint()) {
      throw std::invalid_argument("<multi_scatter.crystal> cannot recognize!");
    } else {
      tmp_crystal_id.emplace_back(pc.GetInt());
    }
  }

  std::vector<float> tmp_population;
  p = Pointer("/population").Get(root);
  if (p == nullptr || !p->IsArray()) {
    throw std::invalid_argument("<multi_scatter.population> cannot recognize!");
  }
  for (auto& pp : p->GetArray()) {
    if (!pp.IsNumber()) {
      throw std::invalid_argument("<multi_scatter.population> cannot recognize!");
    } else {
      tmp_population.emplace_back(static_cast<float>(pp.GetDouble()));
    }
  }

  std::vector<int> tmp_filter_id;
  p = Pointer("/ray_path_filter").Get(root);
  if (p == nullptr || !p->IsArray()) {
    throw std::invalid_argument("<multi_scatter.ray_path_filter> cannot recognize!");
  }
  for (auto& pf : p->GetArray()) {
    if (!pf.IsUint()) {
      throw std::invalid_argument("<multi_scatter.ray_path_filter> cannot recognize!");
    } else {
      tmp_filter_id.emplace_back(pf.GetInt());
    }
  }

  for (size_t i = 0; i < tmp_crystal_id.size(); i++) {
    crystal_infos_.emplace_back(CrystalInfo{ tmp_crystal_id[i], tmp_filter_id[i], tmp_population[i] });
  }

  NormalizeCrystalPopulation();
}

}  // namespace icehalo
