#include "context/multi_scatter_context.h"

#include <algorithm>

#include "core/render.h"
#include "util/threadingpool.h"


namespace icehalo {

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


void MultiScatterContext::AddCrystalInfo(const CrystalContext* crystal_ctx,  // crystal context
                                         AbstractRayPathFilter* filter,      // ray path filter for this crystal
                                         float population) {                 // population of this crystal
  crystal_infos_.emplace_back(CrystalInfo{ crystal_ctx, filter, population });
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

}  // namespace icehalo
