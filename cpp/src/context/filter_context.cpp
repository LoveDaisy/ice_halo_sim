#include "context/filter_context.h"

#include <algorithm>
#include <limits>

#include "rapidjson/document.h"
#include "rapidjson/pointer.h"

namespace icehalo {

using rapidjson::Pointer;


RayPathFilterContext::RayPathFilterContext() : id_(0), filter_{} {}


int RayPathFilterContext::GetId() const {
  return id_;
}


AbstractRayPathFilter* RayPathFilterContext::GetFilter() const {
  return filter_.get();
}


RayPathFilterContextPtrU RayPathFilterContext::CreateDefault() {
  RayPathFilterContextPtrU ctx{ new RayPathFilterContext };
  return ctx;
}


void RayPathFilterContext::SaveToJson(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator) {
  Pointer("/id").Set(root, id_, allocator);
  filter_->SaveToJson(root, allocator);
}


void RayPathFilterContext::LoadFromJson(const rapidjson::Value& root) {
  auto p = Pointer("/type").Get(root);
  if (p == nullptr || !p->IsString()) {
    throw std::invalid_argument("<type> cannot recognize!");
  }

  if (root["type"] == "none") {
    filter_.reset(new NoneRayPathFilter);
  } else if (root["type"] == "specific") {
    filter_.reset(new SpecificRayPathFilter);
  } else if (root["type"] == "general") {
    filter_.reset(new GeneralRayPathFilter);
  } else {
    throw std::invalid_argument("<type> cannot recognize!");
  }
  filter_->LoadFromJson(root);

  p = Pointer("/id").Get(root);
  if (p == nullptr || !p->IsInt()) {
    throw std::invalid_argument("<id> cannot recognize!");
  }
  id_ = p->GetInt();
}

}  // namespace icehalo
