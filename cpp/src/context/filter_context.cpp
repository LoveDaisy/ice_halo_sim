#include "context/filter_context.hpp"

#include <algorithm>

#include "core/filter.hpp"

namespace icehalo {

RayPathFilterContext::RayPathFilterContext() : id_(kInvalidId) {}


ShortIdType RayPathFilterContext::GetId() const {
  return id_;
}


AbstractRayPathFilter* RayPathFilterContext::GetFilter() const {
  return filter_.get();
}


RayPathFilterContextPtrU RayPathFilterContext::CreateDefault() {
  RayPathFilterContextPtrU ctx{ new RayPathFilterContext };
  return ctx;
}


void to_json(nlohmann::json& obj, const RayPathFilterContext& ctx) {
  obj["id"] = ctx.id_;
  to_json(obj, *ctx.filter_);
}


void from_json(const nlohmann::json& obj, RayPathFilterContext& ctx) {
  obj.at("id").get_to(ctx.id_);
  auto type = obj.at("type").get<std::string>();
  if (type == "none") {
    ctx.filter_.reset(new NoneRayPathFilter);
  } else if (type == "specific") {
    ctx.filter_.reset(new SpecificRayPathFilter);
  } else if (type == "general") {
    ctx.filter_.reset(new GeneralRayPathFilter);
  } else {
    throw nlohmann::detail::other_error::create(-1, "filter type cannot recognize!", obj);
  }
  from_json(obj, *ctx.filter_);
}

}  // namespace icehalo
