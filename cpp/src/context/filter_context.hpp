#ifndef SRC_CONTEXT_FILTER_CONTEXT_H_
#define SRC_CONTEXT_FILTER_CONTEXT_H_

#include <memory>
#include <unordered_map>
#include <unordered_set>

#include "core/crystal.hpp"
#include "core/filter.hpp"
#include "core/optics.hpp"
#include "io/json_util.hpp"
#include "io/serialize.hpp"

namespace icehalo {

class RayPathFilterContext;
using RayPathFilterContextPtrU = std::unique_ptr<RayPathFilterContext>;

class RayPathFilterContext {
 public:
  ShortIdType GetId() const;
  AbstractRayPathFilter* GetFilter() const;

  static RayPathFilterContextPtrU CreateDefault();

  friend void to_json(nlohmann::json& obj, const RayPathFilterContext& ctx);
  friend void from_json(const nlohmann::json& obj, RayPathFilterContext& ctx);

 private:
  RayPathFilterContext();

  ShortIdType id_;
  RayPathFilterPtrU filter_;
};

}  // namespace icehalo


#endif  // SRC_CONTEXT_FILTER_CONTEXT_H_
