#ifndef SRC_CONTEXT_FILTER_CONTEXT_H_
#define SRC_CONTEXT_FILTER_CONTEXT_H_

#include <memory>
#include <unordered_map>
#include <unordered_set>

#include "core/crystal.h"
#include "core/filter.h"
#include "core/optics.h"
#include "io/serialize.h"


namespace icehalo {

class RayPathFilterContext;
using RayPathFilterContextPtrU = std::unique_ptr<RayPathFilterContext>;

class RayPathFilterContext : public IJsonizable {
 public:
  int GetId() const;
  AbstractRayPathFilter* GetFilter() const;

  void SaveToJson(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator) override;
  void LoadFromJson(const rapidjson::Value& root) override;

  static RayPathFilterContextPtrU CreateDefault();

 private:
  RayPathFilterContext();

  using FilterParser = std::function<RayPathFilterPtrU(const rapidjson::Value&)>;

  int id_;
  RayPathFilterPtrU filter_;
};

}  // namespace icehalo


#endif  // SRC_CONTEXT_FILTER_CONTEXT_H_
