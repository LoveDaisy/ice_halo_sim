#ifndef CONFIG_FILTER_CONFIG_H_
#define CONFIG_FILTER_CONFIG_H_

#include <variant>
#include <vector>

#include "core/def.hpp"
#include "json.hpp"

namespace icehalo {
namespace v3 {

struct NoneFilterParam {};

struct RaypathFilterParam {
  std::vector<IdType> raypath_;
};

struct EntryExitFilterParam {
  IdType entry_;
  IdType exit_;
};

struct DirectionFilterParam {
  float lon_;    // Degree
  float lat_;    // Degree
  float radii_;  // Degree
};

struct CrystalFilterParam {
  IdType crystal_id_;
};

using SimpleFilterParam =
    std::variant<NoneFilterParam, RaypathFilterParam, EntryExitFilterParam, DirectionFilterParam, CrystalFilterParam>;

struct ComplexFilterParam {
  std::vector<std::vector<std::pair<IdType, SimpleFilterParam>>> filters_;  // (f + ...) * (f + ...) * ...
};

using FilterParam = std::variant<SimpleFilterParam, ComplexFilterParam>;

struct FilterConfig {
  static constexpr uint8_t kSymNone = 0;
  static constexpr uint8_t kSymP = 1;
  static constexpr uint8_t kSymB = 2;
  static constexpr uint8_t kSymD = 4;

  enum Action {
    kFilterIn,
    kFilterOut,
  };

  IdType id_;
  uint8_t symmetry_;
  Action action_;
  FilterParam param_;
};

// convert to/from json object
void to_json(nlohmann::json& j, const FilterConfig& f);
void from_json(const nlohmann::json& j, FilterConfig& f);

}  // namespace v3
}  // namespace icehalo

#endif  // CONFIG_FILTER_CONFIG_H_