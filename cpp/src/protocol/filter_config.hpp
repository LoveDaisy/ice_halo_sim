#ifndef PROTOCOL_FILTER_CONFIG_H_
#define PROTOCOL_FILTER_CONFIG_H_

#include <variant>
#include <vector>

#include "core/core_def.hpp"

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

struct ComplexFilterParam {
  std::vector<std::vector<IdType>> filters_;  // (f + ...) * (f + ...) * ...
};

using FilterParam = std::variant<NoneFilterParam, RaypathFilterParam, EntryExitFilterParam, DirectionFilterParam,
                                 CrystalFilterParam, ComplexFilterParam>;

struct FilterConfig {
  enum Symmetry {
    kNone,
    kPrismatic,
    kBasal,
    kDirectional,
  };

  enum Action {
    kFilterIn,
    kFilterOut,
  };

  IdType id;
  Symmetry symmetry_;
  Action action_;
  FilterParam param_;
};

}  // namespace v3
}  // namespace icehalo

#endif  // PROTOCOL_FILTER_CONFIG_H_
