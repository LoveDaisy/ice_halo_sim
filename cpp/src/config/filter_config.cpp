#include "config/filter_config.hpp"

#include <variant>
#include <vector>

#include "core/def.hpp"
#include "include/log.hpp"
#include "json.hpp"

namespace icehalo {
namespace v3 {

// =============== FilterConfig ===============
struct SimpleFilterParamToJson {
  nlohmann::json& j_;

  void operator()(const NoneFilterParam& /* p */) { j_["type"] = "none"; }

  void operator()(const RaypathFilterParam& p) {
    j_["type"] = "raypath";
    j_["raypath"] = p.raypath_;
  }

  void operator()(const EntryExitFilterParam& p) {
    j_["type"] = "entry_exit";
    j_["entry"] = p.entry_;
    j_["exit"] = p.exit_;
  }

  void operator()(const DirectionFilterParam& p) {
    j_["type"] = "direction";
    j_["az"] = p.lon_;
    j_["el"] = p.lat_;
    j_["radii"] = p.radii_;
  }

  void operator()(const CrystalFilterParam& p) {
    j_["type"] = "crystal";
    j_["crystal_id"] = p.crystal_id_;
  }
};

struct FilterParamToJson {
  nlohmann::json& j_;

  void operator()(const SimpleFilterParam& p) { std::visit(SimpleFilterParamToJson{ j_ }, p); }

  void operator()(const ComplexFilterParam& p) {
    j_["type"] = "complex";
    for (const auto& c : p.filters_) {
      if (c.size() == 1) {
        j_["composition"].emplace_back(c[0].first);
      } else {
        std::vector<IdType> tmp_id;
        for (const auto& cc : c) {
          tmp_id.emplace_back(cc.first);
        }
        j_["composition"].emplace_back(tmp_id);
      }
    }
  }
};

void to_json(nlohmann::json& j, const FilterConfig& f) {
  j["id"] = f.id_;

  switch (f.action_) {
    case FilterConfig::kFilterIn:
      j["action"] = "filter_in";
      break;
    case FilterConfig::kFilterOut:
      j["action"] = "filter_out";
      break;
  }

  std::string sym;
  if (f.symmetry_ | FilterConfig::kSymP) {
    sym += "P";
  }
  if (f.symmetry_ | FilterConfig::kSymB) {
    sym += "B";
  }
  if (f.symmetry_ | FilterConfig::kSymD) {
    sym += "D";
  }
  j["symmetry"] = sym;

  std::visit(FilterParamToJson{ j }, f.param_);
}

void from_json(const nlohmann::json& j, FilterConfig& f) {
  j.at("id").get_to(f.id_);

  const auto& type = j.at("type");
  if (type == "none") {
    f.param_ = NoneFilterParam();
  } else if (type == "raypath") {
    RaypathFilterParam p{};
    j.at("raypath").get_to(p.raypath_);
    f.param_ = p;
  } else if (type == "entry_exit") {
    EntryExitFilterParam p{};
    j.at("entry").get_to(p.entry_);
    j.at("exit").get_to(p.exit_);
    f.param_ = p;
  } else if (type == "direction") {
    DirectionFilterParam p{};
    j.at("az").get_to(p.lon_);
    j.at("el").get_to(p.lat_);
    j.at("radii").get_to(p.radii_);
    f.param_ = p;
  } else if (type == "crystal") {
    CrystalFilterParam p{};
    j.at("crystal_id").get_to(p.crystal_id_);
    f.param_ = p;
  } else if (type == "complex") {
    // NOTE: It is **INCOMPLETED**. Other data will be set in ConfigManager.
    ComplexFilterParam p{};
    f.param_ = p;
  }

  f.symmetry_ = FilterConfig::kSymNone;
  if (j.contains("symmetry")) {
    // Symmetry
    auto sym = j.at("symmetry").get<std::string>();
    for (auto c : sym) {
      if (c == 'P') {
        f.symmetry_ |= FilterConfig::kSymP;
      }
      if (c == 'B') {
        f.symmetry_ |= FilterConfig::kSymB;
      }
      if (c == 'D') {
        f.symmetry_ |= FilterConfig::kSymD;
      }
    }
  }

  f.action_ = FilterConfig::kFilterIn;
  if (j.contains("action")) {
    // Action
    const auto& j_action = j.at("action");
    if (j_action == "filter_in") {
      f.action_ = FilterConfig::kFilterIn;
    }
    if (j_action == "filter_out") {
      f.action_ = FilterConfig::kFilterOut;
    }
  }
}

}  // namespace v3
}  // namespace icehalo
