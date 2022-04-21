#include "protocol/filter_config.hpp"

#include <variant>
#include <vector>

#include "core/core_def.hpp"
#include "util/log.hpp"

namespace icehalo {
namespace v3 {

// =============== FilterConfig ===============
void to_json(nlohmann::json& j, const FilterConfig& f) {
  j["id"] = f.id;

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

  if (std::holds_alternative<NoneFilterParam>(f.param_)) {
    j["type"] = "none";
  } else if (std::holds_alternative<RaypathFilterParam>(f.param_)) {
    const auto& p = std::get<RaypathFilterParam>(f.param_);
    j["type"] = "raypath";
    j["raypath"] = p.raypath_;
  } else if (std::holds_alternative<EntryExitFilterParam>(f.param_)) {
    const auto& p = std::get<EntryExitFilterParam>(f.param_);
    j["type"] = "entry_exit";
    j["entry"] = p.entry_;
    j["exit"] = p.exit_;
  } else if (std::holds_alternative<DirectionFilterParam>(f.param_)) {
    const auto& p = std::get<DirectionFilterParam>(f.param_);
    j["type"] = "direction";
    j["az"] = p.lon_;
    j["el"] = p.lat_;
    j["radii"] = p.radii_;
  } else if (std::holds_alternative<CrystalFilterParam>(f.param_)) {
    const auto& p = std::get<CrystalFilterParam>(f.param_);
    j["type"] = "crystal";
    j["crystal_id"] = p.crystal_id_;
  } else if (std::holds_alternative<ComplexFilterParam>(f.param_)) {
    const auto& p = std::get<ComplexFilterParam>(f.param_);
    j["type"] = "complex";
    for (const auto& c : p.filters_) {
      if (c.size() == 1) {
        j["composition"].emplace_back(c[0]);
      } else {
        j["composition"].emplace_back(c);
      }
    }
  }
}

void from_json(const nlohmann::json& j, FilterConfig& f) {
  j.at("id").get_to(f.id);

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
    ComplexFilterParam p{};
    const auto& cmp = j.at("composition");
    for (const auto& c : cmp) {
      std::vector<IdType> f;
      for (const auto& cc : c) {
        f.emplace_back(cc.get<IdType>());
      }
      p.filters_.emplace_back(f);
    }
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
