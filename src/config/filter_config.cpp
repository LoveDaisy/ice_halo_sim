#include "config/filter_config.hpp"

#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

#include "core/def.hpp"

namespace lumice {

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
    if (p.entry_.has_value()) {
      j_["entry"] = *p.entry_;
    }
    if (p.exit_.has_value()) {
      j_["exit"] = *p.exit_;
    }
    if (p.min_len_ > 1) {
      j_["min_len"] = p.min_len_;
    }
    if (p.max_len_.has_value()) {
      j_["max_len"] = *p.max_len_;
    }
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
  if (f.symmetry_ & FilterConfig::kSymP) {
    sym += "P";
  }
  if (f.symmetry_ & FilterConfig::kSymB) {
    sym += "B";
  }
  if (f.symmetry_ & FilterConfig::kSymD) {
    sym += "D";
  }
  j["symmetry"] = sym;

  std::visit(FilterParamToJson{ j }, f.param_);
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
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
    if (j.contains("entry") && !j.at("entry").is_null()) {
      p.entry_ = j.at("entry").get<IdType>();
    }
    if (j.contains("exit") && !j.at("exit").is_null()) {
      p.exit_ = j.at("exit").get<IdType>();
    }
    if (j.contains("min_len") && !j.at("min_len").is_null()) {
      p.min_len_ = j.at("min_len").get<size_t>();
    }
    if (j.contains("max_len") && !j.at("max_len").is_null()) {
      p.max_len_ = j.at("max_len").get<size_t>();
    }
    // Validation per plan §8 AC: min_len >= 1; if max_len set, min_len <= max_len <= kMaxHits.
    if (p.min_len_ < 1) {
      throw std::runtime_error("entry_exit filter: min_len must be >= 1, got " + std::to_string(p.min_len_));
    }
    if (p.max_len_.has_value()) {
      if (*p.max_len_ < p.min_len_) {
        throw std::runtime_error("entry_exit filter: max_len (" + std::to_string(*p.max_len_) +
                                 ") must be >= min_len (" + std::to_string(p.min_len_) + ")");
      }
      if (*p.max_len_ > kMaxHits) {
        throw std::runtime_error("entry_exit filter: max_len (" + std::to_string(*p.max_len_) + ") exceeds kMaxHits (" +
                                 std::to_string(kMaxHits) + ")");
      }
    }
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

}  // namespace lumice
