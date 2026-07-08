#include "config/filter_config.hpp"

#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

#include "core/def.hpp"

namespace lumice {

// =============== SimpleFilterParam JSON ===============
namespace {

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

}  // namespace

void to_json(nlohmann::json& j, const SimpleFilterParam& p) {
  std::visit(SimpleFilterParamToJson{ j }, p);
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void from_json(const nlohmann::json& j, SimpleFilterParam& p) {
  // Missing `type` defaults to match-all (NoneFilterParam). Used by
  // Design-2 RaypathColorRef where the only required fields on the wire are
  // {layer, crystal} and predicate defaults to match-all whole-crystal.
  if (!j.contains("type")) {
    p = NoneFilterParam{};
    return;
  }
  const auto& type = j.at("type");
  if (type == "none") {
    p = NoneFilterParam{};
  } else if (type == "raypath") {
    RaypathFilterParam rp{};
    j.at("raypath").get_to(rp.raypath_);
    p = rp;
  } else if (type == "entry_exit") {
    EntryExitFilterParam ee{};
    if (j.contains("entry") && !j.at("entry").is_null()) {
      ee.entry_ = j.at("entry").get<IdType>();
    }
    if (j.contains("exit") && !j.at("exit").is_null()) {
      ee.exit_ = j.at("exit").get<IdType>();
    }
    if (j.contains("min_len") && !j.at("min_len").is_null()) {
      ee.min_len_ = j.at("min_len").get<size_t>();
    }
    if (j.contains("max_len") && !j.at("max_len").is_null()) {
      ee.max_len_ = j.at("max_len").get<size_t>();
    }
    if (ee.min_len_ < 1) {
      throw std::runtime_error("entry_exit filter: min_len must be >= 1, got " + std::to_string(ee.min_len_));
    }
    if (ee.max_len_.has_value()) {
      if (*ee.max_len_ < ee.min_len_) {
        throw std::runtime_error("entry_exit filter: max_len (" + std::to_string(*ee.max_len_) +
                                 ") must be >= min_len (" + std::to_string(ee.min_len_) + ")");
      }
      if (*ee.max_len_ > kMaxHits) {
        throw std::runtime_error("entry_exit filter: max_len (" + std::to_string(*ee.max_len_) +
                                 ") exceeds kMaxHits (" + std::to_string(kMaxHits) + ")");
      }
    }
    p = ee;
  } else if (type == "direction") {
    DirectionFilterParam d{};
    j.at("az").get_to(d.lon_);
    j.at("el").get_to(d.lat_);
    j.at("radii").get_to(d.radii_);
    p = d;
  } else if (type == "crystal") {
    CrystalFilterParam c{};
    j.at("crystal_id").get_to(c.crystal_id_);
    p = c;
  } else {
    throw std::runtime_error("SimpleFilterParam: unknown type " + type.dump());
  }
}

// =============== FilterConfig ===============

namespace {

struct FilterParamToJson {
  nlohmann::json& j_;

  void operator()(const SimpleFilterParam& p) { to_json(j_, p); }

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

}  // namespace

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

void from_json(const nlohmann::json& j, FilterConfig& f) {
  j.at("id").get_to(f.id_);

  const auto& type = j.at("type");
  if (type == "complex") {
    // NOTE: It is **INCOMPLETED**. Other data will be set in ConfigManager.
    f.param_ = ComplexFilterParam{};
  } else {
    // All simple types share a single home in SimpleFilterParam's from_json.
    SimpleFilterParam sp{};
    from_json(j, sp);
    f.param_ = sp;
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
