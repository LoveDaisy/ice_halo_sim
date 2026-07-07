#include "config/raypath_color_config.hpp"

#include <nlohmann/json.hpp>

namespace lumice {

void to_json(nlohmann::json& j, const RaypathColorRef& r) {
  j = nlohmann::json::object();
  j["layer"] = r.layer_;
  j["crystal"] = r.crystal_;
  if (r.has_filter_) {
    j["filter"] = r.filter_;
  }
  if (r.has_summand_) {
    j["summand"] = r.summand_;
  }
}

void from_json(const nlohmann::json& j, RaypathColorRef& r) {
  j.at("layer").get_to(r.layer_);
  j.at("crystal").get_to(r.crystal_);
  r.has_filter_ = j.contains("filter");
  if (r.has_filter_) {
    j.at("filter").get_to(r.filter_);
  } else {
    r.filter_ = kInvalidId;
  }
  r.has_summand_ = j.contains("summand");
  if (r.has_summand_) {
    j.at("summand").get_to(r.summand_);
  } else {
    r.summand_ = 0;
  }
}

void to_json(nlohmann::json& j, const ColorClassConfig& c) {
  j = nlohmann::json::object();
  j["color"] = { c.color_[0], c.color_[1], c.color_[2] };
  if (c.combine_ != "any") {
    j["combine"] = c.combine_;
  }
  if (!c.visible_) {
    j["visible"] = c.visible_;
  }
  if (c.solo_) {
    j["solo"] = c.solo_;
  }
  j["match"] = nlohmann::json::array();
  for (const auto& r : c.match_) {
    j["match"].emplace_back(r);
  }
}

void from_json(const nlohmann::json& j, ColorClassConfig& c) {
  const auto& jc = j.at("color");
  c.color_[0] = jc.at(0).get<float>();
  c.color_[1] = jc.at(1).get<float>();
  c.color_[2] = jc.at(2).get<float>();
  c.combine_ = j.value("combine", std::string("any"));
  c.visible_ = j.value("visible", true);
  c.solo_ = j.value("solo", false);
  c.match_.clear();
  for (const auto& jr : j.at("match")) {
    c.match_.emplace_back(jr.get<RaypathColorRef>());
  }
}

// Wire format:
//   - default mode ("dominant"): bare array of ColorClassConfig — minimal.
//   - non-default mode: { "mode": ..., "classes": [ ... ] } object form.
// from_json accepts both shapes so either version parses.
void to_json(nlohmann::json& j, const RaypathColorConfig& c) {
  if (c.mode_ == "dominant") {
    j = nlohmann::json::array();
    for (const auto& cls : c.classes_) {
      j.emplace_back(cls);
    }
    return;
  }
  j = nlohmann::json::object();
  j["mode"] = c.mode_;
  j["classes"] = nlohmann::json::array();
  for (const auto& cls : c.classes_) {
    j["classes"].emplace_back(cls);
  }
}

void from_json(const nlohmann::json& j, RaypathColorConfig& c) {
  c.classes_.clear();
  c.mode_ = "dominant";
  if (j.is_object()) {
    c.mode_ = j.value("mode", std::string("dominant"));
    if (j.contains("classes")) {
      for (const auto& jc : j.at("classes")) {
        c.classes_.emplace_back(jc.get<ColorClassConfig>());
      }
    }
    return;
  }
  for (const auto& jc : j) {
    c.classes_.emplace_back(jc.get<ColorClassConfig>());
  }
}

}  // namespace lumice
