// Tests for src/config/raypath_color_config.hpp — JSON round-trip of the
// task-339.2 color-class schema (supersedes 336.1 flat form) and its optional
// wiring through ConfigManager. The backward-compat guarantee (missing
// top-level "raypath_color" key → empty classes_) is unchanged from 336.1.

#include <gtest/gtest.h>

#include <fstream>
#include <nlohmann/json.hpp>
#include <string>

#include "config/config_manager.hpp"
#include "config/raypath_color_config.hpp"

extern std::string config_file_name;

namespace {

using lumice::ColorClassConfig;
using lumice::ConfigManager;
using lumice::RaypathColorConfig;
using lumice::RaypathColorRef;

RaypathColorRef MakeRef(uint16_t layer, uint16_t crystal, bool has_filter, uint16_t filter, bool has_summand,
                        uint16_t summand) {
  RaypathColorRef r{};
  r.layer_ = layer;
  r.crystal_ = crystal;
  r.has_filter_ = has_filter;
  r.filter_ = filter;
  r.has_summand_ = has_summand;
  r.summand_ = summand;
  return r;
}

ColorClassConfig MakeClass(float r, float g, float b, std::vector<RaypathColorRef> match) {
  ColorClassConfig c{};
  c.color_[0] = r;
  c.color_[1] = g;
  c.color_[2] = b;
  c.match_ = std::move(match);
  return c;
}

}  // namespace

// ---- RaypathColorRef JSON round-trip: full quadruple ----

TEST(RaypathColorRef, JsonRoundTripFullQuadruple) {
  auto original = MakeRef(1, 2, true, 3, true, 4);
  nlohmann::json j = original;
  EXPECT_EQ(j.at("layer").get<uint16_t>(), 1u);
  EXPECT_EQ(j.at("crystal").get<uint16_t>(), 2u);
  EXPECT_EQ(j.at("filter").get<uint16_t>(), 3u);
  EXPECT_EQ(j.at("summand").get<uint16_t>(), 4u);

  auto restored = j.get<RaypathColorRef>();
  EXPECT_EQ(restored.layer_, 1);
  EXPECT_EQ(restored.crystal_, 2);
  EXPECT_TRUE(restored.has_filter_);
  EXPECT_EQ(restored.filter_, 3);
  EXPECT_TRUE(restored.has_summand_);
  EXPECT_EQ(restored.summand_, 4);
}

TEST(RaypathColorRef, JsonOmitsAbsentFilterAndSummand) {
  auto original = MakeRef(0, 5, false, 0, false, 0);
  nlohmann::json j = original;
  EXPECT_TRUE(j.contains("layer"));
  EXPECT_TRUE(j.contains("crystal"));
  EXPECT_FALSE(j.contains("filter"));
  EXPECT_FALSE(j.contains("summand"));

  auto restored = j.get<RaypathColorRef>();
  EXPECT_FALSE(restored.has_filter_);
  EXPECT_FALSE(restored.has_summand_);
}

TEST(RaypathColorRef, JsonFilterPresentSummandOmitted) {
  auto original = MakeRef(0, 1, true, 2, false, 0);
  nlohmann::json j = original;
  EXPECT_TRUE(j.contains("filter"));
  EXPECT_FALSE(j.contains("summand"));
  auto restored = j.get<RaypathColorRef>();
  EXPECT_TRUE(restored.has_filter_);
  EXPECT_EQ(restored.filter_, 2);
  EXPECT_FALSE(restored.has_summand_);
}

// ---- ColorClassConfig JSON round-trip ----

TEST(ColorClassConfig, JsonRoundTripDefaults) {
  auto original = MakeClass(1.0f, 0.5f, 0.25f, { MakeRef(0, 1, true, 1, true, 0) });
  nlohmann::json j = original;
  // Defaults omitted:
  EXPECT_FALSE(j.contains("combine"));
  EXPECT_FALSE(j.contains("visible"));
  EXPECT_FALSE(j.contains("solo"));
  ASSERT_TRUE(j.at("match").is_array());
  ASSERT_EQ(j.at("match").size(), 1u);

  auto restored = j.get<ColorClassConfig>();
  EXPECT_FLOAT_EQ(restored.color_[0], 1.0f);
  EXPECT_FLOAT_EQ(restored.color_[1], 0.5f);
  EXPECT_FLOAT_EQ(restored.color_[2], 0.25f);
  EXPECT_EQ(restored.combine_, "any");
  EXPECT_TRUE(restored.visible_);
  EXPECT_FALSE(restored.solo_);
  ASSERT_EQ(restored.match_.size(), 1u);
}

TEST(ColorClassConfig, JsonEmitsNonDefaultFields) {
  auto cls = MakeClass(0.0f, 0.0f, 1.0f, { MakeRef(0, 1, false, 0, false, 0) });
  cls.combine_ = "all";
  cls.visible_ = false;
  cls.solo_ = true;
  nlohmann::json j = cls;
  EXPECT_EQ(j.at("combine").get<std::string>(), "all");
  EXPECT_EQ(j.at("visible").get<bool>(), false);
  EXPECT_EQ(j.at("solo").get<bool>(), true);
}

// ---- RaypathColorConfig JSON round-trip: default mode = bare array ----

TEST(RaypathColorConfig, JsonRoundTripDefaultModeBareArray) {
  RaypathColorConfig cfg;
  cfg.classes_.push_back(MakeClass(1.0f, 0.0f, 0.0f, { MakeRef(0, 1, true, 1, false, 0) }));
  cfg.classes_.push_back(MakeClass(0.0f, 1.0f, 0.0f, { MakeRef(0, 1, true, 4, true, 0) }));

  nlohmann::json j = cfg;
  ASSERT_TRUE(j.is_array());
  ASSERT_EQ(j.size(), 2u);

  auto restored = j.get<RaypathColorConfig>();
  ASSERT_EQ(restored.classes_.size(), 2u);
  EXPECT_EQ(restored.mode_, "dominant");
  EXPECT_FLOAT_EQ(restored.classes_[0].color_[0], 1.0f);
  EXPECT_FLOAT_EQ(restored.classes_[1].color_[1], 1.0f);
}

TEST(RaypathColorConfig, JsonRoundTripNonDefaultModeObjectForm) {
  RaypathColorConfig cfg;
  cfg.mode_ = "additive";
  cfg.classes_.push_back(MakeClass(0.0f, 0.0f, 1.0f, { MakeRef(0, 1, true, 4, true, 1) }));

  nlohmann::json j = cfg;
  ASSERT_TRUE(j.is_object());
  EXPECT_EQ(j.at("mode").get<std::string>(), "additive");
  ASSERT_TRUE(j.at("classes").is_array());
  ASSERT_EQ(j.at("classes").size(), 1u);

  auto restored = j.get<RaypathColorConfig>();
  EXPECT_EQ(restored.mode_, "additive");
  ASSERT_EQ(restored.classes_.size(), 1u);
  EXPECT_FLOAT_EQ(restored.classes_[0].color_[2], 1.0f);
}

TEST(RaypathColorConfig, EmptyJsonRoundTrip) {
  RaypathColorConfig cfg;
  nlohmann::json j = cfg;
  ASSERT_TRUE(j.is_array());
  EXPECT_EQ(j.size(), 0u);
  auto restored = j.get<RaypathColorConfig>();
  EXPECT_TRUE(restored.classes_.empty());
  EXPECT_EQ(restored.mode_, "dominant");
}

// ---- ConfigManager wiring: backward compatibility ----

TEST(ConfigManagerRaypathColor, LegacyFixtureWithoutKeyParsesToEmpty) {
  std::ifstream f(config_file_name);
  ASSERT_TRUE(f.is_open()) << "cannot open v3 fixture: " << config_file_name;
  nlohmann::json j;
  f >> j;
  EXPECT_FALSE(j.contains("raypath_color")) << "fixture must NOT declare raypath_color (backward-compat anchor)";

  auto manager = j.get<ConfigManager>();
  EXPECT_TRUE(manager.raypath_color_.classes_.empty())
      << "missing top-level raypath_color key must yield an empty RaypathColorConfig";
}

TEST(ConfigManagerRaypathColor, ToJsonOmitsKeyWhenEmpty) {
  std::ifstream f(config_file_name);
  ASSERT_TRUE(f.is_open());
  nlohmann::json j_in;
  f >> j_in;
  auto manager = j_in.get<ConfigManager>();
  ASSERT_TRUE(manager.raypath_color_.classes_.empty());

  nlohmann::json j_out = manager;
  EXPECT_FALSE(j_out.contains("raypath_color"))
      << "to_json must omit the key when classes_ is empty (matches crystal/filter/render convention)";
}

// ---- ConfigManager wiring: end-to-end round-trip with color-class schema ----

TEST(ConfigManagerRaypathColor, ParsesColorSectionWhenPresent) {
  std::ifstream f(config_file_name);
  ASSERT_TRUE(f.is_open());
  nlohmann::json j_in;
  f >> j_in;

  j_in["raypath_color"] = nlohmann::json::array({
      { { "color", { 1.0f, 0.0f, 0.0f } }, { "match", { { { "layer", 0 }, { "crystal", 1 }, { "filter", 1 } } } } },
      { { "color", { 0.0f, 1.0f, 0.0f } },
        { "match", { { { "layer", 0 }, { "crystal", 1 }, { "filter", 4 }, { "summand", 0 } } } } },
      { { "color", { 0.0f, 0.0f, 1.0f } },
        { "match", { { { "layer", 0 }, { "crystal", 1 }, { "filter", 4 }, { "summand", 1 } } } } },
  });

  auto manager = j_in.get<ConfigManager>();
  ASSERT_EQ(manager.raypath_color_.classes_.size(), 3u);
  EXPECT_FLOAT_EQ(manager.raypath_color_.classes_[0].color_[0], 1.0f);
  EXPECT_FLOAT_EQ(manager.raypath_color_.classes_[1].color_[1], 1.0f);
  EXPECT_FLOAT_EQ(manager.raypath_color_.classes_[2].color_[2], 1.0f);
  ASSERT_EQ(manager.raypath_color_.classes_[0].match_.size(), 1u);
  EXPECT_TRUE(manager.raypath_color_.classes_[0].match_[0].has_filter_);
  EXPECT_FALSE(manager.raypath_color_.classes_[0].match_[0].has_summand_);
}

TEST(ConfigManagerRaypathColor, ToJsonEmitsColorSectionWhenPresent) {
  ConfigManager m{};
  m.raypath_color_.classes_.push_back(MakeClass(0.25f, 0.5f, 0.75f, { MakeRef(0, 1, true, 1, false, 0) }));

  nlohmann::json j = m;
  ASSERT_TRUE(j.contains("raypath_color"));
  ASSERT_TRUE(j.at("raypath_color").is_array());
  ASSERT_EQ(j.at("raypath_color").size(), 1u);
  const auto& j_cls = j.at("raypath_color").at(0);
  EXPECT_FLOAT_EQ(j_cls.at("color").at(0).get<float>(), 0.25f);
  EXPECT_FLOAT_EQ(j_cls.at("color").at(1).get<float>(), 0.5f);
  EXPECT_FLOAT_EQ(j_cls.at("color").at(2).get<float>(), 0.75f);
  ASSERT_TRUE(j_cls.at("match").is_array());
  ASSERT_EQ(j_cls.at("match").size(), 1u);
  EXPECT_EQ(j_cls.at("match").at(0).at("crystal").get<uint16_t>(), 1u);
  EXPECT_EQ(j_cls.at("match").at(0).at("filter").get<uint16_t>(), 1u);
  EXPECT_FALSE(j_cls.at("match").at(0).contains("summand"));
}
