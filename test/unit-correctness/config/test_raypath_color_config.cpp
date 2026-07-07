// Tests for src/config/raypath_color_config.hpp — JSON round-trip of the
// per-raypath color schema (task-336.1 Step 1) and its optional wiring
// through ConfigManager (Step 2), including the backward-compat guarantee
// that a config without a "raypath_color" key parses to an empty entries_.

#include <gtest/gtest.h>

#include <fstream>
#include <nlohmann/json.hpp>
#include <string>

#include "config/config_manager.hpp"
#include "config/raypath_color_config.hpp"

extern std::string config_file_name;

namespace {

using lumice::ConfigManager;
using lumice::RaypathColorConfig;
using lumice::RaypathColorEntry;

RaypathColorEntry MakeEntry(uint16_t layer, uint16_t crystal_slot, uint16_t summand, float r, float g, float b) {
  RaypathColorEntry e{};
  e.layer_ = layer;
  e.crystal_id_ = crystal_slot;
  e.summand_idx_ = summand;
  e.color_[0] = r;
  e.color_[1] = g;
  e.color_[2] = b;
  return e;
}

}  // namespace

// ---- RaypathColorEntry JSON round-trip ----

TEST(RaypathColorEntry, JsonRoundTrip) {
  auto original = MakeEntry(1, 2, 3, 0.25f, 0.5f, 0.75f);
  nlohmann::json j = original;

  EXPECT_EQ(j.at("layer").get<uint16_t>(), 1u);
  EXPECT_EQ(j.at("crystal_slot").get<uint16_t>(), 2u);
  EXPECT_EQ(j.at("summand").get<uint16_t>(), 3u);
  ASSERT_TRUE(j.at("color").is_array());
  ASSERT_EQ(j.at("color").size(), 3u);

  auto restored = j.get<RaypathColorEntry>();
  EXPECT_EQ(restored.layer_, original.layer_);
  EXPECT_EQ(restored.crystal_id_, original.crystal_id_);
  EXPECT_EQ(restored.summand_idx_, original.summand_idx_);
  EXPECT_FLOAT_EQ(restored.color_[0], original.color_[0]);
  EXPECT_FLOAT_EQ(restored.color_[1], original.color_[1]);
  EXPECT_FLOAT_EQ(restored.color_[2], original.color_[2]);
}

// ---- RaypathColorConfig JSON round-trip: three-arcs plan §3.2 anchor ----

TEST(RaypathColorConfig, JsonRoundTripThreeArcs) {
  RaypathColorConfig cfg;
  cfg.entries_.push_back(MakeEntry(0, 0, 0, 1.0f, 0.0f, 0.0f));  // 晶体1 · 3-5 · 红
  cfg.entries_.push_back(MakeEntry(0, 1, 0, 0.0f, 1.0f, 0.0f));  // 晶体2 · 3 · 绿
  cfg.entries_.push_back(MakeEntry(0, 1, 1, 0.0f, 0.0f, 1.0f));  // 晶体2 · 3-5 · 蓝

  nlohmann::json j = cfg;
  ASSERT_TRUE(j.is_array());
  ASSERT_EQ(j.size(), 3u);

  auto restored = j.get<RaypathColorConfig>();
  ASSERT_EQ(restored.entries_.size(), 3u);
  EXPECT_EQ(restored.entries_[0].crystal_id_, 0);
  EXPECT_EQ(restored.entries_[1].crystal_id_, 1);
  EXPECT_EQ(restored.entries_[1].summand_idx_, 0);
  EXPECT_EQ(restored.entries_[2].summand_idx_, 1);
  EXPECT_FLOAT_EQ(restored.entries_[0].color_[0], 1.0f);
  EXPECT_FLOAT_EQ(restored.entries_[1].color_[1], 1.0f);
  EXPECT_FLOAT_EQ(restored.entries_[2].color_[2], 1.0f);
}

TEST(RaypathColorConfig, EmptyJsonRoundTrip) {
  RaypathColorConfig cfg;
  nlohmann::json j = cfg;
  ASSERT_TRUE(j.is_array());
  EXPECT_EQ(j.size(), 0u);
  auto restored = j.get<RaypathColorConfig>();
  EXPECT_TRUE(restored.entries_.empty());
}

// ---- ConfigManager wiring: backward compatibility (Step 2 AC-1) ----

TEST(ConfigManagerRaypathColor, LegacyFixtureWithoutKeyParsesToEmpty) {
  std::ifstream f(config_file_name);
  ASSERT_TRUE(f.is_open()) << "cannot open v3 fixture: " << config_file_name;
  nlohmann::json j;
  f >> j;
  EXPECT_FALSE(j.contains("raypath_color")) << "fixture must NOT declare raypath_color (backward-compat anchor)";

  auto manager = j.get<ConfigManager>();
  EXPECT_TRUE(manager.raypath_color_.entries_.empty())
      << "missing top-level raypath_color key must yield an empty RaypathColorConfig";
}

TEST(ConfigManagerRaypathColor, ToJsonOmitsKeyWhenEmpty) {
  std::ifstream f(config_file_name);
  ASSERT_TRUE(f.is_open());
  nlohmann::json j_in;
  f >> j_in;
  auto manager = j_in.get<ConfigManager>();
  ASSERT_TRUE(manager.raypath_color_.entries_.empty());

  nlohmann::json j_out = manager;
  EXPECT_FALSE(j_out.contains("raypath_color"))
      << "to_json must omit the key when entries_ is empty (matches crystal/filter/render convention)";
}

// ---- ConfigManager wiring: end-to-end round-trip with raypath_color ----

TEST(ConfigManagerRaypathColor, ParsesColorSectionWhenPresent) {
  std::ifstream f(config_file_name);
  ASSERT_TRUE(f.is_open());
  nlohmann::json j_in;
  f >> j_in;

  // Inject the plan §3.2 three-arcs color section on top of the v3 fixture.
  j_in["raypath_color"] = nlohmann::json::array({
      { { "layer", 0 }, { "crystal_slot", 0 }, { "summand", 0 }, { "color", { 1.0f, 0.0f, 0.0f } } },
      { { "layer", 0 }, { "crystal_slot", 1 }, { "summand", 0 }, { "color", { 0.0f, 1.0f, 0.0f } } },
      { { "layer", 0 }, { "crystal_slot", 1 }, { "summand", 1 }, { "color", { 0.0f, 0.0f, 1.0f } } },
  });

  auto manager = j_in.get<ConfigManager>();
  ASSERT_EQ(manager.raypath_color_.entries_.size(), 3u);
  EXPECT_EQ(manager.raypath_color_.entries_[0].layer_, 0);
  EXPECT_EQ(manager.raypath_color_.entries_[1].crystal_id_, 1);
  EXPECT_EQ(manager.raypath_color_.entries_[2].summand_idx_, 1);
  EXPECT_FLOAT_EQ(manager.raypath_color_.entries_[2].color_[2], 1.0f);
}

TEST(ConfigManagerRaypathColor, ToJsonEmitsColorSectionWhenPresent) {
  // Build a minimal ConfigManager directly (bypass fixture-JSON round-trip
  // caveats — the ConfigManager top-level to_json/from_json are not fully
  // symmetric on unrelated fields like light_source. Here we only care about
  // the raypath_color slice being emitted.)
  ConfigManager m{};
  m.raypath_color_.entries_.push_back(MakeEntry(0, 1, 0, 0.25f, 0.5f, 0.75f));

  nlohmann::json j = m;
  ASSERT_TRUE(j.contains("raypath_color"));
  ASSERT_TRUE(j.at("raypath_color").is_array());
  ASSERT_EQ(j.at("raypath_color").size(), 1u);
  const auto& j_entry = j.at("raypath_color").at(0);
  EXPECT_EQ(j_entry.at("layer").get<uint16_t>(), 0u);
  EXPECT_EQ(j_entry.at("crystal_slot").get<uint16_t>(), 1u);
  EXPECT_EQ(j_entry.at("summand").get<uint16_t>(), 0u);
  EXPECT_FLOAT_EQ(j_entry.at("color").at(0).get<float>(), 0.25f);
  EXPECT_FLOAT_EQ(j_entry.at("color").at(1).get<float>(), 0.5f);
  EXPECT_FLOAT_EQ(j_entry.at("color").at(2).get<float>(), 0.75f);
}
