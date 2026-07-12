// Tests for src/config/raypath_color_config.hpp — JSON round-trip of the
// Design-2 (2026-07-08, doc/gui-custom-spectrum-and-raypath-color.md §4.0
// SUPERSEDES 339/§4.7) placement-scoped predicate schema and its optional
// wiring through ConfigManager. The backward-compat guarantee (missing
// top-level "raypath_color" key → empty classes_) is unchanged.

#include <gtest/gtest.h>

#include <fstream>
#include <nlohmann/json.hpp>
#include <string>
#include <variant>

#include "config/config_manager.hpp"
#include "config/filter_config.hpp"
#include "config/raypath_color_config.hpp"

extern std::string config_file_name;

namespace {

using lumice::ColorClassConfig;
using lumice::ConfigManager;
using lumice::EntryExitFilterParam;
using lumice::NoneFilterParam;
using lumice::RaypathColorConfig;
using lumice::RaypathColorRef;
using lumice::RaypathFilterParam;
using lumice::SimpleFilterParam;

RaypathColorRef MakeRef(uint16_t layer, uint16_t crystal, SimpleFilterParam predicate = NoneFilterParam{}) {
  RaypathColorRef r{};
  r.layer_ = layer;
  r.crystal_ = crystal;
  r.predicate_ = std::move(predicate);
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

// ---- RaypathColorRef JSON round-trip ----

TEST(RaypathColorRef, JsonRoundTripMatchAllDefault) {
  auto original = MakeRef(1, 2);
  nlohmann::json j = original;
  EXPECT_EQ(j.at("layer").get<uint16_t>(), 1u);
  EXPECT_EQ(j.at("crystal").get<uint16_t>(), 2u);
  // match-all whole-crystal ref emits NO predicate `type` on the wire
  // (Design 2 minimal shape).
  EXPECT_FALSE(j.contains("type"));

  auto restored = j.get<RaypathColorRef>();
  EXPECT_EQ(restored.layer_, 1);
  EXPECT_EQ(restored.crystal_, 2);
  EXPECT_TRUE(std::holds_alternative<NoneFilterParam>(restored.predicate_));
}

TEST(RaypathColorRef, JsonRoundTripRaypathPredicate) {
  RaypathFilterParam rp{ /*raypath=*/{ 3, 5, 1 } };
  auto original = MakeRef(0, 1, SimpleFilterParam{ rp });
  nlohmann::json j = original;
  EXPECT_EQ(j.at("type").get<std::string>(), "raypath");
  ASSERT_TRUE(j.at("raypath").is_array());
  EXPECT_EQ(j.at("raypath").size(), 3u);

  auto restored = j.get<RaypathColorRef>();
  ASSERT_TRUE(std::holds_alternative<RaypathFilterParam>(restored.predicate_));
  const auto& rr = std::get<RaypathFilterParam>(restored.predicate_);
  ASSERT_EQ(rr.raypath_.size(), 3u);
  EXPECT_EQ(rr.raypath_[0], 3u);
  EXPECT_EQ(rr.raypath_[1], 5u);
  EXPECT_EQ(rr.raypath_[2], 1u);
}

TEST(RaypathColorRef, JsonRoundTripEntryExitPredicate) {
  EntryExitFilterParam ee{};
  ee.min_len_ = 2;
  ee.max_len_ = 2;
  auto original = MakeRef(0, 5, SimpleFilterParam{ ee });
  nlohmann::json j = original;
  EXPECT_EQ(j.at("type").get<std::string>(), "entry_exit");
  EXPECT_EQ(j.at("min_len").get<size_t>(), 2u);
  EXPECT_EQ(j.at("max_len").get<size_t>(), 2u);

  auto restored = j.get<RaypathColorRef>();
  ASSERT_TRUE(std::holds_alternative<EntryExitFilterParam>(restored.predicate_));
  const auto& rr = std::get<EntryExitFilterParam>(restored.predicate_);
  EXPECT_EQ(rr.min_len_, 2u);
  ASSERT_TRUE(rr.max_len_.has_value());
  EXPECT_EQ(*rr.max_len_, 2u);
  EXPECT_FALSE(rr.entry_.has_value());
  EXPECT_FALSE(rr.exit_.has_value());
}

// ---- Symmetry field (scrum-color-predicate-symmetry AC4) — default omitted,
// non-default round-trips via the same P/B/D encoding as FilterConfig.

TEST(RaypathColorRef, JsonRoundTripSymmetryOmittedWhenDefault) {
  auto original = MakeRef(0, 1);
  ASSERT_EQ(original.symmetry_, lumice::FilterConfig::kSymNone);
  nlohmann::json j = original;
  EXPECT_FALSE(j.contains("symmetry")) << "default kSymNone must be omitted on the wire (AC4)";
  auto restored = j.get<RaypathColorRef>();
  EXPECT_EQ(restored.symmetry_, lumice::FilterConfig::kSymNone);
}

TEST(RaypathColorRef, JsonRoundTripSymmetryNonDefaultPB) {
  auto original = MakeRef(0, 1);
  original.symmetry_ = lumice::FilterConfig::kSymP | lumice::FilterConfig::kSymB;
  nlohmann::json j = original;
  ASSERT_TRUE(j.contains("symmetry"));
  EXPECT_EQ(j.at("symmetry").get<std::string>(), "PB") << "encoding must match FilterConfig::to_json's P/B/D order";
  auto restored = j.get<RaypathColorRef>();
  EXPECT_EQ(restored.symmetry_, lumice::FilterConfig::kSymP | lumice::FilterConfig::kSymB);
}

TEST(RaypathColorRef, JsonRoundTripSymmetryAllPBD) {
  auto original = MakeRef(1, 2);
  original.symmetry_ = lumice::FilterConfig::kSymP | lumice::FilterConfig::kSymB | lumice::FilterConfig::kSymD;
  nlohmann::json j = original;
  EXPECT_EQ(j.at("symmetry").get<std::string>(), "PBD");
  auto restored = j.get<RaypathColorRef>();
  EXPECT_EQ(restored.symmetry_,
            lumice::FilterConfig::kSymP | lumice::FilterConfig::kSymB | lumice::FilterConfig::kSymD);
}

TEST(RaypathColorRef, JsonRoundTripSymmetryEncodingMatchesFilterConfig) {
  // Cross-check: the single-source helpers used by both FilterConfig::to_json
  // and RaypathColorRef::to_json must produce the same P/B/D string encoding
  // (AC4 anchor — a divergence would mean a user's config could parse under
  // FilterConfig but not RaypathColorRef or vice versa).
  uint8_t combos[] = {
    lumice::FilterConfig::kSymNone,
    lumice::FilterConfig::kSymP,
    lumice::FilterConfig::kSymB,
    lumice::FilterConfig::kSymD,
    static_cast<uint8_t>(lumice::FilterConfig::kSymP | lumice::FilterConfig::kSymB),
    static_cast<uint8_t>(lumice::FilterConfig::kSymP | lumice::FilterConfig::kSymD),
    static_cast<uint8_t>(lumice::FilterConfig::kSymB | lumice::FilterConfig::kSymD),
    static_cast<uint8_t>(lumice::FilterConfig::kSymP | lumice::FilterConfig::kSymB | lumice::FilterConfig::kSymD),
  };
  for (uint8_t sym : combos) {
    std::string encoded = lumice::FilterSymmetryToString(sym);
    uint8_t decoded = lumice::FilterSymmetryFromString(encoded);
    EXPECT_EQ(decoded, sym) << "encoding round-trip failed for symmetry=" << int(sym) << " (" << encoded << ")";
  }
}

// ---- SimpleFilterParam publicly exposed JSON round-trip (extracted from
// FilterConfig so RaypathColorRef and FilterConfig share one wire form).

TEST(SimpleFilterParam, JsonRoundTripNone) {
  SimpleFilterParam p = NoneFilterParam{};
  nlohmann::json j;
  to_json(j, p);
  EXPECT_EQ(j.at("type").get<std::string>(), "none");

  SimpleFilterParam back{};
  from_json(j, back);
  EXPECT_TRUE(std::holds_alternative<NoneFilterParam>(back));
}

TEST(SimpleFilterParam, JsonMissingTypeIsMatchAll) {
  // Missing `type` key is the RaypathColorRef whole-crystal default. Standalone
  // helper must decode it as NoneFilterParam (single home; no per-caller
  // divergence between FilterConfig and RaypathColorRef).
  nlohmann::json j = nlohmann::json::object();
  SimpleFilterParam p{};
  from_json(j, p);
  EXPECT_TRUE(std::holds_alternative<NoneFilterParam>(p));
}

// ---- ColorClassConfig JSON round-trip ----

TEST(ColorClassConfig, JsonRoundTripDefaults) {
  auto original = MakeClass(1.0f, 0.5f, 0.25f, { MakeRef(0, 1) });
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
  auto cls = MakeClass(0.0f, 0.0f, 1.0f, { MakeRef(0, 1) });
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
  cfg.classes_.push_back(MakeClass(1.0f, 0.0f, 0.0f, { MakeRef(0, 1) }));
  EntryExitFilterParam ee{};
  ee.min_len_ = 3;
  cfg.classes_.push_back(MakeClass(0.0f, 1.0f, 0.0f, { MakeRef(0, 1, SimpleFilterParam{ ee }) }));

  nlohmann::json j = cfg;
  ASSERT_TRUE(j.is_array());
  ASSERT_EQ(j.size(), 2u);

  auto restored = j.get<RaypathColorConfig>();
  ASSERT_EQ(restored.classes_.size(), 2u);
  EXPECT_EQ(restored.mode_, "dominant");
  EXPECT_FLOAT_EQ(restored.classes_[0].color_[0], 1.0f);
  EXPECT_FLOAT_EQ(restored.classes_[1].color_[1], 1.0f);
  EXPECT_TRUE(std::holds_alternative<NoneFilterParam>(restored.classes_[0].match_[0].predicate_));
  ASSERT_TRUE(std::holds_alternative<EntryExitFilterParam>(restored.classes_[1].match_[0].predicate_));
  EXPECT_EQ(std::get<EntryExitFilterParam>(restored.classes_[1].match_[0].predicate_).min_len_, 3u);
}

TEST(RaypathColorConfig, JsonRoundTripNonDefaultModeObjectForm) {
  RaypathColorConfig cfg;
  cfg.mode_ = "additive";
  EntryExitFilterParam ee{};
  ee.min_len_ = 2;
  ee.max_len_ = 2;
  cfg.classes_.push_back(MakeClass(0.0f, 0.0f, 1.0f, { MakeRef(0, 1, SimpleFilterParam{ ee }) }));

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

// ---- ConfigManager wiring: end-to-end round-trip with Design-2 schema ----

TEST(ConfigManagerRaypathColor, ParsesColorSectionWhenPresent) {
  std::ifstream f(config_file_name);
  ASSERT_TRUE(f.is_open());
  nlohmann::json j_in;
  f >> j_in;

  j_in["raypath_color"] = nlohmann::json::array({
      // Whole-crystal (match-all) — no `type` field on the wire.
      { { "color", { 1.0f, 0.0f, 0.0f } }, { "match", { { { "layer", 0 }, { "crystal", 1 } } } } },
      // EE predicate: len == 2.
      { { "color", { 0.0f, 1.0f, 0.0f } },
        { "match",
          { { { "layer", 0 }, { "crystal", 1 }, { "type", "entry_exit" }, { "min_len", 2 }, { "max_len", 2 } } } } },
      // EE predicate: len >= 3.
      { { "color", { 0.0f, 0.0f, 1.0f } },
        { "match", { { { "layer", 0 }, { "crystal", 1 }, { "type", "entry_exit" }, { "min_len", 3 } } } } },
  });

  auto manager = j_in.get<ConfigManager>();
  ASSERT_EQ(manager.raypath_color_.classes_.size(), 3u);
  EXPECT_FLOAT_EQ(manager.raypath_color_.classes_[0].color_[0], 1.0f);
  EXPECT_FLOAT_EQ(manager.raypath_color_.classes_[1].color_[1], 1.0f);
  EXPECT_FLOAT_EQ(manager.raypath_color_.classes_[2].color_[2], 1.0f);
  ASSERT_EQ(manager.raypath_color_.classes_[0].match_.size(), 1u);
  EXPECT_TRUE(std::holds_alternative<NoneFilterParam>(manager.raypath_color_.classes_[0].match_[0].predicate_));
  EXPECT_TRUE(std::holds_alternative<EntryExitFilterParam>(manager.raypath_color_.classes_[1].match_[0].predicate_));
  EXPECT_TRUE(std::holds_alternative<EntryExitFilterParam>(manager.raypath_color_.classes_[2].match_[0].predicate_));
}

TEST(ConfigManagerRaypathColor, ToJsonEmitsColorSectionWhenPresent) {
  ConfigManager m{};
  m.raypath_color_.classes_.push_back(MakeClass(0.25f, 0.5f, 0.75f, { MakeRef(0, 1) }));

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
  // Design 2: match-all whole-crystal has no wire predicate.
  EXPECT_FALSE(j_cls.at("match").at(0).contains("type"));
}
