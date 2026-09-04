#include <gtest/gtest.h>

#include <nlohmann/json.hpp>
#include <string>
#include <utility>
#include <vector>

#include "config/config_compare.hpp"
#include "config/render_config.hpp"

namespace {

// Construct a baseline RenderConfig with non-default values to avoid false negatives
// from comparing two default-constructed (all-zero) configs.
lumice::RenderConfig MakeBaseline() {
  lumice::RenderConfig cfg;
  cfg.id_ = 42;
  cfg.lens_ = { lumice::LensParam::kFisheyeEqualArea, 120.0f };
  cfg.lens_shift_[0] = 10;
  cfg.lens_shift_[1] = 20;
  cfg.resolution_[0] = 1920;
  cfg.resolution_[1] = 1080;
  cfg.view_ = { 45.0f, 30.0f, 10.0f };
  cfg.visible_ = lumice::RenderConfig::kFull;
  cfg.background_[0] = 0.1f;
  cfg.background_[1] = 0.2f;
  cfg.background_[2] = 0.3f;
  cfg.ray_color_[0] = 1.0f;
  cfg.ray_color_[1] = 0.5f;
  cfg.ray_color_[2] = 0.0f;
  cfg.intensity_factor_ = 2.0f;
  cfg.overlap_ = 0.1f;
  cfg.horizon_ = false;
  // Leave grids and ms_filter empty for baseline.
  return cfg;
}


TEST(RenderConfigTest, IdenticalConfigs_ReturnFalse) {
  auto a = MakeBaseline();
  auto b = MakeBaseline();
  EXPECT_FALSE(lumice::NeedsRebuild(a, b));
}

TEST(RenderConfigTest, EachLayoutField_ReturnsTrue) {
  auto base = MakeBaseline();

  // resolution
  {
    auto mod = base;
    mod.resolution_[0] = 3840;
    EXPECT_TRUE(lumice::NeedsRebuild(base, mod)) << "resolution width";
  }
  {
    auto mod = base;
    mod.resolution_[1] = 2160;
    EXPECT_TRUE(lumice::NeedsRebuild(base, mod)) << "resolution height";
  }

  // lens type
  {
    auto mod = base;
    mod.lens_.type_ = lumice::LensParam::kRectangular;
    EXPECT_TRUE(lumice::NeedsRebuild(base, mod)) << "lens type";
  }

  // lens fov
  {
    auto mod = base;
    mod.lens_.fov_ = 60.0f;
    EXPECT_TRUE(lumice::NeedsRebuild(base, mod)) << "lens fov";
  }

  // lens_shift
  {
    auto mod = base;
    mod.lens_shift_[0] = 99;
    EXPECT_TRUE(lumice::NeedsRebuild(base, mod)) << "lens_shift[0]";
  }
  {
    auto mod = base;
    mod.lens_shift_[1] = 99;
    EXPECT_TRUE(lumice::NeedsRebuild(base, mod)) << "lens_shift[1]";
  }

  // view (azimuth)
  {
    auto mod = base;
    mod.view_.az_ = 180.0f;
    EXPECT_TRUE(lumice::NeedsRebuild(base, mod)) << "view azimuth";
  }

  // view (elevation)
  {
    auto mod = base;
    mod.view_.el_ = -45.0f;
    EXPECT_TRUE(lumice::NeedsRebuild(base, mod)) << "view elevation";
  }

  // view (roll)
  {
    auto mod = base;
    mod.view_.ro_ = 90.0f;
    EXPECT_TRUE(lumice::NeedsRebuild(base, mod)) << "view roll";
  }

  // visible
  {
    auto mod = base;
    mod.visible_ = lumice::RenderConfig::kUpper;
    EXPECT_TRUE(lumice::NeedsRebuild(base, mod)) << "visible";
  }

  // overlap
  {
    auto mod = base;
    mod.overlap_ = 0.5f;
    EXPECT_TRUE(lumice::NeedsRebuild(base, mod)) << "overlap";
  }

  // front — layout, not appearance: it is an input to the same visible_mask_ that visible_ is an
  // input to, and that mask is built once in the consumer's constructor.
  {
    auto mod = base;
    mod.front_ = true;
    EXPECT_TRUE(lumice::NeedsRebuild(base, mod)) << "front";
  }
}

TEST(RenderConfigTest, EachAppearanceField_ReturnsFalse) {
  auto base = MakeBaseline();

  // id
  {
    auto mod = base;
    mod.id_ = 999;
    EXPECT_FALSE(lumice::NeedsRebuild(base, mod)) << "id";
  }

  // background
  {
    auto mod = base;
    mod.background_[0] = 1.0f;
    EXPECT_FALSE(lumice::NeedsRebuild(base, mod)) << "background";
  }

  // ray_color
  {
    auto mod = base;
    mod.ray_color_[2] = 1.0f;
    EXPECT_FALSE(lumice::NeedsRebuild(base, mod)) << "ray_color";
  }

  // intensity_factor
  {
    auto mod = base;
    mod.intensity_factor_ = 10.0f;
    EXPECT_FALSE(lumice::NeedsRebuild(base, mod)) << "intensity_factor";
  }

  // angular_dist_grid
  {
    auto mod = base;
    mod.angular_dist_grid_.push_back(lumice::GridLineParam{ 10.0f, 2.0f, 0.5f, { 1, 0, 0 } });
    EXPECT_FALSE(lumice::NeedsRebuild(base, mod)) << "angular_dist_grid";
  }

  // elevation_grid
  {
    auto mod = base;
    mod.elevation_grid_.push_back(lumice::GridLineParam{ 22.0f, 1.0f, 1.0f, { 0, 1, 0 } });
    EXPECT_FALSE(lumice::NeedsRebuild(base, mod)) << "elevation_grid";
  }

  // longitude_grid
  {
    auto mod = base;
    mod.longitude_grid_.push_back(lumice::GridLineParam{ 90.0f, 1.0f, 1.0f, { 0, 0, 1 } });
    EXPECT_FALSE(lumice::NeedsRebuild(base, mod)) << "longitude_grid";
  }

  // horizon
  {
    auto mod = base;
    mod.horizon_ = true;
    EXPECT_FALSE(lumice::NeedsRebuild(base, mod)) << "horizon";
  }

  // ev_mode: it selects WHICH exposure formula PostSnapshot uses, not the accumulation layout,
  // so it must NOT force a consumer rebuild (same classification as intensity_factor above).
  {
    auto mod = base;
    mod.ev_mode_ = lumice::RenderConfig::kAbsolute;
    EXPECT_FALSE(lumice::NeedsRebuild(base, mod)) << "ev_mode";
  }
}

TEST(RenderConfigTest, Symmetry_LayoutChanges) {
  auto base = MakeBaseline();

  // Test a representative subset of layout fields for symmetry.
  auto mod_res = base;
  mod_res.resolution_[0] = 3840;
  EXPECT_EQ(lumice::NeedsRebuild(base, mod_res), lumice::NeedsRebuild(mod_res, base)) << "resolution symmetry";

  auto mod_lens = base;
  mod_lens.lens_.type_ = lumice::LensParam::kRectangular;
  EXPECT_EQ(lumice::NeedsRebuild(base, mod_lens), lumice::NeedsRebuild(mod_lens, base)) << "lens symmetry";

  auto mod_view = base;
  mod_view.view_.az_ = 180.0f;
  EXPECT_EQ(lumice::NeedsRebuild(base, mod_view), lumice::NeedsRebuild(mod_view, base)) << "view symmetry";

  auto mod_overlap = base;
  mod_overlap.overlap_ = 0.5f;
  EXPECT_EQ(lumice::NeedsRebuild(base, mod_overlap), lumice::NeedsRebuild(mod_overlap, base)) << "overlap symmetry";
}

TEST(RenderConfigTest, MixedChanges_LayoutPlusAppearance) {
  auto base = MakeBaseline();

  // Change both layout (resolution) and appearance (background) — layout dominates.
  auto mod = base;
  mod.resolution_[0] = 3840;
  mod.background_[0] = 0.9f;
  mod.intensity_factor_ = 0.1f;
  EXPECT_TRUE(lumice::NeedsRebuild(base, mod)) << "layout change should dominate appearance changes";
}

// ===== ev_mode: default, JSON round trip, equality =====

TEST(RenderConfigEvModeTest, DefaultIsRelative) {
  lumice::RenderConfig cfg;
  EXPECT_EQ(cfg.ev_mode_, lumice::RenderConfig::kRelative);
}

TEST(RenderConfigEvModeTest, ToJson_EmitsModeString) {
  auto cfg = MakeBaseline();
  cfg.ev_mode_ = lumice::RenderConfig::kAbsolute;
  nlohmann::json j = cfg;
  EXPECT_EQ(j.at("ev_mode").get<std::string>(), "absolute");

  cfg.ev_mode_ = lumice::RenderConfig::kRelative;
  nlohmann::json j2 = cfg;
  EXPECT_EQ(j2.at("ev_mode").get<std::string>(), "relative");
}

TEST(RenderConfigEvModeTest, FromJson_BothValuesRoundTrip) {
  for (const auto& [text, expected] : std::vector<std::pair<std::string, lumice::RenderConfig::EvMode>>{
           { "relative", lumice::RenderConfig::kRelative }, { "absolute", lumice::RenderConfig::kAbsolute } }) {
    auto mode = lumice::RenderConfig::kAbsolute;  // seed with the non-default so "relative" is a real read
    nlohmann::json(text).get_to(mode);
    EXPECT_EQ(mode, expected) << text;
  }
}

// An unrecognized string must land on the SAME mode a missing key does. nlohmann maps an unknown
// value to the first table entry, and kRelative is first on purpose — this pins that ordering.
TEST(RenderConfigEvModeTest, FromJson_UnknownStringFallsBackToRelative) {
  auto mode = lumice::RenderConfig::kAbsolute;
  nlohmann::json("no_such_mode").get_to(mode);
  EXPECT_EQ(mode, lumice::RenderConfig::kRelative);
}

// operator== is a full equality predicate (unlike NeedsRebuild, which asks a narrower question),
// so it MUST see ev_mode.
TEST(RenderConfigEvModeTest, OperatorEq_ComparesEvMode) {
  auto a = MakeBaseline();
  auto b = MakeBaseline();
  EXPECT_TRUE(a == b);
  b.ev_mode_ = lumice::RenderConfig::kAbsolute;
  EXPECT_FALSE(a == b);
}

// The meridian list added in v4.18, held to the same three properties the parallels already have:
// it round-trips through JSON under its own key, a missing key leaves it empty (an old config is
// not silently given lines), and operator== sees it.
TEST(RenderConfigLongitudeGridTest, ToJson_EmitsUnderTheGridLongitudeKey) {
  auto cfg = MakeBaseline();
  cfg.longitude_grid_.push_back(lumice::GridLineParam{ -90.0f, 1.5f, 0.4f, { 0.2f, 0.4f, 0.6f } });
  cfg.longitude_grid_.push_back(lumice::GridLineParam{ 180.0f, 1.0f, 1.0f, { 1, 1, 1 } });

  nlohmann::json j = cfg;

  ASSERT_TRUE(j.contains("grid"));
  ASSERT_TRUE(j["grid"].contains("longitude"));
  ASSERT_EQ(j["grid"]["longitude"].size(), 2u);
  EXPECT_NEAR(j["grid"]["longitude"][0]["value"].get<float>(), -90.0f, 1e-5f);
  EXPECT_NEAR(j["grid"]["longitude"][1]["value"].get<float>(), 180.0f, 1e-5f);
  // The parallels keep their own key: the two families are separate lists, not one merged array.
  ASSERT_TRUE(j["grid"].contains("elevation"));
  EXPECT_EQ(j["grid"]["elevation"].size(), 0u);
}

TEST(RenderConfigLongitudeGridTest, OperatorEq_ComparesLongitudeGrid) {
  auto a = MakeBaseline();
  auto b = MakeBaseline();
  EXPECT_TRUE(a == b);
  b.longitude_grid_.push_back(lumice::GridLineParam{ 0.0f, 1.0f, 1.0f, { 1, 1, 1 } });
  EXPECT_FALSE(a == b);
  // ... and does not confuse it with the parallels, which is the failure a copy-pasted comparison
  // term would produce.
  a.elevation_grid_.push_back(lumice::GridLineParam{ 0.0f, 1.0f, 1.0f, { 1, 1, 1 } });
  EXPECT_FALSE(a == b);
}

// The zenith / nadir marker block (v4.19). Held to the same properties the line families are, plus
// the one that is only true of THIS field: its appearance defaults are non-zero, so "missing key"
// and "zero-initialized" are different states and a decoder that conflates them is wrong.
TEST(RenderConfigZenithNadirTest, ToJson_EmitsUnderTheGridZenithNadirKey) {
  auto cfg = MakeBaseline();
  cfg.zenith_nadir_.enabled_ = true;
  cfg.zenith_nadir_.radius_px_ = 14.0f;
  cfg.zenith_nadir_.opacity_ = 0.25f;
  cfg.zenith_nadir_.color_[0] = 0.1f;
  cfg.zenith_nadir_.color_[1] = 0.7f;
  cfg.zenith_nadir_.color_[2] = 0.9f;

  nlohmann::json j = cfg;

  ASSERT_TRUE(j.contains("grid"));
  ASSERT_TRUE(j["grid"].contains("zenith_nadir"));
  const auto& z = j["grid"]["zenith_nadir"];
  EXPECT_TRUE(z["enabled"].get<bool>());
  EXPECT_NEAR(z["radius_px"].get<float>(), 14.0f, 1e-5f);
  EXPECT_NEAR(z["opacity"].get<float>(), 0.25f, 1e-5f);
  EXPECT_NEAR(z["color"][1].get<float>(), 0.7f, 1e-5f);

  lumice::ZenithNadirParam back;
  z.get_to(back);
  EXPECT_TRUE(back == cfg.zenith_nadir_);
}

TEST(RenderConfigZenithNadirTest, DefaultIsOffWithTheGuiAppearanceValues) {
  const lumice::ZenithNadirParam z;
  // Opt-in, like horizon_: a config that predates the field must not gain a marker.
  EXPECT_FALSE(z.enabled_);
  // The three appearance values are the GUI control's own defaults, and they are NOT zero — which
  // is why the C API decoder seeds them from this struct rather than relying on value-init.
  EXPECT_NEAR(z.radius_px_, 8.0f, 1e-5f);
  EXPECT_NEAR(z.opacity_, 0.6f, 1e-5f);
  EXPECT_NEAR(z.color_[0], 0.8f, 1e-5f);
  EXPECT_NEAR(z.color_[1], 0.2f, 1e-5f);
  EXPECT_NEAR(z.color_[2], 0.2f, 1e-5f);
}

TEST(RenderConfigZenithNadirTest, FromJson_PartialObjectKeepsTheMemberDefaults) {
  // The middle case between "no key" and "every key": whatever the object omits keeps the struct's
  // own default, not a zero. A decoder that resets the struct before reading turns a document that
  // only says `{"enabled": true}` into an invisible marker (radius 0, alpha 0, black).
  lumice::ZenithNadirParam z;
  const nlohmann::json j = nlohmann::json::parse(R"({ "enabled": true })");
  j.get_to(z);
  EXPECT_TRUE(z.enabled_);
  EXPECT_NEAR(z.radius_px_, 8.0f, 1e-5f);
  EXPECT_NEAR(z.opacity_, 0.6f, 1e-5f);
  EXPECT_NEAR(z.color_[0], 0.8f, 1e-5f);
}

TEST(RenderConfigZenithNadirTest, OperatorEq_ComparesEveryField) {
  auto a = MakeBaseline();
  auto b = MakeBaseline();
  EXPECT_TRUE(a == b);
  b.zenith_nadir_.enabled_ = true;
  EXPECT_FALSE(a == b);
  b = MakeBaseline();
  b.zenith_nadir_.radius_px_ = 3.0f;
  EXPECT_FALSE(a == b) << "the radius is part of the config's identity, not a display-only extra";
  b = MakeBaseline();
  b.zenith_nadir_.opacity_ = 0.1f;
  EXPECT_FALSE(a == b);
  b = MakeBaseline();
  b.zenith_nadir_.color_[2] = 0.5f;
  EXPECT_FALSE(a == b);
}

TEST(RenderConfigZenithNadirTest, NeedsRebuild_TreatsTheMarkerAsAppearance) {
  // Every field of this block is appearance: none of them changes which pixel images which
  // direction, so a consumer must be REUSED across the change (ResetWith), not rebuilt.
  auto a = MakeBaseline();
  auto b = MakeBaseline();
  b.zenith_nadir_.enabled_ = true;
  b.zenith_nadir_.radius_px_ = 20.0f;
  b.zenith_nadir_.opacity_ = 0.9f;
  b.zenith_nadir_.color_[0] = 0.0f;
  EXPECT_FALSE(lumice::NeedsRebuild(a, b));
}

// The reference-point markers (v4.25) — the generalization of zenith_nadir above to N named
// directions with per-entry colour. Two properties carry most of the weight here, and neither is
// visible from the struct alone: an unknown id must be REJECTED rather than silently resolved (the
// enum-fallback trap the RenderConfigFrontTest cases below pin for `visible`), and the family's
// two shared appearance values must be non-zero defaults for the same reason zenith_nadir's are.
TEST(RenderConfigMarkersTest, ToJson_EmitsUnderGridMarkersWithSiblingFamilyKeys) {
  auto cfg = MakeBaseline();
  cfg.markers_.push_back({ lumice::MarkerRefId::kSun, true, { 1.0f, 0.9f, 0.2f } });
  cfg.markers_.push_back({ lumice::MarkerRefId::kAntisolar, false, { 0.2f, 0.4f, 1.0f } });
  cfg.markers_opacity_ = 0.35f;
  cfg.markers_radius_px_ = 12.0f;

  nlohmann::json j = cfg;

  ASSERT_TRUE(j.contains("grid"));
  ASSERT_TRUE(j["grid"].contains("markers"));
  const auto& m = j["grid"]["markers"];
  ASSERT_TRUE(m.is_array());
  ASSERT_EQ(m.size(), 2u);
  // The id is the annotation layer's own word, not a number: a persisted schema that spelled these
  // as indices would break the moment an id was inserted rather than appended.
  EXPECT_EQ(m[0]["id"].get<std::string>(), "sun");
  EXPECT_TRUE(m[0]["enabled"].get<bool>());
  EXPECT_NEAR(m[0]["color"][1].get<float>(), 0.9f, 1e-5f);
  EXPECT_EQ(m[1]["id"].get<std::string>(), "antisolar");
  EXPECT_FALSE(m[1]["enabled"].get<bool>());

  // Family-wide, and SIBLING keys of the array rather than members of a wrapper object — same
  // shape as every other appearance knob under "grid".
  EXPECT_NEAR(j["grid"]["markers_opacity"].get<float>(), 0.35f, 1e-5f);
  EXPECT_NEAR(j["grid"]["markers_radius_px"].get<float>(), 12.0f, 1e-5f);

  std::vector<lumice::MarkerStyleParam> back;
  m.get_to(back);
  EXPECT_TRUE(back == cfg.markers_);
}

TEST(RenderConfigMarkersTest, DefaultIsAnEmptyListWithTheZenithNadirAppearanceValues) {
  const lumice::RenderConfig cfg;
  // Empty is what makes the family opt-in AND what the renderer reads as "absent" when deciding
  // between this list and the legacy zenith_nadir block.
  EXPECT_TRUE(cfg.markers_.empty());
  // Non-zero, like ZenithNadirParam's: a decoder that value-initializes instead of seeding from
  // this struct yields zero-radius fully transparent rings, i.e. a marker that draws nothing.
  EXPECT_NEAR(cfg.markers_opacity_, 0.6f, 1e-5f);
  EXPECT_NEAR(cfg.markers_radius_px_, 8.0f, 1e-5f);
}

TEST(RenderConfigMarkersTest, FromJson_EveryIdSpellingRoundTrips) {
  // The whole id vocabulary in one case: a spelling that decodes to the wrong direction would draw
  // a ring somewhere plausible, so each name is pinned to its enumerator by value.
  const std::pair<const char*, lumice::MarkerRefId> kExpected[] = {
    { "zenith", lumice::MarkerRefId::kZenith },
    { "nadir", lumice::MarkerRefId::kNadir },
    { "sun", lumice::MarkerRefId::kSun },
    { "subsun", lumice::MarkerRefId::kSubsun },
    { "anthelion", lumice::MarkerRefId::kAnthelion },
    { "antisolar", lumice::MarkerRefId::kAntisolar },
  };
  for (const auto& [name, id] : kExpected) {
    lumice::MarkerStyleParam m;
    const nlohmann::json j = { { "id", name } };
    j.get_to(m);
    EXPECT_EQ(m.id_, id) << "id spelling [" << name << "] decoded to the wrong direction";
    // Round-trips back to the same word, so the reader and the writer share one vocabulary.
    const nlohmann::json out = m;
    EXPECT_EQ(out["id"].get<std::string>(), name);
  }
}

TEST(RenderConfigMarkersTest, FromJson_UnknownIdIsRejected) {
  // The point of the whole hand-written codec. NLOHMANN_JSON_SERIALIZE_ENUM would map this to the
  // FIRST entry (the zenith) and report nothing — the same silent-mapping defect
  // doc/gui-state-governance.md records for "front" becoming "upper".
  lumice::MarkerStyleParam m;
  const nlohmann::json j = nlohmann::json::parse(R"({ "id": "sundog" })");
  EXPECT_THROW(j.get_to(m), nlohmann::json::exception);
  // And it is NOT left resolved to the zenith by a partial write before the throw.
  EXPECT_EQ(m.id_, lumice::MarkerRefId::kZenith) << "member default, not a decoded value";
}

TEST(RenderConfigMarkersTest, FromJson_MissingIdIsRejected) {
  // Unlike ZenithNadirParam, where every key is optional: there, the defaults describe a complete
  // marker; here, an entry with no id names no direction at all.
  lumice::MarkerStyleParam m;
  const nlohmann::json j = nlohmann::json::parse(R"({ "enabled": true })");
  EXPECT_THROW(j.get_to(m), nlohmann::json::exception);
}

TEST(RenderConfigMarkersTest, FromJson_PartialEntryKeepsTheMemberDefaults) {
  // `id` is mandatory, the rest are not: an entry that gives only the id keeps the struct's colour
  // and its (off) enabled state, exactly as ZenithNadirParam's partial-object rule works.
  lumice::MarkerStyleParam m;
  const nlohmann::json j = nlohmann::json::parse(R"({ "id": "subsun" })");
  j.get_to(m);
  EXPECT_EQ(m.id_, lumice::MarkerRefId::kSubsun);
  EXPECT_FALSE(m.enabled_);
  EXPECT_NEAR(m.color_[0], 0.8f, 1e-5f);
  EXPECT_NEAR(m.color_[1], 0.2f, 1e-5f);
}

TEST(RenderConfigMarkersTest, HasDuplicateMarkerId_DetectsRepeatsAndReportsWhich) {
  std::vector<lumice::MarkerStyleParam> unique = {
    { lumice::MarkerRefId::kZenith, true, { 1.0f, 0.0f, 0.0f } },
    { lumice::MarkerRefId::kSun, true, { 0.0f, 1.0f, 0.0f } },
  };
  lumice::MarkerRefId dup = lumice::MarkerRefId::kNadir;
  EXPECT_FALSE(lumice::HasDuplicateMarkerId(unique, &dup));

  // Empty and single-entry lists have nothing to repeat — the boundary the loop bounds must get
  // right, since an off-by-one there would report every one-entry list as a duplicate.
  EXPECT_FALSE(lumice::HasDuplicateMarkerId({}, &dup));
  EXPECT_FALSE(lumice::HasDuplicateMarkerId({ unique[0] }, &dup));

  // Differing colours, same id: still a duplicate. That is the case worth pinning, because it is
  // the one where "keep the last" would look like a reasonable merge rule.
  std::vector<lumice::MarkerStyleParam> repeated = {
    { lumice::MarkerRefId::kSun, true, { 1.0f, 0.0f, 0.0f } },
    { lumice::MarkerRefId::kNadir, true, { 0.0f, 1.0f, 0.0f } },
    { lumice::MarkerRefId::kSun, false, { 0.0f, 0.0f, 1.0f } },
  };
  EXPECT_TRUE(lumice::HasDuplicateMarkerId(repeated, &dup));
  EXPECT_EQ(dup, lumice::MarkerRefId::kSun);

  // The out-parameter is optional.
  EXPECT_TRUE(lumice::HasDuplicateMarkerId(repeated, nullptr));
}

TEST(RenderConfigMarkersTest, OperatorEq_ComparesEveryMarkerField) {
  auto a = MakeBaseline();
  a.markers_.push_back({ lumice::MarkerRefId::kSun, true, { 1.0f, 0.9f, 0.2f } });
  auto b = a;
  EXPECT_TRUE(a == b);

  b = a;
  b.markers_[0].id_ = lumice::MarkerRefId::kSubsun;
  EXPECT_FALSE(a == b);
  b = a;
  b.markers_[0].enabled_ = false;
  EXPECT_FALSE(a == b);
  b = a;
  b.markers_[0].color_[2] = 0.5f;
  EXPECT_FALSE(a == b);
  b = a;
  b.markers_.clear();
  EXPECT_FALSE(a == b);
  b = a;
  b.markers_opacity_ = 0.1f;
  EXPECT_FALSE(a == b) << "family opacity is part of the config's identity, not a display-only extra";
  b = a;
  b.markers_radius_px_ = 3.0f;
  EXPECT_FALSE(a == b);
}

TEST(RenderConfigMarkersTest, NeedsRebuild_TreatsTheMarkersAsAppearance) {
  // Same classification as zenith_nadir: none of these changes which pixel images which direction,
  // so a consumer must be REUSED across the change (ResetWith), not rebuilt.
  auto a = MakeBaseline();
  auto b = MakeBaseline();
  b.markers_.push_back({ lumice::MarkerRefId::kAnthelion, true, { 0.0f, 1.0f, 1.0f } });
  b.markers_opacity_ = 0.9f;
  b.markers_radius_px_ = 20.0f;
  EXPECT_FALSE(lumice::NeedsRebuild(a, b));
}

// The front-hemisphere clip (v4.20). The property that matters most here is NEGATIVE: it must be a
// key of its own and must NOT be expressible through "visible". NLOHMANN_JSON_SERIALIZE_ENUM maps
// an unregistered string to the FIRST table entry without an error, so a config that tried to say
// "visible": "front" would decode to kUpper and render the wrong half in silence.
TEST(RenderConfigFrontTest, ToJson_EmitsATopLevelFrontKeyBesideVisible) {
  auto cfg = MakeBaseline();
  cfg.visible_ = lumice::RenderConfig::kFull;
  cfg.front_ = true;

  nlohmann::json j = cfg;

  ASSERT_TRUE(j.contains("front"));
  EXPECT_TRUE(j["front"].is_boolean());
  EXPECT_TRUE(j["front"].get<bool>());
  // Beside "visible", not inside it and not inside "grid": the two clips are orthogonal, and front
  // is not an annotation.
  EXPECT_EQ(j["visible"].get<std::string>(), "full");
  EXPECT_FALSE(j["grid"].contains("front"));

  // Written unconditionally, like "visible" — an off clip is stated, not omitted.
  cfg.front_ = false;
  nlohmann::json j_off = cfg;
  ASSERT_TRUE(j_off.contains("front"));
  EXPECT_FALSE(j_off["front"].get<bool>());
}

TEST(RenderConfigFrontTest, DefaultIsOffAndAnUnregisteredVisibleStringIsNotIt) {
  EXPECT_FALSE(lumice::RenderConfig{}.front_);

  // The trap this field exists to avoid, pinned as a fact about the enum rather than a warning in
  // prose: "front" is not a VisibleRange, and asking for it yields kUpper with no error.
  auto decoded = lumice::RenderConfig::kFull;
  nlohmann::json("front").get_to(decoded);
  EXPECT_EQ(decoded, lumice::RenderConfig::kUpper);
}

TEST(RenderConfigFrontTest, OperatorEq_ComparesFront) {
  auto a = MakeBaseline();
  auto b = MakeBaseline();
  EXPECT_TRUE(a == b);
  b.front_ = true;
  EXPECT_FALSE(a == b);
  // Not aliased onto visible_: two configs that differ ONLY in front must still compare unequal
  // while their visible_ agree.
  EXPECT_EQ(a.visible_, b.visible_);
}

}  // namespace
