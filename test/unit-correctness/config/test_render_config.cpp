#include <gtest/gtest.h>
#include <spdlog/sinks/ostream_sink.h>

#include <memory>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "config/config_compare.hpp"
#include "config/render_config.hpp"
#include "util/color_space.hpp"
#include "util/logger.hpp"

namespace {

// Captures everything the global logger emits for the lifetime of the object. RAII rather than a
// manual remove_sink at the end of each test, because GetSharedSink() is a process-wide singleton:
// an ASSERT_* returning early with the sink still attached would leave later tests in this binary
// writing into a destroyed ostringstream. Same shape as the copy in test_crystal_sync_group.cpp.
class LogCapture {
 public:
  LogCapture() : sink_(std::make_shared<spdlog::sinks::ostream_sink_mt>(oss_)) {
    lumice::GetSharedSink()->add_sink(sink_);
  }

  ~LogCapture() { lumice::GetSharedSink()->remove_sink(sink_); }

  LogCapture(const LogCapture&) = delete;
  LogCapture& operator=(const LogCapture&) = delete;

  std::string Text() const { return oss_.str(); }

 private:
  std::ostringstream oss_;
  std::shared_ptr<spdlog::sinks::ostream_sink_mt> sink_;
};

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

  // paper
  {
    auto mod = base;
    mod.paper_[0] = 0.5f;
    EXPECT_FALSE(lumice::NeedsRebuild(base, mod)) << "paper";
  }

  // tone
  {
    auto mod = base;
    mod.tone_ = lumice::RenderConfig::kPrint;
    EXPECT_FALSE(lumice::NeedsRebuild(base, mod)) << "tone";
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

  // view_dist_grid: the axis-referenced twin, same classification. Its reference direction is
  // derived from view_, which IS layout — but that is view_'s classification, not this list's.
  {
    auto mod = base;
    mod.view_dist_grid_.push_back(lumice::GridLineParam{ 10.0f, 2.0f, 0.5f, { 1, 0, 0 } });
    EXPECT_FALSE(lumice::NeedsRebuild(base, mod)) << "view_dist_grid";
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

  // The three family line switches, same classification as `horizon` above and for the same
  // reason: they decide whether a line is composited onto the finished image, never the shape of
  // the buffer it is composited onto.
  {
    auto mod = base;
    mod.elevation_grid_line_ = false;
    EXPECT_FALSE(lumice::NeedsRebuild(base, mod)) << "elevation_grid_line";
  }
  {
    auto mod = base;
    mod.longitude_grid_line_ = false;
    EXPECT_FALSE(lumice::NeedsRebuild(base, mod)) << "longitude_grid_line";
  }
  {
    auto mod = base;
    mod.angular_dist_grid_line_ = false;
    EXPECT_FALSE(lumice::NeedsRebuild(base, mod)) << "angular_dist_grid_line";
  }
  {
    auto mod = base;
    mod.view_dist_grid_line_ = false;
    EXPECT_FALSE(lumice::NeedsRebuild(base, mod)) << "view_dist_grid_line";
  }
  {
    auto mod = base;
    mod.view_dist_label_ = true;
    EXPECT_FALSE(lumice::NeedsRebuild(base, mod)) << "view_dist_label";
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

// ===== tone / paper: defaults, JSON round trip, warn-and-fall-back, appearance classification =====
//
// The two fields of the print display mode (doc/print-mode-subtractive-ink.md). No operator reads
// either of them yet — the field chain lands ahead of the operator on purpose — so what is pinned
// here is the chain itself: the values survive JSON both ways, an unknown tone SAYS SO instead of
// silently becoming `screen`, and neither field is mistaken for a layout field.

TEST(RenderConfigToneTest, DefaultIsScreen) {
  lumice::RenderConfig cfg;
  EXPECT_EQ(cfg.tone_, lumice::RenderConfig::kScreen);
}

// White, the opposite of background_'s black, and the whole reason `paper` is a second field
// rather than a reuse of `background`: under the subtractive operator a black ground gives
// out = 0 * 10^(-D) == 0 for every pixel, an all-black page reachable by merely ticking the mode
// on. See doc/print-mode-subtractive-ink.md decision D5.
TEST(RenderConfigPaperTest, DefaultIsWhite) {
  lumice::RenderConfig cfg;
  EXPECT_FLOAT_EQ(cfg.paper_[0], 1.0f);
  EXPECT_FLOAT_EQ(cfg.paper_[1], 1.0f);
  EXPECT_FLOAT_EQ(cfg.paper_[2], 1.0f);
}

TEST(RenderConfigToneTest, ToJson_EmitsModeString) {
  auto cfg = MakeBaseline();
  cfg.tone_ = lumice::RenderConfig::kPrint;
  nlohmann::json j = cfg;
  EXPECT_EQ(j.at("tone").get<std::string>(), "print");

  cfg.tone_ = lumice::RenderConfig::kScreen;
  nlohmann::json j2 = cfg;
  EXPECT_EQ(j2.at("tone").get<std::string>(), "screen");
}

TEST(RenderConfigToneTest, FromJson_BothValuesRoundTrip) {
  for (const auto& [text, expected] : std::vector<std::pair<std::string, lumice::RenderConfig::Tone>>{
           { "screen", lumice::RenderConfig::kScreen }, { "print", lumice::RenderConfig::kPrint } }) {
    auto tone = lumice::RenderConfig::kPrint;  // seed with the non-default so "screen" is a real read
    nlohmann::json(text).get_to(tone);
    EXPECT_EQ(tone, expected) << text;
  }
}

// The one place tone departs from ev_mode beside it, and the reason it gets a hand-written codec
// instead of NLOHMANN_JSON_SERIALIZE_ENUM: the fall back to `screen` must be ANNOUNCED. `screen`
// and `print` name two structurally different operators, so a typo silently landing on `screen`
// hands back exactly the picture the author was trying to leave. Both halves are asserted — the
// value AND the warning — because either alone would pass with the other missing.
TEST(RenderConfigToneTest, FromJson_UnknownStringWarnsAndFallsBackToScreen) {
  auto tone = lumice::RenderConfig::kPrint;
  std::string logged;
  {
    LogCapture capture;
    nlohmann::json("no_such_tone").get_to(tone);
    logged = capture.Text();
  }
  EXPECT_EQ(tone, lumice::RenderConfig::kScreen);
  EXPECT_NE(logged.find("no_such_tone"), std::string::npos) << logged;
}

// The control arm for the test above: a RECOGNIZED value must not warn. Without it, a codec that
// warned on every parse would pass the assertion above while spamming a correct config's load.
TEST(RenderConfigToneTest, FromJson_KnownStringIsSilent) {
  auto tone = lumice::RenderConfig::kScreen;
  std::string logged;
  {
    LogCapture capture;
    nlohmann::json("print").get_to(tone);
    logged = capture.Text();
  }
  EXPECT_EQ(tone, lumice::RenderConfig::kPrint);
  EXPECT_TRUE(logged.empty()) << logged;
}

// paper_ is linear in the struct and sRGB in the document, exactly as background_ is. Asserting
// the CONVERSION rather than the value round-tripping is what separates "the key is written" from
// "the key is written in the right space" — a pass-through would satisfy a round trip too.
TEST(RenderConfigPaperTest, ToJson_EmitsSrgb) {
  auto cfg = MakeBaseline();
  cfg.paper_[0] = 0.25f;
  cfg.paper_[1] = 0.5f;
  cfg.paper_[2] = 1.0f;
  nlohmann::json j = cfg;
  ASSERT_EQ(j.at("paper").size(), 3u);
  EXPECT_NEAR(j.at("paper")[0].get<float>(), lumice::LinearToSrgb(0.25f), 1e-5f);
  EXPECT_NEAR(j.at("paper")[1].get<float>(), lumice::LinearToSrgb(0.5f), 1e-5f);
  EXPECT_NEAR(j.at("paper")[2].get<float>(), lumice::LinearToSrgb(1.0f), 1e-5f);
}

// operator== is a full equality predicate (unlike NeedsRebuild, which asks a narrower question),
// so it MUST see both fields. Each is flipped on its own: comparing them together would pass with
// only one of the two comparisons wired up.
TEST(RenderConfigToneTest, OperatorEq_ComparesToneAndPaper) {
  auto a = MakeBaseline();
  {
    auto b = MakeBaseline();
    EXPECT_TRUE(a == b);
    b.tone_ = lumice::RenderConfig::kPrint;
    EXPECT_FALSE(a == b) << "tone";
  }
  {
    auto b = MakeBaseline();
    b.paper_[1] = 0.5f;
    EXPECT_FALSE(a == b) << "paper";
  }
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

// ===== The three family LINE switches (v4.26) =====
//
// One per angle list — the parallels, the meridians, the angular-distance circles — and the thing
// that makes them unlike every other annotation field in this struct is the DIRECTION of their
// default. `horizon_`, the three *_label_ switches, `zenith_nadir_` and `markers_` are all opt-in,
// because each of them turns an annotation on that a config predating the field never asked for.
// These three turn one OFF: the config that predates them was already drawing the lines its
// non-empty angle list names, and a false default would silently stop it.
TEST(RenderConfigFamilyLineSwitchTest, DefaultIsOnUnlikeEveryOtherAnnotationSwitch) {
  const lumice::RenderConfig defaults;
  EXPECT_TRUE(defaults.elevation_grid_line_);
  EXPECT_TRUE(defaults.longitude_grid_line_);
  EXPECT_TRUE(defaults.angular_dist_grid_line_);
  EXPECT_TRUE(defaults.view_dist_grid_line_);
  // Read together with the opt-in neighbours, because "true" is only meaningful here as the
  // deliberate opposite of what sits beside it — a copy-paste that gave these the same default as
  // `horizon_` would look perfectly consistent in isolation.
  EXPECT_FALSE(defaults.horizon_);
  EXPECT_FALSE(defaults.horizon_label_);
  EXPECT_FALSE(defaults.grid_label_);
  EXPECT_FALSE(defaults.angular_dist_label_);
  EXPECT_FALSE(defaults.view_dist_label_);
  EXPECT_TRUE(defaults.view_dist_grid_.empty());
}

TEST(RenderConfigFamilyLineSwitchTest, ToJson_EmitsOneKeyPerFamilyUnderGrid) {
  auto cfg = MakeBaseline();
  cfg.elevation_grid_line_ = false;
  cfg.longitude_grid_line_ = true;
  cfg.angular_dist_grid_line_ = false;
  cfg.view_dist_grid_line_ = true;

  const nlohmann::json j = cfg;

  ASSERT_TRUE(j.contains("grid"));
  ASSERT_TRUE(j["grid"].contains("elevation_line"));
  ASSERT_TRUE(j["grid"].contains("longitude_line"));
  ASSERT_TRUE(j["grid"].contains("angular_dist_line"));
  ASSERT_TRUE(j["grid"].contains("view_dist_line"));
  // Distinct values in one document: a writer that emitted the same member every time passes any
  // test that sets them all alike. view_dist_line is set to the OPPOSITE of the angular_dist_line
  // beside it for the same reason — the twin most likely to be pasted from.
  EXPECT_FALSE(j["grid"]["elevation_line"].get<bool>());
  EXPECT_TRUE(j["grid"]["longitude_line"].get<bool>());
  EXPECT_FALSE(j["grid"]["angular_dist_line"].get<bool>());
  EXPECT_TRUE(j["grid"]["view_dist_line"].get<bool>());
}

// The axis-referenced family's three keys, written under "grid" with the same spelling rule as
// the angular_dist three (list key bare, switches suffixed). The list carries a value the
// angular_dist list does not, so a writer that crossed the two twins is caught, not just one that
// dropped a key. The decode side (config_manager.cpp) is pinned through the whole parse path in
// test_json.cpp, which owns the ConfigManager fixture.
TEST(RenderConfigFamilyLineSwitchTest, ViewDistThreeKeysAreWrittenUnderGrid) {
  auto cfg = MakeBaseline();
  cfg.angular_dist_grid_.push_back(lumice::GridLineParam{ 22.0f, 1.0f, 1.0f, { 1, 1, 1 } });
  cfg.view_dist_grid_.push_back(lumice::GridLineParam{ 35.0f, 2.0f, 0.5f, { 0, 1, 0 } });
  cfg.view_dist_grid_line_ = false;
  cfg.view_dist_label_ = true;

  const nlohmann::json j = cfg;
  ASSERT_TRUE(j["grid"].contains("view_dist"));
  ASSERT_TRUE(j["grid"].contains("view_dist_line"));
  ASSERT_TRUE(j["grid"].contains("view_dist_label"));
  EXPECT_FALSE(j["grid"].contains("view_dist_grid"));
  EXPECT_FALSE(j["grid"].contains("view_dist_grid_line"));
  ASSERT_EQ(j["grid"]["view_dist"].size(), 1u);
  EXPECT_NEAR(j["grid"]["view_dist"][0]["value"].get<float>(), 35.0f, 1e-5f);
  EXPECT_FALSE(j["grid"]["view_dist_line"].get<bool>());
  EXPECT_TRUE(j["grid"]["view_dist_label"].get<bool>());
  // The twin is untouched by the new keys.
  ASSERT_EQ(j["grid"]["angular_dist"].size(), 1u);
  EXPECT_NEAR(j["grid"]["angular_dist"][0]["value"].get<float>(), 22.0f, 1e-5f);
  EXPECT_TRUE(j["grid"]["angular_dist_line"].get<bool>());
}

// The key names carry no "grid" of their own — they already sit under "grid", exactly as
// "elevation" / "longitude" / "angular_dist" do while their C++ members are *_grid_. Pinned
// because the C++ spelling and the JSON spelling deliberately differ, so a rename on one side
// that "fixed" the asymmetry would be a silent schema break.
TEST(RenderConfigFamilyLineSwitchTest, ToJson_UsesTheListsOwnKeySpelling) {
  const nlohmann::json j = MakeBaseline();
  EXPECT_FALSE(j["grid"].contains("elevation_grid_line"));
  EXPECT_FALSE(j["grid"].contains("longitude_grid_line"));
  EXPECT_FALSE(j["grid"].contains("angular_dist_grid_line"));
}

// The switch does not touch the list it gates. Stated because the alternative encoding — "no lines"
// meaning "empty list" — is what this field replaced, and a decoder or a writer that still cleared
// the list would take the family's LABELS with it, which is the whole defect being fixed.
TEST(RenderConfigFamilyLineSwitchTest, TheSwitchLeavesItsAngleListAlone) {
  auto cfg = MakeBaseline();
  cfg.elevation_grid_.push_back(lumice::GridLineParam{ 30.0f, 1.0f, 1.0f, { 1, 1, 1 } });
  cfg.angular_dist_grid_.push_back(lumice::GridLineParam{ 22.0f, 1.0f, 1.0f, { 1, 1, 1 } });
  cfg.elevation_grid_line_ = false;
  cfg.angular_dist_grid_line_ = false;

  const nlohmann::json j = cfg;
  ASSERT_EQ(j["grid"]["elevation"].size(), 1u);
  ASSERT_EQ(j["grid"]["angular_dist"].size(), 1u);
  EXPECT_NEAR(j["grid"]["elevation"][0]["value"].get<float>(), 30.0f, 1e-5f);
  EXPECT_NEAR(j["grid"]["angular_dist"][0]["value"].get<float>(), 22.0f, 1e-5f);
}

TEST(RenderConfigFamilyLineSwitchTest, OperatorEq_ComparesEachOfTheThree) {
  // One case per field rather than one flipping all three: a comparison that reads the same member
  // three times, or that reads only one of them, passes the combined form.
  {
    auto a = MakeBaseline();
    auto b = MakeBaseline();
    ASSERT_TRUE(a == b);
    b.elevation_grid_line_ = false;
    EXPECT_FALSE(a == b) << "elevation_grid_line";
  }
  {
    auto a = MakeBaseline();
    auto b = MakeBaseline();
    b.longitude_grid_line_ = false;
    EXPECT_FALSE(a == b) << "longitude_grid_line";
  }
  {
    auto a = MakeBaseline();
    auto b = MakeBaseline();
    b.angular_dist_grid_line_ = false;
    EXPECT_FALSE(a == b) << "angular_dist_grid_line";
  }
  {
    auto a = MakeBaseline();
    auto b = MakeBaseline();
    b.view_dist_grid_line_ = false;
    EXPECT_FALSE(a == b) << "view_dist_grid_line";
  }
  {
    auto a = MakeBaseline();
    auto b = MakeBaseline();
    b.view_dist_label_ = true;
    EXPECT_FALSE(a == b) << "view_dist_label";
  }
  {
    auto a = MakeBaseline();
    auto b = MakeBaseline();
    b.view_dist_grid_.push_back(lumice::GridLineParam{ 35.0f, 1.0f, 1.0f, { 1, 1, 1 } });
    EXPECT_FALSE(a == b) << "view_dist_grid";
  }
}

}  // namespace
