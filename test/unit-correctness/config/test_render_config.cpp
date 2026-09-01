#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

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

}  // namespace
