#include <gtest/gtest.h>

#include <cstddef>
#include <fstream>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "config/config_manager.hpp"
#include "config/crystal_config.hpp"
#include "config/filter_config.hpp"
#include "config/light_config.hpp"
#include "config/proj_config.hpp"
#include "config/render_config.hpp"
#include "core/def.hpp"
#include "core/math.hpp"
#include "util/illuminant.hpp"

extern std::string config_file_name;
using namespace lumice;

namespace {

class V3TestJson : public ::testing::Test {
 protected:
  void SetUp() override {
    std::ifstream f(config_file_name);
    f >> config_json_;
  }

  nlohmann::json config_json_;
};

// =============== LightSource ===============
TEST_F(V3TestJson, LightSource_Sun) {
  const auto& j_light = config_json_.at("scene").at("light_source");
  auto s = j_light.get<LightSourceConfig>();

  ASSERT_TRUE(std::holds_alternative<std::vector<WlParam>>(s.spectrum_));
  const auto& wl_params = std::get<std::vector<WlParam>>(s.spectrum_);
  ASSERT_EQ(wl_params.size(), 1);
  ASSERT_NEAR(wl_params[0].wl_, 550.0f, 1e-5);
  ASSERT_NEAR(wl_params[0].weight_, 1.0f, 1e-5);

  const auto& p = s.param_;

  ASSERT_NEAR(p.azimuth_, 0.0f, 1e-5);
  ASSERT_NEAR(p.altitude_, 20.0f, 1e-5);
  ASSERT_NEAR(p.diameter_, 0.5f, 1e-5);
}

TEST(LightSourceSpectrum, IlluminantParsing) {
  // Test all supported illuminant types
  struct TestCase {
    const char* name;
    IlluminantType expected;
  };
  TestCase cases[] = {
    { "D50", IlluminantType::kD50 }, { "D55", IlluminantType::kD55 }, { "D65", IlluminantType::kD65 },
    { "D75", IlluminantType::kD75 }, { "A", IlluminantType::kA },     { "E", IlluminantType::kE },
  };

  for (const auto& tc : cases) {
    nlohmann::json j = {
      { "type", "sun" },
      { "altitude", 20.0 },
      { "spectrum", tc.name },
    };
    auto s = j.get<LightSourceConfig>();
    ASSERT_TRUE(std::holds_alternative<IlluminantType>(s.spectrum_)) << "Failed for: " << tc.name;
    ASSERT_EQ(std::get<IlluminantType>(s.spectrum_), tc.expected) << "Failed for: " << tc.name;
  }
}

TEST(LightSourceSpectrum, DiscreteRoundTrip) {
  nlohmann::json j_in = {
    { "type", "sun" },
    { "altitude", 20.0 },
    { "spectrum", nlohmann::json::array(
                      { { { "wavelength", 420 }, { "weight", 0.5 } }, { { "wavelength", 550 }, { "weight", 1.0 } } }) },
  };
  auto s = j_in.get<LightSourceConfig>();
  nlohmann::json j_out;
  to_json(j_out, s);

  ASSERT_TRUE(j_out["spectrum"].is_array());
  ASSERT_EQ(j_out["spectrum"].size(), 2);
  ASSERT_NEAR(j_out["spectrum"][0]["wavelength"].get<float>(), 420.0f, 1e-5);
  ASSERT_NEAR(j_out["spectrum"][0]["weight"].get<float>(), 0.5f, 1e-5);
  ASSERT_NEAR(j_out["spectrum"][1]["wavelength"].get<float>(), 550.0f, 1e-5);
  ASSERT_NEAR(j_out["spectrum"][1]["weight"].get<float>(), 1.0f, 1e-5);
}

TEST(LightSourceSpectrum, IlluminantRoundTrip) {
  nlohmann::json j_in = {
    { "type", "sun" },
    { "altitude", 20.0 },
    { "spectrum", "D65" },
  };
  auto s = j_in.get<LightSourceConfig>();
  nlohmann::json j_out;
  to_json(j_out, s);

  ASSERT_TRUE(j_out["spectrum"].is_string());
  ASSERT_EQ(j_out["spectrum"].get<std::string>(), "D65");
}


// =============== SPD Query ===============
TEST(IlluminantSpd, D65At560nm) {
  // D65 is normalized to 100.0 at 560nm (S0[52]=100, S1[52]=0, S2[52]=0)
  float spd = GetIlluminantSpd(IlluminantType::kD65, 560.0f);
  ASSERT_NEAR(spd, 100.0f, 0.5f);
}

TEST(IlluminantSpd, D50At560nm) {
  float spd = GetIlluminantSpd(IlluminantType::kD50, 560.0f);
  // At 560nm, S0=100, S1=0, S2=0, so SPD = 100 regardless of M1, M2
  ASSERT_NEAR(spd, 100.0f, 0.5f);
}

TEST(IlluminantSpd, AAt560nm) {
  // Illuminant A is normalized to 100.0 at 560nm by formula
  float spd = GetIlluminantSpd(IlluminantType::kA, 560.0f);
  ASSERT_NEAR(spd, 100.0f, 0.1f);
}

TEST(IlluminantSpd, EConstant) {
  // E illuminant has constant SPD = 1.0
  ASSERT_NEAR(GetIlluminantSpd(IlluminantType::kE, 380.0f), 1.0f, 1e-5);
  ASSERT_NEAR(GetIlluminantSpd(IlluminantType::kE, 550.0f), 1.0f, 1e-5);
  ASSERT_NEAR(GetIlluminantSpd(IlluminantType::kE, 780.0f), 1.0f, 1e-5);
}

TEST(IlluminantSpd, OutOfRange) {
  // Out of range wavelengths should return 0.0
  ASSERT_NEAR(GetIlluminantSpd(IlluminantType::kD65, 200.0f), 0.0f, 1e-5);
  ASSERT_NEAR(GetIlluminantSpd(IlluminantType::kD65, 900.0f), 0.0f, 1e-5);
  ASSERT_NEAR(GetIlluminantSpd(IlluminantType::kA, 200.0f), 0.0f, 1e-5);
  ASSERT_NEAR(GetIlluminantSpd(IlluminantType::kE, 200.0f), 0.0f, 1e-5);
}

TEST(IlluminantSpd, DSeriesPositive) {
  // All D-series should produce positive SPD in visible range
  IlluminantType types[] = { IlluminantType::kD50, IlluminantType::kD55, IlluminantType::kD65, IlluminantType::kD75 };
  for (auto type : types) {
    for (float wl = 380.0f; wl <= 780.0f; wl += 10.0f) {
      ASSERT_GT(GetIlluminantSpd(type, wl), 0.0f) << "type=" << static_cast<int>(type) << " wl=" << wl;
    }
  }
}


// =============== Crystal ===============
// Type-erased on purpose — the same macro checks kNoRandom / kUniform / kGaussian distributions
// below, so it reads the generic center/spread slots rather than any named accessor.
#define CHECK_DISTRIBUTION(d, t, c, s) \
  do {                                 \
    ASSERT_EQ(d.type, t);              \
    ASSERT_NEAR(d.center, c, 1e-5);    \
    ASSERT_NEAR(d.spread, s, 1e-5);    \
  } while (false)

TEST_F(V3TestJson, Distribution) {
  const auto& j_height = config_json_.at("crystal")[1].at("shape").at("height");
  auto d = j_height.get<Distribution>();

  CHECK_DISTRIBUTION(d, DistributionType::kGaussian, 1.3, 0.2);
}

TEST_F(V3TestJson, Crystal_PrismSimple) {
  const auto& j_crystal = config_json_.at("crystal")[1];
  auto c = j_crystal.get<CrystalConfig>();

  ASSERT_EQ(c.id_, 2);
  ASSERT_TRUE(std::holds_alternative<PrismCrystalParam>(c.param_));

  const auto& p = std::get<PrismCrystalParam>(c.param_);
  CHECK_DISTRIBUTION(p.h_, DistributionType::kGaussian, 1.3, 0.2);
  for (const auto& x : p.d_) {
    CHECK_DISTRIBUTION(x, DistributionType::kNoRandom, 1.0f, 0.0f);
  }
}

TEST_F(V3TestJson, Crystal_PrismFull) {
  const auto& j_crystal = config_json_.at("crystal")[3];
  auto c = j_crystal.get<CrystalConfig>();

  ASSERT_EQ(c.id_, 4);
  ASSERT_TRUE(std::holds_alternative<PrismCrystalParam>(c.param_));

  const auto& p = std::get<PrismCrystalParam>(c.param_);
  CHECK_DISTRIBUTION(p.h_, DistributionType::kUniform, 0.5f, 0.4f);
  for (const auto& x : p.d_) {
    CHECK_DISTRIBUTION(x, DistributionType::kGaussian, 1.0f, 0.2f);
  }

  const auto& axis = c.axis_;
  CHECK_DISTRIBUTION(axis.azimuth_dist, DistributionType::kUniform, 0, 360);
  CHECK_DISTRIBUTION(axis.latitude_dist, DistributionType::kGaussian, 90, 1.2);
  CHECK_DISTRIBUTION(axis.azimuth_dist, DistributionType::kUniform, 0, 360);
}

TEST_F(V3TestJson, Crystal_PyramidSimple) {
  const auto& j_crystal = config_json_.at("crystal")[4];
  auto c = j_crystal.get<CrystalConfig>();

  ASSERT_EQ(c.id_, 5);
  ASSERT_TRUE(std::holds_alternative<PyramidCrystalParam>(c.param_));

  const auto& p = std::get<PyramidCrystalParam>(c.param_);
  CHECK_DISTRIBUTION(p.h_pyr_u_, DistributionType::kNoRandom, 0.1, 0);
  CHECK_DISTRIBUTION(p.h_pyr_l_, DistributionType::kNoRandom, 0.5, 0);
  CHECK_DISTRIBUTION(p.h_prs_, DistributionType::kNoRandom, 1.2, 0);

  // {2,0,3} → alpha ≈ 38.57°; {1,0,1} → alpha ≈ 28.00°
  EXPECT_NEAR(p.wedge_angle_u_, 38.57f, 0.01f);
  EXPECT_NEAR(p.wedge_angle_l_, 28.00f, 0.01f);

  for (const auto& x : p.d_) {
    CHECK_DISTRIBUTION(x, DistributionType::kNoRandom, 1.0f, 0.0f);
  }

  const auto& axis = c.axis_;
  CHECK_DISTRIBUTION(axis.azimuth_dist, DistributionType::kUniform, 0, 360);
  CHECK_DISTRIBUTION(axis.latitude_dist, DistributionType::kNoRandom, 90, 0);
  CHECK_DISTRIBUTION(axis.azimuth_dist, DistributionType::kUniform, 0, 360);
}


// An `axis` slot written as an object must name its `type`. It used to be optional, and the
// answer it silently produced depended on which slot you were in: `zenith` kept the constructor's
// kNoRandom and threw "std" away, `azimuth` / `roll` kept the uniform-360 seed and threw away what
// "mean" was asking for. All three slots are pinned separately for exactly that reason — the three
// wrong answers were different, so one case could not have covered the others.
//
// Asserting the message, not merely the throw: this narrows a published contract, and a config
// written outside this repo can only be migrated by what the error tells its author. Crystal 3 of
// the fixture carries all three slots in object form.
TEST_F(V3TestJson, Crystal_AxisSlotObjectMissingTypeRejected) {
  for (const char* slot : { "zenith", "azimuth", "roll" }) {
    SCOPED_TRACE(slot);
    auto j = config_json_;
    auto& j_axis = j.at("crystal")[2].at("axis");
    ASSERT_TRUE(j_axis.at(slot).is_object()) << "fixture must write this slot in object form";
    j_axis.at(slot).erase("type");

    try {
      (void)j.get<ConfigManager>();
      FAIL() << "expected std::invalid_argument for an axis." << slot << " object with no \"type\"";
    } catch (const std::invalid_argument& e) {
      const std::string msg = e.what();
      EXPECT_NE(msg.find("crystal[id=3]"), std::string::npos) << msg;              // which crystal
      EXPECT_NE(msg.find(std::string("axis.") + slot), std::string::npos) << msg;  // which slot
      EXPECT_NE(msg.find("\"type\""), std::string::npos) << msg;                   // what is missing
      // Both legal ways to write it — the migration guidance, not decoration.
      EXPECT_NE(msg.find(std::string("\"") + slot + "\": 20"), std::string::npos) << msg;
      EXPECT_NE(msg.find("\"type\": \"gauss\""), std::string::npos) << msg;
    }
  }
}

// A present `axis` has always required `zenith`; only the message changes. It used to come from
// nlohmann (`[json.exception.out_of_range.403] key 'zenith' not found`), which names neither the
// crystal nor a way forward.
TEST_F(V3TestJson, Crystal_AxisWithoutZenithRejectedWithOwnMessage) {
  auto j = config_json_;
  auto& j_axis = j.at("crystal")[2].at("axis");
  ASSERT_TRUE(j_axis.contains("zenith"));
  j_axis.erase("zenith");

  try {
    (void)j.get<ConfigManager>();
    FAIL() << "expected std::invalid_argument for an `axis` object with no \"zenith\"";
  } catch (const nlohmann::json::out_of_range& e) {
    FAIL() << "still nlohmann's raw key-not-found, which is what this test exists to replace: " << e.what();
  } catch (const std::invalid_argument& e) {
    const std::string msg = e.what();
    EXPECT_NE(msg.find("crystal[id=3]"), std::string::npos) << msg;
    EXPECT_NE(msg.find("zenith"), std::string::npos) << msg;
    // Says how to opt out of `axis` altogether, since that — not a zenith value — is what an
    // author who wrote a partial `axis` most likely wanted.
    EXPECT_NE(msg.find("omit `axis`"), std::string::npos) << msg;
    EXPECT_NE(msg.find("\"type\": \"gauss\""), std::string::npos) << msg;
  }
}

// The seeds that answer "azimuth / roll KEY absent" are deliberate (2022) and 47 slots in this
// repo's corpus rely on them. Deleting the accidental second job they used to do — standing in as
// the `type` of a slot that WAS written but left typeless — must not touch them. Positive
// evidence, because "we did not edit those lines" is not evidence.
TEST_F(V3TestJson, Crystal_AxisKeysAbsentStillSeedUniform360) {
  auto j = config_json_;
  auto& j_axis = j.at("crystal")[2].at("axis");
  j_axis.erase("azimuth");
  j_axis.erase("roll");

  auto m = j.get<ConfigManager>();
  const auto& axis = m.crystals_.at(3).axis_;
  CHECK_DISTRIBUTION(axis.azimuth_dist, DistributionType::kUniform, 0, 360);
  CHECK_DISTRIBUTION(axis.roll_dist, DistributionType::kUniform, 0, 360);
}

// Helper to create a minimal pyramid CrystalConfig JSON
nlohmann::json MakePyramidJson(int id, std::array<int, 3> upper_idx, std::array<int, 3> lower_idx) {
  return {
    { "id", id },
    { "type", "pyramid" },
    { "shape",
      { { "prism_h", 1.0 },
        { "upper_h", 0.3 },
        { "lower_h", 0.3 },
        { "upper_indices", upper_idx },
        { "lower_indices", lower_idx } } },
  };
}

TEST(MillerIndexFallback, EquivalentIndicesProduceSameAngle) {
  // {2,0,2} and {1,0,1} should produce the same wedge angle via Miller→alpha conversion
  auto j1 = MakePyramidJson(1, { 2, 0, 2 }, { 4, 0, 2 });
  auto c1 = j1.get<CrystalConfig>();
  const auto& p1 = std::get<PyramidCrystalParam>(c1.param_);

  auto j2 = MakePyramidJson(1, { 1, 0, 1 }, { 2, 0, 1 });
  auto c2 = j2.get<CrystalConfig>();
  const auto& p2 = std::get<PyramidCrystalParam>(c2.param_);

  EXPECT_NEAR(p1.wedge_angle_u_, p2.wedge_angle_u_, 0.01f);  // Both → {1,0,1} → 28.0°
  EXPECT_NEAR(p1.wedge_angle_l_, p2.wedge_angle_l_, 0.01f);  // {4,0,2}→{2,0,1}→14.9°, {2,0,1}→14.9°
}

TEST(MillerIndexFallback, IrreducibleIndicesConvertCorrectly) {
  auto j = MakePyramidJson(1, { 2, 0, 3 }, { 1, 0, 1 });
  auto c = j.get<CrystalConfig>();
  const auto& p = std::get<PyramidCrystalParam>(c.param_);

  EXPECT_NEAR(p.wedge_angle_u_, 38.57f, 0.01f);  // {2,0,3} → 38.57°
  EXPECT_NEAR(p.wedge_angle_l_, 28.00f, 0.01f);  // {1,0,1} → 28.00°
}

TEST(FaceDistanceRoundTrip, NoScalingApplied) {
  // Parse once: face_distance values should stay unchanged (no kSqrt3_4 scaling)
  auto j1 = MakePyramidJson(1, { 2, 0, 2 }, { 1, 0, 1 });
  j1["shape"]["face_distance"] = { 1.0, 0.8, 1.0, 1.2, 1.0, 0.9 };

  auto c1 = j1.get<CrystalConfig>();
  const auto& p1 = std::get<PyramidCrystalParam>(c1.param_);

  // Verify no scaling applied. Value() also pins that these stay kNoRandom face distances.
  EXPECT_NEAR(p1.d_[0].Value(), 1.0f, 1e-5f);
  EXPECT_NEAR(p1.d_[1].Value(), 0.8f, 1e-5f);
  EXPECT_NEAR(p1.d_[3].Value(), 1.2f, 1e-5f);
  EXPECT_NEAR(p1.d_[5].Value(), 0.9f, 1e-5f);

  // Verify wedge angle: {2,0,2} → same as {1,0,1} → 28.00°
  EXPECT_NEAR(p1.wedge_angle_u_, 28.00f, 0.01f);

  // Round-trip: parse same config again — should get identical values
  auto j2 = MakePyramidJson(1, { 1, 0, 1 }, { 1, 0, 1 });
  j2["shape"]["face_distance"] = { 1.0, 0.8, 1.0, 1.2, 1.0, 0.9 };

  auto c2 = j2.get<CrystalConfig>();
  const auto& p2 = std::get<PyramidCrystalParam>(c2.param_);

  for (int i = 0; i < 6; i++) {
    EXPECT_NEAR(p2.d_[i].Value(), p1.d_[i].Value(), 1e-5f);
  }
  EXPECT_NEAR(p2.wedge_angle_u_, p1.wedge_angle_u_, 1e-5f);
  EXPECT_NEAR(p2.wedge_angle_l_, p1.wedge_angle_l_, 1e-5f);
}


// =============== Filter ===============
TEST_F(V3TestJson, Filter_None) {
  const auto& j_filter = config_json_.at("filter")[0];
  auto f = j_filter.get<FilterConfig>();

  ASSERT_EQ(f.id_, 1);
  const auto& sp = std::get<SimpleFilterParam>(f.param_);
  ASSERT_TRUE(std::holds_alternative<NoneFilterParam>(sp));
  ASSERT_EQ(f.action_, FilterConfig::kFilterIn);
  ASSERT_EQ(f.symmetry_, FilterConfig::kSymNone);
}

TEST_F(V3TestJson, Filter_Raypath) {
  const auto& j_filter = config_json_.at("filter")[1];
  auto f = j_filter.get<FilterConfig>();

  ASSERT_EQ(f.id_, 2);
  const auto& sp = std::get<SimpleFilterParam>(f.param_);
  ASSERT_TRUE(std::holds_alternative<RaypathFilterParam>(sp));
  ASSERT_EQ(f.action_, FilterConfig::kFilterIn);
  ASSERT_EQ(f.symmetry_, FilterConfig::kSymP | FilterConfig::kSymB | FilterConfig::kSymD);

  const auto& p = std::get<RaypathFilterParam>(sp);
  std::vector<IdType> expect_raypath = { 3, 1, 5, 7, 4 };
  ASSERT_EQ(p.raypath_, expect_raypath);
}

TEST_F(V3TestJson, Filter_EntryExit) {
  const auto& j_filter = config_json_.at("filter")[3];
  auto f = j_filter.get<FilterConfig>();

  ASSERT_EQ(f.id_, 4);
  const auto& sp = std::get<SimpleFilterParam>(f.param_);
  ASSERT_TRUE(std::holds_alternative<EntryExitFilterParam>(sp));
  ASSERT_EQ(f.action_, FilterConfig::kFilterIn);
  ASSERT_EQ(f.symmetry_, FilterConfig::kSymNone);

  const auto& p = std::get<EntryExitFilterParam>(sp);
  ASSERT_EQ(p.entry_, 3);
  ASSERT_EQ(p.exit_, 5);
}

TEST_F(V3TestJson, Filter_Direction) {
  const auto& j_filter = config_json_.at("filter")[4];
  auto f = j_filter.get<FilterConfig>();

  ASSERT_EQ(f.id_, 5);
  const auto& sp = std::get<SimpleFilterParam>(f.param_);
  ASSERT_TRUE(std::holds_alternative<DirectionFilterParam>(sp));
  ASSERT_EQ(f.action_, FilterConfig::kFilterOut);
  ASSERT_EQ(f.symmetry_, FilterConfig::kSymNone);

  const auto& p = std::get<DirectionFilterParam>(sp);
  ASSERT_NEAR(p.lat_, 25, 1e-5);
  ASSERT_NEAR(p.lon_, 180, 1e-5);
  ASSERT_NEAR(p.radii_, 0.5, 1e-5);
}

TEST_F(V3TestJson, Filter_Crystal) {
  const auto& j_filter = config_json_.at("filter")[5];
  auto f = j_filter.get<FilterConfig>();

  ASSERT_EQ(f.id_, 6);
  const auto& sp = std::get<SimpleFilterParam>(f.param_);
  ASSERT_TRUE(std::holds_alternative<CrystalFilterParam>(sp));
  ASSERT_EQ(f.action_, FilterConfig::kFilterIn);
  ASSERT_EQ(f.symmetry_, FilterConfig::kSymNone);

  const auto& p = std::get<CrystalFilterParam>(sp);
  ASSERT_EQ(p.crystal_id_, 3);
}

TEST_F(V3TestJson, Filter_Complex) {
  auto manager = config_json_.get<ConfigManager>();
  auto f = manager.filters_.at(7);

  ASSERT_EQ(f.id_, 7);
  ASSERT_TRUE(std::holds_alternative<ComplexFilterParam>(f.param_));
  ASSERT_EQ(f.action_, FilterConfig::kFilterIn);
  ASSERT_EQ(f.symmetry_, FilterConfig::kSymNone);

  const auto& p = std::get<ComplexFilterParam>(f.param_);
  std::vector<std::vector<IdType>> expect_composition = { { 1 }, { 2, 6 }, { 5 } };
  ASSERT_EQ(p.filters_.size(), expect_composition.size());
  for (size_t i = 0; i < p.filters_.size(); i++) {
    ASSERT_EQ(p.filters_[i].size(), expect_composition[i].size());
    for (size_t j = 0; j < p.filters_[i].size(); j++) {
      ASSERT_EQ(p.filters_[i][j].first, expect_composition[i][j]);
    }
  }
}


// =============== Scene ===============
TEST_F(V3TestJson, Scene_SingleScattering) {
  auto manager = config_json_.get<ConfigManager>();
  const auto& s = manager.scene_;

  ASSERT_EQ(s.max_hits_, 7);
  ASSERT_EQ(s.ray_num_, 2);
  // Golden config has no `geom_clock` key -> stays at the disabled default,
  // covering the "knob-off / backward-compatible" acceptance criterion.
  ASSERT_EQ(s.geom_clock_, 0u);
  ASSERT_EQ(s.ms_.size(), 1);

  ASSERT_NEAR(s.ms_[0].prob_, 0.0f, 1e-5);
  ASSERT_EQ(s.ms_[0].setting_.size(), 1);
  ASSERT_EQ(s.ms_[0].setting_[0].crystal_.id_, 1);
  ASSERT_NEAR(s.ms_[0].setting_[0].crystal_proportion_, 10, 1e-5);
  ASSERT_EQ(s.ms_[0].setting_[0].filter_.id_, kInvalidId);
  const auto& sp = std::get<SimpleFilterParam>(s.ms_[0].setting_[0].filter_.param_);
  ASSERT_TRUE(std::holds_alternative<NoneFilterParam>(sp));
}

// Safe domain for scene.geom_clock is {0} U [1, kGeomClockMax]. Parse both
// endpoints of the accepted range plus rejection of out-of-range values.
TEST_F(V3TestJson, Scene_GeomClockAcceptedValues) {
  for (size_t k : { size_t{ 0 }, size_t{ 1 }, kGeomClockMax }) {
    auto j = config_json_;
    j.at("scene")["geom_clock"] = k;
    auto manager = j.get<ConfigManager>();
    ASSERT_EQ(manager.scene_.geom_clock_, k) << "geom_clock=" << k;
  }
}

TEST_F(V3TestJson, Scene_GeomClockRejectsOutOfRange) {
  {
    auto j = config_json_;
    j.at("scene")["geom_clock"] = kGeomClockMax + 1;  // above upper bound
    EXPECT_THROW(j.get<ConfigManager>(), std::invalid_argument);
  }
  {
    auto j = config_json_;
    j.at("scene")["geom_clock"] = -1;  // negative
    EXPECT_THROW(j.get<ConfigManager>(), std::invalid_argument);
  }
}

// `prob` is required: omitting it used to take MsInfo's value-initialized 0.0f silently, which is
// how the GUI's legacy loader came to fall back to a different value nobody had agreed on. The
// ruling was to delete the implicit default rather than write it down, so there is nothing left
// for a downstream loader to mirror wrongly. Asserting the message content, not just the throw:
// this narrows a published contract, and a hand-written config outside this repo can only be
// migrated by what the error says — the layer index and the old value are the migration guidance.
TEST_F(V3TestJson, Scene_ScatteringMissingProbRejected) {
  auto j = config_json_;
  ASSERT_FALSE(j.at("scene").at("scattering").empty());
  j.at("scene")["scattering"][0].erase("prob");

  try {
    (void)j.get<ConfigManager>();
    FAIL() << "expected std::invalid_argument for a scattering layer with no \"prob\"";
  } catch (const std::invalid_argument& e) {
    const std::string msg = e.what();
    EXPECT_NE(msg.find("scattering[0]"), std::string::npos) << msg;  // names the layer
    EXPECT_NE(msg.find("prob"), std::string::npos) << msg;
    EXPECT_NE(msg.find("0.0"), std::string::npos) << msg;  // states the historical default
  }
}

TEST_F(V3TestJson, Scene_GeomClockRoundTrip) {
  // 0 (default) must not emit the key; a non-zero value must round-trip.
  auto manager = config_json_.get<ConfigManager>();
  ASSERT_EQ(manager.scene_.geom_clock_, 0u);
  nlohmann::json j_off = manager.scene_;
  EXPECT_FALSE(j_off.contains("geom_clock"));

  manager.scene_.geom_clock_ = kGeomClockMax;
  nlohmann::json j_on = manager.scene_;
  ASSERT_TRUE(j_on.contains("geom_clock"));
  EXPECT_EQ(j_on.at("geom_clock").get<size_t>(), kGeomClockMax);
}

// =============== Lens Orthographic ===============

TEST(LensConfigOrthographic, MaxFovIs180) {
  EXPECT_FLOAT_EQ(MaxFov(LensParam::kFisheyeOrthographic), 180.0f);
  EXPECT_FLOAT_EQ(MaxFov(LensParam::kDualFisheyeOrthographic), 180.0f);
}

TEST(LensConfigOrthographic, FovDirectRoundTrip) {
  nlohmann::json j = { { "type", "fisheye_orthographic" }, { "fov", 90.0f } };
  auto l = j.get<LensParam>();
  EXPECT_EQ(l.type_, LensParam::kFisheyeOrthographic);
  EXPECT_NEAR(l.fov_, 90.0f, 1e-5f);

  nlohmann::json out = l;
  EXPECT_EQ(out["type"], "fisheye_orthographic");
  EXPECT_NEAR(out["fov"].get<float>(), 90.0f, 1e-5f);
}

TEST(LensConfigOrthographic, FovDirectBoundary) {
  nlohmann::json ok = { { "type", "fisheye_orthographic" }, { "fov", 180.0f } };
  EXPECT_NO_THROW(ok.get<LensParam>());

  nlohmann::json over = { { "type", "fisheye_orthographic" }, { "fov", 180.01f } };
  EXPECT_THROW(over.get<LensParam>(), nlohmann::detail::out_of_range);

  nlohmann::json zero = { { "type", "fisheye_orthographic" }, { "fov", 0.0f } };
  EXPECT_THROW(zero.get<LensParam>(), nlohmann::detail::out_of_range);
}

TEST(LensConfigOrthographic, DualFovDirectRoundTrip) {
  nlohmann::json j = { { "type", "dual_fisheye_orthographic" }, { "fov", 180.0f } };
  auto l = j.get<LensParam>();
  EXPECT_EQ(l.type_, LensParam::kDualFisheyeOrthographic);
  EXPECT_NEAR(l.fov_, 180.0f, 1e-5f);

  nlohmann::json out = l;
  EXPECT_EQ(out["type"], "dual_fisheye_orthographic");
}

TEST(LensConfigOrthographic, FCalcFovNearBoundary) {
  // Orthographic: sin(fov/2) = d/f, d = kHalfShortEdge = 12mm.
  // Avoid the exact f=12 singularity: asin(1.0) * 2 * kRadToDegree can round to
  // 180 + 1 ULP on some platforms and trip the (fov > MaxFov) guard; the precise
  // 180 ° boundary is already covered by FovDirectBoundary via the direct fov path.
  nlohmann::json j = { { "type", "fisheye_orthographic" }, { "f", 12.5f } };
  auto l = j.get<LensParam>();
  EXPECT_GT(l.fov_, 145.0f);
  EXPECT_LT(l.fov_, 155.0f);
}

TEST(LensConfigOrthographic, FCalcMidRange) {
  // f = 24 -> sin(fov/2) = 12/24 = 0.5 -> fov = 60 deg.
  nlohmann::json j = { { "type", "fisheye_orthographic" }, { "f", 24.0f } };
  auto l = j.get<LensParam>();
  EXPECT_NEAR(l.fov_, 60.0f, 1e-3f);
}

TEST(LensConfigOrthographic, FCalcTooShortThrows) {
  // f = 10 -> d/f = 1.2 > 1, asin domain violation.
  nlohmann::json j = { { "type", "fisheye_orthographic" }, { "f", 10.0f } };
  EXPECT_THROW(j.get<LensParam>(), nlohmann::detail::out_of_range);
}

// =============== Lens Globe ===============

TEST(LensConfigGlobe, MaxFovIs90) {
  EXPECT_FLOAT_EQ(MaxFov(LensParam::kGlobe), 90.0f);
}

TEST(LensConfigGlobe, FovDirectRoundTrip) {
  nlohmann::json j = { { "type", "globe" }, { "fov", 30.0f } };
  auto l = j.get<LensParam>();
  EXPECT_EQ(l.type_, LensParam::kGlobe);
  EXPECT_NEAR(l.fov_, 30.0f, 1e-5f);

  nlohmann::json out = l;
  EXPECT_EQ(out["type"], "globe");
  EXPECT_NEAR(out["fov"].get<float>(), 30.0f, 1e-5f);
}

TEST(LensConfigGlobe, FovDirectBoundary) {
  nlohmann::json ok = { { "type", "globe" }, { "fov", 90.0f } };
  EXPECT_NO_THROW(ok.get<LensParam>());

  nlohmann::json over = { { "type", "globe" }, { "fov", 90.01f } };
  EXPECT_THROW(over.get<LensParam>(), nlohmann::detail::out_of_range);

  nlohmann::json zero = { { "type", "globe" }, { "fov", 0.0f } };
  EXPECT_THROW(zero.get<LensParam>(), nlohmann::detail::out_of_range);
}

TEST(LensConfigGlobe, FFieldMapsLikeLinear) {
  // 315.4: globe is now renderable and its on-image scale uses the linear focal
  // relation (focal = img_radius/tan(fov/2)), so f→fov mirrors the linear lens.
  // f = 12 (= kHalfShortEdge d) → fov = 2*atan2(d, f) = 2*atan(1) = 90°.
  nlohmann::json j = { { "type", "globe" }, { "f", 12.0f } };
  auto l = j.get<LensParam>();
  EXPECT_EQ(l.type_, LensParam::kGlobe);
  EXPECT_NEAR(l.fov_, 90.0f, 1e-3f);

  // Cross-check against the linear lens for the same f.
  nlohmann::json jl = { { "type", "linear" }, { "f", 12.0f } };
  auto ll = jl.get<LensParam>();
  EXPECT_NEAR(l.fov_, ll.fov_, 1e-5f);
}


// =============== Crystal schema key names ===============
// Every key below is written as a bare string literal ON PURPOSE. These tests pin the wire format
// against the file corpus in the wild, so their expectations must be independent of the functions
// under test: expressing them as ShapeScalarSyncKeyName(...) would make the authority table and its
// own test drift together and pass forever. That is also what makes them the red-state criterion —
// renaming a key in core turns these red even though core would still be self-consistent, which is
// the one class of drift a parser-vs-parser differential test cannot see.
//
// The sync_group SUB-map keys are pinned the same way in test_crystal_sync_group.cpp; these cover
// the top-level `shape` object and the `axis` object.

TEST(CrystalSchemaKeyNames, PrismToJsonEmitsTheDocumentedKeys) {
  PrismCrystalParam p;
  p.h_ = Distribution{ DistributionType::kGaussian, 1.3f, 0.2f };
  for (int i = 0; i < 6; i++) {
    p.d_[i] = Distribution{ DistributionType::kNoRandom, 0.7f + 0.1f * i, 0.0f };
  }

  nlohmann::json j = p;

  ASSERT_TRUE(j.contains("height"));
  EXPECT_EQ(j["height"]["mean"].get<float>(), 1.3f);
  EXPECT_EQ(j["height"]["std"].get<float>(), 0.2f);
  ASSERT_TRUE(j.contains("face_distance"));
  ASSERT_EQ(j["face_distance"].size(), 6u);
  EXPECT_NEAR(j["face_distance"][0].get<float>(), 0.7f, 1e-5f);
  EXPECT_NEAR(j["face_distance"][5].get<float>(), 1.2f, 1e-5f);

  // The pyramid-only keys must be ABSENT, not merely defaulted. This pins the type branch rather
  // than just the spelling: a to_json that emitted every key for every type would satisfy the
  // positive assertions above while putting a wedge angle on a crystal that has no pyramid.
  EXPECT_FALSE(j.contains("prism_h"));
  EXPECT_FALSE(j.contains("upper_h"));
  EXPECT_FALSE(j.contains("lower_h"));
  EXPECT_FALSE(j.contains("upper_wedge_angle"));
  EXPECT_FALSE(j.contains("lower_wedge_angle"));
  EXPECT_FALSE(j.contains("upper_indices"));
  EXPECT_FALSE(j.contains("lower_indices"));
}

TEST(CrystalSchemaKeyNames, PyramidToJsonEmitsTheDocumentedKeys) {
  PyramidCrystalParam p;
  p.h_prs_ = Distribution{ DistributionType::kNoRandom, 1.2f, 0.0f };
  p.h_pyr_u_ = Distribution{ DistributionType::kNoRandom, 0.1f, 0.0f };
  p.h_pyr_l_ = Distribution{ DistributionType::kNoRandom, 0.5f, 0.0f };
  p.wedge_angle_u_ = 38.57f;
  p.wedge_angle_l_ = 14.9f;
  for (auto& d : p.d_) {
    d = Distribution{ DistributionType::kNoRandom, 0.9f, 0.0f };
  }

  nlohmann::json j = p;

  ASSERT_TRUE(j.contains("prism_h"));
  EXPECT_NEAR(j["prism_h"].get<float>(), 1.2f, 1e-5f);
  ASSERT_TRUE(j.contains("upper_h"));
  EXPECT_NEAR(j["upper_h"].get<float>(), 0.1f, 1e-5f);
  ASSERT_TRUE(j.contains("lower_h"));
  EXPECT_NEAR(j["lower_h"].get<float>(), 0.5f, 1e-5f);
  ASSERT_TRUE(j.contains("upper_wedge_angle"));
  EXPECT_NEAR(j["upper_wedge_angle"].get<float>(), 38.57f, 1e-5f);
  ASSERT_TRUE(j.contains("lower_wedge_angle"));
  EXPECT_NEAR(j["lower_wedge_angle"].get<float>(), 14.9f, 1e-5f);
  ASSERT_TRUE(j.contains("face_distance"));
  ASSERT_EQ(j["face_distance"].size(), 6u);
  EXPECT_NEAR(j["face_distance"][3].get<float>(), 0.9f, 1e-5f);

  // Miller indices are a read-only legacy spelling — the write side never emits them.
  EXPECT_FALSE(j.contains("upper_indices"));
  EXPECT_FALSE(j.contains("lower_indices"));
  // And the prism-only height key stays off a pyramid.
  EXPECT_FALSE(j.contains("height"));
}

TEST(CrystalSchemaKeyNames, PyramidFromJsonReadsTheDocumentedKeys) {
  // Explicit angles win over indices, so give both and expect the angles.
  nlohmann::json j = {
    { "prism_h", 1.2f },
    { "upper_h", 0.1f },
    { "lower_h", 0.5f },
    { "upper_wedge_angle", 33.0f },
    { "lower_wedge_angle", 22.0f },
    { "upper_indices", nlohmann::json::array({ 2, 0, 3 }) },
    { "lower_indices", nlohmann::json::array({ 1, 0, 1 }) },
    { "face_distance", nlohmann::json::array({ 0.5f, 0.6f, 0.7f, 0.8f, 0.9f, 1.0f }) },
  };

  auto p = j.get<PyramidCrystalParam>();
  EXPECT_NEAR(p.h_prs_.center, 1.2f, 1e-5f);
  EXPECT_NEAR(p.h_pyr_u_.center, 0.1f, 1e-5f);
  EXPECT_NEAR(p.h_pyr_l_.center, 0.5f, 1e-5f);
  EXPECT_NEAR(p.wedge_angle_u_, 33.0f, 1e-5f);
  EXPECT_NEAR(p.wedge_angle_l_, 22.0f, 1e-5f);
  EXPECT_NEAR(p.d_[0].center, 0.5f, 1e-5f);
  EXPECT_NEAR(p.d_[5].center, 1.0f, 1e-5f);

  // Drop the explicit angles and the Miller keys take over — the fallback reads its own two keys,
  // not the ones it falls back from. Expected angles are the literals MillerIndexFallback pins.
  j.erase("upper_wedge_angle");
  j.erase("lower_wedge_angle");
  auto q = j.get<PyramidCrystalParam>();
  EXPECT_NEAR(q.wedge_angle_u_, 38.57f, 0.01f);  // {2,0,3}
  EXPECT_NEAR(q.wedge_angle_l_, 28.00f, 0.01f);  // {1,0,1}
}

TEST(CrystalSchemaKeyNames, PrismFromJsonReadsTheDocumentedKeys) {
  nlohmann::json j = {
    { "height", 1.7f },
    { "face_distance", nlohmann::json::array({ 0.5f, 0.6f, 0.7f, 0.8f, 0.9f, 1.0f }) },
  };

  auto p = j.get<PrismCrystalParam>();
  EXPECT_NEAR(p.h_.center, 1.7f, 1e-5f);
  EXPECT_NEAR(p.d_[0].center, 0.5f, 1e-5f);
  EXPECT_NEAR(p.d_[5].center, 1.0f, 1e-5f);
}

TEST(CrystalSchemaKeyNames, AxisJsonUsesZenithAzimuthRoll) {
  AxisDistribution axis;
  axis.latitude_dist = Distribution{ DistributionType::kNoRandom, 65.0f, 0.0f };  // zenith 25
  axis.azimuth_dist = Distribution{ DistributionType::kUniform, 10.0f, 40.0f };
  axis.roll_dist = Distribution{ DistributionType::kGaussian, 5.0f, 1.5f };

  nlohmann::json j = axis;

  ASSERT_TRUE(j.contains("zenith"));
  EXPECT_NEAR(j["zenith"].get<float>(), 25.0f, 1e-5f);  // 90 - latitude, the wire's own quantity
  ASSERT_TRUE(j.contains("azimuth"));
  EXPECT_NEAR(j["azimuth"]["mean"].get<float>(), 10.0f, 1e-5f);
  ASSERT_TRUE(j.contains("roll"));
  EXPECT_NEAR(j["roll"]["mean"].get<float>(), 5.0f, 1e-5f);
  EXPECT_NEAR(j["roll"]["std"].get<float>(), 1.5f, 1e-5f);

  // Read side: each of the three keys lands in its own slot, so a swap would show.
  auto back = j.get<AxisDistribution>();
  EXPECT_NEAR(back.latitude_dist.center, 65.0f, 1e-5f);
  EXPECT_NEAR(back.azimuth_dist.center, 10.0f, 1e-5f);
  EXPECT_NEAR(back.azimuth_dist.spread, 40.0f, 1e-5f);
  EXPECT_NEAR(back.roll_dist.center, 5.0f, 1e-5f);
  EXPECT_NEAR(back.roll_dist.spread, 1.5f, 1e-5f);
}

}  // namespace
