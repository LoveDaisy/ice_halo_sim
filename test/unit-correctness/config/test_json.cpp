#include <gtest/gtest.h>

#include <cstddef>
#include <fstream>
#include <nlohmann/json.hpp>
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
#define CHECK_DISTRIBUTION(d, t, m, s) \
  do {                                 \
    ASSERT_EQ(d.type, t);              \
    ASSERT_NEAR(d.mean, m, 1e-5);      \
    ASSERT_NEAR(d.std, s, 1e-5);       \
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

  // Verify no scaling applied
  EXPECT_NEAR(p1.d_[0].mean, 1.0f, 1e-5f);
  EXPECT_NEAR(p1.d_[1].mean, 0.8f, 1e-5f);
  EXPECT_NEAR(p1.d_[3].mean, 1.2f, 1e-5f);
  EXPECT_NEAR(p1.d_[5].mean, 0.9f, 1e-5f);

  // Verify wedge angle: {2,0,2} → same as {1,0,1} → 28.00°
  EXPECT_NEAR(p1.wedge_angle_u_, 28.00f, 0.01f);

  // Round-trip: parse same config again — should get identical values
  auto j2 = MakePyramidJson(1, { 1, 0, 1 }, { 1, 0, 1 });
  j2["shape"]["face_distance"] = { 1.0, 0.8, 1.0, 1.2, 1.0, 0.9 };

  auto c2 = j2.get<CrystalConfig>();
  const auto& p2 = std::get<PyramidCrystalParam>(c2.param_);

  for (int i = 0; i < 6; i++) {
    EXPECT_NEAR(p2.d_[i].mean, p1.d_[i].mean, 1e-5f);
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
  ASSERT_EQ(s.ms_.size(), 1);

  ASSERT_NEAR(s.ms_[0].prob_, 0.0f, 1e-5);
  ASSERT_EQ(s.ms_[0].setting_.size(), 1);
  ASSERT_EQ(s.ms_[0].setting_[0].crystal_.id_, 1);
  ASSERT_NEAR(s.ms_[0].setting_[0].crystal_proportion_, 10, 1e-5);
  ASSERT_EQ(s.ms_[0].setting_[0].filter_.id_, kInvalidId);
  const auto& sp = std::get<SimpleFilterParam>(s.ms_[0].setting_[0].filter_.param_);
  ASSERT_TRUE(std::holds_alternative<NoneFilterParam>(sp));
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

TEST(LensConfigGlobe, FFieldRejected) {
  // Globe lens does not support f-derived fov.
  nlohmann::json j = { { "type", "globe" }, { "f", 12.0f } };
  EXPECT_THROW(j.get<LensParam>(), nlohmann::detail::out_of_range);
}

}  // namespace
