// Unit tests for filter subsystem: RaypathFilter, ComplexFilter, symmetry, hash consistency.
#include <gtest/gtest.h>

#include <nlohmann/json.hpp>
#include <set>
#include <vector>

#include "config/filter_config.hpp"
#include "core/crystal.hpp"
#include "core/def.hpp"
#include "core/filter.hpp"
#include "core/raypath.hpp"

using namespace lumice;  // NOLINT(google-build-using-namespace)


class FilterTest : public ::testing::Test {
 protected:
  void SetUp() override { crystal_ = Crystal::CreatePrism(1.0f); }

  static RaySeg MakeRay(const std::vector<IdType>& rp_vec) {
    RaySeg r{};
    r.rp_.Clear();
    for (auto fn : rp_vec) {
      r.rp_ << fn;
    }
    r.state_ = RaySeg::kOutgoing;
    r.fid_ = -1;
    r.w_ = 1.0f;
    return r;
  }

  Crystal crystal_{ Crystal::CreatePrism(1.0f) };
};


// Bug #0: triangle 0 maps to valid face number
TEST_F(FilterTest, Triangle0_FaceMapping) {
  auto fn0 = crystal_.GetFn(0);
  EXPECT_NE(fn0, kInvalidId);
  EXPECT_EQ(fn0, 1);  // top basal face
}


// Bug #1: symmetry JSON round-trip with partial flags
TEST_F(FilterTest, SymmetryRoundTrip_PartialFlags) {
  FilterConfig config{};
  config.id_ = 1;
  config.symmetry_ = FilterConfig::kSymP;  // Only P, not PBD
  config.action_ = FilterConfig::kFilterIn;
  RaypathFilterParam p{};
  p.raypath_ = { 3, 5 };
  config.param_ = SimpleFilterParam{ p };

  nlohmann::json j;
  to_json(j, config);

  // After fix: should be "P" not "PBD"
  EXPECT_EQ(j.at("symmetry").get<std::string>(), "P");

  FilterConfig config2{};
  from_json(j, config2);
  EXPECT_EQ(config2.symmetry_, FilterConfig::kSymP);
}


TEST_F(FilterTest, SymmetryRoundTrip_None) {
  FilterConfig config{};
  config.id_ = 1;
  config.symmetry_ = FilterConfig::kSymNone;
  config.action_ = FilterConfig::kFilterIn;
  config.param_ = SimpleFilterParam{ NoneFilterParam{} };

  nlohmann::json j;
  to_json(j, config);

  // kSymNone: symmetry string should be empty
  EXPECT_EQ(j.at("symmetry").get<std::string>(), "");

  FilterConfig config2{};
  from_json(j, config2);
  EXPECT_EQ(config2.symmetry_, FilterConfig::kSymNone);
}


// RaypathFilter: no symmetry — exact match only
TEST_F(FilterTest, RaypathFilter_NoSymmetry) {
  FilterConfig config{};
  config.id_ = 1;
  config.action_ = FilterConfig::kFilterIn;
  config.symmetry_ = FilterConfig::kSymNone;
  RaypathFilterParam p{};
  p.raypath_ = { 3, 5 };
  config.param_ = SimpleFilterParam{ p };

  auto filter = Filter::Create(config);
  filter->InitCrystalSymmetry(crystal_, config.symmetry_);

  EXPECT_TRUE(filter->Check(MakeRay({ 3, 5 })));
  EXPECT_FALSE(filter->Check(MakeRay({ 4, 6 })));
  EXPECT_FALSE(filter->Check(MakeRay({ 3, 7 })));
  EXPECT_FALSE(filter->Check(MakeRay({ 3, 1, 5 })));
  EXPECT_FALSE(filter->Check(MakeRay({ 3 })));
  EXPECT_FALSE(filter->Check(MakeRay({})));
}


// RaypathFilter: P symmetry — 6 rotational variants
TEST_F(FilterTest, RaypathFilter_PSymmetry) {
  FilterConfig config{};
  config.id_ = 1;
  config.action_ = FilterConfig::kFilterIn;
  config.symmetry_ = FilterConfig::kSymP;
  RaypathFilterParam p{};
  p.raypath_ = { 3, 5 };
  config.param_ = SimpleFilterParam{ p };

  auto filter = Filter::Create(config);
  filter->InitCrystalSymmetry(crystal_, config.symmetry_);

  EXPECT_TRUE(filter->Check(MakeRay({ 3, 5 })));
  EXPECT_TRUE(filter->Check(MakeRay({ 4, 6 })));
  EXPECT_TRUE(filter->Check(MakeRay({ 5, 7 })));
  EXPECT_TRUE(filter->Check(MakeRay({ 6, 8 })));
  EXPECT_TRUE(filter->Check(MakeRay({ 7, 3 })));
  EXPECT_TRUE(filter->Check(MakeRay({ 8, 4 })));

  EXPECT_FALSE(filter->Check(MakeRay({ 3, 7 })));  // D-mirror, no D sym
  EXPECT_FALSE(filter->Check(MakeRay({ 3, 4 })));
}


// RaypathFilter: PBD symmetry — 12 variants
TEST_F(FilterTest, RaypathFilter_PBDSymmetry) {
  FilterConfig config{};
  config.id_ = 1;
  config.action_ = FilterConfig::kFilterIn;
  config.symmetry_ = FilterConfig::kSymP | FilterConfig::kSymB | FilterConfig::kSymD;
  RaypathFilterParam p{};
  p.raypath_ = { 3, 5 };
  config.param_ = SimpleFilterParam{ p };

  auto filter = Filter::Create(config);
  filter->InitCrystalSymmetry(crystal_, config.symmetry_);

  // Count accepted 2-element prism paths
  int accepted = 0;
  for (IdType a = 3; a <= 8; a++) {
    for (IdType b = 3; b <= 8; b++) {
      if (a == b) {
        continue;
      }
      if (filter->Check(MakeRay({ a, b }))) {
        accepted++;
      }
    }
  }
  EXPECT_EQ(accepted, 12);

  // Non-matching paths
  EXPECT_FALSE(filter->Check(MakeRay({ 3, 1 })));
  EXPECT_FALSE(filter->Check(MakeRay({ 3, 5, 3 })));
}


// ExpandRaypath: verify unique hashes
TEST_F(FilterTest, ExpandRaypath_UniqueHashes) {
  auto expanded = crystal_.ExpandRaypath({ 3, 5 }, FilterConfig::kSymP | FilterConfig::kSymB | FilterConfig::kSymD);
  EXPECT_EQ(expanded.size(), 12u);

  RaypathHash h;
  std::set<size_t> hashes;
  for (const auto& rp : expanded) {
    EXPECT_TRUE(hashes.insert(h(rp)).second) << "Duplicate hash in ExpandRaypath output";
  }
}


// Hash consistency: vector<IdType> vs RaypathRecorder
TEST_F(FilterTest, HashConsistency) {
  RaypathHash h;

  std::vector<IdType> vec{ 3, 5 };
  RaypathRecorder rec;
  rec.Clear();
  rec << 3 << 5;
  EXPECT_EQ(h(vec), h(rec));

  std::vector<IdType> vec2{ 3, 1, 5, 7, 4 };
  RaypathRecorder rec2;
  rec2.Clear();
  for (auto x : vec2) {
    rec2 << x;
  }
  EXPECT_EQ(h(vec2), h(rec2));
}


// ComplexFilter: InitCrystalSymmetry propagation
TEST_F(FilterTest, ComplexFilter_Propagation) {
  // Create a ComplexFilter with one OR group containing a single RaypathFilter
  RaypathFilterParam rp_param{};
  rp_param.raypath_ = { 3, 5 };

  ComplexFilterParam cp{};
  std::vector<std::pair<IdType, SimpleFilterParam>> and_group;
  and_group.emplace_back(1, SimpleFilterParam{ rp_param });
  cp.filters_.push_back(and_group);

  FilterConfig config{};
  config.id_ = 10;
  config.action_ = FilterConfig::kFilterIn;
  config.symmetry_ = FilterConfig::kSymP;
  config.param_ = cp;

  auto filter = Filter::Create(config);
  filter->InitCrystalSymmetry(crystal_, config.symmetry_);

  // Should match {3,5} and P-symmetric variants
  EXPECT_TRUE(filter->Check(MakeRay({ 3, 5 })));
  EXPECT_TRUE(filter->Check(MakeRay({ 4, 6 })));

  // Should NOT match non-P variants or different paths
  EXPECT_FALSE(filter->Check(MakeRay({ 3, 7 })));
  EXPECT_FALSE(filter->Check(MakeRay({ 3, 1 })));
}
