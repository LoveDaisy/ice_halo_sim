// Unit tests for filter subsystem: RaypathFilter, ComplexFilter, symmetry, hash consistency,
// and raypath text validation.
#include <gtest/gtest.h>

#include <nlohmann/json.hpp>
#include <set>
#include <vector>

#include "config/filter_config.hpp"
#include "config/raypath_validation.hpp"
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


// DirectionFilter: basic direction matching
TEST_F(FilterTest, DirectionFilter_BasicMatch) {
  // Target direction: azimuth=0, elevation=0 → d_ = (1, 0, 0), radii=10°
  DirectionFilterParam dp{};
  dp.lon_ = 0.0f;
  dp.lat_ = 0.0f;
  dp.radii_ = 10.0f;

  FilterConfig config{};
  config.id_ = 1;
  config.action_ = FilterConfig::kFilterIn;
  config.param_ = SimpleFilterParam{ dp };

  auto filter = Filter::Create(config);

  // Ray along (1, 0, 0) — exactly matching
  RaySeg r1{};
  r1.d_[0] = 1.0f;
  r1.d_[1] = 0.0f;
  r1.d_[2] = 0.0f;
  r1.state_ = RaySeg::kOutgoing;
  EXPECT_TRUE(filter->Check(r1));

  // Ray along (0, 1, 0) — 90° away, should fail
  RaySeg r2{};
  r2.d_[0] = 0.0f;
  r2.d_[1] = 1.0f;
  r2.d_[2] = 0.0f;
  r2.state_ = RaySeg::kOutgoing;
  EXPECT_FALSE(filter->Check(r2));
}


// DirectionFilter: no state guard — works regardless of state_
TEST_F(FilterTest, DirectionFilter_NoStateGuard) {
  DirectionFilterParam dp{};
  dp.lon_ = 0.0f;
  dp.lat_ = 0.0f;
  dp.radii_ = 10.0f;

  FilterConfig config{};
  config.id_ = 1;
  config.action_ = FilterConfig::kFilterIn;
  config.param_ = SimpleFilterParam{ dp };

  auto filter = Filter::Create(config);

  // Matching direction with various non-kOutgoing states
  RaySeg r{};
  r.d_[0] = 1.0f;
  r.d_[1] = 0.0f;
  r.d_[2] = 0.0f;

  r.state_ = RaySeg::kNormal;
  EXPECT_TRUE(filter->Check(r));

  r.state_ = RaySeg::kContinue;
  EXPECT_TRUE(filter->Check(r));

  r.state_ = RaySeg::kStopped;
  EXPECT_TRUE(filter->Check(r));
}


// DirectionFilter: kFilterOut mode correctly excludes matching directions
TEST_F(FilterTest, DirectionFilter_FilterOut) {
  DirectionFilterParam dp{};
  dp.lon_ = 0.0f;
  dp.lat_ = 0.0f;
  dp.radii_ = 10.0f;

  FilterConfig config{};
  config.id_ = 1;
  config.action_ = FilterConfig::kFilterOut;
  config.param_ = SimpleFilterParam{ dp };

  auto filter = Filter::Create(config);

  // Matching direction → should be excluded (Check returns false)
  RaySeg r1{};
  r1.d_[0] = 1.0f;
  r1.d_[1] = 0.0f;
  r1.d_[2] = 0.0f;
  r1.state_ = RaySeg::kOutgoing;
  EXPECT_FALSE(filter->Check(r1));

  // Non-matching direction → should pass (Check returns true)
  RaySeg r2{};
  r2.d_[0] = 0.0f;
  r2.d_[1] = 1.0f;
  r2.d_[2] = 0.0f;
  r2.state_ = RaySeg::kOutgoing;
  EXPECT_TRUE(filter->Check(r2));

  // kFilterOut + non-kOutgoing state + matching direction → should be excluded
  RaySeg r3{};
  r3.d_[0] = 1.0f;
  r3.d_[1] = 0.0f;
  r3.d_[2] = 0.0f;
  r3.state_ = RaySeg::kContinue;
  EXPECT_FALSE(filter->Check(r3));
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


// ========== ValidateRaypathText Tests ==========

TEST(ValidateRaypathTextTest, EmptyString_IsValid) {
  EXPECT_EQ(ValidateRaypathText(""), RaypathValidation::kValid);
}

TEST(ValidateRaypathTextTest, SingleDigit_IsValid) {
  EXPECT_EQ(ValidateRaypathText("3"), RaypathValidation::kValid);
}

TEST(ValidateRaypathTextTest, MultipleDashSeparated_IsValid) {
  EXPECT_EQ(ValidateRaypathText("3-1-5"), RaypathValidation::kValid);
}

TEST(ValidateRaypathTextTest, CommaSeparated_IsValid) {
  EXPECT_EQ(ValidateRaypathText("3,1,5"), RaypathValidation::kValid);
}

TEST(ValidateRaypathTextTest, MixedSeparators_IsValid) {
  EXPECT_EQ(ValidateRaypathText("3-1,5"), RaypathValidation::kValid);
}

TEST(ValidateRaypathTextTest, Zero_IsValid) {
  EXPECT_EQ(ValidateRaypathText("0"), RaypathValidation::kValid);
}

TEST(ValidateRaypathTextTest, LongPath_IsValid) {
  EXPECT_EQ(ValidateRaypathText("3-1-5-7-4"), RaypathValidation::kValid);
}

TEST(ValidateRaypathTextTest, TrailingDash_IsIncomplete) {
  EXPECT_EQ(ValidateRaypathText("3-5-"), RaypathValidation::kIncomplete);
}

TEST(ValidateRaypathTextTest, SingleTrailingDash_IsIncomplete) {
  EXPECT_EQ(ValidateRaypathText("3-"), RaypathValidation::kIncomplete);
}

TEST(ValidateRaypathTextTest, TrailingComma_IsIncomplete) {
  EXPECT_EQ(ValidateRaypathText("3,"), RaypathValidation::kIncomplete);
}

TEST(ValidateRaypathTextTest, SingleDash_IsIncomplete) {
  EXPECT_EQ(ValidateRaypathText("-"), RaypathValidation::kIncomplete);
}

TEST(ValidateRaypathTextTest, SingleComma_IsIncomplete) {
  EXPECT_EQ(ValidateRaypathText(","), RaypathValidation::kIncomplete);
}

TEST(ValidateRaypathTextTest, LeadingDashSingle_IsIncomplete) {
  EXPECT_EQ(ValidateRaypathText("-3"), RaypathValidation::kIncomplete);
}

TEST(ValidateRaypathTextTest, LeadingDashMultiple_IsIncomplete) {
  EXPECT_EQ(ValidateRaypathText("-3-5"), RaypathValidation::kIncomplete);
}

TEST(ValidateRaypathTextTest, NonNumericToken_IsInvalid) {
  EXPECT_EQ(ValidateRaypathText("3-abc-5"), RaypathValidation::kInvalid);
}

TEST(ValidateRaypathTextTest, PureNonNumeric_IsInvalid) {
  EXPECT_EQ(ValidateRaypathText("abc"), RaypathValidation::kInvalid);
}

TEST(ValidateRaypathTextTest, ConsecutiveDashes_IsInvalid) {
  EXPECT_EQ(ValidateRaypathText("3--5"), RaypathValidation::kInvalid);
}

TEST(ValidateRaypathTextTest, ConsecutiveCommas_IsInvalid) {
  EXPECT_EQ(ValidateRaypathText("3,,5"), RaypathValidation::kInvalid);
}

TEST(ValidateRaypathTextTest, TokenWithSpace_IsInvalid) {
  EXPECT_EQ(ValidateRaypathText("3- -5"), RaypathValidation::kInvalid);
}

TEST(ValidateRaypathTextTest, SpecialChars_IsInvalid) {
  EXPECT_EQ(ValidateRaypathText("3-(-1)-5"), RaypathValidation::kInvalid);
}

TEST(ValidateRaypathTextTest, MultiDigitNumbers_IsValid) {
  EXPECT_EQ(ValidateRaypathText("12-345-6"), RaypathValidation::kValid);
}

TEST(ValidateRaypathTextTest, DoubleLeadingSeparator_IsInvalid) {
  // "--3": second '-' creates an empty interior token → kInvalid (not kIncomplete)
  EXPECT_EQ(ValidateRaypathText("--3"), RaypathValidation::kInvalid);
}


// ========== IsLegalFace Tests (core/crystal.hpp) ==========

TEST(IsLegalFaceTest, Prism_BasalFaces_Legal) {
  EXPECT_TRUE(IsLegalFace(CrystalKind::kPrism, 1));
  EXPECT_TRUE(IsLegalFace(CrystalKind::kPrism, 2));
}

TEST(IsLegalFaceTest, Prism_LateralFaces_Legal) {
  for (int f = 3; f <= 8; ++f) {
    EXPECT_TRUE(IsLegalFace(CrystalKind::kPrism, f)) << "Prism face " << f;
  }
}

TEST(IsLegalFaceTest, Prism_PyramidFaces_Illegal) {
  // Upper/lower pyramidal faces are not legal on a prism.
  for (int f = 13; f <= 18; ++f) {
    EXPECT_FALSE(IsLegalFace(CrystalKind::kPrism, f)) << "Prism upper pyramid face " << f;
  }
  for (int f = 23; f <= 28; ++f) {
    EXPECT_FALSE(IsLegalFace(CrystalKind::kPrism, f)) << "Prism lower pyramid face " << f;
  }
}

TEST(IsLegalFaceTest, Prism_GapAndOutOfRange_Illegal) {
  // Gaps between the legal bands.
  EXPECT_FALSE(IsLegalFace(CrystalKind::kPrism, 0));
  EXPECT_FALSE(IsLegalFace(CrystalKind::kPrism, 9));
  EXPECT_FALSE(IsLegalFace(CrystalKind::kPrism, 10));
  EXPECT_FALSE(IsLegalFace(CrystalKind::kPrism, 12));
  EXPECT_FALSE(IsLegalFace(CrystalKind::kPrism, 19));
  EXPECT_FALSE(IsLegalFace(CrystalKind::kPrism, 22));
  EXPECT_FALSE(IsLegalFace(CrystalKind::kPrism, 29));
  EXPECT_FALSE(IsLegalFace(CrystalKind::kPrism, 51));
  EXPECT_FALSE(IsLegalFace(CrystalKind::kPrism, -1));
}

TEST(IsLegalFaceTest, Pyramid_BasalAndLateralFaces_Legal) {
  EXPECT_TRUE(IsLegalFace(CrystalKind::kPyramid, 1));
  EXPECT_TRUE(IsLegalFace(CrystalKind::kPyramid, 2));
  for (int f = 3; f <= 8; ++f) {
    EXPECT_TRUE(IsLegalFace(CrystalKind::kPyramid, f)) << "Pyramid face " << f;
  }
}

TEST(IsLegalFaceTest, Pyramid_UpperAndLowerPyramidFaces_Legal) {
  for (int f = 13; f <= 18; ++f) {
    EXPECT_TRUE(IsLegalFace(CrystalKind::kPyramid, f)) << "Pyramid upper face " << f;
  }
  for (int f = 23; f <= 28; ++f) {
    EXPECT_TRUE(IsLegalFace(CrystalKind::kPyramid, f)) << "Pyramid lower face " << f;
  }
}

TEST(IsLegalFaceTest, Pyramid_Gaps_Illegal) {
  // Gaps between the legal bands (9-12, 19-22, 29+).
  EXPECT_FALSE(IsLegalFace(CrystalKind::kPyramid, 0));
  EXPECT_FALSE(IsLegalFace(CrystalKind::kPyramid, 9));
  EXPECT_FALSE(IsLegalFace(CrystalKind::kPyramid, 10));
  EXPECT_FALSE(IsLegalFace(CrystalKind::kPyramid, 11));
  EXPECT_FALSE(IsLegalFace(CrystalKind::kPyramid, 12));
  EXPECT_FALSE(IsLegalFace(CrystalKind::kPyramid, 19));
  EXPECT_FALSE(IsLegalFace(CrystalKind::kPyramid, 20));
  EXPECT_FALSE(IsLegalFace(CrystalKind::kPyramid, 22));
  EXPECT_FALSE(IsLegalFace(CrystalKind::kPyramid, 29));
  EXPECT_FALSE(IsLegalFace(CrystalKind::kPyramid, 51));
  EXPECT_FALSE(IsLegalFace(CrystalKind::kPyramid, -1));
}


// ========== ValidateRaypathText (with CrystalKind) Tests ==========
// Note: The single-argument ValidateRaypathText(text) is kept untouched and
// remains syntax-only (so "0" and "51" still return kValid). The richer
// semantics — global-union + kind-specific face-number checks — are confined
// to the two-argument overload tested below. `-` and `,` are token separators
// (no range semantics), so e.g. "3-1-5" is equivalent to "3,1,5".

TEST(ValidateRaypathTextWithKindTest, Prism_BasicLegal) {
  EXPECT_EQ(ValidateRaypathText("1", CrystalKind::kPrism).state, RaypathValidation::kValid);
  EXPECT_EQ(ValidateRaypathText("2", CrystalKind::kPrism).state, RaypathValidation::kValid);
  EXPECT_EQ(ValidateRaypathText("3-8", CrystalKind::kPrism).state, RaypathValidation::kValid);
  EXPECT_EQ(ValidateRaypathText("1,3-5", CrystalKind::kPrism).state, RaypathValidation::kValid);
  EXPECT_EQ(ValidateRaypathText("8", CrystalKind::kPrism).state, RaypathValidation::kValid);  // boundary
}

TEST(ValidateRaypathTextWithKindTest, Pyramid_BasicLegal) {
  EXPECT_EQ(ValidateRaypathText("1", CrystalKind::kPyramid).state, RaypathValidation::kValid);
  EXPECT_EQ(ValidateRaypathText("2", CrystalKind::kPyramid).state, RaypathValidation::kValid);
  EXPECT_EQ(ValidateRaypathText("3-8", CrystalKind::kPyramid).state, RaypathValidation::kValid);
  EXPECT_EQ(ValidateRaypathText("13-18", CrystalKind::kPyramid).state, RaypathValidation::kValid);
  EXPECT_EQ(ValidateRaypathText("23-28", CrystalKind::kPyramid).state, RaypathValidation::kValid);
  EXPECT_EQ(ValidateRaypathText("1,3-5,13", CrystalKind::kPyramid).state, RaypathValidation::kValid);
  EXPECT_EQ(ValidateRaypathText("13", CrystalKind::kPyramid).state, RaypathValidation::kValid);  // boundary
}

TEST(ValidateRaypathTextWithKindTest, SyntaxInvalid_PropagatesInvalid) {
  // "3,,5" is a syntax error (empty interior token) → kInvalid, "Invalid raypath".
  const auto r = ValidateRaypathText("3,,5", CrystalKind::kPrism);
  EXPECT_EQ(r.state, RaypathValidation::kInvalid);
  EXPECT_EQ(r.message, "Invalid raypath");
}

TEST(ValidateRaypathTextWithKindTest, SyntaxIncomplete_TakesPriorityOverFaceCheck) {
  // Trailing dash → kIncomplete; message should be empty even if face would be illegal.
  const auto r = ValidateRaypathText("3-", CrystalKind::kPrism);
  EXPECT_EQ(r.state, RaypathValidation::kIncomplete);
  EXPECT_EQ(r.message, "");
}

TEST(ValidateRaypathTextWithKindTest, GlobalInvalid_OutsideUnion) {
  // 51 is outside every legal band; message mentions "outside" and the face number.
  const auto r51 = ValidateRaypathText("51", CrystalKind::kPrism);
  EXPECT_EQ(r51.state, RaypathValidation::kInvalid);
  EXPECT_NE(r51.message.find("outside"), std::string::npos);
  EXPECT_NE(r51.message.find("51"), std::string::npos);

  // 0 is in the gap (not in {1,2} ∪ {3..8} ∪ ...). Even on pyramid it's outside.
  const auto r0_prism = ValidateRaypathText("0", CrystalKind::kPrism);
  EXPECT_EQ(r0_prism.state, RaypathValidation::kInvalid);
  EXPECT_NE(r0_prism.message.find("outside"), std::string::npos);

  const auto r0_pyr = ValidateRaypathText("0", CrystalKind::kPyramid);
  EXPECT_EQ(r0_pyr.state, RaypathValidation::kInvalid);
  EXPECT_NE(r0_pyr.message.find("outside"), std::string::npos);

  // Other gap values.
  EXPECT_EQ(ValidateRaypathText("12", CrystalKind::kPyramid).state, RaypathValidation::kInvalid);
  EXPECT_EQ(ValidateRaypathText("19-22", CrystalKind::kPyramid).state, RaypathValidation::kInvalid);
  EXPECT_EQ(ValidateRaypathText("29", CrystalKind::kPyramid).state, RaypathValidation::kInvalid);
}

TEST(ValidateRaypathTextWithKindTest, TypeSpecificInvalid_OnPrism) {
  // 13 is in the global union but illegal on a prism.
  const auto r13 = ValidateRaypathText("13", CrystalKind::kPrism);
  EXPECT_EQ(r13.state, RaypathValidation::kInvalid);
  EXPECT_NE(r13.message.find("Prism"), std::string::npos);
  EXPECT_NE(r13.message.find("13"), std::string::npos);
  // Specifically not the "outside" message — it's within the union.
  EXPECT_EQ(r13.message.find("outside"), std::string::npos);

  // Any upper/lower pyramid face is illegal on a prism.
  EXPECT_EQ(ValidateRaypathText("15", CrystalKind::kPrism).state, RaypathValidation::kInvalid);
  EXPECT_EQ(ValidateRaypathText("23", CrystalKind::kPrism).state, RaypathValidation::kInvalid);
  EXPECT_EQ(ValidateRaypathText("28", CrystalKind::kPrism).state, RaypathValidation::kInvalid);
}

TEST(ValidateRaypathTextWithKindTest, FirstInvalidTokenDeterminesMessage) {
  // Sequence "9,13" on pyramid: 9 is outside-union → message references 9, not 13.
  const auto r = ValidateRaypathText("9,13", CrystalKind::kPyramid);
  EXPECT_EQ(r.state, RaypathValidation::kInvalid);
  EXPECT_NE(r.message.find("9"), std::string::npos);
  EXPECT_NE(r.message.find("outside"), std::string::npos);
}

TEST(ValidateRaypathTextWithKindTest, SingleArg_Untouched) {
  // Contract: the single-argument overload still performs syntax-only validation
  // and does NOT reject out-of-range faces. This guards against regressions
  // that would change legacy callers' behaviour.
  EXPECT_EQ(ValidateRaypathText("0"), RaypathValidation::kValid);
  EXPECT_EQ(ValidateRaypathText("51"), RaypathValidation::kValid);
  EXPECT_EQ(ValidateRaypathText("13"), RaypathValidation::kValid);
  EXPECT_EQ(ValidateRaypathText("9,13"), RaypathValidation::kValid);
}

TEST(ValidateRaypathTextWithKindTest, KValidMessageIsEmpty) {
  const auto r = ValidateRaypathText("3-5,1", CrystalKind::kPrism);
  EXPECT_EQ(r.state, RaypathValidation::kValid);
  EXPECT_TRUE(r.message.empty());
}

TEST(ValidateRaypathTextWithKindTest, EmptyText_IsValid) {
  const auto r = ValidateRaypathText("", CrystalKind::kPrism);
  EXPECT_EQ(r.state, RaypathValidation::kValid);
  EXPECT_TRUE(r.message.empty());
}
