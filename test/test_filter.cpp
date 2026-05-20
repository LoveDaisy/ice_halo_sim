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
#include "core/math.hpp"
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
    r.from_face_ = kInvalidId;
    r.to_face_ = kInvalidId;
    r.w_ = 1.0f;
    return r;
  }

  // Returns an AxisDistribution suitable for enabling D symmetry (az=uniform-360, roll=0).
  static AxisDistribution DEnablingAxis() {
    AxisDistribution d{};
    d.azimuth_dist.type = DistributionType::kUniform;
    d.azimuth_dist.std = 360.0f;
    d.azimuth_dist.mean = 0.0f;
    d.roll_dist.type = DistributionType::kNoRandom;
    d.roll_dist.mean = 0.0f;
    return d;
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


// ExpandRaypath: verify unique hashes (D enabled via sigma_a=0, d_applicable=true)
TEST_F(FilterTest, ExpandRaypath_UniqueHashes) {
  auto expanded =
      crystal_.ExpandRaypath({ 3, 5 }, FilterConfig::kSymP | FilterConfig::kSymB | FilterConfig::kSymD, 0, true);
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
  auto r = ValidateRaypathText("3,,5", CrystalKind::kPrism);
  EXPECT_EQ(r.state, RaypathValidation::kInvalid);
  EXPECT_EQ(r.message, "Invalid raypath");
}

TEST(ValidateRaypathTextWithKindTest, SyntaxIncomplete_TakesPriorityOverFaceCheck) {
  // Trailing dash → kIncomplete; message should be empty even if face would be illegal.
  auto r = ValidateRaypathText("3-", CrystalKind::kPrism);
  EXPECT_EQ(r.state, RaypathValidation::kIncomplete);
  EXPECT_EQ(r.message, "");
}

TEST(ValidateRaypathTextWithKindTest, GlobalInvalid_OutsideUnion) {
  // 51 is outside every legal band; message mentions "outside" and the face number.
  auto r51 = ValidateRaypathText("51", CrystalKind::kPrism);
  EXPECT_EQ(r51.state, RaypathValidation::kInvalid);
  EXPECT_NE(r51.message.find("outside"), std::string::npos);
  EXPECT_NE(r51.message.find("51"), std::string::npos);

  // 0 is in the gap (not in {1,2} ∪ {3..8} ∪ ...). Even on pyramid it's outside.
  auto r0_prism = ValidateRaypathText("0", CrystalKind::kPrism);
  EXPECT_EQ(r0_prism.state, RaypathValidation::kInvalid);
  EXPECT_NE(r0_prism.message.find("outside"), std::string::npos);

  auto r0_pyr = ValidateRaypathText("0", CrystalKind::kPyramid);
  EXPECT_EQ(r0_pyr.state, RaypathValidation::kInvalid);
  EXPECT_NE(r0_pyr.message.find("outside"), std::string::npos);

  // Other gap values.
  EXPECT_EQ(ValidateRaypathText("12", CrystalKind::kPyramid).state, RaypathValidation::kInvalid);
  EXPECT_EQ(ValidateRaypathText("19-22", CrystalKind::kPyramid).state, RaypathValidation::kInvalid);
  EXPECT_EQ(ValidateRaypathText("29", CrystalKind::kPyramid).state, RaypathValidation::kInvalid);
}

TEST(ValidateRaypathTextWithKindTest, TypeSpecificInvalid_OnPrism) {
  // 13 is in the global union but illegal on a prism.
  auto r13 = ValidateRaypathText("13", CrystalKind::kPrism);
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
  auto r = ValidateRaypathText("9,13", CrystalKind::kPyramid);
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
  auto r = ValidateRaypathText("3-5,1", CrystalKind::kPrism);
  EXPECT_EQ(r.state, RaypathValidation::kValid);
  EXPECT_TRUE(r.message.empty());
}

TEST(ValidateRaypathTextWithKindTest, EmptyText_IsValid) {
  auto r = ValidateRaypathText("", CrystalKind::kPrism);
  EXPECT_EQ(r.state, RaypathValidation::kValid);
  EXPECT_TRUE(r.message.empty());
}

TEST(ValidateRaypathTextWithKindTest, OverlongDigitToken_IsInvalid) {
  // A pathological 10-digit token passes the syntax-only validator (single
  // argument) — there is no length cap there. The two-argument overload
  // must still reject it without invoking int overflow UB in the tokenizer.
  // Values include patterns that, under two's-complement wrap, would map to
  // otherwise-legal face numbers (e.g. 4294967299 ≡ 3 mod 2^32).
  EXPECT_EQ(ValidateRaypathText("9999999999", CrystalKind::kPrism).state, RaypathValidation::kInvalid);
  EXPECT_EQ(ValidateRaypathText("9999999999", CrystalKind::kPyramid).state, RaypathValidation::kInvalid);
  EXPECT_EQ(ValidateRaypathText("4294967299", CrystalKind::kPrism).state, RaypathValidation::kInvalid);
  EXPECT_EQ(ValidateRaypathText("123456", CrystalKind::kPyramid).state, RaypathValidation::kInvalid);
  // 4 digits already exceeds the max-3-digit cap and is outside the legal
  // union on both kinds.
  EXPECT_EQ(ValidateRaypathText("1000", CrystalKind::kPrism).state, RaypathValidation::kInvalid);
  EXPECT_EQ(ValidateRaypathText("1000", CrystalKind::kPyramid).state, RaypathValidation::kInvalid);
}

TEST(ValidateRaypathTextWithKindTest, OverlongTokenMessageMentionsOutside) {
  auto r = ValidateRaypathText("9999999999", CrystalKind::kPrism);
  EXPECT_EQ(r.state, RaypathValidation::kInvalid);
  EXPECT_NE(r.message.find("outside"), std::string::npos);
}


// Contract test: after wiring IsLegalFaceGlobal to delegate to
// IsLegalFace(kPyramid, ...), the validator's "global union" stage should
// accept exactly the pyramid-legal faces. If a future CrystalKind extends
// the union beyond the pyramid set, this test will flag the divergence so
// IsLegalFaceGlobal can be generalised to OR-combine every kind.
TEST(IsLegalFaceTest, PyramidSetEqualsValidatorGlobalStage) {
  for (int f = 0; f <= 30; ++f) {
    bool pyramid_ok = IsLegalFace(CrystalKind::kPyramid, f);
    auto r = ValidateRaypathText(std::to_string(f), CrystalKind::kPyramid);
    if (pyramid_ok) {
      EXPECT_EQ(r.state, RaypathValidation::kValid) << "face=" << f;
    } else {
      EXPECT_EQ(r.state, RaypathValidation::kInvalid) << "face=" << f;
      // Because IsLegalFaceGlobal == IsLegalFace(kPyramid, ...), any illegal
      // face on pyramid triggers the "outside the legal range" message
      // (never the type-specific branch).
      EXPECT_NE(r.message.find("outside"), std::string::npos) << "face=" << f;
    }
  }
}


// ========== ValidateFaceNumberText Tests ==========
// Single face-number text input used by the Entry-Exit filter sub-panel.
// Differs from ValidateRaypathText in that separators ('-', ',') are
// rejected outright — Entry / Exit each take exactly one face number.

TEST(ValidateFaceNumberTextTest, Empty_IsIncomplete) {
  auto r = ValidateFaceNumberText("", CrystalKind::kPrism);
  EXPECT_EQ(r.state, RaypathValidation::kIncomplete);
  EXPECT_TRUE(r.message.empty());
}

TEST(ValidateFaceNumberTextTest, SingleLegalFace_IsValid) {
  auto r = ValidateFaceNumberText("3", CrystalKind::kPrism);
  EXPECT_EQ(r.state, RaypathValidation::kValid);
  EXPECT_TRUE(r.message.empty());
  auto r2 = ValidateFaceNumberText("13", CrystalKind::kPyramid);
  EXPECT_EQ(r2.state, RaypathValidation::kValid);
}

TEST(ValidateFaceNumberTextTest, ZeroIsOutsideLegalRange) {
  auto r = ValidateFaceNumberText("0", CrystalKind::kPrism);
  EXPECT_EQ(r.state, RaypathValidation::kInvalid);
  EXPECT_NE(r.message.find("outside"), std::string::npos);
}

TEST(ValidateFaceNumberTextTest, Separator_IsInvalid) {
  // Dashes / commas / semicolons are raypath syntax — face number rejects.
  for (const char* s : { "1-2", "3,5", "3-", ",5", "1;2" }) {
    auto r = ValidateFaceNumberText(s, CrystalKind::kPrism);
    EXPECT_EQ(r.state, RaypathValidation::kInvalid) << "input=" << s;
    EXPECT_NE(r.message.find("non-negative integer"), std::string::npos) << "input=" << s;
  }
}

TEST(ValidateFaceNumberTextTest, NonDigit_IsInvalid) {
  auto r = ValidateFaceNumberText("3a", CrystalKind::kPrism);
  EXPECT_EQ(r.state, RaypathValidation::kInvalid);
  auto r2 = ValidateFaceNumberText("abc", CrystalKind::kPrism);
  EXPECT_EQ(r2.state, RaypathValidation::kInvalid);
}

TEST(ValidateFaceNumberTextTest, OverlongDigitToken_IsInvalid) {
  auto r = ValidateFaceNumberText("9999", CrystalKind::kPrism);
  EXPECT_EQ(r.state, RaypathValidation::kInvalid);
  EXPECT_NE(r.message.find("out of range"), std::string::npos);
}

TEST(ValidateFaceNumberTextTest, FaceLegalGlobalButNotKindSpecific_IsInvalid) {
  // Face 13 is legal on Pyramid (basal/lateral) but illegal on Prism.
  auto r = ValidateFaceNumberText("13", CrystalKind::kPrism);
  EXPECT_EQ(r.state, RaypathValidation::kInvalid);
  EXPECT_NE(r.message.find("not legal"), std::string::npos);
  EXPECT_NE(r.message.find("Prism"), std::string::npos);
}

TEST(ValidateFaceNumberTextTest, FaceOutsideAnyKindSet_IsInvalid) {
  auto r = ValidateFaceNumberText("100", CrystalKind::kPyramid);
  EXPECT_EQ(r.state, RaypathValidation::kInvalid);
  EXPECT_NE(r.message.find("outside"), std::string::npos);
}


// ---- B symmetry: pyramid face swap tests ----

// B symmetry: upper pyramid face 13 should expand to lower pyramid face 23
TEST(SymmetryB_PyramidSwap, UpperPyramid_ExpandsToLower) {
  Crystal pyramid = Crystal::CreatePyramid(1.0f, 1.0f, 1.0f);

  auto expanded = pyramid.ExpandRaypath({ 13, 5 }, FilterConfig::kSymB, 0, false);
  bool found_lower = false;
  for (const auto& rp : expanded) {
    for (auto fn : rp) {
      if (fn == 23) {
        found_lower = true;
      }
    }
  }
  EXPECT_TRUE(found_lower) << "B expansion should include lower pyramid variant (23)";
}


// ---- ReduceRaypath 4-param overload direct tests ----

// D symmetry with sigma_a=0: reflected path {3,5} < original {3,7}, so reduction returns {3,5}
TEST(ReduceRaypath4Param, D_SigmaA0_ReflectedSmaller) {
  Crystal prism = Crystal::CreatePrism(1.0f);
  auto result = prism.ReduceRaypath({ 3, 7 }, FilterConfig::kSymD, 0, true);
  EXPECT_EQ(result, (std::vector<IdType>{ 3, 5 }));
}

// D symmetry with sigma_a=5: reflected path {5,3} < original {6,8}, so reduction returns {5,3}
TEST(ReduceRaypath4Param, D_SigmaA5_ReflectedSmaller) {
  Crystal prism = Crystal::CreatePrism(1.0f);
  auto result = prism.ReduceRaypath({ 6, 8 }, FilterConfig::kSymD, 5, true);
  EXPECT_EQ(result, (std::vector<IdType>{ 5, 3 }));
}

// B symmetry: original {13,5} < reflected {23,5}, so reduction keeps original
TEST(ReduceRaypath4Param, B_OriginalSmaller) {
  Crystal pyramid = Crystal::CreatePyramid(1.0f, 1.0f, 1.0f);
  auto result = pyramid.ReduceRaypath({ 13, 5 }, FilterConfig::kSymB, 0, false);
  EXPECT_EQ(result, (std::vector<IdType>{ 13, 5 }));
}

// ---- ReduceRaypath 4-param pyramid + D coverage ----

// sigma_a=0: face 18 (pri=5) reflects to 14 (pri=1); face 6 is a fixed point (pri=3→3).
// {14,6} < {18,6} → returns {14,6}. Before fix: bug returned {18,6} (D noop on pyramid).
TEST(ReduceRaypath4Param, D_Pyramid_SigmaA0_FaceReflected) {
  Crystal pyramid = Crystal::CreatePyramid(1.0f, 1.0f, 1.0f);
  auto result = pyramid.ReduceRaypath({ 18, 6 }, FilterConfig::kSymD, 0, true);
  EXPECT_EQ(result, (std::vector<IdType>{ 14, 6 }));
}

// sigma_a=0 mixed path: upper pyramid 18 → 14; prism 5 → 7; lower pyramid 23 stays (fixed point).
// Reflected {14,7,23} < {18,5,23} → returns {14,7,23}.
TEST(ReduceRaypath4Param, D_Pyramid_SigmaA0_MixedPath) {
  Crystal pyramid = Crystal::CreatePyramid(1.0f, 1.0f, 1.0f);
  auto result = pyramid.ReduceRaypath({ 18, 5, 23 }, FilterConfig::kSymD, 0, true);
  EXPECT_EQ(result, (std::vector<IdType>{ 14, 7, 23 }));
}

// sigma_a=0: pyramid faces 13 (pri=0) and 16 (pri=3) are D fixed points → unchanged.
TEST(ReduceRaypath4Param, D_Pyramid_SigmaA0_FixedPoint) {
  Crystal pyramid = Crystal::CreatePyramid(1.0f, 1.0f, 1.0f);
  auto result = pyramid.ReduceRaypath({ 13, 16 }, FilterConfig::kSymD, 0, true);
  EXPECT_EQ(result, (std::vector<IdType>{ 13, 16 }));
}

// sigma_a=5, fn_period_=6: no fixed points (2*pri ≡ 5 mod 6 has no integer solution).
// 17 (pyr=1,pri=4) → 1*10 + (5-4+6)%6 + 3 = 14; prism 3 (pri=0) → (5-0+6)%6 + 3 = 8.
// Reflected {14,8} < {17,3} → returns {14,8}.
TEST(ReduceRaypath4Param, D_Pyramid_SigmaA5_FaceReflected) {
  Crystal pyramid = Crystal::CreatePyramid(1.0f, 1.0f, 1.0f);
  auto result = pyramid.ReduceRaypath({ 17, 3 }, FilterConfig::kSymD, 5, true);
  EXPECT_EQ(result, (std::vector<IdType>{ 14, 8 }));
}

// ---- ExpandRaypath 4-param pyramid + D coverage ----

// sigma_a=0: D-reflect {14,6} → {18,6} (different), expansion has 2 unique entries.
TEST(ExpandRaypath4Param, D_Pyramid_SigmaA0_TwoVariants) {
  Crystal pyramid = Crystal::CreatePyramid(1.0f, 1.0f, 1.0f);
  auto expanded = pyramid.ExpandRaypath({ 14, 6 }, FilterConfig::kSymD, 0, true);
  ASSERT_EQ(expanded.size(), 2u);
  bool found_18 = false;
  for (const auto& rp : expanded) {
    if (rp.size() == 2 && rp[0] == 18 && rp[1] == 6) {
      found_18 = true;
    }
  }
  EXPECT_TRUE(found_18) << "D expansion of {14,6} must contain {18,6}";
}

// P+D, sigma_a=0, starting from pyramid {18,6}: 6 P rotations × 2 D mirrors = 12 unique paths.
// Before fix: D skipped pyramid faces, yielding only 10 unique paths.
TEST(ExpandRaypath4Param, D_Pyramid_SigmaA0_PD_12Paths) {
  Crystal pyramid = Crystal::CreatePyramid(1.0f, 1.0f, 1.0f);
  auto expanded = pyramid.ExpandRaypath({ 18, 6 }, FilterConfig::kSymP | FilterConfig::kSymD, 0, true);
  EXPECT_EQ(expanded.size(), 12u);

  RaypathHash h;
  std::set<size_t> hashes;
  for (const auto& rp : expanded) {
    EXPECT_TRUE(hashes.insert(h(rp)).second) << "Duplicate hash in expanded pyramid P+D paths";
  }

  bool found_14_6 = false;
  bool found_18_4 = false;
  for (const auto& rp : expanded) {
    if (rp.size() == 2 && rp[0] == 14 && rp[1] == 6) {
      found_14_6 = true;
    }
    if (rp.size() == 2 && rp[0] == 18 && rp[1] == 4) {
      found_18_4 = true;
    }
  }
  EXPECT_TRUE(found_14_6) << "D mirror {14,6} must be in P+D expansion of pyramid {18,6}";
  EXPECT_TRUE(found_18_4) << "D mirror {18,4} must be in P+D expansion of pyramid {18,6}";
}

// sigma_a=5: D-reflect {14,8} → {17,3}. Expansion has 2 unique entries.
TEST(ExpandRaypath4Param, D_Pyramid_SigmaA5_TwoVariants) {
  Crystal pyramid = Crystal::CreatePyramid(1.0f, 1.0f, 1.0f);
  auto expanded = pyramid.ExpandRaypath({ 14, 8 }, FilterConfig::kSymD, 5, true);
  ASSERT_EQ(expanded.size(), 2u);
  bool found_17_3 = false;
  for (const auto& rp : expanded) {
    if (rp.size() == 2 && rp[0] == 17 && rp[1] == 3) {
      found_17_3 = true;
    }
  }
  EXPECT_TRUE(found_17_3) << "D expansion of {14,8} with sigma_a=5 must contain {17,3}";
}

// ---- ReduceRaypath orbit invariant: same orbit must reduce to same canonical ----
//
// Bug context: when kSymP|kSymD is set with sigma_a != 0, the D-reflected candidate
// is not re-P-canonicalized before lex comparison, so different orbit members can
// reduce to different representatives. These tests pin the property "ReduceRaypath
// is constant on each ExpandRaypath orbit" across sigma_a ∈ {0..5}.

TEST(ReduceRaypath_OrbitInvariant, PD_Pyramid_AllSigmaA) {
  Crystal pyramid = Crystal::CreatePyramid(1.0f, 1.0f, 1.0f);
  constexpr uint8_t kSym = FilterConfig::kSymP | FilterConfig::kSymD;
  for (int sigma_a = 0; sigma_a < 6; sigma_a++) {
    auto orbit = pyramid.ExpandRaypath({ 14, 6 }, kSym, sigma_a, true);
    ASSERT_EQ(orbit.size(), 12u) << "orbit size mismatch at sigma_a=" << sigma_a;
    auto first = pyramid.ReduceRaypath(orbit[0], kSym, sigma_a, true);
    for (size_t i = 1; i < orbit.size(); i++) {
      auto reduced = pyramid.ReduceRaypath(orbit[i], kSym, sigma_a, true);
      EXPECT_EQ(reduced, first) << "orbit member " << i << " reduces to different canonical at sigma_a=" << sigma_a;
    }
  }
}

TEST(ReduceRaypath_OrbitInvariant, PD_Pyramid_KnownBugCase_SigmaA5) {
  Crystal pyramid = Crystal::CreatePyramid(1.0f, 1.0f, 1.0f);
  constexpr uint8_t kSym = FilterConfig::kSymP | FilterConfig::kSymD;
  auto r1 = pyramid.ReduceRaypath({ 14, 6 }, kSym, 5, true);
  auto r2 = pyramid.ReduceRaypath({ 18, 6 }, kSym, 5, true);
  EXPECT_EQ(r1, r2) << "Same orbit must map to same canonical form";
}

TEST(ReduceRaypath_OrbitInvariant, PD_Prism_AllSigmaA) {
  Crystal prism = Crystal::CreatePrism(1.0f);
  constexpr uint8_t kSym = FilterConfig::kSymP | FilterConfig::kSymD;
  for (int sigma_a = 0; sigma_a < 6; sigma_a++) {
    auto orbit = prism.ExpandRaypath({ 3, 7 }, kSym, sigma_a, true);
    ASSERT_GE(orbit.size(), 2u);
    auto first = prism.ReduceRaypath(orbit[0], kSym, sigma_a, true);
    for (const auto& member : orbit) {
      EXPECT_EQ(prism.ReduceRaypath(member, kSym, sigma_a, true), first)
          << "prism orbit member reduces differently at sigma_a=" << sigma_a;
    }
  }
}

TEST(ReduceRaypath_OrbitInvariant, D_Only_Pyramid_AllSigmaA) {
  Crystal pyramid = Crystal::CreatePyramid(1.0f, 1.0f, 1.0f);
  constexpr uint8_t kSym = FilterConfig::kSymD;
  for (int sigma_a = 0; sigma_a < 6; sigma_a++) {
    auto orbit = pyramid.ExpandRaypath({ 14, 6 }, kSym, sigma_a, true);
    ASSERT_EQ(orbit.size(), 2u) << "D-only orbit must have 2 members at sigma_a=" << sigma_a;
    auto first = pyramid.ReduceRaypath(orbit[0], kSym, sigma_a, true);
    for (const auto& member : orbit) {
      EXPECT_EQ(pyramid.ReduceRaypath(member, kSym, sigma_a, true), first)
          << "D-only orbit member reduces differently at sigma_a=" << sigma_a;
    }
  }
}
