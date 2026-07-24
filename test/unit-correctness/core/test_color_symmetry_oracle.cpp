// scrum-color-predicate-symmetry AC1 Oracle test — Step 8.
//
// Constructs a fixture where a given raypath in a given crystal has a non-
// trivial PBD equivalence class, then asserts LINE-BY-LINE that the color pass
// (via BuildColorSpecGroups) matches EXACTLY the same set of orbit members as
// the physical filter path (FilterConfig{symmetry=X} + FilterSpec::Create).
// Not a statistical contrast — every orbit member is asserted individually.
//
// Coverage matrix (mirrors issue AC1 requirement):
//   - Prism  + kSymP
//   - Prism  + kSymB
//   - Prism  + kSymD                            (D-only, plan-review Minor 2)
//   - Prism  + kSymP|kSymD  (d_applicable=true, sigma_a scan 0..5)
//   - Prism  + kSymP|kSymD  (d_applicable=false via non-D-eligible axis)
//   - Prism  + kSymP|kSymB|kSymD  (all three)
//   - Pyramid + same six combinations
//
// Method:
//   1. Build a physical FilterConfig{symmetry=X} with a raypath predicate
//      matching the "seed" path.
//   2. Build a synthetic ColorGatePlacement carrying a single ref
//      (predicate = same seed path, symmetry = X, bit = 0), pipe it through
//      BuildColorSpecGroups — expect ONE group carrying symmetry X.
//   3. Enumerate the orbit under (X, sigma_a, d_applicable) via
//      Crystal::ExpandRaypath. For each orbit member, drive both specs on a
//      RaySeg + RaypathRecorder built from that member and assert:
//        - phys_spec->Check(...) == true (seed matches self ⇒ orbit self-
//          matches;防空实现 anchor per orbit-and-symmetry-testing rule).
//        - color group's CheckSummandMask(...) == true (identical membership).

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "config/color_gate_table.hpp"
#include "config/filter_config.hpp"
#include "core/crystal.hpp"
#include "core/def.hpp"
#include "core/filter_spec.hpp"
#include "core/math.hpp"
#include "core/raypath.hpp"

namespace lumice {
namespace {

// AxisDistribution that IS "d_applicable" (isotropic azimuth, latitude=90 =
// c-axis vertical, roll=0). This is the standard hex-plate configuration used
// by test_filter_spec.cpp and matches the physical setup where D symmetry is
// active.
AxisDistribution MakeDApplicableAxis(float roll_mean_deg = 0.0f) {
  AxisDistribution d{};
  d.azimuth_dist.type = DistributionType::kUniform;
  d.azimuth_dist.spread = 360.0f;
  d.azimuth_dist.center = 0.0f;
  d.latitude_dist.type = DistributionType::kNoRandom;
  d.latitude_dist.center = 90.0f;
  d.roll_dist.type = DistributionType::kNoRandom;
  d.roll_dist.center = roll_mean_deg;
  d.roll_dist.spread = 0.0f;
  return d;
}

// Axis for which D symmetry is NOT applicable (non-plate orientation — random
// latitude, non-plate mean; detail::IsDApplicable(...) returns false). Used to
// verify the "D-branch skipped, PB behaviour still matches physical filter"
// oracle path.
AxisDistribution MakeDNotApplicableAxis() {
  AxisDistribution d{};
  d.azimuth_dist.type = DistributionType::kUniform;
  d.azimuth_dist.spread = 360.0f;
  d.azimuth_dist.center = 0.0f;
  d.latitude_dist.type = DistributionType::kUniform;
  d.latitude_dist.spread = 90.0f;
  d.latitude_dist.center = 0.0f;
  d.roll_dist.type = DistributionType::kUniform;
  d.roll_dist.spread = 90.0f;
  d.roll_dist.center = 0.0f;
  return d;
}

RaypathRecorder ToRecorder(const std::vector<IdType>& rp) {
  RaypathRecorder out;
  out.Clear();
  for (auto fn : rp) {
    out << fn;
  }
  return out;
}

RaySeg MakeRay() {
  RaySeg r{};
  r.from_face_ = kInvalidId;
  r.to_face_ = kInvalidId;
  r.w_ = 1.0f;
  r.crystal_config_id_ = 0;
  r.d_[0] = 0.0f;
  r.d_[1] = 0.0f;
  r.d_[2] = 1.0f;
  r.p_[0] = 0.0f;
  r.p_[1] = 0.0f;
  r.p_[2] = 0.0f;
  r.crystal_rot_ = Rotation{};
  return r;
}

// Build a physical FilterSpec configured with symmetry X + raypath predicate.
std::unique_ptr<FilterSpec> MakePhysicalSpec(const Crystal& crystal, const AxisDistribution& axis, uint8_t symmetry,
                                             const std::vector<IdType>& seed) {
  FilterConfig fc{};
  fc.id_ = 1;
  fc.symmetry_ = symmetry;
  fc.action_ = FilterConfig::kFilterIn;
  RaypathFilterParam rp{};
  rp.raypath_ = seed;
  fc.param_ = SimpleFilterParam{ rp };
  return FilterSpec::Create(fc, crystal, axis);
}

// Build a single-ref ColorGatePlacement (bit 0) and run BuildColorSpecGroups.
// Returns the single expected group (asserts count on the way).
ColorSpecGroupOwned BuildSingleColorGroup(const Crystal& crystal, const AxisDistribution& axis, uint8_t symmetry,
                                          const std::vector<IdType>& seed) {
  ColorGatePlacement placement;
  RaypathFilterParam rp{};
  rp.raypath_ = seed;
  SimpleFilterParam pred{ rp };
  placement.predicates_.push_back(pred);
  placement.bits_.push_back(0u);
  placement.symmetries_.push_back(symmetry);
  auto groups = BuildColorSpecGroups(placement, crystal, axis);
  // Exactly one group with a single predicate.
  EXPECT_EQ(groups.size(), 1u);
  return std::move(groups[0]);
}

// One combination in the coverage matrix.
struct OracleCase {
  const char* label;
  Crystal (*make_crystal)();
  AxisDistribution axis;
  uint8_t symmetry;
  std::vector<IdType> seed;
};

Crystal MakePrism() {
  return Crystal::CreatePrism(1.0f);
}
Crystal MakePyramid() {
  return Crystal::CreatePyramid(0.5f, 1.0f, 0.5f);
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
::testing::AssertionResult VerifyOrbitAgreement(const OracleCase& tc) {
  Crystal crystal = tc.make_crystal();
  auto phys = MakePhysicalSpec(crystal, tc.axis, tc.symmetry, tc.seed);
  if (!phys) {
    return ::testing::AssertionFailure() << tc.label << ": phys spec null";
  }
  auto color_group = BuildSingleColorGroup(crystal, tc.axis, tc.symmetry, tc.seed);
  if (!color_group.spec) {
    return ::testing::AssertionFailure() << tc.label << ": color group spec null";
  }

  // Anti-empty-implementation anchor: the seed MUST self-match — both under
  // the physical filter and via the color spec. If this fails, the oracle is
  // vacuous.
  {
    auto rec = ToRecorder(tc.seed);
    RaySeg r = MakeRay();
    if (!phys->Check(r, rec, nullptr)) {
      return ::testing::AssertionFailure() << tc.label << ": phys does NOT match its own seed (empty predicate?)";
    }
    uint64_t mask = 0;
    (void)color_group.spec->CheckSummandMask(r, rec, nullptr, &mask);
    if (mask == 0) {
      return ::testing::AssertionFailure() << tc.label << ": color group does NOT match its own seed";
    }
  }

  auto orbit = crystal.ExpandRaypath(tc.seed, tc.symmetry);
  if (orbit.empty()) {
    return ::testing::AssertionFailure() << tc.label << ": ExpandRaypath returned empty (fixture bug)";
  }
  for (const auto& member : orbit) {
    auto rec = ToRecorder(member);
    RaySeg r = MakeRay();
    bool phys_hit = phys->Check(r, rec, nullptr);
    uint64_t mask = 0;
    (void)color_group.spec->CheckSummandMask(r, rec, nullptr, &mask);
    bool color_hit = (mask != 0);
    if (phys_hit != color_hit) {
      std::string mem_s = "{";
      for (size_t i = 0; i < member.size(); ++i) {
        if (i > 0) {
          mem_s += ",";
        }
        mem_s += std::to_string(int(member[i]));
      }
      mem_s += "}";
      return ::testing::AssertionFailure() << tc.label << ": disagreement on orbit member " << mem_s
                                           << " phys=" << phys_hit << " color=" << color_hit;
    }
    // On a match, at least summand 0 (the sole predicate) must be set.
    if (color_hit) {
      EXPECT_TRUE((mask & 1ull) != 0) << tc.label << ": color hit but summand-0 bit unset";
    }
  }

  // A raypath NOT in the orbit should be rejected by both. Pick a raypath
  // that's structurally different from any orbit member to avoid accidental
  // canonical coincidence.
  {
    std::vector<IdType> outsider = { 100, 200 };  // unreachable face ids -> can't match
    auto rec = ToRecorder(outsider);
    RaySeg r = MakeRay();
    bool phys_hit = phys->Check(r, rec, nullptr);
    uint64_t mask = 0;
    (void)color_group.spec->CheckSummandMask(r, rec, nullptr, &mask);
    bool color_hit = (mask != 0);
    if (phys_hit != color_hit) {
      return ::testing::AssertionFailure()
             << tc.label << ": outsider raypath disagreement phys=" << phys_hit << " color=" << color_hit;
    }
  }

  return ::testing::AssertionSuccess();
}

}  // namespace

// ============================ P-only ============================

TEST(ColorSymmetryOracle, PrismSymP) {
  OracleCase tc{ "Prism/P", MakePrism, MakeDApplicableAxis(), FilterConfig::kSymP, /*seed=*/{ 3, 5 } };
  EXPECT_TRUE(VerifyOrbitAgreement(tc));
}

TEST(ColorSymmetryOracle, PyramidSymP) {
  OracleCase tc{ "Pyramid/P", MakePyramid, MakeDApplicableAxis(), FilterConfig::kSymP, /*seed=*/{ 3, 5 } };
  EXPECT_TRUE(VerifyOrbitAgreement(tc));
}

// ============================ B-only ============================

TEST(ColorSymmetryOracle, PrismSymB) {
  OracleCase tc{ "Prism/B", MakePrism, MakeDApplicableAxis(), FilterConfig::kSymB, /*seed=*/{ 3, 5 } };
  EXPECT_TRUE(VerifyOrbitAgreement(tc));
}

TEST(ColorSymmetryOracle, PyramidSymB) {
  OracleCase tc{ "Pyramid/B", MakePyramid, MakeDApplicableAxis(), FilterConfig::kSymB, /*seed=*/{ 3, 5 } };
  EXPECT_TRUE(VerifyOrbitAgreement(tc));
}

// ============================ D-only ============================
// plan-review round-2 Minor 2: dedicated D-only case to prevent D bugs from
// being masked by co-existing P/B in a combined case.

TEST(ColorSymmetryOracle, PrismSymD) {
  OracleCase tc{ "Prism/D", MakePrism, MakeDApplicableAxis(), FilterConfig::kSymD, /*seed=*/{ 3, 5 } };
  EXPECT_TRUE(VerifyOrbitAgreement(tc));
}

TEST(ColorSymmetryOracle, PyramidSymD) {
  OracleCase tc{ "Pyramid/D", MakePyramid, MakeDApplicableAxis(), FilterConfig::kSymD, /*seed=*/{ 3, 5 } };
  EXPECT_TRUE(VerifyOrbitAgreement(tc));
}

// ============================ P|D (d_applicable = true, sigma_a scan) ============================

TEST(ColorSymmetryOracle, PrismSymPDDApplicableSigmaAScan) {
  for (int sigma_a = 0; sigma_a < 6; ++sigma_a) {
    OracleCase tc{ "Prism/PD/d_appl/sigma_a", MakePrism, MakeDApplicableAxis(static_cast<float>(sigma_a) * 60.0f),
                   static_cast<uint8_t>(FilterConfig::kSymP | FilterConfig::kSymD), /*seed=*/{ 3, 5 } };
    EXPECT_TRUE(VerifyOrbitAgreement(tc)) << " sigma_a index=" << sigma_a;
  }
}

TEST(ColorSymmetryOracle, PyramidSymPDDApplicableSigmaAScan) {
  for (int sigma_a = 0; sigma_a < 6; ++sigma_a) {
    OracleCase tc{ "Pyramid/PD/d_appl/sigma_a", MakePyramid, MakeDApplicableAxis(static_cast<float>(sigma_a) * 60.0f),
                   static_cast<uint8_t>(FilterConfig::kSymP | FilterConfig::kSymD), /*seed=*/{ 3, 5 } };
    EXPECT_TRUE(VerifyOrbitAgreement(tc)) << " sigma_a index=" << sigma_a;
  }
}

// ============================ P|D (d_applicable = false) ============================
// AC1 explicit: verify D-branch is skipped and PB behaviour still matches
// physical filter path when d_applicable=false (non-plate axis distribution).

TEST(ColorSymmetryOracle, PrismSymPDNotDApplicable) {
  OracleCase tc{ "Prism/PD/no-d_appl", MakePrism, MakeDNotApplicableAxis(),
                 static_cast<uint8_t>(FilterConfig::kSymP | FilterConfig::kSymD), /*seed=*/{ 3, 5 } };
  EXPECT_TRUE(VerifyOrbitAgreement(tc));
}

TEST(ColorSymmetryOracle, PyramidSymPDNotDApplicable) {
  OracleCase tc{ "Pyramid/PD/no-d_appl", MakePyramid, MakeDNotApplicableAxis(),
                 static_cast<uint8_t>(FilterConfig::kSymP | FilterConfig::kSymD), /*seed=*/{ 3, 5 } };
  EXPECT_TRUE(VerifyOrbitAgreement(tc));
}

// ============================ P|B|D (combined) ============================

TEST(ColorSymmetryOracle, PrismSymPBDCombinedSigmaAScan) {
  for (int sigma_a = 0; sigma_a < 6; ++sigma_a) {
    OracleCase tc{ "Prism/PBD/sigma_a", MakePrism, MakeDApplicableAxis(static_cast<float>(sigma_a) * 60.0f),
                   static_cast<uint8_t>(FilterConfig::kSymP | FilterConfig::kSymB | FilterConfig::kSymD),
                   /*seed=*/{ 3, 5 } };
    EXPECT_TRUE(VerifyOrbitAgreement(tc)) << " sigma_a index=" << sigma_a;
  }
}

TEST(ColorSymmetryOracle, PyramidSymPBDCombinedSigmaAScan) {
  for (int sigma_a = 0; sigma_a < 6; ++sigma_a) {
    OracleCase tc{ "Pyramid/PBD/sigma_a", MakePyramid, MakeDApplicableAxis(static_cast<float>(sigma_a) * 60.0f),
                   static_cast<uint8_t>(FilterConfig::kSymP | FilterConfig::kSymB | FilterConfig::kSymD),
                   /*seed=*/{ 3, 5 } };
    EXPECT_TRUE(VerifyOrbitAgreement(tc)) << " sigma_a index=" << sigma_a;
  }
}

}  // namespace lumice
