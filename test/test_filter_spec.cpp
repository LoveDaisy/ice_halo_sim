// Unit tests for FilterSpec (canonical-form matcher) — AC-2 of
// task-filter-spec-matcher-split.
//
//   AC-2  detail::ReduceRecorder produces the same canonical as
//         Crystal::ReduceRaypath across (symmetry × sigma_a × crystal × raypath).
//
// AC-3 (FilterSpec vs Filter B-oracle parity) was removed in
// task-filter-callers-migration; the B oracle's canonical-form path is gone,
// so the parity check has no oracle to compare against.

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "config/filter_config.hpp"
#include "core/crystal.hpp"
#include "core/def.hpp"
#include "core/filter_spec.hpp"
#include "core/raypath.hpp"

namespace lumice {
namespace {

RaypathRecorder ToRecorder(const std::vector<IdType>& rp) {
  RaypathRecorder out;
  out.Clear();
  for (auto fn : rp) {
    out << fn;
  }
  return out;
}

std::string FormatRaypath(const std::vector<IdType>& rp) {
  std::string s = "{";
  for (size_t i = 0; i < rp.size(); i++) {
    if (i > 0) {
      s += ",";
    }
    s += std::to_string(rp[i]);
  }
  s += "}";
  return s;
}

std::string FormatRecorder(const RaypathRecorder& rp) {
  std::string s = "{";
  for (size_t i = 0; i < rp.size_; i++) {
    if (i > 0) {
      s += ",";
    }
    s += std::to_string(static_cast<int>(rp.recorder_[i]));
  }
  s += "}";
  return s;
}

::testing::AssertionResult VerifyReduceRecorderOracle(const Crystal& crystal, uint8_t symmetry, int sigma_a,
                                                      bool d_applicable, const std::vector<IdType>& rp_seed) {
  auto oracle_vec = crystal.ReduceRaypath(rp_seed, symmetry, sigma_a, d_applicable);
  auto oracle_rec = ToRecorder(oracle_vec);

  auto actual = ToRecorder(rp_seed);
  detail::ReduceRecorder(actual, symmetry, sigma_a, d_applicable);

  if (actual != oracle_rec) {
    return ::testing::AssertionFailure() << "ReduceRecorder mismatch: seed=" << FormatRaypath(rp_seed)
                                         << " sym=" << static_cast<int>(symmetry) << " sigma_a=" << sigma_a
                                         << " d_applicable=" << d_applicable
                                         << "; oracle=" << FormatRecorder(oracle_rec)
                                         << " actual=" << FormatRecorder(actual);
  }
  return ::testing::AssertionSuccess();
}

// Helper to build axis dist with given roll_mean and az-uniform-360.
AxisDistribution MakeAxis(float roll_mean_deg) {
  AxisDistribution d{};
  d.azimuth_dist.type = DistributionType::kUniform;
  d.azimuth_dist.std = 360.0f;
  d.azimuth_dist.mean = 0.0f;
  d.latitude_dist.type = DistributionType::kNoRandom;
  d.latitude_dist.mean = 90.0f;
  d.roll_dist.type = DistributionType::kNoRandom;
  d.roll_dist.mean = roll_mean_deg;
  d.roll_dist.std = 0.0f;
  return d;
}

RaySeg MakeRay(const std::vector<IdType>& rp_vec) {
  RaySeg r{};
  r.rp_.Clear();
  for (auto fn : rp_vec) {
    r.rp_ << fn;
  }
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
  return r;
}

std::vector<RaySeg> BuildPrismRayBatch() {
  std::vector<RaySeg> rays;
  // empty raypath
  rays.push_back(MakeRay({}));
  // single-segment basal + prism
  rays.push_back(MakeRay({ 1 }));
  rays.push_back(MakeRay({ 3 }));
  // pairs (a,b) ∈ {3..8}²
  for (IdType a = 3; a <= 8; a++) {
    for (IdType b = 3; b <= 8; b++) {
      if (a == b) {
        continue;
      }
      rays.push_back(MakeRay({ a, b }));
    }
  }
  // include basal+prism mixes
  rays.push_back(MakeRay({ 1, 3 }));
  rays.push_back(MakeRay({ 2, 5, 1 }));
  rays.push_back(MakeRay({ 3, 6, 4 }));
  return rays;
}

std::vector<RaySeg> BuildPyramidRayBatch() {
  std::vector<RaySeg> rays;
  // pyramid + prism pairs
  for (IdType u = 13; u <= 18; u++) {
    for (IdType p = 3; p <= 8; p++) {
      rays.push_back(MakeRay({ u, p }));
    }
  }
  for (IdType l = 23; l <= 28; l++) {
    rays.push_back(MakeRay({ l, 4 }));
  }
  rays.push_back(MakeRay({ 13, 5, 23 }));
  rays.push_back(MakeRay({ 14, 25, 6 }));
  return rays;
}

// =============== AC-2: detail::ReduceRecorder vs Crystal::ReduceRaypath ===============

TEST(FilterSpecReduceRecorder, Prism_P_SigmaA0) {
  Crystal crystal = Crystal::CreatePrism(1.0f);
  std::vector<std::vector<IdType>> seeds{
    { 3, 5 }, { 4, 6 }, { 5, 7 }, { 6, 8 }, { 3, 8, 5 }, { 4, 5, 6, 7 }, { 1, 3 }, { 1, 2, 3 },
  };
  for (const auto& s : seeds) {
    EXPECT_TRUE(VerifyReduceRecorderOracle(crystal, FilterConfig::kSymP, /*sigma_a=*/0, /*d_applicable=*/false, s));
  }
}

TEST(FilterSpecReduceRecorder, Prism_P_SigmaA1_5) {
  Crystal crystal = Crystal::CreatePrism(1.0f);
  std::vector<std::vector<IdType>> seeds{
    { 3, 5 },
    { 4, 6, 8 },
    { 5, 3, 7 },
    { 1, 4, 7 },
  };
  for (int sigma_a : { 1, 2, 3, 4, 5 }) {
    for (const auto& s : seeds) {
      // sigma_a is meaningless without kSymD; verify it is still ignored by ReduceRecorder when d_applicable=false.
      EXPECT_TRUE(VerifyReduceRecorderOracle(crystal, FilterConfig::kSymP, sigma_a, /*d_applicable=*/false, s));
    }
  }
}

TEST(FilterSpecReduceRecorder, Prism_PD_VariousSigmaA) {
  Crystal crystal = Crystal::CreatePrism(1.0f);
  std::vector<std::vector<IdType>> seeds{
    { 3, 5 }, { 4, 6 }, { 5, 8 }, { 3, 7, 5 }, { 4, 6, 8 }, { 1, 3, 2 },
  };
  for (int sigma_a : { 0, 1, 2, 3, 4, 5 }) {
    for (const auto& s : seeds) {
      EXPECT_TRUE(VerifyReduceRecorderOracle(crystal, FilterConfig::kSymP | FilterConfig::kSymD, sigma_a,
                                             /*d_applicable=*/true, s));
    }
  }
}

TEST(FilterSpecReduceRecorder, Prism_D_NotApplicable_Skipped) {
  Crystal crystal = Crystal::CreatePrism(1.0f);
  // d_applicable=false → D should be skipped even when symmetry bit is set.
  EXPECT_TRUE(VerifyReduceRecorderOracle(crystal, FilterConfig::kSymP | FilterConfig::kSymD, /*sigma_a=*/3,
                                         /*d_applicable=*/false, { 3, 5 }));
}

TEST(FilterSpecReduceRecorder, Prism_B) {
  Crystal crystal = Crystal::CreatePrism(1.0f);
  std::vector<std::vector<IdType>> seeds{
    { 1, 3 }, { 2, 5 }, { 1, 4, 2 }, { 3, 1, 5 }, { 2, 6 },
  };
  for (const auto& s : seeds) {
    EXPECT_TRUE(VerifyReduceRecorderOracle(crystal, FilterConfig::kSymB, /*sigma_a=*/0, /*d_applicable=*/false, s));
  }
}

TEST(FilterSpecReduceRecorder, Prism_PB) {
  Crystal crystal = Crystal::CreatePrism(1.0f);
  std::vector<std::vector<IdType>> seeds{
    { 1, 3 }, { 2, 5 }, { 1, 4, 6 }, { 3, 1, 5 }, { 2, 6, 8 },
  };
  for (const auto& s : seeds) {
    EXPECT_TRUE(VerifyReduceRecorderOracle(crystal, FilterConfig::kSymP | FilterConfig::kSymB, /*sigma_a=*/0,
                                           /*d_applicable=*/false, s));
  }
}

TEST(FilterSpecReduceRecorder, Prism_PDB_AllSigmaA) {
  Crystal crystal = Crystal::CreatePrism(1.0f);
  std::vector<std::vector<IdType>> seeds{
    { 1, 3 }, { 2, 5 }, { 3, 5 }, { 1, 4, 6 }, { 3, 1, 5 }, { 2, 6, 8 },
  };
  for (int sigma_a : { 0, 1, 2, 3, 4, 5 }) {
    for (const auto& s : seeds) {
      EXPECT_TRUE(VerifyReduceRecorderOracle(crystal, FilterConfig::kSymP | FilterConfig::kSymD | FilterConfig::kSymB,
                                             sigma_a, /*d_applicable=*/true, s));
    }
  }
}

TEST(FilterSpecReduceRecorder, Pyramid_P) {
  Crystal crystal = Crystal::CreatePyramid(1.0f, 1.0f, 1.0f);
  std::vector<std::vector<IdType>> seeds{
    { 13, 5 }, { 14, 6 }, { 15, 7 }, { 23, 5 }, { 25, 8 }, { 13, 14 }, { 3, 13 }, { 14, 3, 25 },
  };
  for (const auto& s : seeds) {
    EXPECT_TRUE(VerifyReduceRecorderOracle(crystal, FilterConfig::kSymP, /*sigma_a=*/0, /*d_applicable=*/false, s));
  }
}

TEST(FilterSpecReduceRecorder, Pyramid_PDB) {
  Crystal crystal = Crystal::CreatePyramid(1.0f, 1.0f, 1.0f);
  std::vector<std::vector<IdType>> seeds{
    { 13, 5 }, { 14, 6 }, { 23, 5 }, { 25, 8 }, { 13, 14 }, { 1, 13 }, { 2, 25 },
  };
  for (int sigma_a : { 0, 3, 5 }) {
    for (const auto& s : seeds) {
      EXPECT_TRUE(VerifyReduceRecorderOracle(crystal, FilterConfig::kSymP | FilterConfig::kSymD | FilterConfig::kSymB,
                                             sigma_a, /*d_applicable=*/true, s));
    }
  }
}

TEST(FilterSpecReduceRecorder, MultiSegment_Hex) {
  Crystal crystal = Crystal::CreatePrism(1.0f);
  std::vector<std::vector<IdType>> seeds{
    { 3, 5, 7, 4 },
    { 4, 6, 8, 5, 3 },
    { 1, 3, 5, 2, 7 },
  };
  for (const auto& s : seeds) {
    EXPECT_TRUE(VerifyReduceRecorderOracle(crystal, FilterConfig::kSymP | FilterConfig::kSymD | FilterConfig::kSymB,
                                           /*sigma_a=*/0, /*d_applicable=*/true, s));
  }
}

// =============== AC-1/AC-2: FilterSpec.Match() orbit-invariant ===============

// sigma_a -> roll_mean_deg inverse mapping for detail::ComputeSigmaA
// (roll_mean -> n = round(roll/30)%6 -> sigma_a = (6-n)%6).
constexpr float kSigmaARollDeg[6] = { 0.0f, 150.0f, 120.0f, 90.0f, 60.0f, 30.0f };

// Returns AssertionSuccess iff all orbit members produce Match()=true.
// Precondition: the filter was built from seed_rp, so spec->Match(MakeRay(seed_rp))
// must be true; if not, the FilterSpec was misconfigured and the orbit sweep would
// be vacuously satisfied with expected=false for all members.
::testing::AssertionResult VerifyMatchOrbitInvariant(FilterSpec* spec, const Crystal& crystal,
                                                     const std::vector<IdType>& seed_rp, uint8_t symmetry, int sigma_a,
                                                     bool d_applicable) {
  auto orbit = crystal.ExpandRaypath(seed_rp, symmetry, sigma_a, d_applicable);
  if (orbit.empty()) {
    return ::testing::AssertionFailure() << "ExpandRaypath returned empty orbit for seed=" << FormatRaypath(seed_rp);
  }
  RaySeg seed_ray = MakeRay(seed_rp);
  if (!spec->Match(seed_ray)) {
    return ::testing::AssertionFailure() << "Seed raypath " << FormatRaypath(seed_rp)
                                         << " did not match its own filter (sym=" << static_cast<int>(symmetry)
                                         << " sigma_a=" << sigma_a << ") - filter misconfigured, orbit sweep aborted";
  }
  for (size_t i = 0; i < orbit.size(); i++) {
    RaySeg ri = MakeRay(orbit[i]);
    if (!spec->Match(ri)) {
      return ::testing::AssertionFailure() << "Orbit member " << i << " " << FormatRaypath(orbit[i])
                                           << " Match()=false != expected=true (seed=" << FormatRaypath(seed_rp)
                                           << " sym=" << static_cast<int>(symmetry) << " sigma_a=" << sigma_a << ")";
    }
  }
  return ::testing::AssertionSuccess();
}

std::unique_ptr<FilterSpec> MakeRaypathSpec(const Crystal& crystal, const std::vector<IdType>& rp, uint8_t symmetry,
                                            float roll_mean_deg) {
  FilterConfig cfg{};
  cfg.id_ = 1;
  cfg.symmetry_ = symmetry;
  cfg.action_ = FilterConfig::kFilterIn;
  RaypathFilterParam p{};
  p.raypath_ = rp;
  cfg.param_ = SimpleFilterParam{ p };
  return FilterSpec::Create(cfg, crystal, MakeAxis(roll_mean_deg));
}

// Build a ComplexSpec where each entry of rps is one OR-clause with one AND-filter
// (a RaypathFilterParam). Pair-id 0 is a clause-id placeholder; ComplexSpec ignores
// it (only used downstream by Filter B-oracle for clause routing, which is gone).
std::unique_ptr<FilterSpec> MakeComplexSpec(const Crystal& crystal, const std::vector<std::vector<IdType>>& rps,
                                            uint8_t symmetry, float roll_mean_deg) {
  FilterConfig cfg{};
  cfg.id_ = 1;
  cfg.symmetry_ = symmetry;
  cfg.action_ = FilterConfig::kFilterIn;
  ComplexFilterParam cp{};
  for (const auto& rp : rps) {
    RaypathFilterParam rpp{};
    rpp.raypath_ = rp;
    std::vector<std::pair<IdType, SimpleFilterParam>> and_clause;
    and_clause.emplace_back(IdType{ 0 }, SimpleFilterParam{ rpp });
    cp.filters_.push_back(std::move(and_clause));
  }
  cfg.param_ = cp;
  return FilterSpec::Create(cfg, crystal, MakeAxis(roll_mean_deg));
}

// N value selection: orbit-invariant is a property of the group structure, independent
// of N (ComplexFilter OR-clause count). One representative N per (crystal, symmetry)
// covers the property; sigma_a sweeps are added for symmetries containing kSymD.

TEST(FilterSpec_MatchOrbitInvariant, Prism_P_N1) {
  Crystal crystal = Crystal::CreatePrism(1.0f);
  constexpr uint8_t kSym = FilterConfig::kSymP;
  std::vector<IdType> seed{ 3, 5 };
  auto spec = MakeRaypathSpec(crystal, seed, kSym, kSigmaARollDeg[0]);
  ASSERT_NE(spec, nullptr);
  EXPECT_TRUE(VerifyMatchOrbitInvariant(spec.get(), crystal, seed, kSym, /*sigma_a=*/0, /*d_applicable=*/false));
}

TEST(FilterSpec_MatchOrbitInvariant, Prism_PD_N2_AllSigmaA) {
  Crystal crystal = Crystal::CreatePrism(1.0f);
  constexpr uint8_t kSym = FilterConfig::kSymP | FilterConfig::kSymD;
  std::vector<std::vector<IdType>> seeds{ { 3, 5 }, { 4, 6 } };
  for (int sigma_a = 0; sigma_a < 6; sigma_a++) {
    SCOPED_TRACE("sigma_a=" + std::to_string(sigma_a));
    auto spec = MakeComplexSpec(crystal, seeds, kSym, kSigmaARollDeg[sigma_a]);
    ASSERT_NE(spec, nullptr);
    for (const auto& seed : seeds) {
      EXPECT_TRUE(VerifyMatchOrbitInvariant(spec.get(), crystal, seed, kSym, sigma_a, /*d_applicable=*/true));
    }
  }
}

TEST(FilterSpec_MatchOrbitInvariant, Prism_B_N1) {
  Crystal crystal = Crystal::CreatePrism(1.0f);
  constexpr uint8_t kSym = FilterConfig::kSymB;
  std::vector<IdType> seed{ 1, 3 };
  auto spec = MakeRaypathSpec(crystal, seed, kSym, kSigmaARollDeg[0]);
  ASSERT_NE(spec, nullptr);
  EXPECT_TRUE(VerifyMatchOrbitInvariant(spec.get(), crystal, seed, kSym, /*sigma_a=*/0, /*d_applicable=*/false));
}

TEST(FilterSpec_MatchOrbitInvariant, Prism_PDB_N4_AllSigmaA) {
  Crystal crystal = Crystal::CreatePrism(1.0f);
  constexpr uint8_t kSym = FilterConfig::kSymP | FilterConfig::kSymD | FilterConfig::kSymB;
  std::vector<std::vector<IdType>> seeds{ { 1, 3 }, { 2, 5 }, { 3, 5 }, { 4, 6 } };
  for (int sigma_a = 0; sigma_a < 6; sigma_a++) {
    SCOPED_TRACE("sigma_a=" + std::to_string(sigma_a));
    auto spec = MakeComplexSpec(crystal, seeds, kSym, kSigmaARollDeg[sigma_a]);
    ASSERT_NE(spec, nullptr);
    for (const auto& seed : seeds) {
      EXPECT_TRUE(VerifyMatchOrbitInvariant(spec.get(), crystal, seed, kSym, sigma_a, /*d_applicable=*/true));
    }
  }
}

TEST(FilterSpec_MatchOrbitInvariant, Pyramid_P_N1) {
  Crystal crystal = Crystal::CreatePyramid(1.0f, 1.0f, 1.0f);
  constexpr uint8_t kSym = FilterConfig::kSymP;
  std::vector<IdType> seed{ 14, 6 };
  auto spec = MakeRaypathSpec(crystal, seed, kSym, kSigmaARollDeg[0]);
  ASSERT_NE(spec, nullptr);
  EXPECT_TRUE(VerifyMatchOrbitInvariant(spec.get(), crystal, seed, kSym, /*sigma_a=*/0, /*d_applicable=*/false));
}

TEST(FilterSpec_MatchOrbitInvariant, Pyramid_PD_N2_AllSigmaA) {
  Crystal crystal = Crystal::CreatePyramid(1.0f, 1.0f, 1.0f);
  constexpr uint8_t kSym = FilterConfig::kSymP | FilterConfig::kSymD;
  std::vector<std::vector<IdType>> seeds{ { 14, 6 }, { 18, 5 } };
  for (int sigma_a = 0; sigma_a < 6; sigma_a++) {
    SCOPED_TRACE("sigma_a=" + std::to_string(sigma_a));
    auto spec = MakeComplexSpec(crystal, seeds, kSym, kSigmaARollDeg[sigma_a]);
    ASSERT_NE(spec, nullptr);
    for (const auto& seed : seeds) {
      EXPECT_TRUE(VerifyMatchOrbitInvariant(spec.get(), crystal, seed, kSym, sigma_a, /*d_applicable=*/true));
    }
  }
}

TEST(FilterSpec_MatchOrbitInvariant, Pyramid_B_N1) {
  Crystal crystal = Crystal::CreatePyramid(1.0f, 1.0f, 1.0f);
  constexpr uint8_t kSym = FilterConfig::kSymB;
  std::vector<IdType> seed{ 13, 5 };
  auto spec = MakeRaypathSpec(crystal, seed, kSym, kSigmaARollDeg[0]);
  ASSERT_NE(spec, nullptr);
  EXPECT_TRUE(VerifyMatchOrbitInvariant(spec.get(), crystal, seed, kSym, /*sigma_a=*/0, /*d_applicable=*/false));
}

TEST(FilterSpec_MatchOrbitInvariant, Pyramid_PDB_N4_AllSigmaA) {
  Crystal crystal = Crystal::CreatePyramid(1.0f, 1.0f, 1.0f);
  constexpr uint8_t kSym = FilterConfig::kSymP | FilterConfig::kSymD | FilterConfig::kSymB;
  std::vector<std::vector<IdType>> seeds{ { 13, 5 }, { 14, 6 }, { 23, 5 }, { 25, 8 } };
  for (int sigma_a = 0; sigma_a < 6; sigma_a++) {
    SCOPED_TRACE("sigma_a=" + std::to_string(sigma_a));
    auto spec = MakeComplexSpec(crystal, seeds, kSym, kSigmaARollDeg[sigma_a]);
    ASSERT_NE(spec, nullptr);
    for (const auto& seed : seeds) {
      EXPECT_TRUE(VerifyMatchOrbitInvariant(spec.get(), crystal, seed, kSym, sigma_a, /*d_applicable=*/true));
    }
  }
}

// =============== AC-1: anti-pattern guard ===============

// FilterSpec.Match() must NOT re-initialize internals per ray.
//
// Background: InitCrystalSymmetry() was called per-ray in the old hot path.
// task-filter-callers-migration removed every call site; static grep
//   grep -rn "InitCrystalSymmetry" src/ test/ bench/  -> zero hits.
//
// Three-layer defense:
//   1. grep ZERO_HITS: API removal (strongest, prevents reintroduction).
//   2. const interface: Match() is const, so non-mutable members cannot be written.
//   3. This test (behaviour): two passes return bit-identical results, detecting
//      non-deterministic mutable state. Deterministic per-ray reinit (idempotent)
//      can still pass here and is covered by layers 1+2.
TEST(FilterSpec_AntiPattern, NoPerRayReinit_ComplexFilter_PD_SigmaA5) {
  Crystal pyramid = Crystal::CreatePyramid(1.0f, 1.0f, 1.0f);
  constexpr uint8_t kSym = FilterConfig::kSymP | FilterConfig::kSymD;
  // sigma_a=5 (roll_mean=30deg) is the known bug case from fix-d-symmetry-pyramid-skip (#208).
  auto spec = MakeComplexSpec(pyramid, { { 14, 6 }, { 18, 5 } }, kSym, kSigmaARollDeg[5]);
  ASSERT_NE(spec, nullptr);

  std::vector<RaySeg> batch;
  auto orbit = pyramid.ExpandRaypath({ 14, 6 }, kSym, /*sigma_a=*/5, /*d_applicable=*/true);
  for (const auto& rp : orbit) {
    batch.push_back(MakeRay(rp));
  }
  batch.push_back(MakeRay({ 1 }));     // non-member: basal-only
  batch.push_back(MakeRay({ 3, 4 }));  // non-member: prism-prism

  std::vector<bool> pass1;
  pass1.reserve(batch.size());
  for (const auto& r : batch) {
    pass1.push_back(spec->Match(r));
  }
  for (size_t i = 0; i < batch.size(); i++) {
    EXPECT_EQ(spec->Match(batch[i]), pass1[i])
        << "Pass 2 diverged at ray " << i << " - non-deterministic internal state";
  }
}

// =============== AC-3: multi-crystal canonical isolation ===============

TEST(FilterSpec_MultiCrystal, IndependentCanonical) {
  Crystal prism = Crystal::CreatePrism(1.0f);
  Crystal pyramid = Crystal::CreatePyramid(1.0f, 1.0f, 1.0f);
  constexpr uint8_t kSym = FilterConfig::kSymP;
  auto prism_spec = MakeRaypathSpec(prism, { 3, 5 }, kSym, kSigmaARollDeg[0]);
  auto pyr_spec = MakeRaypathSpec(pyramid, { 14, 6 }, kSym, kSigmaARollDeg[0]);
  ASSERT_NE(prism_spec, nullptr);
  ASSERT_NE(pyr_spec, nullptr);

  auto prism_orbit = prism.ExpandRaypath({ 3, 5 }, kSym, /*sigma_a=*/0, /*d_applicable=*/false);
  for (const auto& rp : prism_orbit) {
    EXPECT_TRUE(prism_spec->Match(MakeRay(rp)))
        << "prism orbit member " << FormatRaypath(rp) << " not matched by prism spec";
    EXPECT_FALSE(pyr_spec->Match(MakeRay(rp)))
        << "prism orbit member " << FormatRaypath(rp) << " matched by pyramid spec (canonical leak)";
  }
  auto pyr_orbit = pyramid.ExpandRaypath({ 14, 6 }, kSym, /*sigma_a=*/0, /*d_applicable=*/false);
  for (const auto& rp : pyr_orbit) {
    EXPECT_TRUE(pyr_spec->Match(MakeRay(rp)))
        << "pyramid orbit member " << FormatRaypath(rp) << " not matched by pyramid spec";
    EXPECT_FALSE(prism_spec->Match(MakeRay(rp)))
        << "pyramid orbit member " << FormatRaypath(rp) << " matched by prism spec (canonical leak)";
  }
}

// =============== AC-2: random 1000+ ReduceRecorder vs Crystal::ReduceRaypath ===============

TEST(FilterSpecReduceRecorder, Random1000_AllSymmetries) {
  // Fixed seed: reproducibility hard-requirement (AC-3 no flaky risk).
  constexpr uint32_t kSeed = 20260519u;
  std::mt19937 rng(kSeed);
  Crystal prism = Crystal::CreatePrism(1.0f);
  Crystal pyramid = Crystal::CreatePyramid(1.0f, 1.0f, 1.0f);
  std::vector<std::pair<const Crystal*, uint8_t>> combos = {
    { &prism, FilterConfig::kSymP },
    { &prism, FilterConfig::kSymP | FilterConfig::kSymD },
    { &prism, FilterConfig::kSymP | FilterConfig::kSymD | FilterConfig::kSymB },
    { &pyramid, FilterConfig::kSymP | FilterConfig::kSymD },
  };
  constexpr int kRaysPerCombo = 300;  // 4 * 300 = 1200 > 1000
  std::vector<IdType> prism_faces = { 1, 2, 3, 4, 5, 6, 7, 8 };
  std::vector<IdType> pyramid_faces = { 1, 2, 13, 14, 15, 16, 17, 18, 23, 24, 25, 26, 27, 28, 3, 4, 5, 6, 7, 8 };
  for (auto& [crystal_ptr, sym] : combos) {
    bool is_pyramid = (crystal_ptr == &pyramid);
    auto& faces = is_pyramid ? pyramid_faces : prism_faces;
    std::uniform_int_distribution<int> len_dist(1, 4);
    std::uniform_int_distribution<int> face_dist(0, static_cast<int>(faces.size()) - 1);
    std::uniform_int_distribution<int> sigma_dist(0, 5);
    std::bernoulli_distribution d_app(0.5);
    for (int k = 0; k < kRaysPerCombo; k++) {
      int len = len_dist(rng);
      std::vector<IdType> rp;
      rp.reserve(len);
      for (int l = 0; l < len; l++) {
        rp.push_back(faces[face_dist(rng)]);
      }
      int sigma_a = (sym & FilterConfig::kSymD) ? sigma_dist(rng) : 0;
      bool d_applicable = (sym & FilterConfig::kSymD) ? d_app(rng) : false;
      EXPECT_TRUE(VerifyReduceRecorderOracle(*crystal_ptr, sym, sigma_a, d_applicable, rp))
          << "Failed at k=" << k << " sym=" << static_cast<int>(sym);
    }
  }
}

// =============== DirectionSpec tests (port of DirectionFilter_* legacy tests) ===============

std::unique_ptr<FilterSpec> MakeDirectionSpec(float lon_deg, float lat_deg, float radii_deg,
                                              FilterConfig::Action action = FilterConfig::kFilterIn) {
  FilterConfig cfg{};
  cfg.id_ = 1;
  cfg.symmetry_ = FilterConfig::kSymNone;
  cfg.action_ = action;
  DirectionFilterParam dp{};
  dp.lon_ = lon_deg;
  dp.lat_ = lat_deg;
  dp.radii_ = radii_deg;
  cfg.param_ = SimpleFilterParam{ dp };
  Crystal crystal = Crystal::CreatePrism(1.0f);
  return FilterSpec::Create(cfg, crystal, MakeAxis(0.0f));
}

TEST(DirectionSpec, BasicMatch) {
  auto spec = MakeDirectionSpec(0.0f, 0.0f, 10.0f);

  auto r1 = MakeRay({});
  r1.d_[0] = 1.0f;
  r1.d_[1] = 0.0f;
  r1.d_[2] = 0.0f;
  EXPECT_TRUE(spec->Check(r1));

  auto r2 = MakeRay({});
  r2.d_[0] = 0.0f;
  r2.d_[1] = 1.0f;
  r2.d_[2] = 0.0f;
  EXPECT_FALSE(spec->Check(r2));
}

TEST(DirectionSpec, StateAgnostic) {
  // DirectionSpec reads only d_; is_continue_ and w_ must not affect the result.
  auto spec = MakeDirectionSpec(0.0f, 0.0f, 10.0f);

  auto r = MakeRay({});
  r.d_[0] = 1.0f;
  r.d_[1] = 0.0f;
  r.d_[2] = 0.0f;

  r.is_continue_ = false;
  r.w_ = 1.0f;
  EXPECT_TRUE(spec->Check(r));

  r.is_continue_ = true;
  EXPECT_TRUE(spec->Check(r));

  r.w_ = -1.0f;
  EXPECT_TRUE(spec->Check(r));
}

TEST(DirectionSpec, FilterOut) {
  auto spec = MakeDirectionSpec(0.0f, 0.0f, 10.0f, FilterConfig::kFilterOut);

  auto r1 = MakeRay({});
  r1.d_[0] = 1.0f;
  r1.d_[1] = 0.0f;
  r1.d_[2] = 0.0f;
  EXPECT_FALSE(spec->Check(r1));

  auto r2 = MakeRay({});
  r2.d_[0] = 0.0f;
  r2.d_[1] = 1.0f;
  r2.d_[2] = 0.0f;
  EXPECT_TRUE(spec->Check(r2));

  auto r3 = MakeRay({});
  r3.d_[0] = 1.0f;
  r3.d_[1] = 0.0f;
  r3.d_[2] = 0.0f;
  r3.is_continue_ = true;
  EXPECT_FALSE(spec->Check(r3));
}

}  // namespace
}  // namespace lumice
