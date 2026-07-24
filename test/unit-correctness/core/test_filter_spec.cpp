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
    // Tests build recorders via ToRecorder which feeds operator<< (inline-only),
    // so reading data_ directly is safe — sizes here are < kInlineCap.
    s += std::to_string(static_cast<int>(rp.data_[i]));
  }
  s += "}";
  return s;
}

::testing::AssertionResult VerifyReduceRecorderOracle(const Crystal& crystal, uint8_t symmetry, int sigma_a,
                                                      bool d_applicable, const std::vector<IdType>& rp_seed) {
  auto oracle_vec = crystal.ReduceRaypath(rp_seed, symmetry, sigma_a, d_applicable);
  auto oracle_rec = ToRecorder(oracle_vec);

  auto actual = ToRecorder(rp_seed);
  // ToRecorder yields inline-only recorders (rp_seed ≤ 5 elements), so it is
  // safe to canonicalise data_ in-place via the buffer-level ReduceBuffer.
  detail::ReduceBuffer(actual.data_, actual.size_, symmetry, sigma_a, d_applicable);

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
  d.azimuth_dist.spread = 360.0f;
  d.azimuth_dist.center = 0.0f;
  d.latitude_dist.type = DistributionType::kNoRandom;
  d.latitude_dist.center = 90.0f;
  d.roll_dist.type = DistributionType::kNoRandom;
  d.roll_dist.center = roll_mean_deg;
  d.roll_dist.spread = 0.0f;
  return d;
}

RaySeg MakeRay() {
  // RaypathRecorder lives on RayBuffer::recorders_ now; build it separately
  // via ToRecorder(rp_vec) when a Match()/Check() call needs it.
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
  return r;
}

// Convenience wrappers: build (RaySeg, RaypathRecorder) from a raypath face
// vector and invoke spec->Match / spec->Check. Reduces call-site noise after
// the SoA-prize split moved RaypathRecorder out of RaySeg.
inline bool SpecMatch(const FilterSpec* spec, const std::vector<IdType>& rp_vec) {
  return spec->Match(MakeRay(), ToRecorder(rp_vec));
}
inline bool SpecCheck(const FilterSpec* spec, const std::vector<IdType>& rp_vec) {
  return spec->Check(MakeRay(), ToRecorder(rp_vec));
}

// NOTE: MakeRay() returns a fixed direction/origin RaySeg; the raypath structure
// these helpers used to encode via brace-list args was already discarded by
// MakeRay's body. Callers that need raypath identity pass it directly into
// ToRecorder()/SpecMatch()/SpecCheck(). The batch size and shape are what
// downstream sweeps consume — see e.g. NoPerRayReinit_*.
std::vector<RaySeg> BuildPrismRayBatch() {
  std::vector<RaySeg> rays;
  // empty raypath
  rays.push_back(MakeRay());
  // single-segment basal + prism
  rays.push_back(MakeRay());
  rays.push_back(MakeRay());
  // pairs (a,b) ∈ {3..8}²
  for (IdType a = 3; a <= 8; a++) {
    for (IdType b = 3; b <= 8; b++) {
      if (a == b) {
        continue;
      }
      rays.push_back(MakeRay());
    }
  }
  // include basal+prism mixes
  rays.push_back(MakeRay());
  rays.push_back(MakeRay());
  rays.push_back(MakeRay());
  return rays;
}

std::vector<RaySeg> BuildPyramidRayBatch() {
  std::vector<RaySeg> rays;
  // pyramid + prism pairs
  for (IdType u = 13; u <= 18; u++) {
    for (IdType p = 3; p <= 8; p++) {
      rays.push_back(MakeRay());
    }
  }
  for (IdType l = 23; l <= 28; l++) {
    rays.push_back(MakeRay());
  }
  rays.push_back(MakeRay());
  rays.push_back(MakeRay());
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
// Precondition: the filter was built from seed_rp, so SpecMatch(spec.get(), seed_rp)
// must be true; if not, the FilterSpec was misconfigured and the orbit sweep would
// be vacuously satisfied with expected=false for all members.
::testing::AssertionResult VerifyMatchOrbitInvariant(FilterSpec* spec, const Crystal& crystal,
                                                     const std::vector<IdType>& seed_rp, uint8_t symmetry, int sigma_a,
                                                     bool d_applicable) {
  auto orbit = crystal.ExpandRaypath(seed_rp, symmetry, sigma_a, d_applicable);
  if (orbit.empty()) {
    return ::testing::AssertionFailure() << "ExpandRaypath returned empty orbit for seed=" << FormatRaypath(seed_rp);
  }
  RaySeg seed_ray = MakeRay();
  if (!spec->Match(seed_ray, ToRecorder(seed_rp))) {
    return ::testing::AssertionFailure() << "Seed raypath " << FormatRaypath(seed_rp)
                                         << " did not match its own filter (sym=" << static_cast<int>(symmetry)
                                         << " sigma_a=" << sigma_a << ") - filter misconfigured, orbit sweep aborted";
  }
  for (size_t i = 0; i < orbit.size(); i++) {
    RaySeg ri = MakeRay();
    if (!spec->Match(ri, ToRecorder(orbit[i]))) {
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
  std::vector<RaypathRecorder> batch_rec;
  auto orbit = pyramid.ExpandRaypath({ 14, 6 }, kSym, /*sigma_a=*/5, /*d_applicable=*/true);
  for (const auto& rp : orbit) {
    batch.push_back(MakeRay());
    batch_rec.push_back(ToRecorder(rp));
  }
  batch.push_back(MakeRay());  // non-member: basal-only
  batch_rec.push_back(ToRecorder({ 1 }));
  batch.push_back(MakeRay());  // non-member: prism-prism
  batch_rec.push_back(ToRecorder({ 3, 4 }));

  std::vector<bool> pass1;
  pass1.reserve(batch.size());
  for (size_t i = 0; i < batch.size(); i++) {
    pass1.push_back(spec->Match(batch[i], batch_rec[i]));
  }
  for (size_t i = 0; i < batch.size(); i++) {
    EXPECT_EQ(spec->Match(batch[i], batch_rec[i]), pass1[i])
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
    EXPECT_TRUE(SpecMatch(prism_spec.get(), rp))
        << "prism orbit member " << FormatRaypath(rp) << " not matched by prism spec";
    EXPECT_FALSE(SpecMatch(pyr_spec.get(), rp))
        << "prism orbit member " << FormatRaypath(rp) << " matched by pyramid spec (canonical leak)";
  }
  auto pyr_orbit = pyramid.ExpandRaypath({ 14, 6 }, kSym, /*sigma_a=*/0, /*d_applicable=*/false);
  for (const auto& rp : pyr_orbit) {
    EXPECT_TRUE(SpecMatch(pyr_spec.get(), rp))
        << "pyramid orbit member " << FormatRaypath(rp) << " not matched by pyramid spec";
    EXPECT_FALSE(SpecMatch(prism_spec.get(), rp))
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
  cfg.symmetry_ = FilterConfig::kSymNone;  // DirectionSpec checks direction only; symmetry is irrelevant
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

  auto r1 = MakeRay();
  r1.d_[0] = 1.0f;
  r1.d_[1] = 0.0f;
  r1.d_[2] = 0.0f;
  EXPECT_TRUE(spec->Check(r1, RaypathRecorder{}));

  auto r2 = MakeRay();
  r2.d_[0] = 0.0f;
  r2.d_[1] = 1.0f;
  r2.d_[2] = 0.0f;
  EXPECT_FALSE(spec->Check(r2, RaypathRecorder{}));
}

TEST(DirectionSpec, StateAgnostic) {
  // DirectionSpec reads only d_; is_continue_ and w_ must not affect the result.
  auto spec = MakeDirectionSpec(0.0f, 0.0f, 10.0f);

  auto r = MakeRay();
  r.d_[0] = 1.0f;
  r.d_[1] = 0.0f;
  r.d_[2] = 0.0f;

  r.is_continue_ = false;
  r.w_ = 1.0f;
  EXPECT_TRUE(spec->Check(r, RaypathRecorder{}));

  r.is_continue_ = true;
  EXPECT_TRUE(spec->Check(r, RaypathRecorder{}));

  r.w_ = -1.0f;
  EXPECT_TRUE(spec->Check(r, RaypathRecorder{}));
}

TEST(DirectionSpec, FilterOut) {
  auto spec = MakeDirectionSpec(0.0f, 0.0f, 10.0f, FilterConfig::kFilterOut);

  auto r1 = MakeRay();
  r1.d_[0] = 1.0f;
  r1.d_[1] = 0.0f;
  r1.d_[2] = 0.0f;
  EXPECT_FALSE(spec->Check(r1, RaypathRecorder{}));

  auto r2 = MakeRay();
  r2.d_[0] = 0.0f;
  r2.d_[1] = 1.0f;
  r2.d_[2] = 0.0f;
  EXPECT_TRUE(spec->Check(r2, RaypathRecorder{}));

  auto r3 = MakeRay();
  r3.d_[0] = 1.0f;
  r3.d_[1] = 0.0f;
  r3.d_[2] = 0.0f;
  r3.is_continue_ = true;
  EXPECT_FALSE(spec->Check(r3, RaypathRecorder{}));
}

// =============== EntryExitSpec Match matrix ===============
//
// Cover the four wildcard combinations (entry/exit ∈ {value, nullopt})
// crossed with length-mode variations (no bound / strict / upper / range /
// same-face reflection). Also pin orbit-invariance under PBD when both ends
// are set (regression guard for the legacy single-value behavior).

std::unique_ptr<FilterSpec> MakeEESpec(const Crystal& crystal, std::optional<IdType> entry, std::optional<IdType> exit,
                                       size_t min_len = 1, std::optional<size_t> max_len = std::nullopt,
                                       uint8_t symmetry = FilterConfig::kSymNone,
                                       float roll_mean_deg = kSigmaARollDeg[0]) {
  FilterConfig cfg{};
  cfg.id_ = 1;
  cfg.symmetry_ = symmetry;
  cfg.action_ = FilterConfig::kFilterIn;
  EntryExitFilterParam p{};
  p.entry_ = entry;
  p.exit_ = exit;
  p.min_len_ = min_len;
  p.max_len_ = max_len;
  cfg.param_ = SimpleFilterParam{ p };
  return FilterSpec::Create(cfg, crystal, MakeAxis(roll_mean_deg));
}

TEST(EntryExitSpec_Match, LegacySingleValue_Match) {
  Crystal prism = Crystal::CreatePrism(1.0f);
  auto spec = MakeEESpec(prism, IdType{ 3 }, IdType{ 5 });
  ASSERT_NE(spec, nullptr);
  EXPECT_TRUE(SpecMatch(spec.get(), { 3, 5 }));
  EXPECT_TRUE(SpecMatch(spec.get(), { 3, 4, 5 }));
  EXPECT_FALSE(SpecMatch(spec.get(), { 3, 4, 6 }));
  EXPECT_FALSE(SpecMatch(spec.get(), { 4, 5 }));
}

TEST(EntryExitSpec_Match, WildcardEntry_OnlyExitConstrains) {
  Crystal prism = Crystal::CreatePrism(1.0f);
  auto spec = MakeEESpec(prism, std::nullopt, IdType{ 5 });
  ASSERT_NE(spec, nullptr);
  EXPECT_TRUE(SpecMatch(spec.get(), { 3, 5 }));
  EXPECT_TRUE(SpecMatch(spec.get(), { 7, 5 }));
  EXPECT_TRUE(SpecMatch(spec.get(), { 5 }));      // size=1: ee={5} matches
  EXPECT_FALSE(SpecMatch(spec.get(), { 3, 4 }));  // exit != 5
  EXPECT_FALSE(SpecMatch(spec.get(), { 5, 7 }));  // last = 7
}

TEST(EntryExitSpec_Match, WildcardExit_OnlyEntryConstrains) {
  Crystal prism = Crystal::CreatePrism(1.0f);
  auto spec = MakeEESpec(prism, IdType{ 3 }, std::nullopt);
  ASSERT_NE(spec, nullptr);
  EXPECT_TRUE(SpecMatch(spec.get(), { 3, 5 }));
  EXPECT_TRUE(SpecMatch(spec.get(), { 3, 4, 7 }));
  EXPECT_TRUE(SpecMatch(spec.get(), { 3 }));
  EXPECT_FALSE(SpecMatch(spec.get(), { 4, 5 }));  // entry != 3
}

TEST(EntryExitSpec_Match, DoubleWildcard_AcceptsAnyNonEmpty) {
  Crystal prism = Crystal::CreatePrism(1.0f);
  auto spec = MakeEESpec(prism, std::nullopt, std::nullopt);
  ASSERT_NE(spec, nullptr);
  EXPECT_TRUE(SpecMatch(spec.get(), { 3 }));
  EXPECT_TRUE(SpecMatch(spec.get(), { 3, 5 }));
  EXPECT_TRUE(SpecMatch(spec.get(), { 1, 2, 4 }));
  EXPECT_FALSE(SpecMatch(spec.get(), {}));  // empty path always rejected
}

TEST(EntryExitSpec_Match, SameFaceReflection_size1_Match) {
  // Regression guard: entry=exit, single-hit reflection must match.
  Crystal prism = Crystal::CreatePrism(1.0f);
  auto spec = MakeEESpec(prism, IdType{ 3 }, IdType{ 3 });
  ASSERT_NE(spec, nullptr);
  EXPECT_TRUE(SpecMatch(spec.get(), { 3 }));
  EXPECT_TRUE(SpecMatch(spec.get(), { 3, 5, 3 }));  // also matches longer paths starting+ending on 3
  EXPECT_FALSE(SpecMatch(spec.get(), { 3, 5 }));    // entry=3, exit=5
}

TEST(EntryExitSpec_Match, StrictLength_Match) {
  Crystal prism = Crystal::CreatePrism(1.0f);
  auto spec = MakeEESpec(prism, IdType{ 3 }, IdType{ 5 }, /*min_len=*/2, /*max_len=*/2);
  ASSERT_NE(spec, nullptr);
  EXPECT_TRUE(SpecMatch(spec.get(), { 3, 5 }));      // size=2 strict
  EXPECT_FALSE(SpecMatch(spec.get(), { 3, 4, 5 }));  // size=3 rejected
  EXPECT_FALSE(SpecMatch(spec.get(), { 3 }));        // size=1 < min
}

TEST(EntryExitSpec_Match, UpperBound_Match) {
  Crystal prism = Crystal::CreatePrism(1.0f);
  auto spec = MakeEESpec(prism, IdType{ 3 }, IdType{ 5 }, /*min_len=*/1, /*max_len=*/3);
  ASSERT_NE(spec, nullptr);
  EXPECT_TRUE(SpecMatch(spec.get(), { 3, 5 }));
  EXPECT_TRUE(SpecMatch(spec.get(), { 3, 4, 5 }));
  EXPECT_FALSE(SpecMatch(spec.get(), { 3, 4, 6, 5 }));  // size=4 > max
}

TEST(EntryExitSpec_Match, RangeBound_WildcardEnds) {
  // Double-wildcard + length range [2,4]: accept any path with 2..4 hits.
  Crystal prism = Crystal::CreatePrism(1.0f);
  auto spec = MakeEESpec(prism, std::nullopt, std::nullopt, /*min_len=*/2, /*max_len=*/4);
  ASSERT_NE(spec, nullptr);
  EXPECT_FALSE(SpecMatch(spec.get(), { 3 }));  // size=1 < min
  EXPECT_TRUE(SpecMatch(spec.get(), { 3, 5 }));
  EXPECT_TRUE(SpecMatch(spec.get(), { 3, 4, 5 }));
  EXPECT_TRUE(SpecMatch(spec.get(), { 3, 4, 6, 5 }));
  EXPECT_FALSE(SpecMatch(spec.get(), { 3, 4, 5, 6, 7 }));  // size=5 > max
}

TEST(EntryExitSpec_Match, NoUpperBoundDefault_AcceptsLongPath) {
  Crystal prism = Crystal::CreatePrism(1.0f);
  auto spec = MakeEESpec(prism, IdType{ 3 }, IdType{ 5 });  // min_len=1, max_len=nullopt
  ASSERT_NE(spec, nullptr);
  std::vector<IdType> long_path{ 3 };
  for (int i = 0; i < 60; i++) {  // up to ~kMaxHits=64
    long_path.push_back(static_cast<IdType>(4 + (i % 4)));
  }
  long_path.push_back(5);
  EXPECT_TRUE(SpecMatch(spec.get(), long_path));
}

TEST(EntryExitSpec_Match, EmptyRay_RejectedRegardlessOfWildcards) {
  Crystal prism = Crystal::CreatePrism(1.0f);
  for (bool e_wild : { false, true }) {
    for (bool x_wild : { false, true }) {
      auto spec = MakeEESpec(prism, e_wild ? std::optional<IdType>{} : std::optional<IdType>{ 3 },
                             x_wild ? std::optional<IdType>{} : std::optional<IdType>{ 5 });
      ASSERT_NE(spec, nullptr);
      EXPECT_FALSE(SpecMatch(spec.get(), {})) << "e_wild=" << e_wild << " x_wild=" << x_wild;
    }
  }
}

TEST(EntryExitSpec_Match, MinLenAboveSize_Rejects) {
  Crystal prism = Crystal::CreatePrism(1.0f);
  auto spec = MakeEESpec(prism, std::nullopt, std::nullopt, /*min_len=*/3, /*max_len=*/std::nullopt);
  ASSERT_NE(spec, nullptr);
  EXPECT_FALSE(SpecMatch(spec.get(), { 3 }));
  EXPECT_FALSE(SpecMatch(spec.get(), { 3, 5 }));
  EXPECT_TRUE(SpecMatch(spec.get(), { 3, 5, 7 }));
}

// PBD symmetry: every orbit member of (entry=3, exit=5) under PBD must
// trigger Match()=true when the spec is configured with the seed pair.
TEST(EntryExitSpec_Match, PBD_OrbitInvariant) {
  Crystal prism = Crystal::CreatePrism(1.0f);
  constexpr uint8_t kSym = FilterConfig::kSymP | FilterConfig::kSymD | FilterConfig::kSymB;
  for (int sigma_a = 0; sigma_a < 6; sigma_a++) {
    auto spec = MakeEESpec(prism, IdType{ 3 }, IdType{ 5 }, /*min_len=*/1, /*max_len=*/std::nullopt, kSym,
                           kSigmaARollDeg[sigma_a]);
    ASSERT_NE(spec, nullptr);
    auto orbit = prism.ExpandRaypath({ 3, 5 }, kSym, sigma_a, /*d_applicable=*/true);
    ASSERT_FALSE(orbit.empty());
    for (size_t i = 0; i < orbit.size(); i++) {
      SCOPED_TRACE("sigma_a=" + std::to_string(sigma_a) + " orbit[" + std::to_string(i) +
                   "]=" + FormatRaypath(orbit[i]));
      EXPECT_TRUE(SpecMatch(spec.get(), orbit[i]));
    }
  }
}

// =============== task-host-abi-cpu-caps AC2: CPU runtime honors > 16 OR clauses ===============

// The core FilterSpec CPU runtime uses `std::vector<std::vector<unique_ptr>>` internally
// (filter_spec.cpp:325), so its OR-clause capacity was never actually bounded by the ABI
// LUMICE_MAX_CONFIG_CLAUSES=16 — the cap only lived in the ABI layout. This test proves
// that CPU-side Match() semantics work correctly on a ComplexSpec built with N > 16 real
// OR-clauses (only one of which corresponds to the raypath being probed), i.e. the CPU
// path was ready for widening all along and the plan §2 "widen ABI to unlock runtime"
// framing holds.
TEST(FilterSpec_ManyOrClauses, MatchOnlyMatchingClause) {
  Crystal crystal = Crystal::CreatePrism(1.0f);
  constexpr uint8_t kSym = FilterConfig::kSymNone;
  // Build 30 distinct real raypaths (> the pre-v4.9 ABI cap of 16), varying face numbers over
  // valid prism range [1, 8]. Then Match against the single raypath at index 24 — only its
  // matching clause should be satisfied; every other clause must reject it (proving that
  // ComplexSpec-OR does not spuriously match beyond the intended clause).
  std::vector<std::vector<IdType>> seeds;
  for (int i = 0; i < 30; i++) {
    IdType f0 = static_cast<IdType>(1 + (i % 8));
    IdType f1 = static_cast<IdType>(1 + ((i + 3) % 8));
    seeds.push_back({ f0, f1 });
  }
  auto spec = MakeComplexSpec(crystal, seeds, kSym, kSigmaARollDeg[0]);
  ASSERT_NE(spec, nullptr);

  // Probe against each seed: MUST match (its own clause is in the OR).
  for (size_t i = 0; i < seeds.size(); i++) {
    SCOPED_TRACE("seed[" + std::to_string(i) + "]=" + FormatRaypath(seeds[i]));
    EXPECT_TRUE(SpecMatch(spec.get(), seeds[i])) << "OR clause " << i << " (>16) not matched by its own seed";
  }

  // Probe against a raypath that does NOT appear in any of the 30 clauses: MUST reject.
  // Use a 3-hop raypath that cannot be produced by any 2-hop seed.
  std::vector<IdType> non_member = { 3, 5, 7 };
  EXPECT_FALSE(SpecMatch(spec.get(), non_member))
      << "ComplexSpec spuriously matched a raypath outside all " << seeds.size() << " OR clauses";
}

}  // namespace
}  // namespace lumice
