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
#include <string>
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

}  // namespace
}  // namespace lumice
