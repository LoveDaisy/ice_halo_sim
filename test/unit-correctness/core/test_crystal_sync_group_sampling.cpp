#include <cmath>
#include <cstring>

#include "config/crystal_config.hpp"
#include "core/crystal.hpp"
#include "core/math.hpp"
#include "core/trace_ops.hpp"
#include "gtest/gtest.h"

// Shape-scalar sync groups, core sampling side.
//
// The suite's first job is a hard compatibility gate: declaring no sync group
// must leave the RNG stream byte-identical to what MakeCrystal produced before
// sync groups existed. That gate is written as a CONTRACT REPLAY, not as a table
// of captured values: a second RandomNumberGenerator on the same seed is driven
// by hand, in the order the contract fixes (prism h_ then d[0..5]; pyramid
// h_pyr_u_, h_prs_, h_pyr_l_, then d[0..5] — the draw order, not the declaration
// order), and the two streams are compared value for value. Both run on the same
// standard library inside the same binary, so the comparison is bit-exact on
// every platform; and because the reference is written from the contract rather
// than transcribed from the sampler, reordering a draw or adding/dropping one
// still desynchronizes the streams and fails.
//
// An earlier form of this gate hard-coded hex-float sample values captured on a
// single machine. Only std::mt19937 is portable by specification; the adaptors
// on top of it (normal_distribution, uniform_real_distribution) are
// implementation defined, and normal_distribution's polar method caches a second
// value, so even the NUMBER of engine outputs it consumes varies. Such a table
// can therefore only ever hold on the standard library it was captured on. Same
// defect family as the platform-dependent pyramid oracle retired in
// doc/numerical-robustness.md §8, and the same resolution: align on the
// contract, retire the absolute oracle.

namespace {

namespace ns = lumice;

// Consecutive draws replayed per test — one rng instance consumed continuously,
// so the sampler's per-crystal state (the sync-group cache) is exercised across
// crystals rather than only on a fresh sampler.
constexpr int kContractReplayDraws = 20;

// Prism fixture: rng seed 20260726, h_ = Gaussian(1.0, 0.3), all six
// face_distance = Uniform(center 1.0, spread 0.6), sync_group_ all zero.
constexpr uint32_t kPrismGoldenSeed = 20260726u;

// Pyramid fixture: rng seed 20260727, h_pyr_u_ = Gaussian(0.3, 0.1),
// h_prs_ = Uniform(center 1.0, spread 0.4), h_pyr_l_ = Laplacian(0.3, 0.1) —
// three deliberately different distribution types, so a mixed-up draw order
// between the three heights cannot pass unnoticed. All six face_distance =
// Gaussian(1.0, 0.5), sync_group_ all zero.
constexpr uint32_t kPyramidGoldenSeed = 20260727u;

ns::PrismCrystalParam GoldenPrismParam() {
  ns::PrismCrystalParam p;
  p.h_ = ns::Distribution{ ns::DistributionType::kGaussian, 1.0f, 0.3f };
  for (auto& d : p.d_) {
    d = ns::Distribution{ ns::DistributionType::kUniform, 1.0f, 0.6f };
  }
  return p;
}

ns::PyramidCrystalParam GoldenPyramidParam() {
  ns::PyramidCrystalParam p;
  p.h_pyr_u_ = ns::Distribution{ ns::DistributionType::kGaussian, 0.3f, 0.1f };
  p.h_prs_ = ns::Distribution{ ns::DistributionType::kUniform, 1.0f, 0.4f };
  p.h_pyr_l_ = ns::Distribution{ ns::DistributionType::kLaplacian, 0.3f, 0.1f };
  for (auto& d : p.d_) {
    d = ns::Distribution{ ns::DistributionType::kGaussian, 1.0f, 0.5f };
  }
  return p;
}

bool CfGeomBytesEqual(const ns::CrystalGeom& a, const ns::CrystalGeom& b) {
  return std::memcmp(&a, &b, sizeof(ns::CrystalGeom)) == 0;
}

// ==================================================================================================
// AC1 — no sync declaration must not perturb the RNG stream, value for value.

TEST(ShapeScalarSyncGroupSampling, PrismNoSyncMatchesContractOrder) {
  const ns::PrismCrystalParam p = GoldenPrismParam();
  ns::RandomNumberGenerator rng(kPrismGoldenSeed);
  ns::RandomNumberGenerator ref(kPrismGoldenSeed);

  // One rng instance each, consumed continuously across the draws — the
  // lifecycle MakeCrystal runs under. Rebuilding them per draw would reset both
  // streams and make every row identical.
  for (int draw = 0; draw < kContractReplayDraws; draw++) {
    // Hand-replayed contract stream: h_, then d[0..5] — seven draws in this
    // order. std::abs folds the height only, matching the fold site inside
    // SamplePrismShapeScalars; face_distance stays signed.
    const float exp_h = std::abs(ref.Get(p.h_));
    float exp_dist[6]{};
    for (int i = 0; i < 6; i++) {
      exp_dist[i] = ref.Get(p.d_[i]);
    }

    float dist[6]{};
    const float h = ns::SamplePrismShapeScalars(rng, p, dist);
    ASSERT_EQ(h, exp_h) << "prism height diverged at draw " << draw;
    for (int i = 0; i < 6; i++) {
      ASSERT_EQ(dist[i], exp_dist[i]) << "prism face_distance[" << i << "] diverged at draw " << draw;
    }
  }

  // Both streams must still sit at the same position: a sampler that drew an
  // extra value (or one fewer) somewhere could in principle still have matched
  // every assertion above only if the surplus draw came last, which this catches.
  ASSERT_EQ(rng.GetUniform(), ref.GetUniform()) << "prism sampling consumed a different number of RNG draws";
}

TEST(ShapeScalarSyncGroupSampling, PyramidNoSyncMatchesContractOrder) {
  const ns::PyramidCrystalParam p = GoldenPyramidParam();
  ns::RandomNumberGenerator rng(kPyramidGoldenSeed);
  ns::RandomNumberGenerator ref(kPyramidGoldenSeed);

  for (int draw = 0; draw < kContractReplayDraws; draw++) {
    // Contract stream: upper, prism, lower height, then d[0..5] — the draw
    // order, not the declaration order of PyramidCrystalParam. All three
    // heights fold with std::abs, the face distances do not.
    const float exp_h1 = std::abs(ref.Get(p.h_pyr_u_));
    const float exp_h2 = std::abs(ref.Get(p.h_prs_));
    const float exp_h3 = std::abs(ref.Get(p.h_pyr_l_));
    float exp_dist[6]{};
    for (int i = 0; i < 6; i++) {
      exp_dist[i] = ref.Get(p.d_[i]);
    }

    float h1 = 0.f;
    float h2 = 0.f;
    float h3 = 0.f;
    float dist[6]{};
    ns::SamplePyramidShapeScalars(rng, p, h1, h2, h3, dist);
    ASSERT_EQ(h1, exp_h1) << "pyramid upper_h diverged at draw " << draw;
    ASSERT_EQ(h2, exp_h2) << "pyramid prism_h diverged at draw " << draw;
    ASSERT_EQ(h3, exp_h3) << "pyramid lower_h diverged at draw " << draw;
    for (int i = 0; i < 6; i++) {
      ASSERT_EQ(dist[i], exp_dist[i]) << "pyramid face_distance[" << i << "] diverged at draw " << draw;
    }
  }

  ASSERT_EQ(rng.GetUniform(), ref.GetUniform()) << "pyramid sampling consumed a different number of RNG draws";
}

// The two tests above pin the extracted sampler against the contract. These two
// pin the *full MakeCrystal pipeline* against the same contract, so a divergence
// between "what the sampler draws" and "what MakeCrystal actually builds a
// crystal from" cannot hide: MakeCrystal is the entry point all three backends
// use, the sampler is only its first half.
TEST(ShapeScalarSyncGroupSampling, PrismMakeCrystalAgreesWithContractGeometry) {
  const ns::PrismCrystalParam p = GoldenPrismParam();
  ns::RandomNumberGenerator rng(kPrismGoldenSeed);
  ns::RandomNumberGenerator ref(kPrismGoldenSeed);

  for (int draw = 0; draw < kContractReplayDraws; draw++) {
    const float ref_h = std::abs(ref.Get(p.h_));
    float ref_dist[6]{};
    for (int i = 0; i < 6; i++) {
      ref_dist[i] = ref.Get(p.d_[i]);
    }

    const ns::Crystal from_pipeline = ns::MakeCrystal(rng, ns::CrystalParam{ p });
    const ns::Crystal from_contract = ns::Crystal::CreatePrism(ref_h, ref_dist);
    EXPECT_TRUE(CfGeomBytesEqual(from_pipeline.CfGeom(), from_contract.CfGeom()))
        << "MakeCrystal geometry differs from the contract-order scalars' geometry at draw " << draw;
  }
}

TEST(ShapeScalarSyncGroupSampling, PyramidMakeCrystalAgreesWithContractGeometry) {
  const ns::PyramidCrystalParam p = GoldenPyramidParam();
  ns::RandomNumberGenerator rng(kPyramidGoldenSeed);
  ns::RandomNumberGenerator ref(kPyramidGoldenSeed);

  for (int draw = 0; draw < kContractReplayDraws; draw++) {
    const float ref_h1 = std::abs(ref.Get(p.h_pyr_u_));
    const float ref_h2 = std::abs(ref.Get(p.h_prs_));
    const float ref_h3 = std::abs(ref.Get(p.h_pyr_l_));
    float ref_dist[6]{};
    for (int i = 0; i < 6; i++) {
      ref_dist[i] = ref.Get(p.d_[i]);
    }

    const ns::Crystal from_pipeline = ns::MakeCrystal(rng, ns::CrystalParam{ p });
    const ns::Crystal from_contract =
        ns::Crystal::CreatePyramid(p.wedge_angle_u_, p.wedge_angle_l_, ref_h1, ref_h2, ref_h3, ref_dist);
    EXPECT_TRUE(CfGeomBytesEqual(from_pipeline.CfGeom(), from_contract.CfGeom()))
        << "MakeCrystal geometry differs from the contract-order scalars' geometry at draw " << draw;
  }
}

// ==================================================================================================
// AC2 — members of one group share one value and one draw.

TEST(ShapeScalarSyncGroupSampling, PrismGroupSharesOneValueAndOneDraw) {
  constexpr uint32_t kSeed = 4242u;
  // Group {FACE_0, FACE_2, FACE_4}: the C3 case, one of two alternating groups.
  // Group members carry the same distribution (this test exercises the sampler,
  // not NormalizeSyncGroups); the independent slots carry a very different one so
  // a wrong draw order shows up as a wrong magnitude, not just a wrong bit.
  const ns::Distribution grouped{ ns::DistributionType::kUniform, 0.0f, 2.0f };
  const ns::Distribution loner{ ns::DistributionType::kGaussian, 50.0f, 1.0f };

  ns::PrismCrystalParam p;
  p.h_ = loner;
  p.d_[0] = p.d_[2] = p.d_[4] = grouped;
  p.d_[1] = p.d_[3] = p.d_[5] = loner;
  p.sync_group_[ns::kShapeScalarFace0] = 1;
  p.sync_group_[ns::kShapeScalarFace2] = 1;
  p.sync_group_[ns::kShapeScalarFace4] = 1;

  ns::RandomNumberGenerator rng(kSeed);
  ns::RandomNumberGenerator ref(kSeed);

  // Two consecutive draws: the group cache must be per-crystal, so draw 1 has to
  // re-draw the group rather than reuse draw 0's value.
  for (int draw = 0; draw < 2; draw++) {
    // Hand-replayed expected stream: h, d0, d1, d3, d5 — five draws, in this
    // order. d2 and d4 consume nothing and reuse d0.
    const float exp_h = std::abs(ref.Get(p.h_));
    const float exp_d0 = ref.Get(p.d_[0]);
    const float exp_d1 = ref.Get(p.d_[1]);
    const float exp_d3 = ref.Get(p.d_[3]);
    const float exp_d5 = ref.Get(p.d_[5]);

    float dist[6]{};
    const float h = ns::SamplePrismShapeScalars(rng, p, dist);

    ASSERT_EQ(h, exp_h) << "draw " << draw;
    ASSERT_EQ(dist[0], exp_d0) << "draw " << draw;
    ASSERT_EQ(dist[1], exp_d1) << "draw " << draw;
    ASSERT_EQ(dist[2], exp_d0) << "grouped face 2 must reuse face 0's value bit-for-bit, draw " << draw;
    ASSERT_EQ(dist[3], exp_d3) << "draw " << draw;
    ASSERT_EQ(dist[4], exp_d0) << "grouped face 4 must reuse face 0's value bit-for-bit, draw " << draw;
    ASSERT_EQ(dist[5], exp_d5) << "draw " << draw;
  }

  // The two streams are still in lockstep, which is the actual draw-count
  // assertion: the sampler consumed exactly the 5 draws replayed above per
  // crystal. Had the grouped slots each drawn (7 per crystal), rng would be 4
  // draws ahead of ref and this would fail.
  ASSERT_EQ(rng.GetUniform(), ref.GetUniform()) << "grouped sampling consumed a different number of RNG draws";
}

TEST(ShapeScalarSyncGroupSampling, PyramidTwoGroupsAlternateIndependently) {
  constexpr uint32_t kSeed = 909u;
  // Full C3 habit: {FACE_0, FACE_2, FACE_4} and {FACE_1, FACE_3, FACE_5} each
  // sync, so the hexagon is (a, b, a, b, a, b) — a single checkbox-style "sync
  // all" could never express this.
  ns::PyramidCrystalParam p = GoldenPyramidParam();
  p.sync_group_[ns::kShapeScalarFace0] = 1;
  p.sync_group_[ns::kShapeScalarFace2] = 1;
  p.sync_group_[ns::kShapeScalarFace4] = 1;
  p.sync_group_[ns::kShapeScalarFace1] = 2;
  p.sync_group_[ns::kShapeScalarFace3] = 2;
  p.sync_group_[ns::kShapeScalarFace5] = 2;
  // Upper and lower pyramidal heights sync too (the symmetric-pyramid case).
  p.sync_group_[ns::kShapeScalarUpperH] = 3;
  p.sync_group_[ns::kShapeScalarLowerH] = 3;
  p.h_pyr_l_ = p.h_pyr_u_;

  ns::RandomNumberGenerator rng(kSeed);
  for (int draw = 0; draw < 8; draw++) {
    float h1 = 0.f;
    float h2 = 0.f;
    float h3 = 0.f;
    float dist[6]{};
    ns::SamplePyramidShapeScalars(rng, p, h1, h2, h3, dist);
    ASSERT_EQ(dist[0], dist[2]) << "draw " << draw;
    ASSERT_EQ(dist[0], dist[4]) << "draw " << draw;
    ASSERT_EQ(dist[1], dist[3]) << "draw " << draw;
    ASSERT_EQ(dist[1], dist[5]) << "draw " << draw;
    ASSERT_EQ(h1, h3) << "upper/lower pyramidal heights share a group, draw " << draw;
    // Distinct groups must not collapse onto each other — a cache keyed on
    // "any non-zero group" instead of the group id would make these equal.
    ASSERT_NE(dist[0], dist[1]) << "the two face groups must draw independently, draw " << draw;
  }
}

// ==================================================================================================
// A group spanning a height and a face distance shares the raw draw; the
// height member folds it with std::abs at its own use site.

TEST(ShapeScalarSyncGroupSampling, MixedHeightFaceGroupSharesRawDrawAndFoldsHeightOnly) {
  ns::PrismCrystalParam p;
  // Centered well below zero with a tight spread, so the shared draw is
  // negative with overwhelming probability — that is what makes the abs()
  // asymmetry observable at all.
  p.h_ = ns::Distribution{ ns::DistributionType::kGaussian, -1.0f, 0.01f };
  p.d_[1] = p.h_;
  for (int i = 0; i < 6; i++) {
    if (i != 1) {
      p.d_[i] = ns::Distribution{ ns::DistributionType::kNoRandom, 1.0f, 0.0f };
    }
  }
  p.sync_group_[ns::kShapeScalarHeight] = 1;
  p.sync_group_[ns::kShapeScalarFace1] = 1;

  ns::RandomNumberGenerator rng(31337u);
  for (int draw = 0; draw < 8; draw++) {
    float dist[6]{};
    const float h = ns::SamplePrismShapeScalars(rng, p, dist);
    ASSERT_LT(dist[1], 0.f) << "fixture precondition: the shared draw must be negative, draw " << draw;
    ASSERT_EQ(h, std::abs(dist[1])) << "height must be the absolute value of the group's shared raw draw, draw "
                                    << draw;
  }
}

}  // namespace
