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
// sync groups existed. The kPrism*/kPyramid* golden arrays below were captured
// from that pre-change implementation (temporary hex-float dump inside
// CrystalMaker, one rng instance, 20 consecutive MakeCrystal calls), so they are
// an external oracle rather than a transcription of the new code's own behavior.
// If they ever need regenerating, that is a red flag, not a chore: every seeded
// reference in the repo (golden / parity / e2e) moves with this stream.

namespace {

namespace ns = lumice;

constexpr int kGoldenDraws = 20;

// Prism golden: rng seed 20260726, h_ = Gaussian(1.0, 0.3), all six
// face_distance = Uniform(center 1.0, spread 0.6), sync_group_ all zero.
constexpr uint32_t kPrismGoldenSeed = 20260726u;

constexpr float kPrismGoldenH[kGoldenDraws] = {
  0x1.a13438p-1f, 0x1.39f4c2p-1f, 0x1.24d3e2p-1f, 0x1.748ad4p-1f, 0x1.c446fap-1f, 0x1.0deacap+0f, 0x1.8fd5ep-1f,
  0x1.b1c74p+0f,  0x1.84aa8cp-1f, 0x1.558e3ep-1f, 0x1.354ba2p-1f, 0x1.873076p-1f, 0x1.01221ap-1f, 0x1.2f66b8p+0f,
  0x1.7a0706p-1f, 0x1.992eeap+0f, 0x1.fa15f6p-2f, 0x1.d9bffcp-1f, 0x1.501b9p-1f,  0x1.bbba1ap-1f,
};
constexpr float kPrismGoldenD[kGoldenDraws][6] = {
  { 0x1.baab4p-1f, 0x1.168e16p+0f, 0x1.d08ap-1f, 0x1.4871fap+0f, 0x1.3e0ba4p+0f, 0x1.32e02cp+0f },
  { 0x1.7b4f8p-1f, 0x1.fe6176p-1f, 0x1.9bf48ep-1f, 0x1.148a0ep+0f, 0x1.6a2718p-1f, 0x1.c623e8p-1f },
  { 0x1.2cbdf6p+0f, 0x1.2df93cp+0f, 0x1.372e78p+0f, 0x1.0936ccp+0f, 0x1.39aa72p+0f, 0x1.6d2fe6p-1f },
  { 0x1.0eb29ap+0f, 0x1.093b6p+0f, 0x1.0c32e6p+0f, 0x1.37b9ecp+0f, 0x1.36b908p+0f, 0x1.96854cp-1f },
  { 0x1.3dcb94p+0f, 0x1.673de2p-1f, 0x1.3eea96p+0f, 0x1.28f61cp+0f, 0x1.b25016p-1f, 0x1.c93b7ep-1f },
  { 0x1.e0fac2p-1f, 0x1.2949f2p+0f, 0x1.1ffdd4p+0f, 0x1.397fe4p+0f, 0x1.94118p-1f, 0x1.cdcc5ep-1f },
  { 0x1.9479a2p-1f, 0x1.b89218p-1f, 0x1.1b34bcp+0f, 0x1.21dc06p+0f, 0x1.0d74bcp+0f, 0x1.099feap+0f },
  { 0x1.19271ap+0f, 0x1.e33a4ep-1f, 0x1.1afdb8p+0f, 0x1.1cf0c2p+0f, 0x1.412f1p+0f, 0x1.2e6c52p+0f },
  { 0x1.0de706p+0f, 0x1.38eb18p+0f, 0x1.bd1b74p-1f, 0x1.07eb0ap+0f, 0x1.2d0a9cp+0f, 0x1.c3f55ep-1f },
  { 0x1.1bf264p+0f, 0x1.2f14c8p+0f, 0x1.32adb8p+0f, 0x1.3b773ep+0f, 0x1.1c1d7ap+0f, 0x1.427252p+0f },
  { 0x1.45401cp+0f, 0x1.a4984ep-1f, 0x1.42f59p+0f, 0x1.e5b6b2p-1f, 0x1.0ed59ap+0f, 0x1.26cf34p+0f },
  { 0x1.88d9d8p-1f, 0x1.3155fcp+0f, 0x1.c73324p-1f, 0x1.f08432p-1f, 0x1.3ac3aep+0f, 0x1.d6ee94p-1f },
  { 0x1.f0da26p-1f, 0x1.0259e4p+0f, 0x1.fbbf2ep-1f, 0x1.cd3404p-1f, 0x1.4acb46p+0f, 0x1.d6059p-1f },
  { 0x1.49b702p+0f, 0x1.66a51ap-1f, 0x1.9b6a44p-1f, 0x1.e6f32p-1f, 0x1.2875eep+0f, 0x1.a4844ap-1f },
  { 0x1.49e9acp+0f, 0x1.d9ee96p-1f, 0x1.2d758ep+0f, 0x1.1a9b04p+0f, 0x1.a4d6f2p-1f, 0x1.17e6fep+0f },
  { 0x1.1e24ap+0f, 0x1.ea83e4p-1f, 0x1.6f1c26p-1f, 0x1.0604f8p+0f, 0x1.3a3578p+0f, 0x1.30f41p+0f },
  { 0x1.fec928p-1f, 0x1.8e5e74p-1f, 0x1.c773b4p-1f, 0x1.ef983ap-1f, 0x1.94658p-1f, 0x1.88760cp-1f },
  { 0x1.0fdc5ep+0f, 0x1.80544cp-1f, 0x1.97a49cp-1f, 0x1.257b3cp+0f, 0x1.bb6bc4p-1f, 0x1.2e5fb8p+0f },
  { 0x1.2864e8p+0f, 0x1.433df6p+0f, 0x1.ce2f64p-1f, 0x1.f0d9a8p-1f, 0x1.cf4f7cp-1f, 0x1.2adfd8p+0f },
  { 0x1.11d8a4p+0f, 0x1.0506d2p+0f, 0x1.21f06cp+0f, 0x1.a33a82p-1f, 0x1.93754p-1f, 0x1.153592p+0f },
};

// Pyramid golden: rng seed 20260727, h_pyr_u_ = Gaussian(0.3, 0.1),
// h_prs_ = Uniform(center 1.0, spread 0.4), h_pyr_l_ = Laplacian(0.3, 0.1) —
// three deliberately different distribution types, so a mixed-up draw order
// between the three heights cannot pass unnoticed. All six face_distance =
// Gaussian(1.0, 0.5), sync_group_ all zero.
constexpr uint32_t kPyramidGoldenSeed = 20260727u;

constexpr float kPyramidGoldenH1[kGoldenDraws] = {
  0x1.764b88p-2f, 0x1.85ee1ep-2f, 0x1.8dcc86p-3f, 0x1.8d6cd8p-2f, 0x1.862c92p-2f, 0x1.bccc7cp-2f, 0x1.fd17f2p-3f,
  0x1.3c7eaap-3f, 0x1.893176p-4f, 0x1.f00f8cp-3f, 0x1.0498d4p-2f, 0x1.a73358p-2f, 0x1.ca30eep-2f, 0x1.2c1962p-2f,
  0x1.35e4a4p-2f, 0x1.fb92b6p-3f, 0x1.359ab6p-2f, 0x1.34c628p-2f, 0x1.881ddcp-2f, 0x1.45e57ap-2f,
};
constexpr float kPyramidGoldenH2[kGoldenDraws] = {
  0x1.1f995ap+0f, 0x1.cbaa3cp-1f, 0x1.1c552cp+0f, 0x1.1aae94p+0f, 0x1.05e6dep+0f, 0x1.1c2648p+0f, 0x1.02a6aap+0f,
  0x1.25893ap+0f, 0x1.2d012p+0f,  0x1.26db9p+0f,  0x1.12bb64p+0f, 0x1.a78cdp-1f,  0x1.9dfe0ep-1f, 0x1.11eac6p+0f,
  0x1.097a04p+0f, 0x1.19b9d2p+0f, 0x1.f2b008p-1f, 0x1.c4c67cp-1f, 0x1.ab68f2p-1f, 0x1.2bdb4ap+0f,
};
constexpr float kPyramidGoldenH3[kGoldenDraws] = {
  0x1.aaed8ep-2f, 0x1.e81ap-4f,   0x1.258016p-2f, 0x1.490ddep-2f, 0x1.2f71e6p-2f, 0x1.d33a5cp-3f, 0x1.3a1654p-2f,
  0x1.7ece8p-2f,  0x1.25678p-2f,  0x1.718ce4p-2f, 0x1.833d48p-3f, 0x1.382d64p-2f, 0x1.d0ee5cp-2f, 0x1.0ef692p-3f,
  0x1.431cdcp-4f, 0x1.2eb044p-2f, 0x1.6303cp-2f,  0x1.e6b0d6p-4f, 0x1.2acd68p-2f, 0x1.ed6e32p-3f,
};
constexpr float kPyramidGoldenD[kGoldenDraws][6] = {
  { 0x1.77b53p+0f, 0x1.cad678p-2f, 0x1.53d0f8p-1f, 0x1.f37f5p-2f, 0x1.14429p+1f, 0x1.a089eep-1f },
  { 0x1.48d38p-6f, 0x1.ee3578p-1f, 0x1.65ef64p+0f, 0x1.a08d74p-1f, 0x1.6f6ad2p-1f, 0x1.139276p+0f },
  { 0x1.d62dap-2f, 0x1.d65be4p-1f, 0x1.ac5834p-2f, 0x1.25633ep-1f, 0x1.7c9316p+0f, 0x1.23d864p-1f },
  { 0x1.28d99cp+0f, 0x1.14aa22p+0f, 0x1.b549dap-1f, 0x1.ea8e84p-2f, 0x1.5d1c7ep-1f, -0x1.60eacp-5f },
  { 0x1.2585e4p-2f, 0x1.e1dbecp-1f, 0x1.dcb7cp-1f, 0x1.a8b88p+0f, 0x1.5b0cf4p+0f, 0x1.00e098p-2f },
  { 0x1.61c2eep-1f, 0x1.05197cp+0f, 0x1.4aa888p-1f, 0x1.b0e758p-1f, 0x1.b2d7ecp-2f, 0x1.22678cp-1f },
  { 0x1.7b0bp+0f, 0x1.5d48ep+0f, 0x1.25d24p-2f, 0x1.a0108cp-2f, 0x1.443d86p+0f, 0x1.10b9fcp+0f },
  { 0x1.ccdefap+0f, 0x1.f9ecccp-1f, 0x1.3ef062p+1f, 0x1.f1a694p-1f, 0x1.99e0f6p+0f, 0x1.246cecp+1f },
  { 0x1.4b3aacp+0f, 0x1.2958e6p+0f, 0x1.b1e866p-1f, 0x1.06f37ep+0f, 0x1.77aa4p-1f, 0x1.536a88p+1f },
  { 0x1.0e5c2ep+0f, 0x1.474bf4p+0f, 0x1.555696p+0f, 0x1.956e76p-1f, 0x1.f14ac8p+0f, 0x1.eb6654p-1f },
  { 0x1.2405f2p-1f, 0x1.91859ep+0f, 0x1.49589ap+0f, 0x1.9e3c62p-1f, 0x1.db26dcp+0f, 0x1.7605ccp+0f },
  { 0x1.8278ep+0f, 0x1.131f22p+1f, 0x1.1fa8cp+0f, 0x1.427faep+0f, 0x1.3db826p+0f, 0x1.0b5ffp-2f },
  { 0x1.01d42cp-1f, 0x1.5dd4dep-1f, 0x1.492a9cp+0f, 0x1.8647d4p-1f, 0x1.0a6636p+0f, 0x1.2a0618p+0f },
  { 0x1.5790e8p-1f, 0x1.b58cfcp-1f, 0x1.ed8696p-1f, 0x1.57e4cp-1f, 0x1.0f66a6p+0f, 0x1.c73d4cp-1f },
  { 0x1.80e57p+0f, 0x1.2862ep-1f, 0x1.0f8e5ep+1f, 0x1.7e8d58p-2f, 0x1.7a023cp-1f, 0x1.2eeb62p+0f },
  { 0x1.38af62p-1f, 0x1.855638p+0f, 0x1.0881c6p-1f, 0x1.625268p-1f, 0x1.638412p+0f, 0x1.57064ap+0f },
  { 0x1.2b869ep+0f, 0x1.0f2646p+0f, 0x1.4a598p-1f, 0x1.031cep+0f, 0x1.cd4998p+0f, 0x1.fdc336p+0f },
  { 0x1.0584c8p-1f, 0x1.89a9dp+0f, 0x1.13448ep+0f, 0x1.516788p-2f, 0x1.07c95p+0f, 0x1.69c4f6p+0f },
  { 0x1.08b95p-1f, 0x1.064a02p+1f, 0x1.6c7e5p+0f, 0x1.d5842ep-1f, 0x1.06741p+0f, 0x1.73fc64p+0f },
  { 0x1.af4544p-2f, 0x1.c15a4p+0f, 0x1.4d0fc8p+0f, 0x1.fba91p-2f, 0x1.205d4cp-1f, 0x1.952238p+0f },
};

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

TEST(ShapeScalarSyncGroupSampling, PrismNoSyncMatchesPreChangeGolden) {
  const ns::PrismCrystalParam p = GoldenPrismParam();
  ns::RandomNumberGenerator rng(kPrismGoldenSeed);

  // One rng instance, consumed continuously across the draws — exactly the
  // lifecycle the golden was captured under. Rebuilding it per draw would reset
  // the stream and make every row identical.
  for (int draw = 0; draw < kGoldenDraws; draw++) {
    float dist[6]{};
    const float h = ns::SamplePrismShapeScalars(rng, p, dist);
    ASSERT_EQ(h, kPrismGoldenH[draw]) << "prism height diverged at draw " << draw;
    for (int i = 0; i < 6; i++) {
      ASSERT_EQ(dist[i], kPrismGoldenD[draw][i]) << "prism face_distance[" << i << "] diverged at draw " << draw;
    }
  }
}

TEST(ShapeScalarSyncGroupSampling, PyramidNoSyncMatchesPreChangeGolden) {
  const ns::PyramidCrystalParam p = GoldenPyramidParam();
  ns::RandomNumberGenerator rng(kPyramidGoldenSeed);

  for (int draw = 0; draw < kGoldenDraws; draw++) {
    float h1 = 0.f;
    float h2 = 0.f;
    float h3 = 0.f;
    float dist[6]{};
    ns::SamplePyramidShapeScalars(rng, p, h1, h2, h3, dist);
    // h1/h2/h3 are upper / prism / lower — the draw order, not the declaration
    // order of PyramidCrystalParam.
    ASSERT_EQ(h1, kPyramidGoldenH1[draw]) << "pyramid upper_h diverged at draw " << draw;
    ASSERT_EQ(h2, kPyramidGoldenH2[draw]) << "pyramid prism_h diverged at draw " << draw;
    ASSERT_EQ(h3, kPyramidGoldenH3[draw]) << "pyramid lower_h diverged at draw " << draw;
    for (int i = 0; i < 6; i++) {
      ASSERT_EQ(dist[i], kPyramidGoldenD[draw][i]) << "pyramid face_distance[" << i << "] diverged at draw " << draw;
    }
  }
}

// The two tests above pin the extracted sampler against the golden. These two
// pin the *full MakeCrystal pipeline* against the same golden, so a divergence
// between "what the sampler draws" and "what MakeCrystal actually builds a
// crystal from" cannot hide: MakeCrystal is the entry point all three backends
// use, the sampler is only its first half.
TEST(ShapeScalarSyncGroupSampling, PrismMakeCrystalAgreesWithGoldenGeometry) {
  const ns::PrismCrystalParam p = GoldenPrismParam();
  ns::RandomNumberGenerator rng(kPrismGoldenSeed);

  for (int draw = 0; draw < kGoldenDraws; draw++) {
    const ns::Crystal from_pipeline = ns::MakeCrystal(rng, ns::CrystalParam{ p });
    float golden_dist[6]{};
    std::memcpy(golden_dist, kPrismGoldenD[draw], sizeof(golden_dist));
    const ns::Crystal from_golden = ns::Crystal::CreatePrism(kPrismGoldenH[draw], golden_dist);
    EXPECT_TRUE(CfGeomBytesEqual(from_pipeline.CfGeom(), from_golden.CfGeom()))
        << "MakeCrystal geometry differs from the golden scalars' geometry at draw " << draw;
  }
}

TEST(ShapeScalarSyncGroupSampling, PyramidMakeCrystalAgreesWithGoldenGeometry) {
  const ns::PyramidCrystalParam p = GoldenPyramidParam();
  ns::RandomNumberGenerator rng(kPyramidGoldenSeed);

  for (int draw = 0; draw < kGoldenDraws; draw++) {
    const ns::Crystal from_pipeline = ns::MakeCrystal(rng, ns::CrystalParam{ p });
    float golden_dist[6]{};
    std::memcpy(golden_dist, kPyramidGoldenD[draw], sizeof(golden_dist));
    const ns::Crystal from_golden =
        ns::Crystal::CreatePyramid(p.wedge_angle_u_, p.wedge_angle_l_, kPyramidGoldenH1[draw], kPyramidGoldenH2[draw],
                                   kPyramidGoldenH3[draw], golden_dist);
    EXPECT_TRUE(CfGeomBytesEqual(from_pipeline.CfGeom(), from_golden.CfGeom()))
        << "MakeCrystal geometry differs from the golden scalars' geometry at draw " << draw;
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
