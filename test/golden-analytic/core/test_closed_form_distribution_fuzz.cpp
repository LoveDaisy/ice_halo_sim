// Distribution-level fuzz for the closed-form prism and pyramid evaluators.
//
// WHY THIS EXISTS, and why it is a separate file from the golden tests.
//
// test_closed_form_{prism,pyramid}.cpp assert exact agreement on FIXED, pre-selected shape pools.
// Those pools are selected by structural margin: a well-conditioned entry has every corner pair
// separated by at least 50x the closed form's merge tolerance, a degenerate entry has some pair
// inside 1x. The selection is what makes an exact assertion legitimate there — with the boundary
// that far away, agreement is not a matter of which sample was drawn.
//
// It also leaves the whole band BETWEEN 1x and 50x uncovered, by construction. That band is where
// merge-strategy divergence actually lives, and it is exactly what the pools cannot contain
// without giving up the exactness that justifies them. It used to be covered by a random sweep of
// 100000 well-conditioned and 40000-per-sigma degenerate samples; that sweep was removed because
// it asserted EXPECT_EQ(mismatch, 0) on samples drawn through std::normal_distribution, whose
// sequence differs per standard library, so CI went red on libstdc++ for a divergence libc++ never
// drew. Deleting it was the right emergency call and left a real hole. This file refills the hole
// without reopening the defect, by changing both halves of what went wrong:
//
//   * The samples are drawn through support/portable_random.hpp, straight off mt19937, whose
//     output sequence the standard fixes. Every platform therefore fuzzes the SAME shapes — a
//     failure here reproduces everywhere rather than depending on who ran it.
//   * The comparative assertions are ratio ceilings, not equalities. Closed form and production
//     resolve near-coincident corners differently by design; over a broad draw some samples land
//     inside that difference, and demanding zero would be demanding the two implementations share
//     a merge strategy they deliberately do not.
//
// The two assertion shapes here are not interchangeable, and the distinction is the whole lesson:
//
//   * A UNIVERSAL INVARIANT — finiteness, index bounds, a convex-polygon identity — is true for
//     every legal input or the code is broken. Those keep `EXPECT_EQ(count, 0)`. Random sampling
//     cannot make such an assertion flaky; it can only decide how many chances it gets to fire.
//   * A COMPARISON MEDIATED BY A TOLERANCE gets a ceiling. Its outcome depends on how close the
//     drawn shape sits to the merge boundary, so a bound on the RATE is the strongest honest
//     statement available.
//
// Ceilings below are set from a measured run, with the measurement and its margin stated at each
// one. They are gates on a regression in the rate, not descriptions of the current rate.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <random>
#include <tuple>

#include "core/geo3d.hpp"
#include "core/geo3d_closedform.hpp"
#include "core/math.hpp"
#include "support/exact_prism_oracle.hpp"
#include "support/portable_random.hpp"

namespace lumice {
namespace {

constexpr int kPrismSideCnt = kClosedFormPrismSideCnt;

// Sample counts. Deliberately not the pre-removal 100000/40000: the point of the sweep is to
// resolve a rate, and the rates measured here are percents, not parts-per-million, so a few
// thousand samples put the sampling error an order of magnitude under every ceiling's margin
// while keeping the whole file inside a second. Raising N would buy a tighter estimate of a
// number no gate reads.
constexpr int kPrismSamplesPerTier = 4000;
constexpr int kPyramidSamples = 4000;

// Shape note: this result struct and PyramidFuzzResult below are both "universal-invariant counts +
// oracle/production comparison counts", differing only in field names and dimensionality (2D corner
// vs 3D vertex/face). Two call sites do not clear this repo's bar for extracting a shared template —
// leave them as they are. If a third closed-form geometry ever needs the same kind of distribution
// fuzz, that is the trigger to fold this shape into one shared counter type instead of copying again.
struct PrismTierResult {
  int drawn = 0;
  int evaluated = 0;    // oracle did not refuse
  int refused = 0;      // int64 guard tripped — not a defect, an input outside the budget
  int mismatch = 0;     // closed form vs exact oracle corner count
  int nonfinite = 0;    // universal invariant: a corner coordinate was not finite
  int ring_broken = 0;  // universal invariant: #present side faces != corner_cnt
  int prod_compared = 0;
  int prod_empty = 0;
  int prod_diff = 0;  // closed form vs production vertex count
};

PrismTierResult RunPrismTier(uint32_t seed, float sigma, int n) {
  PrismTierResult r{};
  std::mt19937 rng(seed);
  for (int s = 0; s < n; ++s) {
    float dist[kPrismSideCnt];
    for (float& d : dist) {
      d = test::PortableGaussianFloat(rng, 1.0f, sigma);
    }
    r.drawn++;

    const auto cf = ComputeClosedFormPrism(1.0f, dist);

    // Universal invariant 1: every emitted corner is a finite number. A non-finite coordinate is
    // the signature of an empty feasible region escaping its guard, and no input makes it legal.
    for (int c = 0; c < cf.corner_cnt; ++c) {
      if (!std::isfinite(cf.corner_x[c]) || !std::isfinite(cf.corner_y[c])) {
        r.nonfinite++;
        break;
      }
    }
    // Universal invariant 2: in a convex cross-section every present side face contributes exactly
    // two corners and every corner is shared by exactly two present side faces, so the two counts
    // are equal. This is pure combinatorics — it cannot be softened by a tolerance.
    int present_sides = 0;
    for (int i = 0; i < kPrismSideCnt; ++i) {
      if (cf.face_present[2 + i]) {
        present_sides++;
      }
    }
    if (present_sides != cf.corner_cnt) {
      r.ring_broken++;
    }

    const auto ex = test_support::ExactPrism(dist);
    if (ex.refused) {
      r.refused++;
      continue;
    }
    r.evaluated++;
    if (cf.corner_cnt != ex.corner_count) {
      r.mismatch++;
    }

    // Second comparison, against the production solver rather than the integer oracle. This is the
    // arm with a nonzero rate: production dedups vertices with its own tolerance, so the two
    // disagree on shapes near a corner coincidence. It is also the arm the removed sweep covered.
    float coef[kMaxHexCrystalPlanes * 4];
    const size_t plane_cnt = FillHexCrystalCoef(0.0f, 0.0f, 0.0f, 1.0f, 0.0f, dist, coef);
    if (plane_cnt == 0) {
      r.prod_empty++;
      continue;
    }
    // `prod_vtx` is a std::unique_ptr<float[]> (src/core/math.hpp): owning, freshly heap-allocated
    // per call and freed by RAII at scope exit — no leak, but this loop does pay one alloc/free per
    // iteration since only the vertex count is compared below. Accepted at these tier sizes (this
    // file's whole run measures well under a second); revisit only if this loop's N grows enough for
    // the allocation cost to matter.
    auto [prod_vtx, prod_cnt] = SolveConvexPolyhedronVtxD(static_cast<int>(plane_cnt), coef);
    (void)prod_vtx;
    if (prod_cnt == 0) {
      r.prod_empty++;
      continue;
    }
    r.prod_compared++;
    // Each 2D corner lifts to two 3D vertices (top and bottom cap).
    if (prod_cnt != cf.corner_cnt * 2) {
      r.prod_diff++;
    }
  }
  return r;
}

// sigma tiers mirror the gaussian tiers the prism fuzz in test_crystal.cpp sweeps, so the two
// layers describe the same input regimes: 0.15 stays inside the healthy path, 0.80 is deep in the
// near-coincident-corner territory the merge tolerance governs.
struct PrismTier {
  const char* label;
  uint32_t seed;
  float sigma;
  double oracle_ceiling;  // cf vs exact oracle, fraction of evaluated samples
  double prod_ceiling;    // cf vs production, fraction of compared samples
};

TEST(ClosedFormDistributionFuzz, PrismCornerCountTracksTheExactOracleAcrossTheSigmaSweep) {
  // Measured on the committed seeds, 4000 samples per tier (the same lines are printed on every
  // run, so drift is visible without reading this comment):
  //
  //   vs exact oracle : 0 divergences out of 16000 evaluated, at every sigma.
  //   vs production   : 1 divergence out of 15433 compared — a single sample at sigma=0.30.
  //
  // That single event is the whole argument for this file. It is not a defect: production dedups
  // vertices with a tolerance the zero-tolerance closed-form corner ring does not share, so shapes
  // sitting near a corner coincidence resolve differently, which is what the degenerate pool in
  // test_closed_form_prism.cpp characterises deliberately. What it shows is that an unselected
  // draw DOES reach that band — roughly once in fifteen thousand shapes — so the removed sweep's
  // EXPECT_EQ(mismatch, 0) was never a safe assertion at these sample counts, on any standard
  // library. Whichever one drew such a sample first went red, and the fix at the time was to stop
  // drawing rather than to stop demanding zero.
  //
  // The ceiling is one common value, 0.001, i.e. 4 divergences per 4000-sample tier. Deliberately
  // an absolute-count bar rather than a multiple of the measured rate: the measured rate is 0 in
  // three of four tiers, and 3x zero is zero, which would be the removed assertion wearing a
  // ceiling's clothes. It can be this tight because the sample source is deterministic on every
  // platform — there is no sampling noise for the margin to absorb, so the only thing that moves
  // these counts is a change in the code under test.
  //
  // Calibrated against a deliberate regression rather than guessed. Multiplying this evaluator's
  // corner-merge tolerance by 20 — a drift small enough that all five existing prism tests stay
  // green, because the fixed pools sit at 50x margin or inside 1x and a 20x shift lands between —
  // moves these counts to 0 / 4 / 9 / 8 across the four tiers. At 4 per tier the ceiling catches
  // that at three of the four sigmas; at 8 it would have caught it at one, barely. The gap between
  // those two numbers is the whole reason this file exists: the band between 1x and 50x is
  // invisible to every other test of this evaluator.
  static const PrismTier kTiers[] = {
    { "sigma=0.15", 0xD15701u, 0.15f, 0.001, 0.001 },
    { "sigma=0.30", 0xD15702u, 0.30f, 0.001, 0.001 },
    { "sigma=0.50", 0xD15703u, 0.50f, 0.001, 0.001 },
    { "sigma=0.80", 0xD15704u, 0.80f, 0.001, 0.001 },
  };

  for (const PrismTier& t : kTiers) {
    const PrismTierResult r = RunPrismTier(t.seed, t.sigma, kPrismSamplesPerTier);
    const double rate = (r.evaluated > 0) ? static_cast<double>(r.mismatch) / r.evaluated : 0.0;
    const double prod_rate = (r.prod_compared > 0) ? static_cast<double>(r.prod_diff) / r.prod_compared : 0.0;
    std::fprintf(stderr,
                 "[distribution-fuzz] prism %-10s drawn=%d evaluated=%d refused=%d oracle_diff=%d "
                 "rate=%.4f | prod_compared=%d prod_empty=%d prod_diff=%d prod_rate=%.4f\n",
                 t.label, r.drawn, r.evaluated, r.refused, r.mismatch, rate, r.prod_compared, r.prod_empty, r.prod_diff,
                 prod_rate);

    // Universal invariants: exact zero, on purpose. See the file header for why these do not get a
    // ceiling while the comparison below does.
    EXPECT_EQ(r.nonfinite, 0) << t.label << ": closed form emitted a non-finite corner";
    EXPECT_EQ(r.ring_broken, 0) << t.label << ": #present side faces != corner_cnt";

    // Anti-vacuity: a tier that evaluated nothing would satisfy every assertion above.
    EXPECT_GT(r.evaluated, kPrismSamplesPerTier / 2)
        << t.label << ": the oracle refused most of the tier, so this tier asserted almost nothing";

    // The tolerance-mediated comparison. EXPECT_LT, not EXPECT_EQ: the closed form merges
    // near-coincident corners the zero-tolerance integer oracle keeps distinct, so a nonzero rate
    // at wide sigma is the designed behaviour, not a defect.
    EXPECT_LT(rate, t.oracle_ceiling) << t.label
                                      << ": corner-count divergence from the exact oracle rose above its ceiling ("
                                      << r.mismatch << " / " << r.evaluated << ")";
    EXPECT_LT(prod_rate, t.prod_ceiling) << t.label
                                         << ": vertex-count divergence from production rose above its ceiling ("
                                         << r.prod_diff << " / " << r.prod_compared << ")";
  }
}

struct PyramidFuzzResult {
  int drawn = 0;
  int emitted = 0;         // cf produced at least one present face
  int nonfinite = 0;       // universal: a vertex or plane coefficient was not finite
  int index_oob = 0;       // universal: a face referenced a vertex outside the pool
  int face_ring_bad = 0;   // universal: present face with < 3 vertices, or absent face with any
  int prod_empty = 0;      // production rejected the shape (its own gate, not a defect here)
  int vtx_count_diff = 0;  // closed form vs production vertex count
  int compared = 0;        // both sides produced something
};

PyramidFuzzResult RunPyramidFuzz(uint32_t seed, int n) {
  PyramidFuzzResult r{};
  std::mt19937 rng(seed);
  for (int s = 0; s < n; ++s) {
    // Wedge angles span the legal range including the extreme-flat tail above 87 degrees, which is
    // where the B-ring defect family lived. Heights cover truncated, apex-reaching and one-sided
    // cones. dist[] carries the same gaussian irregularity the prism tiers use.
    const float upper_alpha = test::PortableUniformFloat(rng, 1.0f, 89.5f);
    const float lower_alpha = test::PortableUniformFloat(rng, 1.0f, 89.5f);
    const float h1 = test::PortableUniformFloat(rng, 0.0f, 1.0f);
    const float h2 = test::PortableUniformFloat(rng, 0.0f, 2.0f);
    const float h3 = test::PortableUniformFloat(rng, 0.0f, 1.0f);
    float dist[kPrismSideCnt];
    for (float& d : dist) {
      d = test::PortableGaussianFloat(rng, 1.0f, 0.3f);
    }
    r.drawn++;

    const auto cf = ComputeClosedFormPyramid(upper_alpha, lower_alpha, h1, h2, h3, dist);

    int present = 0;
    bool bad_nonfinite = false;
    bool bad_index = false;
    bool bad_ring = false;
    for (int i = 0; i < cf.vtx_cnt; ++i) {
      if (!std::isfinite(cf.vtx[i * 3 + 0]) || !std::isfinite(cf.vtx[i * 3 + 1]) || !std::isfinite(cf.vtx[i * 3 + 2])) {
        bad_nonfinite = true;
        break;
      }
    }
    for (int f = 0; f < kClosedFormPyramidFaceCnt; ++f) {
      for (int k = 0; k < 4; ++k) {
        if (!std::isfinite(cf.plane_coef[f * 4 + k])) {
          bad_nonfinite = true;
        }
      }
      if (!cf.face_present[f]) {
        // The struct documents face_vtx_cnt as 0 for an absent face. A stale ring left behind on a
        // dropped face is how a consumer walks vertices that no longer bound anything.
        if (cf.face_vtx_cnt[f] != 0) {
          bad_ring = true;
        }
        continue;
      }
      present++;
      if (cf.face_vtx_cnt[f] < 3) {
        bad_ring = true;
      }
      for (int k = 0; k < cf.face_vtx_cnt[f]; ++k) {
        const int vi = cf.face_vtx[f][k];
        if (vi < 0 || vi >= cf.vtx_cnt) {
          bad_index = true;
        }
      }
    }
    r.nonfinite += bad_nonfinite ? 1 : 0;
    r.index_oob += bad_index ? 1 : 0;
    r.face_ring_bad += bad_ring ? 1 : 0;
    if (present == 0) {
      continue;
    }
    r.emitted++;

    // Production path, for the cross-implementation half.
    float coef[kMaxHexCrystalPlanes * 4];
    const size_t plane_cnt = FillHexCrystalCoef(upper_alpha, lower_alpha, h1, h2, h3, dist, coef);
    if (plane_cnt == 0) {
      r.prod_empty++;
      continue;
    }
    // `prod_vtx` is a std::unique_ptr<float[]> (src/core/math.hpp): owning, freshly heap-allocated
    // per call and freed by RAII at scope exit — no leak, but this loop does pay one alloc/free per
    // iteration since only the vertex count is compared below. Accepted at these tier sizes (this
    // file's whole run measures well under a second); revisit only if this loop's N grows enough for
    // the allocation cost to matter.
    auto [prod_vtx, prod_cnt] = SolveConvexPolyhedronVtxD(static_cast<int>(plane_cnt), coef);
    (void)prod_vtx;
    if (prod_cnt == 0) {
      r.prod_empty++;
      continue;
    }
    r.compared++;
    if (prod_cnt != cf.vtx_cnt) {
      r.vtx_count_diff++;
    }
  }
  return r;
}

TEST(ClosedFormDistributionFuzz, PyramidStaysStructurallyValidAndNearProductionAcrossTheParamSpace) {
  const PyramidFuzzResult r = RunPyramidFuzz(0xD15705u, kPyramidSamples);
  const double diff_rate = (r.compared > 0) ? static_cast<double>(r.vtx_count_diff) / r.compared : 0.0;
  std::fprintf(stderr,
               "[distribution-fuzz] pyramid drawn=%d emitted=%d compared=%d prod_empty=%d "
               "vtx_count_diff=%d rate=%.4f\n",
               r.drawn, r.emitted, r.compared, r.prod_empty, r.vtx_count_diff, diff_rate);

  // Universal invariants. These are the propositions the fixed pools also assert, extended here to
  // shapes nobody selected — which is the coverage this file exists to restore. A structurally
  // invalid polyhedron (an apex recorded on every side face, a lost basal face, a face ring
  // pointing outside the vertex pool) is wrong for every input, so zero is the right bar.
  EXPECT_EQ(r.nonfinite, 0) << "closed form emitted a non-finite vertex or plane coefficient";
  EXPECT_EQ(r.index_oob, 0) << "a present face referenced a vertex index outside the pool";
  EXPECT_EQ(r.face_ring_bad, 0) << "a present face carried fewer than 3 vertices, or an absent face carried any";

  // Anti-vacuity, both directions: the sweep must actually reach shapes the evaluator accepts, and
  // must actually get to compare them against production.
  EXPECT_GT(r.emitted, kPyramidSamples / 2) << "most draws produced no present face — the parameter box has drifted "
                                               "off the legal region and this test is asserting almost nothing";
  EXPECT_GT(r.compared, kPyramidSamples / 4) << "production rejected most of the sweep, so the cross-implementation "
                                                "half asserted almost nothing";

  // Tolerance-mediated: production's vertex dedup and the closed form's corner merge use different
  // tolerances, so their vertex counts legitimately differ on shapes near a coincidence. Measured
  // 22 / 4000 = 0.55%; ceiling at 2% is roughly 3.6x that, which a structural regression clears by
  // an order of magnitude. The pyramid rate is two orders above the prism's because the parameter
  // box swept here is far wider — both wedge angles run to the extreme-flat tail past 87 degrees,
  // where the cone planes approach the basal plane and near-coincident vertices are ordinary
  // rather than rare.
  EXPECT_LT(diff_rate, 0.02) << "vertex-count divergence from production rose above its ceiling (" << r.vtx_count_diff
                             << " / " << r.compared << ")";
}

}  // namespace
}  // namespace lumice
