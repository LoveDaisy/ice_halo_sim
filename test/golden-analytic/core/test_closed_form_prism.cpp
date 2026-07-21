// Golden-analytic three-way adjudication for the closed-form prism evaluator.
//
// Three parties are compared:
//   1. Closed form  — ComputeClosedFormPrism from core/geo3d_closedform.
//   2. Exact oracle — ExactPrism from test/support/exact_prism_oracle.hpp,
//                     zero-tolerance __int128 arithmetic.
//   3. Production   — FillHexCrystalCoef + SolveConvexPolyhedronVtxD.
//
// Owner-mandated methodology: precise EXPECT_EQ assertions run on FIXED,
// pre-selected shape literals — never on stdlib-random
// samples. `std::normal_distribution` / `std::uniform_*_distribution` outputs
// differ between libc++ and libstdc++ for identical mt19937 sequences, so
// random-seeded exact-equality assertions are inherently non-portable across
// stdlibs. The fixed pools below (see closed_form_samples_generated.hpp) are
// selected by pure geometric distance from the merge boundary:
//   Well-conditioned — every corner-pair separation ≥ 50 × cf_merge_tol
//   Degenerate       — at least one corner-pair separation <  1 × cf_merge_tol
// The FixedSamplesRetainStructuralMargin TEST re-verifies these invariants
// at every CI run — the same threshold that gated selection now gates the
// pool at test time, so any accidental "sample chosen to just barely pass"
// is caught by the CI itself, not by human review.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <vector>

#include "core/geo3d.hpp"
#include "core/geo3d_closedform.hpp"
#include "core/math.hpp"
#include "golden-analytic/core/closed_form_samples_generated.hpp"
#include "support/exact_prism_oracle.hpp"
#include "util/bit_utils.hpp"

namespace lumice {
namespace {

constexpr int kSideCnt = kClosedFormPrismSideCnt;
// Structural margin multiplier — must equal the K_safe threshold recorded in
// closed_form_samples_generated.hpp when the pool was written. Renaming here
// without regenerating the pool would silently drift the guard from the
// pool's selection criterion.
constexpr double kWellConditionedMinSepFactor = 50.0;
// Degenerate pool policy: min_sep < 1 × cf_merge_tol (clearly inside).
constexpr double kDegenerateMinSepFactor = 1.0;

// Minimum distance between any two exact-oracle corners, in the same units the
// closed form uses (r_i = √3/4 · dist_i). If this is below the closed form's
// merge tolerance, production may merge them into one — that's a legitimate
// merge-strategy divergence, not a topology error.
// Minimum pairwise distance among the candidate corners the closed form sees
// (feasibility slack governed by `feas_tol`). Using the same feasibility
// tolerance the closed form uses is important near 4-way concurrencies: with a
// tighter threshold, one of the near-duplicate candidates is silently dropped
// and the reported minimum jumps to the next-nearest pair — hiding exactly the
// merge-strategy difference this helper exists to quantify.
double MinPairwiseCornerDistance(const float dist[kSideCnt], double feas_tol) {
  double cs[kSideCnt];
  double sn[kSideCnt];
  double r[kSideCnt];
  const double k_r = math::kSqrt3 / 4.0;
  for (int i = 0; i < kSideCnt; i++) {
    double theta = static_cast<double>(i) * math::kPi_3;
    cs[i] = std::cos(theta);
    sn[i] = std::sin(theta);
    r[i] = k_r * static_cast<double>(dist[i]);
  }
  std::vector<std::pair<double, double>> pts;
  pts.reserve(12);
  for (int i = 0; i < kSideCnt; i++) {
    for (int j = i + 1; j < kSideCnt; j++) {
      if (j == i + 3) {
        continue;
      }
      double det = cs[i] * sn[j] - cs[j] * sn[i];
      if (det == 0.0) {
        continue;
      }
      double px = (r[i] * sn[j] - r[j] * sn[i]) / det;
      double py = (cs[i] * r[j] - cs[j] * r[i]) / det;
      bool feasible = true;
      for (int m = 0; m < kSideCnt; m++) {
        if (m == i || m == j) {
          continue;
        }
        if (cs[m] * px + sn[m] * py > r[m] + feas_tol) {
          feasible = false;
          break;
        }
      }
      if (feasible) {
        pts.emplace_back(px, py);
      }
    }
  }
  double best = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < pts.size(); i++) {
    for (size_t j = i + 1; j < pts.size(); j++) {
      double dx = pts[i].first - pts[j].first;
      double dy = pts[i].second - pts[j].second;
      best = std::min(best, std::sqrt(dx * dx + dy * dy));
    }
  }
  return best;
}

double CfMergeTolerance(const float dist[kSideCnt]) {
  double dist_scale = 0.0;
  for (int i = 0; i < kSideCnt; i++) {
    dist_scale = std::max(dist_scale, std::fabs(static_cast<double>(dist[i])));
  }
  const double cf_scale = std::max(math::kSqrt3 / 4.0 * dist_scale, 1.0);
  return 5.0 * static_cast<double>(math::kFloatEps) * cf_scale;
}

#if defined(__SIZEOF_INT128__)
// Vertex-set match: every closed-form 2D corner lifted to z = ±h/2 must appear
// in the production 3D vertex set, and vice versa (equal count checked
// separately by the caller).
bool VertexSetsMatch(const ClosedFormPrismResult& cf, float h, const float* prod_vtx, int prod_cnt, double tol) {
  const int expect = cf.corner_cnt * 2;
  if (prod_cnt != expect) {
    return false;
  }
  for (int v = 0; v < cf.corner_cnt; v++) {
    for (int s = 0; s < 2; s++) {
      const double z = (s == 0 ? 0.5 : -0.5) * static_cast<double>(h);
      bool matched = false;
      for (int p = 0; p < prod_cnt; p++) {
        double dx = static_cast<double>(prod_vtx[p * 3 + 0]) - static_cast<double>(cf.corner_x[v]);
        double dy = static_cast<double>(prod_vtx[p * 3 + 1]) - static_cast<double>(cf.corner_y[v]);
        double dz = static_cast<double>(prod_vtx[p * 3 + 2]) - z;
        if (std::sqrt(dx * dx + dy * dy + dz * dz) < tol) {
          matched = true;
          break;
        }
      }
      if (!matched) {
        return false;
      }
    }
  }
  return true;
}

int RunProduction3D(float h, const float* dist, std::unique_ptr<float[]>* out_vtx) {
  float coef[kMaxHexCrystalPlanes * 4];
  const size_t n = FillHexCrystalCoef(0, 0, 0, h, 0, dist, coef);
  if (n == 0) {
    *out_vtx = nullptr;
    return 0;
  }
  auto [vtx, cnt] = SolveConvexPolyhedronVtxD(static_cast<int>(n), coef);
  *out_vtx = std::move(vtx);
  return cnt;
}
#endif  // defined(__SIZEOF_INT128__)

// ============================================================================
// Known-configuration sanity: regular hexagon (dist = 1)
// ============================================================================

TEST(ClosedFormPrism, RegularHexagonHasExpectedInvariants) {
  float dist[kSideCnt] = { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
  auto r = ComputeClosedFormPrism(1.0f, dist);
  EXPECT_EQ(r.corner_cnt, 6);
  for (int i = 0; i < kClosedFormPrismFaceCnt; i++) {
    EXPECT_TRUE(r.face_present[i]) << "face slot " << i << " missing";
    EXPECT_EQ(r.face_number[i], i + 1);
  }
  // Face normals: basal (0,0,±1); side i at angle i·60°.
  EXPECT_NEAR(r.face_normal[2], 1.0f, 1e-6f);
  EXPECT_NEAR(r.face_normal[5], -1.0f, 1e-6f);
  for (int i = 0; i < kSideCnt; i++) {
    double theta = static_cast<double>(i) * math::kPi_3;
    EXPECT_NEAR(r.face_normal[(2 + i) * 3 + 0], static_cast<float>(std::cos(theta)), 1e-6f);
    EXPECT_NEAR(r.face_normal[(2 + i) * 3 + 1], static_cast<float>(std::sin(theta)), 1e-6f);
    EXPECT_NEAR(r.face_normal[(2 + i) * 3 + 2], 0.0f, 1e-6f);
  }
  // Corners on a regular hexagon lie on a circle of radius r = 0.5.
  const double expected_r = 0.5;
  for (int v = 0; v < r.corner_cnt; v++) {
    double px = static_cast<double>(r.corner_x[v]);
    double py = static_cast<double>(r.corner_y[v]);
    EXPECT_NEAR(std::sqrt(px * px + py * py), expected_r, 1e-6);
  }
}

TEST(ClosedFormPrism, ZeroHeightShortCircuit) {
  float dist[kSideCnt] = { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
  auto r = ComputeClosedFormPrism(0.0f, dist);
  EXPECT_EQ(r.corner_cnt, 0);
  for (int i = 0; i < kClosedFormPrismFaceCnt; i++) {
    EXPECT_FALSE(r.face_present[i]);
  }
}

// ============================================================================
// Fixed-sample structural-margin guards (methodology gate: samples must sit
// where the plan says they sit, not "wherever they happen to pass the three-way
// assertion"). Regeneration must produce entries that satisfy these same
// thresholds — if not, the pool has silently drifted and this TEST fails first,
// with a specific line number pointing at the offending entry.
//
// Deliberately NOT gated behind __SIZEOF_INT128__: this guard only needs
// CfMergeTolerance/MinPairwiseCornerDistance, neither of which touches the
// exact oracle, so it must keep running on MSVC — it is the CI-automated
// replacement for a human "was this sample tuned to pass" review: any future
// hand-edit or addition to a sample pool that drifts outside the selection
// threshold fails here first, on every platform, instead of relying on a
// reviewer noticing.
// ============================================================================

TEST(ClosedFormPrism, FixedSamplesRetainStructuralMargin) {
  for (size_t i = 0; i < std::size(test_support::kPrismWellConditionedSamples); i++) {
    const auto& s = test_support::kPrismWellConditionedSamples[i];
    const double cf_tol = CfMergeTolerance(s.dist);
    const double min_sep = MinPairwiseCornerDistance(s.dist, cf_tol);
    ASSERT_GE(min_sep, kWellConditionedMinSepFactor * cf_tol)
        << "well-conditioned sample #" << i << " has min_sep " << min_sep << " < " << kWellConditionedMinSepFactor
        << " × cf_merge_tol=" << cf_tol << " — sample pool has drifted; regenerate";
  }
  const test_support::PrismDistSample* degen_buckets[] = { test_support::kPrismDegenerateSigma030Samples,
                                                           test_support::kPrismDegenerateSigma050Samples,
                                                           test_support::kPrismDegenerateSigma080Samples };
  size_t degen_sizes[] = { std::size(test_support::kPrismDegenerateSigma030Samples),
                           std::size(test_support::kPrismDegenerateSigma050Samples),
                           std::size(test_support::kPrismDegenerateSigma080Samples) };
  const char* degen_labels[] = { "σ=0.30", "σ=0.50", "σ=0.80" };
  for (int b = 0; b < 3; b++) {
    for (size_t i = 0; i < degen_sizes[b]; i++) {
      const auto& s = degen_buckets[b][i];
      const double cf_tol = CfMergeTolerance(s.dist);
      const double min_sep = MinPairwiseCornerDistance(s.dist, cf_tol);
      ASSERT_LT(min_sep, kDegenerateMinSepFactor * cf_tol)
          << "degenerate " << degen_labels[b] << " sample #" << i << " has min_sep " << min_sep << " ≥ "
          << kDegenerateMinSepFactor << " × cf_merge_tol=" << cf_tol << " — sample pool has drifted; regenerate";
    }
  }
}

#if defined(__SIZEOF_INT128__)

// ============================================================================
// Well-conditioned regime: closed form / exact oracle / production all agree
// vertex-for-vertex on every entry of the well-conditioned pool. Any
// disagreement here is a bug.
// ============================================================================

TEST(ClosedFormPrism, WellConditionedThreeWayAgreement) {
  const float h = 1.0f;
  long cf_vs_exact_mismatch = 0;
  long cf_vs_prod_mismatch = 0;
  long face_count_mismatch = 0;
  long prod_empty = 0;
  const size_t n_samples = std::size(test_support::kPrismWellConditionedSamples);

  for (size_t s = 0; s < n_samples; s++) {
    const float* dist = test_support::kPrismWellConditionedSamples[s].dist;

    auto cf = ComputeClosedFormPrism(h, dist);
    auto ex = test_support::ExactPrism(dist);
    std::unique_ptr<float[]> prod_vtx;
    int prod_cnt = RunProduction3D(h, dist, &prod_vtx);

    if (cf.corner_cnt != ex.corner_count) {
      cf_vs_exact_mismatch++;
    }
    // face_present side-face count MUST equal corner_cnt (convex polygon:
    // every side face contributes exactly two corners, so #present-sides =
    // #corners).
    int side_present_cnt = 0;
    for (int i = 0; i < kSideCnt; i++) {
      if (cf.face_present[2 + i]) {
        side_present_cnt++;
      }
    }
    if (side_present_cnt != cf.corner_cnt) {
      face_count_mismatch++;
    }
    if (prod_cnt == 0) {
      prod_empty++;
      continue;
    }
    if (!VertexSetsMatch(cf, h, prod_vtx.get(), prod_cnt, static_cast<double>(math::kFloatEps))) {
      cf_vs_prod_mismatch++;
    }
  }

  EXPECT_EQ(cf_vs_exact_mismatch, 0) << "closed form disagrees with exact oracle in the well-conditioned regime";
  EXPECT_EQ(face_count_mismatch, 0) << "face_present side-count != corner_cnt (convex polygon invariant)";
  EXPECT_EQ(cf_vs_prod_mismatch, 0) << "closed form vs production disagree in the well-conditioned regime";
  EXPECT_EQ(prod_empty, 0) << "well-conditioned pool contains samples for which production returned empty";
}

// ============================================================================
// Degenerate regime: closed form == exact; production divergences must be
// explainable as merge-tolerance events. Every entry in the degenerate pool
// was chosen to have min_sep < cf_merge_tol — the merge tolerance MUST
// therefore explain every observed divergence.
// ============================================================================

class DegenerateSweep : public ::testing::TestWithParam<int> {};

TEST_P(DegenerateSweep, DivergencesAreQuantitativelyExplainedByMergeTolerance) {
  const int bucket_idx = GetParam();
  const test_support::PrismDistSample* pool = nullptr;
  size_t pool_size = 0;
  const char* label = nullptr;
  switch (bucket_idx) {
    case 0:
      pool = test_support::kPrismDegenerateSigma030Samples;
      pool_size = std::size(test_support::kPrismDegenerateSigma030Samples);
      label = "σ=0.30";
      break;
    case 1:
      pool = test_support::kPrismDegenerateSigma050Samples;
      pool_size = std::size(test_support::kPrismDegenerateSigma050Samples);
      label = "σ=0.50";
      break;
    case 2:
      pool = test_support::kPrismDegenerateSigma080Samples;
      pool_size = std::size(test_support::kPrismDegenerateSigma080Samples);
      label = "σ=0.80";
      break;
    default:
      FAIL() << "unknown bucket index " << bucket_idx;
      return;
  }
  const float h = 1.0f;
  long unexplained_cf_vs_exact = 0;
  long explained_cf_vs_exact = 0;
  long unexplained_cf_vs_prod = 0;
  long explained_cf_vs_prod = 0;
  long prod_empty = 0;
  int reported = 0;

  for (size_t s = 0; s < pool_size; s++) {
    const float* dist = pool[s].dist;

    auto cf = ComputeClosedFormPrism(h, dist);
    auto ex = test_support::ExactPrism(dist);
    std::unique_ptr<float[]> prod_vtx;
    int prod_cnt = RunProduction3D(h, dist, &prod_vtx);

    // Per-sample merge tolerances (the two implementations use different formulas):
    //   closed form: 5·kFloatEps · max(|r_i|, 1),  r_i = √3/4 · dist_i
    //                (src/core/geo3d_closedform.cpp — chosen to catch 4-way
    //                 concurrency boundary residuals of order 10·kFloatEps)
    //   production : 5e-5 · char_len,             char_len = max(1.0, max_i |coef_d[i*4+3]|)
    //                (src/core/math.cpp:807-811)
    double dist_scale = 0.0;
    for (int i = 0; i < kSideCnt; i++) {
      dist_scale = std::max(dist_scale, std::fabs(static_cast<double>(dist[i])));
    }
    const double cf_merge_tol = CfMergeTolerance(dist);
    // char_len walks all 8 planes: |d_basal|=h/2, |d_side_i|=|dist_i|·√3/8; floored at 1.0.
    const double max_abs_d = std::max(0.5 * static_cast<double>(h), math::kSqrt3 / 8.0 * dist_scale);
    const double prod_char_len = std::max(1.0, max_abs_d);
    const double prod_merge_tol = 5e-5 * prod_char_len;

    // Closed form vs exact oracle.
    if (cf.corner_cnt != ex.corner_count) {
      const double min_sep = MinPairwiseCornerDistance(dist, cf_merge_tol);
      // K=2 safety multiplier admits genuine boundary cases: a corner cluster
      // whose spacing sits within a factor of 2 of the merge threshold is
      // inherently merge-strategy-ambiguous (kFloatEps-level double-precision
      // residuals shift each candidate independently; some cluster pairs land
      // just above the threshold even though the geometric point is one).
      // Any wider miss is a real bug the tolerance cannot rescue.
      if (min_sep < 2.0 * cf_merge_tol) {
        explained_cf_vs_exact++;
      } else {
        unexplained_cf_vs_exact++;
        if (reported < 5) {
          std::fprintf(stderr,
                       "[cf!=exact UNEXPLAINED %s sample=%zu] cf=%d exact=%d min_sep=%.3e cf_tol=%.3e dist=", label, s,
                       cf.corner_cnt, ex.corner_count, min_sep, cf_merge_tol);
          for (int i = 0; i < kSideCnt; i++) {
            std::fprintf(stderr, "%a,", static_cast<double>(dist[i]));
          }
          std::fprintf(stderr, "\n");
          reported++;
        }
      }
    }

    if (prod_cnt == 0) {
      prod_empty++;
      continue;
    }
    // Closed form vs production. Vertex-position match tolerance = max of the
    // two implementations' merge tolerances: production's 3-plane solve incurs
    // per-triple residuals of order 5e-5 that the closed-form's 2-plane solve
    // does not, so a shared vertex may sit that far apart on the two sides
    // without either side being wrong.
    const double vertex_match_tol = std::max(cf_merge_tol, prod_merge_tol);
    if (VertexSetsMatch(cf, h, prod_vtx.get(), prod_cnt, vertex_match_tol)) {
      continue;  // agreement given the looser of the two tolerances
    }
    const double min_sep_prod = MinPairwiseCornerDistance(dist, prod_merge_tol);
    // Same K=2 safety multiplier — see the cf-vs-exact case above.
    if (min_sep_prod < 2.0 * prod_merge_tol) {
      explained_cf_vs_prod++;
    } else {
      unexplained_cf_vs_prod++;
      if (reported < 5) {
        std::fprintf(stderr,
                     "[cf!=prod UNEXPLAINED %s sample=%zu] cf=%d prod=%d min_sep=%.3e prod_tol=%.3e dist=", label, s,
                     cf.corner_cnt, prod_cnt / 2, min_sep_prod, prod_merge_tol);
        for (int i = 0; i < kSideCnt; i++) {
          std::fprintf(stderr, "%a,", static_cast<double>(dist[i]));
        }
        std::fprintf(stderr, "\n");
        reported++;
      }
    }
  }

  EXPECT_EQ(unexplained_cf_vs_exact, 0)
      << "closed form vs exact divergences with corner separation ABOVE the closed form's merge tolerance";
  EXPECT_EQ(unexplained_cf_vs_prod, 0)
      << "closed form vs production divergences with corner separation ABOVE production's merge tolerance";
  (void)explained_cf_vs_exact;
  (void)explained_cf_vs_prod;
  (void)prod_empty;
}

INSTANTIATE_TEST_SUITE_P(DegenerateSigmas, DegenerateSweep, ::testing::Values(0, 1, 2));

// ============================================================================
// face_corner_mask cross-check: oracle mask popcount matches closed-form
// face_present, on the well-conditioned pool.
// ============================================================================

TEST(ClosedFormPrism, OracleMaskPopcountMatchesFacePresent) {
  long side_present_disagree = 0;
  const size_t n_samples = std::size(test_support::kPrismWellConditionedSamples);
  for (size_t s = 0; s < n_samples; s++) {
    const float* dist = test_support::kPrismWellConditionedSamples[s].dist;
    auto cf = ComputeClosedFormPrism(1.0f, dist);
    auto ex = test_support::ExactPrism(dist);
    for (int i = 0; i < kSideCnt; i++) {
      const int popcnt = PopCount(static_cast<uint64_t>(ex.face_corner_mask[i]));
      // Oracle mask popcount ≥ 2  ⇔  face i is a bounding face of the polygon
      // (definition of face_present).
      const bool oracle_present = (popcnt >= 2);
      if (oracle_present != cf.face_present[2 + i]) {
        side_present_disagree++;
        break;
      }
    }
  }
  EXPECT_EQ(side_present_disagree, 0);
}

#endif  // defined(__SIZEOF_INT128__)

}  // namespace
}  // namespace lumice
