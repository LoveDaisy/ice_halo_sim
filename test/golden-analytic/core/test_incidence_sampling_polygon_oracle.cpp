// Golden-analytic harness for crystal entry-point incidence sampling (口径 B).
//
// This is the analytic ground-truth gate for the entry-point sampler
// `InitRay_p_fid`. The sampler picks a face area-weighted by projected area and
// then a point uniformly within the chosen face; that target is a closed form of
// the crystal geometry, so its correctness is judged against an analytic oracle
// (test/support/incidence_sampling_oracle.hpp) rather than a frozen reference
// sampler. The gate exists so a later refactor of the sampler (triangle → polygon
// granularity) can be accepted on distribution-invariance without a bit-exact
// fallback.
//
// Discipline this file enforces:
//   * OracleMathSelfProof — the analytic math is verified against hand-computed
//     values FIRST, independent of any sampling code (else "sampler self-proof"
//     would only be the oracle validating its own formulas).
//   * Ac1SelfProof / Ac2SelfProof — the CURRENT production sampler is verified
//     green across prism + pyramid + degenerate face-drop fixtures × several
//     incidence directions. This is the oracle-self-proof precondition: only a
//     sampler-vs-oracle agreement on the existing sampler lets a future change
//     distinguish "sampler correct / oracle blind" from "sampler wrong".
//   * Ac1RedStateCatchesBiasedWeight — an intentionally biased sampler (area
//     weight `|d·n|·A` instead of `max(-d·n·A, 0)`, i.e. dropping the front-face
//     sign) MUST be rejected, proving the comparator has teeth.
//
// The statistical thresholds (k-sigma widths, moment tolerance) are calibrated
// constants: see DISABLED_CalibrationScan below and the comments at each
// constant for the observed margins they were set from.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "core/crystal.hpp"
#include "core/geo3d.hpp"
#include "core/math.hpp"
#include "support/incidence_sampling_oracle.hpp"

namespace lumice {
namespace {

using test_support::Ac1Verdict;
using test_support::Ac2Verdict;
using test_support::EntrySamples;

// ---- Statistical parameters ---------------------------------------------------
// Sample count per (fixture, direction). Large enough that even faces receiving
// a few-percent share get thousands of hits (AC2 needs a stable per-face moment
// estimate). DISABLED_CalibrationScan over 12 seeds at this N observed
// AC1 |z|=4.45, AC2 centroid σ=4.44, AC2 moment σ=2.48; the thresholds below
// carry a safety margin over that envelope.
constexpr size_t kSampleN = 150000;

// AC1 binomial acceptance width. Calibration max |z| over all
// (seed × fixture × direction × face) was 4.45; 6.0 leaves headroom without
// letting the biased sampler (z in the hundreds) slip through.
constexpr double kAc1KSigma = 6.0;

// AC2 centroid per-axis SE combined z-score cap. Calibration max 4.44; 6.5 gives
// margin. In-face centroid must sit on the analytic (fan-mixture) centroid within
// sampling noise.
constexpr double kAc2CentroidKSigma = 6.5;

// AC2 covariance-trace deviation cap, as a count-scaled z-score (relative
// deviation / sqrt(2/count)). Calibration max was 2.48 across 12 seeds (uniform
// distributions have low kurtosis, so the estimator is tighter than the Gaussian
// sqrt(2/count) reference — z stays small); 6.0 gives ample margin while still
// catching a non-uniform in-face distribution (which shifts the trace by many
// sigma at these sample counts).
constexpr double kAc2MomentKSigma = 6.0;

// ---- Fixtures ----------------------------------------------------------------
struct CrystalFixture {
  const char* label;
  Crystal crystal;
};

std::vector<CrystalFixture> MakeFixtures() {
  std::vector<CrystalFixture> fx;
  // Prism: slender / regular / flat — spans the aspect-ratio range.
  fx.push_back({ "prism_h0.2", Crystal::CreatePrism(0.2f) });
  fx.push_back({ "prism_h1.2", Crystal::CreatePrism(1.2f) });
  fx.push_back({ "prism_h5.0", Crystal::CreatePrism(5.0f) });

  // Well-conditioned pyramids (from the closed-form pyramid test's regular pool).
  const float kUnitDist[6] = { 1, 1, 1, 1, 1, 1 };
  fx.push_back({ "pyr_shoulder_a", Crystal::CreatePyramid(28.0f, 28.0f, 0.6f, 1.0f, 0.6f, kUnitDist) });
  fx.push_back({ "pyr_shoulder_b", Crystal::CreatePyramid(60.0f, 30.0f, 1.2f, 0.4f, 0.7f, kUnitDist) });

  // Degenerate face-drop pyramids — reuse the fixed configurations that are
  // known to collapse at least one face (MakeFaceDropBatch in
  // test_closed_form_pyramid.cpp). Not re-invented degenerate numbers.
  // MakeFaceDropBatch has anonymous-namespace (internal) linkage in that
  // translation unit, so it cannot be called from here directly — the
  // dist[] literals below are a hand-copy and MUST be kept numerically in
  // sync with MakeFaceDropBatch's face-drop samples if those ever change.
  const float kDrop0[6] = { 0.3f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
  const float kDrop1[6] = { 1.0f, 0.4f, 1.0f, 0.5f, 1.0f, 1.0f };
  const float kDrop2[6] = { 1.0f, 1.0f, 0.35f, 1.0f, 1.0f, 0.9f };
  fx.push_back({ "pyr_drop_0", Crystal::CreatePyramid(28.0f, 28.0f, 1.0f, 1.0f, 1.0f, kDrop0) });
  fx.push_back({ "pyr_drop_1", Crystal::CreatePyramid(30.0f, 30.0f, 1.0f, 0.5f, 1.0f, kDrop1) });
  fx.push_back({ "pyr_drop_2", Crystal::CreatePyramid(45.0f, 60.0f, 0.8f, 0.6f, 0.8f, kDrop2) });
  return fx;
}

// A fixed, geometry-agnostic direction pool. Directions are filtered per fixture
// by the oracle to those that actually illuminate ≥2 faces, then a spread across
// the max-probability range is chosen (near-normal through near-grazing).
std::vector<std::array<float, 3>> CandidateDirections() {
  std::vector<std::array<float, 3>> dirs;
  const float axes[6][3] = { { 1, 0, 0 }, { -1, 0, 0 }, { 0, 1, 0 }, { 0, -1, 0 }, { 0, 0, 1 }, { 0, 0, -1 } };
  for (const auto& a : axes) {
    dirs.push_back({ a[0], a[1], a[2] });
  }
  const float s = 1.0f / std::sqrt(3.0f);
  for (int sx = -1; sx <= 1; sx += 2) {
    for (int sy = -1; sy <= 1; sy += 2) {
      for (int sz = -1; sz <= 1; sz += 2) {
        dirs.push_back({ sx * s, sy * s, sz * s });
      }
    }
  }
  // Horizontal directions at offset azimuths — near-grazing on the side faces.
  for (int k = 0; k < 6; k++) {
    const float az = (static_cast<float>(k) * 60.0f + 15.0f) * math::kDegreeToRad;
    dirs.push_back({ std::cos(az), std::sin(az), 0.0f });
  }
  return dirs;
}

// For one fixture, select up to `max_dirs` directions spread across the
// max-probability spectrum, each illuminating ≥2 present faces.
std::vector<std::array<float, 3>> SelectDirections(const Crystal& crystal, size_t max_dirs = 4) {
  const auto pool = CandidateDirections();
  struct Scored {
    std::array<float, 3> d;
    double max_p;
  };
  std::vector<Scored> valid;
  for (const auto& d : pool) {
    const auto probs = test_support::ComputeProjectedFaceAreaDistribution(crystal.CfGeom(), d.data());
    int positive = 0;
    double max_p = 0.0;
    for (double p : probs) {
      if (p > 1e-6) {
        positive++;
      }
      max_p = std::max(max_p, p);
    }
    if (positive >= 2) {
      valid.push_back({ d, max_p });
    }
  }
  std::sort(valid.begin(), valid.end(), [](const Scored& a, const Scored& b) { return a.max_p > b.max_p; });
  std::vector<std::array<float, 3>> out;
  if (valid.empty()) {
    return out;
  }
  const size_t n = valid.size();
  const size_t take = std::min(max_dirs, n);
  for (size_t i = 0; i < take; i++) {
    // Spread indices across [0, n-1] so we cover high-max_p (near-normal) to
    // low-max_p (near-grazing / multi-face).
    const size_t idx = (take == 1) ? 0 : (i * (n - 1)) / (take - 1);
    out.push_back(valid[idx].d);
  }
  return out;
}

// ---- Layer 1 self-proof: analytic math vs hand-computed values ----------------
TEST(IncidenceSamplingOracle, OracleMathSelfProof) {
  namespace ts = test_support;

  // Non-axis-aligned triangle, area = 3 (base 2 on x, height 3 on z).
  {
    const double tri[9] = { 0, 0, 0, 2, 0, 0, 0, 0, 3 };
    EXPECT_NEAR(ts::PolygonArea3D(tri, 3), 3.0, 1e-9);
  }

  // Unit square in the xy-plane: area 1, normal +z, centroid (0.5,0.5),
  // var = 1/12 per axis, zero covariance. Local basis: origin=v0, e1=+x, e2=+y.
  {
    const double sq[12] = { 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0 };
    EXPECT_NEAR(ts::PolygonArea3D(sq, 4), 1.0, 1e-12);
    double n[3];
    ts::PolygonNewellNormal(sq, 4, n);
    EXPECT_NEAR(n[0], 0.0, 1e-12);
    EXPECT_NEAR(n[1], 0.0, 1e-12);
    EXPECT_NEAR(std::abs(n[2]), 1.0, 1e-12);
    ts::FaceBasis2D basis = ts::BuildFaceLocalBasis(sq, 4, n);
    double uv[8];
    for (int k = 0; k < 4; k++) {
      ts::ProjectToLocal2D(sq + k * 3, basis, &uv[k * 2], &uv[k * 2 + 1]);
    }
    ts::Polygon2DMoments m = ts::ComputePolygon2DMoments(uv, 4);
    EXPECT_NEAR(m.area, 1.0, 1e-12);
    EXPECT_NEAR(m.cu, 0.5, 1e-12);
    EXPECT_NEAR(m.cv, 0.5, 1e-12);
    EXPECT_NEAR(m.var_u, 1.0 / 12.0, 1e-12);
    EXPECT_NEAR(m.var_v, 1.0 / 12.0, 1e-12);
    EXPECT_NEAR(m.cov_uv, 0.0, 1e-12);

    // Cross-check the fan path against the whole-polygon path: the unit square
    // fanned from corner 0 is triangles [v0,v1,v2] and [v0,v2,v3]; the
    // equal-area mixture of their per-triangle moments must reproduce the
    // polygon moments (this binds ComputeTriangle2DMoments + the mixture formula
    // to ComputePolygon2DMoments, so a bug in either path is caught).
    const double t0[6] = { uv[0], uv[1], uv[2], uv[3], uv[4], uv[5] };
    const double t1[6] = { uv[0], uv[1], uv[4], uv[5], uv[6], uv[7] };
    ts::Polygon2DMoments m0 = ts::ComputeTriangle2DMoments(t0);
    ts::Polygon2DMoments m1 = ts::ComputeTriangle2DMoments(t1);
    const double w0 = m0.area;
    const double w1 = m1.area;
    const double ws = w0 + w1;
    const double mix_cu = (w0 * m0.cu + w1 * m1.cu) / ws;
    const double mix_cv = (w0 * m0.cv + w1 * m1.cv) / ws;
    const double e_uu = (w0 * (m0.var_u + m0.cu * m0.cu) + w1 * (m1.var_u + m1.cu * m1.cu)) / ws;
    const double e_vv = (w0 * (m0.var_v + m0.cv * m0.cv) + w1 * (m1.var_v + m1.cv * m1.cv)) / ws;
    EXPECT_NEAR(mix_cu, m.cu, 1e-12);
    EXPECT_NEAR(mix_cv, m.cv, 1e-12);
    EXPECT_NEAR(e_uu - mix_cu * mix_cu, m.var_u, 1e-12);
    EXPECT_NEAR(e_vv - mix_cv * mix_cv, m.var_v, 1e-12);
  }

  // Regular unit-circumradius hexagon in the xy-plane. Independent, memorization-
  // free invariants: area = 3√3/2·R², 3D centroid reconstructs to origin, and the
  // in-plane distribution is isotropic (var_u ≈ var_v, cov ≈ 0).
  {
    double hex[18];
    for (int k = 0; k < 6; k++) {
      const double a = static_cast<double>(k) * (M_PI / 3.0);
      hex[k * 3 + 0] = std::cos(a);
      hex[k * 3 + 1] = std::sin(a);
      hex[k * 3 + 2] = 0.0;
    }
    EXPECT_NEAR(ts::PolygonArea3D(hex, 6), 3.0 * std::sqrt(3.0) / 2.0, 1e-9);
    double n[3];
    ts::PolygonNewellNormal(hex, 6, n);
    EXPECT_NEAR(std::abs(n[2]), 1.0, 1e-12);
    ts::FaceBasis2D basis = ts::BuildFaceLocalBasis(hex, 6, n);
    double uv[12];
    for (int k = 0; k < 6; k++) {
      ts::ProjectToLocal2D(hex + k * 3, basis, &uv[k * 2], &uv[k * 2 + 1]);
    }
    ts::Polygon2DMoments m = ts::ComputePolygon2DMoments(uv, 6);
    // Reconstruct the 3D centroid from local coords → must be the origin.
    double c3d[3];
    for (int i = 0; i < 3; i++) {
      c3d[i] = basis.origin[i] + m.cu * basis.e1[i] + m.cv * basis.e2[i];
    }
    EXPECT_NEAR(c3d[0], 0.0, 1e-9);
    EXPECT_NEAR(c3d[1], 0.0, 1e-9);
    EXPECT_NEAR(c3d[2], 0.0, 1e-9);
    // Isotropy: equal variances, negligible covariance.
    EXPECT_NEAR(m.var_u, m.var_v, 1e-9);
    EXPECT_NEAR(m.cov_uv, 0.0, 1e-9);
    EXPECT_GT(m.var_u, 0.0);
  }
}

// ---- Fixture sanity: every fixture builds a non-degenerate polyhedron ---------
TEST(IncidenceSamplingOracle, FixturesAreValid) {
  for (auto& f : MakeFixtures()) {
    EXPECT_GE(f.crystal.PolygonFaceCount(), 4u) << f.label << " produced too few faces";
    EXPECT_GE(SelectDirections(f.crystal).size(), 1u) << f.label << " yielded no usable direction";
  }
}

// ---- AC1 self-proof: production sampler matches the projected-area target ------
TEST(IncidenceSamplingOracle, Ac1SelfProof) {
  auto fixtures = MakeFixtures();
  uint32_t seed = 12345;
  for (auto& f : fixtures) {
    const auto dirs = SelectDirections(f.crystal);
    ASSERT_FALSE(dirs.empty()) << f.label;
    for (const auto& d : dirs) {
      EntrySamples samples = test_support::DriveEntrySampling(f.crystal, d.data(), kSampleN, seed++);
      Ac1Verdict v = test_support::CheckProjectedAreaDistribution(f.crystal, d.data(), samples.face, kAc1KSigma);
      EXPECT_TRUE(v.pass) << f.label << " dir=(" << d[0] << "," << d[1] << "," << d[2] << ")"
                          << " max|z|=" << v.max_abs_z << " worst_face=" << v.worst_face
                          << " max_rel_dev=" << v.max_relative_dev << " zero_leak=" << v.zero_weight_leak;
    }
  }
}

// ---- AC2 self-proof: production sampler is uniform within each face ------------
TEST(IncidenceSamplingOracle, Ac2SelfProof) {
  auto fixtures = MakeFixtures();
  uint32_t seed = 777;
  for (auto& f : fixtures) {
    const auto dirs = SelectDirections(f.crystal);
    ASSERT_FALSE(dirs.empty()) << f.label;
    for (const auto& d : dirs) {
      EntrySamples samples = test_support::DriveEntrySampling(f.crystal, d.data(), kSampleN, seed++);
      Ac2Verdict v =
          test_support::CheckInFaceUniformity(f.crystal, d.data(), samples, kAc2CentroidKSigma, kAc2MomentKSigma);
      EXPECT_TRUE(v.pass) << f.label << " dir=(" << d[0] << "," << d[1] << "," << d[2] << ")"
                          << " max_centroid_sigma=" << v.max_centroid_dev_sigma
                          << " max_moment_sigma=" << v.max_moment_dev_sigma
                          << " max_moment_rel=" << v.max_moment_relative_dev << " worst_face=" << v.worst_face;
    }
  }
}

// ---- Red state: a biased sampler must be caught by the AC1 comparator ----------
//
// The biased sampler exists ONLY here (production `InitRay_p_fid` is untouched).
// It replicates the production loop body but injects the exact bias named in the
// issue: the area weight drops the front-face sign, using |d·n|·A instead of
// max(-d·n·A, 0). This lights back faces that the correct target gives zero
// weight, so the AC1 comparator (built on the true, unmodified oracle) must
// reject it. Every non-weight primitive (RandomSample, SampleTrianglePoint,
// PolygonFaceOfTri, the triangle getters) is the production implementation.
EntrySamples SampleEntryPointBiased(const Crystal& crystal, const float d[3], size_t n, uint32_t seed) {
  RandomNumberGenerator::GetInstance().SetSeed(seed);
  const size_t total_faces = crystal.TotalTriangles();
  const float* face_area = crystal.GetTirangleArea();
  const float* face_norm = crystal.GetTriangleNormal();
  const float* face_vtx = crystal.GetTriangleVtx();

  std::vector<float> proj_prob(total_faces);
  EntrySamples out;
  out.face.resize(n);
  out.point.resize(n);
  for (size_t i = 0; i < n; i++) {
    for (size_t j = 0; j < total_faces; j++) {
      // BIAS: absolute value instead of max(-dot, 0) — lights back faces too.
      proj_prob[j] = std::abs(Dot3(d, face_norm + j * 3)) * face_area[j];
    }
    int tri_id = 0;
    RandomSample(static_cast<int>(total_faces), proj_prob.data(), &tri_id);
    float p[3];
    SampleTrianglePoint(face_vtx + tri_id * 9, p);
    out.point[i] = { p[0], p[1], p[2] };
    out.face[i] = crystal.PolygonFaceOfTri(tri_id);
  }
  return out;
}

TEST(IncidenceSamplingOracle, Ac1RedStateCatchesBiasedWeight) {
  // Pick a fixture + direction where flipping the sign clearly re-weights face
  // selection: a strongly directional incidence (one face near-normal, its
  // opposite face fully back-lit). A prism with a near-normal side incidence is
  // the cleanest such case — the back side face has target weight 0 but the
  // biased |dot| version gives it the SAME weight as the front face.
  Crystal prism = Crystal::CreatePrism(1.2f);
  const auto dirs = SelectDirections(prism);
  ASSERT_FALSE(dirs.empty());

  // Use the most directional (highest max_p) direction — index 0 after the
  // spread selection is the near-normal one.
  const std::array<float, 3> d = dirs.front();

  // Sanity: the unbiased sampler passes on this same direction (green baseline).
  {
    EntrySamples good = test_support::DriveEntrySampling(prism, d.data(), kSampleN, 2024);
    Ac1Verdict vg = test_support::CheckProjectedAreaDistribution(prism, d.data(), good.face, kAc1KSigma);
    ASSERT_TRUE(vg.pass) << "baseline (unbiased) sampler unexpectedly failed; red-state test is not isolating "
                            "the injected bias. max|z|="
                         << vg.max_abs_z;
  }

  // The biased sampler must be rejected — fed to the SAME comparator + oracle.
  EntrySamples bad = SampleEntryPointBiased(prism, d.data(), kSampleN, 2024);
  Ac1Verdict vb = test_support::CheckProjectedAreaDistribution(prism, d.data(), bad.face, kAc1KSigma);
  EXPECT_FALSE(vb.pass) << "biased sampler slipped past AC1 — comparator has no teeth. max|z|=" << vb.max_abs_z
                        << " zero_leak=" << vb.zero_weight_leak;
  // The bias should be gross, not marginal (far beyond the k-sigma band).
  EXPECT_GT(vb.max_abs_z, 10.0 * kAc1KSigma)
      << "injected bias barely exceeded threshold — pick a more directional case to avoid flakiness";
}

// ---- Calibration scan (disabled; run manually to (re)derive thresholds) -------
// Run with: golden_analytic_test --gtest_also_run_disabled_tests \
//   --gtest_filter='*DISABLED_CalibrationScan*'
// Prints the max AC1 |z|, AC2 centroid sigma, and AC2 moment relative deviation
// across several seeds × fixtures × directions, so the constants above can be set
// with margin over the observed sampling-noise envelope.
TEST(IncidenceSamplingOracle, DISABLED_CalibrationScan) {
  auto fixtures = MakeFixtures();
  double max_ac1_z = 0.0;
  double max_ac2_centroid = 0.0;
  double max_ac2_moment_sigma = 0.0;
  double max_ac2_moment_rel = 0.0;
  const std::vector<uint32_t> seeds = { 1, 2, 3, 5, 8, 13, 21, 34, 55, 89, 144, 233 };
  for (uint32_t seed : seeds) {
    for (auto& f : fixtures) {
      const auto dirs = SelectDirections(f.crystal);
      for (const auto& d : dirs) {
        EntrySamples s = test_support::DriveEntrySampling(f.crystal, d.data(), kSampleN, seed);
        // Use a very loose threshold so pass/fail never short-circuits the scan.
        Ac1Verdict v1 = test_support::CheckProjectedAreaDistribution(f.crystal, d.data(), s.face, 1e9);
        Ac2Verdict v2 = test_support::CheckInFaceUniformity(f.crystal, d.data(), s, 1e9, 1e9);
        max_ac1_z = std::max(max_ac1_z, v1.max_abs_z);
        max_ac2_centroid = std::max(max_ac2_centroid, v2.max_centroid_dev_sigma);
        max_ac2_moment_sigma = std::max(max_ac2_moment_sigma, v2.max_moment_dev_sigma);
        max_ac2_moment_rel = std::max(max_ac2_moment_rel, v2.max_moment_relative_dev);
      }
    }
  }
  std::printf(
      "[calibration] max AC1 |z|=%.3f  max AC2 centroid sigma=%.3f  max AC2 moment sigma=%.3f  max AC2 moment "
      "rel=%.5f\n",
      max_ac1_z, max_ac2_centroid, max_ac2_moment_sigma, max_ac2_moment_rel);
}

}  // namespace
}  // namespace lumice
