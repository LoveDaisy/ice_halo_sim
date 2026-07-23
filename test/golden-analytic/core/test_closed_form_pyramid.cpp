// Contract tests for the closed-form pyramid evaluator (ComputeClosedFormPyramid).
//
// These tests assert the contracts the production closed form actually holds,
// each consuming ONLY ComputeClosedFormPyramid's own output — no second
// "exact" adjudicator. A prior revision drove a symbolic-α exact oracle whose
// feasibility / incidence / degenerate decisions were resolved through a
// double-precision refuse-on-ambiguity filter; those decisions were
// platform-dependent (FMA contraction flips borderline signs), which is a
// category error rather than an implementation bug: free-symbol algebra is the
// algebra of the generic point and is structurally blind to degeneracy (the
// regular pyramid a1==a2, three-face concurrency, an edge vanishing first).
// See doc/numerical-robustness.md §8 for the full rationale.
//
// The three contracts:
//   1. Topology  — TopologyMatchesGoldenConstants: after the closed-form
//      rewrite the topology (vertex count / faces present / solver branches
//      walked) is a CONSTANT of the parametrization. Assert cf's topology
//      against a stored snapshot of cf's own output — pure integer/bitmask
//      equality, zero platform surface.
//   2. Vertex positions — VertexPlaneSelfConsistency: cf's stated vertices must
//      satisfy cf's own stated plane equations (self-consistency, not
//      comparison to an external truth), within a LOOSE physical tolerance
//      (relative to the sample's characteristic length, never an absolute
//      epsilon on a scale-varying quantity — cf. numerical-robustness.md §2).
//   3. Boundary behavior — DegenerateContractSafe: degenerate inputs must obey
//      the graceful-degradation contract production already trusts — bounded,
//      finite, no overflow, collapsed (near-zero-area) faces rather than a
//      crash. Weak assertions: no specific topology is demanded.
//
// Sample pools are pre-generated (closed_form_samples_generated.hpp) so the
// assertions are portable across libc++/libstdc++/MSVC. Every well-conditioned
// entry has min_sep ≥ 50 × prod_merge_tol; every degenerate entry has
// min_sep < 1 × prod_merge_tol. FixedSamplesRetainStructuralMargin re-verifies
// these invariants at CI time, so pool drift is caught by CI, not by review.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

#include "core/geo3d.hpp"
#include "core/geo3d_closedform.hpp"
#include "core/math.hpp"
#include "golden-analytic/core/closed_form_samples_generated.hpp"
#include "golden-analytic/core/pyramid_topology_golden_generated.hpp"

namespace lumice {
namespace {

constexpr int kSideCnt = kClosedFormPyramidSideCnt;
// Structural margin multipliers — must equal the K_safe threshold recorded in
// closed_form_samples_generated.hpp when the pool was written.
constexpr double kWellConditionedMinSepFactor = 50.0;
constexpr double kDegenerateMinSepFactor = 1.0;

// ---- Sample record used only within this TU (for test-time construction). --
struct PyramidSample {
  float upper_alpha;
  float lower_alpha;
  float h1;
  float h2;
  float h3;
  float dist[kSideCnt];
};

PyramidSample MakeSample(const test_support::PyramidDirectSample& s) {
  PyramidSample out;
  out.upper_alpha = s.upper_alpha;
  out.lower_alpha = s.lower_alpha;
  out.h1 = s.h1;
  out.h2 = s.h2;
  out.h3 = s.h3;
  for (int i = 0; i < kSideCnt; i++) {
    out.dist[i] = s.dist[i];
  }
  return out;
}

// ---- Cone slope from a wedge angle — mirrors ComputeClosedFormPyramid; used
//      only by ProductionCharLen's merge-tolerance scale estimate.
double A1FromAlpha(float alpha, float h_side) {
  constexpr float kMinAlpha = 0.1f;
  constexpr float kMaxAlpha = 89.9f;
  if (!(h_side > math::kFloatEps && alpha >= kMinAlpha && alpha <= kMaxAlpha)) {
    return -1.0;
  }
  return static_cast<double>(math::kSqrt3_4) /
         std::tan(static_cast<double>(alpha) * static_cast<double>(math::kDegreeToRad));
}

// ---- Production 3D solve wrapper --------------------------------------------
// Wraps FillHexCrystalCoef + SolveConvexPolyhedronVtxD; used by
// FixedSamplesRetainStructuralMargin only (the structural-margin guard measures
// the production dedup boundary, so it must run the production solve).
int RunProduction3D(float upper_alpha, float lower_alpha, float h1, float h2, float h3, const float dist[kSideCnt],
                    std::unique_ptr<float[]>* out_vtx) {
  float coef[kMaxHexCrystalPlanes * 4];
  const size_t n = FillHexCrystalCoef(upper_alpha, lower_alpha, h1, h2, h3, dist, coef);
  if (n == 0) {
    *out_vtx = nullptr;
    return 0;
  }
  auto [vtx, cnt] = SolveConvexPolyhedronVtxD(static_cast<int>(n), coef);
  *out_vtx = std::move(vtx);
  return cnt;
}

// ---- Minimum pairwise vertex distance (used by the structural-margin guard).
double MinPairwiseVertexDistance(const float* vtx, int cnt) {
  double best = std::numeric_limits<double>::infinity();
  for (int i = 0; i < cnt; i++) {
    for (int j = i + 1; j < cnt; j++) {
      double dx = static_cast<double>(vtx[i * 3 + 0]) - static_cast<double>(vtx[j * 3 + 0]);
      double dy = static_cast<double>(vtx[i * 3 + 1]) - static_cast<double>(vtx[j * 3 + 1]);
      double dz = static_cast<double>(vtx[i * 3 + 2]) - static_cast<double>(vtx[j * 3 + 2]);
      best = std::min(best, std::sqrt(dx * dx + dy * dy + dz * dz));
    }
  }
  return best;
}

// Character length for production's merge tolerance formula (mirrors the
// char_len computation inlined inside SolveConvexPolyhedronVtxD in
// src/core/math.cpp — not a standalone function, so there is no call site to
// reuse directly; kept in sync by hand), floored at 1.0. It also serves as the
// characteristic length for the loose physical tolerances in
// VertexPlaneSelfConsistency / DegenerateContractSafe. Walks all populated
// plane d's FillHexCrystalCoef produced; here we use the max-abs input
// parameter as a safe proxy (basal d = h/2, side d = √3/8 · dist_i,
// cone d = (h/2 + a·dist)·√3/8).
double ProductionCharLen(const PyramidSample& s) {
  double max_abs_d = 0.5 * static_cast<double>(s.h1 + s.h2 + s.h3);
  double max_dist = 0.0;
  for (float d : s.dist) {
    max_dist = std::max(max_dist, std::fabs(static_cast<double>(d)));
  }
  double au = std::max(0.0, A1FromAlpha(s.upper_alpha, s.h1));
  double al = std::max(0.0, A1FromAlpha(s.lower_alpha, s.h3));
  max_abs_d = std::max(max_abs_d, math::kSqrt3 / 8.0 * max_dist);
  max_abs_d = std::max(max_abs_d, math::kSqrt3 / 8.0 * (0.5 * static_cast<double>(s.h2) + std::max(au, al) * max_dist));
  return std::max(1.0, max_abs_d);
}

double ProductionMergeTolerance(const PyramidSample& s) {
  return 5e-5 * ProductionCharLen(s);
}

// Alpha derivation from a Miller (i1, i4) pair — mirrors production Miller
// entry's alpha computation in geo3d.cpp:556/558.
float AlphaFromMiller(int i1, int i4) {
  if (i1 == 0) {
    return 0.0f;
  }
  return std::atan(math::kSqrt3_2 * static_cast<float>(i4) / static_cast<float>(i1) / kIceCrystalC) *
         math::kRadToDegree;
}

// ---- Topology fingerprint helpers -------------------------------------------

// Pack ClosedFormPyramidResult::face_present[20] into a bitmask (bit i = slot i).
uint32_t PackFacePresent(const ClosedFormPyramidResult& cf) {
  uint32_t mask = 0;
  for (int slot = 0; slot < kClosedFormPyramidFaceCnt; slot++) {
    if (cf.face_present[slot]) {
      mask |= (1u << slot);
    }
  }
  return mask;
}

// Area of a planar polygon in 3D via the Newell / vector-area formula — robust
// to the polygon's orientation and independent of any face-normal sign
// convention. Test-only; production has no standalone polygon-area function.
double FacePolygonArea(const ClosedFormPyramidResult& cf, int slot) {
  const int m = cf.face_vtx_cnt[slot];
  double nx = 0.0;
  double ny = 0.0;
  double nz = 0.0;
  for (int k = 0; k < m; k++) {
    const int a = cf.face_vtx[slot][k];
    const int b = cf.face_vtx[slot][(k + 1) % m];
    const double ax = cf.vtx[a * 3 + 0];
    const double ay = cf.vtx[a * 3 + 1];
    const double az = cf.vtx[a * 3 + 2];
    const double bx = cf.vtx[b * 3 + 0];
    const double by = cf.vtx[b * 3 + 1];
    const double bz = cf.vtx[b * 3 + 2];
    nx += ay * bz - az * by;
    ny += az * bx - ax * bz;
    nz += ax * by - ay * bx;
  }
  return 0.5 * std::sqrt(nx * nx + ny * ny + nz * nz);
}

// ============================================================================
// Fixed-sample structural-margin guards — the same threshold that gated
// selection now gates the pool at test time. Regeneration that produces a
// pool outside these thresholds fails here first, with a specific offending
// entry index.
//
// The guard uses only RunProduction3D/ProductionMergeTolerance/
// MinPairwiseVertexDistance and therefore runs on every platform — it is the
// CI-automated replacement for a human "was this sample tuned to pass"
// review: any future hand-edit or addition to a sample pool that drifts
// outside the selection threshold fails here first, instead of relying on a
// reviewer noticing. Covers all four sample-bucket families exercised
// elsewhere in this file: well-conditioned direct-wedge, Miller-index,
// flat-tail (α=85..89.5°), and degenerate.
// ============================================================================

TEST(ClosedFormPyramid, FixedSamplesRetainStructuralMargin) {
  // Well-conditioned direct-wedge pool.
  for (size_t i = 0; i < std::size(test_support::kPyramidWellConditionedSamples); i++) {
    PyramidSample s = MakeSample(test_support::kPyramidWellConditionedSamples[i]);
    std::unique_ptr<float[]> prod_vtx;
    int prod_cnt = RunProduction3D(s.upper_alpha, s.lower_alpha, s.h1, s.h2, s.h3, s.dist, &prod_vtx);
    ASSERT_GT(prod_cnt, 0) << "well-conditioned sample #" << i << " produces empty production result";
    const double prod_tol = ProductionMergeTolerance(s);
    const double min_sep = MinPairwiseVertexDistance(prod_vtx.get(), prod_cnt);
    ASSERT_GE(min_sep, kWellConditionedMinSepFactor * prod_tol)
        << "well-conditioned sample #" << i << " has min_sep " << min_sep << " < " << kWellConditionedMinSepFactor
        << " × prod_merge_tol=" << prod_tol << " — sample pool has drifted; regenerate";
  }
  // Miller-index pool — same well-conditioned threshold (see comment above).
  for (size_t i = 0; i < std::size(test_support::kPyramidMillerSamples); i++) {
    const auto& m = test_support::kPyramidMillerSamples[i];
    PyramidSample s;
    s.upper_alpha = AlphaFromMiller(m.upper_i1, m.upper_i4);
    s.lower_alpha = AlphaFromMiller(m.lower_i1, m.lower_i4);
    s.h1 = m.h1;
    s.h2 = m.h2;
    s.h3 = m.h3;
    for (int j = 0; j < kSideCnt; j++) {
      s.dist[j] = m.dist[j];
    }
    std::unique_ptr<float[]> prod_vtx;
    int prod_cnt = RunProduction3D(s.upper_alpha, s.lower_alpha, s.h1, s.h2, s.h3, s.dist, &prod_vtx);
    ASSERT_GT(prod_cnt, 0) << "Miller sample #" << i << " produces empty production result";
    const double prod_tol = ProductionMergeTolerance(s);
    const double min_sep = MinPairwiseVertexDistance(prod_vtx.get(), prod_cnt);
    ASSERT_GE(min_sep, kWellConditionedMinSepFactor * prod_tol)
        << "Miller sample #" << i << " has min_sep " << min_sep << " < " << kWellConditionedMinSepFactor
        << " × prod_merge_tol=" << prod_tol << " — sample pool has drifted; regenerate";
  }
  // Flat-tail pools (α=85..89.5°) — same well-conditioned threshold.
  const test_support::PyramidDirectSample* flat_pools[] = {
    test_support::kPyramidFlatTailAlpha85Samples,  test_support::kPyramidFlatTailAlpha87Samples,
    test_support::kPyramidFlatTailAlpha875Samples, test_support::kPyramidFlatTailAlpha88Samples,
    test_support::kPyramidFlatTailAlpha89Samples,  test_support::kPyramidFlatTailAlpha895Samples
  };
  size_t flat_sizes[] = {
    std::size(test_support::kPyramidFlatTailAlpha85Samples),  std::size(test_support::kPyramidFlatTailAlpha87Samples),
    std::size(test_support::kPyramidFlatTailAlpha875Samples), std::size(test_support::kPyramidFlatTailAlpha88Samples),
    std::size(test_support::kPyramidFlatTailAlpha89Samples),  std::size(test_support::kPyramidFlatTailAlpha895Samples)
  };
  const char* flat_labels[] = { "α=85", "α=87", "α=87.5", "α=88", "α=89", "α=89.5" };
  for (int b = 0; b < 6; b++) {
    for (size_t i = 0; i < flat_sizes[b]; i++) {
      PyramidSample s = MakeSample(flat_pools[b][i]);
      std::unique_ptr<float[]> prod_vtx;
      int prod_cnt = RunProduction3D(s.upper_alpha, s.lower_alpha, s.h1, s.h2, s.h3, s.dist, &prod_vtx);
      ASSERT_GT(prod_cnt, 0) << "flat-tail " << flat_labels[b] << " sample #" << i
                             << " produces empty production result";
      const double prod_tol = ProductionMergeTolerance(s);
      const double min_sep = MinPairwiseVertexDistance(prod_vtx.get(), prod_cnt);
      ASSERT_GE(min_sep, kWellConditionedMinSepFactor * prod_tol)
          << "flat-tail " << flat_labels[b] << " sample #" << i << " has min_sep " << min_sep << " < "
          << kWellConditionedMinSepFactor << " × prod_merge_tol=" << prod_tol
          << " — sample pool has drifted; regenerate";
    }
  }
  // Degenerate pools.
  const test_support::PyramidDirectSample* degen_buckets[] = { test_support::kPyramidDegenerateSigma030Samples,
                                                               test_support::kPyramidDegenerateSigma050Samples };
  size_t degen_sizes[] = { std::size(test_support::kPyramidDegenerateSigma030Samples),
                           std::size(test_support::kPyramidDegenerateSigma050Samples) };
  const char* degen_labels[] = { "σ=0.30", "σ=0.50" };
  for (int b = 0; b < 2; b++) {
    for (size_t i = 0; i < degen_sizes[b]; i++) {
      PyramidSample s = MakeSample(degen_buckets[b][i]);
      std::unique_ptr<float[]> prod_vtx;
      int prod_cnt = RunProduction3D(s.upper_alpha, s.lower_alpha, s.h1, s.h2, s.h3, s.dist, &prod_vtx);
      ASSERT_GT(prod_cnt, 0) << "degenerate " << degen_labels[b] << " sample #" << i << " produces empty production";
      const double prod_tol = ProductionMergeTolerance(s);
      const double min_sep = MinPairwiseVertexDistance(prod_vtx.get(), prod_cnt);
      ASSERT_LT(min_sep, kDegenerateMinSepFactor * prod_tol)
          << "degenerate " << degen_labels[b] << " sample #" << i << " has min_sep " << min_sep << " ≥ "
          << kDegenerateMinSepFactor << " × prod_merge_tol=" << prod_tol << " — sample pool has drifted; regenerate";
    }
  }
}

// ============================================================================
// Contract 1 — Topology golden constants.
//
// After the closed-form rewrite the pyramid topology is a constant of the
// parametrization. Assert cf's (vtx_cnt, face_present_mask, path_tag_union)
// against the stored snapshot (pyramid_topology_golden_generated.hpp), a pure
// integer/bitmask equality with zero platform surface. This subsumes the old
// RegularPyramidAllThreeAgree / WellConditioned{Direct,Miller} / flat-tail
// sweeps / OracleMatchesInt128Reference tests, whose only genuine content was
// "cf produces the expected topology" — now expressed without any second
// exact-arithmetic witness.
// ============================================================================

// Compile-time invariant: each golden array length must match its
// corresponding sample-pool array length; catches silent index-misalignment if
// either is regenerated without the other.
static_assert(std::size(test_support::kPyramidWellConditionedTopology) ==
              std::size(test_support::kPyramidWellConditionedSamples));
static_assert(std::size(test_support::kPyramidMillerTopology) == std::size(test_support::kPyramidMillerSamples));
static_assert(std::size(test_support::kPyramidFlatTailAlpha85Topology) ==
              std::size(test_support::kPyramidFlatTailAlpha85Samples));
static_assert(std::size(test_support::kPyramidFlatTailAlpha87Topology) ==
              std::size(test_support::kPyramidFlatTailAlpha87Samples));
static_assert(std::size(test_support::kPyramidFlatTailAlpha875Topology) ==
              std::size(test_support::kPyramidFlatTailAlpha875Samples));
static_assert(std::size(test_support::kPyramidFlatTailAlpha88Topology) ==
              std::size(test_support::kPyramidFlatTailAlpha88Samples));
static_assert(std::size(test_support::kPyramidFlatTailAlpha89Topology) ==
              std::size(test_support::kPyramidFlatTailAlpha89Samples));
static_assert(std::size(test_support::kPyramidFlatTailAlpha895Topology) ==
              std::size(test_support::kPyramidFlatTailAlpha895Samples));

void CheckDirectTopology(const char* label, const test_support::PyramidDirectSample* pool, size_t n,
                         const test_support::PyramidTopologyGolden* golden) {
  for (size_t i = 0; i < n; i++) {
    const auto& p = pool[i];
    auto cf = ComputeClosedFormPyramid(p.upper_alpha, p.lower_alpha, p.h1, p.h2, p.h3, p.dist);
    const auto& g = golden[i];
    EXPECT_EQ(cf.vtx_cnt, g.vtx_cnt) << label << "#" << i << ": vtx_cnt drift";
    EXPECT_EQ(PackFacePresent(cf), g.face_present_mask) << label << "#" << i << ": face_present drift";
    EXPECT_EQ(cf.path_tag_union, g.path_tag_union) << label << "#" << i << ": path_tag_union drift";
  }
}

TEST(ClosedFormPyramid, TopologyMatchesGoldenConstants) {
  CheckDirectTopology("wc", test_support::kPyramidWellConditionedSamples,
                      std::size(test_support::kPyramidWellConditionedSamples),
                      test_support::kPyramidWellConditionedTopology);
  CheckDirectTopology("f85", test_support::kPyramidFlatTailAlpha85Samples,
                      std::size(test_support::kPyramidFlatTailAlpha85Samples),
                      test_support::kPyramidFlatTailAlpha85Topology);
  CheckDirectTopology("f87", test_support::kPyramidFlatTailAlpha87Samples,
                      std::size(test_support::kPyramidFlatTailAlpha87Samples),
                      test_support::kPyramidFlatTailAlpha87Topology);
  CheckDirectTopology("f875", test_support::kPyramidFlatTailAlpha875Samples,
                      std::size(test_support::kPyramidFlatTailAlpha875Samples),
                      test_support::kPyramidFlatTailAlpha875Topology);
  CheckDirectTopology("f88", test_support::kPyramidFlatTailAlpha88Samples,
                      std::size(test_support::kPyramidFlatTailAlpha88Samples),
                      test_support::kPyramidFlatTailAlpha88Topology);
  CheckDirectTopology("f89", test_support::kPyramidFlatTailAlpha89Samples,
                      std::size(test_support::kPyramidFlatTailAlpha89Samples),
                      test_support::kPyramidFlatTailAlpha89Topology);
  CheckDirectTopology("f895", test_support::kPyramidFlatTailAlpha895Samples,
                      std::size(test_support::kPyramidFlatTailAlpha895Samples),
                      test_support::kPyramidFlatTailAlpha895Topology);

  // Miller pool.
  for (size_t i = 0; i < std::size(test_support::kPyramidMillerSamples); i++) {
    const auto& m = test_support::kPyramidMillerSamples[i];
    auto cf = ComputeClosedFormPyramid(m.upper_i1, m.upper_i4, m.lower_i1, m.lower_i4, m.h1, m.h2, m.h3, m.dist);
    const auto& g = test_support::kPyramidMillerTopology[i];
    EXPECT_EQ(cf.vtx_cnt, g.vtx_cnt) << "mil#" << i << ": vtx_cnt drift";
    EXPECT_EQ(PackFacePresent(cf), g.face_present_mask) << "mil#" << i << ": face_present drift";
    EXPECT_EQ(cf.path_tag_union, g.path_tag_union) << "mil#" << i << ": path_tag_union drift";
  }

  // Regular hexagonal pyramid — the owner-mandated invariant, hand-written
  // (not pool-derived): six cone planes pass exactly through each apex; the
  // closed form must recognise each apex as a SINGLE vertex (12 belt + 2 apex),
  // never a cluster of near-coincident points.
  {
    const float dist1[6] = { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
    auto cf = ComputeClosedFormPyramid(28.0f, 28.0f, 1.0f, 1.0f, 1.0f, dist1);
    EXPECT_EQ(cf.vtx_cnt, 14) << "regular α=28°, h=1,1,1, dist=1: expected 14 vertices (12 belt + 2 apex)";
  }
}

// ============================================================================
// Contract 2 — Vertex/plane self-consistency.
//
// cf's stated vertices must be mutually consistent with cf's own stated plane
// equations, as a convex body. This needs no exact witness. Two self-consistent
// facts are asserted, both at a LOOSE physical tolerance relative to the
// sample's characteristic length — never an absolute epsilon on a scale-varying
// quantity (numerical-robustness.md §2):
//   (a) Convexity: every body vertex lies inside (or on) every present face's
//       bounding half-space a·x+b·y+c·z+d ≤ 0.
//   (b) Supported faces: every present face carries ≥3 coplanar body vertices,
//       so a "present" plane is a genuine supporting polygon, not a floating
//       coefficient.
//
// NOTE (why not "every face_vtx entry lies on its own plane"): cf deliberately
// over-associates apex-line-degenerate endpoints with all six cone faces
// (geo3d_closedform.cpp appends the apex point to every 8+i / 14+i slot even
// where the point is not on that cone's plane), so face_vtx is not a strict
// on-plane list. (b) instead counts on-plane vertices from the full body set,
// which is robust to that over-association.
//
// The well-conditioned pool is used (each entry is ≥50× off production's merge
// boundary, so platform rounding can never flip a decision).
// ============================================================================

TEST(ClosedFormPyramid, VertexPlaneSelfConsistency) {
  // Loose physical tolerance: 1e-4 of the characteristic length is
  // geometrically meaningless (well above float round-off of an O(1)-coordinate
  // three-plane intersection, well below any real feature separation).
  constexpr double kRelTol = 1e-4;
  double worst_halfspace = 0.0;                 // max positive signed distance of any vertex to any present plane
  int min_on_plane = kClosedFormPyramidMaxVtx;  // min coplanar-vertex count over all present faces

  for (size_t si = 0; si < std::size(test_support::kPyramidWellConditionedSamples); si++) {
    const auto& p = test_support::kPyramidWellConditionedSamples[si];
    auto cf = ComputeClosedFormPyramid(p.upper_alpha, p.lower_alpha, p.h1, p.h2, p.h3, p.dist);
    const double char_len = ProductionCharLen(MakeSample(p));
    const double tol = kRelTol * char_len;

    for (int slot = 0; slot < kClosedFormPyramidFaceCnt; slot++) {
      if (!cf.face_present[slot]) {
        continue;
      }
      const double a = cf.plane_coef[slot * 4 + 0];
      const double b = cf.plane_coef[slot * 4 + 1];
      const double c = cf.plane_coef[slot * 4 + 2];
      const double d = cf.plane_coef[slot * 4 + 3];
      const double norm = std::sqrt(a * a + b * b + c * c);
      ASSERT_GT(norm, 0.0) << "wc#" << si << " slot " << slot << ": present face has a zero plane normal";

      int on_plane = 0;
      for (int v = 0; v < cf.vtx_cnt; v++) {
        const double signed_dist = (a * cf.vtx[v * 3 + 0] + b * cf.vtx[v * 3 + 1] + c * cf.vtx[v * 3 + 2] + d) / norm;
        // (a) Convexity: every body vertex is inside (or on) this half-space.
        worst_halfspace = std::max(worst_halfspace, signed_dist);
        EXPECT_LE(signed_dist, tol) << "wc#" << si << " slot " << slot << ": body vtx " << v
                                    << " violates half-space by " << signed_dist << " (tol " << tol << ")";
        if (std::fabs(signed_dist) <= tol) {
          on_plane++;
        }
      }
      // (b) A present face is supported by ≥3 coplanar body vertices.
      min_on_plane = std::min(min_on_plane, on_plane);
      EXPECT_GE(on_plane, 3) << "wc#" << si << " slot " << slot << ": present face has only " << on_plane
                             << " coplanar body vertices (< 3) — plane is not a genuine supporting polygon";
    }
  }
  std::fprintf(stderr, "[vertex-plane self-consistency] worst_halfspace=%.3e min_on_plane=%d\n", worst_halfspace,
               min_on_plane);
}

// ============================================================================
// Contract 3 — Degenerate graceful-degradation contract.
//
// Replaces the old DegeneratePyramidSweep (which adjudicated cf vs the
// symbolic-α oracle). Degenerate inputs must obey the contract production
// already trusts: bounded structure, finite coordinates, no overflow, and
// collapsed (near-zero-area) faces rather than a crash — never a specific
// topology. Weak assertions only, so platform rounding cannot flip a verdict.
// A NaN/overflow here would be a REAL production bug outside this task's scope
// (production is unchanged) and must be escalated, not masked.
// ============================================================================

TEST(ClosedFormPyramid, DegenerateContractSafe) {
  // Upper area bound: a present face cannot exceed a few × the characteristic
  // area — a generous ceiling that only trips on runaway/NaN, never on a
  // legitimately shaped face (observed max ratio ≈ 0.5).
  constexpr double kAreaUpperFactor = 4.0;
  // A face at or below this fraction of the characteristic area counts as
  // "collapsed" (near-zero) — the graceful-degradation signature.
  constexpr double kNearZeroFrac = 1e-6;

  const test_support::PyramidDirectSample* buckets[] = { test_support::kPyramidDegenerateSigma030Samples,
                                                         test_support::kPyramidDegenerateSigma050Samples };
  size_t sizes[] = { std::size(test_support::kPyramidDegenerateSigma030Samples),
                     std::size(test_support::kPyramidDegenerateSigma050Samples) };
  const char* labels[] = { "σ=0.30", "σ=0.50" };

  for (int b = 0; b < 2; b++) {
    int collapsed_face_samples = 0;  // samples exhibiting a near-zero-area face
    for (size_t i = 0; i < sizes[b]; i++) {
      const auto& p = buckets[b][i];
      auto cf = ComputeClosedFormPyramid(p.upper_alpha, p.lower_alpha, p.h1, p.h2, p.h3, p.dist);
      const double char_area = ProductionCharLen(MakeSample(p)) * ProductionCharLen(MakeSample(p));

      // (1) Structural bounds — the "no overflow / no out-of-bounds" contract.
      ASSERT_GE(cf.vtx_cnt, 0) << labels[b] << "#" << i << ": negative vtx_cnt";
      ASSERT_LT(cf.vtx_cnt, kClosedFormPyramidMaxVtx) << labels[b] << "#" << i << ": vtx_cnt overruns pool";
      for (int slot = 0; slot < kClosedFormPyramidFaceCnt; slot++) {
        ASSERT_GE(cf.face_vtx_cnt[slot], 0) << labels[b] << "#" << i << " slot " << slot << ": negative face_vtx_cnt";
        ASSERT_LT(cf.face_vtx_cnt[slot], kClosedFormPyramidMaxFaceVtx)
            << labels[b] << "#" << i << " slot " << slot << ": face_vtx_cnt overruns pool";
      }

      // (2) Finiteness — no NaN/Inf coordinates leaked out of the degenerate path.
      for (int v = 0; v < cf.vtx_cnt; v++) {
        for (int k = 0; k < 3; k++) {
          ASSERT_TRUE(std::isfinite(cf.vtx[v * 3 + k]))
              << labels[b] << "#" << i << ": non-finite vertex component (vtx " << v << ", comp " << k << ")";
        }
      }

      // (3) Bounded, finite, near-zero-collapsing faces — the graceful
      //     degradation signature. Every present face has a finite area within
      //     [0, kAreaUpperFactor × char_area]; a genuinely degenerate sample
      //     collapses at least one face toward zero.
      double min_present_area = std::numeric_limits<double>::infinity();
      for (int slot = 0; slot < kClosedFormPyramidFaceCnt; slot++) {
        if (!cf.face_present[slot]) {
          continue;
        }
        const double area = FacePolygonArea(cf, slot);
        ASSERT_TRUE(std::isfinite(area)) << labels[b] << "#" << i << " slot " << slot << ": non-finite face area";
        EXPECT_GE(area, 0.0) << labels[b] << "#" << i << " slot " << slot << ": negative face area";
        EXPECT_LE(area, kAreaUpperFactor * char_area)
            << labels[b] << "#" << i << " slot " << slot << ": face area " << area << " exceeds " << kAreaUpperFactor
            << " × char_area=" << char_area << " — runaway geometry";
        min_present_area = std::min(min_present_area, area);
      }
      if (min_present_area <= kNearZeroFrac * char_area) {
        collapsed_face_samples++;
      }
    }
    // Detection power: the degenerate regime must genuinely collapse faces on
    // some samples (a sentinel that can never observe the degenerate signature
    // has no teeth — numerical-robustness.md §6). This is an aggregate over the
    // bucket, not a per-sample demand, since not every degenerate sample
    // collapses a face.
    EXPECT_GT(collapsed_face_samples, 0)
        << labels[b] << ": no degenerate sample produced a near-zero-area face — the pool no longer exercises "
        << "the graceful-degradation path (regenerate) or cf changed behavior";
  }
}

// ============================================================================
// Specialised configurations: shoulder / apex / face-drop.
//
// Enforces "no special-case branches": the OR-union of ClosedFormHexPathTag
// bits across a specialised batch MUST be a subset of the union across the
// regular well-conditioned batch. Consumes only cf's own path_tag_union — the
// former oracle/production witnesses added nothing to this assertion and are
// gone. If a specialised batch fires a branch never seen on regular inputs, a
// special-case code path has been added to the solver.
// ============================================================================

uint16_t CollectRegularUnion() {
  uint16_t u = 0;
  for (const auto& raw : test_support::kPyramidWellConditionedSamples) {
    auto cf = ComputeClosedFormPyramid(raw.upper_alpha, raw.lower_alpha, raw.h1, raw.h2, raw.h3, raw.dist);
    u |= cf.path_tag_union;
  }
  return u;
}

struct SpecialisedBatch {
  const char* label;
  std::vector<PyramidSample> samples;
};

SpecialisedBatch MakeShoulderBatch() {
  SpecialisedBatch b{ "shoulder", {} };
  b.samples.push_back({ 28.0f, 28.0f, 0.6f, 1.0f, 0.6f, { 1, 1, 1, 1, 1, 1 } });
  b.samples.push_back({ 30.0f, 45.0f, 0.5f, 0.8f, 1.0f, { 1, 1, 1, 1, 1, 1 } });
  b.samples.push_back({ 60.0f, 30.0f, 1.2f, 0.4f, 0.7f, { 1, 1, 1, 1, 1, 1 } });
  return b;
}

SpecialisedBatch MakeApexBatch() {
  SpecialisedBatch b{ "apex", {} };
  b.samples.push_back({ 28.0f, 28.0f, 2.0f, 0.5f, 2.0f, { 1, 1, 1, 1, 1, 1 } });
  b.samples.push_back({ 20.0f, 30.0f, 3.0f, 0.3f, 1.5f, { 1, 1, 1, 1, 1, 1 } });
  b.samples.push_back({ 45.0f, 45.0f, 1.5f, 0.2f, 1.5f, { 1, 1, 1, 1, 1, 1 } });
  return b;
}

// NOTE: test_incidence_sampling_polygon_oracle.cpp hand-copies these dist[]
// literals (kDrop0/kDrop1/kDrop2) because this function has anonymous-namespace
// (internal) linkage and cannot be called cross-translation-unit. Keep both
// copies numerically in sync if these samples ever change.
SpecialisedBatch MakeFaceDropBatch() {
  SpecialisedBatch b{ "face-drop", {} };
  b.samples.push_back({ 28.0f, 28.0f, 1.0f, 1.0f, 1.0f, { 0.3f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f } });
  b.samples.push_back({ 30.0f, 30.0f, 1.0f, 0.5f, 1.0f, { 1.0f, 0.4f, 1.0f, 0.5f, 1.0f, 1.0f } });
  b.samples.push_back({ 45.0f, 60.0f, 0.8f, 0.6f, 0.8f, { 1.0f, 1.0f, 0.35f, 1.0f, 1.0f, 0.9f } });
  return b;
}

TEST(ClosedFormPyramid, SpecialisedConfigurationsAgreeAndNoSpecialCaseBranches) {
  // Reference union across the fixed well-conditioned pool — every branch the
  // closed-form solver walks on ordinary input.
  const uint16_t regular_union = CollectRegularUnion();
  EXPECT_NE(regular_union, 0u) << "regular pool failed to walk any tagged branch — solver instrumentation broken";

  const std::vector<SpecialisedBatch> batches{ MakeShoulderBatch(), MakeApexBatch(), MakeFaceDropBatch() };
  for (const auto& b : batches) {
    uint16_t batch_union = 0;
    for (const PyramidSample& s : b.samples) {
      auto cf = ComputeClosedFormPyramid(s.upper_alpha, s.lower_alpha, s.h1, s.h2, s.h3, s.dist);
      batch_union |= cf.path_tag_union;
    }
    // Set-inclusion, not equality: a specialised batch may hit FEWER branches
    // than the regular batch (small N, less coverage), but MUST NOT introduce
    // a branch bit unseen on regular inputs — that would indicate a
    // special-case code path added for the specialised scenario.
    EXPECT_EQ(batch_union & ~regular_union, 0u)
        << b.label << " batch triggered path-tag bits absent from the regular batch " << "(batch=0x" << std::hex
        << batch_union << " regular=0x" << regular_union << std::dec << ") — check for special-case branches";
  }
}

}  // namespace
}  // namespace lumice
