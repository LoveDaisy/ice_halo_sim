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
#include <spdlog/sinks/ostream_sink.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "core/geo3d.hpp"
#include "core/geo3d_closedform.hpp"
#include "core/math.hpp"
#include "golden-analytic/core/closed_form_samples_generated.hpp"
#include "golden-analytic/core/pyramid_topology_golden_generated.hpp"
#include "util/logger.hpp"

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
//   (c) Strict polygon membership: every entry of a present face's OWN vertex
//       list (face_vtx[slot]) lies on that face's plane. (b) searches the whole
//       body for suitable vertices and therefore cannot see a polygon that
//       lists a vertex belonging to some other face; (c) reads the polygon cf
//       actually publishes. This is the form consumers depend on —
//       simulator.cpp's BuildEntrySubTris fans face_vtx into entry triangles,
//       so an off-plane entry places sampled entry points off the surface.
//
// (c) replaces a NOTE that an earlier revision of this file carried, which
// declared the check un-assertable because cf "deliberately over-associates"
// apex-line-degenerate endpoints with all six cone faces. That was a defect
// being read as a design: when the hexagon is irregular the apex LP maximum
// degenerates from a point to a segment, and each endpoint of that segment is
// tight on only a SUBSET of the six cone planes. Appending it to all six put
// polygon entries far off their own stated plane, broke the Euler
// characteristic and left non-manifold edges. EnumerateApexPoints now reports
// which cone planes actually pass through each apex point and the emitters
// filter on it, so (c) is a strict invariant, not an aspiration.
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
  double worst_polygon_off_plane = 0.0;         // max |signed distance| of a face_vtx entry to its OWN plane

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

      // (c) Every entry of this face's own published polygon lies on this
      //     face's plane.
      for (int k = 0; k < cf.face_vtx_cnt[slot]; k++) {
        const int vi = cf.face_vtx[slot][k];
        const double off = (a * cf.vtx[vi * 3 + 0] + b * cf.vtx[vi * 3 + 1] + c * cf.vtx[vi * 3 + 2] + d) / norm;
        worst_polygon_off_plane = std::max(worst_polygon_off_plane, std::fabs(off));
        EXPECT_LE(std::fabs(off), tol) << "wc#" << si << " slot " << slot << ": face_vtx[" << k << "] = vtx " << vi
                                       << " is " << std::fabs(off) << " off its own plane (tol " << tol
                                       << ") — the published polygon is not an on-plane list";
      }
    }
  }
  std::fprintf(stderr,
               "[vertex-plane self-consistency] worst_halfspace=%.3e min_on_plane=%d worst_polygon_off_plane=%.3e\n",
               worst_halfspace, min_on_plane, worst_polygon_off_plane);
}

// ============================================================================
// Contract 2b — Structural validity of the emitted polyhedron, over a
// parameter SCAN rather than a single shape.
//
// Contract 2 runs one fixed pool; the defects this contract guards against
// live in parameter COMBINATIONS (irregular hexagon × apex-reaching height ×
// absent prism section × one-sided cone), so a scan is the assertion's
// substance, not decoration. Every case below is a legal crystal, and cf must
// emit a closed solid for each: the published polygons are the only thing
// downstream consumers (simulator.cpp's BuildEntrySubTris entry sampling,
// crystal.cpp's mesh population) ever read, so a polygon set that is not a
// closed 2-manifold is a live wrong-physics condition, not cosmetics.
//
// Four assertions per case, all on cf's own output, all judged against the
// sample's characteristic length rather than an absolute epsilon
// (doc/numerical-robustness.md §2):
//   1. every face_vtx entry lies on its own face's plane;
//   2. V − E + F == 2 (Euler characteristic of a sphere);
//   3. every polygon edge is shared by exactly 2 present faces (2-manifold);
//   4. every present face has non-zero area relative to the characteristic
//      area (a "present" face that bounds nothing is not present).
// ============================================================================

struct ScanCase {
  const char* label;
  float upper_alpha;
  float lower_alpha;
  float h1;  // upper_h
  float h2;  // prism_h
  float h3;  // lower_h
  float dist[kSideCnt];
};

// Census of cf's published polygon set: the vertex/edge/face counts an Euler
// and manifold check needs, plus the worst on-plane deviation and smallest
// face area. Reads face_vtx / face_present only — no re-derivation of the
// topology from the planes, which is the point: the contract is about what cf
// publishes, not about what could be recomputed from its coefficients.
struct StructuralCensus {
  int v = 0;                     // vertices referenced by ≥1 present face
  int e = 0;                     // distinct undirected polygon edges
  int f = 0;                     // present faces
  int non_manifold_edges = 0;    // edges NOT shared by exactly 2 present faces
  double worst_off_plane = 0.0;  // max |signed distance| of a face_vtx entry to its own plane
  double min_face_area = std::numeric_limits<double>::infinity();
};

StructuralCensus TakeStructuralCensus(const ClosedFormPyramidResult& cf) {
  StructuralCensus census;
  std::map<std::pair<int, int>, int> edge_use;
  std::vector<bool> referenced(static_cast<size_t>(std::max(cf.vtx_cnt, 1)), false);

  for (int slot = 0; slot < kClosedFormPyramidFaceCnt; slot++) {
    if (!cf.face_present[slot]) {
      continue;
    }
    census.f++;
    const int m = cf.face_vtx_cnt[slot];
    const double a = cf.plane_coef[slot * 4 + 0];
    const double b = cf.plane_coef[slot * 4 + 1];
    const double c = cf.plane_coef[slot * 4 + 2];
    const double d = cf.plane_coef[slot * 4 + 3];
    const double norm = std::sqrt(a * a + b * b + c * c);
    for (int k = 0; k < m; k++) {
      const int va = cf.face_vtx[slot][k];
      const int vb = cf.face_vtx[slot][(k + 1) % m];
      referenced[static_cast<size_t>(va)] = true;
      edge_use[std::make_pair(std::min(va, vb), std::max(va, vb))]++;
      if (norm > 0.0) {
        const double off = (a * cf.vtx[va * 3 + 0] + b * cf.vtx[va * 3 + 1] + c * cf.vtx[va * 3 + 2] + d) / norm;
        census.worst_off_plane = std::max(census.worst_off_plane, std::fabs(off));
      }
    }
    census.min_face_area = std::min(census.min_face_area, FacePolygonArea(cf, slot));
  }
  for (int i = 0; i < cf.vtx_cnt; i++) {
    if (referenced[static_cast<size_t>(i)]) {
      census.v++;
    }
  }
  census.e = static_cast<int>(edge_use.size());
  for (const auto& [edge, use_cnt] : edge_use) {
    (void)edge;
    if (use_cnt != 2) {
      census.non_manifold_edges++;
    }
  }
  return census;
}

TEST(ClosedFormPyramid, StructuralValidityParamScan) {
  // Same loose relative on-plane tolerance as Contract 2 — well above float
  // round-off of an O(1)-coordinate three-plane intersection, far below any
  // real feature separation.
  constexpr double kRelTol = 1e-4;
  // Non-zero-area floor, the mirror of DegenerateContractSafe's kNearZeroFrac:
  // there a face at/below this fraction of the characteristic area counts as
  // gracefully collapsed; here every present face must stay ABOVE it.
  constexpr double kMinAreaFrac = 1e-6;

  // Regular / M-crystal (the reported irregular shape) / mildly irregular /
  // phase-rotated irregular hexagons, crossed with prism_h ∈ {0, 0.3},
  // upper_h & lower_h ∈ {0, 0.5, 1.0} and one-sided vs two-sided cones.
  // Wedge angles are swapped in the last case so the tight-plane criterion
  // cannot be passing by coincidence on one particular pair of slopes.
  static const ScanCase kCases[] = {
    { "regular/prism0/apex-up", 28.0f, 28.0f, 1.0f, 0.0f, 0.0f, { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f } },
    { "regular/prism.3/apex-up", 28.0f, 28.0f, 1.0f, 0.3f, 0.0f, { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f } },
    { "regular/prism0/trunc-up", 28.0f, 28.0f, 0.5f, 0.0f, 0.0f, { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f } },
    { "M/prism0/apex-up", 17.0f, 28.0f, 1.0f, 0.0f, 0.0f, { 0.2f, 1.0f, 1.0f, 0.2f, 1.0f, 1.0f } },
    { "M/prism.3/apex-up", 17.0f, 28.0f, 1.0f, 0.3f, 0.0f, { 0.2f, 1.0f, 1.0f, 0.2f, 1.0f, 1.0f } },
    { "mild-irregular/prism0/apex-up", 17.0f, 28.0f, 1.0f, 0.0f, 0.0f, { 0.9f, 1.0f, 1.0f, 0.9f, 1.0f, 1.0f } },
    { "M/prism0/apex-both", 17.0f, 28.0f, 1.0f, 0.0f, 1.0f, { 0.2f, 1.0f, 1.0f, 0.2f, 1.0f, 1.0f } },
    { "regular/prism0/apex-down", 28.0f, 28.0f, 0.0f, 0.0f, 1.0f, { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f } },
    { "M/prism.3/trunc-both", 17.0f, 28.0f, 0.5f, 0.3f, 0.5f, { 0.2f, 1.0f, 1.0f, 0.2f, 1.0f, 1.0f } },
    { "rot-irregular/prism0/apex-up", 17.0f, 28.0f, 1.0f, 0.0f, 0.0f, { 1.0f, 0.2f, 1.0f, 1.0f, 0.2f, 1.0f } },
    { "M/prism0/apex-up/wedge-swap", 28.0f, 17.0f, 1.0f, 0.0f, 0.0f, { 0.2f, 1.0f, 1.0f, 0.2f, 1.0f, 1.0f } },
  };

  for (const ScanCase& sc : kCases) {
    auto cf = ComputeClosedFormPyramid(sc.upper_alpha, sc.lower_alpha, sc.h1, sc.h2, sc.h3, sc.dist);

    PyramidSample s;
    s.upper_alpha = sc.upper_alpha;
    s.lower_alpha = sc.lower_alpha;
    s.h1 = sc.h1;
    s.h2 = sc.h2;
    s.h3 = sc.h3;
    for (int i = 0; i < kSideCnt; i++) {
      s.dist[i] = sc.dist[i];
    }
    const double char_len = ProductionCharLen(s);
    const double tol = kRelTol * char_len;
    const double char_area = char_len * char_len;

    const StructuralCensus census = TakeStructuralCensus(cf);

    // EXPECT + continue rather than ASSERT: an ASSERT here returns from the
    // whole TEST body and would hide every later case in the scan behind the
    // first empty one — exactly the coverage loss this contract exists to
    // prevent.
    EXPECT_GT(census.f, 0) << sc.label << ": no present face at all — cf emitted nothing for a legal crystal";
    if (census.f == 0) {
      // Report the empty case in its own shape. The full diagnostic line below
      // would print min_area as `inf` here (no present face ever lowers the
      // running minimum), which reads as a runaway value rather than as what it
      // is: cf emitted nothing at all. This case is not hypothetical — it is
      // the signature of the vanishing-pyramid defect tracked separately, so
      // the line a future reader greps for should say so plainly.
      std::fprintf(stderr, "[structural-scan] %-30s EMPTY — cf emitted no present face\n", sc.label);
      continue;
    }

    std::fprintf(stderr,
                 "[structural-scan] %-30s V=%2d E=%2d F=%2d chi=%3d non_manifold=%2d off_plane=%.3e min_area=%.3e\n",
                 sc.label, census.v, census.e, census.f, census.v - census.e + census.f, census.non_manifold_edges,
                 census.worst_off_plane, census.min_face_area);

    // (1) Every published polygon entry lies on its own face's plane.
    EXPECT_LE(census.worst_off_plane, tol)
        << sc.label << ": a face_vtx entry is " << census.worst_off_plane << " off its own plane (tol " << tol << ")";
    // (2) Closed surface of genus 0.
    EXPECT_EQ(census.v - census.e + census.f, 2)
        << sc.label << ": V−E+F = " << (census.v - census.e + census.f) << " (V=" << census.v << " E=" << census.e
        << " F=" << census.f << ") — the emitted polygon set is not a closed sphere-like surface";
    // (3) 2-manifold: every edge borders exactly two faces.
    EXPECT_EQ(census.non_manifold_edges, 0) << sc.label << ": " << census.non_manifold_edges
                                            << " polygon edge(s) are not shared by exactly 2 present faces";
    // (4) A present face must actually bound area.
    EXPECT_GT(census.min_face_area, kMinAreaFrac * char_area)
        << sc.label << ": smallest present-face area " << census.min_face_area << " ≤ " << kMinAreaFrac
        << " × char_area=" << char_area << " — a face is marked present but bounds nothing";
  }
}

// ============================================================================
// Contract 2c — the near-apex height window.
//
// upper_h / lower_h approaching 1 walks a cone's truncation plane onto its
// apex. Two decisions partition that axis: "does a cross-section polygon still
// exist" (the 2D solver's feasibility tolerance) and "has the apex collapsed"
// (the gate that switches the emitter to direct apex enumeration). Choose the
// two independently and the band between them belongs to neither: the whole
// cone is dropped without a word, the crystal comes out as an open prism, and
// every consumer downstream keeps running on it. The scan below is what makes
// that band an assertion rather than a code-reading exercise.
//
// It walks 1 − h from 1e-3 down to 1e-9 (eight points per decade) for the
// regular hexagon and for one whose apex degenerates to a ridge, with and
// without a prism section, on each cone side independently. The lower side is
// not assumed to mirror the upper one — it is scanned.
//
// The assertions differ from StructuralValidityParamScan in exactly one place,
// and the difference removes an inapplicable proxy rather than rigor: that
// scan's minimum-area floor is a fixed fraction of the characteristic area,
// which near the apex asserts "this crystal is not close to its apex" rather
// than "this polygon is valid" — the top cap's area genuinely tends to zero as
// h → 1. What replaces it is stronger where it counts: every one of the six
// cone faces on the coned side must be PRESENT, which is exactly the property
// a vanishing cone destroys.
// ============================================================================

// Number of present faces among the six-slot band starting at base_slot
// (8 = upper cone, 14 = lower cone).
int PresentFaceCountInBand(const ClosedFormPyramidResult& cf, int base_slot) {
  int n = 0;
  for (int i = 0; i < kSideCnt; i++) {
    if (cf.face_present[base_slot + i]) {
      n++;
    }
  }
  return n;
}

TEST(ClosedFormPyramid, NearApexHeightWindowStructuralValidity) {
  // Same loose relative on-plane tolerance as the other structural contracts.
  constexpr double kRelTol = 1e-4;
  // 1 − h sweep: eight points per decade from 1e-3 to 1e-9. The tail below
  // ~6e-8 is where float32 rounds h to exactly 1.0f; it is scanned rather than
  // trimmed, because "the window closes again once the input rounds onto the
  // apex" is a property of the float32 grid and not of the solver — widening h
  // to double would move that edge, so the scan should be able to see it.
  constexpr int kStepsPerDecade = 8;
  constexpr int kDecades = 6;

  struct Shape {
    const char* label;
    float upper_alpha;
    float lower_alpha;
    float dist[kSideCnt];
  };
  // "regular": the apex is a single point where all six cone planes concur.
  // "ridge": one opposite pair pulled in, so the apex degenerates to a segment.
  // The two collapse modes shrink the cross section at different rates (a point
  // contracts isotropically, a ridge contracts into a sliver), so neither one
  // stands in for the other.
  static const Shape kShapes[] = {
    { "regular", 28.0f, 28.0f, { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f } },
    { "ridge", 17.0f, 28.0f, { 0.2f, 1.0f, 1.0f, 0.2f, 1.0f, 1.0f } },
  };
  static const float kPrismH[] = { 0.0f, 0.3f };

  int checked = 0;
  std::vector<std::string> failures;

  for (const Shape& sh : kShapes) {
    for (float prism_h : kPrismH) {
      for (int side = 0; side < 2; side++) {
        const char* side_label = (side == 0) ? "upper" : "lower";
        const int cone_base_slot = (side == 0) ? 8 : 14;
        // Per-group tally, printed unconditionally: on a green run it states
        // the window was walked and found closed; on a red one it states how
        // wide the open band is and where it sits, which is the diagnostic a
        // reader needs and which 200-plus individual failure lines bury.
        int group_bad = 0;
        double bad_t_hi = 0.0;
        double bad_t_lo = 0.0;
        for (int step = 0; step <= kStepsPerDecade * kDecades; step++) {
          const double t = 1e-3 * std::pow(10.0, -static_cast<double>(step) / kStepsPerDecade);
          const float h = static_cast<float>(1.0 - t);
          const float h1 = (side == 0) ? h : 0.0f;
          const float h3 = (side == 0) ? 0.0f : h;

          auto cf = ComputeClosedFormPyramid(sh.upper_alpha, sh.lower_alpha, h1, prism_h, h3, sh.dist);

          PyramidSample s;
          s.upper_alpha = sh.upper_alpha;
          s.lower_alpha = sh.lower_alpha;
          s.h1 = h1;
          s.h2 = prism_h;
          s.h3 = h3;
          for (int i = 0; i < kSideCnt; i++) {
            s.dist[i] = sh.dist[i];
          }
          const double tol = kRelTol * ProductionCharLen(s);

          const StructuralCensus census = TakeStructuralCensus(cf);
          const int cone_faces = PresentFaceCountInBand(cf, cone_base_slot);
          checked++;

          const bool ok = census.f > 0 && cone_faces == kSideCnt && census.worst_off_plane <= tol &&
                          census.v - census.e + census.f == 2 && census.non_manifold_edges == 0 &&
                          census.min_face_area > 0.0;
          if (!ok) {
            if (group_bad == 0) {
              bad_t_hi = t;
            }
            bad_t_lo = t;
            group_bad++;
            char buf[320];
            std::snprintf(buf, sizeof(buf),
                          "%s/prism%.1f/%s 1-h=%.3e: cone_faces=%d/6 V=%d E=%d F=%d chi=%d non_manifold=%d "
                          "off_plane=%.3e (tol %.3e) min_area=%.3e",
                          sh.label, static_cast<double>(prism_h), side_label, t, cone_faces, census.v, census.e,
                          census.f, census.v - census.e + census.f, census.non_manifold_edges, census.worst_off_plane,
                          tol, census.min_face_area);
            failures.emplace_back(buf);
          }
        }
        if (group_bad == 0) {
          std::fprintf(stderr, "[near-apex-window] %-8s prism=%.1f %-5s: %2d/%2d heights valid\n", sh.label,
                       static_cast<double>(prism_h), side_label, kStepsPerDecade * kDecades + 1,
                       kStepsPerDecade * kDecades + 1);
        } else {
          std::fprintf(stderr, "[near-apex-window] %-8s prism=%.1f %-5s: %2d/%2d INVALID, 1-h in [%.3e, %.3e]\n",
                       sh.label, static_cast<double>(prism_h), side_label, group_bad, kStepsPerDecade * kDecades + 1,
                       bad_t_lo, bad_t_hi);
        }
      }
    }
  }

  std::fprintf(stderr, "[near-apex-window] checked=%d failed=%zu\n", checked, failures.size());
  for (size_t i = 0; i < failures.size() && i < 8; i++) {
    std::fprintf(stderr, "[near-apex-window] %s\n", failures[i].c_str());
  }
  EXPECT_TRUE(failures.empty()) << failures.size() << " of " << checked
                                << " near-apex heights produced a structurally invalid solid (first: "
                                << (failures.empty() ? std::string("-") : failures.front()) << ")";
}

// ============================================================================
// Contract 2d — the apex-rescue degradation must stay unreachable, and must
// announce itself if it ever becomes reachable again.
//
// "A cone's truncation cross section is empty while the collapse gate says the
// cone is not at its apex" is the state that used to swallow a whole cone in
// silence. Both decisions now read one tolerance, so no legal input should be
// able to reach it — and this sweep is what makes "should" checkable, over
// shape patterns (which faces are pulled in), overall scale, spread ratio,
// wedge angle, prism height, height approaching 1 from 1e-2 down to 1e-7, and
// both cone sides.
//
// Two things keep the sweep from being a sentinel with no teeth, which this
// file has been burned by before:
//   • an anti-vacuous counter — the sweep must actually land in the near-apex
//     regime (configurations whose top cap is gone while the requested height
//     is below 1, i.e. the apex path was taken) many times over, or a green
//     result means only that the sweep never went where the defect lives;
//   • the first offending configuration, if one is ever found, is re-run under
//     a log capture and the captured text is reported — so the failure names
//     the input AND proves the warning that accompanies the degradation
//     actually reaches the log.
//
// Detection power was measured by inverting the gate back to the absolute
// epsilon it replaced; scripts/verify_apex_rescue_warning_detection_power.sh
// re-runs that experiment on demand.
// ============================================================================

// Captures everything the global logger emits for the lifetime of the object.
// RAII rather than a manual remove_sink, because GetSharedSink() is a
// process-wide singleton: an early return with the sink still attached would
// leave later tests in this binary writing into a destroyed ostringstream.
class LogCapture {
 public:
  LogCapture() : sink_(std::make_shared<spdlog::sinks::ostream_sink_mt>(oss_)) { GetSharedSink()->add_sink(sink_); }

  ~LogCapture() { GetSharedSink()->remove_sink(sink_); }

  LogCapture(const LogCapture&) = delete;
  LogCapture& operator=(const LogCapture&) = delete;

  std::string Text() const { return oss_.str(); }

 private:
  std::ostringstream oss_;
  std::shared_ptr<spdlog::sinks::ostream_sink_mt> sink_;
};

TEST(ClosedFormPyramid, ApexRescueDegradationNeverFiresOnLegalShapes) {
  // Which of the six face distances are pulled in — the apex is a point when
  // none are, a ridge for an opposite pair, and something else again for an
  // adjacent pair or a three-fold pattern. Each shrinks the near-apex cross
  // section along a different number of axes.
  struct Pattern {
    const char* label;
    int pulled_in[kSideCnt];
  };
  static const Pattern kPatterns[] = {
    { "none", { 0, 0, 0, 0, 0, 0 } },       { "opposite-pair", { 1, 0, 0, 1, 0, 0 } },
    { "single", { 1, 0, 0, 0, 0, 0 } },     { "adjacent-pair", { 1, 1, 0, 0, 0, 0 } },
    { "three-fold", { 1, 0, 1, 0, 1, 0 } }, { "five", { 1, 1, 1, 1, 1, 0 } },
  };
  // Overall size spans the range real crystals use and four decades past it in
  // both directions: the tolerance this test is about is scale-clamped, so
  // whether a configuration sits above or below the clamp changes which end of
  // the binding is doing the work.
  static const double kScales[] = { 0.01, 0.05, 0.2, 1.0, 5.0, 20.0, 100.0, 1000.0 };
  static const double kRatios[] = { 1.0, 0.9, 0.5, 0.2, 0.05, 0.01, 0.002 };
  static const float kAlphas[] = { 0.2f, 5.0f, 28.0f, 60.0f, 89.0f };
  static const float kPrismH[] = { 0.0f, 0.3f, 2.0f };
  constexpr int kStepsPerDecade = 8;
  constexpr int kDecades = 5;

  int swept = 0;
  int apex_path_taken = 0;
  int degraded = 0;
  std::string first_offender;
  float offender_args[5] = {};  // alpha, h1, h2, h3 + scale, for the capture re-run
  float offender_dist[kSideCnt] = {};

  for (const Pattern& pat : kPatterns) {
    for (double scale : kScales) {
      for (double ratio : kRatios) {
        float dist[kSideCnt];
        for (int i = 0; i < kSideCnt; i++) {
          dist[i] = static_cast<float>(scale * (pat.pulled_in[i] != 0 ? ratio : 1.0));
        }
        for (float alpha : kAlphas) {
          for (float prism_h : kPrismH) {
            for (int step = 0; step <= kStepsPerDecade * kDecades; step++) {
              const double t = 1e-2 * std::pow(10.0, -static_cast<double>(step) / kStepsPerDecade);
              const float h = static_cast<float>(1.0 - t);
              for (int side = 0; side < 2; side++) {
                const float h1 = (side == 0) ? h : 0.0f;
                const float h3 = (side == 0) ? 0.0f : h;
                auto cf = ComputeClosedFormPyramid(alpha, alpha, h1, prism_h, h3, dist);
                swept++;
                // The apex path is what the collapse gate switches to; its
                // signature is the coned side's basal cap being gone even
                // though a truncation below the apex was requested.
                if (!cf.face_present[side == 0 ? 0 : 1]) {
                  apex_path_taken++;
                }
                if ((cf.path_tag_union & kClosedFormPathTagApexRescueDegraded) != 0) {
                  degraded++;
                  if (first_offender.empty()) {
                    char buf[256];
                    std::snprintf(buf, sizeof(buf), "%s scale=%g ratio=%g alpha=%g prism_h=%g %s 1-h=%.3e", pat.label,
                                  scale, ratio, static_cast<double>(alpha), static_cast<double>(prism_h),
                                  (side == 0) ? "upper" : "lower", t);
                    first_offender = buf;
                    offender_args[0] = alpha;
                    offender_args[1] = h1;
                    offender_args[2] = prism_h;
                    offender_args[3] = h3;
                    for (int i = 0; i < kSideCnt; i++) {
                      offender_dist[i] = dist[i];
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
  }

  std::fprintf(stderr, "[apex-rescue] swept=%d apex_path_taken=%d degraded=%d\n", swept, apex_path_taken, degraded);

  // Anti-vacuous: a sweep that never reaches the apex path cannot say anything
  // about a degradation that only exists there.
  EXPECT_GT(apex_path_taken, swept / 100) << "the sweep barely ever took the apex path (" << apex_path_taken << " of "
                                          << swept << ") — it is not exercising the regime the degradation lives in";

  if (degraded > 0) {
    // Re-run the first offender alone, under a log capture, so the failure
    // reports the input AND the warning the degradation is required to emit.
    std::string text;
    {
      LogCapture capture;
      ComputeClosedFormPyramid(offender_args[0], offender_args[0], offender_args[1], offender_args[2], offender_args[3],
                               offender_dist);
      text = capture.Text();
    }
    EXPECT_NE(text.find("no cross-section"), std::string::npos)
        << "the apex-rescue degradation fired without logging a warning naming the input — captured log was: " << text;
    std::fprintf(stderr, "[apex-rescue] first offender: %s\n[apex-rescue] captured log: %s\n", first_offender.c_str(),
                 text.c_str());
  }
  EXPECT_EQ(degraded, 0) << degraded << " of " << swept
                         << " legal shapes reached the apex-rescue degradation — the collapse gate and the "
                            "cross-section feasibility test no longer agree on where the apex starts. First: "
                         << first_offender;
}

// ============================================================================
// Contract 2e — what the apex snap costs.
//
// The collapse gate claims a neighbourhood of the apex and snaps the requested
// truncation onto it, so a height landing inside that neighbourhood yields a
// solid whose tip is slightly too tall. That is the price of closing the band,
// and it is only worth paying if it is bounded by the gate's own width — the
// alternative in the same neighbourhood is not an exact crystal, it is a
// crystal missing a cone.
//
// The bound is stated in inset units, where the gate is defined: the solver's
// feasibility tolerance divided by the fixed factor √3/4 relating a half-plane
// offset to an inset. It is written out here rather than read from production,
// because a test that recomputes the implementation's own expression and
// compares it to itself asserts nothing.
//
// Measured, not assumed: the sweep reports the largest gap it actually snapped
// away, both as an absolute inset (comparable to the scale-clamped 1.15e-4) and
// as a fraction of each configuration's own gate width, plus the resulting
// vertical displacement of the tip.
// ============================================================================

TEST(ClosedFormPyramid, ApexSnapDeviationStaysWithinTheGateWidth) {
  // √3/4 — a mathematical constant, restated rather than imported because it
  // cannot drift; nobody decides its value.
  constexpr double kInsetFactor = 0.25 * 1.7320508075688772935;
  // The gate-width formula below is deliberately re-derived here instead of
  // calling GapToleranceForScale: a test that computes its expectation with the
  // production expression proves only that the expression equals itself. The
  // coefficient, though, is a tuned number someone may change on purpose, and a
  // stale copy of it would leave this test quietly measuring a gate that no
  // longer exists. So the formula stays independent and only the constant is
  // pinned to production's.
  constexpr double kTolCoefficient = 5.0;
  static_assert(kTolCoefficient == lumice::kClosedFormGapToleranceCoefficient,
                "test's gate-width coefficient drifted from GapToleranceForScale's");

  static const double kScales[] = { 0.01, 0.2, 1.0, 20.0, 1000.0 };
  static const double kRatios[] = { 1.0, 0.9, 0.5, 0.2, 0.02 };
  static const float kAlphas[] = { 0.2f, 5.0f, 28.0f, 60.0f, 89.0f };
  static const float kPrismH[] = { 0.0f, 0.3f };
  // Dense enough in 1 − h that some samples land just inside the gate edge,
  // which is where the bound is actually under test.
  constexpr int kStepsPerDecade = 32;
  constexpr int kDecades = 6;

  int snapped = 0;
  double worst_ratio = 0.0;         // snapped gap / that configuration's gate width
  double worst_gap_clamped = 0.0;   // largest snapped gap where the scale clamp was active
  double worst_tip_shift = 0.0;     // tip displacement, relative to the crystal size
  double worst_gap_fraction = 0.0;  // snapped gap as a fraction of the apex inset
  std::string worst_label;

  for (double scale : kScales) {
    for (double ratio : kRatios) {
      // One opposite pair pulled in (ridge apex) and the regular hexagon are
      // both covered: ratio == 1.0 degenerates the first family into the
      // second.
      float dist[kSideCnt] = {
        static_cast<float>(scale * ratio), static_cast<float>(scale), static_cast<float>(scale),
        static_cast<float>(scale * ratio), static_cast<float>(scale), static_cast<float>(scale)
      };
      for (float alpha : kAlphas) {
        for (float prism_h : kPrismH) {
          for (int side = 0; side < 2; side++) {
            // The apex inset of this shape, read from a request that reaches
            // the apex outright. The snap is then detectable exactly: a
            // truncation below the apex reports its own inset, a snapped one
            // reports this value. Absence of the basal cap is NOT a usable
            // signal — at a near-horizontal wedge the cap's own corners merge
            // under the 3D vertex dedup tolerance, which scales with the tip's
            // z, so the cap can be gone for reasons that have nothing to do
            // with this gate.
            const float apex_h1 = (side == 0) ? 1.0f : 0.0f;
            const float apex_h3 = (side == 0) ? 0.0f : 1.0f;
            auto at_apex = ComputeClosedFormPyramid(alpha, alpha, apex_h1, prism_h, apex_h3, dist);
            const float apex_inset_f = (side == 0) ? at_apex.inset_at_top : at_apex.inset_at_bottom;
            if (!(apex_inset_f > 0.0f)) {
              continue;  // degenerate LP — there is no apex to snap onto.
            }
            const double apex_inset = static_cast<double>(apex_inset_f);

            for (int step = 0; step <= kStepsPerDecade * kDecades; step++) {
              const double t = 1e-2 * std::pow(10.0, -static_cast<double>(step) / kStepsPerDecade);
              const float h = static_cast<float>(1.0 - t);
              const float h1 = (side == 0) ? h : 0.0f;
              const float h3 = (side == 0) ? 0.0f : h;
              auto cf = ComputeClosedFormPyramid(alpha, alpha, h1, prism_h, h3, dist);

              const float inset_f = (side == 0) ? cf.inset_at_top : cf.inset_at_bottom;
              if (inset_f != apex_inset_f) {
                continue;  // truncated as requested; nothing was snapped away.
              }
              // The gap the request asked for and the snap removed.
              const double gap = (1.0 - static_cast<double>(h)) * apex_inset;
              snapped++;

              // The gate width at this configuration: the same scale→tolerance
              // clamp the solver applies, evaluated on the requested cross
              // section, divided back into inset units.
              double offset_scale = 0.0;
              const double m_requested = static_cast<double>(h) * apex_inset;
              for (int i = 0; i < kSideCnt; i++) {
                offset_scale =
                    std::max(offset_scale, std::fabs(kInsetFactor * (static_cast<double>(dist[i]) - m_requested)));
              }
              const bool clamp_active = offset_scale <= 1.0;
              const double gate_width =
                  kTolCoefficient * static_cast<double>(math::kFloatEps) * std::max(offset_scale, 1.0) / kInsetFactor;
              const double ratio_of_gate = gap / gate_width;
              const double a1 = A1FromAlpha(alpha, (side == 0) ? h1 : h3);
              const double tip_shift = std::max(0.0, a1) * gap;

              if (ratio_of_gate > worst_ratio) {
                worst_ratio = ratio_of_gate;
                char buf[224];
                std::snprintf(buf, sizeof(buf), "scale=%g ratio=%g alpha=%g prism_h=%g %s 1-h=%.3e gap=%.4e", scale,
                              ratio, static_cast<double>(alpha), static_cast<double>(prism_h),
                              (side == 0) ? "upper" : "lower", t, gap);
                worst_label = buf;
              }
              if (clamp_active) {
                worst_gap_clamped = std::max(worst_gap_clamped, gap);
              }
              worst_tip_shift = std::max(worst_tip_shift, tip_shift / scale);
              worst_gap_fraction = std::max(worst_gap_fraction, gap / apex_inset);
            }
          }
        }
      }
    }
  }

  const double kClampedGateWidth = kTolCoefficient * static_cast<double>(math::kFloatEps) / kInsetFactor;
  std::fprintf(stderr,
               "[apex-snap] snapped=%d worst_gap/gate=%.4f worst_gap(clamped)=%.4e (clamped gate width %.4e) "
               "worst_gap/apex_inset=%.4e worst_tip_shift/scale=%.4e\n",
               snapped, worst_ratio, worst_gap_clamped, kClampedGateWidth, worst_gap_fraction, worst_tip_shift);
  std::fprintf(stderr, "[apex-snap] worst case: %s\n", worst_label.c_str());

  // Anti-vacuous, both directions: the sweep must snap something, and must get
  // close enough to the gate edge that the bound is genuinely under test rather
  // than satisfied by samples sitting deep inside it.
  ASSERT_GT(snapped, 0) << "no configuration took the apex path — the sweep never exercised the snap";
  EXPECT_GT(worst_ratio, 0.5) << "the closest sample sat at " << worst_ratio
                              << " of its gate width — the sweep is too coarse to test the bound";

  EXPECT_LE(worst_ratio, 1.0) << "a snapped gap exceeded the gate width that authorised it (" << worst_label
                              << ") — the inset↔offset conversion in the gate is not the one that bounds the error";
  EXPECT_LE(worst_gap_clamped, kClampedGateWidth) << "largest snapped gap in the scale-clamped regime was "
                                                  << worst_gap_clamped << ", above the predicted " << kClampedGateWidth;
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
