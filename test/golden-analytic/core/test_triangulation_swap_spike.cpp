// Triangulation-swap diagnostic scaffold: compares production triangle
// enumeration (FillHexCrystalCoef + CreateConvexPolyhedronMesh + Triangulate,
// see src/core/geo3d.cpp) against a candidate closed-form fan triangulation
// (ComputeClosedFormPrism / ComputeClosedFormPyramid + fan-per-face). Purpose:
// answer whether a natural closed-form fan produces triangles bit-identical to
// production or whether an explicit reindexing pass is required to preserve
// RNG bit-parity of InitRay_p_fid.RandomSample + SampleTrianglePoint
// (src/core/simulator.cpp).
//
// Output is diagnostic: the tests always pass — they print multiset-equality vs
// sequence-equality statistics on a swept parameter grid. Retained as a report,
// not a verdict.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <tuple>
#include <vector>

#include "core/geo3d.hpp"
#include "core/geo3d_closedform.hpp"
#include "core/math.hpp"

namespace lumice {
namespace {

struct Triangle {
  float v[3][3];  // [vertex][xyz]
};

std::vector<Triangle> ExtractTriangles(const Mesh& mesh) {
  std::vector<Triangle> tris;
  auto tri_cnt = mesh.GetTriangleCnt();
  tris.reserve(tri_cnt);
  const auto* vtx = const_cast<Mesh&>(mesh).GetVtxPtr(0);
  const auto* tri_idx = const_cast<Mesh&>(mesh).GetTrianglePtr(0);
  for (size_t i = 0; i < tri_cnt; i++) {
    Triangle t;
    for (int k = 0; k < 3; k++) {
      int idx = tri_idx[i * 3 + k];
      for (int j = 0; j < 3; j++) {
        t.v[k][j] = vtx[idx * 3 + j];
      }
    }
    tris.push_back(t);
  }
  return tris;
}

bool VtxEq(const float* a, const float* b, float tol) {
  return std::fabs(a[0] - b[0]) <= tol && std::fabs(a[1] - b[1]) <= tol && std::fabs(a[2] - b[2]) <= tol;
}

bool TriEqExact(const Triangle& a, const Triangle& b, float tol) {
  for (int i = 0; i < 3; i++) {
    if (!VtxEq(a.v[i], b.v[i], tol)) {
      return false;
    }
  }
  return true;
}

// True iff a and b are the same triangle up to any permutation of vertex slots.
bool TriEqUnordered(const Triangle& a, const Triangle& b, float tol) {
  static const int kPerm[6][3] = { { 0, 1, 2 }, { 0, 2, 1 }, { 1, 0, 2 }, { 1, 2, 0 }, { 2, 0, 1 }, { 2, 1, 0 } };
  for (auto& p : kPerm) {
    if (VtxEq(a.v[0], b.v[p[0]], tol) && VtxEq(a.v[1], b.v[p[1]], tol) && VtxEq(a.v[2], b.v[p[2]], tol)) {
      return true;
    }
  }
  return false;
}

// True iff a matches b as an ORIENTED triangle (up to cyclic rotation only,
// no winding flip). This is the tighter test that a fan-triangulation
// candidate must satisfy for the same face — same winding, same fan start
// vertex.
bool TriEqCyclic(const Triangle& a, const Triangle& b, float tol) {
  static const int kRot[3][3] = { { 0, 1, 2 }, { 1, 2, 0 }, { 2, 0, 1 } };
  for (auto& r : kRot) {
    if (VtxEq(a.v[0], b.v[r[0]], tol) && VtxEq(a.v[1], b.v[r[1]], tol) && VtxEq(a.v[2], b.v[r[2]], tol)) {
      return true;
    }
  }
  return false;
}

// ---- Candidate closed-form fan triangulation for prism ---------------------
// Builds triangles from ClosedFormPrismResult by fanning each face's CCW ring.
// Face order and per-face vertex order follow the closed-form's own convention:
//   slot 0: upper basal = corner ring at z=+h/2 in CCW order (as returned by
//           ComputeClosedFormPrism's `corner_x/corner_y[]`, i.e. the shared
//           2D CCW ring)
//   slot 1: lower basal = same corners at z=-h/2, reversed for outward normal
//   slot 2+i: side face i = quad (top[i], top[i+1], bot[i+1], bot[i]) CCW
// This is the *simplest* candidate; alignment with production ordering is
// exactly what the spike measures.
std::vector<Triangle> ClosedFormPrismTriangulation(float h, const float dist[6]) {
  auto result = ComputeClosedFormPrism(h, dist);
  std::vector<Triangle> tris;
  int n = result.corner_cnt;
  if (n < 3)
    return tris;
  const float half_h = h * 0.5f;
  auto top = [&](int i) { return std::array<float, 3>{ result.corner_x[i], result.corner_y[i], half_h }; };
  auto bot = [&](int i) { return std::array<float, 3>{ result.corner_x[i], result.corner_y[i], -half_h }; };
  auto emit = [&](const std::array<float, 3>& a, const std::array<float, 3>& b, const std::array<float, 3>& c) {
    Triangle t;
    for (int j = 0; j < 3; j++)
      t.v[0][j] = a[j];
    for (int j = 0; j < 3; j++)
      t.v[1][j] = b[j];
    for (int j = 0; j < 3; j++)
      t.v[2][j] = c[j];
    tris.push_back(t);
  };
  // Upper basal fan: (top[0], top[i-1], top[i]) i=2..n-1
  for (int i = 2; i < n; i++)
    emit(top(0), top(i - 1), top(i));
  // Lower basal fan (reversed for outward -z): (bot[0], bot[i], bot[i-1])
  for (int i = 2; i < n; i++)
    emit(bot(0), bot(i), bot(i - 1));
  // Side quads split into 2 tris each, fan from top[i]:
  //   (top[i], top[(i+1)%n], bot[(i+1)%n]) and (top[i], bot[(i+1)%n], bot[i])
  for (int i = 0; i < n; i++) {
    int j = (i + 1) % n;
    emit(top(i), top(j), bot(j));
    emit(top(i), bot(j), bot(i));
  }
  return tris;
}

struct CompareStats {
  int total_prod = 0;
  int total_cf = 0;
  int seq_match = 0;        // bit-identical sequence position
  int cyclic_match = 0;     // same cyclic rotation (same fan-order winding)
  int unordered_match = 0;  // present as unordered multiset entry
  int prod_only = 0;        // in production, missing from cf
  int cf_only = 0;          // in cf, missing from production
};

CompareStats CompareTriangleSets(const std::vector<Triangle>& prod, const std::vector<Triangle>& cf, float tol) {
  CompareStats s{};
  s.total_prod = static_cast<int>(prod.size());
  s.total_cf = static_cast<int>(cf.size());
  int n_min = static_cast<int>(std::min(prod.size(), cf.size()));
  for (int i = 0; i < n_min; i++) {
    if (TriEqExact(prod[i], cf[i], tol)) {
      s.seq_match++;
    }
    if (TriEqCyclic(prod[i], cf[i], tol)) {
      s.cyclic_match++;
    }
  }
  std::vector<bool> used(cf.size(), false);
  for (const auto& p : prod) {
    bool found = false;
    for (size_t k = 0; k < cf.size(); k++) {
      if (used[k]) {
        continue;
      }
      if (TriEqUnordered(p, cf[k], tol)) {
        used[k] = true;
        s.unordered_match++;
        found = true;
        break;
      }
    }
    if (!found) {
      s.prod_only++;
    }
  }
  for (size_t k = 0; k < cf.size(); k++) {
    if (!used[k]) {
      s.cf_only++;
    }
  }
  return s;
}

TEST(TriangulationSwapSpike, PrismSweepReport) {
  // Regular + slightly perturbed prisms. Keep it small — this is a report,
  // not a stress test.
  const float h_grid[] = { 0.5f, 1.0f, 2.0f, 5.0f };
  const int per_h = 32;
  std::mt19937 rng(0xC0FFEEu);
  std::normal_distribution<float> jitter(0.0f, 0.05f);

  int total_samples = 0;
  int total_seq_full = 0;  // all triangles bit-identical
  int total_set_full = 0;  // multi-sets match (bit-parity via reindex possible)
  int total_broken = 0;    // multi-sets differ (deep problem)
  long long total_seq_hits = 0;
  long long total_cyclic_hits = 0;
  long long total_set_hits = 0;
  long long total_tris_prod = 0;
  long long total_tris_cf = 0;
  std::string first_mismatch_report;

  for (float h : h_grid) {
    for (int k = 0; k < per_h; k++) {
      float dist[6];
      for (int i = 0; i < 6; i++)
        dist[i] = 1.0f + jitter(rng);
      float coef[kMaxHexCrystalPlanes * 4];
      auto cnt = FillHexCrystalCoef(0, 0, 0, h, 0, dist, coef);
      if (cnt == 0)
        continue;
      Mesh prod_mesh = CreateConvexPolyhedronMesh(static_cast<int>(cnt), coef);
      auto prod = ExtractTriangles(prod_mesh);
      auto cf = ClosedFormPrismTriangulation(h, dist);
      auto stats = CompareTriangleSets(prod, cf, 1e-4f);
      total_samples++;
      total_seq_hits += stats.seq_match;
      total_cyclic_hits += stats.cyclic_match;
      total_set_hits += stats.unordered_match;
      total_tris_prod += stats.total_prod;
      total_tris_cf += stats.total_cf;
      bool set_full = stats.prod_only == 0 && stats.cf_only == 0 && stats.total_prod == stats.total_cf;
      bool seq_full = set_full && stats.seq_match == stats.total_prod;
      if (seq_full)
        total_seq_full++;
      if (set_full)
        total_set_full++;
      if (!set_full) {
        total_broken++;
        if (first_mismatch_report.empty()) {
          char buf[512];
          std::snprintf(buf, sizeof(buf),
                        "  prism h=%.3f dist=[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]"
                        " prod=%d cf=%d seq=%d set=%d prod_only=%d cf_only=%d\n",
                        static_cast<double>(h), static_cast<double>(dist[0]), static_cast<double>(dist[1]),
                        static_cast<double>(dist[2]), static_cast<double>(dist[3]), static_cast<double>(dist[4]),
                        static_cast<double>(dist[5]), stats.total_prod, stats.total_cf, stats.seq_match,
                        stats.unordered_match, stats.prod_only, stats.cf_only);
          first_mismatch_report = buf;
        }
      }
    }
  }
  std::printf("\n=== Prism triangulation-swap spike ===\n");
  std::printf("samples=%d  seq_full=%d  set_full=%d  set_broken=%d\n", total_samples, total_seq_full, total_set_full,
              total_broken);
  std::printf(
      "total_prod_tris=%lld  total_cf_tris=%lld  seq_hits=%lld  "
      "cyclic_hits=%lld  set_hits=%lld\n",
      total_tris_prod, total_tris_cf, total_seq_hits, total_cyclic_hits, total_set_hits);
  if (!first_mismatch_report.empty()) {
    std::printf("first set-mismatch:\n%s", first_mismatch_report.c_str());
  }
}

// ---- Candidate closed-form fan triangulation for pyramid -------------------
// Reads ClosedFormPyramidResult.face_vtx[slot][…] (already CCW) and fan-
// triangulates each face from its own vertex 0.
std::vector<Triangle> ClosedFormPyramidTriangulation(const ClosedFormPyramidResult& result) {
  std::vector<Triangle> tris;
  for (int s = 0; s < kClosedFormPyramidFaceCnt; s++) {
    if (!result.face_present[s])
      continue;
    int n = result.face_vtx_cnt[s];
    if (n < 3)
      continue;
    auto vtx = [&](int local_i) {
      int gi = result.face_vtx[s][local_i];
      return std::array<float, 3>{ result.vtx[gi * 3 + 0], result.vtx[gi * 3 + 1], result.vtx[gi * 3 + 2] };
    };
    auto emit = [&](const std::array<float, 3>& a, const std::array<float, 3>& b, const std::array<float, 3>& c) {
      Triangle t;
      for (int j = 0; j < 3; j++)
        t.v[0][j] = a[j];
      for (int j = 0; j < 3; j++)
        t.v[1][j] = b[j];
      for (int j = 0; j < 3; j++)
        t.v[2][j] = c[j];
      tris.push_back(t);
    };
    for (int i = 2; i < n; i++)
      emit(vtx(0), vtx(i - 1), vtx(i));
  }
  return tris;
}

TEST(TriangulationSwapSpike, PyramidSweepReport) {
  struct PyramidCase {
    float alpha_upper;
    float alpha_lower;
    float h1;
    float h2;
    float h3;
  };
  const PyramidCase grid[] = {
    { 60.0f, 60.0f, 0.5f, 1.0f, 0.5f }, { 45.0f, 45.0f, 0.3f, 1.5f, 0.3f },
    { 70.0f, 30.0f, 0.4f, 1.0f, 0.6f }, { 87.0f, 87.0f, 0.5f, 1.0f, 0.5f },  // extreme-wedge sentinel neighborhood
    { 88.0f, 88.0f, 0.5f, 1.0f, 0.5f }, { 60.0f, 60.0f, 0.5f, 0.0f, 0.5f },  // bipyramid (h2=0)
  };
  std::mt19937 rng(0xBADCAFEu);
  std::normal_distribution<float> jitter(0.0f, 0.02f);

  int total_samples = 0;
  int total_seq_full = 0;
  int total_set_full = 0;
  int total_broken = 0;
  long long total_tris_prod = 0;
  long long total_tris_cf = 0;
  long long total_seq_hits = 0;
  long long total_cyclic_hits = 0;
  long long total_set_hits = 0;
  std::string first_mismatch_report;

  for (const auto& c : grid) {
    for (int k = 0; k < 16; k++) {
      float dist[6];
      for (int i = 0; i < 6; i++)
        dist[i] = 1.0f + jitter(rng);
      float coef[kMaxHexCrystalPlanes * 4];
      auto cnt = FillHexCrystalCoef(c.alpha_upper, c.alpha_lower, c.h1, c.h2, c.h3, dist, coef);
      if (cnt == 0)
        continue;
      Mesh prod_mesh = CreateConvexPolyhedronMesh(static_cast<int>(cnt), coef);
      auto prod = ExtractTriangles(prod_mesh);
      auto result = ComputeClosedFormPyramid(c.alpha_upper, c.alpha_lower, c.h1, c.h2, c.h3, dist);
      auto cf = ClosedFormPyramidTriangulation(result);
      auto stats = CompareTriangleSets(prod, cf, 1e-3f);
      total_samples++;
      total_seq_hits += stats.seq_match;
      total_cyclic_hits += stats.cyclic_match;
      total_set_hits += stats.unordered_match;
      total_tris_prod += stats.total_prod;
      total_tris_cf += stats.total_cf;
      bool set_full = stats.prod_only == 0 && stats.cf_only == 0 && stats.total_prod == stats.total_cf;
      bool seq_full = set_full && stats.seq_match == stats.total_prod;
      if (seq_full)
        total_seq_full++;
      if (set_full)
        total_set_full++;
      if (!set_full) {
        total_broken++;
        if (first_mismatch_report.empty()) {
          char buf[512];
          std::snprintf(buf, sizeof(buf),
                        "  pyr a=(%.1f,%.1f) h=(%.2f,%.2f,%.2f) prod=%d cf=%d seq=%d "
                        "set=%d prod_only=%d cf_only=%d\n",
                        static_cast<double>(c.alpha_upper), static_cast<double>(c.alpha_lower),
                        static_cast<double>(c.h1), static_cast<double>(c.h2), static_cast<double>(c.h3),
                        stats.total_prod, stats.total_cf, stats.seq_match, stats.unordered_match, stats.prod_only,
                        stats.cf_only);
          first_mismatch_report = buf;
        }
      }
    }
  }
  std::printf("\n=== Pyramid triangulation-swap spike ===\n");
  std::printf("samples=%d  seq_full=%d  set_full=%d  set_broken=%d\n", total_samples, total_seq_full, total_set_full,
              total_broken);
  std::printf(
      "total_prod_tris=%lld  total_cf_tris=%lld  seq_hits=%lld  "
      "cyclic_hits=%lld  set_hits=%lld\n",
      total_tris_prod, total_tris_cf, total_seq_hits, total_cyclic_hits, total_set_hits);
  if (!first_mismatch_report.empty()) {
    std::printf("first set-mismatch:\n%s", first_mismatch_report.c_str());
  }
}

// ---- Diagnostic Experiment B ------------------------------------------------
// For a single regular prism, dump production and natural-closed-form triangle
// lists side by side so a human can eyeball whether they differ only by
// fan-start choice (category 2) or by tessellation (category 3).

TEST(TriangulationSwapSpike, PrismSingleSampleDump) {
  const float h = 1.0f;
  const float dist[6] = { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
  float coef[kMaxHexCrystalPlanes * 4];
  auto cnt = FillHexCrystalCoef(0, 0, 0, h, 0, dist, coef);
  Mesh prod_mesh = CreateConvexPolyhedronMesh(static_cast<int>(cnt), coef);
  auto prod = ExtractTriangles(prod_mesh);
  auto cf = ClosedFormPrismTriangulation(h, dist);

  std::printf("\n=== Regular prism h=1.0, dist=[1..1] ===\n");
  std::printf("production vtx pool (%zu):\n", const_cast<Mesh&>(prod_mesh).GetVtxCnt());
  auto* pv = const_cast<Mesh&>(prod_mesh).GetVtxPtr(0);
  for (size_t i = 0; i < const_cast<Mesh&>(prod_mesh).GetVtxCnt(); i++) {
    std::printf("  v%02zu = (%+7.4f, %+7.4f, %+7.4f)\n", i, static_cast<double>(pv[i * 3 + 0]),
                static_cast<double>(pv[i * 3 + 1]), static_cast<double>(pv[i * 3 + 2]));
  }
  auto* pt = const_cast<Mesh&>(prod_mesh).GetTrianglePtr(0);
  std::printf("production triangles (%zu):\n", const_cast<Mesh&>(prod_mesh).GetTriangleCnt());
  for (size_t i = 0; i < const_cast<Mesh&>(prod_mesh).GetTriangleCnt(); i++) {
    std::printf("  t%02zu = (%2d, %2d, %2d)\n", i, pt[i * 3 + 0], pt[i * 3 + 1], pt[i * 3 + 2]);
  }
  std::printf("closed-form triangles (natural) — first 6:\n");
  for (size_t i = 0; i < cf.size() && i < 6; i++) {
    std::printf(
        "  t%02zu = (%+7.4f,%+7.4f,%+7.4f) (%+7.4f,%+7.4f,%+7.4f) "
        "(%+7.4f,%+7.4f,%+7.4f)\n",
        i, static_cast<double>(cf[i].v[0][0]), static_cast<double>(cf[i].v[0][1]), static_cast<double>(cf[i].v[0][2]),
        static_cast<double>(cf[i].v[1][0]), static_cast<double>(cf[i].v[1][1]), static_cast<double>(cf[i].v[1][2]),
        static_cast<double>(cf[i].v[2][0]), static_cast<double>(cf[i].v[2][1]), static_cast<double>(cf[i].v[2][2]));
  }
}

}  // namespace
}  // namespace lumice
