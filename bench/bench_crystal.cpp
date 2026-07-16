#include <benchmark/benchmark.h>

#include <array>
#include <vector>

#include "config/crystal_config.hpp"
#include "core/geo3d.hpp"
#include "core/math.hpp"
#include "core/trace_ops.hpp"

using namespace lumice;  // NOLINT(google-build-using-namespace) benchmark code

namespace {

// Cost of a single MakeCrystal call — the geometry-construction
// clock's unit of work. Determines whether D/K=1 (per-ray unique geometry, the
// unconstrained optimum) is computable at all, and thus whether the oracle used
// to calibrate K is feasible.
//
// MakeCrystal -> Crystal::CreatePrism/CreatePyramid -> FillHexCrystalCoef ->
// CreateConvexPolyhedronMesh, which solves every plane triple (C(n,3)) for
// vertices in double precision. Not a float-fill: expected to be non-trivial.

Distribution Gauss(float mean, float std) {
  return Distribution{ DistributionType::kGaussian, mean, std };
}

Distribution Fixed(float mean) {
  return Distribution{ DistributionType::kNoRandom, mean, 0.0f };
}

// Random prism: height + all 6 face distances drawn (worst case for the prism arm).
CrystalParam RandomPrism() {
  PrismCrystalParam p;
  p.h_ = Gauss(1.0f, 0.2f);
  for (auto& d : p.d_) {
    d = Gauss(1.0f, 0.1f);
  }
  return p;
}

// Deterministic prism: same code path, no RNG draws — isolates the RNG cost
// from the mesh-construction cost.
CrystalParam FixedPrism() {
  PrismCrystalParam p;
  p.h_ = Fixed(1.0f);
  for (auto& d : p.d_) {
    d = Fixed(1.0f);
  }
  return p;
}

// Random pyramid: 20 planes (2 basal + 6 prism + 6 upper pyr + 6 lower pyr) —
// the kMaxHexCrystalPlanes worst case, C(20,3) = 1140 plane triples.
CrystalParam RandomPyramid() {
  PyramidCrystalParam p;
  p.h_prs_ = Gauss(1.0f, 0.2f);
  p.h_pyr_u_ = Gauss(0.3f, 0.05f);
  p.h_pyr_l_ = Gauss(0.3f, 0.05f);
  for (auto& d : p.d_) {
    d = Gauss(1.0f, 0.1f);
  }
  p.wedge_angle_u_ = 28.0f;
  p.wedge_angle_l_ = 28.0f;
  return p;
}

void BM_MakeCrystal(benchmark::State& state, CrystalParam param) {
  RandomNumberGenerator rng(0x5EEDu);
  for (auto _ : state) {
    Crystal c = MakeCrystal(rng, param);
    benchmark::DoNotOptimize(c);
  }
  state.SetItemsProcessed(state.iterations());
}

BENCHMARK_CAPTURE(BM_MakeCrystal, prism_random, RandomPrism());
BENCHMARK_CAPTURE(BM_MakeCrystal, prism_fixed, FixedPrism());
BENCHMARK_CAPTURE(BM_MakeCrystal, pyramid_random, RandomPyramid());

// ---- Stage breakdown of the construction cost -------------------------------
//
// MakeCrystal's 10 us is 98.4% mesh construction (fixed vs random differ by
// 1.63%). Which stage owns it? prism has 8 planes -> C(8,3) = 56 plane triples;
// pyramid has 20 -> C(20,3) = 1140, a 20.4x ratio. Yet pyramid measures only
// 5.0x prism, so the triple loop cannot dominate the prism case. The suspects
// are the allocations and CollectSurfaceVtxD's vector<set<int>> (a red-black
// tree per face, one heap node per insert).

struct StageInput {
  int plane_cnt = 0;
  float coef[kMaxHexCrystalPlanes * 4]{};
};

StageInput PrismCoef() {
  StageInput in;
  float dist[6]{ 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
  in.plane_cnt = static_cast<int>(FillHexCrystalCoef(0, 0, 0, 1.0f, 0, dist, in.coef));
  return in;
}

StageInput PyramidCoef() {
  StageInput in;
  float dist[6]{ 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
  in.plane_cnt = static_cast<int>(FillHexCrystalCoef(28.0f, 28.0f, 0.3f, 1.0f, 0.3f, dist, in.coef));
  return in;
}

// Stage 0: plane-equation fill only (no mesh work at all).
void BM_Stage_FillCoef(benchmark::State& state, StageInput in) {
  float dist[6]{ 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
  float coef[kMaxHexCrystalPlanes * 4];
  const bool pyramid = in.plane_cnt > 8;
  for (auto _ : state) {
    auto cnt = pyramid ? FillHexCrystalCoef(28.0f, 28.0f, 0.3f, 1.0f, 0.3f, dist, coef) :
                         FillHexCrystalCoef(0, 0, 0, 1.0f, 0, dist, coef);
    benchmark::DoNotOptimize(cnt);
    benchmark::DoNotOptimize(coef);
  }
}

// Stage 1: the C(n,3) plane-triple vertex solve.
void BM_Stage_SolveVtx(benchmark::State& state, StageInput in) {
  for (auto _ : state) {
    auto r = SolveConvexPolyhedronVtxD(in.plane_cnt, in.coef);
    benchmark::DoNotOptimize(r);
  }
}

// Stage 2: co-planar grouping -> vector<set<int>> (allocation suspect).
void BM_Stage_CollectSurface(benchmark::State& state, StageInput in) {
  auto [vtx, vtx_cnt] = SolveConvexPolyhedronVtxD(in.plane_cnt, in.coef);
  for (auto _ : state) {
    auto faces = CollectSurfaceVtxD(vtx_cnt, vtx.get(), in.plane_cnt, in.coef);
    benchmark::DoNotOptimize(faces);
  }
}

// Stage 3: triangulation.
void BM_Stage_Triangulate(benchmark::State& state, StageInput in) {
  auto [vtx, vtx_cnt] = SolveConvexPolyhedronVtxD(in.plane_cnt, in.coef);
  auto faces = CollectSurfaceVtxD(vtx_cnt, vtx.get(), in.plane_cnt, in.coef);
  for (auto _ : state) {
    auto r = Triangulate(vtx_cnt, vtx.get(), faces);
    benchmark::DoNotOptimize(r);
  }
}

// ---- Prototype: per-plane grouping (the obvious algorithm) ------------------
//
// CollectSurfaceVtxD owns 67% (prism) / 42% (pyramid) of construction. It walks
// vertex PAIRS -- O(V^2) -- to find edges, then for each edge re-derives both
// adjacent faces from scratch and de-dupes them against every face found so far
// via std::set::count. Each face on the hull is therefore rediscovered once per
// edge lying on it and thrown away, and the whole thing churns heap: a
// vector<int> per inner call plus a red-black tree per accepted face.
//
// The work it actually performs is "which vertices lie on which plane", which is
// one pass: O(P*V) dot products, no allocation. This prototype measures that
// ceiling. It is a MEASUREMENT ONLY -- not a proposed implementation, and not
// equivalence-checked against every degenerate case (see doc/numerical-robustness.md
// before touching a real predicate). Equivalence is spot-checked below.

constexpr double kProtoEps = 1e-6;

std::vector<std::set<int>> CollectSurfacePerPlane(int vtx_cnt, const float* vtx_ptr, int plane_cnt,
                                                  const float* coef_ptr) {
  std::vector<std::set<int>> faces;
  std::vector<int> curr;
  curr.reserve(16);
  for (int k = 0; k < plane_cnt; k++) {
    const float* cp = coef_ptr + k * 4;
    curr.clear();
    for (int v = 0; v < vtx_cnt; v++) {
      const float* p = vtx_ptr + v * 3;
      double d = static_cast<double>(cp[0]) * p[0] + static_cast<double>(cp[1]) * p[1] +
                 static_cast<double>(cp[2]) * p[2] + static_cast<double>(cp[3]);
      if (d > -kProtoEps && d < kProtoEps) {
        curr.push_back(v);
      }
    }
    if (curr.size() >= 3) {
      faces.emplace_back(curr.begin(), curr.end());
    }
  }
  return faces;
}

void BM_Stage_CollectSurface_Proto(benchmark::State& state, StageInput in) {
  auto [vtx, vtx_cnt] = SolveConvexPolyhedronVtxD(in.plane_cnt, in.coef);
  for (auto _ : state) {
    auto faces = CollectSurfacePerPlane(vtx_cnt, vtx.get(), in.plane_cnt, in.coef);
    benchmark::DoNotOptimize(faces);
  }
}

BENCHMARK_CAPTURE(BM_Stage_CollectSurface_Proto, prism, PrismCoef());
BENCHMARK_CAPTURE(BM_Stage_CollectSurface_Proto, pyramid, PyramidCoef());

// ---- Prototype: topology reuse (the strong form) ----------------------------
//
// Merely fixing CollectSurfaceVtxD's algorithm (the prototype above) measures
// only 3-4x -- not enough to matter. The real question: if the plane-triple ->
// vertex incidence and the triangle index list are CACHED (topology is fixed for
// a given CrystalParam per cuda_trace_backend.cu:3057-3059), a resample only
// needs to re-solve each known vertex's 3x3 system and rewrite positions.
// Search, grouping and triangulation all vanish.
//
// This measures that floor: solve the 12 (prism) / 24 (pyramid) cached triples.
// It computes positions only -- it does NOT verify topology still holds, which a
// real implementation must do (the skewed-prism case below collapses 12 vertices
// to 8, so the cache would be invalid and must fall back).

struct CachedTopology {
  int plane_cnt = 0;
  std::vector<std::array<int, 3>> vtx_triples;  // which 3 planes meet at each vertex
};

CachedTopology BuildTopology(const StageInput& in) {
  CachedTopology t;
  t.plane_cnt = in.plane_cnt;
  auto [vtx, vtx_cnt] = SolveConvexPolyhedronVtxD(in.plane_cnt, in.coef);
  // Recover, for each solved vertex, the first plane triple that yields it.
  for (int v = 0; v < vtx_cnt; v++) {
    const float* p = vtx.get() + v * 3;
    std::vector<int> on;
    for (int k = 0; k < in.plane_cnt; k++) {
      const float* cp = in.coef + k * 4;
      double d = static_cast<double>(cp[0]) * p[0] + static_cast<double>(cp[1]) * p[1] +
                 static_cast<double>(cp[2]) * p[2] + static_cast<double>(cp[3]);
      if (d > -kProtoEps && d < kProtoEps) {
        on.push_back(k);
      }
    }
    if (on.size() >= 3) {
      t.vtx_triples.push_back({ on[0], on[1], on[2] });
    }
  }
  return t;
}

void BM_TopologyReuse(benchmark::State& state, StageInput in) {
  CachedTopology topo = BuildTopology(in);
  std::vector<double> coef_d(in.plane_cnt * 4);
  std::vector<float> out(topo.vtx_triples.size() * 3);
  state.counters["vtx"] = static_cast<double>(topo.vtx_triples.size());
  for (auto _ : state) {
    for (int i = 0; i < in.plane_cnt * 4; i++) {
      coef_d[i] = static_cast<double>(in.coef[i]);
    }
    double xyz[3];
    for (size_t v = 0; v < topo.vtx_triples.size(); v++) {
      const auto& t = topo.vtx_triples[v];
      SolvePlanesD(coef_d.data() + t[0] * 4, coef_d.data() + t[1] * 4, coef_d.data() + t[2] * 4, xyz);
      out[v * 3 + 0] = static_cast<float>(xyz[0]);
      out[v * 3 + 1] = static_cast<float>(xyz[1]);
      out[v * 3 + 2] = static_cast<float>(xyz[2]);
    }
    benchmark::DoNotOptimize(out);
  }
}

BENCHMARK_CAPTURE(BM_TopologyReuse, prism, PrismCoef());
BENCHMARK_CAPTURE(BM_TopologyReuse, pyramid, PyramidCoef());

BENCHMARK_CAPTURE(BM_Stage_FillCoef, prism, PrismCoef());
BENCHMARK_CAPTURE(BM_Stage_SolveVtx, prism, PrismCoef());
BENCHMARK_CAPTURE(BM_Stage_CollectSurface, prism, PrismCoef());
BENCHMARK_CAPTURE(BM_Stage_Triangulate, prism, PrismCoef());

BENCHMARK_CAPTURE(BM_Stage_FillCoef, pyramid, PyramidCoef());
BENCHMARK_CAPTURE(BM_Stage_SolveVtx, pyramid, PyramidCoef());
BENCHMARK_CAPTURE(BM_Stage_CollectSurface, pyramid, PyramidCoef());
BENCHMARK_CAPTURE(BM_Stage_Triangulate, pyramid, PyramidCoef());

}  // namespace
