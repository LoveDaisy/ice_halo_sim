// FilterSpec::Match micro-benchmark: canonical-form matcher throughput
// for single and complex (OR-of-raypaths) filters under P+D symmetry.

#include <benchmark/benchmark.h>

#include <cstdint>
#include <utility>
#include <vector>

#include "config/filter_config.hpp"
#include "core/crystal.hpp"
#include "core/filter_spec.hpp"
#include "core/math.hpp"
#include "core/raypath.hpp"

using namespace lumice;  // NOLINT(google-build-using-namespace) benchmark code

namespace {

// Build an AxisDistribution with az-uniform, roll=0 (sigma_a=0, D applicable in P+D).
AxisDistribution MakeAzUniformRoll0() {
  AxisDistribution d{};
  d.azimuth_dist.type = DistributionType::kUniform;
  d.azimuth_dist.std = 360.0f;
  d.azimuth_dist.mean = 0.0f;
  d.latitude_dist.type = DistributionType::kNoRandom;
  d.latitude_dist.mean = 90.0f;
  d.roll_dist.type = DistributionType::kNoRandom;
  d.roll_dist.mean = 0.0f;
  d.roll_dist.std = 0.0f;
  return d;
}

// A ray plus its raypath recorder. #247.4 moved RaypathRecorder out of RaySeg into
// RayBuffer's parallel recorders_ array, so the matcher now takes the two separately.
// These recorders stay inline-only (2 hits << RaypathRecorder::kInlineCap), hence
// bare operator<< and a null overflow arena at the Match call sites.
struct BenchRay {
  RaySeg seg;
  RaypathRecorder rec;
};

// Pack raypath ids into a BenchRay.
BenchRay MakeRay(std::initializer_list<IdType> fns) {
  BenchRay r{};
  r.rec.Clear();
  for (auto fn : fns) {
    r.rec << fn;
  }
  r.seg.from_face_ = kInvalidId;
  r.seg.to_face_ = kInvalidId;
  r.seg.w_ = 1.0f;
  return r;
}

// Build a small dataset of 2-segment prism-face rays cycling through all (a,b) with a,b in [3..8], a != b.
// Returns 30 distinct rays — mix of orbit members (accepted by P+D on {3,5}) and non-members.
// CAVEAT (exp #1-3 workload artifact): the 30 rays distribute over only 3 orbits: {3,5}/{3,7} (12),
// {3,6} (6), {3,8} (12) — so OR-filters cycling {3,5}/{3,6}/{3,7}/{3,8} achieve 100% coverage at N=4,
// masking true O(N) cost behind universal early-out. Use MakeNonMatchingRayBatch for worst-case scaling.
std::vector<BenchRay> MakeRayBatch() {
  std::vector<BenchRay> rays;
  rays.reserve(30);
  for (IdType a = 3; a <= 8; a++) {
    for (IdType b = 3; b <= 8; b++) {
      if (a == b) {
        continue;
      }
      rays.push_back(MakeRay({ a, b }));
    }
  }
  return rays;
}

// Build a 30-ray batch where every ray contains a pyramid-upper face (13..17) so all rays
// fall outside prism-only OR-filter accept sets. Worst-case workload for measuring linear-N scaling.
std::vector<BenchRay> MakeNonMatchingRayBatch() {
  std::vector<BenchRay> rays;
  rays.reserve(30);
  for (IdType a = 13; a <= 17; a++) {
    for (IdType b = 3; b <= 8; b++) {
      rays.push_back(MakeRay({ a, b }));
    }
  }
  return rays;
}

// Build a ComplexFilter FilterConfig with N OR-clauses, each single raypath {3, 5+i%4}.
FilterConfig MakeComplexCfg(int N) {
  FilterConfig cfg{};
  cfg.symmetry_ = FilterConfig::kSymP | FilterConfig::kSymD;
  cfg.action_ = FilterConfig::kFilterIn;
  ComplexFilterParam cp{};
  for (int i = 0; i < N; i++) {
    RaypathFilterParam rp{};
    rp.raypath_ = { 3, static_cast<IdType>(5 + (i % 4)) };
    cp.filters_.emplace_back(
        std::vector<std::pair<IdType, SimpleFilterParam>>{ { static_cast<IdType>(i), SimpleFilterParam{ rp } } });
  }
  cfg.param_ = FilterParam{ cp };
  return cfg;
}

}  // namespace


// ------------------------------ C2-native production FilterSpec (RaypathSpec, N=1) ------------------------------
// Per-ray cost = detail::ReduceRecorder(rp_copy) + element-wise compare against canonical.
static void BM_FilterMatch_C2Native_N1_PD(benchmark::State& state) {
  Crystal crystal = Crystal::CreatePrism(1.0f);
  auto axis = MakeAzUniformRoll0();
  FilterConfig spec_cfg{};
  spec_cfg.symmetry_ = FilterConfig::kSymP | FilterConfig::kSymD;
  spec_cfg.action_ = FilterConfig::kFilterIn;
  RaypathFilterParam rp_p{};
  rp_p.raypath_ = { 3, 5 };
  spec_cfg.param_ = SimpleFilterParam{ rp_p };
  auto spec = FilterSpec::Create(spec_cfg, crystal, axis);

  auto rays = MakeRayBatch();
  size_t idx = 0;
  size_t n = rays.size();

  for (auto _ : state) {
    bool r = spec->Match(rays[idx].seg, rays[idx].rec);
    benchmark::DoNotOptimize(r);
    idx++;
    if (idx == n) {
      idx = 0;
    }
  }
  state.SetItemsProcessed(state.iterations());
}
BENCHMARK(BM_FilterMatch_C2Native_N1_PD)->Unit(benchmark::kNanosecond);


// ------------------------------ C2-native ComplexFilter scaling (mixed match/no-match) ------------------------------
static void BM_FilterMatch_C2Native_ComplexN_PD(benchmark::State& state) {
  int N = static_cast<int>(state.range(0));
  Crystal crystal = Crystal::CreatePrism(1.0f);
  auto axis = MakeAzUniformRoll0();
  auto spec = FilterSpec::Create(MakeComplexCfg(N), crystal, axis);

  auto rays = MakeRayBatch();
  size_t idx = 0;
  size_t n = rays.size();

  for (auto _ : state) {
    bool r = spec->Match(rays[idx].seg, rays[idx].rec);
    benchmark::DoNotOptimize(r);
    idx++;
    if (idx == n) {
      idx = 0;
    }
  }
  state.SetItemsProcessed(state.iterations());
}
BENCHMARK(BM_FilterMatch_C2Native_ComplexN_PD)->Arg(1)->Arg(2)->Arg(4)->Arg(8)->Arg(16)->Unit(benchmark::kNanosecond);


// ------------------------------ C2-native ComplexFilter scaling (worst-case no-match) ------------------------------
static void BM_FilterMatch_C2Native_ComplexN_PD_NoMatch(benchmark::State& state) {
  int N = static_cast<int>(state.range(0));
  Crystal crystal = Crystal::CreatePrism(1.0f);
  auto axis = MakeAzUniformRoll0();
  auto spec = FilterSpec::Create(MakeComplexCfg(N), crystal, axis);

  auto rays = MakeNonMatchingRayBatch();

  // Sanity: every ray must miss the filter (no-match workload).
  for (const auto& r : rays) {
    if (spec->Match(r.seg, r.rec)) {
      state.SkipWithError("non-matching ray batch unexpectedly matched FilterSpec");
      return;
    }
  }

  size_t idx = 0;
  size_t n = rays.size();

  for (auto _ : state) {
    bool r = spec->Match(rays[idx].seg, rays[idx].rec);
    benchmark::DoNotOptimize(r);
    idx++;
    if (idx == n) {
      idx = 0;
    }
  }
  state.SetItemsProcessed(state.iterations());
}
BENCHMARK(BM_FilterMatch_C2Native_ComplexN_PD_NoMatch)
    ->Arg(1)
    ->Arg(2)
    ->Arg(4)
    ->Arg(8)
    ->Arg(16)
    ->Unit(benchmark::kNanosecond);
