// Filter Match benchmark for scrum-filter-architecture-refactor / explore-filter-refactor-bench.
//
// Compares two candidate per-ray paths for filter matching:
//   - Path B: existing RaypathFilter with InitCrystalSymmetry pre-built hash set
//             (caller-side cached form; matches simulator.cpp's symmetry_initialized flag).
//   - Path C2: canonical-form matcher — construct-time ReduceRaypath produces canonical
//              representative; per-ray Match runs ReduceRaypath on the candidate raypath
//              and compares element-wise.
//
// First experiment focuses on N=1 single RaypathFilter to validate H2 (B faster at N=1)
// and establish framework baseline. N=2/4 ComplexFilter and orbit-vs-non-orbit ray mixes
// are deferred to subsequent experiments.

#include <benchmark/benchmark.h>

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "config/filter_config.hpp"
#include "core/crystal.hpp"
#include "core/filter.hpp"
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

// Pack raypath ids into a RaySeg.
RaySeg MakeRay(std::initializer_list<IdType> fns) {
  RaySeg r{};
  r.rp_.Clear();
  for (auto fn : fns) {
    r.rp_ << fn;
  }
  r.from_face_ = kInvalidId;
  r.to_face_ = kInvalidId;
  r.w_ = 1.0f;
  return r;
}

// Build a small dataset of 2-segment prism-face rays cycling through all (a,b) with a,b in [3..8], a != b.
// Returns 30 distinct rays — mix of orbit members (accepted by P+D on {3,5}) and non-members.
std::vector<RaySeg> MakeRayBatch() {
  std::vector<RaySeg> rays;
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

// In-place RaypathRecorder canonicalize for prism (fn_period=6), kSymP|kSymD, sigma_a=0,
// d_applicable=true. Mirrors Crystal::ReduceRaypath logic but operates on the 8-byte fixed
// buffer to model the production-grade C2 path with zero heap allocation. Caveat: kSymB and
// pyramid (fn_period=3) variants are intentionally omitted — extend when those test points
// are added to the explore matrix.
//
// Correctness is verified at benchmark startup by comparing against Crystal::ReduceRaypath
// on the seeded ray batch (see C2NativeMatcher::SelfCheck).
static void ReduceRecorder_PrismPD_SigmaA0(RaypathRecorder& rp) {
  // P canonical shift: rotate prism-face primary indices so the first prism face becomes pri=0.
  IdType first_pri = kInvalidId;
  for (size_t i = 0; i < rp.size_; i++) {
    uint8_t x = rp.recorder_[i];
    if (x < 3) {
      continue;
    }
    uint8_t pyr = x / 10;
    uint8_t pri = x % 10;
    if (first_pri == kInvalidId) {
      first_pri = pri;
    }
    pri = static_cast<uint8_t>((pri + 6 - first_pri) % 6) + 3;
    rp.recorder_[i] = static_cast<uint8_t>(pyr * 10) + pri;
  }
  // D reflect (sigma_a=0): for each prism face, new_pri = (0 - (pri-3) + 6) % 6 + 3.
  uint8_t reflected[kMaxHits];
  for (size_t i = 0; i < rp.size_; i++) {
    uint8_t x = rp.recorder_[i];
    if (x < 3) {
      reflected[i] = x;
      continue;
    }
    uint8_t pyr = x / 10;
    uint8_t pri = static_cast<uint8_t>((x % 10) - 3);
    pri = static_cast<uint8_t>((0u - pri + 6u) % 6u);
    reflected[i] = static_cast<uint8_t>(pyr * 10) + pri + 3;
  }
  // Re-P-canonicalize reflected (D image may no longer be P-canonical).
  IdType r_first_pri = kInvalidId;
  for (size_t i = 0; i < rp.size_; i++) {
    uint8_t x = reflected[i];
    if (x < 3) {
      continue;
    }
    uint8_t pyr = x / 10;
    uint8_t pri = x % 10;
    if (r_first_pri == kInvalidId) {
      r_first_pri = pri;
    }
    pri = static_cast<uint8_t>((pri + 6 - r_first_pri) % 6) + 3;
    reflected[i] = static_cast<uint8_t>(pyr * 10) + pri;
  }
  // Lex pick smaller: if reflected < rp, copy back.
  int cmp = 0;
  for (size_t i = 0; i < rp.size_; i++) {
    if (reflected[i] != rp.recorder_[i]) {
      cmp = reflected[i] < rp.recorder_[i] ? -1 : 1;
      break;
    }
  }
  if (cmp < 0) {
    for (size_t i = 0; i < rp.size_; i++) {
      rp.recorder_[i] = reflected[i];
    }
  }
}

// C2 prototype matcher: pre-compute canonical at construction, per-ray canonicalize the candidate.
// Honest stand-in for the C2 path being evaluated by scrum-210; intentionally uses the same
// Crystal::ReduceRaypath that the production C2 would consume. The std::vector conversion is the
// expected per-ray cost — a real C2 implementation may further inline the canonicalize into a
// RaypathRecorder-native form to shave allocation overhead.
class C2Matcher {
 public:
  C2Matcher(const Crystal& crystal, const std::vector<IdType>& rp, uint8_t symmetry, int sigma_a, bool d_applicable)
      : crystal_(&crystal),
        symmetry_(symmetry),
        sigma_a_(sigma_a),
        d_applicable_(d_applicable),
        canonical_(crystal.ReduceRaypath(rp, symmetry, sigma_a, d_applicable)) {
    scratch_.reserve(8);
  }

  bool Match(const RaySeg& ray) const {
    scratch_.clear();
    for (auto fn : ray.rp_) {
      scratch_.push_back(fn);
    }
    auto reduced = crystal_->ReduceRaypath(scratch_, symmetry_, sigma_a_, d_applicable_);
    return reduced == canonical_;
  }

 private:
  const Crystal* crystal_;
  uint8_t symmetry_;
  int sigma_a_;
  bool d_applicable_;
  std::vector<IdType> canonical_;
  mutable std::vector<IdType> scratch_;
};

// C2 native matcher: stack canonical, no heap. Operates on RaypathRecorder buffer directly.
class C2NativeMatcher {
 public:
  C2NativeMatcher(const Crystal& crystal, const std::vector<IdType>& rp, uint8_t symmetry, int sigma_a,
                  bool d_applicable) {
    auto canonical_vec = crystal.ReduceRaypath(rp, symmetry, sigma_a, d_applicable);
    canonical_.Clear();
    for (auto fn : canonical_vec) {
      canonical_ << fn;
    }
  }

  bool Match(const RaySeg& ray) const {
    RaypathRecorder rp = ray.rp_;
    ReduceRecorder_PrismPD_SigmaA0(rp);
    if (rp.size_ != canonical_.size_) {
      return false;
    }
    for (size_t i = 0; i < rp.size_; i++) {
      if (rp.recorder_[i] != canonical_.recorder_[i]) {
        return false;
      }
    }
    return true;
  }

 private:
  RaypathRecorder canonical_;
};

// Build a RaypathFilter wired via Filter::Create (matches production path) with N=1 raypath={3,5}.
FilterPtrU MakeFilterB_N1_PD() {
  FilterConfig cfg{};
  cfg.symmetry_ = FilterConfig::kSymP | FilterConfig::kSymD;
  cfg.action_ = FilterConfig::kFilterIn;
  RaypathFilterParam p{};
  p.raypath_ = { 3, 5 };
  cfg.param_ = SimpleFilterParam{ p };
  return Filter::Create(cfg);
}

}  // namespace


// ------------------------------ Path B (caller-cached) ------------------------------
// Per-ray cost = unordered_set<size_t>::count(RaypathHash(rp)).
static void BM_FilterMatch_B_N1_PD(benchmark::State& state) {
  Crystal crystal = Crystal::CreatePrism(1.0f);
  auto axis = MakeAzUniformRoll0();
  auto filter = MakeFilterB_N1_PD();
  filter->InitCrystalSymmetry(crystal, FilterConfig::kSymP | FilterConfig::kSymD, axis);

  auto rays = MakeRayBatch();
  size_t idx = 0;
  size_t n = rays.size();

  for (auto _ : state) {
    bool r = filter->Check(rays[idx]);
    benchmark::DoNotOptimize(r);
    idx++;
    if (idx == n) {
      idx = 0;
    }
  }
  state.SetItemsProcessed(state.iterations());
}
BENCHMARK(BM_FilterMatch_B_N1_PD)->Unit(benchmark::kNanosecond);


// ------------------------------ Path C2 (per-ray canonical) ------------------------------
// Per-ray cost = ReduceRaypath(rp.to_vector(), sym, sigma_a, d_applicable) == canonical.
static void BM_FilterMatch_C2_N1_PD(benchmark::State& state) {
  Crystal crystal = Crystal::CreatePrism(1.0f);
  const uint8_t sym = FilterConfig::kSymP | FilterConfig::kSymD;
  const int sigma_a = 0;  // roll_mean=0 → sigma_a=0 (matches MakeAzUniformRoll0)
  const bool d_applicable = true;
  C2Matcher matcher(crystal, { 3, 5 }, sym, sigma_a, d_applicable);

  auto rays = MakeRayBatch();
  size_t idx = 0;
  size_t n = rays.size();

  for (auto _ : state) {
    bool r = matcher.Match(rays[idx]);
    benchmark::DoNotOptimize(r);
    idx++;
    if (idx == n) {
      idx = 0;
    }
  }
  state.SetItemsProcessed(state.iterations());
}
BENCHMARK(BM_FilterMatch_C2_N1_PD)->Unit(benchmark::kNanosecond);


// ------------------------------ Path C2-native (in-place RaypathRecorder canonical) ------------------------------
// Per-ray cost = ReduceRecorder_PrismPD_SigmaA0(rp_copy) + element-wise compare against canonical.
// Zero heap allocation; modeled to estimate the floor of a production-grade C2 implementation.
//
// Self-check: at benchmark setup, runs every ray through both C2NativeMatcher and the existing
// RaypathFilter (Path B), asserting answer parity. Aborts the benchmark on disagreement so a
// canonicalization bug cannot silently produce misleading nanosecond numbers.
static void BM_FilterMatch_C2Native_N1_PD(benchmark::State& state) {
  Crystal crystal = Crystal::CreatePrism(1.0f);
  auto axis = MakeAzUniformRoll0();
  const uint8_t sym = FilterConfig::kSymP | FilterConfig::kSymD;
  const int sigma_a = 0;
  const bool d_applicable = true;
  C2NativeMatcher matcher(crystal, { 3, 5 }, sym, sigma_a, d_applicable);

  auto rays = MakeRayBatch();

  // Parity self-check against the existing RaypathFilter (Path B) to guard against silent
  // canonicalization bugs that would make C2-native look artificially fast.
  {
    auto ref_filter = MakeFilterB_N1_PD();
    ref_filter->InitCrystalSymmetry(crystal, sym, axis);
    for (const auto& r : rays) {
      if (matcher.Match(r) != ref_filter->Check(r)) {
        state.SkipWithError("C2NativeMatcher answer disagrees with Path B reference");
        return;
      }
    }
  }

  size_t idx = 0;
  size_t n = rays.size();

  for (auto _ : state) {
    bool r = matcher.Match(rays[idx]);
    benchmark::DoNotOptimize(r);
    idx++;
    if (idx == n) {
      idx = 0;
    }
  }
  state.SetItemsProcessed(state.iterations());
}
BENCHMARK(BM_FilterMatch_C2Native_N1_PD)->Unit(benchmark::kNanosecond);
