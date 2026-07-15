// CPU-side parity test for `lm_filter::DeviceFilterCheck` (the CUDA device
// filter matcher in `src/core/shared/filter_shared.h`) against the legacy
// host `FilterSpec::Check`. The shared header compiles into both host C++
// (this test) and CUDA (cuda_trace_backend.cu) via `LM_FN`, so a single
// host-only sweep validates the implementation that the CUDA kernel will run.
//
// AC: scrum-cuda-backend-complete 296.5 task-cuda-filter Step 2 — the review
// pulled this from "可选" to "必须验证" so a filter parity failure on dev49
// can be diagnosed without spinning up an end-to-end Metal/CUDA harness.
//
// Coverage: Raypath / EntryExit / Direction / Crystal / Complex (OR-of-AND of
// Simple sub-specs) — the same five types `BuildDeviceFilterDesc` produces.
// 1M+ raw checks across two AxisDistribution variants and both action modes.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <random>
#include <utility>
#include <variant>
#include <vector>

#include "config/filter_config.hpp"
#include "core/crystal.hpp"
#include "core/def.hpp"
#include "core/device_filter_desc.hpp"
#include "core/filter_spec.hpp"
#include "core/raypath.hpp"
#include "core/shared/filter_shared.h"

namespace lumice {
namespace {

Crystal MakeHexPrism() {
  return Crystal::CreatePrism(1.0f);
}

// d_applicable=true axis (roll fixed, sigma_a non-zero) vs d_applicable=false
// axis (roll uniform, sigma_a meaningless). Same logic the Metal parity
// fixture uses (test_metal_filter_match_parity.mm:MakeDApplicableAxis).
AxisDistribution MakeAxis(bool d_applicable) {
  AxisDistribution d{};
  d.azimuth_dist.type = DistributionType::kUniform;
  d.azimuth_dist.std = 360.0f;
  d.azimuth_dist.mean = 0.0f;
  d.latitude_dist.type = DistributionType::kNoRandom;
  d.latitude_dist.mean = 90.0f;
  if (d_applicable) {
    d.roll_dist.type = DistributionType::kNoRandom;
    d.roll_dist.mean = 30.0f;  // sigma_a=5 per kSigmaARollDeg inverse
    d.roll_dist.std = 0.0f;
  } else {
    d.roll_dist.type = DistributionType::kUniform;
    d.roll_dist.mean = 0.0f;
    d.roll_dist.std = 360.0f;
  }
  return d;
}

FilterConfig MakeComplexConfig(uint8_t symmetry, FilterConfig::Action action,
                               std::vector<std::vector<SimpleFilterParam>> or_clauses) {
  FilterConfig cfg{};
  cfg.symmetry_ = symmetry;
  cfg.action_ = action;
  ComplexFilterParam cp;
  cp.filters_.reserve(or_clauses.size());
  IdType synthetic_id = 0;
  for (auto& clause : or_clauses) {
    std::vector<std::pair<IdType, SimpleFilterParam>> and_terms;
    and_terms.reserve(clause.size());
    for (auto& sp : clause) {
      and_terms.emplace_back(synthetic_id++, std::move(sp));
    }
    cp.filters_.push_back(std::move(and_terms));
  }
  cfg.param_ = std::move(cp);
  return cfg;
}

struct Fixture {
  Crystal crystal;
  AxisDistribution axis;
  std::vector<DeviceFilterDesc> descs;
  std::vector<uint8_t> getfn;
  uint32_t getfn_offsets[2] = { 0u, 0u };
  std::vector<DeviceFilterDesc> complex_subs;
  // task-device-flat-and-terms: parallel flat AND-term counts buffer,
  // indexed via each Complex parent's `and_terms_start`.
  std::vector<uint8_t> and_term_counts;
  std::vector<FilterConfig> filter_configs;
  std::vector<std::unique_ptr<FilterSpec>> host_specs;
};

Fixture BuildFixture(bool d_applicable_axis) {
  Fixture fx;
  fx.crystal = MakeHexPrism();
  fx.axis = MakeAxis(d_applicable_axis);

  auto push = [&](FilterConfig cfg) { fx.filter_configs.push_back(std::move(cfg)); };

  // 0: None
  {
    FilterConfig cfg{};
    cfg.symmetry_ = FilterConfig::kSymNone;
    cfg.action_ = FilterConfig::kFilterIn;
    cfg.param_ = NoneFilterParam{};
    push(cfg);
  }
  // 1: Raypath PBD {3,5}
  {
    FilterConfig cfg{};
    cfg.symmetry_ = static_cast<uint8_t>(FilterConfig::kSymP | FilterConfig::kSymB | FilterConfig::kSymD);
    cfg.action_ = FilterConfig::kFilterIn;
    RaypathFilterParam p;
    p.raypath_ = std::vector<IdType>{ 3, 5 };
    cfg.param_ = p;
    push(cfg);
  }
  // 2: Raypath BD {4,6}, action=Out
  {
    FilterConfig cfg{};
    cfg.symmetry_ = static_cast<uint8_t>(FilterConfig::kSymB | FilterConfig::kSymD);
    cfg.action_ = FilterConfig::kFilterOut;
    RaypathFilterParam p;
    p.raypath_ = std::vector<IdType>{ 4, 6 };
    cfg.param_ = p;
    push(cfg);
  }
  // 3: EntryExit (entry=3, exit=5, min_len=2, max_len=4), P
  {
    FilterConfig cfg{};
    cfg.symmetry_ = FilterConfig::kSymP;
    cfg.action_ = FilterConfig::kFilterIn;
    EntryExitFilterParam p;
    p.entry_ = IdType{ 3 };
    p.exit_ = IdType{ 5 };
    p.min_len_ = 2;
    p.max_len_ = 4;
    cfg.param_ = p;
    push(cfg);
  }
  // 4: EntryExit (entry only)
  {
    FilterConfig cfg{};
    cfg.symmetry_ = FilterConfig::kSymNone;
    cfg.action_ = FilterConfig::kFilterIn;
    EntryExitFilterParam p;
    p.entry_ = IdType{ 3 };
    p.min_len_ = 1;
    cfg.param_ = p;
    push(cfg);
  }
  // 5: Direction (forward cone, cos cutoff)
  {
    FilterConfig cfg{};
    cfg.symmetry_ = FilterConfig::kSymNone;
    cfg.action_ = FilterConfig::kFilterIn;
    DirectionFilterParam p;
    p.lon_ = 0.0f;
    p.lat_ = 0.0f;
    p.radii_ = 30.0f;
    cfg.param_ = p;
    push(cfg);
  }
  // 6: Crystal id=7
  {
    FilterConfig cfg{};
    cfg.symmetry_ = FilterConfig::kSymNone;
    cfg.action_ = FilterConfig::kFilterIn;
    CrystalFilterParam p;
    p.crystal_id_ = 7;
    cfg.param_ = p;
    push(cfg);
  }
  // 7: Complex P/Out: OR over two raypath clauses, each 1 AND-term
  {
    RaypathFilterParam a;
    a.raypath_ = std::vector<IdType>{ 3, 5 };
    RaypathFilterParam b;
    b.raypath_ = std::vector<IdType>{ 4, 6 };
    push(MakeComplexConfig(static_cast<uint8_t>(FilterConfig::kSymP), FilterConfig::kFilterOut,
                           { { SimpleFilterParam{ a } }, { SimpleFilterParam{ b } } }));
  }
  // 8: Complex PBD/In: 1 OR × (raypath ∧ crystal_id)
  {
    RaypathFilterParam a;
    a.raypath_ = std::vector<IdType>{ 3, 5 };
    CrystalFilterParam c;
    c.crystal_id_ = 7;
    push(MakeComplexConfig(static_cast<uint8_t>(FilterConfig::kSymP | FilterConfig::kSymB | FilterConfig::kSymD),
                           FilterConfig::kFilterIn, { { SimpleFilterParam{ a }, SimpleFilterParam{ c } } }));
  }
  // 9: task-device-flat-and-terms — big-OR Complex, 20 OR-clauses × 1 AND
  // (raypath), sweeping realistic hex face pairs. Directly exercises the flat
  // `and_term_counts_buf` path (host-inline array is gone) with a clause count
  // well above the legacy 8-cap. AC1's primary evidence: >8-clause Complex on
  // the shared device matcher must match FilterSpec::Check byte-for-byte.
  {
    std::vector<std::vector<SimpleFilterParam>> or_clauses;
    or_clauses.reserve(20);
    static constexpr std::pair<IdType, IdType> kFacePairs[20] = {
      { 3, 5 }, { 3, 6 }, { 3, 7 }, { 3, 8 }, { 4, 5 }, { 4, 6 }, { 4, 7 }, { 4, 8 }, { 5, 3 }, { 5, 4 },
      { 5, 6 }, { 5, 7 }, { 6, 3 }, { 6, 4 }, { 6, 5 }, { 6, 7 }, { 7, 3 }, { 7, 4 }, { 7, 5 }, { 8, 3 },
    };
    for (const auto& fp : kFacePairs) {
      RaypathFilterParam p;
      p.raypath_ = std::vector<IdType>{ fp.first, fp.second };
      or_clauses.push_back({ SimpleFilterParam{ p } });
    }
    push(MakeComplexConfig(static_cast<uint8_t>(FilterConfig::kSymP | FilterConfig::kSymB | FilterConfig::kSymD),
                           FilterConfig::kFilterIn, std::move(or_clauses)));
  }

  fx.descs.reserve(fx.filter_configs.size());
  fx.host_specs.reserve(fx.filter_configs.size());
  for (const auto& cfg : fx.filter_configs) {
    DeviceFilterDesc desc = detail::BuildDeviceFilterDesc(cfg, fx.crystal, fx.axis);
    if (desc.type == kDeviceFilterTypeComplex) {
      const auto* cp = std::get_if<ComplexFilterParam>(&cfg.param_);
      desc.sub_desc_start = static_cast<uint32_t>(fx.complex_subs.size());
      desc.and_terms_start = static_cast<uint32_t>(fx.and_term_counts.size());
      detail::BuildComplexSubDescs(*cp, fx.crystal, desc.symmetry, desc.sigma_a, desc.d_applicable != 0u,
                                   fx.complex_subs, fx.and_term_counts);
    }
    fx.descs.push_back(desc);
    fx.host_specs.push_back(FilterSpec::Create(cfg, fx.crystal, fx.axis));
  }
  fx.getfn = detail::BuildDeviceGetFnBytes(fx.crystal);
  fx.getfn_offsets[0] = 0u;
  fx.getfn_offsets[1] = static_cast<uint32_t>(fx.getfn.size());
  // Pad complex_subs with a dummy entry so the device-side ptr is always
  // valid (mirrors EnsureFilterBuffers' 1-element fallback for layouts with
  // no Complex filter at all — kept defensive here even though our fixture
  // always carries Complex entries).
  if (fx.complex_subs.empty()) {
    fx.complex_subs.push_back(DeviceFilterDesc{});
  }
  if (fx.and_term_counts.empty()) {
    fx.and_term_counts.push_back(0u);
  }
  return fx;
}

// One random ray: raw poly-index path + dir + crystal_config_id + filter
// index. Mirrors the Metal parity fixture's ParityRay so the assertion logic
// stays directly comparable.
struct Ray {
  std::vector<uint8_t> raw_poly;
  std::vector<uint8_t> face_number;
  uint8_t path_len = 0;
  uint16_t crystal_config_id = 0;
  float dir[3] = { 0.0f, 0.0f, 1.0f };
  uint32_t filter_idx = 0;
};

std::vector<Ray> GenerateRays(const Fixture& fx, std::mt19937& rng, size_t n_rays, size_t cap) {
  std::vector<Ray> out;
  out.reserve(n_rays);
  size_t poly_n = fx.crystal.PolygonFaceCount();
  std::uniform_int_distribution<uint32_t> poly_dist(0u, static_cast<uint32_t>(poly_n - 1u));
  std::uniform_int_distribution<uint32_t> len_dist(1u, static_cast<uint32_t>(std::min<size_t>(cap, 5u)));
  std::uniform_int_distribution<uint32_t> filter_dist(0u, static_cast<uint32_t>(fx.descs.size() - 1u));
  std::uniform_int_distribution<uint32_t> cid_dist(0u, 15u);
  std::uniform_real_distribution<float> uni(-1.0f, 1.0f);

  for (size_t i = 0; i < n_rays; ++i) {
    Ray r;
    uint32_t len = len_dist(rng);
    r.path_len = static_cast<uint8_t>(len);
    r.raw_poly.assign(cap, 0);
    r.face_number.assign(cap, 0);
    for (uint32_t k = 0; k < len; ++k) {
      uint32_t pi = poly_dist(rng);
      r.raw_poly[k] = static_cast<uint8_t>(pi);
      r.face_number[k] = fx.getfn[pi];
    }
    r.crystal_config_id = static_cast<uint16_t>(cid_dist(rng));
    float dx = uni(rng), dy = uni(rng), dz = std::abs(uni(rng)) + 0.1f;
    float n = std::sqrt(dx * dx + dy * dy + dz * dz);
    r.dir[0] = dx / n;
    r.dir[1] = dy / n;
    r.dir[2] = dz / n;
    r.filter_idx = filter_dist(rng);
    out.push_back(std::move(r));
  }
  return out;
}

// FilterSpec::Check on (raypath, dir, crystal_config_id). Mirrors
// test_metal_filter_match_parity.mm:HostExpected.
bool HostCheck(const Fixture& fx, const Ray& r) {
  RaySeg ray{};
  ray.d_[0] = r.dir[0];
  ray.d_[1] = r.dir[1];
  ray.d_[2] = r.dir[2];
  ray.crystal_config_id_ = static_cast<IdType>(r.crystal_config_id);
  RaypathRecorder rec{};
  rec.Clear();
  rec.size_ = r.path_len;
  for (uint8_t k = 0; k < r.path_len; ++k) {
    rec.data_[k] = r.face_number[k];
  }
  const FilterSpec* spec = fx.host_specs[r.filter_idx].get();
  return spec->Check(ray, rec, nullptr);
}

bool DeviceCheck(const Fixture& fx, const Ray& r) {
  return lm_filter::DeviceFilterCheck(fx.descs[r.filter_idx], fx.complex_subs.data(), fx.and_term_counts.data(),
                                      r.raw_poly.data(), r.path_len, fx.getfn.data(), fx.getfn_offsets,
                                      /*crystal_slot=*/0u, r.dir, static_cast<uint32_t>(r.crystal_config_id));
}

TEST(DeviceFilterCheckHost, RaypathEntryExitDirCrystalComplex_ParityWithFilterSpec) {
  constexpr size_t kPerFixture = 100'000;
  for (int axis_d_app : { 0, 1 }) {
    Fixture fx = BuildFixture(axis_d_app != 0);
    std::mt19937 rng(0xCAFEBABEu + static_cast<uint32_t>(axis_d_app));
    auto rays = GenerateRays(fx, rng, kPerFixture, /*cap=*/15);
    size_t mismatch = 0;
    for (const auto& r : rays) {
      bool host_b = HostCheck(fx, r);
      bool dev_b = DeviceCheck(fx, r);
      if (host_b != dev_b) {
        ++mismatch;
        if (mismatch <= 5) {
          ADD_FAILURE() << "DeviceFilterCheck mismatch axis_d_app=" << axis_d_app << " filter_idx=" << r.filter_idx
                        << " path_len=" << static_cast<int>(r.path_len) << " host=" << host_b << " dev=" << dev_b;
        }
      }
    }
    EXPECT_EQ(mismatch, 0u) << "axis_d_app=" << axis_d_app << " kPerFixture=" << kPerFixture;
  }
}

}  // namespace
}  // namespace lumice
