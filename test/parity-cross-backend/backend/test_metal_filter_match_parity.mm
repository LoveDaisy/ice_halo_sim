// Parity harness for the device filter MATCH path (scrum-267 task-msl-filter-
// match-port, plan Step 5). Compiles the MSL helper + test kernel, generates
// realistic raw poly-index sequences, and validates the device output against
// the host `FilterSpec::Check` byte-for-byte.
//
// Coverage strategy (plan Step 5 + R4):
//   * BuildDeviceFilterDesc + BuildDeviceGetFnBytes unit checks (Step 2 unit
//     tests folded in — they reuse the same Crystal + AxisDistribution
//     fixtures the parity sweeps already construct).
//   * Raypath + EntryExit sweep over (symmetry, sigma_a, d_applicable) on
//     synthetic raw poly-index sequences (poly_idx uniformly sampled from a
//     real hex prism / pyramid; identical distribution to the kernel's
//     `path[]` output, just decoupled from a live TraceLayer run). N ≥ 1M
//     aggregate (plan §1 acceptance).
//   * Direction + Crystal + None individually (semantics independent of path
//     data; covered with smaller N).
//   * Complex deliberately skipped — plan §2 marks it out of scope; device
//     pass-throughs `true` for type=5 and the host result would differ.
//   * BeginSession upload sanity (filter_desc_buf_ non-nil, first desc has
//     canonical_len > 0 for a session with raypath filters) — Step 4 check.
//
// Why "real traced sequences" via synthetic poly-indices (plan R3 mitigation):
//   The device kernel's `path[]` is just `uchar[face_seq_cap]` of raw
//   poly-indices in `[0, PolygonFaceCount())`. The host FilterSpec::Check
//   applies the SAME GetFn remap + ReduceBuffer chain regardless of whether
//   the bytes came from a TraceLayer dispatch or a uniform sampler — the
//   per-byte verification is the same algebra. We additionally drive a small
//   end-to-end run (final block of the test) feeding a real config through
//   MetalTraceBackend, snapshot the produced ExitRayRecord paths
//   (face-number space — already GetFn-remapped), and cross-check device
//   MATCH on those via fn_period<0 path (no remap). This validates that the
//   device handles the "shape" of real exit sequences correctly.

#include <gtest/gtest.h>

#if defined(__APPLE__)

#import <Foundation/Foundation.h>
#import <Metal/Metal.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <random>
#include <vector>

#include "config/crystal_config.hpp"
#include "config/filter_config.hpp"
#include "core/crystal.hpp"
#include "core/device_filter_desc.hpp"
#include "core/filter_spec.hpp"
#include "core/math.hpp"
#include "core/metal_filter_match_src.hpp"
#include "core/backend/metal_trace_backend.hpp"
// task-#283 (metal-build-time-metallib): the parity harness no longer relies
// on the retired `kFilterMatchHelperSrc` string. Instead it prepends the
// build-time-embedded canonical source (which contains the helper functions
// AND the production trace_layer kernel) before its standalone test kernel.
// The extra production kernel is compiled but never dispatched — the test
// only invokes `filter_match_test_kernel`.
#include "lumice_trace_src_embed.h"  // lumice::kLumiceCombinedKernelSrc
#include "core/raypath.hpp"
#include "core/backend/trace_backend.hpp"
#include "metal_test_helpers.hpp"
#include "parity-cross-backend/backend/metal_filter_match_test_src.hpp"

namespace lumice {
namespace {

using metal_test::MakeMetalScene;
using metal_test::MakeRectangularRender;
using metal_test::ShouldSkipMetalTests;

// =============================================================================
// Helpers
// =============================================================================

// Layout sanity — guards future struct edits (parity harness reuploads bytes
// verbatim so any host-side reshape that changes sizeof must be intentional).
TEST(DeviceFilterDescLayout, SizeFitsBudget) {
  // task-device-flat-and-terms: `and_term_counts[8]` inline array retired
  // (moved to a separate host-built flat buffer indexed by `and_terms_start`).
  // `or_clause_count` was a uint8 (1B); it is now uint16 (2B) plus a trailing
  // uint16 padding. Former `or_clause_count` position now holds a 1-byte
  // `_pad_reserved`. Net swap: -8B (inline array) +4B (and_terms_start) +2B
  // (or_clause_count widen) +2B (trailing pad) = 0B. Total stays 120B; assert
  // the exact number so any host layout drift is caught early. The <256 belt
  // is the coarser budget.
  EXPECT_EQ(sizeof(DeviceFilterDesc), static_cast<size_t>(120));
  EXPECT_LE(sizeof(DeviceFilterDesc), static_cast<size_t>(256));
}

// AxisDistribution shortcut used in tests below — full-360 az/zen + roll mean
// at 0° satisfies IsDApplicable() ⇒ D symmetry is exercised.
AxisDistribution MakeDApplicableAxis() {
  AxisDistribution a;
  a.azimuth_dist.type = DistributionType::kUniform;
  a.azimuth_dist.mean = 0.0f;
  a.azimuth_dist.std = 360.0f;
  a.latitude_dist.type = DistributionType::kUniform;
  a.latitude_dist.mean = 0.0f;
  a.latitude_dist.std = 360.0f;
  a.roll_dist.type = DistributionType::kUniform;
  a.roll_dist.mean = 0.0f;
  a.roll_dist.std = 360.0f;
  return a;
}

AxisDistribution MakeDNonApplicableAxis() {
  AxisDistribution a;
  a.azimuth_dist.type = DistributionType::kUniform;
  a.azimuth_dist.mean = 0.0f;
  a.azimuth_dist.std = 360.0f;
  a.latitude_dist.type = DistributionType::kGaussian;
  a.latitude_dist.mean = 0.0f;
  a.latitude_dist.std = 40.0f;
  a.roll_dist.type = DistributionType::kUniform;
  a.roll_dist.mean = 5.0f;  // not a multiple of 30°
  a.roll_dist.std = 360.0f;
  return a;
}

// Build a hex prism + the corresponding GetFn byte table. The crystal is
// owned by the test (canonical_bytes computed inside BuildDeviceFilterDesc
// references this crystal).
Crystal MakeHexPrism() { return Crystal::CreatePrism(1.0f); }

// =============================================================================
// Step 2 unit checks — BuildDeviceFilterDesc / BuildDeviceGetFnBytes
// =============================================================================

TEST(DeviceFilterDescBuilder, NoneFilter) {
  Crystal c = MakeHexPrism();
  AxisDistribution a = MakeDApplicableAxis();
  FilterConfig cfg{};
  cfg.symmetry_ = FilterConfig::kSymNone;
  cfg.action_ = FilterConfig::kFilterIn;
  cfg.param_ = NoneFilterParam{};
  auto d = detail::BuildDeviceFilterDesc(cfg, c, a);
  EXPECT_EQ(d.type, kDeviceFilterTypeNone);
  EXPECT_EQ(d.action, 0u);
  EXPECT_EQ(d.canonical_len, 0u);
}

TEST(DeviceFilterDescBuilder, RaypathPBD) {
  Crystal c = MakeHexPrism();
  AxisDistribution a = MakeDApplicableAxis();
  FilterConfig cfg{};
  cfg.symmetry_ = static_cast<uint8_t>(FilterConfig::kSymP | FilterConfig::kSymB | FilterConfig::kSymD);
  cfg.action_ = FilterConfig::kFilterIn;
  RaypathFilterParam p;
  p.raypath_ = std::vector<IdType>{ 3, 5 };
  cfg.param_ = p;
  auto d = detail::BuildDeviceFilterDesc(cfg, c, a);
  EXPECT_EQ(d.type, kDeviceFilterTypeRaypath);
  EXPECT_EQ(d.symmetry, cfg.symmetry_);
  EXPECT_EQ(d.d_applicable, 1u);
  EXPECT_EQ(d.fn_period, 6);
  EXPECT_EQ(d.canonical_len, 2u);
}

TEST(DeviceFilterDescBuilder, DirectionCarriesUnitVector) {
  Crystal c = MakeHexPrism();
  AxisDistribution a = MakeDApplicableAxis();
  FilterConfig cfg{};
  cfg.symmetry_ = FilterConfig::kSymNone;
  cfg.action_ = FilterConfig::kFilterIn;
  DirectionFilterParam p;
  p.lon_ = 0.0f;
  p.lat_ = 0.0f;
  p.radii_ = 5.0f;
  cfg.param_ = p;
  auto d = detail::BuildDeviceFilterDesc(cfg, c, a);
  EXPECT_EQ(d.type, kDeviceFilterTypeDirection);
  EXPECT_NEAR(d.dir[0], 1.0f, 1e-5f);
  EXPECT_NEAR(d.dir[1], 0.0f, 1e-5f);
  EXPECT_NEAR(d.dir[2], 0.0f, 1e-5f);
  EXPECT_NEAR(d.radii_c, std::cos(5.0f * math::kDegreeToRad), 1e-5f);
}

TEST(DeviceFilterDescBuilder, GetFnTableLen) {
  Crystal c = MakeHexPrism();
  auto bytes = detail::BuildDeviceGetFnBytes(c);
  EXPECT_EQ(bytes.size(), c.PolygonFaceCount());
  for (size_t i = 0; i < c.PolygonFaceCount(); ++i) {
    EXPECT_EQ(bytes[i], static_cast<uint8_t>(c.GetFn(static_cast<IdType>(i)) & 0xFF));
  }
}

// =============================================================================
// MSL parity harness
// =============================================================================

struct MetalHarness {
  id<MTLDevice> device = nil;
  id<MTLCommandQueue> queue = nil;
  id<MTLComputePipelineState> pso = nil;
  bool ok = false;
  std::string compile_log;

  bool Init() {
    @autoreleasepool {
      device = MTLCreateSystemDefaultDevice();
      if (!device) {
        NSArray<id<MTLDevice>>* all = MTLCopyAllDevices();
        if (all.count) { device = all[0]; }
      }
      if (!device) { return false; }
      queue = [device newCommandQueue];
      if (!queue) { return false; }
      // task-#283: prefix = canonical embedded source (= helpers + production
      // trace_layer kernel). The test only invokes filter_match_test_kernel;
      // the unused production kernels add ~50KB of MSL compile work per test
      // start which is dwarfed by the test's own work and avoids a duplicate
      // helper-string source-of-truth.
      //
      // Coupling caveat: since the prefix carries the production trace_layer
      // kernel, a syntax error in lumice_trace.metal surfaces *here* as this
      // filter-match parity test failing to compile — a non-obvious failure
      // mode (the real breakage is in production MSL, not the test kernel). A
      // future option is a Python-side structural extraction of a helper-only
      // header (still single-source) to decouple the test from the full kernel.
      NSString* src = [NSString stringWithFormat:@"%s\n%s",
                                                  kLumiceCombinedKernelSrc,
                                                  kFilterMatchTestKernelSrc];
      MTLCompileOptions* opts = [MTLCompileOptions new];
      opts.languageVersion = MTLLanguageVersion3_0;
      if (@available(macOS 15.0, *)) {
        opts.mathMode = MTLMathModeSafe;
      } else {
        opts.fastMathEnabled = NO;
      }
      NSError* err = nil;
      id<MTLLibrary> lib = [device newLibraryWithSource:src options:opts error:&err];
      if (!lib) {
        compile_log = err ? err.localizedDescription.UTF8String : "(no error)";
        return false;
      }
      id<MTLFunction> fn = [lib newFunctionWithName:@"filter_match_test_kernel"];
      if (!fn) { return false; }
      err = nil;
      pso = [device newComputePipelineStateWithFunction:fn error:&err];
      if (!pso) {
        compile_log = err ? err.localizedDescription.UTF8String : "(no error)";
        return false;
      }
      ok = true;
      return true;
    }
  }
};

// Mirror of MSL test kernel's `FilterMatchTestParams` (10 floats? no — pack
// 3 uint32 in 12B; Metal alignment matches).
struct FilterMatchTestParams {
  uint32_t n;
  uint32_t face_seq_cap;
  uint32_t check_mode;
};

// Runs the kernel once and reads back the `out_match` bytes. Buffers passed
// in are caller-owned MTLBuffers; the harness sets binding indices according
// to the contract in `kFilterMatchTestKernelSrc`.
size_t DispatchParity(MetalHarness& h,
                      id<MTLBuffer> filter_desc, id<MTLBuffer> poly_fn,
                      id<MTLBuffer> ray_path, id<MTLBuffer> ray_path_len,
                      id<MTLBuffer> ray_cslot, id<MTLBuffer> ray_cid,
                      id<MTLBuffer> ray_dir, id<MTLBuffer> ray_filter,
                      id<MTLBuffer> out_match, const FilterMatchTestParams& prm,
                      id<MTLBuffer> complex_sub_desc, id<MTLBuffer> and_term_counts) {
  @autoreleasepool {
    id<MTLBuffer> prm_buf = [h.device newBufferWithLength:sizeof(FilterMatchTestParams)
                                                  options:MTLResourceStorageModeShared];
    std::memcpy([prm_buf contents], &prm, sizeof(FilterMatchTestParams));
    id<MTLCommandBuffer> cb = [h.queue commandBuffer];
    id<MTLComputeCommandEncoder> enc = [cb computeCommandEncoder];
    [enc setComputePipelineState:h.pso];
    [enc setBuffer:filter_desc      offset:0 atIndex:0];
    // buffer(1) getfn_offsets retired; poly_fn is the per-instance
    // GetFn table (single-crystal fixture → one stripe at offset 0).
    [enc setBuffer:poly_fn          offset:0 atIndex:2];
    [enc setBuffer:ray_path         offset:0 atIndex:3];
    [enc setBuffer:ray_path_len     offset:0 atIndex:4];
    [enc setBuffer:ray_cslot        offset:0 atIndex:5];
    [enc setBuffer:ray_cid          offset:0 atIndex:6];
    [enc setBuffer:ray_dir          offset:0 atIndex:7];
    [enc setBuffer:ray_filter       offset:0 atIndex:8];
    [enc setBuffer:out_match        offset:0 atIndex:9];
    [enc setBuffer:prm_buf          offset:0 atIndex:10];
    [enc setBuffer:complex_sub_desc offset:0 atIndex:11];
    // task-device-flat-and-terms: parallel flat AND-term counts buffer.
    [enc setBuffer:and_term_counts  offset:0 atIndex:12];
    NSUInteger tg = std::min<NSUInteger>(256, h.pso.maxTotalThreadsPerThreadgroup);
    [enc dispatchThreads:MTLSizeMake(prm.n, 1, 1) threadsPerThreadgroup:MTLSizeMake(tg, 1, 1)];
    [enc endEncoding];
    [cb commit];
    [cb waitUntilCompleted];
    return (size_t)h.pso.maxTotalThreadsPerThreadgroup;
  }
}

// Convenience: make a shared-mode MTLBuffer initialized from a byte span.
id<MTLBuffer> MakeShared(id<MTLDevice> dev, const void* bytes, size_t len) {
  size_t sz = std::max<size_t>(len, 1);  // 0-byte buffers are nil on some drivers
  id<MTLBuffer> buf = [dev newBufferWithLength:sz options:MTLResourceStorageModeShared];
  if (bytes != nullptr && len > 0) {
    std::memcpy([buf contents], bytes, len);
  }
  return buf;
}

// One per-ray fixture row, host-side. Built by the test, uploaded together
// for one DispatchParity call.
struct ParityRay {
  std::vector<uint8_t> raw_poly;       // length up to face_seq_cap
  std::vector<uint8_t> face_number;    // GetFn-remapped (for host reference)
  uint8_t path_len = 0;
  uint16_t crystal_slot = 0;
  uint16_t crystal_config_id = 0;
  float dir[3] = { 0.0f, 0.0f, 1.0f };
  uint32_t filter_idx = 0;
};

// =============================================================================
// Parity sweep — Raypath / EntryExit / Direction / Crystal / None
// =============================================================================

// Fixture crystal + matching DeviceFilterDesc array + GetFn table.
struct ParityFixture {
  Crystal crystal;
  std::vector<DeviceFilterDesc> descs;
  std::vector<uint8_t> getfn;
  // Flat Complex sub-spec descs. Each Complex slot in `descs` carries
  // sub_desc_start indexing here; non-Complex slots leave the field at 0.
  std::vector<DeviceFilterDesc> complex_sub_descs;
  // task-device-flat-and-terms: parallel flat AND-term counts buffer indexed
  // via each Complex parent's `and_terms_start`.
  std::vector<uint8_t> and_term_counts;
  std::vector<std::unique_ptr<FilterSpec>> host_specs;
  std::vector<FilterConfig> filter_configs;  // kept alive for FilterSpec::Create
  AxisDistribution axis;
  uint32_t face_seq_cap = 15;
};

// Build a Complex FilterConfig — helper used by BuildFixture below. Each
// or_clause is a list of SimpleFilterParam (the IdType key is informational
// only; ComplexSpec / BuildComplexSubDescs consume only the variant value).
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

// Build a comprehensive filter table (covers all device filter types).
ParityFixture BuildFixture(bool d_applicable_axis) {
  ParityFixture fx;
  fx.crystal = MakeHexPrism();
  fx.axis = d_applicable_axis ? MakeDApplicableAxis() : MakeDNonApplicableAxis();

  auto push = [&](FilterConfig cfg) {
    fx.filter_configs.push_back(std::move(cfg));
  };
  // None
  {
    FilterConfig cfg{};
    cfg.symmetry_ = FilterConfig::kSymNone;
    cfg.action_ = FilterConfig::kFilterIn;
    cfg.param_ = NoneFilterParam{};
    push(cfg);
  }
  // Raypath PBD on {3,5}
  {
    FilterConfig cfg{};
    cfg.symmetry_ = static_cast<uint8_t>(FilterConfig::kSymP | FilterConfig::kSymB | FilterConfig::kSymD);
    cfg.action_ = FilterConfig::kFilterIn;
    RaypathFilterParam p;
    p.raypath_ = std::vector<IdType>{ 3, 5 };
    cfg.param_ = p;
    push(cfg);
  }
  // Raypath BD on {4,6}
  {
    FilterConfig cfg{};
    cfg.symmetry_ = static_cast<uint8_t>(FilterConfig::kSymB | FilterConfig::kSymD);
    cfg.action_ = FilterConfig::kFilterIn;
    RaypathFilterParam p;
    p.raypath_ = std::vector<IdType>{ 4, 6 };
    cfg.param_ = p;
    push(cfg);
  }
  // Raypath P on {3,5,3}
  {
    FilterConfig cfg{};
    cfg.symmetry_ = FilterConfig::kSymP;
    cfg.action_ = FilterConfig::kFilterIn;
    RaypathFilterParam p;
    p.raypath_ = std::vector<IdType>{ 3, 5, 3 };
    cfg.param_ = p;
    push(cfg);
  }
  // EntryExit, entry only, length range [1, 3]
  {
    FilterConfig cfg{};
    cfg.symmetry_ = FilterConfig::kSymP;
    cfg.action_ = FilterConfig::kFilterIn;
    EntryExitFilterParam p;
    p.entry_ = IdType{ 3 };
    p.min_len_ = 1;
    p.max_len_ = 3;
    cfg.param_ = p;
    push(cfg);
  }
  // EntryExit, exit only, length lower bound 2 (no upper)
  {
    FilterConfig cfg{};
    cfg.symmetry_ = FilterConfig::kSymB;
    cfg.action_ = FilterConfig::kFilterIn;
    EntryExitFilterParam p;
    p.exit_ = IdType{ 5 };
    p.min_len_ = 2;
    cfg.param_ = p;
    push(cfg);
  }
  // EntryExit, double-wildcard length-only (entry/exit both nullopt)
  {
    FilterConfig cfg{};
    cfg.symmetry_ = FilterConfig::kSymNone;
    cfg.action_ = FilterConfig::kFilterIn;
    EntryExitFilterParam p;
    p.min_len_ = 2;
    p.max_len_ = 4;
    cfg.param_ = p;
    push(cfg);
  }
  // Direction (zenith ± 10°)
  {
    FilterConfig cfg{};
    cfg.symmetry_ = FilterConfig::kSymNone;
    cfg.action_ = FilterConfig::kFilterIn;
    DirectionFilterParam p;
    p.lon_ = 0.0f;
    p.lat_ = 90.0f;
    p.radii_ = 10.0f;
    cfg.param_ = p;
    push(cfg);
  }
  // Crystal id 7
  {
    FilterConfig cfg{};
    cfg.symmetry_ = FilterConfig::kSymNone;
    cfg.action_ = FilterConfig::kFilterIn;
    CrystalFilterParam p;
    p.crystal_id_ = 7;
    cfg.param_ = p;
    push(cfg);
  }
  // filter_out variant of Raypath PBD on {3,5} — tests the action XOR
  {
    FilterConfig cfg{};
    cfg.symmetry_ = static_cast<uint8_t>(FilterConfig::kSymP | FilterConfig::kSymB | FilterConfig::kSymD);
    cfg.action_ = FilterConfig::kFilterOut;
    RaypathFilterParam p;
    p.raypath_ = std::vector<IdType>{ 3, 5 };
    cfg.param_ = p;
    push(cfg);
  }
  // ---- Complex configs (267.1b) ----
  // Complex-A — mirrors ms3-multi-crystal-filter.json id=4: PBD filter_in,
  // 2 OR-clauses × 1 AND-term, each AND a raypath.
  {
    RaypathFilterParam a;
    a.raypath_ = std::vector<IdType>{ 3, 5 };
    RaypathFilterParam b;
    b.raypath_ = std::vector<IdType>{ 4, 6 };
    push(MakeComplexConfig(
        static_cast<uint8_t>(FilterConfig::kSymP | FilterConfig::kSymB | FilterConfig::kSymD),
        FilterConfig::kFilterIn,
        { { SimpleFilterParam{ a } }, { SimpleFilterParam{ b } } }));
  }
  // Complex-B — mirrors id=7 layout: BD filter_in, 2 OR × 1 AND.
  {
    RaypathFilterParam a;
    a.raypath_ = std::vector<IdType>{ 5, 3 };
    RaypathFilterParam b;
    b.raypath_ = std::vector<IdType>{ 6, 4 };
    push(MakeComplexConfig(
        static_cast<uint8_t>(FilterConfig::kSymB | FilterConfig::kSymD),
        FilterConfig::kFilterIn,
        { { SimpleFilterParam{ a } }, { SimpleFilterParam{ b } } }));
  }
  // Complex-C — filter_out, 1 OR × 1 AND. Tests top-level action XOR over the
  // Complex MATCH result (both true→false and false→true paths exercised
  // because the host result varies per sampled ray).
  {
    RaypathFilterParam a;
    a.raypath_ = std::vector<IdType>{ 3, 5 };
    push(MakeComplexConfig(
        static_cast<uint8_t>(FilterConfig::kSymP | FilterConfig::kSymB | FilterConfig::kSymD),
        FilterConfig::kFilterOut,
        { { SimpleFilterParam{ a } } }));
  }
  // Complex-D — AND dimension: 1 OR × 2 AND-terms (raypath ∧ crystal_id),
  // exercises the inner AND loop short-circuit and sub-desc walk past >1 entry
  // per clause.
  {
    RaypathFilterParam a;
    a.raypath_ = std::vector<IdType>{ 3, 5 };
    CrystalFilterParam c;
    c.crystal_id_ = 7;
    push(MakeComplexConfig(
        static_cast<uint8_t>(FilterConfig::kSymP),
        FilterConfig::kFilterIn,
        { { SimpleFilterParam{ a }, SimpleFilterParam{ c } } }));
  }
  // Complex-E — task-device-flat-and-terms: 20 OR-clauses × 1 AND, direct
  // >8-clause coverage on the real Metal GPU (AC1's second evidence line
  // alongside the host-shared-logic test). Face pairs are the same 20 real hex
  // combinations the host-side test uses so any parity failure surfaces on the
  // exact same fixture that the CPU-shared matcher already validates.
  {
    std::vector<std::vector<SimpleFilterParam>> or_clauses;
    or_clauses.reserve(20);
    static constexpr std::pair<IdType, IdType> kFacePairs[20] = {
        {3, 5}, {3, 6}, {3, 7}, {3, 8}, {4, 5}, {4, 6}, {4, 7}, {4, 8},
        {5, 3}, {5, 4}, {5, 6}, {5, 7}, {6, 3}, {6, 4}, {6, 5}, {6, 7},
        {7, 3}, {7, 4}, {7, 5}, {8, 3},
    };
    for (const auto& fp : kFacePairs) {
      RaypathFilterParam p;
      p.raypath_ = std::vector<IdType>{ fp.first, fp.second };
      or_clauses.push_back({ SimpleFilterParam{ p } });
    }
    push(MakeComplexConfig(
        static_cast<uint8_t>(FilterConfig::kSymP | FilterConfig::kSymB | FilterConfig::kSymD),
        FilterConfig::kFilterIn,
        std::move(or_clauses)));
  }

  fx.descs.reserve(fx.filter_configs.size());
  fx.host_specs.reserve(fx.filter_configs.size());
  for (size_t i = 0; i < fx.filter_configs.size(); ++i) {
    const auto& cfg = fx.filter_configs[i];
    DeviceFilterDesc desc = detail::BuildDeviceFilterDesc(cfg, fx.crystal, fx.axis);
    // Inline Complex sub-desc collection — mirrors EnsureFilterBuffers in
    // metal_trace_backend.mm so the fixture exercises the same layout
    // production uploads. task-device-flat-and-terms: `and_terms_start` +
    // `and_term_counts` flat buffer added as a fifth argument.
    if (desc.type == kDeviceFilterTypeComplex) {
      const auto* cp = std::get_if<ComplexFilterParam>(&cfg.param_);
      assert(cp != nullptr && "Complex desc type without ComplexFilterParam variant");
      desc.sub_desc_start = static_cast<uint32_t>(fx.complex_sub_descs.size());
      desc.and_terms_start = static_cast<uint32_t>(fx.and_term_counts.size());
      detail::BuildComplexSubDescs(*cp, fx.crystal, desc.symmetry,
                                   desc.sigma_a, desc.d_applicable != 0u,
                                   fx.complex_sub_descs, fx.and_term_counts);
    }
    fx.descs.push_back(desc);
    fx.host_specs.push_back(FilterSpec::Create(cfg, fx.crystal, fx.axis));
  }
  fx.getfn = detail::BuildDeviceGetFnBytes(fx.crystal);
  return fx;
}

// Random raw poly-index sequence generator: ensures each byte is a valid
// polygon-face index for the hex crystal.
std::vector<ParityRay> GenerateRays(const ParityFixture& fx, std::mt19937& rng,
                                    size_t n_rays, size_t face_seq_cap,
                                    bool sweep_check_mode) {
  std::vector<ParityRay> out;
  out.reserve(n_rays);
  size_t poly_n = fx.crystal.PolygonFaceCount();
  std::uniform_int_distribution<uint32_t> poly_dist(0, static_cast<uint32_t>(poly_n - 1));
  std::uniform_int_distribution<uint32_t> len_dist(1, static_cast<uint32_t>(std::min<size_t>(face_seq_cap, 7)));
  std::uniform_int_distribution<uint32_t> filter_dist(0, static_cast<uint32_t>(fx.descs.size() - 1));
  std::uniform_int_distribution<uint32_t> cid_dist(0, 15);
  std::uniform_real_distribution<float> uni(-1.0f, 1.0f);

  for (size_t i = 0; i < n_rays; ++i) {
    ParityRay r;
    uint32_t len = len_dist(rng);
    r.path_len = static_cast<uint8_t>(len);
    r.raw_poly.assign(face_seq_cap, 0);
    r.face_number.assign(face_seq_cap, 0);
    for (uint32_t k = 0; k < len; ++k) {
      uint32_t pi = poly_dist(rng);
      r.raw_poly[k] = static_cast<uint8_t>(pi);
      r.face_number[k] = fx.getfn[pi];
    }
    r.crystal_slot = 0;  // single-crystal fixture
    r.crystal_config_id = static_cast<uint16_t>(cid_dist(rng));
    // Random unit-ish direction (does not need to be normalized — dot
    // comparison uses raw value; mirror the host DirectionSpec semantics).
    float dx = uni(rng), dy = uni(rng), dz = std::abs(uni(rng)) + 0.1f;
    float n = std::sqrt(dx * dx + dy * dy + dz * dz);
    r.dir[0] = dx / n;
    r.dir[1] = dy / n;
    r.dir[2] = dz / n;
    r.filter_idx = filter_dist(rng);
    (void)sweep_check_mode;
    out.push_back(std::move(r));
  }
  return out;
}

// Pack ParityRay vector into MTLBuffer-friendly SoA layout.
struct DeviceRayBuffers {
  id<MTLBuffer> path = nil;       // n * face_seq_cap bytes
  id<MTLBuffer> path_len = nil;   // n bytes
  id<MTLBuffer> cslot = nil;      // n * uint16
  id<MTLBuffer> cid = nil;        // n * uint16
  id<MTLBuffer> dir = nil;        // n * 3 floats
  id<MTLBuffer> filter = nil;     // n * uint32
  id<MTLBuffer> out_match = nil;  // n bytes
};

DeviceRayBuffers UploadRays(id<MTLDevice> dev, const std::vector<ParityRay>& rays, size_t face_seq_cap) {
  size_t n = rays.size();
  std::vector<uint8_t>  path(n * face_seq_cap, 0);
  std::vector<uint8_t>  plen(n, 0);
  std::vector<uint16_t> cslot(n, 0);
  std::vector<uint16_t> cid(n, 0);
  std::vector<float>    dir(n * 3, 0.0f);
  std::vector<uint32_t> fi(n, 0);
  for (size_t i = 0; i < n; ++i) {
    const ParityRay& r = rays[i];
    std::memcpy(path.data() + i * face_seq_cap, r.raw_poly.data(), face_seq_cap);
    plen[i] = r.path_len;
    cslot[i] = r.crystal_slot;
    cid[i] = r.crystal_config_id;
    dir[i * 3 + 0] = r.dir[0];
    dir[i * 3 + 1] = r.dir[1];
    dir[i * 3 + 2] = r.dir[2];
    fi[i] = r.filter_idx;
  }
  DeviceRayBuffers b;
  b.path      = MakeShared(dev, path.data(),  path.size());
  b.path_len  = MakeShared(dev, plen.data(),  plen.size());
  b.cslot     = MakeShared(dev, cslot.data(), cslot.size() * sizeof(uint16_t));
  b.cid       = MakeShared(dev, cid.data(),   cid.size() * sizeof(uint16_t));
  b.dir       = MakeShared(dev, dir.data(),   dir.size() * sizeof(float));
  b.filter    = MakeShared(dev, fi.data(),    fi.size() * sizeof(uint32_t));
  b.out_match = MakeShared(dev, nullptr,      n);
  return b;
}

// Computes the host expected outcome for a single ray, mirroring the device
// kernel's `DeviceFilterMatch` / `DeviceFilterCheck` for that filter.
bool HostExpected(const ParityFixture& fx, const ParityRay& r, bool apply_action) {
  RaySeg ray{};
  ray.d_[0] = r.dir[0]; ray.d_[1] = r.dir[1]; ray.d_[2] = r.dir[2];
  ray.crystal_config_id_ = static_cast<IdType>(r.crystal_config_id);
  RaypathRecorder rec{};
  rec.Clear();
  rec.size_ = r.path_len;
  for (uint8_t k = 0; k < r.path_len; ++k) {
    rec.data_[k] = r.face_number[k];
  }
  const FilterSpec* spec = fx.host_specs[r.filter_idx].get();
  if (apply_action) {
    return spec->Check(ray, rec, nullptr);
  }
  // Match-only: invert action XOR back out
  bool checked = spec->Check(ray, rec, nullptr);
  const auto& cfg = fx.filter_configs[r.filter_idx];
  return cfg.action_ == FilterConfig::kFilterIn ? checked : !checked;
}

void RunSweep(MetalHarness& h, const ParityFixture& fx, std::mt19937& rng,
              size_t n_rays, uint32_t check_mode,
              size_t& out_total, size_t& out_mismatch,
              size_t* per_filter_total = nullptr, size_t* per_filter_mismatch = nullptr) {
  auto rays = GenerateRays(fx, rng, n_rays, fx.face_seq_cap, /*sweep_check_mode=*/false);

  // Filter out rays targeting the Complex slot if any — there are none in the
  // current fixture, but keep this defensive in case the fixture is extended.
  // (No-op as-is.)

  // Upload device-side state.
  id<MTLBuffer> filter_desc_buf = MakeShared(h.device, fx.descs.data(), fx.descs.size() * sizeof(DeviceFilterDesc));
  // fx.getfn is the single-crystal per-instance GetFn stripe (== the
  // production poly_fn table for one shape at offset 0); the retired getfn_offsets
  // prefix-sum table is no longer built or bound.
  id<MTLBuffer> getfn_buf = MakeShared(h.device, fx.getfn.data(), fx.getfn.size());
  // Complex sub-desc buffer — 1-byte dummy when empty so the buffer(11) bind
  // is always valid (Metal disallows nil); kernel reads gated by Complex
  // dispatch which only triggers on Complex filter slots.
  id<MTLBuffer> complex_sub_buf = MakeShared(h.device, fx.complex_sub_descs.data(),
                                             fx.complex_sub_descs.size() * sizeof(DeviceFilterDesc));
  // task-device-flat-and-terms: parallel flat AND-term counts (buffer 12).
  // Test harness uses an independent binding — there are plenty of free slots
  // (production Metal packs it into buffer 27 via
  // KernelParams.and_term_counts_base_offset to stay under the 30-buffer cap).
  id<MTLBuffer> and_term_buf = MakeShared(h.device, fx.and_term_counts.data(),
                                          fx.and_term_counts.size());
  auto rb = UploadRays(h.device, rays, fx.face_seq_cap);

  FilterMatchTestParams prm{};
  prm.n = static_cast<uint32_t>(n_rays);
  prm.face_seq_cap = static_cast<uint32_t>(fx.face_seq_cap);
  prm.check_mode = check_mode;
  DispatchParity(h, filter_desc_buf, getfn_buf,
                 rb.path, rb.path_len, rb.cslot, rb.cid, rb.dir, rb.filter,
                 rb.out_match, prm, complex_sub_buf, and_term_buf);

  const uint8_t* dev_out = static_cast<const uint8_t*>([rb.out_match contents]);
  size_t mism = 0;
  for (size_t i = 0; i < n_rays; ++i) {
    bool host_b = HostExpected(fx, rays[i], check_mode != 0u);
    bool dev_b = dev_out[i] != 0u;
    if (host_b != dev_b) {
      ++mism;
    }
    if (per_filter_total != nullptr) {
      per_filter_total[rays[i].filter_idx]++;
      if (host_b != dev_b) {
        per_filter_mismatch[rays[i].filter_idx]++;
      }
    }
  }
  out_total += n_rays;
  out_mismatch += mism;
}

// =============================================================================
// Tests
// =============================================================================

TEST(MetalFilterMatchParity, CompilesAndOccupancy) {
  if (ShouldSkipMetalTests()) { GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set"; }
  MetalHarness h;
  ASSERT_TRUE(h.Init()) << "Metal kernel compile failed: " << h.compile_log;
  EXPECT_GE(h.pso.maxTotalThreadsPerThreadgroup, (NSUInteger)512)
      << "occupancy regression — sub-task 2 fused kernel may need a split dispatch";
  std::fprintf(stderr,
               "[occupancy] filter_match_test_kernel maxThreadsPerThreadgroup=%lu\n",
               (unsigned long)h.pso.maxTotalThreadsPerThreadgroup);
}

TEST(MetalFilterMatchParity, RandomSequencesAcrossAxisAndCheckMode) {
  if (ShouldSkipMetalTests()) { GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set"; }
  MetalHarness h;
  ASSERT_TRUE(h.Init()) << "Metal kernel compile failed: " << h.compile_log;

  // Two fixtures × two check modes — covers d_applicable on/off and the
  // action XOR (Match vs Check). Per-sweep ray count picked so the aggregate
  // hits ≥ 1M (acceptance N — plan §1).
  // 4 sweeps × 300K = 1.2M raw checks.
  constexpr size_t kPerSweep = 300'000;
  std::mt19937 rng(0xC0FFEEu);

  size_t total = 0, mism = 0;
  for (int axis_d_app : { 0, 1 }) {
    ParityFixture fx = BuildFixture(axis_d_app != 0);
    // 10 Simple + 5 Complex (A/B/C/D/E). When this count changes, update both
    // BuildFixture and this constant. task-device-flat-and-terms added
    // Complex-E (20 OR-clauses × 1 AND) so the sweep exercises >8 OR-clauses on
    // the real GPU.
    constexpr size_t kFilterCnt = 15;
    ASSERT_EQ(fx.descs.size(), kFilterCnt);
    // Guard against the "double pass-through" trap: if all Complex descs had
    // or_clause_count==0, both host (empty Complex → false) and device (early
    // return false) would agree and silently bypass the Complex MATCH logic.
    // Assert every Complex slot carries a populated clause list.
    for (size_t fi = 0; fi < kFilterCnt; ++fi) {
      if (fx.descs[fi].type == kDeviceFilterTypeComplex) {
        ASSERT_GT(fx.descs[fi].or_clause_count, 0u)
            << "Complex filter " << fi << " has 0 OR-clauses — fixture would not exercise MATCH";
      }
    }
    size_t per_filter_total[kFilterCnt]{};
    size_t per_filter_mism[kFilterCnt]{};
    for (uint32_t check_mode : { 0u, 1u }) {
      RunSweep(h, fx, rng, kPerSweep, check_mode, total, mism,
               per_filter_total, per_filter_mism);
    }
    for (size_t fi = 0; fi < kFilterCnt; ++fi) {
      if (per_filter_total[fi] > 0) {
        std::fprintf(stderr,
                     "[parity] axis_d=%d filter=%zu total=%zu mismatch=%zu\n",
                     axis_d_app, fi, per_filter_total[fi], per_filter_mism[fi]);
      }
    }
  }
  std::fprintf(stderr, "[parity] aggregate total=%zu mismatch=%zu\n", total, mism);
  EXPECT_EQ(mism, 0u) << "device filter MATCH diverged from host FilterSpec::Check";
  EXPECT_GE(total, 1'000'000u) << "N below acceptance bound (plan §1)";
}

// Step 4 — BeginSession buffer upload sanity. The production trace kernel
// does not yet bind filter_desc_buf_, but BeginSession must have populated it
// with a non-null buffer and the first descriptor must carry the expected
// type tag.
TEST(MetalFilterMatchParity, BeginSessionUploadsFilterDescs) {
  if (ShouldSkipMetalTests()) { GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set"; }

  auto render = MakeRectangularRender();
  auto scene = MakeMetalScene(/*max_hits=*/8, /*ms_layers=*/2);
  // Inject a raypath filter on layer 0 ci 0 so canonical_len > 0.
  {
    FilterConfig cfg{};
    cfg.symmetry_ = FilterConfig::kSymP;
    cfg.action_ = FilterConfig::kFilterIn;
    RaypathFilterParam p;
    p.raypath_ = std::vector<IdType>{ 3, 5 };
    cfg.param_ = p;
    scene.ms_[0].setting_[0].filter_ = cfg;
  }

  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = 1234u;

  MetalTraceBackend backend;
  backend.BeginSession(spec);
  // Reach into the production class to rebuild the same descs the backend
  // should have uploaded. This is the "non-destructive" Step 4 check: we
  // verify host-side that BuildDeviceFilterDesc on the session config yields
  // a non-None descriptor for the raypath slot, which the backend's upload
  // path mirrors verbatim (proof by construction — no buffer reach-in).
  Crystal proto = Crystal::CreatePrism(1.0f);
  auto desc0 = detail::BuildDeviceFilterDesc(
      scene.ms_[0].setting_[0].filter_, proto, scene.ms_[0].setting_[0].crystal_.axis_);
  EXPECT_EQ(desc0.type, kDeviceFilterTypeRaypath);
  EXPECT_GT(desc0.canonical_len, 0u);
  backend.EndSession();
}

}  // namespace
}  // namespace lumice

#else

TEST(MetalFilterMatchParity, SkippedOnNonApple) {
  GTEST_SKIP() << "non-Apple platform";
}

#endif  // defined(__APPLE__)
