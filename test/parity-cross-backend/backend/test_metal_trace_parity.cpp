// CPU-vs-Metal parity harness (scrum sub-task 4).
//
// Productionalises the throwaway oracle from the explore spike: a CPU
// implementation that mirrors MetalTraceBackend's per-thread kernel logic
// byte-for-byte (single-path follow with refract-priority continuation,
// argmax-facing + centroid re-entry between MS layers) so the test fixture
// can assert numeric equivalence against the GPU backend across:
//
//   - Test G — single layer, exit_count strict equal + exit_w_sum/XYZ rel
//   - Test H — 2 layers end-to-end, per-layer stats + final XYZ rel
//   - Test I — single layer with deeper recorder (max_hits=16)
//
// In `LUMICE_SKIP_METAL_TESTS=1` environments each test SKIPs cleanly.
//
// Why a kernel-mirror oracle rather than CpuTraceBackend as reference:
// CpuTraceBackend's HitSurface/Propagate path fans out per hit (1 ray → 2
// rays) and uses area-weighted re-entry sampling — both diverge from the
// kernel's behaviour and would fail strict exit_count parity. The oracle
// here is intentionally a CPU copy of the .mm kernel body.

#include <gtest/gtest.h>

#if defined(__APPLE__)

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <vector>

#include "config/render_config.hpp"
#include "config/sim_data.hpp"
#include "core/color_util.hpp"
#include "core/cpu_trace_backend.hpp"
#include "core/crystal.hpp"
#include "core/def.hpp"
#include "core/geo3d.hpp"
#include "core/math.hpp"
#include "core/metal_trace_backend.hpp"
#include "core/scatter_accum.hpp"
#include "core/simulator.hpp"  // PartitionCrystalRayNum
#include "core/trace_backend.hpp"
#include "core/trace_ops.hpp"
#include "metal_test_helpers.hpp"
#include "util/color_data.hpp"

namespace lumice {
namespace {

using metal_test::ChannelSum;
using metal_test::EnableDeviceGenForStatisticalParity;
using metal_test::ForceHostGenForByteIdentity;
using metal_test::MakeMetalScene;
using metal_test::MakeMultiCrystalScene;
using metal_test::MakeRectangularRender;
using metal_test::RelErr;
using metal_test::ShouldSkipMetalTests;

constexpr float kFloatEps = 1e-5f;

struct PolyArrays {
  std::vector<float> n;         // 3 * poly_cnt
  std::vector<float> d;         // poly_cnt
  std::vector<float> centroid;  // 3 * poly_cnt — same algorithm as
                                // metal_trace_backend.mm UploadCrystal (mean
                                // of the polygon's reference triangle's 3
                                // vertices via GetPolygonFaceTriId).
};

PolyArrays BuildPolyArrays(const Crystal& crystal) {
  size_t poly_cnt = crystal.PolygonFaceCount();
  PolyArrays out;
  out.n.assign(crystal.GetPolygonFaceNormal(), crystal.GetPolygonFaceNormal() + poly_cnt * 3);
  out.d.assign(crystal.GetPolygonFaceDist(), crystal.GetPolygonFaceDist() + poly_cnt);
  out.centroid.assign(poly_cnt * 3, 0.0f);
  const int* poly_tri = crystal.GetPolygonFaceTriId();
  const float* tvtx = crystal.GetTriangleVtx();
  for (size_t f = 0; f < poly_cnt; f++) {
    const float* v = tvtx + static_cast<size_t>(poly_tri[f]) * 9;
    for (int k = 0; k < 3; k++) {
      out.centroid[f * 3 + k] = (v[0 * 3 + k] + v[1 * 3 + k] + v[2 * 3 + k]) / 3.0f;
    }
  }
  return out;
}

float GetReflectRatio(float delta, float rr) {
  float d_sqrt = std::sqrt(delta);
  float rs = (rr - d_sqrt) / (rr + d_sqrt);
  rs *= rs;
  float rp = (1.0f - rr * d_sqrt) / (1.0f + rr * d_sqrt);
  rp *= rp;
  return (rs + rp) * 0.5f;
}

// Per-layer outcome of the kernel-mirror oracle. exit_d/exit_w hold one
// entry per "ray that left the crystal" event (matches the Metal kernel's
// per-thread atomic_fetch_add increment on exit_count). exit_count_total
// is identical to exit_d.size() / 3 — exposed separately for clarity in
// strict-equality assertions.
struct OracleLayerResult {
  size_t exit_count = 0;
  double exit_w_sum = 0.0;  // double precision for the oracle reference
  std::vector<float> exit_d;
  std::vector<float> exit_w;
};

// Run one MS layer of the kernel-mirror oracle.
//
// THIS FUNCTION MUST STAY IN SYNC WITH kKernelSrc (metal_trace_backend.mm).
// It mirrors the per-thread logic of the Metal compute kernel: single-path
// refract-priority follow, argmax-facing re-entry, float Fresnel weights.
// If the Metal kernel's physical model changes (refraction weights, face
// selection, etc.), update this oracle accordingly.
//
// roots_* are crystal-local input rays (matching MetalTraceBackend's first-
// layer upload format from GenerateFirstLayerRoots). The function does NOT
// project — accumulating the exit rays into an XYZ image is the caller's
// responsibility (typically `ScatterOutgoingToXyz` for the final layer to
// match CpuTraceBackend's projection path).
// NOLINTNEXTLINE(readability-function-size,readability-function-cognitive-complexity)
OracleLayerResult OracleTraceLayer(const Crystal& crystal, const PolyArrays& poly, float n_idx, size_t max_hits,
                                   const std::vector<float>& root_d, const std::vector<float>& root_p,
                                   const std::vector<float>& root_w, const std::vector<IdType>& root_tf) {
  size_t n_rays = root_w.size();
  size_t poly_cnt = crystal.PolygonFaceCount();

  OracleLayerResult res;
  res.exit_d.reserve(n_rays * 3);
  res.exit_w.reserve(n_rays);

  for (size_t ri = 0; ri < n_rays; ri++) {
    float dx = root_d[ri * 3 + 0];
    float dy = root_d[ri * 3 + 1];
    float dz = root_d[ri * 3 + 2];
    float ox = root_p[ri * 3 + 0];
    float oy = root_p[ri * 3 + 1];
    float oz = root_p[ri * 3 + 2];
    float w = root_w[ri];
    IdType to_face = root_tf[ri];

    for (size_t hit = 0; hit < max_hits; hit++) {
      if (to_face == kInvalidId) {
        break;
      }
      float nx = poly.n[to_face * 3 + 0];
      float ny = poly.n[to_face * 3 + 1];
      float nz = poly.n[to_face * 3 + 2];
      float cos_theta = dx * nx + dy * ny + dz * nz;
      float rr = (cos_theta > 0.0f) ? n_idx : (1.0f / n_idx);
      float dd = (1.0f - rr * rr) / (cos_theta * cos_theta) + rr * rr;
      bool is_tir = dd <= 0.0f;
      float w_refl = GetReflectRatio(std::max(dd, 0.0f), rr) * w;
      float w_refr = is_tir ? -1.0f : (w - w_refl);
      float rdx = dx - 2.0f * cos_theta * nx;
      float rdy = dy - 2.0f * cos_theta * ny;
      float rdz = dz - 2.0f * cos_theta * nz;
      float sd = std::sqrt(std::max(dd, 0.0f));
      float fdx = 0.0f;
      float fdy = 0.0f;
      float fdz = 0.0f;
      if (is_tir) {
        fdx = rdx;
        fdy = rdy;
        fdz = rdz;
      } else {
        fdx = rr * dx - (rr - sd) * cos_theta * nx;
        fdy = rr * dy - (rr - sd) * cos_theta * ny;
        fdz = rr * dz - (rr - sd) * cos_theta * nz;
      }

      IdType cont_face = kInvalidId;
      float c_dx = 0.0f;
      float c_dy = 0.0f;
      float c_dz = 0.0f;
      float c_ox = 0.0f;
      float c_oy = 0.0f;
      float c_oz = 0.0f;
      float c_w = 0.0f;

      for (int ch = 0; ch < 2; ch++) {
        float cdx = (ch == 0) ? rdx : fdx;
        float cdy = (ch == 0) ? rdy : fdy;
        float cdz = (ch == 0) ? rdz : fdz;
        float cw = (ch == 0) ? w_refl : w_refr;
        if (cw < 0.0f) {
          continue;
        }

        float t_far = 1e30f;
        int far_face = -1;
        for (size_t fi = 0; fi < poly_cnt; fi++) {
          float fnx = poly.n[fi * 3 + 0];
          float fny = poly.n[fi * 3 + 1];
          float fnz = poly.n[fi * 3 + 2];
          float fd = poly.d[fi];
          float denom = cdx * fnx + cdy * fny + cdz * fnz;
          float t = -(ox * fnx + oy * fny + oz * fnz + fd) / denom;
          if (denom > kFloatEps && t < t_far) {
            t_far = t;
            far_face = static_cast<int>(fi);
          }
        }
        float eps_thr = (to_face != kInvalidId && far_face != static_cast<int>(to_face)) ? -kFloatEps : kFloatEps;
        if (far_face >= 0 && t_far > eps_thr) {
          // Refract (ch==1) overwrites reflect (ch==0): single-path priority.
          // Reflect energy is intentionally discarded (mirrors kernel).
          cont_face = static_cast<IdType>(far_face);
          c_dx = cdx;
          c_dy = cdy;
          c_dz = cdz;
          c_ox = ox + t_far * cdx;
          c_oy = oy + t_far * cdy;
          c_oz = oz + t_far * cdz;
          c_w = cw;
        } else {
          // Exit from crystal — record (matches the kernel's atomic
          // exit_cnt/exit_wsum increments in both ms_mode branches).
          res.exit_count++;
          res.exit_w_sum += static_cast<double>(cw);
          res.exit_d.push_back(cdx);
          res.exit_d.push_back(cdy);
          res.exit_d.push_back(cdz);
          res.exit_w.push_back(cw);
        }
      }

      if (cont_face == kInvalidId) {
        to_face = kInvalidId;
      } else {
        dx = c_dx;
        dy = c_dy;
        dz = c_dz;
        ox = c_ox;
        oy = c_oy;
        oz = c_oz;
        w = c_w;
        to_face = cont_face;
      }
    }
  }
  return res;
}

// NOTE (253.2): the former OracleReentryRoots helper (local-frame argmax-facing
// + centroid re-entry, mirroring the PRE-253.2 kernel ms_mode==1 branch) was
// removed. Its local-frame re-entry — no world-transit, no per-ray crystal_rot_
// resample on the next crystal — is exactly the inter-layer frame bug 253.2
// fixes, so it is no longer a valid multi-MS oracle. Multi-layer structural
// parity now uses the real CpuTraceBackend as an independent world-space oracle
// (MetalVsCpuMultiLayerSpatialStructure).

// Reproduce MetalTraceBackend::GenerateFirstLayerRoots exactly: same RNG
// path (SetSeed + global SetSeed + MakeCrystal + InitRayFirstMs) so the
// oracle's roots are byte-equivalent to the rays Metal uploads to the GPU.
struct FirstLayerRoots {
  Crystal crystal;
  float n_idx = 0.0f;
  std::vector<float> d;
  std::vector<float> p;
  std::vector<float> w;
  std::vector<IdType> tf;
};

FirstLayerRoots BuildFirstLayerRoots(RandomNumberGenerator& rng, const SessionSpec& spec, size_t n_rays) {
  const auto& ms = spec.scene->ms_[0];
  const auto& setting = ms.setting_[0];

  FirstLayerRoots out;
  out.crystal = MakeCrystal(rng, setting.crystal_.param_);
  out.n_idx = out.crystal.GetRefractiveIndex(spec.wl.wl_);

  RayBuffer workspace[2]{};
  workspace[0].Reset(n_rays);
  workspace[1].Reset(n_rays);
  RayBuffer all_data = AllocateAllData(*spec.scene, n_rays);

  InitRayFirstMs(rng, spec.scene->light_source_.param_, spec.wl, n_rays, out.crystal, /*curr_crystal_id=*/0,
                 setting.crystal_.axis_, workspace, all_data);

  size_t n_actual = workspace[0].size_;
  out.d.assign(n_actual * 3, 0.0f);
  out.p.assign(n_actual * 3, 0.0f);
  out.w.assign(n_actual, 0.0f);
  out.tf.assign(n_actual, 0);
  for (size_t i = 0; i < n_actual; i++) {
    const RaySeg& r = workspace[0][i];
    out.d[i * 3 + 0] = r.d_[0];
    out.d[i * 3 + 1] = r.d_[1];
    out.d[i * 3 + 2] = r.d_[2];
    out.p[i * 3 + 0] = r.p_[0];
    out.p[i * 3 + 1] = r.p_[1];
    out.p[i * 3 + 2] = r.p_[2];
    out.w[i] = r.w_;
    out.tf[i] = r.to_face_;
  }
  return out;
}

// Seed both the per-call RNG and the global RNG identically to
// MetalTraceBackend::BeginSession's first lines.
void SeedRngsLikeMetal(RandomNumberGenerator& rng, uint32_t seed) {
  // seed=0 is a special "no-reseed" sentinel in MetalTraceBackend; oracle
  // cannot match Metal's RNG state in that case. Guard to prevent silent
  // parity divergence if a test ever passes seed=0.
  EXPECT_NE(seed, 0u) << "seed=0 not supported by oracle (RNG state unknown)";
  if (seed == 0u) {
    return;
  }
  rng.SetSeed(seed);
  RandomNumberGenerator::GetInstance().SetSeed(seed);
}

// =============================================================================
// Test G — single-layer parity
// =============================================================================
TEST(MetalTraceParity, SingleLayerExitStatsAndXyz) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }
  // Oracle mirrors host mt19937 stream; device PCG (task-260.2) cannot align.
  ForceHostGenForByteIdentity();

  auto scene = MakeMetalScene(/*max_hits=*/8, /*ms_layers=*/1);
  auto render = MakeRectangularRender();

  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = 42;

  constexpr size_t kRayCount = 4096;
  HostRayBatch host;
  host.count = kRayCount;
  host.crystal = nullptr;
  host.refractive_index = 0.0f;

  // --- Metal run ---
  std::vector<float> xyz_metal(render.resolution_[0] * render.resolution_[1] * 3, 0.0f);
  LayerStats metal_stats;
  {
    MetalTraceBackend metal;
    metal.BeginSession(spec);
    auto h = metal.TraceLayer(RootRaySource::FromHost(host));
    ASSERT_NE(h, nullptr);
    metal_stats = h->GetLayerStats();
    XyzImageData img{ xyz_metal.data(), render.resolution_[0], render.resolution_[1] };
    metal.ReadbackImage(img);
    metal.EndSession();
  }

  // --- Oracle run (mirror BeginSession RNG path exactly) ---
  RandomNumberGenerator oracle_rng(0);
  SeedRngsLikeMetal(oracle_rng, spec.seed);
  auto roots = BuildFirstLayerRoots(oracle_rng, spec, kRayCount);
  auto poly = BuildPolyArrays(roots.crystal);

  auto oracle =
      OracleTraceLayer(roots.crystal, poly, roots.n_idx, scene.max_hits_, roots.d, roots.p, roots.w, roots.tf);

  // Project oracle exit rays via the canonical CPU projection (matches the
  // path CpuTraceBackend takes in Test E, which agrees with Metal at
  // rel ≤ 5e-4 even though Metal inlines its own az0-projection).
  std::vector<float> xyz_oracle(render.resolution_[0] * render.resolution_[1] * 3, 0.0f);
  Rotation camera_rot = MakeCameraRotation(render);
  ScatterOutgoingToXyz(oracle.exit_d.data(), oracle.exit_w.data(), oracle.exit_w.size(), render, camera_rot,
                       spec.wl.wl_, xyz_oracle.data());

  EXPECT_EQ(metal_stats.exit_count, oracle.exit_count)
      << "exit_count mismatch — metal=" << metal_stats.exit_count << " oracle=" << oracle.exit_count;
  EXPECT_LT(RelErr(metal_stats.exit_w_sum, oracle.exit_w_sum), 5e-4)
      << "exit_w_sum: metal=" << metal_stats.exit_w_sum << " oracle=" << oracle.exit_w_sum;

  for (int c = 0; c < 3; c++) {
    double sm = ChannelSum(xyz_metal, c);
    double so = ChannelSum(xyz_oracle, c);
    EXPECT_LT(RelErr(sm, so), 5e-4) << "channel=" << c << " metal=" << sm << " oracle=" << so;
  }
}

// =============================================================================
// Test H — two-layer MS end-to-end parity
// =============================================================================
TEST(MetalTraceParity, TwoLayerExitStatsAndXyz) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }
  ForceHostGenForByteIdentity();

  auto scene = MakeMetalScene(/*max_hits=*/6, /*ms_layers=*/2);
  auto render = MakeRectangularRender();

  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = 7;

  constexpr size_t kRayCount = 2048;
  HostRayBatch host;
  host.count = kRayCount;
  host.crystal = nullptr;
  host.refractive_index = 0.0f;

  // --- Metal run ---
  std::vector<float> xyz_metal(render.resolution_[0] * render.resolution_[1] * 3, 0.0f);
  LayerStats metal_layer0_stats;
  LayerStats metal_layer1_stats;
  size_t metal_cont0 = 0;
  {
    MetalTraceBackend metal;
    metal.BeginSession(spec);

    auto h0 = metal.TraceLayer(RootRaySource::FromHost(host));
    ASSERT_NE(h0, nullptr);
    metal_layer0_stats = h0->GetLayerStats();
    metal_cont0 = h0->ContinuationCount();

    RecombineSpec rspec;
    rspec.shuffle = false;
    auto roots1 = metal.Recombine(std::move(h0), rspec);

    auto h1 = metal.TraceLayer(roots1);
    ASSERT_NE(h1, nullptr);
    metal_layer1_stats = h1->GetLayerStats();
    EXPECT_EQ(h1->ContinuationCount(), 0u);

    XyzImageData img{ xyz_metal.data(), render.resolution_[0], render.resolution_[1] };
    metal.ReadbackImage(img);
    metal.EndSession();
  }

  // --- Oracle run ---
  RandomNumberGenerator oracle_rng(0);
  SeedRngsLikeMetal(oracle_rng, spec.seed);

  // Layer 0: same MakeCrystal + InitRayFirstMs path as Metal.
  auto roots0 = BuildFirstLayerRoots(oracle_rng, spec, kRayCount);
  auto poly0 = BuildPolyArrays(roots0.crystal);
  auto oracle_l0 =
      OracleTraceLayer(roots0.crystal, poly0, roots0.n_idx, scene.max_hits_, roots0.d, roots0.p, roots0.w, roots0.tf);

  // scrum-267 task-fused-emit-gate: cont_count is now the post-gate
  // (filter+prob) continuation count, not the geometric polygon-exit count.
  // For the no-filter test scene with layer-0 prob_=0.6 we expect roughly
  // 60% of polygon exits to survive the gate; the strict-equality assertion
  // of the old design (kernel writes every exit to cont buffer) no longer
  // holds. The kernel exit_count atomic still increments per filter_pass
  // ray, so for no-filter scenes it matches the oracle's geometric count.
  EXPECT_GT(metal_cont0, 0u) << "layer0 produced no continuations after gate";
  EXPECT_LE(metal_cont0, oracle_l0.exit_count) << "layer0 continuation_count cannot exceed polygon-exit count";
  EXPECT_EQ(metal_layer0_stats.exit_count, oracle_l0.exit_count) << "layer0 exit_count mismatch";
  EXPECT_LT(RelErr(metal_layer0_stats.exit_w_sum, oracle_l0.exit_w_sum), 5e-4)
      << "layer0 exit_w_sum: metal=" << metal_layer0_stats.exit_w_sum << " oracle=" << oracle_l0.exit_w_sum;

  // Layer-1 kernel-mirror comparison RETIRED (253.2). The kernel-mirror re-entry
  // (OracleReentryRoots: local-frame argmax-facing + centroid, no world-transit,
  // no crystal_rot_ resample) is exactly the pre-253.2 inter-layer frame BUG —
  // it is no longer a correct baseline now that Metal returns continuation rays
  // to world space and resamples a new per-ray rotation + entry face on the next
  // crystal (mirroring CPU CollectData + InitRayOtherMs). Multi-MS structural
  // correctness for layer 1+ is validated against the independent CpuTraceBackend
  // world-space oracle in MetalVsCpuMultiLayerSpatialStructure (per-pixel
  // correlation). Here we keep the still-valid layer-0 exit-stats checks above
  // and a metal-side sanity check that the 2-layer drive produced finite light.
  EXPECT_GT(metal_layer1_stats.exit_count, 0u) << "layer1 produced no exits";
  EXPECT_TRUE(std::isfinite(metal_layer1_stats.exit_w_sum));
  EXPECT_GT(ChannelSum(xyz_metal, 1), 0.0) << "2-layer drive produced no Y-channel light";
}

// =============================================================================
// Test I — deep-recorder single-layer parity (max_hits=16)
// =============================================================================
TEST(MetalTraceParity, DeepRecorderSingleLayer) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }
  ForceHostGenForByteIdentity();

  auto scene = MakeMetalScene(/*max_hits=*/16, /*ms_layers=*/1);
  auto render = MakeRectangularRender();

  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = 123;

  constexpr size_t kRayCount = 4096;
  HostRayBatch host;
  host.count = kRayCount;
  host.crystal = nullptr;
  host.refractive_index = 0.0f;

  std::vector<float> xyz_metal(render.resolution_[0] * render.resolution_[1] * 3, 0.0f);
  LayerStats metal_stats;
  {
    MetalTraceBackend metal;
    metal.BeginSession(spec);
    auto h = metal.TraceLayer(RootRaySource::FromHost(host));
    ASSERT_NE(h, nullptr);
    metal_stats = h->GetLayerStats();
    XyzImageData img{ xyz_metal.data(), render.resolution_[0], render.resolution_[1] };
    metal.ReadbackImage(img);
    metal.EndSession();
  }

  RandomNumberGenerator oracle_rng(0);
  SeedRngsLikeMetal(oracle_rng, spec.seed);
  auto roots = BuildFirstLayerRoots(oracle_rng, spec, kRayCount);
  auto poly = BuildPolyArrays(roots.crystal);
  auto oracle =
      OracleTraceLayer(roots.crystal, poly, roots.n_idx, scene.max_hits_, roots.d, roots.p, roots.w, roots.tf);

  std::vector<float> xyz_oracle(render.resolution_[0] * render.resolution_[1] * 3, 0.0f);
  Rotation camera_rot = MakeCameraRotation(render);
  ScatterOutgoingToXyz(oracle.exit_d.data(), oracle.exit_w.data(), oracle.exit_w.size(), render, camera_rot,
                       spec.wl.wl_, xyz_oracle.data());

  EXPECT_EQ(metal_stats.exit_count, oracle.exit_count);
  EXPECT_LT(RelErr(metal_stats.exit_w_sum, oracle.exit_w_sum), 5e-4)
      << "metal=" << metal_stats.exit_w_sum << " oracle=" << oracle.exit_w_sum;
  for (int c = 0; c < 3; c++) {
    double sm = ChannelSum(xyz_metal, c);
    double so = ChannelSum(xyz_oracle, c);
    EXPECT_LT(RelErr(sm, so), 5e-4) << "channel=" << c << " metal=" << sm << " oracle=" << so;
  }
}

// =============================================================================
// Multi-population oracle helpers (Test J / K).
//
// Mirrors MetalTraceBackend::TraceLayer's multi-ci loop: per-ci
// PartitionCrystalRayNum-sliced root rays, MakeCrystal then InitRayFirstMs
// (or staged continuation slice for non-first MS), OracleTraceLayer per ci,
// concatenated exit_d / exit_w in ci order (matches Metal's append behaviour
// driven by the atomic counter resuming from counter_init = cumulative).
// =============================================================================
struct MultiPopFirstLayerResult {
  size_t total_exit_count = 0;
  double total_exit_w_sum = 0.0;
  std::vector<float> exit_d;
  std::vector<float> exit_w;
  // Per-ci crystals retained so callers can drive a second MS layer's
  // re-entry geometry through the same poly tables Metal uses.
  std::vector<Crystal> crystals;
  std::vector<PolyArrays> polys;
};

MultiPopFirstLayerResult OracleRunFirstLayerMultiPop(RandomNumberGenerator& rng, const SessionSpec& spec,
                                                     size_t total_ray_num) {
  const auto& ms = spec.scene->ms_[0];
  size_t crystal_cnt = ms.setting_.size();
  std::vector<float> proportions;
  proportions.reserve(crystal_cnt);
  for (size_t ci = 0; ci < crystal_cnt; ci++) {
    proportions.push_back(ms.setting_[ci].crystal_proportion_);
  }
  std::vector<double> carry(crystal_cnt, 0.0);
  auto crystal_ray_num = PartitionCrystalRayNum(proportions, total_ray_num, carry);

  MultiPopFirstLayerResult out;
  out.crystals.reserve(crystal_cnt);
  out.polys.reserve(crystal_cnt);
  for (size_t ci = 0; ci < crystal_cnt; ci++) {
    size_t ci_n = crystal_ray_num[ci];
    const auto& setting = ms.setting_[ci];

    // Match MetalTraceBackend: ResolveLayerCrystalForCi calls MakeCrystal
    // unconditionally (host crystal path is not used in this test).
    Crystal crystal = MakeCrystal(rng, setting.crystal_.param_);
    float n_idx = crystal.GetRefractiveIndex(spec.wl.wl_);
    PolyArrays poly = BuildPolyArrays(crystal);

    if (ci_n == 0) {
      // Still record the crystal so the Metal-side RNG cadence stays
      // aligned even if a ci is empty (in practice 70/30 + small ray
      // count rarely produces ci_n=0 but the structure must mirror).
      out.crystals.push_back(std::move(crystal));
      out.polys.push_back(std::move(poly));
      continue;
    }

    // InitRayFirstMs draws ci_n × per-ray RNG samples — same as Metal's
    // GenerateFirstLayerRootsForCi.
    RayBuffer workspace[2]{};
    workspace[0].Reset(ci_n);
    workspace[1].Reset(ci_n);
    RayBuffer all_data = AllocateAllData(*spec.scene, ci_n);
    InitRayFirstMs(rng, spec.scene->light_source_.param_, spec.wl, ci_n, crystal,
                   /*curr_crystal_id=*/ci, setting.crystal_.axis_, workspace, all_data);

    size_t n_actual = workspace[0].size_;
    std::vector<float> root_d(n_actual * 3, 0.0f);
    std::vector<float> root_p(n_actual * 3, 0.0f);
    std::vector<float> root_w(n_actual, 0.0f);
    std::vector<IdType> root_tf(n_actual, 0);
    for (size_t i = 0; i < n_actual; i++) {
      const RaySeg& r = workspace[0][i];
      root_d[i * 3 + 0] = r.d_[0];
      root_d[i * 3 + 1] = r.d_[1];
      root_d[i * 3 + 2] = r.d_[2];
      root_p[i * 3 + 0] = r.p_[0];
      root_p[i * 3 + 1] = r.p_[1];
      root_p[i * 3 + 2] = r.p_[2];
      root_w[i] = r.w_;
      root_tf[i] = r.to_face_;
    }

    auto ci_result = OracleTraceLayer(crystal, poly, n_idx, spec.scene->max_hits_, root_d, root_p, root_w, root_tf);
    out.total_exit_count += ci_result.exit_count;
    out.total_exit_w_sum += ci_result.exit_w_sum;
    out.exit_d.insert(out.exit_d.end(), ci_result.exit_d.begin(), ci_result.exit_d.end());
    out.exit_w.insert(out.exit_w.end(), ci_result.exit_w.begin(), ci_result.exit_w.end());

    out.crystals.push_back(std::move(crystal));
    out.polys.push_back(std::move(poly));
  }
  return out;
}

// =============================================================================
// Test J — multi-population single-layer parity (≥2 crystals, 0.7/0.3 split).
//
// Validates: backend per-crystal dispatch produces the same total exit_count /
// exit_w_sum / XYZ as the oracle's per-ci concatenation. The kernel's atomic
// counter resuming from counter_init = cont_counts[out_slot] is exercised
// implicitly — Test K extends to two MS layers where the continuation buffer
// is read back through Recombine.
// =============================================================================
TEST(MetalTraceParity, MultiPopSingleLayerExitStatsAndXyz) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }

  auto scene = MakeMultiCrystalScene(/*max_hits=*/8, /*ms_layers=*/1, /*crystal_count=*/2);
  auto render = MakeRectangularRender();

  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = 42;

  constexpr size_t kRayCount = 4096;
  HostRayBatch host;
  host.count = kRayCount;
  host.crystal = nullptr;
  host.refractive_index = 0.0f;

  std::vector<float> xyz_metal(render.resolution_[0] * render.resolution_[1] * 3, 0.0f);
  LayerStats metal_stats;
  {
    MetalTraceBackend metal;
    metal.BeginSession(spec);
    auto h = metal.TraceLayer(RootRaySource::FromHost(host));
    ASSERT_NE(h, nullptr);
    metal_stats = h->GetLayerStats();
    XyzImageData img{ xyz_metal.data(), render.resolution_[0], render.resolution_[1] };
    metal.ReadbackImage(img);
    metal.EndSession();
  }

  RandomNumberGenerator oracle_rng(0);
  SeedRngsLikeMetal(oracle_rng, spec.seed);
  auto oracle = OracleRunFirstLayerMultiPop(oracle_rng, spec, kRayCount);

  std::vector<float> xyz_oracle(render.resolution_[0] * render.resolution_[1] * 3, 0.0f);
  Rotation camera_rot = MakeCameraRotation(render);
  ScatterOutgoingToXyz(oracle.exit_d.data(), oracle.exit_w.data(), oracle.exit_w.size(), render, camera_rot,
                       spec.wl.wl_, xyz_oracle.data());

  EXPECT_EQ(metal_stats.exit_count, oracle.total_exit_count)
      << "exit_count mismatch — metal=" << metal_stats.exit_count << " oracle=" << oracle.total_exit_count;
  EXPECT_LT(RelErr(metal_stats.exit_w_sum, oracle.total_exit_w_sum), 5e-4)
      << "exit_w_sum: metal=" << metal_stats.exit_w_sum << " oracle=" << oracle.total_exit_w_sum;
  for (int c = 0; c < 3; c++) {
    double sm = ChannelSum(xyz_metal, c);
    double so = ChannelSum(xyz_oracle, c);
    EXPECT_LT(RelErr(sm, so), 5e-4) << "channel=" << c << " metal=" << sm << " oracle=" << so;
  }
}

// =============================================================================
// Test K — multi-population × 2 MS layers parity.
//
// Validates that the cross-ci counter_init append works under the MS
// recombine path: Layer 0 writes cont_*[0] with all ci's exits concatenated;
// argmax-facing + centroid re-entry feeds Layer 1 which performs its own
// per-ci dispatch, this time staging cont slices into root_* via memcpy.
// =============================================================================
TEST(MetalTraceParity, MultiPopTwoLayerExitStatsAndXyz) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }

  auto scene = MakeMultiCrystalScene(/*max_hits=*/6, /*ms_layers=*/2, /*crystal_count=*/2);
  auto render = MakeRectangularRender();

  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = 7;

  constexpr size_t kRayCount = 2048;
  HostRayBatch host;
  host.count = kRayCount;
  host.crystal = nullptr;
  host.refractive_index = 0.0f;

  std::vector<float> xyz_metal(render.resolution_[0] * render.resolution_[1] * 3, 0.0f);
  LayerStats metal_layer0_stats;
  LayerStats metal_layer1_stats;
  size_t metal_cont0 = 0;
  {
    MetalTraceBackend metal;
    metal.BeginSession(spec);
    auto h0 = metal.TraceLayer(RootRaySource::FromHost(host));
    ASSERT_NE(h0, nullptr);
    metal_layer0_stats = h0->GetLayerStats();
    metal_cont0 = h0->ContinuationCount();
    RecombineSpec rspec;
    rspec.shuffle = false;
    auto roots1 = metal.Recombine(std::move(h0), rspec);
    auto h1 = metal.TraceLayer(roots1);
    ASSERT_NE(h1, nullptr);
    metal_layer1_stats = h1->GetLayerStats();
    EXPECT_EQ(h1->ContinuationCount(), 0u);
    XyzImageData img{ xyz_metal.data(), render.resolution_[0], render.resolution_[1] };
    metal.ReadbackImage(img);
    metal.EndSession();
  }

  RandomNumberGenerator oracle_rng(0);
  SeedRngsLikeMetal(oracle_rng, spec.seed);

  // Layer 0 — multi-pop oracle.
  auto oracle_l0 = OracleRunFirstLayerMultiPop(oracle_rng, spec, kRayCount);
  // scrum-267 task-fused-emit-gate: same semantic shift as
  // TwoLayerExitStatsAndXyz — cont_count is now post-gate, not the geometric
  // polygon-exit count. The kernel exit_count atomic still tracks filter_pass
  // rays (= oracle's geometric count for no-filter scenes).
  EXPECT_GT(metal_cont0, 0u) << "layer0 produced no continuations after gate";
  EXPECT_LE(metal_cont0, oracle_l0.total_exit_count) << "layer0 continuation_count cannot exceed polygon-exit count";
  EXPECT_EQ(metal_layer0_stats.exit_count, oracle_l0.total_exit_count) << "layer0 exit_count mismatch";
  EXPECT_LT(RelErr(metal_layer0_stats.exit_w_sum, oracle_l0.total_exit_w_sum), 5e-4)
      << "layer0 exit_w_sum: metal=" << metal_layer0_stats.exit_w_sum << " oracle=" << oracle_l0.total_exit_w_sum;

  // Layer-1 multi-pop kernel-mirror comparison RETIRED (253.2) — same reason as
  // TwoLayerExitStatsAndXyz: OracleReentryRoots' local-frame re-entry (no world-
  // transit, no per-ray crystal_rot_ resample) is the pre-253.2 inter-layer frame
  // BUG and is no longer a correct baseline. Multi-MS structural correctness is
  // validated against the independent CpuTraceBackend world-space oracle in
  // MetalVsCpuMultiLayerSpatialStructure. The still-valid layer-0 multi-pop exit-
  // stats checks above are kept; below is a metal-side sanity check only.
  EXPECT_GT(metal_layer1_stats.exit_count, 0u) << "layer1 produced no exits";
  EXPECT_TRUE(std::isfinite(metal_layer1_stats.exit_w_sum));
  EXPECT_GT(ChannelSum(xyz_metal, 1), 0.0) << "2-layer multi-pop drive produced no Y-channel light";
}

// =============================================================================
// Test L — multi-population continuation-append capacity invariant.
//
// The Metal kernel's per-ray atomic-counter contribution is bounded by
// 2 * max_hits, and out_cap = total * (2 * max_hits + 4). So the cumulative
// produced count across all ci dispatches is structurally bounded below
// out_cap and the overflow assert/clamp path in DispatchLayer is
// unreachable from valid inputs. This test therefore validates the
// **invariant** rather than the failure path: in a tight-margin
// configuration (small max_hits, ≥2 ci append), cumulative produced stays
// within out_cap, and the layer-0 continuation count equals the sum of
// per-ci exits (no rays silently dropped by overflow clamping).
//
// Aligns with 251.3 sentinel/overflow methodology: assert the safety
// envelope around an invariant boundary, not the post-failure state.
// =============================================================================
TEST(MetalTraceParity, MultiPopContinuationAppendInvariant) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }

  // Tight margin: small max_hits keeps out_cap close to actual produced
  // count; 2 layers so cont_*[0] is non-trivially populated; 70/30 split
  // forces non-empty ci=0 AND ci=1 appends.
  auto scene = MakeMultiCrystalScene(/*max_hits=*/2, /*ms_layers=*/2, /*crystal_count=*/2);
  auto render = MakeRectangularRender();

  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = 123;

  constexpr size_t kRayCount = 1024;
  HostRayBatch host;
  host.count = kRayCount;
  host.crystal = nullptr;
  host.refractive_index = 0.0f;

  MetalTraceBackend metal;
  metal.BeginSession(spec);
  auto h0 = metal.TraceLayer(RootRaySource::FromHost(host));
  ASSERT_NE(h0, nullptr);
  size_t cont0 = h0->ContinuationCount();
  LayerStats stats0 = h0->GetLayerStats();

  // scrum-267 task-fused-emit-gate: the prior "cont0 == stats0.exit_count"
  // invariant assumed the kernel wrote every polygon exit to cont buffer.
  // The emit gate now decides on-device whether each exit continues
  // (filter_pass && rng<prob) or drops/mid-exits, so cont0 is a strict
  // subset of stats0.exit_count. Replace the equality with the relaxed
  // invariant (cont count never exceeds polygon-exit count) — overflow
  // truncation would surface as cont0 > out_cap_bound below.
  EXPECT_LE(cont0, stats0.exit_count) << "Layer 0 continuation count exceeded exit_count — gate bookkeeping broken";

  // Invariant 2: cumulative produced stays within the structural bound
  // out_cap = total * (2*max_hits + 4) = 1024 * 8 = 8192.
  size_t out_cap_bound = kRayCount * (2 * scene.max_hits_ + 4);
  EXPECT_LE(cont0, out_cap_bound) << "Layer 0 cumulative exceeds out_cap structural bound";

  // Drive into layer 1 to exercise the non-first_ms multi-ci memcpy
  // path; verify no crash and reasonable counts.
  RecombineSpec rspec;
  rspec.shuffle = false;
  auto roots1 = metal.Recombine(std::move(h0), rspec);
  auto h1 = metal.TraceLayer(roots1);
  ASSERT_NE(h1, nullptr);
  EXPECT_EQ(h1->ContinuationCount(), 0u) << "Final layer must drain continuation";
  EXPECT_GT(h1->GetLayerStats().exit_count, 0u);
  metal.EndSession();
}

// Validates the Y-channel reverse formula used by Simulator::SimulateOneWavelengthWithBackend:
//
//   total_landed_weight = (Σ_pixels XYZ[i].y) / kCmfY[wl]
//
// The formula is derived from SpectrumToXyz's accumulation: xyz[i*3+1] += kCmfY[wl] * w_i.
// Therefore Σ_pixels xyz[i*3+1] = kCmfY[wl] * Σ(w_i for in-bounds rays) = kCmfY[wl] * total_landed_weight.
//
// CpuTraceBackend uses SpectrumToXyz AND tracks TotalLandedWeight() independently. The identity
// sum_Y / cie_y == TotalLandedWeight() must hold exactly (up to float accumulation noise).
// MetalTraceBackend's kernel uses the same semantic formula; if the identity holds for
// CpuTraceBackend, it validates the formula for any backend using this XYZ accumulation pattern.
TEST(MetalTraceParity, TotalLandedWeightFormulaIdentity) {
  // Part 1: CpuTraceBackend formula identity — no Metal required, no skip gate.
  // Uses a fisheye-equal-area render (180° FOV) so that rays landing in the lower
  // hemisphere (d.z < 0, the observer-side) project into a non-empty in-bounds region.
  // (kRectangular + kUpper filters out d.z < 0 rays for this scene geometry.)
  auto scene = MakeMetalScene(/*max_hits=*/4, /*ms_layers=*/1);
  RenderConfig fisheye_render;
  fisheye_render.id_ = 0;
  fisheye_render.lens_.type_ = LensParam::kFisheyeEqualArea;
  fisheye_render.lens_.fov_ = 180.0f;
  fisheye_render.resolution_[0] = 64;
  fisheye_render.resolution_[1] = 64;
  fisheye_render.view_.az_ = 0.0f;
  fisheye_render.view_.el_ = 90.0f;
  fisheye_render.view_.ro_ = 0.0f;
  fisheye_render.visible_ = RenderConfig::kUpper;
  constexpr float kWl = 550.0f;

  SessionSpec cpu_spec;
  cpu_spec.scene = &scene;
  cpu_spec.render = &fisheye_render;
  cpu_spec.wl = WlParam{ kWl, 1.0f };
  cpu_spec.seed = 42;

  constexpr size_t kRayCount = 4096;
  HostRayBatch host;
  host.count = kRayCount;
  host.crystal = nullptr;
  host.refractive_index = 0.0f;

  int fw = fisheye_render.resolution_[0];
  int fh = fisheye_render.resolution_[1];
  auto fpix = static_cast<size_t>(fw) * static_cast<size_t>(fh);
  std::vector<float> xyz_cpu(fpix * 3, 0.0f);

  CpuTraceBackend cpu;
  cpu.BeginSession(cpu_spec);
  cpu.TraceLayer(RootRaySource::FromHost(host));
  XyzImageData cpu_img{ xyz_cpu.data(), fw, fh };
  cpu.ReadbackImage(cpu_img);
  float direct_total = cpu.TotalLandedWeight();
  cpu.EndSession();

  int wl_key = static_cast<int>(kWl + 0.5f);
  ASSERT_GE(wl_key, kCmfMinWavelength);
  ASSERT_LE(wl_key, kCmfMaxWavelength);
  float cie_y = kCmfY[wl_key - kCmfMinWavelength];
  ASSERT_GT(cie_y, 1e-7f);

  double sum_y_cpu = 0.0;
  for (size_t i = 0; i < fpix; i++) {
    sum_y_cpu += xyz_cpu[i * 3 + 1];
  }
  auto formula_total = static_cast<float>(sum_y_cpu / cie_y);

  ASSERT_GT(direct_total, 0.0f) << "CpuTraceBackend produced no landed rays";
  float rel = std::abs(formula_total - direct_total) / direct_total;
  // The formula is a mathematical identity (SpectrumToXyz accumulates cie_y*w per ray,
  // TotalLandedWeight() accumulates w per ray independently). Float summation order
  // differences give rel ≤ 1e-4 in practice.
  EXPECT_LE(rel, 1e-3f) << "sum_Y / cie_y diverges from TotalLandedWeight() — formula semantic error"
                        << " formula=" << formula_total << " direct=" << direct_total << " rel=" << rel;

  // Part 2: Metal formula quantification.
  //
  // Run Metal and CpuTraceBackend on the same kRectangular scene and compare
  // total_landed_weight (sum_Y / cie_y) for both.
  //
  // Architectural note: Metal v1 uses an equirectangular projection covering
  // 360° of longitude, while CPU kRectangular clips to the configured FOV.
  // Metal therefore captures a larger fraction of exit rays as "in-bounds" and
  // consistently produces total_landed_weight > CPU. This is a known geometric
  // difference between the two backends, not a formula bug. The EV
  // normalization pipeline compensates per-session so final rendered brightness
  // converges given sufficient wavelength samples; the per-wavelength ratio is
  // not required to be 1.0. rel ≤ 5e-4 parity is achievable only if both
  // backends use the same projection geometry, which is a Metal v2 milestone.
  //
  // This test verifies: (a) Metal formula is non-zero/finite, and (b) Metal
  // captures a plausible fraction of rays relative to CPU (0.5x–5x).
  if (!ShouldSkipMetalTests()) {
    auto metal_render = MakeRectangularRender();
    int mw = metal_render.resolution_[0];
    int mh = metal_render.resolution_[1];
    auto mpix = static_cast<size_t>(mw) * static_cast<size_t>(mh);
    std::vector<float> xyz_metal(mpix * 3, 0.0f);
    std::vector<float> xyz_cpu_rect(mpix * 3, 0.0f);

    {
      SessionSpec metal_spec;
      metal_spec.scene = &scene;
      metal_spec.render = &metal_render;
      metal_spec.wl = WlParam{ kWl, 1.0f };
      metal_spec.seed = 42;

      MetalTraceBackend metal;
      metal.BeginSession(metal_spec);
      metal.TraceLayer(RootRaySource::FromHost(host));
      XyzImageData metal_img{ xyz_metal.data(), mw, mh };
      metal.ReadbackImage(metal_img);
      metal.EndSession();
    }
    {
      SessionSpec cpu_rect_spec;
      cpu_rect_spec.scene = &scene;
      cpu_rect_spec.render = &metal_render;
      cpu_rect_spec.wl = WlParam{ kWl, 1.0f };
      cpu_rect_spec.seed = 42;

      CpuTraceBackend cpu_rect;
      cpu_rect.BeginSession(cpu_rect_spec);
      cpu_rect.TraceLayer(RootRaySource::FromHost(host));
      float cpu_rect_total = cpu_rect.TotalLandedWeight();
      cpu_rect.EndSession();

      double sum_y_metal = 0.0;
      for (size_t i = 0; i < mpix; i++) {
        sum_y_metal += xyz_metal[i * 3 + 1];
      }
      auto metal_total = static_cast<float>(sum_y_metal / cie_y);

      ASSERT_GT(metal_total, 0.0f) << "Metal sum_Y / cie_y is zero — no rays landed";
      EXPECT_FALSE(std::isnan(metal_total)) << "Metal sum_Y / cie_y is NaN";
      EXPECT_FALSE(std::isinf(metal_total)) << "Metal sum_Y / cie_y is Inf";
      ASSERT_GT(cpu_rect_total, 0.0f) << "CpuTraceBackend (rect) produced no landed rays";

      // Both backends trace the same scene; ratio > 1 is expected due to
      // Metal's broader equirectangular projection (see note above).
      float ratio = metal_total / cpu_rect_total;
      EXPECT_GT(ratio, 0.5f) << "Metal captures far fewer rays than CPU — unexpected";
      EXPECT_LT(ratio, 5.0f) << "Metal captures far more rays than CPU — unexpected";
    }
  }
}

// Pearson correlation of the Y (luminance) channel between two XYZ images.
// Scale-invariant, so it is robust to the magnitude differences between
// backends (CpuTraceBackend fans out per hit and clips to FOV; the Metal v1
// kernel single-paths and covers 360° longitude — see Test E's note). What it
// IS sensitive to is the SPATIAL distribution: a world-space halo ring vs a
// crystal-local horizontal band correlate very differently. This is the
// spatial-aware metric the ChannelSum-based oracle tests lack (scrum.md §1.17).
double YChannelCorrelation(const std::vector<float>& a, const std::vector<float>& b) {
  assert(a.size() == b.size() && "YChannelCorrelation: images must have equal size");
  size_t n = a.size() / 3;
  double ma = 0.0;
  double mb = 0.0;
  for (size_t i = 0; i < n; i++) {
    ma += a[i * 3 + 1];
    mb += b[i * 3 + 1];
  }
  ma /= static_cast<double>(n);
  mb /= static_cast<double>(n);
  double num = 0.0;
  double da = 0.0;
  double db = 0.0;
  for (size_t i = 0; i < n; i++) {
    double x = a[i * 3 + 1] - ma;
    double y = b[i * 3 + 1] - mb;
    num += x * y;
    da += x * x;
    db += y * y;
  }
  return num / (std::sqrt(da * db) + 1e-30);
}

// =============================================================================
// Test J — single-layer SPATIAL parity (frame-correctness harness, scrum 253.1)
//
// The independent oracle is the real CpuTraceBackend (world-space projection),
// NOT the kernel-mirror OracleTraceLayer — the mirror shares the Metal kernel's
// crystal-local projection blind spot (scrum.md §1.17, a02). The metric is a
// per-pixel Y-channel correlation, spatially sensitive enough to tell a halo
// ring from a horizontal band.
//
//   - Positive arm: Metal (world-space, after the 253.1 frame fix) must
//     correlate highly with the CpuTraceBackend oracle — same ring structure.
//   - Negative arm (AC4): the crystal-LOCAL projection (OracleTraceLayer +
//     ScatterOutgoingToXyz, which is exactly the pre-253.1 Metal behaviour)
//     must correlate FAR LOWER with the world-space oracle. This proves the
//     harness is not spatially blind: had Metal stayed local-frame, it FAILS.
//
// Uses a full-random crystal orientation (mirroring metal_smoke.json) so the
// scene actually forms a halo ring; with a fixed orientation local vs world
// differ only by a global rotation and the band/ring contrast vanishes.
// =============================================================================
TEST(MetalTraceParity, MetalVsCpuSingleLayerSpatialStructure) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }

  auto scene = MakeMetalScene(/*max_hits=*/8, /*ms_layers=*/1);
  // Full-random orientation (azimuth / zenith / roll uniform 360°) so a real
  // halo ring forms — mirrors examples-style metal_smoke.json.
  auto& axis = scene.ms_[0].setting_[0].crystal_.axis_;
  axis.azimuth_dist = Distribution{ DistributionType::kUniform, 0.0f, 360.0f };
  axis.latitude_dist = Distribution{ DistributionType::kUniform, 0.0f, 360.0f };
  axis.roll_dist = Distribution{ DistributionType::kUniform, 0.0f, 360.0f };

  auto render = MakeRectangularRender();
  render.resolution_[0] = 128;
  render.resolution_[1] = 64;

  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = 42;

  constexpr size_t kRayCount = 262144;
  HostRayBatch host;
  host.count = kRayCount;
  host.crystal = nullptr;
  host.refractive_index = 0.0f;

  int w = render.resolution_[0];
  int h = render.resolution_[1];
  auto pix = static_cast<size_t>(w) * static_cast<size_t>(h);

  // --- Metal (world-space, after the 253.1 frame fix) ---
  std::vector<float> xyz_metal(pix * 3, 0.0f);
  {
    MetalTraceBackend metal;
    metal.BeginSession(spec);
    metal.TraceLayer(RootRaySource::FromHost(host));
    XyzImageData img{ xyz_metal.data(), w, h };
    metal.ReadbackImage(img);
    metal.EndSession();
  }

  // --- CpuTraceBackend (independent world-space oracle) ---
  std::vector<float> xyz_cpu(pix * 3, 0.0f);
  {
    CpuTraceBackend cpu;
    cpu.BeginSession(spec);
    cpu.TraceLayer(RootRaySource::FromHost(host));
    XyzImageData img{ xyz_cpu.data(), w, h };
    cpu.ReadbackImage(img);
    cpu.EndSession();
  }

  // --- Negative control: crystal-LOCAL projection (== pre-253.1 Metal) ---
  // OracleTraceLayer traces in the crystal-local frame and never applies
  // crystal_rot_; ScatterOutgoingToXyz then projects those local-frame
  // directions directly — reproducing exactly the band the frame bug produced.
  // Originally planned as option (b) test-only identity-matrix injection;
  // replaced by reusing OracleTraceLayer per the 2026-06-08 DECISION in
  // progress.md (a02 independent oracle + a04 zero production change).
  RandomNumberGenerator oracle_rng(0);
  SeedRngsLikeMetal(oracle_rng, spec.seed);
  auto roots = BuildFirstLayerRoots(oracle_rng, spec, kRayCount);
  auto poly = BuildPolyArrays(roots.crystal);
  auto oracle =
      OracleTraceLayer(roots.crystal, poly, roots.n_idx, scene.max_hits_, roots.d, roots.p, roots.w, roots.tf);
  std::vector<float> xyz_local(pix * 3, 0.0f);
  Rotation camera_rot = MakeCameraRotation(render);
  ScatterOutgoingToXyz(oracle.exit_d.data(), oracle.exit_w.data(), oracle.exit_w.size(), render, camera_rot,
                       spec.wl.wl_, xyz_local.data());

  double corr_fix = YChannelCorrelation(xyz_metal, xyz_cpu);
  double corr_broken = YChannelCorrelation(xyz_local, xyz_cpu);
  std::cout << "[MEASURE] corr_fix(metal_world vs cpu_world)=" << corr_fix
            << "  corr_broken(local vs cpu_world)=" << corr_broken << "  margin=" << (corr_fix - corr_broken)
            << std::endl;

  // Positive: world-space Metal matches the independent CPU oracle's ring.
  // Measured corr_fix ≈ 0.9999, corr_broken ≈ 0.028 (margin ≈ 0.97) on an
  // M-series device; thresholds sit far below/above to tolerate RNG-path noise
  // and are never relaxed to pass (red line). The production CLI path (2M rays,
  // many crystals) measured ≈ 0.92 — still well clear of 0.80.
  EXPECT_GT(corr_fix, 0.80) << "Metal does not structurally match the CpuTraceBackend world-space oracle";

  // Negative (AC4): the crystal-local projection must collapse correlation.
  // If this ever passes the positive threshold, the harness has gone spatially
  // blind and the frame regression would slip through.
  EXPECT_LT(corr_broken, 0.60) << "local-frame projection correlates too highly — harness may be spatially blind";
  EXPECT_GT(corr_fix - corr_broken, 0.20)
      << "fix/broken separation too small — metric not discriminating frame correctness";
}

// =============================================================================
// Test K — MULTI-LAYER spatial parity (multi-MS frame-transit harness, 253.2)
//
// Validates the 253.2 inter-layer frame transit (kernel continuation world-
// return + host resample-to-new-local) against the real CpuTraceBackend as the
// independent world-space oracle — NOT the kernel-mirror OracleTraceLayer,
// whose local-frame re-entry (OracleReentryRoots, no world-transit / no
// resample) is exactly the pre-253.2 frame bug and is therefore retired as a
// multi-MS structural baseline (see TwoLayer* tests). Two MS layers (layer 0
// prob 0.6 → continuation, layer 1 accumulate). Spatial Y-channel correlation;
// if the inter-layer transit were wrong (continuation stuck in the previous
// layer's local frame), the second layer's contribution would scatter and the
// correlation would collapse.
// =============================================================================
TEST(MetalTraceParity, MetalVsCpuMultiLayerSpatialStructure) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }

  auto scene = MakeMetalScene(/*max_hits=*/6, /*ms_layers=*/2);
  // Full-random orientation on BOTH layers so a real halo ring forms and the
  // inter-layer frame transit is genuinely exercised.
  for (auto& ms : scene.ms_) {
    auto& axis = ms.setting_[0].crystal_.axis_;
    axis.azimuth_dist = Distribution{ DistributionType::kUniform, 0.0f, 360.0f };
    axis.latitude_dist = Distribution{ DistributionType::kUniform, 0.0f, 360.0f };
    axis.roll_dist = Distribution{ DistributionType::kUniform, 0.0f, 360.0f };
  }

  auto render = MakeRectangularRender();
  render.resolution_[0] = 128;
  render.resolution_[1] = 64;

  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = 7;

  constexpr size_t kRayCount = 262144;
  HostRayBatch host;
  host.count = kRayCount;
  host.crystal = nullptr;
  host.refractive_index = 0.0f;

  int w = render.resolution_[0];
  int h = render.resolution_[1];
  auto pix = static_cast<size_t>(w) * static_cast<size_t>(h);

  // Drive a 2-layer session through the TraceBackend seam (host → layer0 →
  // Recombine → layer1 → readback). shuffle=false: Metal v1 requires it and the
  // CPU oracle stays comparable.
  auto run_two_layer = [&](auto& be, std::vector<float>& xyz) {
    be.BeginSession(spec);
    auto h0 = be.TraceLayer(RootRaySource::FromHost(host));
    RecombineSpec rspec;
    rspec.shuffle = false;
    auto roots1 = be.Recombine(std::move(h0), rspec);
    auto h1 = be.TraceLayer(roots1);
    // h1 (the final layer) is kept alive until after ReadbackImage so the
    // accumulated XYZ image is fully populated; ReadbackImage reads the
    // session accumulator (independent of the handle), and EndSession releases
    // session state. The handle destructs at lambda scope end — after readback.
    XyzImageData img{ xyz.data(), w, h };
    be.ReadbackImage(img);
    be.EndSession();
    (void)h1;
  };

  std::vector<float> xyz_metal(pix * 3, 0.0f);
  std::vector<float> xyz_cpu(pix * 3, 0.0f);
  {
    MetalTraceBackend metal;
    run_two_layer(metal, xyz_metal);
  }
  {
    CpuTraceBackend cpu;
    run_two_layer(cpu, xyz_cpu);
  }

  double corr = YChannelCorrelation(xyz_metal, xyz_cpu);
  std::cout << "[MEASURE] multi-MS corr(metal_world vs cpu_world)=" << corr << std::endl;

  // Threshold calibrated from measurement: corr ≈ 0.997 (stable across runs:
  // 0.997003 / 0.996988). Held at 0.95 — ample margin below the observed value
  // yet tight enough to catch a partial inter-layer frame regression (e.g. a
  // transpose sign flip or a single-ci-branch error that would still clear a
  // looser 0.80 bar). Never relaxed to pass (red line).
  EXPECT_GT(corr, 0.95) << "Metal multi-MS does not structurally match the CpuTraceBackend world-space oracle"
                        << " — inter-layer frame transit likely wrong";
  // shuffle=false: Metal v1 requires it (asserts !shuffle in Recombine). The
  // shuffle=true Recombine path is NOT covered here — add coverage once Metal's
  // device-side shuffle is implemented.
}

// =============================================================================
// Dual-fisheye equal-area parity (task-metal-gui-default Step 5)
//
// Validates the kernel's new proj_type==1 path against the production CPU
// projection (projection::FisheyeEqualAreaForward + DualFisheyeToPixel +
// render.cpp:192-214 overlap dual-write, all reached via the public
// ScatterOutgoingToXyz helper in core/scatter_accum.hpp). Per scrum-253 review
// guidance (memory 253.1) the CPU side runs the REAL production code path —
// no test-local re-implementation of the dual-fisheye math.
// =============================================================================
using metal_test::MakeDualFisheyeEARender;

TEST(MetalTraceParity, DualFisheyeEA_Single_NoOverlap) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }
  ForceHostGenForByteIdentity();

  auto scene = MakeMetalScene(/*max_hits=*/8, /*ms_layers=*/1);
  auto render = MakeDualFisheyeEARender(/*overlap=*/0.0f);

  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = 42;

  constexpr size_t kRayCount = 4096;
  HostRayBatch host;
  host.count = kRayCount;
  host.crystal = nullptr;
  host.refractive_index = 0.0f;

  std::vector<float> xyz_metal(render.resolution_[0] * render.resolution_[1] * 3, 0.0f);
  LayerStats metal_stats;
  {
    MetalTraceBackend metal;
    metal.BeginSession(spec);
    auto h = metal.TraceLayer(RootRaySource::FromHost(host));
    ASSERT_NE(h, nullptr);
    metal_stats = h->GetLayerStats();
    XyzImageData img{ xyz_metal.data(), render.resolution_[0], render.resolution_[1] };
    metal.ReadbackImage(img);
    metal.EndSession();
  }

  RandomNumberGenerator oracle_rng(0);
  SeedRngsLikeMetal(oracle_rng, spec.seed);
  auto roots = BuildFirstLayerRoots(oracle_rng, spec, kRayCount);
  auto poly = BuildPolyArrays(roots.crystal);
  auto oracle =
      OracleTraceLayer(roots.crystal, poly, roots.n_idx, scene.max_hits_, roots.d, roots.p, roots.w, roots.tf);

  // Project oracle exit rays through the REAL CPU projection (production
  // ScatterOutgoingToXyz reaches FisheyeEqualAreaForward + DualFisheyeToPixel).
  std::vector<float> xyz_oracle(render.resolution_[0] * render.resolution_[1] * 3, 0.0f);
  Rotation camera_rot = MakeCameraRotation(render);
  ScatterOutgoingToXyz(oracle.exit_d.data(), oracle.exit_w.data(), oracle.exit_w.size(), render, camera_rot,
                       spec.wl.wl_, xyz_oracle.data());

  EXPECT_EQ(metal_stats.exit_count, oracle.exit_count)
      << "exit_count mismatch — metal=" << metal_stats.exit_count << " oracle=" << oracle.exit_count;
  EXPECT_LT(RelErr(metal_stats.exit_w_sum, oracle.exit_w_sum), 5e-4)
      << "exit_w_sum: metal=" << metal_stats.exit_w_sum << " oracle=" << oracle.exit_w_sum;

  for (int c = 0; c < 3; c++) {
    double sm = ChannelSum(xyz_metal, c);
    double so = ChannelSum(xyz_oracle, c);
    EXPECT_LT(RelErr(sm, so), 5e-4) << "channel=" << c << " metal=" << sm << " oracle=" << so;
  }
}

TEST(MetalTraceParity, DualFisheyeEA_Single_WithOverlap) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }
  ForceHostGenForByteIdentity();

  // overlap = sin(5°) ≈ kDualFisheyeOverlap (gui_state.hpp:135) — matches the
  // exact value the GUI live-preview path commits via c_api.cpp.
  auto scene = MakeMetalScene(/*max_hits=*/8, /*ms_layers=*/1);
  auto render = MakeDualFisheyeEARender(/*overlap=*/0.0872f);

  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = 42;

  constexpr size_t kRayCount = 4096;
  HostRayBatch host;
  host.count = kRayCount;
  host.crystal = nullptr;
  host.refractive_index = 0.0f;

  std::vector<float> xyz_metal(render.resolution_[0] * render.resolution_[1] * 3, 0.0f);
  LayerStats metal_stats;
  {
    MetalTraceBackend metal;
    metal.BeginSession(spec);
    auto h = metal.TraceLayer(RootRaySource::FromHost(host));
    ASSERT_NE(h, nullptr);
    metal_stats = h->GetLayerStats();
    XyzImageData img{ xyz_metal.data(), render.resolution_[0], render.resolution_[1] };
    metal.ReadbackImage(img);
    metal.EndSession();
  }

  RandomNumberGenerator oracle_rng(0);
  SeedRngsLikeMetal(oracle_rng, spec.seed);
  auto roots = BuildFirstLayerRoots(oracle_rng, spec, kRayCount);
  auto poly = BuildPolyArrays(roots.crystal);
  auto oracle =
      OracleTraceLayer(roots.crystal, poly, roots.n_idx, scene.max_hits_, roots.d, roots.p, roots.w, roots.tf);

  std::vector<float> xyz_oracle(render.resolution_[0] * render.resolution_[1] * 3, 0.0f);
  Rotation camera_rot = MakeCameraRotation(render);
  ScatterOutgoingToXyz(oracle.exit_d.data(), oracle.exit_w.data(), oracle.exit_w.size(), render, camera_rot,
                       spec.wl.wl_, xyz_oracle.data());

  // exit_count and exit_w_sum are accumulated per-exit-event in the kernel
  // (a single increment) — the overlap dual-write writes only to image, NOT
  // to exit_wsum (matches CPU render.cpp:217 "Pass 2 does NOT update
  // total_intensity_"). The strict equality here pins that invariant: if the
  // kernel ever double-counts overlap rays into exit_wsum, parity breaks.
  EXPECT_EQ(metal_stats.exit_count, oracle.exit_count)
      << "exit_count mismatch — metal=" << metal_stats.exit_count << " oracle=" << oracle.exit_count;
  EXPECT_LT(RelErr(metal_stats.exit_w_sum, oracle.exit_w_sum), 5e-4)
      << "exit_w_sum: metal=" << metal_stats.exit_w_sum << " oracle=" << oracle.exit_w_sum;

  for (int c = 0; c < 3; c++) {
    double sm = ChannelSum(xyz_metal, c);
    double so = ChannelSum(xyz_oracle, c);
    EXPECT_LT(RelErr(sm, so), 5e-4) << "channel=" << c << " metal=" << sm << " oracle=" << so;
  }
}

TEST(MetalTraceParity, DualFisheyeEA_MultiMS_WithOverlap) {
  // GUI live-preview commonly drives multi-MS scenes; this case validates the
  // dual-fisheye-EA kernel + inter-layer frame transit (253.2) together,
  // against the production CpuTraceBackend on the same config.
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }

  auto scene = MakeMetalScene(/*max_hits=*/6, /*ms_layers=*/2);
  for (auto& ms : scene.ms_) {
    auto& axis = ms.setting_[0].crystal_.axis_;
    axis.azimuth_dist = Distribution{ DistributionType::kUniform, 0.0f, 360.0f };
    axis.latitude_dist = Distribution{ DistributionType::kUniform, 0.0f, 360.0f };
    axis.roll_dist = Distribution{ DistributionType::kUniform, 0.0f, 360.0f };
  }
  auto render = MakeDualFisheyeEARender(/*overlap=*/0.0872f);

  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = 7;

  constexpr size_t kRayCount = 262144;
  HostRayBatch host;
  host.count = kRayCount;
  host.crystal = nullptr;
  host.refractive_index = 0.0f;

  int w = render.resolution_[0];
  int h = render.resolution_[1];
  auto pix = static_cast<size_t>(w) * static_cast<size_t>(h);

  auto run_two_layer = [&](auto& be, std::vector<float>& xyz) {
    be.BeginSession(spec);
    auto h0 = be.TraceLayer(RootRaySource::FromHost(host));
    RecombineSpec rspec;
    rspec.shuffle = false;
    auto roots1 = be.Recombine(std::move(h0), rspec);
    auto h1 = be.TraceLayer(roots1);
    XyzImageData img{ xyz.data(), w, h };
    be.ReadbackImage(img);
    be.EndSession();
    (void)h1;
  };

  std::vector<float> xyz_metal(pix * 3, 0.0f);
  std::vector<float> xyz_cpu(pix * 3, 0.0f);
  {
    MetalTraceBackend metal;
    run_two_layer(metal, xyz_metal);
  }
  {
    CpuTraceBackend cpu;
    run_two_layer(cpu, xyz_cpu);
  }

  double corr = YChannelCorrelation(xyz_metal, xyz_cpu);
  std::cout << "[MEASURE] dual-fisheye-EA multi-MS corr(metal vs cpu)=" << corr << std::endl;

  // Held at 0.95 (same threshold as MetalVsCpuMultiLayerSpatialStructure).
  // Dual-fisheye-EA has no transcendentals (only sqrt) so parity should be
  // tighter than rectangular, but the bar stays at 0.95 to detect partial
  // regressions while tolerating sampling noise at 262144 rays.
  EXPECT_GT(corr, 0.95) << "Metal dual-fisheye-EA multi-MS does not structurally match CpuTraceBackend";
}

// =============================================================================
// Per-ray wavelength parity gates (task-270.7 / explore-269 P0).
//
// scrum-268.8 introduced per-ray wavelength sampling via WlPool / D65; before
// this gate the parity/golden suite kept spec.wl = WlParam{550} for every
// Metal test, so a regression that fed all rays the wrong wavelength (or
// dropped the per-ray wl_idx lookup on a fallback path) would not surface in
// CI. The two assertions below cover both arms of the wl_pool contract:
//
//  - SingleWlParityAndCmfChannelRatios (oracle parity at 450 + 650 nm, plus
//    CMF channel-ratio sanity): verifies the discrete-wavelength fallback
//    populates the pool with the correct per-ray (n_idx, cmf_*) entry. If
//    the kernel ever reads the wrong wl_idx slot, oracle parity breaks; if
//    CMF lookup is ever frozen at a single wavelength (e.g. 550), the
//    Z-channel ratio between 450 nm and 650 nm collapses (CMF_Z(450) ≈
//    1.77, CMF_Z(650) ≈ 0 — a > 1000× ratio).
//
//  - D65IlluminantModeBlendsPerRayWavelengths (D65 statistical-mixing gate):
//    verifies the illuminant path actually samples M wavelengths across
//    [380, 780] nm per ray. If the path were ever to degrade to a single
//    wavelength (e.g. only 650 nm), Z-channel light vanishes. The test
//    internally re-runs a 650 nm single-wl baseline (no cross-TEST state) and
//    asserts D65 Z-channel >> 650 nm Z-channel.
//
// Self-check (manual): swap the 650 nm baseline to WlParam{450} (which has a
// large CMF_Z) — the D65 vs 650-baseline ratio assertion goes red, proving
// the gate actually depends on per-ray wavelength sampling rather than just
// "any non-zero Z light".
// =============================================================================
TEST(MetalTraceParity, SingleWlParityAndCmfChannelRatios) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }
  // Byte-identity oracle: mirror the host mt19937 stream (Metal device-gen
  // PCG cannot align bit-for-bit).
  ForceHostGenForByteIdentity();

  auto scene = MakeMetalScene(/*max_hits=*/8, /*ms_layers=*/1);
  auto render = MakeRectangularRender();

  constexpr size_t kRayCount = 4096;
  HostRayBatch host;
  host.count = kRayCount;
  host.crystal = nullptr;
  host.refractive_index = 0.0f;

  auto run_one_wl = [&](float wl_nm, uint32_t seed, std::vector<float>& xyz_metal_out,
                        std::vector<float>& xyz_oracle_out, LayerStats& metal_stats_out,
                        OracleLayerResult& oracle_out) {
    SessionSpec spec;
    spec.scene = &scene;
    spec.render = &render;
    spec.wl = WlParam{ wl_nm, 1.0f };
    spec.seed = seed;

    xyz_metal_out.assign(render.resolution_[0] * render.resolution_[1] * 3, 0.0f);
    {
      MetalTraceBackend metal;
      metal.BeginSession(spec);
      auto h = metal.TraceLayer(RootRaySource::FromHost(host));
      ASSERT_NE(h, nullptr);
      metal_stats_out = h->GetLayerStats();
      XyzImageData img{ xyz_metal_out.data(), render.resolution_[0], render.resolution_[1] };
      metal.ReadbackImage(img);
      metal.EndSession();
    }

    RandomNumberGenerator oracle_rng(0);
    SeedRngsLikeMetal(oracle_rng, spec.seed);
    auto roots = BuildFirstLayerRoots(oracle_rng, spec, kRayCount);
    auto poly = BuildPolyArrays(roots.crystal);
    oracle_out =
        OracleTraceLayer(roots.crystal, poly, roots.n_idx, scene.max_hits_, roots.d, roots.p, roots.w, roots.tf);

    xyz_oracle_out.assign(render.resolution_[0] * render.resolution_[1] * 3, 0.0f);
    Rotation camera_rot = MakeCameraRotation(render);
    ScatterOutgoingToXyz(oracle_out.exit_d.data(), oracle_out.exit_w.data(), oracle_out.exit_w.size(), render,
                         camera_rot, spec.wl.wl_, xyz_oracle_out.data());
  };

  // --- 450 nm arm ---
  std::vector<float> xyz_metal_450;
  std::vector<float> xyz_oracle_450;
  LayerStats metal_stats_450;
  OracleLayerResult oracle_450;
  ASSERT_NO_FATAL_FAILURE(run_one_wl(450.0f, /*seed=*/42, xyz_metal_450, xyz_oracle_450, metal_stats_450, oracle_450));

  // --- 650 nm arm ---
  std::vector<float> xyz_metal_650;
  std::vector<float> xyz_oracle_650;
  LayerStats metal_stats_650;
  OracleLayerResult oracle_650;
  ASSERT_NO_FATAL_FAILURE(run_one_wl(650.0f, /*seed=*/42, xyz_metal_650, xyz_oracle_650, metal_stats_650, oracle_650));

  // Per-wavelength oracle parity — the byte-identity test catches a regression
  // that feeds the kernel the wrong wl_idx (e.g. all rays read pool slot 0).
  EXPECT_EQ(metal_stats_450.exit_count, oracle_450.exit_count) << "450 nm exit_count mismatch";
  EXPECT_LT(RelErr(metal_stats_450.exit_w_sum, oracle_450.exit_w_sum), 5e-4)
      << "450 nm exit_w_sum: metal=" << metal_stats_450.exit_w_sum << " oracle=" << oracle_450.exit_w_sum;
  EXPECT_EQ(metal_stats_650.exit_count, oracle_650.exit_count) << "650 nm exit_count mismatch";
  EXPECT_LT(RelErr(metal_stats_650.exit_w_sum, oracle_650.exit_w_sum), 5e-4)
      << "650 nm exit_w_sum: metal=" << metal_stats_650.exit_w_sum << " oracle=" << oracle_650.exit_w_sum;
  for (int c = 0; c < 3; c++) {
    double sm_450 = ChannelSum(xyz_metal_450, c);
    double so_450 = ChannelSum(xyz_oracle_450, c);
    EXPECT_LT(RelErr(sm_450, so_450), 5e-4) << "450 nm channel=" << c << " metal=" << sm_450 << " oracle=" << so_450;
    double sm_650 = ChannelSum(xyz_metal_650, c);
    double so_650 = ChannelSum(xyz_oracle_650, c);
    EXPECT_LT(RelErr(sm_650, so_650), 5e-4) << "650 nm channel=" << c << " metal=" << sm_650 << " oracle=" << so_650;
  }

  // CMF channel-ratio sanity — the per-wavelength CMF lookup must actually
  // differ across wl. CMF_Z(450) ≈ 1.77, CMF_Z(650) ≈ 0 (kCmfZ table), so the
  // Z-channel sum at 450 nm must be vastly larger than at 650 nm. A regression
  // that froze CMF at a single wavelength (e.g. always wl_pool[0]) would
  // collapse this ratio. 100× is a deliberately conservative floor: measured
  // ratio is several orders of magnitude.
  double z_450 = ChannelSum(xyz_metal_450, 2);
  double z_650 = ChannelSum(xyz_metal_650, 2);
  std::cout << "[MEASURE] per-wl CMF Z: Z_450=" << z_450 << " Z_650=" << z_650 << " ratio=" << (z_450 / (z_650 + 1e-12))
            << std::endl;
  EXPECT_GT(z_450, 100.0 * (z_650 + 1e-12))
      << "Z-channel ratio between 450 nm and 650 nm collapsed — per-ray CMF lookup may be frozen at a single wl"
      << " Z_450=" << z_450 << " Z_650=" << z_650;
}

TEST(MetalTraceParity, D65IlluminantModeBlendsPerRayWavelengths) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }

  // larger sample count to suppress D65 per-ray sampling noise (M=64 pool ×
  // PCG wl_idx). Empirically the D65 vs 650-baseline Z ratio is in the
  // hundreds; 8192 rays put the standard deviation comfortably below the
  // floor used by the assertion.
  constexpr size_t kRayCount = 8192;
  auto render = MakeRectangularRender();
  HostRayBatch host;
  host.count = kRayCount;
  host.crystal = nullptr;
  host.refractive_index = 0.0f;

  // --- D65 illuminant arm: per-ray wavelength sampled from D65 SPD ---
  auto scene_d65 = MakeMetalScene(/*max_hits=*/8, /*ms_layers=*/1);
  scene_d65.light_source_.spectrum_ = IlluminantType::kD65;
  std::vector<float> xyz_metal_d65(render.resolution_[0] * render.resolution_[1] * 3, 0.0f);
  {
    // Device-gen PCG path: D65 path validates the production routing where
    // simulator.cpp:672 passes a zero-wl WlParam and the backend samples wl
    // per ray on device. ForceHostGenForByteIdentity (left over from the
    // previous test) would force the host root-gen path; reset it here so
    // this test exercises the production single-worker route.
    EnableDeviceGenForStatisticalParity();

    SessionSpec spec;
    spec.scene = &scene_d65;
    spec.render = &render;
    spec.wl = WlParam{};  // matches simulator.cpp:674 (zero-wl sentinel)
    spec.seed = 42;

    MetalTraceBackend metal;
    metal.BeginSession(spec);
    auto h = metal.TraceLayer(RootRaySource::FromHost(host));
    ASSERT_NE(h, nullptr);
    XyzImageData img{ xyz_metal_d65.data(), render.resolution_[0], render.resolution_[1] };
    metal.ReadbackImage(img);
    metal.EndSession();
  }

  // --- Single-wl 650 nm baseline arm: same scene shape, Z-channel ≈ 0 ---
  auto scene_650 = MakeMetalScene(/*max_hits=*/8, /*ms_layers=*/1);
  std::vector<float> xyz_metal_650(render.resolution_[0] * render.resolution_[1] * 3, 0.0f);
  {
    EnableDeviceGenForStatisticalParity();  // stay on production path

    SessionSpec spec;
    spec.scene = &scene_650;
    spec.render = &render;
    spec.wl = WlParam{ 650.0f, 1.0f };
    spec.seed = 42;

    MetalTraceBackend metal;
    metal.BeginSession(spec);
    auto h = metal.TraceLayer(RootRaySource::FromHost(host));
    ASSERT_NE(h, nullptr);
    XyzImageData img{ xyz_metal_650.data(), render.resolution_[0], render.resolution_[1] };
    metal.ReadbackImage(img);
    metal.EndSession();
  }

  double x_d65 = ChannelSum(xyz_metal_d65, 0);
  double y_d65 = ChannelSum(xyz_metal_d65, 1);
  double z_d65 = ChannelSum(xyz_metal_d65, 2);
  double z_650 = ChannelSum(xyz_metal_650, 2);
  std::cout << "[MEASURE] D65 vs 650 nm baseline: X_D65=" << x_d65 << " Y_D65=" << y_d65 << " Z_D65=" << z_d65
            << " Z_650=" << z_650 << " ratio=" << (z_d65 / (z_650 + 1e-12)) << std::endl;

  // D65 must light all three channels — a degenerate single-wl regression
  // (e.g. only 650 nm sampled) collapses Z to ≈ 0.
  EXPECT_GT(x_d65, 0.0) << "D65 produced no X-channel light — per-ray wavelength sampling may be broken";
  EXPECT_GT(y_d65, 0.0) << "D65 produced no Y-channel light — per-ray wavelength sampling may be broken";
  EXPECT_GT(z_d65, 0.0) << "D65 produced no Z-channel light — per-ray wavelength sampling may be broken";

  // D65 Z-channel must vastly exceed pure-650-nm Z-channel because D65
  // includes substantial blue (450 nm CMF_Z ≈ 1.77) while CMF_Z(650) ≈ 0.
  // Floor (50×) is conservative — measured value is hundreds of × — but
  // tight enough that "the kernel collapsed to a single red wavelength"
  // regression goes red.
  EXPECT_GT(z_d65, 50.0 * (z_650 + 1e-12))
      << "D65 Z-channel did not exceed 650-nm-only Z-channel by ≥ 50× — per-ray wavelength sampling may be"
      << " collapsing onto a single wl. Z_D65=" << z_d65 << " Z_650=" << z_650;
}

}  // namespace
}  // namespace lumice

#endif  // defined(__APPLE__)
