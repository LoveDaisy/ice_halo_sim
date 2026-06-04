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
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <vector>

#include "config/render_config.hpp"
#include "config/sim_data.hpp"
#include "core/crystal.hpp"
#include "core/def.hpp"
#include "core/geo3d.hpp"
#include "core/math.hpp"
#include "core/metal_trace_backend.hpp"
#include "core/scatter_accum.hpp"
#include "core/trace_backend.hpp"
#include "core/trace_ops.hpp"
#include "metal_test_helpers.hpp"

namespace lumice {
namespace {

using metal_test::ChannelSum;
using metal_test::MakeMetalScene;
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

// Apply argmax-facing + centroid re-entry to a batch of exit rays. Mirrors
// the kernel's ms_mode==1 branch where exits become the next layer's roots.
// NOLINTNEXTLINE(readability-function-size)
void OracleReentryRoots(const PolyArrays& poly, size_t poly_cnt, const std::vector<float>& exit_d,
                        const std::vector<float>& exit_w, std::vector<float>& out_d, std::vector<float>& out_p,
                        std::vector<float>& out_w, std::vector<IdType>& out_tf) {
  size_t n = exit_w.size();
  out_d.assign(exit_d.begin(), exit_d.end());
  out_w.assign(exit_w.begin(), exit_w.end());
  out_p.assign(n * 3, 0.0f);
  out_tf.assign(n, 0);
  for (size_t i = 0; i < n; i++) {
    float dx = exit_d[i * 3 + 0];
    float dy = exit_d[i * 3 + 1];
    float dz = exit_d[i * 3 + 2];
    float inv_len = 1.0f / std::sqrt(dx * dx + dy * dy + dz * dz);
    float ux = dx * inv_len;
    float uy = dy * inv_len;
    float uz = dz * inv_len;
    int ef = 0;
    float best = -1e30f;
    for (size_t fi = 0; fi < poly_cnt; fi++) {
      float facing = -(ux * poly.n[fi * 3 + 0] + uy * poly.n[fi * 3 + 1] + uz * poly.n[fi * 3 + 2]);
      if (facing > best) {
        best = facing;
        ef = static_cast<int>(fi);
      }
    }
    out_p[i * 3 + 0] = poly.centroid[ef * 3 + 0];
    out_p[i * 3 + 1] = poly.centroid[ef * 3 + 1];
    out_p[i * 3 + 2] = poly.centroid[ef * 3 + 2];
    out_tf[i] = static_cast<IdType>(ef);
  }
}

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
  ASSERT_NE(seed, 0u) << "seed=0 not supported by oracle (RNG state unknown)";
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

  EXPECT_EQ(metal_cont0, oracle_l0.exit_count) << "layer0 continuation_count vs oracle exit_count mismatch";
  EXPECT_EQ(metal_layer0_stats.exit_count, oracle_l0.exit_count) << "layer0 exit_count mismatch";
  EXPECT_LT(RelErr(metal_layer0_stats.exit_w_sum, oracle_l0.exit_w_sum), 5e-4)
      << "layer0 exit_w_sum: metal=" << metal_layer0_stats.exit_w_sum << " oracle=" << oracle_l0.exit_w_sum;

  // Re-entry: argmax-facing + centroid (matches kernel ms_mode==1 branch).
  std::vector<float> next_d;
  std::vector<float> next_p;
  std::vector<float> next_w;
  std::vector<IdType> next_tf;
  OracleReentryRoots(poly0, roots0.crystal.PolygonFaceCount(), oracle_l0.exit_d, oracle_l0.exit_w, next_d, next_p,
                     next_w, next_tf);

  // Layer 1: same RNG consumption as Metal (ResolveLayerCrystal calls
  // MakeCrystal on every non-first layer regardless of host crystal).
  const auto& setting1 = scene.ms_[1].setting_[0];
  Crystal crystal1 = MakeCrystal(oracle_rng, setting1.crystal_.param_);
  float n_idx1 = crystal1.GetRefractiveIndex(spec.wl.wl_);
  auto poly1 = BuildPolyArrays(crystal1);

  auto oracle_l1 = OracleTraceLayer(crystal1, poly1, n_idx1, scene.max_hits_, next_d, next_p, next_w, next_tf);

  EXPECT_EQ(metal_layer1_stats.exit_count, oracle_l1.exit_count) << "layer1 exit_count mismatch";
  EXPECT_LT(RelErr(metal_layer1_stats.exit_w_sum, oracle_l1.exit_w_sum), 5e-4)
      << "layer1 exit_w_sum: metal=" << metal_layer1_stats.exit_w_sum << " oracle=" << oracle_l1.exit_w_sum;

  // Final XYZ comparison.
  std::vector<float> xyz_oracle(render.resolution_[0] * render.resolution_[1] * 3, 0.0f);
  Rotation camera_rot = MakeCameraRotation(render);
  ScatterOutgoingToXyz(oracle_l1.exit_d.data(), oracle_l1.exit_w.data(), oracle_l1.exit_w.size(), render, camera_rot,
                       spec.wl.wl_, xyz_oracle.data());
  for (int c = 0; c < 3; c++) {
    double sm = ChannelSum(xyz_metal, c);
    double so = ChannelSum(xyz_oracle, c);
    EXPECT_LT(RelErr(sm, so), 5e-4) << "channel=" << c << " metal=" << sm << " oracle=" << so;
  }
}

// =============================================================================
// Test I — deep-recorder single-layer parity (max_hits=16)
// =============================================================================
TEST(MetalTraceParity, DeepRecorderSingleLayer) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }

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

}  // namespace
}  // namespace lumice

#endif  // defined(__APPLE__)
