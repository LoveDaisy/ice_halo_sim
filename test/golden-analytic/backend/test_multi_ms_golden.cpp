// Golden analytic anchors for the Metal trace kernel.
//
// These tests drive the real MetalTraceBackend with analytically constructed
// rays via the test-only HostRayBatch::d/p/w/tf injection path and compare
// the kernel's exit rays against closed-form physics expectations — Snell's
// law for parallel-slab refraction, Fresnel weights, total energy bound, and
// the 2-MS continuation analytic anchor.
//
// Orthogonal to the OracleTraceLayer mirror (which catches "kernel does X,
// oracle does Y" divergence in parity tests): these anchors catch the
// shared-formula bug class where CPU mirror AND kernel implement the same
// wrong formula.
//
// Extracted from test_metal_trace_parity.cpp following the testing-architecture.md
// §1.2 purpose-primary split. The first three anchors (NormalIncidence /
// Snell30 / EnergyConservation) validate the continuation engine;
// MultiMsContinuationNormalIncidence is the G3 anchor for the device-resident
// continuation path. See doc/testing-architecture.md §1.2 and §6.

#include <gtest/gtest.h>

#if defined(__APPLE__)

#include <algorithm>
#include <cmath>
#include <vector>

#include "core/backend/metal_trace_backend.hpp"
#include "core/backend/trace_backend.hpp"
#include "core/crystal.hpp"
#include "core/def.hpp"
#include "core/math.hpp"
#include "core/trace_ops.hpp"
#include "metal_test_helpers.hpp"

namespace lumice {
namespace {

using metal_test::ForceHostGenForByteIdentity;
using metal_test::MakeMetalScene;
using metal_test::MakeRectangularRender;
using metal_test::ShouldSkipMetalTests;

struct PolyArrays {
  std::vector<float> n;         // 3 * poly_cnt
  std::vector<float> d;         // poly_cnt
  std::vector<float> centroid;  // 3 * poly_cnt
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

struct PrismFaces {
  IdType top = kInvalidId;
  IdType bot = kInvalidId;
};

PrismFaces FindTopBotFaces(const PolyArrays& poly, size_t poly_cnt) {
  PrismFaces out;
  for (size_t fi = 0; fi < poly_cnt; fi++) {
    float nz = poly.n[fi * 3 + 2];
    if (nz > 0.9f) {
      out.top = static_cast<IdType>(fi);
    } else if (nz < -0.9f) {
      out.bot = static_cast<IdType>(fi);
    }
  }
  return out;
}

// Independent textbook Fresnel transmittance — derived directly from the Snell
// law + rs/rp amplitude coefficients, NOT the kernel's dd/GetReflectRatio
// parameterisation. This independence is the whole point of the golden WEIGHT
// anchor: a shared bug in the kernel's Fresnel formula must NOT be mirrored in
// the expected value, otherwise the weight assertion degrades into a tautology.
// `rr = n_incident / n_transmit`; `cos_theta_abs = |cosθ_incidence|`.
float FresnelTransmitDelta(float cos_theta_abs, float rr) {
  float sin_i2 = std::max(0.0f, 1.0f - cos_theta_abs * cos_theta_abs);
  float sin_t2 = rr * rr * sin_i2;
  if (sin_t2 >= 1.0f) {
    return 0.0f;  // TIR — full reflection
  }
  float cos_t = std::sqrt(1.0f - sin_t2);
  float rs = (rr * cos_theta_abs - cos_t) / (rr * cos_theta_abs + cos_t);
  float rp = (rr * cos_t - cos_theta_abs) / (rr * cos_t + cos_theta_abs);
  float reflectance = 0.5f * (rs * rs + rp * rp);
  return 1.0f - reflectance;
}

int FindExitByDirection(const std::vector<ExitRayRecord>& exits, float dx, float dy, float dz, float tol = 5e-3f) {
  int best = -1;
  float best_diff = tol;
  for (size_t i = 0; i < exits.size(); i++) {
    float diff = std::abs(exits[i].dir[0] - dx) + std::abs(exits[i].dir[1] - dy) + std::abs(exits[i].dir[2] - dz);
    if (diff < best_diff) {
      best_diff = diff;
      best = static_cast<int>(i);
    }
  }
  return best;
}

float SumWeightByDirection(const std::vector<ExitRayRecord>& exits, float dx, float dy, float dz,
                           float cos_tol = 0.9999f) {
  float sum = 0.0f;
  for (const auto& e : exits) {
    float dot = e.dir[0] * dx + e.dir[1] * dy + e.dir[2] * dz;
    if (dot >= cos_tol) {
      sum += e.weight;
    }
  }
  return sum;
}

}  // namespace

// =============================================================================
// GoldenRay_NormalIncidence — vertical ray through a parallel slab
// =============================================================================
TEST(MetalGoldenRay, NormalIncidenceParallelSlab) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }
  ForceHostGenForByteIdentity();

  auto scene = MakeMetalScene(/*max_hits=*/2, /*ms_layers=*/1);
  auto render = MakeRectangularRender();

  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = 42;

  RandomNumberGenerator local_rng(0);
  local_rng.SetSeed(spec.seed);
  Crystal crystal = MakeCrystal(local_rng, scene.ms_[0].setting_[0].crystal_.param_);
  float n_idx = crystal.GetRefractiveIndex(spec.wl.wl_);

  auto poly = BuildPolyArrays(crystal);
  size_t poly_cnt = crystal.PolygonFaceCount();
  PrismFaces faces = FindTopBotFaces(poly, poly_cnt);
  ASSERT_NE(faces.top, kInvalidId) << "top face (normal ≈ +z) not found";
  ASSERT_NE(faces.bot, kInvalidId) << "bottom face (normal ≈ -z) not found";

  std::vector<float> d = { 0.0f, 0.0f, -1.0f };
  std::vector<float> p = { 0.0f, 0.0f, 0.5f };
  std::vector<float> w = { 1.0f };
  std::vector<IdType> tf = { faces.top };

  HostRayBatch host;
  host.count = 1;
  host.d = d.data();
  host.p = p.data();
  host.w = w.data();
  host.tf = tf.data();
  host.crystal = &crystal;
  host.refractive_index = n_idx;
  host.crystal_id = 0;

  std::vector<ExitRayRecord> exits;
  size_t exit_count = 0;
  {
    MetalTraceBackend metal;
    metal.BeginSession(spec);
    auto h = metal.TraceLayer(RootRaySource::FromHost(host));
    ASSERT_NE(h, nullptr);
    exit_count = metal.ReadbackExitRays(exits);
    metal.EndSession();
  }

  ASSERT_GE(exit_count, 2u) << "normal-incidence must emit ≥ 2 exits (reflect-at-entry + transmit-out)";

  float r_normal = (n_idx - 1.0f) / (n_idx + 1.0f);
  r_normal *= r_normal;
  float t_normal = 1.0f - r_normal;

  int idx_up = FindExitByDirection(exits, 0.0f, 0.0f, 1.0f);
  ASSERT_GE(idx_up, 0) << "no exit found near direction (0,0,+1)";
  EXPECT_NEAR(exits[idx_up].dir[0], 0.0f, 1e-4f);
  EXPECT_NEAR(exits[idx_up].dir[1], 0.0f, 1e-4f);
  EXPECT_NEAR(exits[idx_up].dir[2], 1.0f, 1e-4f);
  EXPECT_NEAR(exits[idx_up].weight, r_normal, 5e-4f)
      << "back-reflect weight: got=" << exits[idx_up].weight << " expected=" << r_normal;

  int idx_down = FindExitByDirection(exits, 0.0f, 0.0f, -1.0f);
  ASSERT_GE(idx_down, 0) << "no exit found near direction (0,0,-1)";
  EXPECT_NEAR(exits[idx_down].dir[0], 0.0f, 1e-4f);
  EXPECT_NEAR(exits[idx_down].dir[1], 0.0f, 1e-4f);
  EXPECT_NEAR(exits[idx_down].dir[2], -1.0f, 1e-4f);
  EXPECT_NEAR(exits[idx_down].weight, t_normal * t_normal, 5e-4f)
      << "forward-transmit weight: got=" << exits[idx_down].weight << " expected=" << (t_normal * t_normal);
}

// =============================================================================
// GoldenRay_Snell30 — oblique ray through a parallel slab preserves direction
// =============================================================================
TEST(MetalGoldenRay, Snell30ParallelSlab) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }
  ForceHostGenForByteIdentity();

  auto scene = MakeMetalScene(/*max_hits=*/4, /*ms_layers=*/1);
  auto render = MakeRectangularRender();

  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = 42;

  RandomNumberGenerator local_rng(0);
  local_rng.SetSeed(spec.seed);
  Crystal crystal = MakeCrystal(local_rng, scene.ms_[0].setting_[0].crystal_.param_);
  float n_idx = crystal.GetRefractiveIndex(spec.wl.wl_);

  auto poly = BuildPolyArrays(crystal);
  size_t poly_cnt = crystal.PolygonFaceCount();
  PrismFaces faces = FindTopBotFaces(poly, poly_cnt);
  ASSERT_NE(faces.top, kInvalidId);
  ASSERT_NE(faces.bot, kInvalidId);

  constexpr float kTheta = 30.0f * 3.14159265358979323846f / 180.0f;
  float sin_t = std::sin(kTheta);
  float cos_t = std::cos(kTheta);
  std::vector<float> d = { sin_t, 0.0f, -cos_t };
  std::vector<float> p = { 0.0f, 0.0f, 0.5f };
  std::vector<float> w = { 1.0f };
  std::vector<IdType> tf = { faces.top };

  HostRayBatch host;
  host.count = 1;
  host.d = d.data();
  host.p = p.data();
  host.w = w.data();
  host.tf = tf.data();
  host.crystal = &crystal;
  host.refractive_index = n_idx;
  host.crystal_id = 0;

  std::vector<ExitRayRecord> exits;
  size_t exit_count = 0;
  {
    MetalTraceBackend metal;
    metal.BeginSession(spec);
    auto h = metal.TraceLayer(RootRaySource::FromHost(host));
    ASSERT_NE(h, nullptr);
    exit_count = metal.ReadbackExitRays(exits);
    metal.EndSession();
  }

  ASSERT_GE(exit_count, 2u) << "Snell 30° must emit ≥ 2 exits (reflect-at-entry + transmit-out)";

  float sin_inside = sin_t / n_idx;
  float cos_inside = std::sqrt(std::max(0.0f, 1.0f - sin_inside * sin_inside));
  float t_in = FresnelTransmitDelta(cos_t, 1.0f / n_idx);
  float t_out = FresnelTransmitDelta(cos_inside, n_idx);
  float r_in = 1.0f - t_in;

  int idx_up = FindExitByDirection(exits, sin_t, 0.0f, cos_t);
  ASSERT_GE(idx_up, 0) << "no exit found near (sin30, 0, +cos30)";
  EXPECT_NEAR(exits[idx_up].weight, r_in, 5e-4f)
      << "reflect-at-entry weight: got=" << exits[idx_up].weight << " expected=" << r_in;

  int idx_down = FindExitByDirection(exits, sin_t, 0.0f, -cos_t);
  ASSERT_GE(idx_down, 0) << "no exit found near (sin30, 0, -cos30)";
  EXPECT_NEAR(exits[idx_down].dir[0], sin_t, 1e-4f);
  EXPECT_NEAR(exits[idx_down].dir[1], 0.0f, 1e-4f);
  EXPECT_NEAR(exits[idx_down].dir[2], -cos_t, 1e-4f);
  EXPECT_NEAR(exits[idx_down].weight, t_in * t_out, 5e-4f)
      << "Snell 30° transmitted weight: got=" << exits[idx_down].weight << " expected=" << (t_in * t_out)
      << " (cos_in=" << cos_t << " cos_inside=" << cos_inside << ")";
}

// =============================================================================
// GoldenRay_EnergyConservation — Σw_exit ≤ w_initial, never grows
// =============================================================================
TEST(MetalGoldenRay, EnergyConservationSingleRay) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }
  ForceHostGenForByteIdentity();

  auto scene = MakeMetalScene(/*max_hits=*/8, /*ms_layers=*/1);
  auto render = MakeRectangularRender();

  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = 42;

  RandomNumberGenerator local_rng(0);
  local_rng.SetSeed(spec.seed);
  Crystal crystal = MakeCrystal(local_rng, scene.ms_[0].setting_[0].crystal_.param_);
  float n_idx = crystal.GetRefractiveIndex(spec.wl.wl_);

  auto poly = BuildPolyArrays(crystal);
  size_t poly_cnt = crystal.PolygonFaceCount();
  PrismFaces faces = FindTopBotFaces(poly, poly_cnt);
  ASSERT_NE(faces.top, kInvalidId);

  constexpr float kTheta = 45.0f * 3.14159265358979323846f / 180.0f;
  float sin_t = std::sin(kTheta);
  float cos_t = std::cos(kTheta);
  std::vector<float> d = { sin_t, 0.0f, -cos_t };
  std::vector<float> p = { 0.0f, 0.0f, 0.5f };
  std::vector<float> w = { 1.0f };
  std::vector<IdType> tf = { faces.top };

  HostRayBatch host;
  host.count = 1;
  host.d = d.data();
  host.p = p.data();
  host.w = w.data();
  host.tf = tf.data();
  host.crystal = &crystal;
  host.refractive_index = n_idx;
  host.crystal_id = 0;

  std::vector<ExitRayRecord> exits;
  size_t exit_count = 0;
  {
    MetalTraceBackend metal;
    metal.BeginSession(spec);
    auto h = metal.TraceLayer(RootRaySource::FromHost(host));
    ASSERT_NE(h, nullptr);
    exit_count = metal.ReadbackExitRays(exits);
    metal.EndSession();
  }

  ASSERT_GE(exit_count, 1u) << "ray must produce at least one exit (kernel never absorbs everything)";
  double sum_w = 0.0;
  for (size_t i = 0; i < exit_count; i++) {
    EXPECT_GE(exits[i].weight, 0.0f) << "exit weight must be non-negative (i=" << i << ")";
    sum_w += static_cast<double>(exits[i].weight);
  }
  EXPECT_LE(sum_w, 1.0 + 1e-4) << "energy gain: Σw_exit=" << sum_w << " > w_initial=1.0 (exit_count=" << exit_count
                               << ")";
}

// =============================================================================
// GoldenRay_MultiMsContinuationNormalIncidence — 2-MS straight-through anchor
//
// G3 (scrum-gpu-single-engine-orchestration): the first absolute analytic
// anchor on transit_root_kernel's device-resident continuation path. The three
// preceding MetalGoldenRay tests all stop at single-MS slab; multi-MS
// continuation correctness was previously gated only by legacy-relative corr,
// which Scrum-267 showed has masked real bugs (+16% energy, orientation
// undersampling). This test asserts the Fresnel weight chain over 4 face
// crossings against an INDEPENDENT analytic expectation (no legacy in the
// expected value).
//
// Analytic derivation: see header comment in the original location
// (test_metal_trace_parity.cpp pre-270.3). Expected sums:
//   Σw(0,0,-1) = T⁴ + R²        (dominant; T⁴ ≈ 0.923 for n≈1.31)
//   Σw(0,0,+1) = 2·R·T²         (≈ 0.0377)
//   total      = (T²+R)² < 1    (energy conservative — no gain across MS)
//
// IMPORTANT prob trap: ReadbackExitRays drops final-layer exits when
// rng < final_prob. prob=1.0 would drop EVERY final exit. layer-0 prob=1.0
// makes all layer-0 hits become continuation rays (no mid-exits); the final
// layer MUST stay at prob=0.0 (the MakeMetalScene default for the last MS).
// =============================================================================
TEST(MetalGoldenRay, MultiMsContinuationNormalIncidence) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }
  ForceHostGenForByteIdentity();

  auto scene = MakeMetalScene(/*max_hits=*/2, /*ms_layers=*/2);
  scene.ms_[0].prob_ = 1.0f;
  auto render = MakeRectangularRender();

  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = 42;

  RandomNumberGenerator local_rng(0);
  local_rng.SetSeed(spec.seed);
  Crystal crystal = MakeCrystal(local_rng, scene.ms_[0].setting_[0].crystal_.param_);
  float n_idx = crystal.GetRefractiveIndex(spec.wl.wl_);

  auto poly = BuildPolyArrays(crystal);
  size_t poly_cnt = crystal.PolygonFaceCount();
  PrismFaces faces = FindTopBotFaces(poly, poly_cnt);
  ASSERT_NE(faces.top, kInvalidId) << "top face not found";
  ASSERT_NE(faces.bot, kInvalidId) << "bottom face not found";

  std::vector<float> d = { 0.0f, 0.0f, -1.0f };
  std::vector<float> p = { 0.0f, 0.0f, 0.5f };
  std::vector<float> w = { 1.0f };
  std::vector<IdType> tf = { faces.top };

  HostRayBatch host;
  host.count = 1;
  host.d = d.data();
  host.p = p.data();
  host.w = w.data();
  host.tf = tf.data();
  host.crystal = &crystal;
  host.refractive_index = n_idx;
  host.crystal_id = 0;

  float r_normal = (n_idx - 1.0f) / (n_idx + 1.0f);
  r_normal *= r_normal;
  float t_normal = 1.0f - r_normal;

  std::vector<ExitRayRecord> exits;
  size_t exit_count = 0;
  {
    MetalTraceBackend metal;
    metal.BeginSession(spec);

    auto h0 = metal.TraceLayer(RootRaySource::FromHost(host));
    ASSERT_NE(h0, nullptr);
    ASSERT_GE(h0->ContinuationCount(), 1u) << "layer-0 must produce at least 1 continuation ray (prob=1.0, no filter)";

    RecombineSpec rspec;
    rspec.shuffle = false;
    auto roots1 = metal.Recombine(std::move(h0), rspec);

    auto h1 = metal.TraceLayer(roots1);
    ASSERT_NE(h1, nullptr);

    exit_count = metal.ReadbackExitRays(exits);
    metal.EndSession();
  }

  ASSERT_GE(exit_count, 2u) << "2-MS continuation must produce ≥ 2 exits "
                               "(≥1 downward from B-transmit + ≥1 upward from A-transmit)";

  float expected_down = t_normal * t_normal * t_normal * t_normal + r_normal * r_normal;
  float sum_down = SumWeightByDirection(exits, 0.0f, 0.0f, -1.0f);
  EXPECT_NEAR(sum_down, expected_down, 5e-4f) << "Σw(0,0,-1) = T⁴+R²: got=" << sum_down << " expected=" << expected_down
                                              << " (n_idx=" << n_idx << " T=" << t_normal << " R=" << r_normal << ")";

  float expected_up = 2.0f * r_normal * t_normal * t_normal;
  float sum_up = SumWeightByDirection(exits, 0.0f, 0.0f, +1.0f);
  EXPECT_NEAR(sum_up, expected_up, 5e-4f) << "Σw(0,0,+1) = 2RT²: got=" << sum_up << " expected=" << expected_up;

  double total = 0.0;
  for (size_t i = 0; i < exit_count; i++) {
    EXPECT_GE(exits[i].weight, 0.0f) << "negative weight at i=" << i;
    total += static_cast<double>(exits[i].weight);
  }
  EXPECT_LE(total, 1.0 + 1e-4) << "energy gain: Σw_exits=" << total << " > 1.0 (exit_count=" << exit_count << ")";
}

}  // namespace lumice

#else  // !defined(__APPLE__)

// Non-Apple platforms: no Metal backend, no analytic anchors on Metal.
// Provide a placeholder so the empty TU compiles cleanly under all toolchains.

#endif  // defined(__APPLE__)
