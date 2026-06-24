// Golden analytic anchors for the CPU trace backend.
//
// Drives CpuTraceBackend with analytically-constructed rays via the
// HostRayBatch::d/p/w/tf injection path (wired by task-backend-scaffold-and-
// golden-anchor, scrum-cuda-backend-mvp subtask 1) and compares the backend's
// exit rays against closed-form physics expectations — back-reflection /
// forward-transmission directions, Fresnel weight chain, and total energy
// bound.
//
// Mirrors the four MetalGoldenRay anchors in test_multi_ms_golden.cpp but
// runs cross-platform (no Apple guard) so the analytic battery covers CPU
// even on non-Metal hosts. Together with test_projection.cpp's 38
// projection-forward anchors, the `golden-analytic` label now satisfies AC4
// for cuda-backend-mvp subtask 1: reflection direction, refraction direction
// (Snell), Fresnel reflection ratio, and projection forward transform.
//
// Anchors here are INDEPENDENT of the kernel's Fresnel formula — the expected
// values are derived from the textbook Snell + rs/rp formulas in
// FresnelTransmitDelta(), not from the backend's internal parameterisation.
// This independence is what distinguishes absolute anchors from parity tests.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "core/backend/cpu_trace_backend.hpp"
#include "core/backend/trace_backend.hpp"
#include "core/crystal.hpp"
#include "core/def.hpp"
#include "core/math.hpp"
#include "core/projection.hpp"
#include "core/trace_ops.hpp"
#include "cpu_test_helpers.hpp"

namespace lumice {
namespace {

using cpu_test::MakeCpuScene;
using cpu_test::MakeRectangularRender;

struct PolyArrays {
  std::vector<float> n;  // 3 * poly_cnt
};

PolyArrays BuildPolyArrays(const Crystal& crystal) {
  size_t poly_cnt = crystal.PolygonFaceCount();
  PolyArrays out;
  out.n.assign(crystal.GetPolygonFaceNormal(), crystal.GetPolygonFaceNormal() + poly_cnt * 3);
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

// Textbook Fresnel transmittance — derived directly from Snell + rs/rp
// amplitude coefficients. INDEPENDENT of the trace kernel's parameterisation
// (which is the whole point of the golden anchor).
// `rr = n_incident / n_transmit`; `cos_theta_abs = |cosθ_incidence|`.
float FresnelTransmitDelta(float cos_theta_abs, float rr) {
  float sin_i2 = std::max(0.0f, 1.0f - cos_theta_abs * cos_theta_abs);
  float sin_t2 = rr * rr * sin_i2;
  if (sin_t2 >= 1.0f) {
    return 0.0f;  // TIR
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

// Build a SessionSpec + crystal for the parallel-slab anchors, mirroring the
// MetalGoldenRay setup so the CPU and Metal anchors form a side-by-side
// absolute battery.
struct SlabFixture {
  SceneConfig scene;
  RenderConfig render;
  SessionSpec spec;
  Crystal crystal;
  float n_idx;
  PrismFaces faces;
};

SlabFixture BuildSlabFixture(size_t max_hits) {
  SlabFixture f;
  f.scene = MakeCpuScene(/*max_hits=*/max_hits, /*ms_layers=*/1);
  f.render = MakeRectangularRender();
  f.spec.scene = &f.scene;
  f.spec.render = &f.render;
  f.spec.wl = WlParam{ 550.0f, 1.0f };
  f.spec.seed = 42;

  RandomNumberGenerator local_rng(0);
  local_rng.SetSeed(f.spec.seed);
  f.crystal = MakeCrystal(local_rng, f.scene.ms_[0].setting_[0].crystal_.param_);
  f.n_idx = f.crystal.GetRefractiveIndex(f.spec.wl.wl_);

  auto poly = BuildPolyArrays(f.crystal);
  size_t poly_cnt = f.crystal.PolygonFaceCount();
  f.faces = FindTopBotFaces(poly, poly_cnt);
  return f;
}

}  // namespace

// =============================================================================
// CpuGoldenRay_NormalIncidence — vertical ray through a parallel slab
//
// Validates: (a) back-reflect direction = (0,0,+1) with weight = R = ((n-1)/
// (n+1))², (b) forward-transmit direction = (0,0,-1) with weight = T² where
// T = 1-R. Both assertions use textbook closed-form values independent of the
// trace kernel.
// =============================================================================
TEST(CpuGoldenRay, NormalIncidenceParallelSlab) {
  auto f = BuildSlabFixture(/*max_hits=*/2);
  ASSERT_NE(f.faces.top, kInvalidId) << "top face (normal ≈ +z) not found";
  ASSERT_NE(f.faces.bot, kInvalidId) << "bottom face (normal ≈ -z) not found";

  std::vector<float> d = { 0.0f, 0.0f, -1.0f };
  std::vector<float> p = { 0.0f, 0.0f, 0.5f };
  std::vector<float> w = { 1.0f };
  std::vector<IdType> tf = { f.faces.top };

  HostRayBatch host;
  host.count = 1;
  host.d = d.data();
  host.p = p.data();
  host.w = w.data();
  host.tf = tf.data();
  host.crystal = &f.crystal;
  host.refractive_index = f.n_idx;
  host.crystal_id = 0;

  std::vector<ExitRayRecord> exits;
  size_t exit_count = 0;
  {
    CpuTraceBackend cpu;
    cpu.BeginSession(f.spec);
    auto h = cpu.TraceLayer(RootRaySource::FromHost(host));
    ASSERT_NE(h, nullptr);
    exit_count = cpu.ReadbackExitRays(exits);
    cpu.EndSession();
  }

  ASSERT_GE(exit_count, 2u) << "normal-incidence must emit ≥ 2 exits (reflect-at-entry + transmit-out)";

  float r_normal = (f.n_idx - 1.0f) / (f.n_idx + 1.0f);
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
// CpuGoldenRay_Snell30 — oblique ray through a parallel slab preserves
// direction; weights match the textbook Fresnel chain (independent formula).
// =============================================================================
TEST(CpuGoldenRay, Snell30ParallelSlab) {
  auto f = BuildSlabFixture(/*max_hits=*/4);
  ASSERT_NE(f.faces.top, kInvalidId);
  ASSERT_NE(f.faces.bot, kInvalidId);

  constexpr float kTheta = 30.0f * 3.14159265358979323846f / 180.0f;
  float sin_t = std::sin(kTheta);
  float cos_t = std::cos(kTheta);
  std::vector<float> d = { sin_t, 0.0f, -cos_t };
  std::vector<float> p = { 0.0f, 0.0f, 0.5f };
  std::vector<float> w = { 1.0f };
  std::vector<IdType> tf = { f.faces.top };

  HostRayBatch host;
  host.count = 1;
  host.d = d.data();
  host.p = p.data();
  host.w = w.data();
  host.tf = tf.data();
  host.crystal = &f.crystal;
  host.refractive_index = f.n_idx;
  host.crystal_id = 0;

  std::vector<ExitRayRecord> exits;
  size_t exit_count = 0;
  {
    CpuTraceBackend cpu;
    cpu.BeginSession(f.spec);
    auto h = cpu.TraceLayer(RootRaySource::FromHost(host));
    ASSERT_NE(h, nullptr);
    exit_count = cpu.ReadbackExitRays(exits);
    cpu.EndSession();
  }

  ASSERT_GE(exit_count, 2u) << "Snell 30° must emit ≥ 2 exits (reflect-at-entry + transmit-out)";

  float sin_inside = sin_t / f.n_idx;
  float cos_inside = std::sqrt(std::max(0.0f, 1.0f - sin_inside * sin_inside));
  float t_in = FresnelTransmitDelta(cos_t, 1.0f / f.n_idx);
  float t_out = FresnelTransmitDelta(cos_inside, f.n_idx);
  float r_in = 1.0f - t_in;

  // Reflection at entry: direction = (sin30, 0, +cos30), weight = r_in.
  int idx_up = FindExitByDirection(exits, sin_t, 0.0f, cos_t);
  ASSERT_GE(idx_up, 0) << "no exit found near (sin30, 0, +cos30)";
  EXPECT_NEAR(exits[idx_up].weight, r_in, 5e-4f)
      << "reflect-at-entry weight: got=" << exits[idx_up].weight << " expected=" << r_in;

  // Forward transmit: parallel slab preserves direction (sin30, 0, -cos30);
  // weight = t_in · t_out.
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
// CpuGoldenRay_EnergyConservation — Σw_exit ≤ w_initial, never grows.
//
// Coarser than the directional anchors above but catches the failure mode of
// the kernel synthesising energy (e.g. double-counting a continuation ray as
// an exit) — which corr-only parity oracles previously masked (scrum-267).
// =============================================================================
TEST(CpuGoldenRay, EnergyConservationSingleRay) {
  auto f = BuildSlabFixture(/*max_hits=*/8);
  ASSERT_NE(f.faces.top, kInvalidId);

  constexpr float kTheta = 45.0f * 3.14159265358979323846f / 180.0f;
  float sin_t = std::sin(kTheta);
  float cos_t = std::cos(kTheta);
  std::vector<float> d = { sin_t, 0.0f, -cos_t };
  std::vector<float> p = { 0.0f, 0.0f, 0.5f };
  std::vector<float> w = { 1.0f };
  std::vector<IdType> tf = { f.faces.top };

  HostRayBatch host;
  host.count = 1;
  host.d = d.data();
  host.p = p.data();
  host.w = w.data();
  host.tf = tf.data();
  host.crystal = &f.crystal;
  host.refractive_index = f.n_idx;
  host.crystal_id = 0;

  std::vector<ExitRayRecord> exits;
  size_t exit_count = 0;
  {
    CpuTraceBackend cpu;
    cpu.BeginSession(f.spec);
    auto h = cpu.TraceLayer(RootRaySource::FromHost(host));
    ASSERT_NE(h, nullptr);
    exit_count = cpu.ReadbackExitRays(exits);
    cpu.EndSession();
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
// CpuGoldenRay_ProjectionForwardKnownDir — closed-form projection coord anchor
//
// Independent absolute anchor: pass a known direction through the project
// forward transform and assert the (x, y) coordinate against a hand-derived
// expectation. AC4's "一种投影正变换坐标" requirement; complements the trace-
// based anchors above with a pure-math anchor on the projection function the
// consumer hands the exit rays.
//
// Rectangular projection convention (src/core/projection.cpp:139): given a
// unit-length direction (dx, dy, dz), the forward transform yields
//   x = atan2(dy, dx)        (longitude / azimuth, radians)
//   y = asin(clamp(dz,-1,1)) (latitude / elevation, radians)
// `valid = true` always (no behind-camera rejection — that is LinearForward's
// behavior, exercised separately in test_projection.cpp).
// =============================================================================
TEST(CpuGoldenRay, ProjectionForwardKnownDir) {
  // Direction at azimuth 0°, elevation 45°: (dx, dy, dz) = (√2/2, 0, √2/2).
  constexpr float kSqrt2Over2 = 0.70710678118f;
  constexpr float kPiOver4 = 0.78539816339f;

  auto r = projection::RectangularForward(kSqrt2Over2, 0.0f, kSqrt2Over2);
  ASSERT_TRUE(r.valid);
  EXPECT_NEAR(r.x, 0.0f, 1e-5f) << "azimuth atan2(0, √2/2) = 0";
  EXPECT_NEAR(r.y, kPiOver4, 5e-5f) << "elevation asin(√2/2) = π/4";

  // Second known-dir anchor: nadir direction (0, 0, -1) maps to (lon=0 by
  // atan2 convention, lat=-π/2). Lon is undefined at the pole; atan2(0, 0)
  // returns 0 by C++ contract.
  constexpr float kPiOver2 = 1.57079632679f;
  auto r_nadir = projection::RectangularForward(0.0f, 0.0f, -1.0f);
  ASSERT_TRUE(r_nadir.valid);
  EXPECT_NEAR(r_nadir.x, 0.0f, 1e-5f);
  EXPECT_NEAR(r_nadir.y, -kPiOver2, 1e-5f) << "south pole maps to lat = -π/2";
}

}  // namespace lumice
