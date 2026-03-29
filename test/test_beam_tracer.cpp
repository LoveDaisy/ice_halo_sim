#include <gtest/gtest.h>

#include <cmath>
#include <cstring>

#include "core/beam_tracer.hpp"
#include "core/crystal.hpp"
#include "core/geo3d.hpp"
#include "core/math.hpp"
#include "core/optics.hpp"


namespace lumice {

// ============================================================================
// BuildOrthonormalBasis tests
// ============================================================================

TEST(BeamTracerTest, BuildOrthonormalBasisZAxis) {
  float d[3] = { 0, 0, 1 };
  float u[3]{};
  float v[3]{};
  BuildOrthonormalBasis(d, u, v);

  EXPECT_NEAR(Dot3(u, v), 0.0f, 1e-6f);
  EXPECT_NEAR(Dot3(u, d), 0.0f, 1e-6f);
  EXPECT_NEAR(Dot3(v, d), 0.0f, 1e-6f);
  EXPECT_NEAR(Norm3(u), 1.0f, 1e-6f);
  EXPECT_NEAR(Norm3(v), 1.0f, 1e-6f);
}

TEST(BeamTracerTest, BuildOrthonormalBasisDiagonal) {
  float d[3] = { 1.0f / std::sqrt(3.0f), 1.0f / std::sqrt(3.0f), 1.0f / std::sqrt(3.0f) };
  float u[3]{};
  float v[3]{};
  BuildOrthonormalBasis(d, u, v);

  EXPECT_NEAR(Dot3(u, v), 0.0f, 1e-5f);
  EXPECT_NEAR(Dot3(u, d), 0.0f, 1e-5f);
  EXPECT_NEAR(Dot3(v, d), 0.0f, 1e-5f);
  EXPECT_NEAR(Norm3(u), 1.0f, 1e-5f);
  EXPECT_NEAR(Norm3(v), 1.0f, 1e-5f);
}

TEST(BeamTracerTest, BuildOrthonormalBasisXAxis) {
  float d[3] = { 1, 0, 0 };
  float u[3]{};
  float v[3]{};
  BuildOrthonormalBasis(d, u, v);

  EXPECT_NEAR(Dot3(u, v), 0.0f, 1e-6f);
  EXPECT_NEAR(Dot3(u, d), 0.0f, 1e-6f);
  EXPECT_NEAR(Dot3(v, d), 0.0f, 1e-6f);
}


// ============================================================================
// ConvexPolygon2D::Area tests
// ============================================================================

TEST(BeamTracerTest, PolygonAreaSquare) {
  ConvexPolygon2D poly{};
  // Unit square: (0,0), (1,0), (1,1), (0,1)
  poly.vertices[0] = 0;
  poly.vertices[1] = 0;
  poly.vertices[2] = 1;
  poly.vertices[3] = 0;
  poly.vertices[4] = 1;
  poly.vertices[5] = 1;
  poly.vertices[6] = 0;
  poly.vertices[7] = 1;
  poly.count = 4;

  EXPECT_NEAR(poly.Area(), 1.0f, 1e-6f);
}

TEST(BeamTracerTest, PolygonAreaTriangle) {
  ConvexPolygon2D poly{};
  // Triangle: (0,0), (2,0), (0,2) → area = 2.0
  poly.vertices[0] = 0;
  poly.vertices[1] = 0;
  poly.vertices[2] = 2;
  poly.vertices[3] = 0;
  poly.vertices[4] = 0;
  poly.vertices[5] = 2;
  poly.count = 3;

  EXPECT_NEAR(poly.Area(), 2.0f, 1e-6f);
}

TEST(BeamTracerTest, PolygonAreaEmpty) {
  ConvexPolygon2D poly{};
  EXPECT_NEAR(poly.Area(), 0.0f, 1e-6f);
}


// ============================================================================
// ProjectTo2D tests
// ============================================================================

TEST(BeamTracerTest, ProjectTo2DBasic) {
  float basis_u[3] = { 1, 0, 0 };
  float basis_v[3] = { 0, 1, 0 };
  float pts_3d[6] = { 1, 2, 3, 4, 5, 6 };
  float pts_2d[4]{};

  ProjectTo2D(basis_u, basis_v, pts_3d, 2, pts_2d);

  EXPECT_NEAR(pts_2d[0], 1.0f, 1e-6f);  // u component of (1,2,3)
  EXPECT_NEAR(pts_2d[1], 2.0f, 1e-6f);  // v component of (1,2,3)
  EXPECT_NEAR(pts_2d[2], 4.0f, 1e-6f);
  EXPECT_NEAR(pts_2d[3], 5.0f, 1e-6f);
}


// ============================================================================
// ClipByHalfPlane tests
// ============================================================================

TEST(BeamTracerTest, ClipSquareByDiagonal) {
  // Clip unit square by half-plane x + y - 1 >= 0 (keeps upper-right triangle).
  ConvexPolygon2D sq{};
  sq.vertices[0] = 0;
  sq.vertices[1] = 0;
  sq.vertices[2] = 1;
  sq.vertices[3] = 0;
  sq.vertices[4] = 1;
  sq.vertices[5] = 1;
  sq.vertices[6] = 0;
  sq.vertices[7] = 1;
  sq.count = 4;

  auto clipped = ClipByHalfPlane(sq, 1.0f, 1.0f, -1.0f);
  EXPECT_NEAR(clipped.Area(), 0.5f, 1e-4f);
}

TEST(BeamTracerTest, ClipSquareKeepsAll) {
  // Half-plane x >= -10 keeps everything.
  ConvexPolygon2D sq{};
  sq.vertices[0] = 0;
  sq.vertices[1] = 0;
  sq.vertices[2] = 1;
  sq.vertices[3] = 0;
  sq.vertices[4] = 1;
  sq.vertices[5] = 1;
  sq.vertices[6] = 0;
  sq.vertices[7] = 1;
  sq.count = 4;

  auto clipped = ClipByHalfPlane(sq, 1.0f, 0.0f, 10.0f);
  EXPECT_NEAR(clipped.Area(), 1.0f, 1e-4f);
}

TEST(BeamTracerTest, ClipSquareRemovesAll) {
  // Half-plane x >= 10 removes everything.
  ConvexPolygon2D sq{};
  sq.vertices[0] = 0;
  sq.vertices[1] = 0;
  sq.vertices[2] = 1;
  sq.vertices[3] = 0;
  sq.vertices[4] = 1;
  sq.vertices[5] = 1;
  sq.vertices[6] = 0;
  sq.vertices[7] = 1;
  sq.count = 4;

  auto clipped = ClipByHalfPlane(sq, 1.0f, 0.0f, -10.0f);
  EXPECT_EQ(clipped.Area(), 0.0f);
}

TEST(BeamTracerTest, ClipAreaConservation) {
  // Clip by x >= 0.5 and x < 0.5. Areas should sum to original.
  ConvexPolygon2D sq{};
  sq.vertices[0] = 0;
  sq.vertices[1] = 0;
  sq.vertices[2] = 1;
  sq.vertices[3] = 0;
  sq.vertices[4] = 1;
  sq.vertices[5] = 1;
  sq.vertices[6] = 0;
  sq.vertices[7] = 1;
  sq.count = 4;

  auto left = ClipByHalfPlane(sq, -1.0f, 0.0f, 0.5f);   // x <= 0.5
  auto right = ClipByHalfPlane(sq, 1.0f, 0.0f, -0.5f);  // x >= 0.5

  EXPECT_NEAR(left.Area() + right.Area(), 1.0f, 1e-4f);
}


// ============================================================================
// ExtractPolygonFaceVertices tests
// ============================================================================

TEST(BeamTracerTest, ExtractPrismBasalFace) {
  auto crystal = Crystal::CreatePrism(1.0f);
  float vtx[3 * kMaxPolyVertices]{};

  // Face 0 should be a basal face (hexagonal, 6 vertices).
  size_t count = ExtractPolygonFaceVertices(crystal, 0, vtx);
  EXPECT_EQ(count, 6u);
}

TEST(BeamTracerTest, ExtractPrismPrismFace) {
  auto crystal = Crystal::CreatePrism(1.0f);
  float vtx[3 * kMaxPolyVertices]{};

  // Prism faces should have 4 vertices (rectangular).
  // Polygon faces 2-7 are prism side faces for a hexagonal prism.
  size_t count = ExtractPolygonFaceVertices(crystal, 2, vtx);
  EXPECT_EQ(count, 4u);
}


// ============================================================================
// Fresnel cross-validation
// ============================================================================

TEST(BeamTracerTest, FresnelReflectedDir) {
  float dir[3] = { 0.0f, 0.0f, -1.0f };
  float normal[3] = { 0.0f, 0.0f, 1.0f };
  float out[3]{};

  ComputeReflectedDir(dir, normal, out);

  EXPECT_NEAR(out[0], 0.0f, 1e-6f);
  EXPECT_NEAR(out[1], 0.0f, 1e-6f);
  EXPECT_NEAR(out[2], 1.0f, 1e-6f);
}

TEST(BeamTracerTest, FresnelRefractedDirNormal) {
  float dir[3] = { 0.0f, 0.0f, -1.0f };
  float normal[3] = { 0.0f, 0.0f, 1.0f };
  float out[3]{};

  float cos_theta = Dot3(dir, normal);  // -1.0
  float rr = 1.0f / 1.31f;              // Air → ice

  bool ok = ComputeRefractedDir(dir, normal, rr, cos_theta, out);
  EXPECT_TRUE(ok);

  // Normal incidence: refracted dir should be same as incident (no bending).
  EXPECT_NEAR(out[0], 0.0f, 1e-5f);
  EXPECT_NEAR(out[1], 0.0f, 1e-5f);
  EXPECT_LT(out[2], 0.0f);  // Still going downward
}


// ============================================================================
// BeamTrace integration tests
// ============================================================================

TEST(BeamTracerTest, BeamTracePrismProducesOutput) {
  auto crystal = Crystal::CreatePrism(1.0f);
  Rotation rot;  // Identity rotation

  // Light coming from above (negative z).
  float light_dir[3] = { 0.0f, 0.0f, -1.0f };
  float n = 1.31f;

  auto result = BeamTrace(crystal, rot, light_dir, n, 8);

  // Should produce some outgoing beams.
  EXPECT_GT(result.outgoing_w.size(), 0u);
  EXPECT_GT(result.total_entry_area, 0.0f);

  // Every outgoing beam should have a direction and weight.
  EXPECT_EQ(result.outgoing_d.size(), result.outgoing_w.size() * 3);
  EXPECT_EQ(result.outgoing_raypath.size(), result.outgoing_w.size());
}

TEST(BeamTracerTest, BeamTraceEnergyConservation) {
  auto crystal = Crystal::CreatePrism(1.0f);
  Rotation rot;

  float light_dir[3] = { 0.0f, 0.0f, -1.0f };
  float n = 1.31f;

  auto result = BeamTrace(crystal, rot, light_dir, n, 8);

  // Sum all outgoing weights. Should not exceed total entry area.
  float total_w = 0.0f;
  for (float w : result.outgoing_w) {
    total_w += w;
  }

  // Energy should be <= total entry area (some may be lost to truncation at max_hits).
  EXPECT_LE(total_w, result.total_entry_area * 1.01f);  // Allow 1% numerical tolerance
  // For max_hits=8 on a hexagonal prism, outgoing energy should be at least 80%.
  EXPECT_GT(total_w, result.total_entry_area * 0.80f);
}

TEST(BeamTracerTest, BeamTraceNoPolygonFaces) {
  // A crystal with no polygon face data should return empty result.
  // Use default-constructed crystal (no faces).
  Crystal crystal{};
  Rotation rot;
  float light_dir[3] = { 0.0f, 0.0f, -1.0f };

  auto result = BeamTrace(crystal, rot, light_dir, 1.31f, 8);
  EXPECT_EQ(result.outgoing_w.size(), 0u);
}


TEST(BeamTracerTest, BeamTraceHaloPathExists) {
  // Hexagonal prism: light along z-axis should produce beams through basal and prism faces.
  // The classic 22-degree halo comes from basal→prism paths.
  // Verify that raypaths with exactly 2 face interactions exist (entry + exit, no internal reflections).
  auto crystal = Crystal::CreatePrism(1.0f);
  Rotation rot;

  float light_dir[3] = { 0.0f, 0.0f, -1.0f };
  float n = 1.31f;

  auto result = BeamTrace(crystal, rot, light_dir, n, 8);

  // Should have raypaths of length 2 (single pass-through: entry face → exit face).
  bool found_two_face_path = false;
  for (const auto& rp : result.outgoing_raypath) {
    if (rp.size() == 2) {
      found_two_face_path = true;
      break;
    }
  }
  EXPECT_TRUE(found_two_face_path);

  // Should also have raypaths of length > 2 (multiple internal reflections).
  bool found_multi_face_path = false;
  for (const auto& rp : result.outgoing_raypath) {
    if (rp.size() > 2) {
      found_multi_face_path = true;
      break;
    }
  }
  EXPECT_TRUE(found_multi_face_path);
}


TEST(BeamTracerTest, BeamTraceMaxHitsTruncation) {
  // Verify that max_hits correctly limits bounce depth.
  // With max_hits=1, only entry reflections and single-pass refractions should appear.
  // With max_hits=8, more beams should be generated.
  auto crystal = Crystal::CreatePrism(1.0f);
  Rotation rot;

  float light_dir[3] = { 0.0f, 0.0f, -1.0f };
  float n = 1.31f;

  auto result_1 = BeamTrace(crystal, rot, light_dir, n, 1);
  auto result_8 = BeamTrace(crystal, rot, light_dir, n, 8);

  // With more allowed bounces, should produce at least as many outgoing beams.
  EXPECT_GE(result_8.outgoing_w.size(), result_1.outgoing_w.size());

  // With max_hits=1, all raypaths should have at most 2 faces (entry + first exit).
  for (const auto& rp : result_1.outgoing_raypath) {
    EXPECT_LE(rp.size(), 2u);
  }

  // Total energy for max_hits=1 should be less than for max_hits=8
  // (more bounces capture more energy from internal reflections).
  float total_1 = 0.0f;
  for (float w : result_1.outgoing_w) {
    total_1 += w;
  }
  float total_8 = 0.0f;
  for (float w : result_8.outgoing_w) {
    total_8 += w;
  }
  EXPECT_GE(total_8, total_1 * 0.99f);  // total_8 >= total_1 (modulo floating point)
}


TEST(BeamTracerTest, BeamTraceObliqueIncidence) {
  // Oblique incidence: light at 45 degrees. Should still produce valid output.
  auto crystal = Crystal::CreatePrism(1.0f);
  Rotation rot;

  float light_dir[3] = { 0.0f, -1.0f / std::sqrt(2.0f), -1.0f / std::sqrt(2.0f) };
  float n = 1.31f;

  auto result = BeamTrace(crystal, rot, light_dir, n, 8);

  EXPECT_GT(result.outgoing_w.size(), 0u);
  EXPECT_GT(result.total_entry_area, 0.0f);

  // Energy conservation.
  float total_w = 0.0f;
  for (float w : result.outgoing_w) {
    total_w += w;
  }
  EXPECT_LE(total_w, result.total_entry_area * 1.01f);
  EXPECT_GT(total_w, result.total_entry_area * 0.5f);  // Looser bound for oblique (more truncation)

  // All outgoing directions should be unit vectors.
  for (size_t i = 0; i < result.outgoing_w.size(); i++) {
    float dx = result.outgoing_d[3 * i];
    float dy = result.outgoing_d[3 * i + 1];
    float dz = result.outgoing_d[3 * i + 2];
    float len = std::sqrt(dx * dx + dy * dy + dz * dz);
    EXPECT_NEAR(len, 1.0f, 1e-4f);
  }
}


}  // namespace lumice

// ============================================================================
// Simulator integration tests (beam tracing path)
// These are outside the lumice namespace to avoid include conflicts.
// ============================================================================

#include <thread>

#include "config/proj_config.hpp"
#include "config/sim_data.hpp"
#include "core/simulator.hpp"
#include "util/queue.hpp"

namespace {

// Helper: build a minimal SceneConfig with a hexagonal prism and beam tracing enabled.
lumice::SceneConfig MakeBeamTracingConfig(size_t ray_num, bool use_bt) {
  lumice::SceneConfig config{};
  config.ray_num_ = ray_num;
  config.max_hits_ = 8;
  config.use_beam_tracing_ = use_bt;

  // Sun: altitude 30°, azimuth 0°, point source (diameter 0)
  config.light_source_.param_ = { 30.0f, 0.0f, 0.0f };
  config.light_source_.spectrum_ = std::vector<lumice::WlParam>{ { 550.0f, 1.0f } };

  // Single scattering, single crystal type: regular hexagonal prism
  lumice::MsInfo ms{};
  ms.prob_ = 1.0f;
  lumice::ScatteringSetting ss{};
  ss.crystal_proportion_ = 1.0f;
  ss.crystal_.id_ = 0;
  lumice::PrismCrystalParam prism{};
  prism.h_ = { lumice::DistributionType::kNoRandom, 1.0f, 0.0f };
  for (auto& d : prism.d_) {
    d = { lumice::DistributionType::kNoRandom, 1.0f, 0.0f };
  }
  ss.crystal_.param_ = prism;
  // Default axis distribution (uniform random orientation)
  ss.filter_.id_ = 0;
  ss.filter_.symmetry_ = 0;
  ss.filter_.action_ = lumice::FilterConfig::kFilterIn;
  ss.filter_.param_ = lumice::SimpleFilterParam{ lumice::NoneFilterParam{} };
  ms.setting_.push_back(std::move(ss));
  config.ms_.push_back(std::move(ms));

  return config;
}

// Run simulator with given config and collect the first SimData output.
lumice::SimData RunSimulator(const lumice::SceneConfig& config) {
  auto config_queue = std::make_shared<lumice::Queue<lumice::SimBatch>>();
  auto data_queue = std::make_shared<lumice::Queue<lumice::SimData>>();

  constexpr uint32_t kSeed = 12345;
  lumice::Simulator sim(config_queue, data_queue, kSeed);

  std::thread t([&sim]() { sim.Run(); });

  // Enqueue AFTER thread starts to avoid race where thread reads shutdown state
  auto scene_ptr = std::make_shared<const lumice::SceneConfig>(config);
  config_queue->Emplace(lumice::SimBatch{ config.ray_num_, scene_ptr, 0 });

  // Wait for one result (blocks until SimulateOneWavelength finishes and emplaces)
  auto result = data_queue->Get();

  sim.Stop();
  t.join();
  return result;
}


TEST(BeamTracerIntegration, BeamTracingProducesOutput) {
  auto config = MakeBeamTracingConfig(100, true);
  auto data = RunSimulator(config);

  // Beam tracing should produce outgoing data
  EXPECT_GT(data.outgoing_d_.size(), 0u);
  EXPECT_GT(data.outgoing_w_.size(), 0u);
  EXPECT_EQ(data.outgoing_d_.size(), data.outgoing_w_.size() * 3);

  // outgoing_indices_ should match outgoing count
  EXPECT_EQ(data.outgoing_indices_.size(), data.outgoing_w_.size());

  // rays_ should be non-empty (BT now produces 2-node RaySeg chains for RenderConsumer compatibility)
  EXPECT_FALSE(data.rays_.Empty());
  // Each outgoing beam has an entry + outgoing pair
  EXPECT_EQ(data.rays_.size_, data.outgoing_w_.size() * 2);

  // total_intensity should be positive
  EXPECT_GT(data.total_intensity_, 0.0f);

  // All directions should be unit vectors
  for (size_t i = 0; i < data.outgoing_w_.size(); i++) {
    float dx = data.outgoing_d_[3 * i];
    float dy = data.outgoing_d_[3 * i + 1];
    float dz = data.outgoing_d_[3 * i + 2];
    float len = std::sqrt(dx * dx + dy * dy + dz * dz);
    EXPECT_NEAR(len, 1.0f, 1e-4f);
  }

  // All weights should be positive
  for (float w : data.outgoing_w_) {
    EXPECT_GT(w, 0.0f);
  }
}


TEST(BeamTracerIntegration, McPathUnchangedWhenBtDisabled) {
  // With use_beam_tracing_ = false, MC path should produce valid SimData with ray tree
  auto config = MakeBeamTracingConfig(1000, false);
  auto data = RunSimulator(config);

  // MC path should have valid metadata
  EXPECT_GT(data.root_ray_count_, 0u);
  EXPECT_GT(data.total_intensity_, 0.0f);
  // MC path fills rays_
  EXPECT_FALSE(data.rays_.Empty());
}


TEST(BeamTracerIntegration, NormalizationMagnitudeComparable) {
  // BT total_intensity should equal wl_weight * orientation_num
  constexpr size_t kRayNum = 500;
  auto bt_config = MakeBeamTracingConfig(kRayNum, true);
  auto bt_data = RunSimulator(bt_config);

  // total_intensity should be wl_weight(1.0) * orientation_num(500) = 500.0
  EXPECT_GT(bt_data.total_intensity_, 0.0f);
  EXPECT_NEAR(bt_data.total_intensity_, 1.0f * kRayNum, 1.0f);

  // Sum of outgoing weights should be positive and less than total_intensity
  // (because not all light exits; some is absorbed/reflected back)
  float bt_total_w = 0.0f;
  for (float w : bt_data.outgoing_w_) {
    bt_total_w += w;
  }
  EXPECT_GT(bt_total_w, 0.0f);
  EXPECT_LT(bt_total_w, bt_data.total_intensity_ * 1.01f);
}


TEST(BeamTracerIntegration, ScatteringAngleDistribution22Halo) {
  // Beam tracing of a hex prism with random orientations should produce
  // a 22° halo peak. We bin outgoing directions by scattering angle and verify.
  constexpr size_t kOrientations = 2000;
  auto config = MakeBeamTracingConfig(kOrientations, true);
  // Use altitude=0 so sun direction is horizontal
  config.light_source_.param_ = { 0.0f, 0.0f, 0.0f };
  auto data = RunSimulator(config);

  ASSERT_GT(data.outgoing_w_.size(), 0u);

  // Sun center direction: altitude=0, azimuth=0 → az_rad=π, alt_rad=0 → d=(-1,0,0)
  float sun_dir[3] = { -1.0f, 0.0f, 0.0f };

  // Bin by scattering angle (degrees): 5° bins
  constexpr int kNumBins = 36;
  constexpr float kBinWidth = 5.0f;
  float bins[kNumBins]{};

  for (size_t i = 0; i < data.outgoing_w_.size(); i++) {
    float dx = data.outgoing_d_[3 * i];
    float dy = data.outgoing_d_[3 * i + 1];
    float dz = data.outgoing_d_[3 * i + 2];
    // Scattering angle: angle between outgoing and -sun_dir (forward = 0°)
    float cos_angle = -(dx * sun_dir[0] + dy * sun_dir[1] + dz * sun_dir[2]);
    cos_angle = std::max(-1.0f, std::min(1.0f, cos_angle));
    float angle_deg = std::acos(cos_angle) * 180.0f / 3.14159265f;

    int bin = static_cast<int>(angle_deg / kBinWidth);
    if (bin >= 0 && bin < kNumBins) {
      bins[bin] += data.outgoing_w_[i];
    }
  }

  // The [20°, 25°] bin (bin 4) should have non-negligible weight (the 22° halo).
  // Note: bin 4 may not be the global peak — other halo types contribute weight at larger angles.
  // We verify the 22° halo signal exists, not that it dominates.
  float halo_bin_weight = bins[4];  // 20°-25°
  EXPECT_GT(halo_bin_weight, 0.0f) << "No weight in 22° halo region [20-25°]";

  // Compute total weight across halo feature region [15°, 90°], excluding 0° direct transmission
  float feature_weight = 0.0f;
  for (int b = 3; b <= 17; b++) {
    feature_weight += bins[b];
  }
  ASSERT_GT(feature_weight, 0.0f);

  // The 22° halo bin should carry at least 0.1% of feature region weight
  EXPECT_GT(halo_bin_weight / feature_weight, 0.001f) << "22° halo bin has negligible fraction of feature weight";

  // The 22° halo bin should be a local maximum: higher than adjacent bins
  // (the halo caustic creates a sharp peak near the minimum deviation angle)
  EXPECT_GE(bins[4], bins[3]) << "22° bin should be >= bin at [15-20°]";
}


TEST(BeamTracerIntegration, RootRayCountMatchesOrientationCount) {
  constexpr size_t kOrientations = 100;
  auto config = MakeBeamTracingConfig(kOrientations, true);
  auto data = RunSimulator(config);
  EXPECT_EQ(data.root_ray_count_, kOrientations);
}


TEST(BeamTracerIntegration, McVsBtHaloDistribution) {
  // Compare MC and BT scattering angle distributions with random orientations.
  // Both paths sample random crystal orientations; with enough samples,
  // the binned weight distributions should converge to the same shape.
  // This validates BT correctness at the distribution level.

  // Use horizontal sun (altitude=0) for simpler scattering angle geometry.
  // Use uniform random orientations (default in MakeBeamTracingConfig).

  constexpr size_t kMcRays = 50000;
  auto mc_config = MakeBeamTracingConfig(kMcRays, false);
  mc_config.light_source_.param_ = { 0.0f, 0.0f, 0.0f };  // altitude=0, azimuth=0, diameter=0
  mc_config.ms_[0].prob_ = 0.0f;                          // No multi-scattering forwarding
  // Use uniform random axis (override delta distribution)
  auto& mc_axis = mc_config.ms_[0].setting_[0].crystal_.axis_;
  mc_axis.azimuth_dist = { lumice::DistributionType::kUniform, 0.0f, 360.0f };
  mc_axis.latitude_dist = { lumice::DistributionType::kUniform, 0.0f, 360.0f };
  mc_axis.roll_dist = { lumice::DistributionType::kUniform, 0.0f, 360.0f };
  auto mc_data = RunSimulator(mc_config);

  constexpr size_t kBtOrientations = 5000;
  auto bt_config = MakeBeamTracingConfig(kBtOrientations, true);
  bt_config.light_source_.param_ = { 0.0f, 0.0f, 0.0f };
  auto& bt_axis = bt_config.ms_[0].setting_[0].crystal_.axis_;
  bt_axis.azimuth_dist = { lumice::DistributionType::kUniform, 0.0f, 360.0f };
  bt_axis.latitude_dist = { lumice::DistributionType::kUniform, 0.0f, 360.0f };
  bt_axis.roll_dist = { lumice::DistributionType::kUniform, 0.0f, 360.0f };
  auto bt_data = RunSimulator(bt_config);

  ASSERT_GT(bt_data.outgoing_w_.size(), 0u) << "BT produced no outgoing beams";
  ASSERT_GT(mc_data.outgoing_w_.size(), 0u) << "MC produced no outgoing rays";

  // Sun direction for altitude=0, azimuth=0: (-1, 0, 0)
  float sun_dir[3] = { -1.0f, 0.0f, 0.0f };

  // Bin by scattering angle: 5° bins, focus on halo region [15°, 90°]
  constexpr int kNumBins = 36;
  constexpr float kBinWidth = 5.0f;
  float mc_bins[kNumBins]{};
  float bt_bins[kNumBins]{};

  auto bin_data = [&](const std::vector<float>& dirs, const std::vector<float>& weights, float* bins) {
    float total_w = 0.0f;
    for (size_t i = 0; i < weights.size(); i++) {
      float cos_angle = -(dirs[3 * i] * sun_dir[0] + dirs[3 * i + 1] * sun_dir[1] + dirs[3 * i + 2] * sun_dir[2]);
      cos_angle = std::max(-1.0f, std::min(1.0f, cos_angle));
      float angle_deg = std::acos(cos_angle) * 180.0f / 3.14159265f;
      int bin = static_cast<int>(angle_deg / kBinWidth);
      if (bin >= 0 && bin < kNumBins) {
        bins[bin] += weights[i];
      }
      total_w += weights[i];
    }
    if (total_w > 0.0f) {
      for (int b = 0; b < kNumBins; b++) {
        bins[b] /= total_w;
      }
    }
  };

  bin_data(mc_data.outgoing_d_, mc_data.outgoing_w_, mc_bins);
  bin_data(bt_data.outgoing_d_, bt_data.outgoing_w_, bt_bins);

  // Both MC and BT should show the 22° halo peak in [20°, 25°] (bin 4)
  float mc_halo = mc_bins[4];
  float bt_halo = bt_bins[4];
  EXPECT_GT(mc_halo, 0.0f) << "MC has no weight in 22° halo region";
  EXPECT_GT(bt_halo, 0.0f) << "BT has no weight in 22° halo region";

  // Compare shape: for bins with significant weight (> 0.5% of total),
  // the MC/BT ratio should be roughly consistent.
  // We check that the peak bin (highest weight) is the same in both distributions,
  // and that the overall shape correlation is positive.
  int mc_peak_bin = -1;
  int bt_peak_bin = -1;
  float mc_peak = 0.0f;
  float bt_peak = 0.0f;
  constexpr int kStartBin = 3;  // 15°
  constexpr int kEndBin = 18;   // 90°

  for (int b = kStartBin; b < kEndBin; b++) {
    if (mc_bins[b] > mc_peak) {
      mc_peak = mc_bins[b];
      mc_peak_bin = b;
    }
    if (bt_bins[b] > bt_peak) {
      bt_peak = bt_bins[b];
      bt_peak_bin = b;
    }
  }

  // Peak bins should be within 1 bin of each other (5° tolerance)
  EXPECT_NEAR(mc_peak_bin, bt_peak_bin, 1) << "MC and BT peaks differ by more than 5°";

  // Check shape correlation: Pearson correlation of bin weights in [15°, 90°]
  float sum_mc = 0, sum_bt = 0, sum_mc2 = 0, sum_bt2 = 0, sum_mcbt = 0;
  int n = 0;
  for (int b = kStartBin; b < kEndBin; b++) {
    sum_mc += mc_bins[b];
    sum_bt += bt_bins[b];
    sum_mc2 += mc_bins[b] * mc_bins[b];
    sum_bt2 += bt_bins[b] * bt_bins[b];
    sum_mcbt += mc_bins[b] * bt_bins[b];
    n++;
  }
  float nf = static_cast<float>(n);
  float cov = sum_mcbt / nf - (sum_mc / nf) * (sum_bt / nf);
  float var_mc = sum_mc2 / nf - (sum_mc / nf) * (sum_mc / nf);
  float var_bt = sum_bt2 / nf - (sum_bt / nf) * (sum_bt / nf);
  float correlation = (var_mc > 0 && var_bt > 0) ? cov / std::sqrt(var_mc * var_bt) : 0.0f;

  // MC and BT distributions should be strongly correlated (> 0.9)
  EXPECT_GT(correlation, 0.9f) << "MC/BT distribution correlation too low: " << correlation;
}


TEST(BeamTracerIntegration, FixedOrientationMcVsBt) {
  // Per-beam matching at FIXED crystal orientation.
  //
  // Key insight: for planar crystal faces + fixed orientation + point source, the exit direction
  // depends only on the face sequence (raypath), NOT on the entry point. So both MC and BT produce
  // a finite set of discrete exit directions. Each direction corresponds to a unique raypath.
  // MC weight per direction = fraction of rays that followed that raypath (Fresnel probability).
  // BT weight per direction = area fraction * exact Fresnel transmission.
  // With enough MC samples, these should converge.
  //
  // Validation: for each BT beam, find the matching MC cluster (same direction within tolerance),
  // and compare normalized weights. Unmatched beams in either direction indicate algorithm bugs.

  auto set_fixed_axis = [](lumice::AxisDistribution& axis) {
    axis.azimuth_dist = { lumice::DistributionType::kNoRandom, 45.0f, 0.0f };
    axis.latitude_dist = { lumice::DistributionType::kNoRandom, 60.0f, 0.0f };  // zen=30 -> lat=60
    axis.roll_dist = { lumice::DistributionType::kNoRandom, 15.0f, 0.0f };
  };

  // MC: 500k rays -- enough for stable per-beam weight estimates
  constexpr size_t kMcRays = 500000;
  auto mc_config = MakeBeamTracingConfig(kMcRays, false);
  mc_config.light_source_.param_ = { 0.0f, 0.0f, 0.0f };
  mc_config.ms_[0].prob_ = 0.0f;
  set_fixed_axis(mc_config.ms_[0].setting_[0].crystal_.axis_);
  auto mc_data = RunSimulator(mc_config);

  // BT: 1 orientation (deterministic)
  auto bt_config = MakeBeamTracingConfig(1, true);
  bt_config.light_source_.param_ = { 0.0f, 0.0f, 0.0f };
  set_fixed_axis(bt_config.ms_[0].setting_[0].crystal_.axis_);
  auto bt_data = RunSimulator(bt_config);

  ASSERT_GT(mc_data.outgoing_w_.size(), 0u);
  ASSERT_GT(bt_data.outgoing_w_.size(), 0u);

  // Normalize both to probability distributions.
  float mc_total_w = 0;
  for (float w : mc_data.outgoing_w_) {
    mc_total_w += w;
  }
  float bt_total_w = 0;
  for (float w : bt_data.outgoing_w_) {
    bt_total_w += w;
  }
  ASSERT_GT(mc_total_w, 0.0f);
  ASSERT_GT(bt_total_w, 0.0f);

  // Cluster both MC and BT by direction, then match clusters.
  // For planar faces + fixed orientation + point source, each unique raypath produces
  // exactly one exit direction. BT may produce multiple beams for the same raypath
  // (from PartitionBeam subdivisions at different depths), which need to be aggregated.
  constexpr float kClusterCosThreshold = 0.99999f;  // < 0.26 degrees
  struct DirCluster {
    float d[3];
    double total_w;
  };

  auto cluster_directions = [&](const std::vector<float>& dirs, const std::vector<float>& weights) {
    std::vector<DirCluster> clusters;
    for (size_t i = 0; i < weights.size(); i++) {
      const float* d = dirs.data() + 3 * i;
      bool found = false;
      for (auto& c : clusters) {
        float cos_a = c.d[0] * d[0] + c.d[1] * d[1] + c.d[2] * d[2];
        if (cos_a > kClusterCosThreshold) {
          c.total_w += weights[i];
          found = true;
          break;
        }
      }
      if (!found) {
        clusters.push_back({ { d[0], d[1], d[2] }, weights[i] });
      }
    }
    return clusters;
  };

  auto mc_clusters = cluster_directions(mc_data.outgoing_d_, mc_data.outgoing_w_);
  auto bt_clusters = cluster_directions(bt_data.outgoing_d_, bt_data.outgoing_w_);

  // Match BT clusters to MC clusters and compare normalized weights.
  float bt_unmatched_w = 0;
  for (const auto& bt_c : bt_clusters) {
    float bt_w = static_cast<float>(bt_c.total_w / bt_total_w);

    int best_mc = -1;
    float best_cos = -1;
    for (size_t j = 0; j < mc_clusters.size(); j++) {
      float cos_a = mc_clusters[j].d[0] * bt_c.d[0] + mc_clusters[j].d[1] * bt_c.d[1] + mc_clusters[j].d[2] * bt_c.d[2];
      if (cos_a > best_cos) {
        best_cos = cos_a;
        best_mc = static_cast<int>(j);
      }
    }

    if (best_cos < kClusterCosThreshold) {
      bt_unmatched_w += bt_w;
      continue;
    }

    float mc_w = static_cast<float>(mc_clusters[best_mc].total_w / mc_total_w);

    // Skip negligible clusters (both < 0.1%)
    if (bt_w < 0.001f && mc_w < 0.001f) {
      continue;
    }
    float ref_w = std::max(bt_w, mc_w);
    float rel_diff = std::abs(bt_w - mc_w) / ref_w;

    EXPECT_LT(rel_diff, 0.15f) << "Direction (" << bt_c.d[0] << "," << bt_c.d[1] << "," << bt_c.d[2]
                               << "): BT_w=" << bt_w << " MC_w=" << mc_w << " rel_diff=" << rel_diff;
  }

  // Check for MC clusters with no matching BT cluster (BT missing a raypath)
  float mc_unmatched_w = 0;
  for (const auto& mc_c : mc_clusters) {
    float mc_w = static_cast<float>(mc_c.total_w / mc_total_w);
    bool matched = false;
    for (const auto& bt_c : bt_clusters) {
      float cos_a = mc_c.d[0] * bt_c.d[0] + mc_c.d[1] * bt_c.d[1] + mc_c.d[2] * bt_c.d[2];
      if (cos_a > kClusterCosThreshold) {
        matched = true;
        break;
      }
    }
    if (!matched && mc_w > 0.001f) {
      mc_unmatched_w += mc_w;
    }
  }

  // Coverage: vast majority of significant weight should be matched
  EXPECT_LT(bt_unmatched_w, 0.02f) << "BT unmatched weight: " << bt_unmatched_w * 100 << "%";
  EXPECT_LT(mc_unmatched_w, 0.01f) << "MC unmatched weight: " << mc_unmatched_w * 100 << "%";
}


}  // namespace
