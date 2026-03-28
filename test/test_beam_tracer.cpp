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

  // rays_ should be empty (beam tracing doesn't produce ray tree)
  EXPECT_TRUE(data.rays_.Empty());

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
  // MC path fills rays_ (beam tracing does not)
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


}  // namespace
