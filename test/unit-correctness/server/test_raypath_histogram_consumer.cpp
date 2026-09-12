// RaypathHistogramConsumer: synthetic SimData batches (no simulation) pin the
// three ROI modes, the cross-worker merge, the exact ring split and the sort
// order; one real-Simulator scene pins that the top chain of a 22° halo is the
// 22° raypath.
//
//   AC1  Full-sky / in-frame / cone energy and count against hand sums; the
//        ring split is exact: the consumer's ring prefix sums equal a filter
//        the test applies itself on angular distance.
//   AC2  Y authority: the full-sky Σ energy over a batch equals the Σ Y of a
//        RenderConsumer's raw image of the same batch, within the float
//        summation bound stated at the assertion.
//   AC3  End to end: the 22° halo scene's top chain, under Simulator::Run()
//        recording at its finest and ReduceRaypathHistogram under P|B|D, is
//        the reduced form of face path 3->5 computed by Crystal::ReduceRaypath
//        — no platform-dependent literal.
//   AC4  (removed with the cone stop target in v4.34; the in-ROI ray count it
//        exposed is now read as the Σ count over the snapshot's entries.)
//   AC5  Ties on energy order by display string.
// Also: an id space per producer (two workers' local ids collide, chains do
// not), batches without chain ids are ignored, a never-delivered id is dropped
// not crashed on, and Reset() forgets producers as well as buckets.
//
// The bounded record (the BoundedRecord suite): with a row capacity k over a
// synthetic stream of known chain energies, every chain above E/k has a row,
// no row is under its chain's true energy or over it by more than its own
// error_bound_, no error_bound_ exceeds E/k, and Σ rows + other is whole — in
// energy, count and (cone) ring split; a capacity above the chain count is
// the identity with every error 0; the sentinel's rays land in the other
// bucket with the producer's truncation count beside them; Reset() empties
// all of it. The reduction adds the errors of the rows it merges (≤ orbit
// size × E/k) and passes the record-level scalars through at every symmetry.
//
// The read-time reduction (the ReadTimeReduction / ChainDisplayFormat /
// ReduceContext suites at the bottom): Σ energy and Σ count are conserved and
// the row count is monotone over none / P / P|B / P|B|D on a hand-built finest
// result; the display format's four owner-specified shapes are pinned to the
// character; the per-layer multi-crystal flag is a property of the layer and
// survives one crystal id being alone on one layer and shared on another.

#include <gtest/gtest.h>

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <thread>
#include <utility>
#include <variant>
#include <vector>

#include "config/color_class_table.hpp"
#include "config/crystal_config.hpp"
#include "config/filter_config.hpp"
#include "config/light_config.hpp"
#include "config/proj_config.hpp"
#include "config/render_config.hpp"
#include "config/sim_data.hpp"
#include "core/chain_id_table.hpp"
#include "core/color_util.hpp"
#include "core/crystal.hpp"
#include "core/def.hpp"
#include "core/lens_proj_build.hpp"
#include "core/projection.hpp"
#include "core/scatter_accum.hpp"
#include "core/simulator.hpp"
#include "server/raypath_histogram_consumer.hpp"
#include "server/render.hpp"
#include "util/queue.hpp"

namespace lumice {
namespace {

using Seg = std::vector<IdType>;

constexpr uint8_t kSymAll = FilterConfig::kSymP | FilterConfig::kSymB | FilterConfig::kSymD;

// A synthetic batch: rays are appended with AddRay, chains declared with
// AddChain in FlushDelta() order (ascending ids, parents first).
struct Batch {
  SimData data;
  explicit Batch(uint32_t producer, float wl = 550.0f) {
    data.curr_wl_ = wl;
    data.producer_effective_seed_ = producer;
  }
  Batch& AddChain(uint32_t id, uint32_t parent, IdType crystal, Seg seg) {
    ChainIdTableEntry e;
    e.id = id;
    e.parent_id = parent;
    e.crystal_id = crystal;
    e.segment = std::move(seg);
    data.chain_id_table_delta_.push_back(std::move(e));
    return *this;
  }
  Batch& AddRay(uint32_t chain_id, float w, float dx, float dy, float dz) {
    data.outgoing_d_.insert(data.outgoing_d_.end(), { dx, dy, dz });
    data.outgoing_w_.push_back(w);
    data.outgoing_chain_id_.push_back(chain_id);
    return *this;
  }
  Batch& AddRay(uint32_t chain_id, float w, const float d[3]) { return AddRay(chain_id, w, d[0], d[1], d[2]); }
};

RaypathHistogramResult Snapshot(RaypathHistogramConsumer& c) {
  c.PrepareSnapshot();
  auto r = c.GetResult();
  EXPECT_TRUE(std::holds_alternative<RaypathHistogramResult>(r));
  return std::get<RaypathHistogramResult>(r);
}

// The number of rays counted into the ROI: Σ count over the entries plus the
// other bucket, which between them are the only places the consumer keeps it
// (a ray is counted iff it is in the ROI and its chain id resolves — to a row,
// or to the sentinel the other bucket stands for).
size_t RoiHitCount(RaypathHistogramConsumer& c) {
  const auto r = Snapshot(c);
  size_t n = r.other_count_;
  for (const auto& e : r.entries_) {
    n += e.count_;
  }
  return n;
}

const RaypathHistogramEntry* Find(const RaypathHistogramResult& r, const std::string& display) {
  for (const auto& e : r.entries_) {
    if (e.display_ == display) {
      return &e;
    }
  }
  return nullptr;
}

double Y(float wl, float w) {
  return static_cast<double>(SpectrumToYSingle(wl, w));
}

RaypathRoiSpec FullSky() {
  RaypathRoiSpec roi;
  roi.mode_ = RaypathRoiMode::kFullSky;
  return roi;
}

// World direction of the camera-frame direction `c` under `cfg`'s view — the
// same map the render-domain mask uses (lens_proj_build.hpp CameraDirToWorld),
// so "on axis" / "θ off axis" mean here what they mean to the projection.
void WorldDir(const RenderConfig& cfg, float cx, float cy, float cz, float out[3]) {
  const Rotation rot = MakeCameraRotation(cfg);
  const auto w = mask_detail::CameraDirToWorld(rot, projection::Dir3{ cx, cy, cz, true });
  out[0] = w.x;
  out[1] = w.y;
  out[2] = w.z;
}

// ---------------------------------------------------------------------------
// Step-3 smoke: one chain, one ray, full sky.
// ---------------------------------------------------------------------------
TEST(RaypathHistogramConsumer, OneRayOneChainFullSky) {
  RaypathHistogramConsumer c(FullSky());
  Batch b(7);
  b.AddChain(1, 0, 3, { 1, 3 }).AddRay(1, 0.5f, 0.0f, 0.0f, 1.0f);
  c.Consume(b.data);
  auto r = Snapshot(c);
  EXPECT_EQ(r.roi_mode_, RaypathRoiMode::kFullSky);
  ASSERT_EQ(r.entries_.size(), 1u);
  EXPECT_EQ(r.entries_[0].display_, "crystal3(1-3)");
  ASSERT_EQ(r.entries_[0].chain_.size(), 1u);
  EXPECT_EQ(r.entries_[0].chain_[0].crystal_id, 3u);
  EXPECT_EQ(r.entries_[0].chain_[0].segment, (Seg{ 1, 3 }));
  EXPECT_EQ(r.entries_[0].count_, 1u);
  EXPECT_DOUBLE_EQ(r.entries_[0].energy_, Y(550.0f, 0.5f));
  EXPECT_TRUE(r.entries_[0].ring_energy_.empty());
  EXPECT_EQ(RoiHitCount(c), 1u);
}

// ---------------------------------------------------------------------------
// AC1 full sky: several chains over several batches, per-batch and per-ray
// wavelengths, a depth-2 chain, the hand sums.
// ---------------------------------------------------------------------------
TEST(RaypathHistogramConsumer, FullSkyAccumulatesPerChainAcrossBatchesAndWavelengths) {
  RaypathHistogramConsumer c(FullSky());
  // Batch 1 at 550 nm: chain A (id 1) twice, chain B (id 2) once.
  Batch b1(7, 550.0f);
  b1.AddChain(1, 0, 1, { 3, 5 }).AddChain(2, 0, 1, { 1, 3, 2 });
  b1.AddRay(1, 1.0f, 0, 0, 1).AddRay(2, 0.25f, 0, 1, 0).AddRay(1, 0.5f, 1, 0, 0);
  c.Consume(b1.data);
  // Batch 2: per-ray wavelengths (curr_wl_ must then be ignored); a new
  // depth-2 chain C under A (id 3) plus more of A.
  Batch b2(7, 999.0f);  // 999 nm is out of CMF range: if it were used, Y would be 0
  b2.AddChain(3, 1, 2, { 4 });
  b2.data.outgoing_wl_ = { 450.0f, 600.0f };
  b2.AddRay(3, 2.0f, 0, 0, -1).AddRay(1, 0.125f, 0, -1, 0);
  c.Consume(b2.data);

  auto r = Snapshot(c);
  ASSERT_EQ(r.entries_.size(), 3u);
  const auto* a = Find(r, "crystal1(3-5)");
  const auto* b = Find(r, "crystal1(1-3-2)");
  const auto* cc = Find(r, "crystal1(3-5)-crystal2(4)");
  ASSERT_NE(a, nullptr);
  ASSERT_NE(b, nullptr);
  ASSERT_NE(cc, nullptr);
  EXPECT_EQ(a->count_, 3u);
  EXPECT_DOUBLE_EQ(a->energy_, Y(550, 1.0f) + Y(550, 0.5f) + Y(600, 0.125f));
  EXPECT_EQ(b->count_, 1u);
  EXPECT_DOUBLE_EQ(b->energy_, Y(550, 0.25f));
  EXPECT_EQ(cc->count_, 1u);
  EXPECT_DOUBLE_EQ(cc->energy_, Y(450, 2.0f));
  ASSERT_EQ(cc->chain_.size(), 2u);
  EXPECT_EQ(cc->chain_[0].crystal_id, 1u);
  EXPECT_EQ(cc->chain_[0].segment, (Seg{ 3, 5 }));
  EXPECT_EQ(cc->chain_[1].crystal_id, 2u);
  EXPECT_EQ(cc->chain_[1].segment, (Seg{ 4 }));
  EXPECT_EQ(RoiHitCount(c), 5u);
  // Energy-descending: A carries the most (Y at 550 is the CMF peak).
  EXPECT_EQ(r.entries_[0].display_, "crystal1(3-5)");
}

// ---------------------------------------------------------------------------
// AC1 in frame: a linear 60° frame on the horizon. On-axis lands; 40° off
// axis misses the frame; behind the camera misses the projection; and the
// `visible` clip drops one hemisphere by the sign of the world z — the
// decision-B correction of the issue's "ProjectExitToPixel already handles
// visible" premise (it does not: with `full` both hemispheres count).
// ---------------------------------------------------------------------------
RenderConfig HorizonLinear60(RenderConfig::VisibleRange visible) {
  RenderConfig cfg;
  cfg.id_ = 0;
  cfg.lens_.type_ = LensParam::kLinear;
  cfg.lens_.fov_ = 60.0f;
  cfg.resolution_[0] = 64;
  cfg.resolution_[1] = 48;
  cfg.view_.az_ = 0.0f;
  cfg.view_.el_ = 0.0f;
  cfg.view_.ro_ = 0.0f;
  cfg.visible_ = visible;
  return cfg;
}

TEST(RaypathHistogramConsumer, InFrameCountsLandedRaysAndAppliesTheVisibleClip) {
  const RenderConfig full = HorizonLinear60(RenderConfig::kFull);
  float on_axis[3];
  float off_axis_40[3];
  float behind[3];
  float up_10[3];
  float down_10[3];
  WorldDir(full, 0.0f, 0.0f, 1.0f, on_axis);
  WorldDir(full, std::sin(40.0f * math::kDegreeToRad), 0.0f, std::cos(40.0f * math::kDegreeToRad), off_axis_40);
  WorldDir(full, 0.0f, 0.0f, -1.0f, behind);
  // Two rays 10° above / below the optical axis. Which of ±cy is "up" is the
  // camera frame's business; label them by the world z they come out with,
  // which is what VisibleByRange reads.
  WorldDir(full, 0.0f, std::sin(10.0f * math::kDegreeToRad), std::cos(10.0f * math::kDegreeToRad), up_10);
  WorldDir(full, 0.0f, -std::sin(10.0f * math::kDegreeToRad), std::cos(10.0f * math::kDegreeToRad), down_10);
  if (up_10[2] > 0.0f) {
    std::swap(up_10, down_10);
  }
  ASSERT_LT(up_10[2], 0.0f) << "fixture: the 'from above' ray must have world z < 0";
  ASSERT_GT(down_10[2], 0.0f) << "fixture: the 'from below' ray must have world z > 0";

  auto run = [&](RenderConfig::VisibleRange visible) {
    RaypathRoiSpec roi;
    roi.mode_ = RaypathRoiMode::kInFrame;
    roi.frame_config_ = HorizonLinear60(visible);
    RaypathHistogramConsumer c(roi);
    Batch b(1);
    b.AddChain(1, 0, 1, { 3, 5 }).AddChain(2, 0, 1, { 3, 6 }).AddChain(3, 0, 1, { 1, 2 }).AddChain(4, 0, 1, { 3, 4 });
    b.AddRay(1, 1.0f, on_axis);      // in frame
    b.AddRay(2, 1.0f, off_axis_40);  // outside a 60° frame
    b.AddRay(2, 1.0f, behind);       // projection miss
    b.AddRay(3, 0.5f, up_10);        // in frame, from above (wz < 0)
    b.AddRay(4, 0.25f, down_10);     // in frame, from below (wz > 0)
    c.Consume(b.data);
    return Snapshot(c);
  };

  auto r_full = run(RenderConfig::kFull);
  EXPECT_EQ(r_full.roi_mode_, RaypathRoiMode::kInFrame);
  ASSERT_NE(Find(r_full, "crystal1(3-5)"), nullptr) << "positive control: the on-axis ray must land";
  EXPECT_EQ(Find(r_full, "crystal1(3-5)")->count_, 1u);
  EXPECT_EQ(Find(r_full, "crystal1(3-6)"), nullptr) << "off-frame and behind-camera rays must not count";
  ASSERT_NE(Find(r_full, "crystal1(1-2)"), nullptr);
  ASSERT_NE(Find(r_full, "crystal1(3-4)"), nullptr);
  EXPECT_DOUBLE_EQ(Find(r_full, "crystal1(1-2)")->energy_, Y(550, 0.5f));
  EXPECT_DOUBLE_EQ(Find(r_full, "crystal1(3-4)")->energy_, Y(550, 0.25f));

  auto r_upper = run(RenderConfig::kUpper);
  ASSERT_NE(Find(r_upper, "crystal1(1-2)"), nullptr) << "`upper` keeps the ray from above";
  EXPECT_EQ(Find(r_upper, "crystal1(3-4)"), nullptr) << "`upper` drops the ray from below";

  auto r_lower = run(RenderConfig::kLower);
  EXPECT_EQ(Find(r_lower, "crystal1(1-2)"), nullptr) << "`lower` drops the ray from above";
  ASSERT_NE(Find(r_lower, "crystal1(3-4)"), nullptr) << "`lower` keeps the ray from below";
}

// The second clip, `front`, on a lens that images behind the camera: an
// equirectangular frame sees a ray 120° off axis, and `front` drops it.
TEST(RaypathHistogramConsumer, InFrameAppliesTheFrontClip) {
  RenderConfig cfg;
  cfg.id_ = 0;
  cfg.lens_.type_ = LensParam::kRectangular;
  cfg.lens_.fov_ = 300.0f;
  cfg.resolution_[0] = 128;
  cfg.resolution_[1] = 64;
  cfg.view_.el_ = 0.0f;
  cfg.visible_ = RenderConfig::kFull;
  float back_120[3];
  WorldDir(cfg, std::sin(120.0f * math::kDegreeToRad), 0.0f, std::cos(120.0f * math::kDegreeToRad), back_120);
  float on_axis[3];
  WorldDir(cfg, 0.0f, 0.0f, 1.0f, on_axis);

  auto run = [&](bool front) {
    RaypathRoiSpec roi;
    roi.mode_ = RaypathRoiMode::kInFrame;
    roi.frame_config_ = cfg;
    roi.frame_config_.front_ = front;
    RaypathHistogramConsumer c(roi);
    Batch b(1);
    b.AddChain(1, 0, 1, { 3, 5 }).AddChain(2, 0, 1, { 3, 6 });
    b.AddRay(1, 1.0f, on_axis).AddRay(2, 1.0f, back_120);
    c.Consume(b.data);
    return Snapshot(c);
  };
  auto r_off = run(false);
  ASSERT_NE(Find(r_off, "crystal1(3-5)"), nullptr);
  ASSERT_NE(Find(r_off, "crystal1(3-6)"), nullptr) << "positive control: without `front` the 120° ray is imaged";
  auto r_on = run(true);
  ASSERT_NE(Find(r_on, "crystal1(3-5)"), nullptr);
  EXPECT_EQ(Find(r_on, "crystal1(3-6)"), nullptr) << "`front` drops the hemisphere behind the camera";
}

// ---------------------------------------------------------------------------
// AC1 cone: membership by angular distance and an exact ring split.
// ---------------------------------------------------------------------------
struct ConeRay {
  float angle;  // from the centre, radians
  float w;
  uint32_t chain;
};

// Direction at `angle` from +z, in the plane through +x, slightly rotated so
// no two rays coincide.
void ConeDir(float angle, float twist, float out[3]) {
  out[0] = std::sin(angle) * std::cos(twist);
  out[1] = std::sin(angle) * std::sin(twist);
  out[2] = std::cos(angle);
}

TEST(RaypathHistogramConsumer, ConeMembershipAndRingSplitAreExact) {
  constexpr float kRadius = 0.5f;
  constexpr int kRings = 4;
  constexpr float kWidth = kRadius / kRings;  // 0.125, exact in binary
  RaypathRoiSpec roi;
  roi.mode_ = RaypathRoiMode::kCone;
  // Not unit length on purpose: the constructor normalises.
  roi.cone_center_[0] = 0.0f;
  roi.cone_center_[1] = 0.0f;
  roi.cone_center_[2] = 2.0f;
  roi.cone_radius_rad_ = kRadius;
  roi.cone_ring_count_ = kRings;
  RaypathHistogramConsumer c(roi);
  EXPECT_FLOAT_EQ(c.Roi().cone_center_[2], 1.0f);

  const std::vector<ConeRay> rays = {
    { 0.01f, 1.0f, 1 },
    { 0.13f, 0.5f, 2 },
    { 2.0f * kWidth - 1e-3f, 0.25f, 1 },   // just inside ring 1
    { 2.0f * kWidth + 1e-3f, 0.125f, 2 },  // just inside ring 2: the boundary is at 2.0 widths
    { 2.5f * kWidth, 2.0f, 1 },            // ring 2 under floor; 3 under rounding
    { 0.49f, 4.0f, 2 },                    // last ring
    { kRadius + 1e-3f, 8.0f, 1 },          // outside
    { 1.0f, 16.0f, 2 },                    // outside
    { 3.0f, 32.0f, 1 },                    // the other hemisphere
  };
  Batch b(1);
  b.AddChain(1, 0, 1, { 3, 5 }).AddChain(2, 0, 1, { 1, 2 });
  float twist = 0.0f;
  for (const auto& ray : rays) {
    float d[3];
    ConeDir(ray.angle, twist, d);
    twist += 0.7f;
    b.AddRay(ray.chain, ray.w, d);
  }
  c.Consume(b.data);
  auto r = Snapshot(c);
  EXPECT_EQ(r.roi_mode_, RaypathRoiMode::kCone);
  EXPECT_EQ(r.cone_ring_count_, kRings);
  EXPECT_FLOAT_EQ(r.cone_radius_rad_, kRadius);
  ASSERT_EQ(r.entries_.size(), 2u);

  // Membership: 6 in, 3 out.
  EXPECT_EQ(RoiHitCount(c), 6u);
  const auto* e1 = Find(r, "crystal1(3-5)");
  const auto* e2 = Find(r, "crystal1(1-2)");
  ASSERT_NE(e1, nullptr);
  ASSERT_NE(e2, nullptr);
  EXPECT_EQ(e1->count_, 3u);
  EXPECT_EQ(e2->count_, 3u);
  ASSERT_EQ(e1->ring_energy_.size(), static_cast<size_t>(kRings));
  ASSERT_EQ(e2->ring_energy_.size(), static_cast<size_t>(kRings));

  // The ring split is exact: for every k, the consumer's Σ ring[0..k] over
  // both chains equals the test's own sum over rays with angle < (k+1)·width.
  // Rays sit ≥ 1e-3 rad from every ring boundary except the 2.5-width one,
  // which is 0.5 widths from either — so acos rounding cannot move any of
  // them across a boundary and the two paths must agree on the set.
  for (int k = 0; k < kRings; k++) {
    double consumer = 0.0;
    for (int j = 0; j <= k; j++) {
      consumer += e1->ring_energy_[j] + e2->ring_energy_[j];
    }
    double independent = 0.0;
    size_t members = 0;
    for (const auto& ray : rays) {
      if (ray.angle < static_cast<float>(k + 1) * kWidth) {
        independent += Y(550, ray.w);
        members++;
      }
    }
    EXPECT_GT(members, 0u);
    EXPECT_NEAR(consumer, independent, 1e-12 * independent) << "ring prefix " << k;
  }
  // Ring 2 holds exactly the 2.0+1e-3 (chain 2) and the 2.5 (chain 1) rays.
  EXPECT_DOUBLE_EQ(e1->ring_energy_[2], Y(550, 2.0f));
  EXPECT_DOUBLE_EQ(e2->ring_energy_[2], Y(550, 0.125f));
  // Ring 1 holds the 0.13 (chain 2) and the 2.0-1e-3 (chain 1) rays.
  EXPECT_DOUBLE_EQ(e1->ring_energy_[1], Y(550, 0.25f));
  EXPECT_DOUBLE_EQ(e2->ring_energy_[1], Y(550, 0.5f));
  // Σ rings == energy.
  for (const auto* e : { e1, e2 }) {
    double s = 0.0;
    for (double v : e->ring_energy_) {
      s += v;
    }
    EXPECT_NEAR(s, e->energy_, 1e-12 * e->energy_);
  }
}

TEST(RaypathHistogramConsumer, ConeWithOneRingAndAnOffAxisCentre) {
  RaypathRoiSpec roi;
  roi.mode_ = RaypathRoiMode::kCone;
  roi.cone_center_[0] = 1.0f;
  roi.cone_center_[1] = 1.0f;
  roi.cone_center_[2] = 0.0f;
  roi.cone_radius_rad_ = 0.2f;
  roi.cone_ring_count_ = 1;
  RaypathHistogramConsumer c(roi);
  Batch b(1);
  b.AddChain(1, 0, 1, { 3, 5 });
  const float s = std::sqrt(0.5f);
  b.AddRay(1, 1.0f, s, s, 0.0f);        // on the centre
  b.AddRay(1, 1.0f, 0.0f, 0.0f, 1.0f);  // 90° away
  c.Consume(b.data);
  auto r = Snapshot(c);
  ASSERT_EQ(r.entries_.size(), 1u);
  EXPECT_EQ(r.entries_[0].count_, 1u);
  ASSERT_EQ(r.entries_[0].ring_energy_.size(), 1u);
  EXPECT_DOUBLE_EQ(r.entries_[0].ring_energy_[0], r.entries_[0].energy_);
}

// ---------------------------------------------------------------------------
// Two producers, colliding local ids, interleaved as a queue delivers them.
// ---------------------------------------------------------------------------
TEST(RaypathHistogramConsumer, MergesProducersByChainNotByLocalId) {
  RaypathHistogramConsumer c(FullSky());
  // Producer 10: X=(3-5) as 1, Y=(1-2) as 2. Producer 20: Y as 1, X as 2.
  Batch p10a(10);
  p10a.AddChain(1, 0, 1, { 3, 5 }).AddRay(1, 1.0f, 0, 0, 1);
  Batch p20a(20);
  p20a.AddChain(1, 0, 1, { 1, 2 }).AddRay(1, 2.0f, 0, 0, 1);
  Batch p10b(10);
  p10b.AddChain(2, 0, 1, { 1, 2 }).AddRay(2, 4.0f, 0, 0, 1).AddRay(1, 8.0f, 0, 0, 1);
  Batch p20b(20);
  p20b.AddChain(2, 0, 1, { 3, 5 }).AddRay(2, 16.0f, 0, 0, 1).AddRay(1, 32.0f, 0, 0, 1);
  // A later batch that only references earlier ids (empty delta).
  Batch p20c(20);
  p20c.AddRay(2, 64.0f, 0, 0, 1);
  for (const auto* b : { &p10a, &p20a, &p10b, &p20b, &p20c }) {
    c.Consume(b->data);
  }
  auto r = Snapshot(c);
  ASSERT_EQ(r.entries_.size(), 2u) << "two chains, not four";
  const auto* x = Find(r, "crystal1(3-5)");
  const auto* y = Find(r, "crystal1(1-2)");
  ASSERT_NE(x, nullptr);
  ASSERT_NE(y, nullptr);
  EXPECT_EQ(x->count_, 4u);
  EXPECT_DOUBLE_EQ(x->energy_, Y(550, 1.0f) + Y(550, 8.0f) + Y(550, 16.0f) + Y(550, 64.0f));
  EXPECT_EQ(y->count_, 3u);
  EXPECT_DOUBLE_EQ(y->energy_, Y(550, 2.0f) + Y(550, 4.0f) + Y(550, 32.0f));
}

// ---------------------------------------------------------------------------
// Robustness: no chain ids → ignored; a never-delivered id → dropped.
// ---------------------------------------------------------------------------
TEST(RaypathHistogramConsumer, BatchesWithoutChainIdsAreIgnoredAndUnknownIdsDropped) {
  RaypathHistogramConsumer c(FullSky());
  SimData no_ids;
  no_ids.curr_wl_ = 550.0f;
  no_ids.outgoing_d_ = { 0, 0, 1 };
  no_ids.outgoing_w_ = { 1.0f };
  c.Consume(no_ids);
  EXPECT_EQ(RoiHitCount(c), 0u);

  Batch b(1);
  b.AddChain(1, 0, 1, { 3, 5 }).AddRay(1, 1.0f, 0, 0, 1).AddRay(42, 1.0f, 0, 0, 1);
  c.Consume(b.data);
  auto r = Snapshot(c);
  ASSERT_EQ(r.entries_.size(), 1u);
  EXPECT_EQ(r.entries_[0].count_, 1u);
  EXPECT_EQ(RoiHitCount(c), 1u) << "a dropped ray is not an ROI hit";
}

// ---------------------------------------------------------------------------
// AC5: energy descending, ties by display string ascending.
// ---------------------------------------------------------------------------
TEST(RaypathHistogramConsumer, SortsByEnergyThenDisplay) {
  RaypathHistogramConsumer c(FullSky());
  Batch b(1);
  b.AddChain(1, 0, 2, { 3, 5 }).AddChain(2, 0, 1, { 3, 5 }).AddChain(3, 0, 1, { 1, 2 }).AddChain(4, 0, 3, { 4 });
  b.AddRay(1, 1.0f, 0, 0, 1).AddRay(2, 1.0f, 0, 0, 1).AddRay(3, 1.0f, 0, 0, 1).AddRay(4, 2.0f, 0, 0, 1);
  c.Consume(b.data);
  auto r = Snapshot(c);
  ASSERT_EQ(r.entries_.size(), 4u);
  EXPECT_EQ(r.entries_[0].display_, "crystal3(4)");
  EXPECT_EQ(r.entries_[1].display_, "crystal1(1-2)");
  EXPECT_EQ(r.entries_[2].display_, "crystal1(3-5)");
  EXPECT_EQ(r.entries_[3].display_, "crystal2(3-5)");
  // Stable across a second snapshot with the map in a different state.
  Batch more(1);
  more.AddRay(3, 0.0f, 0, 0, 1);  // zero weight: counts, adds no energy
  c.Consume(more.data);
  auto r2 = Snapshot(c);
  ASSERT_EQ(r2.entries_.size(), 4u);
  for (size_t i = 0; i < 4; i++) {
    EXPECT_EQ(r2.entries_[i].display_, r.entries_[i].display_);
  }
}

// Reset() forgets producers too: the same local id 1 delivered again after a
// Reset() is a fresh session, not a non-monotonic id, and the buckets restart.
TEST(RaypathHistogramConsumer, ResetForgetsBucketsAndProducers) {
  RaypathHistogramConsumer c(FullSky());
  Batch b(1);
  b.AddChain(1, 0, 1, { 3, 5 }).AddRay(1, 1.0f, 0, 0, 1);
  c.Consume(b.data);
  c.Reset();
  EXPECT_TRUE(Snapshot(c).entries_.empty());
  Batch again(1);
  again.AddChain(1, 0, 1, { 1, 2 }).AddRay(1, 1.0f, 0, 0, 1);
  c.Consume(again.data);
  auto r = Snapshot(c);
  ASSERT_EQ(r.entries_.size(), 1u);
  EXPECT_EQ(r.entries_[0].display_, "crystal1(1-2)");
  EXPECT_EQ(r.entries_[0].count_, 1u);
}

// ---------------------------------------------------------------------------
// AC2: Y authority. The same batch through this consumer (full sky) and
// through a RenderConsumer whose frame images every ray; the two Σ Y agree.
// ---------------------------------------------------------------------------
TEST(RaypathHistogramConsumer, FullSkyEnergyEqualsRenderConsumerImageY) {
  // A 180° equal-area frame looking up sees every ray within 80° of its axis
  // exactly once (no dual-fisheye overlap, no rim cull), so the image's Σ Y is
  // the Σ over rays of Y(wl)·w — the same terms this consumer sums.
  RenderConfig cfg;
  cfg.id_ = 0;
  cfg.lens_.type_ = LensParam::kFisheyeEqualArea;
  cfg.lens_.fov_ = 180.0f;
  cfg.resolution_[0] = 32;
  cfg.resolution_[1] = 32;
  cfg.view_.el_ = 90.0f;
  cfg.visible_ = RenderConfig::kFull;

  // Deterministic pseudo-random rays (an LCG, not the engine's RNG), varied
  // weights and per-ray wavelengths. Wavelengths stay in [480, 620] nm and
  // weights in [0.1, 1.1] so every term is bounded away from zero — the
  // tolerance below has to be smaller than any one term for a dropped ray to
  // be detectable, and a 400 nm ray at w=0.01 contributes ~1e-5.
  constexpr size_t kRays = 4000;
  uint32_t state = 12345u;
  auto next = [&state]() {
    state = state * 1664525u + 1013904223u;
    return static_cast<float>(state >> 8) / static_cast<float>(1u << 24);
  };
  Batch b(1);
  b.AddChain(1, 0, 1, { 3, 5 }).AddChain(2, 0, 1, { 1, 2 }).AddChain(3, 0, 1, { 3, 6 });
  b.data.ray_seg_count_ = kRays;
  double expected_sum = 0.0;
  double abs_sum = 0.0;
  for (size_t i = 0; i < kRays; i++) {
    const float theta = next() * 80.0f * math::kDegreeToRad;
    const float phi = next() * 2.0f * math::kPi;
    float d[3];
    WorldDir(cfg, std::sin(theta) * std::cos(phi), std::sin(theta) * std::sin(phi), std::cos(theta), d);
    const float w = 0.1f + next();
    const float wl = 480.0f + next() * 140.0f;
    b.data.outgoing_wl_.push_back(wl);
    b.AddRay(1 + static_cast<uint32_t>(i % 3), w, d);
    expected_sum += Y(wl, w);
    abs_sum += std::fabs(Y(wl, w));
  }
  b.data.outgoing_component_.assign(kRays, 0u);

  RaypathHistogramConsumer hist(FullSky());
  hist.Consume(b.data);
  auto r = Snapshot(hist);
  double hist_sum = 0.0;
  size_t hist_count = 0;
  for (const auto& e : r.entries_) {
    hist_sum += e.energy_;
    hist_count += e.count_;
  }
  EXPECT_EQ(hist_count, kRays);
  EXPECT_NEAR(hist_sum, expected_sum, 1e-9 * abs_sum) << "double accumulation of the same float terms";

  RenderConsumer rc(cfg, ColorClassTable{});
  rc.Consume(b.data);
  rc.PrepareSnapshot();
  const RawXyzResult raw = rc.GetRawXyzResult();
  ASSERT_NE(raw.xyz_buffer_, nullptr);
  double image_sum = 0.0;
  const size_t total_pix = static_cast<size_t>(raw.img_width_) * static_cast<size_t>(raw.img_height_);
  for (size_t i = 0; i < total_pix; i++) {
    image_sum += raw.xyz_buffer_[i * 3 + 1];
  }
  ASSERT_GT(image_sum, 0.0) << "positive control: the frame must have imaged the rays";
  // Tolerance: RenderConsumer's CPU path does per-pixel float `+=` with no
  // compensation. Pixel p receives n_p additions, each losing at most
  // FLT_EPSILON of a running partial sum bounded by that pixel's total S_p,
  // so the image is within Σ_p n_p·FLT_EPSILON·S_p ≤ max_p(n_p)·FLT_EPSILON·Σ|y|
  // of the exact sum; the histogram side is double and negligible beside
  // that. max_p(n_p) is counted with the same projection the render used.
  // Bit equality is not claimed.
  std::map<int, size_t> per_pixel;
  {
    const Rotation rot = MakeCameraRotation(cfg);
    const auto pp = BuildProjParams(cfg, rot, static_cast<float>(std::min(cfg.resolution_[0], cfg.resolution_[1])));
    for (size_t i = 0; i < kRays; i++) {
      const auto hit = lm_proj::ProjectExitToPixel(pp, b.data.outgoing_d_[i * 3], b.data.outgoing_d_[i * 3 + 1],
                                                   b.data.outgoing_d_[i * 3 + 2]);
      if (hit.count != 1) {
        ADD_FAILURE() << "fixture: ray " << i << " must land exactly once (count " << hit.count << ")";
        continue;
      }
      per_pixel[hit.hits[0].py * cfg.resolution_[0] + hit.hits[0].px]++;
    }
  }
  size_t max_per_pixel = 0;
  for (const auto& [pix, n] : per_pixel) {
    max_per_pixel = std::max(max_per_pixel, n);
  }
  const double bound = static_cast<double>(max_per_pixel) * FLT_EPSILON * abs_sum;
  EXPECT_NEAR(hist_sum, image_sum, bound) << "Σ energy (histogram) vs Σ Y (image); bound=" << bound;
  // And the bound is not so loose that it would let a dropped ray through:
  // the smallest single term is far above it.
  double min_term = 1e300;
  for (size_t i = 0; i < kRays; i++) {
    min_term = std::min(min_term, Y(b.data.outgoing_wl_[i], b.data.outgoing_w_[i]));
  }
  EXPECT_GT(min_term, bound) << "a single missing ray must exceed the tolerance";
}

// ---------------------------------------------------------------------------
// AC3: end to end. The 22° halo scene (prism, random orientation, sun 20°),
// analysis on, one Simulator, every batch into the consumer; the top chain is
// the reduced form of the 22° path 3->5 on that crystal.
// ---------------------------------------------------------------------------
std::vector<SimData> RunSceneToBatches(const SceneConfig& scene, size_t ray_num, size_t batches, uint32_t seed) {
  auto config_queue = std::make_shared<Queue<SimBatch>>();
  auto data_queue = std::make_shared<Queue<SimData>>();
  Simulator sim(config_queue, data_queue, seed);
  // As the server records: at the finest, the reduction being the reader's.
  sim.SetAnalysisChainId(true, FilterConfig::kSymNone);
  auto shared_scene = std::make_shared<const SceneConfig>(scene);
  for (size_t b = 0; b < batches; ++b) {
    SimBatch batch;
    batch.ray_num_ = ray_num;
    batch.scene_ = shared_scene;
    batch.generation_ = 1;
    config_queue->Emplace(std::move(batch));
  }
  config_queue->Emplace(SimBatch{});  // ray_num_ == 0 → Run() exits
  std::thread runner([&] { sim.Run(); });
  runner.join();
  std::vector<SimData> out;
  while (!data_queue->Empty()) {
    out.push_back(data_queue->Get());
  }
  return out;
}

// The direction sunlight travels for a SunParam: SampleSphCapPoint(az+180,
// -alt) in the simulator, i.e. the antipode of where the sun sits.
void SunlightDir(const SunParam& sun, float out[3]) {
  const float lon = (sun.azimuth_ + 180.0f) * math::kDegreeToRad;
  const float lat = -sun.altitude_ * math::kDegreeToRad;
  out[0] = std::cos(lat) * std::cos(lon);
  out[1] = std::cos(lat) * std::sin(lon);
  out[2] = std::sin(lat);
}

// The 22° halo scene: the crystal of test/e2e/configs/halo_22.json, read
// through core's own parser rather than retyped, so the two cannot drift
// apart; sun at 20°, one wavelength.
SceneConfig Halo22Scene() {
  const nlohmann::json crystal_json = {
    { "id", 1 },
    { "type", "prism" },
    { "shape", { { "height", 1.2 } } },
    { "axis",
      { { "zenith", { { "type", "uniform" }, { "mean", 90 }, { "std", 360 } } },
        { "azimuth", { { "type", "uniform" }, { "mean", 0 }, { "std", 360 } } } } },
  };
  SceneConfig scene;
  scene.ray_num_ = 0;
  scene.max_hits_ = 7;
  scene.light_source_.param_ = SunParam{ 20.0f, 0.0f, 0.5f };
  scene.light_source_.spectrum_ = std::vector<WlParam>{ { 550.0f, 1.0f } };
  MsInfo ms;
  ms.prob_ = 0.0f;
  ScatteringSetting s{};
  s.filter_.id_ = 0;
  s.filter_.symmetry_ = FilterConfig::kSymNone;
  s.filter_.action_ = FilterConfig::kFilterIn;
  s.filter_.param_ = SimpleFilterParam{ NoneFilterParam{} };
  s.crystal_ = crystal_json.get<CrystalConfig>();
  s.crystal_proportion_ = 1.0f;
  ms.setting_.push_back(std::move(s));
  scene.ms_.push_back(std::move(ms));
  return scene;
}

// The reduced form of face path `rp` on the scene's crystal, by the SAME
// authority the simulator interns with (Crystal::ReduceRaypath with the
// axis-derived D parameters) — never a literal.
Seg ExpectedSegment(const SimData& batch, const Seg& rp) {
  const Crystal& crystal = batch.crystals_[0];
  const AxisDistribution& axis = batch.crystal_axis_dists_[0];
  const bool d_applicable = detail::IsDApplicable(axis);
  const int sigma_a = d_applicable ? detail::ComputeSigmaA(axis.roll_dist.center) : 0;
  return crystal.ReduceRaypath(rp, kSymAll, sigma_a, d_applicable);
}

class Halo22 : public ::testing::Test {
 protected:
  // 8 × 20k rays with a fixed seed: enough that the leads asserted below are
  // wide (see the ratio assertions), fixed so the run is reproducible.
  static void SetUpTestSuite() {
    scene_ = new SceneConfig(Halo22Scene());
    batches_ = new std::vector<SimData>(RunSceneToBatches(*scene_, 20000, 8, 20260911u));
  }
  static void TearDownTestSuite() {
    delete batches_;
    delete scene_;
    batches_ = nullptr;
    scene_ = nullptr;
  }
  static SceneConfig* scene_;
  static std::vector<SimData>* batches_;
};
SceneConfig* Halo22::scene_ = nullptr;
std::vector<SimData>* Halo22::batches_ = nullptr;

// Full sky: the 22° path is the strongest chain, ahead of the undeviated pass
// through opposite prism faces (3->6) — the sun's own image, which is #2 with
// ~80% of the halo's energy and the reason an ROI exists at all. The chains
// are recorded at their finest and reduced under P|B|D when read, and the
// result is what the old record-time P|B|D reduction produced: the same top
// two segments by the same Crystal::ReduceRaypath authority.
TEST_F(Halo22, FullSkyTopChainIsThe22DegreePathAheadOfTheUndeviatedPass) {
  const auto& batches = *batches_;
  ASSERT_EQ(batches.size(), 8u);
  ASSERT_FALSE(batches[0].crystals_.empty());
  const Seg halo = ExpectedSegment(batches[0], Seg{ 3, 5 });
  const Seg through = ExpectedSegment(batches[0], Seg{ 3, 6 });
  ASSERT_NE(halo, through);

  RaypathHistogramConsumer c(FullSky(), BuildRaypathReduceContext(*scene_));
  size_t delivered = 0;
  for (const auto& sd : batches) {
    EXPECT_EQ(sd.outgoing_chain_id_.size(), sd.outgoing_w_.size());
    delivered += sd.outgoing_w_.size();
    c.Consume(sd);
  }
  ASSERT_GT(delivered, 10000u);
  EXPECT_EQ(RoiHitCount(c), delivered);
  const auto finest = Snapshot(c);
  // Positive control on the reduction: recorded at the finest, the 22° path
  // is spread over its orbit (six prism-face rotations at least), so the
  // finest result has strictly more rows than the reduced one, and the raw
  // record carries the ONE unreduced sequence {3, 5} as well as its images.
  ASSERT_NE(Find(finest, "crystal1(3-5)"), nullptr) << "the finest record keeps the raw face sequence";
  ASSERT_NE(Find(finest, "crystal1(4-6)"), nullptr) << "and its P-image";
  auto r = ReduceRaypathHistogram(finest, kSymAll);
  EXPECT_LT(r.entries_.size(), finest.entries_.size()) << "P|B|D merges the orbit";
  ASSERT_GE(r.entries_.size(), 2u);
  for (const auto& e : r.entries_) {
    if (e.chain_.size() != 1u) {
      ADD_FAILURE() << "MS=1: every chain has one layer, " << e.display_ << " has " << e.chain_.size();
      continue;
    }
    EXPECT_EQ(e.chain_[0].crystal_id, 1u);
  }
  ASSERT_EQ(r.entries_[0].chain_.size(), 1u);
  ASSERT_EQ(r.entries_[1].chain_.size(), 1u);
  EXPECT_EQ(r.entries_[0].chain_[0].segment, halo)
      << "top chain is " << r.entries_[0].display_ << ", expected the 22° path";
  EXPECT_EQ(r.entries_[1].chain_[0].segment, through)
      << "#2 is " << r.entries_[1].display_ << ", expected the undeviated pass";
  // The display text is the read-time authority's, in the single-crystal
  // single-layer shape: the bare face sequence.
  EXPECT_EQ(r.entries_[0].display_, "3-5");
  EXPECT_EQ(r.entries_[1].display_, "3-6");
  // Measured 1.24× at this seed over 160k rays, where the counts' own noise is
  // well under 1%: a 1.1× floor is far from the noise and still catches a
  // reordering.
  EXPECT_GT(r.entries_[0].energy_, 1.1 * r.entries_[1].energy_) << "the 22° path's lead over the undeviated pass";
  size_t counted = 0;
  double energy = 0.0;
  for (const auto& e : r.entries_) {
    counted += e.count_;
    energy += e.energy_;
  }
  // The record is bounded (kRaypathHistogramCapacity rows, ChainIdInterningTable::
  // kDefaultCapacity chains per producer) and this scene, recorded at its finest,
  // exceeds both; what no row holds is in the other bucket, and the two together
  // are every ray delivered.
  EXPECT_EQ(counted + r.other_count_, delivered);
  EXPECT_EQ(r.other_count_, finest.other_count_) << "the reduction passes the bucket through";
  EXPECT_EQ(r.truncated_chain_count_, finest.truncated_chain_count_);
  // Conservation across the four symmetries a reader can ask for, on real
  // data: the sums are the finest sums, the row count never grows, and the
  // 22° path's ORBIT — the rows that fold into "3-5" under P|B|D — carries
  // the same energy at every one of them. Not "the top row is the 22° path
  // at every symmetry": that is false, and for a physical reason. The 22°
  // path's orbit has 12 finest members (six rotations, two mirror images)
  // where the undeviated pass's has 6 (rotations only — 3-6 is its own
  // mirror image), so at the finest each undeviated row carries ~2x a halo
  // row's energy and "4-7" leads; under P the halo is still split in two
  // (3-5 / 3-7) against one 3-6 row, and 3-6 leads; only D folds the mirror
  // and lets 3-5 lead. Recorded so the next reader does not "fix" it.
  size_t finest_count = 0;
  double finest_energy = 0.0;
  for (const auto& e : finest.entries_) {
    finest_count += e.count_;
    finest_energy += e.energy_;
  }
  EXPECT_EQ(counted, finest_count);
  EXPECT_NEAR(energy, finest_energy, 1e-12 * finest_energy);
  size_t prev_rows = finest.entries_.size() + 1;
  for (const uint8_t sym : { FilterConfig::kSymNone, FilterConfig::kSymP,
                             static_cast<uint8_t>(FilterConfig::kSymP | FilterConfig::kSymB), kSymAll }) {
    const auto rr = ReduceRaypathHistogram(finest, sym);
    size_t cnt = 0;
    double en = 0.0;
    for (const auto& e : rr.entries_) {
      cnt += e.count_;
      en += e.energy_;
    }
    EXPECT_EQ(cnt, finest_count) << "symmetry " << int(sym);
    EXPECT_NEAR(en, finest_energy, 1e-12 * finest_energy) << "symmetry " << int(sym);
    EXPECT_LE(rr.entries_.size(), prev_rows) << "symmetry " << int(sym) << ": rows never grow as bits are added";
    prev_rows = rr.entries_.size();
    if (rr.entries_.empty()) {
      ADD_FAILURE() << "symmetry " << int(sym) << ": no rows";
      continue;
    }
    // The orbit's rows, classified by the authority itself (each row's segment
    // reduced the rest of the way under P|B|D), sum to the one P|B|D row.
    double orbit_energy = 0.0;
    size_t orbit_count = 0;
    size_t orbit_rows = 0;
    for (const auto& e : rr.entries_) {
      if (ExpectedSegment(batches[0], e.chain_[0].segment) == halo) {
        orbit_energy += e.energy_;
        orbit_count += e.count_;
        orbit_rows++;
      }
    }
    EXPECT_EQ(orbit_count, r.entries_[0].count_) << "symmetry " << int(sym);
    EXPECT_NEAR(orbit_energy, r.entries_[0].energy_, 1e-12 * r.entries_[0].energy_) << "symmetry " << int(sym);
    // 12 finest members (six rotations x two mirror images), 2 under P and
    // P|B (the mirror pair), 1 under P|B|D.
    const size_t expected_rows = sym == FilterConfig::kSymNone ? 12u : sym == kSymAll ? 1u : 2u;
    EXPECT_EQ(orbit_rows, expected_rows) << "symmetry " << int(sym);
    // And the top row belongs to one of the two known orbits: the halo's or
    // the undeviated pass's.
    const Seg top_class = ExpectedSegment(batches[0], rr.entries_[0].chain_[0].segment);
    EXPECT_TRUE(top_class == halo || top_class == through)
        << "symmetry " << int(sym) << ": top chain is " << rr.entries_[0].display_;
  }
}

// Cone on the 22° ring: the top chain is the 22° path, by a wide lead, and
// the undeviated pass is absent (it lands 22° away, on the sun). Also AC4's
// stop target on real data.
TEST_F(Halo22, ConeOnThe22DegreeRingRanksThe22DegreePathFirst) {
  const auto& batches = *batches_;
  ASSERT_FALSE(batches[0].crystals_.empty());
  const Seg halo = ExpectedSegment(batches[0], Seg{ 3, 5 });
  const Seg through = ExpectedSegment(batches[0], Seg{ 3, 6 });

  // Fixture premise: the direction convention. Nearly every undeviated ray
  // must travel along SunlightDir() (within the sun's 0.5° disc plus a margin).
  float sun_dir[3];
  SunlightDir(scene_->light_source_.param_, sun_dir);
  {
    ChainIdMerger m;
    size_t through_rays = 0;
    size_t aligned = 0;
    for (const auto& sd : batches) {
      m.Absorb(sd.producer_effective_seed_, sd.chain_id_table_delta_);
      for (size_t i = 0; i < sd.outgoing_w_.size(); i++) {
        const uint32_t id = m.Resolve(sd.producer_effective_seed_, sd.outgoing_chain_id_[i]);
        if (id == ChainIdMerger::kUnresolved) {
          ADD_FAILURE() << "ray " << i << " carries an id its producer never delivered";
          continue;
        }
        if (id == ChainIdInterningTable::kOverflowChainId) {
          continue;  // a chain the producer's bounded table had no room for: no segment to read
        }
        if (m.Table().EntryAt(id).segment != through) {
          continue;
        }
        through_rays++;
        const float dot = sun_dir[0] * sd.outgoing_d_[i * 3] + sun_dir[1] * sd.outgoing_d_[i * 3 + 1] +
                          sun_dir[2] * sd.outgoing_d_[i * 3 + 2];
        if (dot > std::cos(1.0f * math::kDegreeToRad)) {
          aligned++;
        }
      }
    }
    ASSERT_GT(through_rays, 1000u);
    ASSERT_GT(aligned, through_rays * 9 / 10) << "SunlightDir() does not match the simulator's convention";
  }

  // A point on the 22° ring straight above the sun: altitude 20 + 23 (the
  // halo's energy sits just outside the 22° minimum deviation), radius 2.5°.
  SunParam above = scene_->light_source_.param_;
  above.altitude_ += 23.0f;
  RaypathRoiSpec roi;
  roi.mode_ = RaypathRoiMode::kCone;
  SunlightDir(above, roi.cone_center_);
  roi.cone_radius_rad_ = 2.5f * math::kDegreeToRad;
  roi.cone_ring_count_ = 5;
  RaypathHistogramConsumer c(roi, BuildRaypathReduceContext(*scene_));
  for (const auto& sd : batches) {
    c.Consume(sd);
  }
  EXPECT_GE(RoiHitCount(c), 200u) << "the cone on the halo's brightest arc gathers rays from every batch";

  auto r = ReduceRaypathHistogram(Snapshot(c), kSymAll);
  ASSERT_GE(r.entries_.size(), 1u);
  const auto& top = r.entries_[0];
  ASSERT_EQ(top.chain_.size(), 1u);
  EXPECT_EQ(top.chain_[0].segment, halo) << "top chain in the cone is " << top.display_ << ", expected the 22° path";
  if (r.entries_.size() > 1) {
    EXPECT_GT(top.energy_, 3.0 * r.entries_[1].energy_)
        << "the lead over #2 (" << r.entries_[1].display_ << ") must be wide enough that no seed reorders them";
  }
  for (const auto& e : r.entries_) {
    EXPECT_NE(e.chain_[0].segment, through) << "the undeviated pass cannot land 23° from the sun";
    if (e.ring_energy_.size() != 5u) {
      ADD_FAILURE() << e.display_ << ": " << e.ring_energy_.size() << " rings, expected 5";
      continue;
    }
    double s = 0.0;
    for (double v : e.ring_energy_) {
      s += v;
    }
    EXPECT_NEAR(s, e.energy_, 1e-12 * e.energy_);
  }
}

// ---------------------------------------------------------------------------
// The bounded record. Ten single-layer chains, w = 1 per ray so energy is
// count × Y(550, 1); the order is adversarial — the eight light chains arrive
// first and fill every row, then the two heavy ones arrive with no row free —
// so what the assertions see is the eviction path, not the "there was room"
// path.
// ---------------------------------------------------------------------------
struct SyntheticStream {
  std::vector<size_t> rays_per_chain;  // chain i (1-based) has rays_per_chain[i-1] rays
  Batch batch{ 7 };
  size_t total_rays = 0;
  explicit SyntheticStream(std::vector<size_t> rays, bool cone = false) : rays_per_chain(std::move(rays)) {
    for (size_t i = 0; i < rays_per_chain.size(); i++) {
      batch.AddChain(static_cast<uint32_t>(i + 1), 0, 1, Seg{ static_cast<IdType>(i + 1) });
    }
    // Round-robin over the chains, each round adding one ray to every chain
    // that still has rays left: every chain shows up early and the heavy
    // ones keep arriving after the light ones have taken every row.
    bool any = true;
    for (size_t round = 0; any; round++) {
      any = false;
      for (size_t i = 0; i < rays_per_chain.size(); i++) {
        if (round < rays_per_chain[i]) {
          any = true;
          // Cone: chain i's rays in ring i mod 3 (5° cone, 3 rings, +z centre).
          const float angle = cone ? (static_cast<float>(i % 3) + 0.5f) * (5.0f / 3.0f) * math::kDegreeToRad : 0.0f;
          batch.AddRay(static_cast<uint32_t>(i + 1), 1.0f, std::sin(angle), 0.0f, std::cos(angle));
          total_rays++;
        }
      }
    }
  }
  double ChainEnergy(size_t i) const { return static_cast<double>(rays_per_chain[i - 1]) * Y(550.0f, 1.0f); }
  double Total() const { return static_cast<double>(total_rays) * Y(550.0f, 1.0f); }
  static std::string Display(size_t i) { return "crystal1(" + std::to_string(i) + ")"; }
};

RaypathRoiSpec Cone5Deg3Rings() {
  RaypathRoiSpec roi;
  roi.mode_ = RaypathRoiMode::kCone;
  roi.cone_center_[0] = 0.0f;
  roi.cone_center_[1] = 0.0f;
  roi.cone_center_[2] = 1.0f;
  roi.cone_radius_rad_ = 5.0f * math::kDegreeToRad;
  roi.cone_ring_count_ = 3;
  return roi;
}

// The Space-Saving guarantee on `r` for a stream `st` recorded at capacity `k`.
void ExpectSpaceSavingGuarantee(const RaypathHistogramResult& r, const SyntheticStream& st, size_t k) {
  const double total = st.Total();
  const double bound = total / static_cast<double>(k);
  const double tol = 1e-12 * total;
  EXPECT_LE(r.entries_.size(), k);
  double sum_energy = r.other_energy_;
  size_t sum_count = r.other_count_;
  double max_err = 0.0;
  for (const auto& e : r.entries_) {
    sum_energy += e.energy_;
    sum_count += e.count_;
    max_err = std::max(max_err, e.error_bound_);
    EXPECT_LE(e.error_bound_, bound + tol) << e.display_ << ": a row's error never exceeds E/k";
    if (e.chain_.size() != 1u) {
      ADD_FAILURE() << e.display_ << ": single-layer stream, " << e.chain_.size() << " layers";
      continue;
    }
    const size_t i = e.chain_[0].segment[0];
    const double truth = st.ChainEnergy(i);
    EXPECT_GE(e.energy_, truth - tol) << e.display_ << ": a row never under-estimates its chain";
    EXPECT_LE(e.energy_ - e.error_bound_, truth + tol) << e.display_ << ": the truth lies within the error";
    double rings = 0.0;
    for (double v : e.ring_energy_) {
      rings += v;
    }
    if (!e.ring_energy_.empty()) {
      EXPECT_NEAR(rings, e.energy_, tol) << e.display_ << ": the ring split is as whole as the energy";
    }
  }
  EXPECT_NEAR(sum_energy, total, tol) << "Σ rows + other is every ray's energy";
  EXPECT_EQ(sum_count, st.total_rays) << "Σ rows + other is every ray";
  EXPECT_DOUBLE_EQ(r.max_row_error_, max_err);
  for (size_t i = 1; i <= st.rays_per_chain.size(); i++) {
    if (st.ChainEnergy(i) > bound) {
      EXPECT_NE(Find(r, SyntheticStream::Display(i)), nullptr) << "chain " << i << " is above E/k and must have a row";
    }
  }
}

TEST(BoundedRecord, EveryChainAboveTotalOverKHasARowAndNoRowIsOffByMoreThanTotalOverK) {
  // Eight light chains (2 rays), two heavy (40 and 30): E/4 = 24 rays' worth,
  // so exactly the two heavy chains must have rows.
  const SyntheticStream st({ 2, 2, 2, 2, 2, 2, 2, 2, 40, 30 });
  constexpr size_t kCap = 4;
  RaypathHistogramConsumer c(FullSky(), {}, kCap);
  EXPECT_EQ(c.Capacity(), kCap);
  c.Consume(st.batch.data);
  const auto r = Snapshot(c);
  ExpectSpaceSavingGuarantee(r, st, kCap);
  EXPECT_EQ(r.entries_.size(), kCap) << "ten chains into four rows: full";
  EXPECT_GT(r.max_row_error_, 0.0) << "positive control: the eviction path ran";
  EXPECT_DOUBLE_EQ(r.other_energy_, 0.0) << "no sentinel ray: the bucket is empty, evictions do not fill it";
  EXPECT_EQ(r.other_count_, 0u);
  EXPECT_EQ(r.truncated_chain_count_, 0u);
  // The two heavy rows are the top two, each over by at most its own error.
  ASSERT_GE(r.entries_.size(), 2u);
  EXPECT_EQ(r.entries_[0].display_, SyntheticStream::Display(9));
  EXPECT_EQ(r.entries_[1].display_, SyntheticStream::Display(10));
}

// The ruler is the capacity: the same stream at k = 2, 4 and 10 evicts less
// as k grows, and at k >= the chain count the record is the unbounded one —
// every chain its own row, every error 0, which is the identity the small
// single-layer scenes rely on.
TEST(BoundedRecord, CapacityIsTheRulerAndAboveTheChainCountTheRecordIsExact) {
  const SyntheticStream st({ 2, 2, 2, 2, 2, 2, 2, 2, 40, 30 });
  double prev_err = 1e300;
  for (const size_t k : { size_t{ 2 }, size_t{ 4 }, size_t{ 10 }, size_t{ 4096 } }) {
    RaypathHistogramConsumer c(FullSky(), {}, k);
    c.Consume(st.batch.data);
    const auto r = Snapshot(c);
    ExpectSpaceSavingGuarantee(r, st, k);
    EXPECT_LE(r.max_row_error_, prev_err) << "k=" << k << ": a larger table is never less certain";
    prev_err = r.max_row_error_;
    if (k >= st.rays_per_chain.size()) {
      EXPECT_EQ(r.entries_.size(), st.rays_per_chain.size()) << "k=" << k;
      EXPECT_DOUBLE_EQ(r.max_row_error_, 0.0) << "k=" << k << ": no eviction, no error";
      for (const auto& e : r.entries_) {
        EXPECT_DOUBLE_EQ(e.error_bound_, 0.0);
        EXPECT_NEAR(e.energy_, st.ChainEnergy(e.chain_[0].segment[0]), 1e-12 * st.Total()) << e.display_;
      }
    } else {
      EXPECT_EQ(r.entries_.size(), k) << "k=" << k;
      EXPECT_GT(r.max_row_error_, 0.0) << "k=" << k;
    }
  }
}

TEST(BoundedRecord, ConeRingsTravelWithTheRowTheyAreTakenOverInto) {
  const SyntheticStream st({ 3, 3, 3, 3, 3, 3, 50, 20 }, /*cone=*/true);
  RaypathHistogramConsumer c(Cone5Deg3Rings(), {}, 3);
  c.Consume(st.batch.data);
  const auto r = Snapshot(c);
  ExpectSpaceSavingGuarantee(r, st, 3);  // includes Σ rings == energy per row
  EXPECT_GT(r.max_row_error_, 0.0);
  // And the reduction keeps the rings whole too.
  const auto rr = ReduceRaypathHistogram(r, kSymAll);
  for (const auto& e : rr.entries_) {
    double rings = 0.0;
    for (double v : e.ring_energy_) {
      rings += v;
    }
    EXPECT_NEAR(rings, e.energy_, 1e-12 * st.Total()) << e.display_;
  }
}

TEST(BoundedRecord, SentinelRaysFillTheOtherBucketBesideTheProducersTruncationCount) {
  constexpr uint32_t kOverflow = ChainIdInterningTable::kOverflowChainId;
  Batch b(3);
  b.AddChain(1, 0, 1, Seg{ 3, 5 });
  b.AddRay(1, 1.0f, 0.0f, 0.0f, 1.0f);
  b.AddRay(kOverflow, 0.5f, 0.0f, 0.0f, 1.0f);
  b.AddRay(kOverflow, 0.25f, 0.0f, 0.0f, 1.0f);
  b.data.chain_id_overflow_count_ = 7;
  RaypathHistogramConsumer c(FullSky(), {}, 4096);
  c.Consume(b.data);
  // A second batch from the same producer: the count accumulates across
  // batches whether or not the batch has any sentinel ray.
  Batch b2(3);
  b2.AddRay(1, 1.0f, 0.0f, 0.0f, 1.0f);
  b2.data.chain_id_overflow_count_ = 2;
  c.Consume(b2.data);
  const auto r = Snapshot(c);
  ASSERT_EQ(r.entries_.size(), 1u) << "the sentinel is never a row";
  EXPECT_DOUBLE_EQ(r.entries_[0].energy_, Y(550.0f, 1.0f) * 2.0);
  EXPECT_EQ(r.entries_[0].count_, 2u);
  EXPECT_DOUBLE_EQ(r.entries_[0].error_bound_, 0.0);
  EXPECT_DOUBLE_EQ(r.other_energy_, Y(550.0f, 0.5f) + Y(550.0f, 0.25f));
  EXPECT_EQ(r.other_count_, 2u);
  EXPECT_EQ(r.truncated_chain_count_, 9u);
  EXPECT_DOUBLE_EQ(r.max_row_error_, 0.0);
  // Cone: the sentinel's rays are ring-split into the bucket like any other
  // ray, and the row count is unaffected.
  RaypathHistogramConsumer cc(Cone5Deg3Rings(), {}, 4096);
  cc.Consume(b.data);
  const auto rc = Snapshot(cc);
  EXPECT_EQ(rc.entries_.size(), 1u);
  EXPECT_EQ(rc.other_count_, 2u);

  c.Reset();
  const auto after = Snapshot(c);
  EXPECT_TRUE(after.entries_.empty());
  EXPECT_DOUBLE_EQ(after.other_energy_, 0.0);
  EXPECT_EQ(after.other_count_, 0u);
  EXPECT_EQ(after.truncated_chain_count_, 0u);
  EXPECT_DOUBLE_EQ(after.max_row_error_, 0.0);
  // And the rows are free again: the same stream lands in the same shape.
  c.Consume(b.data);
  EXPECT_EQ(Snapshot(c).entries_.size(), 1u);
}

// A row's error after many evictions of the same slot stays at most E/k:
// stream where every chain is seen once, so every arrival past capacity is
// an eviction, and the taken-over energy chains through the slot.
TEST(BoundedRecord, ChainedTakeOversNeverPushARowsErrorPastTotalOverK) {
  std::vector<size_t> rays(200, 1);
  rays[0] = 100;  // one heavy chain, first to arrive, that must survive
  const SyntheticStream st(rays);
  constexpr size_t kCap = 8;
  RaypathHistogramConsumer c(FullSky(), {}, kCap);
  c.Consume(st.batch.data);
  const auto r = Snapshot(c);
  ExpectSpaceSavingGuarantee(r, st, kCap);
  EXPECT_NE(Find(r, SyntheticStream::Display(1)), nullptr);
  EXPECT_EQ(r.entries_[0].display_, SyntheticStream::Display(1));
  // Round-robin, so the heavy chain was the minimum (a one-ray tie) early on
  // and lost its row at least once: it is back, over by at most E/k, never
  // under. The 200-way slot churn behind it is what the guarantee is for.
  EXPECT_GT(r.entries_[0].error_bound_, 0.0) << "positive control: the heavy chain did get evicted once";
}

// ---------------------------------------------------------------------------
// Read-time reduction on a hand-built finest result. Crystal 1 has D
// applicable with sigma_a = 0 (the σ-mirror keeps prism face 3 and maps
// pri k -> -k), crystal 2 has no D. Five finest rows on crystal 1:
//   {3,5}  {4,6}   the same path one prism face apart      -> P merges them
//   {3,7}          the σ-mirror image of {3,5}             -> D merges it
//   {1,3,5} {2,3,5} the same path through opposite basals  -> B merges them
// so the row count walks 5 -> 4 (P) -> 3 (P|B) -> 2 (P|B|D) while the sums
// stay put. Each expected representative is Crystal::ReduceRaypath's word,
// re-derived here through ReduceRaypathByPeriod rather than typed.
// ---------------------------------------------------------------------------
// Every row carries an error_bound_ of 0.5 — as if recorded at a capacity k
// with E/k = 0.5 (the record-level scalars below say so: 15 units of energy
// in rows + 5 in the other bucket = 20, k = 40) — so a merged row's error is
// 0.5 × its orbit size.
constexpr double kFixtureRowError = 0.5;
constexpr double kFixtureTotalOverK = 0.5;  // (15 + 5) / 40

RaypathHistogramResult FinestFixture() {
  RaypathHistogramResult r;
  r.roi_mode_ = RaypathRoiMode::kFullSky;
  r.reduce_ctx_.crystal_params_[1] = RaypathCrystalReduceParams{ 0, true };
  r.reduce_ctx_.crystal_params_[2] = RaypathCrystalReduceParams{ 0, false };
  r.reduce_ctx_.layer_multi_crystal_ = { false };
  r.other_energy_ = 5.0;
  r.other_count_ = 50;
  r.truncated_chain_count_ = 12;
  r.max_row_error_ = kFixtureRowError;
  auto add = [&r](Seg seg, double energy, size_t count) {
    RaypathHistogramEntry e;
    e.chain_.push_back(RaypathChainSegment{ 1, std::move(seg) });
    e.display_ = "finest";
    e.energy_ = energy;
    e.count_ = count;
    e.error_bound_ = kFixtureRowError;
    r.entries_.push_back(std::move(e));
  };
  add({ 3, 5 }, 5.0, 50);
  add({ 4, 6 }, 4.0, 40);
  add({ 3, 7 }, 3.0, 30);
  add({ 1, 3, 5 }, 2.0, 20);
  add({ 2, 3, 5 }, 1.0, 10);
  return r;
}

void ExpectSums(const RaypathHistogramResult& r, double energy, size_t count, const char* what) {
  double e = 0.0;
  size_t c = 0;
  for (const auto& entry : r.entries_) {
    e += entry.energy_;
    c += entry.count_;
  }
  EXPECT_DOUBLE_EQ(e, energy) << what;
  EXPECT_EQ(c, count) << what;
}

TEST(ReadTimeReduction, RowCountIsMonotoneAndSumsAreConservedOverTheFourSymmetries) {
  const auto finest = FinestFixture();
  const auto none = ReduceRaypathHistogram(finest, FilterConfig::kSymNone);
  const auto p = ReduceRaypathHistogram(finest, FilterConfig::kSymP);
  const auto pb = ReduceRaypathHistogram(finest, FilterConfig::kSymP | FilterConfig::kSymB);
  const auto pbd = ReduceRaypathHistogram(finest, kSymAll);
  EXPECT_EQ(none.entries_.size(), 5u);
  EXPECT_EQ(p.entries_.size(), 4u);
  EXPECT_EQ(pb.entries_.size(), 3u);
  EXPECT_EQ(pbd.entries_.size(), 2u);
  for (const auto* r : { &none, &p, &pb, &pbd }) {
    ExpectSums(*r, 15.0, 150, "every symmetry keeps the finest sums");
    EXPECT_EQ(r->roi_mode_, RaypathRoiMode::kFullSky);
    EXPECT_EQ(r->reduce_ctx_.layer_multi_crystal_, finest.reduce_ctx_.layer_multi_crystal_);
    for (size_t i = 1; i < r->entries_.size(); i++) {
      EXPECT_GE(r->entries_[i - 1].energy_, r->entries_[i].energy_) << "energy descending";
    }
  }
  // Under P, {3,5} and {4,6} became one row carrying both; {3,7} stayed apart.
  const auto* p35 = Find(p, "3-5");
  ASSERT_NE(p35, nullptr);
  EXPECT_DOUBLE_EQ(p35->energy_, 9.0);
  EXPECT_EQ(p35->count_, 90u);
  EXPECT_NE(Find(p, "3-7"), nullptr);
  // P|B|D: two rows, whose segments are what the reduction authority says.
  EXPECT_EQ(pbd.entries_[0].chain_[0].segment, ReduceRaypathByPeriod({ 4, 6 }, kSymAll, 0, true, 6));
  EXPECT_EQ(pbd.entries_[0].chain_[0].segment, ReduceRaypathByPeriod({ 3, 7 }, kSymAll, 0, true, 6));
  EXPECT_DOUBLE_EQ(pbd.entries_[0].energy_, 12.0);
  EXPECT_EQ(pbd.entries_[0].count_, 120u);
  EXPECT_EQ(pbd.entries_[1].chain_[0].segment, ReduceRaypathByPeriod({ 2, 3, 5 }, kSymAll, 0, true, 6));
  EXPECT_DOUBLE_EQ(pbd.entries_[1].energy_, 3.0);
  EXPECT_EQ(pbd.entries_[1].count_, 30u);
  EXPECT_EQ(pbd.entries_[0].display_, "3-5");
  EXPECT_EQ(pbd.entries_[1].display_, "1-3-5");
  // Symmetry 0 is not a bypass: the display text is the read-time format even then.
  EXPECT_NE(Find(none, "3-5"), nullptr);
  EXPECT_EQ(Find(none, "finest"), nullptr);
}

// AC4: a merged row's error is the Σ of its finest rows' — m rows each ≤ E/k
// bound the merged row by m × E/k — and the record-level scalars are the
// same at every symmetry, being about no chain.
TEST(ReadTimeReduction, MergedRowErrorIsTheOrbitSumAndRecordScalarsPassThrough) {
  const auto finest = FinestFixture();
  const auto none = ReduceRaypathHistogram(finest, FilterConfig::kSymNone);
  const auto p = ReduceRaypathHistogram(finest, FilterConfig::kSymP);
  const auto pbd = ReduceRaypathHistogram(finest, kSymAll);
  for (const auto* r : { &none, &p, &pbd }) {
    EXPECT_DOUBLE_EQ(r->other_energy_, 5.0);
    EXPECT_EQ(r->other_count_, 50u);
    EXPECT_EQ(r->truncated_chain_count_, 12u);
    double max_err = 0.0;
    for (const auto& e : r->entries_) {
      max_err = std::max(max_err, e.error_bound_);
    }
    EXPECT_DOUBLE_EQ(r->max_row_error_, max_err) << "the max is over this result's own rows";
  }
  EXPECT_DOUBLE_EQ(none.max_row_error_, kFixtureRowError);
  EXPECT_DOUBLE_EQ(p.max_row_error_, 2.0 * kFixtureRowError);
  EXPECT_DOUBLE_EQ(pbd.max_row_error_, 3.0 * kFixtureRowError);
  // Orbit sizes by the reduction authority itself, not typed: how many finest
  // rows reduce onto each row's segment under that symmetry.
  auto orbit_of = [&finest](const RaypathHistogramEntry& e, uint8_t sym) {
    size_t n = 0;
    for (const auto& f : finest.entries_) {
      if (ReduceRaypathByPeriod(f.chain_[0].segment, sym, 0, true, 6) == e.chain_[0].segment) {
        n++;
      }
    }
    return n;
  };
  for (const auto& [r, sym] : { std::pair{ &none, FilterConfig::kSymNone }, std::pair{ &p, FilterConfig::kSymP },
                                std::pair{ &pbd, kSymAll } }) {
    for (const auto& e : r->entries_) {
      const size_t orbit = orbit_of(e, sym);
      EXPECT_GE(orbit, 1u) << e.display_;
      EXPECT_DOUBLE_EQ(e.error_bound_, kFixtureRowError * static_cast<double>(orbit)) << e.display_;
      EXPECT_LE(e.error_bound_, static_cast<double>(orbit) * kFixtureTotalOverK + 1e-12) << e.display_;
    }
  }
  // Symmetry 0 merges nothing: every error is the finest one.
  for (const auto& e : none.entries_) {
    EXPECT_DOUBLE_EQ(e.error_bound_, kFixtureRowError);
  }
  // Under P, {3,5} and {4,6} became one row: error 2 × 0.5 = 1.0 ≤ 2 × E/k.
  const auto* p35 = Find(p, "3-5");
  ASSERT_NE(p35, nullptr);
  EXPECT_DOUBLE_EQ(p35->error_bound_, 2.0 * kFixtureRowError);
  // Under P|B|D the top row is the 3-member orbit {3,5} {4,6} {3,7}: 1.5.
  ASSERT_EQ(pbd.entries_.size(), 2u);
  EXPECT_DOUBLE_EQ(pbd.entries_[0].error_bound_, 3.0 * kFixtureRowError);
  EXPECT_DOUBLE_EQ(pbd.entries_[1].error_bound_, 2.0 * kFixtureRowError);
}

TEST(ReadTimeReduction, DIsAppliedPerLayerWithThatLayersCrystalParameters) {
  // A two-layer chain: crystal 1 (D on) then crystal 2 (D off). {3,7} on
  // crystal 1 is {3,5}'s D-image; on crystal 2 it is not, so the two chains
  // {3,7}->{3,7} and {3,5}->{3,5} merge on layer 0 only, and stay two rows.
  RaypathHistogramResult finest;
  finest.reduce_ctx_.crystal_params_[1] = RaypathCrystalReduceParams{ 0, true };
  finest.reduce_ctx_.crystal_params_[2] = RaypathCrystalReduceParams{ 0, false };
  finest.reduce_ctx_.layer_multi_crystal_ = { false, false };
  auto add = [&finest](Seg a, Seg b, double energy) {
    RaypathHistogramEntry e;
    e.chain_.push_back(RaypathChainSegment{ 1, std::move(a) });
    e.chain_.push_back(RaypathChainSegment{ 2, std::move(b) });
    e.energy_ = energy;
    e.count_ = 1;
    finest.entries_.push_back(std::move(e));
  };
  add({ 3, 5 }, { 3, 5 }, 3.0);
  add({ 3, 7 }, { 3, 7 }, 2.0);
  add({ 3, 7 }, { 3, 5 }, 1.0);
  const auto r = ReduceRaypathHistogram(finest, kSymAll);
  ASSERT_EQ(r.entries_.size(), 2u);
  EXPECT_EQ(r.entries_[0].display_, "(3-5) -> (3-5)");
  EXPECT_DOUBLE_EQ(r.entries_[0].energy_, 4.0);
  EXPECT_EQ(r.entries_[0].count_, 2u);
  EXPECT_EQ(r.entries_[1].display_, "(3-5) -> (3-7)");
  EXPECT_DOUBLE_EQ(r.entries_[1].energy_, 2.0);
  ASSERT_EQ(r.entries_[1].chain_.size(), 2u);
  EXPECT_EQ(r.entries_[1].chain_[0].crystal_id, 1u);
  EXPECT_EQ(r.entries_[1].chain_[1].crystal_id, 2u);
}

TEST(ReadTimeReduction, RingEnergiesAreSummedElementwiseAndTheConeEchoIsKept) {
  RaypathHistogramResult finest;
  finest.roi_mode_ = RaypathRoiMode::kCone;
  finest.cone_ring_count_ = 3;
  finest.cone_radius_rad_ = 0.25f;
  finest.reduce_ctx_.crystal_params_[1] = RaypathCrystalReduceParams{ 0, false };
  finest.reduce_ctx_.layer_multi_crystal_ = { false };
  auto add = [&finest](Seg seg, std::vector<double> rings) {
    RaypathHistogramEntry e;
    e.chain_.push_back(RaypathChainSegment{ 1, std::move(seg) });
    e.count_ = 1;
    for (double v : rings) {
      e.energy_ += v;
    }
    e.ring_energy_ = std::move(rings);
    finest.entries_.push_back(std::move(e));
  };
  add({ 3, 5 }, { 1.0, 2.0, 3.0 });
  add({ 5, 7 }, { 0.5, 0.0, 1.5 });
  const auto r = ReduceRaypathHistogram(finest, FilterConfig::kSymP);
  EXPECT_EQ(r.roi_mode_, RaypathRoiMode::kCone);
  EXPECT_EQ(r.cone_ring_count_, 3);
  EXPECT_FLOAT_EQ(r.cone_radius_rad_, 0.25f);
  ASSERT_EQ(r.entries_.size(), 1u);
  EXPECT_EQ(r.entries_[0].ring_energy_, (std::vector<double>{ 1.5, 2.0, 4.5 }));
  EXPECT_DOUBLE_EQ(r.entries_[0].energy_, 8.0);
  EXPECT_EQ(r.entries_[0].count_, 2u);
}

TEST(ReadTimeReduction, UnknownCrystalIdReducesWithDOffRatherThanFailing) {
  RaypathHistogramResult finest;  // empty context: nothing is described
  finest.reduce_ctx_.layer_multi_crystal_ = { false };
  for (const Seg& seg : { Seg{ 3, 5 }, Seg{ 3, 7 } }) {
    RaypathHistogramEntry e;
    e.chain_.push_back(RaypathChainSegment{ 9, seg });
    e.energy_ = 1.0;
    e.count_ = 1;
    finest.entries_.push_back(std::move(e));
  }
  // With D off (unknown crystal), {3,7} is not {3,5}'s image: two rows under P|B|D.
  const auto r = ReduceRaypathHistogram(finest, kSymAll);
  EXPECT_EQ(r.entries_.size(), 2u);
  ExpectSums(r, 2.0, 2, "unknown crystal");
}

// ---------------------------------------------------------------------------
// The display format, to the character (issue AC4): the four owner-specified
// shapes, plus the defensive branch for a chain deeper than the context.
// ---------------------------------------------------------------------------
TEST(ChainDisplayFormat, FourOwnerSpecifiedShapes) {
  const std::vector<RaypathChainSegment> one{ { 1, { 3, 5 } } };
  EXPECT_EQ(FormatRaypathChainDisplay(one, { false }), "3-5");
  EXPECT_EQ(FormatRaypathChainDisplay(one, { true }), "C1(3-5)");
  const std::vector<RaypathChainSegment> two{ { 1, { 3, 5 } }, { 1, { 1, 3 } } };
  EXPECT_EQ(FormatRaypathChainDisplay(two, { false, false }), "(3-5) -> (1-3)");
  const std::vector<RaypathChainSegment> two_ids{ { 1, { 1, 3 } }, { 4, { 3, 5 } } };
  EXPECT_EQ(FormatRaypathChainDisplay(two_ids, { true, true }), "C1(1-3) -> C4(3-5)");
}

TEST(ChainDisplayFormat, MultiCrystalIsDecidedPerLayerNotPerCrystal) {
  // The same crystal id on both layers, alone on the first and shared on the
  // second: only the second layer names it.
  const std::vector<RaypathChainSegment> chain{ { 2, { 3, 5 } }, { 2, { 1, 3 } } };
  EXPECT_EQ(FormatRaypathChainDisplay(chain, { false, true }), "(3-5) -> C2(1-3)");
  EXPECT_EQ(FormatRaypathChainDisplay(chain, { true, false }), "C2(3-5) -> (1-3)");
}

TEST(ChainDisplayFormat, LayerPastTheContextIsLabelledSingleCrystal) {
  const std::vector<RaypathChainSegment> chain{ { 1, { 3, 5 } }, { 1, { 1, 3 } }, { 1, { 4 } } };
  EXPECT_EQ(FormatRaypathChainDisplay(chain, { true }), "C1(3-5) -> (1-3) -> (4)");
  EXPECT_EQ(FormatRaypathChainDisplay(chain, {}), "(3-5) -> (1-3) -> (4)");
  EXPECT_EQ(FormatRaypathChainDisplay({}, {}), "");
}

// ---------------------------------------------------------------------------
// The context the server builds from a scene, and its round trip through a
// snapshot.
// ---------------------------------------------------------------------------
CrystalConfig PrismWithAxis(IdType id, bool d_applicable_axis) {
  nlohmann::json j = {
    { "id", id },
    { "type", "prism" },
    { "shape", { { "height", 1.2 } } },
    { "axis",
      { { "zenith", { { "type", "uniform" }, { "mean", 90 }, { "std", 360 } } },
        { "azimuth", { { "type", "uniform" }, { "mean", 0 }, { "std", d_applicable_axis ? 360 : 90 } } } } },
  };
  return j.get<CrystalConfig>();
}

ScatteringSetting Setting(const CrystalConfig& crystal) {
  ScatteringSetting s{};
  s.filter_.id_ = 0;
  s.filter_.symmetry_ = FilterConfig::kSymNone;
  s.filter_.action_ = FilterConfig::kFilterIn;
  s.filter_.param_ = SimpleFilterParam{ NoneFilterParam{} };
  s.crystal_ = crystal;
  s.crystal_proportion_ = 1.0f;
  return s;
}

TEST(ReduceContext, LayerFlagIsPerLayerWhenOneCrystalIsAloneOnOneLayerAndSharedOnAnother) {
  // Crystal 1 alone on layer 0; crystals 1 and 2 together on layer 1. A map
  // keyed by crystal id could only hold one answer for crystal 1; the layer
  // vector holds both, in the order the hit loop walks them.
  SceneConfig scene = Halo22Scene();
  scene.ms_.clear();
  const CrystalConfig c1 = PrismWithAxis(1, true);
  const CrystalConfig c2 = PrismWithAxis(2, false);
  MsInfo l0;
  l0.prob_ = 0.5f;
  l0.setting_.push_back(Setting(c1));
  MsInfo l1;
  l1.prob_ = 0.0f;
  l1.setting_.push_back(Setting(c1));
  l1.setting_.push_back(Setting(c2));
  scene.ms_.push_back(std::move(l0));
  scene.ms_.push_back(std::move(l1));

  const RaypathReduceContext ctx = BuildRaypathReduceContext(scene);
  EXPECT_EQ(ctx.layer_multi_crystal_, (std::vector<bool>{ false, true }));
  ASSERT_EQ(ctx.crystal_params_.size(), 2u);
  // The same derivation the simulator's layer context makes, from the axis.
  EXPECT_EQ(ctx.crystal_params_.at(1).d_applicable, detail::IsDApplicable(c1.axis_));
  EXPECT_TRUE(ctx.crystal_params_.at(1).d_applicable);
  EXPECT_EQ(ctx.crystal_params_.at(1).sigma_a, detail::ComputeSigmaA(c1.axis_.roll_dist.center));
  EXPECT_FALSE(ctx.crystal_params_.at(2).d_applicable);
  EXPECT_EQ(ctx.crystal_params_.at(2).sigma_a, 0);

  // Round trip: what the consumer was built with is what its snapshot carries.
  RaypathHistogramConsumer c(FullSky(), ctx);
  const auto r = Snapshot(c);
  EXPECT_EQ(r.reduce_ctx_.layer_multi_crystal_, ctx.layer_multi_crystal_);
  EXPECT_EQ(r.reduce_ctx_.crystal_params_.size(), 2u);
  EXPECT_EQ(r.reduce_ctx_.crystal_params_.at(1).d_applicable, true);
  EXPECT_EQ(r.reduce_ctx_.crystal_params_.at(2).d_applicable, false);
  // And the same layer facts drive the display of a chain through both layers.
  RaypathHistogramResult finest = r;
  RaypathHistogramEntry e;
  e.chain_.push_back(RaypathChainSegment{ 1, { 3, 5 } });
  e.chain_.push_back(RaypathChainSegment{ 1, { 1, 3 } });
  e.energy_ = 1.0;
  e.count_ = 1;
  finest.entries_.push_back(e);
  const auto reduced = ReduceRaypathHistogram(finest, kSymAll);
  ASSERT_EQ(reduced.entries_.size(), 1u);
  EXPECT_EQ(reduced.entries_[0].display_, "(3-5) -> C1(1-3)");
}

// A frame's reduce cache holds one memo per symmetry (0..7), not one shared slot: two callers
// with different, each fixed for their own lifetime, symmetries reading the same frame must not
// evict each other's memo. Interleave reads at two symmetries and check the memo returned for a
// given symmetry is the very same object across both readings (pointer identity), which a shared
// single slot could not provide once the other symmetry had been read in between.
TEST(ReducedRaypathHistogramOf, DistinctSymmetriesKeepIndependentMemosAndDoNotEvictEachOther) {
  RaypathHistogramResult finest;
  RaypathHistogramEntry e;
  e.chain_.push_back(RaypathChainSegment{ 1, { 3, 5 } });
  e.energy_ = 1.0;
  e.count_ = 1;
  finest.entries_.push_back(e);

  ResultFrame frame;
  frame.raypath_histogram_result_ = finest;
  frame.raypath_reduce_cache_ = std::make_shared<ResultFrame::RaypathReduceCache>();

  const auto sym0_first = ReducedRaypathHistogramOf(frame, 0);
  const auto sym7_first = ReducedRaypathHistogramOf(frame, 7);
  const auto sym0_second = ReducedRaypathHistogramOf(frame, 0);
  const auto sym7_second = ReducedRaypathHistogramOf(frame, 7);

  ASSERT_NE(sym0_first, nullptr);
  ASSERT_NE(sym7_first, nullptr);
  EXPECT_EQ(sym0_first.get(), sym0_second.get())
      << "symmetry=0's memo should survive an interleaved symmetry=7 read, not be evicted by it";
  EXPECT_EQ(sym7_first.get(), sym7_second.get())
      << "symmetry=7's memo should survive an interleaved symmetry=0 read, not be evicted by it";
}

}  // namespace
}  // namespace lumice
