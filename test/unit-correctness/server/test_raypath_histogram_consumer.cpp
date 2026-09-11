// RaypathHistogramConsumer: synthetic SimData batches (no simulation) pin the
// three ROI modes, the cross-worker merge, the exact ring split, the stop
// counter and the sort order; one real-Simulator scene pins that the top
// chain of a 22° halo is the 22° raypath.

#include <gtest/gtest.h>

#include <cstdint>
#include <variant>
#include <vector>

#include "config/sim_data.hpp"
#include "core/chain_id_table.hpp"
#include "core/color_util.hpp"
#include "server/raypath_histogram_consumer.hpp"

namespace lumice {
namespace {

using Seg = std::vector<IdType>;

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
};

RaypathHistogramResult Snapshot(RaypathHistogramConsumer& c) {
  c.PrepareSnapshot();
  auto r = c.GetResult();
  EXPECT_TRUE(std::holds_alternative<RaypathHistogramResult>(r));
  return std::get<RaypathHistogramResult>(r);
}

// Step-3 smoke: one chain, one ray, full sky.
TEST(RaypathHistogramConsumer, OneRayOneChainFullSky) {
  RaypathRoiSpec roi;
  roi.mode_ = RaypathRoiMode::kFullSky;
  RaypathHistogramConsumer c(roi);
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
  EXPECT_DOUBLE_EQ(r.entries_[0].energy_, static_cast<double>(SpectrumToYSingle(550.0f, 0.5f)));
  EXPECT_TRUE(r.entries_[0].ring_energy_.empty());
  EXPECT_EQ(c.LiveRoiHitCount(), 1u);
  EXPECT_FALSE(c.RoiTargetReached());
}

}  // namespace
}  // namespace lumice
