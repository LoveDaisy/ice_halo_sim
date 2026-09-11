// The analysis run's consumer: no image, a histogram of complete raypath
// chains (doc/raypath-analysis-panel.md §3). Every outgoing ray a batch
// delivers carries a chain id (SimData::outgoing_chain_id_); this consumer
// resolves it to a session-wide id through ChainIdMerger, decides whether the
// ray's exit direction is inside the requested ROI, and if so adds the ray's
// luminance Y · weight to that chain's bucket. A snapshot is the buckets
// sorted by energy.
//
// Y comes from the same authority RenderConsumer's colour-class lanes use
// (core/color_util.hpp SpectrumToYSingle) — no normalisation of its own, so
// a chain's energy here and its pixels' Y in a render of the same batches sum
// to the same number (test_raypath_histogram_consumer.cpp pins that).
#ifndef CONSUMER_RAYPATH_HISTOGRAM_H_
#define CONSUMER_RAYPATH_HISTOGRAM_H_

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "config/render_config.hpp"
#include "core/chain_id_table.hpp"
#include "core/geo3d.hpp"
#include "core/shared/projection_shared.h"
#include "server/consumer.hpp"
#include "server/server.hpp"
#include "util/logger.hpp"

namespace lumice {

// The request: which rays count. Only the fields of the chosen mode are read.
struct RaypathRoiSpec {
  RaypathRoiMode mode_ = RaypathRoiMode::kFullSky;
  // kInFrame: the frame whose lens / view / visible / front decide membership.
  RenderConfig frame_config_;
  // kCone: world-space centre direction (normalised at construction; a zero
  // vector is rejected), angular radius, ring count, and the number of
  // in-cone rays after which RoiTargetReached() turns true (0 = never).
  float cone_center_[3]{ 0.0f, 0.0f, 1.0f };
  float cone_radius_rad_ = 0.0f;
  int cone_ring_count_ = 1;
  size_t cone_stop_target_ = 0;
};

class RaypathHistogramConsumer : public IConsume {
 public:
  explicit RaypathHistogramConsumer(RaypathRoiSpec roi);

  void Consume(const SimData& data) override;
  void PrepareSnapshot() override;
  Result GetResult() const override;
  void Reset() override;

  // Live (un-snapshotted) number of rays counted into the ROI so far. Same
  // thread contract as StatsConsumer::LiveSimRays(): the caller holds the
  // server's consumer mutex, under which Consume() mutates the counter.
  size_t LiveRoiHitCount() const { return roi_hit_count_; }

  // kCone with a non-zero cone_stop_target_: whether LiveRoiHitCount() has
  // reached it. Always false in the other modes — their run length is the
  // ray budget the server already enforces, not this consumer's business.
  // Same thread contract as LiveRoiHitCount().
  bool RoiTargetReached() const;

  const RaypathRoiSpec& Roi() const { return roi_; }

 private:
  struct Entry {
    double energy_ = 0.0;
    size_t count_ = 0;
    std::vector<double> ring_energy_;  // kCone only, sized on first touch
  };

  // Decision B in the design record: the frame test is the forward
  // projection plus the two display clips, on the ray's world direction.
  bool InFrame(float wx, float wy, float wz) const;
  // Membership is the dot-product threshold; `*out_ring` (written only when
  // true) is the angular-distance ring, floor(angle / ring width) clamped to
  // the last ring.
  bool ConeMembership(float wx, float wy, float wz, int* out_ring) const;

  RaypathRoiSpec roi_;

  // kInFrame, computed once from frame_config_ the way RenderConsumer does.
  Rotation rot_;
  lm_proj::ProjParams proj_params_{};
  float forward_[3]{ 0.0f, 0.0f, 0.0f };
  // kCone.
  float cos_radius_ = 1.0f;
  float ring_width_rad_ = 0.0f;

  ChainIdMerger merger_;
  std::unordered_map<uint32_t, Entry> live_;
  size_t roi_hit_count_ = 0;
  std::vector<RaypathHistogramEntry> snapshot_entries_;

  // One-shot diagnostics: the data contract is either honoured for the whole
  // session or broken on the first batch, so repeating the line buys nothing.
  bool logged_unresolved_ = false;
  bool logged_delta_contract_ = false;

  Logger logger_{ "RaypathHistogram" };
};

}  // namespace lumice

#endif  // CONSUMER_RAYPATH_HISTOGRAM_H_
