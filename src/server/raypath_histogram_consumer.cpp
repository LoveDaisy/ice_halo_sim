#include "server/raypath_histogram_consumer.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "core/color_util.hpp"
#include "core/lens_proj_build.hpp"
#include "core/scatter_accum.hpp"

namespace lumice {

RaypathHistogramConsumer::RaypathHistogramConsumer(RaypathRoiSpec roi) : roi_(std::move(roi)) {
  switch (roi_.mode_) {
    case RaypathRoiMode::kFullSky:
      break;
    case RaypathRoiMode::kInFrame: {
      const auto& cfg = roi_.frame_config_;
      rot_ = MakeCameraRotation(cfg);
      const float short_pix = static_cast<float>(std::min(cfg.resolution_[0], cfg.resolution_[1]));
      proj_params_ = BuildProjParams(cfg, rot_, short_pix);
      mask_detail::CameraForward(rot_, forward_);
      break;
    }
    case RaypathRoiMode::kCone: {
      float* c = roi_.cone_center_;
      const float len = std::sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]);
      if (!(len > 0.0f) || !std::isfinite(len)) {
        ILOG_ERROR(logger_, "cone centre is not a direction (|c|={}); falling back to +z", len);
        c[0] = 0.0f;
        c[1] = 0.0f;
        c[2] = 1.0f;
      } else {
        c[0] /= len;
        c[1] /= len;
        c[2] /= len;
      }
      if (!(roi_.cone_radius_rad_ > 0.0f)) {
        ILOG_ERROR(logger_, "cone radius must be positive (got {}); nothing will be counted", roi_.cone_radius_rad_);
      }
      if (roi_.cone_ring_count_ < 1) {
        ILOG_WARN(logger_, "cone ring count {} < 1; using 1", roi_.cone_ring_count_);
        roi_.cone_ring_count_ = 1;
      }
      cos_radius_ = std::cos(roi_.cone_radius_rad_);
      ring_width_rad_ = roi_.cone_radius_rad_ / static_cast<float>(roi_.cone_ring_count_);
      break;
    }
  }
}

bool RaypathHistogramConsumer::InFrame(float wx, float wy, float wz) const {
  const auto hit = lm_proj::ProjectExitToPixel(proj_params_, wx, wy, wz);
  if (hit.count <= 0) {
    return false;
  }
  // hits[0] is the main projection; a dual-fisheye overlap dual-write (hits[1])
  // never lands a ray the main hit did not, so it plays no part here.
  const int px = hit.hits[0].px;
  const int py = hit.hits[0].py;
  const auto& cfg = roi_.frame_config_;
  if (px < 0 || px >= cfg.resolution_[0] || py < 0 || py >= cfg.resolution_[1]) {
    return false;
  }
  return mask_detail::VisibleByRange(cfg.visible_, wz) && mask_detail::FrontVisible(cfg.front_, forward_, wx, wy, wz);
}

bool RaypathHistogramConsumer::ConeMembership(float wx, float wy, float wz, int* out_ring) const {
  if (!(roi_.cone_radius_rad_ > 0.0f)) {
    return false;
  }
  const float* c = roi_.cone_center_;
  const float dot = c[0] * wx + c[1] * wy + c[2] * wz;
  // Membership on the dot product itself: monotone, scale-free, no epsilon.
  if (dot < cos_radius_) {
    return false;
  }
  // The ring needs the angle. The clamp only keeps a dot pushed past ±1 by
  // rounding out of acos's NaN — it is not part of the decision.
  const float angle = std::acos(std::min(1.0f, std::max(-1.0f, dot)));
  int ring = static_cast<int>(std::floor(angle / ring_width_rad_));
  *out_ring = std::min(roi_.cone_ring_count_ - 1, std::max(0, ring));
  return true;
}

void RaypathHistogramConsumer::Consume(const SimData& data) {
  if (data.outgoing_chain_id_.empty()) {
    // No chain ids: a GPU / CpuTraceBackend batch, or a Simulator whose
    // analysis mode is off. Not a fault of this batch, and not worth a line —
    // a run mixing such batches in is expected to see many of them.
    return;
  }
  // The whole delta first, unconditionally: later batches name these entries
  // as parents whether or not any ray of THIS batch survives the ROI.
  const auto report = merger_.Absorb(data.producer_effective_seed_, data.chain_id_table_delta_);
  if ((report.orphaned > 0 || report.non_monotonic > 0) && !logged_delta_contract_) {
    ILOG_WARN(logger_,
              "chain-id delta contract broken by producer {}: {} orphaned entries, {} non-ascending ids "
              "(two workers sharing one producer key, or a producer restarted without Reset()); "
              "reported once",
              data.producer_effective_seed_, report.orphaned, report.non_monotonic);
    logged_delta_contract_ = true;
  }

  const size_t n = data.outgoing_w_.size();
  if (data.outgoing_chain_id_.size() != n) {
    if (!logged_delta_contract_) {
      ILOG_WARN(logger_, "outgoing_chain_id_ has {} entries for {} rays; batch skipped (reported once)",
                data.outgoing_chain_id_.size(), n);
      logged_delta_contract_ = true;
    }
    return;
  }
  const bool per_ray_wl = !data.outgoing_wl_.empty();
  const bool cone = roi_.mode_ == RaypathRoiMode::kCone;
  for (size_t i = 0; i < n; i++) {
    int ring = 0;
    bool counted = true;
    switch (roi_.mode_) {
      case RaypathRoiMode::kFullSky:
        break;
      case RaypathRoiMode::kInFrame:
        counted = InFrame(data.outgoing_d_[i * 3 + 0], data.outgoing_d_[i * 3 + 1], data.outgoing_d_[i * 3 + 2]);
        break;
      case RaypathRoiMode::kCone:
        counted = ConeMembership(data.outgoing_d_[i * 3 + 0], data.outgoing_d_[i * 3 + 1], data.outgoing_d_[i * 3 + 2],
                                 &ring);
        break;
    }
    if (!counted) {
      continue;
    }
    const uint32_t merged_id = merger_.Resolve(data.producer_effective_seed_, data.outgoing_chain_id_[i]);
    if (merged_id == ChainIdMerger::kUnresolved) {
      if (!logged_unresolved_) {
        ILOG_ERROR(logger_, "ray carries chain id {} that producer {} never delivered; ray dropped (reported once)",
                   data.outgoing_chain_id_[i], data.producer_effective_seed_);
        logged_unresolved_ = true;
      }
      continue;
    }
    const float wl = per_ray_wl ? data.outgoing_wl_[i] : data.curr_wl_;
    const double y = SpectrumToYSingle(wl, data.outgoing_w_[i]);
    auto& e = live_[merged_id];
    e.energy_ += y;
    e.count_ += 1;
    if (cone) {
      if (e.ring_energy_.empty()) {
        e.ring_energy_.assign(static_cast<size_t>(roi_.cone_ring_count_), 0.0);
      }
      e.ring_energy_[static_cast<size_t>(ring)] += y;
    }
    roi_hit_count_ += 1;
  }
}

void RaypathHistogramConsumer::PrepareSnapshot() {
  snapshot_entries_.clear();
  snapshot_entries_.reserve(live_.size());
  const auto& table = merger_.Table();
  for (const auto& [id, e] : live_) {
    RaypathHistogramEntry out;
    for (const auto& seg : table.Segments(id)) {
      out.chain_.push_back(RaypathChainSegment{ seg.crystal_id, seg.segment });
    }
    out.display_ = table.Format(id);
    out.energy_ = e.energy_;
    out.count_ = e.count_;
    out.ring_energy_ = e.ring_energy_;
    snapshot_entries_.push_back(std::move(out));
  }
  std::sort(snapshot_entries_.begin(), snapshot_entries_.end(),
            [](const RaypathHistogramEntry& a, const RaypathHistogramEntry& b) {
              if (a.energy_ != b.energy_) {
                return a.energy_ > b.energy_;
              }
              return a.display_ < b.display_;
            });
}

Result RaypathHistogramConsumer::GetResult() const {
  RaypathHistogramResult r;
  r.entries_ = snapshot_entries_;
  r.roi_mode_ = roi_.mode_;
  if (roi_.mode_ == RaypathRoiMode::kCone) {
    r.cone_ring_count_ = roi_.cone_ring_count_;
    r.cone_radius_rad_ = roi_.cone_radius_rad_;
  }
  return r;
}

void RaypathHistogramConsumer::Reset() {
  live_.clear();
  snapshot_entries_.clear();
  merger_.Clear();
  roi_hit_count_ = 0;
  logged_unresolved_ = false;
  logged_delta_contract_ = false;
}

bool RaypathHistogramConsumer::RoiTargetReached() const {
  return roi_.mode_ == RaypathRoiMode::kCone && roi_.cone_stop_target_ > 0 && roi_hit_count_ >= roi_.cone_stop_target_;
}

}  // namespace lumice
