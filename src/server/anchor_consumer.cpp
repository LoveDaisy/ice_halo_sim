#include "server/anchor_consumer.hpp"

#include <cstring>

#include "core/anchor_buffer.hpp"
#include "core/color_util.hpp"

namespace lumice {

namespace {
constexpr size_t kAnchorPixels = static_cast<size_t>(kAnchorWidth) * static_cast<size_t>(kAnchorHeight);
}  // namespace

AnchorConsumer::AnchorConsumer()
    : anchor_y_(std::make_unique<float[]>(kAnchorPixels)), proj_params_(BuildAnchorProjParams()) {
  std::memset(anchor_y_.get(), 0, kAnchorPixels * sizeof(float));
}

void AnchorConsumer::Consume(const SimData& data) {
  // Mirrors RenderConsumer::Consume's own opening branch, and for the same reason: a
  // device-fused batch carries no host-side ray directions to project, only the plane the
  // device already accumulated. The two payload forms are mutually exclusive.
  if (!data.xyz_pixel_data_.empty()) {
    AccumulateDevicePlane(data);
    return;
  }
  AccumulateOutgoing(data);
}

void AnchorConsumer::AccumulateOutgoing(const SimData& data) {
  const size_t count = data.outgoing_w_.size();
  if (count == 0) {
    // 0-exit accounting batch (every ray filtered) — nothing landed anywhere, including
    // here. Same no-op the render path takes.
    return;
  }
  const float* d = data.outgoing_d_.data();
  const float* w = data.outgoing_w_.data();
  // Per-ray wavelength when the producer supplies it, else the batch's own. Same rule and
  // same precedence as RenderConsumer, so the anchor's Y is a strict slice of the Y the
  // renderers accumulate rather than a second, differently-clipped spectrum.
  const bool per_ray_wl = !data.outgoing_wl_.empty();
  const float* wl_buf = per_ray_wl ? data.outgoing_wl_.data() : nullptr;

  float* plane = anchor_y_.get();
  for (size_t i = 0; i < count; ++i) {
    const auto hit = lm_proj::ProjectExitToPixel(proj_params_, d[i * 3 + 0], d[i * 3 + 1], d[i * 3 + 2]);
    const float wl = per_ray_wl ? wl_buf[i] : data.curr_wl_;
    const float y = SpectrumToYSingle(wl, w[i]);
    if (y == 0.0f) {
      continue;
    }
    for (int k = 0; k < hit.count; ++k) {
      const int px = hit.hits[k].px;
      const int py = hit.hits[k].py;
      if (px < 0 || px >= kAnchorWidth || py < 0 || py >= kAnchorHeight) {
        continue;
      }
      plane[static_cast<size_t>(py) * static_cast<size_t>(kAnchorWidth) + static_cast<size_t>(px)] += y;
    }
  }
}

void AnchorConsumer::AccumulateDevicePlane(const SimData& data) {
  if (data.anchor_y_pixel_data_.size() != kAnchorPixels) {
    // Either the backend does not accumulate an anchor plane, or it disagrees with this
    // build about the anchor's geometry. Both are silent-wrong-number risks, but neither
    // is recoverable here and neither can be diagnosed from a per-batch log line that
    // would then fire thousands of times; the cross-backend agreement test is what fails
    // loudly. Leaving the plane untouched keeps the scalar at 0, which reads as "no
    // anchor" rather than as a plausible wrong value.
    return;
  }
  const float* src = data.anchor_y_pixel_data_.data();
  float* dst = anchor_y_.get();
  // Plain accumulation, deliberately without the Neumaier compensation the render path
  // carries. The two are not the same problem: a render buffer's float error is visible as
  // banding in the OUTPUT PIXELS, while this plane feeds a single order statistic over
  // box-summed bins, where a relative error of order sqrt(n_batches) * eps is orders of
  // magnitude below the P99's own sampling noise. Compensation here would cost a second
  // 8 MB buffer to move a digit nothing reads.
  for (size_t p = 0; p < kAnchorPixels; ++p) {
    dst[p] += src[p];
  }
}

void AnchorConsumer::PrepareSnapshot() {
  snapshot_l99_sky_ = AnchorL99Sky(anchor_y_.get());
}

void AnchorConsumer::Reset() {
  std::memset(anchor_y_.get(), 0, kAnchorPixels * sizeof(float));
  snapshot_l99_sky_ = 0.0f;
}

}  // namespace lumice
