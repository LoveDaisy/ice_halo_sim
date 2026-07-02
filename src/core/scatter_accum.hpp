#ifndef CORE_SCATTER_ACCUM_H_
#define CORE_SCATTER_ACCUM_H_

#include <algorithm>
#include <cstddef>
#include <memory>

#include "config/render_config.hpp"
#include "core/color_util.hpp"
#include "core/geo3d.hpp"
#include "core/lens_proj_build.hpp"
#include "core/math.hpp"
#include "core/shared/projection_shared.h"

namespace lumice {

// Camera rotation for a RenderConfig — matches RenderConsumer ctor.
inline Rotation MakeCameraRotation(const RenderConfig& cfg) {
  Rotation rot;
  float ax_z[3]{ 0, 0, 1 };
  float ax_y[3]{ 0, 1, 0 };
  rot.Chain({ ax_z, (-90.0f + cfg.view_.ro_) * math::kDegreeToRad })
      .Chain({ ax_y, (90.0f - cfg.view_.el_) * math::kDegreeToRad })
      .Chain({ ax_z, cfg.view_.az_ * math::kDegreeToRad });
  return rot;
}

// Project `count` outgoing rays (d[3*i..], w[i]) through the lens described
// by `cfg` and scatter-add their CIE 1931 contribution at `wl` into the
// caller-provided XYZ buffer (W * H * 3 floats).
//
// Behavioural equivalence: matches RenderConsumer::Consume(), including the
// dual-fisheye overlap dual-write pass. Every ray is projected exactly once
// via lm_proj::ProjectExitToPixel (single arithmetic source); the returned
// PixelHit.bump_landed separates "primary" (counts into landed_weight) from
// "overlap dual-write" (does not) and drains into two batches for the
// existing SpectrumToXyz batch interface. The scratch buffers are allocated
// per call here — for hot paths the backend should cache its own.
//
// This is the GPU "register-resident accumulator drain" boundary on the
// CPU backend: an O(N) pass that turns outgoing rays into XYZ deltas.
//
// NOTE: this function does NOT track total_intensity_. Callers that need
// normalization should sum landed weight separately (the CPU backend stores
// it in CpuTraceBackend::total_landed_weight_ for parity with
// RenderConsumer::total_intensity_).
inline void ScatterOutgoingToXyz(const float* d, const float* w, size_t count, const RenderConfig& cfg,
                                 const Rotation& camera_rot, float wl, float* xyz_buf,
                                 float* opt_landed_weight = nullptr) {
  if (count == 0) {
    return;
  }
  auto short_pix = static_cast<float>(std::min(cfg.resolution_[0], cfg.resolution_[1]));
  const auto proj_params = BuildProjParams(cfg, camera_rot, short_pix);

  // Two batch arrays: main hits (bump_landed=true) and overlap hits (=false).
  // Overlap only fires for dual-fisheye with max_abs_dz>0 (ProjectExitToPixel
  // never emits hits[1] otherwise), so the overlap buffers are only
  // allocated when the projection is actually configured for overlap;
  // allocation is O(count) worst case (every ray in the overlap band).
  auto main_w = std::make_unique<float[]>(count);
  auto main_xy = std::make_unique<int[]>(count);
  const bool has_overlap = proj_params.max_abs_dz > 0.0f;
  std::unique_ptr<float[]> overlap_w;
  std::unique_ptr<int[]> overlap_xy;
  if (has_overlap) {
    overlap_w = std::make_unique<float[]>(count);
    overlap_xy = std::make_unique<int[]>(count);
  }

  const int w_res = cfg.resolution_[0];
  const int h_res = cfg.resolution_[1];

  size_t main_n = 0;
  size_t overlap_n = 0;
  float landed_weight = 0.0f;

  for (size_t i = 0; i < count; ++i) {
    auto hit = lm_proj::ProjectExitToPixel(proj_params, d[i * 3 + 0], d[i * 3 + 1], d[i * 3 + 2]);
    for (int k = 0; k < hit.count; ++k) {
      int px = hit.hits[k].px;
      int py = hit.hits[k].py;
      if (px < 0 || px >= w_res || py < 0 || py >= h_res) {
        continue;
      }
      if (hit.hits[k].bump_landed) {
        main_xy[main_n] = py * w_res + px;
        main_w[main_n] = w[i];
        landed_weight += w[i];
        ++main_n;
      } else {
        overlap_xy[overlap_n] = py * w_res + px;
        overlap_w[overlap_n] = w[i];
        ++overlap_n;
      }
    }
  }

  SpectrumToXyz(wl, main_w.get(), main_xy.get(), xyz_buf, main_n);
  if (opt_landed_weight != nullptr) {
    *opt_landed_weight += landed_weight;
  }
  if (overlap_n > 0) {
    // Overlap batch does NOT update landed weight — preserves normalization
    // parity with RenderConsumer::Consume Pass 2 semantics.
    SpectrumToXyz(wl, overlap_w.get(), overlap_xy.get(), xyz_buf, overlap_n);
  }
}

}  // namespace lumice

#endif  // CORE_SCATTER_ACCUM_H_
