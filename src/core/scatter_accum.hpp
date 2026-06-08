#ifndef CORE_SCATTER_ACCUM_H_
#define CORE_SCATTER_ACCUM_H_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <memory>

#include "config/render_config.hpp"
#include "core/color_util.hpp"
#include "core/geo3d.hpp"
#include "core/lens_proj.hpp"
#include "core/math.hpp"
#include "core/projection.hpp"

namespace lumice {

namespace scatter_accum_internal {

// Build LensProjParam + camera rotation from a RenderConfig. Mirrors the
// equivalent setup performed in RenderConsumer's constructor and Consume()
// (see server/render.cpp). Extracted to a free function so backends can build
// projection state without instantiating RenderConsumer.
inline LensProjParam MakeProjParam(const RenderConfig& cfg, const Rotation& rot, float short_pix) {
  LensProjParam p{ cfg.lens_.fov_,
                   short_pix,
                   rot,
                   cfg.visible_,
                   { cfg.resolution_[0], cfg.resolution_[1] },
                   { cfg.lens_shift_[0], cfg.lens_shift_[1] },
                   0.0f,
                   1.0f };
  if (cfg.overlap_ > 0) {
    switch (cfg.lens_.type_) {
      case LensParam::kDualFisheyeEqualArea:
        p.max_abs_dz_ = cfg.overlap_;
        p.r_scale_ = projection::ComputeEARScale(cfg.overlap_);
        break;
      case LensParam::kDualFisheyeEquidistant:
        p.max_abs_dz_ = cfg.overlap_;
        p.r_scale_ = ComputeEDRScale(cfg.overlap_);
        break;
      case LensParam::kDualFisheyeStereographic:
        p.max_abs_dz_ = cfg.overlap_;
        p.r_scale_ = ComputeSTRScale(cfg.overlap_);
        break;
      default:
        // Non-dual-fisheye: overlap ignored. Dual orthographic also falls here
        // (overlap support is deferred — see server/render.cpp).
        break;
    }
  }
  return p;
}

inline projection::ProjXY OverlapForward(LensParam::LensType type, float sky_x, float sky_y, float z_hemi, float r_s) {
  switch (type) {
    case LensParam::kDualFisheyeEqualArea:
      return projection::FisheyeEqualAreaForward(sky_x, sky_y, z_hemi, r_s);
    case LensParam::kDualFisheyeEquidistant:
      return projection::FisheyeEquidistantForward(sky_x, sky_y, z_hemi, r_s);
    case LensParam::kDualFisheyeStereographic:
      return projection::FisheyeStereographicForward(sky_x, sky_y, z_hemi, r_s);
    case LensParam::kDualFisheyeOrthographic:
      return projection::FisheyeOrthographicForward(sky_x, sky_y, z_hemi, r_s);
    default:
      return projection::ProjXY{ 0, 0, false };
  }
}

}  // namespace scatter_accum_internal

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
// dual-fisheye overlap dual-write pass. The scratch_* buffers are allocated
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
  auto proj_param = scatter_accum_internal::MakeProjParam(cfg, camera_rot, short_pix);

  // Per-call scratch. d is read-only throughout; w_buf and xy_buf need
  // writable storage for in-place compaction in Pass 1.
  auto w_buf = std::make_unique<float[]>(count);
  auto xy_buf = std::make_unique<int[]>(count * 2);
  std::memcpy(w_buf.get(), w, count * sizeof(float));

  auto lens_proj = GetProjFunc(cfg.lens_.type_);
  // lens_proj reads d to compute pixel coords; it does not modify the array.
  lens_proj(proj_param, const_cast<float*>(d), xy_buf.get(), count);

  // Save weights before pass-1 compaction (pass 2 reads originals by ray idx).
  std::unique_ptr<float[]> overlap_w_buf;
  if (proj_param.max_abs_dz_ > 0) {
    overlap_w_buf = std::make_unique<float[]>(count);
    std::memcpy(overlap_w_buf.get(), w_buf.get(), count * sizeof(float));
  }

  // Pass 1: in-bounds compaction + spectrum scatter-add.
  size_t final_ray_num = 0;
  float landed_weight = 0;
  for (size_t i = 0; i < count; i++) {
    int px = xy_buf[i * 2 + 0];
    int py = xy_buf[i * 2 + 1];
    if (px < 0 || px >= cfg.resolution_[0] ||  //
        py < 0 || py >= cfg.resolution_[1]) {
      continue;
    }
    xy_buf[final_ray_num] = py * cfg.resolution_[0] + px;
    w_buf[final_ray_num] = w_buf[i];
    landed_weight += w_buf[i];
    final_ray_num++;
  }
  SpectrumToXyz(wl, w_buf.get(), xy_buf.get(), xyz_buf, final_ray_num);
  if (opt_landed_weight != nullptr) {
    *opt_landed_weight += landed_weight;
  }

  // Pass 2: overlap dual-write (dual fisheye with overlap zone).
  if (proj_param.max_abs_dz_ > 0) {
    size_t overlap_count = 0;
    int w_res = cfg.resolution_[0];
    int h_res = cfg.resolution_[1];
    for (size_t i = 0; i < count; i++) {
      float sky_x = -d[i * 3 + 0];
      float sky_y = -d[i * 3 + 1];
      float sky_z = -d[i * 3 + 2];
      if (std::abs(sky_z) >= proj_param.max_abs_dz_) {
        continue;
      }
      bool primary_upper = (sky_z >= 0);
      float z_hemi_opp = primary_upper ? -sky_z : sky_z;
      auto proj =
          scatter_accum_internal::OverlapForward(cfg.lens_.type_, sky_x, sky_y, z_hemi_opp, proj_param.r_scale_);
      float fx = 0;
      float fy = 0;
      projection::DualFisheyeToPixel(proj.x, proj.y, !primary_upper, w_res, h_res, &fx, &fy);
      int ox = static_cast<int>(std::floor(fx + 0.5f));
      int oy = static_cast<int>(std::floor(fy + 0.5f));
      if (ox < 0 || ox >= w_res || oy < 0 || oy >= h_res) {
        continue;
      }
      xy_buf[overlap_count] = oy * w_res + ox;
      w_buf[overlap_count] = overlap_w_buf[i];
      overlap_count++;
    }
    if (overlap_count > 0) {
      // Pass 2 does NOT update landed weight (preserves normalization parity
      // with RenderConsumer::Consume).
      SpectrumToXyz(wl, w_buf.get(), xy_buf.get(), xyz_buf, overlap_count);
    }
  }
}

}  // namespace lumice

#endif  // CORE_SCATTER_ACCUM_H_
