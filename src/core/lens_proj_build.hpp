#ifndef CORE_LENS_PROJ_BUILD_H_
#define CORE_LENS_PROJ_BUILD_H_

#include <algorithm>
#include <cmath>
#include <cstring>

#include "config/render_config.hpp"
#include "core/geo3d.hpp"
#include "core/math.hpp"
#include "core/projection.hpp"
#include "core/shared/projection_shared.h"

namespace lumice {

// Host-only helper: assemble the POD lm_proj::ProjParams consumed by
// lm_proj::ProjectExitToPixel. Predigests all trig-heavy setup (per-type
// `scale`, dual-fisheye `r_scale` + overlap threshold, rectangular `az0`)
// so the per-ray function stays branch/mul only. This header is a single
// source of truth for legacy CPU sites: lens_proj.hpp's per-type
// `*Project` thin wrappers, scatter_accum.hpp::ScatterOutgoingToXyz,
// and server/render.cpp::RenderConsumer::Consume.
inline lm_proj::ProjParams BuildProjParams(const RenderConfig& cfg, const Rotation& rot, float short_pix) {
  lm_proj::ProjParams p{};
  p.proj_type = static_cast<int>(cfg.lens_.type_);
  p.img_w = cfg.resolution_[0];
  p.img_h = cfg.resolution_[1];
  p.visible_range = static_cast<int>(cfg.visible_);
  p.lens_shift_x = cfg.lens_shift_[0];
  p.lens_shift_y = cfg.lens_shift_[1];
  p.scale = 1.0f;
  p.az0 = 0.0f;
  p.r_scale = 1.0f;
  p.max_abs_dz = 0.0f;

  // rot[9] carries the row-major camera rotation for single-lens types
  // (kLinear + 4 single-fisheye). Other types don't read it but we fill
  // identity for POD determinism (avoids leaving uninitialized floats).
  const float* mat = rot.GetMat();
  std::memcpy(p.rot, mat, 9 * sizeof(float));

  const float fov_rad = cfg.lens_.fov_ * math::kDegreeToRad;

  switch (cfg.lens_.type_) {
    case LensParam::kLinear:
      p.scale = short_pix / 2.0f / std::tan(fov_rad / 2.0f);
      break;
    case LensParam::kFisheyeEqualArea:
      p.scale = short_pix / 2.0f / std::sqrt(2.0f) / std::sin(fov_rad / 4.0f);
      break;
    case LensParam::kFisheyeEquidistant:
      p.scale = short_pix * math::kPi_2 / (cfg.lens_.fov_ * math::kDegreeToRad);
      break;
    case LensParam::kFisheyeStereographic:
      p.scale = short_pix / 2.0f / std::tan(fov_rad / 4.0f);
      break;
    case LensParam::kFisheyeOrthographic:
      p.scale = short_pix / 2.0f / std::sin(fov_rad / 2.0f);
      break;
    case LensParam::kRectangular: {
      auto short_res = std::min(cfg.resolution_[0] / 2, cfg.resolution_[1]);
      p.scale = static_cast<float>(short_res) / math::kPi;
      float ax_z[3]{ 0, 0, 1 };
      rot.Apply(ax_z);
      p.az0 = std::atan2(ax_z[1], ax_z[0]);
      break;
    }
    case LensParam::kDualFisheyeEqualArea:
      if (cfg.overlap_ > 0) {
        p.max_abs_dz = cfg.overlap_;
        p.r_scale = projection::ComputeEARScale(cfg.overlap_);
      }
      break;
    case LensParam::kDualFisheyeEquidistant:
      if (cfg.overlap_ > 0) {
        p.max_abs_dz = cfg.overlap_;
        p.r_scale = projection::ComputeEDRScale(cfg.overlap_);
      }
      break;
    case LensParam::kDualFisheyeStereographic:
      if (cfg.overlap_ > 0) {
        p.max_abs_dz = cfg.overlap_;
        p.r_scale = projection::ComputeSTRScale(cfg.overlap_);
      }
      break;
    case LensParam::kDualFisheyeOrthographic:
      // Overlap support requires a non-trivial ComputeORScale derivation and
      // is deferred (see server/render.cpp legacy note). r_scale stays 1.0,
      // max_abs_dz stays 0 → ProjectExitToPixel emits primary hit only.
      break;
    case LensParam::kGlobe:
      // 315.4 will implement globe. ProjectExitToPixel currently returns
      // count=0 for the globe type.
      break;
  }
  return p;
}

}  // namespace lumice

#endif  // CORE_LENS_PROJ_BUILD_H_
