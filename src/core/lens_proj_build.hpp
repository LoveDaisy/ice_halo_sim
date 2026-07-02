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

// Per-type `scale`/`az0` derivation — the one place this trig lives. Shared
// by BuildProjParams() (RenderConfig-driven) and lens_proj.hpp::ToShared()
// (LensProjParam-driven) so the two callers can never drift apart.
struct ScaleAz0 {
  float scale;
  float az0;
};

inline ScaleAz0 ComputeScaleAz0(LensParam::LensType type, float fov_rad, float short_pix, int res_w, int res_h,
                                const Rotation& rot) {
  ScaleAz0 out{ 1.0f, 0.0f };
  switch (type) {
    case LensParam::kLinear:
      out.scale = short_pix / 2.0f / std::tan(fov_rad / 2.0f);
      break;
    case LensParam::kFisheyeEqualArea:
      out.scale = short_pix / 2.0f / std::sqrt(2.0f) / std::sin(fov_rad / 4.0f);
      break;
    case LensParam::kFisheyeEquidistant:
      out.scale = short_pix * math::kPi_2 / fov_rad;
      break;
    case LensParam::kFisheyeStereographic:
      out.scale = short_pix / 2.0f / std::tan(fov_rad / 4.0f);
      break;
    case LensParam::kFisheyeOrthographic:
      out.scale = short_pix / 2.0f / std::sin(fov_rad / 2.0f);
      break;
    case LensParam::kRectangular: {
      auto short_res = std::min(res_w / 2, res_h);
      out.scale = static_cast<float>(short_res) / math::kPi;
      float ax_z[3]{ 0, 0, 1 };
      rot.Apply(ax_z);
      out.az0 = std::atan2(ax_z[1], ax_z[0]);
      break;
    }
    case LensParam::kGlobe:
      // Globe uses the same focal = img_radius/tan(fov/2) as linear (the GUI
      // shader `globeInverse` computes focal identically). scale == focal so
      // ProjectExitToPixel's globe branch is pure mul; az0 unused.
      out.scale = short_pix / 2.0f / std::tan(fov_rad / 2.0f);
      break;
    case LensParam::kDualFisheyeEqualArea:
    case LensParam::kDualFisheyeEquidistant:
    case LensParam::kDualFisheyeStereographic:
    case LensParam::kDualFisheyeOrthographic:
      // Dual-fisheye types: scale unused (r_scale carries the coverage
      // control); az0 unused.
      break;
  }
  return out;
}

// Host-only helper: assemble the POD lm_proj::ProjParams consumed by
// lm_proj::ProjectExitToPixel. Predigests all trig-heavy setup (per-type
// `scale`, dual-fisheye `r_scale` + overlap threshold, rectangular `az0`)
// so the per-ray function stays branch/mul only. This header is a single
// source of truth for legacy CPU sites: lens_proj.hpp's per-type
// `*Project` thin wrappers (via ComputeScaleAz0), scatter_accum.hpp's
// ScatterOutgoingToXyz, and server/render.cpp's RenderConsumer::Consume.
inline lm_proj::ProjParams BuildProjParams(const RenderConfig& cfg, const Rotation& rot, float short_pix) {
  lm_proj::ProjParams p{};
  p.proj_type = static_cast<int>(cfg.lens_.type_);
  p.img_w = cfg.resolution_[0];
  p.img_h = cfg.resolution_[1];
  p.visible_range = static_cast<int>(cfg.visible_);
  p.lens_shift_x = cfg.lens_shift_[0];
  p.lens_shift_y = cfg.lens_shift_[1];
  p.r_scale = 1.0f;
  p.max_abs_dz = 0.0f;

  // rot[9] carries the row-major camera rotation for single-lens types
  // (kLinear + 4 single-fisheye). Other types don't read it but we fill
  // identity for POD determinism (avoids leaving uninitialized floats).
  const float* mat = rot.GetMat();
  std::memcpy(p.rot, mat, 9 * sizeof(float));

  const float fov_rad = cfg.lens_.fov_ * math::kDegreeToRad;
  const auto sa = ComputeScaleAz0(cfg.lens_.type_, fov_rad, short_pix, cfg.resolution_[0], cfg.resolution_[1], rot);
  p.scale = sa.scale;
  p.az0 = sa.az0;

  switch (cfg.lens_.type_) {
    case LensParam::kLinear:
    case LensParam::kFisheyeEqualArea:
    case LensParam::kFisheyeEquidistant:
    case LensParam::kFisheyeStereographic:
    case LensParam::kFisheyeOrthographic:
    case LensParam::kRectangular:
      break;
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
      // Globe carries no r_scale / max_abs_dz; its perspective scale is in
      // p.scale (ComputeScaleAz0). ProjectExitToPixel's globe branch handles it.
      break;
  }
  return p;
}

}  // namespace lumice

#endif  // CORE_LENS_PROJ_BUILD_H_
