#ifndef CORE_LENS_PROJ_H_
#define CORE_LENS_PROJ_H_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <map>

#include "config/render_config.hpp"
#include "core/geo3d.hpp"
#include "core/math.hpp"
#include "core/projection.hpp"
#include "core/shared/projection_shared.h"

namespace lumice {

// Shared lens-projection primitives used by both server/render.cpp and
// core/scatter_accum.hpp. All 10 per-type `*Project` functions are thin
// wrappers over `lm_proj::ProjectExitToPixel` (single arithmetic source,
// see doc/task-unify-shared-projection). `LensProjParam` remains the
// external descriptor so existing consumers (GetProjFunc, tests) are not
// disturbed.

struct LensProjParam {
  float fov_;
  float short_pix_;
  Rotation rot_;
  RenderConfig::VisibleRange visible_range_;
  int resolution_[2];        // x, y
  int lens_shift_[2];        // dx, dy
  float max_abs_dz_ = 0.0f;  // overlap zone |sky.z| threshold (0 = no overlap)
  float r_scale_ = 1.0f;     // projection r_scale for overlap normalization
};

using ProjFunc = std::function<void(const LensProjParam&, const float*, int*, size_t)>;

namespace lens_proj_internal {

// Build the shared POD from a legacy LensProjParam + a runtime type. Mirrors
// lumice::BuildProjParams(cfg, rot, short_pix) but starts from the
// pre-computed max_abs_dz_ / r_scale_ carried by LensProjParam (lens.hpp
// legacy callers already predigest those in scatter_accum / render.cpp).
inline lm_proj::ProjParams ToShared(const LensProjParam& p, LensParam::LensType type) {
  lm_proj::ProjParams s{};
  s.proj_type = static_cast<int>(type);
  s.img_w = p.resolution_[0];
  s.img_h = p.resolution_[1];
  s.visible_range = static_cast<int>(p.visible_range_);
  s.lens_shift_x = p.lens_shift_[0];
  s.lens_shift_y = p.lens_shift_[1];
  s.max_abs_dz = p.max_abs_dz_;
  s.r_scale = p.r_scale_;
  s.scale = 1.0f;
  s.az0 = 0.0f;

  const float* mat = p.rot_.GetMat();
  std::memcpy(s.rot, mat, 9 * sizeof(float));

  const float fov_rad = p.fov_ * math::kDegreeToRad;
  switch (type) {
    case LensParam::kLinear:
      s.scale = p.short_pix_ / 2.0f / std::tan(fov_rad / 2.0f);
      break;
    case LensParam::kFisheyeEqualArea:
      s.scale = p.short_pix_ / 2.0f / std::sqrt(2.0f) / std::sin(fov_rad / 4.0f);
      break;
    case LensParam::kFisheyeEquidistant:
      s.scale = p.short_pix_ * math::kPi_2 / fov_rad;
      break;
    case LensParam::kFisheyeStereographic:
      s.scale = p.short_pix_ / 2.0f / std::tan(fov_rad / 4.0f);
      break;
    case LensParam::kFisheyeOrthographic:
      s.scale = p.short_pix_ / 2.0f / std::sin(fov_rad / 2.0f);
      break;
    case LensParam::kRectangular: {
      auto short_res = std::min(p.resolution_[0] / 2, p.resolution_[1]);
      s.scale = static_cast<float>(short_res) / math::kPi;
      float ax_z[3]{ 0, 0, 1 };
      p.rot_.Apply(ax_z);
      s.az0 = std::atan2(ax_z[1], ax_z[0]);
      break;
    }
    case LensParam::kDualFisheyeEqualArea:
    case LensParam::kDualFisheyeEquidistant:
    case LensParam::kDualFisheyeStereographic:
    case LensParam::kDualFisheyeOrthographic:
    case LensParam::kGlobe:
      // Dual-fisheye types: scale unused (r_scale carries the coverage
      // control); az0 unused. kGlobe reserved for 315.4.
      break;
  }
  return s;
}

// Common thin-wrapper body: for each ray, dispatch to ProjectExitToPixel
// and write the main hit to xy (miss → {-1,-1}). Overlap dual-write is
// consumed by ScatterOutgoingToXyz / render.cpp directly, not by
// GetProjFunc — legacy *Project callers only ever cared about hits[0].
inline void ProjectMainHits(const LensProjParam& p, LensParam::LensType type, const float* d, int* xy, size_t num) {
  auto shared = ToShared(p, type);
  for (size_t i = 0; i < num; i++, d += 3, xy += 2) {
    auto r = lm_proj::ProjectExitToPixel(shared, d[0], d[1], d[2]);
    if (r.count == 0) {
      xy[0] = -1;
      xy[1] = -1;
    } else {
      xy[0] = r.hits[0].px;
      xy[1] = r.hits[0].py;
    }
  }
}

}  // namespace lens_proj_internal

inline void LinearProject(const LensProjParam& p, const float* d, int* xy, size_t num = 1) {
  lens_proj_internal::ProjectMainHits(p, LensParam::kLinear, d, xy, num);
}

inline void FisheyeEqualAreaProject(const LensProjParam& p, const float* d, int* xy, size_t num = 1) {
  lens_proj_internal::ProjectMainHits(p, LensParam::kFisheyeEqualArea, d, xy, num);
}

inline void FisheyeEquidistantProject(const LensProjParam& p, const float* d, int* xy, size_t num = 1) {
  lens_proj_internal::ProjectMainHits(p, LensParam::kFisheyeEquidistant, d, xy, num);
}

inline void FisheyeStereographicProject(const LensProjParam& p, const float* d, int* xy, size_t num = 1) {
  lens_proj_internal::ProjectMainHits(p, LensParam::kFisheyeStereographic, d, xy, num);
}

inline void FisheyeOrthographicProject(const LensProjParam& p, const float* d, int* xy, size_t num = 1) {
  lens_proj_internal::ProjectMainHits(p, LensParam::kFisheyeOrthographic, d, xy, num);
}

inline void DualFisheyeEqualAreaProject(const LensProjParam& p, const float* d, int* xy, size_t num = 1) {
  lens_proj_internal::ProjectMainHits(p, LensParam::kDualFisheyeEqualArea, d, xy, num);
}

inline void DualFisheyeEquidistantProject(const LensProjParam& p, const float* d, int* xy, size_t num = 1) {
  lens_proj_internal::ProjectMainHits(p, LensParam::kDualFisheyeEquidistant, d, xy, num);
}

inline void DualFisheyeStereographicProject(const LensProjParam& p, const float* d, int* xy, size_t num = 1) {
  lens_proj_internal::ProjectMainHits(p, LensParam::kDualFisheyeStereographic, d, xy, num);
}

inline void DualFisheyeOrthographicProject(const LensProjParam& p, const float* d, int* xy, size_t num = 1) {
  lens_proj_internal::ProjectMainHits(p, LensParam::kDualFisheyeOrthographic, d, xy, num);
}

inline void RectangularProject(const LensProjParam& p, const float* d, int* xy, size_t num = 1) {
  lens_proj_internal::ProjectMainHits(p, LensParam::kRectangular, d, xy, num);
}

inline ProjFunc GetProjFunc(LensParam::LensType type) {
  static const std::map<LensParam::LensType, ProjFunc> lens_proj_map{
    { LensParam::kLinear, LinearProject },
    { LensParam::kFisheyeEqualArea, FisheyeEqualAreaProject },
    { LensParam::kFisheyeEquidistant, FisheyeEquidistantProject },
    { LensParam::kFisheyeStereographic, FisheyeStereographicProject },
    { LensParam::kDualFisheyeEqualArea, DualFisheyeEqualAreaProject },
    { LensParam::kDualFisheyeEquidistant, DualFisheyeEquidistantProject },
    { LensParam::kDualFisheyeStereographic, DualFisheyeStereographicProject },
    { LensParam::kRectangular, RectangularProject },
    { LensParam::kFisheyeOrthographic, FisheyeOrthographicProject },
    { LensParam::kDualFisheyeOrthographic, DualFisheyeOrthographicProject },
  };
  return lens_proj_map.at(type);
}

}  // namespace lumice

#endif  // CORE_LENS_PROJ_H_
