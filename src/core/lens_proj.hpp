#ifndef CORE_LENS_PROJ_H_
#define CORE_LENS_PROJ_H_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <map>

#include "config/render_config.hpp"
#include "core/geo3d.hpp"
#include "core/lens_proj_build.hpp"
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

// Build the shared POD from a legacy LensProjParam + a runtime type. Starts
// from the pre-computed max_abs_dz_ / r_scale_ carried by LensProjParam
// (predigested by lens_proj.hpp's own callers — currently test_projection.cpp
// and any future direct `*Project` caller; scatter_accum.hpp / render.cpp
// build lm_proj::ProjParams directly via BuildProjParams(cfg, rot, short_pix)
// and do not go through this path). The per-type `scale`/`az0` derivation
// itself is NOT duplicated here — it delegates to
// lumice::ComputeScaleAz0() (lens_proj_build.hpp), the single source of
// truth also used by BuildProjParams().
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

  const float* mat = p.rot_.GetMat();
  std::memcpy(s.rot, mat, 9 * sizeof(float));

  const float fov_rad = p.fov_ * math::kDegreeToRad;
  const auto sa = ComputeScaleAz0(type, fov_rad, p.short_pix_, p.resolution_[0], p.resolution_[1], p.rot_);
  s.scale = sa.scale;
  s.az0 = sa.az0;
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
