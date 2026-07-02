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

namespace lumice {

// Shared lens-projection primitives used by both server/render.cpp and
// core/scatter_accum.hpp. Functions and constants in this header are tagged
// `inline` / `inline constexpr` so multi-TU include is ODR-safe.
//
// Behaviour is identical to the legacy file-local definitions that used to
// live in server/render.cpp — this is a pure header extraction.

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

inline void LinearProject(const LensProjParam& p, const float* d, int* xy, size_t num = 1) {
  float scale = p.short_pix_ / 2.0f / std::tan(p.fov_ / 2.0f * math::kDegreeToRad);
  for (size_t i = 0; i < num; i++, d += 3, xy += 2) {
    if ((p.visible_range_ == RenderConfig::kUpper && d[2] > 0) ||  //
        (p.visible_range_ == RenderConfig::kLower && d[2] < 0)) {
      xy[0] = -1;
      xy[1] = -1;
      continue;
    }

    float d_cam[3]{ -d[0], -d[1], -d[2] };
    p.rot_.ApplyInverse(d_cam);
    auto proj = projection::LinearForward(d_cam[0], d_cam[1], d_cam[2]);
    if (!proj.valid) {
      xy[0] = -1;
      xy[1] = -1;
      continue;
    }

    xy[0] = static_cast<int>(std::floor(proj.x * scale + p.resolution_[0] / 2.0f + 0.5f + p.lens_shift_[0]));
    xy[1] = static_cast<int>(std::floor(proj.y * scale + p.resolution_[1] / 2.0f + 0.5f + p.lens_shift_[1]));
  }
}

inline void FisheyeEqualAreaProject(const LensProjParam& p, const float* d, int* xy, size_t num = 1) {
  float scale = p.short_pix_ / 2.0f / std::sqrt(2.0f) / std::sin(p.fov_ / 4.0f * math::kDegreeToRad);

  for (size_t i = 0; i < num; i++, d += 3, xy += 2) {
    if ((p.visible_range_ == RenderConfig::kUpper && d[2] > 0) ||  //
        (p.visible_range_ == RenderConfig::kLower && d[2] < 0)) {
      xy[0] = -1;
      xy[1] = -1;
      continue;
    }

    float d_cam[3]{ -d[0], -d[1], -d[2] };
    p.rot_.ApplyInverse(d_cam);
    if (d_cam[2] <= 0) {
      xy[0] = -1;
      xy[1] = -1;
      continue;
    }
    auto proj = projection::FisheyeEqualAreaForward(d_cam[0], d_cam[1], d_cam[2]);

    xy[0] = static_cast<int>(std::floor(proj.x * scale + p.resolution_[0] / 2.0f + 0.5f + p.lens_shift_[0]));
    xy[1] = static_cast<int>(std::floor(proj.y * scale + p.resolution_[1] / 2.0f + 0.5f + p.lens_shift_[1]));
  }
}

inline void FisheyeEquidistantProject(const LensProjParam& p, const float* d, int* xy, size_t num = 1) {
  float scale = p.short_pix_ * math::kPi_2 / (p.fov_ * math::kDegreeToRad);

  for (size_t i = 0; i < num; i++, d += 3, xy += 2) {
    if ((p.visible_range_ == RenderConfig::kUpper && d[2] > 0) ||  //
        (p.visible_range_ == RenderConfig::kLower && d[2] < 0)) {
      xy[0] = -1;
      xy[1] = -1;
      continue;
    }

    float d_cam[3]{ -d[0], -d[1], -d[2] };
    p.rot_.ApplyInverse(d_cam);
    if (d_cam[2] <= 0) {
      xy[0] = -1;
      xy[1] = -1;
      continue;
    }
    auto proj = projection::FisheyeEquidistantForward(d_cam[0], d_cam[1], d_cam[2]);

    xy[0] = static_cast<int>(std::floor(proj.x * scale + p.resolution_[0] / 2.0f + 0.5f + p.lens_shift_[0]));
    xy[1] = static_cast<int>(std::floor(proj.y * scale + p.resolution_[1] / 2.0f + 0.5f + p.lens_shift_[1]));
  }
}

inline void FisheyeStereographicProject(const LensProjParam& p, const float* d, int* xy, size_t num = 1) {
  float scale = p.short_pix_ / 2.0f / std::tan(p.fov_ / 4.0f * math::kDegreeToRad);

  for (size_t i = 0; i < num; i++, d += 3, xy += 2) {
    if ((p.visible_range_ == RenderConfig::kUpper && d[2] > 0) ||  //
        (p.visible_range_ == RenderConfig::kLower && d[2] < 0)) {
      xy[0] = -1;
      xy[1] = -1;
      continue;
    }

    float d_cam[3]{ -d[0], -d[1], -d[2] };
    p.rot_.ApplyInverse(d_cam);
    if (d_cam[2] <= 0) {
      xy[0] = -1;
      xy[1] = -1;
      continue;
    }
    auto proj = projection::FisheyeStereographicForward(d_cam[0], d_cam[1], d_cam[2]);

    xy[0] = static_cast<int>(std::floor(proj.x * scale + p.resolution_[0] / 2.0f + 0.5f + p.lens_shift_[0]));
    xy[1] = static_cast<int>(std::floor(proj.y * scale + p.resolution_[1] / 2.0f + 0.5f + p.lens_shift_[1]));
  }
}

inline void FisheyeOrthographicProject(const LensProjParam& p, const float* d, int* xy, size_t num = 1) {
  float scale = p.short_pix_ / 2.0f / std::sin(p.fov_ / 2.0f * math::kDegreeToRad);

  for (size_t i = 0; i < num; i++, d += 3, xy += 2) {
    if ((p.visible_range_ == RenderConfig::kUpper && d[2] > 0) ||  //
        (p.visible_range_ == RenderConfig::kLower && d[2] < 0)) {
      xy[0] = -1;
      xy[1] = -1;
      continue;
    }

    float d_cam[3]{ -d[0], -d[1], -d[2] };
    p.rot_.ApplyInverse(d_cam);
    if (d_cam[2] <= 0) {
      xy[0] = -1;
      xy[1] = -1;
      continue;
    }
    auto proj = projection::FisheyeOrthographicForward(d_cam[0], d_cam[1], d_cam[2]);

    xy[0] = static_cast<int>(std::floor(proj.x * scale + p.resolution_[0] / 2.0f + 0.5f + p.lens_shift_[0]));
    xy[1] = static_cast<int>(std::floor(proj.y * scale + p.resolution_[1] / 2.0f + 0.5f + p.lens_shift_[1]));
  }
}

inline void DualFisheyeEqualAreaProject(const LensProjParam& p, const float* d, int* xy, size_t num = 1) {
  for (size_t i = 0; i < num; i++, d += 3, xy += 2) {
    float sky_x = -d[0], sky_y = -d[1], sky_z = -d[2];
    bool is_upper = (sky_z >= 0);
    float z_hemi = is_upper ? sky_z : -sky_z;
    auto proj = projection::FisheyeEqualAreaForward(sky_x, sky_y, z_hemi, p.r_scale_);
    float fx, fy;
    projection::DualFisheyeToPixel(proj.x, proj.y, is_upper, p.resolution_[0], p.resolution_[1], &fx, &fy);
    xy[0] = static_cast<int>(std::floor(fx + 0.5f));
    xy[1] = static_cast<int>(std::floor(fy + 0.5f));
  }
}

inline void DualFisheyeEquidistantProject(const LensProjParam& p, const float* d, int* xy, size_t num = 1) {
  for (size_t i = 0; i < num; i++, d += 3, xy += 2) {
    float sky_x = -d[0], sky_y = -d[1], sky_z = -d[2];
    bool is_upper = (sky_z >= 0);
    float z_hemi = is_upper ? sky_z : -sky_z;
    auto proj = projection::FisheyeEquidistantForward(sky_x, sky_y, z_hemi, p.r_scale_);
    float fx, fy;
    projection::DualFisheyeToPixel(proj.x, proj.y, is_upper, p.resolution_[0], p.resolution_[1], &fx, &fy);
    xy[0] = static_cast<int>(std::floor(fx + 0.5f));
    xy[1] = static_cast<int>(std::floor(fy + 0.5f));
  }
}

inline void DualFisheyeStereographicProject(const LensProjParam& p, const float* d, int* xy, size_t num = 1) {
  for (size_t i = 0; i < num; i++, d += 3, xy += 2) {
    float sky_x = -d[0], sky_y = -d[1], sky_z = -d[2];
    bool is_upper = (sky_z >= 0);
    float z_hemi = is_upper ? sky_z : -sky_z;
    auto proj = projection::FisheyeStereographicForward(sky_x, sky_y, z_hemi, p.r_scale_);
    float fx, fy;
    projection::DualFisheyeToPixel(proj.x, proj.y, is_upper, p.resolution_[0], p.resolution_[1], &fx, &fy);
    xy[0] = static_cast<int>(std::floor(fx + 0.5f));
    xy[1] = static_cast<int>(std::floor(fy + 0.5f));
  }
}

inline void DualFisheyeOrthographicProject(const LensProjParam& p, const float* d, int* xy, size_t num = 1) {
  for (size_t i = 0; i < num; i++, d += 3, xy += 2) {
    float sky_x = -d[0], sky_y = -d[1], sky_z = -d[2];
    bool is_upper = (sky_z >= 0);
    float z_hemi = is_upper ? sky_z : -sky_z;
    auto proj = projection::FisheyeOrthographicForward(sky_x, sky_y, z_hemi, p.r_scale_);
    float fx, fy;
    projection::DualFisheyeToPixel(proj.x, proj.y, is_upper, p.resolution_[0], p.resolution_[1], &fx, &fy);
    xy[0] = static_cast<int>(std::floor(fx + 0.5f));
    xy[1] = static_cast<int>(std::floor(fy + 0.5f));
  }
}

inline void RectangularProject(const LensProjParam& p, const float* d, int* xy, size_t num = 1) {
  auto short_res = std::min(p.resolution_[0] / 2, p.resolution_[1]);
  float scale = short_res / math::kPi;

  float ax_z[3]{ 0, 0, 1 };
  p.rot_.Apply(ax_z);
  float az0 = std::atan2(ax_z[1], ax_z[0]);
  for (size_t i = 0; i < num; i++, d += 3, xy += 2) {
    auto proj = projection::RectangularForward(-d[0], -d[1], -d[2]);
    float lon = proj.x - az0;  // subtract camera azimuth offset
    while (lon < -math::kPi) {
      lon += 2 * math::kPi;
    }
    while (lon > math::kPi) {
      lon -= 2 * math::kPi;
    }

    int raw_x = static_cast<int>(std::floor(lon * scale + p.resolution_[0] / 2.0f + 0.5f));
    xy[0] = ((raw_x % p.resolution_[0]) + p.resolution_[0]) % p.resolution_[0];
    xy[1] = static_cast<int>(std::floor(-proj.y * scale + p.resolution_[1] / 2.0f + 0.5f));
  }
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
