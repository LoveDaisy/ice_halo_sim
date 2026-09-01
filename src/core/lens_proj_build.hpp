#ifndef CORE_LENS_PROJ_BUILD_H_
#define CORE_LENS_PROJ_BUILD_H_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

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


// ==================================================================================================
// Per-pixel render-domain mask
//
// `ProjectExitToPixel` is a FORWARD projection: it maps a sky direction to a pixel and never
// visits a pixel that no ray reaches. So the render path has no notion of "this pixel images
// nothing" — an unlit pixel inside the image circle and a pixel outside the lens entirely are
// both simply zero XYZ. Anything that paints a per-pixel constant (today: the background fill in
// RenderConsumer::PostSnapshot) needs that missing distinction, and this is where it is built.
//
// The mask is the exact inverse of the region ProjectExitToPixel can write to, derived from that
// function's own equations rather than transcribed from the GUI's display shader — see each
// branch below. It depends only on resolution / lens / view / visible / overlap, which is
// precisely the field set NeedsRebuild treats as layout (background is explicitly on the
// appearance-only side), so one build per consumer lifetime is enough and no invalidation
// logic is needed.
//
// PIXEL CENTRE CONVENTION: pixel (px, py) samples at (px + 0.5 - res/2), i.e. an image
// symmetric about the frame centre, matching the GUI shader's `pos = v_ndc * u_resolution *
// 0.5`. `ProjectExitToPixel`'s own binning is `floor(v + 0.5)` about `res/2`, which puts its
// addressable span at [-res/2 - 0.5, res/2 - 0.5) — half a pixel off centre. The mask keeps the
// symmetric convention: it is the one that agrees with the GUI, and the half-pixel bias belongs
// to the forward binning, not here.
namespace mask_detail {

// A world-space direction plus whether the pixel images anything at all.
struct MaskDir {
  float x;
  float y;
  float z;
  bool valid;
};

// Camera-frame direction back to world. ProjectExitToPixel computes c = R^T * (-w)
// (ApplyRotTranspose), so the inverse of that step is w = -R * c.
inline MaskDir CameraDirToWorld(const Rotation& rot, const projection::Dir3& c) {
  if (!c.valid) {
    return { 0.0f, 0.0f, 0.0f, false };
  }
  float d[3]{ c.x, c.y, c.z };
  rot.Apply(d);
  return { -d[0], -d[1], -d[2], true };
}

// Single-lens family (linear + 4 single fisheye). The forward negates the horizontal component
// after projecting (screen handedness: right = +az), so the projected coordinates that produced
// the pixel are (-u, v) and the inverse consumes them in that order.
//
// Each fisheye inverse rejects beyond its own rim radius, and that rim is the same theta the
// forward's per-type cull stops at (projection.cpp's kFisheyeStereographicMaxR and the
// kFisheyeAntipodeMinCz / kFisheyeStereographicMinCz notes in projection_shared.h). That makes
// this test exactly "could a ray have landed here", which is what the mask is for. It is also the
// same region the GUI shader's `fisheyeInverse` accepts — equal-area and equidistant now run to
// theta = 180 deg on both sides, orthographic stops at the equator on both sides, and
// stereographic's core-only numerical cap sits far off any real frame. test_visible_mask_gui_parity.cpp
// asserts that agreement pixel by pixel; before 474.1 it pinned a divergence there instead.
inline MaskDir SingleLensPixelToWorld(LensParam::LensType type, const Rotation& rot, float u, float v) {
  projection::Dir3 c{ 0, 0, 0, false };
  switch (type) {
    case LensParam::kLinear:
      c = projection::LinearInverse(-u, v);
      break;
    case LensParam::kFisheyeEqualArea:
      c = projection::FisheyeEqualAreaInverse(-u, v, 1.0f);
      break;
    case LensParam::kFisheyeEquidistant:
      c = projection::FisheyeEquidistantInverse(-u, v, 1.0f);
      break;
    case LensParam::kFisheyeStereographic:
      c = projection::FisheyeStereographicInverse(-u, v, 1.0f);
      break;
    default:  // kFisheyeOrthographic — the only remaining member of this family
      c = projection::FisheyeOrthographicInverse(-u, v, 1.0f);
      break;
  }
  return CameraDirToWorld(rot, c);
}

// Dual fisheye. The ONLY invalid region is "outside both circles": PixelToDualFisheye already
// returns normalized coordinates with r <= 1, and every per-type inverse rejects only beyond
// that (orthographic rejects r > r_scale, and BuildProjParams never gives dual-orthographic an
// r_scale other than 1). r_scale therefore changes the recovered direction — hence the
// visibility half of the mask — but never the domain half.
inline MaskDir DualFisheyePixelToWorld(LensParam::LensType type, const lm_proj::ProjParams& p, int px, int py) {
  float x_norm = 0.0f;
  float y_norm = 0.0f;
  bool is_upper = false;
  if (!projection::PixelToDualFisheye(static_cast<float>(px) + 0.5f, static_cast<float>(py) + 0.5f, p.img_w, p.img_h,
                                      &x_norm, &y_norm, &is_upper)) {
    return { 0.0f, 0.0f, 0.0f, false };
  }
  projection::Dir3 s{ 0, 0, 0, false };
  switch (type) {
    case LensParam::kDualFisheyeEqualArea:
      s = projection::FisheyeEqualAreaInverse(x_norm, y_norm, p.r_scale);
      break;
    case LensParam::kDualFisheyeEquidistant:
      s = projection::FisheyeEquidistantInverse(x_norm, y_norm, p.r_scale);
      break;
    case LensParam::kDualFisheyeStereographic:
      s = projection::FisheyeStereographicInverse(x_norm, y_norm, p.r_scale);
      break;
    default:  // kDualFisheyeOrthographic
      s = projection::FisheyeOrthographicInverse(x_norm, y_norm, p.r_scale);
      break;
  }
  if (!s.valid) {
    return { 0.0f, 0.0f, 0.0f, false };
  }
  // The forward feeds Forward(sx, sy, |sz|) with s = -w and flips the hemisphere afterwards.
  const float sz = is_upper ? s.z : -s.z;
  return { -s.x, -s.y, -sz, true };
}

// Equirectangular. RectangularInverse itself never rejects, but the forward's
// py = floor(-lat * scale + h/2 + 0.5) only spans h/2 +/- (pi/2) * scale, so when
// scale = min(w/2, h)/pi is set by the WIDTH (any canvas taller than 2:1) there are polar caps
// no latitude maps into. |lat| > pi/2 is that region, and it is the same guard the GUI shader's
// `rectangularInverse` applies. On a 2:1 or wider canvas the caps are empty and the domain half
// of the mask is trivially true — a correct conclusion, not an omitted test.
inline MaskDir RectangularPixelToWorld(const lm_proj::ProjParams& p, int px, int py) {
  const float lon = (static_cast<float>(px) + 0.5f - static_cast<float>(p.img_w) / 2.0f) / p.scale + p.az0;
  const float lat = -(static_cast<float>(py) + 0.5f - static_cast<float>(p.img_h) / 2.0f) / p.scale;
  if (std::fabs(lat) > math::kPi_2) {
    return { 0.0f, 0.0f, 0.0f, false };
  }
  // RectangularForward is fed (-w), so its inverse returns (-w) and w is the negation.
  const projection::Dir3 d = projection::RectangularInverse(lon, lat);
  return { -d.x, -d.y, -d.z, true };
}

// Whether a world direction survives the configured visible-hemisphere restriction. Applied to
// EVERY lens type, deliberately: the GUI shader tests `u_visible` after computing world_dir and
// independently of the lens branch, whereas ProjectExitToPixel's ray cull sits inside the
// single-lens branch only. This is the mask's decision about where the SKY is, so it follows the
// GUI's uniform rule; the ray-cull asymmetry is a separate, pre-existing gap in the shared
// backend header and is not touched here.
inline bool VisibleByRange(RenderConfig::VisibleRange range, float wz) {
  if (range == RenderConfig::kUpper && wz > 0.0f) {
    return false;
  }
  if (range == RenderConfig::kLower && wz < 0.0f) {
    return false;
  }
  return true;
}

// The per-pixel inverse, lens branch and all: pixel (px, py) -> the world direction it images.
// Extracted so that every mask built from this projection reads the SAME inverse. A second
// annotation mask (BuildHorizonMask below) exists today and grid annotations may follow;
// each one re-deriving the direction is how two masks of the same frame start disagreeing about
// where the sky is.
inline MaskDir PixelToWorld(const RenderConfig& cfg, const lm_proj::ProjParams& p, const Rotation& rot, int px,
                            int py) {
  const int width = cfg.resolution_[0];
  const int height = cfg.resolution_[1];
  switch (cfg.lens_.type_) {
    case LensParam::kLinear:
    case LensParam::kFisheyeEqualArea:
    case LensParam::kFisheyeEquidistant:
    case LensParam::kFisheyeStereographic:
    case LensParam::kFisheyeOrthographic: {
      const float u =
          (static_cast<float>(px) + 0.5f - static_cast<float>(width) / 2.0f - static_cast<float>(p.lens_shift_x)) /
          p.scale;
      const float v =
          (static_cast<float>(py) + 0.5f - static_cast<float>(height) / 2.0f - static_cast<float>(p.lens_shift_y)) /
          p.scale;
      return SingleLensPixelToWorld(cfg.lens_.type_, rot, u, v);
    }
    case LensParam::kDualFisheyeEqualArea:
    case LensParam::kDualFisheyeEquidistant:
    case LensParam::kDualFisheyeStereographic:
    case LensParam::kDualFisheyeOrthographic:
      return DualFisheyePixelToWorld(cfg.lens_.type_, p, px, py);
    case LensParam::kRectangular:
      return RectangularPixelToWorld(p, px, py);
    case LensParam::kGlobe: {
      const float x =
          static_cast<float>(px) + 0.5f - static_cast<float>(width) / 2.0f - static_cast<float>(p.lens_shift_x);
      const float y =
          static_cast<float>(py) + 0.5f - static_cast<float>(height) / 2.0f - static_cast<float>(p.lens_shift_y);
      return CameraDirToWorld(rot, projection::GlobeInverse(x, y, p.scale));
    }
  }
  return { 0.0f, 0.0f, 0.0f, false };
}

// Altitude in DEGREES for a world direction, in the GUI shader's own terms
// (preview_renderer.cpp overlayAuxLines: `asin(clamp(-world_dir.z, -1, 1)) * DEG`). Degrees
// rather than the raw z because the line-width rule below is stated in degrees on both sides;
// converting once here is what lets the two implementations be compared without a unit step in
// between.
inline float AltitudeDeg(const MaskDir& dir) {
  return std::asin(std::clamp(-dir.z, -1.0f, 1.0f)) * 180.0f / math::kPi;
}

// Marks the horizon line (altitude = 0) over a whole frame's worth of per-pixel altitude, given
// in degrees. Row-major W*H in, the same W*H out.
//
// TWO masks come in, and they are not the same mask. `imaged` says the pixel carries a direction
// at all, and is what the local gradient is measured across; `drawable` says the annotation is
// allowed on that pixel, and is `imaged` narrowed by the configured visible hemisphere. Measuring
// the gradient across `drawable` instead is a defect with a very specific shape: the horizon IS
// the edge of the visible hemisphere, so under `visible: upper` every pixel on the line has its
// vertical neighbour excluded, the vertical difference reads as zero, the width collapses to the
// 1e-4 clamp and the line vanishes exactly where it was asked for. The GPU has no such trap —
// fwidth reads the quad's fragments, which the `pixel_visible` branch does not remove.
//
// The rule is the preview shader's, transposed one primitive at a time
// (preview_renderer.cpp overlayAuxLines, horizon section):
//   fw_alt = clamp(fwidth(altitude_deg), 1e-4, 2.0);  t = 1 - smoothstep(0, fw_alt*1.5, |alt|)
// `t > 0` — the pixels the shader tints at all — is exactly `|alt| < fw_alt * 1.5`, which is what
// this returns. `fwidth` is |dFdx| + |dFdy|, which off the GPU is a forward difference against the
// right and lower neighbours (the last row/column differences backwards instead, the only place
// this can differ from a rasterizer's derivatives and only by which side of the pixel is sampled).
//
// A FIXED angular half-width is not an option here, which is why this takes the trouble:
// degrees-per-pixel spans orders of magnitude across the lens/FOV space this renderer supports, so
// one constant is either a line invisible at narrow FOV or a band tens of pixels deep at wide FOV.
// A pixel whose neighbours are both outside the lens's domain gets no line: no local scale can be
// estimated there, and guessing one is how a horizon appears in a corner that images nothing.
inline std::vector<uint8_t> HorizonLineFromAltitudeField(const std::vector<float>& alt_deg,
                                                         const std::vector<uint8_t>& imaged,
                                                         const std::vector<uint8_t>& drawable, int width, int height) {
  const size_t n = static_cast<size_t>(width) * static_cast<size_t>(height);
  std::vector<uint8_t> line(n, 0);
  if (alt_deg.size() != n || imaged.size() != n || drawable.size() != n) {
    return line;
  }
  for (int py = 0; py < height; py++) {
    for (int px = 0; px < width; px++) {
      const size_t i = static_cast<size_t>(py) * static_cast<size_t>(width) + static_cast<size_t>(px);
      if (drawable[i] == 0) {
        continue;
      }
      float fw = 0.0f;
      bool any = false;
      const int nx = (px + 1 < width) ? px + 1 : px - 1;
      const int ny = (py + 1 < height) ? py + 1 : py - 1;
      if (nx >= 0 && nx < width) {
        const size_t j = static_cast<size_t>(py) * static_cast<size_t>(width) + static_cast<size_t>(nx);
        if (imaged[j] != 0) {
          fw += std::fabs(alt_deg[j] - alt_deg[i]);
          any = true;
        }
      }
      if (ny >= 0 && ny < height) {
        const size_t j = static_cast<size_t>(ny) * static_cast<size_t>(width) + static_cast<size_t>(px);
        if (imaged[j] != 0) {
          fw += std::fabs(alt_deg[j] - alt_deg[i]);
          any = true;
        }
      }
      if (!any) {
        continue;
      }
      const float half_width = std::clamp(fw, 1e-4f, 2.0f) * 1.5f;
      line[i] = (std::fabs(alt_deg[i]) < half_width) ? 1 : 0;
    }
  }
  return line;
}

}  // namespace mask_detail

// Builds the W*H mask described above: 1 where the lens images a visible piece of sky, 0
// elsewhere. Row-major, indexed py * width + px — the same indexing PostSnapshot walks.
// Returns an empty vector for a degenerate resolution.
inline std::vector<uint8_t> BuildVisibleMask(const RenderConfig& cfg, const Rotation& rot, float short_pix) {
  const int width = cfg.resolution_[0];
  const int height = cfg.resolution_[1];
  if (width <= 0 || height <= 0) {
    return {};
  }
  std::vector<uint8_t> mask(static_cast<size_t>(width) * static_cast<size_t>(height), 0);

  // Reuse the very params the forward path runs on (scale / az0 / r_scale), so the mask cannot
  // drift from the projection it is the inverse of.
  const lm_proj::ProjParams p = BuildProjParams(cfg, rot, short_pix);

  for (int py = 0; py < height; py++) {
    for (int px = 0; px < width; px++) {
      const mask_detail::MaskDir dir = mask_detail::PixelToWorld(cfg, p, rot, px, py);
      const bool on = dir.valid && mask_detail::VisibleByRange(cfg.visible_, dir.z);
      mask[static_cast<size_t>(py) * static_cast<size_t>(width) + static_cast<size_t>(px)] = on ? 1 : 0;
    }
  }
  return mask;
}

// Builds the W*H mask of the CELESTIAL HORIZON annotation: 1 where a line at altitude = 0 should
// be drawn, 0 elsewhere. Same layout and same indexing as BuildVisibleMask, and a SEPARATE array
// rather than a second bit of the visibility mask on purpose — "can this pixel show sky" and "does
// this pixel carry an annotation" are two questions, and future grid annotations are more of the
// second kind, not more bits of the first.
//
// The rule is the preview shader's own (preview_renderer.cpp overlayAuxLines, horizon section):
// draw where |altitude_deg| falls inside 1.5x the local per-pixel altitude gradient, gated on the
// pixel being both imageable AND inside the configured visible hemisphere — the shader applies its
// overlay under exactly that `result.w >= 0.5 && pixel_visible` gate, so a half-sky config shows
// the horizon ending at the sky's edge on both sides rather than only on one.
//
// Returns an empty vector for a degenerate resolution.
inline std::vector<uint8_t> BuildHorizonMask(const RenderConfig& cfg, const Rotation& rot, float short_pix) {
  const int width = cfg.resolution_[0];
  const int height = cfg.resolution_[1];
  if (width <= 0 || height <= 0) {
    return {};
  }
  const size_t n = static_cast<size_t>(width) * static_cast<size_t>(height);
  const lm_proj::ProjParams p = BuildProjParams(cfg, rot, short_pix);

  // One inverse per pixel, not three: the width rule needs each pixel's neighbours' altitudes, so
  // the altitudes are materialized once and the differences read out of the field. Same total cost
  // as BuildVisibleMask, which is what makes it reasonable to build this unconditionally (see the
  // member's comment in render.hpp for why it cannot be built only when the flag is on).
  std::vector<float> alt_deg(n, 0.0f);
  std::vector<uint8_t> imaged(n, 0);
  std::vector<uint8_t> drawable(n, 0);
  for (int py = 0; py < height; py++) {
    for (int px = 0; px < width; px++) {
      const mask_detail::MaskDir dir = mask_detail::PixelToWorld(cfg, p, rot, px, py);
      if (!dir.valid) {
        continue;
      }
      const size_t i = static_cast<size_t>(py) * static_cast<size_t>(width) + static_cast<size_t>(px);
      alt_deg[i] = mask_detail::AltitudeDeg(dir);
      imaged[i] = 1;
      drawable[i] = mask_detail::VisibleByRange(cfg.visible_, dir.z) ? 1 : 0;
    }
  }
  return mask_detail::HorizonLineFromAltitudeField(alt_deg, imaged, drawable, width, height);
}

}  // namespace lumice

#endif  // CORE_LENS_PROJ_BUILD_H_
