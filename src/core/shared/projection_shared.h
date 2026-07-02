// Single-source pure-math projection forward functions.
// Shared by host C++ (src/core/projection.cpp), MSL kernel
// (src/core/metal/lumice_trace.metal), and (future) CUDA kernel.
//
// Surface contract: scalar value semantics only. Caller owns
// pixel-layout and visibility-rejection concerns.
//
// `lm_proj::ProjXY` is a transition-stage struct that intentionally mirrors
// `lumice::projection::ProjXY` (same fields, same order, trivially copyable).
// CPU wrappers copy the result field-by-field at the boundary. A future
// consolidation step (post CUDA backend MVP) may collapse the two into one.
#ifndef LM_PROJ_SHARED_H_
#define LM_PROJ_SHARED_H_

#include "lm_shims.h"

namespace lm_proj {

struct ProjXY {
  float x;
  float y;
  bool valid;
};

// Equal-area fisheye forward: k = r_scale / sqrt(1 + dz).
// Mirrors projection.cpp:FisheyeEqualAreaForward and lumice_trace.metal :736.
LM_FN ProjXY FisheyeEqualAreaForward(float dx, float dy, float dz, float r_scale) {
  float k = r_scale / LM_SQRT(1.0f + LM_CLAMP(dz, -1.0f + 1e-6f, 1.0f));
  return { k * dx, k * dy, true };
}

// Equidistant fisheye forward: scale = r_scale * theta / (pi/2 * rho).
LM_FN ProjXY FisheyeEquidistantForward(float dx, float dy, float dz, float r_scale) {
  float rho = LM_SQRT(dx * dx + dy * dy);
  if (rho < 1e-10f) {
    return { 0.0f, 0.0f, true };
  }
  float theta = LM_ACOS(LM_CLAMP(dz, -1.0f, 1.0f));
  float scale = r_scale * theta / (LM_PI_2F * rho);
  return { scale * dx, scale * dy, true };
}

// Stereographic fisheye forward: scale = r_scale * tan(theta/2) / rho.
LM_FN ProjXY FisheyeStereographicForward(float dx, float dy, float dz, float r_scale) {
  float rho = LM_SQRT(dx * dx + dy * dy);
  if (rho < 1e-10f) {
    return { 0.0f, 0.0f, true };
  }
  float theta = LM_ACOS(LM_CLAMP(dz, -1.0f, 1.0f));
  float scale = r_scale * LM_TAN(theta / 2.0f) / rho;
  return { scale * dx, scale * dy, true };
}

// Orthographic fisheye forward: r = sin(theta). Rejects dz < 0 (aliasing).
LM_FN ProjXY FisheyeOrthographicForward(float dx, float dy, float dz, float r_scale) {
  if (dz < 0.0f) {
    return { 0.0f, 0.0f, false };
  }
  return { r_scale * dx, r_scale * dy, true };
}

// Equirectangular forward: x = atan2(dy, dx), y = asin(dz).
LM_FN ProjXY RectangularForward(float dx, float dy, float dz) {
  float lon = LM_ATAN2(dy, dx);
  float lat = LM_ASIN(LM_CLAMP(dz, -1.0f, 1.0f));
  return { lon, lat, true };
}

// Linear (perspective) forward: dz<=0 rejected (behind camera); else (dx/dz, dy/dz).
// Mirrors projection.cpp:LinearForward.
LM_FN ProjXY LinearForward(float dx, float dy, float dz) {
  if (dz <= 0.0f) {
    return { 0.0f, 0.0f, false };
  }
  return { dx / dz, dy / dz, true };
}

// ============================================================================
// Unified forward projection: world/sky dir → 0..2 pixel hits.
// Single source consumed by legacy CPU (lens_proj.hpp, scatter_accum.hpp,
// server/render.cpp) and (315.3) Metal/CUDA kernels.
// ============================================================================

// POD projection parameters — host predigests all trig-heavy setup so the
// per-ray function stays branch/mul only. `proj_type` uses LensParam::LensType
// integer values (0..10; 10 = globe, reserved for 315.4).
// `visible_range` uses RenderConfig::VisibleRange (0=upper, 1=lower, 2=full).
// `rot[9]` is a row-major camera rotation matrix; read only by single-lens
// types (kLinear + 4 single-fisheye); other types treat it as unused (host
// should still fill identity for POD determinism).
struct ProjParams {
  int proj_type;
  int img_w;
  int img_h;
  int visible_range;
  int lens_shift_x;
  int lens_shift_y;
  float scale;
  float az0;
  float r_scale;
  float max_abs_dz;
  float rot[9];
};

struct PixelHit {
  int px;
  int py;
  bool bump_landed;
};

struct ProjResult {
  PixelHit hits[2];
  int count;
};

// Match RenderConfig::VisibleRange: 0=kUpper, 1=kLower, 2=kFull.
LM_CONSTANT int kVisibleUpper = 0;
LM_CONSTANT int kVisibleLower = 1;

// Match LensParam::LensType integer values.
LM_CONSTANT int kProjLinear = 0;
LM_CONSTANT int kProjFisheyeEqualArea = 1;
LM_CONSTANT int kProjFisheyeEquidistant = 2;
LM_CONSTANT int kProjFisheyeStereographic = 3;
LM_CONSTANT int kProjDualFisheyeEqualArea = 4;
LM_CONSTANT int kProjDualFisheyeEquidistant = 5;
LM_CONSTANT int kProjDualFisheyeStereographic = 6;
LM_CONSTANT int kProjRectangular = 7;
LM_CONSTANT int kProjFisheyeOrthographic = 8;
LM_CONSTANT int kProjDualFisheyeOrthographic = 9;
LM_CONSTANT int kProjGlobe = 10;

// Globe lens camera distance (eye-space). The camera sits at (0,0,kGlobeCameraD)
// looking toward the unit sphere at the origin (a genuine finite-distance sphere
// perspective, NOT an orthographic alias). This is the single device-side source
// of truth for the globe projection.
//   MUST match src/gui/gui_constants.hpp:173 (GUI kGlobeCameraD, = the shader
//   `globeInverse` kGlobeCameraDist) — the CLI↔GUI globe consistency contract
//   (315.4). The constant cannot cross the C-API boundary, so this comment anchor
//   plus test/golden-analytic/projection round-trip check guard the two copies.
LM_CONSTANT float kGlobeCameraD = 4.0f;

// Apply the transpose of a row-major 3x3 matrix (equivalent to Rotation::ApplyInverse
// in src/core/geo3d.cpp:79). Splits out of the switch so both fisheye and linear
// single-lens branches share one implementation.
LM_FN void ApplyRotTranspose(LM_THREAD const float* rot, float in0, float in1, float in2, LM_THREAD float* o0,
                             LM_THREAD float* o1, LM_THREAD float* o2) {
  *o0 = rot[0] * in0 + rot[3] * in1 + rot[6] * in2;
  *o1 = rot[1] * in0 + rot[4] * in1 + rot[7] * in2;
  *o2 = rot[2] * in0 + rot[5] * in1 + rot[8] * in2;
}

// Inline of projection.cpp:DualFisheyeToPixel — computes pixel (fx,fy) for a
// hemisphere-normalized (x_norm,y_norm) plus which circle (upper=left / lower=right).
LM_FN void DualFisheyeToPixelXY(float x_norm, float y_norm, bool is_upper, int width, int height, LM_THREAD float* fx,
                                LM_THREAD float* fy) {
  int short_res = LM_MIN(width / 2, height);
  float r = static_cast<float>(short_res) / 2.0f;
  float cy = static_cast<float>(height) / 2.0f;
  if (is_upper) {
    float cx = static_cast<float>(width) / 2.0f - r;
    *fx = -y_norm * r + cx;
    *fy = x_norm * r + cy;
  } else {
    float cx = static_cast<float>(width) / 2.0f + r;
    *fx = y_norm * r + cx;
    *fy = x_norm * r + cy;
  }
}

// Forward-project a world-space direction to 0/1/2 pixel hits.
//   hits[0]           — main projection (bump_landed=true) if produced.
//   hits[1]           — dual-fisheye overlap dual-write (bump_landed=false).
//   count             — 0=miss/cull, 1=main only, 2=main + overlap.
// Bounds culling (px/py against 0..img_w/h-1) is intentionally NOT done here;
// callers do the range check on the returned integer pixels. This function's
// own miss contract is `count=0` (hits[0] left unwritten); the legacy CPU
// convention of translating a miss to pixel {-1,-1} is applied one layer up,
// by lens_proj.hpp's callers, not by ProjectExitToPixel itself.
// NOLINTNEXTLINE(readability-function-cognitive-complexity)
LM_FN ProjResult ProjectExitToPixel(LM_THREAD const ProjParams& p, float wx, float wy, float wz) {
  ProjResult r;
  r.count = 0;

  int t = p.proj_type;
  if (t == kProjLinear || t == kProjFisheyeEqualArea || t == kProjFisheyeEquidistant ||
      t == kProjFisheyeStereographic || t == kProjFisheyeOrthographic) {
    // Single-lens family — camera-frame cull + rot-inverse + forward.
    if ((p.visible_range == kVisibleUpper && wz > 0.0f) || (p.visible_range == kVisibleLower && wz < 0.0f)) {
      return r;
    }
    float cx = 0.0f;
    float cy = 0.0f;
    float cz = 0.0f;
    ApplyRotTranspose(p.rot, -wx, -wy, -wz, &cx, &cy, &cz);

    ProjXY xy = { 0.0f, 0.0f, false };
    if (t == kProjLinear) {
      xy = LinearForward(cx, cy, cz);
    } else {
      // Fisheye 4 types: additional cz<=0 rejection (past-horizon).
      if (cz <= 0.0f) {
        return r;
      }
      if (t == kProjFisheyeEqualArea) {
        xy = FisheyeEqualAreaForward(cx, cy, cz, 1.0f);
      } else if (t == kProjFisheyeEquidistant) {
        xy = FisheyeEquidistantForward(cx, cy, cz, 1.0f);
      } else if (t == kProjFisheyeStereographic) {
        xy = FisheyeStereographicForward(cx, cy, cz, 1.0f);
      } else {  // kProjFisheyeOrthographic
        xy = FisheyeOrthographicForward(cx, cy, cz, 1.0f);
      }
    }
    if (!xy.valid) {
      return r;
    }
    int px = static_cast<int>(
        LM_FLOOR(xy.x * p.scale + static_cast<float>(p.img_w) / 2.0f + 0.5f + static_cast<float>(p.lens_shift_x)));
    int py = static_cast<int>(
        LM_FLOOR(xy.y * p.scale + static_cast<float>(p.img_h) / 2.0f + 0.5f + static_cast<float>(p.lens_shift_y)));
    r.hits[0].px = px;
    r.hits[0].py = py;
    r.hits[0].bump_landed = true;
    r.count = 1;
    return r;
  }

  if (t == kProjRectangular) {
    // Rectangular: no per-ray rotation; camera azimuth pre-computed into az0.
    ProjXY proj = RectangularForward(-wx, -wy, -wz);
    float lon = proj.x - p.az0;
    // Use CPU's canonical while-loop wrap for bit-exact parity with legacy
    // (Metal side uses floor-based single-expression wrap; that path is 315.3's
    // parity concern, not this task's — see plan.md risk 1).
    while (lon < -LM_PI_F) {
      lon += 2.0f * LM_PI_F;
    }
    while (lon > LM_PI_F) {
      lon -= 2.0f * LM_PI_F;
    }
    int raw_x = static_cast<int>(LM_FLOOR(lon * p.scale + static_cast<float>(p.img_w) / 2.0f + 0.5f));
    int px = ((raw_x % p.img_w) + p.img_w) % p.img_w;
    int py = static_cast<int>(LM_FLOOR(-proj.y * p.scale + static_cast<float>(p.img_h) / 2.0f + 0.5f));
    r.hits[0].px = px;
    r.hits[0].py = py;
    r.hits[0].bump_landed = true;
    r.count = 1;
    return r;
  }

  if (t == kProjDualFisheyeEqualArea || t == kProjDualFisheyeEquidistant || t == kProjDualFisheyeStereographic ||
      t == kProjDualFisheyeOrthographic) {
    float sx = -wx;
    float sy = -wy;
    float sz = -wz;
    bool is_upper = (sz >= 0.0f);
    float z_hemi = is_upper ? sz : -sz;
    ProjXY xy = { 0.0f, 0.0f, false };
    if (t == kProjDualFisheyeEqualArea) {
      xy = FisheyeEqualAreaForward(sx, sy, z_hemi, p.r_scale);
    } else if (t == kProjDualFisheyeEquidistant) {
      xy = FisheyeEquidistantForward(sx, sy, z_hemi, p.r_scale);
    } else if (t == kProjDualFisheyeStereographic) {
      xy = FisheyeStereographicForward(sx, sy, z_hemi, p.r_scale);
    } else {  // kProjDualFisheyeOrthographic
      xy = FisheyeOrthographicForward(sx, sy, z_hemi, p.r_scale);
    }
    // Primary write — even if xy.valid==false the legacy dual-fisheye path
    // still called DualFisheyeToPixel and wrote a pixel (only ortho sets
    // valid=false, and legacy code stored the resulting pixel unchecked).
    // Preserve that behaviour: forward hits[0] regardless of xy.valid.
    float fx = 0.0f;
    float fy = 0.0f;
    DualFisheyeToPixelXY(xy.x, xy.y, is_upper, p.img_w, p.img_h, &fx, &fy);
    r.hits[0].px = static_cast<int>(LM_FLOOR(fx + 0.5f));
    r.hits[0].py = static_cast<int>(LM_FLOOR(fy + 0.5f));
    r.hits[0].bump_landed = true;
    r.count = 1;

    // Overlap dual-write: only in the overlap band |sz| < max_abs_dz.
    if (p.max_abs_dz > 0.0f && LM_FABS(sz) < p.max_abs_dz) {
      float z_opp = -z_hemi;
      ProjXY xy2 = { 0.0f, 0.0f, false };
      if (t == kProjDualFisheyeEqualArea) {
        xy2 = FisheyeEqualAreaForward(sx, sy, z_opp, p.r_scale);
      } else if (t == kProjDualFisheyeEquidistant) {
        xy2 = FisheyeEquidistantForward(sx, sy, z_opp, p.r_scale);
      } else if (t == kProjDualFisheyeStereographic) {
        xy2 = FisheyeStereographicForward(sx, sy, z_opp, p.r_scale);
      } else {  // kProjDualFisheyeOrthographic
        xy2 = FisheyeOrthographicForward(sx, sy, z_opp, p.r_scale);
      }
      float fx2 = 0.0f;
      float fy2 = 0.0f;
      DualFisheyeToPixelXY(xy2.x, xy2.y, !is_upper, p.img_w, p.img_h, &fx2, &fy2);
      r.hits[1].px = static_cast<int>(LM_FLOOR(fx2 + 0.5f));
      r.hits[1].py = static_cast<int>(LM_FLOOR(fy2 + 0.5f));
      r.hits[1].bump_landed = false;
      r.count = 2;
    }
    return r;
  }

  if (t == kProjGlobe) {
    // Globe: finite-distance sphere perspective. Camera at (0,0,kGlobeCameraD)
    // in eye space looks at the unit sphere; the near (visible) hemisphere
    // surface point is the sky direction. Mirrors the GUI forward ProjectGlobe
    // (preview_renderer.cpp:919) / overlay_labels.cpp globe branch, which are
    // the numerical inverse of the shader `globeInverse`.
    //
    // The GUI expresses globe in its eye_dir e = WorldToView(w). Here we reuse
    // the SAME eye vector the single-lens family uses: c = R^T * (-w)
    // (ApplyRotTranspose). Single-lens cull parity (shared cull c.z<=0 ==
    // GUI-linear cull e.z>=0) forces the value correspondence c.z = -e.z with
    // the (x,y) numerator shared verbatim between linear and globe on BOTH
    // sides. Substituting into the GUI globe form:
    //   GUI visibility  e.z >  1/D   → cull when  c.z >= -1/D
    //   GUI denom       D - e.z      →            D + c.z   (∈ [3.0, 3.75] for
    //                                             D=4, never approaches 0)
    //   GUI numerator   (e.x, e.y)   →            (c.x, c.y)   [same as linear]
    // Because linear matches the GUI and globe differs from linear by exactly
    // the same delta on both sides, globe matches the GUI by transitivity
    // (including the x/y/row pixel convention — no extra flip is introduced).
    // `p.scale` = focal = img_radius/tan(fov/2), host-computed in ComputeScaleAz0
    // (identical to the linear scale formula, matching GUI focal).
    float cx = 0.0f;
    float cy = 0.0f;
    float cz = 0.0f;
    ApplyRotTranspose(p.rot, -wx, -wy, -wz, &cx, &cy, &cz);
    if (cz >= -1.0f / kGlobeCameraD) {
      return r;  // outside the camera-facing hemisphere → cull
    }
    float denom = kGlobeCameraD + cz;
    int px = static_cast<int>(LM_FLOOR(cx / denom * p.scale + static_cast<float>(p.img_w) / 2.0f + 0.5f +
                                       static_cast<float>(p.lens_shift_x)));
    int py = static_cast<int>(LM_FLOOR(cy / denom * p.scale + static_cast<float>(p.img_h) / 2.0f + 0.5f +
                                       static_cast<float>(p.lens_shift_y)));
    r.hits[0].px = px;
    r.hits[0].py = py;
    r.hits[0].bump_landed = true;
    r.count = 1;
    return r;
  }

  // Anything unknown → miss.
  return r;
}

}  // namespace lm_proj

#endif  // LM_PROJ_SHARED_H_
