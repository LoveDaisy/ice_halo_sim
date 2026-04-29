#include "gui/overlay_labels.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "gui/gui_constants.hpp"
#include "gui/preview_renderer.hpp"

namespace lumice::gui {
namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDeg2Rad = kPi / 180.0f;
constexpr float kRad2Deg = 180.0f / kPi;
constexpr int kSampleStep = 4;  // pixels between edge samples
constexpr float kLabelPadding = 2.0f;

// --- Inverse projection functions ported from shader (preview_renderer.cpp) ---
// Returns direction in view/world space. valid=false if outside projection domain.
// Convention: camera looks toward -z (shader convention).

struct InvResult {
  float x, y, z;
  bool valid;
};

// Synced with shader linearInverse (preview_renderer.cpp line ~158-163)
InvResult LinearInv(float px, float py, float res_x, float res_y, float half_fov) {
  float short_edge = std::min(res_x, res_y);
  float focal = short_edge * 0.5f / std::tan(half_fov);
  float len = std::sqrt(px * px + py * py + focal * focal);
  return { px / len, py / len, -focal / len, true };
}

// Synced with shader fisheyeInverse (preview_renderer.cpp:167-189). type:
// 0=equal_area, 1=equidistant, 2=stereographic, 3=orthographic.
InvResult FisheyeInv(float px, float py, float res_x, float res_y, float half_fov, int type) {
  float img_radius = std::min(res_x, res_y) * 0.5f;
  float r = std::sqrt(px * px + py * py) / img_radius;

  float theta;
  if (type == 0) {  // equal area: r_norm = sin(θ/2) / sin(fov/4)
    float s = r * std::sin(half_fov * 0.5f);
    if (s > 1.0f)
      return { 0, 0, 0, false };
    theta = 2.0f * std::asin(s);
  } else if (type == 1) {  // equidistant: r_norm = θ / half_fov
    theta = r * half_fov;
    if (theta >= kPi)
      return { 0, 0, 0, false };
  } else if (type == 2) {  // stereographic: r_norm = tan(θ/2) / tan(fov/4)
    theta = 2.0f * std::atan(r * std::tan(half_fov * 0.5f));
  } else {  // orthographic (type == 3): r_norm = sin(θ) / sin(fov/2)
    float s = r * std::sin(half_fov);
    if (s > 1.0f)
      return { 0, 0, 0, false };
    theta = std::asin(s);
  }

  float phi = std::atan2(py, px);
  float st = std::sin(theta);
  return { st * std::cos(phi), st * std::sin(phi), -std::cos(theta), true };
}

// Synced with shader dualFisheyeInverse (preview_renderer.cpp line ~190-236)
InvResult DualFisheyeInv(float px, float py, float res_x, float res_y, int type) {
  float short_res = std::min(res_x * 0.5f, res_y);
  float circle_radius = short_res * 0.5f;

  float lx = px + circle_radius, ly = py;
  float rx = px - circle_radius, ry = py;
  float left_r = std::sqrt(lx * lx + ly * ly) / circle_radius;
  float right_r = std::sqrt(rx * rx + ry * ry) / circle_radius;

  bool in_left = left_r <= 1.0f;
  bool in_right = right_r <= 1.0f;
  if (!in_left && !in_right)
    return { 0, 0, 0, false };

  float use_x = in_left ? lx : rx;
  float use_y = in_left ? ly : ry;
  float use_r = in_left ? left_r : right_r;

  float half_pi = kPi * 0.5f;
  float theta;
  if (type == 0) {  // equal area
    float s = use_r * std::sin(half_pi * 0.5f);
    if (s > 1.0f)
      return { 0, 0, 0, false };
    theta = 2.0f * std::asin(s);
  } else if (type == 1) {  // equidistant
    theta = use_r * half_pi;
  } else if (type == 2) {  // stereographic
    theta = 2.0f * std::atan(use_r * std::tan(half_pi * 0.5f));
  } else {  // orthographic (type == 3): use_r is already normalised; θ = asin(use_r)
    if (use_r > 1.0f)
      return { 0, 0, 0, false };
    theta = std::asin(use_r);
  }

  float phi = std::atan2(use_y, use_x);
  float st = std::sin(theta);
  float ct = std::cos(theta);
  if (in_left) {
    return { -st * std::sin(phi), st * std::cos(phi), -ct, true };
  } else {
    return { -st * std::sin(phi), -st * std::cos(phi), ct, true };
  }
}

// Synced with shader rectangularInverse (preview_renderer.cpp line ~240-249)
InvResult RectangularInv(float px, float py, float res_x, float res_y) {
  float short_res = std::min(res_x * 0.5f, res_y);
  float scale = short_res / kPi;
  float lon = px / scale;
  float lat = -py / scale;
  if (std::abs(lat) > kPi * 0.5f)
    return { 0, 0, 0, false };

  float cl = std::cos(lat);
  return { -cl * std::cos(lon), -cl * std::sin(lon), -std::sin(lat), true };
}

// The two range-based dispatches below depend on consecutive enum values:
// (lens_type - kLensTypeFisheyeEqualArea) must equal FisheyeInv's type
// argument 0/1/2 across the [EqualArea, Equidist, Stereographic] band, and
// likewise for the dual fisheye band. Pin the contract at compile time so
// inserting a new fisheye enum mid-band is a build break, not a silent
// label-position regression.
static_assert(kLensTypeFisheyeEquidist - kLensTypeFisheyeEqualArea == 1,
              "Fisheye enum band must remain consecutive (EqualArea→Equidist→Stereographic).");
static_assert(kLensTypeFisheyeStereographic - kLensTypeFisheyeEqualArea == 2,
              "Fisheye enum band must remain consecutive (EqualArea→Equidist→Stereographic).");
static_assert(kLensTypeDualFisheyeEquidist - kLensTypeDualFisheyeEqualArea == 1,
              "Dual fisheye enum band must remain consecutive.");
static_assert(kLensTypeDualFisheyeStereographic - kLensTypeDualFisheyeEqualArea == 2,
              "Dual fisheye enum band must remain consecutive.");

// Compute world direction for a pixel offset from viewport center.
// Synced with shader main() (preview_renderer.cpp:317-332). Dispatch table:
//   lens 0           → LinearInv               (view-transformed)
//   lens 1-3         → FisheyeInv (type 0-2)   (view-transformed)
//   lens 4-6         → DualFisheyeInv          (world-space, no view)
//   lens 7           → RectangularInv          (world-space, no view)
//   lens 8 (single ortho) → FisheyeInv(type=3) (view-transformed)
//   lens 9 (dual ortho)   → DualFisheyeInv(3)  (world-space, no view)
InvResult PixelToWorldDir(float px, float py, float res_x, float res_y, int lens_type, float fov,
                          const float view_matrix[9]) {
  float half_fov = fov * 0.5f * kDeg2Rad;

  InvResult r;
  bool needs_view = true;
  if (lens_type == kLensTypeLinear) {
    r = LinearInv(px, py, res_x, res_y, half_fov);
  } else if (lens_type >= kLensTypeFisheyeEqualArea && lens_type <= kLensTypeFisheyeStereographic) {
    r = FisheyeInv(px, py, res_x, res_y, half_fov, lens_type - kLensTypeFisheyeEqualArea);
  } else if (lens_type >= kLensTypeDualFisheyeEqualArea && lens_type <= kLensTypeDualFisheyeStereographic) {
    r = DualFisheyeInv(px, py, res_x, res_y, lens_type - kLensTypeDualFisheyeEqualArea);
    needs_view = false;
  } else if (lens_type == kLensTypeRectangular) {
    r = RectangularInv(px, py, res_x, res_y);
    needs_view = false;
  } else if (lens_type == kLensTypeFisheyeOrthographic) {
    // Single orthographic: shader uses fisheyeInverse(type=3) and applies view matrix.
    r = FisheyeInv(px, py, res_x, res_y, half_fov, 3);
  } else if (lens_type == kLensTypeDualFisheyeOrthographic) {
    // Dual orthographic: shader uses dualFisheyeInverse(type=3), no view matrix.
    r = DualFisheyeInv(px, py, res_x, res_y, 3);
    needs_view = false;
  } else if (lens_type == kLensTypeGlobe) {
    // Mirror shader globeInverse (preview_renderer.cpp:270-288). Camera sits at
    // eye-space (0, 0, D); intersect ray with the unit sphere at the origin and
    // return hit_eye as a unit-length eye-space direction. The needs_view branch
    // below applies view_matrix (eye → world), matching shader's
    // result = u_view_matrix * hit_eye — these two paths are mathematically
    // equivalent.
    float short_edge = std::min(res_x, res_y);
    float focal = short_edge * 0.5f / std::tan(half_fov);
    float dx = px;
    float dy = py;
    float dz = -focal;
    float dlen = std::sqrt(dx * dx + dy * dy + dz * dz);
    dx /= dlen;
    dy /= dlen;
    dz /= dlen;
    float b = kGlobeCameraD * dz;  // dot(O, d), O = (0, 0, D)
    float c = kGlobeCameraD * kGlobeCameraD - 1.0f;
    float disc = b * b - c;
    if (disc < 0.0f) {
      r = { 0, 0, 0, false };  // ray misses sphere
    } else {
      float t = -b - std::sqrt(disc);
      if (t <= 0.0f) {
        r = { 0, 0, 0, false };  // hit behind camera
      } else {
        float hx = t * dx;
        float hy = t * dy;
        float hz = kGlobeCameraD + t * dz;
        // Mathematically |hit_eye| == 1 on the unit sphere; normalize to absorb
        // float error near grazing incidence (disc ≈ 0), matching FisheyeInv's
        // explicit normalization style.
        float hn = std::sqrt(hx * hx + hy * hy + hz * hz);
        if (hn > 0.0f) {
          hx /= hn;
          hy /= hn;
          hz /= hn;
        }
        r = { hx, hy, hz, true };
      }
    }
    // needs_view stays true: hit_eye is in eye space; the public block multiplies
    // by view_matrix to move it to world space.
  } else {
    r = { 0, 0, 0, false };
  }

  if (!r.valid)
    return r;

  if (needs_view) {
    // view_matrix is column-major 3x3: out[col*3 + row]
    float wx = view_matrix[0] * r.x + view_matrix[3] * r.y + view_matrix[6] * r.z;
    float wy = view_matrix[1] * r.x + view_matrix[4] * r.y + view_matrix[7] * r.z;
    float wz = view_matrix[2] * r.x + view_matrix[5] * r.y + view_matrix[8] * r.z;
    r.x = wx;
    r.y = wy;
    r.z = wz;
  }

  return r;
}

// Forward projection: world direction → pixel offset from viewport center.
// Inverse of PixelToWorldDir. For types 0-3, applies inverse view matrix first.
// Returns valid=false if direction is behind camera or outside projection domain.
struct FwdResult {
  float px, py;
  bool valid;
};

FwdResult WorldDirToPixel(float wx, float wy, float wz, float res_x, float res_y, int lens_type, float fov,
                          const float view_matrix[9]) {
  // View transform applies to all view-transformed lens types — i.e. NOT full-sky.
  // This mirrors the inverse projection's classification: linear (0), single fisheye
  // (1-3), and single orthographic (8) all carry view matrix; dual fisheye (4-6),
  // rectangular (7), and dual orthographic (9) skip it. Reusing LensIsFullSky as the
  // single source of truth keeps WorldDirToPixel coupled to the same lens-class
  // policy as RenderRightPanel / RenderPreviewPanel.
  float dx = wx, dy = wy, dz = wz;
  bool needs_view = !LensIsFullSky(lens_type);
  if (needs_view) {
    // view_matrix is column-major: M[col*3+row]. Transpose = rows become columns.
    dx = view_matrix[0] * wx + view_matrix[1] * wy + view_matrix[2] * wz;
    dy = view_matrix[3] * wx + view_matrix[4] * wy + view_matrix[5] * wz;
    dz = view_matrix[6] * wx + view_matrix[7] * wy + view_matrix[8] * wz;
  }

  float half_fov = fov * 0.5f * kDeg2Rad;
  float short_edge = std::min(res_x, res_y);

  if (lens_type == kLensTypeLinear) {
    if (dz >= 0)
      return { 0, 0, false };  // behind camera (shader convention: camera looks -z)
    float focal = short_edge * 0.5f / std::tan(half_fov);
    return { dx / (-dz) * focal, dy / (-dz) * focal, true };
  } else if ((lens_type >= kLensTypeFisheyeEqualArea && lens_type <= kLensTypeFisheyeStereographic) ||
             lens_type == kLensTypeFisheyeOrthographic) {  // Single fisheye family (incl. orthographic)
    float img_radius = short_edge * 0.5f;
    float theta = std::acos(std::clamp(-dz, -1.0f, 1.0f));  // angle from -z axis
    float rho = std::sqrt(dx * dx + dy * dy);
    if (rho < 1e-8f)
      return { 0, 0, true };  // on optical axis

    float r_norm;
    if (lens_type == kLensTypeFisheyeEqualArea) {
      r_norm = std::sin(theta * 0.5f) / std::sin(half_fov * 0.5f);
    } else if (lens_type == kLensTypeFisheyeEquidist) {
      r_norm = theta / half_fov;
    } else if (lens_type == kLensTypeFisheyeStereographic) {
      r_norm = std::tan(theta * 0.5f) / std::tan(half_fov * 0.5f);
    } else {  // kLensTypeFisheyeOrthographic — shader: r_norm = sin(θ) / sin(fov/2)
      // Domain guard: directions past the orthographic disc edge (θ > half_fov)
      // project to r_norm > 1 in the inverse path; reject them here so the
      // sun-circle interior-label placement doesn't draw off-disc.
      if (theta > half_fov)
        return { 0, 0, false };
      r_norm = std::sin(theta) / std::sin(half_fov);
    }
    float scale = r_norm * img_radius / rho;
    return { dx * scale, dy * scale, true };
  } else if (lens_type == kLensTypeGlobe) {
    // (dx,dy,dz) is the unit-length world dir transformed to eye-space. The world
    // point on the sphere surface in this direction is P_eye = (dx,dy,dz). With
    // the camera at O = (0,0,D), the front hemisphere visible to the camera is
    // P_eye.z > 1/D. See plan §3 design point 4 for the derivation. After this
    // visibility filter, denom = D - dz lies in [D - 1, D - 1/D] = [3.0, 3.75]
    // for D=4.0, so it never approaches zero.
    if (dz <= 1.0f / kGlobeCameraD) {
      return { 0, 0, false };
    }
    float focal = short_edge * 0.5f / std::tan(half_fov);
    float denom = kGlobeCameraD - dz;
    return { dx / denom * focal, dy / denom * focal, true };
  }
  // Full-sky lens types (dual fisheye 4-6, rectangular 7, dual orthographic 9):
  // their shader paths skip the view matrix and the entire sphere is always
  // visible, so sun circles always intersect viewport edges and don't need
  // interior labels. Returning invalid lets the caller fall back to edge labels.
  return { 0, 0, false };
}

// Angle values computed for each edge sample point.
struct SampleAngles {
  float altitude;                         // degrees
  float azimuth;                          // degrees, [-180, 180]
  float sun_dist;                         // degrees, [0, 180]
  float wx = 0.0f, wy = 0.0f, wz = 0.0f;  // world direction (unit vector when valid; (0,0,0) when invalid)
  float screen_x, screen_y;               // ImGui screen coords
  bool valid;
};

// Check if value crosses a target between two samples. Returns interpolated crossing position.
// Uses a small epsilon tolerance to handle floating-point precision loss at fisheye disc edges (r≈1).
// kEps=0.01 is small enough to avoid false-positive clusters at near-tangent points (where the
// original kEps=0.1 caused spurious label groups), yet large enough to catch genuine crossings
// that precision loss at projection boundaries can flip to same-side.
bool Crosses(float v0, float v1, float target, float* t_out) {
  float d0 = v0 - target;
  float d1 = v1 - target;
  constexpr float kEps = 0.01f;
  if (d0 * d1 < kEps) {
    float denom = v1 - v0;
    if (std::abs(denom) < 1e-6f)
      return false;
    *t_out = std::clamp(-d0 / denom, 0.0f, 1.0f);
    return true;
  }
  return false;
}

ImU32 ColorToImU32(const float c[3], int alpha) {
  return IM_COL32(static_cast<int>(c[0] * 255), static_cast<int>(c[1] * 255), static_cast<int>(c[2] * 255), alpha);
}

constexpr int kGroupGrid = 0;
constexpr int kGroupSunCircles = 1;

void AddLabel(std::vector<OverlayLabel>& out, float sx0, float sy0, float sx1, float sy1, float t, const char* fmt,
              float value, ImU32 color, int group = kGroupGrid) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), fmt, value);
  out.push_back({ sx0 + t * (sx1 - sx0), sy0 + t * (sy1 - sy0), std::string(buf), color, false, group });
}

}  // namespace

void ComputeOverlayLabels(const OverlayLabelInput& input, float vp_screen_x, float vp_screen_y, float vp_screen_w,
                          float vp_screen_h, std::vector<OverlayLabel>& out) {
  out.clear();
  if (!input.show_horizon && !input.show_grid && !input.show_sun_circles)
    return;

  // Build view matrix for types 0-3
  float view_matrix[9];
  BuildViewMatrix(input.elevation, input.azimuth, input.roll, view_matrix);

  // Camera forward in world space. BuildViewMatrix stores -forward in column 2
  // (preview_renderer.cpp:600,623), column-major out[col*3+row], so forward = -col2.
  // Used by Front-mode (visible==3) hemisphere check, mirroring shader's
  // `dot(world_dir, u_view_matrix[2]) > 0.0` discard test (preview_renderer.cpp:336).
  const float forward[3] = { -view_matrix[6], -view_matrix[7], -view_matrix[8] };

  // Viewport pixel dimensions (used as u_resolution in shader)
  float res_x = vp_screen_w;
  float res_y = vp_screen_h;

  int horizon_a = static_cast<int>(input.horizon_alpha * 255);
  int grid_a = static_cast<int>(input.grid_alpha * 255);
  int sun_a = static_cast<int>(input.sun_circles_alpha * 255);
  ImU32 horizon_col = ColorToImU32(input.horizon_color, horizon_a);
  ImU32 grid_col = ColorToImU32(input.grid_color, grid_a);
  ImU32 sun_col = ColorToImU32(input.sun_circles_color, sun_a);

  // Compute sun direction dot product for angular distance
  auto dot3 = [](float a0, float a1, float a2, float b0, float b1, float b2) { return a0 * b0 + a1 * b1 + a2 * b2; };

  // Sample along viewport edges and detect crossings.
  // Edge definition: 4 edges, each parameterized by pixel offset from viewport center.
  struct Edge {
    float start_px, start_py;  // pixel offset from center at t=0
    float end_px, end_py;      // pixel offset from center at t=1
    float start_sx, start_sy;  // screen coords at t=0
    float end_sx, end_sy;      // screen coords at t=1
  };

  // NDC y=+1 (pos.y=+hh) is OpenGL top = ImGui screen top (vp_screen_y).
  // NDC y=-1 (pos.y=-hh) is OpenGL bottom = ImGui screen bottom (vp_screen_y + vp_screen_h).
  float hw = res_x * 0.5f, hh = res_y * 0.5f;
  Edge edges[4] = {
    { -hw, hh, hw, hh, vp_screen_x, vp_screen_y, vp_screen_x + vp_screen_w, vp_screen_y },  // top
    { -hw, -hh, hw, -hh, vp_screen_x, vp_screen_y + vp_screen_h, vp_screen_x + vp_screen_w,
      vp_screen_y + vp_screen_h },                                                            // bottom
    { -hw, hh, -hw, -hh, vp_screen_x, vp_screen_y, vp_screen_x, vp_screen_y + vp_screen_h },  // left
    { hw, hh, hw, -hh, vp_screen_x + vp_screen_w, vp_screen_y, vp_screen_x + vp_screen_w,
      vp_screen_y + vp_screen_h },  // right
  };

  // Front-hemisphere predicate: world_dir lies in front of camera.
  // Shader (preview_renderer.cpp:336) uses strict >0 cull because pixel centers don't go
  // through interpolation. Overlay labels sample along curves, interp_dir blends + renormalizes
  // directions between adjacent samples — this introduces float noise at the boundary where
  // the true dot is exactly 0. Strict >= 0 then silently drops valid boundary crossings.
  // kFrontEps = 0.01 ≈ sin(0.57°), semantically matching Upper/Lower's 0.5° altitude tolerance
  // (is_visible below uses `alt >= -0.5f`). Single source of truth for both is_visible
  // (edge-crossing path) and is_visible_front (sun-circle interior path).
  constexpr float kFrontEps = 0.01f;
  auto front_facing = [&](float wx, float wy, float wz) -> bool {
    return forward[0] * wx + forward[1] * wy + forward[2] * wz >= -kFrontEps;
  };

  // Hemisphere visibility check used by edge-crossing label paths.
  // visible: 0=upper (alt>=0), 1=lower (alt<=0), 2=full, 3=front.
  auto is_visible = [&](float alt, float wx, float wy, float wz) -> bool {
    if (input.visible == 0)
      return alt >= -0.5f;  // small tolerance for edge labels
    if (input.visible == 1)
      return alt <= 0.5f;
    if (input.visible == 3)
      return front_facing(wx, wy, wz);
    return true;  // full(2)
  };

  // Front-only visibility check used by sun-circle interior label placement.
  // Kept separate from is_visible to avoid injecting a fake altitude into the
  // Upper/Lower branches — the interior block historically did no is_visible
  // filtering, so other modes must continue to return true unconditionally.
  auto is_visible_front = [&](float wx, float wy, float wz) -> bool {
    if (input.visible != 3)
      return true;
    return front_facing(wx, wy, wz);
  };

  // Interpolate world direction at crossing parameter t and renormalize.
  // Linear-interp + normalize is accurate enough at kSampleStep=4px adjacent samples.
  auto interp_dir = [](const SampleAngles& a, const SampleAngles& b, float t, float& ix, float& iy, float& iz) {
    ix = a.wx + t * (b.wx - a.wx);
    iy = a.wy + t * (b.wy - a.wy);
    iz = a.wz + t * (b.wz - a.wz);
    float len = std::sqrt(ix * ix + iy * iy + iz * iz);
    if (len > 1e-6f) {
      ix /= len;
      iy /= len;
      iz /= len;
    }
  };

  // Crossing detection helper: given two adjacent valid sample points, detect and add labels.
  auto detect_crossings = [&](const SampleAngles& prev, const SampleAngles& cur) {
    float t;

    // Horizon: altitude == 0. Grid skips this latitude (see "skip 0° altitude" below)
    // so the two paths produce a single non-overlapping "0°" label when both are on.
    if (input.show_horizon) {
      if (std::abs(prev.altitude - cur.altitude) < 20.0f && Crosses(prev.altitude, cur.altitude, 0.0f, &t)) {
        float ix, iy, iz;
        interp_dir(prev, cur, t, ix, iy, iz);
        if (is_visible(0.0f, ix, iy, iz)) {
          AddLabel(out, prev.screen_x, prev.screen_y, cur.screen_x, cur.screen_y, t, "%.0f\xC2\xB0", 0.0f, horizon_col);
        }
      }
    }

    // Grid: altitude crosses multiples of 10
    if (input.show_grid) {
      if (std::abs(prev.altitude - cur.altitude) < 20.0f) {
        for (int g = -9; g <= 9; g++) {
          if (g == 0)
            continue;  // skip 0° altitude (horizon) — low value, avoids equator edge clutter
          float target = g * 10.0f;
          if (Crosses(prev.altitude, cur.altitude, target, &t)) {
            float ix, iy, iz;
            interp_dir(prev, cur, t, ix, iy, iz);
            if (is_visible(target, ix, iy, iz)) {
              AddLabel(out, prev.screen_x, prev.screen_y, cur.screen_x, cur.screen_y, t, "%.0f\xC2\xB0", target,
                       grid_col);
            }
          }
        }
      }

      // Grid: azimuth crosses multiples of 10 (with wrap-around handling)
      float az0 = prev.azimuth;
      float az1 = cur.azimuth;
      float delta_az = az1 - az0;
      if (delta_az > 180.0f)
        az1 -= 360.0f;
      if (delta_az < -180.0f)
        az1 += 360.0f;

      if (std::abs(az1 - az0) < 20.0f) {
        for (int g = -18; g <= 18; g++) {
          float target = g * 10.0f;
          if (Crosses(az0, az1, target, &t)) {
            float alt_at_crossing = prev.altitude + t * (cur.altitude - prev.altitude);
            float ix, iy, iz;
            interp_dir(prev, cur, t, ix, iy, iz);
            if (!is_visible(alt_at_crossing, ix, iy, iz))
              continue;
            float label_val = target;
            if (label_val > 180.0f)
              label_val -= 360.0f;
            if (label_val <= -180.0f)
              label_val += 360.0f;
            AddLabel(out, prev.screen_x, prev.screen_y, cur.screen_x, cur.screen_y, t, "%.0f\xC2\xB0", label_val,
                     grid_col);
          }
        }
      }
    }

    // Sun circles: angular distance crosses specified angles
    if (input.show_sun_circles) {
      for (int ci = 0; ci < input.sun_circle_count; ci++) {
        float target = input.sun_circle_angles[ci];
        if (Crosses(prev.sun_dist, cur.sun_dist, target, &t)) {
          float alt_at_crossing = prev.altitude + t * (cur.altitude - prev.altitude);
          float ix, iy, iz;
          interp_dir(prev, cur, t, ix, iy, iz);
          if (!is_visible(alt_at_crossing, ix, iy, iz))
            continue;
          AddLabel(out, prev.screen_x, prev.screen_y, cur.screen_x, cur.screen_y, t, "%.0f\xC2\xB0", target, sun_col,
                   kGroupSunCircles);
        }
      }
    }
  };

  // Helper: compute SampleAngles from a pixel offset and screen position.
  auto make_sample = [&](float px, float py, float sx, float sy) -> SampleAngles {
    InvResult r = PixelToWorldDir(px, py, res_x, res_y, input.lens_type, input.fov, view_matrix);
    SampleAngles s{};
    s.screen_x = sx;
    s.screen_y = sy;
    s.valid = r.valid;
    if (r.valid) {
      s.altitude = std::asin(std::clamp(-r.z, -1.0f, 1.0f)) * kRad2Deg;
      s.azimuth = std::atan2(-r.y, -r.x) * kRad2Deg;
      s.sun_dist = std::acos(std::clamp(dot3(r.x, r.y, r.z, input.sun_dir[0], input.sun_dir[1], input.sun_dir[2]),
                                        -1.0f, 1.0f)) *
                   kRad2Deg;
      s.wx = r.x;
      s.wy = r.y;
      s.wz = r.z;
    }
    return s;
  };

  // === overlay-label sample-source dispatch ===
  // ComputeOverlayLabels combines up to 5 sample sources to produce labels.
  // Activation rules (kept in one place to make per-(lens, visible) coverage
  // auditable; see also LensIsFullSky in gui_constants.hpp):
  //   1. sample_viewport_edges()        : always (every lens, every visible).
  //   2. sample_hemisphere_equator(±1)  : !full_sky AND visible ∈ {Upper, Lower}.
  //   3. sample_front_great_circle()    : !full_sky AND visible == Front.
  //   4. sample_interior_latitudes()    : !full_sky AND lens != Linear; covers
  //                                        the gap when the projection disc is
  //                                        smaller than the viewport (e.g.
  //                                        single-orthographic + fov=180 +
  //                                        visible=Full) so latitude rings
  //                                        wholly inside the disc still get a
  //                                        text label. Per-altitude is_visible
  //                                        filter handles Upper/Lower/Front
  //                                        modes uniformly.
  //   5. sample_sun_circle_interior()   : show_sun_circles AND !full_sky.
  // LensIsFullSky is the SSOT for "is this a world-space / full-sky lens?"
  // (dual fisheye 4-6, rectangular 7, dual orthographic 9 → true).

  // Push the boundary curve a few degrees inward (toward the visible side)
  // before sampling, so labels emitted at boundary crossings don't straddle
  // the visible/invisible split. 3° is a visual-spacing heuristic — at typical
  // fov+lens it projects to ~4–12 px of screen offset, which clears the
  // text height while staying close enough to read as a "boundary label".
  constexpr int kBoundarySamples = 360;
  constexpr float kBoundaryInsetDeg = 3.0f;
  const float boundary_offset = std::sin(kBoundaryInsetDeg * kDeg2Rad);

  // Helper: walk a parametric world-space curve and feed adjacent forward-
  // projected sample pairs into detect_crossings. Shared by hemisphere-equator
  // and front-great-circle samplers.
  auto sample_curve = [&](auto curve_fn) {
    SampleAngles prev{};
    prev.valid = false;
    for (int i = 0; i <= kBoundarySamples; i++) {
      float t = i * (2.0f * kPi / kBoundarySamples);
      float wx_b = 0, wy_b = 0, wz_b = 0;
      curve_fn(t, wx_b, wy_b, wz_b);

      FwdResult fp = WorldDirToPixel(wx_b, wy_b, wz_b, res_x, res_y, input.lens_type, input.fov, view_matrix);
      if (!fp.valid || std::abs(fp.px) > hw || std::abs(fp.py) > hh) {
        prev.valid = false;
        continue;
      }

      float scr_x = vp_screen_x + (fp.px + hw) / res_x * vp_screen_w;
      float scr_y = vp_screen_y + (hh - fp.py) / res_y * vp_screen_h;

      SampleAngles cur = make_sample(fp.px, fp.py, scr_x, scr_y);
      if (prev.valid && cur.valid)
        detect_crossings(prev, cur);
      prev = cur;
    }
  };

  // Source 1: viewport rectangle edges.
  auto sample_viewport_edges = [&]() {
    for (const auto& edge : edges) {
      float edge_len = std::sqrt((edge.end_px - edge.start_px) * (edge.end_px - edge.start_px) +
                                 (edge.end_py - edge.start_py) * (edge.end_py - edge.start_py));
      int num_samples = std::max(2, static_cast<int>(edge_len / kSampleStep));

      SampleAngles prev{};
      prev.valid = false;

      for (int i = 0; i <= num_samples; i++) {
        float frac = static_cast<float>(i) / num_samples;
        float px = edge.start_px + frac * (edge.end_px - edge.start_px);
        float py = edge.start_py + frac * (edge.end_py - edge.start_py);
        float sx = edge.start_sx + frac * (edge.end_sx - edge.start_sx);
        float sy = edge.start_sy + frac * (edge.end_sy - edge.start_sy);

        SampleAngles cur = make_sample(px, py, sx, sy);
        if (prev.valid && cur.valid)
          detect_crossings(prev, cur);
        prev = cur;
      }
    }
  };

  // Source 2: equator (altitude=0). Active under visible=Upper/Lower; the wz
  // sign chooses which side the inset offset is applied to.
  // visible=upper → push toward negative z (altitude=asin(-z) becomes positive);
  // visible=lower → push toward positive z. After offset the world vector
  // is renormalized so WorldDirToPixel's unit-length assumption holds.
  // The az_rad = -π + t parameterization preserves label positions across
  // the v15/v16 refactors that introduced this sampler.
  auto sample_hemisphere_equator = [&](float wz_sign) {
    sample_curve([wz_sign, boundary_offset](float t, float& wx, float& wy, float& wz) {
      float az = -kPi + t;
      wx = -std::cos(az);
      wy = -std::sin(az);
      wz = wz_sign * boundary_offset;
      float len = std::sqrt(wx * wx + wy * wy + wz * wz);
      wx /= len;
      wy /= len;
      wz /= len;
    });
  };

  // Source 3: front-half great circle (dot(world_dir, forward)=0). Active
  // under visible=Front. In view space this is the xy-plane; map to world via
  // view_matrix (column-major: col0/col1 are view-space x/y basis vectors in
  // world coords). Points: world = cos(t)*col0 + sin(t)*col1, then nudged
  // toward forward (opposite of col2 = -forward) by boundary_offset so the
  // boundary samples lie strictly inside the front hemisphere.
  auto sample_front_great_circle = [&]() {
    sample_curve([&](float t, float& wx, float& wy, float& wz) {
      float c = std::cos(t);
      float s = std::sin(t);
      wx = c * view_matrix[0] + s * view_matrix[3] - boundary_offset * view_matrix[6];
      wy = c * view_matrix[1] + s * view_matrix[4] - boundary_offset * view_matrix[7];
      wz = c * view_matrix[2] + s * view_matrix[5] - boundary_offset * view_matrix[8];
      float len = std::sqrt(wx * wx + wy * wy + wz * wz);
      wx /= len;
      wy /= len;
      wz /= len;
    });
  };

  // Source 4 (NEW): place one latitude label per ring whose forward projection
  // lies inside the projection disc but the disc itself does not intersect any
  // viewport edge / hemisphere boundary curve. Walks a small altitude grid
  // (±10°…±80°) and, per altitude, samples 64 azimuths to find the first valid
  // viewport-in + visible pixel. is_visible() handles Upper/Lower/Front modes
  // uniformly. Output is grid-group, no background, no per-source dedup
  // beyond the existing-text check below — bbox overlap suppression in
  // AppendOverlayToDrawList is the second line of defence at draw time.
  // Linear lens excluded by caller dispatch (its latitude rings are screen-
  // space lines already covered by sample_viewport_edges).
  auto sample_interior_latitudes = [&]() {
    if (!input.show_grid)
      return;
    constexpr int kAzimuthSamples = 64;
    constexpr int kAltitudeSteps[] = { -80, -70, -60, -50, -40, -30, -20, -10, 10, 20, 30, 40, 50, 60, 70, 80 };
    for (int alt_deg : kAltitudeSteps) {
      char buf[32];
      std::snprintf(buf, sizeof(buf), "%.0f\xC2\xB0", static_cast<float>(alt_deg));
      bool already_present = false;
      for (const auto& l : out) {
        if (l.group == kGroupGrid && l.text == buf) {
          already_present = true;
          break;
        }
      }
      if (already_present)
        continue;

      const float alt_rad = static_cast<float>(alt_deg) * kDeg2Rad;
      const float cos_a = std::cos(alt_rad);
      const float sin_a = std::sin(alt_rad);

      for (int k = 0; k < kAzimuthSamples; k++) {
        float az = 2.0f * kPi * static_cast<float>(k) / kAzimuthSamples;
        // Direction matched to make_sample's altitude/azimuth conventions:
        //   altitude = asin(-z) → z = -sin(alt)
        //   azimuth  = atan2(-y, -x) → x = -cos(alt)cos(az), y = -cos(alt)sin(az)
        float wx = -cos_a * std::cos(az);
        float wy = -cos_a * std::sin(az);
        float wz = -sin_a;
        if (!is_visible(static_cast<float>(alt_deg), wx, wy, wz))
          continue;
        FwdResult fp = WorldDirToPixel(wx, wy, wz, res_x, res_y, input.lens_type, input.fov, view_matrix);
        if (!fp.valid)
          continue;
        if (std::abs(fp.px) > hw || std::abs(fp.py) > hh)
          continue;

        float scr_x = vp_screen_x + (fp.px + hw) / res_x * vp_screen_w;
        float scr_y = vp_screen_y + (hh - fp.py) / res_y * vp_screen_h;
        out.push_back({ scr_x, scr_y, std::string(buf), grid_col, false, kGroupGrid });
        break;
      }
    }
  };

  // Source 5: sun-circle interior labels. Active when sun circles are enabled
  // and the lens is view-transformed (full-sky lenses always have viewport
  // edges intersect the sun circle).
  auto sample_sun_circle_interior = [&]() {
    for (int ci = 0; ci < input.sun_circle_count; ci++) {
      float angle_deg = input.sun_circle_angles[ci];
      // Check if this circle already has edge labels
      bool has_edge_label = false;
      for (const auto& label : out) {
        char expected[32];
        std::snprintf(expected, sizeof(expected), "%.0f\xC2\xB0", angle_deg);
        if (label.text == expected && label.color == sun_col) {
          has_edge_label = true;
          break;
        }
      }
      if (has_edge_label)
        continue;

      // Generate 4 directions on the sun circle at 90° intervals around the sun direction
      float sun_dist_rad = angle_deg * kDeg2Rad;
      // Build an orthonormal frame around sun_dir
      float sx = input.sun_dir[0], sy = input.sun_dir[1], sz = input.sun_dir[2];
      // Find a vector not parallel to sun_dir
      float ux, uy, uz;
      if (std::abs(sz) < 0.9f) {
        // Cross with (0,0,1)
        ux = sy;
        uy = -sx;
        uz = 0;
      } else {
        // Cross with (1,0,0)
        ux = 0;
        uy = sz;
        uz = -sy;
      }
      float u_len = std::sqrt(ux * ux + uy * uy + uz * uz);
      ux /= u_len;
      uy /= u_len;
      uz /= u_len;
      // v = sun_dir × u
      float vx = sy * uz - sz * uy;
      float vy = sz * ux - sx * uz;
      float vz = sx * uy - sy * ux;

      float cos_sd = std::cos(sun_dist_rad);
      float sin_sd = std::sin(sun_dist_rad);

      constexpr int kNumInteriorLabels = 4;
      for (int li = 0; li < kNumInteriorLabels; li++) {
        float phi = li * kPi * 0.5f;  // 0, 90, 180, 270 degrees
        float cp = std::cos(phi), sp = std::sin(phi);
        // Point on the sun circle: cos(sd)*sun_dir + sin(sd)*(cos(phi)*u + sin(phi)*v)
        float wx = cos_sd * sx + sin_sd * (cp * ux + sp * vx);
        float wy = cos_sd * sy + sin_sd * (cp * uy + sp * vy);
        float wz = cos_sd * sz + sin_sd * (cp * uz + sp * vz);

        // Front-mode hemisphere cull (no-op for other modes; keeps interior labels unchanged).
        if (!is_visible_front(wx, wy, wz))
          continue;

        FwdResult fp = WorldDirToPixel(wx, wy, wz, res_x, res_y, input.lens_type, input.fov, view_matrix);
        if (!fp.valid)
          continue;

        // Check if pixel is within viewport bounds (with margin)
        if (std::abs(fp.px) > hw - 10.0f || std::abs(fp.py) > hh - 10.0f)
          continue;

        // Convert pixel offset to ImGui screen coordinates
        // px goes [-hw, hw] → screen_x goes [vp_screen_x, vp_screen_x + vp_screen_w]
        // py goes [+hh, -hh] → screen_y goes [vp_screen_y, vp_screen_y + vp_screen_h] (y inverted)
        float scr_x = vp_screen_x + (fp.px + hw) / res_x * vp_screen_w;
        float scr_y = vp_screen_y + (hh - fp.py) / res_y * vp_screen_h;

        char buf[32];
        std::snprintf(buf, sizeof(buf), "%.0f\xC2\xB0", angle_deg);
        out.push_back({ scr_x, scr_y, std::string(buf), sun_col, true, kGroupSunCircles });
      }
    }
  };

  // === dispatch ===
  const bool full_sky = LensIsFullSky(input.lens_type);

  sample_viewport_edges();

  if (!full_sky) {
    if (input.visible == kVisibleUpper)
      sample_hemisphere_equator(-1.0f);
    if (input.visible == kVisibleLower)
      sample_hemisphere_equator(+1.0f);
    if (input.visible == kVisibleFront)
      sample_front_great_circle();
    if (input.lens_type != kLensTypeLinear)
      sample_interior_latitudes();
  }

  if (input.show_sun_circles && !full_sky) {
    sample_sun_circle_interior();
  }
}

void AppendOverlayToDrawList(ImDrawList* dl, const std::vector<OverlayLabel>& labels, float vp_screen_x,
                             float vp_screen_y, float vp_screen_w, float vp_screen_h) {
  if (dl == nullptr || labels.empty())
    return;

  // Collect bboxes for collision avoidance (only within same group)
  struct PlacedLabel {
    ImVec2 min, max;
    int group;
  };
  std::vector<PlacedLabel> placed;
  placed.reserve(labels.size());

  for (const auto& label : labels) {
    ImVec2 text_size = ImGui::CalcTextSize(label.text.c_str());
    ImVec2 pos(label.screen_x - text_size.x * 0.5f, label.screen_y - text_size.y * 0.5f);
    // Push the text glyph rect inside the viewport so it doesn't straddle the
    // panel border / export-PNG edge. Collision-avoidance below uses the
    // clamped bbox so labels pushed into the same corner don't pile up.
    pos = detail::ClampLabelPosToViewport(pos, text_size, vp_screen_x, vp_screen_y, vp_screen_w, vp_screen_h);
    ImVec2 bbox_min(pos.x - kLabelPadding, pos.y - kLabelPadding);
    ImVec2 bbox_max(pos.x + text_size.x + kLabelPadding, pos.y + text_size.y + kLabelPadding);

    // Check overlap only with same-group labels
    bool overlaps = false;
    for (const auto& p : placed) {
      if (p.group == label.group && bbox_min.x < p.max.x && bbox_max.x > p.min.x && bbox_min.y < p.max.y &&
          bbox_max.y > p.min.y) {
        overlaps = true;
        break;
      }
    }

    if (!overlaps) {
      if (label.has_bg) {
        // Background alpha scales with label alpha (controlled by overlay slider)
        constexpr int kMaxBgAlpha = 120;
        int label_alpha = (label.color >> IM_COL32_A_SHIFT) & 0xFF;
        int bg_alpha = kMaxBgAlpha * label_alpha / 255;
        dl->AddRectFilled(bbox_min, bbox_max, IM_COL32(0, 0, 0, bg_alpha), 2.0f);
      }
      // Draw text twice with 1px horizontal offset to simulate bold
      const char* text = label.text.c_str();
      dl->AddText(pos, label.color, text);
      dl->AddText(ImVec2(pos.x + 1.0f, pos.y), label.color, text);
      placed.push_back({ bbox_min, bbox_max, label.group });
    }
  }
}

void DrawOverlayLabels(const std::vector<OverlayLabel>& labels, float vp_screen_x, float vp_screen_y, float vp_screen_w,
                       float vp_screen_h) {
  AppendOverlayToDrawList(ImGui::GetWindowDrawList(), labels, vp_screen_x, vp_screen_y, vp_screen_w, vp_screen_h);
}

namespace detail {

// Test-only thin wrapper exposing the anonymous-namespace PixelToWorldDir so
// unit tests can pin per-lens dispatch. See overlay_labels.hpp for contract.
void PixelToWorldDirForTesting(float px, float py, float res_x, float res_y, int lens_type, float fov,
                               const float view_matrix[9], float* out_x, float* out_y, float* out_z, bool* out_valid) {
  InvResult r = PixelToWorldDir(px, py, res_x, res_y, lens_type, fov, view_matrix);
  *out_valid = r.valid;
  if (r.valid) {
    *out_x = r.x;
    *out_y = r.y;
    *out_z = r.z;
  }
}

ImVec2 ClampLabelPosToViewport(ImVec2 pos, ImVec2 text_size, float vp_x, float vp_y, float vp_w, float vp_h) {
  // kViewportInsetPx is a small visual padding (in screen-space pixels) keeping the
  // text glyph rect away from the absolute viewport edge — guards against the
  // half-pixel anti-aliasing fringe and the panel border 1 px line. 2 px is a
  // visual-padding heuristic, not a precise geometric boundary; revisit if a
  // future HiDPI-aware UI pass requires resolution-scaled padding.
  constexpr float kViewportInsetPx = 2.0f;

  // Only clamp if the viewport actually has room for the text + 2×inset. If
  // the viewport is narrower / shorter than that (extreme corner case for
  // panel collapse / tiny export targets), fall back to the original pos so
  // the legacy "centered on label anchor" behaviour is preserved.
  if (vp_w > text_size.x + 2.0f * kViewportInsetPx) {
    pos.x = std::max(pos.x, vp_x + kViewportInsetPx);
    pos.x = std::min(pos.x, vp_x + vp_w - text_size.x - kViewportInsetPx);
  }
  if (vp_h > text_size.y + 2.0f * kViewportInsetPx) {
    pos.y = std::max(pos.y, vp_y + kViewportInsetPx);
    pos.y = std::min(pos.y, vp_y + vp_h - text_size.y - kViewportInsetPx);
  }
  return pos;
}

}  // namespace detail

}  // namespace lumice::gui
