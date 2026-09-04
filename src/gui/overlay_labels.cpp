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
    // Core-pixel-inverse family: flip py (y-up screen → Core's y-down pixel layout),
    // matching the shader's pos.y flip. Keeps this CPU inverse == the shader inverse
    // and (with the matching WorldDirToPixel flip) a mutual inverse of the forward.
    r = DualFisheyeInv(px, -py, res_x, res_y, lens_type - kLensTypeDualFisheyeEqualArea);
    needs_view = false;
  } else if (lens_type == kLensTypeRectangular) {
    r = RectangularInv(px, -py, res_x, res_y);
    needs_view = false;
  } else if (lens_type == kLensTypeFisheyeOrthographic) {
    // Single orthographic: shader uses fisheyeInverse(type=3) and applies view matrix.
    r = FisheyeInv(px, py, res_x, res_y, half_fov, 3);
  } else if (lens_type == kLensTypeDualFisheyeOrthographic) {
    // Dual orthographic: shader uses dualFisheyeInverse(type=3), no view matrix.
    r = DualFisheyeInv(px, -py, res_x, res_y, 3);  // CPI family: flip py (see above)
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
  if (lens_type == kLensTypeRectangular) {
    // Inverse of RectangularInv: lat = asin(-wz), lon = atan2(-wy, -wx).
    // short_res / scale match RectangularInv (l.114-115) byte-for-byte so
    // round-trip (pixel→world→pixel) is exact within float error.
    float short_res = std::min(res_x * 0.5f, res_y);
    float scale = short_res / kPi;
    float clamped_neg_z = std::clamp(-dz, -1.0f, 1.0f);
    float lat = std::asin(clamped_neg_z);
    // |lat| ≈ π/2 (pole singularity): lon is undefined; reject.
    if (std::abs(lat) >= kPi * 0.5f - 1e-6f) {
      return { 0, 0, false };
    }
    float lon = std::atan2(-dy, -dx);
    // CPI family: flip py so this forward stays the mutual inverse of PixelToWorldDir
    // (which now flips py) and matches the shader's flipped display. Raw forward py is
    // -lat*scale; flipped → +lat*scale.
    return { lon * scale, lat * scale, true };
  }
  if ((lens_type >= kLensTypeDualFisheyeEqualArea && lens_type <= kLensTypeDualFisheyeStereographic) ||
      lens_type == kLensTypeDualFisheyeOrthographic) {
    // Inverse of DualFisheyeInv: wz≤0 → left disc (upper hemisphere),
    // wz>0 → right disc (lower hemisphere). Phi formulas derived from the
    // inv branch's (x, y, z) tuple — see comment block below each branch.
    float short_res = std::min(res_x * 0.5f, res_y);
    float circle_radius = short_res * 0.5f;
    int type = (lens_type == kLensTypeDualFisheyeOrthographic) ? 3 : (lens_type - kLensTypeDualFisheyeEqualArea);

    float half_pi = kPi * 0.5f;
    bool in_left = (dz <= 0.0f);
    float theta;
    float phi;
    if (in_left) {
      // Inv left: world = {-st*sin(phi), st*cos(phi), -ct} ⇒
      //   sin(phi) = -wx/sin(theta), cos(phi) = wy/sin(theta).
      theta = std::acos(std::clamp(-dz, -1.0f, 1.0f));
      phi = std::atan2(-dx, dy);
    } else {
      // Inv right: world = {-st*sin(phi), -st*cos(phi), ct} ⇒
      //   sin(phi) = -wx/sin(theta), cos(phi) = -wy/sin(theta).
      theta = std::acos(std::clamp(dz, -1.0f, 1.0f));
      phi = std::atan2(-dx, -dy);
    }

    float use_r;
    if (type == 0) {  // equal area: use_r = sin(theta/2) / sin(π/4)
      use_r = std::sin(theta * 0.5f) / std::sin(half_pi * 0.5f);
    } else if (type == 1) {  // equidistant: use_r = theta / (π/2)
      use_r = theta / half_pi;
    } else if (type == 2) {            // stereographic: use_r = tan(theta/2) / tan(π/4)
      use_r = std::tan(theta * 0.5f);  // tan(π/4) = 1
    } else {                           // orthographic (type==3): use_r = sin(theta)
      use_r = std::sin(theta);
    }

    // Domain guard: use_r > 1 means point lies past the disc edge in the
    // normalised disc space; the inverse's `r > 1.0` check rejects such
    // pixels, so the forward must do the same.
    if (use_r > 1.0f) {
      return { 0, 0, false };
    }

    // Special case: theta ≈ 0 means on the optical axis (disc center). phi
    // is degenerate but use_r ≈ 0 collapses lx/ly to 0, so the result is
    // just the disc center pixel.
    float lx = use_r * circle_radius * std::cos(phi);
    float ly = use_r * circle_radius * std::sin(phi);
    // CPI family: flip py (−ly) to match PixelToWorldDir's py flip + the shader.
    if (in_left) {
      return { lx - circle_radius, -ly, true };
    } else {
      return { lx + circle_radius, -ly, true };
    }
  }
  // Unknown / unsupported lens types: return invalid.
  return { 0, 0, false };
}

ImU32 ColorToImU32(const float c[3], int alpha) {
  return IM_COL32(static_cast<int>(c[0] * 255), static_cast<int>(c[1] * 255), static_cast<int>(c[2] * 255), alpha);
}

}  // namespace

void AppendCurveLabels(const CurveLabelSet& set, float vp_screen_x, float vp_screen_y, std::vector<OverlayLabel>& out) {
  const ImU32 col = ColorToImU32(set.color, static_cast<int>(set.alpha * 255));
  out.reserve(out.size() + set.anchors.size());
  for (const auto& a : set.anchors) {
    // The group comes from the set, not from a constant here: the collision pass has to keep
    // separating the circles from the grid exactly as the walks this replaced did — a 22 deg
    // marker and a 30 deg parallel marker may sit close together, two circle markers may not.
    out.push_back({ vp_screen_x + a.px, vp_screen_y + a.py, a.text, col, set.has_bg, set.group });
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

void WorldDirToPixelForTesting(float wx, float wy, float wz, float res_x, float res_y, int lens_type, float fov,
                               const float view_matrix[9], float* out_px, float* out_py, bool* out_valid) {
  FwdResult r = WorldDirToPixel(wx, wy, wz, res_x, res_y, lens_type, fov, view_matrix);
  *out_valid = r.valid;
  if (r.valid) {
    *out_px = r.px;
    *out_py = r.py;
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
