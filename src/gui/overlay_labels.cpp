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

// Synced with shader fisheyeInverse (preview_renderer.cpp line ~167-185)
InvResult FisheyeInv(float px, float py, float res_x, float res_y, float half_fov, int type) {
  float img_radius = std::min(res_x, res_y) * 0.5f;
  float r = std::sqrt(px * px + py * py) / img_radius;

  float theta;
  if (type == 0) {  // equal area
    float s = r * std::sin(half_fov * 0.5f);
    if (s > 1.0f)
      return { 0, 0, 0, false };
    theta = 2.0f * std::asin(s);
  } else if (type == 1) {  // equidistant
    theta = r * half_fov;
    if (theta >= kPi)
      return { 0, 0, 0, false };
  } else {  // stereographic
    theta = 2.0f * std::atan(r * std::tan(half_fov * 0.5f));
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
  if (type == 0) {
    float s = use_r * std::sin(half_pi * 0.5f);
    if (s > 1.0f)
      return { 0, 0, 0, false };
    theta = 2.0f * std::asin(s);
  } else if (type == 1) {
    theta = use_r * half_pi;
  } else {
    theta = 2.0f * std::atan(use_r * std::tan(half_pi * 0.5f));
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

// Compute world direction for a pixel offset from viewport center.
// Synced with shader main() (preview_renderer.cpp line ~302-318).
InvResult PixelToWorldDir(float px, float py, float res_x, float res_y, int lens_type, float fov,
                          const float view_matrix[9]) {
  float half_fov = fov * 0.5f * kDeg2Rad;

  InvResult r;
  bool needs_view = true;
  if (lens_type == 0) {
    r = LinearInv(px, py, res_x, res_y, half_fov);
  } else if (lens_type >= 1 && lens_type <= 3) {
    r = FisheyeInv(px, py, res_x, res_y, half_fov, lens_type - 1);
  } else if (lens_type >= 4 && lens_type <= 6) {
    r = DualFisheyeInv(px, py, res_x, res_y, lens_type - 4);
    needs_view = false;
  } else {
    r = RectangularInv(px, py, res_x, res_y);
    needs_view = false;
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
  // For types 0-3: transform world→view by multiplying with transpose of view matrix
  float dx = wx, dy = wy, dz = wz;
  bool needs_view = (lens_type >= 0 && lens_type <= 3);
  if (needs_view) {
    // view_matrix is column-major: M[col*3+row]. Transpose = rows become columns.
    dx = view_matrix[0] * wx + view_matrix[1] * wy + view_matrix[2] * wz;
    dy = view_matrix[3] * wx + view_matrix[4] * wy + view_matrix[5] * wz;
    dz = view_matrix[6] * wx + view_matrix[7] * wy + view_matrix[8] * wz;
  }

  float half_fov = fov * 0.5f * kDeg2Rad;
  float short_edge = std::min(res_x, res_y);

  if (lens_type == 0) {  // Linear
    if (dz >= 0)
      return { 0, 0, false };  // behind camera (shader convention: camera looks -z)
    float focal = short_edge * 0.5f / std::tan(half_fov);
    return { dx / (-dz) * focal, dy / (-dz) * focal, true };
  } else if (lens_type >= 1 && lens_type <= 3) {  // Fisheye
    float img_radius = short_edge * 0.5f;
    float theta = std::acos(std::clamp(-dz, -1.0f, 1.0f));  // angle from -z axis
    float rho = std::sqrt(dx * dx + dy * dy);
    if (rho < 1e-8f)
      return { 0, 0, true };  // on optical axis

    float r_norm;
    int type = lens_type - 1;
    if (type == 0) {
      r_norm = std::sin(theta * 0.5f) / std::sin(half_fov * 0.5f);
    } else if (type == 1) {
      r_norm = theta / half_fov;
    } else {
      r_norm = std::tan(theta * 0.5f) / std::tan(half_fov * 0.5f);
    }
    float scale = r_norm * img_radius / rho;
    return { dx * scale, dy * scale, true };
  }
  // Types 4-7: not needed for sun circle interior labels (they use world space directly,
  // and the full sky is always visible, so sun circles always intersect viewport edges).
  return { 0, 0, false };
}

// Angle values computed for each edge sample point.
struct SampleAngles {
  float altitude;            // degrees
  float azimuth;             // degrees, [-180, 180]
  float sun_dist;            // degrees, [0, 180]
  float screen_x, screen_y;  // ImGui screen coords
  bool valid;
};

// Check if value crosses a target between two samples. Returns interpolated crossing position.
bool Crosses(float v0, float v1, float target, float* t_out) {
  if ((v0 - target) * (v1 - target) < 0) {
    *t_out = (target - v0) / (v1 - v0);
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

  // Viewport pixel dimensions (used as u_resolution in shader)
  float res_x = vp_screen_w;
  float res_y = vp_screen_h;

  int grid_a = static_cast<int>(input.grid_alpha * 255);
  int sun_a = static_cast<int>(input.sun_circles_alpha * 255);
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

  // Hemisphere visibility check: returns true if the given altitude is in the visible hemisphere.
  // visible: 0=upper (alt>=0), 1=lower (alt<=0), 2=full (always visible).
  auto is_visible = [&](float alt) -> bool {
    if (input.visible == 0)
      return alt >= -0.5f;  // small tolerance for edge labels
    if (input.visible == 1)
      return alt <= 0.5f;
    return true;
  };

  // Crossing detection helper: given two adjacent valid sample points, detect and add labels.
  auto detect_crossings = [&](const SampleAngles& prev, const SampleAngles& cur) {
    float t;

    // Grid: altitude crosses multiples of 10
    if (input.show_grid) {
      if (std::abs(prev.altitude - cur.altitude) < 20.0f) {
        for (int g = -9; g <= 9; g++) {
          if (g == 0)
            continue;  // skip 0° altitude (horizon) — low value, avoids equator edge clutter
          float target = g * 10.0f;
          if (Crosses(prev.altitude, cur.altitude, target, &t) && is_visible(target)) {
            AddLabel(out, prev.screen_x, prev.screen_y, cur.screen_x, cur.screen_y, t, "%.0f\xC2\xB0", target,
                     grid_col);
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
            if (!is_visible(alt_at_crossing))
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
          if (!is_visible(alt_at_crossing))
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
    }
    return s;
  };

  // Sample along viewport edges
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

  // When visible = upper(0) or lower(1), add the hemisphere boundary (equator) as an extra edge.
  // The equator is defined by altitude = 0 (world_dir.z = 0). Sample it in world space
  // and forward-project to pixel coordinates.
  if (input.visible != 2 && input.lens_type >= 0 && input.lens_type <= 3) {
    constexpr int kEquatorSamples = 360;
    SampleAngles prev{};
    prev.valid = false;

    for (int i = 0; i <= kEquatorSamples; i++) {
      float az_rad = -kPi + i * (2.0f * kPi / kEquatorSamples);
      // Equator direction: altitude=0, world_dir.z=0
      float wx = -std::cos(az_rad);
      float wy = -std::sin(az_rad);
      float wz = 0.0f;

      FwdResult fp = WorldDirToPixel(wx, wy, wz, res_x, res_y, input.lens_type, input.fov, view_matrix);
      if (!fp.valid || std::abs(fp.px) > hw || std::abs(fp.py) > hh) {
        prev.valid = false;
        continue;
      }

      float scr_x = vp_screen_x + (fp.px + hw) / res_x * vp_screen_w;
      float scr_y = vp_screen_y + (hh - fp.py) / res_y * vp_screen_h;

      // Compute angles directly from the known equator direction — avoid inverse projection
      // round-trip which loses precision at fisheye disc edge (r ≈ 1).
      SampleAngles cur{};
      cur.screen_x = scr_x;
      cur.screen_y = scr_y;
      cur.valid = true;
      cur.altitude = 0.0f;  // equator by definition
      cur.azimuth = az_rad * kRad2Deg;
      cur.sun_dist =
          std::acos(std::clamp(dot3(wx, wy, wz, input.sun_dir[0], input.sun_dir[1], input.sun_dir[2]), -1.0f, 1.0f)) *
          kRad2Deg;

      if (prev.valid && cur.valid)
        detect_crossings(prev, cur);
      prev = cur;
    }
  }

  // Sun circles: add interior labels when circles don't intersect viewport edges.
  // Place 4 labels evenly around each sun circle using forward projection.
  if (input.show_sun_circles && input.lens_type >= 0 && input.lens_type <= 3) {
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
  }
}

void DrawOverlayLabels(const std::vector<OverlayLabel>& labels) {
  if (labels.empty())
    return;

  ImDrawList* fg = ImGui::GetForegroundDrawList();

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
        fg->AddRectFilled(bbox_min, bbox_max, IM_COL32(0, 0, 0, bg_alpha), 2.0f);
      }
      // Draw text twice with 1px horizontal offset to simulate bold
      const char* text = label.text.c_str();
      fg->AddText(pos, label.color, text);
      fg->AddText(ImVec2(pos.x + 1.0f, pos.y), label.color, text);
      placed.push_back({ bbox_min, bbox_max, label.group });
    }
  }
}

}  // namespace lumice::gui
