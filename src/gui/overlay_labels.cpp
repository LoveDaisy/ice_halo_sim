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

void AddLabel(std::vector<OverlayLabel>& out, float sx0, float sy0, float sx1, float sy1, float t, const char* fmt,
              float value, ImU32 color) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), fmt, value);
  out.push_back({ sx0 + t * (sx1 - sx0), sy0 + t * (sy1 - sy0), std::string(buf), color });
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

  ImU32 horizon_col = ColorToImU32(input.horizon_color, 200);
  ImU32 grid_col = ColorToImU32(input.grid_color, 200);
  ImU32 sun_col = ColorToImU32(input.sun_circles_color, 200);

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

  float hw = res_x * 0.5f, hh = res_y * 0.5f;
  Edge edges[4] = {
    { -hw, -hh, hw, -hh, vp_screen_x, vp_screen_y, vp_screen_x + vp_screen_w, vp_screen_y },  // top
    { -hw, hh, hw, hh, vp_screen_x, vp_screen_y + vp_screen_h, vp_screen_x + vp_screen_w,
      vp_screen_y + vp_screen_h },                                                            // bottom
    { -hw, -hh, -hw, hh, vp_screen_x, vp_screen_y, vp_screen_x, vp_screen_y + vp_screen_h },  // left
    { hw, -hh, hw, hh, vp_screen_x + vp_screen_w, vp_screen_y, vp_screen_x + vp_screen_w,
      vp_screen_y + vp_screen_h },  // right
  };

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

      InvResult r = PixelToWorldDir(px, py, res_x, res_y, input.lens_type, input.fov, view_matrix);

      SampleAngles cur{};
      cur.screen_x = edge.start_sx + frac * (edge.end_sx - edge.start_sx);
      cur.screen_y = edge.start_sy + frac * (edge.end_sy - edge.start_sy);
      cur.valid = r.valid;

      if (r.valid) {
        cur.altitude = std::asin(std::clamp(-r.z, -1.0f, 1.0f)) * kRad2Deg;
        cur.azimuth = std::atan2(-r.y, -r.x) * kRad2Deg;
        cur.sun_dist = std::acos(std::clamp(dot3(r.x, r.y, r.z, input.sun_dir[0], input.sun_dir[1], input.sun_dir[2]),
                                            -1.0f, 1.0f)) *
                       kRad2Deg;
      }

      if (prev.valid && cur.valid) {
        float t;

        // Horizon: altitude crosses 0
        if (input.show_horizon && Crosses(prev.altitude, cur.altitude, 0.0f, &t)) {
          AddLabel(out, prev.screen_x, prev.screen_y, cur.screen_x, cur.screen_y, t, "0\xC2\xB0", 0.0f, horizon_col);
        }

        // Grid: altitude crosses multiples of 10
        if (input.show_grid) {
          float alt_min = std::min(prev.altitude, cur.altitude);
          float alt_max = std::max(prev.altitude, cur.altitude);
          if (alt_max - alt_min < 20.0f) {  // skip if jump too large (invalid transition)
            for (int g = -8; g <= 8; g++) {
              float target = g * 10.0f;
              if (target == 0.0f)
                continue;  // horizon already handles 0
              if (Crosses(prev.altitude, cur.altitude, target, &t)) {
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

          if (std::abs(az1 - az0) < 20.0f) {  // skip if jump too large
            for (int g = -18; g <= 18; g++) {
              float target = g * 10.0f;
              if (Crosses(az0, az1, target, &t)) {
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
              AddLabel(out, prev.screen_x, prev.screen_y, cur.screen_x, cur.screen_y, t, "%.0f\xC2\xB0", target,
                       sun_col);
            }
          }
        }
      }

      prev = cur;
    }
  }
}

void DrawOverlayLabels(const std::vector<OverlayLabel>& labels) {
  if (labels.empty())
    return;

  ImDrawList* fg = ImGui::GetForegroundDrawList();

  // Collect bboxes for collision avoidance
  struct PlacedLabel {
    ImVec2 min, max;
  };
  std::vector<PlacedLabel> placed;
  placed.reserve(labels.size());

  for (const auto& label : labels) {
    ImVec2 text_size = ImGui::CalcTextSize(label.text.c_str());
    ImVec2 pos(label.screen_x - text_size.x * 0.5f, label.screen_y - text_size.y * 0.5f);
    ImVec2 bbox_min(pos.x - kLabelPadding, pos.y - kLabelPadding);
    ImVec2 bbox_max(pos.x + text_size.x + kLabelPadding, pos.y + text_size.y + kLabelPadding);

    // Check overlap with already placed labels
    bool overlaps = false;
    for (const auto& p : placed) {
      if (bbox_min.x < p.max.x && bbox_max.x > p.min.x && bbox_min.y < p.max.y && bbox_max.y > p.min.y) {
        overlaps = true;
        break;
      }
    }

    if (!overlaps) {
      fg->AddText(pos, label.color, label.text.c_str());
      placed.push_back({ bbox_min, bbox_max });
    }
  }
}

}  // namespace lumice::gui
