#include "core/annotation_overlay.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "core/geo3d.hpp"
#include "core/math.hpp"
#include "core/parallel_rows.hpp"
#include "core/scatter_accum.hpp"  // MakeCameraRotation

namespace lumice::annotation {

namespace {

// Sampling density of the curve walk, ported from overlay_labels.cpp: 1 deg per azimuth step on a
// closed ring, 1 deg per altitude step on a meridian. Both sides must sample the SAME way or the
// boundary-entry anchor lands on a different sample and the parity gate reads a shift that is not
// a projection difference.
constexpr int kCurveAzSteps = 360;
constexpr int kCurveAltSteps = 180;

constexpr float kRad2Deg = 180.0f / math::kPi;

// Whether a sample survives the hemisphere / front policy this layer owns. Deliberately NOT
// delegated to ProjectExitToPixel's own visible_range cull: that cull lives inside the single-lens
// branch only, so relying on it would apply the rule to five lens types and skip the other six.
// ComputeOverlay hands the projection kFull and asks this predicate for every type instead — the
// same uniform-rule choice BuildVisibleMask's VisibleByRange comment argues for.
bool VisibleForLabel(RenderConfig::VisibleRange visible, bool front, const float forward[3], float alt_deg, float wx,
                     float wy, float wz) {
  if (visible == RenderConfig::kUpper && alt_deg < -kLabelHemisphereToleranceDeg) {
    return false;
  }
  if (visible == RenderConfig::kLower && alt_deg > kLabelHemisphereToleranceDeg) {
    return false;
  }
  if (front && forward[0] * wx + forward[1] * wy + forward[2] * wz < -kFrontEps) {
    return false;
  }
  return true;
}

// "%.0f deg" when the value is a whole number of degrees, "%.1f deg" otherwise. The GUI picks its
// format from its single grid_step instead; core has no such thing (it takes explicit angle
// lists), so the value itself decides. The two agree wherever the step is a whole number of
// degrees, which is every configuration shipping today.
std::string FormatAngleDeg(float value_deg) {
  char buf[32];
  const bool integral = std::fabs(value_deg - std::round(value_deg)) < 1e-3f;
  // U+00B0 DEGREE SIGN, UTF-8. Written as escaped bytes so the source file stays ASCII.
  std::snprintf(buf, sizeof(buf), integral ? "%.0f\xC2\xB0" : "%.1f\xC2\xB0", value_deg);
  return { buf };
}

// One walked sample of a level-set curve.
struct CurveSample {
  float wx = 0.0f;
  float wy = 0.0f;
  float wz = 0.0f;
  float px = 0.0f;
  float py = 0.0f;
  // valid projection AND inside the canvas AND inside the hemisphere policy. The only flag the
  // boundary/interior dispatch needs.
  bool vis = false;
};

// Everything the curve walk needs, assembled once per request.
struct WalkContext {
  lm_proj::ProjParams proj{};
  RenderConfig::VisibleRange visible = RenderConfig::kFull;
  bool front = false;
  float forward[3] = { 0.0f, 0.0f, 0.0f };
  int width = 0;
  int height = 0;
};

// Is the projected point on this canvas, and where exactly?
//
// The containment test is CLOSED at the far edge (`<= width`, not `< width`) and the accepted
// point is then clamped to an addressable pixel. Both halves are needed, and for the same reason:
// ProjectExitToPixel bins with floor(v + 0.5) about res/2, half a pixel off the symmetric
// convention the masks and the GUI both use (lens_proj_build.hpp states the offset in its own
// header comment). A direction the symmetric convention places in the LAST row therefore bins to
// `res`. Rejecting it would drop the outermost ring of every fisheye — the equator of an all-sky
// dual fisheye lands exactly there — and, worse, would not merely lose that anchor but move it:
// the meridian walk below searches outward from the equator, so one rejected sample hands the
// label to its neighbour, which on a dual fisheye is the OTHER disc, half a canvas away.
// Accepting and clamping keeps the sample set identical to the GUI's (whose own viewport test is
// likewise closed) while still reporting a pixel a consumer can address.
bool ClampToCanvas(int width, int height, CanvasPoint* p) {
  if (p->px < 0.0f || p->px > static_cast<float>(width) || p->py < 0.0f || p->py > static_cast<float>(height)) {
    return false;
  }
  p->px = std::clamp(p->px, 0.0f, static_cast<float>(width - 1));
  p->py = std::clamp(p->py, 0.0f, static_cast<float>(height - 1));
  return true;
}

CurveSample SampleWorldDir(const WalkContext& ctx, float alt_deg, float wx, float wy, float wz) {
  CurveSample s;
  s.wx = wx;
  s.wy = wy;
  s.wz = wz;
  CanvasPoint cp = ProjectWorldDir(ctx.proj, wx, wy, wz);
  if (!cp.valid) {
    return s;
  }
  if (!ClampToCanvas(ctx.width, ctx.height, &cp)) {
    return s;
  }
  if (!VisibleForLabel(ctx.visible, ctx.front, ctx.forward, alt_deg, wx, wy, wz)) {
    return s;
  }
  s.px = cp.px;
  s.py = cp.py;
  s.vis = true;
  return s;
}

// One label per visible ARC: every invisible->visible transition along the sample sequence emits
// at the entry sample. Closed curves include sample[N] == sample[0], so the wrap-around transition
// is found without extra bookkeeping. When the curve never leaves the visible region there is no
// transition to anchor on, and the first visible sample is the canonical fallback.
// Ported from overlay_labels.cpp's emit_curve_label.
void EmitCurveLabel(const std::vector<CurveSample>& samples, LabelKind kind, int index, float value_deg,
                    const std::string& text, std::vector<Label>& out) {
  int boundary_count = 0;
  for (size_t i = 1; i < samples.size(); ++i) {
    if (!samples[i - 1].vis && samples[i].vis) {
      out.push_back({ samples[i].px, samples[i].py, kind, index, value_deg, text });
      ++boundary_count;
    }
  }
  if (boundary_count > 0) {
    return;
  }
  for (const auto& s : samples) {
    if (s.vis) {
      out.push_back({ s.px, s.py, kind, index, value_deg, text });
      return;
    }
  }
}

// Parallel (constant altitude): a closed curve sweeping azimuth over a full turn.
// Direction convention is mask_detail::AltitudeDeg's own, read backwards:
//   altitude = asin(-z)              -> z = -sin(alt)
//   azimuth  = atan2(-y, -x)         -> x = -cos(alt)cos(az), y = -cos(alt)sin(az)
std::vector<CurveSample> WalkAltitudeCurve(const WalkContext& ctx, float alt_deg) {
  const float alt_rad = alt_deg * math::kDegreeToRad;
  const float cos_a = std::cos(alt_rad);
  const float sin_a = std::sin(alt_rad);
  std::vector<CurveSample> samples;
  samples.reserve(kCurveAzSteps + 1);
  for (int i = 0; i <= kCurveAzSteps; ++i) {
    const float az_rad = 2.0f * math::kPi * static_cast<float>(i) / static_cast<float>(kCurveAzSteps);
    samples.push_back(SampleWorldDir(ctx, alt_deg, -cos_a * std::cos(az_rad), -cos_a * std::sin(az_rad), -sin_a));
  }
  return samples;
}

// Meridian (constant azimuth): an open curve sweeping altitude pole to pole.
std::vector<CurveSample> WalkLongitudeCurve(const WalkContext& ctx, float az_deg) {
  const float az_rad = az_deg * math::kDegreeToRad;
  const float cos_az = std::cos(az_rad);
  const float sin_az = std::sin(az_rad);
  std::vector<CurveSample> samples;
  samples.reserve(kCurveAltSteps + 1);
  for (int i = 0; i <= kCurveAltSteps; ++i) {
    const float alt_deg = -90.0f + 180.0f * static_cast<float>(i) / static_cast<float>(kCurveAltSteps);
    const float alt_rad = alt_deg * math::kDegreeToRad;
    const float cos_a = std::cos(alt_rad);
    samples.push_back(SampleWorldDir(ctx, alt_deg, -cos_a * cos_az, -cos_a * sin_az, -std::sin(alt_rad)));
  }
  return samples;
}

// An orthonormal frame around `dir`, so a ring at fixed angular distance can be swept.
// Returns false when `dir` is degenerate.
bool BuildRingFrame(const float dir[3], float u[3], float v[3]) {
  if (std::fabs(dir[2]) < 0.9f) {
    u[0] = dir[1];
    u[1] = -dir[0];
    u[2] = 0.0f;
  } else {
    u[0] = 0.0f;
    u[1] = dir[2];
    u[2] = -dir[1];
  }
  const float len = std::sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
  if (len < 1e-6f) {
    return false;
  }
  u[0] /= len;
  u[1] /= len;
  u[2] /= len;
  v[0] = dir[1] * u[2] - dir[2] * u[1];
  v[1] = dir[2] * u[0] - dir[0] * u[2];
  v[2] = dir[0] * u[1] - dir[1] * u[0];
  return true;
}

void RingDirAt(const float dir[3], const float u[3], const float v[3], float cos_d, float sin_d, float phi,
               float out[3]) {
  const float cp = std::cos(phi);
  const float sp = std::sin(phi);
  for (int k = 0; k < 3; ++k) {
    out[k] = cos_d * dir[k] + sin_d * (cp * u[k] + sp * v[k]);
  }
}

float AltitudeDegOfDir(float wz) {
  return std::asin(std::clamp(-wz, -1.0f, 1.0f)) * kRad2Deg;
}

// Azimuth in degrees, in (-180, 180]. The inverse of WalkAltitudeCurve's construction, so a
// meridian level set and a meridian curve walk are talking about the same angle.
float AzimuthDegOfDir(float wx, float wy) {
  return std::atan2(-wy, -wx) * kRad2Deg;
}

// Angular distance in degrees between a direction and a (unit) reference.
float AngularDistDegOfDir(const float ref[3], float wx, float wy, float wz) {
  const float d = ref[0] * wx + ref[1] * wy + ref[2] * wz;
  return std::acos(std::clamp(d, -1.0f, 1.0f)) * kRad2Deg;
}

}  // namespace

void SunWorldDir(const SunParam& sun, float* out) {
  const float az = sun.azimuth_ * math::kDegreeToRad;
  const float alt = sun.altitude_ * math::kDegreeToRad;
  const float c_alt = std::cos(alt);
  out[0] = -std::cos(az) * c_alt;
  out[1] = -std::sin(az) * c_alt;
  out[2] = -std::sin(alt);
}

RenderConfig ToRenderConfig(const ViewSnapshot& view) {
  RenderConfig cfg;
  cfg.resolution_[0] = view.width;
  cfg.resolution_[1] = view.height;
  cfg.lens_.type_ = view.lens_type;
  cfg.lens_.fov_ = view.fov_deg;
  cfg.lens_shift_[0] = view.lens_shift[0];
  cfg.lens_shift_[1] = view.lens_shift[1];
  cfg.overlap_ = view.overlap;
  cfg.view_.az_ = view.az_deg;
  cfg.view_.el_ = view.el_deg;
  cfg.view_.ro_ = view.roll_deg;
  cfg.visible_ = view.visible;
  cfg.front_ = view.front;
  return cfg;
}

CanvasPoint ProjectWorldDir(const lm_proj::ProjParams& p, float wx, float wy, float wz) {
  const lm_proj::ProjResult r = lm_proj::ProjectExitToPixel(p, wx, wy, wz);
  if (r.count == 0) {
    return {};
  }
  return { static_cast<float>(r.hits[0].px), static_cast<float>(r.hits[0].py), true };
}

Overlay ComputeOverlay(const Request& req) {
  Overlay out;
  const int width = req.view.width;
  const int height = req.view.height;
  if (width <= 0 || height <= 0) {
    return out;
  }
  out.width = width;
  out.height = height;
  const size_t n = static_cast<size_t>(width) * static_cast<size_t>(height);

  const RenderConfig cfg = ToRenderConfig(req.view);
  const Rotation rot = MakeCameraRotation(cfg);
  const float short_pix = static_cast<float>(std::min(width, height));
  const lm_proj::ProjParams inverse_params = BuildProjParams(cfg, rot, short_pix);

  // The forward used for anchors asks the lens "do you image this direction", nothing else: the
  // hemisphere policy is applied uniformly by VisibleForLabel afterwards (see its comment).
  lm_proj::ProjParams forward_params = inverse_params;
  forward_params.visible_range = static_cast<int>(RenderConfig::kFull);

  // Camera forward in world space. ProjectExitToPixel's single-lens branch computes
  // c = R^T * (-w) and keeps c.z > 0, so the camera looks along -R * (0,0,1).
  float forward[3]{ 0.0f, 0.0f, 1.0f };
  rot.Apply(forward);
  forward[0] = -forward[0];
  forward[1] = -forward[1];
  forward[2] = -forward[2];

  float ref_dir[3]{ req.reference_dir[0], req.reference_dir[1], req.reference_dir[2] };
  {
    const float len = std::sqrt(ref_dir[0] * ref_dir[0] + ref_dir[1] * ref_dir[1] + ref_dir[2] * ref_dir[2]);
    if (len > 1e-8f) {
      ref_dir[0] /= len;
      ref_dir[1] /= len;
      ref_dir[2] /= len;
    } else {
      ref_dir[0] = 0.0f;
      ref_dir[1] = 0.0f;
      ref_dir[2] = -1.0f;
    }
  }

  const bool need_alt = req.horizon || !req.elevation_deg.empty();
  const bool need_az = !req.longitude_deg.empty();
  const bool need_dist = !req.angular_dist_deg.empty();

  std::vector<uint8_t> imaged(n, 0);
  out.drawable.assign(n, 0);
  std::vector<float> alt_field(need_alt ? n : 0, 0.0f);
  std::vector<float> az_field(need_az ? n : 0, 0.0f);
  std::vector<float> dist_field(need_dist ? n : 0, 0.0f);

  // One inverse projection per pixel feeds every field, which is what AC1's "any new angle field
  // goes into the SAME loop" buys: three annotation categories cost one sweep, not three.
  ParallelRows(height, n, [&](int row_begin, int row_end) {
    for (int py = row_begin; py < row_end; py++) {
      for (int px = 0; px < width; px++) {
        const mask_detail::MaskDir dir = mask_detail::PixelToWorld(cfg, inverse_params, rot, px, py);
        if (!dir.valid) {
          continue;
        }
        const size_t i = static_cast<size_t>(py) * static_cast<size_t>(width) + static_cast<size_t>(px);
        imaged[i] = 1;
        bool drawable = mask_detail::VisibleByRange(cfg.visible_, dir.z);
        if (drawable && req.view.front) {
          drawable = forward[0] * dir.x + forward[1] * dir.y + forward[2] * dir.z >= -kFrontEps;
        }
        out.drawable[i] = drawable ? 1 : 0;
        if (need_alt) {
          alt_field[i] = mask_detail::AltitudeDeg(dir);
        }
        if (need_az) {
          az_field[i] = AzimuthDegOfDir(dir.x, dir.y);
        }
        if (need_dist) {
          dist_field[i] = AngularDistDegOfDir(ref_dir, dir.x, dir.y, dir.z);
        }
      }
    }
  });

  if (req.horizon) {
    out.horizon = mask_detail::LevelSetMaskFromField(alt_field, imaged, out.drawable, width, height, { 0.0f }, false);
  }
  if (!req.elevation_deg.empty()) {
    out.elevation =
        mask_detail::LevelSetMaskFromField(alt_field, imaged, out.drawable, width, height, req.elevation_deg, false);
  }
  if (!req.longitude_deg.empty()) {
    // Circular: azimuth wraps at +/-180, and both the local gradient and the distance to a level
    // have to be measured on the circle or the seam draws a spurious full-height meridian.
    out.longitude =
        mask_detail::LevelSetMaskFromField(az_field, imaged, out.drawable, width, height, req.longitude_deg, true);
  }
  if (!req.angular_dist_deg.empty()) {
    out.angular_dist = mask_detail::LevelSetMaskFromField(dist_field, imaged, out.drawable, width, height,
                                                          req.angular_dist_deg, false);
  }

  WalkContext ctx;
  ctx.proj = forward_params;
  ctx.visible = cfg.visible_;
  ctx.front = req.view.front;
  ctx.forward[0] = forward[0];
  ctx.forward[1] = forward[1];
  ctx.forward[2] = forward[2];
  ctx.width = width;
  ctx.height = height;

  if (req.zenith_nadir) {
    // Not level sets: two fixed world directions. altitude = asin(-z), so zenith is z = -1. Run
    // through the same sampler as every curve point, so "the marker is on screen" means exactly
    // what "this anchor is on screen" means everywhere else.
    const CurveSample zs = SampleWorldDir(ctx, 90.0f, 0.0f, 0.0f, -1.0f);
    const CurveSample ns = SampleWorldDir(ctx, -90.0f, 0.0f, 0.0f, 1.0f);
    out.zenith = { zs.px, zs.py, zs.vis };
    out.nadir = { ns.px, ns.py, ns.vis };
  }

  if (!req.labels) {
    return out;
  }

  if (req.horizon) {
    EmitCurveLabel(WalkAltitudeCurve(ctx, 0.0f), kLabelHorizon, -1, 0.0f, FormatAngleDeg(0.0f), out.labels);
  }
  for (size_t k = 0; k < req.elevation_deg.size(); ++k) {
    const float value = req.elevation_deg[k];
    EmitCurveLabel(WalkAltitudeCurve(ctx, value), kLabelElevation, static_cast<int>(k), value, FormatAngleDeg(value),
                   out.labels);
  }
  for (size_t k = 0; k < req.longitude_deg.size(); ++k) {
    const float value = req.longitude_deg[k];
    const std::vector<CurveSample> samples = WalkLongitudeCurve(ctx, value);
    // Meridians converge at the poles, so the generic boundary/first-visible anchor stacks every
    // meridian label on top of the pole. Anchor at the intersection with the reference parallel
    // instead — the equator if it is visible, else the nearest visible sample to it — where
    // meridians are maximally separated in azimuth and the labels stay distinct.
    // (Ported from overlay_labels.cpp's process_longitude_curve; owner-chosen placement.)
    float label_value = value;
    if (label_value > 180.0f) {
      label_value -= 360.0f;
    }
    if (label_value <= -180.0f) {
      label_value += 360.0f;
    }
    const std::string text = FormatAngleDeg(label_value);
    const int mid = kCurveAltSteps / 2;
    for (int off = 0; off <= mid; ++off) {
      const int lo = mid - off;
      const int hi = mid + off;
      if (lo >= 0 && samples[static_cast<size_t>(lo)].vis) {
        out.labels.push_back({ samples[static_cast<size_t>(lo)].px, samples[static_cast<size_t>(lo)].py,
                               kLabelLongitude, static_cast<int>(k), label_value, text });
        break;
      }
      if (hi <= kCurveAltSteps && samples[static_cast<size_t>(hi)].vis) {
        out.labels.push_back({ samples[static_cast<size_t>(hi)].px, samples[static_cast<size_t>(hi)].py,
                               kLabelLongitude, static_cast<int>(k), label_value, text });
        break;
      }
    }
  }
  for (size_t k = 0; k < req.angular_dist_deg.size(); ++k) {
    const float value = req.angular_dist_deg[k];
    float u[3];
    float v[3];
    if (!BuildRingFrame(ref_dir, u, v)) {
      continue;
    }
    const float cos_d = std::cos(value * math::kDegreeToRad);
    const float sin_d = std::sin(value * math::kDegreeToRad);
    std::vector<CurveSample> samples;
    samples.reserve(kCurveAzSteps + 1);
    for (int i = 0; i <= kCurveAzSteps; ++i) {
      const float phi = 2.0f * math::kPi * static_cast<float>(i) / static_cast<float>(kCurveAzSteps);
      float d[3];
      RingDirAt(ref_dir, u, v, cos_d, sin_d, phi, d);
      samples.push_back(SampleWorldDir(ctx, AltitudeDegOfDir(d[2]), d[0], d[1], d[2]));
    }
    const std::string text = FormatAngleDeg(value);

    int boundary_count = 0;
    for (size_t i = 1; i < samples.size(); ++i) {
      if (!samples[i - 1].vis && samples[i].vis) {
        out.labels.push_back({ samples[i].px, samples[i].py, kLabelAngularDist, static_cast<int>(k), value, text });
        ++boundary_count;
      }
    }
    if (boundary_count > 0) {
      continue;
    }
    // Interior mode: the whole ring is in view, so there is no entry point to anchor on. Four
    // canonical anchors a quarter turn apart around the reference direction, matching the GUI.
    const bool any_vis = std::any_of(samples.begin(), samples.end(), [](const CurveSample& s) { return s.vis; });
    if (!any_vis) {
      continue;
    }
    for (int li = 0; li < 4; ++li) {
      float d[3];
      RingDirAt(ref_dir, u, v, cos_d, sin_d, static_cast<float>(li) * math::kPi_2, d);
      const CurveSample s = SampleWorldDir(ctx, AltitudeDegOfDir(d[2]), d[0], d[1], d[2]);
      if (s.vis) {
        out.labels.push_back({ s.px, s.py, kLabelAngularDist, static_cast<int>(k), value, text });
      }
    }
  }

  return out;
}

}  // namespace lumice::annotation
