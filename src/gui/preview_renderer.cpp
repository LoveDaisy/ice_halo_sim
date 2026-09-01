#include "gui/preview_renderer.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "gui/gl_common.h"
#include "gui/gui_logger.hpp"

namespace lumice::gui {

// clang-format off
static const char* kVertexShader = R"glsl(
#version 330 core
layout(location = 0) in vec2 a_pos;
out vec2 v_ndc;
void main() {
  gl_Position = vec4(a_pos, 0.0, 1.0);
  v_ndc = a_pos;  // Pass NDC [-1,1] to fragment shader
}
)glsl";

static const char* kFragmentShader = R"glsl(
#version 330 core

in vec2 v_ndc;               // NDC position [-1, 1]

uniform sampler2D u_texture;
uniform vec2 u_resolution;   // viewport size in pixels
uniform int u_lens_type;     // 0=linear, 1-3=fisheye, 4-6=dual fisheye, 7=rectangular, 8=fisheye_orthographic, 9=dual_fisheye_orthographic, 10=globe
uniform float u_fov;         // full FOV in degrees
uniform mat3 u_view_matrix;  // view-to-world rotation (inverse view)
uniform int u_visible;       // 0=upper, 1=lower, 2=full
uniform int u_front;         // 1=discard back hemisphere
uniform float u_intensity_scale;  // = intensity_factor / per_pixel_intensity (0 = RGB mode)
uniform int u_xyz_mode;           // 1 = XYZ float texture, 0 = sRGB uint8 texture
uniform vec3 u_background;        // sky colour, LINEAR RGB (see PreviewParams::background_color_linear)
uniform sampler2D u_bg_texture;
uniform float u_max_abs_dz;      // overlap zone |sky.z| threshold (0 = no blend)
uniform float u_r_scale;         // projection r_scale for overlap normalization
uniform int u_bg_enabled;
uniform float u_overlay_alpha;
uniform vec2 u_bg_uv_scale;
uniform vec2 u_bg_uv_offset;

// Auxiliary line overlay uniforms
uniform int u_show_horizon;
uniform int u_show_grid;
uniform int u_show_sun_circles;
// 1 where an angular-distance circle passes, computed by core (annotation_overlay.cpp) once per
// settled view and uploaded as an R8 texture. The shader used to derive this per fragment from
// acos(dot(world_dir, sun_dir)); it no longer does, so that the CLI renderer and this preview draw
// the SAME curve rather than two implementations of it. Sampled NEAREST, one texel per fragment.
uniform sampler2D u_angular_dist_mask;
uniform int u_has_angular_dist_mask;
uniform vec3 u_horizon_color;
uniform vec3 u_grid_color;
uniform vec3 u_sun_circles_color;
uniform float u_horizon_alpha;
uniform float u_grid_alpha;
uniform float u_sun_circles_alpha;
uniform float u_grid_step;  // degrees; FOV-adaptive, fed from OverlayDecoration::grid_step

// Zenith / Nadir pixel-space ring marker uniforms (task-gui-zenith-nadir-marker).
// Screen positions are center-origin, y-up, in pixels — same convention as
// `pos = v_ndc * u_resolution * 0.5` (see main()). Sentinel (-9999, -9999)
// trivially fails the distance test so the ring is skipped.
uniform int u_show_zenith_nadir;
uniform vec2 u_zenith_screen_pos;
uniform vec2 u_nadir_screen_pos;
uniform float u_zenith_nadir_radius_px;
uniform vec3 u_zenith_nadir_color;
uniform float u_zenith_nadir_alpha;

// Lens border uniforms: the outline of the projection's own valid image circle
// (where the inverse function's domain guard flips w from 1 to 0). Purely a
// function of u_lens_type / u_fov / u_resolution, so the shader derives the
// radius itself rather than taking a CPU-precomputed screen position.
uniform int u_show_lens_border;
uniform vec3 u_lens_border_color;
uniform float u_lens_border_alpha;

const float PI = 3.14159265358979323846;

// Algorithm synced with CPU: src/util/color_space.hpp (GamutClipXyz + XyzToLinearRgb + LinearToSrgb)
// Matrix values from src/util/color_data.hpp kXyzToRgb (C++ row-major → GLSL column-major)
const mat3 kXyzToRgb = mat3(
     3.2404542, -0.9692660,  0.0556434,   // column 0
    -1.5371385,  1.8760108, -0.2040259,   // column 1
    -0.4985314,  0.0415560,  1.0572252    // column 2
);
const vec3 kWhitePointD65 = vec3(0.95047, 1.00000, 1.08883);

// The XYZ->sRGB chain, deliberately in two halves with a seam between them.
//
// The seam is where the background colour is added, and it has to be there: the addition is a
// radiance sum, so it belongs in LINEAR RGB, before the clamp and before the sRGB transfer curve.
// Add it after the gamma instead and a pixel carrying no halo energy comes back gamma-encoded a
// second time, i.e. NOT the colour the user picked. Keep the two calls adjacent at every call
// site — the halves are not independently meaningful.
//
// CPU equivalents: lumice::XyzToSrgbUint8(..., background) in src/util/color_space.hpp composes
// the same two halves around the same seam — that is the one the .lmc bake reaches through
// LUMICE_XyzToSrgbUint8WithBackground — and so does RenderConsumer::PostSnapshot's use_real_color
// branch (src/server/render.cpp). Three implementations of one chain: one has to be GLSL, and the
// other two are separate because the render loop writes bytes as it composites while the batch
// converter is handed a finished buffer. Nothing enforces that they agree; the pixel comparison in
// test/gui/functional/test_preview_background.cpp is what holds this one to the CPU one.

// Gamut clip + XYZ->RGB matrix. NOT clamped and NOT gamma-encoded: the background composites here.
vec3 xyzToLinearRgb(vec3 xyz) {
    // Normalize by accumulated intensity
    xyz *= u_intensity_scale;
    // Gray point (D65 white scaled by luminance Y)
    vec3 gray = kWhitePointD65 * xyz.y;
    // Gamut clipping: scale color toward gray to keep RGB in [0,1]
    float s = 1.0;
    vec3 diff = xyz - gray;
    mat3 kXyzToRgbT = transpose(kXyzToRgb);
    for (int j = 0; j < 3; j++) {
        float a = dot(-gray, kXyzToRgbT[j]);
        float b = dot(diff, kXyzToRgbT[j]);
        if (a * b > 0.0 && a / b < s) s = a / b;
    }
    xyz = diff * s + gray;
    // XYZ→RGB matrix multiply. The clamp the CPU sibling applies here is a no-op after the gamut
    // clip above (the clip is defined as the scaling that lands this product inside [0,1]); the
    // clamp that matters runs in clampAndGamma, after the background has been added.
    return kXyzToRgb * xyz;
}

// Clamp + sRGB gamma (branchless via mix+step). The second half of the chain above.
vec3 clampAndGamma(vec3 rgb) {
    rgb = clamp(rgb, 0.0, 1.0);
    return mix(rgb * 12.92, 1.055 * pow(rgb, vec3(1.0/2.4)) - 0.055, step(0.0031308, rgb));
}

// Pure math: equal-area fisheye projection (matches C++ FisheyeEqualAreaForward).
// Input: direction components (dx, dy, dz) where dz is along pole axis.
// Output: normalized disc coordinates, r=1 at equator (r_scale=1.0).
vec2 fisheyeEAProject(float dx, float dy, float dz, float r_scale) {
  float k = r_scale / sqrt(1.0 + dz);
  return vec2(k * dx, k * dy);
}

// Layout + UV: convert normalized disc coords to texture UV.
// Matches C++ DualFisheyeToPixel convention (90-deg rotation + hemisphere mirroring).
vec2 dualFisheyeToUV(vec2 xy_norm, bool is_upper) {
  vec2 tex_res = vec2(textureSize(u_texture, 0));
  float short_res = min(tex_res.x * 0.5, tex_res.y);
  float R = short_res * 0.5;

  vec2 pixel;
  if (is_upper) {
    pixel = vec2(-xy_norm.y * R + tex_res.x * 0.5 - R,
                  xy_norm.x * R + tex_res.y * 0.5);
  } else {
    pixel = vec2( xy_norm.y * R + tex_res.x * 0.5 + R,
                  xy_norm.x * R + tex_res.y * 0.5);
  }
  return (pixel + 0.5) / tex_res;  // +0.5: align with OpenGL texel center convention
}

// Convert world direction to dual equal-area fisheye UV (single hemisphere, no blend).
vec2 dirToDualFisheye(vec3 d) {
  vec3 sky = -d;
  bool is_upper = (sky.z >= 0.0);
  float z_hemi = is_upper ? sky.z : -sky.z;
  vec2 xy_norm = fisheyeEAProject(sky.x, sky.y, z_hemi, u_r_scale);
  return dualFisheyeToUV(xy_norm, is_upper);
}

// Sample dual fisheye texture with overlap blending in the equator zone.
vec3 sampleDualFisheye(vec3 world_dir) {
  vec3 sky = -world_dir;
  float z_abs = abs(sky.z);
  bool is_upper = (sky.z >= 0.0);

  if (u_max_abs_dz > 0.0 && z_abs < u_max_abs_dz) {
    // Overlap zone: blend primary and secondary hemispheres.
    // Tent weight: t=0.5 at equator (z_abs=0), t=0 at boundary (z_abs=max_abs_dz).
    // primary weight = 1-t, secondary weight = t.
    float t = (u_max_abs_dz - z_abs) / (2.0 * u_max_abs_dz);

    // Primary: same hemisphere, z_hemi >= 0
    float z_hemi_pri = is_upper ? sky.z : -sky.z;
    vec2 xy_pri = fisheyeEAProject(sky.x, sky.y, z_hemi_pri, u_r_scale);
    vec2 uv_pri = dualFisheyeToUV(xy_pri, is_upper);
    // Secondary: opposite hemisphere, z_hemi < 0 (past equator)
    float z_hemi_opp = is_upper ? -sky.z : sky.z;  // = -|sky.z| < 0
    vec2 xy_sec = fisheyeEAProject(sky.x, sky.y, z_hemi_opp, u_r_scale);
    vec2 uv_sec = dualFisheyeToUV(xy_sec, !is_upper);

    vec3 c1 = texture(u_texture, uv_pri).rgb;
    vec3 c2 = texture(u_texture, uv_sec).rgb;
    return mix(c1, c2, t);
  }

  // Non-overlap: single hemisphere
  vec2 uv = dirToDualFisheye(world_dir);
  return texture(u_texture, uv).rgb;
}

// ============================ Relative illumination ==========================================
//
// The target lens's per-pixel solid angle, normalized to its on-axis value. The CLI bins rays into
// the lens it renders, so its pixels carry energy and the projection's Jacobian is baked in; this
// shader resamples an equal-area all-sky texture, whose texels carry radiance, so without this
// factor the preview is missing the projection's own natural vignetting. Every equal-area branch
// returns exactly 1, so equal-area previews are unchanged bit-for-bit.
//
// THIS IS THE LIVE COPY. src/gui/preview_jacobian.hpp mirrors it on the CPU and carries the full
// derivation, the reason the normalization is on-axis rather than against the source texture's
// texel solid angle, and the sources; test/unit-correctness/gui/test_preview_jacobian.cpp checks
// the mirror against numerical integration rather than against this file, so a transcription error
// made in both places still fails. Keep the two in lockstep.
const float kSingularityGuardPixels = 0.5;

float relIllumRectilinear(float rho, float focal) {
  float c = focal / sqrt(focal * focal + rho * rho);
  return c * c * c;  // cos^3(theta)
}

// type: 0=equal_area, 1=equidistant, 2=stereographic, 3=orthographic. r_norm is 1 at the image
// circle; img_radius is that circle in pixels and is read only by the orthographic clamp.
float relIllumFisheye(int type, float r_norm, float half_fov, float img_radius) {
  if (type == 0) {
    return 1.0;                      // equal area: constant by definition
  } else if (type == 1) {            // equidistant: sin(theta)/theta
    float theta = r_norm * half_fov;
    if (theta < 1e-3) return 1.0 - theta * theta / 6.0;  // series, not a division
    return sin(theta) / theta;
  } else if (type == 2) {            // stereographic: cos^4(theta/2)
    float t = r_norm * tan(half_fov * 0.5);
    float c = 1.0 / sqrt(1.0 + t * t);
    float c2 = c * c;
    return c2 * c2;
  }
  // orthographic: 1/cos(theta), held half a pixel inside the rim it diverges at.
  float r_max = img_radius > kSingularityGuardPixels ? 1.0 - kSingularityGuardPixels / img_radius : 0.0;
  float s = min(r_norm, r_max) * sin(half_fov);
  return 1.0 / sqrt(max(1.0 - s * s, 1e-12));
}

float relIllumEquirect(float lat) {
  return max(cos(lat), 0.0);
}

float relIllumGlobe(float rho, float focal) {
  // kGlobeCameraDist, repeated here for the same reason globeInverse repeats it.
  const float D = 4.0;
  float rho_limb = focal / sqrt(D * D - 1.0);
  float rho_max = max(rho_limb - kSingularityGuardPixels, 0.0);
  float k = min(rho, rho_max) / focal;
  float k2 = k * k;
  float disc = max(1.0 - k2 * (D * D - 1.0), 0.0);
  float mu = (D * k2 + sqrt(disc)) / (k2 + 1.0);
  float denom = max(D * mu - 1.0, 1e-12);
  float num = D - mu;
  return num * num * num / (denom * (D - 1.0) * (D - 1.0));
}

// Compute view direction from pixel for linear projection
// Returns false (via w component) if outside valid range
// `ri` returns the relative illumination at this pixel; see the block above.
vec4 linearInverse(vec2 pos, float half_fov, out float ri) {
  float short_edge = min(u_resolution.x, u_resolution.y);
  float focal = short_edge * 0.5 / tan(half_fov);
  ri = relIllumRectilinear(length(pos), focal);
  vec3 d = normalize(vec3(pos, -focal));
  return vec4(d, 1.0);
}

// Compute view direction for fisheye projections
// type: 0=equal_area, 1=equidistant, 2=stereographic, 3=orthographic
vec4 fisheyeInverse(vec2 pos, float half_fov, int type, out float ri) {
  float img_radius = min(u_resolution.x, u_resolution.y) * 0.5;  // short_edge/2 — matches Core's short_pix_/2
  float r = length(pos) / img_radius;
  ri = relIllumFisheye(type, r, half_fov, img_radius);

  float theta;
  if (type == 0) {        // equal area: r_norm = sin(θ/2) / sin(fov/4)
    float s = r * sin(half_fov * 0.5);
    if (s > 1.0) return vec4(0.0, 0.0, 0.0, 0.0);  // asin domain guard
    theta = 2.0 * asin(s);
  } else if (type == 1) { // equidistant: r_norm = θ / half_fov
    theta = r * half_fov;
    if (theta >= PI) return vec4(0.0, 0.0, 0.0, 0.0);
  } else if (type == 2) { // stereographic: r_norm = tan(θ/2) / tan(fov/4)
    theta = 2.0 * atan(r * tan(half_fov * 0.5));
  } else {                // orthographic (type == 3): r_norm = sin(θ) / sin(fov/2)
    float s = r * sin(half_fov);
    if (s > 1.0) return vec4(0.0, 0.0, 0.0, 0.0);  // asin domain guard
    theta = asin(s);
  }

  float phi = atan(pos.y, pos.x);
  return vec4(sin(theta) * cos(phi), sin(theta) * sin(phi), -cos(theta), 1.0);
}

// Dual fisheye: left circle = upper hemisphere, right circle = lower hemisphere
// Returns world-space direction (no view matrix needed) — matches Core convention.
// Core uses az = atan2(-d.y, -d.x) and pixel mapping with PI/2±az offset.
vec4 dualFisheyeInverse(vec2 pos, int type, out float ri) {
  float short_res = min(u_resolution.x * 0.5, u_resolution.y);
  float circle_radius = short_res * 0.5;
  ri = 1.0;  // set for real once the hemisphere, and so the local radius, is known

  // Left circle center at (-circle_radius, 0), right at (+circle_radius, 0)
  vec2 left_pos = pos - vec2(-circle_radius, 0.0);
  vec2 right_pos = pos - vec2(circle_radius, 0.0);

  float left_r = length(left_pos) / circle_radius;
  float right_r = length(right_pos) / circle_radius;

  bool in_left = left_r <= 1.0;
  bool in_right = right_r <= 1.0;
  if (!in_left && !in_right) return vec4(0.0, 0.0, 0.0, 0.0);

  vec2 use_pos = in_left ? left_pos : right_pos;
  float use_r = in_left ? left_r : right_r;
  // Each half of a dual fisheye images one hemisphere, so its half-FOV is fixed at pi/2 whatever
  // u_fov says, and the local radius is measured from that circle's own centre.
  ri = relIllumFisheye(type, use_r, PI * 0.5, circle_radius);

  float theta;
  float half_pi = PI * 0.5;
  if (type == 0) {        // equal area
    float s = use_r * sin(half_pi * 0.5);
    if (s > 1.0) return vec4(0.0, 0.0, 0.0, 0.0);
    theta = 2.0 * asin(s);
  } else if (type == 1) { // equidistant
    theta = use_r * half_pi;
  } else if (type == 2) { // stereographic
    theta = 2.0 * atan(use_r * tan(half_pi * 0.5));
  } else {                // orthographic (type == 3): dual fov is fixed 180°/hemi, sin(half_pi)=1
    // use_r is already normalised to [0, 1] via circle_radius division above.
    float s = use_r;
    if (s > 1.0) return vec4(0.0, 0.0, 0.0, 0.0);
    theta = asin(s);
  }

  // Inverse of Core's forward: pixel (x,y) uses cos(PI/2±az), sin(PI/2±az)
  // so phi_pixel = PI/2 + az (upper) or PI/2 - az (lower)
  float phi = atan(use_pos.y, use_pos.x);
  float st = sin(theta);
  vec3 d;
  if (in_left) {
    // Upper hemisphere: az = phi - PI/2
    // d = (-st*cos(az), -st*sin(az), -cos(theta))
    d = vec3(-st * sin(phi), st * cos(phi), -cos(theta));
  } else {
    // Lower hemisphere: az = PI/2 - phi
    // d = (-st*cos(az), -st*sin(az), cos(theta))
    d = vec3(-st * sin(phi), -st * cos(phi), cos(theta));
  }

  return vec4(d, 1.0);
}
)glsl"
R"glsl(
// Rectangular (equirectangular): always full-sky, returns world-space direction.
// Matches Core: scale = min(width/2, height) / PI
vec4 rectangularInverse(vec2 pos, out float ri) {
  float short_res = min(u_resolution.x * 0.5, u_resolution.y);
  float scale = short_res / PI;  // pixels per radian
  float lon = pos.x / scale;
  float lat = -pos.y / scale;
  ri = relIllumEquirect(lat);
  if (abs(lat) > PI * 0.5) return vec4(0.0, 0.0, 0.0, 0.0);

  vec3 d = vec3(-cos(lat) * cos(lon), -cos(lat) * sin(lon), -sin(lat));
  return vec4(d, 1.0);
}

// Globe (outside-in unit sphere): pinhole camera at (0,0,+D) in eye space looking
// toward -z; sphere of radius 1 sits at the origin. Returns the world-space unit
// vector pointing from the sphere center to the ray-sphere hit. needs_view_transform
// must be set false by the caller (u_view_matrix is applied here directly).
//
// kGlobeCameraDist = 4.0 chosen so default fov=30° gives sphere angular diameter
// ~28.96° (~96% of short edge), matching the crystal preview's zoom=1 distance scale.
// Treating hit_eye as a direction is valid because the sphere center coincides with
// the world origin (camera always looks at the sphere center); if that ever changes
// this function must switch to mat4 + explicit world-space ray-sphere intersection.
vec4 globeInverse(vec2 pos, float half_fov, out float ri) {
  // kGlobeCameraDist must match kGlobeCameraD in src/gui/gui_constants.hpp.
  const float kGlobeCameraDist = 4.0;
  float short_edge = min(u_resolution.x, u_resolution.y);
  float focal = short_edge * 0.5 / tan(half_fov);
  ri = relIllumGlobe(length(pos), focal);
  vec3 d = normalize(vec3(pos, -focal));  // ray dir in eye space

  // Solve |O + t*d|^2 = 1 with O = (0, 0, D):
  //   t^2 + 2*D*d.z*t + (D^2 - 1) = 0
  float b = kGlobeCameraDist * d.z;  // = dot(O, d)
  float c = kGlobeCameraDist * kGlobeCameraDist - 1.0;
  float disc = b * b - c;
  if (disc < 0.0) return vec4(0.0, 0.0, 0.0, 0.0);  // ray misses sphere
  float t = -b - sqrt(disc);                        // closest positive root (d.z < 0)
  if (t <= 0.0) return vec4(0.0, 0.0, 0.0, 0.0);    // hit behind camera

  vec3 hit_eye = vec3(0.0, 0.0, kGlobeCameraDist) + t * d;  // unit-length on sphere
  vec3 hit_world = u_view_matrix * hit_eye;
  return vec4(normalize(hit_world), 1.0);
}

// Overlay auxiliary lines on top of final_color.
// world_dir must be a valid world-space direction.
// pos_pix: pixel-space position of the current fragment, center-origin (0,0),
// y-up. Matches the CPU helper ProjectWorldDirToScreen for marker overlays.
vec3 overlayAuxLines(vec3 world_dir, vec3 color, vec2 pos_pix) {
  float DEG = 180.0 / PI;
  float altitude_deg = asin(clamp(-world_dir.z, -1.0, 1.0)) * DEG;
  float azimuth_deg = atan(-world_dir.y, -world_dir.x) * DEG;  // [-180, 180]

  // Adaptive line width via screen-space derivatives, clamped to avoid singularities
  float fw_alt = clamp(fwidth(altitude_deg), 1e-4, 2.0);
  float fw_az = clamp(fwidth(azimuth_deg), 1e-4, 5.0);

  // Coordinate grid (interval = u_grid_step, FOV-adaptive) — drawn first so
  // other lines overlay on top
  if (u_show_grid != 0) {
    float half_step = u_grid_step * 0.5;
    // Altitude grid lines
    float d_alt = mod(abs(altitude_deg) + half_step, u_grid_step) - half_step;
    float t_alt = 1.0 - smoothstep(0.0, fw_alt * 1.5, abs(d_alt));

    // Azimuth grid lines — suppress near poles to avoid fwidth divergence
    float t_az = 0.0;
    if (abs(altitude_deg) < 85.0) {
      float d_az = mod(azimuth_deg + 180.0 + half_step, u_grid_step) - half_step;
      t_az = 1.0 - smoothstep(0.0, fw_az * 1.5, abs(d_az));
    }

    float t = max(t_alt, t_az);
    color = mix(color, u_grid_color, t * u_grid_alpha);
  }

  // Sun angular distance circles. Geometry from core's mask; colour and alpha still this
  // consumer's own, which is the whole shape of the split — core says where, the drawer says how.
  //
  // uv.y is flipped because the two coordinate systems disagree about which end is the top: the
  // mask is row-major from the TOP-left (core's convention, shared with the CLI's image buffer)
  // while v_ndc.y is +1 at the top. The 0.5 in the x term and the flip together put the sample at
  // the CENTRE of the texel whose pixel centre core inverse-projected, so there is no half-pixel
  // shift between where the mask says the line is and where it is drawn.
  if (u_show_sun_circles != 0 && u_has_angular_dist_mask != 0) {
    vec2 mask_uv = vec2(v_ndc.x * 0.5 + 0.5, 0.5 - v_ndc.y * 0.5);
    // The mask stores 1, not 255, for a lit pixel; an R8 texture normalizes that to 1/255, which
    // as a blend weight is a line you cannot see. It is a BOOLEAN, so read it as one rather than
    // scaling it — any nonzero texel is on. (Sampled NEAREST, so there are no intermediate values
    // to lose by thresholding.)
    float t = texture(u_angular_dist_mask, mask_uv).r > 0.0 ? 1.0 : 0.0;
    color = mix(color, u_sun_circles_color, t * u_sun_circles_alpha);
  }

  // Horizon line (altitude = 0) — drawn last so it's most visible
  if (u_show_horizon != 0) {
    float d = abs(altitude_deg);
    float t = 1.0 - smoothstep(0.0, fw_alt * 1.5, d);
    color = mix(color, u_horizon_color, t * u_horizon_alpha);
  }

  // Zenith / Nadir pixel-space ring markers — drawn last so they sit on top
  // of all other overlays. CPU passes sentinel (-9999, -9999) when the marker
  // is offscreen / behind the camera; the distance test rejects it naturally.
  if (u_show_zenith_nadir != 0) {
    const float kRingHalfWidthPx = 1.5;
    float dz = length(pos_pix - u_zenith_screen_pos);
    float tz = 1.0 - smoothstep(0.0, kRingHalfWidthPx,
                                abs(dz - u_zenith_nadir_radius_px));
    color = mix(color, u_zenith_nadir_color, tz * u_zenith_nadir_alpha);
    float dn = length(pos_pix - u_nadir_screen_pos);
    float tn = 1.0 - smoothstep(0.0, kRingHalfWidthPx,
                                abs(dn - u_zenith_nadir_radius_px));
    color = mix(color, u_zenith_nadir_color, tn * u_zenith_nadir_alpha);
  }

  return color;
}

// Lens border: outline the projection's own valid image region — the locus where
// the active inverse function's domain guard flips w from 1 to 0. Only the fisheye
// family has one: the single-lens asin guards (equal-area / equidistant /
// orthographic) and dual fisheye's hard circle clip. linear / single-lens
// stereographic / rectangular / globe have no such circle and are skipped.
// Keep in lockstep with fisheyeInverse / dualFisheyeInverse above; the CPU-side
// classifier is LensHasBorder() in gui_constants.hpp.
//
// pos: center-origin, y-up pixel position (the raw `pos` of main(), NOT pos_ovl).
// The dual-fisheye y flip is irrelevant here because both border circles are
// centered on y = 0 and are therefore symmetric under it.
vec3 overlayLensBorder(vec3 color, vec2 pos, float half_fov) {
  if (u_show_lens_border == 0) return color;

  const float kBorderHalfWidthPx = 1.5;
  const float kMinSin = 1e-4;  // below this the boundary radius diverges: no border exists

  if ((u_lens_type >= 4 && u_lens_type <= 6) || u_lens_type == 9) {
    // Dual fisheye: two hard-clipped circles, identical for all four variants
    // (the clip lives in dualFisheyeInverse before the per-type theta branch).
    float short_res = min(u_resolution.x * 0.5, u_resolution.y);
    float circle_radius = short_res * 0.5;
    float dl = abs(length(pos - vec2(-circle_radius, 0.0)) - circle_radius);
    float tl = 1.0 - smoothstep(0.0, kBorderHalfWidthPx, dl);
    color = mix(color, u_lens_border_color, tl * u_lens_border_alpha);
    float dr = abs(length(pos - vec2(circle_radius, 0.0)) - circle_radius);
    float tr = 1.0 - smoothstep(0.0, kBorderHalfWidthPx, dr);
    color = mix(color, u_lens_border_color, tr * u_lens_border_alpha);
    return color;
  }

  // Single-lens fisheye family. r_boundary is in units of img_radius, read straight
  // off the corresponding guard in fisheyeInverse:
  //   equal area   (type 0): s = r * sin(half_fov/2) > 1  ->  r = 1 / sin(half_fov/2)
  //   equidistant  (type 1): theta = r * half_fov >= PI   ->  r = PI / half_fov
  //   orthographic (type 3): s = r * sin(half_fov) > 1    ->  r = 1 / sin(half_fov)
  float r_boundary = 0.0;
  if (u_lens_type == 1) {
    float sn = sin(half_fov * 0.5);
    if (sn < kMinSin) return color;
    r_boundary = 1.0 / sn;
  } else if (u_lens_type == 2) {
    if (half_fov < kMinSin) return color;
    r_boundary = PI / half_fov;
  } else if (u_lens_type == 8) {
    float sn = sin(half_fov);
    if (sn < kMinSin) return color;
    r_boundary = 1.0 / sn;
  } else {
    return color;  // 0 linear, 3 stereographic, 7 rectangular, 10 globe: no border
  }

  float img_radius = min(u_resolution.x, u_resolution.y) * 0.5;
  float radius_px = img_radius * r_boundary;
  float d = abs(length(pos) - radius_px);
  float t = 1.0 - smoothstep(0.0, kBorderHalfWidthPx, d);
  return mix(color, u_lens_border_color, t * u_lens_border_alpha);
}

out vec4 frag_color;

void main() {
  vec2 pos = v_ndc * u_resolution * 0.5;  // Convert NDC [-1,1] to pixel offset from center (y-up)
  float half_fov = u_fov * 0.5 * PI / 180.0;

  // The core-pixel-inverse family (dual fisheye 4-6/9, rectangular 7) inverts Core's
  // y-DOWN pixel layout (DualFisheyeToPixel / RectangularForward), but `pos` here is
  // y-UP (GL NDC). Feeding y-up pos straight in flips the display vertically vs the
  // CLI render. Flip pos.y for exactly that family so GUI matches CLI. The view-matrix
  // family (linear/fisheye/ortho/globe) is self-consistent and keeps raw pos. The same
  // flip is mirrored in overlay_labels.cpp (PixelToWorldDir / WorldDirToPixel) so grid
  // labels, zenith/nadir markers and mouse interaction follow the flipped content. See
  // scrum gui-lens-math-cli-alignment (owner: dual-fisheye/rectangular follow CLI).
  vec2 pos_ovl = pos;  // pixel pos handed to overlayAuxLines (flipped for the CPI family)

  vec4 result = vec4(0.0, 0.0, 0.0, 0.0);
  bool needs_view_transform = true;
  // The target lens's relative illumination at this pixel. Every branch below writes it; an
  // unknown u_lens_type leaves it at the identity, alongside the black result.w = 0 that case
  // already produces.
  float rel_illum = 1.0;
  if (u_lens_type == 0) {
    result = linearInverse(pos, half_fov, rel_illum);
  } else if (u_lens_type >= 1 && u_lens_type <= 3) {
    result = fisheyeInverse(pos, half_fov, u_lens_type - 1, rel_illum);
  } else if (u_lens_type >= 4 && u_lens_type <= 6) {
    pos_ovl = vec2(pos.x, -pos.y);
    result = dualFisheyeInverse(pos_ovl, u_lens_type - 4, rel_illum);
    needs_view_transform = false;  // Core dual fisheye works in world space
  } else if (u_lens_type == 7) {
    pos_ovl = vec2(pos.x, -pos.y);
    result = rectangularInverse(pos_ovl, rel_illum);
    needs_view_transform = false;  // Core rectangular works in world space
  } else if (u_lens_type == 8) {   // kLensTypeFisheyeOrthographic
    result = fisheyeInverse(pos, half_fov, 3, rel_illum);
  } else if (u_lens_type == 9) {   // kLensTypeDualFisheyeOrthographic
    pos_ovl = vec2(pos.x, -pos.y);
    result = dualFisheyeInverse(pos_ovl, 3, rel_illum);
    needs_view_transform = false;  // Core dual fisheye works in world space
  } else if (u_lens_type == 10) {  // kLensTypeGlobe
    result = globeInverse(pos, half_fov, rel_illum);
    needs_view_transform = false;  // globeInverse already returned world-space dir
  }
  // else: unknown u_lens_type → result.w = 0 (black). static_assert in gui_state.hpp
  // pins kLensTypeCount so any out-of-range value is a compile-time catchable mismatch.

  // Eliminated early returns so bg mixing can always execute at the end.
  vec3 final_color = vec3(0.0);
  vec3 world_dir = vec3(0.0);
  bool pixel_visible = false;

  if (result.w >= 0.5) {
    world_dir = needs_view_transform ? u_view_matrix * result.xyz : result.xyz;

    // Visible hemisphere check
    // In equirect convention: lat = asin(-dz), lat > 0 means upper sky
    float lat = asin(clamp(-world_dir.z, -1.0, 1.0));
    pixel_visible = true;
    if (u_visible == 0 && lat < 0.0) pixel_visible = false;   // upper: discard lower hemisphere
    if (u_visible == 1 && lat > 0.0) pixel_visible = false;   // lower: discard upper hemisphere
    // u_front: discard back hemisphere (AND with base). u_view_matrix[2] = -forward
    // (see BuildViewMatrix), so dot > 0 means world_dir is behind the camera.
    if (u_front == 1 && dot(world_dir, u_view_matrix[2]) > 0.0) pixel_visible = false;

    if (pixel_visible) {
      vec3 tex_color = sampleDualFisheye(world_dir);
      if (u_xyz_mode == 1) {
        // The projection's own natural vignetting, applied to the energy BEFORE the exposure scale
        // multiplies it: it states how much of this direction's radiance the target lens's pixel
        // collects, so it belongs to the quantity being exposed rather than to the exposure. See
        // the relIllum* block above and src/gui/preview_jacobian.hpp for the derivation.
        //
        // Only in XYZ mode, and that is a stated boundary rather than an oversight. u_xyz_mode == 0
        // is an 8-bit texture baked by LUMICE_XyzToSrgbUint8WithBackground, which has already
        // composited the sky colour into every texel. Vignetting it would dim the BACKGROUND too,
        // which neither the CLI nor the branch below does -- the sky joins after the exposure,
        // undimmed -- and the composite cannot be undone from this side. So a document loaded from
        // a .lmc shows a non-equal-area projection without its vignetting; equal-area ones are
        // unaffected either way, their factor being exactly 1.
        tex_color *= rel_illum;

        // The sky colour joins the halo here: inside this gate, and in linear RGB between the two
        // halves of the colour chain. Both placements are load-bearing.
        //
        // Inside the gate, because outside it nothing was ever projected — beyond the lens's image
        // circle, in the half-sky `visible` discards, behind the camera under `front`. Painting
        // those would turn a 180 deg fisheye into a solid rectangle of sky with an invisible circle
        // inside it. Between the halves, because the addition is a radiance sum: after the transfer
        // curve instead, a pixel with no halo energy comes back gamma-encoded twice and stops being
        // the colour the user picked.
        //
        // KNOWN, DELIBERATE DIVERGENCE FROM THE CLI, on single-lens fisheyes at wide FOV. The CLI's
        // projection is a forward one and a single lens images one hemisphere, so its background
        // stops at the equator (theta = 90 deg) whatever the FOV. This shader runs the same law
        // backwards from a pixel and stops only at that law's own domain, which for equal area at
        // fov=180 reaches theta = 180 deg — a radius of sqrt(2) image radii. It can, because it is
        // re-projecting an all-sky source texture rather than imaging a scene through one lens. So
        // between those two radii is an annulus this renderer paints and the CLI leaves black.
        // NEITHER SIDE IS A BUG. Do NOT narrow this gate to match the CLI: what a wide-FOV
        // single-lens preview should show is an open product question, not a defect, and answering
        // it here would answer it silently. test/gui/functional/test_preview_background.cpp pins
        // the annulus so that whenever it IS answered, it lands as a red.
        //
        // Only in XYZ mode. u_xyz_mode == 0 is a static 8-bit texture — a document loaded from
        // disk, whose pixels were already baked with whatever background was in effect when it was
        // saved (LUMICE_XyzToSrgbUint8WithBackground, app.cpp RefreshCpuTextureForSave). Adding
        // here too would apply it twice.
        tex_color = clampAndGamma(xyzToLinearRgb(tex_color) + u_background);
      }
      final_color = tex_color;
    }
  }

  // Background image overlay (contain mode with letterbox)
  if (u_bg_enabled != 0) {
    vec2 bg_uv = v_ndc * u_bg_uv_scale + u_bg_uv_offset;
    vec3 bg_color = vec3(0.0);  // Black letterbox for out-of-bounds UV
    if (bg_uv.x >= 0.0 && bg_uv.x <= 1.0 && bg_uv.y >= 0.0 && bg_uv.y <= 1.0) {
      bg_color = texture(u_bg_texture, bg_uv).rgb;
    }
    final_color = bg_color * (1.0 - u_overlay_alpha) + final_color * u_overlay_alpha;
  }

  // Auxiliary line overlay (on top of everything, only in visible region)
  if (result.w >= 0.5 && pixel_visible) {
    final_color = overlayAuxLines(world_dir, final_color, pos_ovl);
  }

  // Lens border — deliberately OUTSIDE the `result.w >= 0.5 && pixel_visible` gate.
  // It draws the lens image circle itself, an optical property of the projection,
  // so it must stay whole regardless of which half-sky the user chose to display
  // (u_visible / u_front) and regardless of whether the pixel maps to a direction
  // at all. That is exactly the black region users cannot tell from the background.
  final_color = overlayLensBorder(final_color, pos, half_fov);

  frag_color = vec4(final_color, 1.0);
}
)glsl";
// clang-format on

static unsigned int CompileShader(unsigned int type, const char* source) {
  unsigned int shader = glCreateShader(type);
  glShaderSource(shader, 1, &source, nullptr);
  glCompileShader(shader);

  int success;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
  if (!success) {
    char log[512];
    glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
    GUI_LOG_ERROR("Shader compile error: {}", log);
    glDeleteShader(shader);
    return 0;
  }
  return shader;
}

bool PreviewRenderer::Init() {
  // Compile shaders
  unsigned int vs = CompileShader(GL_VERTEX_SHADER, kVertexShader);
  unsigned int fs = CompileShader(GL_FRAGMENT_SHADER, kFragmentShader);
  if (!vs || !fs) {
    return false;
  }

  shader_program_ = glCreateProgram();
  glAttachShader(shader_program_, vs);
  glAttachShader(shader_program_, fs);
  glLinkProgram(shader_program_);

  int success;
  glGetProgramiv(shader_program_, GL_LINK_STATUS, &success);
  if (!success) {
    char log[512];
    glGetProgramInfoLog(shader_program_, sizeof(log), nullptr, log);
    GUI_LOG_ERROR("Shader link error: {}", log);
    return false;
  }

  glDeleteShader(vs);
  glDeleteShader(fs);

  // Fullscreen quad (two triangles, NDC coordinates)
  // clang-format off
  float quad_vertices[] = {
    -1.0f, -1.0f,
     1.0f, -1.0f,
    -1.0f,  1.0f,
     1.0f, -1.0f,
     1.0f,  1.0f,
    -1.0f,  1.0f,
  };
  // clang-format on

  glGenVertexArrays(1, &vao_);
  glGenBuffers(1, &vbo_);

  glBindVertexArray(vao_);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glBufferData(GL_ARRAY_BUFFER, sizeof(quad_vertices), quad_vertices, GL_STATIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), nullptr);
  glBindVertexArray(0);

  // Create equirect texture
  glGenTextures(1, &texture_);
  glBindTexture(GL_TEXTURE_2D, texture_);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);  // dual fisheye: no horizontal wrap
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glBindTexture(GL_TEXTURE_2D, 0);
  // Initialize with 1x1 black pixel so the texture has valid GL storage.
  // This allows the shader to sample black (transparent) when no simulation data exists,
  // enabling background-only rendering before the simulation is started.
  // Shared with Render()'s needs_gl_blank_ consumer so a single code path owns
  // "sim layer reset to black" (both init and post-ClearTexture reuse the same
  // GL sequence + xyz_mode_ = false side effect).
  UploadBlankSimTexture();

  // Create background texture.
  //
  // The background is a photograph, and it is nearly always larger than the viewport it is fitted
  // into: a 6000x4000 frame in a ~900px preview panel is a 4x reduction, and the loader only halves
  // images past 4096 on a side, so the texture reaching this point is still several times the
  // viewport. GL_LINEAR averages 2x2 texels however large the footprint under a fragment is, so at
  // that reduction it reads 4 texels of a 16-texel block and calls the result the block's colour —
  // undersampling, which turns photographic grain into speckle that reshuffles itself whenever the
  // view moves. GL_LINEAR_MIPMAP_LINEAR reads a mip level chosen for the actual footprint instead,
  // where the averaging has already been done. MAG stays GL_LINEAR: magnification has no footprint
  // to cover and no mip level above 0 to reach for.
  glGenTextures(1, &bg_texture_);
  glBindTexture(GL_TEXTURE_2D, bg_texture_);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glBindTexture(GL_TEXTURE_2D, 0);

  return true;
}

void PreviewRenderer::Destroy() {
  for (int i = 0; i < 2; ++i) {
    if (pbo_fence_[i] != nullptr) {
      glDeleteSync(static_cast<GLsync>(pbo_fence_[i]));
      pbo_fence_[i] = nullptr;
    }
  }
  for (int i = 0; i < 2; ++i) {
    if (pbo_[i] != 0) {
      glDeleteBuffers(1, &pbo_[i]);
      pbo_[i] = 0;
    }
  }
  pbo_byte_count_ = { 0, 0 };
  pbo_index_ = 0;

  if (texture_) {
    glDeleteTextures(1, &texture_);
    texture_ = 0;
  }
  if (bg_texture_) {
    glDeleteTextures(1, &bg_texture_);
    bg_texture_ = 0;
  }
  if (angular_dist_tex_) {
    glDeleteTextures(1, &angular_dist_tex_);
    angular_dist_tex_ = 0;
    angular_dist_tex_generation_ = 0;
  }
  if (vbo_) {
    glDeleteBuffers(1, &vbo_);
    vbo_ = 0;
  }
  if (vao_) {
    glDeleteVertexArrays(1, &vao_);
    vao_ = 0;
  }
  if (shader_program_) {
    glDeleteProgram(shader_program_);
    shader_program_ = 0;
  }
  tex_width_ = 0;
  tex_height_ = 0;
  tex_data_.clear();
  bg_width_ = 0;
  bg_height_ = 0;
  bg_aspect_ = 1.0f;
}

void PreviewRenderer::ClearTexture() {
  tex_width_ = 0;
  tex_height_ = 0;
  tex_data_.clear();
  // GL reset deferred to next Render() (main thread) — this method is called
  // from gui_test coroutine workers with no GL context; a direct gl* call
  // would SIGILL. See preview_renderer.hpp needs_gl_blank_ contract.
  needs_gl_blank_ = true;
}

// Upload a 1x1 black pixel into the GL texture_ storage and reset xyz_mode_.
// Reused by Init() (initial GL storage) and Render() (post-ClearTexture
// deferred reset). Must be called on the main thread (owns GL context).
// Does NOT touch tex_width_/tex_height_ — those track "does the app have
// real sim/loaded pixel data" (drives HasTexture()) and must stay 0 across
// this call so ClearTexture()'s CPU-side "cleared" state remains visible to
// callers like Save. Side effect: xyz_mode_ = false, because a 1x1 RGB uint8
// pixel is not XYZ float data — leaving xyz_mode_ = true would mislead the
// shader on the next real upload path.
void PreviewRenderer::UploadBlankSimTexture() {
  static const unsigned char kBlack[3] = { 0, 0, 0 };
  glBindTexture(GL_TEXTURE_2D, texture_);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 1, 1, 0, GL_RGB, GL_UNSIGNED_BYTE, kBlack);
  glBindTexture(GL_TEXTURE_2D, 0);
  xyz_mode_ = false;
}

void PreviewRenderer::UpdateCpuTextureData(const unsigned char* data, int width, int height) {
  if (!data || width <= 0 || height <= 0) {
    return;
  }
  size_t byte_count = static_cast<size_t>(width) * height * 3;
  tex_data_.assign(data, data + byte_count);
  tex_width_ = width;
  tex_height_ = height;
}

void PreviewRenderer::UploadTexture(const unsigned char* data, int width, int height) {
  if (!texture_ || !data) {
    return;
  }
  // Fresh real pixel data is about to land in texture_; make the newest write
  // win over any pending deferred blank so a Clear→Upload sequence (with no
  // intervening Render) doesn't get overwritten with black on the next frame.
  needs_gl_blank_ = false;

  // Keep CPU-side copy for .lmc file save
  size_t byte_count = static_cast<size_t>(width) * height * 3;
  tex_data_.assign(data, data + byte_count);

  glBindTexture(GL_TEXTURE_2D, texture_);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);  // RGB data may not be 4-byte aligned

  if (width != tex_width_ || height != tex_height_ || xyz_mode_) {
    // Re-allocate texture when switching from float to uint8 or size changed
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
    tex_width_ = width;
    tex_height_ = height;
  } else {
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, data);
  }

  xyz_mode_ = false;
  glBindTexture(GL_TEXTURE_2D, 0);
}

void PreviewRenderer::UploadXyzTexture(const float* data, int width, int height) {
  if (!texture_ || !data || width <= 0 || height <= 0) {
    return;
  }
  // Fresh real pixel data is about to land in texture_; make the newest write
  // win over any pending deferred blank (see UploadTexture for rationale).
  needs_gl_blank_ = false;

  // Do NOT update tex_data_ (CPU copy) — XYZ float data is not suitable for .lmc save.
  size_t byte_count = static_cast<size_t>(width) * height * 3 * sizeof(float);
  int w = pbo_index_;

  // Wait for prior fence (ensure GPU finished reading PBO[w])
  if (pbo_fence_[w] != nullptr) {
    auto sync = static_cast<GLsync>(pbo_fence_[w]);
    GLenum wait_result = glClientWaitSync(sync, GL_SYNC_FLUSH_COMMANDS_BIT, 2'000'000'000ULL);
    glDeleteSync(sync);
    pbo_fence_[w] = nullptr;
    if (wait_result == GL_TIMEOUT_EXPIRED || wait_result == GL_WAIT_FAILED) {
      GUI_LOG_WARNING("[GL] UploadXyzTexture: fence wait failed (result={}), skipping frame", wait_result);
      return;
    }
  }

  // Lazy-init + resize PBO
  if (pbo_[w] == 0) {
    glGenBuffers(1, &pbo_[w]);
  }
  glBindBuffer(GL_PIXEL_UNPACK_BUFFER, pbo_[w]);
  if (pbo_byte_count_[w] != byte_count) {
    glBufferData(GL_PIXEL_UNPACK_BUFFER, static_cast<GLsizeiptr>(byte_count), nullptr, GL_STREAM_DRAW);
    pbo_byte_count_[w] = byte_count;
  }

  // Map PBO and write data
  void* ptr = glMapBufferRange(GL_PIXEL_UNPACK_BUFFER, 0, static_cast<GLsizeiptr>(byte_count),
                               GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT | GL_MAP_UNSYNCHRONIZED_BIT);
  if (!ptr) {
    GUI_LOG_ERROR("[GL] UploadXyzTexture: glMapBufferRange failed");
    glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
    return;
  }
  std::memcpy(ptr, data, byte_count);
  if (glUnmapBuffer(GL_PIXEL_UNPACK_BUFFER) == GL_FALSE) {
    GUI_LOG_WARNING("[GL] UploadXyzTexture: glUnmapBuffer failed, buffer data may be corrupt, skipping frame");
    glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
    return;
  }

  // Upload from PBO to texture (async DMA)
  glBindTexture(GL_TEXTURE_2D, texture_);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
  if (width != tex_width_ || height != tex_height_ || !xyz_mode_) {
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, width, height, 0, GL_RGB, GL_FLOAT, nullptr);
    tex_width_ = width;
    tex_height_ = height;
  } else {
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_RGB, GL_FLOAT, nullptr);
  }

  GLenum err = glGetError();
  if (err != GL_NO_ERROR) {
    GUI_LOG_WARNING("[GL] UploadXyzTexture: glGetError={} after {}x{} upload", err, width, height);
  }
  pbo_fence_[w] = static_cast<void*>(glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0));
  if (!pbo_fence_[w]) {
    GUI_LOG_WARNING("[GL] UploadXyzTexture: glFenceSync failed, next write to slot {} will be unguarded", w);
  }
  glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
  xyz_mode_ = true;
  glBindTexture(GL_TEXTURE_2D, 0);
  pbo_index_ = 1 - pbo_index_;
}

void PreviewRenderer::UploadBgTexture(const unsigned char* data, int width, int height) {
  if (!bg_texture_ || !data || width <= 0 || height <= 0) {
    return;
  }

  glBindTexture(GL_TEXTURE_2D, bg_texture_);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);  // RGB data may not be 4-byte aligned

  if (width != bg_width_ || height != bg_height_) {
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
    bg_width_ = width;
    bg_height_ = height;
  } else {
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, data);
  }

  // After the branches meet, so that both are covered. The MIN filter set in Init() samples mip
  // levels, and only level 0 was written above; without this the smaller levels hold whatever the
  // previous image left there. That is a live hazard on the glTexSubImage2D branch in particular:
  // it overwrites the pixels in place and leaves the rest of the chain untouched, so a user
  // swapping one photograph for another of the same size would keep seeing the old one wherever the
  // fit minifies.
  glGenerateMipmap(GL_TEXTURE_2D);

  bg_aspect_ = static_cast<float>(width) / static_cast<float>(height);
  glBindTexture(GL_TEXTURE_2D, 0);
}

void PreviewRenderer::UploadAngularDistMask(const unsigned char* data, int width, int height) {
  if (data == nullptr || width <= 0 || height <= 0) {
    ClearAngularDistMask();
    return;
  }
  if (angular_dist_tex_ == 0) {
    glGenTextures(1, &angular_dist_tex_);
    if (angular_dist_tex_ == 0) {
      return;
    }
  }
  glBindTexture(GL_TEXTURE_2D, angular_dist_tex_);
  // NEAREST, and it has to be: the mask is a per-pixel yes/no, and filtering it would smear the
  // line across neighbours the mask generator deliberately left out. CLAMP_TO_EDGE for the same
  // reason a wrap would be wrong — a circle must not reappear on the opposite edge.
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  // One byte per texel with no row padding: the default GL_UNPACK_ALIGNMENT of 4 would misread
  // every row of a width that is not a multiple of 4, which most viewport widths are not.
  GLint prev_alignment = 4;
  glGetIntegerv(GL_UNPACK_ALIGNMENT, &prev_alignment);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, width, height, 0, GL_RED, GL_UNSIGNED_BYTE, data);
  glPixelStorei(GL_UNPACK_ALIGNMENT, prev_alignment);
  glBindTexture(GL_TEXTURE_2D, 0);
}

void PreviewRenderer::ClearAngularDistMask() {
  if (angular_dist_tex_ != 0) {
    glDeleteTextures(1, &angular_dist_tex_);
    angular_dist_tex_ = 0;
  }
}

void PreviewRenderer::ClearBackground() {
  bg_width_ = 0;
  bg_height_ = 0;
  bg_aspect_ = 1.0f;
}

// Build view-to-world rotation matrix from elevation, azimuth, roll (degrees).
//
// Convention: at (az=0, el=0, roll=0), the camera looks at the equirect center,
// which is world direction (-1, 0, 0).  Increasing azimuth rotates the camera
// rightward in the equirect (toward increasing longitude / -Y direction).
// Increasing elevation tilts the camera upward (toward upper sky / -Z direction).
//
// Camera basis at (az=a, el=e, roll=0):
//   forward = (-cos(e)*cos(a), -cos(e)*sin(a), -sin(e))   [equirect az=a, el=e]
//   right   = (sin(a), -cos(a), 0)                         [horizontal, toward +lon]
//   up      = (sin(e)*cos(a), sin(e)*sin(a), -cos(e))      [completes RH basis]
//
// Roll rotates right and up around the forward axis:
//   col0' = cos(r)*right + sin(r)*up
//   col1' = -sin(r)*right + cos(r)*up
//   col2  = -forward  (unchanged by roll)
//
// OpenGL column-major: out[col*3 + row].
void BuildViewMatrix(float elevation_deg, float azimuth_deg, float roll_deg, float out[9]) {
  constexpr float kDeg2Rad = 3.14159265358979323846f / 180.0f;
  float a = azimuth_deg * kDeg2Rad;
  float e = elevation_deg * kDeg2Rad;
  float r = roll_deg * kDeg2Rad;

  float ca = std::cos(a), sa = std::sin(a);
  float ce = std::cos(e), se = std::sin(e);
  float cr = std::cos(r), sr = std::sin(r);

  // Column 0 = cos(r)*right + sin(r)*up
  out[0] = cr * sa + sr * se * ca;
  out[1] = -cr * ca + sr * se * sa;
  out[2] = -sr * ce;

  // Column 1 = -sin(r)*right + cos(r)*up
  out[3] = -sr * sa + cr * se * ca;
  out[4] = sr * ca + cr * se * sa;
  out[5] = -cr * ce;

  // Column 2 = -forward = (cos(e)*cos(a), cos(e)*sin(a), sin(e))
  out[6] = ce * ca;
  out[7] = ce * sa;
  out[8] = se;
}

// Sentinel value indicating the world direction does not project to a renderable
// screen position under the current lens / camera setup. Kept identical to the
// magic shader value so the GPU distance test trivially rejects it.
static constexpr std::array<float, 2> kProjectSentinel = { kOverlaySentinel, kOverlaySentinel };

static bool IsInViewport(float px, float py, float vp_w, float vp_h) {
  // Half-pixel margin keeps points landing exactly on the viewport edge
  // (e.g. fisheye r = img_radius for the equator) inside the visible region
  // — the shader naturally clips anything truly outside.
  constexpr float kEdgeMargin = 0.5f;
  return std::abs(px) <= vp_w * 0.5f + kEdgeMargin && std::abs(py) <= vp_h * 0.5f + kEdgeMargin;
}

// Apply transpose(view_matrix) * world_dir. Column-major view-to-world.
static void WorldToView(const ViewProjection& vp, const float world_dir[3], float out_view[3]) {
  float vm[9];
  BuildViewMatrix(vp.elevation, vp.azimuth, vp.roll, vm);
  for (int c = 0; c < 3; ++c) {
    out_view[c] = vm[c * 3 + 0] * world_dir[0] + vm[c * 3 + 1] * world_dir[1] + vm[c * 3 + 2] * world_dir[2];
  }
}

// Small epsilon absorbs single-precision noise from BuildViewMatrix (e.g.
// cos(π/2) ≈ -4.37e-8 leaks into view_dir.z when the camera is tilted to
// elevation 90°), so directions right on the horizon still classify as
// "in front" instead of being rejected by the behind-camera guard.
constexpr float kBehindCameraEps = 1e-5f;

// Linear pinhole: behind-camera ⇒ sentinel; otherwise standard perspective divide.
// SYNC: ComputeDragGainDegPerPixel's kLensTypeLinear branch is this formula's
// hand-derived d(theta)/d(r) at theta=0 — changing r(theta) here needs that branch rechecked.
static std::array<float, 2> ProjectLinear(const float view_dir[3], float half_fov, float img_radius) {
  if (view_dir[2] >= -kBehindCameraEps) {
    return kProjectSentinel;
  }
  float focal = img_radius / std::tan(half_fov);
  float px = focal * view_dir[0] / (-view_dir[2]);
  float py = focal * view_dir[1] / (-view_dir[2]);
  return { px, py };
}

// Forward equation for the single-hemisphere fisheye family (lens=1..3, 8).
// fisheye_type matches the shader's `fisheyeInverse(... int type)` switch:
// 0=equal_area, 1=equidistant, 2=stereographic, 3=orthographic.
// SYNC: ComputeDragGainDegPerPixel's four single-fisheye branches are these
// r_norm(theta) laws' hand-derived d(theta)/d(r) at theta=0 — changing any of them
// here needs the matching branch rechecked.
static std::array<float, 2> ProjectFisheye(const float view_dir[3], float half_fov, float img_radius,
                                           int fisheye_type) {
  if (view_dir[2] > kBehindCameraEps) {
    return kProjectSentinel;
  }
  float theta = std::acos(std::clamp(-view_dir[2], -1.0f, 1.0f));
  float r_norm = 0.0f;
  if (fisheye_type == 0) {
    float denom = std::sin(half_fov * 0.5f);
    if (denom <= 0.0f) {
      return kProjectSentinel;
    }
    r_norm = std::sin(theta * 0.5f) / denom;
  } else if (fisheye_type == 1) {
    if (half_fov <= 0.0f) {
      return kProjectSentinel;
    }
    r_norm = theta / half_fov;
  } else if (fisheye_type == 2) {
    float denom = std::tan(half_fov * 0.5f);
    if (denom <= 0.0f) {
      return kProjectSentinel;
    }
    r_norm = std::tan(theta * 0.5f) / denom;
  } else {  // orthographic
    float denom = std::sin(half_fov);
    if (denom <= 0.0f) {
      return kProjectSentinel;
    }
    r_norm = std::sin(theta) / denom;
  }
  // Circular viewport clip: reject points outside the imaging circle. For
  // FOV<180° fisheye in non-square viewports the rectangular IsInViewport
  // check downstream is not sufficient — directions past the imaging disc
  // would otherwise leak into the black-bar region. 0.5/img_radius converts
  // IsInViewport's 0.5px edge margin into normalized-radius units so the
  // boundary is handled consistently.
  if (r_norm > 1.0f + 0.5f / img_radius) {
    return kProjectSentinel;
  }
  float r = r_norm * img_radius;
  float phi = std::atan2(view_dir[1], view_dir[0]);
  return { r * std::cos(phi), r * std::sin(phi) };
}

// Compute r_norm for a dual-fisheye projection (theta in [0, pi/2]).
// fisheye_type semantics match ProjectFisheye but the field of view is fixed
// at half_pi per hemisphere (matches shader dualFisheyeInverse()).
static float DualFisheyeRNorm(float theta, int fisheye_type) {
  constexpr float kHalfPi = 1.57079632679489661923f;
  if (fisheye_type == 0) {
    return std::sin(theta * 0.5f) / std::sin(kHalfPi * 0.5f);
  }
  if (fisheye_type == 1) {
    return theta / kHalfPi;
  }
  if (fisheye_type == 2) {
    return std::tan(theta * 0.5f) / std::tan(kHalfPi * 0.5f);
  }
  // orthographic: r_norm = sin(theta)
  return std::sin(theta);
}

// Forward for dual-fisheye family (lens=4..6, 9). world_dir.z<0 ⇒ left (upper)
// circle, world_dir.z>0 ⇒ right (lower) circle. The phi solve mirrors the
// hemisphere case split in shader dualFisheyeInverse().
static std::array<float, 2> ProjectDualFisheye(const float world_dir[3], float short_res_dual, int fisheye_type) {
  float circle_radius = short_res_dual * 0.5f;
  bool is_upper = world_dir[2] < 0.0f;
  float cx = is_upper ? -circle_radius : circle_radius;
  float z_abs = std::abs(world_dir[2]);
  float theta = std::acos(std::clamp(z_abs, -1.0f, 1.0f));
  float r = DualFisheyeRNorm(theta, fisheye_type) * circle_radius;
  // theta=0 (zenith / nadir) ⇒ sin(theta)=0, phi is irrelevant and the marker
  // lands exactly at the circle center — the common case for this helper's caller.
  float phi = 0.0f;
  float sin_theta = std::sin(theta);
  if (sin_theta > 1e-6f) {
    // Shader: upper d = (-st*sin(phi),  st*cos(phi), -ct)
    //         lower d = (-st*sin(phi), -st*cos(phi), +ct)
    float sx = -world_dir[0] / sin_theta;                              // = sin(phi)
    float cy = (is_upper ? world_dir[1] : -world_dir[1]) / sin_theta;  // = cos(phi)
    phi = std::atan2(sx, cy);
  }
  return { cx + r * std::cos(phi), r * std::sin(phi) };
}

// Equirectangular forward: inverse of shader rectangularInverse().
// At the poles (|lat|=π/2) world_dir.xy ≈ 0 and atan2 is singular — anchor
// lon=0 so zenith/nadir project to the column directly in front of the
// camera (mid-column of the viewport), not to the φ=±π edge.
static std::array<float, 2> ProjectRectangular(const float world_dir[3], float short_res_dual) {
  constexpr float kPi = 3.14159265358979323846f;
  constexpr float kPoleEps = 1e-6f;
  float scale = short_res_dual / kPi;
  float lat = std::asin(std::clamp(-world_dir[2], -1.0f, 1.0f));
  float xy_norm_sq = world_dir[0] * world_dir[0] + world_dir[1] * world_dir[1];
  float lon = (xy_norm_sq < kPoleEps) ? 0.0f : std::atan2(-world_dir[1], -world_dir[0]);
  return { lon * scale, -lat * scale };
}

// Forward projection for globe lens. eye_dir is the world-direction transformed
// to eye space via WorldToView (= the world point on the unit sphere expressed
// in eye coordinates). The camera sits at O = (0, 0, kGlobeCameraD); the front
// hemisphere visible to the camera satisfies eye_dir.z > 1/kGlobeCameraD.
// Math is line-for-line equivalent to overlay_labels.cpp::WorldDirToPixel's
// globe branch — kGlobeCameraD is the single source of truth (gui_constants.hpp).
// SYNC: ComputeDragGainDegPerPixel's kLensTypeGlobe branch is this formula's
// hand-derived d(theta)/d(r) at theta=0 — changing the camera model here needs that
// branch rechecked.
static std::array<float, 2> ProjectGlobe(const float eye_dir[3], float half_fov, float img_radius) {
  if (eye_dir[2] <= 1.0f / kGlobeCameraD) {
    return kProjectSentinel;
  }
  float focal = img_radius / std::tan(half_fov);
  float denom = kGlobeCameraD - eye_dir[2];
  return { eye_dir[0] / denom * focal, eye_dir[1] / denom * focal };
}

// See declaration in preview_renderer.hpp for contract.
// NOTE: must be updated when adding a new kLensType* constant.
std::array<float, 2> ProjectWorldDirToScreen(const ViewProjection& vp, const float world_dir[3], int vp_w, int vp_h) {
  constexpr float kPi = 3.14159265358979323846f;
  if (vp_w <= 0 || vp_h <= 0) {
    return kProjectSentinel;
  }

  float half_fov = vp.fov * 0.5f * kPi / 180.0f;
  auto vp_w_f = static_cast<float>(vp_w);
  auto vp_h_f = static_cast<float>(vp_h);
  float img_radius = std::min(vp_w_f, vp_h_f) * 0.5f;
  float short_res_dual = std::min(vp_w_f * 0.5f, vp_h_f);

  int lt = vp.lens_type;
  // Single source of truth for the "needs view transform" classification:
  // !LensIsFullSky covers linear, single fisheye family (incl. orthographic)
  // and globe, mirroring overlay_labels.cpp::WorldDirToPixel.
  bool needs_view_transform = !LensIsFullSky(lt);

  float local_dir[3];
  if (needs_view_transform) {
    WorldToView(vp, world_dir, local_dir);
  } else {
    local_dir[0] = world_dir[0];
    local_dir[1] = world_dir[1];
    local_dir[2] = world_dir[2];
  }

  std::array<float, 2> out = kProjectSentinel;
  if (lt == kLensTypeLinear) {
    out = ProjectLinear(local_dir, half_fov, img_radius);
  } else if (lt == kLensTypeFisheyeEqualArea) {
    out = ProjectFisheye(local_dir, half_fov, img_radius, 0);
  } else if (lt == kLensTypeFisheyeEquidist) {
    out = ProjectFisheye(local_dir, half_fov, img_radius, 1);
  } else if (lt == kLensTypeFisheyeStereographic) {
    out = ProjectFisheye(local_dir, half_fov, img_radius, 2);
  } else if (lt == kLensTypeFisheyeOrthographic) {
    out = ProjectFisheye(local_dir, half_fov, img_radius, 3);
  } else if (lt == kLensTypeDualFisheyeEqualArea) {
    out = ProjectDualFisheye(local_dir, short_res_dual, 0);
  } else if (lt == kLensTypeDualFisheyeEquidist) {
    out = ProjectDualFisheye(local_dir, short_res_dual, 1);
  } else if (lt == kLensTypeDualFisheyeStereographic) {
    out = ProjectDualFisheye(local_dir, short_res_dual, 2);
  } else if (lt == kLensTypeDualFisheyeOrthographic) {
    out = ProjectDualFisheye(local_dir, short_res_dual, 3);
  } else if (lt == kLensTypeRectangular) {
    out = ProjectRectangular(local_dir, short_res_dual);
  } else if (lt == kLensTypeGlobe) {
    // local_dir is the WorldToView-transformed direction (needs_view_transform
    // = true for globe via !LensIsFullSky), i.e. the eye_dir expected by
    // ProjectGlobe.
    out = ProjectGlobe(local_dir, half_fov, img_radius);
  } else {
    assert(false && "ProjectWorldDirToScreen: unhandled lens type");
    return kProjectSentinel;
  }

  if (out == kProjectSentinel || !IsInViewport(out[0], out[1], vp_w_f, vp_h_f)) {
    return kProjectSentinel;
  }
  return out;
}

// See declaration in preview_renderer.hpp for contract.
//
// Each branch is the reciprocal of the corresponding forward projection's radial
// derivative dr/dtheta evaluated at theta = 0 (the frame center), i.e. it answers
// "one pixel of screen radius is how many radians of view angle?". img_radius is
// min(vp_w, vp_h)/2 — the same definition ProjectWorldDirToScreen uses, so the gain
// and the projection that consumes it share one pixel/DPI convention and window
// resize needs no extra handling. The table below is that 1:1 law; the returned
// value is it scaled by kDragSensitivity (see the constant for why).
//
//   lens                        forward r(theta)                       dtheta/dr at 0
//   --------------------------  -------------------------------------  ---------------------------
//   kLensTypeLinear             r = focal*tan(theta),                   tan(hf) / R
//                               focal = R/tan(hf)          (ProjectLinear)
//   kLensTypeFisheyeEqualArea   r/R = sin(theta/2)/sin(hf/2)   (ProjectFisheye, type 0)
//                                                                       2*sin(hf/2) / R
//   kLensTypeFisheyeEquidist    r/R = theta/hf                 (ProjectFisheye, type 1)
//                                                                       hf / R   (exact for all theta)
//   kLensTypeFisheyeStereogr.   r/R = tan(theta/2)/tan(hf/2)   (ProjectFisheye, type 2)
//                                                                       2*tan(hf/2) / R
//   kLensTypeFisheyeOrthogr.    r/R = sin(theta)/sin(hf)       (ProjectFisheye, type 3)
//                                                                       sin(hf) / R
//   kLensTypeGlobe              r = focal*x_eye/(D - z_eye)    (ProjectGlobe)
//                                                                       (D-1)*tan(hf) / R
//   (hf = half_fov in radians, R = img_radius, D = kGlobeCameraD)
//
// LUMICE_MaxFov() keeps every expression finite over each lens's legal FOV range
// (linear caps at 179°, short of tan's pole at 180°; globe at 90°), so no extra
// numerical guard is needed beyond the degenerate-viewport check.
// bg_uv = v_ndc * scale + offset maps NDC [-1,1] to texture UV [0,1] with a contain fit
// (letterbox on aspect mismatch), then the user's pan/zoom on top. See the header for the
// conventions; the identity case is asserted bit-for-bit in test_preview_renderer.cpp.
BgUvTransform ComputeBgUvTransform(int vp_w, int vp_h, float bg_aspect, float pan_x, float pan_y, float zoom) {
  float vp_aspect = static_cast<float>(vp_w) / static_cast<float>(vp_h);
  float sx = 1.0f;
  float sy = 1.0f;
  if (vp_aspect > bg_aspect) {
    sx = bg_aspect / vp_aspect;
  } else {
    sy = vp_aspect / bg_aspect;
  }
  BgUvTransform t{};
  t.scale_x = 0.5f / (sx * zoom);
  // Y flip: stbi loads top-down, GL texture origin is bottom-left
  t.scale_y = -0.5f / (sy * zoom);
  t.offset_x = 0.5f + pan_x;
  t.offset_y = 0.5f + pan_y;
  return t;
}

float ComputeDragGainDegPerPixel(int lens_type, float fov_deg, int vp_w, int vp_h) {
  constexpr float kPi = 3.14159265358979323846f;
  // Pre-fov-aware sensitivity. Only reachable via the default branch below, which
  // no caller should hit — app_panels.cpp only drags when !LensIsFullSky. It is the
  // historical feel value itself, so kDragSensitivity is deliberately NOT applied to it.
  constexpr float kLegacyGainDegPerPixel = 0.3f;

  // How many screen pixels the content moves per dragged pixel. The closed forms below
  // are the 1:1 law (one pixel of mouse motion, one pixel of content motion at the frame
  // center); this multiplier re-anchors that law without touching its shape.
  //
  // 1:1 is a hard constraint on a touchscreen — the finger is physically on the content —
  // but on a mouse it is only one candidate anchor, and it lost the owner's drag test:
  // measured against the historical 0.3 deg/px on a 900 px short side, the 1:1 law is 2.4x
  // slower at fov=90°, 4.1x at 60°, 8.8x at 30°, and only overtakes it above fov=134° —
  // i.e. slower everywhere in daily use. 2.5 puts linear at fov=90°/900 px at 0.318 deg/px,
  // within 6% of the constant people are already used to.
  //
  // What this does NOT change is the property the fov-aware law exists for: the screen
  // displacement per dragged pixel stays constant across each lens's whole FOV range. Only
  // the value of that constant moves, from 1 to k.
  constexpr float kDragSensitivity = 2.5f;

  if (vp_w <= 0 || vp_h <= 0) {
    return 0.0f;  // degenerate viewport: produce no rotation this frame
  }
  float img_radius = std::min(static_cast<float>(vp_w), static_cast<float>(vp_h)) * 0.5f;
  float half_fov = fov_deg * 0.5f * kPi / 180.0f;

  float rad_per_px = 0.0f;
  if (lens_type == kLensTypeLinear) {
    rad_per_px = std::tan(half_fov) / img_radius;
  } else if (lens_type == kLensTypeFisheyeEqualArea) {
    rad_per_px = 2.0f * std::sin(half_fov * 0.5f) / img_radius;
  } else if (lens_type == kLensTypeFisheyeEquidist) {
    rad_per_px = half_fov / img_radius;
  } else if (lens_type == kLensTypeFisheyeStereographic) {
    rad_per_px = 2.0f * std::tan(half_fov * 0.5f) / img_radius;
  } else if (lens_type == kLensTypeFisheyeOrthographic) {
    rad_per_px = std::sin(half_fov) / img_radius;
  } else if (lens_type == kLensTypeGlobe) {
    rad_per_px = (kGlobeCameraD - 1.0f) * std::tan(half_fov) / img_radius;
  } else {
    // Full-sky (or a newly added) lens type. A release-visible warning rather than a
    // bare assert: assert compiles out, and a new lens type that is draggable but
    // missing from this switch would otherwise silently fall back to the old,
    // fov-blind feel with nothing to notice it by.
    // Warn once per lens type, not once per frame: this is called from the drag handler, so an
    // unthrottled warning would emit tens of lines per second and bury the very signal it exists
    // to raise. The latch is per type rather than a single global flag so a second unknown type
    // still gets its own line instead of being swallowed by the first one.
    //
    // thread_local rather than a bare static, even though every caller today is the ImGui
    // main loop and a bare static would do: the promise this latch makes is "one line per
    // lens type per thread", which degrades gracefully (an extra line) if someone ever
    // calls this off the UI thread, whereas a bare static would be an unsynchronized
    // read-modify-write and thus a data race. The cost of the choice is that promise's
    // "per thread" qualifier, stated here rather than left for a reader to infer.
    static thread_local unsigned int warned_lens_types = 0u;
    const bool latchable = lens_type >= 0 && lens_type < static_cast<int>(sizeof(warned_lens_types) * 8);
    const unsigned int bit = latchable ? (1u << lens_type) : 0u;
    if (!latchable || (warned_lens_types & bit) == 0u) {
      warned_lens_types |= bit;
      GUI_LOG_WARNING("ComputeDragGainDegPerPixel: no drag-gain law for lens_type {}; falling back to {} deg/px",
                      lens_type, kLegacyGainDegPerPixel);
    }
    return kLegacyGainDegPerPixel;
  }
  return rad_per_px * kDragSensitivity * 180.0f / kPi;
}

void PreviewRenderer::Render(int vp_x, int vp_y, int vp_w, int vp_h, const PreviewParams& params) {
  if (!shader_program_ || !texture_ || vp_w <= 0 || vp_h <= 0) {
    return;
  }

  // Consume any deferred GL blank request queued by ClearTexture() before
  // any texture binding / sampling below runs, so the sim layer samples a
  // fresh 1x1 black instead of the previous scene's stale pixels. Must run
  // before glActiveTexture/glBindTexture(texture_) below to avoid stashing
  // state; UploadBlankSimTexture unbinds itself.
  if (needs_gl_blank_) {
    UploadBlankSimTexture();
    needs_gl_blank_ = false;
  }

  // Save/restore GL state that ImGui might depend on
  GLint prev_program;
  glGetIntegerv(GL_CURRENT_PROGRAM, &prev_program);
  GLint prev_vao;
  glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &prev_vao);
  GLint prev_viewport[4];
  glGetIntegerv(GL_VIEWPORT, prev_viewport);

  glViewport(vp_x, vp_y, vp_w, vp_h);
  glUseProgram(shader_program_);

  // Set uniforms
  glUniform2f(glGetUniformLocation(shader_program_, "u_resolution"), static_cast<float>(vp_w),
              static_cast<float>(vp_h));
  glUniform1i(glGetUniformLocation(shader_program_, "u_lens_type"), params.view_proj.lens_type);
  glUniform1f(glGetUniformLocation(shader_program_, "u_fov"), params.view_proj.fov);
  glUniform1i(glGetUniformLocation(shader_program_, "u_visible"), params.view_proj.visible);
  glUniform1i(glGetUniformLocation(shader_program_, "u_front"), params.view_proj.front ? 1 : 0);
  glUniform1i(glGetUniformLocation(shader_program_, "u_xyz_mode"), xyz_mode_ ? 1 : 0);
  glUniform3f(glGetUniformLocation(shader_program_, "u_background"), params.background_color_linear[0],
              params.background_color_linear[1], params.background_color_linear[2]);
  glUniform1f(glGetUniformLocation(shader_program_, "u_intensity_scale"), params.exposure.intensity_scale);
  glUniform1f(glGetUniformLocation(shader_program_, "u_max_abs_dz"), params.source.max_abs_dz);
  glUniform1f(glGetUniformLocation(shader_program_, "u_r_scale"), params.source.r_scale);

  float view_matrix[9];
  BuildViewMatrix(params.view_proj.elevation, params.view_proj.azimuth, params.view_proj.roll, view_matrix);
  glUniformMatrix3fv(glGetUniformLocation(shader_program_, "u_view_matrix"), 1, GL_FALSE, view_matrix);

  // Bind equirect texture to unit 0
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, texture_);
  glUniform1i(glGetUniformLocation(shader_program_, "u_texture"), 0);

  // Background image uniforms
  glUniform1i(glGetUniformLocation(shader_program_, "u_bg_enabled"), params.bg.enabled ? 1 : 0);
  if (params.bg.enabled) {
    glUniform1f(glGetUniformLocation(shader_program_, "u_overlay_alpha"), params.bg.alpha);

    // CPU-side contain-fit + pan/zoom UV calculation (ComputeBgUvTransform, declared in the
    // header so it is unit-testable without a GL context).
    const BgUvTransform bg_uv =
        ComputeBgUvTransform(vp_w, vp_h, params.bg.aspect, params.bg.pan_x, params.bg.pan_y, params.bg.zoom);
    glUniform2f(glGetUniformLocation(shader_program_, "u_bg_uv_scale"), bg_uv.scale_x, bg_uv.scale_y);
    glUniform2f(glGetUniformLocation(shader_program_, "u_bg_uv_offset"), bg_uv.offset_x, bg_uv.offset_y);

    // Bind bg texture to unit 1
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, bg_texture_);
    glUniform1i(glGetUniformLocation(shader_program_, "u_bg_texture"), 1);
    glActiveTexture(GL_TEXTURE0);
  }

  // Auxiliary line overlay uniforms
  const auto& ov = params.overlay;
  // Take the caller's mask if it is newer than what the texture holds. Gated on the generation
  // rather than on the pointer: the cache reuses its buffer, so an unchanged pointer says nothing
  // about whether the contents moved, and re-uploading a W*H texture every frame would give back
  // most of what computing the mask off the draw loop was for.
  if (ov.angular_dist_mask == nullptr) {
    ClearAngularDistMask();
    angular_dist_tex_generation_ = 0;
  } else if (ov.angular_dist_mask_generation != angular_dist_tex_generation_) {
    UploadAngularDistMask(ov.angular_dist_mask, ov.angular_dist_mask_w, ov.angular_dist_mask_h);
    angular_dist_tex_generation_ = ov.angular_dist_mask_generation;
  }
  glUniform1i(glGetUniformLocation(shader_program_, "u_show_horizon"), ov.show_horizon ? 1 : 0);
  glUniform1i(glGetUniformLocation(shader_program_, "u_show_grid"), ov.show_grid ? 1 : 0);
  glUniform1i(glGetUniformLocation(shader_program_, "u_show_sun_circles"), ov.show_sun_circles ? 1 : 0);
  // Unit 2 (0 is the sim texture, 1 the background image). The mask is uploaded by
  // UploadAngularDistMask, which the caller drives off its AnnotationOverlayCache; a frame in
  // which nothing has been uploaded yet draws no circles rather than drawing them somewhere
  // guessed at.
  glUniform1i(glGetUniformLocation(shader_program_, "u_has_angular_dist_mask"), angular_dist_tex_ != 0 ? 1 : 0);
  if (angular_dist_tex_ != 0) {
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, angular_dist_tex_);
    glUniform1i(glGetUniformLocation(shader_program_, "u_angular_dist_mask"), 2);
    glActiveTexture(GL_TEXTURE0);
  }
  glUniform3f(glGetUniformLocation(shader_program_, "u_horizon_color"), ov.horizon_color[0], ov.horizon_color[1],
              ov.horizon_color[2]);
  glUniform3f(glGetUniformLocation(shader_program_, "u_grid_color"), ov.grid_color[0], ov.grid_color[1],
              ov.grid_color[2]);
  glUniform3f(glGetUniformLocation(shader_program_, "u_sun_circles_color"), ov.sun_circles_color[0],
              ov.sun_circles_color[1], ov.sun_circles_color[2]);
  glUniform1f(glGetUniformLocation(shader_program_, "u_horizon_alpha"), ov.horizon_alpha);
  glUniform1f(glGetUniformLocation(shader_program_, "u_grid_alpha"), ov.grid_alpha);
  glUniform1f(glGetUniformLocation(shader_program_, "u_sun_circles_alpha"), ov.sun_circles_alpha);
  glUniform1f(glGetUniformLocation(shader_program_, "u_grid_step"), ov.grid_step);

  glUniform1i(glGetUniformLocation(shader_program_, "u_show_zenith_nadir"), ov.show_zenith_nadir ? 1 : 0);
  glUniform2f(glGetUniformLocation(shader_program_, "u_zenith_screen_pos"), ov.zenith_screen_pos[0],
              ov.zenith_screen_pos[1]);
  glUniform2f(glGetUniformLocation(shader_program_, "u_nadir_screen_pos"), ov.nadir_screen_pos[0],
              ov.nadir_screen_pos[1]);
  glUniform1f(glGetUniformLocation(shader_program_, "u_zenith_nadir_radius_px"), ov.zenith_nadir_radius_px);
  glUniform3f(glGetUniformLocation(shader_program_, "u_zenith_nadir_color"), ov.zenith_nadir_color[0],
              ov.zenith_nadir_color[1], ov.zenith_nadir_color[2]);
  glUniform1f(glGetUniformLocation(shader_program_, "u_zenith_nadir_alpha"), ov.zenith_nadir_alpha);

  glUniform1i(glGetUniformLocation(shader_program_, "u_show_lens_border"), ov.show_lens_border ? 1 : 0);
  glUniform3f(glGetUniformLocation(shader_program_, "u_lens_border_color"), ov.lens_border_color[0],
              ov.lens_border_color[1], ov.lens_border_color[2]);
  glUniform1f(glGetUniformLocation(shader_program_, "u_lens_border_alpha"), ov.lens_border_alpha);

  // Draw fullscreen quad
  glBindVertexArray(vao_);
  glDrawArrays(GL_TRIANGLES, 0, 6);

  // Restore state
  glBindVertexArray(static_cast<GLuint>(prev_vao));
  glUseProgram(static_cast<GLuint>(prev_program));
  glViewport(prev_viewport[0], prev_viewport[1], prev_viewport[2], prev_viewport[3]);
}

}  // namespace lumice::gui
