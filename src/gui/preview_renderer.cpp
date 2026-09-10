#include "gui/preview_renderer.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "gui/gl_common.h"
#include "gui/gui_logger.hpp"
#include "util/annotation_line_width.hpp"

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
uniform int u_tex_mode;           // kTexModeSrgbComposited / kTexModeXyz / kTexModeSrgbRadiance
uniform vec3 u_background;        // sky colour, LINEAR RGB (see PreviewParams::background_color_linear)
uniform vec3 u_paper;             // paper colour, LINEAR RGB (see PreviewParams::paper_color_linear)
uniform int u_tone;               // 0 = screen (additive), 1 = print (subtractive) — config::RenderConfig::Tone
uniform sampler2D u_bg_texture;
uniform float u_max_abs_dz;      // overlap zone |sky.z| threshold (0 = no blend)
uniform float u_r_scale;         // projection r_scale for overlap normalization
uniform int u_bg_enabled;
uniform float u_overlay_alpha;
uniform vec2 u_bg_uv_scale;
uniform vec2 u_bg_uv_offset;

// Auxiliary line overlay uniforms.
//
// The four curve families — the celestial horizon, the parallels and meridians of the coordinate
// grid, the circles of constant angular distance from a reference direction — are evaluated HERE,
// per fragment, from this fragment's own world direction. Each is a level set of a world-space
// angle field, and the field, the levels and the line-width rule are the same definition the CLI
// renderer evaluates on the CPU (src/core/annotation_overlay.cpp over
// mask_detail::LevelSetMaskFromField); the two are one curve computed by two evaluators, and
// test/gui/parity/test_gui_cli_export_parity.cpp is the gate that keeps their pixels on top of
// each other. What this buys over having core rasterize the curve and uploading a mask is the
// thing the picture itself already has: it is re-projected every frame, so the lines drawn from
// the same per-fragment direction move with it, on the same frame, at no CPU cost and with no
// debounce.
uniform int u_show_horizon;
uniform int u_show_grid;
uniform int u_show_sun_circles;
// The level lists, PACKED FOUR TO A vec4. Not `float u_x[N]`: default-block uniform packing is
// implementation-defined and a float array commonly costs one whole vec4 register per element,
// which at these lengths would exceed GL_MAX_FRAGMENT_UNIFORM_COMPONENTS on drivers that report
// the common 4096. Packed, the lists take 516 registers (2064 components) and fit with room.
// Element i of a list is list[i >> 2][i & 3] (levelAt below).
//
// The two grid families share ONE array: the parallels occupy [0, u_elevation_count), the
// meridians [u_elevation_count, u_elevation_count + u_longitude_count), each range sorted
// ascending by the uploader. One array rather than two so that the nearest-level search reads
// the uniform in place through an index range — a uniform array handed to a GLSL function as a
// parameter is passed BY VALUE, and this driver honours that with a copy of all 256 vec4 per
// call per fragment (measured: 3 ms -> 950 ms a frame). The literal sizes are
// 2 * kMaxOverlayLevels / 4 and kMaxSunCircles / 4 (preview_renderer.hpp / gui_constants.hpp);
// static_asserts after this string pin the two spellings together.
uniform vec4 u_grid_levels_deg[512];
uniform int u_elevation_count;
uniform int u_longitude_count;
uniform vec4 u_angular_dist_deg[4];
uniform int u_angular_dist_count;
// The direction the angular-distance circles are centred on (the sun), a unit vector in the world
// frame this shader's world_dir lives in — the direction light TRAVELS, altitude = asin(-z).
uniform vec3 u_reference_dir;
// The line-width rule's three constants, uploaded from src/util/annotation_line_width.hpp rather
// than written here: GLSL cannot include the header, and the alternative — the same three digits
// spelled a second time in this string — is exactly the drift the header exists to prevent.
uniform float u_line_fwidth_min_deg;
uniform float u_line_fwidth_max_deg;
uniform float u_line_half_width_px;
uniform vec3 u_horizon_color;
uniform vec3 u_grid_color;
uniform vec3 u_sun_circles_color;
uniform float u_horizon_alpha;
uniform float u_grid_alpha;
uniform float u_sun_circles_alpha;

// Sky reference-point ring marker uniforms. One array slot per core marker id
// (LUMICE_ANNOTATION_MARKER_*), so the CPU uploads all six in two calls and the
// loop below needs no count.
// Screen positions are center-origin, y-up, in pixels — same convention as
// `pos = v_ndc * u_resolution * 0.5` (see main()). Sentinel (-9999, -9999)
// trivially fails the distance test so that ring is skipped, which is how a
// marker the user switched off is expressed: there is no enable uniform.
// The literal 6 is LUMICE_ANNOTATION_MARKER_COUNT; GLSL cannot include the
// header, so a static_assert after this string pins the two spellings together.
uniform vec2 u_marker_screen_pos[6];
uniform vec3 u_marker_color[6];
uniform float u_markers_radius_px;
uniform float u_markers_alpha;

// Lens border uniforms: the outline of the projection's own valid image circle
// (where the inverse function's domain guard flips w from 1 to 0). Purely a
// function of u_lens_type / u_fov / u_resolution, so the shader derives the
// radius itself rather than taking a CPU-precomputed screen position.
uniform int u_show_lens_border;
uniform vec3 u_lens_border_color;
uniform float u_lens_border_alpha;

const float PI = 3.14159265358979323846;

// Source-texture semantics. Mirrors PreviewRenderer::TextureMode (preview_renderer.hpp) — the two
// enumerations are one contract and their values must stay in step.
//
//   kTexModeSrgbComposited  8-bit sRGB texels that ALREADY carry the sky colour, drawn as they are.
//                           Two producers: ClearTexture()'s 1x1 black pixel, and a .lmc written
//                           before the format carried radiance-only textures (v <= 3). Neither can
//                           be corrected here: the sky is summed into the texel and un-summing it
//                           is not invertible where the bake clipped.
//   kTexModeXyz             float XYZ radiance from the live simulation.
//   kTexModeSrgbRadiance    8-bit sRGB texels carrying the halo's radiance ALONE, exposure already
//                           applied, no sky. Everything a display still owes the picture — the
//                           lens's relative illumination, then the sky — is applied below, by the
//                           same two steps and in the same order as the XYZ branch.
const int kTexModeSrgbComposited = 0;
const int kTexModeXyz = 1;
const int kTexModeSrgbRadiance = 2;

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
// the same two halves around the same seam, and so does RenderConsumer::PostSnapshot's
// use_real_color branch (src/server/render.cpp). Three implementations of one chain: one has to be
// GLSL, and the other two are separate because the render loop writes bytes as it composites while
// the batch converter is handed a finished buffer. Nothing enforces that they agree; the pixel
// comparison in test/gui/functional/test_preview_background.cpp is what holds this one to the CPU
// one.
//
// The .lmc bake is NOT one of them any more: it reaches the background-free overload, because the
// sky is a setting rather than part of the stored picture and joins right here instead. See
// app.cpp's RefreshCpuTextureForSave and PreviewRenderer::TextureMode.

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

// The inverse of clampAndGamma's transfer curve, for the one branch that starts from sRGB BYTES
// and still has linear-light work left to do. Mirrors lumice::SrgbToLinear (src/util/color_space.hpp).
vec3 srgbToLinear(vec3 srgb) {
    return mix(srgb / 12.92, pow((srgb + 0.055) / 1.055, vec3(2.4)), step(0.04045, srgb));
}

// Clamp + sRGB gamma (branchless via mix+step). The second half of the chain above.
vec3 clampAndGamma(vec3 rgb) {
    rgb = clamp(rgb, 0.0, 1.0);
    return mix(rgb * 12.92, 1.055 * pow(rgb, vec3(1.0/2.4)) - 0.055, step(0.0031308, rgb));
}

// The subtractive (u_tone == 1) operator: ink on paper, replacing the additive `radiance + sky`
// wherever it applies.
//
// HAND-TRANSCRIBED from src/util/ink_transfer.hpp (kInkGamma / InkOpticalDensity /
// InkTransmittance), which is the authority. GLSL cannot #include a C++ header, so this is a copy
// and not a shared implementation — the same arrangement, and for the same reason, as the relIllum*
// block's relationship to src/gui/preview_jacobian.hpp. kInkGamma below MUST equal
// lumice::kInkGamma; nothing in the build enforces that, and what does instead is
// test/gui/functional/test_preview_print_mode.cpp, which renders through this shader and compares
// against values the C++ function produced.
//
// `e` is the exposed scalar — CIE Y after u_intensity_scale and the lens's relative illumination —
// which is the same quantity the CLI calls xyz[1] in RenderConsumer::PostSnapshot.
const float kInkGamma = 11.0;
vec3 subtractiveInk(float e, vec3 paper) {
    // GLSL has no log10 builtin; log(x)/log(10) is the whole of the difference from the C++ line.
    float density = kInkGamma * log(1.0 + max(e, 0.0)) / log(10.0);
    return paper * pow(10.0, -density);
}

// One annotation layer composited over what is already there.
//
// Structurally the same as BlendAnnotation() in src/server/render.cpp — one tone branch, the same
// algebra, and print reading no colour at all — but NOT signature-compatible, and deliberately so:
// this side keeps its own two independent multipliers (the edge falloff `t` and the layer's alpha)
// where the CPU caller has already folded them into one. Aligning the signatures would mean
// changing one side's arithmetic to match the other's spelling.
//
// print multiplies by (1 - t*alpha), i.e. the existing opacity IS the ink coverage. That is what
// makes an annotation visible on white paper by construction rather than by picking a colour that
// contrasts, and why print needs no per-mode palette.
vec3 blendAnnotationColor(vec3 base, vec3 lineColor, float t, float alpha, int tone) {
    return (tone == 1) ? base * (1.0 - t * alpha) : mix(base, lineColor, t * alpha);
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
  // `pixel` is in core's binning convention (ProjectExitToPixel takes floor of it), so texel `px`
  // owns [px, px+1) and its centre is px + 0.5 — see doc/coordinate-convention.md §11. GL's own
  // addressing is stated in exactly those terms: `uv * tex_res` = px + 0.5 hits the centre of
  // texel px. So the conversion is the plain division and nothing else. Adding 0.5 here applies
  // the same half-texel centring a SECOND time, which does not read a different texel — it moves
  // every fragment onto a texel corner and turns every sample into a 2x2 bilinear average.
  return pixel / tex_res;
}
)glsl"
R"glsl(
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

// THE inverse: the fragment-space position `pos` (centre-origin, y-up pixels) to the world
// direction the active lens images there, with w = 1 when the lens images it at all and 0 when
// it does not. Every branch below writes `ri`, the target lens's relative illumination at that
// pixel; an unknown u_lens_type leaves it at the identity, alongside the black w = 0 it already
// produces. `pos_ovl` receives the position in the overlay's own space: the core-pixel-inverse
// family (dual fisheye 4-6/9, rectangular 7) inverts Core's y-DOWN pixel layout
// (DualFisheyeToPixel / RectangularForward) while `pos` is y-UP (GL NDC), so for exactly that
// family the y is flipped — feeding y-up straight in would flip the display vertically against
// the CLI render. The view-matrix family (linear/fisheye/ortho/globe) is self-consistent and keeps
// raw pos. The same flip is mirrored in overlay_labels.cpp (PixelToWorldDir / WorldDirToPixel) so
// grid labels, markers and mouse interaction follow the flipped content. See scrum
// gui-lens-math-cli-alignment (owner: dual-fisheye/rectangular follow CLI).
//
// ONE function because it has TWO callers: main(), for the picture, and overlayAuxLines, which
// evaluates it at a fragment's right and lower neighbours to take the forward differences the
// annotation line rule is defined on. The direction a line is drawn from and the direction the
// picture is sampled at must be the same inverse, or the line lands beside the picture.
vec4 inverseWorldDir(vec2 pos, float half_fov, out float ri, out vec2 pos_ovl) {
  pos_ovl = pos;
  ri = 1.0;
  vec4 result = vec4(0.0, 0.0, 0.0, 0.0);
  bool needs_view_transform = true;
  if (u_lens_type == 0) {
    result = linearInverse(pos, half_fov, ri);
  } else if (u_lens_type >= 1 && u_lens_type <= 3) {
    result = fisheyeInverse(pos, half_fov, u_lens_type - 1, ri);
  } else if (u_lens_type >= 4 && u_lens_type <= 6) {
    pos_ovl = vec2(pos.x, -pos.y);
    result = dualFisheyeInverse(pos_ovl, u_lens_type - 4, ri);
    needs_view_transform = false;  // Core dual fisheye works in world space
  } else if (u_lens_type == 7) {
    pos_ovl = vec2(pos.x, -pos.y);
    result = rectangularInverse(pos_ovl, ri);
    needs_view_transform = false;  // Core rectangular works in world space
  } else if (u_lens_type == 8) {   // kLensTypeFisheyeOrthographic
    result = fisheyeInverse(pos, half_fov, 3, ri);
  } else if (u_lens_type == 9) {   // kLensTypeDualFisheyeOrthographic
    pos_ovl = vec2(pos.x, -pos.y);
    result = dualFisheyeInverse(pos_ovl, 3, ri);
    needs_view_transform = false;  // Core dual fisheye works in world space
  } else if (u_lens_type == 10) {  // kLensTypeGlobe
    result = globeInverse(pos, half_fov, ri);
    needs_view_transform = false;  // globeInverse already returned world-space dir
  }
  // else: unknown u_lens_type → result.w = 0 (black). static_assert in gui_state.hpp
  // pins kLensTypeCount so any out-of-range value is a compile-time catchable mismatch.
  if (result.w >= 0.5 && needs_view_transform) {
    result.xyz = u_view_matrix * result.xyz;
  }
  return result;
}

// The local gradient of an angle field, clamped as src/util/annotation_line_width.hpp's rule
// clamps it — the one number the line's width and its edge are both measured in. `fw` is the
// hardware fwidth() for a non-circular field; the CIRCULAR field (azimuth) must not take the raw
// fwidth(), see wrapAngleDiffDeg below. CPU twin: mask_detail::LevelSetMaskFromField
// (src/core/lens_proj_build.hpp), with a forward difference in the fwidth slot.
float lineGradientDeg(float fw) {
  return clamp(fw, u_line_fwidth_min_deg, u_line_fwidth_max_deg);
}

// An angle difference in degrees folded into [-180, 180). For CIRCULAR fields only (azimuth): a
// naive difference across the +/-180 seam reads the 179 -> -179 step as 358 instead of 2, both
// when the local gradient is measured (fwidth would report a wall of gradient along the seam and
// the clamp would turn it into a 3 px band down the anti-meridian) and when the distance to a
// level is measured. Altitude ([-90, 90]) and angular distance ([0, 180]) have no seam and must
// NOT go through this. CPU twin: mask_detail::WrapAngleDiffDeg, whose range is (-180, 180]; the
// two differ only at exactly 180, where both sides take the absolute value and agree.
float wrapAngleDiffDeg(float d) {
  return d - 360.0 * floor((d + 180.0) / 360.0);
}

// Whether a fragment at distance `d_deg` (already wrapped for a circular field) from a level is
// on the line, given the local gradient `grad` in degrees per pixel: the SET the CPU marks,
// |d| < grad * kAnnotationLineHalfWidthPx, and nothing more. 1 or 0.
//
// HARD-EDGED ON PURPOSE, and the purpose is measured. Two antialiased shapes were tried here and
// both were rejected by test/gui/parity/test_gui_cli_export_parity.cpp, whose subject is that the
// preview and the CLI draw ONE line: a smoothstep from the level out to the half-width (the
// profile this function had before the curves were ever rasterized by core) integrates to half the
// CLI's ink and read 18 / 15 / 20 dB against thresholds of 27 / 27 / 34 on its three scenes; the
// pixel-coverage ramp of the same set (50 % contour on the CLI's edge, same ink) recovered the
// positions to under half a pixel but still read 24 / 15 / 27 — a one-pixel ramp on both edges of
// every line is a few thousand half-covered pixels per frame against the CLI's hard band. The
// hard set is what the mask sampled NEAREST used to give, so it is also the look the preview has
// shipped with; antialiasing it is a decision for both renderers at once (src/server/render.cpp
// PostSnapshot composites the same set with no ramp), not for this side alone.
float lineCoverage(float d_deg, float grad) {
  return abs(d_deg) < grad * u_line_half_width_px ? 1.0 : 0.0;
}

// Element i of the packed grid level array.
float gridLevelAt(int i) {
  return u_grid_levels_deg[i >> 2][i & 3];
}

// The distance, in degrees, from `field_deg` to the NEAREST level in u_grid_levels_deg[begin, end),
// a range sorted ascending (the uploader sorts; see UploadGridLevels). Coverage falls off
// monotonically with distance, so the largest coverage over a list is the coverage of its nearest
// level, and finding that is a binary search — a dozen steps — where a linear pass over the list
// is up to a thousand (720 meridians and 360 parallels at the narrowest field of view, per
// fragment, per frame; measured at 1600x1200 on an M2 Max as +10 ms a frame linearly, most of a
// 60 fps budget spent on an edge case).
//
// `circular` folds the field onto the azimuth circle: the nearest level may then be the range's
// first or last element reached across the +/-180 seam, so those two are tried through
// wrapAngleDiffDeg on top of the two neighbours the search finds.
float nearestGridLevelDistDeg(float field_deg, int begin, int end, bool circular) {
  if (end <= begin) return 1e9;
  // First index in [begin, end) whose level is >= field_deg. 12 iterations halve 1024 to one.
  int lo = begin;
  int hi = end;
  for (int it = 0; it < 12; ++it) {
    if (lo >= hi) break;
    int mid = (lo + hi) >> 1;
    if (gridLevelAt(mid) < field_deg) lo = mid + 1; else hi = mid;
  }
  float d = 1e9;
  if (lo < end) d = min(d, abs(field_deg - gridLevelAt(lo)));
  if (lo > begin) d = min(d, abs(field_deg - gridLevelAt(lo - 1)));
  if (circular) {
    d = min(d, abs(wrapAngleDiffDeg(field_deg - gridLevelAt(begin))));
    d = min(d, abs(wrapAngleDiffDeg(field_deg - gridLevelAt(end - 1))));
  }
  return d;
}

// Sky reference-point ring markers — drawn last so they sit on top of all other overlays. Their
// positions are NOT derived from the fragment's direction: a marker is a named direction core
// projects (LUMICE_ComputeAnnotationAnchors), handed over as pixel-space uniforms. CPU passes
// sentinel (-9999, -9999) for a marker that is switched off, offscreen or behind the camera; the
// distance test rejects it naturally, which is why there is no per-marker enable to read.
//
// A constant-bound loop over the whole id space rather than six unrolled blocks: the bound is a
// compile-time constant so there is no dynamic branching, and the alternative is the same
// arithmetic written six times. Order is id order, and the LATER id wins where two rings overlap
// — the same last-writer rule the three curve families follow.
//
// pos_pix: pixel-space position of the current fragment, center-origin (0,0), y-up. Matches the
// CPU helper ProjectWorldDirToScreen for marker overlays.
vec3 overlayMarkers(vec3 color, vec2 pos_pix) {
  const float kRingHalfWidthPx = 1.5;
  for (int i = 0; i < 6; ++i) {
    float d = length(pos_pix - u_marker_screen_pos[i]);
    float t = 1.0 - smoothstep(0.0, kRingHalfWidthPx, abs(d - u_markers_radius_px));
    color = blendAnnotationColor(color, u_marker_color[i], t, u_markers_alpha, u_tone);
  }
  return color;
}

)glsl"
R"glsl(
// Overlay auxiliary lines on top of final_color.
//
// world_dir: this fragment's unit world-space direction (the one the picture was sampled at), in
// the convention every annotation direction uses — altitude = asin(-z), azimuth = atan2(-y, -x).
// The caller only reaches here for a fragment the lens images AND the hemisphere policy admits,
// so the clip the CLI applies through its `drawable` mask is applied here by the call site.
// pos: the fragment's raw position (centre-origin, y-up), from which the neighbours' directions
// are re-derived; pos_pix: the same in the overlay's space (flipped for the CPI family), which is
// what the marker positions are stated in.
vec3 overlayAuxLines(vec3 world_dir, vec3 color, vec2 pos_pix, vec2 pos, float half_fov) {
  const float DEG = 180.0 / PI;

  // The three angle fields. Each formula is the shader-side twin of a core function, named here so
  // a change on either side has somewhere to look:
  //   altitude_deg      mask_detail::AltitudeDeg           (src/core/lens_proj_build.hpp)
  //   azimuth_deg       annotation::AzimuthDegOfDir        (src/core/annotation_overlay.cpp)
  //   angular_dist_deg  annotation::AngularDistDegOfDir    (src/core/annotation_overlay.cpp)
  float altitude_deg = asin(clamp(-world_dir.z, -1.0, 1.0)) * DEG;
  float azimuth_deg = atan(-world_dir.y, -world_dir.x) * DEG;
  float angular_dist_deg = acos(clamp(dot(world_dir, u_reference_dir), -1.0, 1.0)) * DEG;

  // The local gradient of each field, in degrees per pixel, as the FORWARD DIFFERENCE against the
  // right and lower neighbours — the same two differences, against the same two pixels, that
  // mask_detail::LevelSetMaskFromField takes on the CPU (src/core/lens_proj_build.hpp), with the
  // same three rules: a neighbour the lens does not image contributes nothing; the last column /
  // row differences backwards instead; a pixel with no imaged neighbour at all gets no line.
  //
  // NOT the hardware fwidth(). That derivative is taken across a 2x2 quad, so half the fragments
  // read a backward difference and the quad's helper invocations reach outside the lens's domain
  // at its rim; each is a pixel here and there that one renderer lights and the other does not,
  // and against a hard-edged line a whole-frame comparison of the two arms pays for every such
  // pixel at full amplitude (measured on the rectilinear parity scene as 213 pixels for 1.8 dB).
  // Three inverse projections per fragment instead of one is the price of the two evaluators
  // agreeing on which pixels are the line, and it is a few trig calls.
  //
  // "Lower" is the next IMAGE row, which in this y-up fragment space is pos.y - 1; the last
  // row/column test is the CPU's `px + 1 < width` written in centre-origin coordinates.
  vec2 half_res = u_resolution * 0.5;
  vec2 step_x = vec2(pos.x + 0.5 < half_res.x ? 1.0 : -1.0, 0.0);
  vec2 step_y = vec2(0.0, -(pos.y - 0.5 > -half_res.y ? 1.0 : -1.0));
  float ri_unused;
  vec2 ovl_unused;
  vec4 nx = inverseWorldDir(pos + step_x, half_fov, ri_unused, ovl_unused);
  vec4 ny = inverseWorldDir(pos + step_y, half_fov, ri_unused, ovl_unused);
  if (nx.w < 0.5 && ny.w < 0.5) {
    // No local scale can be measured here, so no curve is drawn — the CPU's rule, and the one
    // that keeps a horizon out of a corner that images nothing.
    return overlayMarkers(color, pos_pix);
  }
  float fw_alt = 0.0;
  float fw_az = 0.0;
  float fw_dist = 0.0;
  if (nx.w >= 0.5) {
    fw_alt += abs(asin(clamp(-nx.z, -1.0, 1.0)) * DEG - altitude_deg);
    fw_az += abs(wrapAngleDiffDeg(atan(-nx.y, -nx.x) * DEG - azimuth_deg));
    fw_dist += abs(acos(clamp(dot(nx.xyz, u_reference_dir), -1.0, 1.0)) * DEG - angular_dist_deg);
  }
  if (ny.w >= 0.5) {
    fw_alt += abs(asin(clamp(-ny.z, -1.0, 1.0)) * DEG - altitude_deg);
    fw_az += abs(wrapAngleDiffDeg(atan(-ny.y, -ny.x) * DEG - azimuth_deg));
    fw_dist += abs(acos(clamp(dot(ny.xyz, u_reference_dir), -1.0, 1.0)) * DEG - angular_dist_deg);
  }
  float grad_alt = lineGradientDeg(fw_alt);
  float grad_az = lineGradientDeg(fw_az);
  float grad_dist = lineGradientDeg(fw_dist);

  // Coordinate grid — drawn first so the other lines overlay on top. Parallels and meridians share
  // one colour and one alpha, so their coverages are merged before the blend, exactly as the CLI
  // composites the two families' masks into one layer.
  if (u_show_grid != 0) {
    int ec = u_elevation_count;
    float t = lineCoverage(nearestGridLevelDistDeg(altitude_deg, 0, ec, false), grad_alt);
    t = max(t, lineCoverage(nearestGridLevelDistDeg(azimuth_deg, ec, ec + u_longitude_count, true), grad_az));
    color = blendAnnotationColor(color, u_grid_color, t, u_grid_alpha, u_tone);
  }

  // Circles of constant angular distance from u_reference_dir. A linear pass: the list is at most
  // kMaxSunCircles long, so there is nothing for a search to save.
  if (u_show_sun_circles != 0) {
    float t = 0.0;
    for (int i = 0; i < u_angular_dist_count; ++i) {
      t = max(t, lineCoverage(angular_dist_deg - u_angular_dist_deg[i >> 2][i & 3], grad_dist));
    }
    color = blendAnnotationColor(color, u_sun_circles_color, t, u_sun_circles_alpha, u_tone);
  }

  // Horizon line (altitude = 0) — drawn last of the curves so it's most visible.
  if (u_show_horizon != 0) {
    color = blendAnnotationColor(color, u_horizon_color, lineCoverage(altitude_deg, grad_alt), u_horizon_alpha, u_tone);
  }

  return overlayMarkers(color, pos_pix);
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
    color = blendAnnotationColor(color, u_lens_border_color, tl, u_lens_border_alpha, u_tone);
    float dr = abs(length(pos - vec2(circle_radius, 0.0)) - circle_radius);
    float tr = 1.0 - smoothstep(0.0, kBorderHalfWidthPx, dr);
    color = blendAnnotationColor(color, u_lens_border_color, tr, u_lens_border_alpha, u_tone);
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
  return blendAnnotationColor(color, u_lens_border_color, t, u_lens_border_alpha, u_tone);
}
)glsl"
R"glsl(
out vec4 frag_color;

void main() {
  vec2 pos = v_ndc * u_resolution * 0.5;  // Convert NDC [-1,1] to pixel offset from center (y-up)
  float half_fov = u_fov * 0.5 * PI / 180.0;

  // The inverse, and the overlay-space position (see inverseWorldDir for the y flip it applies
  // to the core-pixel-inverse family).
  vec2 pos_ovl = pos;  // pixel pos handed to overlayAuxLines (flipped for the CPI family)
  float rel_illum = 1.0;
  vec4 result = inverseWorldDir(pos, half_fov, rel_illum, pos_ovl);

  // Eliminated early returns so bg mixing can always execute at the end.
  //
  // The starting value is this mode's ZERO-ENERGY colour, and it stands for every pixel the branch
  // below leaves alone: outside the lens's image circle, in the half-sky `visible` discards, behind
  // the camera under `front`. Under print that is bare paper — which is what subtractiveInk(0.0,
  // u_paper) returns, written here in its already-simplified form since 10^0 is 1. The CLI states
  // the same rule at its own two spellings (the masked-pixel branch and FillZeroEnergyImage in
  // src/server/render.cpp).
  vec3 final_color = (u_tone == 1) ? clampAndGamma(u_paper) : vec3(0.0);
  vec3 world_dir = vec3(0.0);
  bool pixel_visible = false;

  if (result.w >= 0.5) {
    world_dir = result.xyz;

    // Visible hemisphere check
    // In equirect convention: lat = asin(-dz), lat > 0 means upper sky
    //
    // SYNC:visible-hemisphere-predicate — core states the same rule in C++, as
    // lens_proj_build.hpp's VisibleByRange (`kUpper && wz > 0` -> not visible). GLSL cannot call
    // into it, so the rule is written twice on purpose; the two are kept honest by
    // test_visible_mask_gui_parity.cpp, which compares the resulting masks pixel by pixel across
    // every lens type and all three values. Both sides are a DISPLAY clip and neither may become
    // an energy cull -- the branch below never samples the texture for an excluded pixel, and
    // core's twin (SYNC:visible-mask-zero) zeroes one that already accumulated.
    float lat = asin(clamp(-world_dir.z, -1.0, 1.0));
    pixel_visible = true;
    if (u_visible == 0 && lat < 0.0) pixel_visible = false;   // upper: discard lower hemisphere
    if (u_visible == 1 && lat > 0.0) pixel_visible = false;   // lower: discard upper hemisphere
    // u_front: discard back hemisphere (AND with base). u_view_matrix[2] = -forward
    // (see BuildViewMatrix), so dot > 0 means world_dir is behind the camera.
    if (u_front == 1 && dot(world_dir, u_view_matrix[2]) > 0.0) pixel_visible = false;

    if (pixel_visible) {
      vec3 tex_color = sampleDualFisheye(world_dir);
      if (u_tex_mode != kTexModeSrgbComposited) {
        // Linear-light radiance for this pixel. The two source formats decode differently and
        // agree from here on: vignetting, then sky, then the transfer curve, in that order and by
        // these same lines. That shared tail is the whole point — it is what makes a document
        // reopened from disk render the picture the live view was showing when it was saved.
        vec3 radiance_linear;
        if (u_tex_mode == kTexModeXyz) {
          // The projection's own natural vignetting, applied to the energy BEFORE the exposure
          // scale multiplies it: it states how much of this direction's radiance the target lens's
          // pixel collects, so it belongs to the quantity being exposed rather than to the
          // exposure. See the relIllum* block above and src/gui/preview_jacobian.hpp.
          radiance_linear = xyzToLinearRgb(tex_color * rel_illum);
        } else {
          // kTexModeSrgbRadiance: the exposure was already applied when these bytes were baked, so
          // there is no u_intensity_scale here and no gamut clip to redo — only the decode. The
          // vignetting is still a display-time property of the TARGET lens, not of the stored
          // picture, so it is applied here rather than baked, exactly as in the branch above. What
          // makes that possible is that the bake no longer sums the sky into the texel: scaling a
          // composited texel would dim the sky too, which neither the CLI nor this shader does.
          radiance_linear = srgbToLinear(tex_color) * rel_illum;
        }

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
        // Not in kTexModeSrgbComposited. Those texels are a 1x1 black clear, or a .lmc written
        // before the format carried radiance-only textures — whose pixels were baked with whatever
        // background was in effect when they were saved. Adding here too would apply it twice, and
        // subtracting the old one back out is not invertible where the bake clipped.
        if (u_tone == 1 && u_tex_mode == kTexModeXyz) {
          // print: ink on paper replaces the whole additive statement above. The gamut clip and the
          // matrix are skipped along with the sky — print is greyscale by construction, so what is
          // taken is the exposed scalar alone. tex_color.y is CIE Y and the two multipliers are the
          // ones xyzToLinearRgb would have applied to it, in the same order, so this is the same
          // number the CLI calls xyz[1] (src/server/render.cpp).
          float e = tex_color.y * rel_illum * u_intensity_scale;
          tex_color = clampAndGamma(subtractiveInk(e, u_paper));
        } else if (u_tone == 1) {
          // kTexModeSrgbRadiance under print: a KNOWN GAP, held here on purpose rather than left to
          // look supported. These texels are a reopened .lmc whose exposure was already baked in, so
          // the exposed scalar `e` the operator needs is not directly available — recovering it
          // would mean inverting the bake through the standard luminance coefficients, which is a
          // relationship nothing in this repo has verified. Until it is, such a document keeps the
          // screen result until the next simulation run replaces the texture with a live XYZ one.
          // See doc/print-mode-subtractive-ink.md.
          tex_color = clampAndGamma(radiance_linear + u_background);
        } else {
          tex_color = clampAndGamma(radiance_linear + u_background);
        }
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
    final_color = overlayAuxLines(world_dir, final_color, pos_ovl, pos, half_fov);
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

// The `6` written into u_marker_screen_pos[6] / u_marker_color[6] and into the loop bound of
// overlayAuxLines above. GLSL cannot include lumice.h, so the two spellings of the id-space size
// are pinned here instead of merely being expected to match: growing the marker family without
// widening the arrays would upload six of N points and silently drop the rest.
static_assert(LUMICE_ANNOTATION_MARKER_COUNT == 6,
              "the fragment shader hard-codes 6 marker uniform slots; update kFragmentShader's "
              "u_marker_screen_pos[6] / u_marker_color[6] and its loop bound together with this");

// The `512` and `4` written into u_grid_levels_deg[512] / u_angular_dist_deg[4] above, pinned the
// same way: four levels per vec4, two grid families in one array, so the length is 2 * capacity / 4.
static_assert(kMaxOverlayLevels % 4 == 0 && 2 * kMaxOverlayLevels / 4 == 512,
              "the fragment shader hard-codes vec4 u_grid_levels_deg[512]; update kFragmentShader together "
              "with kMaxOverlayLevels");
static_assert(kMaxSunCircles % 4 == 0 && kMaxSunCircles / 4 == 4,
              "the fragment shader hard-codes vec4 u_angular_dist_deg[4]; update kFragmentShader together "
              "with kMaxSunCircles");

// clang-format on

// The kTexMode* block inside the fragment shader above and PreviewRenderer::TextureMode are one
// contract across a language boundary GL gives us no way to check: Render() sends the C++ value
// and the shader compares it against its own literals. These assertions pin the C++ half to the
// numbers written up there, so a change made only here is a build error. The other direction —
// editing the GLSL literals alone — the compiler cannot see, so ADDING A FOURTH MODE means
// touching all four places: the enum (preview_renderer.hpp), the const block in the shader
// string, the branch in the shader body that reads u_tex_mode, and this assertion list.
static_assert(static_cast<int>(PreviewRenderer::TextureMode::kSrgbComposited) == 0,
              "kTexModeSrgbComposited in the fragment shader is 0");
static_assert(static_cast<int>(PreviewRenderer::TextureMode::kXyz) == 1, "kTexModeXyz in the fragment shader is 1");
static_assert(static_cast<int>(PreviewRenderer::TextureMode::kSrgbRadiance) == 2,
              "kTexModeSrgbRadiance in the fragment shader is 2");

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
  // GL sequence + tex_mode_ reset side effect).
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

// Upload a 1x1 black pixel into the GL texture_ storage and reset tex_mode_.
// Reused by Init() (initial GL storage) and Render() (post-ClearTexture
// deferred reset). Must be called on the main thread (owns GL context).
// Does NOT touch tex_width_/tex_height_ — those track "does the app have
// real sim/loaded pixel data" (drives HasTexture()) and must stay 0 across
// this call so ClearTexture()'s CPU-side "cleared" state remains visible to
// callers like Save. Side effect: tex_mode_ = kSrgbComposited, because a 1x1 RGB
// uint8 pixel is not XYZ float data — leaving a stale mode would mislead the
// shader on the next real upload path. Composited rather than radiance-only on
// purpose: this pixel means "there is nothing to show", and the radiance mode
// would paint the sky colour over the whole frame instead of leaving it black.
void PreviewRenderer::UploadBlankSimTexture() {
  static const unsigned char kBlack[3] = { 0, 0, 0 };
  glBindTexture(GL_TEXTURE_2D, texture_);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 1, 1, 0, GL_RGB, GL_UNSIGNED_BYTE, kBlack);
  glBindTexture(GL_TEXTURE_2D, 0);
  tex_mode_ = TextureMode::kSrgbComposited;
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
  UploadUint8Texture(data, width, height, TextureMode::kSrgbComposited);
}

void PreviewRenderer::UploadRadianceTexture(const unsigned char* data, int width, int height) {
  UploadUint8Texture(data, width, height, TextureMode::kSrgbRadiance);
}

// The GL half both uint8 entry points share. `mode` is the only thing that differs, and it is a
// property of what the CALLER baked, which is why it is a parameter here rather than a guess.
void PreviewRenderer::UploadUint8Texture(const unsigned char* data, int width, int height, TextureMode mode) {
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

  if (width != tex_width_ || height != tex_height_ || tex_mode_ == TextureMode::kXyz) {
    // Re-allocate texture when switching from float to uint8 or size changed
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
    tex_width_ = width;
    tex_height_ = height;
  } else {
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, data);
  }

  tex_mode_ = mode;
  glBindTexture(GL_TEXTURE_2D, 0);
}

void PreviewRenderer::UploadXyzTexture(const float* data, int width, int height) {
  if (!texture_ || !data || width <= 0 || height <= 0) {
    return;
  }
  // Fresh real pixel data is about to land in texture_; make the newest write
  // win over any pending deferred blank (see UploadUint8Texture for rationale).
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
  if (width != tex_width_ || height != tex_height_ || tex_mode_ != TextureMode::kXyz) {
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
  tex_mode_ = TextureMode::kXyz;
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

NdcPoint ScreenDeltaToNdcDelta(float dx_pt, float dy_pt, float dpi_scale_x, float dpi_scale_y, int vp_w, int vp_h) {
  if (vp_w <= 0 || vp_h <= 0) {
    return NdcPoint{ 0.0f, 0.0f };
  }
  return NdcPoint{ dx_pt * dpi_scale_x * 2.0f / static_cast<float>(vp_w),
                   -dy_pt * dpi_scale_y * 2.0f / static_cast<float>(vp_h) };
}

NdcPoint ScreenPosToNdc(float x_pt, float y_pt, float dpi_scale_x, float dpi_scale_y, int vp_w, int vp_h) {
  const NdcPoint d = ScreenDeltaToNdcDelta(x_pt, y_pt, dpi_scale_x, dpi_scale_y, vp_w, vp_h);
  // The viewport's top-left corner is NDC (-1, +1); the displacement above is measured from it.
  return NdcPoint{ -1.0f + d.x, 1.0f + d.y };
}

std::optional<BgPixelIndex> BgUvToPixelIndex(float u, float v, int img_w, int img_h) {
  if (img_w <= 0 || img_h <= 0) {
    return std::nullopt;
  }
  // Same square the shader tests before it samples u_bg_texture (preview_renderer's fragment
  // source, background overlay stage) — outside it the shader paints the black letterbox, so
  // there is no photo pixel to report.
  if (!(u >= 0.0f && u <= 1.0f && v >= 0.0f && v <= 1.0f)) {
    return std::nullopt;
  }
  // floor, then clamp: u == 1.0f is inside the square by the test above but floors to img_w,
  // one past the last column.
  int col = static_cast<int>(std::floor(u * static_cast<float>(img_w)));
  int row = static_cast<int>(std::floor(v * static_cast<float>(img_h)));
  col = std::max(0, std::min(img_w - 1, col));
  row = std::max(0, std::min(img_h - 1, row));
  return BgPixelIndex{ col, row };
}

std::optional<std::array<float, 3>> SampleBgColorAtScreenPos(float x_pt, float y_pt, const BgSampleGeometry& geom,
                                                             const std::vector<unsigned char>& pixels) {
  const NdcPoint ndc = ScreenPosToNdc(x_pt, y_pt, geom.dpi_scale_x, geom.dpi_scale_y, geom.vp_w, geom.vp_h);
  const BgUvTransform t = ComputeBgUvTransform(geom.vp_w, geom.vp_h, geom.bg_aspect, geom.pan_x, geom.pan_y, geom.zoom);
  // The shader's own line, on the CPU: bg_uv = v_ndc * u_bg_uv_scale + u_bg_uv_offset.
  const float u = ndc.x * t.scale_x + t.offset_x;
  const float v = ndc.y * t.scale_y + t.offset_y;
  const std::optional<BgPixelIndex> idx = BgUvToPixelIndex(u, v, geom.img_w, geom.img_h);
  if (!idx) {
    return std::nullopt;
  }
  const size_t offset =
      (static_cast<size_t>(idx->row) * static_cast<size_t>(geom.img_w) + static_cast<size_t>(idx->col)) * 3u;
  if (offset + 2u >= pixels.size()) {
    // geom's dimensions and the buffer disagree — no copy loaded, or a stale one. Report "nothing
    // here" rather than reading past the end.
    return std::nullopt;
  }
  return std::array<float, 3>{ static_cast<float>(pixels[offset]) / 255.0f,
                               static_cast<float>(pixels[offset + 1]) / 255.0f,
                               static_cast<float>(pixels[offset + 2]) / 255.0f };
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

// The two grid level lists into their shared packed vec4 uniform array, and the circles into
// theirs. Each list is clamped to its capacity — the shader reads `count` elements and never past
// what was uploaded — copied into a zero-padded scratch buffer so the last vec4 is whole, and the
// two grid ranges are SORTED ascending, which is the precondition the shader's nearest-level
// search (nearestGridLevelDistDeg) rests on. Sorting here rather than asking callers to is what
// makes that precondition a fact of the upload instead of a contract every caller has to know
// about; the order of a level list carries no meaning anywhere else.
static void UploadGridLevels(unsigned int program, const std::vector<float>& elevation_deg,
                             const std::vector<float>& longitude_deg) {
  const int ec = std::min(static_cast<int>(elevation_deg.size()), kMaxOverlayLevels);
  const int lc = std::min(static_cast<int>(longitude_deg.size()), kMaxOverlayLevels);
  const int vec4_count = (ec + lc + 3) / 4;
  std::vector<float> packed(static_cast<size_t>(vec4_count) * 4, 0.0f);
  std::copy(elevation_deg.begin(), elevation_deg.begin() + ec, packed.begin());
  std::copy(longitude_deg.begin(), longitude_deg.begin() + lc, packed.begin() + ec);
  std::sort(packed.begin(), packed.begin() + ec);
  std::sort(packed.begin() + ec, packed.begin() + ec + lc);
  // "[0]" rather than the bare array name, for the reason the marker upload below gives.
  if (vec4_count > 0) {
    glUniform4fv(glGetUniformLocation(program, "u_grid_levels_deg[0]"), vec4_count, packed.data());
  }
  glUniform1i(glGetUniformLocation(program, "u_elevation_count"), ec);
  glUniform1i(glGetUniformLocation(program, "u_longitude_count"), lc);
}

static void UploadCircleLevels(unsigned int program, const std::vector<float>& angular_dist_deg) {
  const int count = std::min(static_cast<int>(angular_dist_deg.size()), kMaxSunCircles);
  const int vec4_count = (count + 3) / 4;
  std::vector<float> packed(static_cast<size_t>(vec4_count) * 4, 0.0f);
  std::copy(angular_dist_deg.begin(), angular_dist_deg.begin() + count, packed.begin());
  if (vec4_count > 0) {
    glUniform4fv(glGetUniformLocation(program, "u_angular_dist_deg[0]"), vec4_count, packed.data());
  }
  glUniform1i(glGetUniformLocation(program, "u_angular_dist_count"), count);
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
  glUniform1i(glGetUniformLocation(shader_program_, "u_tex_mode"), static_cast<int>(tex_mode_));
  glUniform3f(glGetUniformLocation(shader_program_, "u_background"), params.background_color_linear[0],
              params.background_color_linear[1], params.background_color_linear[2]);
  glUniform3f(glGetUniformLocation(shader_program_, "u_paper"), params.paper_color_linear[0],
              params.paper_color_linear[1], params.paper_color_linear[2]);
  glUniform1i(glGetUniformLocation(shader_program_, "u_tone"), params.tone);
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
  glUniform1i(glGetUniformLocation(shader_program_, "u_show_horizon"), ov.show_horizon ? 1 : 0);
  glUniform1i(glGetUniformLocation(shader_program_, "u_show_grid"), ov.show_grid ? 1 : 0);
  glUniform1i(glGetUniformLocation(shader_program_, "u_show_sun_circles"), ov.show_sun_circles ? 1 : 0);
  // The curve definitions: level lists, packed four to a vec4 (see the uniform block in
  // kFragmentShader for why), the circles' centre, and the line-width rule's constants — the
  // latter from their single owner in src/util/, never as digits in the GLSL source.
  UploadGridLevels(shader_program_, ov.elevation_deg, ov.longitude_deg);
  UploadCircleLevels(shader_program_, ov.angular_dist_deg);
  glUniform3f(glGetUniformLocation(shader_program_, "u_reference_dir"), ov.reference_dir[0], ov.reference_dir[1],
              ov.reference_dir[2]);
  glUniform1f(glGetUniformLocation(shader_program_, "u_line_fwidth_min_deg"), kAnnotationLineFwidthMinDeg);
  glUniform1f(glGetUniformLocation(shader_program_, "u_line_fwidth_max_deg"), kAnnotationLineFwidthMaxDeg);
  glUniform1f(glGetUniformLocation(shader_program_, "u_line_half_width_px"), kAnnotationLineHalfWidthPx);
  glUniform3f(glGetUniformLocation(shader_program_, "u_horizon_color"), ov.horizon_color[0], ov.horizon_color[1],
              ov.horizon_color[2]);
  glUniform3f(glGetUniformLocation(shader_program_, "u_grid_color"), ov.grid_color[0], ov.grid_color[1],
              ov.grid_color[2]);
  glUniform3f(glGetUniformLocation(shader_program_, "u_sun_circles_color"), ov.sun_circles_color[0],
              ov.sun_circles_color[1], ov.sun_circles_color[2]);
  glUniform1f(glGetUniformLocation(shader_program_, "u_horizon_alpha"), ov.horizon_alpha);
  glUniform1f(glGetUniformLocation(shader_program_, "u_grid_alpha"), ov.grid_alpha);
  glUniform1f(glGetUniformLocation(shader_program_, "u_sun_circles_alpha"), ov.sun_circles_alpha);

  // "[0]" and not the bare array name: glGetUniformLocation is specified to resolve the FIRST
  // ELEMENT of an array, and while most drivers also accept the bare name, the spec does not
  // require it. A miss returns -1, which glUniform* then silently ignores — so the failure mode
  // this spelling avoids is six markers that never draw and no error anywhere.
  // std::array's elements are contiguous and each inner array is exactly its two/three floats, so
  // one *fv call with a count of six uploads the whole family.
  static_assert(sizeof(ov.marker_screen_pos) == sizeof(float) * 2 * LUMICE_ANNOTATION_MARKER_COUNT,
                "marker_screen_pos must be a flat float[N][2] for the single glUniform2fv upload");
  static_assert(sizeof(ov.marker_color) == sizeof(float) * 3 * LUMICE_ANNOTATION_MARKER_COUNT,
                "marker_color must be a flat float[N][3] for the single glUniform3fv upload");
  glUniform2fv(glGetUniformLocation(shader_program_, "u_marker_screen_pos[0]"), LUMICE_ANNOTATION_MARKER_COUNT,
               ov.marker_screen_pos[0].data());
  glUniform3fv(glGetUniformLocation(shader_program_, "u_marker_color[0]"), LUMICE_ANNOTATION_MARKER_COUNT,
               ov.marker_color[0].data());
  glUniform1f(glGetUniformLocation(shader_program_, "u_markers_radius_px"), ov.markers_radius_px);
  glUniform1f(glGetUniformLocation(shader_program_, "u_markers_alpha"), ov.markers_alpha);

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
