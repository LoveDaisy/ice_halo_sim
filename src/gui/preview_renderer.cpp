#include "gui/preview_renderer.hpp"

#include <cmath>
#include <cstdio>

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
uniform int u_lens_type;     // 0=linear, 1-3=fisheye, 4-6=dual fisheye, 7=rectangular
uniform float u_fov;         // full FOV in degrees
uniform mat3 u_view_matrix;  // view-to-world rotation (inverse view)
uniform int u_visible;       // 0=upper, 1=lower, 2=full
uniform float u_intensity_scale;  // = intensity_factor / per_pixel_intensity (0 = RGB mode)
uniform int u_xyz_mode;           // 1 = XYZ float texture, 0 = sRGB uint8 texture
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
uniform vec3 u_sun_dir;
uniform float u_sun_circle_angles[16];
uniform int u_sun_circle_count;

const float PI = 3.14159265358979323846;

// Algorithm synced with CPU: src/util/color_space.hpp (GamutClipXyz + XyzToLinearRgb + LinearToSrgb)
// Matrix values from src/util/color_data.hpp kXyzToRgb (C++ row-major → GLSL column-major)
const mat3 kXyzToRgb = mat3(
     3.2404542, -0.9692660,  0.0556434,   // column 0
    -1.5371385,  1.8760108, -0.2040259,   // column 1
    -0.4985314,  0.0415560,  1.0572252    // column 2
);
const vec3 kWhitePointD65 = vec3(0.95047, 1.00000, 1.08883);

// CPU equivalent: lumice::XyzToSrgb() in src/util/color_space.hpp
vec3 xyzToSrgb(vec3 xyz) {
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
    // XYZ→RGB matrix multiply + clamp
    vec3 rgb = clamp(kXyzToRgb * xyz, 0.0, 1.0);
    // sRGB gamma (branchless via mix+step)
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

// Compute view direction from pixel for linear projection
// Returns false (via w component) if outside valid range
vec4 linearInverse(vec2 pos, float half_fov) {
  float short_edge = min(u_resolution.x, u_resolution.y);
  float focal = short_edge * 0.5 / tan(half_fov);
  vec3 d = normalize(vec3(pos, -focal));
  return vec4(d, 1.0);
}

// Compute view direction for fisheye projections
// type: 0=equal_area, 1=equidistant, 2=stereographic
vec4 fisheyeInverse(vec2 pos, float half_fov, int type) {
  float img_radius = min(u_resolution.x, u_resolution.y) * 0.5;  // short_edge/2 — matches Core's short_pix_/2
  float r = length(pos) / img_radius;

  float theta;
  if (type == 0) {        // equal area: r_norm = sin(θ/2) / sin(fov/4)
    float s = r * sin(half_fov * 0.5);
    if (s > 1.0) return vec4(0.0, 0.0, 0.0, 0.0);  // asin domain guard
    theta = 2.0 * asin(s);
  } else if (type == 1) { // equidistant: r_norm = θ / half_fov
    theta = r * half_fov;
    if (theta >= PI) return vec4(0.0, 0.0, 0.0, 0.0);
  } else {                // stereographic: r_norm = tan(θ/2) / tan(fov/4)
    theta = 2.0 * atan(r * tan(half_fov * 0.5));
  }

  float phi = atan(pos.y, pos.x);
  return vec4(sin(theta) * cos(phi), sin(theta) * sin(phi), -cos(theta), 1.0);
}

// Dual fisheye: left circle = upper hemisphere, right circle = lower hemisphere
// Returns world-space direction (no view matrix needed) — matches Core convention.
// Core uses az = atan2(-d.y, -d.x) and pixel mapping with PI/2±az offset.
vec4 dualFisheyeInverse(vec2 pos, int type) {
  float short_res = min(u_resolution.x * 0.5, u_resolution.y);
  float circle_radius = short_res * 0.5;

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

  float theta;
  float half_pi = PI * 0.5;
  if (type == 0) {        // equal area
    float s = use_r * sin(half_pi * 0.5);
    if (s > 1.0) return vec4(0.0, 0.0, 0.0, 0.0);
    theta = 2.0 * asin(s);
  } else if (type == 1) { // equidistant
    theta = use_r * half_pi;
  } else {                // stereographic
    theta = 2.0 * atan(use_r * tan(half_pi * 0.5));
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

// Rectangular (equirectangular): always full-sky, returns world-space direction.
// Matches Core: scale = min(width/2, height) / PI
vec4 rectangularInverse(vec2 pos) {
  float short_res = min(u_resolution.x * 0.5, u_resolution.y);
  float scale = short_res / PI;  // pixels per radian
  float lon = pos.x / scale;
  float lat = -pos.y / scale;
  if (abs(lat) > PI * 0.5) return vec4(0.0, 0.0, 0.0, 0.0);

  vec3 d = vec3(-cos(lat) * cos(lon), -cos(lat) * sin(lon), -sin(lat));
  return vec4(d, 1.0);
}

// Overlay auxiliary lines on top of final_color.
// world_dir must be a valid world-space direction.
vec3 overlayAuxLines(vec3 world_dir, vec3 color) {
  float DEG = 180.0 / PI;
  float altitude_deg = asin(clamp(-world_dir.z, -1.0, 1.0)) * DEG;
  float azimuth_deg = atan(-world_dir.y, -world_dir.x) * DEG;  // [-180, 180]

  // Adaptive line width via screen-space derivatives, clamped to avoid singularities
  float fw_alt = clamp(fwidth(altitude_deg), 0.1, 2.0);
  float fw_az = clamp(fwidth(azimuth_deg), 0.1, 5.0);

  // Horizon line (altitude = 0)
  if (u_show_horizon != 0) {
    float d = abs(altitude_deg);
    float t = 1.0 - smoothstep(0.0, fw_alt * 1.5, d);
    color = mix(color, vec3(0.8, 0.2, 0.2), t * 0.6);
  }

  // Coordinate grid (10 degree intervals)
  if (u_show_grid != 0) {
    // Altitude grid lines
    float d_alt = mod(abs(altitude_deg) + 5.0, 10.0) - 5.0;
    float t_alt = 1.0 - smoothstep(0.0, fw_alt * 1.5, abs(d_alt));

    // Azimuth grid lines — suppress near poles to avoid fwidth divergence
    float t_az = 0.0;
    if (abs(altitude_deg) < 85.0) {
      float d_az = mod(azimuth_deg + 185.0, 10.0) - 5.0;  // +185 = +180 offset + 5 centering
      t_az = 1.0 - smoothstep(0.0, fw_az * 1.5, abs(d_az));
    }

    float t = max(t_alt, t_az);
    color = mix(color, vec3(1.0), t * 0.3);
  }

  // Sun angular distance circles
  if (u_show_sun_circles != 0) {
    float ang_dist_deg = acos(clamp(dot(world_dir, u_sun_dir), -1.0, 1.0)) * DEG;
    float fw_ang = clamp(fwidth(ang_dist_deg), 0.1, 2.0);
    for (int i = 0; i < u_sun_circle_count; i++) {
      float d = abs(ang_dist_deg - u_sun_circle_angles[i]);
      float t = 1.0 - smoothstep(0.0, fw_ang * 1.5, d);
      color = mix(color, vec3(1.0, 0.9, 0.3), t * 0.5);
    }
  }

  return color;
}

out vec4 frag_color;

void main() {
  vec2 pos = v_ndc * u_resolution * 0.5;  // Convert NDC [-1,1] to pixel offset from center
  float half_fov = u_fov * 0.5 * PI / 180.0;

  vec4 result;
  bool needs_view_transform = true;
  if (u_lens_type == 0) {
    result = linearInverse(pos, half_fov);
  } else if (u_lens_type >= 1 && u_lens_type <= 3) {
    result = fisheyeInverse(pos, half_fov, u_lens_type - 1);
  } else if (u_lens_type >= 4 && u_lens_type <= 6) {
    result = dualFisheyeInverse(pos, u_lens_type - 4);
    needs_view_transform = false;  // Core dual fisheye works in world space
  } else {
    result = rectangularInverse(pos);
    needs_view_transform = false;  // Core rectangular works in world space
  }

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
    if (u_visible == 0 && lat < 0.0) pixel_visible = false;
    if (u_visible == 1 && lat > 0.0) pixel_visible = false;

    if (pixel_visible) {
      vec3 tex_color = sampleDualFisheye(world_dir);
      if (u_xyz_mode == 1) {
        tex_color = xyzToSrgb(tex_color);
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
    final_color = overlayAuxLines(world_dir, final_color);
  }

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
  // Initialize with 1x1 black pixel so the texture has valid GL storage.
  // This allows the shader to sample black (transparent) when no simulation data exists,
  // enabling background-only rendering before the simulation is started.
  static const unsigned char kBlack[3] = { 0, 0, 0 };
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 1, 1, 0, GL_RGB, GL_UNSIGNED_BYTE, kBlack);
  glBindTexture(GL_TEXTURE_2D, 0);

  // Create background texture
  glGenTextures(1, &bg_texture_);
  glBindTexture(GL_TEXTURE_2D, bg_texture_);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glBindTexture(GL_TEXTURE_2D, 0);

  return true;
}

void PreviewRenderer::Destroy() {
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

  // Do NOT update tex_data_ (CPU copy) — XYZ float data is not suitable for .lmc save.
  // For .lmc save, call LUMICE_GetRenderResults() to get sRGB uint8.

  glBindTexture(GL_TEXTURE_2D, texture_);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 4);  // float data is 4-byte aligned

  if (width != tex_width_ || height != tex_height_ || !xyz_mode_) {
    // Re-allocate texture when switching from uint8 to float or size changed
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, width, height, 0, GL_RGB, GL_FLOAT, data);
    tex_width_ = width;
    tex_height_ = height;
  } else {
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_RGB, GL_FLOAT, data);
  }

  GLenum err = glGetError();
  if (err != GL_NO_ERROR) {
    GUI_LOG_WARNING("[GL] UploadXyzTexture: glGetError={} after {}x{} upload", err, width, height);
  }

  xyz_mode_ = true;
  // No glFinish() needed: glTexSubImage2D copies data to driver memory synchronously,
  // and subsequent draw calls implicitly synchronize. glFinish() was overly aggressive
  // and could cause GPU command queue stalls leading to TDR on Windows.
  glBindTexture(GL_TEXTURE_2D, 0);
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
static void BuildViewMatrix(float elevation_deg, float azimuth_deg, float roll_deg, float out[9]) {
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

void PreviewRenderer::Render(int vp_x, int vp_y, int vp_w, int vp_h, const PreviewParams& params) {
  if (!shader_program_ || !texture_ || vp_w <= 0 || vp_h <= 0) {
    return;
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
  glUniform1i(glGetUniformLocation(shader_program_, "u_lens_type"), params.lens_type);
  glUniform1f(glGetUniformLocation(shader_program_, "u_fov"), params.fov);
  glUniform1i(glGetUniformLocation(shader_program_, "u_visible"), params.visible);
  glUniform1i(glGetUniformLocation(shader_program_, "u_xyz_mode"), xyz_mode_ ? 1 : 0);
  glUniform1f(glGetUniformLocation(shader_program_, "u_intensity_scale"), params.intensity_scale);
  glUniform1f(glGetUniformLocation(shader_program_, "u_max_abs_dz"), params.max_abs_dz);
  glUniform1f(glGetUniformLocation(shader_program_, "u_r_scale"), params.r_scale);

  float view_matrix[9];
  BuildViewMatrix(params.elevation, params.azimuth, params.roll, view_matrix);
  glUniformMatrix3fv(glGetUniformLocation(shader_program_, "u_view_matrix"), 1, GL_FALSE, view_matrix);

  // Bind equirect texture to unit 0
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, texture_);
  glUniform1i(glGetUniformLocation(shader_program_, "u_texture"), 0);

  // Background image uniforms
  glUniform1i(glGetUniformLocation(shader_program_, "u_bg_enabled"), params.bg_enabled ? 1 : 0);
  if (params.bg_enabled) {
    glUniform1f(glGetUniformLocation(shader_program_, "u_overlay_alpha"), params.bg_alpha);

    // CPU-side contain mode UV calculation
    // bg_uv = v_ndc * scale + offset maps NDC [-1,1] to texture UV [0,1]
    // with contain fit (letterbox for aspect mismatch)
    float vp_aspect = static_cast<float>(vp_w) / static_cast<float>(vp_h);
    float sx = 1.0f;
    float sy = 1.0f;
    if (vp_aspect > params.bg_aspect) {
      sx = params.bg_aspect / vp_aspect;
    } else {
      sy = vp_aspect / params.bg_aspect;
    }
    float scale_x = 0.5f / sx;
    // Y flip: stbi loads top-down, GL texture origin is bottom-left
    float scale_y = -0.5f / sy;
    glUniform2f(glGetUniformLocation(shader_program_, "u_bg_uv_scale"), scale_x, scale_y);
    glUniform2f(glGetUniformLocation(shader_program_, "u_bg_uv_offset"), 0.5f, 0.5f);

    // Bind bg texture to unit 1
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, bg_texture_);
    glUniform1i(glGetUniformLocation(shader_program_, "u_bg_texture"), 1);
    glActiveTexture(GL_TEXTURE0);
  }

  // Auxiliary line overlay uniforms
  static_assert(kMaxSunCircles == 16, "Update shader u_sun_circle_angles[N] to match kMaxSunCircles");
  glUniform1i(glGetUniformLocation(shader_program_, "u_show_horizon"), params.show_horizon ? 1 : 0);
  glUniform1i(glGetUniformLocation(shader_program_, "u_show_grid"), params.show_grid ? 1 : 0);
  glUniform1i(glGetUniformLocation(shader_program_, "u_show_sun_circles"), params.show_sun_circles ? 1 : 0);
  glUniform3f(glGetUniformLocation(shader_program_, "u_sun_dir"), params.sun_dir[0], params.sun_dir[1],
              params.sun_dir[2]);
  glUniform1i(glGetUniformLocation(shader_program_, "u_sun_circle_count"), params.sun_circle_count);
  if (params.sun_circle_count > 0) {
    glUniform1fv(glGetUniformLocation(shader_program_, "u_sun_circle_angles"), params.sun_circle_count,
                 params.sun_circle_angles);
  }

  // Draw fullscreen quad
  glBindVertexArray(vao_);
  glDrawArrays(GL_TRIANGLES, 0, 6);

  // Restore state
  glBindVertexArray(static_cast<GLuint>(prev_vao));
  glUseProgram(static_cast<GLuint>(prev_program));
  glViewport(prev_viewport[0], prev_viewport[1], prev_viewport[2], prev_viewport[3]);
}

}  // namespace lumice::gui
