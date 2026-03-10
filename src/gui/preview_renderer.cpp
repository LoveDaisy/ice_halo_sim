#include "gui/preview_renderer.hpp"

#include <cmath>
#include <cstdio>

#ifdef __APPLE__
#include <OpenGL/gl3.h>
#else
#include <GL/gl.h>
#endif

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
uniform vec3 u_ray_color;
uniform vec3 u_background;

const float PI = 3.14159265358979323846;

// Convert world direction to equirectangular UV
vec2 dirToEquirect(vec3 d) {
  float lon = atan(-d.y, -d.x);
  float lat = asin(clamp(-d.z, -1.0, 1.0));
  return vec2(lon / (2.0 * PI) + 0.5, 0.5 - lat / PI);
}

// Compute view direction from pixel for linear projection
// Returns false (via w component) if outside valid range
vec4 linearInverse(vec2 pos, float half_fov) {
  float diag = length(u_resolution);
  float focal = diag * 0.5 / tan(half_fov);
  vec3 d = normalize(vec3(pos, -focal));
  return vec4(d, 1.0);
}

// Compute view direction for fisheye projections
// type: 0=equal_area, 1=equidistant, 2=stereographic
vec4 fisheyeInverse(vec2 pos, float half_fov, int type) {
  float img_radius = min(u_resolution.x, u_resolution.y) * 0.5;
  float r = length(pos) / img_radius;
  if (r > 1.0) return vec4(0.0, 0.0, 0.0, 0.0);

  float theta;
  if (type == 0) {        // equal area: r_norm = sin(θ/2) / sin(fov/4)
    float s = r * sin(half_fov * 0.5);
    if (s > 1.0) return vec4(0.0, 0.0, 0.0, 0.0);
    theta = 2.0 * asin(s);
  } else if (type == 1) { // equidistant: r_norm = θ / half_fov
    theta = r * half_fov;
  } else {                // stereographic: r_norm = tan(θ/2) / tan(fov/4)
    theta = 2.0 * atan(r * tan(half_fov * 0.5));
  }

  float phi = atan(pos.y, pos.x);
  return vec4(sin(theta) * cos(phi), sin(theta) * sin(phi), -cos(theta), 1.0);
}

// Dual fisheye: left circle = upper hemisphere, right circle = lower hemisphere
vec4 dualFisheyeInverse(vec2 pos, float half_fov, int type) {
  float circle_radius = min(u_resolution.x * 0.5, u_resolution.y) * 0.5;
  float quarter_w = u_resolution.x * 0.25;

  // Left circle center: (quarter_w, 0), Right circle center: (-quarter_w, 0)
  // pos is already centered at viewport center
  vec2 left_pos = pos - vec2(-quarter_w, 0.0);
  vec2 right_pos = pos - vec2(quarter_w, 0.0);

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

  float phi = atan(use_pos.y, use_pos.x);
  vec3 d = vec3(sin(theta) * cos(phi), sin(theta) * sin(phi), -cos(theta));

  // Left = upper hemisphere (flip z for lower)
  if (!in_left) {
    d.z = -d.z;
  }

  return vec4(d, 1.0);
}

// Rectangular (equirectangular crop)
vec4 rectangularInverse(vec2 pos, float half_fov) {
  // FOV is diagonal-based
  float diag = length(u_resolution);
  float scale = diag / (2.0 * half_fov);  // pixels per radian
  float lon = pos.x / scale;
  float lat = -pos.y / scale;
  if (abs(lat) > PI * 0.5) return vec4(0.0, 0.0, 0.0, 0.0);

  vec3 d = vec3(-cos(lat) * cos(lon), -cos(lat) * sin(lon), -sin(lat));
  return vec4(d, 1.0);
}

out vec4 frag_color;

void main() {
  vec2 pos = v_ndc * u_resolution * 0.5;  // Convert NDC [-1,1] to pixel offset from center
  float half_fov = u_fov * 0.5 * PI / 180.0;

  vec4 result;
  if (u_lens_type == 0) {
    result = linearInverse(pos, half_fov);
  } else if (u_lens_type >= 1 && u_lens_type <= 3) {
    result = fisheyeInverse(pos, half_fov, u_lens_type - 1);
  } else if (u_lens_type >= 4 && u_lens_type <= 6) {
    result = dualFisheyeInverse(pos, half_fov, u_lens_type - 4);
  } else {
    result = rectangularInverse(pos, half_fov);
  }

  if (result.w < 0.5) {
    frag_color = vec4(u_background, 1.0);
    return;
  }

  vec3 view_dir = result.xyz;
  vec3 world_dir = u_view_matrix * view_dir;

  // Visible hemisphere check
  // In equirect convention: lat = asin(-dz), lat > 0 means upper sky
  float lat = asin(clamp(-world_dir.z, -1.0, 1.0));
  if (u_visible == 0 && lat < 0.0) {
    frag_color = vec4(u_background, 1.0);
    return;
  }
  if (u_visible == 1 && lat > 0.0) {
    frag_color = vec4(u_background, 1.0);
    return;
  }

  vec2 uv = dirToEquirect(world_dir);
  vec3 tex_color = texture(u_texture, uv).rgb;
  vec3 final_color = tex_color * u_ray_color + u_background * (1.0 - tex_color);
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
    fprintf(stderr, "Shader compile error: %s\n", log);
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
    fprintf(stderr, "Shader link error: %s\n", log);
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

  // Create texture
  glGenTextures(1, &texture_);
  glBindTexture(GL_TEXTURE_2D, texture_);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glBindTexture(GL_TEXTURE_2D, 0);

  return true;
}

void PreviewRenderer::Destroy() {
  if (texture_) {
    glDeleteTextures(1, &texture_);
    texture_ = 0;
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
}

void PreviewRenderer::UploadTexture(const unsigned char* data, int width, int height) {
  if (!texture_ || !data) {
    return;
  }

  glBindTexture(GL_TEXTURE_2D, texture_);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);  // RGB data may not be 4-byte aligned

  if (width != tex_width_ || height != tex_height_) {
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
    tex_width_ = width;
    tex_height_ = height;
  } else {
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, data);
  }

  glBindTexture(GL_TEXTURE_2D, 0);
}

// Build view-to-world rotation matrix from elevation, azimuth, roll (degrees)
// Convention: default camera looks at direction (-1, 0, 0), up is (0, 0, -1)
// Azimuth rotates around Z axis, elevation tilts up/down, roll around view axis
static void BuildViewMatrix(float elevation_deg, float azimuth_deg, float roll_deg, float out[9]) {
  constexpr float kDeg2Rad = 3.14159265358979323846f / 180.0f;
  float e = elevation_deg * kDeg2Rad;
  float a = azimuth_deg * kDeg2Rad;
  float r = roll_deg * kDeg2Rad;

  float ce = std::cos(e), se = std::sin(e);
  float ca = std::cos(a), sa = std::sin(a);
  float cr = std::cos(r), sr = std::sin(r);

  // Camera basis vectors (before roll):
  // forward = (-cos(e)*cos(a), -cos(e)*sin(a), -sin(e))
  // right   = (sin(a), -cos(a), 0)
  // up      = (-sin(e)*cos(a), -sin(e)*sin(a), cos(e))

  float fx = -ce * ca, fy = -ce * sa, fz = -se;
  float rx = sa, ry = -ca, rz = 0.0f;
  float ux = -se * ca, uy = -se * sa, uz = ce;

  // Apply roll: rotate right and up around forward axis
  float rx2 = cr * rx + sr * ux;
  float ry2 = cr * ry + sr * uy;
  float rz2 = cr * rz + sr * uz;
  float ux2 = -sr * rx + cr * ux;
  float uy2 = -sr * ry + cr * uy;
  float uz2 = -sr * rz + cr * uz;

  // The view-to-world matrix maps shader view space to world space:
  // In shader: forward=(0,0,-1), right=(1,0,0), up=(0,1,0)
  // Column 0: where shader-X (right) goes in world
  // Column 1: where shader-Y (up) goes in world
  // Column 2: where shader-Z goes in world (shader forward is -Z, so col2 = -forward)

  // OpenGL uniform mat3 is column-major
  out[0] = rx2;
  out[1] = ry2;
  out[2] = rz2;  // column 0: right
  out[3] = ux2;
  out[4] = uy2;
  out[5] = uz2;  // column 1: up
  out[6] = -fx;
  out[7] = -fy;
  out[8] = -fz;  // column 2: -forward
}

void PreviewRenderer::Render(int vp_x, int vp_y, int vp_w, int vp_h, const PreviewParams& params) {
  if (!shader_program_ || !texture_) {
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
  glUniform3f(glGetUniformLocation(shader_program_, "u_ray_color"), params.ray_color[0], params.ray_color[1],
              params.ray_color[2]);
  glUniform3f(glGetUniformLocation(shader_program_, "u_background"), params.background[0], params.background[1],
              params.background[2]);

  float view_matrix[9];
  BuildViewMatrix(params.elevation, params.azimuth, params.roll, view_matrix);
  glUniformMatrix3fv(glGetUniformLocation(shader_program_, "u_view_matrix"), 1, GL_FALSE, view_matrix);

  // Bind texture
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, texture_);
  glUniform1i(glGetUniformLocation(shader_program_, "u_texture"), 0);

  // Draw fullscreen quad
  glBindVertexArray(vao_);
  glDrawArrays(GL_TRIANGLES, 0, 6);

  // Restore state
  glBindVertexArray(static_cast<GLuint>(prev_vao));
  glUseProgram(static_cast<GLuint>(prev_program));
  glViewport(prev_viewport[0], prev_viewport[1], prev_viewport[2], prev_viewport[3]);
}

}  // namespace lumice::gui
