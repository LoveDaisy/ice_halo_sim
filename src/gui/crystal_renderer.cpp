#include "gui/crystal_renderer.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>

#include "gui/gl_common.h"
#include "util/logger.hpp"

namespace lumice::gui {

// clang-format off

// ---- Edge shader: VS + GS + FS (geometry shader computes screen-space line distance for dashing) ----
static const char* const kEdgeVS = R"glsl(
#version 330 core
layout(location = 0) in vec3 a_pos;
uniform mat4 u_mvp;
void main() {
  gl_Position = u_mvp * vec4(a_pos, 1.0);
}
)glsl";

static const char* const kEdgeGS = R"glsl(
#version 330 core
layout(lines) in;
layout(line_strip, max_vertices = 2) out;
noperspective out float v_line_dist;
uniform vec2 u_viewport;
void main() {
  vec2 sp0 = (gl_in[0].gl_Position.xy / gl_in[0].gl_Position.w + 1.0) * 0.5 * u_viewport;
  vec2 sp1 = (gl_in[1].gl_Position.xy / gl_in[1].gl_Position.w + 1.0) * 0.5 * u_viewport;
  float seg_len = length(sp1 - sp0);
  gl_Position = gl_in[0].gl_Position;
  v_line_dist = 0.0;
  EmitVertex();
  gl_Position = gl_in[1].gl_Position;
  v_line_dist = seg_len;
  EmitVertex();
  EndPrimitive();
}
)glsl";

static const char* const kEdgeFS = R"glsl(
#version 330 core
noperspective in float v_line_dist;
uniform vec3 u_color;
uniform int u_dashed;
out vec4 frag_color;
void main() {
  if (u_dashed != 0) {
    float pattern = mod(v_line_dist, 8.0);
    if (pattern > 4.0) discard;
  }
  frag_color = vec4(u_color, 1.0);
}
)glsl";

// ---- Face shader: VS + FS (flat shading via dFdx/dFdy) ----
static const char* const kFaceVS = R"glsl(
#version 330 core
layout(location = 0) in vec3 a_pos;
uniform mat4 u_mvp;
uniform mat4 u_rotation;
out vec3 v_eye_pos;
void main() {
  gl_Position = u_mvp * vec4(a_pos, 1.0);
  v_eye_pos = (u_rotation * vec4(a_pos, 1.0)).xyz;
}
)glsl";

static const char* const kFaceFS = R"glsl(
#version 330 core
in vec3 v_eye_pos;
out vec4 frag_color;
void main() {
  vec3 n = normalize(cross(dFdx(v_eye_pos), dFdy(v_eye_pos)));
  // Ensure normal faces viewer (eye-space camera looks along -Z)
  if (n.z < 0.0) n = -n;
  // Light from upper-left-front in eye space
  vec3 light = normalize(vec3(-0.5, 0.7, 1.0));
  float ndl = max(dot(n, light), 0.0);
  float ambient = 0.15;
  float diffuse = 0.75 * ndl;
  // Specular (Blinn-Phong)
  vec3 view_dir = vec3(0.0, 0.0, 1.0);
  vec3 half_dir = normalize(light + view_dir);
  float spec = pow(max(dot(n, half_dir), 0.0), 32.0);
  vec3 color = vec3(0.55, 0.70, 0.85);  // ice blue
  frag_color = vec4(color * (ambient + diffuse) + vec3(0.3 * spec), 1.0);
}
)glsl";

// clang-format on

static unsigned int CompileShader(unsigned int type, const char* source) {
  unsigned int shader = glCreateShader(type);
  glShaderSource(shader, 1, &source, nullptr);
  glCompileShader(shader);
  int success = 0;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
  if (!success) {
    char log[512];
    glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
    LOG_ERROR("Crystal shader compile error: {}", log);
    glDeleteShader(shader);
    return 0;
  }
  return shader;
}

static unsigned int LinkProgram(unsigned int vs, unsigned int gs, unsigned int fs) {
  unsigned int prog = glCreateProgram();
  glAttachShader(prog, vs);
  if (gs) {
    glAttachShader(prog, gs);
  }
  glAttachShader(prog, fs);
  glLinkProgram(prog);
  int success = 0;
  glGetProgramiv(prog, GL_LINK_STATUS, &success);
  if (!success) {
    char log[512];
    glGetProgramInfoLog(prog, sizeof(log), nullptr, log);
    LOG_ERROR("Crystal shader link error: {}", log);
    glDeleteProgram(prog);
    return 0;
  }
  return prog;
}

bool CrystalRenderer::Init(int width, int height) {
  width_ = width;
  height_ = height;

  // Compile edge shader (VS + GS + FS)
  unsigned int edge_vs = CompileShader(GL_VERTEX_SHADER, kEdgeVS);
  unsigned int edge_gs = CompileShader(GL_GEOMETRY_SHADER, kEdgeGS);
  unsigned int edge_fs = CompileShader(GL_FRAGMENT_SHADER, kEdgeFS);
  if (!edge_vs || !edge_gs || !edge_fs) {
    return false;
  }
  edge_shader_ = LinkProgram(edge_vs, edge_gs, edge_fs);
  glDeleteShader(edge_vs);
  glDeleteShader(edge_gs);
  glDeleteShader(edge_fs);
  if (!edge_shader_) {
    return false;
  }

  // Compile face shader (VS + FS)
  unsigned int face_vs = CompileShader(GL_VERTEX_SHADER, kFaceVS);
  unsigned int face_fs = CompileShader(GL_FRAGMENT_SHADER, kFaceFS);
  if (!face_vs || !face_fs) {
    return false;
  }
  face_shader_ = LinkProgram(face_vs, 0, face_fs);
  glDeleteShader(face_vs);
  glDeleteShader(face_fs);
  if (!face_shader_) {
    return false;
  }

  constexpr int kMsaaSamples = 4;

  // Multisample FBO (render target)
  glGenFramebuffers(1, &fbo_);
  glBindFramebuffer(GL_FRAMEBUFFER, fbo_);

  glGenRenderbuffers(1, &ms_color_rb_);
  glBindRenderbuffer(GL_RENDERBUFFER, ms_color_rb_);
  glRenderbufferStorageMultisample(GL_RENDERBUFFER, kMsaaSamples, GL_RGBA8, width, height);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, ms_color_rb_);

  glGenRenderbuffers(1, &ms_depth_rb_);
  glBindRenderbuffer(GL_RENDERBUFFER, ms_depth_rb_);
  glRenderbufferStorageMultisample(GL_RENDERBUFFER, kMsaaSamples, GL_DEPTH_COMPONENT24, width, height);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, ms_depth_rb_);

  if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
    LOG_ERROR("Crystal MSAA FBO incomplete");
    return false;
  }

  // Resolve FBO (non-multisample, for ImGui display)
  glGenFramebuffers(1, &resolve_fbo_);
  glBindFramebuffer(GL_FRAMEBUFFER, resolve_fbo_);

  glGenTextures(1, &color_tex_);
  glBindTexture(GL_TEXTURE_2D, color_tex_);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, color_tex_, 0);

  if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
    LOG_ERROR("Crystal resolve FBO incomplete");
    return false;
  }

  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  // VAO/VBO/EBOs
  glGenVertexArrays(1, &vao_);
  glGenBuffers(1, &vbo_);
  glGenBuffers(1, &front_ebo_);
  glGenBuffers(1, &back_ebo_);
  glGenBuffers(1, &tri_ebo_);

  glBindVertexArray(vao_);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
  glBindVertexArray(0);

  return true;
}

void CrystalRenderer::Destroy() {
  if (tri_ebo_)
    glDeleteBuffers(1, &tri_ebo_);
  if (back_ebo_)
    glDeleteBuffers(1, &back_ebo_);
  if (front_ebo_)
    glDeleteBuffers(1, &front_ebo_);
  if (vbo_)
    glDeleteBuffers(1, &vbo_);
  if (vao_)
    glDeleteVertexArrays(1, &vao_);
  if (ms_depth_rb_)
    glDeleteRenderbuffers(1, &ms_depth_rb_);
  if (ms_color_rb_)
    glDeleteRenderbuffers(1, &ms_color_rb_);
  if (color_tex_)
    glDeleteTextures(1, &color_tex_);
  if (resolve_fbo_)
    glDeleteFramebuffers(1, &resolve_fbo_);
  if (fbo_)
    glDeleteFramebuffers(1, &fbo_);
  if (face_shader_)
    glDeleteProgram(face_shader_);
  if (edge_shader_)
    glDeleteProgram(edge_shader_);
  tri_ebo_ = back_ebo_ = front_ebo_ = vbo_ = vao_ = 0;
  ms_depth_rb_ = ms_color_rb_ = color_tex_ = resolve_fbo_ = fbo_ = 0;
  face_shader_ = edge_shader_ = 0;
  vertices_.clear();
  vertex_count_ = 0;
  total_edge_count_ = 0;
  tri_count_ = 0;
}

void CrystalRenderer::UpdateMesh(const float* vertices, int vertex_count, const int* edges, int edge_count,
                                 const int* triangles, int triangle_count, const float* edge_face_normals) {
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glBufferData(GL_ARRAY_BUFFER, vertex_count * 3 * sizeof(float), vertices, GL_DYNAMIC_DRAW);

  // Store vertex and edge data for CPU-side classification
  vertex_count_ = vertex_count;
  vertices_.assign(vertices, vertices + vertex_count * 3);
  total_edge_count_ = edge_count;
  all_edges_.assign(edges, edges + edge_count * 2);
  edge_face_normals_.assign(edge_face_normals, edge_face_normals + edge_count * 6);

  // Upload triangle indices
  tri_count_ = triangle_count;
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, tri_ebo_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, triangle_count * 3 * sizeof(int), triangles, GL_DYNAMIC_DRAW);
}

void CrystalRenderer::Render(const float rotation[16], float zoom, CrystalStyle style) {
  if (total_edge_count_ <= 0) {
    return;
  }

  // Save state
  GLint prev_fbo = 0;
  glGetIntegerv(GL_FRAMEBUFFER_BINDING, &prev_fbo);
  GLint prev_viewport[4];
  glGetIntegerv(GL_VIEWPORT, prev_viewport);
  GLboolean prev_depth = glIsEnabled(GL_DEPTH_TEST);
  GLboolean prev_cull = glIsEnabled(GL_CULL_FACE);

  // Bind FBO
  glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
  glViewport(0, 0, width_, height_);
  glClearColor(0.15f, 0.15f, 0.15f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_DEPTH_TEST);

  // Build perspective projection
  float aspect = static_cast<float>(width_) / static_cast<float>(height_);
  constexpr float kFovDeg = 30.0f;
  constexpr float kDeg2Rad = 3.14159265358979323846f / 180.0f;
  float fov_rad = kFovDeg * kDeg2Rad;
  float half_tan = std::tan(fov_rad * 0.5f);
  float dist = zoom / half_tan;
  float near_plane = std::max(dist - 10.0f, 0.1f);
  float far_plane = dist + 10.0f;
  float f = 1.0f / half_tan;

  // Perspective matrix (column-major)
  float proj[16] = {};
  proj[0] = f / aspect;
  proj[5] = f;
  proj[10] = -(far_plane + near_plane) / (far_plane - near_plane);
  proj[11] = -1.0f;
  proj[14] = -2.0f * far_plane * near_plane / (far_plane - near_plane);

  // View translation: translate(0, 0, -dist)
  // Combined: view_rot = rotation with translation applied
  float view_rot[16];
  std::memcpy(view_rot, rotation, 16 * sizeof(float));
  // Set translation column: (0, 0, -dist)
  view_rot[12] = 0.0f;
  view_rot[13] = 0.0f;
  view_rot[14] = -dist;
  view_rot[15] = 1.0f;

  // MVP = proj * view_rot
  float mvp[16] = {};
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      float sum = 0.0f;
      for (int k = 0; k < 4; k++) {
        sum += proj[i + k * 4] * view_rot[k + j * 4];
      }
      mvp[i + j * 4] = sum;
    }
  }

  // Classify edges: front vs back using perspective-correct face normal test.
  // In eye space, camera is at origin. A face is front-facing if dot(n_eye, -p_eye) > 0,
  // where p_eye is the edge midpoint in eye space and n_eye is the rotated face normal.
  std::vector<int> front_edges;
  std::vector<int> back_edges;
  front_edges.reserve(total_edge_count_ * 2);
  back_edges.reserve(total_edge_count_ * 2);

  for (int i = 0; i < total_edge_count_; i++) {
    int v0 = all_edges_[i * 2 + 0];
    int v1 = all_edges_[i * 2 + 1];

    // Compute edge midpoint in object space
    float mx = (vertices_[v0 * 3 + 0] + vertices_[v1 * 3 + 0]) * 0.5f;
    float my = (vertices_[v0 * 3 + 1] + vertices_[v1 * 3 + 1]) * 0.5f;
    float mz = (vertices_[v0 * 3 + 2] + vertices_[v1 * 3 + 2]) * 0.5f;

    // Transform midpoint to eye space using view_rot (4x4, includes translation (0,0,-dist))
    float px = view_rot[0] * mx + view_rot[4] * my + view_rot[8] * mz + view_rot[12];
    float py = view_rot[1] * mx + view_rot[5] * my + view_rot[9] * mz + view_rot[13];
    float pz = view_rot[2] * mx + view_rot[6] * my + view_rot[10] * mz + view_rot[14];

    const float* n0 = &edge_face_normals_[i * 6 + 0];
    const float* n1 = &edge_face_normals_[i * 6 + 3];

    // Rotate normals to eye space (3x3 part of rotation, no translation for direction vectors)
    float rn0x = rotation[0] * n0[0] + rotation[4] * n0[1] + rotation[8] * n0[2];
    float rn0y = rotation[1] * n0[0] + rotation[5] * n0[1] + rotation[9] * n0[2];
    float rn0z = rotation[2] * n0[0] + rotation[6] * n0[1] + rotation[10] * n0[2];
    float rn1x = rotation[0] * n1[0] + rotation[4] * n1[1] + rotation[8] * n1[2];
    float rn1y = rotation[1] * n1[0] + rotation[5] * n1[1] + rotation[9] * n1[2];
    float rn1z = rotation[2] * n1[0] + rotation[6] * n1[1] + rotation[10] * n1[2];

    // Front-facing test: dot(n_eye, -p_eye) > 0, i.e. dot(n_eye, p_eye) < 0
    bool face0_front = (rn0x * px + rn0y * py + rn0z * pz) < 0.0f;
    bool face1_front = (rn1x * px + rn1y * py + rn1z * pz) < 0.0f;

    if (face0_front || face1_front) {
      front_edges.push_back(v0);
      front_edges.push_back(v1);
    } else {
      back_edges.push_back(v0);
      back_edges.push_back(v1);
    }
  }

  int front_count = static_cast<int>(front_edges.size()) / 2;
  int back_count = static_cast<int>(back_edges.size()) / 2;

  // Upload edge indices
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, front_ebo_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, front_edges.size() * sizeof(int), front_edges.data(), GL_STREAM_DRAW);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, back_ebo_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, back_edges.size() * sizeof(int), back_edges.data(), GL_STREAM_DRAW);

  glBindVertexArray(vao_);
  float viewport[2] = { static_cast<float>(width_), static_cast<float>(height_) };

  // --- Draw faces (Shaded mode) ---
  if (style == CrystalStyle::kShaded && tri_count_ > 0) {
    glUseProgram(face_shader_);
    glUniformMatrix4fv(glGetUniformLocation(face_shader_, "u_mvp"), 1, GL_FALSE, mvp);
    glUniformMatrix4fv(glGetUniformLocation(face_shader_, "u_rotation"), 1, GL_FALSE, rotation);

    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, tri_ebo_);
    glDrawElements(GL_TRIANGLES, tri_count_ * 3, GL_UNSIGNED_INT, nullptr);

    glDisable(GL_CULL_FACE);
  }

  // --- Draw edges ---
  glUseProgram(edge_shader_);
  glUniformMatrix4fv(glGetUniformLocation(edge_shader_, "u_mvp"), 1, GL_FALSE, mvp);
  glUniform2fv(glGetUniformLocation(edge_shader_, "u_viewport"), 1, viewport);

  bool draw_all_solid = (style == CrystalStyle::kWireframe);
  bool draw_front = true;
  bool draw_back = (style == CrystalStyle::kWireframe || style == CrystalStyle::kXRay);
  bool back_dashed = (style == CrystalStyle::kXRay);

  if (draw_all_solid) {
    // Wireframe: draw all edges as solid, no classification needed
    // Re-upload all edges to front_ebo for simplicity
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, front_ebo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, all_edges_.size() * sizeof(int), all_edges_.data(), GL_STREAM_DRAW);
    glUniform3f(glGetUniformLocation(edge_shader_, "u_color"), 0.7f, 0.85f, 1.0f);
    glUniform1i(glGetUniformLocation(edge_shader_, "u_dashed"), 0);
    glDrawElements(GL_LINES, total_edge_count_ * 2, GL_UNSIGNED_INT, nullptr);
  } else {
    // Front edges (solid)
    if (draw_front && front_count > 0) {
      float edge_r = 0.7f, edge_g = 0.85f, edge_b = 1.0f;
      if (style == CrystalStyle::kShaded) {
        // Darker edges on shaded surface
        edge_r = 0.3f;
        edge_g = 0.4f;
        edge_b = 0.55f;
      }
      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, front_ebo_);
      glUniform3f(glGetUniformLocation(edge_shader_, "u_color"), edge_r, edge_g, edge_b);
      glUniform1i(glGetUniformLocation(edge_shader_, "u_dashed"), 0);
      glDrawElements(GL_LINES, front_count * 2, GL_UNSIGNED_INT, nullptr);
    }

    // Back edges (dashed or solid, dimmer color)
    if (draw_back && back_count > 0) {
      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, back_ebo_);
      glUniform3f(glGetUniformLocation(edge_shader_, "u_color"), 0.35f, 0.42f, 0.5f);
      glUniform1i(glGetUniformLocation(edge_shader_, "u_dashed"), back_dashed ? 1 : 0);
      glDrawElements(GL_LINES, back_count * 2, GL_UNSIGNED_INT, nullptr);
    }
  }

  // Resolve MSAA: blit from multisample FBO to resolve FBO
  glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo_);
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, resolve_fbo_);
  glBlitFramebuffer(0, 0, width_, height_, 0, 0, width_, height_, GL_COLOR_BUFFER_BIT, GL_LINEAR);

  // Restore state
  glBindFramebuffer(GL_FRAMEBUFFER, static_cast<GLuint>(prev_fbo));
  glViewport(prev_viewport[0], prev_viewport[1], prev_viewport[2], prev_viewport[3]);
  if (!prev_depth) {
    glDisable(GL_DEPTH_TEST);
  }
  if (prev_cull) {
    glEnable(GL_CULL_FACE);
  }
}

uintptr_t CrystalRenderer::GetTextureId() const {
  return static_cast<uintptr_t>(color_tex_);
}

}  // namespace lumice::gui
