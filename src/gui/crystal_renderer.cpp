#include "gui/crystal_renderer.hpp"

#include <cstdio>
#include <cstring>

#ifdef __APPLE__
#include <OpenGL/gl3.h>
#else
#include <GL/gl.h>
#endif

namespace lumice::gui {

// clang-format off
static const char* const kCrystalVS = R"glsl(
#version 330 core
layout(location = 0) in vec3 a_pos;
uniform mat4 u_mvp;
void main() {
  gl_Position = u_mvp * vec4(a_pos, 1.0);
}
)glsl";

static const char* const kCrystalFS = R"glsl(
#version 330 core
uniform vec3 u_color;
out vec4 frag_color;
void main() {
  frag_color = vec4(u_color, 1.0);
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
    fprintf(stderr, "Crystal shader compile error: %s\n", log);
    glDeleteShader(shader);
    return 0;
  }
  return shader;
}

bool CrystalRenderer::Init(int width, int height) {
  width_ = width;
  height_ = height;

  // Compile shader
  unsigned int vs = CompileShader(GL_VERTEX_SHADER, kCrystalVS);
  unsigned int fs = CompileShader(GL_FRAGMENT_SHADER, kCrystalFS);
  if (!vs || !fs) {
    return false;
  }

  shader_ = glCreateProgram();
  glAttachShader(shader_, vs);
  glAttachShader(shader_, fs);
  glLinkProgram(shader_);

  int success = 0;
  glGetProgramiv(shader_, GL_LINK_STATUS, &success);
  if (!success) {
    char log[512];
    glGetProgramInfoLog(shader_, sizeof(log), nullptr, log);
    fprintf(stderr, "Crystal shader link error: %s\n", log);
    return false;
  }
  glDeleteShader(vs);
  glDeleteShader(fs);

  // Create FBO
  glGenFramebuffers(1, &fbo_);
  glBindFramebuffer(GL_FRAMEBUFFER, fbo_);

  // Color texture
  glGenTextures(1, &color_tex_);
  glBindTexture(GL_TEXTURE_2D, color_tex_);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, color_tex_, 0);

  // Depth renderbuffer
  glGenRenderbuffers(1, &depth_rb_);
  glBindRenderbuffer(GL_RENDERBUFFER, depth_rb_);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width, height);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth_rb_);

  if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
    fprintf(stderr, "Crystal FBO incomplete\n");
    return false;
  }

  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  // VAO/VBO/EBO
  glGenVertexArrays(1, &vao_);
  glGenBuffers(1, &vbo_);
  glGenBuffers(1, &ebo_);

  glBindVertexArray(vao_);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
  glBindVertexArray(0);

  return true;
}

void CrystalRenderer::Destroy() {
  if (ebo_)
    glDeleteBuffers(1, &ebo_);
  if (vbo_)
    glDeleteBuffers(1, &vbo_);
  if (vao_)
    glDeleteVertexArrays(1, &vao_);
  if (depth_rb_)
    glDeleteRenderbuffers(1, &depth_rb_);
  if (color_tex_)
    glDeleteTextures(1, &color_tex_);
  if (fbo_)
    glDeleteFramebuffers(1, &fbo_);
  if (shader_)
    glDeleteProgram(shader_);
  ebo_ = vbo_ = vao_ = depth_rb_ = color_tex_ = fbo_ = shader_ = 0;
  edge_count_ = 0;
}

void CrystalRenderer::UpdateMesh(const float* vertices, int vertex_count, const int* edges, int edge_count) {
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glBufferData(GL_ARRAY_BUFFER, vertex_count * 3 * sizeof(float), vertices, GL_DYNAMIC_DRAW);

  // Convert edge pairs to element indices for GL_LINES
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, edge_count * 2 * sizeof(int), edges, GL_DYNAMIC_DRAW);

  edge_count_ = edge_count;
}

void CrystalRenderer::Render(const float rotation[16], float zoom) {
  if (edge_count_ <= 0) {
    return;
  }

  // Save state
  GLint prev_fbo = 0;
  glGetIntegerv(GL_FRAMEBUFFER_BINDING, &prev_fbo);
  GLint prev_viewport[4];
  glGetIntegerv(GL_VIEWPORT, prev_viewport);
  GLboolean prev_depth = glIsEnabled(GL_DEPTH_TEST);

  // Bind FBO
  glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
  glViewport(0, 0, width_, height_);
  glClearColor(0.15f, 0.15f, 0.15f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_DEPTH_TEST);

  glUseProgram(shader_);

  // Build orthographic projection: [-zoom, zoom] in each axis
  float aspect = static_cast<float>(width_) / static_cast<float>(height_);
  float l = -zoom * aspect;
  float r_bound = zoom * aspect;
  float b = -zoom;
  float t = zoom;
  float n = -10.0f;
  float f = 10.0f;

  // Ortho projection matrix (column-major)
  float proj[16] = {};
  proj[0] = 2.0f / (r_bound - l);
  proj[5] = 2.0f / (t - b);
  proj[10] = -2.0f / (f - n);
  proj[12] = -(r_bound + l) / (r_bound - l);
  proj[13] = -(t + b) / (t - b);
  proj[14] = -(f + n) / (f - n);
  proj[15] = 1.0f;

  // MVP = projection * rotation (model is identity)
  float mvp[16] = {};
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      float sum = 0.0f;
      for (int k = 0; k < 4; k++) {
        sum += proj[i + k * 4] * rotation[k + j * 4];
      }
      mvp[i + j * 4] = sum;
    }
  }

  glUniformMatrix4fv(glGetUniformLocation(shader_, "u_mvp"), 1, GL_FALSE, mvp);
  glUniform3f(glGetUniformLocation(shader_, "u_color"), 0.7f, 0.85f, 1.0f);

  glBindVertexArray(vao_);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
  glDrawElements(GL_LINES, edge_count_ * 2, GL_UNSIGNED_INT, nullptr);

  // Restore state
  glBindFramebuffer(GL_FRAMEBUFFER, static_cast<GLuint>(prev_fbo));
  glViewport(prev_viewport[0], prev_viewport[1], prev_viewport[2], prev_viewport[3]);
  if (!prev_depth) {
    glDisable(GL_DEPTH_TEST);
  }
}

uintptr_t CrystalRenderer::GetTextureId() const {
  return static_cast<uintptr_t>(color_tex_);
}

}  // namespace lumice::gui
