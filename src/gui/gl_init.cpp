#include "gui/gl_init.h"

#ifdef _WIN32
#include <GLFW/glfw3.h>

#include <cstdio>

#include "gui/gl_common.h"
#endif

namespace lumice::gui {

bool InitGLLoader() {
#ifdef _WIN32
  int version = gladLoadGL(glfwGetProcAddress);
  if (version == 0) {
    fprintf(stderr, "Failed to initialize OpenGL loader (GLAD)\n");
    return false;
  }
  return true;
#else
  return true;
#endif
}

}  // namespace lumice::gui
