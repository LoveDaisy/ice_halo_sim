#include "gui/gl_init.h"

#ifdef _WIN32
#include <GLFW/glfw3.h>

#include "gui/gl_common.h"
#include "gui/gui_logger.hpp"
#endif

namespace lumice::gui {

bool InitGLLoader() {
#ifdef _WIN32
  int version = gladLoadGL(glfwGetProcAddress);
  if (version == 0) {
    GUI_LOG_ERROR("Failed to initialize OpenGL loader (GLAD)");
    return false;
  }
  return true;
#else
  return true;
#endif
}

}  // namespace lumice::gui
