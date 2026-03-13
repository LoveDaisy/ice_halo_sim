// Platform-specific OpenGL loader initialization.
// Windows: calls gladLoadGL to load GL 3.3 Core Profile function pointers.
// macOS/Linux: no-op (GL functions available via native headers).

#ifndef SRC_GUI_GL_INIT_H_
#define SRC_GUI_GL_INIT_H_

namespace lumice::gui {

// Initialize the OpenGL function loader. Must be called after glfwMakeContextCurrent()
// and before any GL calls. Returns true on success, false on failure.
bool InitGLLoader();

}  // namespace lumice::gui

#endif  // SRC_GUI_GL_INIT_H_
