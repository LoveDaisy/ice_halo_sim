// Platform-specific OpenGL header inclusion.
// macOS: use native <OpenGL/gl3.h> (Core Profile 3.2+).
// Linux: use <GL/gl.h> + <GL/glext.h> with GL_GLEXT_PROTOTYPES
//        to get GL 3.x+ function declarations from Mesa's libGL.

#ifndef SRC_GUI_GL_COMMON_H_
#define SRC_GUI_GL_COMMON_H_

#ifdef __APPLE__
#ifndef GL_SILENCE_DEPRECATION
#define GL_SILENCE_DEPRECATION
#endif
#include <OpenGL/gl3.h>
#elif defined(__linux__)
#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glext.h>
#else
#error "Unsupported platform for GUI OpenGL. Build with -DBUILD_GUI=OFF."
#endif

#endif  // SRC_GUI_GL_COMMON_H_
