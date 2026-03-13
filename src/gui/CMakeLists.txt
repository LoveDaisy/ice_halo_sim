# ==================================================================================================
# GUI target: LumiceGUI
message(STATUS "[gui] Building GUI application")

# imgui has no CMake support — manually list source files
set(IMGUI_DIR ${imgui_SOURCE_DIR})
set(IMGUI_SOURCES
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_demo.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp)

# OBJECT library: shared between LumiceGUI and LumiceGUITests
add_library(lumice_gui_obj OBJECT
    ${PROJ_SRC_DIR}/gui/app.cpp
    ${PROJ_SRC_DIR}/gui/panels.cpp
    ${PROJ_SRC_DIR}/gui/file_io.cpp
    ${PROJ_SRC_DIR}/gui/preview_renderer.cpp
    ${PROJ_SRC_DIR}/gui/crystal_renderer.cpp
    ${PROJ_SRC_DIR}/gui/gl_init.cpp
    ${PROJ_SRC_DIR}/gui/stb_impl.cpp
    ${IMGUI_SOURCES})

target_include_directories(lumice_gui_obj PUBLIC
    ${PROJ_SRC_DIR}
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
    ${stb_SOURCE_DIR})

if(WIN32)
  target_link_libraries(lumice_gui_obj PUBLIC
      lumice
      glfw
      nfd
      glad_gl)
  # IMGUI_IMPL_OPENGL_LOADER_CUSTOM: skip ImGui's built-in imgl3w loader, use GLAD instead
  target_compile_definitions(lumice_gui_obj PUBLIC IMGUI_IMPL_OPENGL_LOADER_CUSTOM)
else()
  target_link_libraries(lumice_gui_obj PUBLIC
      lumice
      glfw
      nfd
      OpenGL::GL)
endif()

# GLFW_INCLUDE_NONE: prevent GLFW from including GL headers — gl_common.h provides them
# per platform (macOS: gl3.h, Linux: GL/gl.h+glext.h, Windows: glad/gl.h)
target_compile_definitions(lumice_gui_obj PUBLIC GLFW_INCLUDE_NONE)

target_compile_definitions(lumice_gui_obj PUBLIC IMGUI_ENABLE_TEST_ENGINE)

if(APPLE)
  target_compile_definitions(lumice_gui_obj PRIVATE GL_SILENCE_DEPRECATION)
endif()

if(NOT MSVC)
  target_compile_options(lumice_gui_obj PRIVATE -Wall -Wextra)
endif()

# Main executable (links stubs for test engine hooks — no-ops in production)
add_executable(LumiceGUI
    ${PROJ_SRC_DIR}/gui/main.cpp
    ${PROJ_SRC_DIR}/gui/imgui_te_stubs.cpp)
target_link_libraries(LumiceGUI PRIVATE lumice_gui_obj)

# macOS frameworks required by GLFW
if(APPLE)
  target_link_libraries(LumiceGUI PRIVATE
      "-framework Cocoa"
      "-framework IOKit"
      "-framework CoreVideo")
endif()

install(TARGETS LumiceGUI
    DESTINATION "${CMAKE_INSTALL_PREFIX}")
