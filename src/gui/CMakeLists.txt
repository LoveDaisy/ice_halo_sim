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
    ${PROJ_SRC_DIR}/gui/stb_impl.cpp
    ${IMGUI_SOURCES})

target_include_directories(lumice_gui_obj PUBLIC
    ${PROJ_SRC_DIR}
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
    ${stb_SOURCE_DIR})

target_link_libraries(lumice_gui_obj PUBLIC
    lumice
    glfw
    nfd
    OpenGL::GL)

if(APPLE)
  target_compile_definitions(lumice_gui_obj PRIVATE GL_SILENCE_DEPRECATION)
endif()

if(NOT MSVC)
  target_compile_options(lumice_gui_obj PRIVATE -Wall -Wextra)
endif()

# Main executable
add_executable(LumiceGUI ${PROJ_SRC_DIR}/gui/main.cpp)
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
