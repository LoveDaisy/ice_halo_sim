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

add_executable(LumiceGUI
    ${PROJ_SRC_DIR}/gui/main.cpp
    ${PROJ_SRC_DIR}/gui/panels.cpp
    ${PROJ_SRC_DIR}/gui/file_io.cpp
    ${PROJ_SRC_DIR}/gui/preview_renderer.cpp
    ${PROJ_SRC_DIR}/gui/crystal_renderer.cpp
    ${IMGUI_SOURCES})

target_include_directories(LumiceGUI PRIVATE
    ${PROJ_SRC_DIR}
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends)

target_link_libraries(LumiceGUI PRIVATE
    lumice
    glfw
    nfd
    OpenGL::GL)

# macOS frameworks required by GLFW
if(APPLE)
  target_link_libraries(LumiceGUI PRIVATE
      "-framework Cocoa"
      "-framework IOKit"
      "-framework CoreVideo")
  target_compile_definitions(LumiceGUI PRIVATE GL_SILENCE_DEPRECATION)
endif()

if(NOT MSVC)
  target_compile_options(LumiceGUI PRIVATE -Wall -Wextra)
endif()

install(TARGETS LumiceGUI
    DESTINATION "${CMAKE_INSTALL_PREFIX}")
