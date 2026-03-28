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
    ${PROJ_SRC_DIR}/gui/app_panels.cpp
    ${PROJ_SRC_DIR}/gui/crystal_preview.cpp
    ${PROJ_SRC_DIR}/gui/panels.cpp
    ${PROJ_SRC_DIR}/gui/file_io.cpp
    ${PROJ_SRC_DIR}/gui/preview_renderer.cpp
    ${PROJ_SRC_DIR}/gui/crystal_renderer.cpp
    ${PROJ_SRC_DIR}/gui/gl_init.cpp
    ${PROJ_SRC_DIR}/gui/server_poller.cpp
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
      glad_gl
      winmm)
  # ImGui auto-detects GLAD2 via __has_include(<glad/gl.h>) — no loader definition needed.
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
target_compile_definitions(lumice_gui_obj PRIVATE SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_DEBUG)

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

# Windows: GUI subsystem (hide console) + embed icon via .rc resource
if(WIN32)
  set(LUMICE_ICO_PATH "${CMAKE_SOURCE_DIR}/resources/lumice.ico")
  configure_file(
      "${PROJ_SRC_DIR}/gui/lumice.rc.in"
      "${CMAKE_CURRENT_BINARY_DIR}/lumice.rc"
      @ONLY)
  target_sources(LumiceGUI PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/lumice.rc")

  # WIN32_EXECUTABLE can be disabled via LUMICE_CONSOLE_GUI for diagnostics.
  # WIN32 subsystem hides the console window but may affect thread scheduling.
  if(NOT LUMICE_CONSOLE_GUI)
    set_target_properties(LumiceGUI PROPERTIES WIN32_EXECUTABLE TRUE)
  endif()
endif()

# macOS: .app bundle with icon and Info.plist
if(APPLE)
  set(LUMICE_ICNS "${CMAKE_SOURCE_DIR}/resources/lumice.icns")
  set_source_files_properties(${LUMICE_ICNS} PROPERTIES
      MACOSX_PACKAGE_LOCATION Resources)
  target_sources(LumiceGUI PRIVATE ${LUMICE_ICNS})

  set_target_properties(LumiceGUI PROPERTIES
      MACOSX_BUNDLE TRUE
      MACOSX_BUNDLE_INFO_PLIST "${PROJ_SRC_DIR}/gui/Info.plist.in"
      MACOSX_BUNDLE_ICON_FILE "lumice.icns")

  target_link_libraries(LumiceGUI PRIVATE
      "-framework Cocoa"
      "-framework IOKit"
      "-framework CoreVideo")
endif()

if(APPLE)
  install(TARGETS LumiceGUI BUNDLE DESTINATION .)
else()
  install(TARGETS LumiceGUI DESTINATION "${CMAKE_INSTALL_PREFIX}")
endif()
