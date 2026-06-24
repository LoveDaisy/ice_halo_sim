cmake_minimum_required(VERSION 3.14)

# macOS: minimum deployment target (Ventura).
# Must be set before project() so the compiler detection picks it up.
if(APPLE AND NOT DEFINED CMAKE_OSX_DEPLOYMENT_TARGET)
  set(CMAKE_OSX_DEPLOYMENT_TARGET "13.0" CACHE STRING "Minimum macOS version")
endif()

project(Lumice VERSION 4.3.1)

set(CMAKE_VERBOSE_MAKEFILE OFF)

# ==================================================================================================
# Basic
set(CMAKE_CXX_STANDARD 17)            # C++17...
set(CMAKE_CXX_STANDARD_REQUIRED ON)   #...is required...
set(CMAKE_CXX_EXTENSIONS OFF)         #...without compiler extensions like gnu++11
set(CMAKE_C_STANDARD 11)              # C11...

option(LUMICE_NATIVE_ARCH "Enable -march=native for release builds (GCC/Clang)" ON)

if(MSVC)
  add_definitions(-DNOMINMAX)
  set(CMAKE_DEBUG_POSTFIX "d")
endif()

# MinGW: statically link GCC runtime (libgcc, libstdc++, libwinpthread) so the
# distributed .exe is self-contained and doesn't require MinGW DLLs on the target machine.
if(MINGW)
  add_link_options(-static)
endif()

# MSVC: use static CRT (/MT) so distributed .exe doesn't require vcruntime DLLs.
if(MSVC)
  set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()

include(CheckIPOSupported)
check_ipo_supported(RESULT result OUTPUT output)

if(result AND "${CMAKE_BUILD_TYPE}" STREQUAL "Release" AND NOT BUILD_SHARED_LIBS)
  message(STATUS "IPO supported. Enable inter-procedural optimization.")
  set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

if(BUILD_SHARED_LIBS)
  set(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif()

# Directories
set(BUILD_DIR "${CMAKE_SOURCE_DIR}/build")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${BUILD_DIR}/${CMAKE_BUILD_TYPE}/lib/${ARCH}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${BUILD_DIR}/${CMAKE_BUILD_TYPE}/lib/${ARCH}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${BUILD_DIR}/${CMAKE_BUILD_TYPE}/bin/${ARCH}")

set(PROJ_SRC_DIR "${CMAKE_SOURCE_DIR}/src")
set(PROJ_TEST_DIR "${CMAKE_SOURCE_DIR}/test")
set(PROJ_ROOT "${CMAKE_SOURCE_DIR}")


# ==================================================================================================
# Find modules
find_package(Threads REQUIRED)  # GoogleTest needs it
if(BUILD_GUI AND NOT WIN32)
  find_package(OpenGL REQUIRED)
endif()

# ==================================================================================================
# CPM.cmake — lightweight dependency manager (wrapper around FetchContent)
# Cache CPM downloads outside cmake_build so clean builds (-k) don't re-download
if(NOT DEFINED CPM_SOURCE_CACHE AND NOT DEFINED ENV{CPM_SOURCE_CACHE})
  set(CPM_SOURCE_CACHE "${CMAKE_SOURCE_DIR}/build/cpm_cache"
      CACHE PATH "Directory to cache CPM dependency sources")
endif()
include(cmake/CPM.cmake)

# nlohmann/json: download only headers (~255KB) instead of full source (~6.7MB)
CPMAddPackage(
  NAME nlohmann_json
  URL https://github.com/nlohmann/json/releases/download/v3.10.5/include.zip
  URL_HASH SHA256=b94997df68856753b72f0d7a3703b7d484d4745c567f3584ef97c96c25a5798e
)
if(nlohmann_json_ADDED)
  add_library(nlohmann_json INTERFACE)
  add_library(nlohmann_json::nlohmann_json ALIAS nlohmann_json)
  target_include_directories(nlohmann_json INTERFACE ${nlohmann_json_SOURCE_DIR}/include)
endif()

# spdlog: header-only mode (skip building the static library)
CPMAddPackage(
  NAME spdlog
  URL https://github.com/gabime/spdlog/archive/refs/tags/v1.15.0.tar.gz
  URL_HASH SHA256=9962648c9b4f1a7bbc76fd8d9172555bad1871fdb14ff4f842ef87949682caa5
  OPTIONS "SPDLOG_BUILD_SHARED OFF" "SPDLOG_ENABLE_PCH OFF" "SPDLOG_BUILD_EXAMPLE OFF"
)
if(spdlog_ADDED)
  set_target_properties(spdlog PROPERTIES EXCLUDE_FROM_ALL YES)
endif()

# tl-expected: header-only expected<T,E> for C++17
CPMAddPackage(
  NAME tl-expected
  URL https://github.com/TartanLlama/expected/archive/refs/tags/v1.1.0.tar.gz
  URL_HASH SHA256=1db357f46dd2b24447156aaf970c4c40a793ef12a8a9c2ad9e096d9801368df6
  OPTIONS "EXPECTED_BUILD_TESTS OFF"
)

# stb: header-only image writing (only stb_image_write.h used)
CPMAddPackage(
  NAME stb
  URL https://github.com/nothings/stb/archive/f1c79c02822848a9bed4315b12c8c8f3761e1296.tar.gz
  URL_HASH SHA256=a39b0f2d96d9b64171ac1fd73295b7c5866f904e653b379da886d050a6900db9
  DOWNLOAD_ONLY YES
)

# GUI dependencies (only when BUILD_GUI is ON)
if(BUILD_GUI)
  # GLFW: windowing and input
  CPMAddPackage(
    NAME glfw
    GITHUB_REPOSITORY glfw/glfw
    GIT_TAG 3.4
    OPTIONS "GLFW_BUILD_DOCS OFF" "GLFW_BUILD_EXAMPLES OFF" "GLFW_BUILD_TESTS OFF"
  )

  # Dear ImGui: immediate-mode GUI (docking branch, no CMake support — download only)
  CPMAddPackage(
    NAME imgui
    GITHUB_REPOSITORY ocornut/imgui
    GIT_TAG v1.91.8-docking
    DOWNLOAD_ONLY YES
  )

  # nativefiledialog-extended: native OS file dialogs
  CPMAddPackage(
    NAME nfd
    GITHUB_REPOSITORY btzy/nativefiledialog-extended
    GIT_TAG v1.2.1
  )

  # Dear ImGui Test Engine (no CMake support — download only)
  CPMAddPackage(
    NAME imgui_test_engine
    GITHUB_REPOSITORY ocornut/imgui_test_engine
    GIT_TAG v1.91.8
    DOWNLOAD_ONLY YES
  )

  # IconFontCppHeaders: ICON_FA_* macros for FontAwesome 6 (header-only).
  # Pinned to a specific commit SHA (rather than `main`) for reproducible builds.
  CPMAddPackage(
    NAME IconFontCppHeaders
    GITHUB_REPOSITORY juliettef/IconFontCppHeaders
    GIT_TAG 3ee7f3d295ae773c0046db8d7b89b886eb2526de
    DOWNLOAD_ONLY YES
  )

  # Font Awesome 6 Free — TTF asset is consumed at configure time by
  # src/gui/CMakeLists.txt (file(READ HEX)) and embedded into the GUI binary.
  CPMAddPackage(
    NAME font_awesome
    GITHUB_REPOSITORY FortAwesome/Font-Awesome
    GIT_TAG 6.7.2
    DOWNLOAD_ONLY YES
  )

  # GLAD2: OpenGL loader for Windows (macOS/Linux use native GL headers)
  # GLAD2 v2.0.8 has its CMakeLists.txt in cmake/ subdirectory (root is a Python package).
  # SOURCE_SUBDIR tells CPM to run add_subdirectory on cmake/ instead of root.
  if(WIN32)
    CPMAddPackage(
      NAME glad
      GITHUB_REPOSITORY Dav1dde/glad
      GIT_TAG v2.0.8
      SOURCE_SUBDIR cmake
    )
    glad_add_library(glad_gl REPRODUCIBLE API gl:core=3.3)
  endif()
endif()

# ==================================================================================================
# Add some definitions
if(${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Linux")
  add_compile_definitions(OS_LINUX)
elseif(${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Darwin")
  add_compile_definitions(OS_MAC)
elseif(${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Windows")
  add_compile_definitions(OS_WIN)
endif()

# ==================================================================================================
# Set submodules
set(lumice_include
  "${PROJ_SRC_DIR}")

set(lumice_config_src
  "${PROJ_SRC_DIR}/config/config_manager.cpp"
  "${PROJ_SRC_DIR}/config/crystal_config.cpp"
  "${PROJ_SRC_DIR}/config/filter_config.cpp"
  "${PROJ_SRC_DIR}/config/light_config.cpp"
  "${PROJ_SRC_DIR}/config/proj_config.cpp"
  "${PROJ_SRC_DIR}/config/raypath_validation.cpp"
  "${PROJ_SRC_DIR}/config/render_config.cpp"
  "${PROJ_SRC_DIR}/config/sim_data.cpp")

set(lumice_core_src
  "${PROJ_SRC_DIR}/core/cpu_trace_backend.cpp"
  "${PROJ_SRC_DIR}/core/crystal.cpp"
  "${PROJ_SRC_DIR}/core/filter_spec.cpp"
  "${PROJ_SRC_DIR}/core/geo3d.cpp"
  "${PROJ_SRC_DIR}/core/math.cpp"
  "${PROJ_SRC_DIR}/core/optics.cpp"
  "${PROJ_SRC_DIR}/core/projection.cpp"
  "${PROJ_SRC_DIR}/core/raypath.cpp"
  "${PROJ_SRC_DIR}/core/simulator.cpp")

# Metal backend (Apple only). task-metal-build-time-metallib (#283): the GPU
# kernel single source of truth is `src/core/metal/lumice_trace.metal`. CMake
# precompiles it to a .metallib at build time and embeds the bytes + a C-string
# fallback into the binary via scripts/embed_metal_artifacts.py. The runtime
# loads the embedded metallib via newLibraryWithData, bypassing macOS 26.5's
# broken MSL source frontend; newLibraryWithSource is retained as fallback
# (disabled when LUMICE_DISABLE_METAL_SOURCE_COMPILE=1, which simulates 26.5
# for the AC2 regression test).
if(APPLE)
  list(APPEND lumice_core_src
    "${PROJ_SRC_DIR}/core/device_filter_desc.cpp"
    "${PROJ_SRC_DIR}/core/metal_filter_match_src.mm"
    "${PROJ_SRC_DIR}/core/metal_trace_backend.mm")
  set_source_files_properties(
    "${PROJ_SRC_DIR}/core/metal_trace_backend.mm"
    "${PROJ_SRC_DIR}/core/metal_filter_match_src.mm"
    PROPERTIES COMPILE_FLAGS "-fobjc-arc")

  # --- Build-time MSL → metallib → embedded C headers --------------------------
  # Tools (probed inside APPLE guard so non-Apple builds don't require Python3).
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  find_program(XCRUN_EXECUTABLE xcrun REQUIRED)

  set(LUMICE_METAL_SRC      "${PROJ_SRC_DIR}/core/metal/lumice_trace.metal")
  set(LUMICE_METAL_GEN_DIR  "${CMAKE_BINARY_DIR}/generated")
  set(LUMICE_METAL_AIR      "${LUMICE_METAL_GEN_DIR}/lumice_trace.air")
  set(LUMICE_METAL_METALLIB "${LUMICE_METAL_GEN_DIR}/lumice_trace.metallib")
  set(LUMICE_METAL_BYTES_H  "${LUMICE_METAL_GEN_DIR}/lumice_trace_metallib_embed.h")
  set(LUMICE_METAL_SRC_H    "${LUMICE_METAL_GEN_DIR}/lumice_trace_src_embed.h")
  set(LUMICE_EMBED_SCRIPT   "${CMAKE_SOURCE_DIR}/scripts/embed_metal_artifacts.py")

  # Compile flags MUST stay byte-identical to EnsurePso's runtime
  # MTLCompileOptions (MSL 3.0 + fno-fast-math/MTLMathModeSafe) so the offline
  # metallib produces the same numerical result as the runtime source path.
  # `-arch air64` prevents fat-AIR output (xcrun metallib rejects fat input).
  add_custom_command(
    OUTPUT  "${LUMICE_METAL_AIR}"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${LUMICE_METAL_GEN_DIR}"
    COMMAND ${XCRUN_EXECUTABLE} metal
            -arch air64
            -std=metal3.0
            -fno-fast-math
            -mmacosx-version-min=13.0
            -c "${LUMICE_METAL_SRC}"
            -o "${LUMICE_METAL_AIR}"
    DEPENDS "${LUMICE_METAL_SRC}"
    COMMENT "Compiling MSL → AIR (lumice_trace.metal → lumice_trace.air)"
    VERBATIM)

  add_custom_command(
    OUTPUT  "${LUMICE_METAL_METALLIB}"
    COMMAND ${XCRUN_EXECUTABLE} metallib
            "${LUMICE_METAL_AIR}"
            -o "${LUMICE_METAL_METALLIB}"
    DEPENDS "${LUMICE_METAL_AIR}"
    COMMENT "Linking metallib (lumice_trace.air → lumice_trace.metallib)"
    VERBATIM)

  add_custom_command(
    OUTPUT  "${LUMICE_METAL_BYTES_H}" "${LUMICE_METAL_SRC_H}"
    COMMAND ${Python3_EXECUTABLE} "${LUMICE_EMBED_SCRIPT}"
            --metallib   "${LUMICE_METAL_METALLIB}"
            --metal-src  "${LUMICE_METAL_SRC}"
            --out-dir    "${LUMICE_METAL_GEN_DIR}"
    DEPENDS "${LUMICE_METAL_METALLIB}" "${LUMICE_METAL_SRC}" "${LUMICE_EMBED_SCRIPT}"
    COMMENT "Embedding metallib + source into C headers"
    VERBATIM)

  add_custom_target(lumice_metal_shaders
    DEPENDS "${LUMICE_METAL_BYTES_H}" "${LUMICE_METAL_SRC_H}")
endif()

set(lumice_server_src
  "${PROJ_SRC_DIR}/server/c_api.cpp"
  "${PROJ_SRC_DIR}/server/render.cpp"
  "${PROJ_SRC_DIR}/server/server.cpp"
  "${PROJ_SRC_DIR}/server/show_rays.cpp"
  "${PROJ_SRC_DIR}/server/stats.cpp")

set(lumice_util_src
  "${PROJ_SRC_DIR}/util/color_space.cpp"
  "${PROJ_SRC_DIR}/util/cpu_info.cpp"
  "${PROJ_SRC_DIR}/util/env_knobs.cpp"
  "${PROJ_SRC_DIR}/util/illuminant.cpp"
  "${PROJ_SRC_DIR}/util/threading_pool.cpp")

set(lumice_src
  ${lumice_config_src} ${lumice_core_src}
  ${lumice_server_src} ${lumice_util_src})

# ==================================================================================================
# OBJECT library — compile sources once, reuse .o files across targets (library, test, bench).
# test/bench link lumice_obj directly so all symbols are accessible regardless of
# -fvisibility=hidden (visibility only affects shared-library export, not executable linking).
add_library(lumice_obj OBJECT ${lumice_src})
target_include_directories(lumice_obj
  PUBLIC
    $<BUILD_INTERFACE:${PROJ_SRC_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${lumice_include})
target_link_libraries(lumice_obj PUBLIC nlohmann_json::nlohmann_json spdlog::spdlog_header_only tl::expected Threads::Threads)
if(APPLE)
  target_link_libraries(lumice_obj PUBLIC "-framework Metal" "-framework Foundation")
  # task-metal-build-time-metallib (#283): metal_trace_backend.mm #includes the
  # generated metallib + source headers; depend on the custom target so the
  # headers exist before the .mm compiles, and add the generated dir to the
  # include path. PRIVATE — these headers are not part of the public API.
  add_dependencies(lumice_obj lumice_metal_shaders)
  target_include_directories(lumice_obj PRIVATE "${LUMICE_METAL_GEN_DIR}")
endif()

if(NOT MSVC)
  target_compile_options(lumice_obj PRIVATE
    -Wall -Wextra -fexceptions -frtti)
  target_compile_definitions(lumice_obj PRIVATE __STDC_FORMAT_MACROS SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE)
  # -fvisibility=hidden: Release/MinSizeRel 下隐藏所有符号，仅导出 lumice.h 中
  # 由 #pragma GCC visibility push(default) 标记的 C API 函数。
  # Debug 构建不设置此标志，所有符号均可见，便于调试。
  # NOTE: 当前仅支持 GCC/Clang。若需支持 MSVC 共享库，需将 pragma 改为
  # LUMICE_API 宏（__declspec(dllexport/dllimport)），参见 lumice.h 中的注释。
  target_compile_options(lumice_obj PRIVATE
    $<$<CONFIG:Debug>:-O0 -g -ggdb>
    $<$<CONFIG:Release>:$<$<BOOL:${LUMICE_NATIVE_ARCH}>:-march=native> -ffunction-sections -fdata-sections -fvisibility=hidden>
    $<$<CONFIG:MinSizeRel>:-ffunction-sections -fdata-sections -fvisibility=hidden>)
endif()

# Library target — static or shared per BUILD_SHARED_LIBS; reuses lumice_obj objects.
add_library(lumice)
target_link_libraries(lumice PUBLIC lumice_obj)
if(NOT MSVC)
  target_link_options(lumice PUBLIC
    $<$<CONFIG:MinSizeRel>:$<$<CXX_COMPILER_ID:AppleClang,Clang>:-dead_strip>>
    $<$<CONFIG:MinSizeRel>:$<$<CXX_COMPILER_ID:GNU>:--gc-sections>>)
endif()

# Shared builds: install the dylib next to the CLI (build/cmake_install/lib) so
# the install tree is self-contained. The CLI's INSTALL_RPATH (@loader_path/lib,
# see src/CMakeLists.txt) resolves @rpath/liblumice.dylib there. Static builds
# archive lumice into the CLI, so there is nothing to install here.
if(BUILD_SHARED_LIBS)
  install(TARGETS lumice LIBRARY DESTINATION lib)
endif()


# ==================================================================================================
# Add subdirectory
add_subdirectory(src)

if(BUILD_TEST)
  enable_testing()
  add_subdirectory(test)
endif()

if(BUILD_BENCH)
  add_subdirectory(bench)
endif()

if(BUILD_GUI)
  add_subdirectory(src/gui)
endif()

if(BUILD_GUI AND BUILD_TEST)
  add_subdirectory(test/gui)
endif()
