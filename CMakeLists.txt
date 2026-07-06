cmake_minimum_required(VERSION 3.14)

# macOS: minimum deployment target (Ventura).
# Must be set before project() so the compiler detection picks it up.
if(APPLE AND NOT DEFINED CMAKE_OSX_DEPLOYMENT_TARGET)
  set(CMAKE_OSX_DEPLOYMENT_TARGET "13.0" CACHE STRING "Minimum macOS version")
endif()

project(Lumice VERSION 4.3.3)

set(CMAKE_VERBOSE_MAKEFILE OFF)

# ==================================================================================================
# Basic
set(CMAKE_CXX_STANDARD 17)            # C++17...
set(CMAKE_CXX_STANDARD_REQUIRED ON)   #...is required...
set(CMAKE_CXX_EXTENSIONS OFF)         #...without compiler extensions like gnu++11
set(CMAKE_C_STANDARD 11)              # C11...

option(LUMICE_NATIVE_ARCH "Enable -march=native for release builds (GCC/Clang)" ON)

# CUDA backend (off by default; turn on for dev49 docker CUDA 11.6+ environments).
# scrum-cuda-backend-mvp subtask 3: enables CudaTraceBackend + lumice_trace.cu.
# Mac/Windows host builds without nvcc must stay zero-regress when this is OFF.
option(LUMICE_CUDA_ENABLED "Enable CUDA backend (requires nvcc / CUDAToolkit)" OFF)

# scrum-306.1 (explore-async-evidence, H7): embed Metal shader sources + line
# tables into the metallib so Xcode/Instruments GPU shader profiler maps occupancy
# / per-line ALU cost back to lumice_trace.metal. No runtime perf impact (only a
# slightly larger embedded metallib blob); keep OFF for release/CI. Turn ON for
# the Xcode profiling build (build/xcode) used to measure Metal kernel occupancy.
option(LUMICE_METAL_SHADER_PROFILE "Embed Metal shader source + line tables for GPU shader profiler" OFF)

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
  # scrum-309: back-port the `#if FMT_USE_FLOAT128` guard on fmt's float128
  # formatter so the bundled fmt compiles under nvcc on Windows. fmt v11 declares
  # `formatter<detail::float128>` unguarded; when FMT_USE_FLOAT128=0 (no
  # __float128) float128 aliases void, and nvcc's device (EDG) frontend on
  # Windows eagerly instantiates formatter<void> → "reference to void is not
  # allowed". Patch the cached source in place (idempotent; no-op once guarded,
  # harmless on Linux where the specialization stays enabled). Done inline with
  # CPM's ${spdlog_SOURCE_DIR} rather than a PATCH_COMMAND so the working
  # directory is unambiguous.
  set(_fmt_fh "${spdlog_SOURCE_DIR}/include/spdlog/fmt/bundled/format.h")
  if(EXISTS "${_fmt_fh}")
    file(READ "${_fmt_fh}" _fmt_c)
    string(FIND "${_fmt_c}" "#if FMT_USE_FLOAT128\ntemplate <typename Char>\nstruct formatter<detail::float128" _fmt_guarded)
    if(_fmt_guarded EQUAL -1)
      string(REPLACE
        "template <typename Char>\nstruct formatter<detail::float128, Char>\n    : detail::native_formatter<detail::float128, Char,\n                               detail::type::float_type> {};"
        "#if FMT_USE_FLOAT128\ntemplate <typename Char>\nstruct formatter<detail::float128, Char>\n    : detail::native_formatter<detail::float128, Char,\n                               detail::type::float_type> {};\n#endif"
        _fmt_c "${_fmt_c}")
      file(WRITE "${_fmt_fh}" "${_fmt_c}")
      message(STATUS "scrum-309: guarded bundled fmt formatter<float128> for nvcc/Windows")
    endif()
  endif()
  if(MSVC AND LUMICE_CUDA_ENABLED)
    # scrum-309 (Windows CUDA build): spdlog's CMake adds bare MSVC flags
    # `/utf-8` and `/Zc:__cplusplus` as INTERFACE compile options on
    # spdlog_header_only WITHOUT the `$<NOT:$<COMPILE_LANGUAGE:CUDA>>` guard it
    # uses for /wd4251. When a CUDA (.cu) TU links spdlog, these `/`-prefixed
    # flags reach nvcc, which on Windows misreads them as input file paths and
    # dies ("nvcc fatal: A single input file is required for a non-link phase
    # when an output file is specified"). Re-scope the interface options to the
    # CXX language so they never reach nvcc; CMake still supplies the MSVC
    # runtime to the CUDA host compiler via its own -Xcompiler handling.
    get_target_property(_spdlog_io spdlog_header_only INTERFACE_COMPILE_OPTIONS)
    if(_spdlog_io)
      set(_spdlog_io_cxx "")
      foreach(_opt IN LISTS _spdlog_io)
        list(APPEND _spdlog_io_cxx "$<$<COMPILE_LANGUAGE:CXX>:${_opt}>")
      endforeach()
      set_target_properties(spdlog_header_only PROPERTIES
        INTERFACE_COMPILE_OPTIONS "${_spdlog_io_cxx}")
    endif()
  endif()
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
  "${PROJ_SRC_DIR}/core/backend/cpu_trace_backend.cpp"
  "${PROJ_SRC_DIR}/core/crystal.cpp"
  "${PROJ_SRC_DIR}/core/device_filter_desc.cpp"
  "${PROJ_SRC_DIR}/core/filter_spec.cpp"
  "${PROJ_SRC_DIR}/core/geo3d.cpp"
  "${PROJ_SRC_DIR}/core/lat_lut.cpp"
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
    "${PROJ_SRC_DIR}/core/metal_filter_match_src.mm"
    "${PROJ_SRC_DIR}/core/backend/metal_trace_backend.mm")
  set_source_files_properties(
    "${PROJ_SRC_DIR}/core/backend/metal_trace_backend.mm"
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
  # Shared single-source math headers consumed by both MSL and host C++.
  # Listed in DEPENDS so any change re-triggers metallib compile + embed.
  set(LUMICE_METAL_SHARED_HDRS
      "${PROJ_SRC_DIR}/core/shared/lm_shims.h"
      "${PROJ_SRC_DIR}/core/shared/projection_shared.h"
      "${PROJ_SRC_DIR}/core/shared/optics_shared.h"
      "${PROJ_SRC_DIR}/core/shared/traversal_shared.h"
      "${PROJ_SRC_DIR}/core/shared/pcg_shared.h"
      "${PROJ_SRC_DIR}/core/shared/accum_shared.h")

  # scrum-306.1 H7: optional shader debug info for the GPU profiler (off by default).
  set(LUMICE_METAL_DEBUG_FLAGS "")
  if(LUMICE_METAL_SHADER_PROFILE)
    set(LUMICE_METAL_DEBUG_FLAGS -frecord-sources -gline-tables-only)
    message(STATUS "[metal] LUMICE_METAL_SHADER_PROFILE=ON → embedding shader sources/line-tables for GPU profiler")
  endif()

  add_custom_command(
    OUTPUT  "${LUMICE_METAL_AIR}"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${LUMICE_METAL_GEN_DIR}"
    COMMAND ${XCRUN_EXECUTABLE} metal
            -arch air64
            -std=metal3.0
            -fno-fast-math
            -mmacosx-version-min=13.0
            ${LUMICE_METAL_DEBUG_FLAGS}
            -c "${LUMICE_METAL_SRC}"
            -o "${LUMICE_METAL_AIR}"
    DEPENDS "${LUMICE_METAL_SRC}" ${LUMICE_METAL_SHARED_HDRS}
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
            ${LUMICE_METAL_SHARED_HDRS}
    COMMENT "Embedding metallib + source into C headers"
    VERBATIM)

  add_custom_target(lumice_metal_shaders
    DEPENDS "${LUMICE_METAL_BYTES_H}" "${LUMICE_METAL_SRC_H}")
endif()

# CUDA backend (off by default). scrum-cuda-backend-mvp subtask 3:
# - nvcc 11.6 (dev49 docker image) is the lowest supported toolchain;
#   CUDA 12.x toolkits also work (still support compute_61).
# - PTX floor = compute_61 (Pascal, 2016+); driver JITs to native sm on any
#   GPU >= sm_61, including future cards.
# - Multi-arch fatbin (scrum-310.2): on top of the compute_61 PTX floor, emit
#   native SASS for the common modern cards so they skip first-launch JIT and
#   the dev49 RTX 4060 (sm_89) bench runs native (not PTX-JIT'd) for fidelity.
# - Mac/Windows host builds without nvcc keep BUILD_GUI/perf paths intact when
#   LUMICE_CUDA_ENABLED is OFF (no enable_language(CUDA) at all).
if(LUMICE_CUDA_ENABLED)
  enable_language(CUDA)
  find_package(CUDAToolkit REQUIRED)
  set(CMAKE_CUDA_STANDARD 17)
  set(CMAKE_CUDA_STANDARD_REQUIRED ON)
  if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    # PTX floor `61-virtual` emits compute_61 PTX only (no SASS); the driver
    # JITs it to native sm on any GPU >= sm_61, including future cards. Owner
    # decision 2026-06-25 (296.1): PTX floor widens forward-compat to
    # Pascal/Turing/Ampere/Ada/Hopper/future; the older `86-virtual` default
    # was an explore #294 dev49 probe artifact that silently excluded
    # Turing/Pascal.
    set(_lumice_cuda_archs "61-virtual")
    # Opportunistic native SASS for common cards: skips first-launch JIT and
    # gives bench fidelity. Each real arch is gated on the toolkit version that
    # introduced that sm, so older toolchains (dev49 docker 11.6, win-builder
    # 11.7) still build — they just emit fewer `-real` archs and fall back to
    # PTX JIT. Distribution/CI builds use a modern toolkit (CI: 12.6) and get
    # the full fatbin. The PTX floor guarantees universal compat regardless.
    if(CUDAToolkit_VERSION VERSION_GREATER_EQUAL "10.0")
      list(APPEND _lumice_cuda_archs "75-real")  # Turing (RTX 20 / GTX 16)
    endif()
    if(CUDAToolkit_VERSION VERSION_GREATER_EQUAL "11.1")
      list(APPEND _lumice_cuda_archs "86-real")  # Ampere (RTX 30)
    endif()
    if(CUDAToolkit_VERSION VERSION_GREATER_EQUAL "11.8")
      list(APPEND _lumice_cuda_archs "89-real")  # Ada Lovelace (RTX 40, dev49)
    endif()
    set(CMAKE_CUDA_ARCHITECTURES "${_lumice_cuda_archs}" CACHE STRING "CUDA archs")
  endif()
  list(APPEND lumice_core_src
    "${PROJ_SRC_DIR}/core/backend/cuda_trace_backend.cu")
  message(STATUS "CUDA backend enabled (arch=${CMAKE_CUDA_ARCHITECTURES})")
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

if(LUMICE_CUDA_ENABLED)
  target_link_libraries(lumice_obj PUBLIC CUDA::cudart)
  target_compile_definitions(lumice_obj PUBLIC LUMICE_CUDA_ENABLED=1)
  if(MSVC)
    # Windows CUDA build (scrum-309).
    # spdlog/fmt require `/utf-8` (Unicode static_assert) and `/Zc:__cplusplus`
    # (correct __cplusplus so fmt picks the right code path; without it the
    # float128 native_formatter instantiates against `void`). The spdlog block
    # above strips these from the CUDA TU because nvcc misreads the bare `/`
    # flags as input file paths; re-supply them to the host compiler via
    # -Xcompiler (nvcc forwards them to cl) so the CUDA host phase matches the
    # plain-CXX build.
    target_compile_options(lumice_obj PRIVATE
      $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=/utf-8,/Zc:__cplusplus>)
    # fmt (bundled in spdlog) trips two MSVC×nvcc-only issues during the CUDA
    # TU's device (EDG) frontend pass, which `/utf-8` (a cl flag) cannot reach:
    #   FMT_UNICODE: EDG reads source under the system code page (936) on this
    #     host, so fmt's "Unicode support requires /utf-8" static_assert fires.
    #   FMT_USE_FLOAT128: nvcc emulates gcc on the host and defines
    #     __SIZEOF_FLOAT128__, so fmt enables its float128 formatter, but the
    #     MSVC backend has no __float128 (float128 aliases void) →
    #     "reference to void is not allowed".
    # Both features are unused on Windows (logs are ASCII; no float128); disable
    # them for the CUDA TU. FMT_USE_FLOAT128 is already 0 under plain cl, so
    # only FMT_UNICODE differs from the CXX TUs (acceptable: ASCII-only logs).
    target_compile_definitions(lumice_obj PRIVATE
      $<$<COMPILE_LANGUAGE:CUDA>:FMT_UNICODE=0 FMT_USE_FLOAT128=0>)
  endif()
endif()

if(NOT MSVC)
  # -Werror=switch is a GCC/Clang host flag; nvcc parses -Werror itself (does
  # NOT forward it) and rejects "switch" as an invalid value. Scope it to CXX so
  # the BackendKind exhaustiveness gate still applies to host TUs while the CUDA
  # (.cu) TU compiles cleanly. The CXX TUs already cover every BackendKind switch.
  target_compile_options(lumice_obj PRIVATE
    -Wall -Wextra $<$<COMPILE_LANGUAGE:CXX>:-Werror=switch> -fexceptions -frtti)
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
if(WIN32 AND BUILD_SHARED_LIBS)
  # scrum-309: the C API uses `#pragma GCC visibility` (gcc/clang only); MSVC
  # exports nothing, so the shared lumice.dll has no usable symbols for the
  # Python e2e/parity harness (ctypes) or any DLL consumer. Auto-export all
  # symbols so the Windows DLL is loadable like the Unix .so/.dylib. (A proper
  # explicit LUMICE_API __declspec export surface is a delivery concern; see
  # backlog #133.)
  set_target_properties(lumice PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
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
