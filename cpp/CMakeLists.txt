cmake_minimum_required(VERSION 3.14)
project(IceHalo VERSION 2.1.1)

set(CMAKE_VERBOSE_MAKEFILE OFF)

# ==================================================================================================
# Basic
set(CMAKE_CXX_STANDARD 17)            # C++17...
set(CMAKE_CXX_STANDARD_REQUIRED ON)   #...is required...
set(CMAKE_CXX_EXTENSIONS OFF)         #...without compiler extensions like gnu++11
set(CMAKE_C_STANDARD 11)              # C11...

if(MSVC)
  add_definitions(-DNOMINMAX)
  set(CMAKE_DEBUG_POSTFIX "d")
endif()

include(CheckIPOSupported)
check_ipo_supported(RESULT result OUTPUT output)

if(result AND "${CMAKE_BUILD_TYPE}" STREQUAL "Release")
  message(STATUS "IPO supported. Enable inter-procedural optimization.")
  set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

# Directories
set(BUILD_DIR "${CMAKE_SOURCE_DIR}/build")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${BUILD_DIR}/${CMAKE_BUILD_TYPE}/lib/${ARCH}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${BUILD_DIR}/${CMAKE_BUILD_TYPE}/lib/${ARCH}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${BUILD_DIR}/${CMAKE_BUILD_TYPE}/bin/${ARCH}")

set(PROJ_SRC_DIR "${CMAKE_SOURCE_DIR}/src")
set(PROJ_TEST_DIR "${CMAKE_SOURCE_DIR}/test")
set(PROJ_ROOT "${CMAKE_SOURCE_DIR}")
set(MODULE_ROOT "${CMAKE_SOURCE_DIR}/thirdparty")

# ==================================================================================================
# Find modules
find_package(Threads REQUIRED)  # GoogleTest needs it
find_package(OpenCV REQUIRED core imgproc imgcodecs)

# nlohmann/json: use FetchContent to pin v3.10.5 (system version may be incompatible)
include(FetchContent)
FetchContent_Declare(json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG v3.10.5)
FetchContent_MakeAvailable(json)

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
set(icehalo_include
  "${PROJ_SRC_DIR}")

set(icehalo_config_src
  "${PROJ_SRC_DIR}/config/config_manager.cpp"
  "${PROJ_SRC_DIR}/config/crystal_config.cpp"
  "${PROJ_SRC_DIR}/config/filter_config.cpp"
  "${PROJ_SRC_DIR}/config/light_config.cpp"
  "${PROJ_SRC_DIR}/config/proj_config.cpp"
  "${PROJ_SRC_DIR}/config/render_config.cpp"
  "${PROJ_SRC_DIR}/config/sim_data.cpp")

set(icehalo_context_src
  "${PROJ_SRC_DIR}/context/camera_context.cpp"
  "${PROJ_SRC_DIR}/context/context.cpp"
  "${PROJ_SRC_DIR}/context/crystal_context.cpp"
  "${PROJ_SRC_DIR}/context/filter_context.cpp"
  "${PROJ_SRC_DIR}/context/multi_scatter_context.cpp"
  "${PROJ_SRC_DIR}/context/render_context.cpp"
  "${PROJ_SRC_DIR}/context/sun_context.cpp")

set(icehalo_core_src
  "${PROJ_SRC_DIR}/core/crystal.cpp"
  "${PROJ_SRC_DIR}/core/filter.cpp"
  "${PROJ_SRC_DIR}/core/geo3d.cpp"
  "${PROJ_SRC_DIR}/core/math.cpp"
  "${PROJ_SRC_DIR}/core/optics.cpp"
  "${PROJ_SRC_DIR}/core/raypath.cpp"
  "${PROJ_SRC_DIR}/core/simulator.cpp")

set(icehalo_io_src
  "${PROJ_SRC_DIR}/io/file.cpp")

set(icehalo_process_src
  "${PROJ_SRC_DIR}/process/render.cpp"
  "${PROJ_SRC_DIR}/process/simulation.cpp")

set(icehalo_server_src
  "${PROJ_SRC_DIR}/server/render.cpp"
  "${PROJ_SRC_DIR}/server/server.cpp"
  "${PROJ_SRC_DIR}/server/show_rays.cpp"
  "${PROJ_SRC_DIR}/server/stats.cpp")

set(icehalo_util_src
  "${PROJ_SRC_DIR}/util/arg_parser.cpp"
  "${PROJ_SRC_DIR}/util/log.cpp"
  "${PROJ_SRC_DIR}/util/obj_pool.cpp"
  "${PROJ_SRC_DIR}/util/threading_pool.cpp")

set(icehalo_src
  ${icehalo_config_src} ${icehalo_context_src} ${icehalo_core_src}
  ${icehalo_io_src} ${icehalo_process_src} ${icehalo_server_src} ${icehalo_util_src})

# ==================================================================================================
# Add targets
add_library(icehalo_lib OBJECT ${icehalo_src})
target_include_directories(icehalo_lib PUBLIC ${icehalo_include})
target_link_libraries(icehalo_lib PUBLIC nlohmann_json::nlohmann_json Threads::Threads)

if(NOT MSVC)
  target_compile_options(icehalo_lib PRIVATE
    -Wall -Wextra -fexceptions -fpermissive -frtti)
  target_compile_definitions(icehalo_lib PRIVATE __STDC_FORMAT_MACROS)
  target_compile_options(icehalo_lib PRIVATE
    $<$<CONFIG:Debug>:-O0 -g -ggdb>
    $<$<CONFIG:Release>:-ffunction-sections -fdata-sections -fvisibility=hidden>
    $<$<CONFIG:MinSizeRel>:-ffunction-sections -fdata-sections -fvisibility=hidden>)
  target_link_options(icehalo_lib PUBLIC
    $<$<CONFIG:MinSizeRel>:$<$<CXX_COMPILER_ID:AppleClang,Clang>:-dead_strip>>
    $<$<CONFIG:MinSizeRel>:$<$<CXX_COMPILER_ID:GNU>:--gc-sections>>)
endif()

if(MULTI_THREAD)
  target_compile_definitions(icehalo_lib PUBLIC MULTI_THREAD)
endif()

if(RANDOM_SEED)
  target_compile_definitions(icehalo_lib PUBLIC RANDOM_SEED)
endif()

if(VERBOSE_LOG)
  target_compile_definitions(icehalo_lib PUBLIC VERBOSE_LOG)
endif()

# ==================================================================================================
# Add subdirectory
add_subdirectory(src)

if(BUILD_TEST)
  enable_testing()
  add_subdirectory(test)
endif()
