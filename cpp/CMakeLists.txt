cmake_minimum_required(VERSION 3.14)
project(IceHalo VERSION 2.1.1)

set(CMAKE_VERBOSE_MAKEFILE OFF)
option(DEBUG "Default OFF" OFF)

# ==================================================================================================
# Compiler related
if("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
  # using regular Clang or AppleClang
  set(CMAKE_EXE_LINKER_FLAGS_MINSIZEREL "${CMAKE_EXE_LINKER_FLAGS_MINSIZEREL} -dead_strip")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  # using GCC
  set(CMAKE_EXE_LINKER_FLAGS_MINSIZEREL "${CMAKE_EXE_LINKER_FLAGS_MINSIZEREL} --gc-sections")
endif()

if(MSVC)
  add_definitions(-DNOMINMAX)
  set(CMAKE_DEBUG_POSTFIX "d")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fexceptions -fpermissive -pthread -frtti")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -msse4 -mavx -D__STDC_FORMAT_MACROS -ftime-trace")
endif()

# ==================================================================================================
# OS related

# ==================================================================================================
# Basic
set(CMAKE_CXX_STANDARD 17)            # C++17...
set(CMAKE_CXX_STANDARD_REQUIRED ON)   #...is required...
set(CMAKE_CXX_EXTENSIONS OFF)         #...without compiler extensions like gnu++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

# Flags
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -Wall -g -ggdb")
set(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} -ffunction-sections -fdata-sections")

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
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

find_package(Threads REQUIRED)  # GoogleTest needs it
find_package(OpenCV REQUIRED core imgproc imgcodecs)
find_package(Boost 1.63.0 COMPONENTS filesystem system REQUIRED)


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
  "${PROJ_SRC_DIR}" ${Boost_INCLUDE_DIRS} "${MODULE_ROOT}/nlohmann_json")

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
  "${PROJ_SRC_DIR}/core/math.cpp"
  "${PROJ_SRC_DIR}/core/optics.cpp")

set(icehalo_process_src
  "${PROJ_SRC_DIR}/process/render.cpp"
  "${PROJ_SRC_DIR}/process/simulation.cpp")

set(icehalo_protocol_src
  "${PROJ_SRC_DIR}/protocol/crystal_config.cpp"
  "${PROJ_SRC_DIR}/protocol/sim_data.cpp")

set(icehalo_io_src
  "${PROJ_SRC_DIR}/io/file.cpp")

set(icehalo_util_src
  "${PROJ_SRC_DIR}/util/arg_parser.cpp"
  "${PROJ_SRC_DIR}/util/log.cpp"
  "${PROJ_SRC_DIR}/util/obj_pool.cpp"
  "${PROJ_SRC_DIR}/util/threading_pool.cpp")
set(icehalo_src
  ${icehalo_context_src} ${icehalo_core_src} ${icehalo_io_src}
  ${icehalo_process_src} ${icehalo_protocol_src} ${icehalo_util_src})

# ==================================================================================================
# Add targets
add_library(icehalo_lib OBJECT ${icehalo_src})
target_include_directories(icehalo_lib PUBLIC ${icehalo_include})
if(MULTI_THREAD)
  target_compile_definitions(icehalo_lib PUBLIC MULTI_THREAD)
endif()
if(RANDOM_SEED)
  target_compile_definitions(icehalo_lib PUBLIC RANDOM_SEED)
endif()

# ==================================================================================================
# Add subdirectory
add_subdirectory(src)
if(BUILD_TEST)
    enable_testing()
    add_subdirectory(test)
endif()

