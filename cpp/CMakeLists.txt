cmake_minimum_required(VERSION 3.12)

project(IceHalo
        VERSION 2.1.1)

set(CMAKE_VERBOSE_MAKEFILE OFF)
option(DEBUG "Default OFF" OFF)

# ==================================================================================================
# Compiler related
if("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
  # using regular Clang or AppleClang
  set(CXX_LINKER_FLAG_STRIP "-dead_strip")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  # using GCC
  set(CXX_LINKER_FLAG_STRIP "--gc-sections")
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
set(OS_NAME "${CMAKE_HOST_SYSTEM_NAME}")

# ==================================================================================================
# Basic
set(CMAKE_CXX_STANDARD 17)            # C++17...
set(CMAKE_CXX_STANDARD_REQUIRED ON)   #...is required...
set(CMAKE_CXX_EXTENSIONS OFF)         #...without compiler extensions like gnu++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

if(DEBUG)
  set(BUILDCFG "Debug")
  set(CMAKE_BUILD_TYPE "Debug")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -Wall -g -ggdb")
  add_compile_definitions(DEBUG)
else()
  set(BUILDCFG "Release")
  set(CMAKE_BUILD_TYPE "Release")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${CXX_LINKER_FLAG_STRIP}")
endif()


set(BUILD_DIR "${CMAKE_SOURCE_DIR}/build")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${BUILD_DIR}/${BUILDCFG}/lib/${ARCH}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${BUILD_DIR}/${BUILDCFG}/lib/${ARCH}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${BUILD_DIR}/${BUILDCFG}/bin/${ARCH}")

set(PROJ_SRC_DIR "${CMAKE_SOURCE_DIR}/src")
set(PROJ_TEST_DIR "${CMAKE_SOURCE_DIR}/test")
set(PROJ_ROOT "${CMAKE_SOURCE_DIR}")
set(MODULE_ROOT "${CMAKE_SOURCE_DIR}/thirdparty")

# ==================================================================================================
enable_testing()
# Google Test
set(GoogleTest_DIR "${MODULE_ROOT}/googletest")

configure_file(CMakeLists.GoogleTest.cmake googletest-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download)
if(result)
  message(FATAL_ERROR "CMake step for googletest failed: ${result}")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} --build .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download)
if(result)
  message(FATAL_ERROR "Build step for googletest failed: ${result}")
endif()

# Prevent overriding the parent project's compiler/linker
# settings on Windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Add googletest directly to our build. This defines
# the gtest and gtest_main targets.
add_subdirectory(${GoogleTest_DIR}
                 ${CMAKE_CURRENT_BINARY_DIR}/googletest-build
                 EXCLUDE_FROM_ALL)

# The gtest/gtest_main targets carry header search path
# dependencies automatically when using CMake 2.8.11 or
# later. Otherwise we have to add them here ourselves.
if (CMAKE_VERSION VERSION_LESS 2.8.11)
  include_directories("${gtest_SOURCE_DIR}/include")
endif()

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
if(${OS_NAME} STREQUAL "Linux")
  add_compile_definitions(OS_LINUX) 
elseif(${OS_NAME} STREQUAL "Darwin")
  add_compile_definitions(OS_MAC)
elseif(${OS_NAME} STREQUAL "Windows")
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

set(icehalo_io_src
  "${PROJ_SRC_DIR}/io/file.cpp")

 set(icehalo_util_src
  "${PROJ_SRC_DIR}/util/arg_parser.cpp"
  "${PROJ_SRC_DIR}/util/log.cpp"
  "${PROJ_SRC_DIR}/util/obj_pool.cpp"
  "${PROJ_SRC_DIR}/util/threading_pool.cpp")

  set(icehalo_src
  ${icehalo_context_src} ${icehalo_core_src} ${icehalo_io_src} ${icehalo_process_src} ${icehalo_util_src})


# ==================================================================================================
# Add subdirectory
add_subdirectory(src)
if(BUILD_TEST)
    add_subdirectory(test)
endif()

