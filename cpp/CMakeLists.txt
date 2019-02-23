cmake_minimum_required(VERSION 2.8.2)

project(IceHalo)

set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_DEBUG_TARGET_PROPERTIES
  INCLUDE_DIRECTORIES
  COMPILE_DEFINITIONS
  POSITION_INDEPENDENT_CODE
  CONTAINER_SIZE_REQUIRED
  LIB_VERSION)
option(DEBUG "Default OFF" OFF)

if(MSVC)
  add_definitions(-DNOMINMAX)
  set(CMAKE_DEBUG_POSTFIX "d")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fexceptions -fpermissive -pthread -frtti")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -msse4 -D__STDC_FORMAT_MACROS")
endif()

set(CMAKE_CXX_STANDARD 11)            # C++11...
set(CMAKE_CXX_STANDARD_REQUIRED ON)   #...is required...
set(CMAKE_CXX_EXTENSIONS OFF)         #...without compiler extensions like gnu++11

if(DEBUG)
  set(BUILDCFG "Debug")
  set(CMAKE_BUILD_TYPE "Debug")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -Wall -g -ggdb")
  add_compile_definitions(DEBUG)
else()
  set(BUILDCFG "Release")
  set(CMAKE_BUILD_TYPE "MinSizeRel")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -s")
endif()

if(MULTI_THREAD)
  add_compile_definitions(MULTI_THREAD)
endif()

if(MULTI_THREAD)
  add_compile_definitions(RANDOM_SEED)
endif()

set(BUILD_DIR "${CMAKE_SOURCE_DIR}/build")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${BUILD_DIR}/${BUILDCFG}/lib/${ARCH}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${BUILD_DIR}/${BUILDCFG}/lib/${ARCH}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${BUILD_DIR}/${BUILDCFG}/bin/${ARCH}")

set(PROJ_SRC_DIR "${CMAKE_SOURCE_DIR}/src")
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
find_package(Boost 1.54.0 COMPONENTS filesystem system REQUIRED)

# ==================================================================================================
# Add subdirectory
add_subdirectory(src)
if(BUILD_TEST)
    add_subdirectory(test)
endif()

