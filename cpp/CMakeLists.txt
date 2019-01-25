cmake_minimum_required(VERSION 2.8.2)

project(IceHalo)

set(CMAKE_VERBOSE_MAKEFILE ON)
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

set(BUILD_DIR "${CMAKE_SOURCE_DIR}/build")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${BUILD_DIR}/${BUILDCFG}/lib/${ARCH}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${BUILD_DIR}/${BUILDCFG}/lib/${ARCH}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${BUILD_DIR}/${BUILDCFG}/bin/${ARCH}")

set(PROJ_SRC_DIR "${CMAKE_SOURCE_DIR}/src")
set(PROJ_ROOT "${CMAKE_SOURCE_DIR}")

add_subdirectory(src)
if(BUILD_TEST)
    add_subdirectory(test)
endif()

