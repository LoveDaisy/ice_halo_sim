cmake_minimum_required(VERSION 3.9)
project(IceHaloSim)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2")

set(SOURCE_FILES
        geometry.cpp
        geometry.h
        linearalgebra.cpp
        linearalgebra.h
        main.cpp
        optics.cpp
        optics.h
        testhelper.cpp
        testhelper.h)

add_executable(IceHaloSim ${SOURCE_FILES})
