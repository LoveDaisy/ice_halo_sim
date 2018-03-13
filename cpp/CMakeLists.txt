cmake_minimum_required(VERSION 3.9)
project(IceHaloSim)

set(CMAKE_CXX_STANDARD 11)

add_executable(IceHaloSim
        geometry.cpp
        geometry.h
        linearalgebra.cpp
        linearalgebra.h
        optics.cpp
        optics.h
        main.cpp
        testhelper.cpp
        testhelper.h)
