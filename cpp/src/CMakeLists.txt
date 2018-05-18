set(SOURCE_FILE
        main.cpp
        geometry.cpp
        linearalgebra.cpp
        optics.cpp
        processhelper.cpp)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -msse4 -pthread -lprofiler -std=c++11")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(executable_output_path ${PROJECT_BINARY_DIR}/bin)

# path to distribution of halide
set(HALIDE_DISTRIB_DIR "/Users/zhangjiajie/Codes/Halide/halide")
if(NOT DEFINED HALIDE_DISTRIB_DIR)
    message(FATAL_ERROR "Variable HALIDE_DISTRIB_DIR is not defined.")
elseif(NOT IS_DIRECTORY ${HALIDE_DISTRIB_DIR})
    message(FATAL_ERROR "HALIDE_DISTRIB_DIR is not valid directory.")
endif()
# set(HALIDE_DISTRIB_USE_STATIC_LIBRARY TRUE)
include("${HALIDE_DISTRIB_DIR}/halide.cmake")

# if(NOT DEFINED HALIDE_MACHINE_PARAMS)
#     set(HALIDE_MACHINE_PARAMS "8,8192,40")
# endif()

find_package(Threads)
# find_package(Gperftools)

halide_library(ray_hit
    SRCS ray_hit_generator.cpp
    GENERATOR_ARGS auto_schedule=true)
halide_library(ray_prop
    SRCS ray_prop_generator.cpp
    GENERATOR_ARGS auto_schedule=true)

add_executable(IceHaloSim ${SOURCE_FILE})
target_link_libraries(IceHaloSim 
    PUBLIC ${CMAKE_THREAD_LIBS_INIT} ray_hit ray_prop)



