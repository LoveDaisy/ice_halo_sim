set(SOURCE_FILE
        main.cpp
        math.cpp
        optics.cpp
        context.cpp
        render.cpp
        threadingpool.cpp)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -msse4 -pthread -std=c++11")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# set(executable_output_path ${PROJECT_BINARY_DIR}/bin)

# # path to distribution of halide
# set(HALIDE_DISTRIB_DIR "/Users/zhangjiajie/Codes/Halide/halide")
# if(NOT DEFINED HALIDE_DISTRIB_DIR)
#     message(FATAL_ERROR "Variable HALIDE_DISTRIB_DIR is not defined.")
# elseif(NOT IS_DIRECTORY ${HALIDE_DISTRIB_DIR})
#     message(FATAL_ERROR "HALIDE_DISTRIB_DIR is not valid directory.")
# endif()
# include("${HALIDE_DISTRIB_DIR}/halide.cmake")

include_directories(${INCLUDE_DIRECTORIES} "${PROJECT_SOURCE_DIR}/rapidjson/include")

find_package(Threads)

# halide_library(ray_hit
#     SRCS ray_hit_generator.cpp
#     GENERATOR_ARGS auto_schedule=true)
# halide_library(ray_prop
#     SRCS ray_prop_generator.cpp
#     GENERATOR_ARGS auto_schedule=true)

add_executable(IceHaloSim ${SOURCE_FILE})
target_link_libraries(IceHaloSim 
    PUBLIC ${CMAKE_THREAD_LIBS_INIT})



