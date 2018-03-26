set(SOURCE_FILE
        main.cpp
        geometry.cpp
        linearalgebra.cpp
        optics.cpp
        testhelper.cpp
        )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")
# set(CMAKE_CXX_STANDARD 11)
set(executable_output_path ${PROJECT_BINARY_DIR}/bin)

# path to distribution of halide
set(HALIDE_DISTRIB_DIR "/Users/zhangjiajie/Codes/Halide/halide")
# set(HALIDE_DISTRIB_USE_STATIC_LIBRARY TRUE)
include_directories("${HALIDE_DISTRIB_DIR}/tools" "${HALIDE_DISTRIB_DIR}/include")

include("${HALIDE_DISTRIB_DIR}/halide_config.cmake")
include("${HALIDE_DISTRIB_DIR}/halide.cmake")

halide_library(ray_hit
    SRCS ray_hit_generator.cpp
    GENERATOR_ARGS auto_schedule=true)
halide_library(ray_prop
    SRCS ray_prop_generator.cpp
    GENERATOR_ARGS auto_schedule=true)

add_executable(IceHaloSim ${SOURCE_FILE})
target_link_libraries(IceHaloSim PUBLIC ray_hit ray_prop)



