set(SOURCE_FILE
        mymath.cpp
        crystal.cpp
        optics.cpp
        context.cpp
        render.cpp
        threadingpool.cpp)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -msse4 -pthread -std=c++11")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

include_directories(${INCLUDE_DIRECTORIES} "${PROJECT_SOURCE_DIR}/rapidjson/include")

find_package(Threads)
find_package(OpenCV REQUIRED core imgproc imgcodecs)

add_executable(IceHaloSim trace_main.cpp ${SOURCE_FILE})
target_link_libraries(IceHaloSim 
    PUBLIC ${CMAKE_THREAD_LIBS_INIT})

add_executable(IceHaloRender render_main.cpp ${SOURCE_FILE})
target_link_libraries(IceHaloRender 
    PUBLIC ${CMAKE_THREAD_LIBS_INIT} ${OpenCV_LIBS})



