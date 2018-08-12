# set(SOURCE_FILE
#         mymath.cpp
#         crystal.cpp
#         optics.cpp
#         context.cpp
#         render.cpp
#         files.cpp
#         threadingpool.cpp)
file(GLOB SOURCE_FILE
    "*.h"
    "*.cpp")

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -msse4 -std=c++11")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 

include_directories(${INCLUDE_DIRECTORIES} "${PROJECT_SOURCE_DIR}/rapidjson/include")

find_package(Threads)
find_package(OpenCV REQUIRED core imgproc imgcodecs)
find_package(Boost 1.54.0 COMPONENTS filesystem system REQUIRED)

if(Boost_FOUND)
    include_directories(${INCLUDE_DIRECTORIES} ${Boost_INCLUDE_DIRS}) 

    add_executable(IceHaloSim trace_main.cpp ${SOURCE_FILE})
    target_link_libraries(IceHaloSim 
        PUBLIC ${CMAKE_THREAD_LIBS_INIT} ${Boost_LIBRARIES})

    add_executable(IceHaloRender render_main.cpp ${SOURCE_FILE})
    target_link_libraries(IceHaloRender 
        PUBLIC ${OpenCV_LIBS} ${Boost_LIBRARIES})
endif()



