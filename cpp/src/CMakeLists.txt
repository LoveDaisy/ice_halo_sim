set(SOURCE_FILE mymath.cpp
                crystal.cpp
                optics.cpp
                context.cpp
                render.cpp
                files.cpp
                threadingpool.cpp)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

find_package(Threads)
find_package(OpenCV REQUIRED core imgproc imgcodecs)
find_package(Boost 1.54.0 COMPONENTS filesystem system REQUIRED)

if(Boost_FOUND)
    add_executable(IceHaloSim trace_main.cpp ${SOURCE_FILE})
    target_include_directories(IceHaloSim
        PUBLIC ${Boost_INCLUDE_DIRS} "${MODULE_ROOT}/rapidjson/include")
    target_link_libraries(IceHaloSim
        PUBLIC ${CMAKE_THREAD_LIBS_INIT} ${Boost_LIBRARIES})
    install(TARGETS IceHaloSim
        DESTINATION "${CMAKE_INSTALL_PREFIX}")

    add_executable(IceHaloRender render_main.cpp ${SOURCE_FILE})
    target_include_directories(IceHaloRender
        PUBLIC ${Boost_INCLUDE_DIRS} "${MODULE_ROOT}/rapidjson/include")
    target_link_libraries(IceHaloRender
        PUBLIC ${OpenCV_LIBS} ${Boost_LIBRARIES})
    install(TARGETS IceHaloRender
        DESTINATION "${CMAKE_INSTALL_PREFIX}")
endif()



