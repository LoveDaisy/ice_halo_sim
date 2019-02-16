set(SOURCE_FILE
    mymath.cpp
    crystal.cpp
    optics.cpp
    context.cpp
    simulation.cpp
    render.cpp
    files.cpp
    threadingpool.cpp)

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
