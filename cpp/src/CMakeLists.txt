if(MULTI_THREAD)
  add_compile_definitions(MULTI_THREAD)
endif()

if(RANDOM_SEED)
  add_compile_definitions(RANDOM_SEED)
endif()


set(SOURCE_FILE
    context.cpp
    crystal.cpp
    files.cpp
    mymath.cpp
    optics.cpp
    render.cpp
    simulation.cpp
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

add_executable(IceHaloEndless endless_main.cpp ${SOURCE_FILE})
target_include_directories(IceHaloEndless
    PUBLIC ${Boost_INCLUDE_DIRS} "${MODULE_ROOT}/rapidjson/include")
target_link_libraries(IceHaloEndless
    PUBLIC ${OpenCV_LIBS} ${Boost_LIBRARIES})
install(TARGETS IceHaloEndless
    DESTINATION "${CMAKE_INSTALL_PREFIX}")
