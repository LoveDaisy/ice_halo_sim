# ==================================================================================================
# Add some definitions
if(MULTI_THREAD)
  add_compile_definitions(MULTI_THREAD)
endif()

if(RANDOM_SEED)
  add_compile_definitions(RANDOM_SEED)
endif()

if(${OS_NAME} STREQUAL "Linux")
  add_compile_definitions(OS_LINUX) 
elseif(${OS_NAME} STREQUAL "Darwin")
  add_compile_definitions(OS_MAC)
elseif(${OS_NAME} STREQUAL "Windows")
  add_compile_definitions(OS_WIN)
endif()



# ==================================================================================================
# Add targets
set(SOURCE_FILE
    context/camera_context.cpp
    context/context.cpp
    context/crystal_context.cpp
    context/filter_context.cpp
    context/multi_scatter_context.cpp
    context/render_context.cpp
    context/sun_context.cpp
    core/crystal.cpp
    core/mymath.cpp
    core/optics.cpp
    core/render.cpp
    core/simulation.cpp
    io/file.cpp
    util/obj_pool.cpp
    util/threadingpool.cpp)

add_executable(IceHaloSim trace_main.cpp ${SOURCE_FILE})
target_include_directories(IceHaloSim
    PUBLIC ${PROJ_SRC_DIR} ${Boost_INCLUDE_DIRS} "${MODULE_ROOT}/rapidjson/include")
target_link_libraries(IceHaloSim
    PUBLIC ${CMAKE_THREAD_LIBS_INIT} ${Boost_LIBRARIES})
install(TARGETS IceHaloSim
    DESTINATION "${CMAKE_INSTALL_PREFIX}")

add_executable(IceHaloRender render_main.cpp ${SOURCE_FILE})
target_include_directories(IceHaloRender
    PUBLIC ${PROJ_SRC_DIR} ${Boost_INCLUDE_DIRS} "${MODULE_ROOT}/rapidjson/include")
target_link_libraries(IceHaloRender
    PUBLIC ${OpenCV_LIBS} ${Boost_LIBRARIES})
install(TARGETS IceHaloRender
    DESTINATION "${CMAKE_INSTALL_PREFIX}")

add_executable(IceHaloEndless endless_main.cpp ${SOURCE_FILE})
target_include_directories(IceHaloEndless
    PUBLIC ${PROJ_SRC_DIR} ${Boost_INCLUDE_DIRS} "${MODULE_ROOT}/rapidjson/include")
target_link_libraries(IceHaloEndless
    PUBLIC ${OpenCV_LIBS} ${Boost_LIBRARIES})
install(TARGETS IceHaloEndless
    DESTINATION "${CMAKE_INSTALL_PREFIX}")
