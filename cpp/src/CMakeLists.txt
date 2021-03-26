# ==================================================================================================
# Add some definitions
if(MULTI_THREAD)
  add_compile_definitions(MULTI_THREAD)
endif()

if(RANDOM_SEED)
  add_compile_definitions(RANDOM_SEED)
endif()

message(STATUS "[src] CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")


# ==================================================================================================
# Add targets
message(STATUS "[src] icehalo_libs: ${icehalo_libs}")
message(STATUS "[src] icehalo_include: ${icehalo_include}")
add_executable(IceHaloSim
    trace_main.cpp
    ${icehalo_src})
target_include_directories(IceHaloSim
    PUBLIC ${icehalo_include})
target_link_libraries(IceHaloSim
    PUBLIC ${OpenCV_LIBS} ${Boost_LIBRARIES})
install(TARGETS IceHaloSim
    DESTINATION "${CMAKE_INSTALL_PREFIX}")

add_executable(IceHaloRender
    render_main.cpp
    ${icehalo_src})
target_include_directories(IceHaloRender
    PUBLIC ${icehalo_include})
target_link_libraries(IceHaloRender
    PUBLIC ${OpenCV_LIBS} ${Boost_LIBRARIES})
install(TARGETS IceHaloRender
    DESTINATION "${CMAKE_INSTALL_PREFIX}")

add_executable(IceHaloEndless
    endless_main.cpp
    ${icehalo_src})
target_include_directories(IceHaloEndless
    PUBLIC ${icehalo_include})
target_link_libraries(IceHaloEndless
    PUBLIC ${OpenCV_LIBS} ${Boost_LIBRARIES})
install(TARGETS IceHaloEndless
    DESTINATION "${CMAKE_INSTALL_PREFIX}")
