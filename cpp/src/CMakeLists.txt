# ==================================================================================================
message(STATUS "[src] CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")


# ==================================================================================================
# Add targets
add_executable(IceHaloEndless
    "${PROJ_SRC_DIR}/endless_main.cpp")
target_link_libraries(IceHaloEndless
    PUBLIC icehalo_lib ${OpenCV_LIBS} Boost::filesystem Boost::system)
target_compile_definitions(IceHaloEndless
    PRIVATE $<$<CONFIG:Debug>:DEBUG>)
install(TARGETS IceHaloEndless
    DESTINATION "${CMAKE_INSTALL_PREFIX}")

add_executable(IceHaloV3
    "${PROJ_SRC_DIR}/main_v3.cpp")
target_link_libraries(IceHaloV3
    PUBLIC icehalo_lib ${OpenCV_LIBS} Boost::filesystem Boost::system)
target_compile_definitions(IceHaloV3
    PRIVATE $<$<CONFIG:Debug>:DEBUG>)
install(TARGETS IceHaloV3
    DESTINATION "${CMAKE_INSTALL_PREFIX}")
